// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! A FreeTouch dongle as the controllers' radio, in place of the Rift headset: found on USB,
//! shown to be one before anything is written to it, read on its own thread, and served to the
//! engine (`engine::Engine::open`) and to the daemon's radio thread as the headset's 64-byte
//! radio reports (`freetouch::radio_report`). Calibration reads, haptics and the radio address
//! go to the dongle's commands; its poll clock goes to the daemon as the radio's clock.

use std::collections::VecDeque;
use std::io;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::freetouch::{self, cmd, Event, Identity, Parser, Reply};
use crate::platform::{hid, serial};

/// The dongle's USB ID (FreeTouch's `docs/PROTOCOL.md`). It is TinyUSB's example ID, which other
/// devices use too, so a port on it is only a candidate.
pub const VENDOR: u16 = 0xcafe;
pub const PRODUCT: u16 = 0x4001;

/// The firmware this host side is written for: FreeTouch v3.1. 3.0 (identity firmware 0) is
/// refused, since it parks on one radio channel unless the host fixes the controllers' hop
/// increment, and older firmware has no identity. Newer firmware is used, and said to be newer.
pub const KNOWN_FIRMWARE: u32 = 0x0301;

/// How long a dongle is given to show itself: it sends its identity when the port opens and its
/// poll clock every 10 ms.
const IDENTIFY_WAIT: Duration = Duration::from_secs(2);
/// The dongle sends a read once and watches 50 polls (100 ms) for the reply; the host retries.
const REPLY_WAIT: Duration = Duration::from_millis(100);
const READ_TRIES: u32 = 10;
/// Reports kept for a reader that fell behind, about 1.2 s at both controllers' rate.
const QUEUE: usize = 512;

/// What carries the dongle's bytes: its serial port, or a test's.
pub trait Link: Send + Sync {
    /// Bytes received, waiting up to `timeout` for the first: 0 when none came. An error means
    /// the dongle is gone.
    fn read(&self, buf: &mut [u8], timeout: Duration) -> io::Result<usize>;
    fn write_all(&self, data: &[u8]) -> io::Result<()>;
}

impl Link for serial::Port {
    fn read(&self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        serial::Port::read(self, buf, timeout)
    }

    fn write_all(&self, data: &[u8]) -> io::Result<()> {
        serial::Port::write_all(self, data)
    }
}

/// Whether a device on the dongle's USB ID names itself as FreeTouch does: the product string
/// "FreeTouch dongle", or on Windows the name of its serial interface, "FreeTouch CDC".
pub fn named_freetouch(name: &str) -> bool {
    matches!(name, "FreeTouch dongle" | "FreeTouch CDC")
}

/// A firmware revision as FreeTouch writes it: 0x0301 is 3.1.
pub fn firmware_text(revision: u32) -> String {
    format!("{}.{}", revision >> 8, revision & 0xff)
}

/// Why a port is not used.
#[derive(Debug)]
pub enum Refusal {
    /// No FreeTouch record came: another device on the same USB ID. Nothing was written to it.
    Silent,
    /// A FreeTouch dongle without an identity: firmware before 3.0.
    NoIdentity,
    /// A FreeTouch dongle whose firmware is older than `KNOWN_FIRMWARE`.
    Old(u32),
    Io(io::Error),
}

impl std::fmt::Display for Refusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        const UPDATE: &str = "update it to FreeTouch 3.1 or later as FreeTouch's INSTALL.md says";
        match self {
            Refusal::Silent => write!(f, "sent nothing a FreeTouch dongle sends within {} s, so it is another device on the same USB ID; nothing was sent to it", IDENTIFY_WAIT.as_secs()),
            Refusal::NoIdentity => write!(f, "is a FreeTouch dongle with firmware older than 3.0, which cv1touch cannot use: {UPDATE}"),
            Refusal::Old(0) => write!(f, "is a FreeTouch dongle with firmware 3.0, which cv1touch cannot use (3.0 does not find the controllers' radio hopping by itself): {UPDATE}"),
            Refusal::Old(v) => write!(f, "is a FreeTouch dongle with firmware {}, which cv1touch cannot use: {UPDATE}", firmware_text(*v)),
            Refusal::Io(e) => write!(f, "could not be read: {e}"),
        }
    }
}

/// Reads `link` until it shows a FreeTouch status record, writing nothing before that; asks for
/// the identity if half of `wait` passed without one. The identity, if the firmware is usable.
pub fn identify(link: &dyn Link, parser: &mut Parser, wait: Duration) -> Result<Identity, Refusal> {
    let start = Instant::now();
    let (mut identified, mut asked) = (false, false);
    let mut buf = [0u8; 512];
    loop {
        while let Some(event) = parser.next_event() {
            let Event::Status(s) = event else { continue };
            // the codes FreeTouch documents
            if !(1..=9).contains(&s.code) {
                continue;
            }
            identified = true;
            if let Some(id) = s.identity() {
                return match id.firmware {
                    v if v < KNOWN_FIRMWARE => Err(Refusal::Old(v)),
                    _ => Ok(id),
                };
            }
        }
        let elapsed = start.elapsed();
        if elapsed >= wait {
            return Err(if identified {
                Refusal::NoIdentity
            } else {
                Refusal::Silent
            });
        }
        if identified && !asked && elapsed >= wait / 2 {
            link.write_all(&cmd::IDENTITY).map_err(Refusal::Io)?;
            asked = true;
        }
        let n = link
            .read(&mut buf, Duration::from_millis(50))
            .map_err(Refusal::Io)?;
        parser.push(&buf[..n]);
    }
}

/// A camera exposure on the dongle's clock: the poll it was learnt from and the exposure (µs),
/// and its count, as the headset's IMU reports carry them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Exposure {
    pub poll_us: u32,
    pub exposure_us: u32,
    pub count: u16,
}

/// What the dongle hands its owner as it reads.
pub struct Hooks {
    /// The host time a poll timestamp on the dongle's clock arrived and that timestamp (µs), from
    /// every controller record, and from the status record sent every 10 ms with the exposure
    /// period the polls announce (µs): the radio's clock, as the headset's IMU reports are with a
    /// headset.
    pub clock: Box<dyn Fn(u64, u32, Option<u32>) + Send + Sync>,
    /// The sensors' newest exposure the owner has timed from that clock since it was last asked,
    /// for the engine (`Dongle::take_exposure`).
    pub exposure: Box<dyn Fn() -> Option<Exposure> + Send + Sync>,
    pub log: Box<dyn Fn(&str) + Send + Sync>,
}

/// Reports waiting for one reader, and what wakes it.
struct Queue {
    reports: Mutex<VecDeque<[u8; 64]>>,
    ready: Condvar,
    waker: hid::Waker,
}

impl Queue {
    fn new() -> io::Result<Queue> {
        Ok(Queue {
            reports: Mutex::new(VecDeque::new()),
            ready: Condvar::new(),
            waker: hid::Waker::new()?,
        })
    }

    fn push(&self, report: [u8; 64]) {
        let mut q = self.reports.lock().unwrap();
        if q.len() == QUEUE {
            q.pop_front();
        }
        q.push_back(report);
        self.waker.wake();
        drop(q);
        self.ready.notify_all();
    }

    fn wake(&self) {
        let _q = self.reports.lock().unwrap();
        self.waker.wake();
        drop(_q);
        self.ready.notify_all();
    }
}

struct FlashRead {
    request: (u8, u16, u16),
    sent: Instant,
    tries: u32,
}

struct Shared {
    link: Box<dyn Link>,
    engine: Queue,
    radio: Queue,
    replies: Mutex<VecDeque<Reply>>,
    read: Mutex<Option<FlashRead>>,
    stop: AtomicBool,
    gone: AtomicBool,
    reply_unread: AtomicBool,
    hooks: Hooks,
}

impl Shared {
    fn dispatch(&self, event: Event) {
        match event {
            Event::Report { input, report } => {
                (self.hooks.clock)(crate::platform::now_ns(), input.poll_us, None);
                if let Some(r) = freetouch::radio_report(input.session, &report) {
                    self.engine.push(r);
                    self.radio.push(r);
                }
            }
            Event::Status(s) => {
                if let Some((poll_us, period_us)) = s.poll_clock() {
                    (self.hooks.clock)(crate::platform::now_ns(), poll_us, Some(period_us));
                }
            }
            Event::Reply(r) => {
                let mut replies = self.replies.lock().unwrap();
                if replies.len() == 16 {
                    replies.pop_front();
                }
                replies.push_back(r);
            }
        }
    }
}

fn read_loop(shared: Arc<Shared>, mut parser: Parser) {
    let mut buf = [0u8; 4096];
    while !shared.stop.load(Ordering::Acquire) {
        match shared.link.read(&mut buf, Duration::from_millis(100)) {
            Ok(n) => parser.push(&buf[..n]),
            Err(e) => {
                (shared.hooks.log)(&format!("FreeTouch dongle: it stopped answering ({e})"));
                shared.gone.store(true, Ordering::Release);
                shared.engine.wake();
                shared.radio.wake();
                return;
            }
        }
        while let Some(event) = parser.next_event() {
            shared.dispatch(event);
        }
    }
}

/// How a calibration read stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Flash {
    Pending,
    Done,
    Failed,
}

/// The `length` bytes of flash in a controller's reply to the config read (`offset`, `length`):
/// a reply frame on the read's session (byte 2) in which the read's argument, offset and length
/// as the request carried them, comes before the data, as the headset's radio passes them on
/// (offset and length, then the data: `encode_radio_data_read_cmd` and `rift_radio_read_flash`).
pub fn flash_reply(frame: &[u8], session: u8, offset: u16, length: u16) -> Option<&[u8]> {
    if frame.get(2) != Some(&session) {
        return None;
    }
    let arg = (u32::from(offset) | (u32::from(length) << 16)).to_le_bytes();
    let at = (5..frame.len().saturating_sub(3)).find(|&i| frame[i..i + 4] == arg)?;
    frame.get(at + 4..at + 4 + usize::from(length))
}

pub struct Dongle {
    shared: Arc<Shared>,
    reader: Option<JoinHandle<()>>,
    pub identity: Identity,
}

impl Dongle {
    /// Opens the serial port at `path` and uses it once it shows itself to be a usable FreeTouch
    /// dongle (`identify`).
    pub fn open(path: &Path, hooks: Hooks) -> Result<Dongle, Refusal> {
        let port = serial::Port::open(path).map_err(Refusal::Io)?;
        Dongle::start(Box::new(port), hooks, IDENTIFY_WAIT)
    }

    pub fn start(link: Box<dyn Link>, hooks: Hooks, wait: Duration) -> Result<Dongle, Refusal> {
        let mut parser = Parser::new();
        let identity = identify(link.as_ref(), &mut parser, wait)?;
        let shared = Arc::new(Shared {
            link,
            engine: Queue::new().map_err(Refusal::Io)?,
            radio: Queue::new().map_err(Refusal::Io)?,
            replies: Mutex::new(VecDeque::new()),
            read: Mutex::new(None),
            stop: AtomicBool::new(false),
            gone: AtomicBool::new(false),
            reply_unread: AtomicBool::new(false),
            hooks,
        });
        let reader = {
            let shared = shared.clone();
            std::thread::Builder::new()
                .name("cv1t-dongle".into())
                .spawn(move || read_loop(shared, parser))
                .map_err(Refusal::Io)?
        };
        Ok(Dongle {
            shared,
            reader: Some(reader),
            identity,
        })
    }

    /// The engine's next report into `buf` (its length), waiting up to `wait` for one: None when
    /// none came. An error once the dongle is gone and nothing is left.
    pub fn read_report(&self, buf: &mut [u8], wait: Duration) -> io::Result<Option<usize>> {
        let q = &self.shared.engine;
        let mut reports = q.reports.lock().unwrap();
        let deadline = Instant::now() + wait;
        loop {
            if let Some(r) = reports.pop_front() {
                let n = r.len().min(buf.len());
                buf[..n].copy_from_slice(&r[..n]);
                return Ok(Some(n));
            }
            if self.shared.gone.load(Ordering::Acquire) {
                return Err(io::ErrorKind::BrokenPipe.into());
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(None);
            }
            reports = q.ready.wait_timeout(reports, left).unwrap().0;
        }
    }

    /// One calibration read of `length` bytes at `offset` from the controller of session
    /// `device_type` (2 left, 3 right, as the headset numbers them), polled with the same
    /// arguments until it ends. The request is sent again after `REPLY_WAIT` without a reply, up
    /// to `READ_TRIES` times.
    pub fn read_flash(&self, device_type: u8, offset: u16, length: u16, out: &mut [u8]) -> Flash {
        let request = (device_type, offset, length);
        let send = || {
            self.shared
                .link
                .write_all(&cmd::config_read(device_type, offset, length))
        };
        let mut read = self.shared.read.lock().unwrap();
        let Some(r) = read.as_mut().filter(|r| r.request == request) else {
            // no older reply may answer this one
            self.shared.replies.lock().unwrap().clear();
            if send().is_err() {
                *read = None;
                return Flash::Failed;
            }
            *read = Some(FlashRead {
                request,
                sent: Instant::now(),
                tries: 1,
            });
            return Flash::Pending;
        };
        let replies: Vec<Reply> = self.shared.replies.lock().unwrap().drain(..).collect();
        for reply in &replies {
            if let Some(data) = flash_reply(reply.frame(), device_type, offset, length) {
                out[..data.len()].copy_from_slice(data);
                *read = None;
                return Flash::Done;
            }
            if reply.frame().get(2) == Some(&device_type)
                && !self.shared.reply_unread.swap(true, Ordering::Relaxed)
            {
                let hex: Vec<String> = reply.frame().iter().map(|b| format!("{b:02x}")).collect();
                (self.shared.hooks.log)(&format!("FreeTouch dongle: a controller's reply to a calibration read is not laid out as cv1touch expects ({}); run cv1touch report and attach the file it writes to your bug report", hex.join(" ")));
            }
        }
        if r.sent.elapsed() >= REPLY_WAIT {
            if r.tries >= READ_TRIES || send().is_err() {
                *read = None;
                return Flash::Failed;
            }
            r.tries += 1;
            r.sent = Instant::now();
        }
        Flash::Pending
    }

    /// The sensors' newest exposure, once, for the engine's tracker (`Hooks::exposure`).
    pub fn take_exposure(&self) -> Option<Exposure> {
        (self.shared.hooks.exposure)()
    }

    /// Gives up the calibration read under way.
    pub fn cancel_read(&self) {
        *self.shared.read.lock().unwrap() = None;
    }

    /// Vibrates the controllers of session `device_type` at `amplitude` (0 stops), 160 Hz when
    /// `low_freq`, else 320 Hz. The dongle sends it three times; it confirms nothing.
    pub fn haptic(&self, device_type: u8, low_freq: bool, amplitude: u8) -> io::Result<()> {
        let hz = if low_freq { 160 } else { 320 };
        self.shared
            .link
            .write_all(&cmd::haptic(device_type, amplitude, hz))
    }

    /// The radio thread's reader: its own copy of every report.
    pub fn radio_port(&self) -> RadioPort {
        RadioPort(self.shared.clone())
    }
}

impl Drop for Dongle {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
    }
}

/// The radio thread's copy of the dongle's reports, read as the headset's radio interface is.
pub struct RadioPort(Arc<Shared>);

impl hid::RadioPort for RadioPort {
    fn wait(&mut self, waker: &hid::Waker, timeout: Option<Duration>) -> io::Result<hid::Woke> {
        let q = &self.0.radio;
        {
            let reports = q.reports.lock().unwrap();
            if !reports.is_empty() || self.0.gone.load(Ordering::Acquire) {
                return Ok(hid::Woke {
                    report: true,
                    waker: false,
                });
            }
            // Empty and not gone under `reports`: clear any wakeup left by a report read after a
            // fast-path wait; any later `push` or `gone` wake acquires `reports` first.
            q.waker.clear();
        }
        q.waker.wait_with(waker, timeout)
    }

    fn try_read(&mut self, buf: &mut [u8]) -> io::Result<Option<usize>> {
        match self.0.radio.reports.lock().unwrap().pop_front() {
            Some(r) => {
                let n = r.len().min(buf.len());
                buf[..n].copy_from_slice(&r[..n]);
                Ok(Some(n))
            }
            None if self.0.gone.load(Ordering::Acquire) => Err(io::ErrorKind::BrokenPipe.into()),
            None => Ok(None),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::hid::RadioPort as _;
    use freetouch::RECORD_SIZE;

    const KEY: [u8; 16] = *b"fictional key 02";
    const ADDRESS: [u8; 5] = [0x0a, 0x0b, 0x0c, 0x0d, 0x0e];

    /// A serial link that hands out queued bytes and keeps what is written.
    /// What a fake answers a write with, given what was written.
    type Answer = fn(&[u8]) -> Vec<u8>;
    type Seen<T> = Arc<Mutex<Vec<T>>>;

    #[derive(Default)]
    struct Fake {
        inbox: Mutex<VecDeque<u8>>,
        written: Mutex<Vec<u8>>,
        gone: AtomicBool,
        answer: Option<Answer>,
    }

    impl Fake {
        fn feed(&self, bytes: &[u8]) {
            self.inbox.lock().unwrap().extend(bytes);
        }
    }

    impl Link for Arc<Fake> {
        fn read(&self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
            if self.gone.load(Ordering::Acquire) {
                return Err(io::ErrorKind::BrokenPipe.into());
            }
            let mut inbox = self.inbox.lock().unwrap();
            if inbox.is_empty() {
                drop(inbox);
                std::thread::sleep(timeout.min(Duration::from_millis(5)));
                return Ok(0);
            }
            let n = buf.len().min(inbox.len());
            for b in &mut buf[..n] {
                *b = inbox.pop_front().unwrap();
            }
            Ok(n)
        }

        fn write_all(&self, data: &[u8]) -> io::Result<()> {
            self.written.lock().unwrap().extend_from_slice(data);
            if let Some(answer) = self.answer {
                self.feed(&answer(data));
            }
            Ok(())
        }
    }

    fn status(code: u32, a: u32, b: u32, raw: &[u8]) -> [u8; RECORD_SIZE] {
        let mut r = [0u8; RECORD_SIZE];
        r[..4].copy_from_slice(b"CV1S");
        r[4..8].copy_from_slice(&code.to_le_bytes());
        r[8..12].copy_from_slice(&a.to_le_bytes());
        r[12..16].copy_from_slice(&b.to_le_bytes());
        r[20..20 + raw.len()].copy_from_slice(raw);
        r
    }

    fn identity(firmware: u32) -> [u8; RECORD_SIZE] {
        status(9, 21, firmware, &[&ADDRESS[..], &KEY[..]].concat())
    }

    /// Each clock sample the hook was given: the poll stamp and the period with it.
    type Clock = Seen<(u32, Option<u32>)>;

    fn hooks() -> (Hooks, Clock, Seen<String>) {
        let clock = Arc::new(Mutex::new(Vec::new()));
        let logs = Arc::new(Mutex::new(Vec::new()));
        let (c, l) = (clock.clone(), logs.clone());
        let hooks = Hooks {
            clock: Box::new(move |_, us, period| c.lock().unwrap().push((us, period))),
            exposure: Box::new(|| None),
            log: Box::new(move |s| l.lock().unwrap().push(s.to_owned())),
        };
        (hooks, clock, logs)
    }

    fn started(fake: &Arc<Fake>) -> Dongle {
        fake.feed(&identity(KNOWN_FIRMWARE));
        Dongle::start(
            Box::new(fake.clone()),
            hooks().0,
            Duration::from_millis(400),
        )
        .unwrap()
    }

    fn until(what: impl Fn() -> bool) {
        let start = Instant::now();
        while !what() {
            assert!(start.elapsed() < Duration::from_secs(5), "timed out");
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// Another device on TinyUSB's example ID, streaming text or nothing, gets not one byte.
    #[test]
    fn a_foreign_device_on_the_same_id_receives_nothing() {
        for bytes in [
            &b""[..],
            b"hello from another TinyUSB gadget\r\n",
            b"CV1Xjunk",
        ] {
            let fake = Arc::new(Fake::default());
            fake.feed(&bytes.repeat(20));
            // the status magic with a code FreeTouch never sends is not a FreeTouch record
            fake.feed(&status(0x4141_4141, 0, 0, &[]));
            let result = Dongle::start(
                Box::new(fake.clone()),
                hooks().0,
                Duration::from_millis(300),
            );
            assert!(matches!(result, Err(Refusal::Silent)));
            assert!(fake.written.lock().unwrap().is_empty());
        }
        assert!(!named_freetouch("TinyUSB Device") && named_freetouch("FreeTouch dongle"));
    }

    /// The identity sent when the port opens is enough: nothing is written to identify it.
    #[test]
    fn a_dongle_is_known_by_its_identity_without_a_write() {
        let fake = Arc::new(Fake::default());
        let dongle = started(&fake);
        assert_eq!(dongle.identity.address, ADDRESS);
        assert!(fake.written.lock().unwrap().is_empty());
    }

    /// A dongle that has shown a FreeTouch status but no identity is asked for it, once.
    #[test]
    fn the_identity_is_asked_for_only_after_a_status_record() {
        let fake = Arc::new(Fake {
            answer: Some(|w| {
                if w == b"I" {
                    identity(KNOWN_FIRMWARE).to_vec()
                } else {
                    Vec::new()
                }
            }),
            ..Default::default()
        });
        fake.feed(&status(7, 1000, 19200, &[]));
        let dongle = Dongle::start(
            Box::new(fake.clone()),
            hooks().0,
            Duration::from_millis(200),
        );
        assert!(dongle.is_ok());
        assert_eq!(&*fake.written.lock().unwrap(), b"I");
        // without an identity ever: firmware before 3.0
        let fake = Arc::new(Fake::default());
        fake.feed(&status(2, 0, 0, &[]));
        let result = Dongle::start(
            Box::new(fake.clone()),
            hooks().0,
            Duration::from_millis(200),
        );
        assert!(matches!(result, Err(Refusal::NoIdentity)));
    }

    /// Firmware 3.0 reports 0 in its identity and is refused, saying why; newer firmware is used.
    #[test]
    fn the_firmware_is_checked() {
        for (firmware, usable) in [(0, false), (0x0300, false), (0x0301, true), (0x0402, true)] {
            let fake = Arc::new(Fake::default());
            fake.feed(&identity(firmware));
            let result = Dongle::start(
                Box::new(fake.clone()),
                hooks().0,
                Duration::from_millis(200),
            );
            assert_eq!(result.is_ok(), usable, "firmware {firmware:#x}");
            if let Err(e) = result {
                assert!(e.to_string().contains("3.1 or later"), "{e}");
            }
        }
        assert!(Refusal::Old(0).to_string().contains("firmware 3.0"));
        assert_eq!(firmware_text(0x0301), "3.1");
    }

    /// Each report reaches the engine and the radio thread as the headset's report, and the poll
    /// clock reaches its hook.
    #[test]
    fn reports_reach_both_readers_and_the_clock_its_hook() {
        let fake = Arc::new(Fake::default());
        fake.feed(&identity(KNOWN_FIRMWARE));
        let (hooks, clock, _) = hooks();
        let dongle =
            Dongle::start(Box::new(fake.clone()), hooks, Duration::from_millis(400)).unwrap();
        let report: [u8; freetouch::REPORT_SIZE] = std::array::from_fn(|i| i as u8 + 1);
        fake.feed(&freetouch::encode_input(&KEY, 1, 5000, 1, 3, 1, &report));
        fake.feed(&status(7, 7000, 19200, &[]));
        let want = freetouch::radio_report(3, &report).unwrap();
        let mut buf = [0u8; 64];
        assert_eq!(
            dongle
                .read_report(&mut buf, Duration::from_secs(5))
                .unwrap(),
            Some(64)
        );
        assert_eq!(buf, want);
        let mut port = dongle.radio_port();
        let waker = hid::Waker::new().unwrap();
        assert!(
            port.wait(&waker, Some(Duration::from_secs(5)))
                .unwrap()
                .report
        );
        assert_eq!(port.try_read(&mut buf).unwrap(), Some(64));
        assert_eq!(buf, want);
        until(|| clock.lock().unwrap().len() == 2);
        // the controller record's poll, then the status record's with the period
        assert_eq!(*clock.lock().unwrap(), [(5000, None), (7000, Some(19200))]);
        // nothing more: the engine's wait ends empty
        assert_eq!(
            dongle
                .read_report(&mut buf, Duration::from_millis(20))
                .unwrap(),
            None
        );
        // a report consumed after a fast-path wait leaves no stale report wakeup:
        // waiting with no waker times out, and waiting with a waker reports only the waker
        assert_eq!(
            port.wait(&waker, Some(Duration::from_millis(20))).unwrap(),
            hid::Woke::default()
        );
        // the radio thread's wait still ends on its waker
        waker.wake();
        assert_eq!(
            port.wait(&waker, Some(Duration::from_secs(5))).unwrap(),
            hid::Woke {
                report: false,
                waker: true,
            }
        );
    }

    fn reply(session: u8, offset: u16, length: u16, data: &[u8]) -> [u8; RECORD_SIZE] {
        let mut frame = vec![37, 1, session, 0x0a, 0x12];
        frame.extend((u32::from(offset) | (u32::from(length) << 16)).to_le_bytes());
        frame.extend(data);
        let mut r = [0u8; RECORD_SIZE];
        r[..4].copy_from_slice(b"CV1R");
        r[4] = frame.len() as u8;
        r[5..5 + frame.len()].copy_from_slice(&frame);
        r
    }

    /// A calibration read sends the config read, takes the data after its own echoed argument,
    /// ignores another session's and an older request's replies, and is sent again when no reply
    /// comes.
    #[test]
    fn calibration_reads_go_to_the_dongle_and_are_retried() {
        let fake = Arc::new(Fake::default());
        let dongle = started(&fake);
        let mut out = [0u8; 16];
        // a reply left from an earlier request does not answer this one
        fake.feed(&reply(3, 0x1bf0, 16, &[0xee; 16]));
        until(|| !dongle.shared.replies.lock().unwrap().is_empty());
        assert_eq!(dongle.read_flash(3, 0x1bf0, 16, &mut out), Flash::Pending);
        assert_eq!(
            &*fake.written.lock().unwrap(),
            &cmd::config_read(3, 0x1bf0, 16)
        );
        assert_eq!(dongle.read_flash(3, 0x1bf0, 16, &mut out), Flash::Pending);
        // no reply within REPLY_WAIT: the read goes out again
        std::thread::sleep(REPLY_WAIT + Duration::from_millis(10));
        assert_eq!(dongle.read_flash(3, 0x1bf0, 16, &mut out), Flash::Pending);
        assert_eq!(
            fake.written.lock().unwrap().len(),
            2 * cmd::config_read(3, 0, 0).len()
        );
        fake.feed(&reply(2, 0x1bf0, 16, &[0x11; 16]));
        fake.feed(&reply(3, 0x1bf0, 16, &[0x5a; 16]));
        until(|| dongle.shared.replies.lock().unwrap().len() == 2);
        assert_eq!(dongle.read_flash(3, 0x1bf0, 16, &mut out), Flash::Done);
        assert_eq!(out, [0x5a; 16]);
        assert_eq!(
            flash_reply(&reply(3, 20, 20, &[1; 20])[5..54], 3, 20, 20),
            Some(&[1u8; 20][..])
        );
        assert_eq!(
            flash_reply(&reply(3, 20, 20, &[1; 10])[5..33], 3, 20, 20),
            None
        );
    }

    /// A read nobody answers ends as failed after its tries.
    #[test]
    fn an_unanswered_read_fails() {
        let fake = Arc::new(Fake::default());
        let dongle = started(&fake);
        let mut out = [0u8; 20];
        let start = Instant::now();
        while dongle.read_flash(2, 0, 20, &mut out) == Flash::Pending {
            assert!(start.elapsed() < Duration::from_secs(5));
            std::thread::sleep(Duration::from_millis(5));
        }
        let sent = fake.written.lock().unwrap().len() / cmd::config_read(2, 0, 0).len();
        assert_eq!(sent, READ_TRIES as usize);
    }

    #[test]
    fn haptics_go_to_the_dongle() {
        let fake = Arc::new(Fake::default());
        let dongle = started(&fake);
        dongle.haptic(2, true, 200).unwrap();
        dongle.haptic(3, false, 0).unwrap();
        let want = [cmd::haptic(2, 200, 160), cmd::haptic(3, 0, 320)].concat();
        assert_eq!(&*fake.written.lock().unwrap(), &want);
    }

    /// On a real dongle: it is found and identified, and reports come.
    ///
    /// Needs a FreeTouch dongle (firmware 3.1 or later) with a paired controller, and no Rift
    /// headset or running cv1touchd. Install `udev/70-cv1touch.rules` (or run as a user with
    /// access to the ttyACM node), plug the dongle in, wake the controller, then run
    /// `cargo test --lib dongle::tests::a_real_dongle -- --ignored --nocapture`. It prints the
    /// port, the firmware and the number of reports in 5 s: about 1000 to 2500 per controller.
    #[test]
    #[ignore = "needs a FreeTouch dongle"]
    fn a_real_dongle() {
        let ports = serial::ports(VENDOR, PRODUCT).unwrap();
        let port = ports
            .iter()
            .find(|p| named_freetouch(&p.name))
            .expect("no FreeTouch dongle");
        let dongle = Dongle::open(&port.path, hooks().0).unwrap();
        let mut reports = 0;
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(5) {
            if dongle
                .read_report(&mut [0; 64], Duration::from_millis(100))
                .unwrap()
                .is_some()
            {
                reports += 1;
            }
        }
        println!(
            "{} firmware {} reports {reports}",
            port.path.display(),
            firmware_text(dongle.identity.firmware)
        );
        assert!(reports > 0);
    }

    /// Unplugged, both readers see it, the engine's after what was left.
    #[test]
    fn an_unplugged_dongle_ends_both_readers() {
        let fake = Arc::new(Fake::default());
        let dongle = started(&fake);
        let mut port = dongle.radio_port();
        fake.gone.store(true, Ordering::Release);
        let mut buf = [0u8; 64];
        until(|| {
            dongle
                .read_report(&mut [0; 64], Duration::from_millis(1))
                .is_err()
        });
        assert!(port.try_read(&mut buf).is_err());
        let waker = hid::Waker::new().unwrap();
        assert!(port.wait(&waker, None).unwrap().report);
    }
}
