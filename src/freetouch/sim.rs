// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! A simulated FreeTouch dongle: the dongle's side of the serial protocol, played from a session
//! recorded with a Rift headset, so that the dongle path runs with no hardware
//! (`examples/freetouch_sim.rs`). What it sends and answers follows FreeTouch's `docs/PROTOCOL.md`
//! as `freetouch` reads it; it shares that reading, so it shows that cv1touch's two sides agree,
//! not that they agree with the firmware.
//!
//! - Each Touch message of the headset's radio reports becomes a `CV1I` record under the
//!   simulated dongle's key, answering 0 to 5 poll slots late in turn.
//! - The dongle's clock is the headset's, started at `Config::start_us`, and drifts from the
//!   host's by `Config::drift_ppm`: the recording plays that much faster (`Sim::playback`), so
//!   that the exposures stay on the grid the dongle's clock announces, as a dongle's do. It is
//!   read from the last report of each burst of the headset's IMU
//!   reports (the next one at least `BURST_GAP_NS` later), which carries a fresh stamp: the first
//!   of a burst repeats a stamp about 1 ms old. A `CV1I` answers the last such stamp before it,
//!   and every 10 ms a status record (code 7) gives one with the exposure period.
//! - The exposure grid: status code 7 announces the period (`X` sets it, `E` slews the phase);
//!   the recording's sensor exposures, on the dongle's clock, are what that grid must reproduce.
//!   `serve` hands each recorded frame to its caller when its arrival plays, with the record it
//!   came from and that exposure (`Exposure`), so that a proof of camera tracking timed from the
//!   dongle can deliver the frame then and check what the daemon makes of it.
//! - Config block reads (`O`, `S`, `Y`, `R`) are answered from a controller's flash image
//!   (`flash_image`) in a `CV1R` frame laid out as `dongle::flash_reply` expects; haptics and
//!   the other commands are recorded (`Sim::commands`).

use std::path::Path;

use super::{encode_input, Identity, RECORD_SIZE, REPORT_SIZE};
use crate::recorder::{self, Reader};

/// An IMU report this long after the one before it starts a new burst.
pub const BURST_GAP_NS: u64 = 200_000;
const STATUS_EVERY_NS: u64 = 10_000_000;
const SLOW_STATUS_EVERY_NS: u64 = 5_000_000_000;
/// The exposure period a dongle announces until told another (the headset's).
pub const DEFAULT_PERIOD_US: u32 = 19_200;
/// Where a Touch keeps its calibration's hash (OpenHMD's `rift_radio_read_calibration_hash`).
pub const HASH_AT: usize = 0x1bf0;

/// What the dongle plays, in the order the recording has it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Item {
    /// The headset's clock (us) at host time `host_ns`, from the last IMU report of a burst.
    Clock { host_ns: u64, us: u32 },
    /// A Touch message the headset's radio passed on: session 2 left, 3 right.
    Touch {
        host_ns: u64,
        session: u8,
        report: [u8; REPORT_SIZE],
    },
    /// A sensor frame that arrived at `host_ns`, exposed at `us` on the headset's clock; `record`
    /// is its FRAME_OBS record's place in the recording (0 the first record).
    Exposure {
        host_ns: u64,
        us: u32,
        serial: String,
        record: u64,
    },
}

impl Item {
    pub fn host_ns(&self) -> u64 {
        match self {
            Item::Clock { host_ns, .. }
            | Item::Touch { host_ns, .. }
            | Item::Exposure { host_ns, .. } => *host_ns,
        }
    }
}

/// The Touch messages of a headset radio report (id 0x0c): two 28-byte messages at 3 and 31,
/// each flags (LE16, 0x1c for a Touch), device type and the 25 bytes a dongle forwards.
pub fn touch_messages(report: &[u8]) -> impl Iterator<Item = (u8, [u8; REPORT_SIZE])> + '_ {
    [3usize, 31].into_iter().filter_map(move |at| {
        let m = report.get(at..at + 3 + REPORT_SIZE)?;
        (report[0] == 0x0c && m[0] == 0x1c && m[1] == 0 && matches!(m[2], 2 | 3))
            .then(|| (m[2], m[3..].try_into().unwrap()))
    })
}

/// The last of each burst of headset IMU reports `(host_ns, stamp)`: the next report comes at
/// least `BURST_GAP_NS` later.
fn fresh(stamps: &mut impl Iterator<Item = (u64, u32)>) -> impl Iterator<Item = (u64, u32)> + '_ {
    let mut held = None;
    std::iter::from_fn(move || loop {
        let Some(s) = stamps.next() else {
            return held.take();
        };
        if let Some(last) = held.replace(s) {
            if s.0.saturating_sub(last.0) >= BURST_GAP_NS {
                return Some(last);
            }
        }
    })
}

/// The items of a recording made with a headset.
pub fn items(path: &Path) -> Result<Vec<Item>, String> {
    let (mut out, mut imu) = (Vec::new(), Vec::new());
    for (record, rec) in (0u64..).zip(Reader::open(path)?) {
        match rec.rtype {
            recorder::RAW_REPORT => match recorder::decode_raw(&rec.payload) {
                Some((0, d)) => {
                    if let Some(us) = crate::track::pipeline::hmd_report_timestamp(d) {
                        imu.push((rec.host_ns, us));
                    }
                }
                Some((1, d)) => {
                    out.extend(touch_messages(d).map(|(session, report)| Item::Touch {
                        host_ns: rec.host_ns,
                        session,
                        report,
                    }))
                }
                _ => {}
            },
            recorder::FRAME_OBS => {
                if let Some((info, _)) = recorder::decode_frame_obs(&rec.payload) {
                    if info.exposure_valid != 0 {
                        out.push(Item::Exposure {
                            host_ns: info.frame_start_host_ns,
                            us: info.exposure_hmd_ts,
                            serial: crate::engine::cstr_field(&info.serial),
                            record,
                        });
                    }
                }
            }
            _ => {}
        }
    }
    out.extend(fresh(&mut imu.into_iter()).map(|(host_ns, us)| Item::Clock { host_ns, us }));
    // a frame is recorded after its processing: play it at its arrival
    out.sort_by_key(Item::host_ns);
    Ok(out)
}

/// A Touch's flash as config block reads see it: header version 1 and the calibration's length,
/// the calibration (JSON) from offset 4, and its hash at `HASH_AT`.
pub fn flash_image(calibration: &[u8], hash: &[u8; 16]) -> Vec<u8> {
    let mut flash = vec![0xff; HASH_AT + 16];
    flash[..2].copy_from_slice(&[1, 0]);
    flash[2..4].copy_from_slice(&(calibration.len() as u16).to_le_bytes());
    flash[4..4 + calibration.len()].copy_from_slice(calibration);
    flash[HASH_AT..].copy_from_slice(hash);
    flash
}

/// The dongle's clock: the headset's, started at `start` (both wrap at 32 bits).
#[derive(Debug, Clone)]
struct DeviceClock {
    start: u32,
    first: Option<u32>,
}

impl DeviceClock {
    fn at(&mut self, us: u32) -> u32 {
        let first = *self.first.get_or_insert(us);
        self.start.wrapping_add(us.wrapping_sub(first))
    }
}

#[derive(Debug, Clone)]
pub struct Config {
    pub identity: Identity,
    /// The dongle's clock against the host's, parts per million (positive: fast).
    pub drift_ppm: f64,
    /// The dongle's clock at the first stamp.
    pub start_us: u32,
    /// The flash of the left and the right controller; None: that controller answers no read.
    pub flash: [Option<Vec<u8>>; 2],
}

/// A command the host sent, as the dongle took it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    Identity,
    Haptic {
        session: u8,
        amplitude: u8,
        frequency_hz: u16,
    },
    /// An `R` with the opcode and session it went out with, and whether a controller answered.
    Read {
        opcode: u8,
        session: u8,
        offset: u16,
        length: u16,
        answered: bool,
    },
    ExposurePeriod(u32),
    ExposurePhase(i32),
    /// Any other command, with its arguments.
    Other(Vec<u8>),
}

/// A recorded exposure on the dongle's clock.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Exposure {
    /// When the frame arrived in the recording.
    pub host_ns: u64,
    /// When that arrival played (`serve`), host clock; 0 if it did not.
    pub played_ns: u64,
    pub device_us: u32,
    pub serial: String,
    /// The frame's FRAME_OBS record in the recording.
    pub record: u64,
}

/// The dongle: what it sends for each item and for each command.
#[derive(Debug, Clone)]
pub struct Sim {
    config: Config,
    clock: DeviceClock,
    sequence: u32,
    late: u32,
    /// The dongle's clock at the last fresh stamp.
    poll_us: Option<u32>,
    next_status_ns: u64,
    next_slow_ns: u64,
    period_us: u32,
    /// What the polls' exposure phase was slewed by, us.
    pub phase_us: i64,
    pending: Vec<u8>,
    read_opcode: u8,
    read_session: u8,
    pub commands: Vec<(u64, Command)>,
    pub exposures: Vec<Exposure>,
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

/// How many bytes a command takes with its arguments; an unknown byte is one.
fn command_len(c: u8) -> usize {
    match c {
        b'V' | b'L' | b'X' | b'E' | b'R' => 5,
        b'M' => 6,
        b'U' => 3,
        b'Q' | b'Z' | b'A' | b'H' | b'O' | b'S' | b'Y' | b'K' => 2,
        _ => 1,
    }
}

impl Sim {
    pub fn new(config: Config) -> Sim {
        Sim {
            clock: DeviceClock {
                start: config.start_us,
                first: None,
            },
            config,
            sequence: 0,
            late: 0,
            poll_us: None,
            next_status_ns: 0,
            next_slow_ns: 0,
            period_us: DEFAULT_PERIOD_US,
            phase_us: 0,
            pending: Vec::new(),
            read_opcode: 0x0a,
            read_session: 0,
            commands: Vec::new(),
            exposures: Vec::new(),
        }
    }

    pub fn identity(&self) -> [u8; RECORD_SIZE] {
        let id = &self.config.identity;
        let mut raw = [0u8; 21];
        raw[..5].copy_from_slice(&id.address);
        raw[5..].copy_from_slice(&id.key);
        status(super::STATUS_IDENTITY, 21, id.firmware, &raw)
    }

    /// What the dongle sends when a host opens its port: its identity.
    pub fn opened(&mut self) -> Vec<[u8; RECORD_SIZE]> {
        self.pending.clear();
        vec![self.identity()]
    }

    /// When a moment `recorded` into the recording plays: the dongle's clock, which is the
    /// recording's, runs `Config::drift_ppm` fast against the host's.
    pub fn playback(&self, recorded: std::time::Duration) -> std::time::Duration {
        recorded.div_f64(1.0 + self.config.drift_ppm * 1e-6)
    }

    /// The exposure period the polls announce, us.
    pub fn period_us(&self) -> u32 {
        self.period_us
    }

    /// The records `item` makes the dongle send.
    pub fn play(&mut self, item: &Item) -> Vec<[u8; RECORD_SIZE]> {
        let mut out = Vec::new();
        match item {
            Item::Clock { host_ns, us } => {
                let poll_us = self.clock.at(*us);
                self.poll_us = Some(poll_us);
                if *host_ns >= self.next_status_ns {
                    self.next_status_ns = host_ns + STATUS_EVERY_NS;
                    out.push(status(
                        super::STATUS_POLL_CLOCK,
                        poll_us,
                        self.period_us,
                        &[],
                    ));
                }
                if *host_ns >= self.next_slow_ns {
                    self.next_slow_ns = host_ns + SLOW_STATUS_EVERY_NS;
                    let firmware = self.config.identity.firmware;
                    out.push(status(5, firmware | ((self.period_us / 100) << 16), 0, &[]));
                    out.push(self.identity());
                }
            }
            Item::Touch {
                session, report, ..
            } => {
                // no controller answers before the dongle's first poll
                let Some(poll_us) = self.poll_us else {
                    return out;
                };
                out.push(encode_input(
                    &self.config.identity.key,
                    self.sequence,
                    poll_us,
                    self.late,
                    *session,
                    *session - 1,
                    report,
                ));
                self.sequence = self.sequence.wrapping_add(1);
                self.late = (self.late + 1) % 6;
            }
            Item::Exposure {
                host_ns,
                us,
                serial,
                record,
            } => {
                let device_us = self.clock.at(*us);
                self.exposures.push(Exposure {
                    host_ns: *host_ns,
                    played_ns: 0,
                    device_us,
                    serial: serial.clone(),
                    record: *record,
                });
            }
        }
        out
    }

    /// Takes the bytes a host wrote, at host time `host_ns`; what the dongle sends back.
    pub fn command(&mut self, host_ns: u64, bytes: &[u8]) -> Vec<[u8; RECORD_SIZE]> {
        self.pending.extend_from_slice(bytes);
        let mut out = Vec::new();
        while let Some(&c) = self.pending.first() {
            let n = command_len(c);
            if self.pending.len() < n {
                break;
            }
            let args: Vec<u8> = self.pending.drain(..n).skip(1).collect();
            let u32_arg = || u32::from_le_bytes(args[..4].try_into().unwrap());
            let command = match c {
                b'I' => {
                    out.push(self.identity());
                    Command::Identity
                }
                b'V' => Command::Haptic {
                    session: args[0],
                    amplitude: args[1],
                    frequency_hz: u16::from_le_bytes([args[2], args[3]]),
                },
                b'X' => {
                    let us = u32_arg();
                    if (5000..=65535).contains(&us) {
                        self.period_us = us;
                    }
                    Command::ExposurePeriod(us)
                }
                b'E' => {
                    let us = u32_arg() as i32;
                    self.phase_us += i64::from(us.clamp(-33333, 33333));
                    Command::ExposurePhase(us)
                }
                b'O' => {
                    self.read_opcode = args[0];
                    continue;
                }
                b'S' => {
                    self.read_session = args[0];
                    continue;
                }
                b'Y' => continue,
                b'R' => {
                    let arg = u32_arg();
                    let (offset, length) = (arg as u16, (arg >> 16) as u16);
                    let reply = self.reply(offset, length);
                    let mut request = [0u8; 34];
                    request[..2].copy_from_slice(&[self.read_opcode, self.read_session]);
                    request[2..6].copy_from_slice(&arg.to_le_bytes());
                    out.push(status(5, 34, 0, &request));
                    out.extend(reply);
                    Command::Read {
                        opcode: self.read_opcode,
                        session: self.read_session,
                        offset,
                        length,
                        answered: reply.is_some(),
                    }
                }
                _ => Command::Other([&[c], &args[..]].concat()),
            };
            self.commands.push((host_ns, command));
        }
        out
    }

    /// A controller's `CV1R` to a config block read: byte 2 of the frame is the session, then
    /// come the read's argument and the data (the layout `dongle::flash_reply` reads).
    fn reply(&self, offset: u16, length: u16) -> Option<[u8; RECORD_SIZE]> {
        let session = self.read_session;
        let flash = self.config.flash[usize::from(session).checked_sub(2)?].as_ref()?;
        let data = flash.get(usize::from(offset)..usize::from(offset) + usize::from(length))?;
        if self.read_opcode != 0x0a || data.len() > 48 - 9 {
            return None;
        }
        let mut r = [0u8; RECORD_SIZE];
        r[..4].copy_from_slice(b"CV1R");
        r[4] = (9 + data.len()) as u8;
        r[5..10].copy_from_slice(&[37, 1, session, 0x0a, 0x12]);
        r[10..14].copy_from_slice(&(u32::from(offset) | (u32::from(length) << 16)).to_le_bytes());
        r[14..14 + data.len()].copy_from_slice(data);
        r[53] = session - 1;
        Some(r)
    }
}

#[cfg(target_os = "linux")]
pub use pty::{serve, Plan, Pty, Served};

#[cfg(target_os = "linux")]
mod pty {
    //! The dongle on a pseudo-terminal: its other end is the port a host opens.
    use std::collections::VecDeque;
    use std::fs::File;
    use std::io::{self, Read, Write};
    use std::os::fd::{AsRawFd, FromRawFd};
    use std::path::{Path, PathBuf};
    use std::time::{Duration, Instant};

    use super::{Exposure, Item, Sim, RECORD_SIZE};

    /// Maximum queued outgoing bytes waiting on a slow or stalled PTY consumer before newer
    /// records are dropped to keep playback and unplug scheduling moving.
    const MAX_PENDING_BYTES: usize = 64 * 1024;

    /// Bounded outgoing byte queue preserving whole-record framing when dropping under pressure.
    #[derive(Default)]
    struct PendingOut {
        chunks: VecDeque<Vec<u8>>,
        head: usize,
        bytes: usize,
    }

    impl PendingOut {
        fn push(&mut self, chunk: Vec<u8>) -> bool {
            if chunk.is_empty() || self.bytes + chunk.len() > MAX_PENDING_BYTES {
                return false;
            }
            self.bytes += chunk.len();
            self.chunks.push_back(chunk);
            true
        }

        fn is_empty(&self) -> bool {
            self.chunks.is_empty()
        }

        fn front_slice(&self) -> Option<&[u8]> {
            self.chunks.front().map(|c| &c[self.head..])
        }

        fn consume(&mut self, n: usize) {
            self.head += n;
            self.bytes = self.bytes.saturating_sub(n);
            if self.chunks.front().is_some_and(|c| self.head >= c.len()) {
                self.chunks.pop_front();
                self.head = 0;
            }
        }

        fn clear(&mut self) {
            self.chunks.clear();
            self.head = 0;
            self.bytes = 0;
        }
    }

    pub struct Pty {
        master: File,
        path: PathBuf,
    }

    impl Pty {
        /// A new pseudo-terminal in raw nonblocking mode, so that nothing echoes the dongle's own
        /// bytes and a stalled consumer cannot block playback.
        pub fn open() -> io::Result<Pty> {
            // SAFETY: pty setup on a descriptor this function owns, with buffers it owns.
            unsafe {
                let fd = libc::posix_openpt(
                    libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC | libc::O_NONBLOCK,
                );
                if fd < 0 {
                    return Err(io::Error::last_os_error());
                }
                let master = File::from_raw_fd(fd);
                let mut name = [0 as libc::c_char; 128];
                let mut t: libc::termios = std::mem::zeroed();
                if libc::grantpt(fd) != 0
                    || libc::unlockpt(fd) != 0
                    || libc::ptsname_r(fd, name.as_mut_ptr(), name.len()) != 0
                    || libc::tcgetattr(fd, &mut t) != 0
                {
                    return Err(io::Error::last_os_error());
                }
                libc::cfmakeraw(&mut t);
                if libc::tcsetattr(fd, libc::TCSANOW, &t) != 0 {
                    return Err(io::Error::last_os_error());
                }
                let path = PathBuf::from(
                    std::ffi::CStr::from_ptr(name.as_ptr())
                        .to_string_lossy()
                        .into_owned(),
                );
                // Opened and closed once, the master reports a hang-up until a host opens it.
                drop(
                    File::options()
                        .read(true)
                        .write(true)
                        .custom_flags(libc::O_NOCTTY)
                        .open(&path)?,
                );
                Ok(Pty { master, path })
            }
        }

        /// The port a host opens.
        pub fn path(&self) -> &Path {
            &self.path
        }

        /// None while no host holds the port open; else the bytes it wrote within `timeout`.
        /// When `want_write` is set, also wakes as soon as the PTY accepts outgoing bytes.
        fn wait(
            &mut self,
            timeout: Duration,
            want_write: bool,
            buf: &mut [u8],
        ) -> io::Result<Option<usize>> {
            let mut fd = libc::pollfd {
                fd: self.master.as_raw_fd(),
                events: libc::POLLIN | if want_write { libc::POLLOUT } else { 0 },
                revents: 0,
            };
            let ms = timeout.as_millis().min(1000) as libc::c_int;
            // SAFETY: one valid pollfd.
            if unsafe { libc::poll(&mut fd, 1, ms) } < 0 {
                let err = io::Error::last_os_error();
                if err.kind() == io::ErrorKind::Interrupted {
                    return Ok(Some(0));
                }
                return Err(err);
            }
            if fd.revents & libc::POLLHUP != 0 {
                std::thread::sleep(timeout.min(Duration::from_millis(5)));
                return Ok(None);
            }
            if fd.revents & libc::POLLIN == 0 {
                return Ok(Some(0));
            }
            match self.master.read(buf) {
                Ok(n) => Ok(Some(n)),
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(Some(0)),
                Err(e) if e.raw_os_error() == Some(libc::EIO) => Ok(None),
                Err(e) => Err(e),
            }
        }

        /// Writes as much of `pending` as the PTY accepts without blocking. Returns `Ok(false)` if
        /// the host closed the port (`EIO`).
        fn flush(&mut self, pending: &mut PendingOut) -> io::Result<bool> {
            while let Some(slice) = pending.front_slice() {
                match self.master.write(slice) {
                    Ok(0) => break,
                    Ok(n) => pending.consume(n),
                    Err(e)
                        if matches!(
                            e.kind(),
                            io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted
                        ) =>
                    {
                        break;
                    }
                    Err(e) if e.raw_os_error() == Some(libc::EIO) => {
                        pending.clear();
                        return Ok(false);
                    }
                    Err(e) => return Err(e),
                }
            }
            Ok(true)
        }
    }

    use std::os::unix::fs::OpenOptionsExt;

    /// How a run goes.
    #[derive(Debug, Clone, Default)]
    pub struct Plan {
        /// This far into the recording (from its first item, which plays when a host first opens
        /// the port): unplug the dongle (close its end) for the given time, then plug it in again
        /// on a new pseudo-terminal.
        pub unplug: Vec<(Duration, Duration)>,
        /// Another device on the dongle's USB ID: it sends a line of text now and then, never a
        /// record, and counts what a host writes to it.
        pub foreign: bool,
        /// Stop this far into the recording.
        pub until: Option<Duration>,
    }

    /// What a run saw.
    #[derive(Debug, Clone, Default)]
    pub struct Served {
        /// Times a host opened the port.
        pub opens: u32,
        /// Records sent.
        pub records: u64,
        /// Bytes a host wrote to the foreign device.
        pub foreign_bytes: u64,
    }

    /// Plays `items` in real time from the dongle on a pseudo-terminal, until they end or
    /// `Plan::until`. `plugged` is told each port's path (at start and on each replug), `frame`
    /// each recorded frame as its arrival plays, and `log` each event. Records go out only while
    /// a host holds the port open, as the dongle streams.
    pub fn serve(
        sim: &mut Sim,
        items: &[Item],
        plan: &Plan,
        plugged: &mut dyn FnMut(&Path) -> io::Result<()>,
        frame: &mut dyn FnMut(&Exposure),
        log: &mut dyn FnMut(&str),
    ) -> io::Result<Served> {
        let mut served = Served::default();
        let mut pty = Pty::open()?;
        plugged(pty.path())?;
        let mut buf = [0u8; 4096];
        let mut pending = PendingOut::default();
        let (mut held, mut start) = (false, None::<Instant>);
        let first_ns = items.first().map_or(0, Item::host_ns);
        let at = |i: &Item, sim: &Sim| sim.playback(Duration::from_nanos(i.host_ns() - first_ns));
        let mut unplug = plan.unplug.iter().peekable();
        let mut next = 0;
        loop {
            let now = start.map_or(Duration::ZERO, |s| s.elapsed());
            if start.is_some() && (plan.until.is_some_and(|u| now >= u) || next == items.len()) {
                return Ok(served);
            }
            if let Some(&&(when, down)) = unplug.peek() {
                if start.is_some() && now >= when {
                    unplug.next();
                    log(&format!("unplugged {:.3} s in", now.as_secs_f64()));
                    pending.clear();
                    drop(pty);
                    std::thread::sleep(down);
                    pty = Pty::open()?;
                    plugged(pty.path())?;
                    log(&format!("plugged in again as {}", pty.path().display()));
                    held = false;
                    continue;
                }
            }
            let mut wait = match (start, items.get(next)) {
                (Some(_), Some(item)) => at(item, sim).saturating_sub(now),
                _ => Duration::from_millis(50),
            };
            if start.is_some() {
                if let Some(&&(when, _)) = unplug.peek() {
                    wait = wait.min(when.saturating_sub(now));
                }
                if let Some(until) = plan.until {
                    wait = wait.min(until.saturating_sub(now));
                }
            }
            let mut out: Vec<[u8; RECORD_SIZE]> = Vec::new();
            match pty.wait(wait, held && !pending.is_empty(), &mut buf)? {
                None => {
                    if held {
                        log("the host closed the port");
                    }
                    held = false;
                    pending.clear();
                }
                Some(n) => {
                    if !held {
                        held = true;
                        served.opens += 1;
                        start.get_or_insert_with(Instant::now);
                        log(&format!("the host opened the port (open {})", served.opens));
                        if !plan.foreign {
                            out.extend(sim.opened());
                        }
                    }
                    if plan.foreign {
                        served.foreign_bytes += n as u64;
                    } else if n > 0 {
                        out.extend(sim.command(crate::platform::now_ns(), &buf[..n]));
                    }
                }
            }
            let now = start.map_or(Duration::ZERO, |s| s.elapsed());
            while let Some(item) = items
                .get(next)
                .filter(|i| start.is_some() && at(i, sim) <= now)
            {
                let records = sim.play(item);
                if let (Item::Exposure { .. }, Some(e)) = (item, sim.exposures.last_mut()) {
                    e.played_ns = crate::platform::now_ns();
                    frame(e);
                }
                if held && !plan.foreign {
                    out.extend(records);
                } else if held && next % 1000 == 0 {
                    pending.push(b"TinyUSB CDC example\r\n".to_vec());
                }
                next += 1;
            }
            if held {
                for r in out {
                    if pending.push(r.to_vec()) {
                        served.records += 1;
                    }
                }
                if !pending.is_empty() && !pty.flush(&mut pending)? {
                    held = false;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::freetouch::{Event, Parser};

    const KEY: [u8; 16] = *b"fictional key 02";

    fn config() -> Config {
        Config {
            identity: Identity {
                address: [9, 8, 7, 6, 5],
                key: KEY,
                firmware: 0x0301,
            },
            drift_ppm: 100.0,
            start_us: u32::MAX - 15_000,
            flash: [
                Some(flash_image(b"{\"left\": 1}", &[0x11; 16])),
                Some(flash_image(b"{\"right\": 2}", &[0x22; 16])),
            ],
        }
    }

    fn report(seed: u8) -> [u8; REPORT_SIZE] {
        std::array::from_fn(|i| seed.wrapping_mul(13).wrapping_add(i as u8))
    }

    fn radio(left: [u8; REPORT_SIZE], right: Option<[u8; REPORT_SIZE]>) -> [u8; 64] {
        let mut r = [0u8; 64];
        r[0] = 0x0c;
        r[3] = 0x1c;
        r[5] = 2;
        r[6..31].copy_from_slice(&left);
        if let Some(right) = right {
            r[31] = 0x1c;
            r[33] = 3;
            r[34..59].copy_from_slice(&right);
        }
        r
    }

    fn imu(us: u32) -> [u8; 64] {
        let mut r = [0u8; 64];
        r[0] = 0x0b;
        r[8..12].copy_from_slice(&us.to_le_bytes());
        r
    }

    /// A recording of `ms` milliseconds as a headset makes it: IMU reports in pairs 0.1 ms
    /// apart every 2 ms, the first of each pair repeating the stamp 1 ms old; a radio report
    /// every 2 ms, with the right controller's message every other time; a frame every 19.2 ms.
    fn recording(ms: u32) -> Vec<u8> {
        let mut b = vec![0u8; 32];
        b[..8].copy_from_slice(recorder::MAGIC);
        b[8..12].copy_from_slice(&recorder::VERSION.to_le_bytes());
        b[12..16].copy_from_slice(&32u32.to_le_bytes());
        let mut put = |rtype: u16, host_ns: u64, payload: &[u8]| {
            b.extend(rtype.to_le_bytes());
            b.extend(0u16.to_le_bytes());
            b.extend((payload.len() as u32).to_le_bytes());
            b.extend(host_ns.to_le_bytes());
            b.extend(payload);
        };
        // the headset's clock is 7 ms behind the host's and wraps 10 ms in
        let stamp = |t_us: u32| t_us.wrapping_sub(17_000);
        for t in (1000..ms * 1000).step_by(2000) {
            let host = u64::from(t) * 1000;
            put(
                2,
                host,
                &[&0u32.to_le_bytes()[..], &imu(stamp(t - 1000))].concat(),
            );
            put(
                2,
                host + 100_000,
                &[&0u32.to_le_bytes()[..], &imu(stamp(t))].concat(),
            );
            let right = (t % 4000 == 1000).then(|| report(t as u8 ^ 0x55));
            put(
                2,
                host + 500_000,
                &[&1u32.to_le_bytes()[..], &radio(report(t as u8), right)].concat(),
            );
        }
        for k in 1..u64::from(ms) * 1000 / 19_200 {
            let mut info = recorder::FrameInfo {
                exposure_valid: 1,
                exposure_hmd_ts: stamp((k * 19_200) as u32),
                frame_start_host_ns: k * 19_200_000 + 3_000_000,
                ..Default::default()
            };
            info.serial[0] = b'S' as _;
            // processed, and recorded, 8 ms after the frame came
            put(4, k * 19_200_000 + 11_000_000, recorder::as_bytes(&info));
        }
        b
    }

    fn played(ms: u32) -> (Vec<Item>, Sim, Vec<[u8; RECORD_SIZE]>) {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("s.rec");
        std::fs::write(&path, recording(ms)).unwrap();
        let items = items(&path).unwrap();
        let mut sim = Sim::new(config());
        let mut out = sim.opened();
        for item in &items {
            out.extend(sim.play(item));
        }
        (items, sim, out)
    }

    fn events(records: &[[u8; RECORD_SIZE]]) -> Vec<Event> {
        let mut p = Parser::new();
        p.push(&records.concat());
        let events: Vec<Event> = std::iter::from_fn(|| p.next_event()).collect();
        assert_eq!(p.losses, Default::default());
        events
    }

    /// The dongle's clock comes from the last report of each burst, never from the stale first.
    #[test]
    fn the_clock_is_read_from_the_fresh_report_of_each_burst() {
        let (items, ..) = played(100);
        let clocks: Vec<(u64, u32)> = items
            .iter()
            .filter_map(|i| match i {
                Item::Clock { host_ns, us } => Some((*host_ns, *us)),
                _ => None,
            })
            .collect();
        assert_eq!(clocks.len(), 50);
        for (host_ns, us) in clocks {
            assert_eq!(host_ns % 2_000_000, 1_100_000);
            assert_eq!(us, ((host_ns / 1000) as u32 - 100).wrapping_sub(17_000));
        }
    }

    /// Every Touch message comes out as a `CV1I` that decrypts to it, from 0 to 5 slots late, on
    /// the headset's clock started near its wrap; status code 7 every 10 ms; the exposures on
    /// the grid the period gives.
    #[test]
    fn reports_and_the_poll_clock_are_the_recordings() {
        let (items, sim, out) = played(1000);
        let touches: Vec<(u8, [u8; REPORT_SIZE])> = items
            .iter()
            .filter_map(|i| match i {
                Item::Touch {
                    session, report, ..
                } => Some((*session, *report)),
                _ => None,
            })
            .collect();
        assert_eq!(touches.len(), 750);
        let events = events(&out);
        let mut seen = Vec::new();
        let mut clocks = Vec::new();
        for e in &events {
            match e {
                Event::Report { input, report } => {
                    seen.push((input.session, *report, input.poll_us))
                }
                Event::Status(s) => clocks.extend(s.poll_clock()),
                Event::Reply(_) => unreachable!(),
            }
        }
        assert_eq!(seen.iter().map(|s| (s.0, s.1)).collect::<Vec<_>>(), touches);
        // answering 0, 1, ... 5 slots late in turn
        let inputs = out.iter().filter(|r| r.starts_with(b"CV1I"));
        for (i, (r, s)) in inputs.zip(&seen).enumerate() {
            let late = i as u32 % 6;
            assert_eq!(
                *r,
                encode_input(&KEY, i as u32, s.2, late, s.0, s.0 - 1, &s.1)
            );
        }
        // 10 ms apart, on the dongle's clock as on the headset's
        assert_eq!(clocks.len(), 100);
        for w in clocks.windows(2) {
            assert_eq!(w[1].0.wrapping_sub(w[0].0), 10_000, "{w:?}");
            assert_eq!(w[1].1, DEFAULT_PERIOD_US);
        }
        // the dongle's clock starts where it was set, and wraps
        assert_eq!(clocks[0].0, config().start_us);
        assert!(clocks[99].0 < 1_000_000);
        // a report answers the last fresh stamp before it, 0.4 ms earlier
        let first_report_clock = seen[0].2;
        assert_eq!(first_report_clock, clocks[0].0);
        assert_eq!(sim.exposures.len(), 51);
        let e = &sim.exposures;
        assert!(e
            .windows(2)
            .all(|w| w[1].device_us.wrapping_sub(w[0].device_us) == 19_200));
        // after the 3 x 500 reports, in the order the recording has them
        assert_eq!((e[0].record, e[0].played_ns), (1500, 0));
        assert_eq!(e[0].host_ns, 19_200_000 + 3_000_000);
    }

    /// Config reads are answered from the session's flash in the layout the host reads; a read
    /// past the flash, of another opcode or for a controller with none goes unanswered; `X` sets
    /// the announced period; haptics and the rest are recorded.
    #[test]
    fn commands_are_answered_and_recorded() {
        let mut c = config();
        c.flash[1] = None;
        let mut sim = Sim::new(c);
        let mut out = sim.command(1, &crate::freetouch::cmd::config_read(2, 0x1bf0, 16)[..4]);
        out.extend(sim.command(2, &crate::freetouch::cmd::config_read(2, 0x1bf0, 16)[4..]));
        let reply = |out: &[[u8; RECORD_SIZE]]| {
            events(out).into_iter().find_map(|e| match e {
                Event::Reply(r) => Some(r),
                _ => None,
            })
        };
        let r = reply(&out).unwrap();
        assert_eq!(
            crate::dongle::flash_reply(r.frame(), 2, 0x1bf0, 16),
            Some(&[0x11; 16][..])
        );
        let r = reply(&sim.command(3, &crate::freetouch::cmd::config_read(2, 4, 11))).unwrap();
        assert_eq!(
            crate::dongle::flash_reply(r.frame(), 2, 4, 11),
            Some(&b"{\"left\": 1}"[..])
        );
        assert!(reply(&sim.command(4, &crate::freetouch::cmd::config_read(3, 0, 20))).is_none());
        assert!(
            reply(&sim.command(5, &crate::freetouch::cmd::config_read(2, 0x1bf8, 16))).is_none()
        );
        let mut info = crate::freetouch::cmd::config_read(2, 0, 20);
        info[1] = 0x88;
        assert!(reply(&sim.command(6, &info)).is_none());
        sim.command(7, &crate::freetouch::cmd::haptic(3, 200, 160));
        sim.command(8, &crate::freetouch::cmd::exposure_period(16_000));
        sim.command(9, &crate::freetouch::cmd::exposure_phase(-40_000));
        let out = sim.command(10, b"I?");
        assert_eq!(out, [sim.identity()]);
        assert_eq!(sim.period_us(), 16_000);
        assert_eq!(sim.phase_us, -33_333);
        let commands: Vec<Command> = sim.commands.iter().map(|c| c.1.clone()).collect();
        let read = |session, offset, length, answered| Command::Read {
            opcode: 0x0a,
            session,
            offset,
            length,
            answered,
        };
        assert_eq!(
            commands,
            [
                read(2, 0x1bf0, 16, true),
                read(2, 4, 11, true),
                read(3, 0, 20, false),
                read(2, 0x1bf8, 16, false),
                Command::Read {
                    opcode: 0x88,
                    session: 2,
                    offset: 0,
                    length: 20,
                    answered: false
                },
                Command::Haptic {
                    session: 3,
                    amplitude: 200,
                    frequency_hz: 160
                },
                Command::ExposurePeriod(16_000),
                Command::ExposurePhase(-40_000),
                Command::Identity,
                Command::Other(vec![b'?']),
            ]
        );
    }

    /// The dongle's clock drifts from the host's: at 100 ppm fast, a second of it plays in
    /// 0.9999 s.
    #[test]
    fn the_recording_plays_at_the_drift() {
        let sim = Sim::new(config());
        let ns = sim.playback(std::time::Duration::from_secs(1)).as_nanos();
        assert!(ns.abs_diff(999_900_010) <= 1, "{ns}");
    }

    #[cfg(target_os = "linux")]
    mod on_a_pty {
        use std::path::PathBuf;
        use std::sync::mpsc;
        use std::time::Duration;

        use super::*;
        use crate::dongle::{Dongle, Flash, Hooks, Refusal};

        fn hooks(clock: mpsc::Sender<(u32, Option<u32>)>) -> Hooks {
            let clock = std::sync::Mutex::new(clock);
            Hooks {
                clock: Box::new(move |_, us, period| {
                    let _ = clock.lock().unwrap().send((us, period));
                }),
                exposure: Box::new(|| None),
                log: Box::new(|_| {}),
            }
        }

        /// The simulator's thread: at the end, the simulator, what it served and the frames it
        /// handed over.
        type Server = std::thread::JoinHandle<(Sim, Served, Vec<Exposure>)>;

        /// Serves `ms` of the synthetic recording on a pty from another thread; the ports it
        /// plugs in, and at the end the simulator and what it served.
        fn serving(ms: u32, plan: Plan) -> (mpsc::Receiver<PathBuf>, Server, Vec<Item>) {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("s.rec");
            std::fs::write(&path, recording(ms)).unwrap();
            let items = items(&path).unwrap();
            let (tx, rx) = mpsc::channel();
            let played = items.clone();
            let server = std::thread::spawn(move || {
                let mut sim = Sim::new(config());
                let mut frames = Vec::new();
                let served = serve(
                    &mut sim,
                    &played,
                    &plan,
                    &mut |p| {
                        let _ = tx.send(p.to_owned());
                        Ok(())
                    },
                    &mut |e| frames.push(e.clone()),
                    &mut |_| {},
                )
                .unwrap();
                (sim, served, frames)
            });
            (rx, server, items)
        }

        fn flash(d: &Dongle, session: u8, offset: u16, length: u16) -> Vec<u8> {
            let mut out = vec![0; usize::from(length)];
            for _ in 0..200 {
                match d.read_flash(session, offset, length, &mut out) {
                    Flash::Pending => std::thread::sleep(Duration::from_millis(5)),
                    Flash::Done => return out,
                    Flash::Failed => panic!("read failed"),
                }
            }
            panic!("read timed out")
        }

        /// The host's dongle code on the simulated dongle's port: it is identified, its reports
        /// are the recording's radio messages, its clock reaches the hook, calibration reads and
        /// haptics go through, and an unplug ends it until the replugged port is opened again.
        #[test]
        fn the_dongle_code_runs_on_the_simulated_dongle() {
            let plan = Plan {
                unplug: vec![(Duration::from_millis(1500), Duration::from_millis(100))],
                ..Plan::default()
            };
            let (ports, server, items) = serving(3000, plan);
            let (clock_tx, clock_rx) = mpsc::channel();
            let port = ports.recv().unwrap();
            let d = Dongle::open(&port, hooks(clock_tx.clone())).unwrap();
            assert_eq!(d.identity.key, KEY);
            assert_eq!(flash(&d, 2, 0x1bf0, 16), [0x11; 16]);
            assert_eq!(flash(&d, 3, 4, 12), b"{\"right\": 2}");
            d.haptic(3, true, 255).unwrap();
            let expected: Vec<[u8; 64]> = items
                .iter()
                .filter_map(|i| match i {
                    Item::Touch {
                        session, report, ..
                    } => crate::freetouch::radio_report(*session, report),
                    _ => None,
                })
                .collect();
            let mut got = Vec::new();
            let mut buf = [0u8; 64];
            let gone = loop {
                match d.read_report(&mut buf, Duration::from_secs(3)) {
                    Ok(Some(_)) => got.push(buf),
                    Ok(None) => panic!("no report and no unplug"),
                    Err(e) => break e,
                }
            };
            assert_eq!(gone.kind(), std::io::ErrorKind::BrokenPipe);
            // about 1.5 s of the recording, in order and none changed
            assert!(got.len() > 500, "{}", got.len());
            assert_eq!(got[..], expected[..got.len()]);
            let clocks: Vec<(u32, Option<u32>)> = clock_rx.try_iter().collect();
            let status_clocks: Vec<(u32, u32)> = clocks
                .iter()
                .filter_map(|&(us, p)| p.map(|p| (us, p)))
                .collect();
            assert!(
                status_clocks.len() > 100 && status_clocks.iter().all(|c| c.1 == DEFAULT_PERIOD_US)
            );
            assert!(clocks.iter().any(|c| c.1.is_none()));
            drop(d);
            let again = ports.recv().unwrap();
            let d = Dongle::open(&again, hooks(clock_tx)).unwrap();
            assert!(d
                .read_report(&mut buf, Duration::from_secs(1))
                .unwrap()
                .is_some());
            drop(d);
            let (sim, served, frames) = server.join().unwrap();
            // each frame handed over as its arrival played, on the announced grid
            assert_eq!(frames, sim.exposures);
            assert!(
                frames.len() > 100
                    && frames.windows(2).all(|w| w[1].played_ns > w[0].played_ns
                        && w[1].record > w[0].record
                        && w[1].device_us.wrapping_sub(w[0].device_us) == sim.period_us())
            );
            assert_eq!(served.opens, 2);
            assert!(sim.commands.contains(&(
                sim.commands
                    .iter()
                    .find(|c| matches!(c.1, Command::Haptic { .. }))
                    .unwrap()
                    .0,
                Command::Haptic {
                    session: 3,
                    amplitude: 255,
                    frequency_hz: 160
                }
            )));
        }

        /// Another device on the same port, sending text: refused, and sent nothing.
        #[test]
        fn a_foreign_device_receives_nothing() {
            let plan = Plan {
                foreign: true,
                until: Some(Duration::from_millis(2500)),
                ..Plan::default()
            };
            let (ports, server, _) = serving(3000, plan);
            let (clock_tx, _clock_rx) = mpsc::channel();
            let refused = Dongle::open(&ports.recv().unwrap(), hooks(clock_tx));
            assert!(matches!(refused, Err(Refusal::Silent)));
            let (_, served, _) = server.join().unwrap();
            assert_eq!(served.opens, 1);
            assert_eq!(served.foreign_bytes, 0);
            assert!(served.records == 0);
        }

        /// A host that keeps the slave port open without reading must not stall playback, frame
        /// delivery, scheduled unplugs, or `Plan::until` even after overflowing the kernel PTY
        /// buffer and the simulator's bounded outgoing queue.
        #[test]
        fn a_stalled_host_does_not_block_playback_or_unplug() {
            let plan = Plan {
                unplug: vec![(Duration::from_millis(300), Duration::from_millis(50))],
                until: Some(Duration::from_millis(650)),
                ..Plan::default()
            };
            // Push >250 KiB of controller records in the first 250 ms — well past Linux's ~64 KiB
            // PTY buffer and `MAX_PENDING_BYTES` — alongside normal clock and exposure items.
            let mut played = Vec::with_capacity(5500);
            let base_ns = 1_000_000u64;
            let total_touches = 5_000u64;
            for i in 0..total_touches {
                let t_us = i * 50;
                let host_ns = base_ns + t_us * 1000;
                if i % 40 == 0 {
                    played.push(Item::Clock {
                        host_ns,
                        us: t_us as u32,
                    });
                }
                played.push(Item::Touch {
                    host_ns,
                    session: 2,
                    report: report((i & 0xff) as u8),
                });
            }
            for t_ms in (250u64..800).step_by(2) {
                let t_us = t_ms * 1000;
                played.push(Item::Clock {
                    host_ns: base_ns + t_us * 1000,
                    us: t_us as u32,
                });
            }
            for k in 0u64..40 {
                let exp_us = k * u64::from(DEFAULT_PERIOD_US);
                played.push(Item::Exposure {
                    host_ns: base_ns + (exp_us + 500) * 1000,
                    us: exp_us as u32,
                    serial: "WMTD1".into(),
                    record: k + 1,
                });
            }
            played.sort_by_key(Item::host_ns);

            let (tx, ports) = mpsc::channel();
            let server = std::thread::spawn(move || {
                let mut sim = Sim::new(config());
                let mut frames = Vec::new();
                let served = serve(
                    &mut sim,
                    &played,
                    &plan,
                    &mut |p| {
                        let _ = tx.send(p.to_owned());
                        Ok(())
                    },
                    &mut |e| frames.push(e.clone()),
                    &mut |_| {},
                )
                .unwrap();
                (sim, served, frames)
            });

            let port = ports.recv_timeout(Duration::from_secs(2)).unwrap();
            // Open the slave port and intentionally never read from it so the kernel PTY buffer
            // fills completely during playback.
            let stalled = std::fs::File::options()
                .read(true)
                .write(true)
                .open(&port)
                .unwrap();
            // The scheduled unplug must still fire on time and expose the replacement port.
            let replugged = ports
                .recv_timeout(Duration::from_secs(2))
                .expect("scheduled unplug stalled on blocked PTY write");
            assert_ne!(replugged, port);
            let (done_tx, done_rx) = mpsc::channel();
            std::thread::spawn(move || {
                let _ = done_tx.send(server.join().unwrap());
            });
            let (sim, served, frames) = done_rx
                .recv_timeout(Duration::from_secs(2))
                .expect("serve hung instead of stopping at Plan::until");
            drop(stalled);
            assert_eq!(served.opens, 1);
            assert!(
                served.records > 0 && served.records < total_touches,
                "expected bounded queue to drop excess records while stalled, served={}",
                served.records
            );
            assert_eq!(frames, sim.exposures);
            assert!(
                frames.len() >= 25,
                "expected frames through until: {}",
                frames.len()
            );
        }
    }
}
