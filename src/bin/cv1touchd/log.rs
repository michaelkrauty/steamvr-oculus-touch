// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Bounded, best-effort daemon output. Only the detached writer touches stderr.
use std::fmt::{self, Write};
use std::io;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread;
use std::time::{Duration, Instant};

const CAPACITY: usize = 256;
const LINE_BYTES: usize = 2048;
const TRUNCATED: &str = " [truncated]";
const EXIT_FLUSH: Duration = Duration::from_millis(200);
const POLL: Duration = Duration::from_millis(2);

struct Line {
    bytes: [u8; LINE_BYTES],
    len: usize,
    truncated: bool,
}

impl Line {
    fn new() -> Self {
        Self {
            bytes: [0; LINE_BYTES],
            len: 0,
            truncated: false,
        }
    }

    fn formatted(args: fmt::Arguments<'_>) -> Self {
        let mut line = Self::new();
        let _ = line.write_fmt(args);
        if line.truncated {
            let end = line.len + TRUNCATED.len();
            line.bytes[line.len..end].copy_from_slice(TRUNCATED.as_bytes());
            line.len = end;
        }
        line.bytes[line.len] = b'\n';
        line.len += 1;
        line
    }

    fn bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl fmt::Write for Line {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let mut n = text.len().min(LINE_BYTES - TRUNCATED.len() - 1 - self.len);
        while !text.is_char_boundary(n) {
            n -= 1;
        }
        self.bytes[self.len..self.len + n].copy_from_slice(&text.as_bytes()[..n]);
        self.len += n;
        if n != text.len() {
            self.truncated = true;
            Err(fmt::Error)
        } else {
            Ok(())
        }
    }
}

/// A bounded MPSC ring with one consumer. A producer only tries its free slot once; competing
/// producers can lose a line, but the consumer never holds a slot advertised as free. No spinning,
/// allocation, or waiting on the producer path, including when another producer was preempted.
struct Queue {
    slots: Box<[Mutex<Option<Line>>]>,
    head: AtomicUsize,
    tail: AtomicUsize,
}

impl Queue {
    fn new() -> Self {
        Self {
            slots: (0..CAPACITY).map(|_| Mutex::new(None)).collect(),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    fn push(&self, line: Line) -> bool {
        let tail = self.tail.load(Ordering::Acquire);
        let head = self.head.load(Ordering::Acquire);
        if tail.wrapping_sub(head) >= CAPACITY {
            return false;
        }
        let Ok(mut slot) = self.slots[tail % CAPACITY].try_lock() else {
            return false;
        };
        if self
            .tail
            .compare_exchange(
                tail,
                tail.wrapping_add(1),
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_err()
        {
            return false;
        }
        *slot = Some(line);
        true
    }

    fn pop(&self) -> Option<Line> {
        let head = self.head.load(Ordering::Relaxed);
        if head == self.tail.load(Ordering::Acquire) {
            return None;
        }
        let line = self.slots[head % CAPACITY]
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        // The guard above is released before advertising that this slot can be reused.
        self.head.store(head.wrapping_add(1), Ordering::Release);
        line
    }
}

struct Shared {
    queue: Queue,
    dropped: AtomicU64,
    pending: AtomicUsize,
    writing: AtomicBool,
    closed: AtomicBool,
}

#[derive(Clone)]
struct Logger {
    shared: Arc<Shared>,
}

impl Logger {
    fn new(writer: impl io::Write + Send + 'static) -> io::Result<Self> {
        let shared = Arc::new(Shared {
            queue: Queue::new(),
            dropped: AtomicU64::new(0),
            pending: AtomicUsize::new(0),
            writing: AtomicBool::new(false),
            closed: AtomicBool::new(false),
        });
        let worker = shared.clone();
        thread::Builder::new()
            .name("cv1t-log".into())
            .spawn(move || {
                write_loop(worker, writer);
            })?;
        Ok(Self { shared })
    }

    fn enqueue(&self, line: Line) {
        if self.shared.closed.load(Ordering::Acquire) {
            return;
        }
        self.shared.pending.fetch_add(1, Ordering::Release);
        if !self.shared.queue.push(line) {
            self.shared.dropped.fetch_add(1, Ordering::Relaxed);
            self.shared.pending.fetch_sub(1, Ordering::Release);
        }
    }

    fn line(&self, args: fmt::Arguments<'_>) {
        self.enqueue(Line::formatted(format_args!("cv1touchd: {args}")));
    }

    fn bytes(&self, bytes: &[u8]) {
        // Native callbacks hand over one complete, already bounded line, copied before returning.
        if bytes.len() <= LINE_BYTES {
            let mut line = Line::new();
            line.bytes[..bytes.len()].copy_from_slice(bytes);
            line.len = bytes.len();
            self.enqueue(line);
        }
    }

    fn flush(&self, within: Duration) -> bool {
        let start = Instant::now();
        loop {
            if self.shared.pending.load(Ordering::Acquire) == 0
                && self.shared.dropped.load(Ordering::Acquire) == 0
                && !self.shared.writing.load(Ordering::Acquire)
            {
                return true;
            }
            if start.elapsed() >= within {
                return false;
            }
            thread::sleep(Duration::from_millis(1));
        }
    }

    fn close(&self, within: Duration) -> bool {
        self.shared.closed.store(true, Ordering::Release);
        self.flush(within)
    }
}

fn write_loop(shared: Arc<Shared>, mut writer: impl io::Write) {
    loop {
        shared.writing.store(true, Ordering::Release);
        while let Some(line) = shared.queue.pop() {
            let written = writer
                .write_all(line.bytes())
                .and_then(|()| writer.flush())
                .is_ok();
            if !written {
                shared.dropped.fetch_add(1, Ordering::Relaxed);
            }
            shared.pending.fetch_sub(1, Ordering::Release);
            // Report losses after a successful write, including losses while that write blocked.
            if written {
                report_drops(&shared, &mut writer);
            }
        }
        report_drops(&shared, &mut writer);
        shared.writing.store(false, Ordering::Release);
        if shared.closed.load(Ordering::Acquire)
            && shared.pending.load(Ordering::Acquire) == 0
            && shared.dropped.load(Ordering::Acquire) == 0
        {
            return;
        }
        thread::sleep(POLL);
    }
}

fn report_drops(shared: &Shared, writer: &mut impl io::Write) {
    let count = shared.dropped.swap(0, Ordering::AcqRel);
    if count != 0 {
        let line = Line::formatted(format_args!("cv1touchd: {count} log lines dropped"));
        if writer
            .write_all(line.bytes())
            .and_then(|()| writer.flush())
            .is_err()
        {
            shared.dropped.fetch_add(count, Ordering::Relaxed);
        }
    }
}

static LOGGER: OnceLock<Logger> = OnceLock::new();

pub(super) struct Guard;

/// Called on the main thread, after it takes shutdown requests and before any engine is opened.
pub(super) fn init() -> io::Result<Guard> {
    if LOGGER.get().is_some() {
        return Ok(Guard);
    }
    install(Logger::new(io::stderr())?)
}

/// A test installs this once in an isolated child process, before any callers start logging.
/// It uses exactly the production ring, writer, native callback and shutdown guard.
#[cfg(test)]
pub(super) fn init_with_writer(writer: impl io::Write + Send + 'static) -> io::Result<Guard> {
    if LOGGER.get().is_some() {
        return Err(io::Error::new(
            io::ErrorKind::AlreadyExists,
            "daemon logger already installed",
        ));
    }
    install(Logger::new(writer)?)
}

fn install(logger: Logger) -> io::Result<Guard> {
    if let Err(unused) = LOGGER.set(logger) {
        unused.close(Duration::ZERO);
        return Err(io::Error::new(
            io::ErrorKind::AlreadyExists,
            "daemon logger already installed",
        ));
    }
    // SAFETY: installed once, before any native worker exists; never changed afterwards.
    unsafe {
        cv1touch::ffi::ohmd_cv1touch_set_log_callback(Some(native_line));
        cv1touch::ffi::ohmd_cv1touch_set_log_fatal_callback(Some(native_fatal));
    };
    // The default hook writes synchronously to stderr, including on a tracking thread.
    std::panic::set_hook(Box::new(|info| line(format_args!("{info}"))));
    Ok(Guard)
}

pub(super) fn line(args: fmt::Arguments<'_>) {
    #[cfg(test)]
    if CAPTURED
        .with_borrow_mut(|lines| lines.as_mut().map(|lines| lines.push(args.to_string())))
        .is_some()
    {
        return;
    }
    if let Some(logger) = LOGGER.get() {
        logger.line(args);
    }
}

#[cfg(test)]
thread_local! {
    static CAPTURED: std::cell::RefCell<Option<Vec<String>>> = const { std::cell::RefCell::new(None) };
}

/// The lines `run` logs on this thread, kept from the logger. Tests only.
#[cfg(test)]
pub(super) fn capture(run: impl FnOnce()) -> Vec<String> {
    let outer = CAPTURED.replace(Some(Vec::new()));
    run();
    CAPTURED.replace(outer).unwrap_or_default()
}

pub(super) fn raw(args: fmt::Arguments<'_>) {
    if let Some(logger) = LOGGER.get() {
        logger.enqueue(Line::formatted(args));
    }
}

unsafe extern "C" fn native_line(bytes: *const std::ffi::c_char, len: usize) {
    cv1touch::engine::ffi_callback(|| {
        if let Some(logger) = LOGGER.get() {
            // SAFETY: the native adapter keeps this bounded buffer alive throughout the callback.
            logger.bytes(unsafe { std::slice::from_raw_parts(bytes.cast(), len) });
        }
    });
}

unsafe extern "C" fn native_fatal() {
    cv1touch::engine::ffi_callback(|| {
        if let Some(logger) = LOGGER.get() {
            logger.close(EXIT_FLUSH);
        }
    });
    // Return to native abort(); never wait for an engine worker or the detached writer.
}

pub(super) fn exit(code: i32) -> ! {
    if let Some(logger) = LOGGER.get() {
        logger.close(EXIT_FLUSH);
    }
    std::process::exit(code);
}

impl Drop for Guard {
    fn drop(&mut self) {
        if let Some(logger) = LOGGER.get() {
            logger.close(EXIT_FLUSH);
        }
        // Never join: the writer may be in an indefinitely blocked OS write. Process exit ends it.
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    struct Output {
        bytes: Arc<Mutex<Vec<u8>>>,
        gate: Option<(mpsc::SyncSender<()>, mpsc::Receiver<()>)>,
    }

    impl io::Write for Output {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            if let Some((entered, release)) = self.gate.take() {
                entered.send(()).unwrap();
                release.recv_timeout(Duration::from_secs(5)).unwrap();
            }
            self.bytes.lock().unwrap().extend_from_slice(bytes);
            Ok(bytes.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    struct Blocked {
        logger: Logger,
        entered: mpsc::Receiver<()>,
        release: mpsc::SyncSender<()>,
        bytes: Arc<Mutex<Vec<u8>>>,
    }

    fn blocked() -> Blocked {
        let bytes = Arc::new(Mutex::new(Vec::new()));
        let (entered, wait) = mpsc::sync_channel(1);
        let (release, gate) = mpsc::sync_channel(1);
        let logger = Logger::new(Output {
            bytes: bytes.clone(),
            gate: Some((entered, gate)),
        })
        .unwrap();
        Blocked {
            logger,
            entered: wait,
            release,
            bytes,
        }
    }

    #[test]
    fn a_blocked_writer_does_not_hold_a_radio_caller_or_its_publication_lock() {
        let Blocked {
            logger,
            entered,
            release,
            ..
        } = blocked();
        logger.line(format_args!("first"));
        entered.recv_timeout(Duration::from_secs(2)).unwrap();
        let publication = Arc::new(Mutex::new(()));
        let lock = publication.clone();
        let (sent, done) = mpsc::channel();
        let caller = logger.clone();
        let radio = thread::Builder::new()
            .name("radio-log-test".into())
            .spawn(move || {
                {
                    let _publishing = lock.lock().unwrap();
                    for n in 0..4096 {
                        caller.line(format_args!("radio {n}"));
                    }
                }
                sent.send(()).unwrap();
            })
            .unwrap();
        let completed = done.recv_timeout(Duration::from_millis(250));
        let lock_free = publication.try_lock().is_ok();
        release.send(()).unwrap();
        radio.join().unwrap();
        logger.close(Duration::from_secs(2));
        assert!(completed.is_ok(), "radio logging waited for the writer");
        assert!(lock_free, "logging kept the publication lock held");
    }

    #[test]
    fn a_full_queue_counts_drops_and_reports_them_once_when_writing_resumes() {
        let Blocked {
            logger,
            entered,
            release,
            bytes,
        } = blocked();
        logger.line(format_args!("first"));
        entered.recv_timeout(Duration::from_secs(2)).unwrap();
        for n in 0..CAPACITY + 37 {
            logger.line(format_args!("queued {n}"));
        }
        let dropped = logger.shared.dropped.load(Ordering::Relaxed);
        release.send(()).unwrap();
        assert!(logger.close(Duration::from_secs(2)));
        assert_eq!(dropped, 37);
        let text = String::from_utf8(bytes.lock().unwrap().clone()).unwrap();
        assert_eq!(text.matches("37 log lines dropped").count(), 1, "{text}");
        for n in 0..CAPACITY {
            assert!(text.contains(&format!("cv1touchd: queued {n}\n")));
        }
        assert!(!text.contains(&format!("cv1touchd: queued {CAPACITY}\n")));
    }

    struct FailFirst(Output);

    impl io::Write for FailFirst {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            if let Some((entered, release)) = self.0.gate.take() {
                entered.send(()).unwrap();
                release.recv_timeout(Duration::from_secs(5)).unwrap();
                return Err(io::Error::other("temporary output failure"));
            }
            io::Write::write(&mut self.0, bytes)
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn recovered_writes_report_drops_before_draining_the_backlog() {
        let bytes = Arc::new(Mutex::new(Vec::new()));
        let (entered, wait) = mpsc::sync_channel(1);
        let (release, gate) = mpsc::sync_channel(1);
        let logger = Logger::new(FailFirst(Output {
            bytes: bytes.clone(),
            gate: Some((entered, gate)),
        }))
        .unwrap();
        logger.line(format_args!("fails"));
        wait.recv_timeout(Duration::from_secs(2)).unwrap();
        for n in 0..3 {
            logger.line(format_args!("queued {n}"));
        }
        release.send(()).unwrap();
        assert!(logger.close(Duration::from_secs(2)));
        assert_eq!(&*bytes.lock().unwrap(), b"cv1touchd: queued 0\ncv1touchd: 1 log lines dropped\ncv1touchd: queued 1\ncv1touchd: queued 2\n");
    }

    #[test]
    fn a_contended_queue_never_waits_for_its_lock() {
        let bytes = Arc::new(Mutex::new(Vec::new()));
        let logger = Logger::new(Output { bytes, gate: None }).unwrap();
        let guard = logger.shared.queue.slots[0].lock().unwrap();
        let (sent, done) = mpsc::channel();
        let caller = logger.clone();
        let t = thread::spawn(move || {
            caller.line(format_args!("contended"));
            sent.send(()).unwrap();
        });
        let completed = done.recv_timeout(Duration::from_millis(250));
        drop(guard);
        t.join().unwrap();
        assert!(logger.close(Duration::from_secs(2)));
        assert!(completed.is_ok(), "enqueue waited for the queue lock");
    }

    #[test]
    fn an_injected_global_sink_uses_the_production_logger_without_blocking_publication() {
        const CHILD: &str = "CV1T_BLOCKED_LOG_TEST_CHILD";
        if std::env::var_os(CHILD).is_some() {
            let bytes = Arc::new(Mutex::new(Vec::new()));
            let (entered, wait) = mpsc::sync_channel(1);
            let (release, gate) = mpsc::sync_channel(1);
            let logging = init_with_writer(Output {
                bytes: bytes.clone(),
                gate: Some((entered, gate)),
            })
            .unwrap();
            assert!(
                matches!(init_with_writer(io::sink()), Err(e) if e.kind() == io::ErrorKind::AlreadyExists)
            );
            // Use the daemon's original free helper, just as a sibling publication test does.
            super::super::log("first");
            wait.recv_timeout(Duration::from_secs(2)).unwrap();
            let publication = Arc::new(Mutex::new(()));
            let lock = publication.clone();
            let (sent, done) = mpsc::channel();
            let caller = thread::spawn(move || {
                {
                    let _publishing = lock.lock().unwrap();
                    super::super::log("while publishing");
                }
                sent.send(()).unwrap();
            });
            let completed = done.recv_timeout(Duration::from_millis(250));
            let lock_free = publication.try_lock().is_ok();
            release.send(()).unwrap();
            caller.join().unwrap();
            drop(logging);
            assert!(completed.is_ok());
            assert!(lock_free);
            assert_eq!(
                &*bytes.lock().unwrap(),
                b"cv1touchd: first\ncv1touchd: while publishing\n"
            );
            return;
        }
        let out = std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "daemon_log::tests::an_injected_global_sink_uses_the_production_logger_without_blocking_publication", "--nocapture"])
            .env(CHILD, "1").output().unwrap();
        assert!(
            out.status.success(),
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
    }

    #[test]
    fn exit_flush_has_a_deadline_even_when_the_writer_is_stuck() {
        let Blocked {
            logger,
            entered,
            release,
            ..
        } = blocked();
        logger.line(format_args!("first"));
        entered.recv_timeout(Duration::from_secs(2)).unwrap();
        let start = Instant::now();
        let drained = logger.close(EXIT_FLUSH);
        let elapsed = start.elapsed();
        release.send(()).unwrap();
        assert!(!drained);
        assert!(
            elapsed < Duration::from_millis(500),
            "flush took {elapsed:?}"
        );
    }
}

#[cfg(test)]
mod rate_tests {
    use super::*;

    struct Collect(Arc<Mutex<Vec<u8>>>);
    impl io::Write for Collect {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(bytes);
            Ok(bytes.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn a_working_sink_keeps_radio_cadence_and_minute_bursts_without_drops() {
        let bytes = Arc::new(Mutex::new(Vec::new()));
        let logger = Logger::new(Collect(bytes.clone())).unwrap();
        // A thousand calls a second is much more logging than the radio normally does.
        // Advance the statistics clock separately to exercise three minute boundaries.
        for n in 0..1200 {
            logger.line(format_args!("radio {n}"));
            if n % 400 == 0 {
                logger.bytes(b"[II] native update\n");
                logger.line(format_args!("left: IMU sample held 0.10 ms"));
                logger.line(format_args!("right: IMU sample held 0.10 ms"));
            }
            thread::sleep(Duration::from_millis(1));
        }
        assert!(logger.close(Duration::from_secs(2)));
        let text = String::from_utf8(bytes.lock().unwrap().clone()).unwrap();
        assert!(!text.contains("log lines dropped"), "{text}");
        assert_eq!(text.lines().count(), 1209);
        let mut expected = String::new();
        for n in 0..1200 {
            use std::fmt::Write;
            writeln!(expected, "cv1touchd: radio {n}").unwrap();
            if n % 400 == 0 {
                expected.push_str("[II] native update\ncv1touchd: left: IMU sample held 0.10 ms\ncv1touchd: right: IMU sample held 0.10 ms\n");
            }
        }
        assert_eq!(text, expected);
    }

    #[test]
    fn a_long_line_is_bounded_and_a_later_line_still_arrives() {
        let bytes = Arc::new(Mutex::new(Vec::new()));
        let logger = Logger::new(Collect(bytes.clone())).unwrap();
        logger.line(format_args!("{}", "é".repeat(10_000)));
        logger.line(format_args!("after"));
        assert!(logger.close(Duration::from_secs(2)));
        let text = String::from_utf8(bytes.lock().unwrap().clone()).unwrap();
        assert!(text.starts_with("cv1touchd: é"));
        assert!(text.ends_with(" [truncated]\ncv1touchd: after\n"));
        assert!(text.len() <= LINE_BYTES + 20);
    }

    #[test]
    fn competing_producers_keep_each_threads_order_and_account_for_every_line() {
        let bytes = Arc::new(Mutex::new(Vec::new()));
        let logger = Logger::new(Collect(bytes.clone())).unwrap();
        thread::scope(|scope| {
            for producer in 0..4 {
                let logger = &logger;
                scope.spawn(move || {
                    for n in 0..300 {
                        logger.line(format_args!("producer {producer} {n}"));
                    }
                });
            }
        });
        assert!(logger.close(Duration::from_secs(2)));
        let text = String::from_utf8(bytes.lock().unwrap().clone()).unwrap();
        let mut last = [None; 4];
        let mut accounted = 0;
        for line in text.lines() {
            let words: Vec<_> = line.split_whitespace().collect();
            if words[1] == "producer" {
                let producer: usize = words[2].parse().unwrap();
                let n: usize = words[3].parse().unwrap();
                assert!(last[producer].is_none_or(|prev| n > prev));
                last[producer] = Some(n);
                accounted += 1;
            } else {
                accounted += words[1].parse::<usize>().unwrap();
            }
        }
        assert_eq!(accounted, 1200);
    }

    #[cfg(unix)]
    #[test]
    fn native_and_rust_lines_share_the_daemon_sink_in_order() {
        const CHILD: &str = "CV1T_NATIVE_LOG_TEST_CHILD";
        if std::env::var_os(CHILD).is_some() {
            let _logging = init_with_writer(std::io::stderr()).unwrap();
            line(format_args!("before native"));
            // SAFETY: on Linux context creation only allocates driver tables; no probing,
            // devices, USB contexts, services, or configuration files are opened. An invalid
            // index is rejected before a driver's open function is considered.
            unsafe {
                let ctx = cv1touch::ffi::ohmd_ctx_create();
                assert!(!ctx.is_null());
                assert!(cv1touch::ffi::ohmd_list_open_device(ctx, -1).is_null());
                cv1touch::ffi::ohmd_ctx_destroy(ctx);
            }
            line(format_args!("after native"));
            return;
        }
        let out = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "daemon_log::rate_tests::native_and_rust_lines_share_the_daemon_sink_in_order",
                "--nocapture",
            ])
            .env(CHILD, "1")
            .output()
            .unwrap();
        assert!(out.status.success());
        let text = String::from_utf8(out.stderr).unwrap();
        assert!(text.contains("cv1touchd: before native\n[EE] no device with index: -1\ncv1touchd: after native\n"), "{text}");
        assert!(!String::from_utf8(out.stdout).unwrap().contains("[EE]"));
    }

    // Offline timing only. stderr must be redirected to the null device by the caller.
    #[test]
    #[ignore]
    fn radio_log_cost() {
        use std::io::Write;
        let mut before = Vec::new();
        let mut after = Vec::new();
        for _ in 0..10_000 {
            let at = Instant::now();
            let message = format!("left: radio back after {:.2} s", std::hint::black_box(0.5));
            let _ = writeln!(std::io::stderr(), "cv1touchd: {message}");
            before.push(at.elapsed().as_nanos());
        }
        let logging = init().unwrap();
        for n in 0..10_000 {
            let at = Instant::now();
            super::line(format_args!(
                "left: radio back after {:.2} s",
                std::hint::black_box(0.5)
            ));
            after.push(at.elapsed().as_nanos());
            if n % 100 == 0 {
                assert!(LOGGER.get().unwrap().flush(Duration::from_secs(1)));
            }
        }
        drop(logging);
        before.sort_unstable();
        after.sort_unstable();
        println!(
            "10000 calls, ns/call median/p99: synchronous {}/{}, queued {}/{}",
            before[5000], before[9900], after[5000], after[9900]
        );
    }
}

#[cfg(all(test, unix))]
mod assertion_tests {
    use super::*;
    use std::io::{Read, Write};
    use std::os::fd::{AsRawFd, FromRawFd};
    use std::os::unix::process::ExitStatusExt;
    use std::process::{Command, Stdio};

    fn native_assertion(full: bool, test: &str) {
        const CHILD: &str = "CV1T_NATIVE_ASSERT_CHILD";
        if std::env::var_os(CHILD).is_some() {
            // No devices or state: call the real math assertion with a NaN quaternion.
            let _logging = init().unwrap();
            // SAFETY: the fixture exposes the real math source with valid NaN input buffers.
            unsafe {
                let path = std::ffi::CString::new(concat!(
                    env!("CV1TOUCH_NATIVE_DIR"),
                    "/tests/libtest_native_assert_bridge.so"
                ))
                .unwrap();
                let library = libc::dlopen(path.as_ptr(), libc::RTLD_NOW);
                assert!(!library.is_null());
                let symbol = libc::dlsym(library, c"cv1touch_test_native_assert".as_ptr());
                assert!(!symbol.is_null());
                let fail: unsafe extern "C" fn() = std::mem::transmute(symbol);
                super::super::expected_abort::prepare();
                fail();
            }
            panic!("native assertion returned");
        }
        let mut fds = [-1; 2];
        // SAFETY: two descriptor slots; each returned fd is owned by one File below.
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
        let mut read = unsafe { std::fs::File::from_raw_fd(fds[0]) };
        let mut write = unsafe { std::fs::File::from_raw_fd(fds[1]) };
        let fd = write.as_raw_fd();
        let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
        assert!(flags >= 0);
        if full {
            assert_eq!(
                unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) },
                0
            );
            loop {
                match write.write(&[b'x'; 4096]) {
                    Ok(_) => {}
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                    Err(e) => panic!("fill stderr: {e}"),
                }
            }
            assert_eq!(unsafe { libc::fcntl(fd, libc::F_SETFL, flags) }, 0);
        }
        let observed = write.try_clone().unwrap();
        let mut child = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", test, "--nocapture"])
            .env(CHILD, "1")
            .stdout(Stdio::null())
            .stderr(Stdio::from(write))
            .spawn()
            .unwrap();
        let start = Instant::now();
        let status = loop {
            if let Some(status) = child.try_wait().unwrap() {
                break Some(status);
            }
            if start.elapsed() >= Duration::from_secs(2) {
                child.kill().unwrap();
                child.wait().unwrap();
                break None;
            }
            thread::sleep(Duration::from_millis(5));
        };
        let after = unsafe { libc::fcntl(observed.as_raw_fd(), libc::F_GETFL) };
        drop(observed);
        let mut bytes = Vec::new();
        read.read_to_end(&mut bytes).unwrap();
        assert!(
            status.is_some(),
            "native assertion waited for a full stderr pipe"
        );
        assert_eq!(status.unwrap().signal(), Some(libc::SIGABRT));
        assert_eq!(
            after, flags,
            "native assertion changed the pipe's blocking mode"
        );
        if !full {
            let text = String::from_utf8(bytes).unwrap();
            assert!(
                text.contains("omath.c:")
                    && text.contains("!isnan(rot->x)")
                    && text.contains("oquatf_to_rotation")
                    && text.contains("failed"),
                "{text}"
            );
        }
    }

    #[test]
    fn a_native_assertion_aborts_with_a_full_stderr_pipe() {
        native_assertion(
            true,
            "daemon_log::assertion_tests::a_native_assertion_aborts_with_a_full_stderr_pipe",
        );
    }

    #[test]
    fn a_native_assertion_reaches_a_healthy_stderr_pipe() {
        native_assertion(
            false,
            "daemon_log::assertion_tests::a_native_assertion_reaches_a_healthy_stderr_pipe",
        );
    }
}
