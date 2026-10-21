// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Supervising the daemon where no service manager does (Windows): the daemon runs as a child
//! process whose output goes to a log file, each line stamped with the time it arrived. After a
//! failure it is started again 2 s later, as systemd's Restart=on-failure and RestartSec=2 do on
//! Linux, unless it refused its arguments or has failed five times in a row within 30 s of
//! starting. Asked to stop, the supervisor asks the daemon to stop and waits for it.

use std::fs::File;
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, ExitStatus, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime};

use crate::log_lines::{LineBuffer, LINE_BYTES, READ_BYTES};
use crate::platform::{self, fs};

/// The log's file name; the earlier ones are cv1touchd.1.log (the newest) to cv1touchd.4.log.
const LOG_NAME: &str = "cv1touchd";
const EARLIER_LOGS: usize = 4;
/// Past this size a log is set aside like an earlier one, and a new one begun.
const LOG_MAX_BYTES: u64 = 16 << 20;
const LOG_QUEUE_LINES: usize = 1024;
const LOG_LOCK: &str = "logs.lock";
const TRUNCATED: &str = " [line truncated]";

fn shorten(text: &str) -> (&str, bool) {
    let mut end = text.len().min(LINE_BYTES);
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    (&text[..end], end < text.len())
}

/// Where the supervisor keeps the daemon's logs.
pub fn log_dir() -> PathBuf {
    platform::dirs::get().state.join("logs")
}

/// The daemon's current log.
pub fn log_file() -> PathBuf {
    log_path(&log_dir(), 0)
}

fn log_path(dir: &Path, age: usize) -> PathBuf {
    if age == 0 {
        dir.join(format!("{LOG_NAME}.log"))
    } else {
        dir.join(format!("{LOG_NAME}.{age}.log"))
    }
}

/// Disk staging keeps a rotation recoverable even if the supervisor exits between moves.
#[derive(Default)]
struct Rotation {
    pending: bool,
    #[cfg(test)]
    attempts: usize,
}

/// Each operation is injectable so a restart can be tested after any filesystem change.
enum RotationStep<'a> {
    Rename(&'a Path, &'a Path),
    Remove(&'a Path),
    CreateDir(&'a Path),
    Ready(&'a Path),
    RemoveDir(&'a Path),
}

impl RotationStep<'_> {
    fn run(self) -> io::Result<()> {
        match self {
            Self::Rename(from, to) => fs::rename_replacing(from, to),
            Self::Remove(path) => std::fs::remove_file(path),
            Self::CreateDir(path) => std::fs::create_dir(path),
            Self::Ready(path) => fs::create_private(path).map(drop),
            Self::RemoveDir(path) => std::fs::remove_dir(path),
        }
    }
}

fn rotation_dir(dir: &Path) -> PathBuf {
    dir.join(format!("{LOG_NAME}.log.rotation"))
}

/// Where the logs of an unfinished rotation wait. It exists only while a rotation is under way
/// or after one was interrupted.
pub fn staging_dir() -> PathBuf {
    rotation_dir(&log_dir())
}

/// An error naming `path`, the file that stops a recovery, and what is wrong with it.
fn in_the_way(kind: io::ErrorKind, path: &Path, why: impl std::fmt::Display) -> io::Error {
    io::Error::new(kind, format!("{} {why}", path.display()))
}

/// An I/O error on `path`, which it names, of the same kind.
fn failed_on<'a>(path: &'a Path, what: &'a str) -> impl FnOnce(io::Error) -> io::Error + 'a {
    move |e| in_the_way(e.kind(), path, format!("{what} ({e})"))
}

/// The reason determines whether moving an unexpected file or fixing access can help.
fn blocked_notice(error: &io::Error, writable: bool) -> String {
    let output = if writable {
        "It keeps adding to cv1touchd.log, up to 16 MiB."
    } else {
        "Daemon output is dropped and counted while logging is unavailable."
    };
    let remedy = if matches!(
        error.kind(),
        io::ErrorKind::AlreadyExists | io::ErrorKind::InvalidData
    ) {
        "Move that file or folder out of the way."
    } else {
        "Close any program that has that file open and check that cv1touch can write to the logs folder."
    };
    format!("cv1touch can't start a new log file because {error}. Tracking isn't affected. {output} {remedy} It tries again on its own.")
}

fn open_current(path: &Path) -> io::Result<(File, u64)> {
    log_exists(path)?;
    let file = fs::append_private(path).map_err(failed_on(path, "could not be opened"))?;
    let written = file
        .metadata()
        .map_err(failed_on(path, "could not have its size read"))?
        .len();
    Ok((file, written))
}

fn log_exists(path: &Path) -> io::Result<bool> {
    match std::fs::symlink_metadata(path) {
        Ok(metadata) if metadata.is_file() => Ok(true),
        Ok(metadata) => {
            let why = if metadata.is_dir() {
                "is a folder where a log file needs to go"
            } else if metadata.file_type().is_symlink() {
                "is a link where a log file needs to go"
            } else {
                "cannot be used as a log file"
            };
            Err(in_the_way(io::ErrorKind::InvalidData, path, why))
        }
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(false),
        Err(e) => Err(failed_on(path, "could not be looked at")(e)),
    }
}

impl Rotation {
    fn pending(&self) -> bool {
        self.pending
    }

    /// None: no transaction; Some(false): restored; Some(true): rotation finished.
    fn recover_with(
        &mut self,
        dir: &Path,
        step: &mut impl FnMut(RotationStep<'_>) -> io::Result<()>,
    ) -> io::Result<Option<bool>> {
        let staging = rotation_dir(dir);
        match std::fs::symlink_metadata(&staging) {
            Ok(metadata) if metadata.is_dir() => {}
            Ok(_) => {
                return Err(in_the_way(
                    io::ErrorKind::InvalidData,
                    &staging,
                    "should be a folder holding saved log files",
                ))
            }
            Err(e) if e.kind() == io::ErrorKind::NotFound => {
                self.pending = false;
                return Ok(None);
            }
            Err(e) => return Err(failed_on(&staging, "could not be looked at")(e)),
        }
        let ready = staging.join("ready");
        let mut committed = false;
        let mut present = [false; EARLIER_LOGS + 1];
        // Validate everything before moving anything. The transaction contains at most five
        // original logs and an empty commit marker, never paths supplied by a journal file.
        let unknown = |path: &Path| {
            in_the_way(
                io::ErrorKind::InvalidData,
                path,
                "is not one of cv1touch's own log files",
            )
        };
        for entry in
            std::fs::read_dir(&staging).map_err(failed_on(&staging, "could not be read"))?
        {
            let entry = entry.map_err(failed_on(&staging, "could not be read"))?;
            let path = entry.path();
            let metadata = std::fs::symlink_metadata(&path)
                .map_err(failed_on(&path, "could not be looked at"))?;
            if !metadata.is_file() {
                return Err(unknown(&path));
            }
            if path == ready && metadata.len() == 0 {
                committed = true;
            } else if let Some(age) =
                (0..=EARLIER_LOGS).find(|age| path == log_path(&staging, *age))
            {
                present[age] = true;
            } else {
                return Err(unknown(&path));
            }
        }
        for (age, exists) in present.iter().enumerate() {
            if *exists && !(committed && age == EARLIER_LOGS) {
                let destination = log_path(dir, age + usize::from(committed));
                if log_exists(&destination)? {
                    return Err(in_the_way(
                        io::ErrorKind::AlreadyExists,
                        &destination,
                        "is already there, where a saved log needs to go",
                    ));
                }
            }
        }
        // From here on files move, and an open log may no longer be the current one.
        self.pending = true;
        for (age, exists) in present.iter().enumerate() {
            if *exists && !(committed && age == EARLIER_LOGS) {
                let from = log_path(&staging, age);
                step(RotationStep::Rename(
                    &from,
                    &log_path(dir, age + usize::from(committed)),
                ))
                .map_err(failed_on(&from, "could not be moved"))?;
            }
        }
        // The oldest log is retired only after all retained logs have reached their places.
        if committed {
            if present[EARLIER_LOGS] {
                let oldest = log_path(&staging, EARLIER_LOGS);
                step(RotationStep::Remove(&oldest))
                    .map_err(failed_on(&oldest, "could not be removed"))?;
            }
            step(RotationStep::Remove(&ready))
                .map_err(failed_on(&ready, "could not be removed"))?;
        }
        step(RotationStep::RemoveDir(&staging))
            .map_err(failed_on(&staging, "could not be removed"))?;
        self.pending = false;
        Ok(Some(committed))
    }

    fn set_aside(&mut self, dir: &Path) -> io::Result<()> {
        self.set_aside_with(dir, |step| step.run())
    }

    fn set_aside_with(
        &mut self,
        dir: &Path,
        mut step: impl FnMut(RotationStep<'_>) -> io::Result<()>,
    ) -> io::Result<()> {
        #[cfg(test)]
        {
            self.attempts += 1;
        }
        self.recover_with(dir, &mut step)?;
        if !log_exists(&log_path(dir, 0))? {
            return Ok(());
        }
        let mut present = [false; EARLIER_LOGS + 1];
        for (age, exists) in present.iter_mut().enumerate() {
            *exists = log_exists(&log_path(dir, age))?;
        }
        let staging = rotation_dir(dir);
        step(RotationStep::CreateDir(&staging))
            .map_err(failed_on(&staging, "could not be created"))?;
        self.pending = true;
        let result = (|| {
            for (age, exists) in present.iter().enumerate() {
                if *exists {
                    let from = log_path(dir, age);
                    step(RotationStep::Rename(&from, &log_path(&staging, age)))
                        .map_err(failed_on(&from, "could not be moved"))?;
                }
            }
            // Creating this marker commits the fully staged set. A restart before it restores
            // original ages; a restart after it finishes publishing the new archive ages.
            let ready = staging.join("ready");
            step(RotationStep::Ready(&ready)).map_err(failed_on(&ready, "could not be created"))
        })();
        if let Err(error) = result {
            self.recover_with(dir, &mut step)?;
            return Err(error);
        }
        self.recover_with(dir, &mut step)?;
        Ok(())
    }
}

#[cfg(test)]
#[derive(Default)]
struct LogHooks {
    before_write: Option<Box<dyn FnOnce() + Send>>,
    before_drop: Option<Box<dyn FnOnce() + Send>>,
}

#[cfg(test)]
impl Drop for LogHooks {
    fn drop(&mut self) {
        if let Some(before_drop) = self.before_drop.take() {
            before_drop();
        }
    }
}

/// The daemon's log: one file per start of the supervisor, and a new one each time it fills up.
pub struct Log {
    // Test teardown blocks before the owned file and lock are released.
    #[cfg(test)]
    hooks: LogHooks,
    dir: PathBuf,
    file: Option<File>,
    /// Held before recovery and for this supervisor's lifetime, across all runtime directories.
    lock: Option<platform::ipc::Lock>,
    stderr_notice: Option<String>,
    warned_unavailable: bool,
    /// The current file size, including bytes from before this supervisor started.
    written: u64,
    rotate_after: Option<Instant>,
    recover_after: Option<Instant>,
    write_after: Option<Instant>,
    /// The old log was set aside, but opening or sizing its replacement has not succeeded.
    replacement_pending: bool,
    rotation: Rotation,
    /// Kept across child restarts so a temporarily unwritable log does not lose its count.
    dropped: Arc<AtomicU64>,
    notices_dropped: Arc<AtomicU64>,
}

impl Log {
    /// Begins this run's log, keeping four earlier ones. A logging failure never stops the
    /// daemon: a blocked rotation keeps appending up to the size limit; an unavailable log
    /// drops and counts output until it can be opened. The notice is this run's first
    /// supervisor line, on the supervisor's stderr if the log cannot be written. Another
    /// supervisor owning the folder disables logging here until it releases ownership.
    pub fn open(dir: &Path) -> io::Result<Log> {
        Self::open_with(dir, |step| step.run())
    }

    fn open_with(
        dir: &Path,
        step: impl FnMut(RotationStep<'_>) -> io::Result<()>,
    ) -> io::Result<Log> {
        Self::open_using(dir, step, open_current)
    }

    fn open_using(
        dir: &Path,
        step: impl FnMut(RotationStep<'_>) -> io::Result<()>,
        reopen: impl FnOnce(&Path) -> io::Result<(File, u64)>,
    ) -> io::Result<Log> {
        let mut log = Log {
            #[cfg(test)]
            hooks: LogHooks::default(),
            dir: dir.into(),
            file: None,
            lock: None,
            written: 0,
            rotate_after: None,
            recover_after: None,
            write_after: None,
            replacement_pending: false,
            rotation: Rotation::default(),
            dropped: Arc::new(AtomicU64::new(0)),
            notices_dropped: Arc::new(AtomicU64::new(0)),
            stderr_notice: None,
            warned_unavailable: false,
        };
        let _ = log.start_with(Instant::now(), step, reopen);
        Ok(log)
    }

    fn start_with(
        &mut self,
        now: Instant,
        mut step: impl FnMut(RotationStep<'_>) -> io::Result<()>,
        reopen: impl FnOnce(&Path) -> io::Result<(File, u64)>,
    ) -> io::Result<()> {
        if self.rotate_after.is_some_and(|at| now < at) {
            return Err(io::Error::other("the daemon log cannot be opened yet"));
        }
        self.rotate_after = Some(now + Duration::from_secs(1));
        self.file = None;
        let retrying_recovery = self.recover_after.is_some();
        let mut owned_elsewhere = false;
        let result: io::Result<()> = (|| {
            if self.lock.is_none() {
                fs::create_private_dir_all(&self.dir)
                    .map_err(failed_on(&self.dir, "could not be created"))?;
                let path = self.dir.join(LOG_LOCK);
                self.lock = Some(platform::ipc::lock_exclusive(&path).map_err(|error| {
                    owned_elsewhere = error.kind() == io::ErrorKind::WouldBlock;
                    failed_on(&path, "could not be locked")(error)
                })?);
            }
            let blocked = if self.replacement_pending {
                None
            } else {
                self.rotation.set_aside_with(&self.dir, &mut step).err()
            };
            // Nothing is open yet. A refused or partly failed recovery must not prevent
            // opening the current path, and no writes can go through a moved handle.
            self.rotation.pending = false;
            if blocked.is_none() {
                self.replacement_pending = true;
            }
            let (file, written) = reopen(&self.path())?;
            self.file = Some(file);
            self.written = written;
            self.replacement_pending = false;
            self.rotate_after = None;
            self.recover_after = blocked.as_ref().map(|_| now + Duration::from_secs(1));
            if let Some(error) = blocked {
                if !retrying_recovery {
                    note(self, &blocked_notice(&error, true));
                }
            }
            Ok(())
        })();
        if let Err(error) = &result {
            if !self.warned_unavailable {
                self.stderr_notice = Some(if owned_elsewhere {
                    format!(
                        "another cv1touch supervisor is using {}; this one is not writing logs \
                         there. Tracking continues; output is dropped and counted until that \
                         supervisor releases the folder.",
                        self.dir.display()
                    )
                } else {
                    blocked_notice(error, false)
                });
                self.warned_unavailable = true;
            }
        }
        result
    }

    /// The writer retries even when the daemon is quiet. Only that thread touches files or stderr.
    fn maintain(&mut self, now: Instant) {
        if self.write_after.is_some_and(|at| now < at) {
            return;
        }
        let write_retry = self.write_after.take().is_some();
        let resumed = if self.file.is_none() || self.recover_after.is_some_and(|at| now >= at) {
            self.start_with(now, |step| step.run(), open_current)
                .is_ok()
        } else if self.rotate_after.is_some() || self.replacement_pending || self.rotation.pending()
        {
            self.rotate_with(now, open_current).is_ok()
        } else {
            false
        };
        if resumed || write_retry {
            let dropped = Arc::clone(&self.dropped);
            report_dropped(self, &dropped);
        }
    }

    pub fn path(&self) -> PathBuf {
        log_path(&self.dir, 0)
    }

    /// Writes `text` as one line, stamped with `at`.
    pub fn line(&mut self, at: SystemTime, text: &str) -> io::Result<()> {
        self.line_at(at, text, Instant::now())
    }

    fn line_at(&mut self, at: SystemTime, text: &str, now: Instant) -> io::Result<()> {
        #[cfg(test)]
        if let Some(before_write) = self.hooks.before_write.take() {
            before_write();
        }
        if self.write_after.is_some_and(|at| now < at) {
            return Err(io::Error::other("the daemon log cannot be written yet"));
        }
        self.write_after = None;
        if self.file.is_none() {
            self.start_with(now, |step| step.run(), open_current)?;
        }
        let (text, truncated) = shorten(text.trim_end_matches(['\r', '\n']));
        let marker = if truncated { TRUNCATED } else { "" };
        let line = format!("{} {}{}\n", utc(at), text, marker);
        if self.rotation.pending()
            || self.rotate_after.is_some()
            || self.written + line.len() as u64 > LOG_MAX_BYTES
        {
            self.rotate_with(now, |path| {
                let file = fs::append_private(path)?;
                let written = file.metadata()?.len();
                Ok((file, written))
            })?;
            if self.written + line.len() as u64 > LOG_MAX_BYTES {
                return Err(io::Error::other("the daemon log is full"));
            }
        }
        let file = self
            .file
            .as_mut()
            .expect("the log was opened before writing");
        if let Err(e) = file.write_all(line.as_bytes()) {
            // Even a failed write may have appended part of a line.
            // An unknown size permits no more appends until rotation succeeds.
            self.written = file.metadata().map_or(LOG_MAX_BYTES, |m| m.len());
            self.write_after = Some(now + Duration::from_secs(1));
            return Err(e);
        }
        self.written += line.len() as u64;
        Ok(())
    }

    fn rotate_with(
        &mut self,
        now: Instant,
        reopen: impl FnOnce(&Path) -> io::Result<(File, u64)>,
    ) -> io::Result<()> {
        // Reopening permits appends, but a failed recovery still delays another rotation.
        if self.rotate_after.is_some_and(|at| now < at)
            || self.recover_after.is_some_and(|at| now < at)
        {
            return Err(io::Error::other(
                "the full daemon log cannot be rotated yet",
            ));
        }
        self.rotate_after = Some(now + Duration::from_secs(1));
        if !self.replacement_pending {
            self.rotation.set_aside(&self.dir)?;
            self.replacement_pending = true;
        }
        let (file, written) = reopen(&self.path())?;
        self.file = Some(file);
        self.written = written;
        self.replacement_pending = false;
        self.rotate_after = None;
        self.recover_after = None;
        Ok(())
    }
}

/// `at` in UTC, to the millisecond: 2026-10-14T17:05:03.123Z.
pub fn utc(at: SystemTime) -> String {
    crate::calendar::Utc::of(at).iso()
}

/// When the daemon is started again after it fails.
#[derive(Debug, Clone, Copy)]
pub struct Policy {
    /// The wait before starting it again.
    pub delay: Duration,
    /// A run that fails sooner than this after its start is a quick failure.
    pub quick: Duration,
    /// After this many quick failures in a row the supervisor gives up.
    pub quick_failures: usize,
}

pub const POLICY: Policy = Policy {
    delay: Duration::from_secs(2),
    quick: Duration::from_secs(30),
    quick_failures: 5,
};

/// The exit status the daemon gives for arguments it does not take: starting it again would not
/// help.
const USAGE_ERROR: i32 = 2;

/// The quick failures in a row.
pub struct Failures {
    policy: Policy,
    in_a_row: usize,
}

impl Failures {
    pub fn new(policy: Policy) -> Failures {
        Failures {
            policy,
            in_a_row: 0,
        }
    }

    /// Counts a failed run that lasted `ran`; whether the daemon may start again.
    pub fn failed(&mut self, ran: Duration) -> bool {
        self.in_a_row = if ran < self.policy.quick {
            self.in_a_row + 1
        } else {
            0
        };
        self.in_a_row < self.policy.quick_failures
    }
}

/// Drains a child's pipe even while the log is slow: a full queue loses a counted line.
fn forward<T: From<String> + Send + 'static>(
    mut from: impl Read + Send + 'static,
    to: Output<T>,
    dropped: Arc<AtomicU64>,
) -> std::thread::JoinHandle<()> {
    struct Reading(Arc<AtomicUsize>);
    impl Drop for Reading {
        fn drop(&mut self) {
            self.0.fetch_sub(1, Ordering::Release);
        }
    }
    to.active.fetch_add(1, Ordering::Relaxed);
    let reading = Reading(to.active.clone());
    std::thread::spawn(move || {
        let _reading = reading;
        let mut lines = LineBuffer::default();
        let mut buf = [0u8; READ_BYTES];
        let mut connected = true;
        loop {
            let n = match from.read(&mut buf) {
                Ok(n) => n,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(_) => break,
            };
            let mut send = |bytes: &[u8], truncated| {
                let decoded = String::from_utf8_lossy(bytes);
                let (text, too_long) = shorten(decoded.trim_end_matches('\r'));
                let mut text = text.to_string();
                if truncated || too_long {
                    while text.len() + TRUNCATED.len() > LINE_BYTES {
                        text.pop();
                    }
                    text.push_str(TRUNCATED);
                }
                match to.try_send(T::from(text)) {
                    Ok(()) => {}
                    Err(TrySendError::Full(_)) => {
                        dropped.fetch_add(1, Ordering::Relaxed);
                    }
                    Err(TrySendError::Disconnected(_)) => {
                        if to.stopped.load(Ordering::Acquire) || to.closed.load(Ordering::Acquire) {
                            connected = false;
                        } else {
                            // A writer that could not start or ended must not back up the pipe.
                            dropped.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            };
            if n == 0 {
                lines.finish(send);
                break;
            }
            lines.push(&buf[..n], &mut send);
            if !connected {
                break;
            }
        }
    })
}

#[cfg(test)]
fn output_channel() -> (Output, mpsc::Receiver<String>) {
    let (to, lines) = Lines::channel();
    (to, lines.from)
}

fn report_dropped(log: &mut Log, dropped: &AtomicU64) {
    let lost = dropped.swap(0, Ordering::Relaxed);
    if lost == 0 {
        return;
    }
    let lines = if lost == 1 { "line" } else { "lines" };
    let text = format!(
        "cv1touchd supervisor: dropped {lost} daemon output {lines} while logging fell behind"
    );
    if log.line(SystemTime::now(), &text).is_err() {
        dropped.fetch_add(lost, Ordering::Relaxed);
    }
}

fn output_line(log: &mut Log, dropped: &AtomicU64, line: &str) {
    report_dropped(log, dropped);
    if log.line(SystemTime::now(), line).is_err() {
        dropped.fetch_add(1, Ordering::Relaxed);
    }
}

fn note(log: &mut Log, text: &str) {
    let dropped = Arc::clone(&log.dropped);
    report_dropped(log, &dropped);
    if log
        .line(SystemTime::now(), &format!("cv1touchd supervisor: {text}"))
        .is_err()
    {
        log.notices_dropped.fetch_add(1, Ordering::Relaxed);
    }
}

/// How long the lines a daemon wrote just before it ended may take to arrive; after that its
/// output is left to whatever it started that still holds it.
const LAST_LINES: Duration = Duration::from_secs(1);

/// One reader's output. The cutoff flag never needs room in the output queue.
#[derive(Clone)]
struct Output<T = String> {
    to: mpsc::SyncSender<T>,
    stopped: Arc<AtomicBool>,
    closed: Arc<AtomicBool>,
    active: Arc<AtomicUsize>,
}

impl<T> Output<T> {
    fn try_send(&self, text: T) -> Result<(), TrySendError<T>> {
        if self.stopped.load(Ordering::Acquire) || self.closed.load(Ordering::Acquire) {
            Err(TrySendError::Disconnected(text))
        } else {
            self.to.try_send(text)
        }
    }
}

/// Output from both readers. Once the deadline passes, stop forwarding new lines and drain
/// those already queued. The consumer owns no sender, so EOF still disconnects it immediately.
struct Lines<T = String> {
    from: mpsc::Receiver<T>,
    stop_readers: Arc<AtomicBool>,
    cutoff: bool,
}

#[cfg(test)]
impl Lines<String> {
    fn channel() -> (Output, Lines) {
        Self::channel_for()
    }
}

impl<T> Lines<T> {
    fn channel_for() -> (Output<T>, Lines<T>) {
        let (to, from) = mpsc::sync_channel(LOG_QUEUE_LINES);
        let stopped = Arc::new(AtomicBool::new(false));
        let lines = Lines {
            from,
            stop_readers: stopped.clone(),
            cutoff: false,
        };
        (
            Output {
                to,
                closed: stopped.clone(),
                stopped,
                active: Arc::new(AtomicUsize::new(0)),
            },
            lines,
        )
    }

    fn next_line(
        &mut self,
        ended: Option<Instant>,
        clock: &dyn Fn() -> Instant,
    ) -> Result<T, RecvTimeoutError> {
        loop {
            let wait = ended.map_or(Duration::from_millis(100), |at| {
                LAST_LINES.saturating_sub(clock().saturating_duration_since(at))
            });
            if ended.is_some() && wait.is_zero() && !self.cutoff {
                self.stop_readers.store(true, Ordering::Release);
                self.cutoff = true;
            }
            match self.from.recv_timeout(wait) {
                Ok(line) => return Ok(line),
                // The deadline passed while waiting. Stop the readers before leaving the queue.
                Err(RecvTimeoutError::Timeout) if ended.is_some() && !self.cutoff => {}
                Err(e) => return Err(e),
            }
        }
    }
}

/// All output shares one bounded FIFO. Only its writer owns the Log and performs I/O.
#[derive(Clone)]
enum Record {
    Daemon(String),
    Notice(String),
    Terminal(String),
}

impl From<String> for Record {
    fn from(text: String) -> Self {
        Self::Daemon(text)
    }
}

/// Terminal failures have their own loss counter, separate from daemon output.
struct Terminal<W> {
    sink: W,
    dropped: Arc<AtomicU64>,
    retry_after: Option<Instant>,
}

impl<W: Write> Terminal<W> {
    fn line(&mut self, text: &str, now: Instant) {
        if self.retry_after.is_some_and(|at| now < at) {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
        if self.sink.write_all(format!("{text}\n").as_bytes()).is_err() {
            self.retry_after = Some(now + Duration::from_secs(1));
            self.dropped.fetch_add(1, Ordering::Relaxed);
        } else {
            self.retry_after = None;
        }
    }

    fn report(&mut self, now: Instant) {
        if self.retry_after.is_some_and(|at| now < at) {
            return;
        }
        let lost = self.dropped.swap(0, Ordering::Relaxed);
        if lost != 0 {
            let unit = if lost == 1 { "notice" } else { "notices" };
            let text = format!(
                "cv1touchd supervisor: dropped {lost} supervisor {unit} while logging was unavailable\n"
            );
            if self.sink.write_all(text.as_bytes()).is_err() {
                self.retry_after = Some(now + Duration::from_secs(1));
                self.dropped.fetch_add(lost, Ordering::Relaxed);
            } else {
                self.retry_after = None;
            }
        }
    }
}

type OpenLog = Box<dyn FnOnce(&Path) -> Log + Send>;
type WriterTask = Box<dyn FnOnce() + Send>;

fn spawn_writer(task: WriterTask) -> io::Result<std::thread::JoinHandle<()>> {
    std::thread::Builder::new()
        .name("cv1t-log-files".into())
        .spawn(task)
}

struct Finished(mpsc::SyncSender<()>, Arc<AtomicBool>);

impl Drop for Finished {
    fn drop(&mut self) {
        self.1.store(true, Ordering::Release);
        let _ = self.0.try_send(());
    }
}

struct LostOnPanic<'a>(&'a AtomicU64);

impl Drop for LostOnPanic<'_> {
    fn drop(&mut self) {
        if std::thread::panicking() {
            self.0.fetch_add(1, Ordering::Relaxed);
        }
    }
}

pub struct Writer {
    output: Output<Record>,
    path: PathBuf,
    dropped: Arc<AtomicU64>,
    notices_dropped: Arc<AtomicU64>,
    ending: Arc<Mutex<Option<Instant>>>,
    finished: mpsc::Receiver<()>,
    done: mpsc::SyncSender<()>,
    // Only the writer locks this. Retaining the receiver preserves the FIFO across failure.
    lines: Arc<Mutex<Lines<Record>>>,
    running: bool,
    completed: Arc<AtomicBool>,
    open: Option<OpenLog>,
    retry_after: Option<Instant>,
    failure_notice: Option<String>,
    failure_reported: bool,
}

impl Writer {
    /// Opening and ownership checks happen on the writer too, so a stalled filesystem cannot
    /// keep the supervisor from starting its child. A failed writer is replaced after one second.
    pub fn open(dir: &Path) -> Self {
        Self::with_open(dir, |dir| {
            Log::open(dir).expect("log errors enter discard and retry")
        })
    }

    fn with_open(dir: &Path, open: impl FnOnce(&Path) -> Log + Send + 'static) -> Self {
        Self::with_spawn(dir, open, spawn_writer)
    }

    fn with_spawn(
        dir: &Path,
        open: impl FnOnce(&Path) -> Log + Send + 'static,
        spawn: impl FnOnce(WriterTask) -> io::Result<std::thread::JoinHandle<()>>,
    ) -> Self {
        let (output, lines) = Lines::<Record>::channel_for();
        let (done, finished) = mpsc::sync_channel(1);
        let mut writer = Self {
            output,
            path: log_path(dir, 0),
            dropped: Arc::new(AtomicU64::new(0)),
            notices_dropped: Arc::new(AtomicU64::new(0)),
            ending: Arc::new(Mutex::new(None)),
            finished,
            done,
            lines: Arc::new(Mutex::new(lines)),
            running: false,
            completed: Arc::new(AtomicBool::new(false)),
            open: Some(Box::new(open)),
            retry_after: None,
            failure_notice: None,
            failure_reported: false,
        };
        writer.start_with(Instant::now(), spawn);
        writer
    }

    fn failed(&mut self, now: Instant, reason: &str) {
        self.retry_after = Some(now + Duration::from_secs(1));
        if !self.failure_reported && self.failure_notice.is_none() {
            self.failure_notice = Some(format!(
                "cv1touchd supervisor: the log writer {reason}. Tracking continues; logging \
                 retries once a second, and output that cannot be kept is dropped and counted."
            ));
        }
    }

    /// No I/O or worker lock here. A live but blocked writer is never replaced.
    fn poll(&mut self, now: Instant) {
        if self.running && self.completed.load(Ordering::Acquire) {
            self.running = false;
            if self
                .ending
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .is_none()
            {
                self.failed(now, "stopped unexpectedly");
            }
        }
        if !self.running
            && self
                .ending
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .is_none()
        {
            self.start_with(now, spawn_writer);
        }
    }

    fn start_with(
        &mut self,
        now: Instant,
        spawn: impl FnOnce(WriterTask) -> io::Result<std::thread::JoinHandle<()>>,
    ) {
        if self.running || self.retry_after.is_some_and(|at| now < at) {
            return;
        }
        let dir = self.path.parent().expect("log folder").to_owned();
        let lines = self.lines.clone();
        let dropped = self.dropped.clone();
        let notices_dropped = self.notices_dropped.clone();
        let ending = self.ending.clone();
        let done = self.done.clone();
        self.completed.store(false, Ordering::Release);
        let completed = self.completed.clone();
        let failure_notice = self.failure_notice.clone();
        let open = self.open.take().unwrap_or_else(|| {
            Box::new(|dir| Log::open(dir).expect("log errors enter discard and retry"))
        });
        let task = Box::new(move || {
            // Declared first, so completion is published after files and locks are dropped,
            // on normal return or unwind. Supervision never joins this thread.
            let _finished = Finished(done, completed);
            let mut lines = lines.lock().unwrap_or_else(|e| e.into_inner());
            let mut log: Option<Log> = None;
            let mut open = Some(open);
            let mut terminal = Terminal {
                sink: io::stderr(),
                dropped: notices_dropped.clone(),
                retry_after: None,
            };
            if let Some(text) = failure_notice {
                terminal.line(&text, Instant::now());
            }
            loop {
                let deadline = *ending.lock().unwrap_or_else(|e| e.into_inner());
                if deadline.is_some_and(|at| Instant::now() >= at) {
                    break;
                }
                if let Some(log) = &mut log {
                    log.maintain(Instant::now());
                    if let Some(text) = log.stderr_notice.take() {
                        terminal.line(&format!("cv1touchd supervisor: {text}"), Instant::now());
                    }
                }
                // Closing admission drains the FIFO without waiting for an inherited pipe.
                let cutoff = deadline.map(|_| Instant::now() - LAST_LINES);
                match lines.next_line(cutoff, &Instant::now) {
                    Ok(record) => {
                        let _lost = LostOnPanic(if matches!(&record, Record::Daemon(_)) {
                            &dropped
                        } else {
                            &notices_dropped
                        });
                        if let Record::Terminal(text) = record {
                            terminal.line(&format!("cv1touchd: {text}"), Instant::now());
                        } else {
                            let log = log.get_or_insert_with(|| {
                                let mut log = open.take().expect("one log per writer")(&dir);
                                log.dropped = dropped.clone();
                                notices_dropped.fetch_add(
                                    log.notices_dropped.load(Ordering::Relaxed),
                                    Ordering::Relaxed,
                                );
                                log.notices_dropped = notices_dropped.clone();
                                log
                            });
                            log.maintain(Instant::now());
                            if let Some(text) = log.stderr_notice.take() {
                                terminal
                                    .line(&format!("cv1touchd supervisor: {text}"), Instant::now());
                            }
                            match record {
                                Record::Daemon(text) => output_line(log, &dropped, &text),
                                Record::Notice(text) => note(log, &text),
                                Record::Terminal(_) => unreachable!("handled before opening a log"),
                            }
                        }
                    }
                    Err(RecvTimeoutError::Disconnected) => break,
                    Err(RecvTimeoutError::Timeout) if deadline.is_some() => break,
                    Err(RecvTimeoutError::Timeout) => {}
                }
                terminal.report(Instant::now());
            }
            // Release files and ownership on this thread, including when it unwinds.
            drop(log);
        });
        match spawn(task) {
            Ok(_thread) => {
                self.running = true;
                self.retry_after = None;
                if self.failure_notice.take().is_some() {
                    self.failure_reported = true;
                }
            }
            Err(error) => self.failed(now, &format!("could not start ({error})")),
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Terminal notices use the same writer and queue as file output, including before a child.
    pub fn terminal(&self, text: &str) {
        self.enqueue(text, true);
    }

    fn note(&self, text: &str) {
        self.enqueue(text, false);
    }

    fn enqueue(&self, text: &str, terminal: bool) {
        let (text, truncated) = shorten(text);
        let text = if truncated {
            format!("{text}{TRUNCATED}")
        } else {
            text.to_owned()
        };
        let record = if terminal {
            Record::Terminal(text)
        } else {
            Record::Notice(text)
        };
        if self.output.try_send(record).is_err() {
            self.notices_dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn readers(&self) -> Output<Record> {
        Output {
            to: self.output.to.clone(),
            closed: self.output.closed.clone(),
            stopped: Arc::new(AtomicBool::new(false)),
            active: Arc::new(AtomicUsize::new(0)),
        }
    }

    /// No join: a blocked open, write or terminal notice can outlive this bounded drain.
    fn finish(&mut self, until: Instant) {
        let mut ending = self.ending.lock().unwrap_or_else(|e| e.into_inner());
        if ending.is_some() {
            return;
        }
        *ending = Some(until);
        self.output.closed.store(true, Ordering::Release);
        drop(ending);
        while Instant::now() < until {
            self.poll(Instant::now());
            if !self.running {
                break;
            }
            let _ = self.finished.recv_timeout(
                until
                    .saturating_duration_since(Instant::now())
                    .min(Duration::from_millis(100)),
            );
        }
    }
}

impl Drop for Writer {
    fn drop(&mut self) {
        self.finish(Instant::now() + LAST_LINES);
    }
}

/// How the supervisor learns that it is to stop, and how it asks the daemon to.
pub struct Stopping<'a> {
    /// A request to stop, if one came: what it was (polled often, so quick).
    pub requested: Box<dyn FnMut() -> Option<String> + 'a>,
    /// Asks the running daemon to stop, returning by the supplied deadline.
    pub ask: Box<dyn FnMut(Instant) -> Result<(), String> + 'a>,
    /// How long to try delivering. A success before that deadline starts the same grace for
    /// the daemon to stop. (After its console window is closed, Windows ends the supervisor
    /// 5 s later whatever this says.)
    pub within: Duration,
}

/// How often a daemon not yet taking requests (it is just starting) is asked again.
const ASK_AGAIN: Duration = Duration::from_millis(250);

#[derive(Debug, PartialEq, Eq)]
enum StopAction {
    Wait,
    Deliver(Instant),
    End,
}

/// All decisions use the poll's time, so delivery and scheduling delays cannot restart a
/// failed request's deadline. Only a successful delivery earns the daemon a fresh grace.
struct Stop {
    within: Duration,
    until: Option<Instant>,
    last_ask: Option<Instant>,
    accepted: bool,
}

impl Stop {
    fn new(within: Duration) -> Stop {
        Stop {
            within,
            until: None,
            last_ask: None,
            accepted: false,
        }
    }

    fn request(&mut self, now: Instant) {
        self.until = Some(now + self.within);
    }

    fn action(&self, now: Instant) -> StopAction {
        let Some(until) = self.until else {
            return StopAction::Wait;
        };
        if now >= until {
            StopAction::End
        } else if !self.accepted
            && self
                .last_ask
                .is_none_or(|at| now.saturating_duration_since(at) >= ASK_AGAIN)
        {
            StopAction::Deliver(until)
        } else {
            StopAction::Wait
        }
    }

    fn delivered(&mut self, now: Instant, accepted: bool) {
        self.last_ask = Some(now);
        if accepted && self.until.is_some_and(|until| now < until) {
            self.accepted = true;
            self.until = Some(now + self.within);
        }
    }
}

/// A daemon's end: its exit status, and whether the supervisor had asked it to stop.
struct Ended {
    status: ExitStatus,
    asked: bool,
    at: Instant,
}

/// Runs one daemon to its end, its output to `log`.
fn run_once(
    command: &mut Command,
    log: &mut Writer,
    stopping: &mut Stopping,
    clock: &dyn Fn() -> Instant,
) -> io::Result<Ended> {
    if let Err(e) = platform::process::end_with_this_process(command) {
        log.note(&format!("the daemon could outlive the supervisor ({e})"));
    }
    platform::process::without_window(command);
    let mut child = command
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;
    log.note(&format!("started the daemon (process {})", child.id()));
    // its standard output and error are read apart, so their lines are logged as they arrive
    let tx = log.readers();
    let dropped = Arc::clone(&log.dropped);
    if let Some(out) = child.stdout.take() {
        forward(out, tx.clone(), Arc::clone(&dropped));
    }
    if let Some(err) = child.stderr.take() {
        forward(err, tx.clone(), Arc::clone(&dropped));
    }

    let mut ended: Option<(ExitStatus, Instant, Instant)> = None;
    let mut stop = Stop::new(stopping.within);
    let mut ending = false;
    loop {
        log.poll(Instant::now());
        if let Some((status, at, real_at)) = ended {
            if tx.active.load(Ordering::Acquire) == 0
                || clock().saturating_duration_since(at) >= LAST_LINES
            {
                tx.stopped.store(true, Ordering::Release);
                return Ok(Ended {
                    status,
                    asked: stop.until.is_some(),
                    at: real_at,
                });
            }
            std::thread::sleep(Duration::from_millis(10));
            continue;
        }
        if stop.until.is_none() {
            if let Some(why) = (stopping.requested)() {
                stop.request(clock());
                log.note(&format!("{why}: asking the daemon to stop"));
            }
        }
        if !ending {
            let mut first_error = None;
            if let StopAction::Deliver(until) = stop.action(clock()) {
                let first = stop.last_ask.is_none();
                let result = (stopping.ask)(until);
                stop.delivered(clock(), result.is_ok());
                if let Err(e) = result {
                    if first {
                        first_error = Some(e);
                    }
                }
            }
            // Check again after delivery, before any diagnostic I/O. A delivery that runs out
            // of time must not postpone the kill or start another attempt.
            if stop.action(clock()) == StopAction::End {
                let _ = child.kill();
                ending = true;
                log.note(&format!(
                    "the daemon did not stop within {} s; ending it",
                    stopping.within.as_secs_f64()
                ));
            }
            if let Some(e) = first_error {
                if ending {
                    log.note(&format!("could not ask it to stop ({e})"));
                } else {
                    log.note(&format!("cannot ask it yet ({e}); asking again"));
                }
            }
        }
        ended = child
            .try_wait()?
            .map(|status| (status, clock(), Instant::now()));
        if ended.is_none() {
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

/// Runs the daemons `command` makes, one at a time, until one stops cleanly (exit status 0) or
/// the supervisor is asked to stop, each line the daemon writes to its standard output or error
/// going to `log`; each ends with the thread that calls this, so call it from the main thread.
/// After a failure the next starts once `policy.delay` has passed, unless the daemon refused its
/// arguments (exit status 2) or the policy allows no more quick failures. What the supervisor has
/// to say goes to the log as "cv1touchd supervisor: ...". The exit code for the supervisor: 0
/// once the daemon stopped, 1 when it gave up.
pub fn run(
    command: impl FnMut() -> Command,
    log: &mut Writer,
    policy: Policy,
    stopping: &mut Stopping,
) -> i32 {
    run_with_clock(command, log, policy, stopping, Instant::now)
}

/// The daemon's run time and deadlines use `clock`; restart delays use elapsed real time.
fn run_with_clock(
    command: impl FnMut() -> Command,
    log: &mut Writer,
    policy: Policy,
    stopping: &mut Stopping,
    clock: impl Fn() -> Instant,
) -> i32 {
    let (code, ended) = run_children(command, log, policy, stopping, clock);
    log.finish(ended.unwrap_or_else(Instant::now) + LAST_LINES);
    code
}

fn run_children(
    mut command: impl FnMut() -> Command,
    log: &mut Writer,
    policy: Policy,
    stopping: &mut Stopping,
    clock: impl Fn() -> Instant,
) -> (i32, Option<Instant>) {
    let mut failures = Failures::new(policy);
    let delay = policy.delay.as_secs_f64();
    loop {
        let started = clock();
        let ended = match run_once(&mut command(), log, stopping, &clock) {
            Ok(Ended { status, asked, at }) if asked || status.success() => {
                log.note(&format!("the daemon stopped ({status})"));
                return (0, Some(at));
            }
            Ok(Ended { status, at, .. }) if status.code() == Some(USAGE_ERROR) => {
                log.note(&format!(
                    "the daemon refused its arguments ({status}); not starting it again"
                ));
                return (1, Some(at));
            }
            Ok(Ended { status, at, .. }) => {
                log.note(&format!("the daemon ended ({status})"));
                Some(at)
            }
            Err(e) => {
                log.note(&format!("cannot start the daemon ({e})"));
                None
            }
        };
        if !failures.failed(clock().saturating_duration_since(started)) {
            log.note(&format!(
                "the daemon failed {} times in a row within {} s of starting; not starting \
                     it again",
                policy.quick_failures,
                policy.quick.as_secs_f64()
            ));
            return (1, ended);
        }
        log.note(&format!("starting it again in {delay} s"));
        let until = Instant::now() + policy.delay;
        while let Some(left) = until.checked_duration_since(Instant::now()) {
            log.poll(Instant::now());
            if let Some(why) = (stopping.requested)() {
                log.note(&format!("{why}: not starting the daemon again"));
                return (0, None);
            }
            std::thread::sleep(left.min(Duration::from_millis(100)));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(unix)]
    #[test]
    fn child_output_drains_even_if_the_started_log_line_stalls() {
        use std::os::fd::OwnedFd;
        use std::os::unix::net::UnixStream;

        let d = dir("startup-stall");
        std::fs::create_dir_all(&d).unwrap();
        let (mut sink, blocked) = UnixStream::pair().unwrap();
        sink.set_nonblocking(true).unwrap();
        loop {
            match sink.write(&[b'x'; 4096]) {
                Ok(_) => {}
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => panic!("could not fill the log sink: {e}"),
            }
        }
        sink.set_nonblocking(false).unwrap();
        let fd: OwnedFd = sink.into();
        let mut log = Writer::with_open(&d, move |dir| {
            let mut log = Log::open(dir).unwrap();
            log.file = Some(File::from(fd));
            log
        });
        let done = d.join("output-drained");
        let child_done = done.clone();
        let supervisor = std::thread::spawn(move || {
            run(
                || {
                    let mut command = stand_in(&child_done);
                    command.env("CV1TOUCH_TEST_FLOOD_OUTPUT", "1");
                    command
                },
                &mut log,
                quick(10, 5),
                &mut never(),
            )
        });
        let until = Instant::now() + Duration::from_secs(5);
        while !done.exists() && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(10));
        }
        let drained = done.exists();
        // Always unblock the log before asserting, so the old reader order can fail cleanly.
        drop(blocked);
        let code = supervisor.join().unwrap();
        std::fs::remove_dir_all(d).unwrap();
        assert_eq!(code, 0);
        assert!(
            drained,
            "the child's output waited for the supervisor's started log line"
        );
    }

    #[test]
    fn lost_output_is_reported_even_after_the_child_that_wrote_it_ends() {
        let d = dir("lost-after-exit");
        std::fs::create_dir_all(log_path(&d, 0)).unwrap();
        let mut log = Writer::open(&d);
        let lost = log.dropped.clone();
        let mut runs = 0;
        let code = run(
            || {
                runs += 1;
                if runs == 1 {
                    shell(if cfg!(windows) {
                        "echo first & exit /b 1"
                    } else {
                        "echo first; exit 1"
                    })
                } else {
                    let until = Instant::now() + Duration::from_secs(5);
                    while lost.load(Ordering::Relaxed) == 0 && Instant::now() < until {
                        std::thread::sleep(Duration::from_millis(10));
                    }
                    assert_eq!(lost.load(Ordering::Relaxed), 1);
                    std::fs::remove_dir(log_path(&d, 0)).unwrap();
                    // A quiet child leaves the writer to retry and report the previous child's loss.
                    shell(if cfg!(windows) {
                        "ping -n 3 127.0.0.1 >nul"
                    } else {
                        "sleep 2"
                    })
                }
            },
            &mut log,
            quick(10, 5),
            &mut never(),
        );
        assert_eq!(code, 0);
        assert_eq!(runs, 2);
        assert_eq!(log.dropped.load(Ordering::Relaxed), 0);
        assert!(texts(log.path())
            .iter()
            .any(|l| l.contains("dropped 1 daemon output line")));
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_slow_log_loses_counted_lines_without_stalling_the_pipe_reader() {
        let (tx, rx) = output_channel();
        let dropped = Arc::new(AtomicU64::new(0));
        let worker = forward(
            std::io::Cursor::new(b"line\n".repeat(1032)),
            tx,
            Arc::clone(&dropped),
        );
        let until = Instant::now() + Duration::from_secs(5);
        while !worker.is_finished() && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(10));
        }
        let finished = worker.is_finished();
        let lines: Vec<_> = rx.try_iter().collect();
        drop(rx); // also releases a reader incorrectly blocked on a full queue
        worker.join().unwrap();
        assert!(
            finished,
            "the daemon's pipe must drain while the log is stalled"
        );
        assert_eq!(lines.len(), 1024);
        assert_eq!(dropped.load(Ordering::Relaxed), 8);
        let d = dir("lost-lines");
        let mut log = Log::open(&d).unwrap();
        output_line(&mut log, &dropped, "caught up");
        assert_eq!(
            texts(&log.path()),
            [
                "cv1touchd supervisor: dropped 8 daemon output lines while logging fell behind",
                "caught up",
            ]
        );
        assert_eq!(dropped.load(Ordering::Relaxed), 0);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn the_pipe_is_read_in_bounded_chunks() {
        struct Metered(
            std::io::Cursor<Vec<u8>>,
            Arc<std::sync::atomic::AtomicUsize>,
        );
        impl Read for Metered {
            fn read(&mut self, bytes: &mut [u8]) -> io::Result<usize> {
                self.1.fetch_max(bytes.len(), Ordering::Relaxed);
                self.0.read(bytes)
            }
        }
        let largest = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let (tx, rx) = output_channel();
        forward(
            Metered(
                std::io::Cursor::new(b"line\n".to_vec()),
                Arc::clone(&largest),
            ),
            tx,
            Arc::new(AtomicU64::new(0)),
        )
        .join()
        .unwrap();
        assert_eq!(rx.recv().unwrap(), "line");
        assert!(largest.load(Ordering::Relaxed) <= 64 * 1024);
    }

    #[test]
    fn the_log_truncates_text_at_a_utf8_boundary() {
        let d = dir("utf8");
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), &("x".to_string() + &"é".repeat(8192)))
            .unwrap();
        let text = texts(&log.path()).pop().unwrap();
        assert!(text.len() <= 8192 + TRUNCATED.len());
        assert!(text.ends_with(TRUNCATED));
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn failed_log_writes_are_counted_until_the_log_recovers() {
        let d = dir("lost-writes");
        let mut log = Log::open(&d).unwrap();
        let blocker = log_path(&d, 1);
        std::fs::create_dir_all(blocker.join("occupied")).unwrap();
        log.written = LOG_MAX_BYTES;
        let dropped = AtomicU64::new(8);
        output_line(&mut log, &dropped, "also lost");
        assert_eq!(dropped.load(Ordering::Relaxed), 9);
        std::fs::remove_dir_all(blocker).unwrap();
        log.rotate_after = None;
        output_line(&mut log, &dropped, "caught up");
        assert_eq!(
            texts(&log.path()),
            [
                "cv1touchd supervisor: dropped 9 daemon output lines while logging fell behind",
                "caught up",
            ]
        );
        assert_eq!(dropped.load(Ordering::Relaxed), 0);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn output_lines_are_truncated_before_they_are_queued() {
        let (tx, rx) = output_channel();
        let input = [vec![b'x'; 32 * 1024], b"\nnext\nlast".to_vec()].concat();
        forward(std::io::Cursor::new(input), tx, Arc::new(AtomicU64::new(0)));
        let first = rx.recv_timeout(Duration::from_secs(5)).unwrap();
        assert!(first.len() < 9 * 1024, "overlong line was kept in full");
        assert!(first.ends_with(" [line truncated]"));
        assert_eq!(rx.recv().unwrap().trim_end(), "next");
        assert_eq!(rx.recv().unwrap(), "last");
    }

    #[test]
    fn lossy_decoding_does_not_expand_a_queued_line_past_its_limit() {
        let (tx, rx) = output_channel();
        let input = [vec![0xff; 8192], vec![b'\n']].concat();
        forward(std::io::Cursor::new(input), tx, Arc::new(AtomicU64::new(0)))
            .join()
            .unwrap();
        let line = rx.recv().unwrap();
        assert!(line.len() <= 8192 + TRUNCATED.len());
        assert!(line.ends_with(TRUNCATED));
        let d = dir("lossy-marker");
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), &line).unwrap();
        assert_eq!(texts(&log.path()), [line]);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_full_log_that_cannot_rotate_does_not_grow() {
        let d = dir("bounded");
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "kept").unwrap();
        std::fs::create_dir_all(log_path(&d, 1).join("occupied")).unwrap();
        let before = read(&log.path());
        log.written = LOG_MAX_BYTES;
        for _ in 0..3 {
            assert!(log.line(SystemTime::now(), "cannot fit").is_err());
        }
        assert_eq!(read(&log.path()), before);
        assert_eq!(log.written, LOG_MAX_BYTES);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn an_append_crossing_the_limit_is_refused_before_writing() {
        let d = dir("crossing");
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "kept").unwrap();
        std::fs::create_dir_all(log_path(&d, 1).join("occupied")).unwrap();
        let before = read(&log.path());
        log.written = LOG_MAX_BYTES - 1;
        assert!(log.line(SystemTime::now(), "crosses the limit").is_err());
        assert_eq!(read(&log.path()), before);
        assert_eq!(log.written, LOG_MAX_BYTES - 1);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_locked_archive_survives_repeated_rotation_attempts() {
        let d = dir("locked-archive");
        std::fs::create_dir_all(&d).unwrap();
        for age in 0..=EARLIER_LOGS {
            std::fs::write(log_path(&d, age), format!("log {age}\n")).unwrap();
        }
        let locked = log_path(&d, 1);
        let mut rotation = Rotation::default();
        for _ in 0..=EARLIER_LOGS + 2 {
            let result = rotation.set_aside_with(&d, |step| match &step {
                RotationStep::Rename(from, to) if *from == locked || *to == locked => Err(
                    io::Error::new(io::ErrorKind::PermissionDenied, "injected archive lock"),
                ),
                _ => step.run(),
            });
            assert!(result.is_err());
            for age in 0..=EARLIER_LOGS {
                assert_eq!(
                    std::fs::read_to_string(log_path(&d, age)).ok(),
                    Some(format!("log {age}\n"))
                );
            }
            assert!(!rotation.pending());
            assert_eq!(
                std::fs::read_dir(&d)
                    .unwrap()
                    .filter(|e| e.as_ref().unwrap().file_name() != LOG_LOCK)
                    .count(),
                EARLIER_LOGS + 1
            );
        }
        rotation.set_aside(&d).unwrap();
        assert!(!log_path(&d, 0).exists());
        for age in 1..=EARLIER_LOGS {
            assert_eq!(read(&log_path(&d, age)), format!("log {}\n", age - 1));
        }
        assert_eq!(
            std::fs::read_dir(&d)
                .unwrap()
                .filter(|e| e.as_ref().unwrap().file_name() != LOG_LOCK)
                .count(),
            EARLIER_LOGS
        );
        std::fs::remove_dir_all(d).unwrap();
    }

    /// No release wrote these names, and the current rotation never touches them.
    #[test]
    fn leftover_moving_files_are_left_alone_and_logging_goes_on() {
        for name in [
            format!("{LOG_NAME}.log.moving"),
            format!("{LOG_NAME}.{EARLIER_LOGS}.log.moving"),
        ] {
            let d = dir(&name);
            seed_archives(&d, &[0, 1, 2, 3, 4]);
            std::fs::write(d.join(&name), "interrupted log\n").unwrap();
            let mut log = Log::open(&d).unwrap();
            log.line(SystemTime::now(), "this run").unwrap();
            assert_eq!(texts(&log.path()), ["this run"]);
            for age in 1..=EARLIER_LOGS {
                assert_eq!(read(&log_path(&d, age)), format!("x log {}\n", age - 1));
            }
            assert_eq!(read(&d.join(&name)), "interrupted log\n");
            drop(log);
            std::fs::remove_dir_all(d).unwrap();
        }
    }

    #[test]
    fn a_restart_recovers_a_rotation_without_its_in_memory_undo_list() {
        let d = dir("rotation-restart");
        std::fs::create_dir_all(&d).unwrap();
        for age in 0..=EARLIER_LOGS {
            std::fs::write(log_path(&d, age), format!("log {age}\n")).unwrap();
        }
        let interrupted = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            Rotation::default().set_aside_with(&d, |step| {
                let moved = matches!(step, RotationStep::Rename(..));
                step.run()?;
                assert!(!moved, "injected supervisor crash after moving a log");
                Ok(())
            })
        }));
        assert!(interrupted.is_err());
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "after restart").unwrap();
        assert!(
            (0..=EARLIER_LOGS).any(|age| read(&log_path(&d, age)).contains("log 0\n")),
            "the current log was stranded outside the archive chain"
        );
        assert_eq!(
            std::fs::read_dir(&d)
                .unwrap()
                .filter(|e| e.as_ref().unwrap().file_name() != LOG_LOCK)
                .count(),
            EARLIER_LOGS + 1
        );
        log.written = LOG_MAX_BYTES;
        log.line(SystemTime::now(), "after normal rotation")
            .unwrap();
        assert_eq!(texts(&log.path()), ["after normal rotation"]);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    fn seed_archives(dir: &Path, ages: &[usize]) {
        std::fs::create_dir_all(dir).unwrap();
        for age in ages {
            std::fs::write(log_path(dir, *age), format!("x log {age}\n")).unwrap();
        }
    }

    /// The logs after a start that recovered an interrupted rotation: the previous run's logs
    /// moved one place older once, the oldest gone, and a new empty current log.
    fn assert_rotated_once(dir: &Path, ages: &[usize]) {
        let mut expected = std::collections::BTreeMap::new();
        expected.insert(log_path(dir, 0), Vec::new());
        for age in ages {
            if *age < EARLIER_LOGS {
                expected.insert(
                    log_path(dir, *age + 1),
                    format!("x log {age}\n").into_bytes(),
                );
            }
        }
        let actual: std::collections::BTreeMap<_, _> = std::fs::read_dir(dir)
            .unwrap()
            .filter(|entry| entry.as_ref().unwrap().file_name() != LOG_LOCK)
            .map(|entry| {
                let path = entry.unwrap().path();
                let bytes = std::fs::read(&path).unwrap();
                (path, bytes)
            })
            .collect();
        assert_eq!(actual, expected);
    }

    fn rotate_after_recovery(log: &mut Log) {
        log.line(SystemTime::now(), "after restart").unwrap();
        let previous = read(&log.path());
        log.written = LOG_MAX_BYTES;
        log.line(SystemTime::now(), "after normal rotation")
            .unwrap();
        assert_eq!(texts(&log.path()), ["after normal rotation"]);
        assert_eq!(read(&log_path(&log.dir, 1)), previous);
        assert!(!rotation_dir(&log.dir).exists());
    }

    #[test]
    fn a_restart_after_each_forward_step_recovers_from_files_alone() {
        for ages in [&[0, 1, 2, 3, 4][..], &[0, 2, 4][..]] {
            let d = dir(&format!("forward-restart-{}", ages.len()));
            seed_archives(&d, ages);
            let mut count = 0;
            Rotation::default()
                .set_aside_with(&d, |step| {
                    step.run()?;
                    count += 1;
                    Ok(())
                })
                .unwrap();
            std::fs::remove_dir_all(&d).unwrap();
            for crash in 1..=count {
                seed_archives(&d, ages);
                let mut calls = 0;
                let interrupted = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    Rotation::default().set_aside_with(&d, |step| {
                        step.run()?;
                        calls += 1;
                        assert_ne!(
                            calls, crash,
                            "injected supervisor crash after operation {crash}"
                        );
                        Ok(())
                    })
                }));
                assert!(interrupted.is_err(), "operation {crash} was not reached");
                let staging = rotation_dir(&d);
                if staging.join("ready").exists()
                    && (0..EARLIER_LOGS).any(|age| log_path(&staging, age).exists())
                {
                    assert!(log_path(&staging, EARLIER_LOGS).exists(), "the oldest archive was deleted before the retained logs reached their places");
                }
                // The old object is gone. Startup has only filenames and the commit marker.
                let mut log = Log::open(&d).unwrap();
                // Before the marker the start undid the pass and rotated again; after it, the
                // start finished that pass. Either way the logs advance exactly once.
                assert_rotated_once(&d, ages);
                rotate_after_recovery(&mut log);
                drop(log);
                std::fs::remove_dir_all(&d).unwrap();
            }
        }
    }

    #[test]
    fn a_restart_after_each_undo_step_recovers_from_files_alone() {
        for ages in [&[0, 1, 2, 3, 4][..], &[0, 2, 4][..]] {
            let d = dir(&format!("undo-restart-{}", ages.len()));
            for staged in 1..=ages.len() {
                for crash in 1..=staged + 1 {
                    seed_archives(&d, ages);
                    let staging = rotation_dir(&d);
                    std::fs::create_dir(&staging).unwrap();
                    for age in &ages[..staged] {
                        std::fs::rename(log_path(&d, *age), log_path(&staging, *age)).unwrap();
                    }
                    let mut calls = 0;
                    let interrupted =
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            Rotation::default().recover_with(&d, &mut |step| {
                                step.run()?;
                                calls += 1;
                                assert_ne!(
                                    calls, crash,
                                    "injected supervisor crash during undo {crash}"
                                );
                                Ok(())
                            })
                        }));
                    assert!(interrupted.is_err());
                    let mut log = Log::open(&d).unwrap();
                    // Whether or not undo got to remove the directory, this start begins a new
                    // log.
                    assert_rotated_once(&d, ages);
                    rotate_after_recovery(&mut log);
                    drop(log);
                    std::fs::remove_dir_all(&d).unwrap();
                }
            }
        }
    }

    /// A refusal: the staging and log files that make recovery refuse, and the one file in the
    /// way, which the log must name.
    fn seed_refusal(d: &Path, kind: usize) -> PathBuf {
        seed_archives(d, &[0, 1, 2, 3, 4]);
        let staging = rotation_dir(d);
        if kind == 4 {
            std::fs::write(&staging, "not a folder").unwrap();
            return staging;
        }
        std::fs::create_dir(&staging).unwrap();
        match kind {
            0 => {
                std::fs::write(staging.join("unrelated"), "keep this").unwrap();
                staging.join("unrelated")
            }
            1 => {
                std::fs::write(staging.join("ready"), "not an empty marker").unwrap();
                staging.join("ready")
            }
            2 => {
                std::fs::write(log_path(&staging, 0), "staged current").unwrap();
                log_path(d, 0)
            }
            3 => {
                std::fs::write(log_path(&staging, 0), "staged current").unwrap();
                std::fs::write(staging.join("ready"), "").unwrap();
                log_path(d, 1)
            }
            5 => {
                std::fs::write(log_path(&staging, 0), "staged current").unwrap();
                std::fs::write(staging.join("ready"), "").unwrap();
                std::fs::remove_file(log_path(d, 1)).unwrap();
                std::fs::create_dir(log_path(d, 1)).unwrap();
                log_path(d, 1)
            }
            _ => {
                std::fs::create_dir(staging.join("folder")).unwrap();
                staging.join("folder")
            }
        }
    }

    #[test]
    fn a_disconnected_writer_still_drains_and_counts_pipe_output() {
        let (to, from) = output_channel();
        drop(from);
        let dropped = Arc::new(AtomicU64::new(0));
        // More than a read chunk: abandoning the reader leaves the child's pipe undrained.
        forward(
            io::Cursor::new("line\n".repeat(100_000).into_bytes()),
            to,
            dropped.clone(),
        )
        .join()
        .unwrap();
        assert_eq!(dropped.load(Ordering::Relaxed), 100_000);
    }

    #[test]
    fn the_writer_keeps_one_bounded_fifo_and_counts_both_kinds_of_loss() {
        let d = dir("writer-fifo");
        let (entered, opening) = mpsc::sync_channel(1);
        let (release, wait) = mpsc::sync_channel(1);
        let mut writer = Writer::with_open(&d, move |dir| {
            entered.send(()).unwrap();
            wait.recv_timeout(Duration::from_secs(5)).unwrap();
            Log::open(dir).unwrap()
        });
        writer.note("first");
        opening.recv_timeout(Duration::from_secs(5)).unwrap();
        use std::fmt::Write as _;
        let mut input = String::new();
        for n in 0..1027 {
            writeln!(input, "line {n}").unwrap();
        }
        forward(
            io::Cursor::new(input.into_bytes()),
            writer.readers(),
            writer.dropped.clone(),
        )
        .join()
        .unwrap();
        writer.note("cannot fit");
        assert_eq!(writer.dropped.load(Ordering::Relaxed), 3);
        assert_eq!(writer.notices_dropped.load(Ordering::Relaxed), 1);
        release.send(()).unwrap();
        writer.finish(Instant::now() + Duration::from_secs(5));
        let lines = texts(writer.path());
        let output = lines
            .iter()
            .filter(|l| l.starts_with("line "))
            .cloned()
            .collect::<Vec<_>>();
        assert_eq!(
            output,
            (0..1024).map(|n| format!("line {n}")).collect::<Vec<_>>()
        );
        let first = lines
            .iter()
            .position(|l| l == "cv1touchd supervisor: first")
            .unwrap();
        assert!(first < lines.iter().position(|l| l == "line 0").unwrap());
        assert!(lines
            .iter()
            .any(|l| l.contains("dropped 3 daemon output lines")));
        assert!(!lines.iter().any(|l| l.contains("cannot fit")));
        assert_eq!(writer.notices_dropped.load(Ordering::Relaxed), 0);
        drop(writer);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_terminal_only_exit_does_not_touch_the_logs_folder() {
        let d = dir("terminal-only");
        let mut writer = Writer::open(&d);
        writer.terminal("a supervisor is already running");
        writer.finish(Instant::now() + LAST_LINES);
        let touched = d.exists();
        if touched {
            std::fs::remove_dir_all(&d).unwrap();
        }
        assert!(!touched, "a refused runtime owner opened or rotated logs");
    }

    #[cfg(unix)]
    fn restart_while_log_is_blocked(
        d: PathBuf,
        open: impl FnOnce(&Path) -> Log + Send + 'static,
        release: impl FnOnce(),
    ) {
        let (done, result) = mpsc::sync_channel(1);
        let root = d.clone();
        let supervisor = std::thread::spawn(move || {
            let mut writer = Writer::with_open(&root, open);
            let mut runs = 0;
            let code = run(
                || {
                    runs += 1;
                    if runs == 1 {
                        shell("exit 1")
                    } else {
                        let mut child = stand_in(&root.join("restarted"));
                        child.env("CV1TOUCH_TEST_FLOOD_OUTPUT", "1");
                        child
                    }
                },
                &mut writer,
                quick(10, 5),
                &mut never(),
            );
            done.send((code, runs)).unwrap();
            writer
        });
        let completed = result.recv_timeout(Duration::from_secs(4));
        let restarted = d.join("restarted").exists();
        // Release even when a mutation blocked supervision, then await cleanup outside its deadline.
        release();
        let writer = supervisor.join().unwrap();
        let _ = writer.finished.recv_timeout(Duration::from_secs(5));
        drop(writer);
        std::fs::remove_dir_all(d).unwrap();
        assert_eq!(
            completed.ok(),
            Some((0, 2)),
            "logging blocked child restart or exit"
        );
        assert!(restarted, "the second child did not drain its output");
    }

    #[cfg(unix)]
    #[test]
    fn a_blocked_log_open_does_not_prevent_a_child_restart() {
        let d = dir("open-restart");
        std::fs::create_dir_all(&d).unwrap();
        let (release, gate) = mpsc::sync_channel(1);
        restart_while_log_is_blocked(
            d,
            move |dir| {
                gate.recv_timeout(Duration::from_secs(10)).unwrap();
                Log::open(dir).unwrap()
            },
            || release.send(()).unwrap(),
        );
    }

    #[cfg(unix)]
    #[test]
    fn a_blocked_log_write_does_not_prevent_a_child_restart() {
        use std::os::fd::OwnedFd;
        use std::os::unix::net::UnixStream;
        let d = dir("file-restart");
        std::fs::create_dir_all(&d).unwrap();
        let (mut sink, blocked) = UnixStream::pair().unwrap();
        sink.set_nonblocking(true).unwrap();
        loop {
            match sink.write(&[b'x'; 4096]) {
                Ok(_) => {}
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => panic!("fill file sink: {e}"),
            }
        }
        sink.set_nonblocking(false).unwrap();
        let fd: OwnedFd = sink.into();
        restart_while_log_is_blocked(
            d,
            move |dir| {
                let mut log = Log::open(dir).unwrap();
                log.file = Some(File::from(fd));
                log
            },
            || drop(blocked),
        );
    }

    #[test]
    fn a_contended_logs_lock_keeps_the_child_running_without_touching_logs() {
        let d = dir("contended-log-owner");
        let owner = Log::open(&d).unwrap();
        let before = archive_files(&d);
        let other = Log::open(&d).expect("logs ownership must not stop tracking");
        assert!(other.lock.is_none());
        assert!(other
            .stderr_notice
            .as_ref()
            .unwrap()
            .contains("another cv1touch supervisor"));
        drop(other);
        let mut other = Writer::open(&d);
        assert_eq!(
            run(
                || shell("echo lost"),
                &mut other,
                quick(10, 5),
                &mut never()
            ),
            0
        );
        assert_eq!(other.dropped.load(Ordering::Relaxed), 1);
        assert_eq!(archive_files(&d), before);
        drop(other);
        drop(owner);
        let recovered = Log::open(&d).unwrap();
        assert!(recovered.lock.is_some());
        drop(recovered);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_real_write_failure_backs_off_before_touching_the_file_again() {
        let d = dir("write-backoff");
        let mut log = Log::open(&d).unwrap();
        // An actual OS write failure on an otherwise readable, empty regular file.
        log.file = Some(File::open(log.path()).unwrap());
        let at = Instant::now();
        assert!(log.line_at(SystemTime::now(), "fails", at).is_err());
        let retry = at + Duration::from_secs(1);
        log.file = Some(fs::append_private(&log.path()).unwrap());
        for _ in 0..10 {
            assert!(log
                .line_at(
                    SystemTime::now(),
                    "too soon",
                    retry - Duration::from_nanos(1)
                )
                .is_err());
        }
        assert_eq!(read(&log.path()), "");
        log.line_at(SystemTime::now(), "recovered", retry).unwrap();
        assert_eq!(texts(&log.path()), ["recovered"]);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_failed_terminal_backs_off_notices_and_summaries_without_losing_counts() {
        #[derive(Default)]
        struct Sink {
            attempts: usize,
            fail: bool,
            bytes: Vec<u8>,
        }
        impl Write for Sink {
            fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
                self.attempts += 1;
                if self.fail {
                    return Err(io::ErrorKind::BrokenPipe.into());
                }
                self.bytes.extend_from_slice(bytes);
                Ok(bytes.len())
            }
            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }
        let mut terminal = Terminal {
            sink: Sink {
                fail: true,
                ..Sink::default()
            },
            dropped: Arc::new(AtomicU64::new(0)),
            retry_after: None,
        };
        let now = Instant::now();
        terminal.line("first", now);
        for ms in 1..1000 {
            terminal.line("still unavailable", now + Duration::from_millis(ms));
            terminal.report(now + Duration::from_millis(ms));
        }
        assert_eq!(
            terminal.sink.attempts, 1,
            "failed terminal was retried before its deadline"
        );
        assert_eq!(terminal.dropped.load(Ordering::Relaxed), 1000);
        terminal.report(now + Duration::from_secs(1));
        assert_eq!(terminal.sink.attempts, 2);
        assert_eq!(terminal.dropped.load(Ordering::Relaxed), 1000);
        for ms in 1001..2000 {
            terminal.report(now + Duration::from_millis(ms));
        }
        assert_eq!(
            terminal.sink.attempts, 2,
            "failed summaries must back off too"
        );
        terminal.sink.fail = false;
        terminal.report(now + Duration::from_secs(2));
        assert_eq!(terminal.dropped.load(Ordering::Relaxed), 0);
        assert!(String::from_utf8(terminal.sink.bytes)
            .unwrap()
            .contains("dropped 1000 supervisor notices"));
    }

    #[test]
    fn a_writer_spawn_failure_preserves_the_queue_and_does_not_stop_the_child() {
        let d = dir("writer-spawn-failure");
        let mut writer = Writer::with_spawn(
            &d,
            |dir| Log::open(dir).unwrap(),
            |_| Err(io::Error::other("injected thread creation failure")),
        );
        assert!(!writer.running);
        assert!(writer
            .failure_notice
            .as_ref()
            .unwrap()
            .contains("could not start"));
        let retry = writer.retry_after.unwrap();
        writer.start_with(retry - Duration::from_nanos(1), |_| {
            panic!("thread creation must back off");
        });
        let input = "queued\n".repeat(LOG_QUEUE_LINES + 5);
        forward(
            std::io::Cursor::new(input),
            writer.readers(),
            writer.dropped.clone(),
        )
        .join()
        .unwrap();
        assert_eq!(writer.dropped.load(Ordering::Relaxed), 5);
        assert!(
            !d.exists(),
            "failed writer must not open files on the supervisor"
        );
        let mut runs = 0;
        assert_eq!(
            run(
                || {
                    runs += 1;
                    if runs == 1 {
                        shell("exit 1")
                    } else {
                        shell(if cfg!(windows) {
                            "ping -n 3 127.0.0.1 >nul & echo recovered"
                        } else {
                            "sleep 2; echo recovered"
                        })
                    }
                },
                &mut writer,
                quick(10, 5),
                &mut never()
            ),
            0
        );
        assert_eq!(runs, 2);
        assert!(
            writer.failure_reported,
            "the replacement must carry the failure notice"
        );
        let lines = texts(writer.path());
        assert!(lines
            .iter()
            .any(|line| line.contains("dropped 5 daemon output lines")));
        assert_eq!(
            lines.iter().filter(|line| *line == "queued").count(),
            LOG_QUEUE_LINES
        );
        assert!(lines.iter().any(|line| line == "recovered"));
        assert_eq!(writer.dropped.load(Ordering::Relaxed), 0);
        drop(writer);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn writer_failures_reach_stderr_once_when_writing_resumes() {
        let d = dir("writer-failure-notices");
        std::fs::create_dir_all(&d).unwrap();
        for (test, reason) in [
            (
                "a_panicking_writer_is_replaced_while_the_child_keeps_running",
                "stopped unexpectedly",
            ),
            (
                "a_writer_spawn_failure_preserves_the_queue_and_does_not_stop_the_child",
                "could not start",
            ),
        ] {
            let capture = d.join(test);
            let mut child = Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    &format!("supervise::tests::{test}"),
                    "--nocapture",
                ])
                .stdout(Stdio::null())
                .stderr(File::create(&capture).unwrap())
                .spawn()
                .unwrap();
            let until = Instant::now() + Duration::from_secs(5);
            let status = loop {
                if let Some(status) = child.try_wait().unwrap() {
                    break status;
                }
                if Instant::now() >= until {
                    let _ = child.kill();
                    child.wait().unwrap();
                    panic!("writer failure stopped supervision");
                }
                std::thread::sleep(Duration::from_millis(10));
            };
            let stderr = read(&capture);
            assert!(status.success(), "{stderr}");
            assert_eq!(
                stderr
                    .matches("cv1touchd supervisor: the log writer")
                    .count(),
                1,
                "{stderr}"
            );
            assert!(stderr.contains(reason), "{stderr}");
            assert!(stderr.contains("Tracking continues"), "{stderr}");
        }
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_failed_writer_is_not_replaced_until_its_teardown_finishes() {
        let d = dir("writer-teardown");
        let (started, teardown) = mpsc::channel();
        let (release, released) = mpsc::channel();
        let mut writer = Writer::with_open(&d, move |dir| {
            let mut log = Log::open(dir).unwrap();
            // These callbacks run only after this Log has moved into the writer's Option.
            log.hooks.before_write = Some(Box::new(|| panic!("injected initialized log failure")));
            log.hooks.before_drop = Some(Box::new(move || {
                started.send(()).unwrap();
                released.recv().unwrap();
            }));
            log
        });
        writer.note("trigger the failure");
        teardown.recv_timeout(Duration::from_secs(5)).unwrap();
        writer.poll(Instant::now() + Duration::from_secs(10));
        let premature = writer.completed.load(Ordering::Acquire)
            || writer.failure_notice.is_some()
            || writer.failure_reported;
        release.send(()).unwrap();
        writer.finish(Instant::now() + Duration::from_secs(5));
        assert!(
            !premature,
            "a writer still releasing files must not be replaced"
        );
        drop(writer);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_panicking_writer_is_replaced_while_the_child_keeps_running() {
        let d = dir("panicking-writer");
        let mut writer = Writer::with_open(&d, |_| panic!("injected writer open failure"));
        let mut runs = 0;
        assert_eq!(
            run(
                || {
                    runs += 1;
                    if runs == 1 {
                        shell("exit 1")
                    } else {
                        shell(if cfg!(windows) {
                            "ping -n 3 127.0.0.1 >nul & echo recovered"
                        } else {
                            "sleep 2; echo recovered"
                        })
                    }
                },
                &mut writer,
                quick(10, 5),
                &mut never()
            ),
            0
        );
        assert_eq!(runs, 2);
        assert!(
            writer.path().exists(),
            "the failed writer was never replaced"
        );
        assert!(writer.failure_reported);
        assert!(texts(writer.path()).iter().any(|line| line == "recovered"));
        drop(writer);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn maintenance_and_full_log_output_share_one_rotation_retry_deadline() {
        let d = dir("shared-rotation-deadline");
        let blocker = log_path(&d, 1);
        std::fs::create_dir_all(&blocker).unwrap();
        File::create(log_path(&d, 0))
            .unwrap()
            .set_len(LOG_MAX_BYTES)
            .unwrap();
        let mut log = Log::open(&d).unwrap();
        let initial_attempts = log.rotation.attempts;
        log.dropped.store(3, Ordering::Relaxed);
        let now = Instant::now() + Duration::from_secs(2);
        for second in 0..3 {
            let at = now + Duration::from_secs(second);
            log.maintain(at);
            assert_eq!(
                log.rotation.attempts,
                initial_attempts + second as usize + 1,
                "maintenance and its loss summary retried the same blocked rotation"
            );
            for ms in [0, 100, 500, 999] {
                let at = at + Duration::from_millis(ms);
                log.maintain(at);
                let dropped = log.dropped.clone();
                output_line(&mut log, &dropped, "lost output");
                assert!(log.line_at(SystemTime::now(), "another line", at).is_err());
                assert_eq!(
                    log.rotation.attempts,
                    initial_attempts + second as usize + 1,
                    "output bypassed the recovery backoff"
                );
            }
        }
        assert_eq!(std::fs::metadata(log.path()).unwrap().len(), LOG_MAX_BYTES);
        assert_eq!(
            initial_attempts, 1,
            "the startup notice shares the same deadline"
        );
        std::fs::remove_dir(&blocker).unwrap();
        log.maintain(now + Duration::from_secs(3));
        assert_eq!(log.rotation.attempts, initial_attempts + 4);
        assert_eq!(log.dropped.load(Ordering::Relaxed), 0);
        assert!(texts(&log.path())
            .iter()
            .any(|line| line.contains("dropped 15 daemon output lines")));
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_quiet_child_retries_recovery_after_a_startup_blocker_is_removed() {
        let d = dir("quiet-startup-recovery");
        seed_archives(&d, &[0, 1, 2, 3, 4]);
        let staging = rotation_dir(&d);
        std::fs::create_dir(&staging).unwrap();
        let blocker = staging.join("unrelated");
        std::fs::write(&blocker, "keep").unwrap();
        let mut log = Log::open(&d).unwrap();
        let current = read(&log.path());
        assert!(current.contains("can't start a new log file"));
        log.maintain(Instant::now() + Duration::from_secs(2));
        assert_eq!(
            read(&log.path()),
            current,
            "retries must not repeat the notice"
        );
        std::fs::remove_file(&blocker).unwrap();
        // No new line arrives to trigger a full-file rotation.
        log.maintain(Instant::now() + Duration::from_secs(4));
        assert!(!staging.exists(), "quiet startup recovery was not retried");
        assert_eq!(read(&log_path(&d, 1)), current);
        assert_eq!(read(&log.path()), "");
        log.line(SystemTime::now(), "after recovery").unwrap();
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_write_retry_does_not_require_a_blocked_rotation_to_succeed() {
        let d = dir("write-retry-with-blocked-rotation");
        let blocker = log_path(&d, 1);
        std::fs::create_dir_all(&blocker).unwrap();
        std::fs::write(log_path(&d, 0), "x previous\n").unwrap();
        let mut log = Log::open(&d).unwrap();
        let before = texts(&log.path());
        assert!(before
            .iter()
            .any(|line| line.contains("can't start a new log file")));
        // Read-only handle: an actual OS write failure, independent of rotation.
        log.file = Some(File::open(log.path()).unwrap());
        let now = Instant::now();
        assert!(log.line_at(SystemTime::now(), "fails", now).is_err());
        log.file = Some(fs::append_private(&log.path()).unwrap());
        log.line_at(SystemTime::now(), "recovered", now + Duration::from_secs(1))
            .expect("a writable current log must not depend on an archive moving");
        let mut expected = before;
        expected.push("recovered".into());
        assert_eq!(texts(&log.path()), expected);
        assert!(blocker.is_dir());
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_quiet_child_recovers_after_a_replacement_cannot_be_opened() {
        let d = dir("quiet-replacement");
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "before").unwrap();
        let at = Instant::now();
        assert!(log
            .rotate_with(at, |_| Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "replacement unavailable"
            )))
            .is_err());
        let until = log.rotate_after.unwrap();
        log.dropped.store(3, Ordering::Relaxed);
        log.maintain(until - Duration::from_nanos(1));
        assert!(!log.path().exists());
        log.maintain(until);
        assert_eq!(texts(&log_path(&d, 1)), ["before"]);
        assert!(texts(&log.path())
            .iter()
            .any(|l| l.contains("dropped 3 daemon output lines")));
        assert_eq!(log.dropped.load(Ordering::Relaxed), 0);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn a_full_supervisor_stderr_does_not_prevent_a_child_restart() {
        use std::os::fd::{AsRawFd, FromRawFd};
        let d = dir("stderr-restart");
        std::fs::create_dir_all(&d).unwrap();
        let mut fds = [-1; 2];
        // SAFETY: pipe initializes both descriptors, each then owned by one File.
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
        let read = unsafe { File::from_raw_fd(fds[0]) };
        let mut write = unsafe { File::from_raw_fd(fds[1]) };
        let fd = write.as_raw_fd();
        let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
        assert!(flags >= 0);
        assert_eq!(
            unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) },
            0
        );
        loop {
            match write.write(&[b'x'; 4096]) {
                Ok(_) => {}
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => panic!("fill supervisor stderr: {e}"),
            }
        }
        assert_eq!(unsafe { libc::fcntl(fd, libc::F_SETFL, flags) }, 0);
        let mut child = Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "supervise::tests::stderr_restart_fixture",
                "--ignored",
                "--nocapture",
            ])
            .env("CV1TOUCH_TEST_STDERR_RESTART", &d)
            .stdout(Stdio::null())
            .stderr(Stdio::from(write))
            .spawn()
            .unwrap();
        let until = Instant::now() + Duration::from_secs(5);
        let status = loop {
            if let Some(status) = child.try_wait().unwrap() {
                break Some(status);
            }
            if Instant::now() >= until {
                child.kill().unwrap();
                child.wait().unwrap();
                break None;
            }
            std::thread::sleep(Duration::from_millis(10));
        };
        drop(read);
        let restarted = d.join("restarted").exists();
        std::fs::remove_dir_all(d).unwrap();
        assert!(
            status.is_some_and(|s| s.success()),
            "supervision waited for stderr: {status:?}"
        );
        assert!(restarted, "the second child never ran");
    }

    // Offline subprocess: the stand-in writes a marker after its output has drained; no devices.
    #[test]
    #[ignore = "run by a_full_supervisor_stderr_does_not_prevent_a_child_restart"]
    fn stderr_restart_fixture() {
        let d = PathBuf::from(std::env::var_os("CV1TOUCH_TEST_STDERR_RESTART").unwrap());
        std::fs::create_dir_all(log_path(&d, 0)).unwrap();
        let mut log = Writer::open(&d);
        let mut runs = 0;
        assert_eq!(
            run(
                || {
                    runs += 1;
                    if runs == 1 {
                        shell("exit 1")
                    } else {
                        let mut c = stand_in(&d.join("restarted"));
                        c.env("CV1TOUCH_TEST_FLOOD_OUTPUT", "1");
                        c
                    }
                },
                &mut log,
                quick(10, 5),
                &mut never()
            ),
            0
        );
        assert_eq!(runs, 2);
    }

    #[test]
    fn round6_initial_metadata_failures_retry_opening_without_aging_logs() {
        let d = dir("initial-metadata");
        seed_archives(&d, &[0, 1, 2, 3, 4]);
        let mut log = Log::open_using(
            &d,
            |step| step.run(),
            |path| {
                let _file = fs::append_private(path)?;
                Err(io::Error::new(
                    io::ErrorKind::PermissionDenied,
                    "injected metadata failure",
                ))
            },
        )
        .expect("a metadata error must not stop tracking");
        for _ in 0..7 {
            let until = log.rotate_after.unwrap();
            assert!(log
                .start_with(
                    until,
                    |_| panic!("the archives must not move again"),
                    |path| {
                        let _file = fs::append_private(path)?;
                        Err(io::Error::new(
                            io::ErrorKind::PermissionDenied,
                            "injected metadata failure",
                        ))
                    }
                )
                .is_err());
            for age in 1..=4 {
                assert_eq!(read(&log_path(&d, age)), format!("x log {}\n", age - 1));
            }
        }
        let until = log.rotate_after.unwrap();
        log.start_with(
            until,
            |_| panic!("only opening needs retrying"),
            open_current,
        )
        .unwrap();
        log.line(SystemTime::now(), "recovered").unwrap();
        assert_eq!(texts(&log.path()), ["recovered"]);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn round6_logging_retries_without_waiting_for_more_child_output() {
        let d = dir("quiet-retry");
        std::fs::create_dir_all(log_path(&d, 0)).unwrap();
        let mut log = Log::open(&d).unwrap();
        let until = log.rotate_after.unwrap();
        log.dropped.store(2, Ordering::Relaxed);
        std::fs::remove_dir(log.path()).unwrap();
        log.maintain(until - Duration::from_millis(1));
        assert!(
            !log.path().exists(),
            "opening ignored the one-second deadline"
        );
        log.maintain(until);
        assert!(texts(&log.path())
            .iter()
            .any(|l| l.contains("dropped 2 daemon output lines")));
        assert_eq!(log.dropped.load(Ordering::Relaxed), 0);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn round6_a_startup_move_failure_names_the_locked_log() {
        let d = dir("initial-rename");
        seed_archives(&d, &[0, 1, 2, 3, 4]);
        let mut log = Log::open_with(&d, |step| match &step {
            RotationStep::Rename(from, _) if *from == log_path(&d, 0) => Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "injected current log lock",
            )),
            _ => step.run(),
        })
        .unwrap();
        let lines = texts(&log.path());
        assert_eq!(lines.len(), 2, "{lines:?}");
        assert!(
            lines[1].contains(&format!("{} could not be moved", log.path().display())),
            "{lines:?}"
        );
        assert!(lines[1].contains("Close any program"), "{lines:?}");
        assert!(!lines[1].contains("Move that file"), "{lines:?}");
        log.line(SystemTime::now(), "goes on").unwrap();
        assert_eq!(texts(&log.path()).last().unwrap(), "goes on");
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn round6_a_folder_at_the_current_log_does_not_stop_the_daemon() {
        let d = dir("unwritable-current");
        let current = log_path(&d, 0);
        std::fs::create_dir_all(&current).unwrap();
        std::fs::write(current.join("keep"), "unrelated data").unwrap();
        let mut log = Log::open(&d).expect("a log error must not stop tracking");
        let dropped = Arc::clone(&log.dropped);
        output_line(&mut log, &dropped, "lost");
        assert_eq!(log.dropped.load(Ordering::Relaxed), 1);
        assert_eq!(read(&current.join("keep")), "unrelated data");
        std::fs::remove_dir_all(&current).unwrap();
        // Opening still waits for the shared one-second deadline after the path is fixed.
        let before = log.rotate_after.unwrap() - Duration::from_millis(1);
        assert!(log.line_at(SystemTime::now(), "too soon", before).is_err());
        assert!(!current.exists());
        log.rotate_after = None;
        let dropped = Arc::clone(&log.dropped);
        output_line(&mut log, &dropped, "recovered");
        let lines = texts(&current);
        assert!(
            lines
                .iter()
                .any(|l| l.contains("dropped 1 daemon output line")),
            "{lines:?}"
        );
        assert_eq!(lines.last().unwrap(), "recovered");
        assert_eq!(dropped.load(Ordering::Relaxed), 0);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn round6_a_link_at_the_current_log_does_not_stop_the_daemon() {
        let d = dir("unwritable-link");
        std::fs::create_dir_all(&d).unwrap();
        let target = d.join("unrelated");
        std::fs::write(&target, "keep this").unwrap();
        std::os::unix::fs::symlink(&target, log_path(&d, 0)).unwrap();
        let mut log = Log::open(&d).expect("a link must not stop tracking");
        let dropped = Arc::clone(&log.dropped);
        output_line(&mut log, &dropped, "lost");
        assert_eq!(log.dropped.load(Ordering::Relaxed), 1);
        assert_eq!(read(&target), "keep this");
        std::fs::remove_file(log.path()).unwrap();
        log.rotate_after = None;
        let dropped = Arc::clone(&log.dropped);
        output_line(&mut log, &dropped, "recovered");
        assert_eq!(texts(&log.path()).last().unwrap(), "recovered");
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[cfg(windows)]
    #[test]
    fn round6_a_locked_current_log_does_not_stop_the_daemon() {
        use std::os::windows::fs::OpenOptionsExt;
        let d = dir("unwritable-lock");
        std::fs::create_dir_all(&d).unwrap();
        let current = log_path(&d, 0);
        std::fs::write(&current, "x previous\n").unwrap();
        let held = File::options()
            .read(true)
            .share_mode(0)
            .open(&current)
            .unwrap();
        let mut log = Log::open(&d).expect("a locked file must not stop tracking");
        let dropped = Arc::clone(&log.dropped);
        output_line(&mut log, &dropped, "lost");
        assert_eq!(log.dropped.load(Ordering::Relaxed), 1);
        drop(held);
        log.rotate_after = None;
        let dropped = Arc::clone(&log.dropped);
        output_line(&mut log, &dropped, "recovered");
        assert_eq!(texts(&log.path()).last().unwrap(), "recovered");
        assert!(
            read(&log.path()).starts_with("x previous\n")
                || read(&log_path(&d, 1)) == "x previous\n"
        );
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn round6_an_unavailable_logs_folder_does_not_stop_the_daemon() {
        let d = dir("unwritable-dir");
        std::fs::write(&d, "unrelated data").unwrap();
        let mut log = Log::open(&d).expect("a log folder error must not stop tracking");
        let dropped = Arc::clone(&log.dropped);
        output_line(&mut log, &dropped, "lost");
        assert_eq!(read(&d), "unrelated data");
        assert_eq!(log.dropped.load(Ordering::Relaxed), 1);
        std::fs::remove_file(&d).unwrap();
        log.rotate_after = None;
        let dropped = Arc::clone(&log.dropped);
        output_line(&mut log, &dropped, "recovered");
        assert_eq!(texts(&log.path()).last().unwrap(), "recovered");
        assert_eq!(dropped.load(Ordering::Relaxed), 0);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn round6_two_log_owners_cannot_recover_the_same_state() {
        let d = dir("logs-lock");
        seed_archives(&d, &[0, 1, 2, 3, 4]);
        let first = Log::open(&d).unwrap();
        let before = archive_files(&d);
        let second = Log::open(&d).unwrap();
        assert!(second.lock.is_none());
        assert_eq!(archive_files(&d), before);
        drop(second);
        drop(first);
        let second = Log::open(&d).unwrap();
        assert!(second.lock.is_some());
        drop(second);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_notice_lost_during_opening_keeps_its_count() {
        let d = dir("opening-notice-count");
        std::fs::create_dir_all(log_path(&d, 1)).unwrap();
        File::create(log_path(&d, 0))
            .unwrap()
            .set_len(LOG_MAX_BYTES)
            .unwrap();
        let output = Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "--ignored",
                "--nocapture",
                "supervise::tests::unavailable_log_fixture",
            ])
            .env("CV1TOUCH_TEST_UNAVAILABLE_LOG", &d)
            .output()
            .unwrap();
        assert!(output.status.success());
        let stderr = String::from_utf8_lossy(&output.stderr);
        let lost: u64 = stderr
            .lines()
            .filter_map(|line| line.strip_prefix("cv1touchd supervisor: dropped "))
            .filter_map(|line| line.split_once(" supervisor notice"))
            .map(|(count, _)| count.parse::<u64>().unwrap())
            .sum();
        std::fs::remove_dir_all(d).unwrap();
        // The blocked-opening notice, then the child's started and stopped notices.
        assert_eq!(lost, 3, "{stderr}");
    }

    #[test]
    fn round6_unavailable_logging_is_reported_to_supervisor_stderr() {
        let d = dir("notice-stderr");
        std::fs::create_dir_all(log_path(&d, 0)).unwrap();
        let output = Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "--ignored",
                "--nocapture",
                "supervise::tests::unavailable_log_fixture",
            ])
            .env("CV1TOUCH_TEST_UNAVAILABLE_LOG", &d)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(
            stderr.contains(&log_path(&d, 0).display().to_string()),
            "{stderr}"
        );
        assert!(stderr.contains("Tracking isn't affected"), "{stderr}");
        assert!(!stderr.contains("rotate"), "{stderr}");
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    #[ignore = "run by round6_unavailable_logging_is_reported_to_supervisor_stderr"]
    fn unavailable_log_fixture() {
        let d =
            PathBuf::from(std::env::var_os("CV1TOUCH_TEST_UNAVAILABLE_LOG").expect("fixture path"));
        let mut log = Writer::open(&d);
        assert_eq!(
            run(|| shell("echo lost"), &mut log, quick(10, 5), &mut never()),
            0
        );
    }

    /// Recovery that cannot proceed blocks rotation only: the logs are left alone, one line
    /// names the file in the way, and the daemon still gets a log to write to.
    #[test]
    fn a_refused_recovery_blocks_rotation_but_not_the_daemon() {
        for kind in 0..7 {
            let d = dir(&format!("foreign-staging-{kind}"));
            let blocker = seed_refusal(&d, kind);
            let current = PathBuf::from(format!("{LOG_NAME}.log"));
            let mut before = archive_files(&d);
            let started = before.remove(&current).unwrap().unwrap();
            let mut log = Log::open(&d)
                .unwrap_or_else(|e| panic!("kind {kind}: the daemon could not start: {e}"));
            assert!(!log.rotation.pending(), "kind {kind}");
            let notice = texts(&log.path()).split_off(1);
            assert_eq!(notice.len(), 1, "kind {kind}: {notice:?}");
            assert!(
                notice[0].starts_with("cv1touchd supervisor: "),
                "kind {kind}: {notice:?}"
            );
            let named = if kind == 2 {
                format!("{} is already there", blocker.display())
            } else {
                blocker.display().to_string()
            };
            assert!(
                notice[0].contains(&named),
                "kind {kind}: the notice does not name {}: {notice:?}",
                blocker.display()
            );
            assert!(
                notice[0].contains("can't start a new log file"),
                "kind {kind}: {notice:?}"
            );
            assert!(notice[0].contains("Move that file or folder"), "{notice:?}");
            assert!(!notice[0].contains("Close any program"), "{notice:?}");
            log.line(SystemTime::now(), "goes on").unwrap();
            assert_eq!(texts(&log.path()).last().unwrap(), "goes on");
            // A full log still cannot rotate, and does not grow.
            log.written = LOG_MAX_BYTES;
            let full = read(&log.path());
            for _ in 0..3 {
                assert!(log.line(SystemTime::now(), "cannot fit").is_err());
            }
            assert_eq!(read(&log.path()), full);
            assert!(!log.rotation.pending(), "kind {kind}");
            drop(log);
            // Every start goes on the same way, and none touches the files in the way.
            for _ in 0..3 {
                Log::open(&d).unwrap();
            }
            let mut after = archive_files(&d);
            let now = after.remove(&current).unwrap().unwrap();
            assert!(now.starts_with(&started), "kind {kind}");
            assert_eq!(after, before, "kind {kind}");
            std::fs::remove_dir_all(d).unwrap();
        }
    }

    /// The blocker may be moved while the supervisor runs, or before its next start.
    #[test]
    fn a_blocked_rotation_resumes_when_the_file_in_the_way_is_moved() {
        for kind in [0, 3, 5] {
            for restart in [false, true] {
                let d = dir(&format!("blocked-resumes-{kind}-{restart}"));
                let blocker = seed_refusal(&d, kind);
                let mut log = Log::open(&d).unwrap();
                log.line(SystemTime::now(), "goes on").unwrap();
                log.written = LOG_MAX_BYTES;
                assert!(log.line(SystemTime::now(), "cannot fit").is_err());
                assert!(!log.rotation.pending());
                let kept = read(&log.path());
                if blocker.is_dir() {
                    std::fs::remove_dir(&blocker).unwrap();
                } else {
                    std::fs::remove_file(&blocker).unwrap();
                }
                if restart {
                    drop(log);
                    log = Log::open(&d).unwrap();
                }
                // Removing the blocker does not bypass the shared recovery/rotation deadline.
                log.line_at(
                    SystemTime::now(),
                    "after the move",
                    Instant::now() + Duration::from_secs(1),
                )
                .unwrap();
                assert_eq!(texts(&log.path()), ["after the move"]);
                let rotated = read(&log_path(&d, 1));
                assert!(rotated.starts_with("x log 0\n") && rotated.contains("goes on"));
                assert_eq!(rotated, kept);
                assert!(!rotation_dir(&d).exists());
                drop(log);
                std::fs::remove_dir_all(d).unwrap();
            }
        }
    }

    /// A start that could not recover: the log is not rotating, says so once and names a file
    /// under `named`, and goes on; no log in `ages` was overwritten or lost.
    fn assert_started_blocked(log: &mut Log, named: &Path, ages: std::ops::RangeInclusive<usize>) {
        assert!(!log.rotation.pending());
        let notice: Vec<_> = texts(&log.path())
            .into_iter()
            .filter(|line| line.starts_with("cv1touchd supervisor: "))
            .collect();
        assert_eq!(notice.len(), 1, "{notice:?}");
        assert!(
            notice[0].contains("can't start a new log file"),
            "{notice:?}"
        );
        assert!(
            notice[0].contains(&named.display().to_string()),
            "{notice:?}"
        );
        log.line(SystemTime::now(), "goes on").unwrap();
        assert_eq!(texts(&log.path()).last().unwrap(), "goes on");
        let mut contents = Vec::new();
        let mut stack = vec![log.dir.clone()];
        while let Some(path) = stack.pop() {
            for entry in std::fs::read_dir(path).unwrap() {
                let path = entry.unwrap().path();
                if path.file_name().is_some_and(|name| name == LOG_LOCK) {
                    continue;
                }
                if path.is_dir() {
                    stack.push(path);
                } else {
                    contents.push(read(&path));
                }
            }
        }
        for age in ages {
            // A log that was restored goes on with the new lines after its own.
            let original = format!("x log {age}\n");
            let found = contents.iter().filter(|c| c.starts_with(&original)).count();
            assert_eq!(found, 1, "log {age} is gone or repeated");
        }
    }

    /// A failure part way through recovery at startup also leaves the daemon running, with the
    /// remaining files where they were and nothing written through a moved file.
    #[test]
    fn a_recovery_that_fails_part_way_does_not_stop_the_daemon() {
        for committed in [false, true] {
            let steps = if committed { 7 } else { 4 };
            for failure in 1..=steps {
                let d = dir(&format!("recovery-io-{committed}-{failure}"));
                seed_archives(&d, &[0, 1, 2, 3, 4]);
                let staging = rotation_dir(&d);
                std::fs::create_dir(&staging).unwrap();
                let staged = if committed { 5 } else { 3 };
                for age in 0..staged {
                    std::fs::rename(log_path(&d, age), log_path(&staging, age)).unwrap();
                }
                if committed {
                    std::fs::write(staging.join("ready"), "").unwrap();
                }
                let mut calls = 0;
                let mut log = Log::open_with(&d, |step| {
                    calls += 1;
                    if calls == failure {
                        return Err(io::Error::new(
                            io::ErrorKind::PermissionDenied,
                            "injected recovery failure",
                        ));
                    }
                    step.run()
                })
                .unwrap_or_else(|e| {
                    panic!("committed {committed}, step {failure}: the daemon could not start: {e}")
                });
                let notice = texts(&log.path()).join("\n");
                assert!(notice.contains("injected recovery failure"), "{notice}");
                // a finished rotation retires the oldest
                let oldest = if committed {
                    EARLIER_LOGS - 1
                } else {
                    EARLIER_LOGS
                };
                assert_started_blocked(&mut log, &staging, 0..=oldest);
                drop(log);
                // The next start has no injected failure.
                let mut log = Log::open(&d).unwrap();
                log.line(SystemTime::now(), "next start").unwrap();
                let lines = texts(&log.path());
                if !committed && failure == 1 {
                    // The log written meanwhile stands where the old current one must go back,
                    // so the files are left alone again, and the line says which one.
                    let found = format!("{} is already there", log.path().display());
                    assert!(lines.iter().any(|line| line.contains(&found)), "{lines:?}");
                    assert!(rotation_dir(&d).exists());
                } else {
                    // Nothing is left to finish: the start rotates as any other, and the log
                    // written meanwhile becomes the newest earlier one.
                    assert_eq!(lines, ["next start"]);
                    let earlier = read(&log_path(&d, 1));
                    assert!(earlier.contains("goes on"), "{earlier}");
                    assert_eq!(earlier.starts_with("x log 0\n"), !committed);
                    let kept = if committed { 0 } else { 1 };
                    for age in 2..=EARLIER_LOGS {
                        let original = format!("x log {}\n", age - 2 + kept);
                        assert_eq!(read(&log_path(&d, age)), original);
                    }
                    assert!(!rotation_dir(&d).exists());
                }
                drop(log);
                std::fs::remove_dir_all(d).unwrap();
            }
        }
    }

    /// A rotation begun at startup whose own undo then fails leaves a log in the staging folder
    /// too; the daemon starts all the same.
    #[test]
    fn a_failed_move_and_undo_at_startup_do_not_stop_the_daemon() {
        let d = dir("startup-undo");
        seed_archives(&d, &[0, 1, 2, 3, 4]);
        let staging = rotation_dir(&d);
        let mut log = Log::open_with(&d, |step| match &step {
            RotationStep::Rename(from, _)
                if *from == log_path(&d, 2) || from.starts_with(&staging) =>
            {
                Err(io::Error::new(
                    io::ErrorKind::PermissionDenied,
                    "injected rename failure",
                ))
            }
            _ => step.run(),
        })
        .unwrap_or_else(|e| panic!("the daemon could not start: {e}"));
        assert_started_blocked(&mut log, &staging, 0..=EARLIER_LOGS);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_start_that_undoes_an_interrupted_rotation_begins_a_new_log() {
        let d = dir("undone-start");
        seed_archives(&d, &[0, 1, 2, 3, 4]);
        let staging = rotation_dir(&d);
        std::fs::create_dir(&staging).unwrap();
        std::fs::rename(log_path(&d, 0), log_path(&staging, 0)).unwrap();
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "this run").unwrap();
        assert_eq!(texts(&log.path()), ["this run"]);
        for age in 1..=EARLIER_LOGS {
            assert_eq!(read(&log_path(&d, age)), format!("x log {}\n", age - 1));
        }
        assert!(!rotation_dir(&d).exists());
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn every_failed_rotation_step_can_be_recovered() {
        let d = dir("rotation-steps");
        for failure in 1..=14 {
            seed_archives(&d, &[0, 1, 2, 3, 4]);
            let mut calls = 0;
            let mut rotation = Rotation::default();
            assert!(rotation
                .set_aside_with(&d, |step| {
                    calls += 1;
                    if calls == failure {
                        Err(io::Error::new(
                            io::ErrorKind::PermissionDenied,
                            "injected rotation failure",
                        ))
                    } else {
                        step.run()
                    }
                })
                .is_err());
            let mut log = Log::open(&d).unwrap();
            // Before the marker, rollback finished and startup rotates normally. After it,
            // startup finishes this rotation. Either way the logs advance exactly once.
            assert_rotated_once(&d, &[0, 1, 2, 3, 4]);
            rotate_after_recovery(&mut log);
            drop(log);
            std::fs::remove_dir_all(&d).unwrap();
        }
    }

    #[test]
    fn an_interrupted_restore_finishes_before_another_rotation() {
        let d = dir("rotation-restore");
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "original").unwrap();
        let original = read(&log.path());
        for age in 1..=EARLIER_LOGS {
            std::fs::write(log_path(&d, age), format!("archive {age}\n")).unwrap();
        }
        let staged_current = log_path(&rotation_dir(&d), 0);
        assert!(log
            .rotation
            .set_aside_with(&d, |step| {
                match &step {
                    RotationStep::Rename(from, _)
                        if *from == log_path(&d, 4) || *from == staged_current =>
                    {
                        Err(io::Error::new(
                            io::ErrorKind::PermissionDenied,
                            "injected move and restore failure",
                        ))
                    }
                    _ => step.run(),
                }
            })
            .is_err());
        assert!(log.rotation.pending());
        assert_eq!(read(&staged_current), original);
        let partial = archive_files(&d);
        for _ in 0..=EARLIER_LOGS + 2 {
            assert!(log
                .rotation
                .set_aside_with(&d, |step| {
                    match &step {
                        RotationStep::Rename(from, _) if *from == staged_current => {
                            Err(io::Error::new(
                                io::ErrorKind::PermissionDenied,
                                "injected restore lock",
                            ))
                        }
                        _ => step.run(),
                    }
                })
                .is_err());
            assert_eq!(archive_files(&d), partial);
        }
        log.maintain(Instant::now());
        assert!(!log.rotation.pending(), "quiet recovery was not retried");
        log.line(SystemTime::now(), "after recovery").unwrap();
        assert!(!log.rotation.pending());
        assert_eq!(texts(&log.path()), ["after recovery"]);
        assert_eq!(read(&log_path(&d, 1)), original);
        for age in 2..=EARLIER_LOGS {
            assert_eq!(read(&log_path(&d, age)), format!("archive {}\n", age - 1));
        }
        assert_eq!(
            std::fs::read_dir(&d)
                .unwrap()
                .filter(|e| e.as_ref().unwrap().file_name() != LOG_LOCK)
                .count(),
            EARLIER_LOGS + 1
        );
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    fn archive_files(dir: &Path) -> std::collections::BTreeMap<PathBuf, Option<Vec<u8>>> {
        fn visit(
            root: &Path,
            path: &Path,
            files: &mut std::collections::BTreeMap<PathBuf, Option<Vec<u8>>>,
        ) {
            for entry in std::fs::read_dir(path).unwrap() {
                let entry = entry.unwrap();
                if entry.path() == root.join(LOG_LOCK) {
                    continue;
                }
                let path = entry.path();
                if entry.file_type().unwrap().is_dir() {
                    files.insert(path.strip_prefix(root).unwrap().to_path_buf(), None);
                    visit(root, &path, files);
                } else {
                    files.insert(
                        path.strip_prefix(root).unwrap().to_path_buf(),
                        Some(std::fs::read(&path).unwrap()),
                    );
                }
            }
        }
        let mut files = std::collections::BTreeMap::new();
        visit(dir, dir, &mut files);
        files
    }

    #[test]
    fn metadata_failures_do_not_rotate_the_replacement_again() {
        let d = dir("replacement-archives");
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "kept").unwrap();
        let before = read(&log.path());
        for age in 1..=EARLIER_LOGS {
            std::fs::write(log_path(&d, age), format!("archive {age}\n")).unwrap();
        }
        let now = Instant::now();
        log.written = LOG_MAX_BYTES;
        let mut attempts = 0;
        for seconds in 0..=EARLIER_LOGS + 2 {
            assert!(log
                .rotate_with(now + Duration::from_secs(seconds as u64), |path| {
                    attempts += 1;
                    let _file = fs::append_private(path)?;
                    Err(io::Error::other("injected replacement metadata failure"))
                })
                .is_err());
            assert_eq!(attempts, seconds + 1);
            assert_eq!(read(&log_path(&d, 1)), before);
            for age in 2..=EARLIER_LOGS {
                assert_eq!(read(&log_path(&d, age)), format!("archive {}\n", age - 1));
            }
            assert!(read(&log.path()).is_empty());
        }
        log.rotate_with(now + Duration::from_secs(10), |path| {
            let file = fs::append_private(path)?;
            let written = file.metadata()?.len();
            Ok((file, written))
        })
        .unwrap();
        log.line(SystemTime::now(), "recovered").unwrap();
        let recovered = read(&log.path());
        log.written = LOG_MAX_BYTES;
        log.line_at(SystemTime::now(), "next", now + Duration::from_secs(11))
            .unwrap();
        assert_eq!(read(&log_path(&d, 1)), recovered);
        assert_eq!(read(&log_path(&d, 2)), before);
        assert_eq!(texts(&log.path()), ["next"]);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn reopening_or_metadata_failure_backs_off_the_whole_rotation() {
        for metadata in [false, true] {
            let d = dir(if metadata {
                "metadata-backoff"
            } else {
                "open-backoff"
            });
            let mut log = Log::open(&d).unwrap();
            log.line(SystemTime::now(), "kept").unwrap();
            let before = read(&log.path());
            log.written = LOG_MAX_BYTES - 128;
            let mut attempts = 0;
            let mut reopen = |path: &Path| -> io::Result<(File, u64)> {
                attempts += 1;
                if metadata {
                    let _file = fs::append_private(path)?;
                }
                Err(io::Error::other("injected replacement log failure"))
            };
            let now = Instant::now();
            assert!(log.rotate_with(now, &mut reopen).is_err());
            assert_eq!(read(&log_path(&d, 1)), before, "the rename succeeded");
            assert!(log
                .rotate_with(now + Duration::from_millis(999), &mut reopen)
                .is_err());
            assert_eq!(attempts, 1, "reopening was retried before the backoff");
            assert!(log
                .line_at(SystemTime::now(), "small", now + Duration::from_millis(999))
                .is_err());
            assert_eq!(
                read(&log_path(&d, 1)),
                before,
                "no append through the old handle"
            );
            log.rotate_with(now + Duration::from_secs(1), |path| {
                let file = fs::append_private(path)?;
                let len = file.metadata()?.len();
                Ok((file, len))
            })
            .unwrap();
            log.line(SystemTime::now(), "recovered").unwrap();
            assert_eq!(texts(&log.path()), ["recovered"]);
            drop(log);
            std::fs::remove_dir_all(d).unwrap();
        }
    }

    #[test]
    fn an_existing_log_counts_toward_the_limit_when_rotation_fails() {
        let d = dir("existing-size");
        std::fs::create_dir_all(log_path(&d, 1).join("occupied")).unwrap();
        std::fs::write(log_path(&d, 0), b"x existing\n").unwrap();
        let log = Log::open(&d).unwrap();
        assert_eq!(log.written, std::fs::metadata(log.path()).unwrap().len());
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    fn dir(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("cv1t-supervise-{}-{tag}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn read(path: &Path) -> String {
        std::fs::read_to_string(path).unwrap()
    }

    #[test]
    fn queued_lines_are_read_even_after_the_last_line_deadline() {
        let (tx, mut lines) = Lines::channel();
        for text in ["out\n", "err\n", "last"] {
            tx.try_send(text.to_string()).unwrap();
        }
        // Keep the sender open, as something the daemon started could still hold its pipes.
        let now = Instant::now();
        let ended = Some(now - LAST_LINES - Duration::from_secs(1));
        for text in ["out\n", "err\n", "last"] {
            assert_eq!(lines.next_line(ended, &|| now), Ok(text.to_string()));
        }
        assert_eq!(
            lines.next_line(ended, &|| now),
            Err(RecvTimeoutError::Timeout)
        );
        drop(tx);
    }

    #[test]
    fn new_output_cannot_hold_the_supervisor_past_the_last_line_deadline() {
        let (tx, mut lines) = Lines::channel();
        let other_reader = tx.clone();
        tx.try_send("out\n".to_string()).unwrap();
        other_reader.try_send("err\n".to_string()).unwrap();
        let now = Instant::now();
        let ended = Some(now - LAST_LINES);
        assert_eq!(lines.next_line(ended, &|| now), Ok("out\n".to_string()));
        // Both readers see the cutoff, whatever their scheduling order.
        let _ = tx.try_send("still writing\n".to_string());
        let _ = other_reader.try_send("also writing\n".to_string());
        assert_eq!(lines.next_line(ended, &|| now), Ok("err\n".to_string()));
        assert_eq!(
            lines.next_line(ended, &|| now),
            Err(RecvTimeoutError::Timeout)
        );
        drop(tx);
        drop(other_reader);
    }

    #[test]
    fn the_output_cutoff_starts_at_the_first_eligible_poll() {
        let (tx, mut lines) = Lines::channel();
        let at = Instant::now();
        tx.try_send("before".to_string()).unwrap();
        let before = at + LAST_LINES - Duration::from_nanos(1);
        assert_eq!(
            lines.next_line(Some(at), &|| before),
            Ok("before".to_string())
        );
        assert!(!lines.stop_readers.load(Ordering::Acquire));
        tx.try_send("last".to_string()).unwrap();
        assert_eq!(
            lines.next_line(Some(at), &|| at + LAST_LINES),
            Ok("last".to_string())
        );
        assert!(lines.stop_readers.load(Ordering::Acquire));
        assert_eq!(
            lines.next_line(Some(at), &|| at + LAST_LINES),
            Err(RecvTimeoutError::Timeout)
        );
    }

    #[test]
    fn the_cutoff_needs_no_room_in_a_full_bounded_output_queue() {
        let (tx, mut lines) = Lines::channel();
        for n in 0..LOG_QUEUE_LINES {
            tx.try_send(n.to_string()).unwrap();
        }
        assert!(matches!(
            tx.try_send("excess".into()),
            Err(TrySendError::Full(_))
        ));
        let ended = Instant::now();
        let clock = || ended + LAST_LINES;
        for n in 0..LOG_QUEUE_LINES {
            assert_eq!(lines.next_line(Some(ended), &clock).unwrap(), n.to_string());
        }
        assert_eq!(
            lines.next_line(Some(ended), &clock),
            Err(RecvTimeoutError::Timeout)
        );
        assert!(lines.cutoff);
        assert!(tx.try_send("after cutoff".into()).is_err());
    }

    #[test]
    fn the_readers_ending_disconnects_the_output_without_the_last_line_deadline() {
        let (tx, mut lines) = Lines::channel();
        tx.try_send("last".to_string()).unwrap();
        drop(tx);
        assert_eq!(lines.next_line(None, &Instant::now), Ok("last".to_string()));
        assert_eq!(
            lines.next_line(None, &Instant::now),
            Err(RecvTimeoutError::Disconnected)
        );
    }

    /// The text of each line, without its stamp.
    fn texts(path: &Path) -> Vec<String> {
        read(path)
            .lines()
            .map(|l| l.split_once(' ').unwrap().1.to_string())
            .collect()
    }

    #[test]
    fn each_start_begins_a_new_log_and_keeps_four_earlier_ones() {
        let d = dir("rotate");
        for n in 0..6 {
            let mut log = Log::open(&d).unwrap();
            log.line(SystemTime::now(), &format!("run {n}\n")).unwrap();
        }
        assert_eq!(texts(&log_path(&d, 0)), ["run 5"]);
        for age in 1..=4 {
            assert_eq!(texts(&log_path(&d, age)), [format!("run {}", 5 - age)]);
        }
        assert!(!log_path(&d, 5).exists());
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_full_log_is_set_aside() {
        let d = dir("full");
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "first").unwrap();
        log.written = LOG_MAX_BYTES;
        log.line(SystemTime::now(), "second").unwrap();
        assert_eq!(texts(&log_path(&d, 1)), ["first"]);
        assert_eq!(texts(&log_path(&d, 0)), ["second"]);
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    /// A sharing lock on the current log leaves all earlier logs where they were.
    #[test]
    fn earlier_logs_survive_a_current_log_that_cannot_be_moved() {
        let d = dir("stuck");
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "current").unwrap();
        for age in 1..=4 {
            std::fs::write(log_path(&d, age), format!("x run {age}\n")).unwrap();
        }
        assert!(log
            .rotation
            .set_aside_with(&d, |step| match &step {
                RotationStep::Rename(from, _) if *from == log_path(&d, 0) => {
                    Err(io::Error::new(
                        io::ErrorKind::PermissionDenied,
                        "injected current log lock",
                    ))
                }
                _ => step.run(),
            })
            .is_err());
        log.line(SystemTime::now(), "goes on").unwrap();
        assert_eq!(texts(&log_path(&d, 0)), ["current", "goes on"]);
        for age in 1..=4 {
            assert_eq!(texts(&log_path(&d, age)), [format!("run {age}")]);
        }
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    /// When the newest earlier log cannot make room (here directories stand in the earlier logs'
    /// places, so none moves), the current log stays where `doctor` and the next start look for
    /// it.
    #[test]
    fn a_log_that_cannot_be_set_aside_stays_the_current_one() {
        let d = dir("no-room");
        for age in 1..=4 {
            std::fs::create_dir_all(log_path(&d, age).join("occupied")).unwrap();
        }
        std::fs::write(log_path(&d, 0), "x current\n").unwrap();
        let mut log = Log::open(&d).unwrap();
        log.line(SystemTime::now(), "goes on").unwrap();
        let lines = texts(&log_path(&d, 0));
        assert_eq!(
            lines.len(),
            3,
            "a startup logging refusal needs a notice: {lines:?}"
        );
        assert_eq!(lines[0], "current");
        assert!(
            lines[1].contains(&log_path(&d, 1).display().to_string()),
            "{lines:?}"
        );
        assert!(lines[1].contains("Tracking isn't affected"), "{lines:?}");
        assert_eq!(lines[2], "goes on");
        assert!(!rotation_dir(&d).exists());
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn quick_failures_in_a_row_end_the_restarts() {
        let mut failures = Failures::new(POLICY);
        let s = Duration::from_secs;
        for _ in 0..4 {
            assert!(failures.failed(s(1)));
        }
        // a run of a while starts the count again
        assert!(failures.failed(s(60)));
        for _ in 0..4 {
            assert!(failures.failed(s(29)));
        }
        assert!(!failures.failed(s(29)));
    }

    /// The shell running `script` (sh on Linux, cmd on Windows).
    fn shell(script: &str) -> Command {
        #[cfg(unix)]
        let mut c = {
            let mut c = Command::new("sh");
            c.arg("-c");
            c
        };
        #[cfg(windows)]
        let mut c = {
            let mut c = Command::new("cmd");
            c.arg("/C");
            c
        };
        c.arg(script);
        c
    }

    /// Never asked to stop.
    fn never() -> Stopping<'static> {
        Stopping {
            requested: Box::new(|| None),
            ask: Box::new(|_| Ok(())),
            within: Duration::from_secs(10),
        }
    }

    /// The file whose appearance the stand-in daemon waits for.
    const STAND_IN: &str = "CV1TOUCH_TEST_STAND_IN";

    /// Stands in for a daemon in other tests, which run this test binary for it: it ends cleanly
    /// once the file named by CV1TOUCH_TEST_STAND_IN exists. Its ready file lets a stop request
    /// wait until it is running; the supervisor ends it if it never stops.
    #[test]
    #[ignore = "run by other tests"]
    fn stand_in_daemon() {
        let Some(file) = std::env::var_os(STAND_IN) else {
            return;
        };
        if std::env::var_os("CV1TOUCH_TEST_FLOOD_OUTPUT").is_some() {
            let mut output = std::io::stdout().lock();
            for _ in 0..32 {
                output.write_all(&[b'x'; 64 * 1024]).unwrap();
            }
            output.write_all(b"\n").unwrap();
            std::fs::write(&file, b"").unwrap();
            return;
        }
        std::fs::write(Path::new(&file).with_extension("ready"), b"").unwrap();
        while !Path::new(&file).exists() {
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    fn stand_in(file: &Path) -> Command {
        let mut c = Command::new(std::env::current_exe().unwrap());
        c.args(["--exact", "supervise::tests::stand_in_daemon", "--ignored"])
            .env(STAND_IN, file);
        c
    }

    /// A stop request after `after`, and how many times the daemon was asked to stop.
    fn stop_after(
        after: Duration,
        ask: impl FnMut() + 'static,
        within: Duration,
    ) -> Stopping<'static> {
        let at = Instant::now() + after;
        let mut ask = ask;
        let mut sent = false;
        Stopping {
            requested: Box::new(move || {
                (!sent && Instant::now() >= at).then(|| {
                    sent = true;
                    "Ctrl+C".to_string()
                })
            }),
            ask: Box::new(move |_| {
                ask();
                Ok(())
            }),
            within,
        }
    }

    #[test]
    fn asked_to_stop_the_supervisor_asks_the_daemon_and_waits_for_it() {
        let d = dir("asked");
        std::fs::create_dir_all(&d).unwrap();
        let mut log = Writer::open(&d);
        let file = d.join("stop");
        let asked = file.clone();
        let mut stopping = stop_after(
            Duration::from_millis(200),
            move || std::fs::write(&asked, b"").unwrap(),
            Duration::from_secs(20),
        );
        let code = run(|| stand_in(&file), &mut log, quick(10, 5), &mut stopping);
        assert_eq!(code, 0);
        let lines = texts(log.path());
        assert!(
            lines.contains(&"cv1touchd supervisor: Ctrl+C: asking the daemon to stop".into()),
            "{lines:?}"
        );
        assert!(lines
            .last()
            .unwrap()
            .starts_with("cv1touchd supervisor: the daemon stopped"));
        assert!(!lines.iter().any(|l| l.contains("ending it")), "{lines:?}");
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    /// A daemon that does not take requests yet (it is just starting) is asked again.
    #[test]
    fn a_daemon_not_yet_taking_requests_is_asked_again() {
        let d = dir("not-yet");
        std::fs::create_dir_all(&d).unwrap();
        let mut log = Writer::open(&d);
        let file = d.join("stop");
        let asked = file.clone();
        let mut tries = 0;
        let at = Instant::now() + Duration::from_millis(100);
        let mut stopping = Stopping {
            requested: Box::new(move || (Instant::now() >= at).then(|| "Ctrl+C".to_string())),
            ask: Box::new(move |_| {
                tries += 1;
                if tries < 3 {
                    return Err("no daemon there yet".into());
                }
                std::fs::write(&asked, b"").map_err(|e| e.to_string())
            }),
            within: Duration::from_secs(20),
        };
        let code = run(|| stand_in(&file), &mut log, quick(10, 5), &mut stopping);
        assert_eq!(code, 0);
        let lines = texts(log.path());
        let not_yet = "cv1touchd supervisor: cannot ask it yet (no daemon there yet); asking again";
        assert_eq!(
            lines.iter().filter(|l| *l == not_yet).count(),
            1,
            "{lines:?}"
        );
        assert!(!lines.iter().any(|l| l.contains("ending it")), "{lines:?}");
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_daemon_that_does_not_stop_when_asked_is_ended() {
        let d = dir("deaf");
        std::fs::create_dir_all(&d).unwrap();
        let mut log = Writer::open(&d);
        let file = d.join("never");
        let mut stopping = stop_when_ready(&file, || Ok(()), Duration::from_secs(10));
        let code = run(|| stand_in(&file), &mut log, quick(10, 5), &mut stopping);
        assert_eq!(code, 0);
        let lines = texts(log.path());
        assert!(
            lines.contains(
                &"cv1touchd supervisor: the daemon did not stop within 10 s; ending it".into()
            ),
            "{lines:?}"
        );
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    /// A request once the stand-in is running, and the time it has to stop after accepting it.
    fn stop_when_ready<'a>(
        file: &Path,
        mut ask: impl FnMut() -> Result<(), String> + 'a,
        within: Duration,
    ) -> Stopping<'a> {
        let ready = file.with_extension("ready");
        Stopping {
            requested: Box::new(move || ready.exists().then(|| "Ctrl+C".to_string())),
            ask: Box::new(move |_| ask()),
            within,
        }
    }

    #[test]
    fn an_accepted_stop_gets_the_full_grace_and_ends_at_the_first_eligible_poll() {
        let at = Instant::now();
        let mut stop = Stop::new(Duration::from_secs(10));
        assert_eq!(stop.action(at), StopAction::Wait);
        stop.request(at);
        assert_eq!(
            stop.action(at),
            StopAction::Deliver(at + Duration::from_secs(10))
        );
        // Sending takes two seconds. The daemon then gets ten seconds, not eight or twenty.
        stop.delivered(at + Duration::from_secs(2), true);
        assert_eq!(
            stop.action(at + Duration::from_secs(12) - Duration::from_nanos(1)),
            StopAction::Wait
        );
        assert_eq!(stop.action(at + Duration::from_secs(12)), StopAction::End);
        assert_eq!(stop.action(at + Duration::from_secs(40)), StopAction::End);
    }

    #[test]
    fn failed_stop_deliveries_share_one_deadline_and_do_not_run_past_it() {
        let at = Instant::now();
        let mut stop = Stop::new(Duration::from_secs(10));
        stop.request(at);
        for seconds in [0, 1, 3, 8] {
            let now = at + Duration::from_secs(seconds);
            assert_eq!(
                stop.action(now),
                StopAction::Deliver(at + Duration::from_secs(10))
            );
            stop.delivered(now, false);
            assert_eq!(
                stop.action(now + Duration::from_millis(249)),
                StopAction::Wait
            );
            assert_eq!(
                stop.action(now + Duration::from_millis(250)),
                StopAction::Deliver(at + Duration::from_secs(10))
            );
        }
        assert_eq!(stop.action(at + Duration::from_secs(10)), StopAction::End);
        assert_eq!(stop.action(at + Duration::from_secs(40)), StopAction::End);
    }

    #[test]
    fn a_delivery_finishing_at_or_after_the_deadline_cannot_extend_it() {
        let at = Instant::now();
        for seconds in [10, 11] {
            let mut stop = Stop::new(Duration::from_secs(10));
            stop.request(at);
            assert_eq!(
                stop.action(at),
                StopAction::Deliver(at + Duration::from_secs(10))
            );
            let finished = at + Duration::from_secs(seconds);
            stop.delivered(finished, true);
            assert_eq!(stop.action(finished), StopAction::End);
        }
    }

    /// A stop request once the log at `path` holds `text`, however long the daemon took to end.
    fn stop_once_logged(path: PathBuf, text: &'static str) -> Stopping<'static> {
        let mut sent = false;
        Stopping {
            requested: Box::new(move || {
                (!sent && std::fs::read_to_string(&path).is_ok_and(|line| line.contains(text)))
                    .then(|| {
                        sent = true;
                        "Ctrl+C".to_string()
                    })
            }),
            ask: Box::new(|_| Ok(())),
            within: Duration::from_secs(10),
        }
    }

    #[test]
    fn a_stop_while_waiting_to_start_the_daemon_again_ends_the_supervisor() {
        let d = dir("waiting");
        std::fs::create_dir_all(&d).unwrap();
        let mut log = Writer::open(&d);
        // the stop comes once the supervisor waits to start the daemon again: a stop after a
        // fixed time came while the daemon still ran on a loaded machine, which stops it instead
        let mut stopping = stop_once_logged(log.path().to_owned(), "starting it again in");
        let started = Instant::now();
        let code = run(
            || shell("exit 1"),
            &mut log,
            quick(20_000, 5),
            &mut stopping,
        );
        assert_eq!(code, 0);
        assert!(started.elapsed() < Duration::from_secs(10));
        assert_eq!(
            texts(log.path()).last().unwrap(),
            "cv1touchd supervisor: Ctrl+C: not starting the daemon again"
        );
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    fn quick(delay_ms: u64, quick_failures: usize) -> Policy {
        Policy {
            delay: Duration::from_millis(delay_ms),
            quick: Duration::from_secs(60),
            quick_failures,
        }
    }

    #[test]
    fn a_failing_daemon_runs_again_after_the_delay_until_the_limit() {
        let d = dir("fails");
        let mut log = Writer::open(&d);
        let mut runs = 0;
        let started = Instant::now();
        let script = if cfg!(windows) {
            "echo out && echo err 1>&2 && <nul set /p =last && exit 3"
        } else {
            "echo out && echo err 1>&2 && printf last && exit 3"
        };
        let at = Instant::now();
        // Hold the drain clock still until both pipe readers reach EOF. Their scheduling
        // cannot discard output; separate clock-controlled tests exercise the cutoff.
        let code = run_with_clock(
            || {
                runs += 1;
                shell(script)
            },
            &mut log,
            quick(150, 3),
            &mut never(),
            || at,
        );
        assert_eq!((code, runs), (1, 3));
        // two waits between three runs
        assert!(started.elapsed() >= Duration::from_millis(300));
        let lines = texts(log.path());
        let ended = lines
            .iter()
            .filter(|l| l.starts_with("cv1touchd supervisor: the daemon ended"))
            .count();
        assert_eq!(ended, 3, "{lines:?}");
        for text in ["out", "err", "last"] {
            let n = lines.iter().filter(|l| l.trim_end() == text).count();
            assert_eq!(n, 3, "{text}: {lines:?}");
        }
        assert_eq!(
            lines.last().unwrap(),
            "cv1touchd supervisor: the daemon failed 3 times in a row within 60 s of starting; \
             not starting it again"
        );
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_clean_stop_ends_the_supervisor() {
        let d = dir("stops");
        let mut log = Writer::open(&d);
        let mut runs = 0;
        let code = run(
            || {
                runs += 1;
                shell(if runs == 1 { "exit 1" } else { "exit 0" })
            },
            &mut log,
            quick(10, 5),
            &mut never(),
        );
        assert_eq!((code, runs), (0, 2));
        assert!(texts(log.path())
            .last()
            .unwrap()
            .starts_with("cv1touchd supervisor: the daemon stopped"));
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn arguments_the_daemon_refuses_end_the_supervisor() {
        let d = dir("usage");
        let mut log = Writer::open(&d);
        let mut runs = 0;
        let code = run(
            || {
                runs += 1;
                shell("exit 2")
            },
            &mut log,
            quick(10, 5),
            &mut never(),
        );
        assert_eq!((code, runs), (1, 1));
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_daemon_that_cannot_start_is_tried_again() {
        let d = dir("missing");
        let mut log = Writer::open(&d);
        let code = run(
            || Command::new(d.join("no-such-daemon")),
            &mut log,
            quick(10, 2),
            &mut never(),
        );
        assert_eq!(code, 1);
        let lines = texts(log.path());
        let tries = lines
            .iter()
            .filter(|l| l.starts_with("cv1touchd supervisor: cannot start the daemon"))
            .count();
        assert_eq!(tries, 2, "{lines:?}");
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }

    /// Something the daemon started that still holds its output does not keep the supervisor
    /// from seeing the daemon end.
    #[test]
    fn the_daemon_ending_is_seen_while_its_output_is_still_held() {
        let d = dir("held");
        let mut log = Writer::open(&d);
        let script = if cfg!(windows) {
            "start /b ping -n 30 127.0.0.1 && exit 0"
        } else {
            "sleep 30 & exit 0"
        };
        let started = Instant::now();
        assert_eq!(
            run(|| shell(script), &mut log, quick(10, 5), &mut never()),
            0
        );
        assert!(started.elapsed() < Duration::from_secs(10));
        drop(log);
        std::fs::remove_dir_all(d).unwrap();
    }
}
