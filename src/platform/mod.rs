// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! What differs between Linux and Windows, behind one interface: the rest of the crate calls these
//! and never the operating system's own APIs directly.

#[cfg(unix)]
mod unix;
#[cfg(unix)]
use unix as os;
#[cfg(windows)]
mod windows;
#[cfg(windows)]
use windows as os;

pub mod fs {
    //! Files that hold the person's calibration and setup: never followed through links, private
    //! to them, and replaced atomically.
    use std::fs::File;
    use std::io;
    use std::path::Path;

    use super::os;

    /// Opens `path` for reading without following a link at its end and without blocking on a
    /// special file; the caller checks it is a regular file.
    pub fn open_read_nofollow(path: &Path) -> io::Result<File> {
        os::open_read_nofollow(path)
    }

    /// Opens `path` for reading without blocking on a special file (links are followed).
    pub fn open_read_nonblocking(path: &Path) -> io::Result<File> {
        os::open_read_nonblocking(path)
    }

    /// Creates `path`, which must not exist yet, readable and writable by this user only.
    pub fn create_private(path: &Path) -> io::Result<File> {
        os::create_private(path)
    }

    /// Creates `path` and its parents, new directories private to this user.
    pub fn create_private_dir_all(path: &Path) -> io::Result<()> {
        os::create_private_dir_all(path)
    }

    /// Opens `path` for appending, creating it readable and writable by this user only; a link
    /// at its end is not followed.
    pub fn append_private(path: &Path) -> io::Result<File> {
        os::append_private(path)
    }

    /// Flushes the file at `path` to disk.
    pub fn sync_file(path: &Path) -> io::Result<()> {
        os::sync_file(path)
    }

    /// Makes a rename into `dir` durable, where the platform can.
    pub fn sync_dir(dir: &Path) -> io::Result<()> {
        os::sync_dir(dir)
    }

    /// Renames `from` over `to`, replacing it.
    pub fn rename_replacing(from: &Path, to: &Path) -> io::Result<()> {
        os::rename_replacing(from, to)
    }

    /// Puts directory `stage` in place of `dir`. With `exists`, the two swap, so `stage` then holds
    /// the old contents; without, `dir` must not exist. Atomic where the platform allows it.
    pub fn swap_dir(stage: &Path, dir: &Path, exists: bool) -> io::Result<()> {
        os::swap_dir(stage, dir, exists)
    }
}

pub mod dylib {
    //! Libraries loaded at run time (the OpenVR client from the SteamVR installation).
    use std::ffi::{c_void, CStr};
    use std::path::Path;

    use super::os;

    /// A loaded library, unloaded when dropped.
    pub struct Library(*mut c_void);

    // SAFETY: a library handle is process-wide; loading and lookups are thread-safe on both
    // platforms.
    unsafe impl Send for Library {}

    impl Library {
        /// Loads the library at `path`, or by `path` as a bare name from the system's search path.
        pub fn open(path: &Path) -> Option<Library> {
            let handle = os::dylib_open(path);
            (!handle.is_null()).then(|| Library(handle))
        }

        /// A handle to nothing, for tests that build a client without a library; never drop it.
        #[cfg(all(test, unix))]
        pub(crate) fn null() -> Library {
            Library(std::ptr::null_mut())
        }

        /// The address of `name`, or null.
        pub fn symbol(&self, name: &CStr) -> *mut c_void {
            os::dylib_symbol(self.0, name)
        }
    }

    impl Drop for Library {
        fn drop(&mut self) {
            os::dylib_close(self.0);
        }
    }
}

/// The monotonic clock every cv1touch timestamp uses, in nanoseconds, shared by all processes:
/// CLOCK_MONOTONIC on Linux, the performance counter on Windows.
pub fn now_ns() -> u64 {
    os::now_ns()
}

/// The named pipe for a control endpoint path (UTF-16, no NUL): `\\.\pipe\cv1touch-` and the path
/// with `\` written as `/`, since a pipe name may hold any character but `\`. None if it would pass
/// the 256-character limit. The SteamVR driver builds the same name (driver/src/haptics.cpp).
#[cfg_attr(unix, allow(dead_code))]
pub(crate) fn pipe_name(path: &[u16]) -> Option<Vec<u16>> {
    let mut name: Vec<u16> = r"\\.\pipe\cv1touch-".encode_utf16().collect();
    name.extend(path.iter().map(|&c| {
        if c == u16::from(b'\\') {
            u16::from(b'/')
        } else {
            c
        }
    }));
    (name.len() <= 256).then_some(name)
}

/// `ticks` of a counter running at `frequency` per second, in nanoseconds, without overflow.
pub fn ticks_to_ns(ticks: u64, frequency: u64) -> u64 {
    (ticks as u128 * 1_000_000_000 / frequency.max(1) as u128) as u64
}

pub mod shm {
    //! Named shared memory between the daemon (one writer, owning the name) and its readers, with
    //! a wake-up for readers waiting on a counter inside it.
    use std::io;

    use super::os;

    /// A segment this process created and owns; gone when dropped.
    pub struct OwnedSegment(os::OwnedSegment);

    impl OwnedSegment {
        /// Takes `name` for this process (one owner per name at a time), replaces anything left
        /// from an earlier owner, and maps `len` zeroed bytes for reading and writing. `name` is
        /// "/" and one more component, as for POSIX shared memory.
        pub fn create(name: &str, len: usize) -> io::Result<OwnedSegment> {
            os::OwnedSegment::create(name, len).map(OwnedSegment)
        }

        pub fn ptr(&self) -> *mut u8 {
            self.0.ptr()
        }

        /// Wakes the readers waiting on the counter at `word` (inside the segment); call after
        /// changing it.
        pub fn wake(&self, word: *mut u32) {
            self.0.wake(word)
        }

        /// Removes the name, once, so that new readers find nothing; the mapping stays until
        /// drop, which removes the name too. On Windows a name lasts while any process holds the
        /// segment, so there it goes when the readers let go.
        pub fn unlink(&self) {
            self.0.unlink()
        }
    }

    #[derive(Debug)]
    pub enum OpenError {
        /// Nobody owns the name (the daemon is not running).
        NotFound,
        /// The segment is smaller than expected (being created, or another layout).
        TooSmall,
        Io(io::Error),
    }

    /// Another process's segment, mapped read-only.
    pub struct OpenedSegment(os::OpenedSegment);

    impl OpenedSegment {
        pub fn open(name: &str, len: usize) -> Result<OpenedSegment, OpenError> {
            os::OpenedSegment::open(name, len).map(OpenedSegment)
        }

        /// The mapping; only read through it.
        pub fn ptr(&self) -> *mut u8 {
            self.0.ptr()
        }

        /// Waits until the counter at `word` (inside the segment) differs from `last_seen`, a
        /// wake-up arrives, or `timeout_ns` passes.
        pub fn wait(&self, word: *mut u32, last_seen: u32, timeout_ns: u64) {
            self.0.wait(word, last_seen, timeout_ns)
        }
    }
}

pub mod dirs {
    //! Per-user directories. Linux follows the XDG base directories (an empty or relative value is
    //! ignored, as the specification says); Windows keeps everything under %LOCALAPPDATA%, where
    //! SteamVR keeps its openvrpaths and the engine its cache.
    use std::ffi::OsString;
    use std::path::PathBuf;

    #[derive(Debug, PartialEq)]
    pub struct Dirs {
        /// Other programs' per-user configuration (OpenHMD's cache, SteamVR's openvrpaths).
        pub config_home: PathBuf,
        /// cv1touch's configuration: calibration, registration, render offsets, input calibration.
        pub config: PathBuf,
        /// cv1touch's state: IMU biases, recordings, setup and pairing records.
        pub state: PathBuf,
        /// cv1touch's data: the installed SteamVR driver.
        pub data: PathBuf,
    }

    /// The directories for an environment (`var` looks a variable up).
    #[cfg(unix)]
    pub fn resolve(var: &dyn Fn(&str) -> Option<OsString>) -> Dirs {
        let home = PathBuf::from(var("HOME").unwrap_or_default());
        let xdg = |name: &str, fallback: &str| {
            var(name)
                .map(PathBuf::from)
                .filter(|p| p.is_absolute())
                .unwrap_or_else(|| home.join(fallback))
        };
        let config_home = xdg("XDG_CONFIG_HOME", ".config");
        Dirs {
            config: config_home.join("cv1touch"),
            state: xdg("XDG_STATE_HOME", ".local/state").join("cv1touch"),
            data: xdg("XDG_DATA_HOME", ".local/share").join("cv1touch"),
            config_home,
        }
    }

    /// The directories for an environment (`var` looks a variable up). The SteamVR driver finds
    /// the daemon under the same %LOCALAPPDATA%\cv1touch (driver/src/haptics.cpp).
    #[cfg(windows)]
    pub fn resolve(var: &dyn Fn(&str) -> Option<OsString>) -> Dirs {
        let local = var("LOCALAPPDATA")
            .map(PathBuf::from)
            .filter(|p| p.is_absolute())
            .unwrap_or_else(std::env::temp_dir);
        let cv1touch = local.join("cv1touch");
        Dirs {
            config: cv1touch.clone(),
            state: cv1touch.join("state"),
            data: cv1touch.join("data"),
            config_home: local,
        }
    }

    pub fn get() -> Dirs {
        resolve(&|name| std::env::var_os(name))
    }

    /// The user's desktop, where Windows users look for a file they were asked to send; None on
    /// Linux, which has no such convention for a command-line tool.
    pub fn desktop() -> Option<PathBuf> {
        super::os::desktop()
    }

    /// OpenHMD's cache of sensor and controller configuration, as the engine resolves it
    /// (ohmd_get_config_dir).
    pub fn openhmd() -> PathBuf {
        get().config_home.join("openhmd")
    }
}

pub mod hid {
    //! The headset's HID interfaces (Rift CV1 2833:0031): input reports, one per read with the
    //! report ID first. hidraw on Linux, the HID class driver on Windows; both let the engine and
    //! this process each read every report.
    use std::io;
    use std::path::{Path, PathBuf};
    use std::time::Duration;

    use super::os;

    /// Interrupts a `Device::wait` from another thread.
    pub struct Waker(pub(super) os::Waker);

    impl Waker {
        pub fn new() -> io::Result<Waker> {
            os::Waker::new().map(Waker)
        }

        pub fn wake(&self) {
            self.0.wake()
        }
    }

    /// What ended a wait (both may be false: the timeout).
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct Woke {
        pub report: bool,
        pub waker: bool,
    }

    pub struct Device(pub(super) os::HidDevice);

    impl Device {
        /// Opens an interface for reading. Reads never block.
        pub fn open(path: &Path) -> io::Result<Device> {
            os::HidDevice::open(path).map(Device)
        }

        /// Waits until a report is ready, `waker` fires, or `timeout` (None: no limit) passes.
        pub fn wait(&mut self, waker: &Waker, timeout: Option<Duration>) -> io::Result<Woke> {
            self.0.wait(&waker.0, timeout)
        }

        /// The next report already received into `buf` (its length), or None when none is
        /// waiting. An error means the device is gone.
        pub fn try_read(&mut self, buf: &mut [u8]) -> io::Result<Option<usize>> {
            self.0.try_read(buf)
        }
    }

    /// The headset's HID interface `interface`, if connected: 0 carries the headset's own
    /// reports and the radio commands, 1 the radio's messages.
    pub fn headset_interface(interface: u8) -> Option<PathBuf> {
        os::headset_interface(interface)
    }

    /// Whether a Windows HID device path is the headset's interface `interface`
    /// (\\?\hid#vid_2833&pid_0031&mi_01#...).
    #[cfg_attr(unix, allow(dead_code))]
    pub(crate) fn is_headset_path(path: &str, interface: u8) -> bool {
        let path = path.to_ascii_lowercase();
        path.contains("vid_2833&pid_0031") && path.contains(&format!("&mi_{interface:02}"))
    }
}

pub mod system {
    //! The daemon's hold on the machine: its threads' priority and CPUs, its memory, and its
    //! power setting. On Linux: SCHED_RR through rtkit, sched_setaffinity and mlockall. On
    //! Windows: thread priorities and affinity masks, opting out of power throttling, and no
    //! memory locking.
    use std::io;

    use super::os;

    /// An operating system thread of this process.
    pub type ThreadId = os::ThreadId;

    /// The calling thread.
    pub fn current_thread() -> ThreadId {
        os::current_thread()
    }

    /// The thread of this process with this name, if one is running.
    pub fn thread_named(name: &str) -> Option<ThreadId> {
        os::thread_named(name)
    }

    /// Runs `thread` ahead of normal threads, so the report or transfer it waits on wakes it at
    /// once. On Linux: SCHED_RR at `priority` through rtkit (no privileges needed; threads it
    /// starts afterwards stay normal), ahead of every normal thread. On Windows: for `priority`
    /// 10 and up the time-critical priority, the top of the normal range, which a normal thread
    /// only ties during a passing boost; below, the highest normal one, which a boosted normal
    /// thread can pass. Says what it set, for the log.
    pub fn make_realtime(thread: ThreadId, priority: u32) -> Result<String, String> {
        os::make_realtime(thread, priority)
    }

    /// The CPUs `thread` may run on (on Windows, within processor group 0: the first 64).
    pub fn allowed_cpus(thread: ThreadId) -> io::Result<Vec<usize>> {
        os::allowed_cpus(thread)
    }

    /// Restricts `thread` to `cpus`, which must be among its allowed ones.
    pub fn set_cpus(thread: ThreadId, cpus: &[usize]) -> io::Result<()> {
        os::set_cpus(thread, cpus)
    }

    /// Keeps the process's pages in RAM once touched, where the platform can: a page swapped out
    /// costs a disk read on whichever thread touches it next. Only touched pages, not the whole
    /// address space (the mapped libraries reserve far more than they use).
    pub fn lock_memory() -> io::Result<()> {
        os::lock_memory()
    }

    /// The calling thread's voluntary context switches (it blocked or slept), where the platform
    /// counts them.
    pub fn voluntary_switches() -> Option<i64> {
        os::voluntary_switches()
    }

    /// The operating system and its version, for a report: "Arch Linux, Linux 7.2.9-arch1-1
    /// x86_64", "Windows 11 Professional 24H2 (build 26100.6584)".
    pub fn description() -> String {
        os::system_description()
    }

    /// 64 random bits from the operating system.
    pub fn random_u64() -> io::Result<u64> {
        os::random_u64()
    }

    /// Keeps Windows from running the process in its power-saving efficiency mode, which slows
    /// a background process's threads; nothing to do on Linux.
    pub fn opt_out_of_throttling() -> io::Result<()> {
        os::opt_out_of_throttling()
    }
}

pub mod console {
    //! The terminal a command-line tool talks to. Lines typed at it are read on a thread of their
    //! own, so that waiting for one ends at a timeout or an interrupt: on Windows a console's
    //! input is ready at the first key, and a read then blocks until Enter. Interrupts set a flag
    //! instead of ending the process: SIGINT, SIGTERM and SIGHUP on Linux; Ctrl+C, Ctrl+Break and
    //! closing the console on Windows.
    use std::fs::File;
    use std::io::{self, BufRead, IsTerminal};
    use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
    use std::sync::{Mutex, OnceLock};
    use std::time::Duration;

    use super::os;

    /// What a wait for a typed line found.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum Line {
        Typed(String),
        /// The input ended (or failed). From a pipe or a file no line comes after it; at a
        /// terminal it was Ctrl+D (Ctrl+Z on Windows), and typing goes on.
        Closed,
        NotYet,
    }

    /// Lines read from a source on a thread of their own.
    pub struct Lines {
        lines: Mutex<Receiver<Line>>,
        /// Lets the reader go on once an end of input it reported has been taken.
        resume: Sender<()>,
    }

    impl Lines {
        /// With `terminal`, an end of input ends only the read it interrupts, as Ctrl+D does at a
        /// terminal; the reader reads on once the end has been taken (so an input that keeps
        /// ending, a terminal that hung up, is read once per wait, not in a loop).
        pub fn spawn(mut source: impl BufRead + Send + 'static, terminal: bool) -> Lines {
            let (tx, rx) = mpsc::channel();
            let (resume, resumed) = mpsc::channel();
            std::thread::Builder::new()
                .name("cv1t-input".into())
                .spawn(move || loop {
                    let mut line = Vec::new();
                    match source.read_until(b'\n', &mut line) {
                        // an end of input, or a failed read (a character the console cannot
                        // give): at a terminal it ends this wait only
                        Ok(0) | Err(_) if terminal => {
                            if tx.send(Line::Closed).is_err() || resumed.recv().is_err() {
                                break;
                            }
                        }
                        // dropping the sender: every wait from now on reads as closed
                        Ok(0) | Err(_) => break,
                        Ok(_) => {
                            let line = String::from_utf8_lossy(&line).into_owned();
                            if tx.send(Line::Typed(line)).is_err() {
                                break;
                            }
                        }
                    }
                })
                .expect("start the input reader");
            Lines {
                lines: Mutex::new(rx),
                resume,
            }
        }

        /// The next line, waiting up to `timeout` for it.
        pub fn next(&self, timeout: Duration) -> Line {
            match self.lines.lock().unwrap().recv_timeout(timeout) {
                Ok(Line::Closed) => {
                    let _ = self.resume.send(());
                    Line::Closed
                }
                Ok(line) => line,
                Err(RecvTimeoutError::Timeout) => Line::NotYet,
                Err(RecvTimeoutError::Disconnected) => Line::Closed,
            }
        }

        /// Drops the lines read and not yet taken.
        pub fn discard(&self) {
            let lines = self.lines.lock().unwrap();
            while let Ok(line) = lines.try_recv() {
                if line == Line::Closed {
                    let _ = self.resume.send(());
                }
            }
        }
    }

    fn stdin() -> &'static Lines {
        static LINES: OnceLock<Lines> = OnceLock::new();
        LINES.get_or_init(|| {
            Lines::spawn(io::BufReader::new(io::stdin()), io::stdin().is_terminal())
        })
    }

    /// The next line typed at the terminal (or read from standard input), waiting up to
    /// `timeout`. All reads of standard input go through here once one has.
    pub fn next_line(timeout: Duration) -> Line {
        stdin().next(timeout)
    }

    /// The next line typed, however long it takes; None once the input ends or, with
    /// interrupts caught, at an interrupt.
    pub fn wait_line() -> Option<String> {
        loop {
            match next_line(Duration::from_millis(200)) {
                Line::Typed(line) => return Some(line),
                Line::Closed => return None,
                Line::NotYet if interrupted() => return None,
                Line::NotYet => {}
            }
        }
    }

    /// Drops what was typed before a prompt, the lines read and the unfinished one the terminal
    /// still holds, so that an Enter pressed during a capture cannot answer the next question.
    pub fn discard_typed_ahead() {
        // the terminal first: a line the reader takes from it meanwhile is then dropped too
        os::flush_console_input();
        stdin().discard();
    }

    /// From now on interrupts set the flag `interrupted` reads instead of ending the process.
    pub fn catch_interrupts() {
        os::catch_interrupts();
    }

    pub fn interrupted() -> bool {
        os::interrupted()
    }

    /// Whether this process is the only one at its console, so that Windows closes the window
    /// when it ends: a program started from Explorer or as an administrator by `run_elevated`
    /// gets a console of its own, whose output vanishes with it. On Linux the terminal stays:
    /// false.
    pub fn closes_with_us() -> bool {
        os::console_closes_with_us()
    }

    /// Sends standard output and error to `to` until the guard is dropped, which puts them back.
    /// On Windows only the process's standard handles move: this process's Rust output and the
    /// programs it starts follow them, but a loaded library's C runtime writes where it did.
    pub fn redirect_output(to: &File) -> io::Result<OutputRedirect> {
        os::OutputRedirect::to(to).map(|guard| OutputRedirect { _guard: guard })
    }

    /// Output going to a file; the guard holds its own handle to it.
    pub struct OutputRedirect {
        _guard: os::OutputRedirect,
    }
}

pub mod shutdown {
    //! Requests to stop the daemon: SIGINT and SIGTERM on Linux; on Windows the console's
    //! control events: Ctrl+C, Ctrl+Break, closing the console, and logoff and shutdown, which
    //! Windows sends only to a console process that has not loaded user32.
    use std::io;
    use std::time::Duration;

    use super::os;

    pub struct Requests(os::ShutdownRequests);

    impl Requests {
        /// Takes the requests over from their default (ending the process at once). On Linux,
        /// call it before starting threads: they inherit the blocked signals. On Windows the
        /// default is back once this is dropped.
        pub fn take() -> io::Result<Requests> {
            os::ShutdownRequests::take().map(Requests)
        }

        /// Waits up to `timeout` for a request; what it was ("signal 15", "Ctrl+C").
        pub fn wait(&self, timeout: Duration) -> Option<String> {
            self.0.wait(timeout)
        }
    }
}

pub mod ipc {
    //! The control channel: one stream per request, private to this user (a Unix-domain socket on
    //! Linux, a named pipe on Windows), and the lock that keeps one daemon per user.
    use std::fs::File;
    use std::io::{self, Read, Write};
    use std::path::{Path, PathBuf};
    use std::time::{Duration, Instant};

    use super::os;

    /// Where this user's daemon keeps its control endpoint and locks.
    pub fn runtime_dir() -> PathBuf {
        os::runtime_dir()
    }

    /// A second Linux control socket beside the poses, visible inside Steam's container.
    #[cfg(unix)]
    pub fn container_socket_path() -> PathBuf {
        os::container_socket_path()
    }

    /// Removes only this user's private socket. Hold the daemon's instance lock until after this.
    #[cfg(unix)]
    pub fn unlink_private_socket(endpoint: &Path) -> io::Result<()> {
        os::unlink_private_socket(endpoint)
    }

    /// An exclusive lock on `path` (created if missing), held until the file is dropped; fails at
    /// once (WouldBlock) while another process holds it.
    pub fn lock_exclusive(path: &Path) -> io::Result<File> {
        os::lock_exclusive(path)
    }

    pub(super) fn remaining(until: Instant) -> io::Result<Duration> {
        until
            .checked_duration_since(Instant::now())
            .filter(|left| !left.is_zero())
            .ok_or_else(|| io::Error::from(io::ErrorKind::TimedOut))
    }

    pub struct Connection(pub(super) os::Connection, Option<Instant>);

    impl Connection {
        /// Connects to the listener at `endpoint`; NotFound when nobody listens there, and
        /// PermissionDenied when another account does.
        pub fn connect(endpoint: &Path) -> io::Result<Connection> {
            os::Connection::connect(endpoint).map(|stream| Connection(stream, None))
        }

        /// The same deadline bounds connecting and every subsequent read and write.
        pub fn connect_until(endpoint: &Path, until: Instant) -> io::Result<Connection> {
            remaining(until)?;
            let stream = os::Connection::connect_until(endpoint, until)?;
            remaining(until)?;
            Ok(Connection(stream, Some(until)))
        }

        /// How long a read or a write may wait (None: as long as it takes).
        pub fn set_timeout(&mut self, timeout: Option<Duration>) -> io::Result<()> {
            self.0.set_timeout(timeout)
        }

        fn prepare(&mut self) -> io::Result<()> {
            if let Some(until) = self.1 {
                self.0.set_timeout(Some(remaining(until)?))?;
            }
            Ok(())
        }

        fn completed(&self, result: io::Result<usize>) -> io::Result<usize> {
            if let Some(until) = self.1 {
                remaining(until)?;
                if result
                    .as_ref()
                    .is_err_and(|e| e.kind() == io::ErrorKind::WouldBlock)
                {
                    return Err(io::Error::from(io::ErrorKind::TimedOut));
                }
            }
            result
        }
    }

    impl Read for Connection {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            self.prepare()?;
            let result = self.0.read(buf);
            self.completed(result)
        }
    }

    impl Write for Connection {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.prepare()?;
            let result = self.0.write(buf);
            self.completed(result)
        }

        fn flush(&mut self) -> io::Result<()> {
            self.prepare()?;
            self.0.flush()
        }
    }

    pub struct Listener(pub(super) os::Listener);

    impl Listener {
        /// A 0600 socket in a real directory owned by this user with mode 0700. Hold the daemon's
        /// instance lock first: only then may a stale socket left by a crash be replaced.
        #[cfg(unix)]
        pub fn bind_private(endpoint: &Path) -> io::Result<Listener> {
            os::Listener::bind_private(endpoint).map(Listener)
        }

        /// Listens at `endpoint`, reachable by this user only. A stale endpoint left by a crashed
        /// owner is replaced: hold the daemon's lock first, so there is no live one.
        pub fn bind(endpoint: &Path) -> io::Result<Listener> {
            os::Listener::bind(endpoint).map(Listener)
        }

        /// The next connection; with `wait` false, WouldBlock when none is pending.
        pub fn accept(&mut self, wait: bool) -> io::Result<Connection> {
            self.0.accept(wait).map(|stream| Connection(stream, None))
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        fn pair(tag: &str) -> (Connection, Connection, PathBuf) {
            let dir =
                std::env::temp_dir().join(format!("cv1t-deadline-{tag}-{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&dir);
            std::fs::create_dir_all(&dir).unwrap();
            let path = dir.join("control.sock");
            let mut listener = Listener::bind(&path).unwrap();
            let client = Connection::connect(&path).unwrap();
            let server = listener.accept(true).unwrap();
            (client, server, dir)
        }

        #[test]
        fn an_expired_request_does_not_consume_its_reply() {
            let (mut client, mut server, dir) = pair("read");
            server.write_all(b"x").unwrap();
            drop(server);
            client.1 = Some(Instant::now() - Duration::from_secs(1));
            let mut byte = [0];
            assert_eq!(
                client.read(&mut byte).unwrap_err().kind(),
                io::ErrorKind::TimedOut
            );
            client.1 = None;
            client.read_exact(&mut byte).unwrap();
            assert_eq!(byte, *b"x");
            drop(client);
            std::fs::remove_dir_all(dir).unwrap();
        }

        #[test]
        fn an_expired_request_does_not_write_its_command() {
            let (mut client, mut server, dir) = pair("write");
            client.1 = Some(Instant::now() - Duration::from_secs(1));
            assert_eq!(
                client.write(b"x").unwrap_err().kind(),
                io::ErrorKind::TimedOut
            );
            drop(client);
            let mut bytes = Vec::new();
            server.read_to_end(&mut bytes).unwrap();
            assert!(bytes.is_empty(), "{bytes:?}");
            drop(server);
            std::fs::remove_dir_all(dir).unwrap();
        }
    }
}

pub mod usb {
    //! The USB devices of one maker, and the driver the system runs each with: whether the engine
    //! can open the sensors depends on it.
    use std::io;

    use super::os;

    /// A USB device, or one of its interfaces, as the system lists it.
    #[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
    pub struct Node {
        /// The system's name for it: the device instance ID on Windows
        /// ("USB\VID_2833&PID_0211\..."), the sysfs name on Linux ("1-2", or "1-2:1.0" for an
        /// interface).
        pub id: String,
        pub product: u16,
        /// How the device or the system describes it.
        pub name: String,
        /// The driver it runs with, if any.
        pub driver: Option<String>,
        /// Why Windows does not run it: its Device Manager problem code (28: no driver).
        pub problem: Option<u32>,
        /// The `id` of what it hangs off: an interface's device; on Windows a device's hub too.
        pub parent: Option<String>,
    }

    /// The present devices of the USB vendor `vendor`, and their interfaces (on Windows, also
    /// the HID collections of their HID interfaces): on Linux each device before its interfaces,
    /// on Windows by `id`.
    pub fn devices(vendor: u16) -> io::Result<Vec<Node>> {
        os::usb_devices(vendor)
    }

    /// The device interface a device node set up by `use_winusb` is offered under (Windows):
    /// libusb finds a WinUSB device by the interface its registry entry names.
    pub const WINUSB_INTERFACE: &str = "{00CFBFC2-E6AB-4A1F-98D6-1E197D3841CC}";

    /// Gives the device node `id` Windows' own WinUSB driver, the "WinUsb Device" of the
    /// Microsoft-signed winusb.inf, as choosing it in Device Manager does, offered under
    /// `WINUSB_INTERFACE`. No other driver or certificate is installed. True when Windows could
    /// not restart the device (a program holds it) and needs a restart to finish. Needs an
    /// administrator. Linux has no such step (udev rules give access): Unsupported.
    pub fn use_winusb(id: &str) -> io::Result<bool> {
        os::use_winusb(id)
    }

    /// Whether `use_winusb` set up the device node `id`: it names `WINUSB_INTERFACE`.
    pub fn set_up_by_us(id: &str) -> bool {
        os::set_up_by_us(id)
    }

    /// Takes the device node `id` out and finds it again, as Device Manager's "Uninstall
    /// device" does: Windows then gives it the driver it would choose itself, or none. True when
    /// Windows needs a restart to finish, as `use_winusb` says. Needs an administrator;
    /// Unsupported on Linux.
    pub fn remove_driver(id: &str) -> io::Result<bool> {
        os::remove_driver(id)
    }

    /// Whether this process may open the device `id` the way the engine does (Linux: its file
    /// under /dev/bus/usb, for reading and writing, which the udev rules allow the active
    /// session): PermissionDenied when not. On Windows the driver decides: Unsupported.
    pub fn can_open(id: &str) -> io::Result<()> {
        os::can_open(id)
    }
}

pub mod sound {
    //! Sounds the command-line tool marks timed steps with.
    use super::os;

    /// Plays `wav`, a WAV file's bytes, to its end: through PipeWire or PulseAudio (paplay) on
    /// Linux, the default output device on Windows. A missing sound server only loses the sound,
    /// silently: its complaints would bury the instructions on the terminal.
    pub fn play_wav(wav: &[u8]) {
        os::play_wav(wav)
    }
}

pub mod process {
    //! Other programs running on this machine.
    use std::io;

    use super::os;

    /// Whether a process of the program `name` runs, under any account: its executable's name
    /// without the extension, of letters, digits, '-' and '_' ("vrserver" is vrserver.exe on
    /// Windows, in any case). Linux asks pgrep (procps); it keeps the first 15 bytes of a
    /// process's name, so a longer `name` is compared by those.
    pub fn running(name: &str) -> io::Result<bool> {
        check_name(name)?;
        os::process_running(name)
    }

    /// The same name check without starting a child process: /proc on Linux, a Toolhelp
    /// snapshot on Windows. Safe to call from the daemon with locked memory and real-time threads.
    pub fn running_without_child(name: &str) -> io::Result<bool> {
        check_name(name)?;
        os::process_running_without_child(name)
    }

    fn check_name(name: &str) -> io::Result<()> {
        if name.is_empty()
            || !name
                .bytes()
                .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("{name:?} is not a plain program name"),
            ));
        }
        Ok(())
    }

    /// Makes the process `command` starts end when this one does, however it ends. On Linux it
    /// gets SIGTERM when the thread that starts it ends: start it from the main thread. On
    /// Windows the whole of this process joins a job object (once), and with it everything it
    /// starts from then on; the job ends them all when this process ends, or at once when one of
    /// them crashes, instead of after Windows Error Reporting has looked at it.
    pub fn end_with_this_process(command: &mut std::process::Command) -> io::Result<()> {
        os::end_with_this_process(command)
    }

    /// Shows the file at `path` selected in a file manager window (Explorer on Windows); false
    /// where there is none to ask (Linux, from a command-line tool).
    pub fn show_file(path: &std::path::Path) -> bool {
        os::show_file(path)
    }

    /// Whether this process has an administrator's rights: an elevated token on Windows, root on
    /// Linux.
    pub fn elevated() -> bool {
        os::elevated()
    }

    /// Runs this program again with `args` (plain words: letters, digits and '-') as an
    /// administrator, Windows asking the user first, and waits for it: its exit code.
    /// PermissionDenied when the user says no; Unsupported on Linux.
    pub fn run_elevated(args: &[&str]) -> io::Result<i32> {
        if !args
            .iter()
            .all(|a| !a.is_empty() && a.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-'))
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("{args:?} are not plain words"),
            ));
        }
        os::run_elevated(args)
    }

    /// Tells the person `text` from a program that has no console to print it in: a message box
    /// titled `title` on Windows, standard error on Linux. Returns once it is dismissed.
    pub fn alert(title: &str, text: &str) {
        os::alert(title, text)
    }

    /// Starts the program `command` runs without a console window of its own (Windows); on
    /// Linux nothing changes.
    pub fn without_window(command: &mut std::process::Command) {
        os::without_window(command)
    }
}

#[cfg(test)]
mod tests {
    use super::fs;
    use super::ipc::{Connection, Listener};
    use std::io::{BufRead, BufReader, Read, Write};

    fn endpoint(tag: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("cv1t-ipc-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir.join("control.sock")
    }

    #[test]
    fn a_connection_carries_a_line_each_way() {
        let path = endpoint("line");
        assert_eq!(
            Connection::connect(&path).err().map(|e| e.kind()),
            Some(std::io::ErrorKind::NotFound)
        );
        let mut listener = Listener::bind(&path).unwrap();
        let server = std::thread::spawn(move || {
            let conn = listener.accept(true).unwrap();
            let mut reader = BufReader::new(conn);
            let mut line = String::new();
            reader.read_line(&mut line).unwrap();
            reader
                .get_mut()
                .write_all(format!("ok {line}").as_bytes())
                .unwrap();
            // the next client is accepted too
            listener.accept(true).unwrap();
        });
        let mut client = Connection::connect(&path).unwrap();
        client
            .set_timeout(Some(std::time::Duration::from_secs(5)))
            .unwrap();
        client.write_all(b"status\n").unwrap();
        let mut reply = String::new();
        BufReader::new(client).read_line(&mut reply).unwrap();
        assert_eq!(reply, "ok status\n");
        drop(Connection::connect(&path).unwrap());
        server.join().unwrap();
    }

    #[test]
    fn a_silent_listener_times_a_read_out_and_a_second_listener_is_refused() {
        let path = endpoint("timeout");
        let mut listener = Listener::bind(&path).unwrap();
        let mut client = Connection::connect(&path).unwrap();
        let _server = listener.accept(true).unwrap();
        client
            .set_timeout(Some(std::time::Duration::from_millis(50)))
            .unwrap();
        let started = std::time::Instant::now();
        let err = client.read(&mut [0u8; 8]).unwrap_err();
        assert!(
            matches!(
                err.kind(),
                std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
            ),
            "{err:?}"
        );
        assert!(started.elapsed() < std::time::Duration::from_secs(2));
        // nothing else pending
        assert_eq!(
            listener.accept(false).err().map(|e| e.kind()),
            Some(std::io::ErrorKind::WouldBlock)
        );
        // (on Linux, finding out that someone listens leaves it an empty connection)
        assert!(Listener::bind(&path).is_err(), "one listener per endpoint");
    }

    #[test]
    fn the_listener_keeps_serving_after_clients_vanish() {
        let path = endpoint("vanish");
        let mut listener = Listener::bind(&path).unwrap();
        let server = std::thread::spawn(move || loop {
            // clients that came and went read as an immediate end of stream
            let mut conn = listener.accept(true).unwrap();
            let mut byte = [0u8; 1];
            if conn.read(&mut byte).unwrap() == 1 {
                conn.write_all(&byte).unwrap();
                return;
            }
        });
        for _ in 0..200 {
            drop(Connection::connect(&path).unwrap());
        }
        let mut client = Connection::connect(&path).unwrap();
        client
            .set_timeout(Some(std::time::Duration::from_secs(5)))
            .unwrap();
        client.write_all(b"x").unwrap();
        let mut byte = [0u8; 1];
        client.read_exact(&mut byte).unwrap();
        assert_eq!(&byte, b"x");
        server.join().unwrap();
    }

    /// Another account could create the daemon's pipe first (pipe names are global): a client
    /// refuses to talk to it. Owning a pipe as another account needs a group that may own
    /// objects, which an elevated administrator has; other tokens skip the test.
    #[cfg(windows)]
    #[test]
    fn a_client_refuses_a_pipe_another_account_owns() {
        use std::os::windows::ffi::OsStrExt;
        use windows_sys::Win32::Foundation::{CloseHandle, LocalFree, INVALID_HANDLE_VALUE};
        use windows_sys::Win32::Security::Authorization::ConvertStringSecurityDescriptorToSecurityDescriptorW;
        use windows_sys::Win32::Security::SECURITY_ATTRIBUTES;
        use windows_sys::Win32::Storage::FileSystem::PIPE_ACCESS_DUPLEX;
        use windows_sys::Win32::System::Pipes::{CreateNamedPipeW, PIPE_TYPE_BYTE};

        let path = endpoint("squat");
        let mut name =
            super::pipe_name(&path.as_os_str().encode_wide().collect::<Vec<u16>>()).unwrap();
        name.push(0);
        // owned by the administrators group, open to everyone
        let sddl: Vec<u16> = std::ffi::OsStr::new("O:BAD:(A;;GA;;;WD)")
            .encode_wide()
            .chain([0])
            .collect();
        // SAFETY: NUL-terminated strings, valid out-pointers; everything is freed below.
        unsafe {
            let mut descriptor = std::ptr::null_mut();
            assert_ne!(
                ConvertStringSecurityDescriptorToSecurityDescriptorW(
                    sddl.as_ptr(),
                    1,
                    &mut descriptor,
                    std::ptr::null_mut()
                ),
                0
            );
            let attributes = SECURITY_ATTRIBUTES {
                nLength: std::mem::size_of::<SECURITY_ATTRIBUTES>() as u32,
                lpSecurityDescriptor: descriptor,
                bInheritHandle: 0,
            };
            let squat = CreateNamedPipeW(
                name.as_ptr(),
                PIPE_ACCESS_DUPLEX,
                PIPE_TYPE_BYTE,
                1,
                512,
                512,
                0,
                &attributes,
            );
            LocalFree(descriptor);
            if squat == INVALID_HANDLE_VALUE {
                let e = std::io::Error::last_os_error();
                // ERROR_INVALID_OWNER: this token may not name the administrators as owner
                assert_eq!(e.raw_os_error(), Some(1307), "{e}");
                eprintln!("skipped: needs an elevated administrator token");
                return;
            }
            let refused = Connection::connect(&path).err().map(|e| e.kind());
            CloseHandle(squat);
            assert_eq!(refused, Some(std::io::ErrorKind::PermissionDenied));
        }
    }

    #[test]
    fn the_daemon_lock_is_exclusive_until_released() {
        let path = endpoint("lock").with_file_name("cv1touchd.lock");
        let first = super::ipc::lock_exclusive(&path).unwrap();
        assert!(super::ipc::lock_exclusive(&path).is_err());
        drop(first);
        drop(super::ipc::lock_exclusive(&path).unwrap());
    }
    use super::shm::{OpenError, OpenedSegment, OwnedSegment};
    use std::sync::atomic::{AtomicU32, Ordering};

    fn segment_name(tag: &str) -> String {
        format!("/cv1t-platform-{tag}-{}", std::process::id())
    }

    /// Input fed through a channel: a read blocks until something is sent, and ends when the
    /// sender is dropped. `reads` counts the reads begun.
    struct Fed(
        std::sync::mpsc::Receiver<Vec<u8>>,
        Vec<u8>,
        std::sync::Arc<AtomicU32>,
    );

    impl Fed {
        fn new(from: std::sync::mpsc::Receiver<Vec<u8>>) -> (Fed, std::sync::Arc<AtomicU32>) {
            let reads = std::sync::Arc::new(AtomicU32::new(0));
            (Fed(from, Vec::new(), reads.clone()), reads)
        }
    }

    impl std::io::Read for Fed {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            self.2.fetch_add(1, Ordering::SeqCst);
            if self.1.is_empty() {
                match self.0.recv() {
                    Ok(bytes) => self.1 = bytes,
                    Err(_) => return Ok(0),
                }
            }
            let n = buf.len().min(self.1.len());
            buf[..n].copy_from_slice(&self.1[..n]);
            self.1.drain(..n);
            Ok(n)
        }
    }

    #[test]
    fn typed_lines_arrive_as_they_come_and_waiting_for_one_ends() {
        use super::console::{Line, Lines};
        use std::time::Duration;
        let (tx, rx) = std::sync::mpsc::channel();
        let (fed, reads) = Fed::new(rx);
        let lines = Lines::spawn(std::io::BufReader::new(fed), false);
        assert_eq!(lines.next(Duration::from_millis(20)), Line::NotYet);
        tx.send(b"half a ".to_vec()).unwrap();
        assert_eq!(
            lines.next(Duration::from_millis(20)),
            Line::NotYet,
            "no Enter yet"
        );
        tx.send(b"line\ntyped ahead\n".to_vec()).unwrap();
        assert_eq!(
            lines.next(Duration::from_secs(5)),
            Line::Typed("half a line\n".into())
        );
        // the second line was read with the first and is queued without any I/O: once the
        // reader has begun a third read, both lines are queued
        assert!(within_five_seconds(|| reads.load(Ordering::SeqCst) == 3));
        lines.discard();
        assert_eq!(lines.next(Duration::from_millis(20)), Line::NotYet);
        drop(tx);
        assert_eq!(lines.next(Duration::from_secs(5)), Line::Closed);
        assert_eq!(lines.next(Duration::from_millis(20)), Line::Closed);
    }

    /// Waits up to five seconds for `done`; whether it came.
    fn within_five_seconds(done: impl Fn() -> bool) -> bool {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while !done() {
            if std::time::Instant::now() > deadline {
                return false;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        true
    }

    /// An empty read is an end of input: Ctrl+D at a terminal, which ends that read only; the
    /// end of a pipe or a file, which ends them all.
    #[test]
    fn at_a_terminal_typing_goes_on_after_an_end_of_input() {
        use super::console::{Line, Lines};
        use std::time::Duration;
        for terminal in [true, false] {
            let (tx, rx) = std::sync::mpsc::channel();
            let (fed, reads) = Fed::new(rx);
            let lines = Lines::spawn(std::io::BufReader::new(fed), terminal);
            tx.send(Vec::new()).unwrap();
            assert_eq!(lines.next(Duration::from_secs(5)), Line::Closed);
            let _ = tx.send(b"hello\n".to_vec());
            let next = lines.next(Duration::from_secs(5));
            if !terminal {
                assert_eq!(next, Line::Closed);
                continue;
            }
            assert_eq!(next, Line::Typed("hello\n".into()));
            // the reader waits in its third read; an end of input it reports is then dropped as
            // typed ahead, which lets it read on
            assert!(within_five_seconds(|| reads.load(Ordering::SeqCst) == 3));
            tx.send(Vec::new()).unwrap();
            assert!(within_five_seconds(|| {
                lines.discard();
                reads.load(Ordering::SeqCst) == 4
            }));
            tx.send(b"again\n".to_vec()).unwrap();
            assert_eq!(
                lines.next(Duration::from_secs(5)),
                Line::Typed("again\n".into())
            );
        }
    }

    /// In a process of its own, with a console of its own: keys typed without Enter while the
    /// reader waits for a line are dropped with the rest, so the next line holds only what is
    /// typed after.
    #[cfg(windows)]
    #[test]
    fn an_unfinished_line_at_a_console_is_discarded() {
        use super::console::{self, Line};
        use std::os::windows::io::IntoRawHandle;
        use std::time::Duration;
        use windows_sys::Win32::System::Console::{
            AllocConsole, FreeConsole, GetConsoleScreenBufferInfo, GetNumberOfConsoleInputEvents,
            SetStdHandle, WriteConsoleInputW, WriteConsoleW, CONSOLE_SCREEN_BUFFER_INFO,
            INPUT_RECORD, KEY_EVENT, STD_INPUT_HANDLE,
        };
        const CHILD: &str = "CV1TOUCH_TEST_CONSOLE";
        if std::env::var_os(CHILD).is_none() {
            let output = std::process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "platform::tests::an_unfinished_line_at_a_console_is_discarded",
                    "--nocapture",
                ])
                .env(CHILD, "1")
                .output()
                .unwrap();
            assert!(output.status.success(), "{output:?}");
            return;
        }
        // SAFETY: this process's console is its own (it runs only this test).
        unsafe {
            FreeConsole();
            assert_ne!(AllocConsole(), 0, "{}", std::io::Error::last_os_error());
        }
        let input = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open("CONIN$")
            .unwrap()
            .into_raw_handle();
        // SAFETY: an open console input handle, kept for the rest of the process.
        assert_ne!(unsafe { SetStdHandle(STD_INPUT_HANDLE, input) }, 0);
        let screen = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open("CONOUT$")
            .unwrap()
            .into_raw_handle();
        let cursor = || {
            // SAFETY: a plain structure filled in through the open screen buffer handle.
            let mut info: CONSOLE_SCREEN_BUFFER_INFO = unsafe { std::mem::zeroed() };
            assert_ne!(unsafe { GetConsoleScreenBufferInfo(screen, &mut info) }, 0);
            (info.dwCursorPosition.X, info.dwCursorPosition.Y)
        };
        let print = |text: &str| {
            let text: Vec<u16> = text.encode_utf16().collect();
            let mut written = 0;
            // SAFETY: the screen buffer handle and a buffer of the stated length.
            let ok = unsafe {
                WriteConsoleW(
                    screen,
                    text.as_ptr(),
                    text.len() as u32,
                    &mut written,
                    std::ptr::null(),
                )
            };
            assert_ne!(ok, 0);
        };
        // the waiting read has taken every key typed so far
        let taken = || {
            within_five_seconds(|| {
                let mut events = 0;
                // SAFETY: the console input handle and an output count.
                unsafe { GetNumberOfConsoleInputEvents(input, &mut events) };
                events == 0
            })
        };
        let type_keys = |text: &str| {
            let records: Vec<INPUT_RECORD> = text
                .encode_utf16()
                .flat_map(|c| {
                    [1, 0].map(|down| {
                        // SAFETY: a plain record, zeroed and then filled in.
                        let mut record: INPUT_RECORD = unsafe { std::mem::zeroed() };
                        record.EventType = KEY_EVENT as u16;
                        record.Event.KeyEvent.bKeyDown = down;
                        record.Event.KeyEvent.wRepeatCount = 1;
                        // the virtual-key code of a letter is its upper case; Enter's is
                        // its character
                        record.Event.KeyEvent.wVirtualKeyCode =
                            u8::try_from(c).map_or(c, |b| u16::from(b.to_ascii_uppercase()));
                        record.Event.KeyEvent.uChar.UnicodeChar = c;
                        record
                    })
                })
                .collect();
            let mut written = 0;
            // SAFETY: the console input handle and records it copies.
            let ok = unsafe {
                WriteConsoleInputW(input, records.as_ptr(), records.len() as u32, &mut written)
            };
            assert_ne!(ok, 0, "{}", std::io::Error::last_os_error());
        };
        // the reader starts, and waits in a read for a line; the tool goes on printing
        assert_eq!(console::next_line(Duration::from_millis(200)), Line::NotYet);
        print("a capture's output\r\nand more\r\n");
        let before = cursor();
        type_keys("q");
        assert!(taken());
        console::discard_typed_ahead();
        assert!(taken());
        // the line is cleared where it was typed, and the cursor stays there
        assert_eq!(cursor(), before);
        type_keys("y\r");
        match console::next_line(Duration::from_secs(5)) {
            Line::Typed(line) => assert_eq!(line.trim_end(), "y"),
            other => panic!("{other:?}"),
        }
    }

    /// In a process of its own: the redirect moves the whole process's output, and the test
    /// harness prints from other threads.
    #[test]
    fn output_goes_to_a_file_until_the_redirect_ends() {
        use std::io::Write;
        const CHILD: &str = "CV1TOUCH_TEST_REDIRECT";
        let Some(path) = std::env::var_os(CHILD) else {
            let dir = std::env::temp_dir().join(format!("cv1t-redirect-{}", std::process::id()));
            std::fs::create_dir_all(&dir).unwrap();
            let path = dir.join("log");
            let output = std::process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "platform::tests::output_goes_to_a_file_until_the_redirect_ends",
                    "--nocapture",
                ])
                .env(CHILD, &path)
                .output()
                .unwrap();
            assert!(output.status.success(), "{output:?}");
            assert_eq!(std::fs::read_to_string(&path).unwrap(), "to the log\n");
            std::fs::remove_dir_all(&dir).unwrap();
            return;
        };
        let log = super::fs::append_private(std::path::Path::new(&path)).unwrap();
        let redirect = super::console::redirect_output(&log).unwrap();
        std::io::stdout().write_all(b"to the log\n").unwrap();
        drop(redirect);
        std::io::stdout()
            .write_all(b"back on the terminal\n")
            .unwrap();
    }

    /// A process that sleeps for half a minute, its output nowhere.
    fn sleeper() -> std::process::Command {
        use std::process::{Command, Stdio};
        #[cfg(unix)]
        let mut c = {
            let mut c = Command::new("sleep");
            c.arg("30");
            c
        };
        #[cfg(windows)]
        let mut c = {
            let mut c = Command::new("ping");
            c.args(["-n", "30", "127.0.0.1"]);
            c
        };
        c.stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null());
        c
    }

    #[cfg(unix)]
    fn alive(pid: u32) -> bool {
        // the state follows the name in parentheses; Z is a process that has ended
        std::fs::read_to_string(format!("/proc/{pid}/stat")).is_ok_and(|stat| {
            stat.rsplit_once(')')
                .is_some_and(|(_, rest)| !rest.trim_start().starts_with('Z'))
        })
    }

    #[cfg(windows)]
    fn alive(pid: u32) -> bool {
        use windows_sys::Win32::Foundation::{CloseHandle, WAIT_TIMEOUT};
        use windows_sys::Win32::System::Threading::{
            OpenProcess, WaitForSingleObject, PROCESS_SYNCHRONIZE,
        };
        // SAFETY: a handle opened for waiting, closed after a wait that does not block.
        unsafe {
            let h = OpenProcess(PROCESS_SYNCHRONIZE, 0, pid);
            if h.is_null() {
                return false;
            }
            let running = WaitForSingleObject(h, 0) == WAIT_TIMEOUT;
            CloseHandle(h);
            running
        }
    }

    /// In a process of its own, which starts two sleepers and ends: the one started after
    /// `end_with_this_process` ends with it, the one started before keeps running. The pids come
    /// back through a file: a pipe would stay open while either sleeper holds it.
    #[test]
    fn a_process_started_to_end_with_this_one_does() {
        use super::process;
        const CHILD: &str = "CV1TOUCH_TEST_END_WITH";
        let Some(pids) = std::env::var_os(CHILD) else {
            let pids = std::env::temp_dir().join(format!("cv1t-end-with-{}", std::process::id()));
            let status = std::process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "platform::tests::a_process_started_to_end_with_this_one_does",
                ])
                .env(CHILD, &pids)
                .stdin(std::process::Stdio::null())
                .stdout(std::process::Stdio::null())
                .stderr(std::process::Stdio::null())
                .status()
                .unwrap();
            assert!(status.success());
            let text = std::fs::read_to_string(&pids).unwrap();
            std::fs::remove_file(&pids).unwrap();
            let pid = |what: &str| -> u32 {
                text.lines()
                    .find_map(|l| l.strip_prefix(what)?.trim().parse().ok())
                    .unwrap_or_else(|| panic!("no {what} in {text}"))
            };
            let (kept, ended) = (pid("kept"), pid("ended"));
            let gone = within_five_seconds(|| !alive(ended));
            let kept_alive = alive(kept);
            #[cfg(unix)]
            // SAFETY: signalling a process this test started.
            unsafe {
                libc::kill(kept as i32, libc::SIGKILL);
            }
            #[cfg(windows)]
            let _ = std::process::Command::new("taskkill")
                .args(["/F", "/PID", &kept.to_string()])
                .output();
            assert!(gone, "the process tied to its starter outlived it");
            assert!(kept_alive, "the untied process ended too");
            return;
        };
        // this process ends before either: what happens to them then is the test
        let kept = sleeper().spawn().unwrap().id();
        let mut tied = sleeper();
        process::end_with_this_process(&mut tied).unwrap();
        let ended = tied.spawn().unwrap().id();
        std::fs::write(pids, format!("kept {kept}\nended {ended}\n")).unwrap();
    }

    #[test]
    fn finds_a_thread_of_ours_by_its_name() {
        use super::system;
        let (tx, rx) = std::sync::mpsc::channel();
        let (stop_tx, stop_rx) = std::sync::mpsc::channel::<()>();
        let t = std::thread::Builder::new()
            .name("cv1t-findme".into())
            .spawn(move || {
                tx.send(system::current_thread()).unwrap();
                stop_rx.recv().ok();
            })
            .unwrap();
        let tid = rx.recv().unwrap();
        assert_eq!(system::thread_named("cv1t-findme"), Some(tid));
        assert_eq!(system::thread_named("cv1t-nobody"), None);
        stop_tx.send(()).unwrap();
        t.join().unwrap();
    }

    #[test]
    fn a_thread_keeps_to_the_cpus_it_is_given() {
        use super::system;
        std::thread::spawn(|| {
            let me = system::current_thread();
            let allowed = system::allowed_cpus(me).unwrap();
            assert!(!allowed.is_empty());
            system::set_cpus(me, &allowed[..1]).unwrap();
            assert_eq!(system::allowed_cpus(me).unwrap(), allowed[..1]);
        })
        .join()
        .unwrap();
    }

    #[test]
    fn the_system_gives_random_bits() {
        use super::system;
        let (a, b) = (system::random_u64().unwrap(), system::random_u64().unwrap());
        assert_ne!(a, b);
    }

    /// The kernel's release on Linux; on Windows a build and update number read from the
    /// registry, and the feature update's name.
    #[test]
    fn the_system_describes_itself() {
        let description = super::system::description();
        if cfg!(windows) {
            let build = description
                .split_once("(build ")
                .and_then(|(_, rest)| rest.strip_suffix(')'))
                .unwrap_or_else(|| panic!("{description}"));
            let (number, update) = build.split_once('.').unwrap();
            assert!(
                number.parse::<u32>().is_ok_and(|n| n >= 10_000),
                "{description}"
            );
            assert!(update.parse::<u32>().is_ok(), "{description}");
            assert!(!description.contains("  "), "{description}");
        } else {
            let release = std::fs::read_to_string("/proc/sys/kernel/osrelease").unwrap();
            assert!(description.contains(release.trim()), "{description}");
        }
    }

    /// Whatever is plugged in (none on a CI runner), the list can be read and holds only the
    /// vendor's devices' entries: on Linux each device before its interfaces, on Windows by ID.
    #[test]
    fn the_usb_devices_of_a_vendor_can_be_listed() {
        let nodes = super::usb::devices(0x2833).unwrap();
        let key = |id: &str| {
            if cfg!(windows) {
                (id.to_string(), String::new())
            } else {
                let (device, interface) = id.split_once(':').unwrap_or((id, ""));
                (device.to_string(), interface.to_string())
            }
        };
        assert!(nodes.windows(2).all(|w| key(&w[0].id) <= key(&w[1].id)));
        if cfg!(windows) {
            assert!(nodes
                .iter()
                .all(|n| n.id.to_uppercase().contains("VID_2833")));
        }
    }

    /// Gives a spare USB device Windows' WinUSB, reads its interface back, and takes the driver
    /// off again, which hands it back to the driver it had. Run as an administrator with the
    /// device's vendor and instance ID: CV1TOUCH_TEST_WINUSB_DEVICE=46F4,USB\VID_46F4&PID_0001\...
    /// cargo test -- --ignored winusb
    #[cfg(windows)]
    #[test]
    #[ignore = "needs an administrator and a spare USB device"]
    fn winusb_is_given_and_taken_off() {
        use super::usb;
        let spec = std::env::var("CV1TOUCH_TEST_WINUSB_DEVICE").unwrap();
        let (vendor, id) = spec.split_once(',').unwrap();
        let vendor = u16::from_str_radix(vendor, 16).unwrap();
        let driver = || {
            usb::devices(vendor)
                .unwrap()
                .into_iter()
                .find(|n| n.id.eq_ignore_ascii_case(id))
                .unwrap()
                .driver
        };
        let before = driver();
        assert!(!before
            .as_deref()
            .is_some_and(|d| d.eq_ignore_ascii_case("WinUSB")));
        assert!(super::process::elevated());
        assert!(!usb::use_winusb(id).unwrap(), "a restart was asked for");
        assert!(driver().is_some_and(|d| d.eq_ignore_ascii_case("WinUSB")));
        assert!(usb::set_up_by_us(id));
        assert!(!usb::remove_driver(id).unwrap(), "a restart was asked for");
        assert_eq!(driver(), before);
        assert!(!usb::set_up_by_us(id));
    }

    /// Under cargo the console is shared with cargo and the shell (Linux has no console that
    /// closes); only a program started on its own, as `run_elevated` does, is alone at one.
    #[test]
    fn the_console_is_shared_under_cargo() {
        assert!(!super::console::closes_with_us());
    }

    /// In a process of its own, with a console of its own, as a program started from Explorer
    /// or by `run_elevated` has: that console closes with it.
    #[cfg(windows)]
    #[test]
    fn the_console_closes_with_a_program_alone_at_it() {
        use windows_sys::Win32::System::Console::{AllocConsole, FreeConsole};
        const CHILD: &str = "CV1TOUCH_TEST_OWN_CONSOLE";
        if std::env::var_os(CHILD).is_none() {
            let output = std::process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "platform::tests::the_console_closes_with_a_program_alone_at_it",
                    "--nocapture",
                ])
                .env(CHILD, "1")
                .output()
                .unwrap();
            assert!(output.status.success(), "{output:?}");
            return;
        }
        // SAFETY: this process's console is its own (it runs only this test).
        unsafe {
            FreeConsole();
            assert_ne!(AllocConsole(), 0, "{}", std::io::Error::last_os_error());
        }
        assert!(super::console::closes_with_us());
    }

    #[test]
    fn power_throttling_can_be_turned_off() {
        super::system::opt_out_of_throttling().unwrap();
    }

    /// This test binary's name is longer than Linux keeps (cv1touch-<hash>), and has .exe on
    /// Windows.
    #[test]
    fn a_running_program_is_found_by_name() {
        use super::process;
        let exe = std::env::current_exe().unwrap();
        let name = exe.file_stem().unwrap().to_str().unwrap();
        assert!(name.len() > 15, "{name}");
        for running in [
            process::running as fn(&str) -> std::io::Result<bool>,
            process::running_without_child,
        ] {
            assert!(running(name).unwrap());
            assert!(running(&name.to_uppercase()).unwrap() == cfg!(windows));
            assert!(!running("cv1touch-not-a-program").unwrap());
            // a name, not an option
            assert!(!running("-cv1touch-not-a-program").unwrap());
            assert!(running("vrserver|init").is_err());
            assert!(running("").is_err());
        }
    }

    /// On Windows a real-time request takes no privileges: the thread runs time-critical.
    #[cfg(windows)]
    #[test]
    fn a_realtime_thread_runs_time_critical() {
        use super::system;
        use windows_sys::Win32::System::Threading::{
            GetCurrentThread, GetThreadPriority, THREAD_PRIORITY_TIME_CRITICAL,
        };
        std::thread::spawn(|| {
            let how = system::make_realtime(system::current_thread(), 10).unwrap();
            assert_eq!(how, "time-critical priority");
            // SAFETY: the calling thread's pseudo-handle.
            assert_eq!(
                unsafe { GetThreadPriority(GetCurrentThread()) },
                THREAD_PRIORITY_TIME_CRITICAL
            );
        })
        .join()
        .unwrap();
    }

    #[test]
    fn the_clock_converts_counter_ticks_exactly_and_moves_forward() {
        assert_eq!(super::ticks_to_ns(10_000_000, 10_000_000), 1_000_000_000);
        // a month of uptime at 10 MHz, which overflows a naive ticks * 1e9 in u64
        let month = 30 * 24 * 3600 * 10_000_000u64;
        assert_eq!(
            super::ticks_to_ns(month, 10_000_000),
            30 * 24 * 3600 * 1_000_000_000
        );
        assert_eq!(super::ticks_to_ns(3, 3_000_000_000), 1);
        let (a, b) = (super::now_ns(), super::now_ns());
        assert!(a > 0 && b >= a);
    }

    fn env(vars: &[(&str, &str)]) -> impl Fn(&str) -> Option<std::ffi::OsString> {
        let vars: std::collections::HashMap<String, String> = vars
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect();
        move |name| vars.get(name).map(Into::into)
    }

    #[cfg(unix)]
    #[test]
    fn linux_directories_follow_xdg_and_fall_back_under_home() {
        use super::dirs::{resolve, Dirs};
        use std::path::PathBuf;
        let home = env(&[("HOME", "/home/u")]);
        assert_eq!(
            resolve(&home),
            Dirs {
                config_home: PathBuf::from("/home/u/.config"),
                config: PathBuf::from("/home/u/.config/cv1touch"),
                state: PathBuf::from("/home/u/.local/state/cv1touch"),
                data: PathBuf::from("/home/u/.local/share/cv1touch"),
            }
        );
        let xdg = env(&[
            ("HOME", "/home/u"),
            ("XDG_CONFIG_HOME", "/c"),
            ("XDG_STATE_HOME", "/s"),
            ("XDG_DATA_HOME", "/d"),
        ]);
        let d = resolve(&xdg);
        assert_eq!(
            (d.config_home, d.config, d.state, d.data),
            (
                PathBuf::from("/c"),
                PathBuf::from("/c/cv1touch"),
                PathBuf::from("/s/cv1touch"),
                PathBuf::from("/d/cv1touch")
            )
        );
        // the specification: an empty or relative value counts as unset
        let odd = env(&[
            ("HOME", "/home/u"),
            ("XDG_CONFIG_HOME", ""),
            ("XDG_STATE_HOME", "relative"),
        ]);
        let d = resolve(&odd);
        assert_eq!(d.config, PathBuf::from("/home/u/.config/cv1touch"));
        assert_eq!(d.state, PathBuf::from("/home/u/.local/state/cv1touch"));
        // no home: relative (the engine agrees: tests/test_config_dir.c)
        assert_eq!(resolve(&env(&[])).config_home, PathBuf::from(".config"));
    }

    #[cfg(windows)]
    #[test]
    fn windows_directories_live_in_local_app_data() {
        use super::dirs::{resolve, Dirs};
        use std::path::PathBuf;
        let local = env(&[("LOCALAPPDATA", r"C:\Users\a b\AppData\Local")]);
        assert_eq!(
            resolve(&local),
            Dirs {
                config_home: PathBuf::from(r"C:\Users\a b\AppData\Local"),
                config: PathBuf::from(r"C:\Users\a b\AppData\Local\cv1touch"),
                state: PathBuf::from(r"C:\Users\a b\AppData\Local\cv1touch\state"),
                data: PathBuf::from(r"C:\Users\a b\AppData\Local\cv1touch\data"),
            }
        );
        // the control endpoint sits in the configuration directory, as the driver expects
        assert_eq!(super::ipc::runtime_dir(), super::dirs::get().config);
        // unset or relative: the temp directory (the engine agrees: tests/test_config_dir.c)
        for odd in [env(&[]), env(&[("LOCALAPPDATA", r"AppData\Local")])] {
            assert_eq!(resolve(&odd).config_home, std::env::temp_dir());
        }
    }

    #[test]
    fn a_hid_wait_ends_on_a_report_a_waker_or_the_timeout() {
        use super::hid::{Waker, Woke};
        use std::time::{Duration, Instant};
        // reports arrive one per read, as from the HID interfaces: a SOCK_SEQPACKET pair on Linux,
        // a message-mode named pipe on Windows; dropping the sender is the device going away
        let (device, sender) = super::os::fake_hid();
        let (mut device, mut sender) = (super::hid::Device(device), Some(sender));
        let waker = std::sync::Arc::new(Waker::new().unwrap());
        let mut buf = [0u8; 64];

        let before = Instant::now();
        assert_eq!(
            device
                .wait(&waker, Some(Duration::from_millis(30)))
                .unwrap(),
            Woke::default()
        );
        assert!(before.elapsed() >= Duration::from_millis(20));
        assert_eq!(device.try_read(&mut buf).unwrap(), None);

        let remote = waker.clone();
        let t = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            remote.wake();
        });
        let before = Instant::now();
        let woke = device.wait(&waker, Some(Duration::from_secs(5))).unwrap();
        t.join().unwrap();
        assert!(woke.waker && !woke.report, "{woke:?}");
        assert!(before.elapsed() < Duration::from_secs(2));

        super::os::fake_hid_send(sender.as_mut().unwrap(), &[0x0c, 1, 2, 3]);
        super::os::fake_hid_send(sender.as_mut().unwrap(), &[0x0c, 4, 5]);
        let mut got = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(2);
        while got.len() < 2 && Instant::now() < deadline {
            device
                .wait(&waker, Some(Duration::from_millis(100)))
                .unwrap();
            while let Some(n) = device.try_read(&mut buf).unwrap() {
                got.push(buf[..n].to_vec());
            }
        }
        assert_eq!(got, [vec![0x0c, 1, 2, 3], vec![0x0c, 4, 5]]);

        sender = None;
        let deadline = Instant::now() + Duration::from_secs(2);
        let gone = loop {
            device
                .wait(&waker, Some(Duration::from_millis(100)))
                .unwrap();
            match device.try_read(&mut buf) {
                Err(e) => break e,
                Ok(_) => assert!(Instant::now() < deadline, "the unplug went unnoticed"),
            }
        };
        let _ = (gone, sender);
    }

    #[test]
    fn a_shutdown_request_ends_the_wait() {
        use std::time::Duration;
        let requests = super::shutdown::Requests::take().unwrap();
        assert_eq!(requests.wait(Duration::from_millis(20)), None);
        super::os::request_shutdown();
        let why = requests.wait(Duration::from_secs(2));
        assert!(why.is_some_and(|w| w == "signal 15" || w == "Ctrl+Break"));
    }

    #[test]
    fn the_headset_interfaces_are_found_by_path() {
        use super::hid::is_headset_path;
        let radio = r"\\?\hid#vid_2833&pid_0031&mi_01#8&2b1a3c4d&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}";
        assert!(is_headset_path(radio, 1));
        assert!(!is_headset_path(radio, 0));
        assert!(is_headset_path(&radio.to_ascii_uppercase(), 1));
        assert!(!is_headset_path(r"\\?\hid#vid_2833&pid_0211&mi_01#...", 1));
    }

    #[test]
    fn a_pipe_is_named_after_its_endpoint_path() {
        let wide = |s: &str| s.encode_utf16().collect::<Vec<u16>>();
        assert_eq!(
            super::pipe_name(&wide(r"C:\Users\a b\AppData\Local\cv1touch\control.sock")),
            Some(wide(
                r"\\.\pipe\cv1touch-C:/Users/a b/AppData/Local/cv1touch/control.sock"
            ))
        );
        let long = format!(r"C:\{}", "x".repeat(250));
        assert_eq!(super::pipe_name(&wide(&long)), None);
    }

    #[test]
    fn a_segment_written_by_its_owner_reads_the_same_elsewhere() {
        let name = segment_name("rw");
        assert!(matches!(
            OpenedSegment::open(&name, 64),
            Err(OpenError::NotFound)
        ));
        let owner = OwnedSegment::create(&name, 64).unwrap();
        // SAFETY: 64 bytes mapped
        unsafe {
            assert!(std::slice::from_raw_parts(owner.ptr(), 64)
                .iter()
                .all(|&b| b == 0));
            owner.ptr().add(8).write(42);
        }
        let reader = OpenedSegment::open(&name, 64).unwrap();
        // SAFETY: as above
        assert_eq!(unsafe { reader.ptr().add(8).read() }, 42);
        assert!(
            OwnedSegment::create(&name, 64).is_err(),
            "one owner at a time"
        );
        drop(reader);
        drop(owner);
        assert!(matches!(
            OpenedSegment::open(&name, 64),
            Err(OpenError::NotFound)
        ));
        // the name is free again; a process another test is starting may hold a forked copy of
        // the ownership lock until it executes (the descriptor closes on exec), so allow a moment
        let free = (0..200).any(|_| {
            OwnedSegment::create(&name, 64).is_ok() || {
                std::thread::sleep(std::time::Duration::from_millis(10));
                false
            }
        });
        assert!(free, "the name stays taken");
    }

    #[test]
    fn a_reader_waiting_on_the_counter_is_woken_by_the_owner() {
        let name = segment_name("wake");
        let owner = OwnedSegment::create(&name, 64).unwrap();
        let reader = OpenedSegment::open(&name, 64).unwrap();
        let word = owner.ptr() as *mut u32;
        // the wait itself, on this thread, released by a bump from another
        let bumper_word = word as usize;
        let owner_ptr = &owner as *const OwnedSegment as usize;
        let bumper = std::thread::spawn(move || {
            std::thread::sleep(std::time::Duration::from_millis(50));
            // SAFETY: the counter is inside the live segment; the owner outlives the thread
            unsafe {
                AtomicU32::from_ptr(bumper_word as *mut u32).fetch_add(1, Ordering::Release);
                (*(owner_ptr as *const OwnedSegment)).wake(bumper_word as *mut u32);
            }
        });
        let before = std::time::Instant::now();
        reader.wait(reader.ptr() as *mut u32, 0, 5_000_000_000);
        let waited = before.elapsed();
        bumper.join().unwrap();
        assert!(waited < std::time::Duration::from_secs(2), "{waited:?}");
        // SAFETY: as above
        assert_eq!(
            unsafe { AtomicU32::from_ptr(reader.ptr() as *mut u32).load(Ordering::Acquire) },
            1
        );
        // with nothing changing, the wait ends at the timeout
        let before = std::time::Instant::now();
        reader.wait(reader.ptr() as *mut u32, 1, 30_000_000);
        assert!(before.elapsed() >= std::time::Duration::from_millis(20));
    }

    /// The driver and a tool can wait at once: every publish must wake all of them, not one.
    #[test]
    fn every_waiting_reader_is_woken_by_each_publish() {
        let name = segment_name("wake-all");
        let owner = OwnedSegment::create(&name, 64).unwrap();
        let readers: Vec<_> = (0..3)
            .map(|_| {
                let reader = OpenedSegment::open(&name, 64).unwrap();
                std::thread::spawn(move || {
                    let word = reader.ptr() as *mut u32;
                    let mut slowest = std::time::Duration::ZERO;
                    for seen in 0..3 {
                        let before = std::time::Instant::now();
                        reader.wait(word, seen, 5_000_000_000);
                        slowest = slowest.max(before.elapsed());
                        // SAFETY: the counter is inside the live segment
                        while unsafe { AtomicU32::from_ptr(word) }.load(Ordering::Acquire) == seen {
                            std::thread::yield_now();
                        }
                    }
                    slowest
                })
            })
            .collect();
        let word = owner.ptr() as *mut u32;
        for _ in 0..3 {
            std::thread::sleep(std::time::Duration::from_millis(100));
            // SAFETY: as above
            unsafe { AtomicU32::from_ptr(word) }.fetch_add(1, Ordering::Release);
            owner.wake(word);
        }
        for reader in readers {
            let slowest = reader.join().unwrap();
            assert!(slowest < std::time::Duration::from_secs(2), "{slowest:?}");
        }
    }

    #[test]
    fn a_system_library_loads_and_resolves_a_symbol() {
        #[cfg(unix)]
        let (lib, sym) = ("libc.so.6", c"getpid");
        #[cfg(windows)]
        let (lib, sym) = ("kernel32.dll", c"GetCurrentProcessId");
        let lib = super::dylib::Library::open(std::path::Path::new(lib)).unwrap();
        assert!(!lib.symbol(sym).is_null());
        assert!(lib.symbol(c"cv1touch_no_such_symbol").is_null());
        assert!(
            super::dylib::Library::open(std::path::Path::new("cv1touch-no-such-lib")).is_none()
        );
    }

    fn scratch(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("cv1t-platform-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn a_private_file_is_created_once_and_read_back() {
        let dir = scratch("private");
        let path = dir.join("a/b/c.json");
        fs::create_private_dir_all(path.parent().unwrap()).unwrap();
        use std::io::{Read, Write};
        fs::create_private(&path).unwrap().write_all(b"{}").unwrap();
        assert!(fs::create_private(&path).is_err(), "never overwrites");
        let mut s = String::new();
        fs::open_read_nofollow(&path)
            .unwrap()
            .read_to_string(&mut s)
            .unwrap();
        assert_eq!(s, "{}");
        fs::sync_dir(path.parent().unwrap()).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_rename_replaces_the_target() {
        let dir = scratch("rename");
        std::fs::write(dir.join("new"), "new").unwrap();
        std::fs::write(dir.join("old"), "old").unwrap();
        fs::rename_replacing(&dir.join("new"), &dir.join("old")).unwrap();
        assert_eq!(std::fs::read_to_string(dir.join("old")).unwrap(), "new");
        assert!(!dir.join("new").exists());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_directory_swap_leaves_the_old_contents_in_the_stage() {
        let dir = scratch("swap");
        let (stage, live) = (dir.join("stage"), dir.join("live"));
        std::fs::create_dir(&stage).unwrap();
        std::fs::write(stage.join("v"), "2").unwrap();
        fs::swap_dir(&stage, &live, false).unwrap();
        assert_eq!(std::fs::read_to_string(live.join("v")).unwrap(), "2");
        std::fs::create_dir(&stage).unwrap();
        std::fs::write(stage.join("v"), "3").unwrap();
        fs::swap_dir(&stage, &live, true).unwrap();
        assert_eq!(std::fs::read_to_string(live.join("v")).unwrap(), "3");
        assert_eq!(std::fs::read_to_string(stage.join("v")).unwrap(), "2");
        // without `exists`, an existing target is never replaced
        std::fs::create_dir(dir.join("other")).unwrap();
        assert!(fs::swap_dir(&dir.join("other"), &live, false).is_err());
        assert_eq!(std::fs::read_to_string(live.join("v")).unwrap(), "3");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
