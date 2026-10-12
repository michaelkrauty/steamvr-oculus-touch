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

pub mod ipc {
    //! The control channel: one stream per request, private to this user (a Unix-domain socket on
    //! Linux, a named pipe on Windows), and the lock that keeps one daemon per user.
    use std::fs::File;
    use std::io::{self, Read, Write};
    use std::path::{Path, PathBuf};
    use std::time::Duration;

    use super::os;

    /// Where this user's daemon keeps its control endpoint and locks.
    pub fn runtime_dir() -> PathBuf {
        os::runtime_dir()
    }

    /// An exclusive lock on `path` (created if missing), held until the file is dropped; fails at
    /// once (WouldBlock) while another process holds it.
    pub fn lock_exclusive(path: &Path) -> io::Result<File> {
        os::lock_exclusive(path)
    }

    pub struct Connection(pub(super) os::Connection);

    impl Connection {
        /// Connects to the listener at `endpoint`; NotFound when nobody listens there, and
        /// PermissionDenied when another account does.
        pub fn connect(endpoint: &Path) -> io::Result<Connection> {
            os::Connection::connect(endpoint).map(Connection)
        }

        /// How long a read or a write may wait (None: as long as it takes).
        pub fn set_timeout(&mut self, timeout: Option<Duration>) -> io::Result<()> {
            self.0.set_timeout(timeout)
        }
    }

    impl Read for Connection {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            self.0.read(buf)
        }
    }

    impl Write for Connection {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.0.write(buf)
        }

        fn flush(&mut self) -> io::Result<()> {
            self.0.flush()
        }
    }

    pub struct Listener(pub(super) os::Listener);

    impl Listener {
        /// Listens at `endpoint`, reachable by this user only. A stale endpoint left by a crashed
        /// owner is replaced: hold the daemon's lock first, so there is no live one.
        pub fn bind(endpoint: &Path) -> io::Result<Listener> {
            os::Listener::bind(endpoint).map(Listener)
        }

        /// The next connection; with `wait` false, WouldBlock when none is pending.
        pub fn accept(&mut self, wait: bool) -> io::Result<Connection> {
            self.0.accept(wait).map(Connection)
        }
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
