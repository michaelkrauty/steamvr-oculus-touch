// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! Linux implementations.

use std::fs::{DirBuilder, File, OpenOptions};
use std::io;
use std::os::unix::ffi::OsStrExt;
use std::os::unix::fs::{DirBuilderExt, OpenOptionsExt};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};

pub fn dylib_open(path: &Path) -> *mut std::ffi::c_void {
    let Ok(name) = std::ffi::CString::new(path.as_os_str().as_bytes()) else {
        return std::ptr::null_mut();
    };
    // SAFETY: a NUL-terminated name.
    unsafe { libc::dlopen(name.as_ptr(), libc::RTLD_NOW | libc::RTLD_LOCAL) }
}

pub fn dylib_symbol(lib: *mut std::ffi::c_void, name: &std::ffi::CStr) -> *mut std::ffi::c_void {
    // SAFETY: a handle from dlopen and a NUL-terminated name.
    unsafe { libc::dlsym(lib, name.as_ptr()) }
}

pub fn dylib_close(lib: *mut std::ffi::c_void) {
    // SAFETY: a handle from dlopen, closed once.
    unsafe { libc::dlclose(lib) };
}

pub fn open_read_nofollow(path: &Path) -> io::Result<File> {
    // O_NONBLOCK makes a FIFO fail the caller's regular-file check instead of blocking
    OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
        .open(path)
}

pub fn open_read_nonblocking(path: &Path) -> io::Result<File> {
    OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NONBLOCK)
        .open(path)
}

pub fn create_private(path: &Path) -> io::Result<File> {
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .custom_flags(libc::O_NOFOLLOW)
        .open(path)
}

pub fn create_private_dir_all(path: &Path) -> io::Result<()> {
    DirBuilder::new().recursive(true).mode(0o700).create(path)
}

pub fn sync_file(path: &Path) -> io::Result<()> {
    File::open(path)?.sync_all()
}

pub fn sync_dir(dir: &Path) -> io::Result<()> {
    File::open(dir)?.sync_all()
}

pub fn rename_replacing(from: &Path, to: &Path) -> io::Result<()> {
    std::fs::rename(from, to)
}

pub fn swap_dir(stage: &Path, dir: &Path, exists: bool) -> io::Result<()> {
    let c = |p: &Path| {
        std::ffi::CString::new(p.as_os_str().as_bytes())
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))
    };
    let (from, to) = (c(stage)?, c(dir)?);
    let flags = if exists {
        libc::RENAME_EXCHANGE
    } else {
        libc::RENAME_NOREPLACE
    };
    // SAFETY: both paths are valid NUL-terminated strings.
    let result = unsafe {
        libc::renameat2(
            libc::AT_FDCWD,
            from.as_ptr(),
            libc::AT_FDCWD,
            to.as_ptr(),
            flags,
        )
    };
    if result != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

pub fn now_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: valid out-pointer.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

fn cname(name: &str) -> io::Result<std::ffi::CString> {
    std::ffi::CString::new(name)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "shm name contains NUL"))
}

// Use the same mount namespace as POSIX shared memory: runtime-dir and XDG overrides
// must not let two writers own one segment. Never unlink a lock file (that splits locks).
pub(crate) fn private_lock_directory(path: &Path) -> io::Result<File> {
    use std::os::unix::fs::MetadataExt;
    match DirBuilder::new().mode(0o700).create(path) {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {}
        Err(e) => return Err(e),
    }
    let dir = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(path)?;
    let meta = dir.metadata()?;
    // SAFETY: geteuid has no arguments and cannot fail.
    if !meta.is_dir() || meta.uid() != unsafe { libc::geteuid() } || meta.mode() & 0o777 != 0o700 {
        return Err(io::Error::new(
            io::ErrorKind::PermissionDenied,
            "shared-memory lock directory must be owned by this user with mode 0700",
        ));
    }
    Ok(dir)
}

fn lock_shm_name(name: &str) -> io::Result<File> {
    use std::os::fd::{AsRawFd, FromRawFd};
    use std::os::unix::fs::MetadataExt;
    let component = name
        .strip_prefix('/')
        .filter(|s| !s.is_empty() && !s.contains('/') && *s != "." && *s != "..")
        .ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "shared-memory name must be / followed by a single name",
            )
        })?;
    let component = cname(component)?;
    // SAFETY: geteuid has no arguments and cannot fail.
    let uid = unsafe { libc::geteuid() };
    let dir = private_lock_directory(Path::new(&format!("/dev/shm/cv1touch-locks-{uid}")))?;
    // SAFETY: live directory fd and NUL-terminated single component; no symlinks followed.
    let fd = unsafe {
        libc::openat(
            dir.as_raw_fd(),
            component.as_ptr(),
            libc::O_CREAT | libc::O_RDWR | libc::O_CLOEXEC | libc::O_NOFOLLOW,
            0o600,
        )
    };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: fresh owned descriptor, closed on every error/return through File.
    let file = unsafe { File::from_raw_fd(fd) };
    let meta = file.metadata()?;
    if !meta.is_file() || meta.uid() != uid || meta.mode() & 0o777 != 0o600 || meta.nlink() != 1 {
        return Err(io::Error::new(
            io::ErrorKind::PermissionDenied,
            "invalid shared-memory ownership lock",
        ));
    }
    // SAFETY: valid fd, held for the segment's lifetime including unlink on drop.
    if unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(file)
}

pub struct OwnedSegment {
    ptr: *mut u8,
    len: usize,
    name: std::ffi::CString,
    unlinked: AtomicBool,
    _ownership: File,
}

// SAFETY: a shared mapping; callers synchronise access to its contents.
unsafe impl Send for OwnedSegment {}
unsafe impl Sync for OwnedSegment {}

impl OwnedSegment {
    pub fn create(name: &str, len: usize) -> io::Result<OwnedSegment> {
        let ownership = lock_shm_name(name)?;
        let cname = cname(name)?;
        // SAFETY: plain POSIX calls; the new mapping is exclusively ours.
        unsafe {
            libc::shm_unlink(cname.as_ptr());
            let fd = libc::shm_open(
                cname.as_ptr(),
                libc::O_CREAT | libc::O_EXCL | libc::O_RDWR | libc::O_CLOEXEC,
                0o600,
            );
            if fd < 0 {
                return Err(io::Error::last_os_error());
            }
            if libc::ftruncate(fd, len as libc::off_t) < 0 {
                let e = io::Error::last_os_error();
                libc::close(fd);
                libc::shm_unlink(cname.as_ptr());
                return Err(e);
            }
            let p = libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            );
            libc::close(fd);
            if p == libc::MAP_FAILED {
                let e = io::Error::last_os_error();
                libc::shm_unlink(cname.as_ptr());
                return Err(e);
            }
            Ok(OwnedSegment {
                ptr: p as *mut u8,
                len,
                name: cname,
                unlinked: AtomicBool::new(false),
                _ownership: ownership,
            })
        }
    }

    pub fn ptr(&self) -> *mut u8 {
        self.ptr
    }

    pub fn wake(&self, word: *mut u32) {
        // SAFETY: FUTEX_WAKE on a word inside the shared mapping.
        unsafe {
            libc::syscall(
                libc::SYS_futex,
                word,
                libc::FUTEX_WAKE,
                i32::MAX,
                std::ptr::null::<libc::timespec>(),
                std::ptr::null::<u32>(),
                0,
            );
        }
    }

    pub fn unlink(&self) {
        if !self.unlinked.swap(true, Ordering::AcqRel) {
            // SAFETY: our name, NUL-terminated.
            unsafe { libc::shm_unlink(self.name.as_ptr()) };
        }
    }
}

impl Drop for OwnedSegment {
    fn drop(&mut self) {
        self.unlink();
        // SAFETY: our mapping.
        unsafe { libc::munmap(self.ptr as *mut libc::c_void, self.len) };
    }
}

pub struct OpenedSegment {
    ptr: *mut u8,
    len: usize,
}

// SAFETY: a read-only shared mapping.
unsafe impl Send for OpenedSegment {}
unsafe impl Sync for OpenedSegment {}

impl OpenedSegment {
    pub fn open(name: &str, len: usize) -> Result<OpenedSegment, super::shm::OpenError> {
        use super::shm::OpenError;
        let cname = cname(name).map_err(OpenError::Io)?;
        // SAFETY: read-only mapping, sized before it is touched.
        unsafe {
            let fd = libc::shm_open(cname.as_ptr(), libc::O_RDONLY | libc::O_CLOEXEC, 0);
            if fd < 0 {
                let e = io::Error::last_os_error();
                return Err(if e.kind() == io::ErrorKind::NotFound {
                    OpenError::NotFound
                } else {
                    OpenError::Io(e)
                });
            }
            // A segment not yet fully sized would SIGBUS on first access: the owner creates it,
            // then sizes it.
            let mut st: libc::stat = std::mem::zeroed();
            if libc::fstat(fd, &mut st) < 0 || (st.st_size as usize) < len {
                libc::close(fd);
                return Err(OpenError::TooSmall);
            }
            let p = libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ,
                libc::MAP_SHARED,
                fd,
                0,
            );
            libc::close(fd);
            if p == libc::MAP_FAILED {
                return Err(OpenError::Io(io::Error::last_os_error()));
            }
            Ok(OpenedSegment {
                ptr: p as *mut u8,
                len,
            })
        }
    }

    pub fn ptr(&self) -> *mut u8 {
        self.ptr
    }

    pub fn wait(&self, word: *mut u32, last_seen: u32, timeout_ns: u64) {
        let ts = libc::timespec {
            tv_sec: (timeout_ns / 1_000_000_000) as libc::time_t,
            tv_nsec: (timeout_ns % 1_000_000_000) as libc::c_long,
        };
        // SAFETY: FUTEX_WAIT on a word inside the shared mapping.
        unsafe {
            libc::syscall(
                libc::SYS_futex,
                word,
                libc::FUTEX_WAIT,
                last_seen,
                &ts as *const libc::timespec,
                std::ptr::null::<u32>(),
                0,
            );
        }
    }
}

impl Drop for OpenedSegment {
    fn drop(&mut self) {
        // SAFETY: our mapping.
        unsafe { libc::munmap(self.ptr as *mut libc::c_void, self.len) };
    }
}

pub fn runtime_dir() -> std::path::PathBuf {
    std::env::var_os("XDG_RUNTIME_DIR")
        .map(std::path::PathBuf::from)
        // SAFETY: getuid has no arguments and cannot fail.
        .unwrap_or_else(|| format!("/run/user/{}", unsafe { libc::getuid() }).into())
        .join("cv1touch")
}

pub fn lock_exclusive(path: &Path) -> io::Result<File> {
    use std::os::fd::AsRawFd;
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .mode(0o600)
        .open(path)?;
    // SAFETY: valid fd owned by `file`.
    if unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(file)
}

pub struct Connection(std::os::unix::net::UnixStream);

impl Connection {
    pub fn connect(endpoint: &Path) -> io::Result<Connection> {
        std::os::unix::net::UnixStream::connect(endpoint).map(Connection)
    }

    pub fn set_timeout(&mut self, timeout: Option<std::time::Duration>) -> io::Result<()> {
        self.0.set_read_timeout(timeout)?;
        self.0.set_write_timeout(timeout)
    }

    pub fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        io::Read::read(&mut self.0, buf)
    }

    pub fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        io::Write::write(&mut self.0, buf)
    }

    pub fn flush(&mut self) -> io::Result<()> {
        io::Write::flush(&mut self.0)
    }
}

pub struct Listener(std::os::unix::net::UnixListener);

impl Listener {
    pub fn bind(endpoint: &Path) -> io::Result<Listener> {
        use std::os::unix::fs::PermissionsExt;
        if endpoint.exists() {
            if std::os::unix::net::UnixStream::connect(endpoint).is_ok() {
                return Err(io::Error::new(
                    io::ErrorKind::AddrInUse,
                    "another process listens there",
                ));
            }
            // nobody answers: left by a crashed owner
            std::fs::remove_file(endpoint)?;
        }
        let listener = std::os::unix::net::UnixListener::bind(endpoint)?;
        std::fs::set_permissions(endpoint, std::fs::Permissions::from_mode(0o600))?;
        Ok(Listener(listener))
    }

    pub fn accept(&mut self, wait: bool) -> io::Result<Connection> {
        self.0.set_nonblocking(!wait)?;
        let (stream, _) = self.0.accept()?;
        stream.set_nonblocking(false)?;
        Ok(Connection(stream))
    }
}

pub struct Waker(std::os::fd::OwnedFd);

impl Waker {
    pub fn new() -> io::Result<Waker> {
        use std::os::fd::FromRawFd;
        // SAFETY: no pointers; the descriptor is owned below.
        let fd = unsafe { libc::eventfd(0, libc::EFD_NONBLOCK | libc::EFD_CLOEXEC) };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: a fresh descriptor nothing else owns.
        Ok(Waker(unsafe { std::os::fd::OwnedFd::from_raw_fd(fd) }))
    }

    pub fn wake(&self) {
        use std::os::fd::AsRawFd;
        let one = 1u64;
        // SAFETY: an eventfd takes one u64.
        unsafe {
            libc::write(
                self.0.as_raw_fd(),
                &one as *const u64 as *const libc::c_void,
                8,
            )
        };
    }

    fn drain(&self) {
        use std::os::fd::AsRawFd;
        let mut v = 0u64;
        // SAFETY: an eventfd reads as one u64.
        unsafe {
            libc::read(
                self.0.as_raw_fd(),
                &mut v as *mut u64 as *mut libc::c_void,
                8,
            )
        };
    }
}

/// A hidraw node, read without blocking.
pub struct HidDevice {
    file: File,
    /// The last wait saw the node fail or hang up: a read with nothing left is the device gone.
    hung_up: bool,
}

impl HidDevice {
    pub fn open(path: &Path) -> io::Result<HidDevice> {
        let file = OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(path)?;
        Ok(HidDevice {
            file,
            hung_up: false,
        })
    }

    pub fn wait(
        &mut self,
        waker: &Waker,
        timeout: Option<std::time::Duration>,
    ) -> io::Result<crate::platform::hid::Woke> {
        use std::os::fd::AsRawFd;
        let mut fds = [
            libc::pollfd {
                fd: self.file.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            },
            libc::pollfd {
                fd: waker.0.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            },
        ];
        let ms = timeout.map_or(-1, |t| t.as_millis().min(i32::MAX as u128) as libc::c_int);
        // SAFETY: a valid pollfd array.
        if unsafe { libc::poll(fds.as_mut_ptr(), 2, ms) } < 0 {
            let e = io::Error::last_os_error();
            return if e.kind() == io::ErrorKind::Interrupted {
                Ok(Default::default())
            } else {
                Err(e)
            };
        }
        self.hung_up = fds[0].revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0;
        let waker_fired = fds[1].revents & libc::POLLIN != 0;
        if waker_fired {
            waker.drain();
        }
        Ok(crate::platform::hid::Woke {
            report: fds[0].revents & libc::POLLIN != 0,
            waker: waker_fired,
        })
    }

    pub fn try_read(&mut self, buf: &mut [u8]) -> io::Result<Option<usize>> {
        use std::io::Read;
        match self.file.read(buf) {
            Ok(0) => Err(io::Error::new(io::ErrorKind::UnexpectedEof, "end of file")),
            Ok(n) => Ok(Some(n)),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock && !self.hung_up => Ok(None),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "the device hung up",
            )),
            Err(e) => Err(e),
        }
    }
}

pub fn headset_interface(interface: u8) -> Option<std::path::PathBuf> {
    headset_hidraw(Path::new("/sys/class/hidraw"), interface)
}

/// The hidraw node of the headset's (Rift CV1 2833:0031) HID interface `interface` among the
/// nodes under `sys_class_hidraw` (normally /sys/class/hidraw).
fn headset_hidraw(sys_class_hidraw: &Path, interface: u8) -> Option<std::path::PathBuf> {
    let mut names: Vec<String> = std::fs::read_dir(sys_class_hidraw)
        .ok()?
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    let suffix = format!("/input{interface}");
    names
        .into_iter()
        .find(|n| {
            let Ok(uevent) =
                std::fs::read_to_string(sys_class_hidraw.join(n).join("device/uevent"))
            else {
                return false;
            };
            uevent.lines().any(|l| l == "HID_ID=0003:00002833:00000031")
                && uevent
                    .lines()
                    .any(|l| l.starts_with("HID_PHYS=") && l.ends_with(&suffix))
        })
        .map(|n| Path::new("/dev").join(n))
}

/// SIGINT and SIGTERM, blocked and read from a signalfd.
pub struct ShutdownRequests(std::os::fd::OwnedFd);

impl ShutdownRequests {
    pub fn take() -> io::Result<ShutdownRequests> {
        use std::os::fd::FromRawFd;
        // SAFETY: plain signal-mask setup; the descriptor is owned below.
        let fd = unsafe {
            let mut mask: libc::sigset_t = std::mem::zeroed();
            libc::sigemptyset(&mut mask);
            libc::sigaddset(&mut mask, libc::SIGINT);
            libc::sigaddset(&mut mask, libc::SIGTERM);
            libc::pthread_sigmask(libc::SIG_BLOCK, &mask, std::ptr::null_mut());
            libc::signalfd(-1, &mask, libc::SFD_CLOEXEC)
        };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: a fresh descriptor nothing else owns.
        Ok(ShutdownRequests(unsafe {
            std::os::fd::OwnedFd::from_raw_fd(fd)
        }))
    }

    pub fn wait(&self, timeout: std::time::Duration) -> Option<String> {
        use std::os::fd::AsRawFd;
        let mut fds = libc::pollfd {
            fd: self.0.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        let ms = timeout.as_millis().min(i32::MAX as u128) as libc::c_int;
        // SAFETY: one valid pollfd.
        if unsafe { libc::poll(&mut fds, 1, ms) } <= 0 || fds.revents & libc::POLLIN == 0 {
            return None;
        }
        // SAFETY: reading one siginfo from our signalfd.
        let mut info: libc::signalfd_siginfo = unsafe { std::mem::zeroed() };
        let n = unsafe {
            libc::read(
                self.0.as_raw_fd(),
                &mut info as *mut _ as *mut libc::c_void,
                std::mem::size_of::<libc::signalfd_siginfo>(),
            )
        };
        (n == std::mem::size_of::<libc::signalfd_siginfo>() as isize)
            .then(|| format!("signal {}", info.ssi_signo))
    }
}

/// SIGTERM to this thread, as `kill` would send the daemon.
#[cfg(test)]
pub fn request_shutdown() {
    // SAFETY: a signal to the calling thread, which blocks it (ShutdownRequests::take).
    unsafe { libc::raise(libc::SIGTERM) };
}

/// A device fed through a SOCK_SEQPACKET pair, which delivers one message per read as hidraw
/// delivers one report.
#[cfg(test)]
pub fn fake_hid() -> (HidDevice, std::os::unix::net::UnixDatagram) {
    use std::os::fd::{FromRawFd, OwnedFd};
    let mut fds = [0; 2];
    // SAFETY: a valid two-element array.
    let ret = unsafe {
        libc::socketpair(
            libc::AF_UNIX,
            libc::SOCK_SEQPACKET | libc::SOCK_NONBLOCK | libc::SOCK_CLOEXEC,
            0,
            fds.as_mut_ptr(),
        )
    };
    assert_eq!(ret, 0, "{}", io::Error::last_os_error());
    // SAFETY: fresh descriptors owned below.
    let (device, sender) = unsafe { (OwnedFd::from_raw_fd(fds[0]), OwnedFd::from_raw_fd(fds[1])) };
    (
        HidDevice {
            file: File::from(device),
            hung_up: false,
        },
        std::os::unix::net::UnixDatagram::from(sender),
    )
}

#[cfg(test)]
pub fn fake_hid_send(sender: &mut std::os::unix::net::UnixDatagram, report: &[u8]) {
    assert_eq!(sender.send(report).unwrap(), report.len());
}

pub type ThreadId = libc::pid_t;

pub fn current_thread() -> ThreadId {
    // SAFETY: no arguments.
    unsafe { libc::gettid() }
}

pub fn thread_named(name: &str) -> Option<ThreadId> {
    std::fs::read_dir("/proc/self/task")
        .ok()?
        .filter_map(|e| e.ok())
        .find_map(|e| {
            let comm = std::fs::read_to_string(e.path().join("comm")).ok()?;
            (comm.trim_end() == name)
                .then(|| e.file_name().to_str()?.parse().ok())
                .flatten()
        })
}

/// rtkit's ceiling on RLIMIT_RTTIME (us): it grants real time only to a process that bounds how
/// long a real-time thread runs without blocking. Past it the kernel kills the process.
const RTTIME_LIMIT_US: u64 = 200_000;

pub fn make_realtime(thread: ThreadId, priority: u32) -> Result<String, String> {
    // SAFETY: plain rlimit structs passed by pointer.
    unsafe {
        let mut lim: libc::rlimit = std::mem::zeroed();
        libc::getrlimit(libc::RLIMIT_RTTIME, &mut lim);
        if lim.rlim_max > RTTIME_LIMIT_US {
            let bounded = libc::rlimit {
                rlim_cur: RTTIME_LIMIT_US,
                rlim_max: RTTIME_LIMIT_US,
            };
            if libc::setrlimit(libc::RLIMIT_RTTIME, &bounded) != 0 {
                return Err(format!(
                    "bounding RLIMIT_RTTIME: {}",
                    io::Error::last_os_error()
                ));
            }
        }
    }
    let out = std::process::Command::new("busctl")
        .args([
            "call",
            "org.freedesktop.RealtimeKit1",
            "/org/freedesktop/RealtimeKit1",
            "org.freedesktop.RealtimeKit1",
            "MakeThreadRealtimeWithPID",
            "ttu",
        ])
        .args([
            std::process::id().to_string(),
            thread.to_string(),
            priority.to_string(),
        ])
        .output()
        .map_err(|e| format!("busctl: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "rtkit: {}",
            String::from_utf8_lossy(&out.stderr).trim()
        ));
    }
    Ok(format!("SCHED_RR {priority}"))
}

pub fn allowed_cpus(thread: ThreadId) -> io::Result<Vec<usize>> {
    // SAFETY: valid output pointer; only queries the requested thread.
    unsafe {
        let mut allowed: libc::cpu_set_t = std::mem::zeroed();
        if libc::sched_getaffinity(thread, std::mem::size_of_val(&allowed), &mut allowed) != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok((0..libc::CPU_SETSIZE as usize)
            .filter(|&cpu| libc::CPU_ISSET(cpu, &allowed))
            .collect())
    }
}

pub fn set_cpus(thread: ThreadId, cpus: &[usize]) -> io::Result<()> {
    // SAFETY: cpu_set_t's all-zero representation is the empty set.
    let mut selected: libc::cpu_set_t = unsafe { std::mem::zeroed() };
    for &cpu in cpus {
        if cpu >= libc::CPU_SETSIZE as usize {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("no CPU {cpu}"),
            ));
        }
        // SAFETY: checked against CPU_SETSIZE.
        unsafe { libc::CPU_SET(cpu, &mut selected) };
    }
    // SAFETY: an initialized set.
    if unsafe { libc::sched_setaffinity(thread, std::mem::size_of_val(&selected), &selected) } != 0
    {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

pub fn lock_memory() -> io::Result<()> {
    // SAFETY: no pointers; only marks this process's mappings.
    if unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE | libc::MCL_ONFAULT) } != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

pub fn voluntary_switches() -> Option<i64> {
    // SAFETY: a zeroed rusage filled for the calling thread.
    let mut u: libc::rusage = unsafe { std::mem::zeroed() };
    (unsafe { libc::getrusage(libc::RUSAGE_THREAD, &mut u) } == 0).then_some(u.ru_nvcsw)
}

pub fn random_u64() -> io::Result<u64> {
    let mut v = 0u64;
    // SAFETY: valid buffer of 8 bytes.
    let n = unsafe { libc::getrandom(&mut v as *mut u64 as *mut libc::c_void, 8, 0) };
    if n == 8 {
        Ok(v)
    } else {
        Err(io::Error::last_os_error())
    }
}

pub fn opt_out_of_throttling() -> io::Result<()> {
    Ok(())
}

pub fn append_private(path: &Path) -> io::Result<File> {
    OpenOptions::new()
        .create(true)
        .append(true)
        .mode(0o600)
        .custom_flags(libc::O_NOFOLLOW)
        .open(path)
}

pub fn flush_console_input() {
    // SAFETY: a plain terminal call on stdin; it fails harmlessly when stdin is not a terminal.
    unsafe { libc::tcflush(0, libc::TCIFLUSH) };
}

static INTERRUPTED: AtomicBool = AtomicBool::new(false);

extern "C" fn on_interrupt(_: libc::c_int) {
    INTERRUPTED.store(true, Ordering::SeqCst);
}

pub fn catch_interrupts() {
    // SAFETY: installing an async-signal-safe handler (one atomic store).
    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = on_interrupt as *const () as usize;
        libc::sigemptyset(&mut sa.sa_mask);
        for sig in [libc::SIGINT, libc::SIGTERM, libc::SIGHUP] {
            libc::sigaction(sig, &sa, std::ptr::null_mut());
        }
    }
}

pub fn interrupted() -> bool {
    INTERRUPTED.load(Ordering::SeqCst)
}

/// Copies of the original stdout and stderr, put back when dropped.
pub struct OutputRedirect([i32; 2]);

impl OutputRedirect {
    pub fn to(file: &File) -> io::Result<OutputRedirect> {
        use std::io::Write;
        use std::os::unix::io::AsRawFd;
        let _ = io::stdout().flush();
        // SAFETY: plain descriptor duplication; the copies are owned by the guard or closed here.
        let saved = unsafe { [libc::dup(1), libc::dup(2)] };
        if saved.contains(&-1) {
            let e = io::Error::last_os_error();
            for fd in saved.into_iter().filter(|&fd| fd >= 0) {
                // SAFETY: closing a descriptor just duplicated.
                unsafe { libc::close(fd) };
            }
            return Err(e);
        }
        // SAFETY: redirecting our own stdout and stderr; the guard undoes it.
        unsafe {
            libc::dup2(file.as_raw_fd(), 1);
            libc::dup2(file.as_raw_fd(), 2);
        }
        Ok(OutputRedirect(saved))
    }
}

impl Drop for OutputRedirect {
    fn drop(&mut self) {
        use std::io::Write;
        let _ = io::stdout().flush();
        // SAFETY: restoring and closing descriptors this guard owns.
        unsafe {
            libc::dup2(self.0[0], 1);
            libc::dup2(self.0[1], 2);
            libc::close(self.0[0]);
            libc::close(self.0[1]);
        }
    }
}

/// The kernel keeps a process's name (comm) in 16 bytes, NUL included; pgrep -x matches no
/// longer pattern.
const PROCESS_NAME_MAX: usize = 15;

/// `name` is plain (no regular expression characters): pgrep matches it as written.
pub fn process_running(name: &str) -> io::Result<bool> {
    let status = std::process::Command::new("pgrep")
        .args(["-x", "--"])
        .arg(&name[..name.len().min(PROCESS_NAME_MAX)])
        .stdout(std::process::Stdio::null())
        .status()
        .map_err(|e| match e.kind() {
            io::ErrorKind::NotFound => {
                io::Error::new(e.kind(), "pgrep is not installed (it comes with procps)")
            }
            _ => io::Error::new(e.kind(), format!("pgrep: {e}")),
        })?;
    match status.code() {
        Some(0) => Ok(true),
        Some(1) => Ok(false),
        _ => Err(io::Error::other(format!("pgrep exited {status}"))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_the_headset_interfaces_among_the_hidraw_nodes() {
        let sys = std::env::temp_dir().join(format!("cv1t-hidraw-{}", std::process::id()));
        let node = |name: &str, uevent: &str| {
            let dir = sys.join(name).join("device");
            std::fs::create_dir_all(&dir).unwrap();
            std::fs::write(dir.join("uevent"), uevent).unwrap();
        };
        node(
            "hidraw3",
            "HID_ID=0003:0000046D:0000C262\nHID_PHYS=usb-0000:00:14.0-5/input1\n",
        );
        node("hidraw27", "DRIVER=hid-generic\nHID_ID=0003:00002833:00000031\nHID_NAME=Oculus VR, Inc. Rift\nHID_PHYS=usb-0000:02:00.0-1.1/input0\n");
        node("hidraw26", "DRIVER=hid-generic\nHID_ID=0003:00002833:00000031\nHID_NAME=Oculus VR, Inc. Rift\nHID_PHYS=usb-0000:02:00.0-1.1/input1\n");
        assert_eq!(
            headset_hidraw(&sys, 1),
            Some(std::path::PathBuf::from("/dev/hidraw26"))
        );
        assert_eq!(
            headset_hidraw(&sys, 0),
            Some(std::path::PathBuf::from("/dev/hidraw27"))
        );
        std::fs::remove_dir_all(sys.join("hidraw26")).unwrap();
        assert_eq!(
            headset_hidraw(&sys, 1),
            None,
            "the headset's other interface is not the radio"
        );
        std::fs::remove_dir_all(&sys).unwrap();
    }

    #[test]
    fn ownership_directory_rejects_symlinks_and_shared_permissions() {
        use std::fs::DirBuilder;
        use std::os::unix::fs::DirBuilderExt;
        use std::os::unix::fs::{symlink, PermissionsExt};
        let dir = std::env::temp_dir().join(format!("cv1t-lockdir-{}", std::process::id()));
        DirBuilder::new().mode(0o700).create(&dir).unwrap();
        let target = dir.join("target");
        let link = dir.join("link");
        DirBuilder::new().mode(0o700).create(&target).unwrap();
        symlink(&target, &link).unwrap();
        assert!(
            private_lock_directory(&link).is_err(),
            "followed a lock-directory symlink"
        );
        std::fs::set_permissions(&target, std::fs::Permissions::from_mode(0o755)).unwrap();
        assert!(
            private_lock_directory(&target).is_err(),
            "accepted a shared lock directory"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
