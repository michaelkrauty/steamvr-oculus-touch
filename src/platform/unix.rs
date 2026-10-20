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

pub fn disk_space(file: &File, _path: &Path) -> io::Result<super::fs::DiskSpace> {
    use std::os::fd::AsRawFd;
    let mut stat = std::mem::MaybeUninit::<libc::statvfs>::uninit();
    // SAFETY: an open descriptor and storage for the result; read only on success.
    if unsafe { libc::fstatvfs(file.as_raw_fd(), stat.as_mut_ptr()) } != 0 {
        return Err(io::Error::last_os_error());
    }
    let stat = unsafe { stat.assume_init() };
    Ok(super::fs::DiskSpace {
        available: stat.f_bavail.saturating_mul(stat.f_frsize),
        capacity: stat.f_blocks.saturating_mul(stat.f_frsize),
    })
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

/// An exclusive `flock`, released when this value is dropped.
///
/// Closing the descriptor is not enough. The lock belongs to the open file description, and a
/// child that another thread is starting holds a copy of that descriptor until it executes or
/// exits, so the close alone can leave the lock held.
///
/// A forked child that ran this destructor without executing would release the parent's lock.
/// This repository has no such child: a process it starts executes, or leaves through `_exit`.
#[derive(Debug)]
pub struct Lock(File);

impl Drop for Lock {
    fn drop(&mut self) {
        use std::os::fd::AsRawFd;
        // SAFETY: `self.0` owns a valid descriptor. LOCK_UN releases the lock for every copy of
        // this open file description, including one a child still holds.
        let _ = unsafe { libc::flock(self.0.as_raw_fd(), libc::LOCK_UN) };
    }
}

fn lock_shm_name(name: &str) -> io::Result<Lock> {
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
    Ok(Lock(file))
}

pub struct OwnedSegment {
    ptr: *mut u8,
    len: usize,
    name: std::ffi::CString,
    unlinked: AtomicBool,
    // Dropped after `Drop::drop` unlinks the segment: the destructor runs before the fields.
    _ownership: Lock,
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

/// The most bytes of path a socket address holds: `sun_path` also stores a terminating NUL.
pub const SOCKET_PATH_LIMIT: usize = {
    // SAFETY: all-zero is a valid sockaddr_un.
    let address: libc::sockaddr_un = unsafe { std::mem::zeroed() };
    address.sun_path.len() - 1
};

pub fn runtime_dir() -> std::path::PathBuf {
    std::env::var_os("XDG_RUNTIME_DIR")
        .map(std::path::PathBuf::from)
        // SAFETY: getuid has no arguments and cannot fail.
        .unwrap_or_else(|| format!("/run/user/{}", unsafe { libc::getuid() }).into())
        .join("cv1touch")
}

fn container_socket_path_for(uid: u32) -> std::path::PathBuf {
    format!("/dev/shm/cv1touch-control-{uid}/control.sock").into()
}

pub fn container_socket_path() -> std::path::PathBuf {
    // SAFETY: geteuid has no arguments and cannot fail.
    container_socket_path_for(unsafe { libc::geteuid() })
}

fn private_control_directory_metadata(meta: &std::fs::Metadata, uid: u32) -> bool {
    use std::os::unix::fs::MetadataExt;
    meta.is_dir() && meta.uid() == uid && meta.mode() & 0o7777 == 0o700
}

fn private_control_directory(dir: &Path, create: bool) -> io::Result<()> {
    use std::os::unix::fs::{MetadataExt, PermissionsExt};
    let created = if create {
        match DirBuilder::new().mode(0o700).create(dir) {
            Ok(()) => true,
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => false,
            Err(e) => return Err(e),
        }
    } else {
        false
    };
    let mut meta = std::fs::symlink_metadata(dir)?;
    // SAFETY: geteuid has no arguments and cannot fail.
    let uid = unsafe { libc::geteuid() };
    if created && meta.is_dir() && meta.uid() == uid {
        // mkdir's mode is masked by umask. Change only the directory just created and owned
        // by us; the sticky /dev/shm parent prevents another user from replacing it.
        std::fs::set_permissions(dir, std::fs::Permissions::from_mode(0o700))?;
        meta = std::fs::symlink_metadata(dir)?;
    }
    if !private_control_directory_metadata(&meta, uid) {
        return Err(io::Error::new(
            io::ErrorKind::PermissionDenied,
            "control socket directory must be owned by this user with mode 0700",
        ));
    }
    Ok(())
}

fn owned_control_socket(endpoint: &Path) -> io::Result<()> {
    use std::os::unix::fs::{FileTypeExt, MetadataExt};
    let meta = std::fs::symlink_metadata(endpoint)?;
    // SAFETY: geteuid has no arguments and cannot fail.
    if !meta.file_type().is_socket() || meta.uid() != unsafe { libc::geteuid() } {
        return Err(io::Error::new(
            io::ErrorKind::PermissionDenied,
            "control socket must be a socket owned by this user",
        ));
    }
    Ok(())
}

pub fn unlink_private_socket(endpoint: &Path) -> io::Result<()> {
    let remove = || {
        let dir = endpoint.parent().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "control socket needs a directory",
            )
        })?;
        private_control_directory(dir, false)?;
        owned_control_socket(endpoint)?;
        std::fs::remove_file(endpoint)
    };
    match remove() {
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
        result => result,
    }
}

pub fn lock_exclusive(path: &Path) -> io::Result<Lock> {
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
    Ok(Lock(file))
}

pub struct Connection(std::os::unix::net::UnixStream);

impl Connection {
    pub fn connect(endpoint: &Path) -> io::Result<Connection> {
        std::os::unix::net::UnixStream::connect(endpoint).map(Connection)
    }

    pub fn connect_until(endpoint: &Path, until: std::time::Instant) -> io::Result<Connection> {
        use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
        use std::time::Duration;

        if let Some(long) = super::ipc::path_too_long(endpoint) {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, long));
        }
        let bytes = endpoint.as_os_str().as_bytes();
        if bytes.contains(&0) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid control socket path",
            ));
        }
        // SAFETY: all-zero is a valid initial sockaddr_un, including its terminating NUL.
        let mut address: libc::sockaddr_un = unsafe { std::mem::zeroed() };
        address.sun_family = libc::AF_UNIX as libc::sa_family_t;
        for (to, from) in address.sun_path.iter_mut().zip(bytes) {
            *to = *from as libc::c_char;
        }
        let len = std::mem::offset_of!(libc::sockaddr_un, sun_path) + bytes.len() + 1;
        // SAFETY: no pointers; nonblocking from creation, including the connect itself.
        let fd = unsafe {
            libc::socket(
                libc::AF_UNIX,
                libc::SOCK_STREAM | libc::SOCK_NONBLOCK | libc::SOCK_CLOEXEC,
                0,
            )
        };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: a fresh descriptor owned only here.
        let fd = unsafe { OwnedFd::from_raw_fd(fd) };
        loop {
            super::ipc::remaining(until)?;
            // SAFETY: a live socket and initialized address of the supplied length.
            let connected = unsafe {
                libc::connect(
                    fd.as_raw_fd(),
                    (&address as *const libc::sockaddr_un).cast(),
                    len as libc::socklen_t,
                )
            };
            if connected == 0 {
                break;
            }
            let e = io::Error::last_os_error();
            if e.raw_os_error() == Some(libc::EISCONN) {
                break;
            }
            let mut ready = libc::pollfd {
                fd: fd.as_raw_fd(),
                events: libc::POLLOUT,
                revents: 0,
            };
            let left = super::ipc::remaining(until)?;
            let (pollfd, count, wait) = match e.raw_os_error() {
                // AF_UNIX does not queue a nonblocking connect when the backlog is full.
                // The unconnected fd is writable already: wait briefly before trying again.
                Some(libc::EAGAIN) => {
                    (std::ptr::null_mut(), 0, left.min(Duration::from_millis(10)))
                }
                Some(libc::EINPROGRESS | libc::EALREADY) => (&mut ready as *mut _, 1, left),
                Some(libc::EINTR) => continue,
                _ => return Err(e),
            };
            let ms = wait.as_millis().clamp(1, i32::MAX as u128) as libc::c_int;
            // SAFETY: either one valid pollfd or no descriptors; the timeout is bounded.
            let result = unsafe { libc::poll(pollfd, count, ms) };
            if result < 0 {
                let e = io::Error::last_os_error();
                if e.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(e);
            }
            if result > 0 {
                let mut error = 0 as libc::c_int;
                let mut len = std::mem::size_of_val(&error) as libc::socklen_t;
                // SAFETY: valid out-pointers and a live socket.
                if unsafe {
                    libc::getsockopt(
                        fd.as_raw_fd(),
                        libc::SOL_SOCKET,
                        libc::SO_ERROR,
                        (&mut error as *mut libc::c_int).cast(),
                        &mut len,
                    )
                } < 0
                {
                    return Err(io::Error::last_os_error());
                }
                if error != 0 {
                    return Err(io::Error::from_raw_os_error(error));
                }
                break;
            }
        }
        super::ipc::remaining(until)?;
        let stream = std::os::unix::net::UnixStream::from(fd);
        stream.set_nonblocking(false)?;
        Ok(Connection(stream))
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

// Shut down before close. A child another thread is starting holds a copy of this descriptor
// until it executes, and close alone would leave that copy accepting. The socket is not cloned:
// the field is private, and there is no try_clone. Drop means stop serving (the accept thread,
// both bind paths, and the tests). A forked child that ran this destructor without executing
// would stop the parent's listener; this repository has no such child.
impl Drop for Listener {
    fn drop(&mut self) {
        use std::os::fd::AsRawFd;
        // SAFETY: self.0 owns the listening socket. SHUT_RDWR applies to every copy of it.
        let _ = unsafe { libc::shutdown(self.0.as_raw_fd(), libc::SHUT_RDWR) };
    }
}

fn probe_control_socket(endpoint: &Path) -> io::Result<()> {
    use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
    // SAFETY: all-zero is a valid initial sockaddr_un; fields and pathname are set below.
    let mut addr: libc::sockaddr_un = unsafe { std::mem::zeroed() };
    let path = endpoint.as_os_str().as_bytes();
    if path.len() >= addr.sun_path.len() || path.contains(&0) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "invalid socket path",
        ));
    }
    addr.sun_family = libc::AF_UNIX as libc::sa_family_t;
    for (to, from) in addr.sun_path.iter_mut().zip(path) {
        *to = *from as libc::c_char;
    }
    // SAFETY: no pointers; a fresh descriptor owned below. A full backlog must not block startup.
    let fd = unsafe {
        libc::socket(
            libc::AF_UNIX,
            libc::SOCK_STREAM | libc::SOCK_NONBLOCK | libc::SOCK_CLOEXEC,
            0,
        )
    };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: the fresh descriptor is owned exactly once.
    let fd = unsafe { OwnedFd::from_raw_fd(fd) };
    // SAFETY: addr is initialized and valid for the supplied length.
    let result = unsafe {
        libc::connect(
            fd.as_raw_fd(),
            &addr as *const _ as *const libc::sockaddr,
            std::mem::size_of_val(&addr) as libc::socklen_t,
        )
    };
    if result < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

impl Listener {
    pub fn bind_private(endpoint: &Path) -> io::Result<Listener> {
        use std::os::unix::fs::PermissionsExt;
        use std::os::unix::net::UnixListener;
        let dir = endpoint.parent().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "control socket needs a directory",
            )
        })?;
        private_control_directory(dir, true)?;
        let listener = match UnixListener::bind(endpoint) {
            Ok(listener) => listener,
            Err(e) if e.kind() == io::ErrorKind::AddrInUse => {
                owned_control_socket(endpoint)?;
                match probe_control_socket(endpoint) {
                    Err(e) if e.kind() == io::ErrorKind::ConnectionRefused => {
                        // The caller holds the instance lock: this owned socket is stale.
                        unlink_private_socket(endpoint)?;
                        UnixListener::bind(endpoint)?
                    }
                    Err(e) => return Err(e),
                    Ok(_) => return Err(e),
                }
            }
            Err(e) => return Err(e),
        };
        if let Err(e) = std::fs::set_permissions(endpoint, std::fs::Permissions::from_mode(0o600)) {
            let _ = unlink_private_socket(endpoint);
            return Err(e);
        }
        Ok(Listener(listener))
    }

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

pub fn console_closes_with_us() -> bool {
    false
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

/// The kernel keeps a process's name (comm) in 16 bytes, NUL included.
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

/// Read the kernel's names without forking: the daemon has locked memory and real-time threads.
pub fn process_running_without_child(name: &str) -> io::Result<bool> {
    process_running_in(Path::new("/proc"), name)
}

fn process_running_in(proc: &Path, name: &str) -> io::Result<bool> {
    let name = &name.as_bytes()[..name.len().min(PROCESS_NAME_MAX)];
    for entry in std::fs::read_dir(proc)? {
        let entry = entry?;
        let pid = entry.file_name();
        if pid.is_empty() || !pid.as_bytes().iter().all(u8::is_ascii_digit) {
            continue;
        }
        match std::fs::read(entry.path().join("comm")) {
            Ok(comm) if comm.strip_suffix(b"\n").unwrap_or(&comm) == name => return Ok(true),
            Ok(_) => {}
            // Processes can exit during the scan, or be hidden by /proc's access policy, as
            // with pgrep. Other errors (including exhausted descriptors) are not "not running".
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::NotFound | io::ErrorKind::PermissionDenied
                ) || e.raw_os_error() == Some(libc::ESRCH) => {}
            Err(e) => return Err(e),
        }
    }
    Ok(false)
}

pub fn end_with_this_process(command: &mut std::process::Command) -> io::Result<()> {
    use std::os::unix::process::CommandExt;
    let starter = std::process::id() as libc::pid_t;
    // SAFETY: prctl and getppid are async-signal-safe, as code between fork and exec must be.
    unsafe {
        command.pre_exec(move || {
            if libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGTERM) == -1 {
                return Err(io::Error::last_os_error());
            }
            // the starter may have ended before the request: then no signal would come
            if libc::getppid() != starter {
                return Err(io::Error::other("its starter ended"));
            }
            Ok(())
        });
    }
    Ok(())
}

pub fn without_window(_command: &mut std::process::Command) {}

pub fn play_wav(wav: &[u8]) {
    use std::io::Write;
    static PLAYED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let path = std::env::temp_dir().join(format!(
        "cv1touch-sound-{}-{}.wav",
        std::process::id(),
        PLAYED.fetch_add(1, Ordering::Relaxed)
    ));
    // a new file of this user's only: a link or a file someone else put there is left alone
    let Ok(mut file) = create_private(&path) else {
        return;
    };
    if file.write_all(wav).is_ok() {
        drop(file);
        let _ = std::process::Command::new("paplay")
            .arg(&path)
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .status();
    }
    let _ = std::fs::remove_file(&path);
}

pub fn usb_devices(vendor: u16) -> io::Result<Vec<crate::platform::usb::Node>> {
    usb_devices_in(Path::new("/sys/bus/usb/devices"), vendor)
}

pub fn usb_sensors() -> io::Result<Vec<crate::platform::usb::Sensor>> {
    usb_sensors_in(Path::new("/sys/bus/usb/devices"), Path::new("/dev/bus/usb"))
}

fn sysfs_read_at(directory: &File, field: &str) -> io::Result<String> {
    use std::io::Read;
    use std::os::fd::{AsRawFd, FromRawFd};
    let field = std::ffi::CString::new(field)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
    // SAFETY: live directory fd and NUL-terminated relative metadata name. The owned
    // File closes the new fd once; openat never substitutes a replacement directory.
    let fd = unsafe {
        libc::openat(
            directory.as_raw_fd(),
            field.as_ptr(),
            libc::O_RDONLY | libc::O_CLOEXEC,
        )
    };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    let mut text = String::new();
    unsafe { File::from_raw_fd(fd) }.read_to_string(&mut text)?;
    Ok(text)
}

/// One sysfs entry is still enumerating or leaving. That is not a present sensor, and it
/// must not fail the scan for every other sensor.
fn sensor_entry_gone(error: &io::Error) -> bool {
    error.kind() == io::ErrorKind::NotFound || error.raw_os_error() == Some(libc::ENODEV)
}

fn usb_sensors_in(root: &Path, dev: &Path) -> io::Result<Vec<crate::platform::usb::Sensor>> {
    use crate::platform::usb::{sensor_serial, Sensor};
    use std::os::unix::fs::MetadataExt;
    let mut sensors = Vec::new();
    for entry in std::fs::read_dir(root)? {
        let entry = entry?;
        let name = entry.file_name();
        let Some(_) = name.to_str().filter(|s| !s.contains(':')) else {
            continue;
        };
        let dir = entry.path();
        let read = |field| std::fs::read_to_string(dir.join(field)).map(|s| s.trim().to_string());
        let vendor = match read("idVendor") {
            Ok(vendor) => vendor,
            Err(error) if sensor_entry_gone(&error) => continue,
            Err(error) => return Err(error),
        };
        let product = match read("idProduct") {
            Ok(product) => product,
            Err(error) if sensor_entry_gone(&error) => continue,
            Err(error) => return Err(error),
        };
        if vendor != "2833" || product != "0211" {
            continue;
        }
        let directory = match OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_PATH | libc::O_DIRECTORY | libc::O_CLOEXEC)
            .open(&dir)
        {
            Ok(directory) => directory,
            Err(error) if sensor_entry_gone(&error) => continue,
            Err(error) => return Err(error),
        };
        // A live device's attributes stay put. A missing one means this device is still
        // enumerating or being removed, never a partial view of a sensor the others must wait on.
        let read = |field: &str| -> io::Result<Option<String>> {
            match sysfs_read_at(&directory, field) {
                Ok(text) => Ok(Some(text.trim().to_owned())),
                Err(error) if sensor_entry_gone(&error) => Ok(None),
                Err(error) => Err(error),
            }
        };
        let (Some(vendor), Some(product)) = (read("idVendor")?, read("idProduct")?) else {
            continue;
        };
        if vendor != "2833" || product != "0211" {
            continue;
        }
        let Some(serial) = read("serial")? else {
            continue;
        };
        if !sensor_serial(&serial) {
            continue;
        }
        let number = |field: &str| -> io::Result<Option<u16>> {
            let Some(text) = read(field)? else {
                return Ok(None);
            };
            text.parse()
                .map(Some)
                .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))
        };
        let (Some(bus), Some(address)) = (number("busnum")?, number("devnum")?) else {
            continue;
        };
        let path = dev.join(format!("{bus:03}/{address:03}"));
        let path = std::ffi::CString::new(path.as_os_str().as_bytes())
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
        // SAFETY: permission query on a valid path; this never opens or claims a USB device.
        let ready = unsafe {
            libc::faccessat(
                libc::AT_FDCWD,
                path.as_ptr(),
                libc::R_OK | libc::W_OK,
                libc::AT_EACCESS,
            )
        } == 0;
        sensors.push(Sensor {
            serial,
            // The native start record retains its own O_PATH pin, so a replacement at the
            // same bus/address/port cannot reuse that inode while the old attempt exists.
            attachment: match directory
                .metadata()
                .ok()
                .map(|m| m.ino())
                .filter(|&i| i != 0)
            {
                Some(inode) => format!("sysfs:{bus}:{address}@{inode:016x}"),
                None => format!("sysfs:{bus}:{address}@unknown"),
            },
            ready,
        });
    }
    sensors.sort_by(|a, b| a.serial.cmp(&b.serial));
    Ok(sensors)
}

/// The interface classes worth naming in a report.
fn interface_class(code: &str) -> &'static str {
    match code {
        "01" => "audio",
        "03" => "HID",
        "0e" => "video",
        "ff" => "vendor-specific",
        _ => "other",
    }
}

/// The USB devices of `vendor` under `root`, laid out as /sys/bus/usb/devices is: a device's
/// directory ("1-2") holds idVendor, idProduct, product and speed (Mb/s); an interface's
/// ("1-2:1.0") holds bInterfaceNumber and bInterfaceClass; each links to its driver. Each device
/// comes before its interfaces.
pub(crate) fn usb_devices_in(
    root: &Path,
    vendor: u16,
) -> io::Result<Vec<crate::platform::usb::Node>> {
    use crate::platform::usb::Node;
    let read = |dir: &Path, name: &str| {
        std::fs::read_to_string(dir.join(name))
            .ok()
            .map(|s| s.trim().to_string())
    };
    let hex = |dir: &Path, name: &str| -> io::Result<u16> {
        let text = std::fs::read_to_string(dir.join(name)).map_err(|error| {
            io::Error::new(
                error.kind(),
                format!("cannot read a USB device's {name}: {error}"),
            )
        })?;
        u16::from_str_radix(text.trim(), 16)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, format!("invalid USB {name}")))
    };
    let digits = |s: &str| !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit());
    let device_name = |name: &str| {
        name.strip_prefix("usb").is_some_and(digits)
            || name
                .split_once('-')
                .is_some_and(|(bus, ports)| digits(bus) && ports.split('.').all(digits))
    };
    let driver = |dir: &Path| {
        std::fs::read_link(dir.join("driver"))
            .ok()?
            .file_name()?
            .to_str()
            .map(String::from)
    };
    let names: Vec<String> = std::fs::read_dir(root)?
        .map(|entry| entry.map(|e| e.file_name().into_string().ok()))
        .collect::<io::Result<Vec<_>>>()?
        .into_iter()
        .flatten()
        .collect();
    let mut products = std::collections::BTreeMap::new();
    let mut nodes = Vec::new();
    for name in names.iter().filter(|n| device_name(n)) {
        let dir = root.join(name);
        if hex(&dir, "idVendor")? != vendor {
            continue;
        }
        let product = hex(&dir, "idProduct")?;
        products.insert(name.as_str(), product);
        let product_name = read(&dir, "product").unwrap_or_default();
        nodes.push(Node {
            id: name.clone(),
            product,
            name: match read(&dir, "speed") {
                Some(speed) => format!("{product_name} ({speed} Mb/s)"),
                None => product_name,
            },
            driver: driver(&dir),
            problem: None,
            parent: None,
        });
    }
    for name in &names {
        let Some((device, product)) = name
            .split_once(':')
            .and_then(|(device, _)| Some((device, products.get(device)?)))
        else {
            continue;
        };
        let dir = root.join(name);
        let number = read(&dir, "bInterfaceNumber").unwrap_or_default();
        let class = interface_class(&read(&dir, "bInterfaceClass").unwrap_or_default());
        nodes.push(Node {
            id: name.clone(),
            product: *product,
            name: format!("interface {number} ({class})"),
            driver: driver(&dir),
            problem: None,
            parent: Some(device.to_string()),
        });
    }
    // by device, then interface: "3-1" and its interfaces before "3-1.1", which ':' after '.'
    // would not give
    let key = |id: &str| {
        let (device, interface) = id.split_once(':').unwrap_or((id, ""));
        (device.to_string(), interface.to_string())
    };
    nodes.sort_by_key(|n| key(&n.id));
    Ok(nodes)
}

pub fn system_description() -> String {
    let release = std::fs::read_to_string("/etc/os-release")
        .ok()
        .and_then(|text| {
            text.lines()
                .find_map(|l| l.strip_prefix("PRETTY_NAME="))
                .map(|v| v.trim_matches('"').to_string())
        })
        .unwrap_or_else(|| "Linux".into());
    // SAFETY: a zeroed structure the call fills in.
    let mut uts: libc::utsname = unsafe { std::mem::zeroed() };
    let kernel = if unsafe { libc::uname(&mut uts) } == 0 {
        // SAFETY: uname NUL-terminates its fields.
        let field = |f: &[libc::c_char]| {
            unsafe { std::ffi::CStr::from_ptr(f.as_ptr()) }
                .to_string_lossy()
                .into_owned()
        };
        format!(
            "{} {} {}",
            field(&uts.sysname),
            field(&uts.release),
            field(&uts.machine)
        )
    } else {
        "an unknown kernel".into()
    };
    format!("{release}, {kernel}")
}

pub fn desktop() -> Option<std::path::PathBuf> {
    None
}

fn udev_gives_access() -> io::Error {
    io::Error::new(
        io::ErrorKind::Unsupported,
        "Linux needs no driver for the sensors: udev rules give access",
    )
}

pub fn use_winusb(_id: &str) -> io::Result<bool> {
    Err(udev_gives_access())
}

pub fn set_up_by_us(_id: &str) -> bool {
    false
}

pub fn remove_driver(_id: &str) -> io::Result<bool> {
    Err(udev_gives_access())
}

pub fn can_open(id: &str) -> io::Result<()> {
    can_open_in(
        Path::new("/sys/bus/usb/devices"),
        Path::new("/dev/bus/usb"),
        id,
    )
}

/// `can_open` with the sysfs devices directory `root` and the device files' directory `dev`:
/// the device's file is named by the bus and device numbers sysfs gives.
pub(crate) fn can_open_in(root: &Path, dev: &Path, id: &str) -> io::Result<()> {
    let number = |name: &str| -> io::Result<u32> {
        let text = std::fs::read_to_string(root.join(id).join(name))?;
        text.trim().parse().map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("{id}: {name} is not a number"),
            )
        })
    };
    let (bus, device) = (number("busnum")?, number("devnum")?);
    std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(dev.join(format!("{bus:03}")).join(format!("{device:03}")))
        .map(drop)
}

pub fn elevated() -> bool {
    // SAFETY: a plain query.
    unsafe { libc::geteuid() == 0 }
}

pub fn run_elevated(_args: &[&str]) -> io::Result<i32> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "run it with sudo instead",
    ))
}

pub fn show_file(_path: &Path) -> bool {
    false
}

pub fn alert(title: &str, text: &str) {
    eprintln!("{title}: {text}");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn process_scan_matches_exact_kernel_names_and_ignores_exited_processes() {
        let proc = std::env::temp_dir().join(format!("cv1t-process-scan-{}", std::process::id()));
        let node = |pid: &str, name: &[u8]| {
            std::fs::create_dir_all(proc.join(pid)).unwrap();
            std::fs::write(proc.join(pid).join("comm"), name).unwrap();
        };
        node("11", b"vrserver.sh\n");
        node("12", b"vrserver-helper\n");
        node("self", b"vrserver\n");
        node("14", b"VRSERVER\n");
        std::fs::create_dir_all(proc.join("13")).unwrap();
        assert!(!process_running_in(&proc, "vrserver").unwrap());
        node("15", b"vrserver\n");
        assert!(process_running_in(&proc, "vrserver").unwrap());
        let long = "cv1touch-long-process";
        node("16", format!("{}\n", &long[..PROCESS_NAME_MAX]).as_bytes());
        assert!(process_running_in(&proc, long).unwrap());
        std::fs::remove_dir_all(&proc).unwrap();
        assert!(process_running_in(&proc, "vrserver").is_err());
    }

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

    fn recycled_usb_address(incarnation_available: bool) {
        use crate::sensor_hotplug::{Monitor, DEBOUNCE_NS};
        use std::os::unix::fs::MetadataExt;
        let root = crate::socket_dir::SocketDir::new();
        let (sys, dev) = (root.path().join("sys"), root.path().join("dev"));
        let camera = sys.join("1-2.3");
        let create = || {
            std::fs::create_dir_all(&camera).unwrap();
            for (field, value) in [
                ("idVendor", "2833"),
                ("idProduct", "0211"),
                ("serial", "SENSORALPHA"),
                ("busnum", "1"),
                ("devnum", "7"),
            ] {
                std::fs::write(camera.join(field), value).unwrap();
            }
        };
        create();
        std::fs::create_dir_all(dev.join("001")).unwrap();
        std::fs::write(dev.join("001/007"), "").unwrap();
        let a_pin = std::fs::File::open(&camera).unwrap();
        let a_ino = a_pin.metadata().unwrap().ino();
        let failed_start = |ino: u64| {
            let mut raw = crate::ffi::ohmd_cv1touch_sensor_stream::default();
            for (out, byte) in raw.serial.iter_mut().zip(b"SENSORALPHA") {
                *out = *byte as std::ffi::c_char;
            }
            raw.error_kind = crate::ffi::OHMD_CV1TOUCH_STREAM_ERROR_OTHER;
            raw.error_code = -1;
            if incarnation_available {
                raw.attachment.kind = crate::ffi::OHMD_CV1TOUCH_ATTACHMENT_SYSFS;
                raw.attachment.data = [1 | (7 << 8), ino as u32, (ino >> 32) as u32];
            }
            crate::engine::SensorStreamStart::from(raw)
        };
        let mut monitor = Monitor::new(Ok(vec![]), &[]);
        monitor.before_open();
        let a_start = failed_start(a_ino);
        // No watcher scan sees A or the gap. B has the same serial, port, bus and address.
        std::fs::remove_dir_all(&camera).unwrap();
        create();
        let b_pin = std::fs::File::open(&camera).unwrap();
        let b_ino = b_pin.metadata().unwrap().ino();
        assert_ne!(
            a_ino, b_ino,
            "the retained metadata reference pins A's inode"
        );
        let b = usb_sensors_in(&sys, &dev).unwrap();
        monitor.reconcile_open(1, &[a_start], Some(&b));
        monitor.update(1, Ok(b.clone()), &[]);
        monitor.update(DEBOUNCE_NS + 1, Ok(b.clone()), &[]);
        monitor.opened();
        assert_eq!(
            monitor.pending().len(),
            1,
            "B was suppressed by A's old or unknown incarnation"
        );
        assert!(
            monitor.waits_for_engine_start(),
            "B was suppressed by A's old or unknown incarnation"
        );
        monitor.before_open();
        monitor.reconcile_open(DEBOUNCE_NS + 1, &[failed_start(b_ino)], Some(&b));
        monitor.opened();
        assert!(
            monitor.pending().is_empty(),
            "an ordinary failure of B's own attempt is not attempted again"
        );
    }

    #[test]
    fn a_reused_usb_address_cannot_consume_a_new_incarnation() {
        recycled_usb_address(true);
    }

    #[test]
    fn an_unavailable_usb_incarnation_stays_pending() {
        recycled_usb_address(false);
    }

    #[test]
    fn hotplug_linux_reads_only_rift_sensor_metadata_and_waits_for_access() {
        let root = std::env::temp_dir().join(format!("cv1t-hotplug-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let (sys, dev) = (root.join("sys"), root.join("dev"));
        let file = |name: &str, value: &str| {
            let path = sys.join(name);
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(path, value).unwrap();
        };
        for (port, vid, pid, serial) in [
            ("1-2", "2833", "0211", "SENSORALPHA"),
            ("1-3", "2833", "0031", "HEADSET"),
            ("1-4", "1234", "0211", "OTHER"),
            ("1-5", "2833", "0211", "invalid-serial"),
        ] {
            file(&format!("{port}/idVendor"), vid);
            file(&format!("{port}/idProduct"), pid);
            file(&format!("{port}/serial"), serial);
            file(&format!("{port}/busnum"), "1");
            file(&format!("{port}/devnum"), "7");
        }
        let mut found = usb_sensors_in(&sys, &dev).unwrap();
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].serial, "SENSORALPHA");
        assert!(!found[0].ready);
        let attachment = found[0].attachment.clone();
        std::fs::create_dir_all(dev.join("001")).unwrap();
        std::fs::write(dev.join("001/007"), "").unwrap();
        use std::os::unix::fs::PermissionsExt;
        let node = dev.join("001/007");
        std::fs::set_permissions(&node, std::fs::Permissions::from_mode(0o000)).unwrap();
        // Root's capability override makes a mode-000 node look writable. The sibling open
        // test skips that case the same way.
        if !elevated() {
            assert!(
                !usb_sensors_in(&sys, &dev).unwrap()[0].ready,
                "an existing USB node without read/write access is not ready"
            );
        }
        std::fs::set_permissions(&node, std::fs::Permissions::from_mode(0o600)).unwrap();
        found = usb_sensors_in(&sys, &dev).unwrap();
        assert!(found[0].ready);
        assert_eq!(found[0].attachment, attachment);
        file("1-2/devnum", "8");
        assert_ne!(
            usb_sensors_in(&sys, &dev).unwrap()[0].attachment,
            attachment
        );
        // An address change is detected too; inode pins also cover address reuse below.
        use crate::sensor_hotplug::{Monitor, DEBOUNCE_NS};
        let mut monitor = Monitor::new(Ok(found), &[]);
        std::fs::write(dev.join("001/008"), "").unwrap();
        let returned = usb_sensors_in(&sys, &dev).unwrap();
        assert!(monitor.update(1, Ok(returned.clone()), &[]).is_none());
        let change = monitor.update(DEBOUNCE_NS + 1, Ok(returned), &[]).unwrap();
        assert_eq!(change.unplugged, ["SENSORALPHA"]);
        assert_eq!(change.plugged.len(), 1);
        assert!(monitor.waits_for_engine_start());
        monitor.before_open();
        monitor.opened();
        assert!(!monitor.waits_for_engine_start());
        std::fs::remove_file(sys.join("1-2/serial")).unwrap();
        assert!(
            usb_sensors_in(&sys, &dev).unwrap().is_empty(),
            "a missing serial is still enumerating or leaving, not a failure of the whole scan"
        );
        std::fs::remove_dir_all(root).unwrap();
    }

    /// NotFound is a missing attribute. ENODEV is a node that leaves between open and read.
    /// Other failures still fail the scan, so this must not treat every error as gone.
    #[test]
    fn sensor_entry_gone_is_a_missing_or_departing_node_only() {
        assert!(sensor_entry_gone(&std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "missing attribute"
        )));
        assert!(sensor_entry_gone(&std::io::Error::from_raw_os_error(
            libc::ENODEV
        )));
        assert!(!sensor_entry_gone(&std::io::Error::new(
            std::io::ErrorKind::PermissionDenied,
            "not a departure"
        )));
    }

    /// One half-enumerated or departing sensor must not hide a sensor that is present.
    #[test]
    fn one_unreadable_sensor_entry_does_not_hide_another() {
        use std::os::unix::fs::PermissionsExt;
        let root = std::env::temp_dir().join(format!("cv1t-hotplug-skip-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let (sys, dev) = (root.join("sys"), root.join("dev"));
        let file = |name: &str, value: &str| {
            let path = sys.join(name);
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(path, value).unwrap();
        };
        file("1-7/idVendor", "2833");
        file("1-7/idProduct", "0211");
        file("1-7/serial", "SENSORBETA");
        file("1-7/busnum", "1");
        file("1-7/devnum", "10");
        std::fs::create_dir_all(dev.join("001")).unwrap();
        std::fs::write(dev.join("001/010"), "").unwrap();
        std::fs::set_permissions(dev.join("001/010"), std::fs::Permissions::from_mode(0o600))
            .unwrap();
        let expect_beta = |why: &str| {
            let found = usb_sensors_in(&sys, &dev).unwrap();
            assert_eq!(found.len(), 1, "{why}: {found:?}");
            assert_eq!(found[0].serial, "SENSORBETA", "{why}");
            assert!(found[0].ready, "{why}");
        };
        file("1-6/idVendor", "2833");
        expect_beta("a missing product id");
        file("1-6/idProduct", "0211");
        expect_beta("a missing serial");
        file("1-6/serial", "SENSORGAMMA");
        file("1-6/devnum", "9");
        expect_beta("a missing bus number");
        file("1-6/busnum", "1");
        std::fs::remove_file(sys.join("1-6/devnum")).unwrap();
        expect_beta("a missing device number");
        file("1-6/devnum", "9");
        file("1-6/busnum", "nope");
        assert!(
            usb_sensors_in(&sys, &dev).is_err(),
            "a garbled bus number is not a device that is leaving"
        );
        std::fs::remove_dir_all(root).unwrap();
    }

    /// A tree laid out as /sys/bus/usb/devices: a sensor with its video interface bound to
    /// uvcvideo, a headset interface with no driver, and another maker's mouse.
    #[test]
    fn usb_devices_are_read_from_sysfs() {
        use crate::platform::usb::Node;
        let root = std::env::temp_dir().join(format!("cv1t-sysfs-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let file = |path: &str, text: &str| {
            let path = root.join(path);
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(path, format!("{text}\n")).unwrap();
        };
        file("1-2/idVendor", "2833");
        file("1-2/idProduct", "0211");
        file("1-2/product", "Rift Sensor");
        file("1-2/speed", "5000");
        std::fs::create_dir_all(root.join("drivers/usb")).unwrap();
        std::os::unix::fs::symlink(root.join("drivers/usb"), root.join("1-2/driver")).unwrap();
        file("1-2:1.0/bInterfaceNumber", "00");
        file("1-2:1.0/bInterfaceClass", "0e");
        std::fs::create_dir_all(root.join("drivers/uvcvideo")).unwrap();
        std::os::unix::fs::symlink(root.join("drivers/uvcvideo"), root.join("1-2:1.0/driver"))
            .unwrap();
        file("3-1/idVendor", "2833");
        file("3-1/idProduct", "0031");
        file("3-1:1.1/bInterfaceNumber", "01");
        file("3-1:1.1/bInterfaceClass", "03");
        // a hub's device below it sorts after the hub's own interface
        file("3-1.2/idVendor", "2833");
        file("3-1.2/idProduct", "0330");
        file("1-4/idVendor", "046d");
        file("1-4/idProduct", "c52b");
        file("1-4:1.0/bInterfaceClass", "03");
        let node =
            |id: &str, product, name: &str, driver: Option<&str>, parent: Option<&str>| Node {
                id: id.into(),
                product,
                name: name.into(),
                driver: driver.map(String::from),
                problem: None,
                parent: parent.map(String::from),
            };
        assert_eq!(
            usb_devices_in(&root, 0x2833).unwrap(),
            [
                node("1-2", 0x0211, "Rift Sensor (5000 Mb/s)", Some("usb"), None),
                node(
                    "1-2:1.0",
                    0x0211,
                    "interface 00 (video)",
                    Some("uvcvideo"),
                    Some("1-2")
                ),
                node("3-1", 0x0031, "", None, None),
                node("3-1:1.1", 0x0031, "interface 01 (HID)", None, Some("3-1")),
                node("3-1.2", 0x0330, "", None, None),
            ]
        );
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn unreadable_usb_identity_is_unknown_instead_of_an_absent_sensor() {
        for field in ["idVendor", "idProduct"] {
            let root = std::env::temp_dir()
                .join(format!("cv1t-usb-identity-{}-{field}", std::process::id()));
            let _ = std::fs::remove_dir_all(&root);
            let device = root.join("1-2");
            std::fs::create_dir_all(&device).unwrap();
            std::fs::write(device.join("idVendor"), "2833\n").unwrap();
            std::fs::write(device.join("idProduct"), "0211\n").unwrap();
            // A directory cannot be read as an identity file, even as an elevated user.
            std::fs::remove_file(device.join(field)).unwrap();
            std::fs::create_dir(device.join(field)).unwrap();
            let result = usb_devices_in(&root, 0x2833);
            std::fs::remove_dir_all(&root).unwrap();
            assert!(
                result.is_err(),
                "unreadable {field} became known absence: {result:?}"
            );
        }
    }

    /// A sensor is opened through its file under /dev/bus/usb, which sysfs names by bus and
    /// device number; one the udev rules do not cover is denied.
    #[test]
    fn a_sensor_is_opened_through_its_device_file() {
        use std::os::unix::fs::PermissionsExt;
        let root = std::env::temp_dir().join(format!("cv1t-usb-open-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let (sys, dev) = (root.join("sys"), root.join("dev"));
        std::fs::create_dir_all(sys.join("2-5")).unwrap();
        std::fs::create_dir_all(dev.join("002")).unwrap();
        std::fs::write(sys.join("2-5/busnum"), "2\n").unwrap();
        std::fs::write(sys.join("2-5/devnum"), "5\n").unwrap();
        std::fs::write(dev.join("002/005"), "").unwrap();
        can_open_in(&sys, &dev, "2-5").unwrap();
        assert_eq!(
            can_open_in(&sys, &dev, "2-6").unwrap_err().kind(),
            io::ErrorKind::NotFound
        );
        // the engine writes to the device too: a file it may only read is as good as none
        for mode in [0o444, 0o000] {
            std::fs::set_permissions(dev.join("002/005"), std::fs::Permissions::from_mode(mode))
                .unwrap();
            if !elevated() {
                assert_eq!(
                    can_open_in(&sys, &dev, "2-5").unwrap_err().kind(),
                    io::ErrorKind::PermissionDenied,
                    "mode {mode:o}"
                );
            }
        }
        std::fs::remove_dir_all(root).unwrap();
    }
}

#[cfg(test)]
mod container_control_tests {
    use super::*;
    use std::os::unix::fs::{FileTypeExt, MetadataExt, PermissionsExt};

    fn endpoint() -> (crate::socket_dir::SocketDir, std::path::PathBuf) {
        let root = crate::socket_dir::SocketDir::new();
        let path = root.path().join("private/control.sock");
        (root, path)
    }

    #[test]
    fn container_control_is_scoped_to_the_effective_user() {
        assert_eq!(
            container_socket_path_for(42),
            Path::new("/dev/shm/cv1touch-control-42/control.sock")
        );
        assert_eq!(
            container_socket_path_for(43),
            Path::new("/dev/shm/cv1touch-control-43/control.sock")
        );
    }

    #[test]
    fn container_control_creates_a_private_directory_and_socket() {
        let (_fixture, path) = endpoint();
        let listener = Listener::bind_private(&path).unwrap();
        let dir = std::fs::symlink_metadata(path.parent().unwrap()).unwrap();
        let socket = std::fs::symlink_metadata(&path).unwrap();
        assert!(dir.is_dir() && !dir.file_type().is_symlink());
        assert_eq!(dir.mode() & 0o7777, 0o700);
        assert!(socket.file_type().is_socket());
        assert_eq!(socket.mode() & 0o7777, 0o600);
        // SAFETY: geteuid has no arguments and cannot fail.
        let uid = unsafe { libc::geteuid() };
        assert_eq!(dir.uid(), uid);
        assert_eq!(socket.uid(), uid);
        assert!(!private_control_directory_metadata(
            &dir,
            uid.wrapping_add(1)
        ));
        drop(listener);
        unlink_private_socket(&path).unwrap();
        assert!(!path.exists());
        assert!(
            path.parent().unwrap().exists(),
            "keep the directory for later connections"
        );
        std::fs::remove_dir_all(path.parent().unwrap().parent().unwrap()).unwrap();
    }

    #[test]
    fn container_control_new_directory_ignores_a_restrictive_umask() {
        // umask is process-wide: change it only in a child running this test alone.
        if std::env::var_os("CV1T_TEST_CONTROL_UMASK").is_none() {
            let status = std::process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "container_control_new_directory_ignores_a_restrictive_umask",
                    "--test-threads=1",
                ])
                .env("CV1T_TEST_CONTROL_UMASK", "1")
                .status()
                .unwrap();
            assert!(status.success());
            return;
        }
        let (_fixture, path) = endpoint();
        // SAFETY: this isolated test process owns its umask; it is restored below.
        let old = unsafe { libc::umask(0o777) };
        let result = Listener::bind_private(&path);
        // SAFETY: restoring this isolated process's original umask.
        unsafe { libc::umask(old) };
        let listener = result.unwrap();
        assert_eq!(
            std::fs::metadata(path.parent().unwrap()).unwrap().mode() & 0o7777,
            0o700
        );
        assert_eq!(std::fs::metadata(&path).unwrap().mode() & 0o7777, 0o600);
        drop(listener);
        unlink_private_socket(&path).unwrap();
        std::fs::remove_dir_all(path.parent().unwrap().parent().unwrap()).unwrap();
    }

    #[test]
    fn container_control_refuses_existing_unsafe_directories_without_changing_them() {
        let (_fixture, path) = endpoint();
        let dir = path.parent().unwrap();
        std::fs::create_dir(dir).unwrap();
        for mode in [0o755, 0o770, 0o1700] {
            std::fs::set_permissions(dir, std::fs::Permissions::from_mode(mode)).unwrap();
            assert!(Listener::bind_private(&path).is_err(), "mode {mode:o}");
            assert_eq!(
                std::fs::symlink_metadata(dir).unwrap().mode() & 0o7777,
                mode
            );
            assert!(!path.exists());
        }
        std::fs::set_permissions(dir, std::fs::Permissions::from_mode(0o700)).unwrap();
        let real = dir.with_file_name("real");
        std::fs::rename(dir, &real).unwrap();
        std::os::unix::fs::symlink(&real, dir).unwrap();
        assert!(Listener::bind_private(&path).is_err());
        assert!(std::fs::symlink_metadata(dir)
            .unwrap()
            .file_type()
            .is_symlink());
        assert!(!real.join("control.sock").exists());
        std::fs::remove_dir_all(dir.parent().unwrap()).unwrap();
    }

    #[test]
    fn container_control_replaces_only_its_stale_socket_and_cleans_up_only_a_socket() {
        let (_fixture, path) = endpoint();
        let first = Listener::bind_private(&path).unwrap();
        assert!(
            Listener::bind_private(&path).is_err(),
            "keep a live listener"
        );
        drop(first);
        let second = Listener::bind_private(&path).unwrap();
        drop(second);
        unlink_private_socket(&path).unwrap();
        std::fs::write(&path, b"keep this file").unwrap();
        assert!(Listener::bind_private(&path).is_err());
        assert!(unlink_private_socket(&path).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), b"keep this file");
        std::fs::remove_file(&path).unwrap();
        let target = path.with_file_name("target");
        std::fs::write(&target, b"keep this target").unwrap();
        std::os::unix::fs::symlink(&target, &path).unwrap();
        assert!(Listener::bind_private(&path).is_err());
        assert!(unlink_private_socket(&path).is_err());
        assert_eq!(std::fs::read(&target).unwrap(), b"keep this target");
        std::fs::remove_dir_all(path.parent().unwrap().parent().unwrap()).unwrap();
    }
}
