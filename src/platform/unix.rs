// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! Linux implementations.

use std::fs::{DirBuilder, File, OpenOptions};
use std::io;
use std::os::unix::ffi::OsStrExt;
use std::os::unix::fs::{DirBuilderExt, OpenOptionsExt};
use std::path::Path;

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
}

impl Drop for OwnedSegment {
    fn drop(&mut self) {
        // SAFETY: our mapping and name.
        unsafe {
            libc::munmap(self.ptr as *mut libc::c_void, self.len);
            libc::shm_unlink(self.name.as_ptr());
        }
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

#[cfg(test)]
mod tests {
    use super::*;

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
