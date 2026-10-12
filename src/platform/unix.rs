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
