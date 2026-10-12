// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! Windows implementations. Files under the user's profile are private to them by the profile's
//! own access control; there are no FIFOs to block on, and directories cannot be flushed.

use std::fs::{File, OpenOptions};
use std::io;
use std::os::windows::fs::{MetadataExt, OpenOptionsExt};
use std::path::Path;
use std::time::Duration;

/// Opens a reparse point (a link) itself rather than its target.
const FILE_FLAG_OPEN_REPARSE_POINT: u32 = 0x0020_0000;
const FILE_ATTRIBUTE_REPARSE_POINT: u32 = 0x0400;
/// Another process (often a virus scanner) briefly holds the file: worth retrying.
const ERROR_ACCESS_DENIED: i32 = 5;
const ERROR_SHARING_VIOLATION: i32 = 32;

pub fn dylib_open(path: &Path) -> *mut std::ffi::c_void {
    use std::os::windows::ffi::OsStrExt;
    let wide: Vec<u16> = path.as_os_str().encode_wide().chain([0]).collect();
    // SAFETY: a NUL-terminated wide string.
    unsafe { windows_sys::Win32::System::LibraryLoader::LoadLibraryW(wide.as_ptr()) as _ }
}

pub fn dylib_symbol(lib: *mut std::ffi::c_void, name: &std::ffi::CStr) -> *mut std::ffi::c_void {
    // SAFETY: a module handle from LoadLibraryW and a NUL-terminated name.
    unsafe {
        windows_sys::Win32::System::LibraryLoader::GetProcAddress(lib as _, name.as_ptr().cast())
            .map_or(std::ptr::null_mut(), |f| f as *mut std::ffi::c_void)
    }
}

pub fn dylib_close(lib: *mut std::ffi::c_void) {
    // SAFETY: a module handle from LoadLibraryW, freed once.
    unsafe { windows_sys::Win32::Foundation::FreeLibrary(lib as _) };
}

pub fn open_read_nofollow(path: &Path) -> io::Result<File> {
    let file = OpenOptions::new()
        .read(true)
        .custom_flags(FILE_FLAG_OPEN_REPARSE_POINT)
        .open(path)?;
    if file.metadata()?.file_attributes() & FILE_ATTRIBUTE_REPARSE_POINT != 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "refusing to follow a link",
        ));
    }
    Ok(file)
}

pub fn open_read_nonblocking(path: &Path) -> io::Result<File> {
    File::open(path)
}

pub fn create_private(path: &Path) -> io::Result<File> {
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .custom_flags(FILE_FLAG_OPEN_REPARSE_POINT)
        .open(path)
}

pub fn create_private_dir_all(path: &Path) -> io::Result<()> {
    std::fs::create_dir_all(path)
}

pub fn sync_file(path: &Path) -> io::Result<()> {
    // flushing needs a handle with write access
    OpenOptions::new().write(true).open(path)?.sync_all()
}

pub fn sync_dir(_dir: &Path) -> io::Result<()> {
    Ok(())
}

fn retrying<T>(mut op: impl FnMut() -> io::Result<T>) -> io::Result<T> {
    let mut wait = Duration::from_millis(10);
    loop {
        match op() {
            Err(e)
                if matches!(
                    e.raw_os_error(),
                    Some(ERROR_ACCESS_DENIED | ERROR_SHARING_VIOLATION)
                ) && wait < Duration::from_secs(1) =>
            {
                std::thread::sleep(wait);
                wait *= 2;
            }
            other => return other,
        }
    }
}

pub fn rename_replacing(from: &Path, to: &Path) -> io::Result<()> {
    retrying(|| std::fs::rename(from, to))
}

pub fn swap_dir(stage: &Path, dir: &Path, exists: bool) -> io::Result<()> {
    if !exists {
        if dir.exists() {
            return Err(io::Error::from(io::ErrorKind::AlreadyExists));
        }
        return retrying(|| std::fs::rename(stage, dir));
    }
    // no atomic exchange for directories: move the old aside, the new in, the old into the stage;
    // a failure in the middle puts the old back
    let aside = dir.with_extension(format!("old-{}", std::process::id()));
    retrying(|| std::fs::rename(dir, &aside))?;
    if let Err(e) = retrying(|| std::fs::rename(stage, dir)) {
        let _ = std::fs::rename(&aside, dir);
        return Err(e);
    }
    retrying(|| std::fs::rename(&aside, stage))
}
