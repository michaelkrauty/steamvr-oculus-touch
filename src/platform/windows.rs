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

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::OnceLock;

use windows_sys::Win32::Foundation::{
    CloseHandle, GetLastError, ERROR_ALREADY_EXISTS, ERROR_FILE_NOT_FOUND, HANDLE,
    INVALID_HANDLE_VALUE,
};
use windows_sys::Win32::System::Memory::{
    CreateFileMappingW, MapViewOfFile, OpenFileMappingW, UnmapViewOfFile, VirtualQuery,
    FILE_MAP_ALL_ACCESS, FILE_MAP_READ, MEMORY_BASIC_INFORMATION, MEMORY_MAPPED_VIEW_ADDRESS,
    PAGE_READWRITE,
};
use windows_sys::Win32::System::Performance::{QueryPerformanceCounter, QueryPerformanceFrequency};
use windows_sys::Win32::System::Threading::{
    CreateEventW, CreateMutexW, OpenEventW, SetEvent, WaitForSingleObject,
};

/// SYNCHRONIZE: waiting on an object is all a reader needs.
const SYNCHRONIZE: u32 = 0x0010_0000;

pub fn now_ns() -> u64 {
    static FREQUENCY: OnceLock<u64> = OnceLock::new();
    let frequency = *FREQUENCY.get_or_init(|| {
        let mut f = 0i64;
        // SAFETY: valid out-pointer; never fails on Windows XP and later.
        unsafe { QueryPerformanceFrequency(&mut f) };
        f.max(1) as u64
    });
    let mut ticks = 0i64;
    // SAFETY: valid out-pointer.
    unsafe { QueryPerformanceCounter(&mut ticks) };
    super::ticks_to_ns(ticks.max(0) as u64, frequency)
}

fn wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain([0]).collect()
}

/// The single component after "/", as on Linux.
fn component(name: &str) -> io::Result<&str> {
    name.strip_prefix('/')
        .filter(|s| !s.is_empty() && !s.contains(['/', '\\']) && *s != "." && *s != "..")
        .ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "shared-memory name must be / followed by a single name",
            )
        })
}

/// The kernel object names for `component`, private to this logon session.
fn object_name(kind: &str, component: &str) -> Vec<u16> {
    wide(&format!("Local\\cv1touch-{kind}-{component}"))
}

/// Bytes mapped at `view`.
fn mapped_len(view: *const std::ffi::c_void) -> usize {
    // SAFETY: zeroed is a valid MEMORY_BASIC_INFORMATION; valid out-pointer and size.
    unsafe {
        let mut info: MEMORY_BASIC_INFORMATION = std::mem::zeroed();
        if VirtualQuery(
            view,
            &mut info,
            std::mem::size_of::<MEMORY_BASIC_INFORMATION>(),
        ) == 0
        {
            return 0;
        }
        info.RegionSize
    }
}

struct Handle(HANDLE);

impl Drop for Handle {
    fn drop(&mut self) {
        if !self.0.is_null() {
            // SAFETY: a handle we opened, closed once.
            unsafe { CloseHandle(self.0) };
        }
    }
}

pub struct OwnedSegment {
    ptr: *mut u8,
    _mapping: Handle,
    event: Handle,
    _owner: Handle,
}

// SAFETY: a shared mapping; callers synchronise access to its contents.
unsafe impl Send for OwnedSegment {}
unsafe impl Sync for OwnedSegment {}

impl OwnedSegment {
    pub fn create(name: &str, len: usize) -> io::Result<OwnedSegment> {
        let component = component(name)?;
        // SAFETY: Win32 calls with NUL-terminated names; every handle is owned by a Handle.
        unsafe {
            // only owners hold this mutex: if it exists, a live process (this one included) owns
            // the name; a crashed owner's handle closed with it
            let owner = Handle(CreateMutexW(
                std::ptr::null(),
                1,
                object_name("owner", component).as_ptr(),
            ));
            if owner.0.is_null() {
                return Err(io::Error::last_os_error());
            }
            if GetLastError() == ERROR_ALREADY_EXISTS {
                return Err(io::Error::new(
                    io::ErrorKind::WouldBlock,
                    "another process owns this shared memory",
                ));
            }
            let mapping = Handle(CreateFileMappingW(
                INVALID_HANDLE_VALUE,
                std::ptr::null(),
                PAGE_READWRITE,
                (len as u64 >> 32) as u32,
                len as u32,
                object_name("shm", component).as_ptr(),
            ));
            if mapping.0.is_null() {
                return Err(io::Error::last_os_error());
            }
            // a reader's view can keep an earlier owner's section alive: it is reused, zeroed
            let reused = GetLastError() == ERROR_ALREADY_EXISTS;
            let view = MapViewOfFile(mapping.0, FILE_MAP_ALL_ACCESS, 0, 0, len);
            if view.Value.is_null() {
                return Err(io::Error::last_os_error());
            }
            if reused {
                if mapped_len(view.Value) < len {
                    UnmapViewOfFile(view);
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "an earlier shared-memory section is too small",
                    ));
                }
                std::ptr::write_bytes(view.Value as *mut u8, 0, len);
            }
            let event = Handle(CreateEventW(
                std::ptr::null(),
                0,
                0,
                object_name("update", component).as_ptr(),
            ));
            Ok(OwnedSegment {
                ptr: view.Value as *mut u8,
                _mapping: mapping,
                event,
                _owner: owner,
            })
        }
    }

    pub fn ptr(&self) -> *mut u8 {
        self.ptr
    }

    pub fn wake(&self, _word: *mut u32) {
        if !self.event.0.is_null() {
            // SAFETY: our event handle.
            unsafe { SetEvent(self.event.0) };
        }
    }
}

impl Drop for OwnedSegment {
    fn drop(&mut self) {
        // SAFETY: our view; the handles close after.
        unsafe {
            UnmapViewOfFile(MEMORY_MAPPED_VIEW_ADDRESS {
                Value: self.ptr as *mut std::ffi::c_void,
            })
        };
    }
}

pub struct OpenedSegment {
    ptr: *mut u8,
    _mapping: Handle,
    event: Handle,
}

// SAFETY: a read-only shared mapping.
unsafe impl Send for OpenedSegment {}
unsafe impl Sync for OpenedSegment {}

impl OpenedSegment {
    pub fn open(name: &str, len: usize) -> Result<OpenedSegment, super::shm::OpenError> {
        use super::shm::OpenError;
        let component = component(name).map_err(OpenError::Io)?;
        // SAFETY: Win32 calls with NUL-terminated names; every handle is owned by a Handle.
        unsafe {
            let mapping = Handle(OpenFileMappingW(
                FILE_MAP_READ,
                0,
                object_name("shm", component).as_ptr(),
            ));
            if mapping.0.is_null() {
                return Err(if GetLastError() == ERROR_FILE_NOT_FOUND {
                    OpenError::NotFound
                } else {
                    OpenError::Io(io::Error::last_os_error())
                });
            }
            let view = MapViewOfFile(mapping.0, FILE_MAP_READ, 0, 0, 0);
            if view.Value.is_null() {
                return Err(OpenError::Io(io::Error::last_os_error()));
            }
            if mapped_len(view.Value) < len {
                UnmapViewOfFile(view);
                return Err(OpenError::TooSmall);
            }
            let event = Handle(OpenEventW(
                SYNCHRONIZE,
                0,
                object_name("update", component).as_ptr(),
            ));
            Ok(OpenedSegment {
                ptr: view.Value as *mut u8,
                _mapping: mapping,
                event,
            })
        }
    }

    pub fn ptr(&self) -> *mut u8 {
        self.ptr
    }

    pub fn wait(&self, word: *mut u32, last_seen: u32, timeout_ns: u64) {
        // SAFETY: the counter is inside the live mapping.
        if unsafe { AtomicU32::from_ptr(word) }.load(Ordering::Acquire) != last_seen {
            return;
        }
        let ms = timeout_ns.div_ceil(1_000_000).min(u32::MAX as u64 - 1) as u32;
        if self.event.0.is_null() {
            std::thread::sleep(Duration::from_millis(ms as u64));
        } else {
            // SAFETY: our event handle.
            unsafe { WaitForSingleObject(self.event.0, ms) };
        }
    }
}

impl Drop for OpenedSegment {
    fn drop(&mut self) {
        // SAFETY: our view; the handles close after.
        unsafe {
            UnmapViewOfFile(MEMORY_MAPPED_VIEW_ADDRESS {
                Value: self.ptr as *mut std::ffi::c_void,
            })
        };
    }
}
