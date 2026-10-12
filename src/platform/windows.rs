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

use windows_sys::Win32::Foundation::{
    LocalFree, ERROR_BROKEN_PIPE, ERROR_IO_PENDING, ERROR_PIPE_BUSY, ERROR_PIPE_CONNECTED,
    GENERIC_READ, GENERIC_WRITE, WAIT_TIMEOUT,
};
use windows_sys::Win32::Security::Authorization::{
    ConvertSidToStringSidW, ConvertStringSecurityDescriptorToSecurityDescriptorW,
};
use windows_sys::Win32::Security::{
    GetTokenInformation, TokenUser, PSECURITY_DESCRIPTOR, SECURITY_ATTRIBUTES, TOKEN_QUERY,
    TOKEN_USER,
};
use windows_sys::Win32::Storage::FileSystem::{
    CreateFileW, ReadFile, WriteFile, FILE_FLAG_FIRST_PIPE_INSTANCE, FILE_FLAG_OVERLAPPED,
    OPEN_EXISTING, PIPE_ACCESS_DUPLEX,
};
use windows_sys::Win32::System::Pipes::{
    ConnectNamedPipe, CreateNamedPipeW, WaitNamedPipeW, PIPE_READMODE_BYTE,
    PIPE_REJECT_REMOTE_CLIENTS, PIPE_TYPE_BYTE, PIPE_UNLIMITED_INSTANCES, PIPE_WAIT,
};
use windows_sys::Win32::System::Threading::{GetCurrentProcess, OpenProcessToken, INFINITE};
use windows_sys::Win32::System::IO::{CancelIoEx, GetOverlappedResult, OVERLAPPED};

pub fn runtime_dir() -> std::path::PathBuf {
    std::env::var_os("LOCALAPPDATA")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(std::env::temp_dir)
        .join("cv1touch")
}

pub fn lock_exclusive(path: &Path) -> io::Result<File> {
    OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .share_mode(0)
        .open(path)
        .map_err(|e| {
            if e.raw_os_error() == Some(ERROR_SHARING_VIOLATION) {
                io::Error::new(io::ErrorKind::WouldBlock, "held by another process")
            } else {
                e
            }
        })
}

/// The pipe for an endpoint path: named after the path, which lies in this user's profile.
fn pipe_name(endpoint: &Path) -> Vec<u16> {
    use sha2::{Digest, Sha256};
    let digest = Sha256::digest(endpoint.to_string_lossy().as_bytes());
    let mut id = String::new();
    for b in &digest[..16] {
        let _ = std::fmt::Write::write_fmt(&mut id, format_args!("{b:02x}"));
    }
    wide(&format!("\\\\.\\pipe\\cv1touch-{id}"))
}

fn manual_event() -> io::Result<Handle> {
    // SAFETY: an unnamed manual-reset event.
    let event = Handle(unsafe { CreateEventW(std::ptr::null(), 1, 0, std::ptr::null()) });
    if event.0.is_null() {
        return Err(io::Error::last_os_error());
    }
    Ok(event)
}

/// Waits for the overlapped operation on `handle` to finish within `timeout`, cancelling it if
/// it does not; the bytes it moved.
fn finish(handle: HANDLE, ov: &mut OVERLAPPED, timeout: Option<Duration>) -> io::Result<usize> {
    let ms = timeout.map_or(INFINITE, |t| {
        t.as_millis().clamp(1, INFINITE as u128 - 1) as u32
    });
    let mut moved = 0u32;
    // SAFETY: `ov` is the live overlapped structure of an operation on `handle`.
    unsafe {
        if WaitForSingleObject(ov.hEvent, ms) == WAIT_TIMEOUT {
            CancelIoEx(handle, ov);
            GetOverlappedResult(handle, ov, &mut moved, 1);
            return Err(io::Error::from(io::ErrorKind::TimedOut));
        }
        if GetOverlappedResult(handle, ov, &mut moved, 0) == 0 {
            return Err(io::Error::last_os_error());
        }
    }
    Ok(moved as usize)
}

pub struct Connection {
    pipe: Handle,
    event: Handle,
    timeout: Option<Duration>,
}

// SAFETY: pipe handles may be used from any thread.
unsafe impl Send for Connection {}

impl Connection {
    pub fn connect(endpoint: &Path) -> io::Result<Connection> {
        let name = pipe_name(endpoint);
        for _ in 0..3 {
            // SAFETY: a NUL-terminated pipe name; the handle is owned below.
            let pipe = unsafe {
                CreateFileW(
                    name.as_ptr(),
                    GENERIC_READ | GENERIC_WRITE,
                    0,
                    std::ptr::null(),
                    OPEN_EXISTING,
                    FILE_FLAG_OVERLAPPED,
                    std::ptr::null_mut(),
                )
            };
            if pipe != INVALID_HANDLE_VALUE {
                return Ok(Connection {
                    pipe: Handle(pipe),
                    event: manual_event()?,
                    timeout: None,
                });
            }
            let e = io::Error::last_os_error();
            if e.raw_os_error() != Some(ERROR_PIPE_BUSY as i32) {
                return Err(e);
            }
            // every instance is busy: wait for the listener to offer the next
            // SAFETY: a NUL-terminated pipe name.
            unsafe { WaitNamedPipeW(name.as_ptr(), 2000) };
        }
        Err(io::Error::from(io::ErrorKind::TimedOut))
    }

    pub fn set_timeout(&mut self, timeout: Option<Duration>) -> io::Result<()> {
        self.timeout = timeout;
        Ok(())
    }

    pub fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        // SAFETY: zeroed is a valid OVERLAPPED; `buf` outlives the operation, which `finish`
        // completes or cancels before returning.
        unsafe {
            let mut ov: OVERLAPPED = std::mem::zeroed();
            ov.hEvent = self.event.0;
            let len = buf.len().min(u32::MAX as usize) as u32;
            if ReadFile(
                self.pipe.0,
                buf.as_mut_ptr(),
                len,
                std::ptr::null_mut(),
                &mut ov,
            ) == 0
            {
                match GetLastError() {
                    ERROR_IO_PENDING => {}
                    ERROR_BROKEN_PIPE => return Ok(0),
                    e => return Err(io::Error::from_raw_os_error(e as i32)),
                }
            }
            match finish(self.pipe.0, &mut ov, self.timeout) {
                Err(e) if e.raw_os_error() == Some(ERROR_BROKEN_PIPE as i32) => Ok(0),
                other => other,
            }
        }
    }

    pub fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        // SAFETY: as in `read`.
        unsafe {
            let mut ov: OVERLAPPED = std::mem::zeroed();
            ov.hEvent = self.event.0;
            let len = buf.len().min(u32::MAX as usize) as u32;
            if WriteFile(
                self.pipe.0,
                buf.as_ptr(),
                len,
                std::ptr::null_mut(),
                &mut ov,
            ) == 0
                && GetLastError() != ERROR_IO_PENDING
            {
                return Err(io::Error::last_os_error());
            }
            finish(self.pipe.0, &mut ov, self.timeout)
        }
    }

    pub fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// A security descriptor granting the current user, and nobody else, full access.
struct UserOnly(PSECURITY_DESCRIPTOR);

impl UserOnly {
    fn new() -> io::Result<UserOnly> {
        // SAFETY: Win32 calls with valid out-pointers; buffers are freed below.
        unsafe {
            let mut token = std::ptr::null_mut();
            if OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &mut token) == 0 {
                return Err(io::Error::last_os_error());
            }
            let token = Handle(token);
            let mut needed = 0u32;
            GetTokenInformation(token.0, TokenUser, std::ptr::null_mut(), 0, &mut needed);
            let mut buf = vec![0u8; needed as usize];
            if GetTokenInformation(
                token.0,
                TokenUser,
                buf.as_mut_ptr().cast(),
                needed,
                &mut needed,
            ) == 0
            {
                return Err(io::Error::last_os_error());
            }
            let user = &*(buf.as_ptr() as *const TOKEN_USER);
            let mut sid_text = std::ptr::null_mut();
            if ConvertSidToStringSidW(user.User.Sid, &mut sid_text) == 0 {
                return Err(io::Error::last_os_error());
            }
            let mut len = 0;
            while *sid_text.add(len) != 0 {
                len += 1;
            }
            let sid = String::from_utf16_lossy(std::slice::from_raw_parts(sid_text, len));
            LocalFree(sid_text.cast());
            let sddl = wide(&format!("D:P(A;;GA;;;{sid})"));
            let mut descriptor = std::ptr::null_mut();
            if ConvertStringSecurityDescriptorToSecurityDescriptorW(
                sddl.as_ptr(),
                1,
                &mut descriptor,
                std::ptr::null_mut(),
            ) == 0
            {
                return Err(io::Error::last_os_error());
            }
            Ok(UserOnly(descriptor))
        }
    }
}

impl Drop for UserOnly {
    fn drop(&mut self) {
        // SAFETY: allocated by ConvertStringSecurityDescriptorToSecurityDescriptorW.
        unsafe { LocalFree(self.0) };
    }
}

/// A pipe instance waiting for its client.
struct Pending {
    pipe: Handle,
    event: Handle,
    // boxed: the system writes to it until the connect completes
    ov: Box<OVERLAPPED>,
}

pub struct Listener {
    name: Vec<u16>,
    security: UserOnly,
    pending: Option<Pending>,
}

// SAFETY: pipe handles may be used from any thread.
unsafe impl Send for Listener {}

impl Listener {
    pub fn bind(endpoint: &Path) -> io::Result<Listener> {
        let mut listener = Listener {
            name: pipe_name(endpoint),
            security: UserOnly::new()?,
            pending: None,
        };
        listener.offer(true)?;
        Ok(listener)
    }

    /// Creates the next pipe instance and starts waiting for a client on it.
    fn offer(&mut self, first: bool) -> io::Result<()> {
        let attributes = SECURITY_ATTRIBUTES {
            nLength: std::mem::size_of::<SECURITY_ATTRIBUTES>() as u32,
            lpSecurityDescriptor: self.security.0,
            bInheritHandle: 0,
        };
        let open_mode = PIPE_ACCESS_DUPLEX
            | FILE_FLAG_OVERLAPPED
            | if first {
                FILE_FLAG_FIRST_PIPE_INSTANCE
            } else {
                0
            };
        // SAFETY: a NUL-terminated name and a valid security descriptor.
        let pipe = unsafe {
            CreateNamedPipeW(
                self.name.as_ptr(),
                open_mode,
                PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT | PIPE_REJECT_REMOTE_CLIENTS,
                PIPE_UNLIMITED_INSTANCES,
                4096,
                4096,
                0,
                &attributes,
            )
        };
        if pipe == INVALID_HANDLE_VALUE {
            let e = io::Error::last_os_error();
            return Err(if first && e.raw_os_error() == Some(ERROR_ACCESS_DENIED) {
                io::Error::new(io::ErrorKind::AddrInUse, "another process listens there")
            } else {
                e
            });
        }
        let pipe = Handle(pipe);
        let event = manual_event()?;
        // SAFETY: zeroed is a valid OVERLAPPED.
        let mut ov: Box<OVERLAPPED> = Box::new(unsafe { std::mem::zeroed() });
        ov.hEvent = event.0;
        // SAFETY: the boxed OVERLAPPED stays put until the connect completes or is cancelled.
        unsafe {
            if ConnectNamedPipe(pipe.0, &mut *ov) == 0 {
                match GetLastError() {
                    ERROR_IO_PENDING => {}
                    // a client arrived between creating and connecting
                    ERROR_PIPE_CONNECTED => {
                        SetEvent(event.0);
                    }
                    e => return Err(io::Error::from_raw_os_error(e as i32)),
                }
            }
        }
        self.pending = Some(Pending { pipe, event, ov });
        Ok(())
    }

    pub fn accept(&mut self, wait: bool) -> io::Result<Connection> {
        let pending = self
            .pending
            .as_ref()
            .ok_or_else(|| io::Error::other("no pipe instance"))?;
        // SAFETY: the pending instance's event.
        if unsafe { WaitForSingleObject(pending.event.0, if wait { INFINITE } else { 0 }) }
            == WAIT_TIMEOUT
        {
            return Err(io::Error::from(io::ErrorKind::WouldBlock));
        }
        let Pending { pipe, event, .. } = self.pending.take().expect("checked above");
        self.offer(false)?;
        Ok(Connection {
            pipe,
            event,
            timeout: None,
        })
    }
}

impl Drop for Listener {
    fn drop(&mut self) {
        if let Some(p) = self.pending.as_mut() {
            let mut moved = 0;
            // SAFETY: cancel the pending connect and wait it out before the box is freed.
            unsafe {
                CancelIoEx(p.pipe.0, &*p.ov);
                GetOverlappedResult(p.pipe.0, &*p.ov, &mut moved, 1);
            }
        }
    }
}
