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
    CreateEventW, CreateMutexW, OpenEventW, ResetEvent, SetEvent, WaitForSingleObject,
};

/// SYNCHRONIZE: waiting on an object is all a reader needs.
const SYNCHRONIZE: u32 = 0x0010_0000;

/// The update events, one per parity of the counter: publish n sets `update{n % 2}` and resets the
/// other, so a reader that saw n waits on `update{(n + 1) % 2}` and every waiter wakes (one
/// auto-reset event would release a single waiter per publish). The SteamVR driver opens the same
/// names (driver/src/shm_client.cpp).
const UPDATE_EVENTS: [&str; 2] = ["update0", "update1"];

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
    events: [Handle; 2],
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
            let events = UPDATE_EVENTS.map(|kind| {
                // a manual-reset event, possibly left signalled by a reader's open handle
                let event = Handle(CreateEventW(
                    std::ptr::null(),
                    1,
                    0,
                    object_name(kind, component).as_ptr(),
                ));
                if !event.0.is_null() {
                    ResetEvent(event.0);
                }
                event
            });
            Ok(OwnedSegment {
                ptr: view.Value as *mut u8,
                _mapping: mapping,
                events,
                _owner: owner,
            })
        }
    }

    pub fn ptr(&self) -> *mut u8 {
        self.ptr
    }

    pub fn wake(&self, word: *mut u32) {
        // SAFETY: the counter is inside our live mapping.
        let n = unsafe { AtomicU32::from_ptr(word) }.load(Ordering::Acquire) as usize;
        let (now, next) = (&self.events[n % 2], &self.events[(n + 1) % 2]);
        // SAFETY: our event handles (null ones are skipped).
        unsafe {
            if !now.0.is_null() {
                SetEvent(now.0);
            }
            if !next.0.is_null() {
                ResetEvent(next.0);
            }
        }
    }

    /// Named objects have no unlink: the name goes with the last handle on the segment.
    pub fn unlink(&self) {}
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
    events: [Handle; 2],
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
            let events = UPDATE_EVENTS.map(|kind| {
                Handle(OpenEventW(
                    SYNCHRONIZE,
                    0,
                    object_name(kind, component).as_ptr(),
                ))
            });
            Ok(OpenedSegment {
                ptr: view.Value as *mut u8,
                _mapping: mapping,
                events,
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
        let event = &self.events[(last_seen as usize + 1) % 2];
        if event.0.is_null() {
            std::thread::sleep(Duration::from_millis(ms as u64));
        } else {
            // SAFETY: our event handle.
            unsafe { WaitForSingleObject(event.0, ms) };
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
    LocalFree, ERROR_BROKEN_PIPE, ERROR_IO_PENDING, ERROR_NO_DATA, ERROR_PIPE_BUSY,
    ERROR_PIPE_CONNECTED, GENERIC_READ, GENERIC_WRITE, WAIT_TIMEOUT,
};
use windows_sys::Win32::Security::Authorization::{
    ConvertSidToStringSidW, ConvertStringSecurityDescriptorToSecurityDescriptorW, GetSecurityInfo,
    SE_KERNEL_OBJECT,
};
use windows_sys::Win32::Security::{
    EqualSid, GetTokenInformation, TokenUser, OWNER_SECURITY_INFORMATION, PSECURITY_DESCRIPTOR,
    PSID, SECURITY_ATTRIBUTES, TOKEN_QUERY, TOKEN_USER,
};
use windows_sys::Win32::Storage::FileSystem::{
    CreateFileW, ReadFile, WriteFile, FILE_FLAG_FIRST_PIPE_INSTANCE, FILE_FLAG_OVERLAPPED,
    OPEN_EXISTING, PIPE_ACCESS_DUPLEX, SECURITY_IDENTIFICATION, SECURITY_SQOS_PRESENT,
};
use windows_sys::Win32::System::Pipes::{
    ConnectNamedPipe, CreateNamedPipeW, WaitNamedPipeW, PIPE_READMODE_BYTE,
    PIPE_REJECT_REMOTE_CLIENTS, PIPE_TYPE_BYTE, PIPE_UNLIMITED_INSTANCES, PIPE_WAIT,
};
use windows_sys::Win32::System::Threading::{GetCurrentProcess, OpenProcessToken, INFINITE};
use windows_sys::Win32::System::IO::{CancelIoEx, GetOverlappedResult, OVERLAPPED};

pub fn runtime_dir() -> std::path::PathBuf {
    super::dirs::get().config
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

fn pipe_name(endpoint: &Path) -> io::Result<Vec<u16>> {
    use std::os::windows::ffi::OsStrExt;
    let path: Vec<u16> = endpoint.as_os_str().encode_wide().collect();
    let mut name = super::pipe_name(&path).ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            "endpoint path too long for a pipe name",
        )
    })?;
    name.push(0);
    Ok(name)
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
        let name = pipe_name(endpoint)?;
        for _ in 0..3 {
            // SAFETY: a NUL-terminated pipe name; the handle is owned below.
            let pipe = unsafe {
                CreateFileW(
                    name.as_ptr(),
                    GENERIC_READ | GENERIC_WRITE,
                    0,
                    std::ptr::null(),
                    OPEN_EXISTING,
                    // identification only: the server may check who we are, not act as us
                    FILE_FLAG_OVERLAPPED | SECURITY_SQOS_PRESENT | SECURITY_IDENTIFICATION,
                    std::ptr::null_mut(),
                )
            };
            if pipe != INVALID_HANDLE_VALUE {
                let pipe = Handle(pipe);
                served_by(pipe.0, &TokenUserInfo::current()?)?;
                return Ok(Connection {
                    pipe,
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

/// The account this process runs as, from its token.
struct TokenUserInfo {
    // u64s: TOKEN_USER and the SID after it need pointer alignment
    buf: Vec<u64>,
}

impl TokenUserInfo {
    fn current() -> io::Result<TokenUserInfo> {
        // SAFETY: Win32 calls with valid out-pointers into a buffer of the size asked for.
        unsafe {
            let mut token = std::ptr::null_mut();
            if OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &mut token) == 0 {
                return Err(io::Error::last_os_error());
            }
            let token = Handle(token);
            let mut needed = 0u32;
            GetTokenInformation(token.0, TokenUser, std::ptr::null_mut(), 0, &mut needed);
            let mut buf = vec![0u64; (needed as usize).div_ceil(8)];
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
            Ok(TokenUserInfo { buf })
        }
    }

    fn sid(&self) -> PSID {
        // SAFETY: the buffer holds the TOKEN_USER that GetTokenInformation wrote.
        unsafe { (*(self.buf.as_ptr() as *const TOKEN_USER)).User.Sid }
    }

    fn sid_string(&self) -> io::Result<String> {
        // SAFETY: a valid SID; the string ConvertSidToStringSidW allocates is freed here.
        unsafe {
            let mut text = std::ptr::null_mut();
            if ConvertSidToStringSidW(self.sid(), &mut text) == 0 {
                return Err(io::Error::last_os_error());
            }
            let mut len = 0;
            while *text.add(len) != 0 {
                len += 1;
            }
            let sid = String::from_utf16_lossy(std::slice::from_raw_parts(text, len));
            LocalFree(text.cast());
            Ok(sid)
        }
    }
}

/// Refuses a pipe that `user` does not own. Pipe names are global, so another account could
/// create ours before the daemon does; the daemon makes its user the owner of its pipe, and
/// only an account with the restore privilege can make another account an owner.
fn served_by(pipe: HANDLE, user: &TokenUserInfo) -> io::Result<()> {
    let mut owner: PSID = std::ptr::null_mut();
    let mut descriptor: PSECURITY_DESCRIPTOR = std::ptr::null_mut();
    // SAFETY: an open pipe handle and valid out-pointers; `owner` points into `descriptor`,
    // which is freed after the comparison.
    let ours = unsafe {
        let error = GetSecurityInfo(
            pipe,
            SE_KERNEL_OBJECT,
            OWNER_SECURITY_INFORMATION,
            &mut owner,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            &mut descriptor,
        );
        if error != 0 {
            return Err(io::Error::from_raw_os_error(error as i32));
        }
        let ours = EqualSid(owner, user.sid()) != 0;
        LocalFree(descriptor);
        ours
    };
    if ours {
        Ok(())
    } else {
        Err(io::Error::new(
            io::ErrorKind::PermissionDenied,
            "the pipe belongs to another account",
        ))
    }
}

/// A security descriptor owned by the current user and granting it, and nobody else, full
/// access.
struct UserOnly(PSECURITY_DESCRIPTOR);

impl UserOnly {
    fn new() -> io::Result<UserOnly> {
        let sid = TokenUserInfo::current()?.sid_string()?;
        // SAFETY: a NUL-terminated SDDL string and a valid out-pointer; freed on drop.
        unsafe {
            let sddl = wide(&format!("O:{sid}D:P(A;;GA;;;{sid})"));
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
            name: pipe_name(endpoint)?,
            security: UserOnly::new()?,
            pending: None,
        };
        listener.offer(true)?;
        Ok(listener)
    }

    /// Creates the next pipe instance and starts waiting for a client on it.
    fn offer(&mut self, first: bool) -> io::Result<()> {
        // a client that opens an instance and closes it before the connect spoils that
        // instance only: offer a fresh one
        loop {
            match self.offer_once(first) {
                Err(e) if e.raw_os_error() == Some(ERROR_NO_DATA as i32) => continue,
                other => return other,
            }
        }
    }

    fn offer_once(&mut self, first: bool) -> io::Result<()> {
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
        if self.pending.is_none() {
            // offering after the last accept failed: try again
            self.offer(false)?;
        }
        let pending = self.pending.as_ref().expect("offered above");
        // SAFETY: the pending instance's event.
        if unsafe { WaitForSingleObject(pending.event.0, if wait { INFINITE } else { 0 }) }
            == WAIT_TIMEOUT
        {
            return Err(io::Error::from(io::ErrorKind::WouldBlock));
        }
        let Pending { pipe, event, .. } = self.pending.take().expect("checked above");
        // the client is served either way; if no instance can be offered now, the next
        // accept tries again and reports why it cannot
        let _ = self.offer(false);
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

use windows_sys::Win32::Devices::DeviceAndDriverInstallation::{
    SetupDiDestroyDeviceInfoList, SetupDiEnumDeviceInterfaces, SetupDiGetClassDevsW,
    SetupDiGetDeviceInterfaceDetailW, DIGCF_DEVICEINTERFACE, DIGCF_PRESENT,
    SP_DEVICE_INTERFACE_DATA, SP_DEVICE_INTERFACE_DETAIL_DATA_W,
};
use windows_sys::Win32::Devices::HumanInterfaceDevice::{
    HidD_FreePreparsedData, HidD_GetHidGuid, HidD_GetPreparsedData, HidD_SetNumInputBuffers,
    HidP_GetCaps, HIDP_CAPS, HIDP_STATUS_SUCCESS,
};
use windows_sys::Win32::Foundation::{ERROR_IO_INCOMPLETE, WAIT_OBJECT_0};
use windows_sys::Win32::Storage::FileSystem::{FILE_SHARE_READ, FILE_SHARE_WRITE};
use windows_sys::Win32::System::Threading::WaitForMultipleObjects;

pub struct Waker(Handle);

// SAFETY: event handles may be signalled from any thread.
unsafe impl Send for Waker {}
unsafe impl Sync for Waker {}

impl Waker {
    pub fn new() -> io::Result<Waker> {
        // SAFETY: an unnamed auto-reset event.
        let event = Handle(unsafe { CreateEventW(std::ptr::null(), 0, 0, std::ptr::null()) });
        if event.0.is_null() {
            return Err(io::Error::last_os_error());
        }
        Ok(Waker(event))
    }

    pub fn wake(&self) {
        // SAFETY: our event handle.
        unsafe { SetEvent(self.0 .0) };
    }
}

/// Reports the HID class driver queues for this handle, beyond the default 32, so a busy moment
/// in the reader drops none.
const HID_INPUT_BUFFERS: u32 = 512;

/// A HID interface opened through the HID class driver, with one read always in flight: a report
/// lands in `buf` as it arrives, and `try_read` takes it and starts the next read.
pub struct HidDevice {
    handle: Handle,
    event: Handle,
    // boxed: the system writes to it until the read completes
    ov: Box<OVERLAPPED>,
    buf: Vec<u8>,
    in_flight: bool,
    ready: Option<usize>,
    /// The error that ended reading (the device unplugged): every later read reports it.
    failed: Option<i32>,
}

// SAFETY: the handles may be used from any thread; the in-flight read's buffers move with it.
unsafe impl Send for HidDevice {}

impl HidDevice {
    pub fn open(path: &Path) -> io::Result<HidDevice> {
        use std::os::windows::ffi::OsStrExt;
        let name: Vec<u16> = path.as_os_str().encode_wide().chain([0]).collect();
        // SAFETY: a NUL-terminated path; the handle is owned below. Sharing as hidapi does lets
        // the engine keep its own handle on the same interface.
        let handle = unsafe {
            CreateFileW(
                name.as_ptr(),
                GENERIC_READ | GENERIC_WRITE,
                FILE_SHARE_READ | FILE_SHARE_WRITE,
                std::ptr::null(),
                OPEN_EXISTING,
                FILE_FLAG_OVERLAPPED,
                std::ptr::null_mut(),
            )
        };
        if handle == INVALID_HANDLE_VALUE {
            return Err(io::Error::last_os_error());
        }
        let handle = Handle(handle);
        let mut caps: HIDP_CAPS = unsafe { std::mem::zeroed() };
        // SAFETY: our device handle; the preparsed data is freed once.
        unsafe {
            let mut data = 0;
            if HidD_GetPreparsedData(handle.0, &mut data) == 0 {
                return Err(io::Error::last_os_error());
            }
            let status = HidP_GetCaps(data, &mut caps);
            HidD_FreePreparsedData(data);
            if status != HIDP_STATUS_SUCCESS {
                return Err(io::Error::other("the HID interface has no capabilities"));
            }
            HidD_SetNumInputBuffers(handle.0, HID_INPUT_BUFFERS);
        }
        HidDevice::from_handle(handle, caps.InputReportByteLength as usize)
    }

    fn from_handle(handle: Handle, input_len: usize) -> io::Result<HidDevice> {
        let mut device = HidDevice {
            handle,
            event: manual_event()?,
            // SAFETY: zeroed is a valid OVERLAPPED.
            ov: Box::new(unsafe { std::mem::zeroed() }),
            buf: vec![0; input_len.max(1)],
            in_flight: false,
            ready: None,
            failed: None,
        };
        device.start_read();
        Ok(device)
    }

    /// Puts the read for the next report in flight, unless one is, or a report is waiting.
    fn start_read(&mut self) {
        if self.in_flight || self.ready.is_some() || self.failed.is_some() {
            return;
        }
        // SAFETY: the boxed OVERLAPPED and `buf` stay put until the read completes or is
        // cancelled (Drop).
        unsafe {
            *self.ov = std::mem::zeroed();
            self.ov.hEvent = self.event.0;
            if ReadFile(
                self.handle.0,
                self.buf.as_mut_ptr(),
                self.buf.len() as u32,
                std::ptr::null_mut(),
                &mut *self.ov,
            ) == 0
            {
                let e = GetLastError();
                if e != ERROR_IO_PENDING {
                    self.failed = Some(e as i32);
                    return;
                }
            }
        }
        self.in_flight = true;
    }

    /// Takes in the read in flight if it has finished.
    fn collect(&mut self) {
        if !self.in_flight {
            return;
        }
        let mut moved = 0u32;
        // SAFETY: the OVERLAPPED of our read on our handle.
        unsafe {
            if GetOverlappedResult(self.handle.0, &*self.ov, &mut moved, 0) == 0 {
                let e = GetLastError();
                if e == ERROR_IO_INCOMPLETE {
                    return;
                }
                self.failed = Some(e as i32);
            } else {
                self.ready = Some(moved as usize);
            }
        }
        self.in_flight = false;
    }

    pub fn wait(
        &mut self,
        waker: &Waker,
        timeout: Option<Duration>,
    ) -> io::Result<crate::platform::hid::Woke> {
        use crate::platform::hid::Woke;
        self.start_read();
        self.collect();
        if self.ready.is_some() || self.failed.is_some() {
            return Ok(Woke {
                report: true,
                waker: false,
            });
        }
        let ms = timeout.map_or(INFINITE, |t| t.as_millis().min(INFINITE as u128 - 1) as u32);
        let handles = [self.event.0, waker.0 .0];
        // SAFETY: two valid handles.
        match unsafe { WaitForMultipleObjects(2, handles.as_ptr(), 0, ms) } {
            WAIT_OBJECT_0 => {
                self.collect();
                Ok(Woke {
                    report: self.ready.is_some() || self.failed.is_some(),
                    waker: false,
                })
            }
            r if r == WAIT_OBJECT_0 + 1 => Ok(Woke {
                report: false,
                waker: true,
            }),
            WAIT_TIMEOUT => Ok(Woke::default()),
            _ => Err(io::Error::last_os_error()),
        }
    }

    pub fn try_read(&mut self, buf: &mut [u8]) -> io::Result<Option<usize>> {
        self.collect();
        if let Some(n) = self.ready.take() {
            let n = n.min(buf.len());
            buf[..n].copy_from_slice(&self.buf[..n]);
            self.start_read();
            return Ok(Some(n));
        }
        if let Some(e) = self.failed {
            return Err(io::Error::from_raw_os_error(e));
        }
        self.start_read();
        Ok(None)
    }
}

impl Drop for HidDevice {
    fn drop(&mut self) {
        if self.in_flight {
            let mut moved = 0;
            // SAFETY: cancel our read and wait it out before its buffers are freed.
            unsafe {
                CancelIoEx(self.handle.0, &*self.ov);
                GetOverlappedResult(self.handle.0, &*self.ov, &mut moved, 1);
            }
        }
    }
}

/// The HID class's device interfaces present, by path.
pub fn headset_interface(interface: u8) -> Option<std::path::PathBuf> {
    // SAFETY: SetupAPI calls with valid out-pointers; the detail buffer is 8-byte aligned and as
    // large as SetupAPI asked for; the device information set is destroyed once.
    unsafe {
        let mut guid = std::mem::zeroed();
        HidD_GetHidGuid(&mut guid);
        let set = SetupDiGetClassDevsW(
            &guid,
            std::ptr::null(),
            std::ptr::null_mut(),
            DIGCF_PRESENT | DIGCF_DEVICEINTERFACE,
        );
        if set == INVALID_HANDLE_VALUE as isize {
            return None;
        }
        let mut found = None;
        for index in 0.. {
            let mut data: SP_DEVICE_INTERFACE_DATA = std::mem::zeroed();
            data.cbSize = std::mem::size_of::<SP_DEVICE_INTERFACE_DATA>() as u32;
            if SetupDiEnumDeviceInterfaces(set, std::ptr::null(), &guid, index, &mut data) == 0 {
                break;
            }
            let mut size = 0u32;
            SetupDiGetDeviceInterfaceDetailW(
                set,
                &data,
                std::ptr::null_mut(),
                0,
                &mut size,
                std::ptr::null_mut(),
            );
            if size == 0 {
                continue;
            }
            let mut storage = vec![0u64; (size as usize).div_ceil(8)];
            let detail = storage.as_mut_ptr() as *mut SP_DEVICE_INTERFACE_DETAIL_DATA_W;
            (*detail).cbSize = std::mem::size_of::<SP_DEVICE_INTERFACE_DETAIL_DATA_W>() as u32;
            if SetupDiGetDeviceInterfaceDetailW(
                set,
                &data,
                detail,
                size,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            ) == 0
            {
                continue;
            }
            let path = std::ptr::addr_of!((*detail).DevicePath) as *const u16;
            let mut len = 0;
            while *path.add(len) != 0 {
                len += 1;
            }
            let path = String::from_utf16_lossy(std::slice::from_raw_parts(path, len));
            if super::hid::is_headset_path(&path, interface) {
                found = Some(std::path::PathBuf::from(path));
                break;
            }
        }
        SetupDiDestroyDeviceInfoList(set);
        found
    }
}

/// The console's control events, set into a manual-reset event by the handler.
pub struct ShutdownRequests;

static SHUTDOWN_EVENT: OnceLock<usize> = OnceLock::new();
static SHUTDOWN_KIND: AtomicU32 = AtomicU32::new(u32::MAX);

unsafe extern "system" fn console_control(kind: u32) -> windows_sys::Win32::Foundation::BOOL {
    use windows_sys::Win32::System::Console::{
        CTRL_CLOSE_EVENT, CTRL_LOGOFF_EVENT, CTRL_SHUTDOWN_EVENT,
    };
    let Some(&event) = SHUTDOWN_EVENT.get() else {
        return 0;
    };
    SHUTDOWN_KIND.store(kind, Ordering::Release);
    // SAFETY: the event lives as long as the process.
    unsafe { SetEvent(event as HANDLE) };
    if matches!(
        kind,
        CTRL_CLOSE_EVENT | CTRL_LOGOFF_EVENT | CTRL_SHUTDOWN_EVENT
    ) {
        // Windows ends the process when this returns: hold it while the daemon cleans up (the
        // process exits from main; the system gives up after a few seconds)
        std::thread::sleep(Duration::from_secs(10));
    }
    1
}

impl ShutdownRequests {
    pub fn take() -> io::Result<ShutdownRequests> {
        use windows_sys::Win32::System::Console::SetConsoleCtrlHandler;
        if SHUTDOWN_EVENT.get().is_none() {
            // SAFETY: an unnamed manual-reset event, kept for the life of the process.
            let event = unsafe { CreateEventW(std::ptr::null(), 1, 0, std::ptr::null()) };
            if event.is_null() {
                return Err(io::Error::last_os_error());
            }
            let _ = SHUTDOWN_EVENT.set(event as usize);
        }
        // SAFETY: a handler that only touches statics.
        if unsafe { SetConsoleCtrlHandler(Some(console_control), 1) } == 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(ShutdownRequests)
    }

    pub fn wait(&self, timeout: Duration) -> Option<String> {
        use windows_sys::Win32::System::Console::{
            CTRL_BREAK_EVENT, CTRL_CLOSE_EVENT, CTRL_C_EVENT, CTRL_LOGOFF_EVENT,
            CTRL_SHUTDOWN_EVENT,
        };
        let event = *SHUTDOWN_EVENT.get()? as HANDLE;
        let ms = timeout.as_millis().min(INFINITE as u128 - 1) as u32;
        // SAFETY: the process-lifetime event.
        if unsafe { WaitForSingleObject(event, ms) } != WAIT_OBJECT_0 {
            return None;
        }
        Some(
            match SHUTDOWN_KIND.load(Ordering::Acquire) {
                CTRL_C_EVENT => "Ctrl+C",
                CTRL_BREAK_EVENT => "Ctrl+Break",
                CTRL_CLOSE_EVENT => "console closed",
                CTRL_LOGOFF_EVENT => "logoff",
                CTRL_SHUTDOWN_EVENT => "system shutdown",
                _ => "console control event",
            }
            .to_string(),
        )
    }
}

impl Drop for ShutdownRequests {
    fn drop(&mut self) {
        use windows_sys::Win32::System::Console::SetConsoleCtrlHandler;
        // SAFETY: removes the handler `take` added.
        unsafe { SetConsoleCtrlHandler(Some(console_control), 0) };
    }
}

/// Ctrl+Break, as the console would deliver it (to the handler directly: a real console event
/// would reach the test runner too).
#[cfg(test)]
pub fn request_shutdown() {
    use windows_sys::Win32::System::Console::CTRL_BREAK_EVENT;
    // SAFETY: the handler only touches statics.
    unsafe { console_control(CTRL_BREAK_EVENT) };
}

/// The far end of a fake HID device: a message-mode pipe delivers one message per read, as the
/// HID class driver delivers one report.
#[cfg(test)]
pub struct FakeHidSender(Handle);

#[cfg(test)]
pub fn fake_hid() -> (HidDevice, FakeHidSender) {
    use std::sync::atomic::AtomicUsize;
    use windows_sys::Win32::Storage::FileSystem::{FILE_WRITE_ATTRIBUTES, PIPE_ACCESS_OUTBOUND};
    use windows_sys::Win32::System::Pipes::{
        SetNamedPipeHandleState, PIPE_READMODE_MESSAGE, PIPE_TYPE_MESSAGE,
    };
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    let name = wide(&format!(
        r"\\.\pipe\cv1t-fake-hid-{}-{}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    // SAFETY: Win32 calls with a NUL-terminated name; the handles are owned below.
    unsafe {
        let server = CreateNamedPipeW(
            name.as_ptr(),
            PIPE_ACCESS_OUTBOUND,
            PIPE_TYPE_MESSAGE | PIPE_WAIT,
            1,
            4096,
            4096,
            0,
            std::ptr::null(),
        );
        assert!(
            server != INVALID_HANDLE_VALUE,
            "{}",
            io::Error::last_os_error()
        );
        let server = Handle(server);
        let client = CreateFileW(
            name.as_ptr(),
            GENERIC_READ | FILE_WRITE_ATTRIBUTES,
            0,
            std::ptr::null(),
            OPEN_EXISTING,
            FILE_FLAG_OVERLAPPED,
            std::ptr::null_mut(),
        );
        assert!(
            client != INVALID_HANDLE_VALUE,
            "{}",
            io::Error::last_os_error()
        );
        let client = Handle(client);
        let mode = PIPE_READMODE_MESSAGE;
        assert!(
            SetNamedPipeHandleState(client.0, &mode, std::ptr::null(), std::ptr::null()) != 0,
            "{}",
            io::Error::last_os_error()
        );
        if ConnectNamedPipe(server.0, std::ptr::null_mut()) == 0 {
            assert_eq!(GetLastError(), ERROR_PIPE_CONNECTED);
        }
        (
            HidDevice::from_handle(client, 64).unwrap(),
            FakeHidSender(server),
        )
    }
}

#[cfg(test)]
pub fn fake_hid_send(sender: &mut FakeHidSender, report: &[u8]) {
    let mut wrote = 0;
    // SAFETY: our pipe handle and a buffer of the stated length.
    let ok = unsafe {
        WriteFile(
            sender.0 .0,
            report.as_ptr(),
            report.len() as u32,
            &mut wrote,
            std::ptr::null_mut(),
        )
    };
    assert!(ok != 0 && wrote as usize == report.len());
}

pub type ThreadId = u32;

pub fn current_thread() -> ThreadId {
    // SAFETY: no arguments.
    unsafe { windows_sys::Win32::System::Threading::GetCurrentThreadId() }
}

/// A thread handle, closed when dropped.
struct ThreadHandle(windows_sys::Win32::Foundation::HANDLE);

impl ThreadHandle {
    fn open(thread: ThreadId, access: u32) -> io::Result<ThreadHandle> {
        // SAFETY: plain call; a null handle is the error.
        let h = unsafe { windows_sys::Win32::System::Threading::OpenThread(access, 0, thread) };
        if h.is_null() {
            return Err(io::Error::last_os_error());
        }
        Ok(ThreadHandle(h))
    }
}

impl Drop for ThreadHandle {
    fn drop(&mut self) {
        // SAFETY: a handle we opened.
        unsafe { windows_sys::Win32::Foundation::CloseHandle(self.0) };
    }
}

/// A thread's description (its name), as SetThreadDescription set it.
fn thread_description(thread: ThreadId) -> Option<String> {
    use windows_sys::Win32::System::Threading::{
        GetThreadDescription, THREAD_QUERY_LIMITED_INFORMATION,
    };
    let h = ThreadHandle::open(thread, THREAD_QUERY_LIMITED_INFORMATION).ok()?;
    let mut text: windows_sys::core::PWSTR = std::ptr::null_mut();
    // SAFETY: a live handle and an output pointer; the string is ours to free.
    unsafe {
        if GetThreadDescription(h.0, &mut text) < 0 || text.is_null() {
            return None;
        }
        let len = (0..).take_while(|&i| *text.add(i) != 0).count();
        let name = String::from_utf16_lossy(std::slice::from_raw_parts(text, len));
        windows_sys::Win32::Foundation::LocalFree(text.cast());
        Some(name)
    }
}

pub fn thread_named(name: &str) -> Option<ThreadId> {
    use windows_sys::Win32::Foundation::{CloseHandle, INVALID_HANDLE_VALUE};
    use windows_sys::Win32::System::Diagnostics::ToolHelp::{
        CreateToolhelp32Snapshot, Thread32First, Thread32Next, TH32CS_SNAPTHREAD, THREADENTRY32,
    };
    // SAFETY: a snapshot of all threads, walked with a correctly sized entry and closed after.
    unsafe {
        let snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
        if snapshot == INVALID_HANDLE_VALUE {
            return None;
        }
        let process = std::process::id();
        let mut entry: THREADENTRY32 = std::mem::zeroed();
        entry.dwSize = std::mem::size_of::<THREADENTRY32>() as u32;
        let mut found = None;
        let mut more = Thread32First(snapshot, &mut entry) != 0;
        while more && found.is_none() {
            if entry.th32OwnerProcessID == process
                && thread_description(entry.th32ThreadID).as_deref() == Some(name)
            {
                found = Some(entry.th32ThreadID);
            }
            more = Thread32Next(snapshot, &mut entry) != 0;
        }
        CloseHandle(snapshot);
        found
    }
}

pub fn make_realtime(thread: ThreadId, priority: u32) -> Result<String, String> {
    use windows_sys::Win32::System::Threading::{
        SetThreadPriority, THREAD_PRIORITY_HIGHEST, THREAD_PRIORITY_TIME_CRITICAL,
        THREAD_SET_LIMITED_INFORMATION,
    };
    let (level, what) = if priority >= 10 {
        (THREAD_PRIORITY_TIME_CRITICAL, "time-critical priority")
    } else {
        (THREAD_PRIORITY_HIGHEST, "highest priority")
    };
    let h =
        ThreadHandle::open(thread, THREAD_SET_LIMITED_INFORMATION).map_err(|e| e.to_string())?;
    // SAFETY: a live handle opened for setting information.
    if unsafe { SetThreadPriority(h.0, level) } == 0 {
        return Err(io::Error::last_os_error().to_string());
    }
    Ok(what.into())
}

/// The thread's own affinity mask. Processor group 0 only (64 CPUs): a thread placed in another
/// group (on a machine with more) is refused.
pub fn allowed_cpus(thread: ThreadId) -> io::Result<Vec<usize>> {
    use windows_sys::Win32::System::SystemInformation::GROUP_AFFINITY;
    use windows_sys::Win32::System::Threading::{
        GetThreadGroupAffinity, THREAD_QUERY_LIMITED_INFORMATION,
    };
    let h = ThreadHandle::open(thread, THREAD_QUERY_LIMITED_INFORMATION)?;
    // SAFETY: a plain structure, all zeros a valid value.
    let mut affinity: GROUP_AFFINITY = unsafe { std::mem::zeroed() };
    // SAFETY: a live handle with query access, and a structure to fill.
    if unsafe { GetThreadGroupAffinity(h.0, &mut affinity) } == 0 {
        return Err(io::Error::last_os_error());
    }
    if affinity.Group != 0 {
        return Err(io::Error::new(
            io::ErrorKind::Unsupported,
            format!("the thread runs in processor group {}", affinity.Group),
        ));
    }
    Ok((0..usize::BITS as usize)
        .filter(|cpu| affinity.Mask & (1 << cpu) != 0)
        .collect())
}

pub fn set_cpus(thread: ThreadId, cpus: &[usize]) -> io::Result<()> {
    use windows_sys::Win32::System::Threading::{
        SetThreadAffinityMask, THREAD_QUERY_INFORMATION, THREAD_SET_INFORMATION,
    };
    let mut mask = 0usize;
    for &cpu in cpus {
        if cpu >= usize::BITS as usize {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("CPU {cpu} is outside processor group 0"),
            ));
        }
        mask |= 1 << cpu;
    }
    let h = ThreadHandle::open(thread, THREAD_SET_INFORMATION | THREAD_QUERY_INFORMATION)?;
    // SAFETY: a live handle with the access the call needs; 0 is the error.
    if unsafe { SetThreadAffinityMask(h.0, mask) } == 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

pub fn lock_memory() -> io::Result<()> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "Windows has no counterpart to mlockall",
    ))
}

/// Windows does not report them per thread, and has no limit on a real-time thread's run to stay
/// under.
pub fn voluntary_switches() -> Option<i64> {
    None
}

pub fn random_u64() -> io::Result<u64> {
    use windows_sys::Win32::Security::Cryptography::{
        BCryptGenRandom, BCRYPT_USE_SYSTEM_PREFERRED_RNG,
    };
    let mut v = 0u64;
    // SAFETY: an 8-byte buffer and the system's preferred generator (no algorithm handle).
    let status = unsafe {
        BCryptGenRandom(
            std::ptr::null_mut(),
            &mut v as *mut u64 as *mut u8,
            8,
            BCRYPT_USE_SYSTEM_PREFERRED_RNG,
        )
    };
    if status < 0 {
        return Err(io::Error::other(format!(
            "BCryptGenRandom: NTSTATUS {status:#x}"
        )));
    }
    Ok(v)
}

pub fn opt_out_of_throttling() -> io::Result<()> {
    use windows_sys::Win32::System::Threading::{
        GetCurrentProcess, ProcessPowerThrottling, SetProcessInformation,
        PROCESS_POWER_THROTTLING_CURRENT_VERSION, PROCESS_POWER_THROTTLING_EXECUTION_SPEED,
        PROCESS_POWER_THROTTLING_STATE,
    };
    let state = PROCESS_POWER_THROTTLING_STATE {
        Version: PROCESS_POWER_THROTTLING_CURRENT_VERSION,
        ControlMask: PROCESS_POWER_THROTTLING_EXECUTION_SPEED,
        StateMask: 0,
    };
    // SAFETY: this process's pseudo-handle and a correctly sized state.
    let ok = unsafe {
        SetProcessInformation(
            GetCurrentProcess(),
            ProcessPowerThrottling,
            &state as *const PROCESS_POWER_THROTTLING_STATE as *const std::ffi::c_void,
            std::mem::size_of::<PROCESS_POWER_THROTTLING_STATE>() as u32,
        )
    };
    if ok == 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}
