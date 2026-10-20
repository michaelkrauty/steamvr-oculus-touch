// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Session recordings, format v2 (little-endian).
//! File header (32 B): magic "CV1TREC\0", u32 version, u32 header_size = 32, u64 start_host_ns,
//! u64 flight_skipped_records.
//! The last field was reserved (zero): reports/commands skipped in the saved flight-recorder
//! window, including its tail through the ring snapshot; zero in other recordings. Old readers
//! ignore it and all record offsets and the version stay the same. u64::MAX means the count
//! is unknown because relevant skip history was unavailable.
//! Record: u16 type, u16 flags = 0, u32 payload_len, u64 host_ns, payload.
//! CONTROLLER_STATE = u32 hand, u32 0, cv1t_controller_state; RAW_REPORT = u32 source, bytes;
//! FRAME_OBS = ohmd_cv1touch_frame_info, n x ohmd_cv1touch_blob; SENSOR_INFO = ohmd_cv1touch_sensor;
//! LED_MODEL = u32 device_id, u32 n, n x ohmd_cv1touch_led; HAPTIC = u32 hand, f32 duration_s,
//! f32 frequency_hz, f32 amplitude (a haptic command as the daemon got it; amplitude 0 is off).
//! INSPECTION_IDENTITY = ohmd_cv1touch_inspection_identity (start, normal end or retrospective
//! invalidation). ENGINE_SESSION: no payload (the engine opened; its identity epochs start over).
//! IDENTITIES_RETIRED = the sensor's serial, UTF-8 (its restored pose was acknowledged and its
//! invalidated identities retired).
//! Tracking threads only copy into a ring under a mutex (no I/O); a writer thread does the file I/O.

use std::fs::{File, OpenOptions};
use std::io::{self, BufReader, Read, Write};
use std::mem::size_of;
use std::path::{Path, PathBuf};
use std::ptr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

pub use crate::platform::fs::DiskSpace;

pub use crate::ffi::ohmd_cv1touch_blob as Blob;
pub use crate::ffi::ohmd_cv1touch_frame_info as FrameInfo;
pub use crate::ffi::ohmd_cv1touch_led as LedEntry;
pub use crate::ffi::ohmd_cv1touch_sensor as SensorInfo;
use crate::shm::ControllerState;

pub const MAGIC: &[u8; 8] = b"CV1TREC\0";
pub const VERSION: u32 = 2;
/// The v2 header's skip count when lost timestamp history prevents an exact count.
pub const UNKNOWN_FLIGHT_SKIPS: u64 = u64::MAX;
pub const CONTROLLER_STATE: u16 = 1;
pub const RAW_REPORT: u16 = 2;
pub const FRAME_OBS: u16 = 4;
pub const SENSOR_INFO: u16 = 5;
pub const LED_MODEL: u16 = 6;
/// A sensor frame's pixels (see `encode_frame`), recorded with `record start PATH --frames`.
pub const RAW_FRAME: u16 = 7;
pub const HAPTIC: u16 = 8;
/// Inspection identity lifetime (48-byte POD); additive to v2, older readers skip it.
pub const INSPECTION_IDENTITY: u16 = 9;
/// The engine (re)opened: identity epochs number afresh. Additive to v2.
pub const ENGINE_SESSION: u16 = 10;
/// A sensor's invalidated identities retired after its restored pose was acknowledged.
/// Additive to v2.
pub const IDENTITIES_RETIRED: u16 = 11;

pub fn decode_inspection_identity(payload: &[u8]) -> Option<crate::engine::InspectionIdentity> {
    (payload.len() == size_of::<crate::engine::InspectionIdentity>())
        .then(|| read_pod(payload))
        .flatten()
}

/// A HAPTIC record's payload.
pub fn haptic_payload(hand: usize, duration_s: f32, frequency_hz: f32, amplitude: f32) -> [u8; 16] {
    let mut p = [0u8; 16];
    p[..4].copy_from_slice(&(hand as u32).to_le_bytes());
    p[4..8].copy_from_slice(&duration_s.to_le_bytes());
    p[8..12].copy_from_slice(&frequency_hz.to_le_bytes());
    p[12..].copy_from_slice(&amplitude.to_le_bytes());
    p
}
const FILE_HEADER_LEN: usize = 32;
const RECORD_HEADER_LEN: usize = 16;
const CHUNK: usize = 256 * 1024;
// No daemon record can exceed its 64 MiB ring. Bound allocations even for corrupt files.
const MAX_RECORD_BYTES: usize = 64 << 20;
// Generous ceiling above the CV1 sensor's 1280 x 960 image.
const MAX_FRAME_PIXELS: usize = 16 << 20;

/// Byte view of a plain-old-data value for recording payloads.
pub fn as_bytes<T: Copy>(v: &T) -> &[u8] {
    // SAFETY: T is Copy POD from the C ABI; reading its bytes is fine.
    unsafe { std::slice::from_raw_parts(v as *const T as *const u8, size_of::<T>()) }
}

pub fn slice_bytes<T: Copy>(v: &[T]) -> &[u8] {
    // SAFETY: as `as_bytes`.
    unsafe { std::slice::from_raw_parts(v.as_ptr() as *const u8, std::mem::size_of_val(v)) }
}

/// LED_MODEL payload prefix: u32 device_id, u32 n.
pub fn led_model_prefix(device_id: i32, n: usize) -> [u8; 8] {
    let mut p = [0u8; 8];
    p[..4].copy_from_slice(&(device_id as u32).to_le_bytes());
    p[4..].copy_from_slice(&(n as u32).to_le_bytes());
    p
}

/// Which session-info records (sensor intrinsics, per-controller LED models) a recording still
/// needs. Each is written as soon as it is available, so a controller that sleeps through the
/// recording does not hold back the sensors or the other controller.
#[derive(Debug, Default, Clone)]
pub struct SessionInfo {
    sensors_written: bool,
    models_written: Vec<i32>,
}

impl SessionInfo {
    /// Starts over (new recording or engine reopen).
    pub fn reset(&mut self) {
        *self = SessionInfo::default();
    }

    /// Given what is available now (sensors open; per controller device id: LED model known), returns
    /// what to write now (sensor info?, device ids of LED models) and marks it written.
    pub fn plan(&mut self, have_sensors: bool, models: &[(i32, bool)]) -> (bool, Vec<i32>) {
        let sensors = have_sensors && !self.sensors_written;
        self.sensors_written |= sensors;
        let new: Vec<i32> = models
            .iter()
            .filter(|(d, known)| *known && !self.models_written.contains(d))
            .map(|(d, _)| *d)
            .collect();
        self.models_written.extend(&new);
        (sensors, new)
    }

    /// True once the sensors and `controllers` LED models are written.
    pub fn complete(&self, controllers: usize) -> bool {
        self.sensors_written && self.models_written.len() >= controllers
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub struct Stats {
    pub records: u64,
    pub dropped: u64,
    pub write_errors: u64,
    pub bytes: u64,
}

/// Absolute ceilings leave room for long evaluation captures; free space is checked separately.
pub const RECORDING_BYTES: u64 = 8 << 30;
pub const FRAME_RECORDING_BYTES: u64 = 16 << 30;

fn disk_reserve(space: DiskSpace) -> u64 {
    // More than thirty 64 MiB rings even on a small disk; at most 20 GiB on a large one.
    (space.capacity / 20).clamp(2 << 30, 20 << 30)
}

fn space_reason(available: u64, reserve: u64) -> String {
    format!(
        "too little disk space: {:.2} GiB free; keeping {:.2} GiB free",
        available as f64 / (1u64 << 30) as f64,
        reserve as f64 / (1u64 << 30) as f64
    )
}

/// Storage operations run without the recorder's ring lock. Implementations must not buffer a
/// successful sync: that acknowledgement means all preceding writes have reached storage.
pub trait RecordingSink: Write + Send {
    fn sync_all(&mut self) -> io::Result<()>;
    fn disk_space(&self) -> io::Result<DiskSpace>;
}

type FinalRecordingStatus = Arc<Mutex<Option<Status>>>;

/// The header and final outcome of a background start, independent of later control commands.
pub struct Starting {
    ready: mpsc::Receiver<Result<(), String>>,
    header: Mutex<Option<Result<(), String>>>,
    finished: FinalRecordingStatus,
}

impl Starting {
    pub fn poll(&self) -> Option<Result<(), String>> {
        let mut header = self.header.lock().unwrap();
        if header.is_none() {
            *header = match self.ready.try_recv() {
                Ok(result) => Some(result),
                Err(mpsc::TryRecvError::Empty) => None,
                Err(mpsc::TryRecvError::Disconnected) => Some(Err(
                    "recording writer stopped before opening its file".into(),
                )),
            };
        }
        header.clone()
    }

    /// Published after close, before a stop can acknowledge and replace this session.
    pub fn final_status(&self) -> Option<Status> {
        self.finished.lock().unwrap().clone()
    }
}

struct FileSink {
    file: File,
    path: PathBuf,
}

impl Write for FileSink {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.file.write(bytes)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.file.flush()
    }
}

impl RecordingSink for FileSink {
    fn sync_all(&mut self) -> io::Result<()> {
        self.file.sync_all()?;
        crate::platform::fs::sync_dir(crate::platform::fs::parent_dir(&self.path))
    }
    fn disk_space(&self) -> io::Result<DiskSpace> {
        crate::platform::fs::disk_space(&self.file, &self.path)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum State {
    Idle,
    Starting,
    Recording,
    Finishing,
    Complete,
    Failed,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct Status {
    pub state: State,
    pub path: Option<PathBuf>,
    /// A session owns the writer until a stop acknowledges its outcome.
    pub pending: bool,
    /// The writer has closed and synced the file (or finished reporting its failure).
    pub finished: bool,
    pub stats: Stats,
    pub byte_limit: u64,
    pub free_bytes: Option<u64>,
    pub reserve_bytes: Option<u64>,
    pub reason: Option<String>,
    pub error: Option<String>,
}

impl Status {
    pub fn message(&self) -> String {
        if let Some(error) = &self.error {
            return format!(
                "recording failed: {error}{}",
                if self.finished {
                    ""
                } else {
                    "; file is still finishing"
                }
            );
        }
        if let Some(reason) = &self.reason {
            return format!(
                "recording ended early: {reason}; {}",
                if self.finished {
                    "file is complete and synced"
                } else {
                    "file is still finishing"
                }
            );
        }
        if !self.finished && self.state == State::Finishing {
            return "recording is still finishing; waiting for storage; check status and try record stop again".into();
        }
        match self.state {
            State::Idle => String::new(),
            State::Starting => "recording is opening its file".into(),
            State::Recording => "recording".into(),
            State::Complete => "recording is complete and synced".into(),
            _ => "recording is still finishing".into(),
        }
    }
}

struct Ring {
    buf: Vec<u8>,
    head: usize,
    used: usize,
    queued_records: u64,
    stop: bool,
    opening: bool,
    session: bool,
    finished: bool,
    path: Option<PathBuf>,
    accepted_bytes: u64,
    byte_limit: u64,
    free_bytes: Option<u64>,
    reserve_bytes: Option<u64>,
    reason: Option<String>,
    error: Option<String>,
    stats: Stats,
}

impl Ring {
    fn status(&self) -> Status {
        let state = if self.error.is_some() {
            State::Failed
        } else if self.path.is_none() {
            State::Idle
        } else if self.finished {
            State::Complete
        } else if self.stop {
            State::Finishing
        } else if self.opening {
            State::Starting
        } else {
            State::Recording
        };
        Status {
            state,
            path: self.path.clone(),
            pending: self.session,
            finished: self.finished,
            stats: self.stats,
            byte_limit: self.byte_limit,
            free_bytes: self.free_bytes,
            reserve_bytes: self.reserve_bytes,
            reason: self.reason.clone(),
            error: self.error.clone(),
        }
    }

    fn put(&mut self, src: &[u8]) {
        let cap = self.buf.len();
        let first = (cap - self.head).min(src.len());
        self.buf[self.head..self.head + first].copy_from_slice(&src[..first]);
        self.buf[..src.len() - first].copy_from_slice(&src[first..]);
        self.head = (self.head + src.len()) % cap;
        self.used += src.len();
    }

    fn take(&mut self, max: usize) -> Vec<u8> {
        let cap = self.buf.len();
        let n = self.used.min(max);
        let tail = (self.head + cap - self.used) % cap;
        let first = (cap - tail).min(n);
        let mut out = Vec::with_capacity(n);
        out.extend_from_slice(&self.buf[tail..tail + first]);
        out.extend_from_slice(&self.buf[..n - first]);
        self.used -= n;
        out
    }

    /// Batches whole records, including one larger than CHUNK. A disk-reserve stop can discard
    /// this batch and the remaining ring without leaving half a record in the file.
    fn take_records(&mut self) -> (Vec<u8>, u64) {
        let cap = self.buf.len();
        let tail = (self.head + cap - self.used) % cap;
        let (mut bytes, mut records) = (0, 0);
        while bytes < self.used {
            let length = std::array::from_fn(|i| self.buf[(tail + bytes + 4 + i) % cap]);
            let n = RECORD_HEADER_LEN + u32::from_le_bytes(length) as usize;
            if bytes > 0 && bytes + n > CHUNK {
                break;
            }
            bytes += n;
            records += 1;
        }
        self.queued_records -= records;
        (self.take(bytes), records)
    }

    fn fail(&mut self, error: String) {
        self.stats.write_errors += 1;
        self.error = Some(match self.error.take() {
            Some(first) => format!("{first}; {error}"),
            None => error,
        });
        self.stop = true;
    }
}

struct Shared {
    ring: Mutex<Ring>,
    cond: Condvar,
    active: AtomicBool,
    diagnostic: fn(&str),
}

impl Shared {
    fn report_diagnostic(&self, message: &str) {
        // A diagnostic failure must not lose the header outcome or leave a session unfinished.
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            (self.diagnostic)(message);
        }));
    }
}

pub struct Recorder {
    shared: Arc<Shared>,
}

/// A session slot reserved before storage I/O. An abandoned start releases its slot.
pub struct ReservedRecording<'a> {
    recorder: &'a Recorder,
    path: PathBuf,
    started: bool,
}

impl ReservedRecording<'_> {
    pub fn start(mut self, start_host_ns: u64) -> Result<(), String> {
        let path = &self.path;
        let result = self
            .recorder
            .start_reserved_session(path, start_host_ns, || {
                let file = OpenOptions::new().write(true).create_new(true).open(path)?;
                Ok(Box::new(FileSink {
                    file,
                    path: path.to_path_buf(),
                }))
            });
        self.started = true;
        result
    }
}

impl Drop for ReservedRecording<'_> {
    fn drop(&mut self) {
        if !self.started {
            self.recorder.cancel_start();
        }
    }
}

fn write_header(
    shared: &Shared,
    file: &mut dyn RecordingSink,
    path: &Path,
    start_host_ns: u64,
) -> Result<(), String> {
    let space = file
        .disk_space()
        .map_err(|e| format!("cannot check free disk space: {e}"))?;
    let reserve = disk_reserve(space);
    {
        let mut ring = shared.ring.lock().unwrap();
        ring.free_bytes = Some(space.available);
        ring.reserve_bytes = Some(reserve);
    }
    if space.available.saturating_sub(reserve) < FILE_HEADER_LEN as u64 {
        return Err(space_reason(space.available, reserve));
    }
    let mut header = [0u8; FILE_HEADER_LEN];
    header[..8].copy_from_slice(MAGIC);
    header[8..12].copy_from_slice(&VERSION.to_le_bytes());
    header[12..16].copy_from_slice(&(FILE_HEADER_LEN as u32).to_le_bytes());
    header[16..24].copy_from_slice(&start_host_ns.to_le_bytes());
    file.write_all(&header)
        .map_err(|e| format!("write {}: {e}", path.display()))?;
    let mut ring = shared.ring.lock().unwrap();
    ring.stats.bytes = FILE_HEADER_LEN as u64;
    ring.opening = false;
    shared.active.store(!ring.stop, Ordering::Release);
    Ok(())
}

// Avoid a filesystem query for each small radio batch. A fresh query also checks what other
// disk users consumed; our own writes spend the cached allowance between those observations.
#[derive(Default)]
struct DiskAllowance {
    available: u64,
    reserve: u64,
    checked: Option<Instant>,
    written: u64,
}

impl DiskAllowance {
    fn permits(&mut self, sink: &dyn RecordingSink, bytes: u64, now: Instant) -> io::Result<bool> {
        if self
            .checked
            .is_none_or(|at| now.duration_since(at) >= Duration::from_secs(1))
            || self.written >= 16 << 20
            || self.available.saturating_sub(self.reserve) < bytes
        {
            let space = sink.disk_space()?;
            self.available = space.available;
            self.reserve = disk_reserve(space);
            self.checked = Some(now);
            self.written = 0;
        }
        Ok(self.available.saturating_sub(self.reserve) >= bytes)
    }

    fn wrote(&mut self, bytes: u64) {
        self.available = self.available.saturating_sub(bytes);
        self.written += bytes;
    }
}

fn writer_main(shared: &Shared, file: &mut dyn RecordingSink) {
    let mut allowance = DiskAllowance::default();
    let mut ring = shared.ring.lock().unwrap();
    loop {
        while ring.used == 0 && !ring.stop {
            ring = shared.cond.wait(ring).unwrap();
        }
        if ring.used == 0 && ring.stop {
            break;
        }
        let (chunk, records) = ring.take_records();
        drop(ring);
        let permitted = allowance.permits(file, chunk.len() as u64, Instant::now());
        ring = shared.ring.lock().unwrap();
        ring.free_bytes = Some(allowance.available);
        ring.reserve_bytes = Some(allowance.reserve);
        match permitted {
            Ok(true) => {}
            Ok(false) => {
                let discarded = records + ring.queued_records;
                ring.stats.records -= discarded;
                ring.stats.dropped += discarded;
                ring.used = 0;
                ring.queued_records = 0;
                ring.stop = true;
                ring.reason = Some(space_reason(allowance.available, allowance.reserve));
                shared.active.store(false, Ordering::Release);
                break;
            }
            Err(error) => {
                ring.fail(format!("cannot check free disk space: {error}"));
                shared.active.store(false, Ordering::Release);
                ring.used = 0;
                ring.queued_records = 0;
                break;
            }
        }
        drop(ring);
        let result = file.write_all(&chunk);
        ring = shared.ring.lock().unwrap();
        match result {
            Ok(()) => {
                ring.stats.bytes += chunk.len() as u64;
                allowance.wrote(chunk.len() as u64);
                ring.free_bytes = Some(allowance.available);
            }
            Err(error) => {
                ring.fail(format!("cannot write the recording: {error}"));
                // A partial write can end inside a record. Never append after it.
                shared.active.store(false, Ordering::Release);
                ring.used = 0;
                ring.queued_records = 0;
                break;
            }
        }
    }
    drop(ring);
    if let Err(error) = file.sync_all() {
        shared
            .ring
            .lock()
            .unwrap()
            .fail(format!("cannot sync the recording to disk: {error}"));
    }
}

impl Recorder {
    pub fn new(ring_bytes: usize) -> Self {
        Self::with_diagnostics(ring_bytes, |_| {})
    }

    /// The writer calls `diagnostic` without holding a recorder lock. The sink must not block.
    pub fn with_diagnostics(ring_bytes: usize, diagnostic: fn(&str)) -> Self {
        Self {
            shared: Arc::new(Shared {
                ring: Mutex::new(Ring {
                    buf: vec![0; ring_bytes],
                    head: 0,
                    used: 0,
                    queued_records: 0,
                    stop: false,
                    opening: false,
                    session: false,
                    finished: true,
                    path: None,
                    accepted_bytes: 0,
                    byte_limit: RECORDING_BYTES,
                    free_bytes: None,
                    reserve_bytes: None,
                    reason: None,
                    error: None,
                    stats: Stats::default(),
                }),
                cond: Condvar::new(),
                active: AtomicBool::new(false),
                diagnostic,
            }),
        }
    }

    /// Creates `path` exclusively (never overwrites). Opening and writing its header can block:
    /// callers must not hold the daemon's control lock here.
    pub fn start(&self, path: &Path, start_host_ns: u64) -> Result<(), String> {
        self.start_with_frames(path, start_host_ns, false)
    }

    pub fn start_with_frames(
        &self,
        path: &Path,
        start_host_ns: u64,
        frames: bool,
    ) -> Result<(), String> {
        self.reserve_with_frames(path, frames)?.start(start_host_ns)
    }

    /// Reserve under the control lock; `ReservedRecording::start` must run after releasing it.
    pub fn reserve_with_frames(
        &self,
        path: &Path,
        frames: bool,
    ) -> Result<ReservedRecording<'_>, String> {
        self.reserve_session(
            path,
            if frames {
                FRAME_RECORDING_BYTES
            } else {
                RECORDING_BYTES
            },
        )?;
        Ok(ReservedRecording {
            recorder: self,
            path: path.to_path_buf(),
            started: false,
        })
    }

    /// Records into an already opened sink, with the same lifecycle and policy as a file.
    pub fn start_with_sink(
        &self,
        path: &Path,
        start_host_ns: u64,
        frames: bool,
        sink: Box<dyn RecordingSink>,
    ) -> Result<(), String> {
        self.start_session(
            path,
            start_host_ns,
            if frames {
                FRAME_RECORDING_BYTES
            } else {
                RECORDING_BYTES
            },
            || Ok(sink),
        )
    }

    fn start_session(
        &self,
        path: &Path,
        start_host_ns: u64,
        byte_limit: u64,
        open: impl FnOnce() -> io::Result<Box<dyn RecordingSink>>,
    ) -> Result<(), String> {
        self.reserve_session(path, byte_limit)?;
        self.start_reserved_session(path, start_host_ns, open)
    }

    fn start_reserved_session(
        &self,
        path: &Path,
        start_host_ns: u64,
        open: impl FnOnce() -> io::Result<Box<dyn RecordingSink>>,
    ) -> Result<(), String> {
        let cancelled = self.shared.ring.lock().unwrap().stop;
        if cancelled {
            self.cancel_start();
            return Err("recording start was cancelled before its file was opened".into());
        }
        let start = || -> Result<(), String> {
            let mut file = open().map_err(|e| format!("open {}: {e}", path.display()))?;
            write_header(&self.shared, &mut *file, path, start_host_ns)?;
            self.spawn_writer(path.to_path_buf(), || Ok(file), None, None)
        };
        if let Err(error) = start() {
            self.start_failed(&error);
            return Err(error);
        }
        Ok(())
    }

    /// Reserves the session before returning; opening and the header run on its writer.
    /// Unlike `start`, the caller must check the returned header outcome before proceeding.
    pub fn start_in_background(&self, path: &Path, start_host_ns: u64) -> Result<Starting, String> {
        let file_path = path.to_path_buf();
        self.start_in_background_with(path, start_host_ns, move || {
            let file = OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&file_path)?;
            Ok(Box::new(FileSink {
                file,
                path: file_path,
            }))
        })
    }

    /// Injected storage uses the same reservation, header and writer as a background file start.
    pub fn start_in_background_with(
        &self,
        path: &Path,
        start_host_ns: u64,
        open: impl FnOnce() -> io::Result<Box<dyn RecordingSink>> + Send + 'static,
    ) -> Result<Starting, String> {
        self.reserve_session(path, RECORDING_BYTES)?;
        let (ready, result) = mpsc::channel();
        let finished = Arc::new(Mutex::new(None));
        if let Err(error) = self.spawn_writer(
            path.to_path_buf(),
            open,
            Some((start_host_ns, ready)),
            Some(Arc::clone(&finished)),
        ) {
            self.start_failed(&error);
            return Err(error);
        }
        Ok(Starting {
            ready: result,
            header: Mutex::new(None),
            finished,
        })
    }

    fn reserve_session(&self, path: &Path, byte_limit: u64) -> Result<(), String> {
        {
            let mut ring = self.shared.ring.lock().unwrap();
            if ring.session {
                let message = if ring.finished {
                    "previous recording ended; run record stop to see its result before starting another"
                } else if ring.stop {
                    "recording is still finishing; check status and stop it before starting another"
                } else {
                    "already recording"
                };
                return Err(message.into());
            }
            ring.head = 0;
            ring.used = 0;
            ring.queued_records = 0;
            ring.stop = false;
            ring.opening = true;
            ring.session = true;
            ring.finished = false;
            ring.path = Some(path.to_path_buf());
            ring.accepted_bytes = FILE_HEADER_LEN as u64;
            ring.byte_limit = byte_limit;
            ring.free_bytes = None;
            ring.reserve_bytes = None;
            ring.reason = None;
            ring.error = None;
            ring.stats = Stats::default();
        }
        Ok(())
    }

    fn cancel_start(&self) {
        self.shared.active.store(false, Ordering::Release);
        let mut ring = self.shared.ring.lock().unwrap();
        ring.stop = true;
        ring.opening = false;
        ring.finished = true;
        ring.session = false;
        ring.error = Some("recording start was cancelled before its file was opened".into());
        drop(ring);
        self.shared.cond.notify_all();
    }

    fn start_failed(&self, error: &str) {
        self.shared.active.store(false, Ordering::Release);
        let mut ring = self.shared.ring.lock().unwrap();
        ring.fail(error.into());
        ring.session = false;
        ring.finished = true;
        drop(ring);
        self.shared.cond.notify_all();
    }

    fn spawn_writer(
        &self,
        path: PathBuf,
        open: impl FnOnce() -> io::Result<Box<dyn RecordingSink>> + Send + 'static,
        mut starting: Option<(u64, mpsc::Sender<Result<(), String>>)>,
        finished: Option<FinalRecordingStatus>,
    ) -> Result<(), String> {
        let shared = Arc::clone(&self.shared);
        std::thread::Builder::new()
            .name("cv1t-recorder".into())
            .spawn(move || {
                let mut file = None;
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    file = Some(open().map_err(|e| format!("open {}: {e}", path.display()))?);
                    let sink = file.as_deref_mut().unwrap();
                    if let Some((start_host_ns, _)) = &starting {
                        write_header(&shared, sink, &path, *start_host_ns)?;
                    }
                    if let Some((_, ready)) = starting.take() {
                        let _ = ready.send(Ok(()));
                    }
                    writer_main(&shared, sink);
                    Ok::<(), String>(())
                }));
                let error = match result {
                    Ok(Ok(())) => None,
                    Ok(Err(error)) => Some(error),
                    Err(_) => Some("recording writer stopped unexpectedly".into()),
                };
                if let Some(error) = error {
                    shared.active.store(false, Ordering::Release);
                    shared.ring.lock().unwrap().fail(error);
                }
                let failure = shared.ring.lock().unwrap().error.clone();
                if let Some(error) = failure {
                    // The writer owns its diagnostic, including if closing the file stalls.
                    // Report a header failure before publishing its outcome to startup.
                    shared.report_diagnostic(&format!("recording failed: {error}"));
                    if let Some((_, ready)) = starting.take() {
                        let _ = ready.send(Err(error));
                    }
                }
                // Closing also happens before `finished`, without a ring or control lock.
                drop(file);
                shared.active.store(false, Ordering::Release);
                let note = {
                    let ring = shared.ring.lock().unwrap();
                    (ring.error.is_none() && ring.reason.is_some()).then(|| {
                        let mut status = ring.status();
                        status.finished = true;
                        status.message()
                    })
                };
                if let Some(note) = note {
                    shared.report_diagnostic(&note);
                }
                let mut ring = shared.ring.lock().unwrap();
                ring.finished = true;
                if let Some(finished) = finished {
                    *finished.lock().unwrap() = Some(ring.status());
                }
                drop(ring);
                shared.cond.notify_all();
            })
            .map(|_| ())
            .map_err(|e| format!("cannot start recorder thread: {e}"))
    }

    pub fn has_session(&self) -> bool {
        self.shared.ring.lock().unwrap().session
    }
    pub fn active(&self) -> bool {
        self.shared.active.load(Ordering::Acquire)
    }
    pub fn status(&self) -> Status {
        self.shared.ring.lock().unwrap().status()
    }

    /// Thread-safe and I/O-free. The byte ceiling includes the header, queued records and any
    /// batch the writer has taken, so even a stalled writer cannot overshoot it.
    pub fn write(&self, rtype: u16, host_ns: u64, parts: &[&[u8]]) {
        if !self.active() {
            return;
        }
        let payload_len: usize = parts.iter().map(|p| p.len()).sum();
        let len = RECORD_HEADER_LEN + payload_len;
        let mut header = [0u8; RECORD_HEADER_LEN];
        header[..2].copy_from_slice(&rtype.to_le_bytes());
        header[4..8].copy_from_slice(&(payload_len as u32).to_le_bytes());
        header[8..16].copy_from_slice(&host_ns.to_le_bytes());
        let mut ring = self.shared.ring.lock().unwrap();
        if !self.active() {
            return;
        }
        if ring.accepted_bytes.saturating_add(len as u64) > ring.byte_limit {
            self.limit_reached(&mut ring);
        } else if ring.buf.len() - ring.used < len {
            ring.stats.dropped += 1;
            return;
        } else {
            ring.put(&header);
            for p in parts {
                ring.put(p);
            }
            ring.stats.records += 1;
            ring.queued_records += 1;
            ring.accepted_bytes += len as u64;
            if ring.accepted_bytes == ring.byte_limit {
                self.limit_reached(&mut ring);
            }
        }
        drop(ring);
        self.shared.cond.notify_all();
    }

    fn limit_reached(&self, ring: &mut Ring) {
        let limit = if ring.byte_limit >= 1 << 30 {
            format!("{} GiB", ring.byte_limit >> 30)
        } else {
            format!("{} bytes", ring.byte_limit)
        };
        ring.reason = Some(format!("{limit} recording size limit reached"));
        ring.stop = true;
        self.shared.active.store(false, Ordering::Release);
    }

    pub fn stats(&self) -> Stats {
        self.shared.ring.lock().unwrap().stats
    }

    /// Offline callers may wait for all storage. Daemon control/shutdown use `stop_timeout`.
    pub fn stop(&self) -> Stats {
        self.stop_wait(None, None)
            .map_or_else(|| self.stats(), |s| s.stats)
    }
    pub fn stop_if(&self, path: &Path) -> Option<Stats> {
        self.stop_wait(Some(path), None).map(|s| s.stats)
    }

    /// Requests a drain without waiting for opening or storage. This also cancels a start
    /// whose header write has not returned yet; it must not become active afterward.
    pub fn request_stop(&self, path: Option<&Path>) -> bool {
        let mut ring = self.shared.ring.lock().unwrap();
        if !ring.session || path.is_some_and(|p| ring.path.as_deref() != Some(p)) {
            return false;
        }
        self.shared.active.store(false, Ordering::Release);
        ring.stop = true;
        self.shared.cond.notify_all();
        true
    }

    /// Stops accepting records immediately. A timeout retains ownership and the outcome for a
    /// later stop; a new start is refused until a completed stop acknowledges this session.
    pub fn stop_timeout(&self, path: Option<&Path>, timeout: Duration) -> Option<Status> {
        self.stop_wait(path, Some(timeout))
    }

    fn stop_wait(&self, path: Option<&Path>, timeout: Option<Duration>) -> Option<Status> {
        let mut ring = self.shared.ring.lock().unwrap();
        if !ring.session || path.is_some_and(|p| ring.path.as_deref() != Some(p)) {
            return None;
        }
        self.shared.active.store(false, Ordering::Release);
        ring.stop = true;
        self.shared.cond.notify_all();
        ring = if let Some(timeout) = timeout {
            self.shared
                .cond
                .wait_timeout_while(ring, timeout, |r| !r.finished)
                .unwrap()
                .0
        } else {
            self.shared.cond.wait_while(ring, |r| !r.finished).unwrap()
        };
        if ring.finished {
            ring.session = false;
        }
        Some(ring.status())
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        // A blocked filesystem cannot be interrupted safely. Its one writer owns Shared until it
        // finishes; dropping the recorder must not join it or delay daemon shutdown.
        self.shared.active.store(false, Ordering::Release);
        self.shared.ring.lock().unwrap().stop = true;
        self.shared.cond.notify_all();
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Record {
    pub rtype: u16,
    pub host_ns: u64,
    pub payload: Vec<u8>,
}

pub struct Reader {
    file: BufReader<File>,
    pub start_host_ns: u64,
    /// Flight recorder contention losses in the saved window; zero in old/other recordings,
    /// UNKNOWN_FLIGHT_SKIPS when relevant timestamp history is unavailable.
    pub flight_skipped_records: u64,
    error: Option<String>,
}

impl Reader {
    pub fn open(path: &Path) -> Result<Self, String> {
        let mut file =
            BufReader::new(File::open(path).map_err(|e| format!("{}: {e}", path.display()))?);
        let mut header = [0u8; FILE_HEADER_LEN];
        if file.read_exact(&mut header).is_err() || &header[..8] != MAGIC {
            return Err(format!("{}: not a cv1touch recording", path.display()));
        }
        let version = u32::from_le_bytes(header[8..12].try_into().unwrap());
        if version != VERSION {
            return Err(format!(
                "{}: recording version {version}, expected {VERSION}",
                path.display()
            ));
        }
        Ok(Self {
            file,
            start_host_ns: u64::from_le_bytes(header[16..24].try_into().unwrap()),
            flight_skipped_records: u64::from_le_bytes(header[24..32].try_into().unwrap()),
            error: None,
        })
    }
    /// A malformed record or I/O failure encountered while iterating. A truncated final
    /// record remains an accepted crash tail.
    pub fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }
}

impl Iterator for Reader {
    type Item = Record;

    /// Stops at end of file or at a truncated final record (crash mid-write).
    fn next(&mut self) -> Option<Record> {
        if self.error.is_some() {
            return None;
        }
        let mut h = [0u8; RECORD_HEADER_LEN];
        if let Err(e) = self.file.read_exact(&mut h) {
            if e.kind() != std::io::ErrorKind::UnexpectedEof {
                self.error = Some(e.to_string());
            }
            return None;
        }
        let rtype = u16::from_le_bytes(h[..2].try_into().unwrap());
        let len = u32::from_le_bytes(h[4..8].try_into().unwrap()) as usize;
        let host_ns = u64::from_le_bytes(h[8..16].try_into().unwrap());
        if len > MAX_RECORD_BYTES {
            self.error = Some(format!(
                "record payload length {len} exceeds {MAX_RECORD_BYTES} bytes"
            ));
            return None;
        }
        let mut payload = vec![0u8; len];
        if let Err(e) = self.file.read_exact(&mut payload) {
            if e.kind() != std::io::ErrorKind::UnexpectedEof {
                self.error = Some(e.to_string());
            }
            return None;
        }
        Some(Record {
            rtype,
            host_ns,
            payload,
        })
    }
}

pub fn decode_state(payload: &[u8]) -> Option<(usize, ControllerState)> {
    if payload.len() != 8 + size_of::<ControllerState>() {
        return None;
    }
    let hand = u32::from_le_bytes(payload[..4].try_into().ok()?) as usize;
    // SAFETY: length checked; unaligned read of a POD struct.
    let state = unsafe { ptr::read_unaligned(payload[8..].as_ptr() as *const ControllerState) };
    Some((hand, state))
}

pub fn decode_raw(payload: &[u8]) -> Option<(u32, &[u8])> {
    let source = u32::from_le_bytes(payload.get(..4)?.try_into().ok()?);
    Some((source, &payload[4..]))
}

fn read_pod<T: Copy>(bytes: &[u8]) -> Option<T> {
    // SAFETY: length checked; unaligned read of a POD struct from the C ABI.
    (bytes.len() >= size_of::<T>())
        .then(|| unsafe { ptr::read_unaligned(bytes.as_ptr() as *const T) })
}

fn read_array<T: Copy>(bytes: &[u8], n: usize) -> Option<Vec<T>> {
    (bytes.len() == n * size_of::<T>()).then(|| {
        (0..n)
            .map(|i| read_pod(&bytes[i * size_of::<T>()..]).unwrap())
            .collect()
    })
}

pub fn decode_frame_obs(payload: &[u8]) -> Option<(FrameInfo, Vec<Blob>)> {
    let info: FrameInfo = read_pod(payload)?;
    let body = &payload[size_of::<FrameInfo>()..];
    if body.len() % size_of::<Blob>() != 0 {
        return None;
    }
    Some((info, read_array(body, body.len() / size_of::<Blob>())?))
}

/// A GRAY8 frame as its rows' runs of pixels above `floor`: tracking frames are black but for the
/// blobs, so this keeps every visible pixel at a few kilobytes a frame. Payload: the frame info,
/// width, height (u16 each), floor (u8), three pad bytes, then (y, x, n: u16; n pixels) per run.
pub fn encode_frame(
    info: &FrameInfo,
    width: usize,
    height: usize,
    stride: usize,
    pixels: &[u8],
    floor: u8,
) -> Vec<u8> {
    let mut out = Vec::with_capacity(size_of::<FrameInfo>() + 8 + 4096);
    out.extend_from_slice(as_bytes(info));
    out.extend_from_slice(&(width as u16).to_le_bytes());
    out.extend_from_slice(&(height as u16).to_le_bytes());
    out.extend_from_slice(&[floor, 0, 0, 0]);
    for y in 0..height.min(pixels.len() / stride.max(1)) {
        let row = &pixels[y * stride..y * stride + width.min(stride)];
        let mut x = 0;
        while x < row.len() {
            if row[x] <= floor {
                x += 1;
                continue;
            }
            let start = x;
            while x < row.len() && row[x] > floor && x - start < u16::MAX as usize {
                x += 1;
            }
            out.extend_from_slice(&(y as u16).to_le_bytes());
            out.extend_from_slice(&(start as u16).to_le_bytes());
            out.extend_from_slice(&((x - start) as u16).to_le_bytes());
            out.extend_from_slice(&row[start..x]);
        }
    }
    out
}

/// The frame info and the full image (pixels at or below the floor read 0).
pub fn decode_raw_frame(payload: &[u8]) -> Option<(FrameInfo, usize, usize, Vec<u8>)> {
    let info: FrameInfo = read_pod(payload)?;
    let mut p = &payload[size_of::<FrameInfo>()..];
    let u16_at = |b: &[u8], i: usize| u16::from_le_bytes([b[i], b[i + 1]]) as usize;
    if p.len() < 8 {
        return None;
    }
    let (w, h) = (u16_at(p, 0), u16_at(p, 2));
    p = &p[8..];
    if w == 0 || h == 0 || w * h > MAX_FRAME_PIXELS {
        return None;
    }
    let mut img = vec![0u8; w * h];
    while !p.is_empty() {
        if p.len() < 6 {
            return None;
        }
        let (y, x, n) = (u16_at(p, 0), u16_at(p, 2), u16_at(p, 4));
        if p.len() < 6 + n || y >= h || x + n > w {
            return None;
        }
        img[y * w + x..y * w + x + n].copy_from_slice(&p[6..6 + n]);
        p = &p[6 + n..];
    }
    Some((info, w, h, img))
}

pub fn decode_sensor_info(payload: &[u8]) -> Option<SensorInfo> {
    (payload.len() == size_of::<SensorInfo>())
        .then(|| read_pod(payload))
        .flatten()
}

pub fn decode_led_model(payload: &[u8]) -> Option<(i32, Vec<LedEntry>)> {
    let device = u32::from_le_bytes(payload.get(..4)?.try_into().ok()?) as i32;
    let n = u32::from_le_bytes(payload.get(4..8)?.try_into().ok()?) as usize;
    Some((device, read_array(&payload[8..], n)?))
}

#[cfg(test)]
mod frame_tests {
    use super::*;

    #[test]
    fn hotplug_legacy_sensor_padding_accepts_every_byte_without_a_rust_bool() {
        // Check the ABI type BEFORE decoding a legacy invalid-bool representation.
        // This makes the old-layout red proof safe to run too.
        assert_eq!(
            std::any::type_name_of_val(&SensorInfo::default().usb_gone),
            "u8"
        );
        for padding in 2..=255u8 {
            let mut bytes = vec![0u8; size_of::<SensorInfo>()];
            bytes[33] = padding;
            bytes[64..68].copy_from_slice(&17u32.to_le_bytes());
            let s = decode_sensor_info(&bytes).unwrap();
            assert_eq!(format!("{:?}", s.usb_gone), padding.to_string());
            assert_eq!(s.frames, 17);
        }
    }

    /// A black frame with two blobs, noise at the floor, and a stride wider than the row: every
    /// pixel above the floor comes back, nothing else, in a few hundred bytes.
    #[test]
    fn a_frame_round_trips_through_its_runs() {
        let (w, h, stride) = (1280usize, 960usize, 1288usize);
        let mut px = vec![0u8; stride * h];
        for (i, v) in px.iter_mut().enumerate() {
            *v = (i % 7 == 0) as u8 * 2; // noise at the floor
        }
        for (cx, cy) in [(100usize, 200usize), (1275, 959)] {
            for y in cy.saturating_sub(3)..(cy + 4).min(h) {
                for x in cx.saturating_sub(3)..(cx + 4).min(w) {
                    px[y * stride + x] = 250
                        - ((x as i32 - cx as i32).abs() * 30 + (y as i32 - cy as i32).abs() * 30)
                            as u8;
                }
            }
        }
        px[5 * stride + w + 2] = 255; // in the stride padding: not part of the image
        let info = FrameInfo {
            frame_start_host_ns: 123_456_789,
            exposure_hmd_ts: 42,
            ..Default::default()
        };
        let payload = encode_frame(&info, w, h, stride, &px, 2);
        assert!(
            payload.len() < size_of::<FrameInfo>() + 1000,
            "{} bytes",
            payload.len()
        );
        let (back, bw, bh, img) = decode_raw_frame(&payload).unwrap();
        assert_eq!(
            (back.frame_start_host_ns, back.exposure_hmd_ts, bw, bh),
            (123_456_789, 42, w, h)
        );
        for y in 0..h {
            for x in 0..w {
                let v = px[y * stride + x];
                assert_eq!(
                    img[y * w + x],
                    if v > 2 { v } else { 0 },
                    "pixel ({x}, {y})"
                );
            }
        }
        assert!(
            decode_raw_frame(&payload[..payload.len() - 1]).is_none(),
            "a truncated run is refused"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shm::HAND_RIGHT;
    use std::ffi::c_char;
    use std::path::PathBuf;

    fn tmp(tag: &str) -> PathBuf {
        let p =
            std::env::temp_dir().join(format!("cv1t-rec-test-{}-{tag}.rec", std::process::id()));
        let _ = std::fs::remove_file(&p);
        p
    }

    fn blobs() -> [Blob; 2] {
        [
            Blob {
                x: 10.5,
                y: 20.25,
                width: 3,
                height: 4,
                area: 12,
                led_id: (1 << 8) | 7,
            },
            Blob {
                x: 100.0,
                y: 200.0,
                width: 5,
                height: 6,
                area: 30,
                led_id: -1,
            },
        ]
    }

    fn write_three(r: &Recorder) {
        let state = ControllerState {
            flags: 7,
            pos: [1.25, 0.0, 0.0],
            ..Default::default()
        };
        r.write(
            CONTROLLER_STATE,
            222,
            &[
                &(HAND_RIGHT as u32).to_le_bytes(),
                &[0u8; 4],
                as_bytes(&state),
            ],
        );
        r.write(RAW_REPORT, 333, &[&1u32.to_le_bytes(), &[1, 2, 3, 4, 5]]);
        let mut info = FrameInfo::default();
        info.serial[..3].copy_from_slice(&[b'W' as c_char, b'M' as c_char, b'T' as c_char]);
        let b = blobs();
        r.write(FRAME_OBS, 444, &[as_bytes(&info), slice_bytes(&b)]);
    }

    #[test]
    fn a_cancelled_start_never_claims_a_completed_file() {
        for stopped in [false, true] {
            let r = Recorder::new(4096);
            let path = std::env::temp_dir().join(format!(
                "cv1t-cancelled-{}-{}.rec",
                std::process::id(),
                crate::shm::now_ns()
            ));
            let reserved = r.reserve_with_frames(&path, false).unwrap();
            if stopped {
                assert!(r.request_stop(None));
                assert!(reserved.start(1).unwrap_err().contains("cancelled"));
            } else {
                drop(reserved);
            }
            let status = r.status();
            assert!(!path.exists());
            assert!(!r.has_session());
            assert_eq!(status.stats.write_errors, 0);
            assert!(
                status.message().contains("cancelled"),
                "{}",
                status.message()
            );
            assert!(
                !status.message().contains("complete and synced"),
                "{}",
                status.message()
            );
        }
    }

    #[test]
    fn a_bare_recording_filename_syncs_and_reads_back() {
        let path = PathBuf::from(format!(
            "cv1t-bare-{}-{}.rec",
            std::process::id(),
            crate::shm::now_ns()
        ));
        let r = Recorder::new(4096);
        r.start(&path, 1).unwrap();
        r.write(HAPTIC, 2, &[&[0; 16]]);
        let status = r.stop_timeout(None, Duration::from_secs(2)).unwrap();
        let mut reader = Reader::open(&path).unwrap();
        let count = reader.by_ref().count();
        std::fs::remove_file(&path).unwrap();
        assert!(
            status.finished && status.error.is_none(),
            "{}",
            status.message()
        );
        assert_eq!(count, 1);
        assert!(reader.error().is_none());
    }

    #[test]
    fn startup_final_status_survives_acknowledgement_and_replacement() {
        struct FailedFinalSync;
        impl Write for FailedFinalSync {
            fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
                Ok(bytes.len())
            }
            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }
        impl RecordingSink for FailedFinalSync {
            fn sync_all(&mut self) -> io::Result<()> {
                Err(io::Error::other("test original sync failed"))
            }
            fn disk_space(&self) -> io::Result<DiskSpace> {
                Ok(DiskSpace {
                    available: 100 << 30,
                    capacity: 200 << 30,
                })
            }
        }
        let r = Recorder::new(4096);
        let starting = r
            .start_in_background_with(Path::new("unused-original.rec"), 1, || {
                Ok(Box::new(FailedFinalSync))
            })
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            if let Some(header) = starting.poll() {
                header.unwrap();
                break;
            }
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(r
            .stop_timeout(None, Duration::from_secs(2))
            .unwrap()
            .error
            .unwrap()
            .contains("test original sync failed"));
        // Completion must already be published when stop acknowledges and releases the slot.
        assert!(starting
            .final_status()
            .unwrap()
            .error
            .unwrap()
            .contains("test original sync failed"));
        assert_eq!(starting.poll(), Some(Ok(())));
        r.start_with_sink(
            Path::new("unused-replacement.rec"),
            2,
            false,
            Box::new(CountingSpace {
                queries: Arc::new(std::sync::atomic::AtomicU64::new(0)),
                available: Arc::new(std::sync::atomic::AtomicU64::new(100 << 30)),
            }),
        )
        .unwrap();
        assert!(r.status().error.is_none());
        assert!(starting
            .final_status()
            .unwrap()
            .error
            .unwrap()
            .contains("test original sync failed"));
        assert!(r
            .stop_timeout(None, Duration::from_secs(2))
            .unwrap()
            .error
            .is_none());
    }

    #[test]
    fn a_panicking_diagnostic_keeps_the_startup_error_and_finishes() {
        let r = Recorder::with_diagnostics(4096, |_| panic!("test diagnostic failed"));
        let starting = r
            .start_in_background_with(Path::new("unused.rec"), 1, || {
                Err(io::Error::other("test recording open failed"))
            })
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        let outcome = loop {
            if let Some(outcome) = starting.poll() {
                break outcome;
            }
            assert!(
                Instant::now() < deadline,
                "startup did not report its outcome"
            );
            std::thread::sleep(Duration::from_millis(1));
        };
        let status = r.stop_timeout(None, Duration::from_millis(100)).unwrap();
        assert!(
            status.finished,
            "a diagnostic failure left the recording unfinished"
        );
        assert!(outcome.unwrap_err().contains("test recording open failed"));
        assert_eq!(status.stats.write_errors, 1);
    }

    struct CountingSpace {
        queries: Arc<std::sync::atomic::AtomicU64>,
        available: Arc<std::sync::atomic::AtomicU64>,
    }
    impl Write for CountingSpace {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            Ok(bytes.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    impl RecordingSink for CountingSpace {
        fn sync_all(&mut self) -> io::Result<()> {
            Ok(())
        }
        fn disk_space(&self) -> io::Result<DiskSpace> {
            self.queries.fetch_add(1, Ordering::Relaxed);
            Ok(DiskSpace {
                available: self.available.load(Ordering::Relaxed),
                capacity: 40 << 30,
            })
        }
    }

    #[test]
    fn disk_queries_follow_time_bytes_and_remaining_allowance() {
        let queries = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let available = Arc::new(std::sync::atomic::AtomicU64::new(100 << 30));
        let sink = CountingSpace {
            queries: Arc::clone(&queries),
            available: Arc::clone(&available),
        };
        let mut allowance = DiskAllowance::default();
        let now = Instant::now();
        for _ in 0..1000 {
            assert!(allowance.permits(&sink, 48, now).unwrap());
            allowance.wrote(48);
        }
        assert_eq!(queries.load(Ordering::Relaxed), 1);
        assert_eq!(allowance.available, (100 << 30) - 48_000);
        available.store(1 << 30, Ordering::Relaxed);
        assert!(allowance
            .permits(&sink, 48, now + Duration::from_millis(999))
            .unwrap());
        assert!(!allowance
            .permits(&sink, 48, now + Duration::from_secs(1))
            .unwrap());
        assert_eq!(queries.load(Ordering::Relaxed), 2);
        available.store(100 << 30, Ordering::Relaxed);
        assert!(allowance
            .permits(&sink, 48, now + Duration::from_secs(1))
            .unwrap());
        assert_eq!(
            queries.load(Ordering::Relaxed),
            3,
            "crossing the cached reserve refreshes immediately"
        );
        allowance.wrote(16 << 20);
        assert!(allowance
            .permits(&sink, 48, now + Duration::from_secs(1))
            .unwrap());
        assert_eq!(
            queries.load(Ordering::Relaxed),
            4,
            "16 MiB refreshes without waiting a second"
        );
    }

    #[test]
    fn small_drained_batches_do_not_query_disk_space_per_sample() {
        let queries = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let r = Recorder::new(4096);
        let begun = Instant::now();
        r.start_with_sink(
            &tmp("query-cadence"),
            1,
            false,
            Box::new(CountingSpace {
                queries: Arc::clone(&queries),
                available: Arc::new(std::sync::atomic::AtomicU64::new(100 << 30)),
            }),
        )
        .unwrap();
        for i in 1..=100 {
            r.write(HAPTIC, i, &[&[0; 16]]);
            wait_until(|| r.stats().bytes == 32 + i * 32);
        }
        r.stop();
        let calls = queries.load(Ordering::Relaxed);
        assert!(
            calls <= 3 + begun.elapsed().as_secs(),
            "{calls} filesystem queries for 100 small batches"
        );
    }

    struct SpaceSink {
        file: File,
        available: Arc<std::sync::atomic::AtomicU64>,
    }
    impl Write for SpaceSink {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            let n = self.file.write(bytes)?;
            self.available.fetch_sub(n as u64, Ordering::Relaxed);
            Ok(n)
        }
        fn flush(&mut self) -> io::Result<()> {
            self.file.flush()
        }
    }
    impl RecordingSink for SpaceSink {
        fn sync_all(&mut self) -> io::Result<()> {
            self.file.sync_all()
        }
        fn disk_space(&self) -> io::Result<DiskSpace> {
            Ok(DiskSpace {
                available: self.available.load(Ordering::Relaxed),
                capacity: 40 << 30,
            })
        }
    }
    fn state_record(r: &Recorder, time: u64) {
        let state = ControllerState {
            flags: crate::hand::FLAG_CONNECTED,
            device_time_ns: time,
            received_host_ns: time,
            publish_host_ns: time + 10,
            ..Default::default()
        };
        r.write(
            CONTROLLER_STATE,
            time,
            &[&0u32.to_le_bytes(), &0u32.to_le_bytes(), as_bytes(&state)],
        );
    }
    fn wait_until(mut predicate: impl FnMut() -> bool) {
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        while !predicate() {
            assert!(
                std::time::Instant::now() < deadline,
                "writer did not progress"
            );
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    #[test]
    fn size_limit_finishes_a_readable_recording_for_summary_and_replay() {
        for limit in [872, 900] {
            let path = tmp(&format!("capped-{limit}"));
            let r = Recorder::new(4096);
            let available = Arc::new(std::sync::atomic::AtomicU64::new(100 << 30));
            r.start_session(&path, 1, limit, || {
                Ok(Box::new(SpaceSink {
                    file: File::create(&path)?,
                    available,
                }))
            })
            .unwrap();
            for time in 1..=3 {
                state_record(&r, time);
            }
            if limit == 872 {
                assert!(
                    !r.active(),
                    "an exactly full file must stop without a fourth sample"
                );
            }
            state_record(&r, 4);
            wait_until(|| r.status().finished);
            assert!(!r.active());
            // A capped session retains its reason until stop acknowledges it.
            assert!(r.start(&tmp("capped-replacement"), 2).is_err());
            let status = r.stop_timeout(None, Duration::from_secs(2)).unwrap();
            assert_eq!(status.state, State::Complete);
            assert!(status.reason.unwrap().contains("size limit"));
            assert_eq!(status.stats.bytes, 872); // 32-byte header + 3 x (16 + 8 + 256).
            assert_eq!(status.stats.write_errors, 0);
            assert_eq!(std::fs::metadata(&path).unwrap().len(), 872);
            let mut reader = Reader::open(&path).unwrap();
            let states: Vec<_> = reader
                .by_ref()
                .map(|r| {
                    let (hand, state) = decode_state(&r.payload).unwrap();
                    (r.host_ns, hand, state)
                })
                .collect();
            assert_eq!(states.len(), 3);
            assert!(reader.error().is_none());
            let window = crate::summary::Window {
                name: "capture".into(),
                start_ns: 1,
                end_ns: 4,
            };
            let summary = crate::summary::window_stats(&states, 0, &window);
            assert_eq!(summary.n, 3);
            assert_eq!(summary.connected, 1.0);
            // No camera/model metadata in this small fixture: replay accepts the capped file
            // but has no optical track to publish. The pipeline's full replay regression covers
            // the sensor/model/frame records in this same unchanged format.
            let calibration = crate::calib::file::CalibrationFile {
                format: crate::calib::file::FORMAT.into(),
                version: 1,
                created_unix_s: 0,
                world: String::new(),
                source: String::new(),
                sensors: Vec::new(),
                stats: crate::calib::file::Stats {
                    reference: String::new(),
                    gravity_mps2: 9.81,
                    gravity_spread_deg: None,
                    motion_segments: 0,
                    samples: 0,
                    rejected_views: 0,
                },
            };
            let replay =
                crate::track::replay::replay(&path, &calibration, &|_| None, &Default::default())
                    .unwrap();
            assert!(replay.hands.iter().all(Vec::is_empty));
            std::fs::remove_file(path).unwrap();
        }
    }

    #[test]
    fn low_disk_space_finishes_only_whole_records_and_says_how_much_is_kept() {
        let path = tmp("disk-reserve");
        let r = Recorder::new(4096);
        let available = Arc::new(std::sync::atomic::AtomicU64::new((2 << 30) + 591));
        r.start_session(&path, 1, RECORDING_BYTES, || {
            Ok(Box::new(SpaceSink {
                file: File::create(&path)?,
                available: Arc::clone(&available),
            }))
        })
        .unwrap();
        state_record(&r, 1);
        wait_until(|| r.stats().bytes == 312);
        // Header and first record spent 312 bytes: one byte short of another whole record.
        assert_eq!(available.load(Ordering::Relaxed), (2 << 30) + 279);
        state_record(&r, 2);
        wait_until(|| r.status().finished);
        let status = r.stop_timeout(None, Duration::from_secs(2)).unwrap();
        assert_eq!(status.state, State::Complete);
        assert!(status.message().contains("GiB free; keeping 2.00 GiB free"));
        assert_eq!(status.stats.bytes, 312);
        assert_eq!(status.stats.records, 1);
        assert_eq!(status.stats.write_errors, 0);
        assert_eq!(std::fs::metadata(&path).unwrap().len(), 312);
        let mut reader = Reader::open(&path).unwrap();
        assert_eq!(reader.by_ref().count(), 1);
        assert!(reader.error().is_none());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn reserve_refuses_a_start_without_enough_space_and_is_capped_on_large_disks() {
        let path = tmp("reserve-refusal");
        let r = Recorder::new(4096);
        let available = Arc::new(std::sync::atomic::AtomicU64::new(1 << 30));
        let error = r
            .start_session(&path, 1, RECORDING_BYTES, || {
                Ok(Box::new(SpaceSink {
                    file: File::create(&path)?,
                    available,
                }))
            })
            .unwrap_err();
        assert!(
            error.contains("1.00 GiB free; keeping 2.00 GiB free"),
            "{error}"
        );
        assert!(!r.active());
        assert_eq!(std::fs::metadata(&path).unwrap().len(), 0);
        std::fs::remove_file(&path).unwrap();
        struct CapacitySink(SpaceSink, u64);
        impl Write for CapacitySink {
            fn write(&mut self, b: &[u8]) -> io::Result<usize> {
                self.0.write(b)
            }
            fn flush(&mut self) -> io::Result<()> {
                self.0.flush()
            }
        }
        impl RecordingSink for CapacitySink {
            fn sync_all(&mut self) -> io::Result<()> {
                self.0.sync_all()
            }
            fn disk_space(&self) -> io::Result<DiskSpace> {
                Ok(DiskSpace {
                    available: self.0.available.load(Ordering::Relaxed),
                    capacity: self.1,
                })
            }
        }
        // The middle of the policy keeps 5%; a multi-terabyte disk keeps at most 20 GiB,
        // admitting a recording with either 21 GiB or hundreds of GiB still free.
        for (free, capacity, reserve, admitted) in [
            (9 << 30, 200 << 30, 10 << 30, false),
            (21 << 30, 8 << 40, 20 << 30, true),
            (420 << 30, 8 << 40, 20 << 30, true),
        ] {
            let available = Arc::new(std::sync::atomic::AtomicU64::new(free));
            let result = r.start_with_sink(
                &path,
                1,
                false,
                Box::new(CapacitySink(
                    SpaceSink {
                        file: File::create(&path).unwrap(),
                        available,
                    },
                    capacity,
                )),
            );
            assert_eq!(r.status().reserve_bytes, Some(reserve));
            if admitted {
                result.unwrap();
                assert_eq!(r.status().byte_limit, 8 << 30);
                state_record(&r, 1);
                let status = r.stop_timeout(None, Duration::from_secs(2)).unwrap();
                assert_eq!(status.stats.records, 1);
                assert!(status.error.is_none() && status.reason.is_none());
            } else {
                let error = result.unwrap_err();
                assert!(
                    error.contains("9.00 GiB free; keeping 10.00 GiB free"),
                    "{error}"
                );
            }
            std::fs::remove_file(&path).unwrap();
        }
    }

    struct SyncGate {
        entered: std::sync::mpsc::Sender<()>,
        release: std::sync::mpsc::Receiver<()>,
    }
    impl Write for SyncGate {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            Ok(bytes.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    impl RecordingSink for SyncGate {
        fn sync_all(&mut self) -> io::Result<()> {
            let _ = self.entered.send(());
            let _ = self.release.recv();
            Ok(())
        }
        fn disk_space(&self) -> io::Result<DiskSpace> {
            Ok(DiskSpace {
                available: 100 << 30,
                capacity: 200 << 30,
            })
        }
    }

    #[test]
    fn dropping_a_recorder_does_not_wait_for_a_blocked_sync() {
        let r = Recorder::new(4096);
        let (entered_tx, entered) = std::sync::mpsc::channel();
        let (release, release_rx) = std::sync::mpsc::channel();
        r.start_with_sink(
            &tmp("drop-blocked"),
            1,
            false,
            Box::new(SyncGate {
                entered: entered_tx,
                release: release_rx,
            }),
        )
        .unwrap();
        assert!(!r.stop_timeout(None, Duration::ZERO).unwrap().finished);
        entered.recv_timeout(Duration::from_secs(2)).unwrap();
        let (done_tx, done) = std::sync::mpsc::channel();
        let dropper = std::thread::spawn(move || {
            drop(r);
            done_tx.send(()).unwrap();
        });
        let dropped = done.recv_timeout(Duration::from_millis(200));
        let _ = release.send(());
        dropper.join().unwrap();
        assert!(dropped.is_ok(), "dropping the recorder waited for storage");
    }

    #[test]
    fn writer_batches_end_at_record_boundaries_even_when_the_ring_wraps() {
        let r = Recorder::new(2 * CHUNK);
        let (first, n) = {
            let mut ring = r.shared.ring.lock().unwrap();
            ring.head = ring.buf.len() - 7;
            for payload_len in [CHUNK - 24, 32] {
                let mut header = [0u8; 16];
                header[4..8].copy_from_slice(&(payload_len as u32).to_le_bytes());
                ring.put(&header);
                ring.put(&vec![0u8; payload_len]);
                ring.queued_records += 1;
            }
            ring.take_records()
        };
        assert_eq!((first.len(), n), (CHUNK - 8, 1));
        let (second, n, left) = {
            let mut ring = r.shared.ring.lock().unwrap();
            let (second, n) = ring.take_records();
            (second, n, (ring.used, ring.queued_records))
        };
        assert_eq!((second.len(), n), (48, 1));
        assert_eq!(left, (0, 0));
    }

    #[test]
    fn scoped_stop_preserves_other_sessions_and_reaps_failed_writer() {
        let path = tmp("scoped-stop");
        let other = tmp("scoped-other");
        let r = Recorder::new(1 << 20);
        assert!(r.stop_if(&path).is_none());
        r.start(&path, 111).unwrap();
        write_three(&r);
        assert!(r.stop_if(&other).is_none());
        assert!(r.active());
        assert!(r.has_session());
        let stats = r.stop_if(&path).unwrap();
        assert_eq!(stats.records, 3);
        assert!(!r.has_session());
        assert!(r.stop_if(&path).is_none());
        r.start(&other, 222).unwrap();
        // Model the writer-error state: active is false but its session still needs reaping.
        r.shared.active.store(false, Ordering::Release);
        r.shared.ring.lock().unwrap().stats.write_errors = 1;
        assert!(r.stop_if(&path).is_none());
        assert!(r.has_session());
        assert_eq!(r.stop_if(&other).unwrap().write_errors, 1);
        assert!(!r.has_session());
        std::fs::remove_file(path).unwrap();
        std::fs::remove_file(other).unwrap();
    }

    #[test]
    fn roundtrip() {
        let path = tmp("roundtrip");
        let r = Recorder::new(1 << 20);
        assert!(!r.active());
        r.start(&path, 111).unwrap();
        assert!(r.active());
        write_three(&r);
        let stats = r.stop();
        assert!(!r.active());
        assert_eq!(
            (stats.records, stats.dropped, stats.write_errors),
            (3, 0, 0)
        );

        let reader = Reader::open(&path).unwrap();
        assert_eq!(reader.start_host_ns, 111);
        assert_eq!(reader.flight_skipped_records, 0);
        let recs: Vec<Record> = reader.collect();
        assert_eq!(
            recs.iter()
                .map(|r| (r.rtype, r.host_ns))
                .collect::<Vec<_>>(),
            [(1, 222), (2, 333), (4, 444)]
        );
        let (hand, state) = decode_state(&recs[0].payload).unwrap();
        assert_eq!((hand, state.flags, state.pos[0]), (HAND_RIGHT, 7, 1.25));
        let (source, data) = decode_raw(&recs[1].payload).unwrap();
        assert_eq!((source, data), (1, &[1u8, 2, 3, 4, 5][..]));
        let (info, got) = decode_frame_obs(&recs[2].payload).unwrap();
        assert_eq!(crate::engine::cstr_field(&info.serial), "WMT");
        assert_eq!((got[0].led_id, got[0].x, got[1].led_id), (263, 10.5, -1));
        assert_eq!(std::fs::metadata(&path).unwrap().len(), stats.bytes);

        // A finished recorder can start a new file.
        let path2 = tmp("roundtrip2");
        r.start(&path2, 1).unwrap();
        r.stop();
        std::fs::remove_file(&path).unwrap();
        std::fs::remove_file(&path2).unwrap();
    }

    /// Review Focus 4: failures are reported, nothing is overwritten, the recorder stays inactive.
    #[test]
    fn start_errors() {
        let r = Recorder::new(1 << 16);
        let missing = std::env::temp_dir()
            .join("nonexistent-cv1t-dir")
            .join("x.rec");
        let err = r.start(&missing, 1).unwrap_err();
        let not_found = std::fs::File::open(&missing).unwrap_err().to_string();
        assert!(err.contains(&not_found), "{err}");
        assert!(!r.active());
        r.write(RAW_REPORT, 1, &[&0u32.to_le_bytes()]);
        assert_eq!(r.stats().records, 0);

        let path = tmp("exists");
        std::fs::write(&path, b"keep me").unwrap();
        let err = r.start(&path, 1).unwrap_err();
        assert!(err.contains("exists"), "{err}");
        assert_eq!(std::fs::read(&path).unwrap(), b"keep me");
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn oversized_record_dropped() {
        let path = tmp("small");
        let r = Recorder::new(64);
        r.start(&path, 1).unwrap();
        let state = ControllerState::default();
        r.write(CONTROLLER_STATE, 1, &[&[0u8; 8], as_bytes(&state)]);
        let stats = r.stop();
        assert_eq!((stats.dropped, stats.records), (1, 0));
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn truncated_tail_is_ignored() {
        let path = tmp("truncated");
        let r = Recorder::new(1 << 20);
        r.start(&path, 1).unwrap();
        write_three(&r);
        r.stop();
        let data = std::fs::read(&path).unwrap();
        std::fs::write(&path, &data[..data.len() - 10]).unwrap(); // crash mid-record
        let hosts: Vec<u64> = Reader::open(&path).unwrap().map(|r| r.host_ns).collect();
        assert_eq!(hosts, [222, 333]);
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn v2_records_round_trip() {
        let path = tmp("v2");
        let r = Recorder::new(1 << 20);
        r.start(&path, 1).unwrap();
        let mut info = FrameInfo::default();
        info.serial[..3].copy_from_slice(&[b'A' as c_char, b'B' as c_char, b'C' as c_char]);
        info.exposure_count = 65535;
        info.n_devices = 1;
        info.devices[0].device_id = 1;
        info.devices[0].pose_valid = 1;
        info.devices[0].cam_pos = [0.1, 0.2, 2.0];
        let blobs = [Blob {
            x: 10.5,
            y: 20.25,
            width: 3,
            height: 4,
            area: 9,
            led_id: (1 << 8) | 7,
        }];
        r.write(FRAME_OBS, 10, &[as_bytes(&info), slice_bytes(&blobs)]);
        let sensor = SensorInfo {
            fx: 715.5,
            k: [0.1, 0.2, 0.3, 0.4],
            ..Default::default()
        };
        r.write(SENSOR_INFO, 11, &[as_bytes(&sensor)]);
        let leds = [LedEntry {
            pos: [0.01, 0.02, 0.03],
            dir: [0.0, 1.0, 0.0],
        }; 2];
        r.write(
            LED_MODEL,
            12,
            &[&led_model_prefix(2, leds.len()), slice_bytes(&leds)],
        );
        r.stop();
        let recs: Vec<Record> = Reader::open(&path).unwrap().collect();
        let (got_info, got_blobs) = decode_frame_obs(&recs[0].payload).unwrap();
        assert_eq!(
            (
                got_info.exposure_count,
                got_info.devices[0].cam_pos,
                got_blobs[0].led_id
            ),
            (65535, [0.1, 0.2, 2.0], 263)
        );
        let s = decode_sensor_info(&recs[1].payload).unwrap();
        assert_eq!((s.fx, s.k), (715.5, [0.1, 0.2, 0.3, 0.4]));
        let (device, model) = decode_led_model(&recs[2].payload).unwrap();
        assert_eq!((device, model.len(), model[1].dir), (2, 2, [0.0, 1.0, 0.0]));
        assert!(decode_frame_obs(&recs[0].payload[..100]).is_none());
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn v2_keeps_old_frames_and_new_identity_lifetimes() {
        let path = tmp("identity-epochs");
        let r = Recorder::new(1 << 20);
        r.start(&path, 1).unwrap();
        let mut info = FrameInfo::default();
        assert_eq!(size_of::<FrameInfo>(), 248);
        assert_eq!(std::mem::size_of_val(&info.devices[0]), 64);
        // Former reserved words remain zero in old v2 recordings.
        r.write(FRAME_OBS, 2, &[as_bytes(&info)]);
        info.devices[0].identity_epoch_lo = 7;
        info.devices[0].identity_epoch_hi = 9;
        info.devices[0].match_flags = crate::engine::MATCH_INSPECTION_IDENTITY;
        let mut event = crate::engine::InspectionIdentity {
            epoch: (9 << 32) | 7,
            device_id: crate::calib::DEVICE_RIGHT,
            state: crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED,
            ..Default::default()
        };
        assert_eq!(std::mem::size_of_val(&event), 48);
        r.write(INSPECTION_IDENTITY, 3, &[as_bytes(&event)]);
        r.write(FRAME_OBS, 4, &[as_bytes(&info)]);
        event.state = crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED;
        r.write(INSPECTION_IDENTITY, 5, &[as_bytes(&event)]);
        r.stop();
        let records: Vec<_> = Reader::open(&path).unwrap().collect();
        let old = decode_frame_obs(&records[0].payload).unwrap().0;
        assert_eq!(
            (
                old.devices[0].identity_epoch_lo,
                old.devices[0].identity_epoch_hi
            ),
            (0, 0)
        );
        let new = decode_frame_obs(&records[2].payload).unwrap().0;
        assert_eq!(
            (
                new.devices[0].identity_epoch_lo,
                new.devices[0].identity_epoch_hi
            ),
            (7, 9)
        );
        let end = decode_inspection_identity(&records[1].payload).unwrap();
        let invalid = decode_inspection_identity(&records[3].payload).unwrap();
        assert_eq!(end.epoch, event.epoch);
        assert_eq!(end.state, crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED);
        assert_eq!(
            invalid.state,
            crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED
        );
        assert!(decode_inspection_identity(&records[1].payload[..47]).is_none());
        assert!(decode_inspection_identity(&[0; 49]).is_none());
        assert_eq!(
            records.iter().map(|r| r.host_ns).collect::<Vec<_>>(),
            [2, 3, 4, 5]
        );
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn reader_rejects_version_1() {
        let path = tmp("v1");
        let mut header = [0u8; 32];
        header[..8].copy_from_slice(MAGIC);
        header[8..12].copy_from_slice(&1u32.to_le_bytes());
        header[12..16].copy_from_slice(&32u32.to_le_bytes());
        std::fs::write(&path, header).unwrap();
        let err = Reader::open(&path).err().unwrap();
        assert!(err.contains("version 1"), "{err}");
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    #[cfg(unix)]
    fn a_write_error_stops_the_recording() {
        const CHILD: &str = "CV1TOUCH_TEST_WRITE_ERROR";
        if std::env::var_os(CHILD).is_none() {
            // Pipes keep RLIMIT_FSIZE confined to the recording, even if the parent test
            // output is redirected to a regular log file that already exceeds the limit.
            let output = std::process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "recorder::tests::a_write_error_stops_the_recording",
                    "--nocapture",
                ])
                .env(CHILD, "1")
                .output()
                .unwrap();
            assert!(
                output.status.success(),
                "write-error subprocess failed: {}\n{}\n{}",
                output.status,
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
            return;
        }
        // Exercise a real partial write, confined to this test's subprocess.
        unsafe {
            libc::signal(libc::SIGXFSZ, libc::SIG_IGN);
            let limit = libc::rlimit {
                rlim_cur: 4096,
                rlim_max: 4096,
            };
            assert_eq!(libc::setrlimit(libc::RLIMIT_FSIZE, &limit), 0);
        }
        let path = tmp("write-error");
        let r = Recorder::new(1 << 16);
        r.start(&path, 1).unwrap();
        r.write(RAW_REPORT, 1, &[&[0u8; 8192]]);
        for _ in 0..100 {
            if r.stats().write_errors > 0 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        let stopped = !r.active();
        let stats = r.stop();
        std::fs::remove_file(&path).unwrap();
        assert_eq!(stats.write_errors, 1);
        assert!(stopped, "failed writes must disable recording immediately");
    }

    #[test]
    fn oversized_record_lengths_are_rejected_before_allocation() {
        let path = tmp("oversized-length");
        let mut data = [0u8; FILE_HEADER_LEN + RECORD_HEADER_LEN];
        data[..8].copy_from_slice(MAGIC);
        data[8..12].copy_from_slice(&VERSION.to_le_bytes());
        data[12..16].copy_from_slice(&(FILE_HEADER_LEN as u32).to_le_bytes());
        let oversized = (64u32 << 20) + 1;
        data[FILE_HEADER_LEN + 4..FILE_HEADER_LEN + 8].copy_from_slice(&oversized.to_le_bytes());
        std::fs::write(&path, data).unwrap();
        // Sparse payload ensures the old implementation would successfully allocate/read it.
        OpenOptions::new()
            .write(true)
            .open(&path)
            .unwrap()
            .set_len(data.len() as u64 + oversized as u64)
            .unwrap();
        let mut reader = Reader::open(&path).unwrap();
        let got = reader.next();
        std::fs::remove_file(&path).unwrap();
        assert!(
            got.is_none(),
            "reader accepted a payload larger than the recorder's maximum ring"
        );
        assert!(reader.error().unwrap().contains("exceeds"));
        assert!(reader.next().is_none(), "failed readers must stay stopped");
    }

    #[test]
    fn raw_frame_dimensions_cannot_allocate_unbounded_images() {
        let mut payload = vec![0u8; size_of::<FrameInfo>()];
        payload.extend_from_slice(&4097u16.to_le_bytes());
        payload.extend_from_slice(&4096u16.to_le_bytes());
        payload.extend_from_slice(&[0; 4]);
        assert!(
            decode_raw_frame(&payload).is_none(),
            "oversized image accepted"
        );
    }

    #[test]
    fn rejects_non_recording() {
        let path = tmp("junk");
        std::fs::write(&path, b"not a recording at all, definitely not").unwrap();
        assert!(Reader::open(&path).is_err());
        std::fs::remove_file(&path).unwrap();
    }

    /// Review fix 5: a sleeping controller must not keep the sensors' intrinsics (or the awake
    /// controller's LED model) out of the recording.
    #[test]
    fn session_info_is_written_piecewise() {
        let mut s = SessionInfo::default();
        assert_eq!(s.plan(false, &[]), (false, vec![]));
        assert_eq!(s.plan(true, &[(1, true), (2, false)]), (true, vec![1]));
        assert_eq!(s.plan(true, &[(1, true), (2, false)]), (false, vec![]));
        assert!(!s.complete(2));
        assert_eq!(s.plan(true, &[(1, true), (2, true)]), (false, vec![2]));
        assert!(s.complete(2));
        s.reset();
        assert_eq!(s.plan(true, &[(1, true), (2, true)]), (true, vec![1, 2]));
    }
}
