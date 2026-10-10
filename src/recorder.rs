// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Session recordings, format v2 (little-endian).
//! File header (32 B): magic "CV1TREC\0", u32 version, u32 header_size = 32, u64 start_host_ns, u64 0.
//! Record: u16 type, u16 flags = 0, u32 payload_len, u64 host_ns, payload.
//! CONTROLLER_STATE = u32 hand, u32 0, cv1t_controller_state; RAW_REPORT = u32 source, bytes;
//! FRAME_OBS = ohmd_cv1touch_frame_info, n x ohmd_cv1touch_blob; SENSOR_INFO = ohmd_cv1touch_sensor;
//! LED_MODEL = u32 device_id, u32 n, n x ohmd_cv1touch_led.
//! Tracking threads only copy into a ring under a mutex (no I/O); a writer thread does the file I/O.

use std::fs::{File, OpenOptions};
use std::io::{BufReader, Read, Write};
use std::mem::size_of;
use std::path::Path;
use std::ptr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;

pub use crate::ffi::ohmd_cv1touch_blob as Blob;
pub use crate::ffi::ohmd_cv1touch_frame_info as FrameInfo;
pub use crate::ffi::ohmd_cv1touch_led as LedEntry;
pub use crate::ffi::ohmd_cv1touch_sensor as SensorInfo;
use crate::shm::ControllerState;

pub const MAGIC: &[u8; 8] = b"CV1TREC\0";
pub const VERSION: u32 = 2;
pub const CONTROLLER_STATE: u16 = 1;
pub const RAW_REPORT: u16 = 2;
pub const FRAME_OBS: u16 = 4;
pub const SENSOR_INFO: u16 = 5;
pub const LED_MODEL: u16 = 6;
/// A sensor frame's pixels (see `encode_frame`), recorded with `record start PATH --frames`.
pub const RAW_FRAME: u16 = 7;
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

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Stats {
    pub records: u64,
    pub dropped: u64,
    pub write_errors: u64,
    pub bytes: u64,
}

struct Ring {
    buf: Vec<u8>,
    head: usize,
    used: usize,
    stop: bool,
    stats: Stats,
}

impl Ring {
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
}

struct Shared {
    ring: Mutex<Ring>,
    cond: Condvar,
    active: AtomicBool,
}

pub struct Recorder {
    shared: Arc<Shared>,
    writer: Mutex<Option<JoinHandle<()>>>,
}

fn writer_main(shared: Arc<Shared>, mut file: File) {
    let mut ring = shared.ring.lock().unwrap();
    loop {
        while ring.used == 0 && !ring.stop {
            ring = shared.cond.wait(ring).unwrap();
        }
        if ring.used == 0 && ring.stop {
            break;
        }
        let chunk = ring.take(CHUNK);
        drop(ring);
        let result = file.write_all(&chunk);
        ring = shared.ring.lock().unwrap();
        match result {
            Ok(()) => ring.stats.bytes += chunk.len() as u64,
            Err(_) => {
                ring.stats.write_errors += 1;
                // A partial write can end inside a record: appending later chunks would make
                // the file unparseable. Leave a recoverable truncated tail and stop.
                shared.active.store(false, Ordering::Release);
                ring.used = 0;
                break;
            }
        }
    }
    drop(ring);
    if file.sync_all().is_err() {
        shared.ring.lock().unwrap().stats.write_errors += 1;
    }
}

impl Recorder {
    pub fn new(ring_bytes: usize) -> Self {
        Self {
            shared: Arc::new(Shared {
                ring: Mutex::new(Ring {
                    buf: vec![0; ring_bytes],
                    head: 0,
                    used: 0,
                    stop: false,
                    stats: Stats::default(),
                }),
                cond: Condvar::new(),
                active: AtomicBool::new(false),
            }),
            writer: Mutex::new(None),
        }
    }

    /// Creates `path` exclusively (never overwrites) and starts the writer thread.
    pub fn start(&self, path: &Path, start_host_ns: u64) -> Result<(), String> {
        let mut writer = self.writer.lock().unwrap();
        if writer.is_some() {
            return Err("already recording".into());
        }
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|e| format!("open {}: {e}", path.display()))?;
        let mut header = [0u8; FILE_HEADER_LEN];
        header[..8].copy_from_slice(MAGIC);
        header[8..12].copy_from_slice(&VERSION.to_le_bytes());
        header[12..16].copy_from_slice(&(FILE_HEADER_LEN as u32).to_le_bytes());
        header[16..24].copy_from_slice(&start_host_ns.to_le_bytes());
        file.write_all(&header)
            .map_err(|e| format!("write {}: {e}", path.display()))?;
        {
            let mut ring = self.shared.ring.lock().unwrap();
            ring.head = 0;
            ring.used = 0;
            ring.stop = false;
            ring.stats = Stats {
                bytes: FILE_HEADER_LEN as u64,
                ..Stats::default()
            };
        }
        self.shared.active.store(true, Ordering::Release);
        let shared = Arc::clone(&self.shared);
        let handle = std::thread::Builder::new()
            .name("cv1t-recorder".into())
            .spawn(move || writer_main(shared, file))
            .map_err(|e| {
                self.shared.active.store(false, Ordering::Release);
                format!("cannot start recorder thread: {e}")
            })?;
        *writer = Some(handle);
        Ok(())
    }

    /// Includes a failed recording awaiting `stop()` to reap its writer and report errors.
    pub fn has_session(&self) -> bool {
        self.writer.lock().unwrap().is_some()
    }

    pub fn active(&self) -> bool {
        self.shared.active.load(Ordering::Acquire)
    }

    /// Thread-safe and I/O-free: copies one record (payload parts concatenated) into the ring, or
    /// counts it as dropped if it does not fit. A no-op while inactive.
    pub fn write(&self, rtype: u16, host_ns: u64, parts: &[&[u8]]) {
        if !self.active() {
            return;
        }
        let payload_len: usize = parts.iter().map(|p| p.len()).sum();
        let mut header = [0u8; RECORD_HEADER_LEN];
        header[..2].copy_from_slice(&rtype.to_le_bytes());
        header[4..8].copy_from_slice(&(payload_len as u32).to_le_bytes());
        header[8..16].copy_from_slice(&host_ns.to_le_bytes());
        let mut ring = self.shared.ring.lock().unwrap();
        if !self.active() {
            return;
        }
        if ring.buf.len() - ring.used < RECORD_HEADER_LEN + payload_len {
            ring.stats.dropped += 1;
            return;
        }
        ring.put(&header);
        for p in parts {
            ring.put(p);
        }
        ring.stats.records += 1;
        drop(ring);
        self.shared.cond.notify_one();
    }

    pub fn stats(&self) -> Stats {
        self.shared.ring.lock().unwrap().stats
    }

    /// Drains the ring, closes the file and returns the session's stats.
    pub fn stop(&self) -> Stats {
        let handle = self.writer.lock().unwrap().take();
        if let Some(handle) = handle {
            self.shared.active.store(false, Ordering::Release);
            self.shared.ring.lock().unwrap().stop = true;
            self.shared.cond.notify_one();
            let _ = handle.join();
        }
        self.stats()
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        self.stop();
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
        let err = r
            .start(Path::new("/nonexistent-cv1t-dir/x.rec"), 1)
            .unwrap_err();
        assert!(err.contains("No such file"), "{err}");
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
