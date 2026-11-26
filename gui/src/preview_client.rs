// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Live sensor previews from cv1touchd's control endpoint (the `preview` command), polled
//! on a worker thread so that reading and decoding never hold up the window.

use serde_json::Value;
use std::{
    collections::{HashMap, VecDeque},
    io::{self, BufRead, BufReader, Write},
    sync::{
        mpsc::{self, RecvTimeoutError, Sender},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};

/// The largest preview image: one sensor pooled by 2.
pub const MAX_PIXELS: usize = 640 * 480;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Every sensor at a quarter of its resolution, about 15 frames a second.
    All,
    /// One sensor at half its resolution, about 30 frames a second.
    One(String),
    /// None, while the window is minimized: the daemon stops decoding every sensor's images.
    Paused,
}

impl Mode {
    fn line(&self) -> String {
        match self {
            Self::All => "preview 4\n".to_owned(),
            Self::One(serial) => format!("preview {serial} 2\n"),
            Self::Paused => "preview stop\n".to_owned(),
        }
    }

    fn interval(&self) -> Duration {
        Duration::from_millis(match self {
            Self::All => 66,
            Self::One(_) => 33,
            Self::Paused => 1000,
        })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum PreviewError {
    DaemonStopped,
    /// The daemon predates the `preview` command.
    UnsupportedDaemon,
    Other(String),
}

/// One detected blob in raw image pixels, and the device and LED index it was matched to.
#[derive(Clone, Debug, PartialEq)]
pub struct Blob {
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
    pub led: Option<(u8, u8)>,
}

/// How one device (OpenHMD id: 0 headset, 1 right, 2 left) matched the frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Device {
    pub device_id: i64,
    pub matched_blobs: u64,
    pub visible_leds: u64,
    pub pose_valid: bool,
    pub match_flags: u64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SensorFrame {
    pub serial: String,
    pub frame_ns: u64,
    pub age_ms: u64,
    pub raw_width: u32,
    pub raw_height: u32,
    /// 0 when the sensor sent no image, only blobs.
    pub width: u32,
    pub height: u32,
    pub max_pixel: u8,
    pub bright_permille: u32,
    /// `width * height` GRAY8 pixels.
    pub pixels: Vec<u8>,
    pub blobs: Vec<Blob>,
    pub devices: Vec<Device>,
    /// Distinct frames a second over the last second, once known.
    pub fps: Option<f32>,
}

/// Every active sensor, in the daemon's sensor order.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PreviewSnapshot {
    pub sensors: Vec<SensorFrame>,
}

/// Standard padded base64; None for anything else.
pub fn base64_decode(text: &str) -> Option<Vec<u8>> {
    let text = text.as_bytes();
    if !text.len().is_multiple_of(4) {
        return None;
    }
    let value = |c: u8| match c {
        b'A'..=b'Z' => Some(c - b'A'),
        b'a'..=b'z' => Some(c - b'a' + 26),
        b'0'..=b'9' => Some(c - b'0' + 52),
        b'+' => Some(62),
        b'/' => Some(63),
        _ => None,
    };
    let chunks = text.len() / 4;
    let mut out = Vec::with_capacity(chunks * 3);
    for (i, chunk) in text.as_chunks::<4>().0.iter().enumerate() {
        let pad = chunk.iter().rev().take_while(|&&c| c == b'=').count();
        if pad > 2 || (pad > 0 && i + 1 < chunks) {
            return None;
        }
        let mut n = 0u32;
        for &c in &chunk[..4 - pad] {
            n = (n << 6) | u32::from(value(c)?);
        }
        n <<= 6 * pad as u32;
        out.extend_from_slice(&n.to_be_bytes()[1..4 - pad]);
    }
    Some(out)
}

/// Inflates zlib data, giving up as soon as it would exceed `limit` bytes.
fn inflate(zlib: &[u8], limit: usize) -> Option<Vec<u8>> {
    miniz_oxide::inflate::decompress_to_vec_zlib_with_limit(zlib, limit).ok()
}

/// The GRAY8 image of exactly `len` pixels in `pixels_b64`.
pub fn decode_pixels(b64: &str, len: usize) -> Option<Vec<u8>> {
    if len > MAX_PIXELS {
        return None;
    }
    inflate(&base64_decode(b64)?, len).filter(|pixels| pixels.len() == len)
}

fn parse_sensor(value: &Value) -> Option<SensorFrame> {
    let n = |key: &str| value[key].as_u64();
    let mut frame = SensorFrame {
        serial: value["serial"].as_str()?.to_owned(),
        frame_ns: n("frame_ns")?,
        age_ms: n("age_ms").unwrap_or(0),
        raw_width: n("raw_width")? as u32,
        raw_height: n("raw_height")? as u32,
        width: n("width")? as u32,
        height: n("height")? as u32,
        max_pixel: n("max_pixel").unwrap_or(0).min(255) as u8,
        bright_permille: n("bright_permille").unwrap_or(0) as u32,
        ..Default::default()
    };
    if frame.width > 0 {
        let len = frame.width as usize * frame.height as usize;
        match (value["encoding"].as_str(), value["pixels_b64"].as_str()) {
            (Some("zlib-gray8-b64"), Some(b64)) => frame.pixels = decode_pixels(b64, len)?,
            _ => return None,
        }
    }
    for blob in value["blobs"].as_array().into_iter().flatten() {
        let f = |i: usize| blob[i].as_f64().map(|v| v as f32);
        let led = blob[4].as_i64()?;
        frame.blobs.push(Blob {
            x: f(0)?,
            y: f(1)?,
            w: f(2)?,
            h: f(3)?,
            led: (led >= 0).then_some(((led >> 8) as u8, (led & 0xff) as u8)),
        });
    }
    for device in value["devices"].as_array().into_iter().flatten() {
        frame.devices.push(Device {
            device_id: device["device_id"].as_i64()?,
            matched_blobs: device["matched_blobs"].as_u64().unwrap_or(0),
            visible_leds: device["visible_leds"].as_u64().unwrap_or(0),
            pose_valid: device["pose_valid"].as_bool().unwrap_or(false),
            match_flags: device["match_flags"].as_u64().unwrap_or(0),
        });
    }
    Some(frame)
}

/// One reply line of the `preview` command. A malformed sensor entry is left out.
pub fn parse_reply(line: &str) -> Result<PreviewSnapshot, PreviewError> {
    let line = line.trim_end();
    if let Some(json) = line.strip_prefix("ok ") {
        let invalid = || PreviewError::Other("cv1touchd sent an invalid preview".into());
        let value: Value = serde_json::from_str(json).map_err(|_| invalid())?;
        let sensors = value["sensors"].as_array().ok_or_else(invalid)?;
        return Ok(PreviewSnapshot {
            sensors: sensors.iter().filter_map(parse_sensor).collect(),
        });
    }
    match line.strip_prefix("err ") {
        Some(e) if e.starts_with("unknown command 'preview'") => {
            Err(PreviewError::UnsupportedDaemon)
        }
        Some(e) => Err(PreviewError::Other(e.to_owned())),
        None => Err(PreviewError::Other(
            "cv1touchd sent an invalid reply".into(),
        )),
    }
}

/// One sensor's frames a second, from the distinct `frame_ns` of the last second: a frame
/// that two polls return counts once.
#[derive(Default)]
pub struct FpsTracker(VecDeque<u64>);

impl FpsTracker {
    pub fn push(&mut self, frame_ns: u64) -> Option<f32> {
        if self.0.back().is_none_or(|&last| frame_ns > last) {
            self.0.push_back(frame_ns);
        }
        let newest = *self.0.back()?;
        while self.0.front().is_some_and(|&t| newest - t > 1_000_000_000) {
            self.0.pop_front();
        }
        let span = newest - self.0.front()?;
        (self.0.len() >= 3 && span > 0).then(|| (self.0.len() - 1) as f32 * 1e9 / span as f32)
    }
}

/// What the window shows: nothing yet, the latest previews, or why there are none.
#[derive(Clone, Debug, PartialEq)]
pub enum Feed {
    Connecting,
    Live(PreviewSnapshot),
    Failed(PreviewError),
}

pub trait Stream: io::Read + io::Write + Send {}
impl<T: io::Read + io::Write + Send> Stream for T {}
pub type Connect = Box<dyn Fn() -> io::Result<Box<dyn Stream>> + Send>;

/// The preview worker. Dropping it sends `preview stop`, so that the daemon frees its preview
/// buffers and stops decoding every sensor's images at once instead of when the lease ends.
pub struct Preview {
    feed: Arc<Mutex<(u64, Feed)>>,
    modes: Option<Sender<Mode>>,
    done: mpsc::Receiver<()>,
}

impl Preview {
    pub fn start(ctx: eframe::egui::Context) -> Self {
        Self::start_with(Box::new(connect), move || ctx.request_repaint())
    }

    pub fn start_with(connect: Connect, repaint: impl Fn() + Send + 'static) -> Self {
        let feed = Arc::new(Mutex::new((0, Feed::Connecting)));
        let (modes, requests) = mpsc::channel();
        let (finished, done) = mpsc::channel();
        let shared = Arc::clone(&feed);
        thread::spawn(move || {
            let mut worker = Worker {
                control: Control::new(connect),
                order: Vec::new(),
                order_at: None,
                fps: HashMap::new(),
                empty_since: None,
            };
            let mut mode = Mode::All;
            let mut stopped = false;
            loop {
                let started = Instant::now();
                let feed = if mode == Mode::Paused {
                    if !stopped {
                        worker.stop();
                    }
                    stopped = true;
                    None
                } else {
                    stopped = false;
                    worker.poll(&mode)
                };
                let wait = match &feed {
                    Some(Feed::Failed(PreviewError::DaemonStopped)) => Duration::from_millis(500),
                    Some(Feed::Failed(_)) => Duration::from_secs(1),
                    _ => mode.interval().saturating_sub(started.elapsed()),
                };
                if let Some(feed) = feed {
                    let mut slot = shared.lock().unwrap();
                    *slot = (slot.0 + 1, feed);
                    drop(slot);
                    repaint();
                }
                match requests.recv_timeout(wait) {
                    Ok(next) => mode = requests.try_iter().last().unwrap_or(next),
                    Err(RecvTimeoutError::Timeout) => {}
                    Err(RecvTimeoutError::Disconnected) => break,
                }
            }
            if !stopped {
                worker.stop();
            }
            let _ = finished.send(());
        });
        Self {
            feed,
            modes: Some(modes),
            done,
        }
    }

    pub fn set_mode(&self, mode: Mode) {
        if let Some(modes) = &self.modes {
            let _ = modes.send(mode);
        }
    }

    /// The latest feed and its update count, if it is newer than update `seen`.
    pub fn latest(&self, seen: u64) -> Option<(u64, Feed)> {
        let feed = self.feed.lock().unwrap();
        (feed.0 != seen).then(|| feed.clone())
    }
}

impl Drop for Preview {
    fn drop(&mut self) {
        self.modes = None;
        // Bounded, so that a hung daemon cannot keep the window from closing.
        let _ = self.done.recv_timeout(Duration::from_secs(1));
    }
}

/// One kept connection to the daemon's control endpoint.
pub struct Control {
    connect: Connect,
    conn: Option<BufReader<Box<dyn Stream>>>,
}

struct Worker {
    control: Control,
    /// The daemon's sensor order (from `status`), so that tiles keep their place and number.
    order: Vec<String>,
    order_at: Option<Instant>,
    fps: HashMap<String, FpsTracker>,
    empty_since: Option<Instant>,
}

/// How often the sensor order is reread.
const ORDER_INTERVAL: Duration = Duration::from_secs(2);
/// How long empty replies may only mean that the daemon is starting the previews: the first
/// reply after a pause has no frames yet.
const STARTING: Duration = Duration::from_secs(1);

impl Control {
    pub fn new(connect: Connect) -> Self {
        Self {
            connect,
            conn: None,
        }
    }

    /// One request and its reply line, reconnecting once when a kept connection has gone
    /// (the daemon restarted).
    pub fn request(&mut self, line: &str) -> Result<String, PreviewError> {
        let reused = self.conn.is_some();
        for retry in [reused, false] {
            if self.conn.is_none() {
                let stream = (self.connect)().map_err(|e| match e.kind() {
                    io::ErrorKind::NotFound | io::ErrorKind::ConnectionRefused => {
                        PreviewError::DaemonStopped
                    }
                    _ => PreviewError::Other(format!("Could not reach cv1touchd: {e}")),
                })?;
                self.conn = Some(BufReader::new(stream));
            }
            let conn = self.conn.as_mut().unwrap();
            let mut reply = String::new();
            let result = conn
                .get_mut()
                .write_all(line.as_bytes())
                .and_then(|()| conn.read_line(&mut reply));
            match result {
                Ok(_) if reply.ends_with('\n') => return Ok(reply),
                _ if retry => self.conn = None,
                Ok(_) => {
                    self.conn = None;
                    return Err(PreviewError::DaemonStopped);
                }
                Err(e) => {
                    self.conn = None;
                    return Err(PreviewError::Other(format!(
                        "Could not read cv1touchd: {e}"
                    )));
                }
            }
        }
        unreachable!("the second attempt never retries")
    }
}

impl Worker {
    fn request(&mut self, line: &str) -> Result<String, PreviewError> {
        self.control.request(line)
    }

    /// The next feed, or None while empty replies may only mean that previews are starting.
    fn poll(&mut self, mode: &Mode) -> Option<Feed> {
        if self
            .order_at
            .is_none_or(|at| at.elapsed() >= ORDER_INTERVAL)
        {
            if let Ok(reply) = self.request("status\n") {
                self.order_at = Some(Instant::now());
                if let Some(Ok(value)) = reply
                    .trim_end()
                    .strip_prefix("ok ")
                    .map(serde_json::from_str::<Value>)
                {
                    self.order = value["sensors"]
                        .as_array()
                        .into_iter()
                        .flatten()
                        .filter_map(|s| Some(s["serial"].as_str()?.to_ascii_uppercase()))
                        .collect();
                }
            }
        }
        let mut snapshot = match self.request(&mode.line()).and_then(|r| parse_reply(&r)) {
            Ok(snapshot) => snapshot,
            Err(error) => {
                self.order_at = None;
                self.empty_since = None;
                return Some(Feed::Failed(error));
            }
        };
        if snapshot.sensors.is_empty() {
            if self.empty_since.get_or_insert_with(Instant::now).elapsed() < STARTING {
                return None;
            }
        } else {
            self.empty_since = None;
        }
        let order = &self.order;
        snapshot.sensors.sort_by_cached_key(|sensor| {
            let serial = sensor.serial.to_ascii_uppercase();
            (
                order
                    .iter()
                    .position(|s| *s == serial)
                    .unwrap_or(usize::MAX),
                serial,
            )
        });
        for sensor in &mut snapshot.sensors {
            let fps = self
                .fps
                .entry(sensor.serial.to_ascii_uppercase())
                .or_default();
            sensor.fps = fps.push(sensor.frame_ns);
        }
        Some(Feed::Live(snapshot))
    }

    /// Ends the preview lease. Best effort: an unreachable daemon ends it itself in 3 s.
    fn stop(&mut self) {
        let _ = self.request(&Mode::Paused.line());
    }
}

#[cfg(unix)]
pub fn connect() -> io::Result<Box<dyn Stream>> {
    let dir = std::env::var_os("XDG_RUNTIME_DIR")
        .ok_or_else(|| io::Error::other("XDG_RUNTIME_DIR is not set"))?;
    let stream = std::os::unix::net::UnixStream::connect(
        std::path::Path::new(&dir).join("cv1touch/control.sock"),
    )?;
    let timeout = Some(Duration::from_secs(2));
    stream.set_read_timeout(timeout)?;
    stream.set_write_timeout(timeout)?;
    Ok(Box::new(stream))
}

#[cfg(windows)]
pub fn connect() -> io::Result<Box<dyn Stream>> {
    pipe::Pipe::connect().map(|pipe| Box::new(pipe) as Box<dyn Stream>)
}

/// One request on a fresh connection, with its reply line.
pub fn request_once(line: &str) -> Result<String, PreviewError> {
    Control::new(Box::new(connect)).request(line)
}

#[cfg(windows)]
mod pipe {
    use std::{io, os::windows::ffi::OsStrExt, path::PathBuf, ptr};
    use windows_sys::Win32::{
        Foundation::{
            CloseHandle, ERROR_BROKEN_PIPE, GENERIC_READ, GENERIC_WRITE, HANDLE,
            INVALID_HANDLE_VALUE,
        },
        Storage::FileSystem::{
            CreateFileW, ReadFile, WriteFile, OPEN_EXISTING, SECURITY_IDENTIFICATION,
            SECURITY_SQOS_PRESENT,
        },
    };

    /// A synchronous connection to the daemon's control pipe.
    pub struct Pipe(HANDLE);

    // SAFETY: the handle is owned by this value alone, which one thread uses at a time.
    unsafe impl Send for Pipe {}

    impl Pipe {
        /// The daemon's pipe: `\\.\pipe\cv1touch-` and its endpoint path with `\` as `/`.
        pub fn connect() -> io::Result<Self> {
            let local = std::env::var_os("LOCALAPPDATA")
                .map(PathBuf::from)
                .filter(|p| p.is_absolute())
                .unwrap_or_else(std::env::temp_dir);
            let endpoint = local.join("cv1touch").join("control.sock");
            let name: Vec<u16> = r"\\.\pipe\cv1touch-"
                .encode_utf16()
                .chain(endpoint.as_os_str().encode_wide().map(|c| {
                    if c == u16::from(b'\\') {
                        u16::from(b'/')
                    } else {
                        c
                    }
                }))
                .chain(Some(0))
                .collect();
            // SAFETY: a NUL-terminated name; the returned value owns the handle.
            let handle = unsafe {
                CreateFileW(
                    name.as_ptr(),
                    GENERIC_READ | GENERIC_WRITE,
                    0,
                    ptr::null(),
                    OPEN_EXISTING,
                    // identification only: the daemon may check who we are, not act as us
                    SECURITY_SQOS_PRESENT | SECURITY_IDENTIFICATION,
                    ptr::null_mut(),
                )
            };
            if handle == INVALID_HANDLE_VALUE {
                return Err(io::Error::last_os_error());
            }
            Ok(Self(handle))
        }
    }

    impl Drop for Pipe {
        fn drop(&mut self) {
            // SAFETY: the handle is open and owned.
            unsafe { CloseHandle(self.0) };
        }
    }

    impl io::Read for Pipe {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            let mut n = 0;
            let len = buf.len().min(u32::MAX as usize) as u32;
            // SAFETY: `buf` has `len` writable bytes.
            if unsafe { ReadFile(self.0, buf.as_mut_ptr(), len, &mut n, ptr::null_mut()) } == 0 {
                let error = io::Error::last_os_error();
                if error.raw_os_error() == Some(ERROR_BROKEN_PIPE as i32) {
                    return Ok(0);
                }
                return Err(error);
            }
            Ok(n as usize)
        }
    }

    impl io::Write for Pipe {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            let mut n = 0;
            let len = buf.len().min(u32::MAX as usize) as u32;
            // SAFETY: `buf` has `len` readable bytes.
            if unsafe { WriteFile(self.0, buf.as_ptr(), len, &mut n, ptr::null_mut()) } == 0 {
                return Err(io::Error::last_os_error());
            }
            Ok(n as usize)
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use serde_json::json;

    fn base64_encode(data: &[u8]) -> String {
        const ABC: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        let mut out = String::new();
        for chunk in data.chunks(3) {
            let n = chunk
                .iter()
                .enumerate()
                .fold(0u32, |n, (i, &b)| n | (u32::from(b) << (16 - 8 * i)));
            for i in 0..4 {
                out.push(if i <= chunk.len() {
                    ABC[((n >> (18 - 6 * i)) & 63) as usize] as char
                } else {
                    '='
                });
            }
        }
        out
    }

    pub fn encode_pixels(pixels: &[u8]) -> String {
        base64_encode(&miniz_oxide::deflate::compress_to_vec_zlib(pixels, 1))
    }

    /// A preview reply line with these sensor entries.
    pub fn reply(sensors: &[Value]) -> String {
        format!(
            "ok {}\n",
            json!({"active": true, "lease_ms": 3000, "sensors": sensors})
        )
    }

    #[test]
    fn base64_round_trips_and_rejects_malformed_text() {
        for len in 0..10 {
            let data: Vec<u8> = (0..len).map(|i| (i * 37 + 200) as u8).collect();
            assert_eq!(base64_decode(&base64_encode(&data)), Some(data));
        }
        assert_eq!(base64_decode("aGk="), Some(b"hi".to_vec()));
        assert_eq!(base64_decode("+/8="), Some(vec![0xfb, 0xff]));
        for bad in ["aGk", "aG=k", "a===", "aGk=aGk=", "aGk*", "aGk=\n"] {
            assert_eq!(base64_decode(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn pixels_decode_only_at_their_exact_size() {
        let pixels: Vec<u8> = (0..320 * 240).map(|i| (i % 7) as u8).collect();
        let b64 = encode_pixels(&pixels);
        assert_eq!(decode_pixels(&b64, pixels.len()), Some(pixels.clone()));
        assert_eq!(decode_pixels(&b64, pixels.len() + 1), None);
        assert_eq!(decode_pixels(&b64, pixels.len() - 1), None);
        // Corrupt base64, and valid base64 of a corrupt stream.
        assert_eq!(decode_pixels(&b64[1..], pixels.len()), None);
        assert_eq!(decode_pixels(&b64[4..], pixels.len()), None);
        let big = vec![0; MAX_PIXELS + 1];
        assert_eq!(decode_pixels(&encode_pixels(&big), big.len()), None);
    }

    #[test]
    fn an_oversized_payload_is_never_inflated_past_the_limit() {
        // 16 MB of zeros compresses to a few kB.
        let bomb = miniz_oxide::deflate::compress_to_vec_zlib(&vec![0; 16 << 20], 1);
        assert!(bomb.len() < 100_000, "{}", bomb.len());
        assert_eq!(inflate(&bomb, MAX_PIXELS), None);
        assert_eq!(inflate(&bomb, 16 << 20).map(|p| p.len()), Some(16 << 20));
        let sensor = json!({"serial": "S1", "frame_ns": 1, "raw_width": 1280,
            "raw_height": 960, "width": 640, "height": 480, "encoding": "zlib-gray8-b64",
            "pixels_b64": base64_encode(&bomb), "blobs": [], "devices": []});
        assert_eq!(parse_reply(&reply(&[sensor])).unwrap().sensors, []);
    }

    #[test]
    fn replies_parse_into_frames_blobs_and_devices() {
        let pixels: Vec<u8> = (0..80 * 60).map(|i| (i % 251) as u8).collect();
        let line = reply(&[
            json!({"serial": "SENSOR1", "frame_ns": 123, "age_ms": 14,
                "raw_width": 320, "raw_height": 240, "width": 80, "height": 60, "max_pixel": 250,
                "bright_permille": 24, "encoding": "zlib-gray8-b64",
                "pixels_b64": encode_pixels(&pixels),
                "blobs": [[10.5, 20.0, 3, 4, 0x205], [1.0, 2.0, 1, 1, -1], [5.0, 6.0, 2, 2, 0x10a]],
                "devices": [{"device_id": 2, "matched_blobs": 9, "visible_leds": 11,
                    "pose_valid": true, "match_flags": 3}]}),
            // Blobs only: no GRAY8 image.
            json!({"serial": "SENSOR2", "frame_ns": 99, "age_ms": 3,
                "raw_width": 0, "raw_height": 0, "width": 0, "height": 0, "max_pixel": 0,
                "bright_permille": 0, "encoding": "zlib-gray8-b64", "pixels_b64": "",
                "blobs": [[100.0, 50.0, 2, 2, -1]], "devices": []}),
            // An image that does not decode.
            json!({"serial": "SENSOR3", "frame_ns": 5, "raw_width": 320, "raw_height": 240,
                "width": 80, "height": 60, "encoding": "zlib-gray8-b64", "pixels_b64": "AAAA"}),
        ]);
        let snapshot = parse_reply(&line).unwrap();
        let [a, b] = &snapshot.sensors[..] else {
            panic!("{snapshot:?}")
        };
        assert_eq!(
            (a.serial.as_str(), a.frame_ns, a.age_ms),
            ("SENSOR1", 123, 14)
        );
        assert_eq!(
            (a.raw_width, a.raw_height, a.width, a.height),
            (320, 240, 80, 60)
        );
        assert_eq!((a.max_pixel, a.bright_permille), (250, 24));
        assert_eq!(a.pixels, pixels);
        assert_eq!(
            a.blobs.iter().map(|b| b.led).collect::<Vec<_>>(),
            [Some((2, 5)), None, Some((1, 10))]
        );
        assert_eq!((a.blobs[0].x, a.blobs[0].w, a.blobs[0].h), (10.5, 3.0, 4.0));
        assert_eq!(
            a.devices,
            [Device {
                device_id: 2,
                matched_blobs: 9,
                visible_leds: 11,
                pose_valid: true,
                match_flags: 3
            }]
        );
        assert_eq!((b.width, b.pixels.len(), b.blobs.len()), (0, 0, 1));
        assert_eq!(
            parse_reply("ok {\"active\":true,\"lease_ms\":3000,\"sensors\":[]}\n"),
            Ok(PreviewSnapshot::default())
        );
        assert_eq!(
            parse_reply("err unknown command 'preview'\n"),
            Err(PreviewError::UnsupportedDaemon)
        );
        assert_eq!(
            parse_reply("err preview: scale must be 2 or 4"),
            Err(PreviewError::Other("preview: scale must be 2 or 4".into()))
        );
        assert!(matches!(parse_reply("ok {"), Err(PreviewError::Other(_))));
        assert!(matches!(parse_reply("ok"), Err(PreviewError::Other(_))));
        assert!(matches!(parse_reply("hello"), Err(PreviewError::Other(_))));
    }

    #[test]
    fn fps_counts_each_frame_once() {
        let mut fps = FpsTracker::default();
        let mut last = None;
        // 15 frames a second, each returned by two polls.
        for i in 0..40u64 {
            last = fps.push(1_000_000_000 + i / 2 * 66_666_667);
        }
        let last = last.unwrap();
        assert!((last - 15.0).abs() < 0.2, "{last}");
        // A sensor that sends nothing new: its last frame alone is no rate.
        let mut fps = FpsTracker::default();
        for _ in 0..5 {
            assert_eq!(fps.push(5), None);
        }
    }

    #[cfg(unix)]
    #[test]
    fn the_worker_polls_switches_mode_and_stops_when_dropped() {
        use std::os::unix::net::{UnixListener, UnixStream};
        let path = std::env::temp_dir().join(format!("preview-{}.sock", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let listener = UnixListener::bind(&path).unwrap();
        let (lines_tx, lines) = mpsc::channel();
        thread::spawn(move || {
            for stream in listener.incoming() {
                let Ok(stream) = stream else { break };
                let lines_tx = lines_tx.clone();
                thread::spawn(move || {
                    let mut writer = stream.try_clone().unwrap();
                    for line in BufReader::new(stream).lines() {
                        let Ok(line) = line else { break };
                        let sensor = |serial| {
                            json!({"serial": serial, "frame_ns": 1, "raw_width": 0,
                                "raw_height": 0, "width": 0, "height": 0})
                        };
                        let answer = match line.as_str() {
                            "status" => {
                                "ok {\"sensors\":[{\"serial\":\"S2\"},{\"serial\":\"S1\"}]}\n"
                                    .to_owned()
                            }
                            "preview stop" => "ok\n".to_owned(),
                            _ => reply(&[sensor("S1"), sensor("s2")]),
                        };
                        let _ = lines_tx.send(line);
                        if writer.write_all(answer.as_bytes()).is_err() {
                            break;
                        }
                    }
                });
            }
        });
        let socket = path.clone();
        let preview = Preview::start_with(
            Box::new(move || Ok(Box::new(UnixStream::connect(&socket)?) as Box<dyn Stream>)),
            || {},
        );
        let next = |want: &str| loop {
            if lines.recv_timeout(Duration::from_secs(5)).unwrap() == want {
                break;
            }
        };
        next("status");
        next("preview 4");
        next("preview 4");
        let (_, feed) = preview.latest(0).unwrap();
        let Feed::Live(snapshot) = feed else {
            panic!("{feed:?}")
        };
        // The daemon's sensor order, whatever the serials' case.
        assert_eq!(
            snapshot
                .sensors
                .iter()
                .map(|s| s.serial.as_str())
                .collect::<Vec<_>>(),
            ["s2", "S1"]
        );
        preview.set_mode(Mode::One("S1".into()));
        next("preview S1 2");
        // Paused: one stop, then nothing until the window is back.
        preview.set_mode(Mode::Paused);
        next("preview stop");
        assert!(lines.recv_timeout(Duration::from_millis(300)).is_err());
        preview.set_mode(Mode::All);
        next("preview 4");
        drop(preview);
        next("preview stop");
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn a_missing_daemon_is_reported_as_stopped() {
        let preview = Preview::start_with(Box::new(|| Err(io::ErrorKind::NotFound.into())), || {});
        let started = Instant::now();
        let feed = loop {
            if let Some((_, feed)) = preview.latest(0) {
                break feed;
            }
            assert!(started.elapsed() < Duration::from_secs(5));
            thread::sleep(Duration::from_millis(5));
        };
        assert_eq!(feed, Feed::Failed(PreviewError::DaemonStopped));
    }
}
