// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Safe wrapper over the vendored OpenHMD engine. Not Send: create and use it on one thread
//! (OpenHMD does no locking with its auto-update thread disabled).

use std::ffi::{c_char, c_int};
#[cfg(feature = "engine")]
use std::ffi::{c_void, CStr};
#[cfg(feature = "engine")]
use std::ptr;
#[cfg(feature = "engine")]
use std::sync::Arc;

use crate::ffi;
#[cfg(feature = "engine")]
use crate::shm::{HAND_LEFT, HAND_RIGHT};
use crate::status::sensor_identity;

pub use ffi::ohmd_cv1touch_blob as Blob;
pub use ffi::ohmd_cv1touch_controller as Controller;

/// A radio thread's sample (`RadioDecoder::decode`) over the engine's last view of its
/// controller: the sample's times, IMU and inputs, with the engine's estimates as of its last update.
pub fn with_radio_sample(last: &Controller, fast: &Controller) -> Controller {
    Controller {
        device_time_ns: fast.device_time_ns,
        received_host_ns: fast.received_host_ns,
        filter_ns: 0,
        imu_accel: fast.imu_accel,
        imu_gyro: fast.imu_gyro,
        buttons: fast.buttons,
        trigger: fast.trigger,
        grip: fast.grip,
        stick: fast.stick,
        cap: fast.cap,
        ..*last
    }
}
pub use ffi::ohmd_cv1touch_frame_info as FrameInfo;
pub use ffi::ohmd_cv1touch_inspection_identity as InspectionIdentity;
pub use ffi::ohmd_cv1touch_led as LedEntry;
pub use ffi::ohmd_cv1touch_sensor as Sensor;

/// Frames delivered by a camera, independent of analysis-buffer completion.
#[derive(Clone, Debug)]
pub struct SensorVideo {
    pub serial: String,
    pub frames_received: u64,
}

impl From<ffi::ohmd_cv1touch_sensor_video> for SensorVideo {
    fn from(video: ffi::ohmd_cv1touch_sensor_video) -> Self {
        Self {
            serial: cstr_field(&video.serial),
            frames_received: video.frames_received,
        }
    }
}

/// Negotiated camera input; JPEG frames become GRAY8 only after decoding.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "lowercase")]
pub enum VideoFormat {
    Raw,
    Jpeg,
}

/// Independent quality stages. Raw video has no JPEG-decoded counter.
#[derive(Clone, Debug)]
pub struct SensorVideoQuality {
    pub serial: String,
    pub video_format: Option<VideoFormat>,
    pub frames_decoded: Option<u64>,
    pub frames_dropped: u64,
}

impl From<ffi::ohmd_cv1touch_sensor_video_quality> for SensorVideoQuality {
    fn from(quality: ffi::ohmd_cv1touch_sensor_video_quality) -> Self {
        let video_format = match quality.video_format {
            ffi::OHMD_CV1TOUCH_VIDEO_RAW => Some(VideoFormat::Raw),
            ffi::OHMD_CV1TOUCH_VIDEO_JPEG => Some(VideoFormat::Jpeg),
            _ => None,
        };
        Self {
            serial: cstr_field(&quality.serial),
            frames_decoded: (video_format == Some(VideoFormat::Jpeg))
                .then_some(quality.frames_decoded),
            frames_dropped: quality.frames_dropped,
            video_format,
        }
    }
}

/// Last USB video start attempt, retained through cleanup and until detach is reported.
#[derive(Clone, Debug, serde::Serialize)]
pub struct SensorStreamStart {
    scope: &'static str,
    pub serial: String,
    pub error_kind: &'static str,
    pub error_code: i32,
    pub os_error: i32,
    /// Setting used by that attempt if it succeeded; none if it failed.
    pub alt_setting: Option<i32>,
}

impl From<ffi::ohmd_cv1touch_sensor_stream> for SensorStreamStart {
    fn from(s: ffi::ohmd_cv1touch_sensor_stream) -> Self {
        Self {
            scope: "last_start_attempt",
            serial: cstr_field(&s.serial),
            error_kind: match s.error_kind {
                ffi::OHMD_CV1TOUCH_STREAM_ERROR_NONE => "none",
                ffi::OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH => "likely_usb_bandwidth",
                _ => "other",
            },
            error_code: s.error_code,
            os_error: s.os_error,
            alt_setting: (s.alt_setting > 0).then_some(s.alt_setting),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SensorPose {
    pub serial: String,
    /// camera -> world, x y z w (OpenCV camera axes).
    pub orient: [f32; 4],
    pub pos: [f32; 3],
}

#[derive(Debug, Clone, Default)]
pub struct EngineOptions {
    pub radio_wait_ms: i32,
    pub hmd_leds_off: bool,
    /// Rift sensor serials to leave closed.
    pub ignore_sensors: Vec<String>,
    /// Never place sensors at runtime; sensors without a fixed pose only observe.
    pub fixed_poses_only: bool,
    /// Calibrated sensor poses, copied into native context ownership before discovery.
    pub fixed_poses: Vec<SensorPose>,
}

// used by the engine, and by tests on any build
#[cfg_attr(not(feature = "engine"), allow(dead_code))]
fn copy_cstr(dst: &mut [c_char], s: &str) {
    let room = dst.len() - 1;
    for (d, b) in dst.iter_mut().zip(s.bytes().take(room)) {
        *d = b as c_char;
    }
}

pub fn build_config(opts: &EngineOptions) -> ffi::ohmd_cv1touch_config {
    ffi::ohmd_cv1touch_config {
        radio_wait_ms: opts.radio_wait_ms,
        hmd_tracking: false,
        hmd_leds_off: opts.hmd_leds_off,
        fixed_poses_only: opts.fixed_poses_only,
        ..Default::default()
    }
}

// used by the engine, and by tests on any build
#[cfg_attr(not(feature = "engine"), allow(dead_code))]
struct SensorConfig {
    ignored: Vec<[c_char; 32]>,
    poses: Vec<ffi::ohmd_cv1touch_sensor_pose>,
}

// used by the engine, and by tests on any build
/// A sensor serial as the engine takes it: 1 to 31 ASCII letters or digits, NUL-terminated.
fn serial(s: &str) -> Result<[c_char; 32], String> {
    if s.is_empty() || s.len() > 31 || !s.bytes().all(|b| b.is_ascii_alphanumeric()) {
        return Err("sensor serials must contain 1 to 31 ASCII letters or digits".into());
    }
    let mut out = [0; 32];
    copy_cstr(&mut out, s);
    Ok(out)
}

/// The engine matches serials without regard to ASCII letter case and refuses a list that names
/// one sensor twice, for every open attempt. Say which entries clash instead.
#[cfg_attr(not(feature = "engine"), allow(dead_code))]
fn refuse_repeated_sensors<'a>(
    serials: impl IntoIterator<Item = &'a str>,
    what: &str,
) -> Result<(), String> {
    let mut seen = std::collections::HashMap::new();
    for serial in serials {
        if let Some(first) = seen.insert(sensor_identity(serial), serial) {
            return Err(format!(
                "{what} {serial} names the same sensor as {first} (serial letter case does not matter)"
            ));
        }
    }
    Ok(())
}

#[cfg_attr(not(feature = "engine"), allow(dead_code))]
fn build_sensor_config(opts: &EngineOptions) -> Result<SensorConfig, String> {
    if opts.ignore_sensors.len() > c_int::MAX as usize
        || opts.fixed_poses.len() > c_int::MAX as usize
    {
        return Err("sensor configuration exceeds native index range".into());
    }
    let ignored = opts
        .ignore_sensors
        .iter()
        .map(|s| serial(s))
        .collect::<Result<Vec<_>, _>>()?;
    let poses = opts
        .fixed_poses
        .iter()
        .map(|p| {
            Ok(ffi::ohmd_cv1touch_sensor_pose {
                serial: serial(&p.serial)?,
                orient: p.orient,
                pos: p.pos,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    refuse_repeated_sensors(
        opts.ignore_sensors.iter().map(String::as_str),
        "ignored sensor",
    )?;
    refuse_repeated_sensors(
        opts.fixed_poses.iter().map(|p| p.serial.as_str()),
        "calibrated sensor",
    )?;
    Ok(SensorConfig { ignored, poses })
}

/// Independent readers of the radio; device and receive clocks must stay with their reader.
#[derive(Clone, Copy, Debug, Default)]
pub enum RadioReader {
    #[default]
    Engine = 0,
    Radio = 1,
}

/// Native identity evidence from relative motion, never a waiver of the fusion attitude gate.
pub const MATCH_INSPECTION_IDENTITY: u32 = ffi::OHMD_CV1TOUCH_MATCH_INSPECTION_IDENTITY;
/// The inspected hand turned enough, on its own, for that identity to be judged.
pub const MATCH_INSPECTION_MOTION: u32 = ffi::OHMD_CV1TOUCH_MATCH_INSPECTION_MOTION;

/// Engine callbacks. `frame` runs on sensor analysis threads with the sensor lock held and
/// `report` on the engine thread: both must only copy data.
pub trait EngineSink: Send + Sync {
    fn frame(&self, info: &FrameInfo, blobs: &[Blob]);
    /// A native label search is about to start; retain its exposure until its deadline.
    fn search_started(&self, _info: &FrameInfo, _remaining_ns: u64) {}
    /// Identity lifetimes are control evidence; retain invalidations even when dropping frames.
    fn inspection_identity(&self, _event: &InspectionIdentity, _host_ns: u64) {}
    fn report(&self, source: i32, host_ns: u64, data: &[u8]);
    /// Runs in decoded message order on the named reader. Copy and return without waiting.
    #[allow(clippy::too_many_arguments)]
    fn activity(
        &self,
        _reader: RadioReader,
        _device: i32,
        _device_ns: u64,
        _host_ns: u64,
        _asleep: bool,
        _gyro_squared: f32,
    ) {
    }
    /// The frame's GRAY8 pixels, just before `frame` for the same frame (valid only for the call).
    fn image(
        &self,
        _info: &FrameInfo,
        _width: usize,
        _height: usize,
        _stride: usize,
        _pixels: &[u8],
    ) {
    }
}

#[cfg(feature = "engine")]
pub struct Engine {
    ctx: *mut ffi::ohmd_context,
    hands: [*mut ffi::ohmd_device; 2],
    last_io_errors: u32,
    sink: *mut Arc<dyn EngineSink>,
}

#[cfg(feature = "engine")]
mod callbacks;
#[cfg(feature = "engine")]
use callbacks::{activity_cb, frame_cb, identity_cb, image_cb, report_cb, search_cb};

/// Run Rust entered from C. A panic must not reach Rust's non-unwinding ABI diagnostic,
/// which writes directly to stderr. The daemon installs its nonblocking panic hook first.
#[cfg(feature = "engine")]
pub fn ffi_callback(run: impl FnOnce()) {
    if let Err(payload) = std::panic::catch_unwind(std::panic::AssertUnwindSafe(run)) {
        // A user panic payload's destructor could panic again. We are terminating anyway.
        std::mem::forget(payload);
        // SAFETY: the native fatal path drains once within its bound and never returns.
        unsafe { ffi::ohmd_cv1touch_abort() };
    }
}

/// A NUL-terminated C char array field as a String.
pub fn cstr_field(s: &[c_char]) -> String {
    let bytes: Vec<u8> = s
        .iter()
        .take_while(|&&c| c != 0)
        .map(|&c| c as u8)
        .collect();
    String::from_utf8_lossy(&bytes).into_owned()
}

pub fn sensor_serial(s: &Sensor) -> String {
    cstr_field(&s.serial)
}

#[cfg(feature = "engine")]
impl Engine {
    /// Probes and opens both Touch controllers with OpenHMD's auto-update thread disabled.
    pub fn open(opts: &EngineOptions, sink: Arc<dyn EngineSink>) -> Result<Engine, String> {
        let sensors = build_sensor_config(opts)?;
        // SAFETY: FFI into OpenHMD; every pointer passed is valid for the call, and `sink` outlives
        // the context (freed in Drop after ohmd_ctx_destroy joins the engine's threads).
        unsafe {
            let ctx = ffi::ohmd_ctx_create();
            if ctx.is_null() {
                return Err("ohmd_ctx_create failed".into());
            }
            let sink = Box::into_raw(Box::new(sink));
            let mut engine = Engine {
                ctx,
                hands: [ptr::null_mut(); 2],
                last_io_errors: 0,
                sink,
            };
            let cfg = build_config(opts);
            ffi::ohmd_cv1touch_configure(ctx, &cfg);
            if ffi::ohmd_cv1touch_configure_sensors(
                ctx,
                sensors.ignored.as_ptr(),
                sensors.ignored.len(),
                sensors.poses.as_ptr(),
                sensors.poses.len(),
            ) != 0
            {
                return Err("native sensor configuration rejected (invalid/duplicate entry or allocation failure)".into());
            }
            ffi::ohmd_cv1touch_set_frame_callback(ctx, Some(frame_cb), sink as *mut c_void);
            ffi::ohmd_cv1touch_set_search_callback(ctx, Some(search_cb), sink as *mut c_void);
            ffi::ohmd_cv1touch_set_identity_callback(ctx, Some(identity_cb), sink as *mut c_void);
            ffi::ohmd_cv1touch_set_image_callback(ctx, Some(image_cb), sink as *mut c_void);
            ffi::ohmd_cv1touch_set_report_callback(ctx, Some(report_cb), sink as *mut c_void);
            ffi::ohmd_cv1touch_set_activity_callback(ctx, Some(activity_cb), sink as *mut c_void);

            let n = ffi::ohmd_ctx_probe(ctx);
            let settings = ffi::ohmd_device_settings_create(ctx);
            let manual: c_int = 0;
            ffi::ohmd_device_settings_seti(settings, ffi::OHMD_IDS_AUTOMATIC_UPDATE, &manual);
            for i in 0..n {
                let mut flags: c_int = 0;
                ffi::ohmd_list_geti(ctx, i, ffi::OHMD_DEVICE_FLAGS, &mut flags);
                // a C enum: unsigned under GCC, int under MSVC
                #[allow(clippy::unnecessary_cast)]
                let flags = flags as ffi::ohmd_device_flags;
                if flags & ffi::OHMD_DEVICE_FLAGS_NULL_DEVICE != 0 {
                    continue;
                }
                let hand = if flags & ffi::OHMD_DEVICE_FLAGS_LEFT_CONTROLLER != 0 {
                    HAND_LEFT
                } else if flags & ffi::OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER != 0 {
                    HAND_RIGHT
                } else {
                    continue;
                };
                if engine.hands[hand].is_null() {
                    engine.hands[hand] = ffi::ohmd_list_open_device_s(ctx, i, settings);
                }
            }
            ffi::ohmd_device_settings_destroy(settings);
            if engine.hands.iter().any(|h| h.is_null()) {
                let why = CStr::from_ptr(ffi::ohmd_ctx_get_error(ctx))
                    .to_string_lossy()
                    .into_owned();
                return Err(format!(
                    "Rift CV1 controllers not available (is the headset on USB?) {why}"
                ));
            }
            engine.last_io_errors = ffi::ohmd_cv1touch_get_io_errors(engine.hands[HAND_RIGHT]);
            Ok(engine)
        }
    }

    /// One engine iteration: blocks up to radio_wait_ms for radio data, then drains pending
    /// reports. Err(()) when headset I/O failed during this iteration (unplugged).
    // The engine exposes only an I/O failure flag, with no error details to retain.
    #[allow(clippy::result_unit_err)]
    pub fn update(&mut self) -> Result<(), ()> {
        // SAFETY: valid context and device.
        unsafe {
            ffi::ohmd_ctx_update(self.ctx);
            let io = ffi::ohmd_cv1touch_get_io_errors(self.hands[HAND_RIGHT]);
            let failed = io != self.last_io_errors;
            self.last_io_errors = io;
            if failed {
                Err(())
            } else {
                Ok(())
            }
        }
    }

    /// A decoder for the radio thread (see `RadioDecoder`).
    ///
    /// # Safety
    /// The decoder points into the engine's headset: it must be dropped before the engine is.
    pub unsafe fn radio_decoder(&self) -> Option<RadioDecoder> {
        // SAFETY: valid device; the caller drops the decoder before the engine.
        let dec = unsafe { ffi::ohmd_cv1touch_radio_decoder_new(self.hands[HAND_RIGHT]) };
        (!dec.is_null()).then_some(RadioDecoder(dec))
    }

    /// Stable factory-calibration identity for this physical controller. Available after
    /// calibration is loaded; query on the engine thread and keep out of public logs.
    pub fn controller_identity(&self, hand: usize) -> Option<String> {
        let device = *self.hands.get(hand)?;
        let mut buf = [0 as c_char; 33];
        // SAFETY: buffer has the advertised size and this Engine owns the device.
        (unsafe { ffi::ohmd_cv1touch_get_controller_identity(device, buf.as_mut_ptr(), 33) } == 0)
            .then(|| cstr_field(&buf))
    }

    pub fn controller(&self, hand: usize) -> Option<Controller> {
        let mut c = Controller::default();
        // SAFETY: valid device and out-pointer.
        (unsafe { ffi::ohmd_cv1touch_get_controller(self.hands[hand], &mut c) } == 0).then_some(c)
    }

    pub fn sensors(&self) -> Vec<Sensor> {
        // SAFETY: valid device and out-pointers.
        unsafe {
            let n = ffi::ohmd_cv1touch_get_sensor_count(self.hands[HAND_RIGHT]);
            (0..n)
                .filter_map(|i| {
                    let mut s = Sensor::default();
                    (ffi::ohmd_cv1touch_get_sensor(self.hands[HAND_RIGHT], i, &mut s) == 0)
                        .then_some(s)
                })
                .collect()
        }
    }

    /// Read current acquisition counters by serial; discovery can change their set/order.
    pub fn sensor_video(&self) -> Vec<SensorVideo> {
        (0..c_int::MAX)
            .map_while(|index| {
                let mut video = ffi::ohmd_cv1touch_sensor_video::default();
                // SAFETY: this engine owns the device; output matches the native record.
                (unsafe {
                    ffi::ohmd_cv1touch_get_sensor_video(self.hands[HAND_RIGHT], index, &mut video)
                } == 0)
                    .then(|| video.into())
            })
            .collect()
    }

    /// Enumerate quality afresh and join by serial: native indices can change.
    pub fn sensor_video_quality(&self) -> Vec<SensorVideoQuality> {
        (0..c_int::MAX)
            .map_while(|index| {
                let mut quality = ffi::ohmd_cv1touch_sensor_video_quality::default();
                // SAFETY: this engine owns the device and the output matches the native ABI.
                (unsafe {
                    ffi::ohmd_cv1touch_get_sensor_video_quality(
                        self.hands[HAND_RIGHT],
                        index,
                        &mut quality,
                    )
                } == 0)
                    .then(|| quality.into())
            })
            .collect()
    }

    /// Read the complete last-start table each time: discovery changes its set/order.
    /// These records can outlive unplugging; they do not establish current USB presence.
    pub fn sensor_streams(&self) -> Vec<SensorStreamStart> {
        (0..c_int::MAX)
            .map_while(|index| {
                let mut stream = ffi::ohmd_cv1touch_sensor_stream::default();
                // SAFETY: this engine owns the device; the output has the advertised layout.
                (unsafe {
                    ffi::ohmd_cv1touch_get_sensor_stream(self.hands[HAND_RIGHT], index, &mut stream)
                } == 0)
                    .then(|| stream.into())
            })
            .collect()
    }

    /// Drop a detached serial's diagnostic without freeing its native sensor object.
    /// Repeated detach notifications are harmless. Call on the engine thread.
    pub fn forget_sensor_stream(&self, serial: &str) -> Result<(), String> {
        let serial_c = self::serial(serial)?;
        // SAFETY: valid device and a complete, NUL-terminated serial.
        if unsafe {
            ffi::ohmd_cv1touch_forget_sensor_stream(self.hands[HAND_RIGHT], serial_c.as_ptr())
        } == 0
        {
            Ok(())
        } else {
            Err(format!(
                "the engine could not clear video diagnostics for sensor {serial}"
            ))
        }
    }

    /// Gives the open sensor `serial` the camera -> world pose it tracks with from its next frame
    /// on (its calibrated pose at start, until then). Err when no such sensor is open.
    pub fn set_sensor_pose(
        &self,
        serial: &str,
        orient_xyzw: [f64; 4],
        pos: [f64; 3],
    ) -> Result<(), String> {
        let pose = ffi::ohmd_cv1touch_sensor_pose {
            serial: self::serial(serial)?,
            orient: orient_xyzw.map(|v| v as f32),
            pos: pos.map(|v| v as f32),
        };
        // SAFETY: valid device and a complete, NUL-terminated pose structure.
        if unsafe { ffi::ohmd_cv1touch_set_sensor_pose(self.hands[HAND_RIGHT], &pose) } == 0 {
            Ok(())
        } else {
            Err(format!("the engine has no open sensor {serial}"))
        }
    }

    /// Requests labels with a native deadline, independently of the daemon's inspection timer.
    pub fn inspect_sensor(
        &self,
        serial: &str,
        hands: [bool; 2],
        timeout_ns: u64,
    ) -> Result<(), String> {
        let serial_c = self::serial(serial)?;
        // SAFETY: owned device and a complete, NUL-terminated serial.
        if unsafe {
            ffi::ohmd_cv1touch_inspect_sensor(
                self.hands[HAND_RIGHT],
                serial_c.as_ptr(),
                device_mask(hands),
                timeout_ns,
            )
        } == 0
        {
            Ok(())
        } else {
            Err(format!("the engine cannot inspect sensor {serial}"))
        }
    }

    /// Whether a queued restoration has reached the sensor and its preceding search stopped.
    /// A sensor no longer open has no search to wait for.
    pub fn sensor_pose_applied(&self, serial: &str) -> bool {
        let Ok(serial_c) = self::serial(serial) else {
            return false;
        };
        // SAFETY: owned device and a complete, NUL-terminated serial.
        unsafe {
            ffi::ohmd_cv1touch_sensor_pose_applied(self.hands[HAND_RIGHT], serial_c.as_ptr()) != 0
        }
    }

    /// The controller's factory LED model (empty until its calibration has been read over the radio).
    pub fn led_model(&self, hand: usize) -> Vec<LedEntry> {
        let mut buf = vec![LedEntry::default(); 64];
        // SAFETY: valid device; buf holds 64 entries.
        let n = unsafe {
            ffi::ohmd_cv1touch_get_led_model(self.hands[hand], buf.as_mut_ptr(), buf.len() as c_int)
        };
        buf.truncate(n.max(0) as usize);
        buf
    }

    pub fn haptic(&mut self, hand: usize, duration_s: f32, frequency_hz: f32, amplitude: f32) {
        // SAFETY: valid device.
        unsafe {
            if amplitude <= 0.0 {
                ffi::ohmd_device_set_haptics_off(self.hands[hand]);
            } else {
                ffi::ohmd_device_set_haptics_on(
                    self.hands[hand],
                    duration_s,
                    frequency_hz,
                    amplitude,
                );
            }
        }
    }
}

#[cfg(feature = "engine")]
impl Drop for Engine {
    fn drop(&mut self) {
        // SAFETY: we own the context; destroying it stops all engine threads before the sink is freed.
        unsafe {
            for h in self.hands {
                if !h.is_null() {
                    ffi::ohmd_close_device(h);
                }
            }
            ffi::ohmd_ctx_destroy(self.ctx);
            drop(Box::from_raw(self.sink));
        }
    }
}

#[cfg(feature = "engine")]
/// Decodes radio reports the radio thread reads through a hidraw handle of its own, on its own copy
/// of each controller's state (`ohmd_cv1touch_radio_decode`): it never waits on the engine. Its
/// samples' device times run on its own clocks. It points into the engine's headset, so it must be
/// dropped before the engine that made it closes.
pub struct RadioDecoder(*mut ffi::ohmd_cv1touch_radio_decoder);

// SAFETY: the decoder's state is its own; of the engine's it reads only each controller's
// calibration, after a flag the engine sets once with release ordering.
#[cfg(feature = "engine")]
unsafe impl Send for RadioDecoder {}

#[cfg(feature = "engine")]
impl RadioDecoder {
    /// The calibrated controllers' samples (IMU, inputs, times) in one radio report received at
    /// `received_ns`: `out[..n]`.
    pub fn decode(&mut self, report: &[u8], received_ns: u64) -> ([Controller; 2], usize) {
        let mut out = [Controller::default(); 2];
        // SAFETY: valid decoder; `out` has room for the two messages a report carries.
        let n = unsafe {
            ffi::ohmd_cv1touch_radio_decode(
                self.0,
                report.as_ptr(),
                report.len() as c_int,
                received_ns,
                out.as_mut_ptr(),
            )
        };
        (out, n.clamp(0, 2) as usize)
    }

    /// Leaves the controllers of `hands` (by hand index) out of the engine's brute-force pose
    /// search until the next call or the decoder's drop: the caller's tracker holds them, or they
    /// are asleep. The engine still matches them against its own priors.
    pub fn set_search_skip(&mut self, hands: [bool; 2]) {
        // SAFETY: valid decoder.
        unsafe { ffi::ohmd_cv1touch_radio_decoder_set_search_skip(self.0, device_mask(hands)) }
    }
}

/// The engine's device mask for hands (bit 1 << OpenHMD device id; the headset, 0, never), as
/// its search skip and inspection requests take it.
#[cfg_attr(not(feature = "engine"), allow(dead_code))]
fn device_mask(hands: [bool; 2]) -> u32 {
    let mut mask = 0u32;
    if hands[crate::shm::HAND_LEFT] {
        mask |= 1 << crate::calib::DEVICE_LEFT;
    }
    if hands[crate::shm::HAND_RIGHT] {
        mask |= 1 << crate::calib::DEVICE_RIGHT;
    }
    mask
}

#[cfg(feature = "engine")]
impl Drop for RadioDecoder {
    fn drop(&mut self) {
        // SAFETY: made by ohmd_cv1touch_radio_decoder_new, freed once.
        unsafe { ffi::ohmd_cv1touch_radio_decoder_free(self.0) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quality_mapping_preserves_unknown_raw_jpeg_and_u64_values() {
        for (format, expected) in [
            (ffi::OHMD_CV1TOUCH_VIDEO_RAW, Some(VideoFormat::Raw)),
            (ffi::OHMD_CV1TOUCH_VIDEO_JPEG, Some(VideoFormat::Jpeg)),
            (ffi::OHMD_CV1TOUCH_VIDEO_UNKNOWN, None),
            (99, None),
        ] {
            for count in [0, u64::MAX] {
                let mut native = ffi::ohmd_cv1touch_sensor_video_quality {
                    video_format: format,
                    frames_decoded: count,
                    frames_dropped: count,
                    ..Default::default()
                };
                native.serial[0] = b'X' as _;
                let q = SensorVideoQuality::from(native);
                assert_eq!(q.serial, "X");
                assert_eq!(q.video_format, expected);
                assert_eq!(
                    q.frames_decoded,
                    (expected == Some(VideoFormat::Jpeg)).then_some(count)
                );
                assert_eq!(q.frames_dropped, count);
            }
        }
        assert_eq!(std::mem::size_of::<ffi::ohmd_cv1touch_sensor_video>(), 40);
        assert_eq!(
            std::mem::size_of::<ffi::ohmd_cv1touch_sensor_video_quality>(),
            56
        );
    }

    #[test]
    fn the_device_mask_names_controllers_by_engine_device_id() {
        assert_eq!(device_mask([false, false]), 0);
        assert_eq!(device_mask([true, false]), 1 << 2, "left is device 2");
        assert_eq!(device_mask([false, true]), 1 << 1, "right is device 1");
        assert_eq!(device_mask([true, true]), 0b110, "never the headset");
    }

    #[test]
    fn config_carries_fixed_poses_and_ignores() {
        let opts = EngineOptions {
            radio_wait_ms: 2,
            hmd_leds_off: true,
            ignore_sensors: vec!["WMTD000000004".into()],
            fixed_poses_only: true,
            fixed_poses: vec![SensorPose {
                serial: "WMTD000000001".into(),
                orient: [0.0, 0.0, 0.0, 1.0],
                pos: [1.0, 2.0, 3.0],
            }],
        };
        let cfg = build_config(&opts);
        assert!(cfg.fixed_poses_only && cfg.hmd_leds_off && cfg.radio_wait_ms == 2);
        let sensors = build_sensor_config(&opts).unwrap();
        assert_eq!(cstr_field(&sensors.ignored[0]), "WMTD000000004");
        assert_eq!(cstr_field(&sensors.poses[0].serial), "WMTD000000001");
        assert_eq!(sensors.poses[0].pos, [1.0, 2.0, 3.0]);
        assert_eq!(sensors.poses.len(), 1);
    }

    #[test]
    fn all_sensor_configuration_entries_are_marshaled_without_truncation() {
        let mut opts = EngineOptions::default();
        for i in 0..65 {
            opts.ignore_sensors.push(format!("TEST{i:06}"));
            opts.fixed_poses.push(SensorPose {
                serial: format!("POSE{i:06}"),
                orient: [0., 0., 0., 1.],
                pos: [i as f32, 0., 0.],
            });
        }
        let sensors = build_sensor_config(&opts).unwrap();
        assert_eq!(sensors.ignored.len(), 65);
        assert_eq!(sensors.poses.len(), 65);
        assert_eq!(cstr_field(&sensors.ignored[64]), "TEST000064");
        assert_eq!(sensors.poses[64].pos[0], 64.);
        for bad in ["", "bad/name", "01234567890123456789012345678901é"] {
            opts.ignore_sensors[0] = bad.into();
            assert!(build_sensor_config(&opts).is_err());
        }
    }

    /// The engine refuses a sensor named twice, case included: say which entries clash, not
    /// "rejected" on every open attempt.
    #[test]
    fn sensors_named_twice_in_any_letter_case_are_refused_by_name() {
        let pose = |serial: &str| SensorPose {
            serial: serial.into(),
            orient: [0., 0., 0., 1.],
            pos: [0., 0., 0.],
        };
        let mut opts = EngineOptions {
            ignore_sensors: vec!["camera1".into(), "CAMERA2".into()],
            fixed_poses: vec![pose("camera3"), pose("CAMERA4")],
            ..Default::default()
        };
        assert!(build_sensor_config(&opts).is_ok());
        opts.ignore_sensors.push("CAMERA1".into());
        let err = build_sensor_config(&opts).err().unwrap();
        assert!(
            err.contains("ignored sensor CAMERA1 names the same sensor as camera1"),
            "{err}"
        );
        opts.ignore_sensors.pop();
        opts.fixed_poses.push(pose("Camera3"));
        let err = build_sensor_config(&opts).err().unwrap();
        assert!(
            err.contains("calibrated sensor Camera3 names the same sensor as camera3"),
            "{err}"
        );
    }

    #[test]
    fn a_radio_sample_rides_on_the_engines_last_controller() {
        let mut last = Controller {
            calibrated: true,
            device_time_ns: 1_000,
            received_host_ns: 5_000,
            last_pose_fix_device_ns: 900,
            filter_ns: 70,
            buttons: 1,
            trigger: 0.1,
            ..Default::default()
        };
        last.view_pos = [1.0, 2.0, 3.0];
        last.gyro_bias = [0.01, 0.0, 0.0];
        last.imu_gyro = [0.5, 0.0, 0.0];
        let fast = Controller {
            calibrated: true,
            device_time_ns: 3_500,
            received_host_ns: 7_600,
            buttons: 3,
            trigger: 0.8,
            grip: 0.4,
            stick: [0.2, -0.1],
            cap: [1.0, 0.0, 0.5, 0.0, 1.2],
            device_id: 2,
            imu_accel: [0.0, 9.8, 0.0],
            imu_gyro: [2.0, 0.0, 0.0],
            ..Default::default()
        };
        let c = with_radio_sample(&last, &fast);
        // the new sample: its times, IMU and inputs
        assert_eq!(
            (c.device_time_ns, c.received_host_ns, c.filter_ns),
            (3_500, 7_600, 0)
        );
        assert_eq!((c.imu_gyro, c.imu_accel), (fast.imu_gyro, fast.imu_accel));
        assert_eq!(
            (c.buttons, c.trigger, c.grip, c.stick, c.cap),
            (3, 0.8, 0.4, [0.2, -0.1], fast.cap)
        );
        // the engine's estimates as of its last update
        assert_eq!(
            (c.view_pos, c.gyro_bias, c.last_pose_fix_device_ns),
            ([1.0, 2.0, 3.0], [0.01, 0.0, 0.0], 900)
        );
        assert!(c.calibrated);
    }
}
