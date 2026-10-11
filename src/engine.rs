// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Safe wrapper over the vendored OpenHMD engine. Not Send: create and use it on one thread
//! (OpenHMD does no locking with its auto-update thread disabled).

use std::ffi::{c_char, c_int, c_void, CStr};
use std::path::{Path, PathBuf};
use std::ptr;
use std::sync::Arc;

use crate::ffi;
use crate::shm::{HAND_LEFT, HAND_RIGHT};

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
pub use ffi::ohmd_cv1touch_led as LedEntry;
pub use ffi::ohmd_cv1touch_sensor as Sensor;

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
    /// Rift sensor serials to leave closed (at most 8).
    pub ignore_sensors: Vec<String>,
    /// Never place sensors at runtime; sensors without a fixed pose only observe.
    pub fixed_poses_only: bool,
    /// Calibrated sensor poses (at most 8).
    pub fixed_poses: Vec<SensorPose>,
}

fn copy_cstr(dst: &mut [c_char], s: &str) {
    let room = dst.len() - 1;
    for (d, b) in dst.iter_mut().zip(s.bytes().take(room)) {
        *d = b as c_char;
    }
}

pub fn build_config(opts: &EngineOptions) -> ffi::ohmd_cv1touch_config {
    let mut cfg = ffi::ohmd_cv1touch_config {
        radio_wait_ms: opts.radio_wait_ms,
        hmd_tracking: false,
        hmd_leds_off: opts.hmd_leds_off,
        fixed_poses_only: opts.fixed_poses_only,
        ..Default::default()
    };
    for (slot, serial) in cfg
        .ignore_sensor_serials
        .iter_mut()
        .zip(&opts.ignore_sensors)
    {
        copy_cstr(slot, serial);
    }
    for (slot, p) in cfg.fixed_poses.iter_mut().zip(&opts.fixed_poses) {
        copy_cstr(&mut slot.serial, &p.serial);
        slot.orient = p.orient;
        slot.pos = p.pos;
    }
    cfg
}

/// Engine callbacks. `frame` runs on sensor analysis threads with the sensor lock held and
/// `report` on the engine thread: both must only copy data.
pub trait EngineSink: Send + Sync {
    fn frame(&self, info: &FrameInfo, blobs: &[Blob]);
    fn report(&self, source: i32, host_ns: u64, data: &[u8]);
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

pub struct Engine {
    ctx: *mut ffi::ohmd_context,
    hands: [*mut ffi::ohmd_device; 2],
    last_io_errors: u32,
    sink: *mut Arc<dyn EngineSink>,
}

unsafe extern "C" fn frame_cb(
    user: *mut c_void,
    info: *const FrameInfo,
    n: c_int,
    blobs: *const Blob,
) {
    if info.is_null() {
        return;
    }
    let sink = &*(user as *const Arc<dyn EngineSink>);
    let blobs = if n > 0 && !blobs.is_null() {
        std::slice::from_raw_parts(blobs, n as usize)
    } else {
        &[]
    };
    sink.frame(&*info, blobs);
}

unsafe extern "C" fn image_cb(
    user: *mut c_void,
    info: *const FrameInfo,
    width: c_int,
    height: c_int,
    stride: c_int,
    pixels: *const u8,
) {
    if info.is_null() || pixels.is_null() || width <= 0 || height <= 0 || stride < width {
        return;
    }
    let sink = &*(user as *const Arc<dyn EngineSink>);
    let px = std::slice::from_raw_parts(pixels, stride as usize * height as usize);
    sink.image(&*info, width as usize, height as usize, stride as usize, px);
}

unsafe extern "C" fn report_cb(
    user: *mut c_void,
    source: c_int,
    ts: u64,
    data: *const u8,
    len: c_int,
) {
    let sink = &*(user as *const Arc<dyn EngineSink>);
    let data = if len > 0 && !data.is_null() {
        std::slice::from_raw_parts(data, len as usize)
    } else {
        &[]
    };
    sink.report(source, ts, data);
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

impl Engine {
    /// Probes and opens both Touch controllers with OpenHMD's auto-update thread disabled.
    pub fn open(opts: &EngineOptions, sink: Arc<dyn EngineSink>) -> Result<Engine, String> {
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
            ffi::ohmd_cv1touch_set_frame_callback(ctx, Some(frame_cb), sink as *mut c_void);
            ffi::ohmd_cv1touch_set_image_callback(ctx, Some(image_cb), sink as *mut c_void);
            ffi::ohmd_cv1touch_set_report_callback(ctx, Some(report_cb), sink as *mut c_void);

            let n = ffi::ohmd_ctx_probe(ctx);
            let settings = ffi::ohmd_device_settings_create(ctx);
            let manual: c_int = 0;
            ffi::ohmd_device_settings_seti(settings, ffi::OHMD_IDS_AUTOMATIC_UPDATE, &manual);
            for i in 0..n {
                let mut flags: c_int = 0;
                ffi::ohmd_list_geti(ctx, i, ffi::OHMD_DEVICE_FLAGS, &mut flags);
                let flags = flags as u32;
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

/// Decodes radio reports the radio thread reads through a hidraw handle of its own, on its own copy
/// of each controller's state (`ohmd_cv1touch_radio_decode`): it never waits on the engine. Its
/// samples' device times run on its own clocks. It points into the engine's headset, so it must be
/// dropped before the engine that made it closes.
pub struct RadioDecoder(*mut ffi::ohmd_cv1touch_radio_decoder);

// SAFETY: the decoder's state is its own; of the engine's it reads only each controller's
// calibration, after a flag the engine sets once with release ordering.
unsafe impl Send for RadioDecoder {}

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
}

impl Drop for RadioDecoder {
    fn drop(&mut self) {
        // SAFETY: made by ohmd_cv1touch_radio_decoder_new, freed once.
        unsafe { ffi::ohmd_cv1touch_radio_decoder_free(self.0) }
    }
}

/// The hidraw node of the headset's radio interface (Rift CV1 2833:0031, interface 1) among the
/// nodes under `sys_class_hidraw` (normally /sys/class/hidraw).
pub fn radio_hidraw(sys_class_hidraw: &Path) -> Option<PathBuf> {
    let mut names: Vec<String> = std::fs::read_dir(sys_class_hidraw)
        .ok()?
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    names
        .into_iter()
        .find(|n| {
            let Ok(uevent) =
                std::fs::read_to_string(sys_class_hidraw.join(n).join("device/uevent"))
            else {
                return false;
            };
            uevent.lines().any(|l| l == "HID_ID=0003:00002833:00000031")
                && uevent
                    .lines()
                    .any(|l| l.starts_with("HID_PHYS=") && l.ends_with("/input1"))
        })
        .map(|n| Path::new("/dev").join(n))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_the_radio_interface_among_the_hidraw_nodes() {
        let sys = std::env::temp_dir().join(format!("cv1t-hidraw-{}", std::process::id()));
        let node = |name: &str, uevent: &str| {
            let dir = sys.join(name).join("device");
            std::fs::create_dir_all(&dir).unwrap();
            std::fs::write(dir.join("uevent"), uevent).unwrap();
        };
        node(
            "hidraw3",
            "HID_ID=0003:0000046D:0000C262\nHID_PHYS=usb-0000:00:14.0-5/input1\n",
        );
        node("hidraw27", "DRIVER=hid-generic\nHID_ID=0003:00002833:00000031\nHID_NAME=Oculus VR, Inc. Rift\nHID_PHYS=usb-0000:02:00.0-1.1/input0\n");
        node("hidraw26", "DRIVER=hid-generic\nHID_ID=0003:00002833:00000031\nHID_NAME=Oculus VR, Inc. Rift\nHID_PHYS=usb-0000:02:00.0-1.1/input1\n");
        assert_eq!(
            radio_hidraw(&sys),
            Some(std::path::PathBuf::from("/dev/hidraw26"))
        );
        std::fs::remove_dir_all(sys.join("hidraw26")).unwrap();
        assert_eq!(
            radio_hidraw(&sys),
            None,
            "the headset's other interface is not the radio"
        );
        std::fs::remove_dir_all(&sys).unwrap();
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
        assert_eq!(cstr_field(&cfg.ignore_sensor_serials[0]), "WMTD000000004");
        assert_eq!(cstr_field(&cfg.fixed_poses[0].serial), "WMTD000000001");
        assert_eq!(cfg.fixed_poses[0].pos, [1.0, 2.0, 3.0]);
        assert_eq!(cstr_field(&cfg.fixed_poses[1].serial), "");
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
