// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! cv1touchd: owns the Rift CV1 headset radio and sensors through the vendored OpenHMD engine and
//! publishes both Touch controllers' state to shared memory.

#[cfg(test)]
#[path = "../../tests/support/socket_dir.rs"]
mod socket_dir;

#[cfg(all(test, unix))]
#[path = "../../tests/support/expected_abort.rs"]
mod expected_abort;

use std::fs;
use std::io;
use std::mem;
use std::path::{Path, PathBuf};
use std::process;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use cv1touch::control::{self, Command};
use cv1touch::engine::{self, Blob, Engine, EngineOptions, EngineSink};
use cv1touch::flight::{self, FlightRecorder};
use cv1touch::flight_writer;
use cv1touch::frame_queue::FrameQueue;
use cv1touch::hand::{GapEvent, Hand, HandConfig};
use cv1touch::platform::{self, hid, ipc, shutdown, system};
use cv1touch::recorder::{self, Recorder};
use cv1touch::shm::{self, ControllerState, SensorStatus, ShmWriter, HAND_LEFT, HAND_RIGHT};
use cv1touch::track::pipeline::{self, Pipeline, Stamped};
use cv1touch::track::predict::{ShapeParams, VelocityShaper};
use cv1touch::track::repose;
use cv1touch::track::tracker::Stats;
use cv1touch::{align, calib, ffi, input_calibration, modes, settings, setup, supervise, track};

#[path = "cv1touchd/log.rs"]
mod daemon_log;

#[cfg(all(test, unix))]
#[path = "cv1touchd/callback_tests.rs"]
mod callback_tests;

macro_rules! log {
    ($($args:tt)*) => { daemon_log::line(format_args!($($args)*)) };
}

/// Compatibility for preformatted diagnostics; formatting callers can use `log!` without a String.
fn log(msg: impl AsRef<str>) {
    daemon_log::line(format_args!("{}", msg.as_ref()));
}

/// Disable headset tracking LEDs by default while retaining the headset radio for Touch.
const DEFAULT_HMD_LEDS_OFF: bool = true;
const IO_FAILURES_BEFORE_REOPEN: u32 = 50;
const SENSOR_PUBLISH_NS: u64 = 250_000_000;
const RECORDER_RING_BYTES: usize = 64 << 20;
const RECORD_STOP_WAIT: Duration = Duration::from_secs(30);
const RECORD_SHUTDOWN_WAIT: Duration = Duration::from_millis(500);
/// Pixels at or below this are treated as sensor dark noise and are not recorded;
/// blobs start at the engine's threshold of 36, so their halos are kept.
const RAW_FRAME_FLOOR: u8 = 7;
const MAX_CLIENTS: usize = 16;
const ENGINE_WORKER: u8 = 1;
const RADIO_WORKER: u8 = 2;

/// Which pose cv1touchd publishes: its own tracker (OpenHMD only acquires) or OpenHMD's.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum TrackerKind {
    Rust,
    OpenHmd,
}

type QueuedFrame = (Box<engine::FrameInfo>, Vec<Blob>);

#[derive(Clone, Copy)]
struct Haptic {
    duration_s: f32,
    frequency_hz: f32,
    amplitude: f32,
}

type StationPoses = Vec<(String, nalgebra::Isometry3<f64>)>;
type TimedStationPoses = (u64, StationPoses);

/// Latest factory-normalized input samples. Keep these before output correction so a second
/// calibration measures the hardware rather than the result of the first calibration.
#[derive(Clone, Default, serde::Serialize)]
struct RawInputs {
    identity: String,
    connected: bool,
    received_host_ns: u64,
    buttons: u32,
    trigger: f32,
    grip: f32,
    stick: [f32; 2],
}

#[derive(Default)]
struct InputRuntime {
    calibration: Option<input_calibration::CalibrationFile>,
    error: Option<String>,
    latest: [RawInputs; 2],
}

impl InputRuntime {
    fn set_identity(&mut self, hand: usize, identity: String) {
        if self.latest[hand].identity != identity {
            // Never associate the previous device's sample with the new device's identity.
            self.latest[hand] = RawInputs {
                identity,
                ..Default::default()
            };
        }
    }

    fn calibration_applies(&self, hand: usize) -> bool {
        self.calibration.as_ref().is_some_and(|c| {
            !self.latest[hand].identity.is_empty()
                && c.hands[hand].identity == self.latest[hand].identity
        })
    }

    fn apply(&mut self, hand: usize, state: &mut ControllerState) {
        let raw = &mut self.latest[hand];
        raw.connected = cv1touch::hand::reporting(state.flags);
        raw.received_host_ns = state.received_host_ns;
        raw.buttons = state.buttons;
        raw.trigger = state.trigger;
        raw.grip = state.grip;
        raw.stick = state.stick;
        let values = if !raw.connected {
            // Zero in factory coordinates can be a partial press after calibration. A dropout
            // must therefore release inputs explicitly, without passing through calibration.
            (0.0, 0.0, [0.0; 2])
        } else if let Some(c) = &self.calibration {
            c.apply(hand, &raw.identity, raw.trigger, raw.grip, raw.stick)
        } else {
            input_calibration::factory_values(raw.trigger, raw.grip, raw.stick)
        };
        (state.trigger, state.grip, state.stick) = values;
    }

    fn status(&self) -> serde_json::Value {
        serde_json::json!({
            "configured": self.calibration.is_some(),
            "calibrated": [self.calibration_applies(0), self.calibration_applies(1)],
            "error": self.error,
        })
    }
}

fn reload_inputs(runtime: &Mutex<InputRuntime>, path: &std::path::Path) -> Result<(), String> {
    // Parse and validate before taking the publication lock; errors retain the last good state.
    let calibration = input_calibration::load(path)?;
    let mut runtime = runtime.lock().unwrap();
    runtime.calibration = calibration;
    runtime.error = None;
    Ok(())
}

struct Daemon {
    shm: ShmWriter,
    rec: Recorder,
    /// Recording I/O never holds the global control lock; competing recording commands refuse.
    record_command: Mutex<()>,
    /// The radio's recent traffic, recorded around a controller's dropout.
    flight: FlightRecorder,
    quit: AtomicBool,
    /// The first critical worker that stopped unexpectedly; read without any worker locks.
    failed_worker: AtomicU8,
    /// A client asked the daemon to stop (control `stop`).
    stop_requested: AtomicBool,
    haptics: Mutex<[Option<Haptic>; 2]>,
    /// Sensor intrinsics and LED models still to be written into the active recording.
    session: Mutex<recorder::SessionInfo>,
    opts: Mutex<EngineOptions>,
    /// Options revision requested under the opts lock, and revision actually opened.
    engine_requested: std::sync::atomic::AtomicU64,
    engine_applied: std::sync::atomic::AtomicU64,
    mode: Mutex<String>,
    reopen: AtomicBool,
    tracker: TrackerKind,
    track_in: Mutex<FrameQueue<QueuedFrame>>,
    /// Complete sensor diagnostics; the SHM block is only a compatibility prefix.
    sensors: Mutex<Vec<SensorStatus>>,
    /// Last start attempts, published with counters; retained until forgotten or engine close.
    sensor_streams: Mutex<Vec<engine::SensorStreamStart>>,
    /// Acquisition counters from the same publication, separate from released analysis buffers.
    sensor_video: Mutex<Vec<engine::SensorVideo>>,
    track_stats: Mutex<[(Stats, bool); 2]>,
    pending_frames_dropped: std::sync::atomic::AtomicU64,
    /// Per sensor serial: its landmarks' newest shift (px) and how many were measured.
    landmark_shifts: Mutex<std::collections::BTreeMap<String, (f64, f64, usize)>>,
    /// Per sensor serial: where it stands with the pose watch (`repose`), for `status`.
    sensor_poses: Mutex<std::collections::BTreeMap<String, PoseNote>>,
    /// Per sensor serial: its agreement with the others, as of the last tick (`status`).
    sensor_agreement: Mutex<std::collections::BTreeMap<String, track::tracker::CameraAgreement>>,
    /// Fewer than two sensors agree with each other (see `repose::Change::Unsure`).
    sensors_unsure: AtomicBool,
    /// The calibration file the poses came from, which a sensor placed again is written to, and
    /// its fingerprint as loaded or last written: a file changed since is not written over.
    calibration: Mutex<Option<LoadedCalibration>>,
    /// The backups the daemon's own writes made, oldest first (see `thin_own_backups`).
    own_backups: Mutex<Vec<PathBuf>>,
    /// How the Lighthouse universe moved since the registration (see `universe_watch`), with
    /// the registration it was measured against (its `created_unix_s`).
    universe_shift: Mutex<Option<(u64, nalgebra::Isometry3<f64>)>>,
    /// Asks `universe_watch` to measure now rather than at its next check.
    universe_recheck: AtomicBool,
    /// The base stations' raw poses as the SteamVR driver last reported them (`stations`), with
    /// when they arrived (shm clock).
    driver_stations: Mutex<Option<TimedStationPoses>>,
    /// The fallback's last state and deadline, for `universe-watch`; never held during a connect.
    universe_connection: Mutex<UniverseConnection>,
    /// `universe-watch off` stops the watch (the registration stays as last followed).
    universe_watch_on: AtomicBool,
    /// What the watch follows the registration through now, for `universe-watch`.
    universe_following: Mutex<String>,
    /// `shaping off` publishes the tracked velocities as they are (see `track::predict`).
    shaping_on: AtomicBool,
    /// Percent of the tracked linear velocity SteamVR extrapolates positions with
    /// (`prediction position`), published in the alignment for the driver; saved in
    /// `settings_path` on every change and read from it at start.
    position_prediction: AtomicU32,
    /// The same for the angular velocity and orientations (`prediction rotation`).
    rotation_prediction: AtomicU32,
    /// Held while the alignment is published, so that the prediction percents published are the
    /// newest (see `publish_alignment`).
    alignment_publish: Mutex<()>,
    settings_path: PathBuf,
    /// The current recording keeps the sensors' pixels (RAW_FRAME records).
    record_frames: AtomicBool,
    /// Extra stamp shift of published poses, ns (`latency` command); saved in `settings_path`
    /// on every change and read from it at start.
    extra_latency_ns: std::sync::atomic::AtomicI64,
    /// How far the controllers are shown pitched from their tracked pose, thousandths of a degree
    /// (`tilt` command), turned into the render offsets published; saved and read like the
    /// above.
    tilt_mdeg: std::sync::atomic::AtomicI32,
    /// The render offsets as fitted (per hand, xyzw), before the tilt: what `tilt` turns, so that
    /// a republish does not turn them again.
    fitted_render_orient: Mutex<[[f32; 4]; 2]>,
    /// Per hand: the newest battery percentage from the radio, -1 before the first.
    battery: [std::sync::atomic::AtomicI32; 2],
    /// Wakes the radio thread: new camera frames or bodies, or time to stop. It lives as long as
    /// the daemon, so a sensor thread never signals a closed (or reused) handle.
    radio_wake: hid::Waker,
    /// The radio thread runs: it records the radio's reports (stamped as they land), not the engine.
    radio_on: AtomicBool,
    /// Input injection for automated tests.
    inject: Injection,
    inputs: Mutex<InputRuntime>,
}

impl Daemon {
    fn wake_radio(&self) {
        self.radio_wake.wake();
    }
}

/// A sensor whose landmarks moved this far (px) two windows running is reported as knocked; one
/// landmark alone (a single blob mode) wanders a few tenths and needs more.
const LANDMARK_ALERT_PX: f64 = 0.4;
const LANDMARK_ALERT_SINGLE_PX: f64 = 1.0;
const LANDMARK_WINDOW_NS: u64 = 60_000_000_000;

const BIAS_SAVE_NS: u64 = 30_000_000_000;

/// Each controller's IMU biases across restarts (see `track::biases`): seeded into a new tracker,
/// taken every 30 s while it tracks and written then and when the engine closes.
struct BiasMemory {
    file: track::biases::BiasFile,
    keys: [Option<String>; 2],
    next_save: u64,
    dirty: bool,
}

impl BiasMemory {
    fn load() -> BiasMemory {
        BiasMemory {
            file: track::biases::load(&track::biases::default_path()),
            keys: [None, None],
            next_save: 0,
            dirty: false,
        }
    }

    fn seed(&mut self, p: &mut Pipeline, hand: usize, leds: &[calib::capture::Led]) {
        let key = track::biases::key(hand, leds);
        if let Some(e) = self.file.controllers.get(&key) {
            p.seed_biases(hand, e.gyro(), e.accel());
            let a = e.accel();
            log!(
                "{}: IMU biases from the last run (accelerometer {:+.3} {:+.3} {:+.3} m/s^2)",
                ["left", "right"][hand],
                a.x,
                a.y,
                a.z
            );
        }
        self.keys[hand] = Some(key);
    }

    /// Every 30 s, the tracking controllers' biases: the file's new contents when they changed, for
    /// `save_biases` (on the engine thread: no file I/O on the radio thread).
    fn update(&mut self, p: &Pipeline, now: u64) -> Option<track::biases::BiasFile> {
        if now < self.next_save {
            return None;
        }
        self.next_save = now + BIAS_SAVE_NS;
        for h in [HAND_LEFT, HAND_RIGHT] {
            let (Some(key), Some(s)) = (self.keys[h].as_ref(), p.latest(h)) else {
                continue;
            };
            if s.output.tracking {
                let (g, a) = (s.output.gyro_bias, s.output.accel_bias);
                let unix = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0, |d| d.as_secs());
                self.file.controllers.insert(
                    key.clone(),
                    track::biases::Entry {
                        gyro: [g.x, g.y, g.z],
                        accel: [a.x, a.y, a.z],
                        updated_unix_s: unix,
                    },
                );
                self.dirty = true;
            }
        }
        self.take_changed()
    }

    /// The file's contents if they changed since last taken.
    fn take_changed(&mut self) -> Option<track::biases::BiasFile> {
        mem::take(&mut self.dirty).then(|| self.file.clone())
    }
}

fn save_biases(f: &track::biases::BiasFile) {
    if let Err(e) = track::biases::save(&track::biases::default_path(), f) {
        log!("IMU biases not saved: {e}");
    }
}

#[derive(Debug, PartialEq, Eq)]
enum LandmarkNotice {
    Moved,
    Back,
}

#[derive(Default)]
struct LandmarkAlert {
    over: u32,
    /// The threshold and landmark count needed to clear the current alert.
    active: Option<(f64, usize)>,
}

impl LandmarkAlert {
    fn update(&mut self, measurement: Option<(f64, usize)>) -> Option<LandmarkNotice> {
        let Some((magnitude, landmarks)) = measurement else {
            self.over = 0;
            return None;
        };
        let threshold = if landmarks >= 2 {
            LANDMARK_ALERT_PX
        } else {
            LANDMARK_ALERT_SINGLE_PX
        };
        if let Some((mut active_threshold, mut active_landmarks)) = self.active {
            if landmarks < active_landmarks {
                return None;
            }
            if threshold < active_threshold {
                active_threshold = threshold;
                active_landmarks = landmarks;
            }
            self.active = Some((active_threshold, active_landmarks));
            if magnitude < active_threshold {
                self.active = None;
                self.over = 0;
                return Some(LandmarkNotice::Back);
            }
            return None;
        }
        self.over = if magnitude >= threshold {
            self.over.saturating_add(1)
        } else {
            0
        };
        if self.over >= 2 {
            self.active = Some((threshold, landmarks));
            Some(LandmarkNotice::Moved)
        } else {
            None
        }
    }
}

/// Watches the landmarks (static infrared sources such as the base stations) for a moved sensor.
struct LandmarkWatch {
    monitor: track::landmarks::BackgroundMonitor,
    /// Per sensor: whether its movement has been reported, and the consecutive windows over
    /// the alert threshold.
    alerts: std::collections::BTreeMap<String, LandmarkAlert>,
    /// Consecutive windows without any landmark measured, per sensor.
    missing: std::collections::BTreeMap<String, u32>,
}

impl LandmarkWatch {
    fn load() -> Option<LandmarkWatch> {
        let path = track::landmarks::default_path();
        match track::landmarks::load(&path) {
            Ok(f) if !f.landmarks.is_empty() => {
                log!(
                    "watching {} landmarks from {} for moved sensors",
                    f.landmarks.len(),
                    path.display()
                );
                Some(LandmarkWatch {
                    monitor: match track::landmarks::BackgroundMonitor::new(
                        f.landmarks,
                        LANDMARK_WINDOW_NS,
                        20,
                    ) {
                        Ok(monitor) => monitor,
                        Err(e) => {
                            log!("could not start the landmark worker: {e}");
                            return None;
                        }
                    },
                    alerts: Default::default(),
                    missing: Default::default(),
                })
            }
            Ok(_) => None,
            Err(e) => {
                if path.exists() {
                    log(e);
                }
                None
            }
        }
    }

    /// Stops watching the sensor `serial`: its landmarks were learned where it stood.
    fn forget(&mut self, d: &Daemon, serial: &str) {
        self.monitor.forget(serial);
        self.alerts
            .retain(|s, _| !cv1touch::status::same_sensor(s, serial));
        self.missing
            .retain(|s, _| !cv1touch::status::same_sensor(s, serial));
        d.landmark_shifts
            .lock()
            .unwrap()
            .retain(|s, _| !cv1touch::status::same_sensor(s, serial));
    }

    fn frame(&mut self, info: &engine::FrameInfo, blobs: &[Blob]) {
        let bytes = info.serial.map(|c| c as u8);
        let end = bytes.iter().position(|c| *c == 0).unwrap_or(bytes.len());
        let Ok(serial) = std::str::from_utf8(&bytes[..end]) else {
            return;
        };
        self.monitor.frame(
            serial,
            info.frame_start_host_ns,
            blobs
                .iter()
                .filter(|b| b.led_id < 0)
                .map(|b| nalgebra::Vector2::new(b.x as f64, b.y as f64)),
        );
        // Capture timestamps can lag the radio loop. Publication uses its current
        // clock in poll(), after overdue windows have been advanced.
    }

    fn poll(&mut self, d: &Daemon, now: u64) {
        let alerts = &mut self.alerts;
        let missing = &mut self.missing;
        let receive = |sensor: &str, shift: Option<track::landmarks::Shift>| {
            let mut out = d.landmark_shifts.lock().unwrap();
            let alert = alerts.entry(sensor.to_owned()).or_default();
            let notice = alert.update(shift.as_ref().map(|s| (s.shift.norm(), s.landmarks)));
            match shift {
                Some(s) => {
                    missing.remove(sensor);
                    out.insert(sensor.to_owned(), (s.shift.x, s.shift.y, s.landmarks));
                    match notice {
                        Some(LandmarkNotice::Moved) => log!("sensor {sensor}: its landmarks moved ({:+.2}, {:+.2}) px: it may have been knocked (cv1touch calibrate refine REC)", s.shift.x, s.shift.y),
                        Some(LandmarkNotice::Back) => log!("sensor {sensor}: landmarks back in place"),
                        None => {}
                    }
                }
                None => {
                    out.remove(sensor);
                    let n = missing.entry(sensor.to_owned()).or_default();
                    *n += 1;
                    if *n == 3 {
                        log!("sensor {sensor}: its landmarks could not be matched safely for 3 minutes (the spots may be hidden, off, or beyond this check's range)");
                    }
                }
            }
        };
        // Reclaim buffers without accepting results, then invalidate overdue sequences.
        // An older recovery must not clear an alert before a newer unknown is reported.
        // All exchanges use try_lock; analysis never holds a lock needed by radio.
        self.monitor.recycle();
        self.monitor.tick(now);
        self.monitor.poll(receive);
        if self.monitor.take_first_fault() {
            log!("the landmark check gave up on one window after an internal error; it carries on with the next window");
        }
    }
}

impl EngineSink for Daemon {
    fn frame(&self, info: &engine::FrameInfo, blobs: &[Blob]) {
        if self.rec.active() {
            self.rec.write(
                recorder::FRAME_OBS,
                info.frame_start_host_ns,
                &[recorder::as_bytes(info), recorder::slice_bytes(blobs)],
            );
        }
        if self.tracker == TrackerKind::Rust {
            self.track_in.lock().unwrap().push_frame(
                &engine::cstr_field(&info.serial),
                info.frame_start_host_ns,
                (Box::new(*info), blobs.to_vec()),
            );
            self.wake_radio();
        }
    }

    fn image(
        &self,
        info: &engine::FrameInfo,
        width: usize,
        height: usize,
        stride: usize,
        pixels: &[u8],
    ) {
        if self.record_frames.load(Ordering::Acquire) && self.rec.active() {
            let payload =
                recorder::encode_frame(info, width, height, stride, pixels, RAW_FRAME_FLOOR);
            self.rec
                .write(recorder::RAW_FRAME, info.frame_start_host_ns, &[&payload]);
        }
    }

    fn report(&self, source: i32, ts: u64, data: &[u8]) {
        // the radio thread records its own copy of the radio's reports, stamped as they land
        let radio_thread = source == 1 && self.radio_on.load(Ordering::Acquire);
        if self.rec.active() && !radio_thread {
            self.rec.write(
                recorder::RAW_REPORT,
                ts,
                &[&(source as u32).to_le_bytes(), data],
            );
        }
        if source == 1 && !radio_thread {
            self.flight
                .note(recorder::RAW_REPORT, ts, &[&1u32.to_le_bytes(), data]);
        }
        if source == 1 {
            for (hand, pct) in cv1touch::hand::battery_from_radio(data) {
                // logged when it changes: it read 100 for hours on fresh cells, and the first
                // drop will show what the channel really is
                let was = self.battery[hand].swap(pct, Ordering::Relaxed);
                if was != pct {
                    log!(
                        "{} Touch battery {pct}%{}",
                        ["left", "right"][hand],
                        if was >= 0 {
                            format!(" (was {was}%)")
                        } else {
                            String::new()
                        }
                    );
                }
            }
        }
        if self.tracker == TrackerKind::Rust && source == 0 {
            if let Some(us) = pipeline::hmd_report_timestamp(data) {
                self.track_in.lock().unwrap().push_clock(ts, us);
            }
        }
    }
}

fn publish_disconnected(d: &Daemon) {
    d.engine_applied.store(0, Ordering::Release);
    d.track_in.lock().unwrap().register([]);
    d.pending_frames_dropped.store(0, Ordering::Relaxed);
    d.inputs.lock().unwrap().latest = Default::default();
    let now = shm::now_ns();
    for h in [HAND_LEFT, HAND_RIGHT] {
        d.shm.publish_hand(
            h,
            &ControllerState {
                publish_host_ns: now,
                battery_percent: -1,
                ..Default::default()
            },
        );
    }
    publish_sensor_snapshot(d, &[], &[], &[]);
    d.shm.signal();
    d.shm.heartbeat(now);
}

fn publish_sensor_snapshot(
    d: &Daemon,
    sensors: &[SensorStatus],
    streams: &[engine::SensorStreamStart],
    video: &[engine::SensorVideo],
) {
    // Status takes these locks in the same order, so counters and reasons share one sample.
    let mut current = d.sensors.lock().unwrap();
    *d.sensor_streams.lock().unwrap() = streams.to_vec();
    *d.sensor_video.lock().unwrap() = video.to_vec();
    *current = sensors.to_vec();
    drop(current);
    d.shm.publish_sensors(sensors);
}

fn sensor_status(d: &Daemon, s: &engine::Sensor) -> SensorStatus {
    let mut out = SensorStatus::default();
    let serial = engine::sensor_serial(s);
    shm::set_serial(&mut out, &serial);
    // a sensor being placed again has no pose in the engine for the while, but a calibrated one
    // in ours: it counts as placed
    let placing = matches!(
        d.sensor_poses.lock().unwrap().get(&serial),
        Some(PoseNote::Moved { .. })
    );
    out.placed = (s.have_camera_pose || placing) as u32;
    out.frames = s.frames;
    out.pose_fixes[HAND_LEFT] = s.pose_fixes[2]; // OpenHMD device id 2 = left
    out.pose_fixes[HAND_RIGHT] = s.pose_fixes[1]; // OpenHMD device id 1 = right
    out.camera_orient = s.camera_orient;
    out.camera_pos = s.camera_pos;
    out
}

/// Writes whatever session info (sensor intrinsics, each controller's LED model) has become
/// available and is not in the recording yet.
fn write_session_info(d: &Daemon, e: &Engine) {
    let mut session = d.session.lock().unwrap();
    if session.complete(2) {
        return;
    }
    let sensors = e.sensors();
    let models: Vec<(i32, Vec<engine::LedEntry>)> = [HAND_LEFT, HAND_RIGHT]
        .iter()
        .filter_map(|&h| Some((e.controller(h)?.device_id, e.led_model(h))))
        .collect();
    let known: Vec<(i32, bool)> = models
        .iter()
        .map(|(dev, m)| (*dev, !m.is_empty()))
        .collect();
    let (write_sensors, devices) = session.plan(!sensors.is_empty(), &known);
    let now = shm::now_ns();
    if write_sensors {
        for s in &sensors {
            d.rec
                .write(recorder::SENSOR_INFO, now, &[recorder::as_bytes(s)]);
        }
    }
    for (device, model) in models.iter().filter(|(dev, _)| devices.contains(dev)) {
        d.rec.write(
            recorder::LED_MODEL,
            now,
            &[
                &recorder::led_model_prefix(*device, model.len()),
                recorder::slice_bytes(model),
            ],
        );
    }
}

/// The tracking pipeline for an engine just opened: calibrated sensors only (None without any, so
/// OpenHMD's poses are published).
fn new_pipeline(d: &Daemon, opts: &EngineOptions, e: &Engine) -> Option<Pipeline> {
    if d.tracker != TrackerKind::Rust || opts.fixed_poses.is_empty() {
        return None;
    }
    let p = calibrated_pipeline(opts, &e.sensors());
    d.track_in.lock().unwrap().clear();
    Some(p)
}

/// A pipeline with the open sensors the calibration has a pose for, registered under the
/// spelling their frames carry (the calibration may spell the serial in another letter case, as
/// the native engine allows).
fn calibrated_pipeline(opts: &EngineOptions, sensors: &[engine::Sensor]) -> Pipeline {
    let mut p = Pipeline::new(pipeline::Options::default());
    for s in sensors {
        let serial = engine::sensor_serial(s);
        if let Some(pose) = modes::calibrated_pose(opts, &serial) {
            p.sensor(
                &serial,
                calib::camera::CameraModel::from_sensor(s),
                calib::iso_from_xyzw(pose.orient, pose.pos),
            );
        }
    }
    p
}

/// The prediction percents a daemon starts with: (positions, orientations).
fn start_prediction(settings: &settings::Settings) -> (u32, u32) {
    (settings.prediction_percent, settings.rotation_percent())
}

/// The saved latency adjustment as the stamp shift it is, ns.
fn start_latency_ns(settings: &settings::Settings) -> i64 {
    (settings.latency_ms() * 1e6).round() as i64
}

/// The saved tilt in thousandths of a degree.
fn start_tilt_mdeg(settings: &settings::Settings) -> i32 {
    (settings.tilt_deg() * 1000.0).round() as i32
}

/// The daemon's tilt, degrees.
fn tilt_deg(d: &Daemon) -> f64 {
    d.tilt_mdeg.load(Ordering::Acquire) as f64 / 1000.0
}

/// The settings as they are now: what every change saves, so that none drops another.
fn current_settings(d: &Daemon) -> settings::Settings {
    let extra_ns = d.extra_latency_ns.load(Ordering::Acquire);
    let tilt_mdeg = d.tilt_mdeg.load(Ordering::Acquire);
    settings::Settings {
        prediction_percent: d.position_prediction.load(Ordering::Acquire),
        rotation_prediction_percent: Some(d.rotation_prediction.load(Ordering::Acquire)),
        latency_adjustment_ms: (extra_ns != 0).then(|| extra_ns as f64 / 1e6),
        tilt_deg: (tilt_mdeg != 0).then(|| tilt_mdeg as f64 / 1000.0),
        ..settings::Settings::default()
    }
}

/// Saves the settings as they are now; the reply's note when that fails.
fn save_settings(d: &Daemon) -> String {
    match settings::save(&d.settings_path, &current_settings(d)) {
        Ok(()) => String::new(),
        Err(e) => format!(" (not saved: {e})"),
    }
}

/// How published poses are stamped off their IMU samples' arrival time (the `latency` reply and
/// the start-up note): the pipeline's measured offset and the adjustment on top of it.
fn latency_text(extra_ns: i64) -> String {
    let total_ms = (pipeline::DEFAULT_EXPOSURE_OFFSET_NS + extra_ns) as f64 / 1e6;
    let adjustment = if extra_ns == 0 {
        "no adjustment".to_string()
    } else {
        format!("{:+.1} ms adjustment", extra_ns as f64 / 1e6)
    };
    let measured = pipeline::DEFAULT_EXPOSURE_OFFSET_NS as f64 / 1e6;
    if total_ms.abs() < 0.05 {
        return format!(
            "published poses stamped at their IMU samples' arrival time ({measured:.1} ms \
             measured, {adjustment})"
        );
    }
    format!(
        "published poses stamped {:.1} ms {} than their IMU samples' arrival time ({measured:.1} \
         ms measured, {adjustment})",
        total_ms.abs(),
        if total_ms < 0.0 { "later" } else { "earlier" },
    )
}

/// `alignment` carrying the prediction percents (positions, orientations) the driver applies.
fn with_prediction(
    mut alignment: shm::Alignment,
    (position, rotation): (u32, u32),
) -> shm::Alignment {
    alignment.prediction_position = position;
    alignment.prediction_rotation = rotation;
    alignment.valid |= ffi::CV1T_ALIGN_PREDICTION;
    alignment
}

/// `alignment` with the controllers shown pitched `tilt_deg` from their tracked pose, positive
/// with the forward end up: each hand's render offset as fitted (`fitted`, LED model <- render
/// frame) turned by that pitch about the render frame's own lateral axis, so that the pose
/// SteamVR hands applications, and the render model with it, pitch together. 0 gives the fitted
/// offsets back exactly, and so does a hand without a fitted offset (its render bit unset).
fn with_tilt(
    mut alignment: shm::Alignment,
    fitted: &[[f32; 4]; 2],
    tilt_deg: f64,
) -> shm::Alignment {
    if tilt_deg == 0.0 {
        alignment.render_orient = *fitted;
        return alignment;
    }
    let pitch = nalgebra::UnitQuaternion::from_axis_angle(
        &nalgebra::Vector3::x_axis(),
        tilt_deg.to_radians(),
    );
    for (hand, q) in fitted.iter().enumerate() {
        let bit = if hand == HAND_LEFT {
            ffi::CV1T_ALIGN_RENDER_LEFT
        } else {
            ffi::CV1T_ALIGN_RENDER_RIGHT
        };
        if alignment.valid & bit == 0 {
            // no fitted offset for this hand (nothing the driver applies): nothing to turn
            alignment.render_orient[hand] = *q;
            continue;
        }
        let fitted = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            q[3] as f64,
            q[0] as f64,
            q[1] as f64,
            q[2] as f64,
        ));
        let turned = fitted * pitch;
        alignment.render_orient[hand] = [
            turned.i as f32,
            turned.j as f32,
            turned.k as f32,
            turned.w as f32,
        ];
    }
    alignment
}

/// `alignment` with the daemon's tilt applied to the fitted render offsets.
fn with_daemon_tilt(d: &Daemon, alignment: shm::Alignment) -> shm::Alignment {
    let fitted = *d.fitted_render_orient.lock().unwrap();
    with_tilt(alignment, &fitted, tilt_deg(d))
}

/// How the controllers are shown, in words.
fn tilt_text(deg: f64) -> String {
    if deg == 0.0 {
        "the controllers are shown as tracked (no tilt)".to_string()
    } else {
        format!(
            "the controllers are shown with the forward end pitched {:.1} deg {} from the tracked \
             pose",
            deg.abs(),
            if deg > 0.0 { "up" } else { "down" },
        )
    }
}

/// The daemon's prediction percents.
fn prediction_percents(d: &Daemon) -> (u32, u32) {
    (
        d.position_prediction.load(Ordering::Acquire),
        d.rotation_prediction.load(Ordering::Acquire),
    )
}

/// Publishes `alignment` with the daemon's prediction percents. One at a time, the percents read
/// while publishing: a reload or the universe watch cannot put back percents a `prediction`
/// request has just changed.
fn publish_alignment(d: &Daemon, alignment: shm::Alignment) {
    let _one = d.alignment_publish.lock().unwrap();
    // `alignment` carries the render offsets as fitted: what the tilt turns from now on
    *d.fitted_render_orient.lock().unwrap() = alignment.render_orient;
    d.shm.publish_alignment(&with_prediction(
        with_daemon_tilt(d, alignment),
        prediction_percents(d),
    ));
}

/// Publishes the alignment in place with the daemon's settings (the tilt on the fitted render
/// offsets, the prediction percents). Read while holding the publishers' lock, it is the newest:
/// a registration the universe watch has just moved is not put back.
fn republish_settings(d: &Daemon) {
    let _one = d.alignment_publish.lock().unwrap();
    if let Some(alignment) = d.shm.alignment() {
        d.shm.publish_alignment(&with_prediction(
            with_daemon_tilt(d, alignment),
            prediction_percents(d),
        ));
    }
}

/// What SteamVR extrapolates positions and orientations with, in words.
fn prediction_text(position: u32, rotation: u32) -> String {
    if position == rotation {
        format!("SteamVR extrapolates with {position}% of the tracked velocities")
    } else {
        format!(
            "SteamVR extrapolates positions with {position}% of the tracked velocity and \
             orientations with {rotation}%"
        )
    }
}

/// Puts the tracker's pose into the published state, stamped `extra_ns` earlier than the
/// pipeline's estimate of when the controller was there (the `latency` adjustment).
/// `shaper` turns the velocities into the ones SteamVR's constant-velocity prediction should use
/// (see `track::predict`). They are published as they are, for the tools that carry a pose
/// forward too; the driver hands SteamVR the share of them the alignment's prediction percents
/// say. The pose is published as tracked.
fn apply_tracked(
    state: &mut ControllerState,
    s: &Stamped,
    extra_ns: i64,
    shaper: &mut VelocityShaper,
    shaping: bool,
) {
    let q = s.output.world_from_model.rotation;
    let t = s.output.world_from_model.translation.vector;
    state.orient = [q.i as f32, q.j as f32, q.k as f32, q.w as f32];
    state.pos = [t.x as f32, t.y as f32, t.z as f32];
    let (w, v) = shaper.shape_body(
        s.host_ns,
        &q,
        s.gyro - s.gyro_bias,
        s.output.tracking.then_some(s.output.vel),
    );
    let (w, v) = if shaping {
        (w, v)
    } else {
        (s.output.ang_vel_world, Some(s.output.vel))
    };
    let v = v.unwrap_or(s.output.vel);
    state.lin_vel = [v.x as f32, v.y as f32, v.z as f32];
    // the driver gives SteamVR ang_vel - gyro_bias as the body rate it integrates in: the shaped
    // world rate, turned into the body
    let body = q.inverse() * w + s.gyro_bias;
    state.ang_vel = [body.x as f32, body.y as f32, body.z as f32];
    state.gyro_bias = [
        s.gyro_bias.x as f32,
        s.gyro_bias.y as f32,
        s.gyro_bias.z as f32,
    ];
    let shift = |t: u64| (t as i64).saturating_sub(extra_ns).max(0) as u64;
    state.sample_host_ns = shift(s.host_ns);
    state.last_position_fix_host_ns = if s.last_fix_host_ns == 0 {
        0
    } else {
        shift(s.last_fix_host_ns)
    };
    if s.output.tracking {
        state.flags |= ffi::CV1T_FLAG_POSITION;
    } else {
        state.flags &= !ffi::CV1T_FLAG_POSITION;
    }
}

const UNIVERSE_CHECK_MS: u64 = 60_000;
const UNIVERSE_RETRY_MAX_MS: u64 = 15 * 60_000;
const UNIVERSE_PROCESS_CHECK_MS: u64 = 10_000;
/// How often the watch reads SteamVR's log and looks at the stations the driver reports (cheap:
/// no SteamVR connection; the driver reports every 2 s).
const UNIVERSE_TICK_MS: u64 = 1_000;
/// The driver's report counts as current this long; past it, the watch connects to SteamVR itself
/// (an older driver that does not report), at most every `UNIVERSE_CHECK_MS` while connected.
const DRIVER_STATIONS_FRESH_NS: u64 = 10_000_000_000;

#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum UniverseConnectionState {
    #[default]
    Unknown,
    Stopped,
    CheckFailed,
    ConnectFailed,
    Connected,
}

#[derive(Clone, Copy)]
struct UniverseConnection {
    next_attempt: Option<std::time::Instant>,
    retry_ms: u64,
    state: UniverseConnectionState,
}

impl Default for UniverseConnection {
    fn default() -> Self {
        Self {
            next_attempt: None,
            retry_ms: UNIVERSE_CHECK_MS,
            state: UniverseConnectionState::Unknown,
        }
    }
}

impl UniverseConnection {
    fn stations(
        &mut self,
        now: std::time::Instant,
        host_ns: u64,
        reported: Option<TimedStationPoses>,
        running: impl FnOnce() -> io::Result<bool>,
        connect: impl FnOnce() -> Result<align::Stations, String>,
        say: impl FnMut(String),
    ) -> Option<align::Stations> {
        match reported.filter(|(at, _)| host_ns.saturating_sub(*at) < DRIVER_STATIONS_FRESH_NS) {
            Some((_, stations)) => Some(stations),
            None => self.connect(now, running, connect, say),
        }
    }

    /// Failed OpenVR inits can leak descriptors inside the runtime. Never initialize it while
    /// vrserver is absent or cannot be checked; a server still starting is retried less often.
    /// Process checks run only when due. A stopped server resets the backoff when next checked,
    /// then is checked every 10 s, without scanning processes on every universe tick.
    fn connect<T>(
        &mut self,
        now: std::time::Instant,
        running: impl FnOnce() -> io::Result<bool>,
        connect: impl FnOnce() -> Result<T, String>,
        mut say: impl FnMut(String),
    ) -> Option<T> {
        if self.next_attempt.is_some_and(|next| now < next) {
            return None;
        }
        match running() {
            Ok(false) => {
                self.next_attempt = Some(now + Duration::from_millis(UNIVERSE_PROCESS_CHECK_MS));
                self.retry_ms = UNIVERSE_CHECK_MS;
                if self.state != UniverseConnectionState::Stopped {
                    say("universe watch: SteamVR is not running; waiting for it".into());
                }
                self.state = UniverseConnectionState::Stopped;
                return None;
            }
            Err(e) => {
                self.next_attempt = Some(now + Duration::from_millis(UNIVERSE_PROCESS_CHECK_MS));
                if self.state != UniverseConnectionState::CheckFailed {
                    say(format!(
                        "universe watch: cannot check SteamVR processes: {e}"
                    ));
                }
                self.state = UniverseConnectionState::CheckFailed;
                return None;
            }
            Ok(true) => {}
        }
        match connect() {
            Ok(client) => {
                self.retry_ms = UNIVERSE_CHECK_MS;
                self.next_attempt = Some(now + Duration::from_millis(UNIVERSE_CHECK_MS));
                if !matches!(
                    self.state,
                    UniverseConnectionState::Unknown | UniverseConnectionState::Connected
                ) {
                    say("universe watch: SteamVR is available again".into());
                }
                self.state = UniverseConnectionState::Connected;
                Some(client)
            }
            Err(e) => {
                self.next_attempt = Some(now + Duration::from_millis(self.retry_ms));
                self.retry_ms = (self.retry_ms * 2).min(UNIVERSE_RETRY_MAX_MS);
                if self.state != UniverseConnectionState::ConnectFailed {
                    say(format!(
                        "universe watch: {e}; retrying less often, up to 15 min apart"
                    ));
                }
                self.state = UniverseConnectionState::ConnectFailed;
                None
            }
        }
    }

    fn status(&self, now: std::time::Instant) -> String {
        let seconds = self.next_attempt.map_or(0, |at| {
            let remaining = at.saturating_duration_since(now);
            remaining.as_secs() + u64::from(remaining.subsec_nanos() != 0)
        });
        match self.state {
            UniverseConnectionState::Unknown => "waiting to check SteamVR".into(),
            UniverseConnectionState::Stopped => {
                format!("waiting for SteamVR to start; next check in {seconds} s")
            }
            UniverseConnectionState::CheckFailed => {
                format!("cannot check whether SteamVR is running; next check in {seconds} s")
            }
            UniverseConnectionState::ConnectFailed => {
                format!("retrying SteamVR; next in {seconds} s")
            }
            UniverseConnectionState::Connected => {
                format!("SteamVR connection succeeded; checking once a minute, next in {seconds} s")
            }
        }
    }
}

/// Keeps the registration on the headset as SteamVR tracks it. SteamVR tracks the headset in the
/// frame of the base station it bootstrapped from, and re-solves the stations' poses relative to
/// each other by centimetres and differently each session (one station by 110 and then 120 mm
/// within a minute on 2026-10-15); the stations themselves do not move. So the registration
/// follows that station alone, from where it is in the world (`align::Registration::station_map`)
/// and where SteamVR has it now; SteamVR's log names it (`steamvr_log`). Through a dropout the
/// watch keeps to the last station named; with none named yet, it follows the stations' moves
/// together. The stations' poses come from the driver every 2 s, or a short-lived background
/// OpenVR connection at most once a minute while SteamVR runs (one that never keeps SteamVR from
/// quitting, with longer waits after failed inits); each layout SteamVR settles on is kept
/// (`align::Layouts`), to place the stations no registration measured.
fn universe_watch(d: Arc<Daemon>) {
    let mut first = true;
    let mut warned_disagree = false;
    let mut warned_log = false;
    let mut warned_layouts = false;
    let mut connection = UniverseConnection::default();
    let mut steamvr_log = cv1touch::steamvr_log::Watch::new(&cv1touch::steamvr_log::server_log());
    let layouts_path = align::layouts_path();
    let modified = |p: &std::path::Path| fs::metadata(p).and_then(|m| m.modified()).ok();
    let mut layouts_at = modified(&layouts_path);
    let mut layouts = align::load_layouts(&layouts_path);
    let mut settling = align::Settling::default();
    let mut following: Option<align::Follow> = None;
    // since when the base station is placed but SteamVR does not report it, and whether it was said
    let mut held: Option<(std::time::Instant, bool)> = None;
    let say = |text: String| *d.universe_following.lock().unwrap() = text;
    while !d.quit.load(Ordering::Acquire) {
        sleep_until(&d, if first { 5_000 } else { UNIVERSE_TICK_MS }, || {
            d.universe_recheck.swap(false, Ordering::AcqRel)
        });
        first = false;
        if !d.universe_watch_on.load(Ordering::Acquire) {
            say(String::new());
            continue;
        }
        match steamvr_log.poll() {
            Ok(_) => warned_log = false,
            Err(e) => {
                if !warned_log {
                    warned_log = true;
                    log!(
                        "universe watch: SteamVR's log {}: {e}",
                        steamvr_log.path().display()
                    );
                }
            }
        }
        let Ok(reg) = align::load_registration(&align::registration_path()) else {
            continue;
        };
        if reg.stations.is_empty() {
            continue;
        }
        // the station SteamVR tracks the headset from, or last tracked it from in this session
        let base = steamvr_log
            .bases
            .last_of_headset(&reg.reference)
            .map(str::to_string);
        // what a registration (or another daemon) added to the layouts
        let at = modified(&layouts_path);
        if at != layouts_at {
            layouts_at = at;
            layouts = align::load_layouts(&layouts_path);
        }
        // the stations as the driver sees them inside SteamVR; else (an older driver) a short
        // SteamVR connection of our own, at most once a minute: each costs vrserver an app connect
        let reported = d.driver_stations.lock().unwrap().clone();
        let now = connection.stations(
            std::time::Instant::now(),
            shm::now_ns(),
            reported,
            || platform::process::running_without_child("vrserver"),
            || cv1touch::vr::Client::connect().map(|client| client.stations()),
            log,
        );
        *d.universe_connection.lock().unwrap() = connection;
        let Some(now) = now else {
            if connection.state != UniverseConnectionState::Connected {
                say(String::new());
            }
            continue;
        };
        if now.is_empty() {
            continue;
        }
        let seen = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_secs());
        if let Some(settled) = settling.see(&now, seen) {
            match align::record_layout(&layouts_path, reg.universe_id, seen, &settled) {
                Ok((_, on_file)) => {
                    warned_layouts = false;
                    layouts = on_file;
                    layouts_at = modified(&layouts_path);
                }
                Err(e) if !warned_layouts => {
                    warned_layouts = true;
                    log!("universe watch: {e} (the layout is not kept)");
                }
                Err(_) => {}
            }
        }
        let Some(how) = reg.follow_now(base.as_deref(), &now, &layouts.of(reg.universe_id)) else {
            match &base {
                // the base has no place in the registration and the stations do not agree
                Some(b) if now.iter().any(|(s, _)| s == b) => {
                    if !warned_disagree {
                        warned_disagree = true;
                        log!("universe watch: SteamVR tracks the headset from base station {b}, which the registration has no place for, and the base stations disagree on how the universe moved; registration left as it is");
                    }
                }
                // a base SteamVR does not report for a moment: left as it is
                Some(b) => {
                    let (since, said) = held.get_or_insert((std::time::Instant::now(), false));
                    if !*said && since.elapsed() > Duration::from_secs(10) {
                        *said = true;
                        log!("universe watch: SteamVR has not reported base station {b}, which it tracks the headset from, for 10 s; registration left as it is");
                    }
                }
                None if !warned_disagree => {
                    warned_disagree = true;
                    log!("universe watch: the base stations disagree on how the universe moved (one bumped?), and SteamVR's log names no station it tracks the headset from; registration left as it is");
                }
                None => {}
            }
            continue;
        };
        if held.take().is_some_and(|(_, said)| said) {
            log!("universe watch: SteamVR reports the base station again");
        }
        warned_disagree = false;
        let s = how.now_from_then();
        let s = if s.translation.vector.norm() < 5e-4 && s.rotation.angle() < 0.01f64.to_radians() {
            nalgebra::Isometry3::identity()
        } else {
            s
        };
        let prev = d
            .universe_shift
            .lock()
            .unwrap()
            .filter(|p| p.0 == reg.created_unix_s)
            .map(|p| p.1);
        let moved = prev.is_none_or(|p| {
            (p.inverse() * s).translation.vector.norm() > 3e-4
                || (p.inverse() * s).rotation.angle() > 0.01f64.to_radians()
        });
        let changed_how = following.as_ref().is_none_or(|f| !f.same_way(&how));
        say(how.to_string());
        if moved {
            *d.universe_shift.lock().unwrap() = Some((reg.created_unix_s, s));
            let (alignment, _) = align::load_with(Some(&s));
            publish_alignment(&d, alignment);
        }
        if (moved
            && (s != nalgebra::Isometry3::identity()
                || prev.is_some_and(|p| p != nalgebra::Isometry3::identity())))
            || changed_how
        {
            log!(
                "universe watch: registration follows {how}: {:.1} mm, {:.2} deg in the play area since it was fitted",
                (s * reg.play_area_point() - reg.play_area_point()).norm() * 1e3,
                s.rotation.angle().to_degrees(),
            );
        }
        following = Some(how);
    }
}

fn sleep_interruptible(d: &Daemon, ms: u64) {
    sleep_until(d, ms, || false);
}

/// Sleeps `ms`, or until quitting or `wake` says so.
fn sleep_until(d: &Daemon, ms: u64, wake: impl Fn() -> bool) {
    for _ in 0..ms / 50 {
        if d.quit.load(Ordering::Acquire) || wake() {
            return;
        }
        thread::sleep(Duration::from_millis(50));
    }
}

/// The only thread that touches OpenHMD.
fn engine_main(d: Arc<Daemon>) {
    let mut last_err = String::new();
    while !d.quit.load(Ordering::Acquire) {
        let (opts, revision) = requested_engine_options(&d);
        let e = match Engine::open(&opts, d.clone()) {
            Ok(e) => e,
            Err(err) => {
                if err != last_err {
                    log!("waiting for headset: {err}");
                    last_err = err;
                }
                publish_disconnected(&d);
                sleep_interruptible(&d, 1000);
                continue;
            }
        };
        last_err.clear();
        watch_worker_resources(&d.failed_worker, ENGINE_WORKER, true, e, |e| {
            engine_session(&d, &opts, revision, e);
        });
        publish_disconnected(&d);
    }
}

/// A decoder is needed before HID reads can be useful: keep the fallback from starting I/O
/// that it would immediately have to cancel (Windows cancellation can wait indefinitely).
fn radio_resources<D, H>(
    decoder: impl FnOnce() -> Option<D>,
    open: impl FnOnce() -> Option<H>,
) -> Option<(D, H)> {
    let decoder = decoder()?;
    let device = open()?;
    Some((decoder, device))
}

/// One open engine, including setup and the radio scope, borrows its owner's native resources.
fn engine_session(d: &Daemon, opts: &EngineOptions, revision: u64, e: &mut Engine) {
    d.engine_applied.store(revision, Ordering::Release);
    d.session.lock().unwrap().reset(); // sensors may differ after a reopen
    d.sensor_poses.lock().unwrap().clear();
    d.sensor_agreement.lock().unwrap().clear();
    d.sensors_unsure.store(false, Ordering::Relaxed);
    log!("engine running with {} sensor(s)", e.sensors().len());
    make_usb_thread_realtime();
    let open: Vec<String> = e.sensors().iter().map(engine::sensor_serial).collect();
    d.track_in.lock().unwrap().register(open.iter().cloned());
    for serial in modes::unposed_sensors(opts, &open) {
        log!("sensor {serial} has no calibrated pose: it only observes (run `cv1touch calibrate`)");
    }
    let link = RadioLink::default();
    // the radio thread holds a decoder pointing into the engine: the scope joins it first
    engine_scope(&d.failed_worker, |scope| {
        let stop = StopRadio { d, link: &link };
        let radio = new_pipeline(d, opts, e).and_then(|pipe| {
            let Some(path) = hid::headset_interface(1) else {
                log("no HID interface for the headset's radio: publishing OpenHMD poses");
                return None;
            };
            // SAFETY: the decoder goes to the scoped radio thread, joined before the engine closes
            let (dec, file) = radio_resources(|| unsafe { e.radio_decoder() }, || {
                match hid::Device::open(&path) {
                    Ok(f) => Some(f),
                    Err(err) => {
                        log!("cannot read the radio at {}: {err}; publishing OpenHMD poses", path.display());
                        None
                    }
                }
            })?;
            // Opening the HID handle already starts a Windows read: mark a startup panic before
            // its local handle can wait for cancellation while unwinding.
            let _startup = WorkerExit { failed: &d.failed_worker, worker: ENGINE_WORKER, panic_only: true };
            log!("publishing cv1touch tracker poses from the radio thread ({}; OpenHMD acquires only)", path.display());
            let watch = LandmarkWatch::load();
            let (d, link) = (d, &link);
            let job = WorkerResources::starting(&d.failed_worker, ENGINE_WORKER, (dec, file, pipe, watch));
            match thread::Builder::new().name("cv1t-radio".into()).spawn_scoped(scope, move || {
                job.run_as(RADIO_WORKER, true, |(dec, file, pipe, watch)| {
                    radio_main(d, link, dec, file, pipe, watch);
                });
            }) {
                Ok(radio) => Some(radio),
                Err(err) => panic!("could not start the radio thread: {err}"),
            }
        });
        engine_loop(d, e, &link, radio.is_some());
        drop(stop);
        if let Some(Err(_)) = radio.map(|r| r.join()) {
            log("radio thread panicked");
        }
    });
    let biases = link.biases.lock().unwrap().take();
    if let Some(f) = biases {
        save_biases(&f);
    }
}

/// Mark startup and loop panics before the scope can wait for a radio thread to finish.
fn engine_scope<'env>(
    failed: &AtomicU8,
    run: impl for<'scope> FnOnce(&'scope thread::Scope<'scope, 'env>),
) {
    thread::scope(|scope| {
        watch_worker(failed, ENGINE_WORKER, true, || run(scope));
    });
}

/// How long `test-inject on` arms input injection: it disarms itself, so a forgotten test leaves
/// nothing switched on.
const INJECT_ARMED_NS: u64 = 600_000_000_000;

/// Input injection for automated tests (`test-inject`, `inject`): off by default, armed for
/// `INJECT_ARMED_NS` at a time. Whichever thread publishes a hand ORs what is pressed into its
/// state (`publish_hand_state`), so each shm slot keeps its one writer; nothing outlives the
/// arming or the daemon.
#[derive(Default)]
struct Injection {
    /// When the arming runs out (shm clock; 0: disarmed); checked without the lock on every publish.
    armed_until: std::sync::atomic::AtomicU64,
    /// Per hand: what is pressed, and until when.
    pressed: Mutex<[(control::Injected, u64); 2]>,
}

impl Injection {
    fn arm(&self, now: u64) {
        *self.pressed.lock().unwrap() = Default::default();
        self.armed_until
            .store(now + INJECT_ARMED_NS, Ordering::Release);
    }

    fn disarm(&self) {
        self.armed_until.store(0, Ordering::Release);
        *self.pressed.lock().unwrap() = Default::default();
    }

    /// How long it stays armed, ns; None when disarmed.
    fn armed_for(&self, now: u64) -> Option<u64> {
        let until = self.armed_until.load(Ordering::Acquire);
        (now < until).then(|| until - now)
    }

    fn press(
        &self,
        hand: usize,
        inputs: control::Injected,
        ms: u32,
        now: u64,
    ) -> Result<(), String> {
        if self.armed_for(now).is_none() {
            return Err("test-inject is off (cv1touch test-inject on)".into());
        }
        self.pressed.lock().unwrap()[hand] = (inputs, now + ms as u64 * 1_000_000);
        Ok(())
    }

    fn apply(&self, hand: usize, now: u64, st: &mut ControllerState) {
        if now >= self.armed_until.load(Ordering::Acquire) {
            return;
        }
        let (inputs, until) = self.pressed.lock().unwrap()[hand];
        if now >= until {
            return;
        }
        st.buttons |= inputs.buttons;
        st.touches |= inputs.touches;
        if inputs.trigger {
            st.trigger = 1.0;
        }
        if inputs.grip {
            st.grip = 1.0;
        }
    }
}

/// Marks a critical worker's failure without touching any state its panic could have poisoned.
struct WorkerExit<'a> {
    failed: &'a AtomicU8,
    worker: u8,
    panic_only: bool,
}

impl Drop for WorkerExit<'_> {
    fn drop(&mut self) {
        if !self.panic_only || thread::panicking() {
            let _ =
                self.failed
                    .compare_exchange(0, self.worker, Ordering::AcqRel, Ordering::Acquire);
        }
    }
}

fn watch_worker(failed: &AtomicU8, worker: u8, panic_only: bool, run: impl FnOnce()) {
    watch_worker_resources(failed, worker, panic_only, (), |()| run());
}

/// The marker is the first field: an abandoned spawn drops it before native resources, even
/// when no panic has begun. Once running, only a panic (or an unexpected engine exit) is fatal.
struct WorkerResources<'a, R> {
    exit: WorkerExit<'a>,
    resources: R,
}

impl<'a, R> WorkerResources<'a, R> {
    fn starting(failed: &'a AtomicU8, worker: u8, resources: R) -> Self {
        Self {
            exit: WorkerExit {
                failed,
                worker,
                panic_only: false,
            },
            resources,
        }
    }

    fn run_as(mut self, worker: u8, panic_only: bool, run: impl FnOnce(&mut R)) {
        self.exit.worker = worker;
        self.exit.panic_only = panic_only;
        run(&mut self.resources);
    }
}

/// Callbacks borrow native resources so failure is marked before their teardown can wait.
fn watch_worker_resources<R>(
    failed: &AtomicU8,
    worker: u8,
    panic_only: bool,
    resources: R,
    run: impl FnOnce(&mut R),
) {
    WorkerResources::starting(failed, worker, resources).run_as(worker, panic_only, run);
}

/// A failed tracker cannot be recovered safely by joining through its possibly poisoned locks.
/// Exit directly so the supervisor can start with fresh state and USB handles.
fn exit_on_worker_failure(failed: &AtomicU8, shared: &ShmWriter) {
    let name = match failed.load(Ordering::Acquire) {
        ENGINE_WORKER => "tracking engine thread",
        RADIO_WORKER => "radio thread",
        _ => return,
    };
    log!("{name} stopped unexpectedly; exiting so the daemon can restart");
    shared.retire();
    daemon_log::exit(1);
}

/// Stops the radio thread when the engine loop ends, a panic included (the scope would wait on
/// the thread forever otherwise).
struct StopRadio<'a> {
    d: &'a Daemon,
    link: &'a RadioLink,
}

impl Drop for StopRadio<'_> {
    fn drop(&mut self) {
        self.link.stop.store(true, Ordering::Release);
        self.d.wake_radio();
    }
}

/// What the engine thread and the radio thread share for one open engine.
#[derive(Default)]
struct RadioLink {
    stop: AtomicBool,
    /// The engine's newest view of each controller (calibration, connection, OpenHMD's pose):
    /// the radio thread's samples go out over it.
    engine_view: Mutex<[Option<engine::Controller>; 2]>,
    /// Which thread publishes each hand.
    hands: HandOwnership,
    /// Bodies the engine found for the radio thread to add.
    bodies: Mutex<Vec<NewBody>>,
    /// The bias file's new contents from the radio thread, for the engine thread to write.
    biases: Mutex<Option<track::biases::BiasFile>>,
    /// Sensor pose changes from the radio thread, for the engine thread to give the engine (and
    /// a sensor placed again to write into the calibration).
    reposed: Mutex<Vec<PoseChange>>,
}

/// The calibration file the daemon's sensor poses came from.
#[derive(Clone)]
struct LoadedCalibration {
    path: PathBuf,
    sha256: String,
}

impl LoadedCalibration {
    /// The file at `path`, fingerprinted; a file that cannot be read is remembered without one,
    /// so that it is never written over.
    fn at(path: PathBuf) -> LoadedCalibration {
        let sha256 = setup::fingerprint(&path).unwrap_or_default();
        LoadedCalibration { path, sha256 }
    }
}

/// How many `.bak-*` copies of the calibration file are kept beside it, newest first.
const KEPT_BACKUPS: usize = 10;
/// A probed sensor that gathers no labelled sighting of a controller in this long is let go: it
/// was not it that moved, or nothing was in its view.
const PROBATION_NS: u64 = 30_000_000_000;
/// A sensor placed again this many times in `PLACEMENTS_WINDOW_NS` is given up on.
const MAX_PLACEMENTS: usize = 3;
const PLACEMENTS_WINDOW_NS: u64 = 1_800_000_000_000;

fn prune_placements(placements: &mut std::collections::BTreeMap<String, Vec<u64>>, now: u64) {
    placements.retain(|_, times| {
        times.retain(|t| now.saturating_sub(*t) <= PLACEMENTS_WINDOW_NS);
        !times.is_empty()
    });
}
/// Releases of a sensor more than this far apart do not count as a run.
const RELEASE_RUN_NS: u64 = 3_600_000_000_000;

/// What the radio thread decided about a sensor's pose in the engine.
enum PoseChange {
    /// The sensor moved: the engine should label its blobs by the long search while it is being
    /// placed, which a sensor without a pose does.
    Cleared(String),
    /// The sensor's pose, placed again from the others (or given back, as calibrated, when it
    /// could not be placed: the engine stops its search then).
    Set(Reposed),
}

/// A sensor placed again from the others (`repose`): its new pose, and how far it is from the
/// calibrated one; `write` is false for a pose given back as it was.
struct Reposed {
    serial: String,
    orient_xyzw: [f32; 4],
    pos: [f32; 3],
    median_px: f64,
    matches: usize,
    moved_mm: f64,
    turned_deg: f64,
    write: bool,
}

/// Where a sensor stands with the pose watch, for `status`.
#[derive(Debug, Clone, serde::Serialize)]
#[serde(tag = "state", rename_all = "snake_case")]
enum PoseNote {
    /// It moved: its frames are probed, not tracked with, until it is placed again.
    Moved { since_ns: u64, attempts: u32 },
    Placed {
        at_ns: u64,
        moved_mm: f64,
        turned_deg: f64,
        median_px: f64,
        matches: usize,
    },
    /// No pose could be found for it from the others: it stays out of tracking.
    GivenUp {
        since_ns: u64,
        attempts: u32,
        reason: String,
    },
    /// Judged moved, then let go: a false alarm (its pose was placed where it was calibrated),
    /// or no pose came in time; tracked with again, as calibrated.
    Released { at_ns: u64, why: String },
}

/// How often the radio thread judges the sensors' agreement and tends the placing.
const REPOSE_TICK_NS: u64 = 1_000_000_000;
/// "Unsure" is logged at most this often.
const UNSURE_LOG_NS: u64 = 60_000_000_000;

/// A batch for the solver thread: the probes, the sensor's camera model and the controllers' LEDs.
struct SolveJob {
    job: repose::Job,
    model: calib::camera::CameraModel,
    bodies: [Option<Vec<calib::capture::Led>>; 2],
}

/// The radio thread's side of placing moved sensors again: judges the agreement every
/// `REPOSE_TICK_NS`, probes a sensor judged moved, gathers its probes and hands batches to a
/// solver thread, and when a pose is confirmed tracks with it at once and tells the engine thread.
struct Reposing {
    monitor: repose::Monitor,
    reposer: repose::Reposer,
    to_solver: std::sync::mpsc::Sender<SolveJob>,
    from_solver: std::sync::mpsc::Receiver<(repose::Job, Result<repose::Solved, String>)>,
    next_tick: u64,
    unsure_until: u64,
    unsure_logged: u64,
    /// When each probed sensor was handed over (tick clock).
    probed_since: std::collections::BTreeMap<String, u64>,
    /// When each sensor was placed again (tick clock), newest last.
    placements: std::collections::BTreeMap<String, Vec<u64>>,
    /// Per sensor: releases in a row (false alarms, timeouts) and when the last was; each one
    /// doubles the cooldown, a placement or a quiet hour ends the run.
    releases: std::collections::BTreeMap<String, (u32, u64)>,
}

impl Reposing {
    fn start() -> Reposing {
        let (to_solver, jobs) = std::sync::mpsc::channel::<SolveJob>();
        let (results, from_solver) = std::sync::mpsc::channel();
        // the solver runs for up to seconds at a time (the placing searches 2000 six-point fits),
        // off the real-time thread; it ends with the radio thread, which holds the only sender
        let spawned = std::thread::Builder::new()
            .name("cv1t-repose".into())
            .spawn(move || {
                for w in jobs {
                    let bodies = [w.bodies[0].as_deref(), w.bodies[1].as_deref()];
                    let result = repose::solve(&w.model, bodies, &w.job);
                    if results.send((w.job, result)).is_err() {
                        break;
                    }
                }
            });
        if let Err(e) = spawned {
            log!("no solver thread for moved sensors ({e})");
        }
        Reposing {
            monitor: repose::Monitor::new(repose::WINDOW_NS),
            reposer: repose::Reposer::default(),
            to_solver,
            from_solver,
            next_tick: 0,
            unsure_until: 0,
            unsure_logged: 0,
            probed_since: Default::default(),
            placements: Default::default(),
            releases: Default::default(),
        }
    }

    /// Lets a probed sensor go, tracked with again as calibrated: the engine gets its pose back,
    /// the monitor leaves it alone for a while, and `status` says `why`.
    fn release(
        &mut self,
        d: &Daemon,
        link: &RadioLink,
        pipe: &mut Pipeline,
        serial: &str,
        now: u64,
        why: &str,
    ) {
        if let Some(camera) = pipe.camera_id(serial) {
            self.reposer.forget(camera);
        }
        pipe.probe(serial, false);
        pipe.reset_agreement(serial);
        self.probed_since.remove(serial);
        let run = self.releases.entry(serial.to_string()).or_insert((0, now));
        if now.saturating_sub(run.1) > RELEASE_RUN_NS {
            run.0 = 0;
        }
        run.0 += 1;
        run.1 = now;
        let cooldown = repose::COOLDOWN_NS << (run.0 - 1).min(4);
        self.monitor.released_until(serial, now + cooldown);
        if let Some(cam) = pipe.camera(serial) {
            let (orient_xyzw, pos) = calib::iso_to_xyzw(&cam.world_from_cam);
            link.reposed.lock().unwrap().push(PoseChange::Set(Reposed {
                serial: serial.to_string(),
                orient_xyzw,
                pos,
                median_px: 0.0,
                matches: 0,
                moved_mm: 0.0,
                turned_deg: 0.0,
                write: false,
            }));
        }
        d.sensor_poses.lock().unwrap().insert(
            serial.to_string(),
            PoseNote::Released {
                at_ns: now,
                why: why.to_string(),
            },
        );
        log!("sensor {serial}: {why}; tracking with it again as calibrated");
    }

    fn tick(
        &mut self,
        d: &Daemon,
        link: &RadioLink,
        pipe: &mut Pipeline,
        watch: &mut Option<LandmarkWatch>,
        now: u64,
    ) {
        if now < self.next_tick {
            return;
        }
        self.next_tick = now + REPOSE_TICK_NS;
        prune_placements(&mut self.placements, now);
        while let Ok((job, result)) = self.from_solver.try_recv() {
            let camera = job.camera;
            let Some(serial) = pipe.serial_of(camera).map(str::to_string) else {
                continue;
            };
            match self.reposer.solved(job, result) {
                repose::Outcome::Nothing => {
                    if let Some(PoseNote::Moved { attempts, .. }) =
                        d.sensor_poses.lock().unwrap().get_mut(&serial)
                    {
                        *attempts += 1;
                    }
                }
                repose::Outcome::Placed { fit, .. } => {
                    let Some(cam) = pipe.camera(&serial).copied() else {
                        continue;
                    };
                    if repose::false_alarm(&cam.world_from_cam, &fit.world_from_cam) {
                        self.release(
                            d,
                            link,
                            pipe,
                            &serial,
                            now,
                            "a false alarm: placed again, it stands where it was calibrated",
                        );
                        continue;
                    }
                    let delta = cam.world_from_cam.inverse() * fit.world_from_cam;
                    let moved_mm = delta.translation.vector.norm() * 1e3;
                    let turned_deg = delta.rotation.angle().to_degrees();
                    self.placements.entry(serial.clone()).or_default().push(now);
                    self.releases.remove(&serial);
                    pipe.sensor(&serial, cam.model, fit.world_from_cam);
                    pipe.probe(&serial, false);
                    pipe.reset_agreement(&serial);
                    self.reposer.forget(camera);
                    self.probed_since.remove(&serial);
                    self.monitor.released(&serial);
                    if let Some(w) = watch.as_mut() {
                        w.forget(d, &serial);
                    }
                    let (orient_xyzw, pos) = calib::iso_to_xyzw(&fit.world_from_cam);
                    link.reposed.lock().unwrap().push(PoseChange::Set(Reposed {
                        serial: serial.clone(),
                        orient_xyzw,
                        pos,
                        median_px: fit.median_px,
                        matches: fit.pairs,
                        moved_mm,
                        turned_deg,
                        write: true,
                    }));
                    d.sensor_poses.lock().unwrap().insert(
                        serial.clone(),
                        PoseNote::Placed {
                            at_ns: now,
                            moved_mm,
                            turned_deg,
                            median_px: fit.median_px,
                            matches: fit.pairs,
                        },
                    );
                    log!(
                        "sensor {serial}: placed again from the other sensors, {moved_mm:.0} mm and {turned_deg:.1} degrees from its calibrated pose ({:.2} px median over {} LED matches in {} frames); tracking with it again",
                        fit.median_px, fit.pairs, fit.frames
                    );
                }
                repose::Outcome::GaveUp {
                    attempts, reason, ..
                } => {
                    // the engine gets its calibrated pose back, so that it stops searching every
                    // frame; our tracker keeps the sensor out, and its frames are dropped
                    pipe.ignore(&serial, true);
                    self.probed_since.remove(&serial);
                    if let Some(cam) = pipe.camera(&serial) {
                        let (orient_xyzw, pos) = calib::iso_to_xyzw(&cam.world_from_cam);
                        link.reposed.lock().unwrap().push(PoseChange::Set(Reposed {
                            serial: serial.clone(),
                            orient_xyzw,
                            pos,
                            median_px: 0.0,
                            matches: 0,
                            moved_mm: 0.0,
                            turned_deg: 0.0,
                            write: false,
                        }));
                    }
                    d.sensor_poses.lock().unwrap().insert(
                        serial.clone(),
                        PoseNote::GivenUp {
                            since_ns: now,
                            attempts,
                            reason: reason.clone(),
                        },
                    );
                    log!(
                        "sensor {serial}: no pose for it from the other sensors after {attempts} tries ({reason}); it stays out of tracking until cv1touch setup or a restart"
                    );
                }
            }
        }
        let agreement = pipe.camera_agreement();
        for change in self.monitor.update(now, &agreement) {
            match change {
                repose::Change::Moved(serial) => {
                    let Some(camera) = pipe.camera_id(&serial) else {
                        continue;
                    };
                    let recent = self.placements.get(&serial).map_or(0, Vec::len);
                    if recent >= MAX_PLACEMENTS {
                        // it keeps moving, or something else is wrong: not again and again
                        pipe.probe(&serial, true);
                        pipe.ignore(&serial, true);
                        d.sensor_poses.lock().unwrap().insert(
                            serial.clone(),
                            PoseNote::GivenUp {
                                since_ns: now,
                                attempts: 0,
                                reason: format!("placed again {recent} times in half an hour"),
                            },
                        );
                        log!(
                            "sensor {serial}: judged moved again, after {recent} placements in half an hour: it stays out of tracking until cv1touch setup or a restart"
                        );
                        continue;
                    }
                    pipe.probe(&serial, true);
                    self.reposer.watch(camera);
                    self.probed_since.insert(serial.clone(), now);
                    link.reposed
                        .lock()
                        .unwrap()
                        .push(PoseChange::Cleared(serial.clone()));
                    d.sensor_poses.lock().unwrap().insert(
                        serial.clone(),
                        PoseNote::Moved {
                            since_ns: now,
                            attempts: 0,
                        },
                    );
                    let a = agreement
                        .iter()
                        .find(|(s, _)| *s == serial)
                        .map(|(_, a)| *a)
                        .unwrap_or_default();
                    let matched = if a.samples == 0 {
                        "no LED matched".to_string()
                    } else {
                        format!(
                            "residual {:.2} px, offset {:.2} px",
                            a.median_px,
                            a.bias_px.norm()
                        )
                    };
                    log!(
                        "sensor {serial}: disagrees with the other sensors (missed {} of {} checks; {matched}): it moved; placing it again from them while the controllers are in its view",
                        a.misses, a.checks
                    );
                }
                repose::Change::Unsure => {
                    self.unsure_until = now + 2 * repose::WINDOW_NS;
                    if now.saturating_sub(self.unsure_logged) >= UNSURE_LOG_NS {
                        self.unsure_logged = now;
                        log!("fewer than two sensors agree with each other: sensors moved, but which cannot be told from here; if tracking is off, run cv1touch setup");
                    }
                }
            }
        }
        d.sensors_unsure
            .store(now < self.unsure_until, Ordering::Relaxed);
        *d.sensor_agreement.lock().unwrap() = agreement.into_iter().collect();
        // a probed sensor that gathers no labelled sighting in `PROBATION_NS`, or whose probes
        // bring no pose in `PATIENCE_NS`, is let go, so that the engine's search on it ends
        let waited_out: Vec<(String, &str)> = self
            .probed_since
            .iter()
            .filter_map(|(serial, since)| {
                let waited = now.saturating_sub(*since);
                let gathered = pipe
                    .camera_id(serial)
                    .is_some_and(|c| self.reposer.gathered(c) > 0);
                if !gathered && waited >= PROBATION_NS {
                    Some((serial.clone(), "no labelled sighting of a controller in it for 30 s: it was not it that moved, or nothing was in its view"))
                } else if waited >= repose::PATIENCE_NS {
                    Some((serial.clone(), "no pose for it in 3 minutes (too little controller motion in its view, or no pose fits)"))
                } else {
                    None
                }
            })
            .collect();
        for (serial, why) in waited_out {
            self.release(d, link, pipe, &serial, now, why);
        }
        for probe in std::mem::take(&mut pipe.probes) {
            self.reposer.add(probe);
        }
        if let Some(job) = self.reposer.batch() {
            let model = pipe
                .serial_of(job.camera)
                .and_then(|serial| pipe.camera(serial))
                .map(|c| c.model);
            let Some(model) = model else {
                return;
            };
            let bodies =
                [HAND_LEFT, HAND_RIGHT].map(|h| pipe.tracker(h).map(|t| t.body.leds.clone()));
            if self
                .to_solver
                .send(SolveJob { job, model, bodies })
                .is_err()
            {
                log!("the solver thread for moved sensors is gone");
            }
        }
    }
}

/// The engine thread's side: the engine tracks with the new pose, the options keep it for a
/// reopen, the calibration file records it (with a backup of the old), and the sensor's
/// landmarks, learned where it stood, are dropped.
fn apply_reposed(d: &Daemon, e: &Engine, r: &Reposed) {
    if let Err(err) = e.set_sensor_pose(
        &r.serial,
        r.orient_xyzw.map(f64::from),
        r.pos.map(f64::from),
    ) {
        log!(
            "sensor {}: the engine did not take the pose ({err}); it has what it had",
            r.serial
        );
    }
    if !r.write {
        return;
    }
    adopt_reposed_pose(&mut d.opts.lock().unwrap(), r);
    let loaded = d.calibration.lock().unwrap().clone();
    let written = match loaded {
        Some(loaded) => match write_reposed(&loaded, r, &platform::dirs::get().state) {
            Ok((backup, sha256)) => {
                log!(
                    "sensor {}: wrote its new pose to {}{}",
                    r.serial,
                    loaded.path.display(),
                    backup
                        .as_ref()
                        .map_or(String::new(), |b| format!(" (previous: {})", b.display()))
                );
                if let Some(c) = d.calibration.lock().unwrap().as_mut() {
                    c.sha256 = sha256;
                }
                if let Some(backup) = backup {
                    thin_own_backups(&mut d.own_backups.lock().unwrap(), backup);
                }
                true
            }
            Err(err) => {
                log!("sensor {}: new pose not written: {err}", r.serial);
                false
            }
        },
        None => {
            log!(
                "sensor {}: new pose not written: no calibration file is loaded",
                r.serial
            );
            false
        }
    };
    if !written {
        // the old pose comes back at the next start: its landmarks stay with it
        return;
    }
    let landmarks = track::landmarks::default_path();
    match track::landmarks::forget_in_file(&landmarks, &r.serial) {
        Ok(0) => {}
        Ok(n) => log!(
            "sensor {}: dropped its {n} landmark(s) from {}, learned where it stood; cv1touch setup learns new ones",
            r.serial,
            landmarks.display()
        ),
        Err(err) => log!("sensor {}: {err}", r.serial),
    }
}

/// Keeps a sensor's new pose in the options a reopened engine starts with. The calibration's
/// own spelling of the serial stays, in whichever letter case it is.
fn adopt_reposed_pose(opts: &mut EngineOptions, r: &Reposed) {
    if let Some(p) = opts
        .fixed_poses
        .iter_mut()
        .find(|p| cv1touch::status::same_sensor(&p.serial, &r.serial))
    {
        p.orient = r.orient_xyzw;
        p.pos = r.pos;
    }
}

/// Writes the sensor's new pose into the loaded calibration file: the backup of the old file and
/// the new file's fingerprint. Not while `cv1touch setup` runs, and not over a file changed
/// since the daemon loaded it. Setup's stages bound to the old file are bound to the new one, so
/// that they stay as they were, and old backups are thinned to `KEPT_BACKUPS`.
fn write_reposed(
    loaded: &LoadedCalibration,
    r: &Reposed,
    state: &std::path::Path,
) -> Result<(Option<PathBuf>, String), String> {
    let path = &loaded.path;
    // held until the journal is written too, so that setup cannot start in between
    let _setup_kept_out = match control::lock_single_instance(&state.join("setup.lock")) {
        Ok(lock) => Some(lock),
        Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
            return Err("cv1touch setup is running: it will write its own calibration".into());
        }
        Err(_) => None,
    };
    let current = setup::fingerprint(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if current != loaded.sha256 {
        return Err(format!(
            "{} changed since the daemon loaded it: restart cv1touchd, or run `cv1touch calibration load`, to use it as it is",
            path.display()
        ));
    }
    let mut cal = calib::file::load(path)?;
    let entry = cal
        .sensor_mut(&r.serial)
        .ok_or_else(|| format!("{} has no sensor {}", path.display(), r.serial))?;
    entry.orient_xyzw = r.orient_xyzw;
    entry.pos = r.pos;
    entry.rms_px = r.median_px;
    entry.observations = r.matches;
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    cal.source = note_in_source(
        &cal.source,
        &format!(
            "{} placed again by cv1touchd from the other sensors ({:.0} mm, {:.1} deg) at unix {now}",
            r.serial, r.moved_mm, r.turned_deg
        ),
    );
    let (backup, sha256) = calib::file::save_with_fingerprint(path, &cal)?;
    let journal = state.join("setup.json");
    if let Ok(Some(mut checkpoint)) = setup::load(&journal) {
        if setup::carry_camera_fingerprint(&mut checkpoint, &loaded.sha256, &sha256) {
            match setup::save(&journal, &checkpoint) {
                Ok(()) => log!("setup's stages stay as they were: they are bound to the new file"),
                Err(e) => log!("{}: {e}", journal.display()),
            }
        }
    }
    Ok((backup, sha256))
}

/// `source` with `note` appended; a source grown past a page keeps its beginning and its end.
fn note_in_source(source: &str, note: &str) -> String {
    const KEEP: usize = 1500;
    let joined = format!("{source}; {note}");
    if joined.len() <= KEEP {
        return joined;
    }
    let head: String = joined.chars().take(300).collect();
    let tail: String = joined
        .chars()
        .rev()
        .take(KEEP - 300 - 7)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .collect();
    format!("{head} [...] {tail}")
}

/// Remembers a backup the daemon's own write made, and removes the daemon's oldest ones beyond
/// `KEPT_BACKUPS`; backups other tools made (setup, calibrate) are never touched.
fn thin_own_backups(own: &mut Vec<PathBuf>, backup: PathBuf) {
    own.push(backup);
    while own.len() > KEPT_BACKUPS {
        let oldest = own.remove(0);
        match std::fs::remove_file(&oldest) {
            Ok(()) => log!(
                "removed the daemon's oldest backup {}, keeping {KEPT_BACKUPS}",
                oldest.display()
            ),
            Err(e) if e.kind() == io::ErrorKind::NotFound => {}
            Err(e) => log!("{}: {e}", oldest.display()),
        }
    }
}

struct NewBody {
    hand: usize,
    device_id: i32,
    leds: Vec<calib::capture::Led>,
    imu: nalgebra::Vector3<f64>,
}

/// Which thread writes a hand's shm slot, which takes one writer at a time: the engine (OpenHMD's
/// pose) until the radio thread has the hand's tracker body and the engine, between two of its own
/// publishes, lets go; the radio thread from then on; the engine again once the radio thread has
/// exited, after its last publish.
#[derive(Default)]
struct HandOwnership {
    wanted: [AtomicBool; 2],
    radio: [AtomicBool; 2],
    radio_gone: AtomicBool,
}

impl HandOwnership {
    /// Radio thread: it has the hand's body and asks for the hand.
    fn request(&self, hand: usize) {
        self.wanted[hand].store(true, Ordering::Release);
    }

    /// Radio thread: whether it may publish the hand.
    fn radio_publishes(&self, hand: usize) -> bool {
        self.radio[hand].load(Ordering::Acquire)
    }

    /// Radio thread, after its last publish: every hand goes back to the engine.
    fn radio_exit(&self) {
        self.radio_gone.store(true, Ordering::Release);
    }

    /// Engine thread, before each publish of the hand: whether it still publishes it, letting go
    /// first if the radio thread asked.
    fn engine_publishes(&self, hand: usize) -> bool {
        if self.radio_gone.load(Ordering::Acquire) {
            return true;
        }
        if self.radio[hand].load(Ordering::Relaxed) {
            return false;
        }
        if self.wanted[hand].load(Ordering::Acquire) {
            self.radio[hand].store(true, Ordering::Release);
            return false;
        }
        true
    }
}

/// The engine thread's loop for one open engine: OpenHMD's update (its own filter, the acquisition
/// the tracker relies on, the headset, haptics), each controller's engine view for the radio
/// thread, and the hands the radio thread does not publish (no tracker body, or no radio thread),
/// from OpenHMD. Returns when the engine is to close.
fn engine_loop(d: &Daemon, e: &mut Engine, link: &RadioLink, radio: bool) {
    let mut hands = [
        Hand::new(HAND_LEFT, HandConfig::default()),
        Hand::new(HAND_RIGHT, HandConfig::default()),
    ];
    let mut shapers = [
        VelocityShaper::new(ShapeParams::default()),
        VelocityShaper::new(ShapeParams::default()),
    ];
    let mut memory: [HandMemory; 2] = Default::default();
    let mut holds = HoldStats::default();
    let mut bodies = BodyFinder::default();
    let (mut failures, mut next_sensors) = (0u32, 0u64);
    while !d.quit.load(Ordering::Acquire) {
        if d.reopen.swap(false, Ordering::AcqRel) {
            log!("reopening the engine for {} mode", d.mode.lock().unwrap());
            return;
        }
        let pending = mem::take(&mut *d.haptics.lock().unwrap());
        for (h, cmd) in pending.iter().enumerate() {
            if let Some(c) = cmd {
                e.haptic(h, c.duration_s, c.frequency_hz, c.amplitude);
            }
        }
        if e.update().is_err() {
            failures += 1;
            if failures >= IO_FAILURES_BEFORE_REOPEN {
                log!("headset I/O failing; reopening");
                return;
            }
        } else {
            failures = 0;
        }
        let now = shm::now_ns();
        let mut arrived = [None; 2];
        let mut published = false;
        for h in [HAND_LEFT, HAND_RIGHT] {
            let read_from = shm::now_ns();
            let Some(c) = e.controller(h) else { continue };
            if let Some(identity) = e.controller_identity(h) {
                d.inputs.lock().unwrap().set_identity(h, identity);
            }
            link.engine_view.lock().unwrap()[h] = Some(c);
            if !link.hands.engine_publishes(h) {
                continue;
            }
            if c.device_time_ns != holds.last_sample[h] {
                holds.last_sample[h] = c.device_time_ns;
                arrived[h] = Some(HoldParts {
                    held_ns: c.received_host_ns,
                    filter_ns: c.filter_ns,
                    update_ns: now.saturating_sub(c.received_host_ns),
                    read_ns: shm::now_ns().saturating_sub(read_from),
                });
            }
            publish_hand_state(
                d,
                h,
                &c,
                now,
                &mut hands[h],
                None,
                &mut shapers[h],
                &mut memory[h],
            );
            published = true;
        }
        if published {
            d.shm.signal();
        }
        d.shm.heartbeat(now);
        holds.note(arrived, shm::now_ns());
        if let Some(f) = link.biases.lock().unwrap().take() {
            save_biases(&f);
        }
        for change in std::mem::take(&mut *link.reposed.lock().unwrap()) {
            match change {
                PoseChange::Cleared(serial) => match e.clear_sensor_pose(&serial) {
                    Ok(()) => log!(
                        "sensor {serial}: the engine labels its blobs by the long search while it is placed"
                    ),
                    Err(err) => log!("sensor {serial}: {err}"),
                },
                PoseChange::Set(r) => apply_reposed(d, e, &r),
            }
        }
        if radio {
            if bodies.find(e, link) {
                d.wake_radio();
            }
        } else {
            // no tracker reads the frames: keep the queue from growing
            d.track_in.lock().unwrap().clear();
        }
        if now >= next_sensors {
            let sensors: Vec<SensorStatus> =
                e.sensors().iter().map(|s| sensor_status(d, s)).collect();
            publish_sensor_snapshot(d, &sensors, &e.sensor_streams(), &e.sensor_video());
            next_sensors = now + SENSOR_PUBLISH_NS;
            if d.rec.active() {
                write_session_info(d, e);
            }
        }
    }
}

/// Finds each controller's tracker body (its LED model and factory IMU position) once the engine
/// has the model, for the radio thread to add.
#[derive(Default)]
struct BodyFinder {
    sent: [bool; 2],
    warned: [bool; 2],
    next_try: [u64; 2],
}

impl BodyFinder {
    /// True when it handed the radio thread a body.
    fn find(&mut self, e: &Engine, link: &RadioLink) -> bool {
        let now = shm::now_ns();
        let mut found = false;
        for h in [HAND_LEFT, HAND_RIGHT] {
            // a controller with no cached configuration is looked up again every few seconds, not
            // on every pass of the engine loop
            if self.sent[h] || now < self.next_try[h] {
                continue;
            }
            let Some(c) = e.controller(h) else { continue };
            let model = e.led_model(h);
            if model.is_empty() {
                continue;
            }
            let leds: Vec<calib::capture::Led> = model
                .iter()
                .map(|l| calib::capture::Led {
                    pos: calib::vec3(l.pos),
                    dir: calib::vec3(l.dir),
                })
                .collect();
            match track::factory::imu_position(&track::factory::openhmd_config_dir(), &leds) {
                Some(imu) => {
                    link.bodies.lock().unwrap().push(NewBody {
                        hand: h,
                        device_id: c.device_id,
                        leds,
                        imu,
                    });
                    self.sent[h] = true;
                    found = true;
                }
                None => {
                    self.next_try[h] = now + 5_000_000_000;
                    if !self.warned[h] {
                        self.warned[h] = true;
                        log!("{}: no factory IMU position cached for its LED model; publishing OpenHMD poses for it", ["left", "right"][h]);
                    }
                }
            }
        }
        found
    }
}

/// How long the radio thread sleeps with nothing to do, ms: it keeps the hands' published state
/// current (a controller set down stops sending) about this often.
const RADIO_IDLE_WAKE_MS: u64 = 20;
const RADIO_REFRESH_NS: u64 = 20_000_000;

/// The radio thread: reads the radio through its own HID handle and publishes each controller
/// sample as it lands (the tracker's pose at it), and between reports feeds the tracker the camera
/// frames one at a time, so a report waits for at most one frame. Runs SCHED_RR (`make_realtime`)
/// with inherited affinity or the explicit `CV1TOUCH_CPUS` selection.
fn radio_main(
    d: &Daemon,
    link: &RadioLink,
    dec: &mut engine::RadioDecoder,
    radio: &mut hid::Device,
    pipe: &mut Pipeline,
    watch: &mut Option<LandmarkWatch>,
) {
    let me = system::current_thread();
    if let Err(err) = set_rt_affinity(me) {
        log!("radio thread keeps the process's CPUs ({err})");
    }
    match system::make_realtime(me, RADIO_RT_PRIORITY) {
        Ok(how) => log!("radio thread real-time ({how})"),
        Err(err) => log!("radio thread stays normal priority ({err})"),
    }
    d.radio_on.store(true, Ordering::Release);
    let _exit = RadioExit { d, link };
    let mut biases = BiasMemory::load();
    let mut reposing = Reposing::start();
    let mut hands = [
        Hand::new(HAND_LEFT, HandConfig::default()),
        Hand::new(HAND_RIGHT, HandConfig::default()),
    ];
    let mut shapers = [
        VelocityShaper::new(ShapeParams::default()),
        VelocityShaper::new(ShapeParams::default()),
    ];
    let mut memory: [HandMemory; 2] = Default::default();
    let mut holds = HoldStats::default();
    let mut newest: [Option<engine::Controller>; 2] = [None; 2];
    let mut last_publish = [0u64; 2];
    let mut search_skip = SearchSkip::default();
    let mut next_stats = 0u64;
    let mut buf = [0u8; 64];
    let mut radio_failed = false;
    let mut rest = Rest::new();
    while !link.stop.load(Ordering::Acquire) && !radio_failed {
        let idle_ms = if d.track_in.lock().unwrap().is_empty() {
            RADIO_IDLE_WAKE_MS
        } else {
            0
        };
        // an interrupted wait just reads what is in
        let _ = radio.wait(&d.radio_wake, Some(Duration::from_millis(idle_ms)));
        // every report that is in
        let mut arrived = [None; 2];
        let mut published = false;
        while !radio_failed {
            let n = match radio.try_read(&mut buf) {
                Ok(Some(n)) => n,
                Ok(None) => break,
                Err(err) => {
                    // the headset gone (waiting on a dead device returns at once, which would
                    // spin this thread): the engine reopens, with a new radio thread
                    log!("radio read failed ({err}); reopening the engine");
                    radio_failed = true;
                    break;
                }
            };
            let at = shm::now_ns();
            let report = &buf[..n];
            if d.rec.active() {
                d.rec
                    .write(recorder::RAW_REPORT, at, &[&1u32.to_le_bytes(), report]);
            }
            d.flight
                .note(recorder::RAW_REPORT, at, &[&1u32.to_le_bytes(), report]);
            let (samples, k) = dec.decode(report, at);
            for s in &samples[..k] {
                let Some(h) = pipeline::device_hand(s.device_id) else {
                    continue;
                };
                if !link.hands.radio_publishes(h) || !pipe.has_body(h) {
                    continue;
                }
                let Some(view) = link.engine_view.lock().unwrap()[h] else {
                    continue;
                };
                let c = radio_sample(&view, s);
                let now = shm::now_ns();
                publish_hand_state(
                    d,
                    h,
                    &c,
                    now,
                    &mut hands[h],
                    Some(pipe),
                    &mut shapers[h],
                    &mut memory[h],
                );
                newest[h] = Some(*s);
                last_publish[h] = now;
                arrived[h] = Some(HoldParts {
                    held_ns: at,
                    ..Default::default()
                });
                published = true;
            }
        }
        if published {
            d.shm.signal();
            holds.note(arrived, shm::now_ns());
        }
        for b in link.bodies.lock().unwrap().drain(..) {
            pipe.body(b.device_id, b.leds.clone(), b.imu);
            log!(
                "{}: tracking with {} LEDs, IMU at ({:.1}, {:.1}, {:.1}) mm",
                ["left", "right"][b.hand],
                b.leds.len(),
                b.imu.x * 1e3,
                b.imu.y * 1e3,
                b.imu.z * 1e3
            );
            biases.seed(pipe, b.hand, &b.leds);
            link.hands.request(b.hand);
        }
        // One fairly scheduled camera frame per pass; callbacks cannot grow a stalled
        // consumer's queue. Release the ingress lock before any tracking work.
        let (clocks, frame) = {
            let mut ingress = d.track_in.lock().unwrap();
            (ingress.take_clocks(), ingress.pop_frame())
        };
        for (host, us) in clocks {
            pipe.hmd_clock_sample(host, us);
        }
        if let Some((info, blobs)) = frame {
            if let Some(w) = watch.as_mut() {
                w.frame(&info, &blobs);
            }
            pipe.frame(&info, &blobs);
        }
        // a hand whose controller went quiet still gets its state refreshed (it times out)
        let now = shm::now_ns();
        if let Some(w) = watch.as_mut() {
            w.poll(d, now);
        }
        let mut refreshed = false;
        for h in [HAND_LEFT, HAND_RIGHT] {
            let (Some(s), Some(view)) = (newest[h], link.engine_view.lock().unwrap()[h]) else {
                continue;
            };
            if link.hands.radio_publishes(h)
                && now.saturating_sub(last_publish[h]) >= RADIO_REFRESH_NS
            {
                publish_hand_state(
                    d,
                    h,
                    &radio_sample(&view, &s),
                    now,
                    &mut hands[h],
                    Some(pipe),
                    &mut shapers[h],
                    &mut memory[h],
                );
                last_publish[h] = now;
                refreshed = true;
            }
        }
        if refreshed {
            d.shm.signal();
        }
        for h in [HAND_LEFT, HAND_RIGHT] {
            // only hands our tracker publishes, and only while fits keep coming: a track no second
            // sensor has confirmed yet may be wrong, and the engine keeps searching meanwhile
            let since_fix = pipe
                .tracker(h)
                .filter(|t| t.tracking())
                .and_then(|t| Some(t.newest_imu()?.saturating_sub(t.last_fix())));
            let since_sample = newest[h].map(|s| now.saturating_sub(s.received_host_ns));
            search_skip.hands[h] = link.hands.radio_publishes(h)
                && pipe.has_body(h)
                && search_needless(since_fix, since_sample);
        }
        if let Some(hands) = search_skip.changed() {
            dec.set_search_skip(hands);
        }
        if let Some(f) = biases.update(pipe, now) {
            *link.biases.lock().unwrap() = Some(f);
        }
        reposing.tick(d, link, pipe, watch, now);
        rest.if_due(now);
        if now >= next_stats {
            d.pending_frames_dropped
                .store(pipe.frames_dropped, Ordering::Relaxed);
            *d.track_stats.lock().unwrap() = [HAND_LEFT, HAND_RIGHT].map(|h| {
                (
                    pipe.stats(h),
                    pipe.latest(h).is_some_and(|s| s.output.tracking),
                )
            });
            next_stats = now + SENSOR_PUBLISH_NS;
        }
    }
    if let Some(f) = biases.take_changed() {
        *link.biases.lock().unwrap() = Some(f);
    }
}

/// How recent our tracker's last optical fit of a controller must be for OpenHMD's brute-force
/// search to leave it out (about five camera frames): once fits stop, the engine searches again.
const SEARCH_SKIP_FIX_NS: u64 = 100_000_000;
/// How long a controller is silent on the radio before the search leaves it out as asleep (its
/// LEDs dark); a shorter radio dropout keeps it searched.
const SEARCH_SKIP_ASLEEP_NS: u64 = 1_000_000_000;

/// Whether OpenHMD's brute-force pose search can leave a controller out, from the controller time
/// since our tracker last fitted it while publishing it (an unconfirmed track does not count) and
/// the host time since its last radio sample (None: none yet). Those long searches for a
/// controller we already fit found little we use (in replays, under 0.5% of the tracker's
/// updates) at up to a core of CPU in play.
fn search_needless(since_fix_ns: Option<u64>, since_sample_ns: Option<u64>) -> bool {
    since_fix_ns.is_some_and(|t| t <= SEARCH_SKIP_FIX_NS)
        || since_sample_ns.is_none_or(|t| t >= SEARCH_SKIP_ASLEEP_NS)
}

/// The controllers the engine is told to leave out of its brute-force search (`search_needless`).
#[derive(Default)]
struct SearchSkip {
    hands: [bool; 2],
    told: Option<[bool; 2]>,
}

impl SearchSkip {
    /// What to tell the engine, when it changed.
    fn changed(&mut self) -> Option<[bool; 2]> {
        (self.told != Some(self.hands)).then(|| {
            self.told = Some(self.hands);
            self.hands
        })
    }
}

/// The radio thread's way out, normal or not: the engine gets the hands back and records the
/// radio again; if the engine did not ask it to stop (a dead radio, a panic), the engine reopens,
/// with a new radio thread.
struct RadioExit<'a> {
    d: &'a Daemon,
    link: &'a RadioLink,
}

impl Drop for RadioExit<'_> {
    fn drop(&mut self) {
        self.link.hands.radio_exit();
        self.d.radio_on.store(false, Ordering::Release);
        if !self.link.stop.load(Ordering::Acquire) {
            self.d.reopen.store(true, Ordering::Release);
        }
    }
}

/// How long the radio thread may run without sleeping before it sleeps a moment of its own:
/// RLIMIT_RTTIME (200 ms) kills the daemon if a real-time thread runs that long unblocked, and a
/// poll that finds a report or a frame waiting returns without sleeping. Windows has no such limit
/// (nor reports a thread's sleeps), so there it never rests.
const RADIO_MAX_RUN_NS: u64 = 50_000_000;
const RADIO_REST: Duration = Duration::from_micros(200);

/// Tells the radio thread when it last slept (its voluntary context switches) and makes it rest
/// if that was `RADIO_MAX_RUN_NS` ago.
struct Rest {
    switches: Option<i64>,
    since: u64,
}

impl Rest {
    fn new() -> Rest {
        Rest {
            switches: system::voluntary_switches(),
            since: shm::now_ns(),
        }
    }

    fn if_due(&mut self, now: u64) {
        let Some(s) = system::voluntary_switches() else {
            return;
        };
        if Some(s) != self.switches {
            (self.switches, self.since) = (Some(s), now);
        } else if now.saturating_sub(self.since) >= RADIO_MAX_RUN_NS {
            thread::sleep(RADIO_REST);
            (self.switches, self.since) = (system::voluntary_switches(), shm::now_ns());
        }
    }
}

/// What `publish_hand_state` keeps per hand from one sample to the next.
#[derive(Debug, Default)]
struct HandMemory {
    /// The tracker's wrong-lock count at the last sample, to log each drop.
    wrong_locks: u64,
    /// Whether the connected controller had a position at the last sample.
    was_tracking: bool,
    /// Since when a connected controller has been without a position: its wake or its loss.
    unfixed_since: Option<u64>,
    /// Where the tracker last had the controller (see `hold_unfixed`).
    last_seen: Option<[f32; 3]>,
    /// The state last published while the controller's samples arrived: what a radio gap holds.
    last_live: Option<ControllerState>,
}

impl HandMemory {
    /// Notes whether the controller is connected and has a position at `now`; returns what
    /// changed for the log: a loss, or a position regained and how long that took. A controller
    /// going to sleep is not losing its position, and one waking is without one from then.
    fn observe(&mut self, connected: bool, tracking: bool, now: u64) -> Option<String> {
        if !connected {
            self.was_tracking = false;
            self.unfixed_since = None;
            return None;
        }
        let was = std::mem::replace(&mut self.was_tracking, tracking);
        match (was, tracking) {
            (false, true) => Some(match self.unfixed_since.take() {
                Some(t) => format!(
                    "tracking after {:.0} ms without a position",
                    now.saturating_sub(t) as f64 * 1e-6
                ),
                None => "tracking".into(),
            }),
            (true, false) => {
                self.unfixed_since = Some(now);
                Some("position lost".into())
            }
            (false, false) => {
                self.unfixed_since.get_or_insert(now);
                None
            }
            (true, true) => None,
        }
    }
}

/// The tracker's part of a hand's state: while the controller is connected, `tracked` puts the
/// tracker's newest output in, and `memory` keeps where a tracking output left the controller, or
/// without an output the controller is held there (`hold_unfixed`).
fn apply_tracker(
    state: &mut ControllerState,
    latest: Option<&Stamped>,
    memory: &mut HandMemory,
    tracked: impl FnOnce(&mut ControllerState, &Stamped),
) {
    let connected = state.flags & ffi::CV1T_FLAG_CONNECTED != 0;
    match latest {
        Some(s) if connected => {
            tracked(state, s);
            if s.output.tracking {
                memory.last_seen = Some(state.pos);
            }
        }
        None if connected => hold_unfixed(state, memory.last_seen),
        _ => state.flags &= !ffi::CV1T_FLAG_POSITION,
    }
}

/// Logs a change in a controller's radio gap. The radio's traffic around a gap of at least
/// `flight::SHORTEST_NS` is recorded (`FlightRecorder`) when it began while the controller was in
/// use, or when it ended (a controller held still does not turn, but one asleep does not come
/// back either); a shorter one that ends is a blip, not recorded.
fn note_gap(d: &Daemon, h: usize, event: GapEvent) {
    let name = ["left", "right"][h];
    let ride_s = HandConfig::default().ride_through_ns as f64 * 1e-9;
    match event {
        GapEvent::Began {
            last_sample_host_ns,
            moving: true,
        } => d.flight.dropout(h, last_sample_host_ns, true),
        GapEvent::Ended {
            last_sample_host_ns,
            silent_ns,
        } if silent_ns >= flight::SHORTEST_NS => d.flight.dropout(h, last_sample_host_ns, false),
        GapEvent::Ended {
            last_sample_host_ns,
            ..
        } => d.flight.cancel(h, last_sample_host_ns),
        _ => {}
    }
    match event {
        GapEvent::Began { moving, .. } => log!(
            "{name}: radio silent ({}): holding it as it was, inputs included, for up to {ride_s:.0} s",
            if moving { "in use" } else { "still" }
        ),
        GapEvent::Ended { silent_ns, .. } => {
            log!("{name}: radio back after {:.2} s", silent_ns as f64 * 1e-9)
        }
        GapEvent::Expired { silent_ns } => log!(
            "{name}: radio silent for {:.0} s: standby, inputs released",
            silent_ns as f64 * 1e-9
        ),
    }
}

/// Hands due dropout recordings to the daemon's single writer without waiting for storage.
fn save_dropouts(d: &Daemon, writer: Option<&flight_writer::Writer>) {
    let unix_s = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |t| t.as_secs());
    for dropout in d.flight.due(shm::now_ns()) {
        let Some(writer) = writer else {
            log("could not record a dropout: the recording thread did not start");
            continue;
        };
        match writer.submit(dropout, unix_s) {
            Ok(Some(dropped)) => log!(
                "{}: dropped a dropout recording because storage fell behind",
                ["left", "right"][dropped.hand]
            ),
            Ok(None) => {}
            Err(e) => log!("could not record a dropout: {e}"),
        }
    }
}

/// What a controller in a radio gap is published as: the state last published while its samples
/// arrived (its pose, and the inputs then held, input calibration applied), neither moving nor
/// turning, without a position. The game keeps the hand where it was and whatever it held.
fn held_through_gap(last: &ControllerState, now: u64) -> ControllerState {
    ControllerState {
        flags: (last.flags & ffi::CV1T_FLAG_CALIBRATED)
            | ffi::CV1T_FLAG_CONNECTED
            | ffi::CV1T_FLAG_RADIO_GAP,
        lin_vel: [0.0; 3],
        // a zero body rate for the driver (ang_vel - gyro_bias = 0)
        ang_vel: last.gyro_bias,
        publish_host_ns: now,
        ..*last
    }
}

/// For a connected controller the tracker shows no position for (it woke, its track was dropped,
/// or its new track awaits a second sensor's confirmation): SteamVR is shown it where the tracker
/// last had it, turning with the engine's IMU orientation but neither moving nor extrapolated,
/// without a position. Not at the engine's position: the engine's own searches leave out the
/// controllers the tracker holds, so its position can be metres off by then. Before the tracker's
/// first fix the engine's position is all there is.
fn hold_unfixed(state: &mut ControllerState, last_seen: Option<[f32; 3]>) {
    if let Some(pos) = last_seen {
        state.pos = pos;
    }
    state.lin_vel = [0.0; 3];
    // a zero body rate for the driver (ang_vel - gyro_bias = 0): no turn to extrapolate
    state.ang_vel = state.gyro_bias;
    state.flags &= !ffi::CV1T_FLAG_POSITION;
    state.last_position_fix_host_ns = 0;
}

/// The radio thread's sample `s` over the engine's view of its controller. The engine's fix
/// times run on the engine's device clock, not the decoder's, so they are dropped: the tracker
/// says whether the hand has a position.
fn radio_sample(view: &engine::Controller, s: &engine::Controller) -> engine::Controller {
    engine::Controller {
        last_pose_fix_device_ns: 0,
        last_orient_fix_device_ns: 0,
        ..engine::with_radio_sample(view, s)
    }
}

/// Publishes one hand's state from `c`: the engine's controller, or the radio thread's sample over
/// the engine's view of it (`radio_sample`), with the tracker's pose when it has the hand's body.
#[allow(clippy::too_many_arguments)]
fn publish_hand_state(
    d: &Daemon,
    h: usize,
    c: &engine::Controller,
    now: u64,
    hand: &mut Hand,
    pipe: Option<&mut Pipeline>,
    shaper: &mut VelocityShaper,
    memory: &mut HandMemory,
) {
    let mut state = hand.update(c, now);
    if state.flags & ffi::CV1T_FLAG_CONNECTED != 0 {
        state.battery_percent = d.battery[h].load(Ordering::Relaxed);
    }
    if let Some(p) = pipe.filter(|p| p.has_body(h)) {
        let v = |a: [f32; 3]| nalgebra::Vector3::new(a[0] as f64, a[1] as f64, a[2] as f64);
        p.imu(
            h,
            c.device_time_ns,
            c.received_host_ns,
            v(c.imu_gyro),
            v(c.imu_accel),
        );
        let wl = p.stats(h).wrong_locks;
        if wl != memory.wrong_locks {
            memory.wrong_locks = wl;
            log!(
                "{}: dropped a wrong lock (accelerometer bias ran off)",
                ["left", "right"][h]
            );
        }
        let latest = p.latest(h);
        apply_tracker(&mut state, latest.as_ref(), memory, |state, s| {
            apply_tracked(
                state,
                s,
                d.extra_latency_ns.load(Ordering::Relaxed),
                shaper,
                d.shaping_on.load(Ordering::Relaxed),
            )
        });
        let connected = state.flags & ffi::CV1T_FLAG_CONNECTED != 0;
        let tracking = latest.is_some_and(|s| s.output.tracking);
        if let Some(change) = memory.observe(connected, tracking, now) {
            log!("{}: {change}", ["left", "right"][h]);
        }
    }
    d.inputs.lock().unwrap().apply(h, &mut state);
    d.inject.apply(h, now, &mut state);
    if state.flags & ffi::CV1T_FLAG_RADIO_GAP != 0 {
        if let Some(last) = &memory.last_live {
            state = held_through_gap(last, now);
        }
    } else {
        memory.last_live = (state.flags & ffi::CV1T_FLAG_CONNECTED != 0).then_some(state);
    }
    if let Some(event) = hand.take_event() {
        note_gap(d, h, event);
    }
    d.shm.publish_hand(h, &state);
    if d.rec.active() {
        d.rec.write(
            recorder::CONTROLLER_STATE,
            now,
            &[
                &(h as u32).to_le_bytes(),
                &[0u8; 4],
                recorder::as_bytes(&state),
            ],
        );
    }
}

/// How long the daemon holds each IMU sample: its radio report's arrival to the shm signal that
/// wakes the driver, and the parts of that. Logged once a minute while samples flow.
#[derive(Default)]
struct HoldStats {
    last_sample: [u64; 2],
    samples: [Vec<HoldParts>; 2],
    since: u64,
}

/// One IMU sample's hold, ns.
#[derive(Clone, Copy, Debug, Default)]
struct HoldParts {
    /// radio arrival to the driver's wake-up
    held_ns: u64,
    /// in OpenHMD's own IMU filter, its lock wait included
    filter_ns: u64,
    /// arrival to the end of the engine update, the filter included
    update_ns: u64,
    /// reading the controller back from the engine (takes the device lock)
    read_ns: u64,
}

impl HoldStats {
    /// `arrived`: per hand, a new sample's parts with `held_ns` still the radio arrival time.
    fn note(&mut self, arrived: [Option<HoldParts>; 2], now: u64) {
        for (h, a) in arrived.iter().enumerate() {
            if let Some(a) = a {
                self.samples[h].push(HoldParts {
                    held_ns: now.saturating_sub(a.held_ns),
                    ..*a
                });
            }
        }
        if self.since == 0 {
            self.since = now;
        }
        if now.saturating_sub(self.since) < 60_000_000_000 {
            return;
        }
        self.since = now;
        for h in 0..2 {
            let v = &mut self.samples[h];
            if v.len() >= 100 {
                log!("{}", hold_summary(["left", "right"][h], v));
            }
            v.clear();
        }
    }
}

fn hold_summary(hand: &str, v: &mut [HoldParts]) -> String {
    let ms = |ns: u64| ns as f64 * 1e-6;
    let col = |f: &dyn Fn(&HoldParts) -> u64| {
        let mut c: Vec<u64> = v.iter().map(f).collect();
        c.sort_unstable();
        let at = |q: f64| ms(c[((c.len() - 1) as f64 * q) as usize]);
        (at(0.5), at(0.99), at(1.0))
    };
    let held = col(&|p| p.held_ns);
    let filter = col(&|p| p.filter_ns);
    let rest = col(&|p| p.update_ns.saturating_sub(p.filter_ns));
    let read = col(&|p| p.read_ns);
    let ours = col(&|p| p.held_ns.saturating_sub(p.update_ns + p.read_ns));
    format!(
        "{hand}: IMU sample held {:.2} ms median, {:.2} p99, {:.2} max (radio arrival to driver wake-up, {} samples); \
         p99/max in OpenHMD's IMU filter {:.2}/{:.2}, rest of its update {:.2}/{:.2}, reading it back {:.2}/{:.2}, ours {:.2}/{:.2}",
        held.0, held.1, held.2, v.len(), filter.1, filter.2, rest.1, rest.2, read.1, read.2, ours.1, ours.2
    )
}

fn status_json(d: &Daemon) -> String {
    let (engine_ready, mode, calibrated_sensors, ignored_sensors) = {
        let opts = d.opts.lock().unwrap();
        let mode = d.mode.lock().unwrap().clone();
        // Publish the requested calibration even while its engine is reopening. Null
        // distinguishes capture/provisional from an older daemon without this field.
        let calibrated = (mode == "calibrated").then(|| {
            opts.fixed_poses
                .iter()
                .map(|p| p.serial.clone())
                .collect::<Vec<_>>()
        });
        (
            engine_is_ready(d, &opts),
            mode,
            calibrated,
            opts.ignore_sensors.clone(),
        )
    };
    let hands = [HAND_LEFT, HAND_RIGHT].map(|h| d.shm.hand(h).unwrap_or_default());
    let (sensors, sensor_streams, sensor_video) = {
        let sensors = d.sensors.lock().unwrap();
        (
            sensors.clone(),
            d.sensor_streams.lock().unwrap().clone(),
            d.sensor_video.lock().unwrap().clone(),
        )
    };
    let recording = d.rec.status();
    let st = recording.stats;
    let mut recording_json = serde_json::to_value(&recording).unwrap();
    recording_json["message"] = serde_json::json!(recording.message());
    let al = d.shm.alignment().unwrap_or_default();
    let hands_json: Vec<String> = hands
        .iter()
        .zip(["left", "right"])
        .zip(d.track_stats.lock().unwrap().iter())
        .map(|((s, name), (t, tracking))| {
            format!(
                "{{\"hand\":\"{name}\",\"flags\":{},\"tracker\":{{\"tracking\":{tracking},\"frames\":{},\"updates\":{},\"not_visible\":{},\"too_few\":{},\"rejected\":{},\"acquisitions\":{},\"good\":{},\"searched\":{},\"unconfirmed\":{},\"refused\":{},\"out_of_order\":{},\"wrong_locks\":{}}}}}",
                s.flags, t.frames, t.updates, t.not_visible, t.too_few, t.rejected, t.acquisitions, t.good_acquisitions, t.searched_acquisitions, t.unconfirmed_acquisitions, t.refused_acquisitions, t.out_of_order, t.wrong_locks
            )
        })
        .collect();
    let shifts = d.landmark_shifts.lock().unwrap();
    let poses = d.sensor_poses.lock().unwrap();
    let agreements = d.sensor_agreement.lock().unwrap();
    let received: std::collections::HashMap<_, _> = sensor_video
        .iter()
        .map(|video| {
            (
                cv1touch::status::sensor_identity(&video.serial),
                video.frames_received,
            )
        })
        .collect();
    let sensors_json: Vec<serde_json::Value> = sensors
        .iter()
        .map(|s| {
            let serial = shm::serial_str(s);
            let shift = shifts
                .iter()
                .find(|(s, _)| cv1touch::status::same_sensor(s, &serial))
                .map(|(_, (x, y, n))| serde_json::json!({"dx": x, "dy": y, "landmarks": n}));
            let pose = poses
                .get(&serial)
                .map(|note| serde_json::to_value(note).unwrap_or_default());
            let agreement = agreements.get(&serial).map(|a| {
                serde_json::json!({"median_px": a.median_px, "bias_px": a.bias_px.norm(),
                "samples": a.samples, "checks": a.checks, "misses": a.misses})
            });
            serde_json::json!({"serial": serial, "placed": s.placed, "frames": s.frames,
            "frames_received": received.get(&cv1touch::status::sensor_identity(&serial)).copied(),
            "fixes": s.pose_fixes, "camera_orient": s.camera_orient, "camera_pos": s.camera_pos,
            "landmark_shift": shift, "pose": pose, "agreement": agreement})
        })
        .collect();
    drop(agreements);
    drop(poses);
    drop(shifts);
    let mut queue = serde_json::to_value(d.track_in.lock().unwrap().stats()).unwrap();
    queue["pending_frames_dropped"] =
        serde_json::json!(d.pending_frames_dropped.load(Ordering::Relaxed));
    format!(
        "ok {{\"generation\":{},\"shm\":{},\"tracker\":\"{:?}\",\"mode\":\"{}\",\"latency_adjustment_ms\":{},\"tilt_deg\":{},\"engine_ready\":{},\"input_calibration\":{},\"alignment\":{{\"registration\":{},\"render\":[{},{}],\"universe\":{}}},\"recording_scoped_stop\":true,\"recording_durable_stop\":true,\"recording_status\":{},\"recording\":{},\"recorded\":{},\"dropped\":{},\"write_errors\":{},\"hands\":[{}],\"sensors\":{},\"sensors_unsure\":{},\"sensor_frames_received\":true,\"sensor_total\":{},\"sensor_shm_count\":{},\"sensor_shm_truncated\":{},\"camera_queue\":{},\"ignored_sensors\":{},\"calibrated_sensors\":{},\"sensor_streams\":{}}}",
        d.shm.generation(),
        serde_json::to_string(d.shm.name()).unwrap(),
        d.tracker,
        mode,
        d.extra_latency_ns.load(Ordering::Relaxed) as f64 / 1e6,
        tilt_deg(d),
        engine_ready,
        d.inputs.lock().unwrap().status(),
        al.valid & ffi::CV1T_ALIGN_LIGHTHOUSE != 0,
        al.valid & ffi::CV1T_ALIGN_RENDER_LEFT != 0,
        al.valid & ffi::CV1T_ALIGN_RENDER_RIGHT != 0,
        al.universe_id,
        recording_json,
        recording.state == recorder::State::Recording,
        st.records,
        st.dropped,
        st.write_errors,
        hands_json.join(","),
        serde_json::to_string(&sensors_json).unwrap(),
        d.sensors_unsure.load(Ordering::Relaxed),
        sensors.len(),
        sensors.len().min(shm::MAX_SENSORS),
        sensors.len() > shm::MAX_SENSORS,
        serde_json::to_string(&queue).unwrap(),
        serde_json::to_string(&ignored_sensors).unwrap(),
        serde_json::to_string(&calibrated_sensors).unwrap(),
        serde_json::to_string(&sensor_streams).unwrap(),
    )
}

/// The reply to `record start`. A recording that opened also keeps the sensors' pixels when it
/// asked for them, and gets its own session info (sensor intrinsics, LED models) written.
fn recording_started(d: &Daemon, frames: bool, started: Result<(), String>) -> String {
    match started {
        Ok(()) => {
            d.record_frames.store(frames, Ordering::Release);
            d.session.lock().unwrap().reset();
            "ok".into()
        }
        Err(error) => format!("err {error}"),
    }
}

fn handle(d: &Daemon, line: &str) -> String {
    match control::parse(line) {
        Err(e) => format!("err {e}"),
        Ok(Command::Stop) => {
            d.stop_requested.store(true, Ordering::Release);
            "ok stopping".into()
        }
        Ok(Command::Haptic {
            hand,
            duration_s,
            frequency_hz,
            amplitude,
        }) => {
            let payload = recorder::haptic_payload(hand, duration_s, frequency_hz, amplitude);
            let now = shm::now_ns();
            d.rec.write(recorder::HAPTIC, now, &[&payload]);
            d.flight.note(recorder::HAPTIC, now, &[&payload]);
            d.haptics.lock().unwrap()[hand] = Some(Haptic {
                duration_s,
                frequency_hz,
                amplitude,
            });
            "ok".into()
        }
        // Control requests start recordings in `handle_control_recording`, which reserves the
        // session under the dispatch turn; this is the same start in one step.
        Ok(Command::RecordStart(path, frames)) => recording_started(
            d,
            frames,
            d.rec.start_with_frames(&path, shm::now_ns(), frames),
        ),
        Ok(Command::RecordStop) => match d.rec.stop_timeout(None, RECORD_STOP_WAIT) {
            Some(status) => {
                d.record_frames.store(false, Ordering::Release);
                record_stop_reply(&status, false)
            }
            None => "err not recording".into(),
        },
        Ok(Command::RecordStopIf(path)) => {
            match d.rec.stop_timeout(Some(&path), RECORD_STOP_WAIT) {
                Some(status) => {
                    d.record_frames.store(false, Ordering::Release);
                    record_stop_reply(&status, true)
                }
                None => "ok not-matching".into(),
            }
        }
        Ok(Command::Status) => status_json(d),
        Ok(Command::Inputs) => {
            let inputs = d.inputs.lock().unwrap();
            format!(
                "ok {}",
                serde_json::json!({
                    "hands": inputs.latest,
                    "calibrated": [inputs.calibration_applies(0), inputs.calibration_applies(1)],
                    "error": inputs.error,
                })
            )
        }
        Ok(Command::InputsReload) => {
            match reload_inputs(&d.inputs, &input_calibration::default_path()) {
                Ok(()) => "ok input calibration reloaded".into(),
                Err(error) => format!("err {error}"),
            }
        }
        Ok(Command::CalibrationCapture) => {
            *d.calibration.lock().unwrap() = None;
            switch_mode(d, modes::capture(&current_opts(d)), "capture")
        }
        Ok(Command::CalibrationProvisional) => {
            *d.calibration.lock().unwrap() = None;
            switch_mode(d, modes::provisional(&current_opts(d)), "provisional")
        }
        Ok(Command::CalibrationLoad(path)) => match modes::calibrated(&current_opts(d), &path) {
            Ok((opts, n)) => {
                *d.calibration.lock().unwrap() = Some(LoadedCalibration::at(path.clone()));
                switch_mode(d, opts, "calibrated");
                format!("ok calibrated: {n} sensor poses from {}", path.display())
            }
            Err(e) => format!("err {e}"),
        },
        Ok(Command::Stations(v)) => {
            let poses: Vec<(String, nalgebra::Isometry3<f64>)> = v
                .iter()
                .map(|(serial, p)| {
                    (
                        serial.clone(),
                        nalgebra::Isometry3::from_parts(
                            nalgebra::Translation3::new(p[0], p[1], p[2]),
                            nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                                p[6], p[3], p[4], p[5],
                            )),
                        ),
                    )
                })
                .collect();
            *d.driver_stations.lock().unwrap() = Some((shm::now_ns(), poses));
            "ok".into()
        }
        Ok(Command::TestInject(on)) => {
            let now = shm::now_ns();
            match on {
                Some(true) => {
                    d.inject.arm(now);
                    log!(
                        "test input injection armed for {} s",
                        INJECT_ARMED_NS / 1_000_000_000
                    );
                }
                Some(false) => d.inject.disarm(),
                None => {}
            }
            match d.inject.armed_for(now) {
                Some(left) => format!("ok test-inject on ({} s left)", left / 1_000_000_000),
                None => "ok test-inject off".into(),
            }
        }
        Ok(Command::Inject { hand, inputs, ms }) => {
            match d.inject.press(hand, inputs, ms, shm::now_ns()) {
                Ok(()) => "ok".into(),
                Err(e) => format!("err {e}"),
            }
        }
        Ok(Command::Shaping(on)) => {
            if let Some(on) = on {
                d.shaping_on.store(on, Ordering::Release);
            }
            format!(
                "ok velocity shaping for SteamVR's prediction {}",
                if d.shaping_on.load(Ordering::Acquire) {
                    "on"
                } else {
                    "off"
                }
            )
        }
        Ok(Command::Prediction { position, rotation }) => {
            let mut unsaved = String::new();
            if position.is_some() || rotation.is_some() {
                // one request at a time (serve_client): the pair saved is the pair set
                if let Some(p) = position {
                    d.position_prediction.store(p, Ordering::Release);
                }
                if let Some(p) = rotation {
                    d.rotation_prediction.store(p, Ordering::Release);
                }
                unsaved = save_settings(d);
                republish_settings(d);
            }
            // the driver applies the percents to whatever is published, OpenHMD's poses too
            format!(
                "ok prediction: {}{unsaved}",
                prediction_text(
                    d.position_prediction.load(Ordering::Acquire),
                    d.rotation_prediction.load(Ordering::Acquire)
                )
            )
        }
        Ok(Command::UniverseWatch(on)) => {
            if let Some(on) = on {
                d.universe_watch_on.store(on, Ordering::Release);
            }
            let src = match d.driver_stations.lock().unwrap().as_ref() {
                Some((at, v)) if shm::now_ns().saturating_sub(*at) < DRIVER_STATIONS_FRESH_NS => {
                    format!(
                        "{} stations reported by the driver {:.1} s ago",
                        v.len(),
                        shm::now_ns().saturating_sub(*at) as f64 * 1e-9
                    )
                }
                _ => format!(
                    "no recent driver report: {}",
                    d.universe_connection
                        .lock()
                        .unwrap()
                        .status(std::time::Instant::now())
                ),
            };
            let following = d.universe_following.lock().unwrap().clone();
            format!(
                "ok universe watch {} ({src}){}",
                if d.universe_watch_on.load(Ordering::Acquire) {
                    "on"
                } else {
                    "off"
                },
                if following.is_empty() {
                    String::new()
                } else {
                    format!("; registration follows {following}")
                }
            )
        }
        Ok(Command::Tilt(v)) => {
            let mut unsaved = String::new();
            if let Some(deg) = v {
                d.tilt_mdeg
                    .store((deg * 1000.0).round() as i32, Ordering::Release);
                unsaved = save_settings(d);
                republish_settings(d);
            }
            format!("ok tilt: {}{unsaved}", tilt_text(tilt_deg(d)))
        }
        Ok(Command::Latency(v)) => {
            let mut unsaved = String::new();
            if let Some(ms) = v {
                d.extra_latency_ns
                    .store((ms * 1e6).round() as i64, Ordering::Release);
                unsaved = save_settings(d);
            }
            format!(
                "ok latency: {}{unsaved}",
                latency_text(d.extra_latency_ns.load(Ordering::Acquire))
            )
        }
        Ok(Command::AlignmentReload) => {
            // a shift measured against another registration does not apply to this one: until
            // the watch measures afresh (now), none
            let created = align::load_registration(&align::registration_path())
                .map(|r| r.created_unix_s)
                .ok();
            let shift = d
                .universe_shift
                .lock()
                .unwrap()
                .filter(|p| Some(p.0) == created)
                .map(|p| p.1);
            d.universe_recheck.store(true, Ordering::Release);
            let (alignment, notes) = align::load_with(shift.as_ref());
            publish_alignment(d, alignment);
            format!("ok {}", notes.join("; "))
        }
    }
}

fn current_opts(d: &Daemon) -> EngineOptions {
    d.opts.lock().unwrap().clone()
}

fn requested_engine_options(d: &Daemon) -> (EngineOptions, u64) {
    let opts = d.opts.lock().unwrap();
    (opts.clone(), d.engine_requested.load(Ordering::Acquire))
}

// The caller keeps the engine options locked through the status snapshot.
fn engine_is_ready(d: &Daemon, _opts: &EngineOptions) -> bool {
    let requested = d.engine_requested.load(Ordering::Acquire);
    requested != 0 && d.engine_applied.load(Ordering::Acquire) == requested
}

/// Installs new engine options; the engine thread reopens the engine with them.
fn switch_mode(d: &Daemon, opts: EngineOptions, mode: &str) -> String {
    {
        let mut current = d.opts.lock().unwrap();
        *current = opts;
        d.engine_requested.fetch_add(1, Ordering::AcqRel);
    }
    *d.mode.lock().unwrap() = mode.to_string();
    d.reopen.store(true, Ordering::Release);
    format!("ok {mode}")
}

/// The radio thread's real-time priority: on Linux SCHED_RR above every normal thread, below
/// PipeWire's (20) and the compositor's (90 and up).
const RADIO_RT_PRIORITY: u32 = 10;

/// Keep the inherited affinity unless the user explicitly chooses CPUs.
/// CV1TOUCH_CPUS accepts comma-separated CPU IDs and inclusive ranges, e.g. "2,4-6".
fn affinity_cpus(spec: &str, allowed: &[usize]) -> Result<Vec<usize>, String> {
    let mut selected = Vec::new();
    for part in spec.split(',') {
        let part = part.trim();
        let (first, last) = part.split_once('-').unwrap_or((part, part));
        let first = first
            .parse::<usize>()
            .map_err(|_| "CV1TOUCH_CPUS: expected CPU IDs or ranges".to_string())?;
        let last = last
            .parse::<usize>()
            .map_err(|_| "CV1TOUCH_CPUS: expected CPU IDs or ranges".to_string())?;
        if first > last || last >= MAX_CPUS {
            return Err("CV1TOUCH_CPUS: invalid CPU range".into());
        }
        for cpu in first..=last {
            if !allowed.contains(&cpu) {
                return Err(format!(
                    "CV1TOUCH_CPUS: CPU {cpu} is outside the thread's allowed affinity"
                ));
            }
            if !selected.contains(&cpu) {
                selected.push(cpu);
            }
        }
    }
    Ok(selected)
}

/// The most CPUs Linux's affinity calls take (CPU_SETSIZE).
const MAX_CPUS: usize = 1024;

fn affinity_setting() -> Result<Option<String>, String> {
    match std::env::var("CV1TOUCH_CPUS") {
        Ok(spec) => Ok(Some(spec)),
        Err(std::env::VarError::NotPresent) => Ok(None),
        Err(e) => Err(format!("CV1TOUCH_CPUS: {e}")),
    }
}

fn set_rt_affinity(thread: system::ThreadId) -> Result<(), String> {
    set_rt_affinity_with(thread, affinity_setting()?.as_deref())
}

fn set_rt_affinity_with(thread: system::ThreadId, spec: Option<&str>) -> Result<(), String> {
    let Some(spec) = spec else { return Ok(()) };
    let allowed = system::allowed_cpus(thread).map_err(|e| e.to_string())?;
    let selected = affinity_cpus(spec, &allowed)?;
    system::set_cpus(thread, &selected).map_err(|e| e.to_string())
}

/// OpenHMD's USB event thread's real-time priority: below the radio thread's.
const USB_RT_PRIORITY: u32 = 9;

/// Makes OpenHMD's USB event thread (cv1t-usb) real-time, on our real-time CPUs. It reaps and
/// resubmits every sensor's video transfers: preempted for more than their queue lasts, it lost
/// video on all of them at once.
fn make_usb_thread_realtime() {
    // the thread names itself once it runs, which may be just after the engine opens
    let found = (0..10).find_map(|i| {
        if i > 0 {
            thread::sleep(Duration::from_millis(50));
        }
        system::thread_named("cv1t-usb")
    });
    let Some(tid) = found else {
        log!("no cv1t-usb thread: OpenHMD's USB event thread stays as it is");
        return;
    };
    if let Err(err) = set_rt_affinity(tid) {
        log!("cv1t-usb keeps the process's CPUs ({err})");
    }
    match system::make_realtime(tid, USB_RT_PRIORITY) {
        Ok(how) => log!("cv1t-usb real-time ({how})"),
        Err(err) => log!("cv1t-usb stays normal priority ({err})"),
    }
}

/// The control endpoint's server; requests stop being handled at `stop`.
struct ControlServer {
    one_at_a_time: Arc<Mutex<()>>,
}

impl ControlServer {
    /// Waits for the request in progress, if any, and handles no other while the guard lives.
    fn stop(&self) -> std::sync::MutexGuard<'_, ()> {
        self.one_at_a_time.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// One of the MAX_CLIENTS client slots, free again when dropped (also when its thread never
/// started).
struct ClientSlot(Arc<std::sync::atomic::AtomicUsize>);

impl ClientSlot {
    fn take(clients: &Arc<std::sync::atomic::AtomicUsize>) -> Option<ClientSlot> {
        if clients.fetch_add(1, Ordering::AcqRel) >= MAX_CLIENTS {
            clients.fetch_sub(1, Ordering::AcqRel);
            return None;
        }
        Some(ClientSlot(Arc::clone(clients)))
    }
}

impl Drop for ClientSlot {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::AcqRel);
    }
}

/// What decided the runtime directory, so that a message can name the setting to change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RuntimeSetting {
    Option,
    Environment,
}

impl RuntimeSetting {
    fn name(self) -> &'static str {
        match self {
            RuntimeSetting::Option => "--runtime-dir",
            RuntimeSetting::Environment => "XDG_RUNTIME_DIR",
        }
    }
}

#[derive(Debug)]
struct ControlSocketPathError {
    path: PathBuf,
    long: ipc::PathTooLong,
    setting: RuntimeSetting,
}

impl ControlSocketPathError {
    fn log(&self) {
        log!(
            "cannot start: the path of its control socket (the file it listens on for commands) is too long ({} bytes; Linux allows at most {} bytes). The path is built from {}: use a shorter one.",
            self.long.bytes,
            self.long.limit,
            self.setting.name()
        );
        // Keep the full path even when it is larger than the bounded logger's line buffer.
        let text = self.path.to_string_lossy();
        let mut remaining = text.as_ref();
        let mut continued = "";
        while !remaining.is_empty() {
            // Even a maximum-size Linux argument or environment value, expanded by
            // lossy decoding, fits the queue when split near its 2048-byte line limit.
            let mut end = remaining.len().min(1900);
            while !remaining.is_char_boundary(end) {
                end -= 1;
            }
            log!("control socket path{continued}: {}", &remaining[..end]);
            remaining = &remaining[end..];
            continued = " (continued)";
        }
    }
}

fn control_socket_path(
    runtime: &Path,
    setting: RuntimeSetting,
) -> Result<PathBuf, ControlSocketPathError> {
    let path = runtime.join("control.sock");
    match ipc::path_too_long(&path) {
        Some(long) => Err(ControlSocketPathError {
            path,
            long,
            setting,
        }),
        None => Ok(path),
    }
}

#[derive(Default)]
struct InstanceOptions {
    shm_name: Option<String>,
    runtime_dir: Option<PathBuf>,
}

impl InstanceOptions {
    fn set_override(&mut self, option: &str, value: String) {
        match option {
            "--shm-name" => self.shm_name = Some(value),
            "--runtime-dir" => self.runtime_dir = Some(PathBuf::from(value)),
            _ => unreachable!(),
        }
    }

    #[cfg(unix)]
    fn with_container_control(
        &self,
        listeners: Vec<ipc::Listener>,
        endpoint: &std::path::Path,
    ) -> (Vec<ipc::Listener>, Option<PathBuf>) {
        if self.shm_name.is_none() && self.runtime_dir.is_none() {
            with_container_control(listeners, endpoint)
        } else {
            // An explicit instance must not receive the default driver's commands.
            (listeners, None)
        }
    }
}

#[cfg(unix)]
fn with_container_control(
    mut listeners: Vec<ipc::Listener>,
    endpoint: &std::path::Path,
) -> (Vec<ipc::Listener>, Option<PathBuf>) {
    let path = match ipc::Listener::bind_private(endpoint) {
        Ok(listener) => {
            listeners.push(listener);
            Some(endpoint.to_path_buf())
        }
        Err(e) => {
            log!(
                "control socket for Steam's container {}: {e}; using the usual socket only",
                endpoint.display()
            );
            None
        }
    };
    (listeners, path)
}

/// One accept thread per endpoint and at most MAX_CLIENTS client threads in total. Both
/// endpoints share the request lock; each client's lines are answered in order.
fn serve_control(d: Arc<Daemon>, listeners: Vec<ipc::Listener>) -> ControlServer {
    let clients = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let one_at_a_time = Arc::new(Mutex::new(()));
    let server = ControlServer {
        one_at_a_time: Arc::clone(&one_at_a_time),
    };
    for mut listener in listeners {
        let (d, clients, one_at_a_time) = (
            Arc::clone(&d),
            Arc::clone(&clients),
            Arc::clone(&one_at_a_time),
        );
        thread::Builder::new()
            .name("cv1t-control".into())
            .spawn(move || {
                let mut failing = None;
                loop {
                    let conn = match listener.accept(true) {
                        Ok(conn) => conn,
                        Err(e) => {
                            // once per failure, not at every retry
                            let why = e.to_string();
                            if failing.as_ref() != Some(&why) {
                                log!("control endpoint: {why}");
                                failing = Some(why);
                            }
                            thread::sleep(Duration::from_millis(100));
                            continue;
                        }
                    };
                    failing = None;
                    let Some(slot) = ClientSlot::take(&clients) else {
                        continue; // dropped: closed at once
                    };
                    let (d, one_at_a_time) = (Arc::clone(&d), Arc::clone(&one_at_a_time));
                    let spawned =
                        thread::Builder::new()
                            .name("cv1t-client".into())
                            .spawn(move || {
                                let _slot = slot;
                                serve_client(&d, conn, &one_at_a_time);
                            });
                    if spawned.is_err() {
                        log("control endpoint: no thread for a client");
                    }
                }
            })
            .expect("control thread");
    }
    server
}

fn record_stop_reply(status: &recorder::Status, scoped: bool) -> String {
    let stats = status.stats;
    let counts = format!(
        "records={} dropped={} write_errors={} bytes={}",
        stats.records, stats.dropped, stats.write_errors, stats.bytes
    );
    if !status.finished || status.error.is_some() || status.reason.is_some() {
        return format!("err {}; {counts}", status.message());
    }
    format!("ok {}{counts}", if scoped { "stopped " } else { "" })
}

/// Recording open/drain/sync can wait for storage. Only its own commands serialize, and a
/// competing recording command refuses at once. Status, haptics and daemon stop stay available.
fn handle_control(d: &Daemon, line: &str, one_at_a_time: &Mutex<()>) -> String {
    handle_control_recording(
        d,
        line,
        one_at_a_time,
        Recorder::reserve_with_frames,
        |reserved| reserved.start(shm::now_ns()),
    )
}

fn handle_control_recording<'a>(
    d: &'a Daemon,
    line: &str,
    one_at_a_time: &Mutex<()>,
    reserve: impl FnOnce(
        &'a Recorder,
        &std::path::Path,
        bool,
    ) -> Result<recorder::ReservedRecording<'a>, String>,
    start: impl FnOnce(recorder::ReservedRecording<'a>) -> Result<(), String>,
) -> String {
    let turn = one_at_a_time.lock().unwrap_or_else(|e| e.into_inner());
    if d.quit.load(Ordering::Acquire) {
        return "err daemon is stopping".into();
    }
    let command = control::parse(line);
    if matches!(
        &command,
        Ok(Command::RecordStart(..) | Command::RecordStop | Command::RecordStopIf(_))
    ) {
        let Ok(_record) = d.record_command.try_lock() else {
            // In particular, a lost start reply must not leave its recording active once a
            // blocked header write returns. Request the matching stop without waiting for I/O.
            let requested = match &command {
                Ok(Command::RecordStop) => d.rec.request_stop(None),
                Ok(Command::RecordStopIf(path)) => d.rec.request_stop(Some(path)),
                _ => false,
            };
            return if requested {
                "err recording stop requested; another recording command is still pending; check status and try again"
            } else {
                "err another recording command is still pending; check status and try again"
            }.into();
        };
        if let Ok(Command::RecordStart(path, frames)) = command {
            let reserved = match reserve(&d.rec, &path, frames) {
                Ok(reserved) => reserved,
                Err(error) => return format!("err {error}"),
            };
            drop(turn);
            return recording_started(d, frames, start(reserved));
        }
        drop(turn);
        handle(d, line)
    } else {
        handle(d, line)
    }
}

/// Both endpoints use the same bounded reader and shared dispatch lock.
fn serve_client(d: &Daemon, conn: ipc::Connection, one_at_a_time: &Mutex<()>) {
    control::serve_client(conn, |line| handle_control(d, line, one_at_a_time));
}

const USAGE: &str = "usage: cv1touchd [--tracker cv1touch|openhmd] [--radio-wait-ms N] [--hmd-leds on|off] [--ignore-sensor SERIAL]... [--calibration PATH | --provisional] [--record PATH] [--shm-name NAME] [--runtime-dir DIR]\n       cv1touchd --supervise [OPTIONS]   run it as a child process, restarted after a failure, its output in a log file\n       cv1touchd --help | --version\nAffinity: inherit by default; CV1TOUCH_CPUS=2,4-6 explicitly selects allowed CPUs.";

fn usage() -> ! {
    let _logging = daemon_log::init().unwrap_or_else(|_| process::exit(1));
    daemon_log::raw(format_args!("{USAGE}"));
    daemon_log::exit(2);
}

/// `--supervise`: runs the daemon with the other arguments as a child process, started again
/// after a failure, its output in a log file (supervise.rs). For Windows, where no service
/// manager does that; on Linux systemd runs the daemon (cv1touchd.service).
fn supervise(args: &[String]) -> i32 {
    if args.iter().any(|a| a == "--supervise") {
        usage();
    }
    // before any thread: as the daemon does, so Ctrl+C, closing the console or SIGTERM reaches
    // the supervisor as a request to stop, which it passes on to the daemon
    let requests = shutdown::Requests::take().unwrap_or_else(|e| {
        eprintln!("cv1touchd: cannot take over shutdown requests: {e}");
        process::exit(1);
    });
    // the last one, as the daemon takes it
    let option = args
        .iter()
        .rposition(|a| a == "--runtime-dir")
        .and_then(|i| args.get(i + 1));
    let setting = if option.is_some() {
        RuntimeSetting::Option
    } else {
        RuntimeSetting::Environment
    };
    let runtime = option.map_or_else(control::runtime_dir, PathBuf::from);
    let endpoint = control_socket_path(&runtime, setting).unwrap_or_else(|e| {
        let _logging = daemon_log::init().unwrap_or_else(|_| process::exit(1));
        e.log();
        daemon_log::exit(1);
    });
    if let Err(e) = platform::fs::create_private_dir_all(&runtime) {
        eprintln!("cv1touchd: {}: {e}", runtime.display());
        return 1;
    }
    // one supervisor per runtime directory, as there is one daemon
    let lock_path = runtime.join("supervisor.lock");
    let _lock = match ipc::lock_exclusive(&lock_path) {
        Ok(lock) => lock,
        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
            println!("cv1touchd: a supervisor is already running");
            return 0;
        }
        Err(e) => {
            eprintln!("cv1touchd: {}: {e}", lock_path.display());
            return 1;
        }
    };
    let mut log = match supervise::Log::open(&supervise::log_dir()) {
        Ok(log) => log,
        Err(e) => {
            eprintln!("cv1touchd: {}: {e}", supervise::log_dir().display());
            return 1;
        }
    };
    let exe = match std::env::current_exe() {
        Ok(exe) => exe,
        Err(e) => {
            eprintln!("cv1touchd: cannot find its own program: {e}");
            return 1;
        }
    };
    println!(
        "cv1touchd: supervising the daemon, its log in {}",
        log.path().display()
    );
    let daemon = || {
        let mut daemon = std::process::Command::new(&exe);
        daemon.args(args);
        daemon
    };
    let mut stopping = supervise::Stopping {
        requested: Box::new(|| requests.wait(Duration::ZERO)),
        ask: Box::new(|until| {
            control::request_until(
                &endpoint,
                "stop",
                until.min(std::time::Instant::now() + Duration::from_secs(2)),
            )
            .map_err(|e| e.to_string())
            .and_then(|reply| reply.starts_with("ok").then_some(()).ok_or(reply))
        }),
        within: Duration::from_secs(10),
    };
    supervise::run(daemon, &mut log, supervise::POLICY, &mut stopping)
}

fn random_u64() -> u64 {
    system::random_u64()
        .ok()
        .filter(|&v| v != 0)
        .unwrap_or_else(shm::now_ns)
}

#[derive(Clone)]
struct ControlEndpoints {
    runtime: PathBuf,
    #[cfg(unix)]
    container: Option<PathBuf>,
}

impl ControlEndpoints {
    fn remove(&self) {
        let _ = fs::remove_file(&self.runtime);
        #[cfg(unix)]
        if let Some(path) = &self.container {
            if let Err(e) = ipc::unlink_private_socket(path) {
                log(format!("removing control socket {}: {e}", path.display()));
            }
        }
    }
}

/// How often the startup wait looks at the recording header, so tracking starts within this of
/// the header being written rather than a whole heartbeat interval later.
const STARTUP_POLL: Duration = Duration::from_millis(10);
/// How often the startup wait refreshes the heartbeat the driver reads.
const STARTUP_HEARTBEAT: Duration = Duration::from_millis(200);

/// Keep control and the heartbeat available before opening hardware, even if startup storage
/// never responds. A header failure is fatal, including one a client has already acknowledged.
fn wait_for_recording_start(
    d: &Daemon,
    starting: &mut Option<recorder::Starting>,
    mut shutdown: impl FnMut(Duration) -> Option<String>,
) -> Result<bool, String> {
    let mut last_heartbeat: Option<std::time::Instant> = None;
    loop {
        let Some(pending) = starting.as_ref() else {
            return Ok(true);
        };
        if let Some(result) = pending.poll() {
            if result.is_ok() && !d.stop_requested.load(Ordering::Acquire) {
                *starting = None;
                return Ok(true);
            }
            return result.map(|()| false);
        }
        if last_heartbeat.is_none_or(|at| at.elapsed() >= STARTUP_HEARTBEAT) {
            d.shm.heartbeat(shm::now_ns());
            last_heartbeat = Some(std::time::Instant::now());
        }
        let why = shutdown(STARTUP_POLL);
        if why.is_some() || d.stop_requested.load(Ordering::Acquire) {
            // Cleanup owns the pending outcome after a shutdown request, including if another
            // client acknowledges and replaces this session before the final storage wait.
            log(format!(
                "{}: shutting down",
                why.as_deref().unwrap_or("asked to stop")
            ));
            return Ok(false);
        }
    }
}

/// Reserve before serving, then keep control available while the initial header is pending.
/// This owns the startup exit path too: main cannot accidentally ignore a fatal header result.
fn serve_with_startup_recording(
    d: Arc<Daemon>,
    listeners: Vec<ipc::Listener>,
    endpoints: &ControlEndpoints,
    start: impl FnOnce(&Recorder) -> Result<Option<recorder::Starting>, String>,
    shutdown: impl FnMut(Duration) -> Option<String>,
) -> ControlServer {
    let recording = start(&d.rec);
    if let Err(error) = &recording {
        log(error); // Reservation/thread-creation failures have no writer to report them.
    }
    let control = serve_control(Arc::clone(&d), listeners);
    let mut starting = None;
    let ready = recording.and_then(|recording| {
        starting = recording;
        wait_for_recording_start(&d, &mut starting, shutdown)
    });
    if ready != Ok(true) {
        daemon_log::exit(finish_recording_startup(
            &d,
            &control,
            endpoints,
            starting.as_ref(),
            ready.err(),
        ));
    }
    control
}

/// No engine threads exist yet. Retire explicitly: control threads still own the daemon, and
/// process exit cannot wait for a file that may be blocked even while closing after an error.
fn finish_recording_startup(
    d: &Daemon,
    control: &ControlServer,
    endpoints: &ControlEndpoints,
    starting: Option<&recorder::Starting>,
    error: Option<String>,
) -> i32 {
    let _no_more_requests = control.stop();
    d.quit.store(true, Ordering::Release);
    let status = d
        .rec
        .stop_timeout(None, RECORD_SHUTDOWN_WAIT)
        .unwrap_or_else(|| d.rec.status());
    // A pending header can fail during the final drain, including before a blocked close.
    // Retain its separate outcome and inspect the drain for later write or sync errors too.
    let late_error = starting
        .and_then(recorder::Starting::poll)
        .and_then(Result::err);
    let final_error = starting
        .and_then(recorder::Starting::final_status)
        .and_then(|status| status.error);
    let error = error.or(late_error).or(final_error);
    if status.path.is_some() && !status.finished {
        log("startup recording is still waiting for storage; file may be incomplete");
    }
    endpoints.remove();
    d.shm.retire();
    log("stopped");
    i32::from(error.is_some() || status.error.is_some())
}

fn finish_recording_shutdown(rec: &Recorder) {
    if let Some(status) = rec.stop_timeout(None, RECORD_SHUTDOWN_WAIT) {
        if !status.finished {
            log("recording is still waiting for storage; file may be incomplete");
        }
    }
}

fn log_running(
    d: &Daemon,
    shm_name: &str,
    socket: &Path,
    emit: impl FnOnce(std::fmt::Arguments<'_>),
) {
    // Control already serves status; do not nest its configuration locks or log with them held.
    let mode = d.mode.lock().unwrap().clone();
    let (hmd_leds_off, ignore_sensors) = {
        let opts = d.opts.lock().unwrap();
        (opts.hmd_leds_off, opts.ignore_sensors.clone())
    };
    emit(format_args!(
        "running: shm {shm_name}, control {}, {} mode, hmd leds {}, ignoring sensors {:?}",
        socket.display(),
        mode,
        if hmd_leds_off { "off" } else { "on" },
        ignore_sensors
    ));
}

fn main() {
    // These queries need neither a device, signal-mask changes, nor locked memory.
    let arguments: Vec<String> = std::env::args().skip(1).collect();
    if arguments.len() == 1 {
        match arguments[0].as_str() {
            "--help" | "-h" => {
                println!("{USAGE}");
                return;
            }
            "--version" | "-V" => {
                println!("cv1touchd {}", env!("CARGO_PKG_VERSION"));
                return;
            }
            _ => {}
        }
    }
    if arguments.first().map(String::as_str) == Some("--supervise") {
        process::exit(supervise(&arguments[1..]));
    }
    // Block shutdown signals before the writer or any engine thread can inherit them.
    let shutdown = shutdown::Requests::take();
    let _logging = daemon_log::init().unwrap_or_else(|_| process::exit(1));
    let shutdown = shutdown.unwrap_or_else(|e| {
        log!("cannot take over shutdown requests: {e}");
        daemon_log::exit(1);
    });
    let affinity = affinity_setting().and_then(|spec| {
        if let Some(spec) = spec {
            let allowed = system::allowed_cpus(system::current_thread())
                .map_err(|e| format!("CV1TOUCH_CPUS: {e}"))?;
            affinity_cpus(&spec, &allowed)?;
        }
        Ok(())
    });
    if let Err(e) = affinity {
        log!("{e}");
        daemon_log::exit(2);
    }
    if let Err(e) = system::opt_out_of_throttling() {
        log!("power throttling stays as Windows sets it ({e})");
    }
    match system::lock_memory() {
        Ok(()) => log!("memory locked: pages stay in RAM once touched"),
        Err(e) => log!("memory not locked ({e}): pages can be swapped out under pressure"),
    }
    let mut opts = EngineOptions {
        radio_wait_ms: 2,
        hmd_leds_off: DEFAULT_HMD_LEDS_OFF,
        ..Default::default()
    };
    let mut record: Option<PathBuf> = None;
    let mut calibration: Option<PathBuf> = None;
    let mut provisional = false;
    let mut instance = InstanceOptions::default();
    let mut tracker = TrackerKind::Rust;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--radio-wait-ms" => {
                opts.radio_wait_ms = args
                    .next()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or_else(|| usage())
            }
            "--hmd-leds" => {
                opts.hmd_leds_off = match args.next().as_deref() {
                    Some("on") => false,
                    Some("off") => true,
                    _ => usage(),
                }
            }
            "--ignore-sensor" => {
                let serial = args.next().unwrap_or_else(|| usage());
                if let Err(e) = modes::ignore_sensor(&mut opts, serial) {
                    log!("{e}");
                    daemon_log::exit(2);
                }
            }
            "--record" => record = Some(PathBuf::from(args.next().unwrap_or_else(|| usage()))),
            "--calibration" => {
                calibration = Some(PathBuf::from(args.next().unwrap_or_else(|| usage())))
            }
            "--provisional" => provisional = true,
            "--tracker" => {
                tracker = match args.next().as_deref() {
                    Some("cv1touch") => TrackerKind::Rust,
                    Some("openhmd") => TrackerKind::OpenHmd,
                    _ => usage(),
                }
            }
            "--shm-name" | "--runtime-dir" => {
                instance.set_override(&arg, args.next().unwrap_or_else(|| usage()))
            }
            _ => usage(),
        }
    }

    let shm_name = instance
        .shm_name
        .clone()
        .unwrap_or_else(|| shm::DEFAULT_NAME.to_string());
    let runtime_dir = instance
        .runtime_dir
        .clone()
        .unwrap_or_else(control::runtime_dir);
    let setting = if instance.runtime_dir.is_some() {
        RuntimeSetting::Option
    } else {
        RuntimeSetting::Environment
    };
    let sock_path = control_socket_path(&runtime_dir, setting).unwrap_or_else(|e| {
        e.log();
        daemon_log::exit(1);
    });
    let (opts, mode, calibration_path) = if provisional {
        (modes::provisional(&opts), "provisional".to_string(), None)
    } else {
        let path = calibration
            .clone()
            .unwrap_or_else(calib::file::default_path);
        if calibration.is_some() || path.exists() {
            match modes::calibrated(&opts, &path) {
                Ok((o, n)) => {
                    log!("calibrated: {n} sensor poses from {}", path.display());
                    (o, "calibrated".to_string(), Some(path))
                }
                Err(e) => {
                    log!("{e} (fix or remove it, or start with --provisional)");
                    daemon_log::exit(1);
                }
            }
        } else {
            log!(
                "no calibration at {}: provisional placement (run `cv1touch setup`)",
                path.display()
            );
            (modes::provisional(&opts), "provisional".to_string(), None)
        }
    };

    if let Err(e) = cv1touch::platform::fs::create_private_dir_all(&runtime_dir) {
        log!("{}: {e}", runtime_dir.display());
        daemon_log::exit(1);
    }
    // Lock before touching shared memory or the socket: a second instance must not clobber the first.
    let lock_path = runtime_dir.join("daemon.lock");
    let _lock = control::lock_single_instance(&lock_path).unwrap_or_else(|e| {
        log!("another cv1touchd holds {} ({e})", lock_path.display());
        daemon_log::exit(1);
    });
    let shm = ShmWriter::create(&shm_name, random_u64()).unwrap_or_else(|e| {
        log!("shared memory {shm_name}: {e}");
        daemon_log::exit(1);
    });
    let settings_path = settings::default_path();
    let settings = settings::load(&settings_path).unwrap_or_else(|e| {
        log!("settings: {e}; using the defaults");
        settings::Settings::default()
    });
    if (settings.prediction_percent, settings.rotation_percent()) != (100, 100) {
        log!(
            "prediction: {} ({})",
            prediction_text(settings.prediction_percent, settings.rotation_percent()),
            settings_path.display()
        );
    }
    if start_latency_ns(&settings) != 0 {
        log!(
            "latency: {} ({})",
            latency_text(start_latency_ns(&settings)),
            settings_path.display()
        );
    }
    let start_tilt_deg = start_tilt_mdeg(&settings) as f64 / 1000.0;
    if start_tilt_deg != 0.0 {
        log!(
            "tilt: {} ({})",
            tilt_text(start_tilt_deg),
            settings_path.display()
        );
    }
    let (alignment, notes) = align::load_default();
    for n in notes {
        log(n);
    }
    let fitted_render_orient = alignment.render_orient;
    shm.publish_alignment(&with_prediction(
        with_tilt(alignment, &fitted_render_orient, start_tilt_deg),
        start_prediction(&settings),
    ));
    let rec = Recorder::with_diagnostics(RECORDER_RING_BYTES, |message| log(message));
    // a stale socket goes (the lock guarantees no live owner); private to this user
    let listener = ipc::Listener::bind(&sock_path).unwrap_or_else(|e| {
        log!("control socket {}: {e}", sock_path.display());
        daemon_log::exit(1);
    });

    let listeners = vec![listener];
    #[cfg(unix)]
    // The shared-memory ownership lock above also protects this default driver's endpoint.
    let (listeners, container_socket) =
        instance.with_container_control(listeners, &ipc::container_socket_path());

    let mut inputs = InputRuntime::default();
    match input_calibration::load(&input_calibration::default_path()) {
        Ok(calibration) => inputs.calibration = calibration,
        Err(error) => {
            log!("{error}; using factory input defaults");
            inputs.error = Some(error);
        }
    }
    let d = Arc::new(Daemon {
        shm,
        rec,
        record_command: Mutex::new(()),
        flight: FlightRecorder::new(),
        quit: AtomicBool::new(false),
        failed_worker: AtomicU8::new(0),
        stop_requested: AtomicBool::new(false),
        haptics: Mutex::new([None; 2]),
        session: Mutex::new(recorder::SessionInfo::default()),
        opts: Mutex::new(opts),
        engine_requested: std::sync::atomic::AtomicU64::new(1),
        engine_applied: std::sync::atomic::AtomicU64::new(0),
        mode: Mutex::new(mode),
        reopen: AtomicBool::new(false),
        tracker,
        track_in: Mutex::new(FrameQueue::default()),
        sensors: Mutex::new(Vec::new()),
        sensor_streams: Mutex::new(Vec::new()),
        sensor_video: Mutex::new(Vec::new()),
        track_stats: Mutex::new([(Stats::default(), false); 2]),
        pending_frames_dropped: std::sync::atomic::AtomicU64::new(0),
        landmark_shifts: Mutex::new(Default::default()),
        sensor_poses: Mutex::new(Default::default()),
        sensor_agreement: Mutex::new(Default::default()),
        sensors_unsure: AtomicBool::new(false),
        calibration: Mutex::new(calibration_path.map(LoadedCalibration::at)),
        own_backups: Mutex::new(Vec::new()),
        universe_shift: Mutex::new(None),
        universe_recheck: AtomicBool::new(false),
        driver_stations: Mutex::new(None),
        universe_connection: Mutex::new(UniverseConnection::default()),
        universe_watch_on: AtomicBool::new(true),
        universe_following: Mutex::new(String::new()),
        shaping_on: AtomicBool::new(true),
        position_prediction: AtomicU32::new(start_prediction(&settings).0),
        rotation_prediction: AtomicU32::new(start_prediction(&settings).1),
        alignment_publish: Mutex::new(()),
        settings_path,
        record_frames: AtomicBool::new(false),
        extra_latency_ns: std::sync::atomic::AtomicI64::new(start_latency_ns(&settings)),
        tilt_mdeg: std::sync::atomic::AtomicI32::new(start_tilt_mdeg(&settings)),
        fitted_render_orient: Mutex::new(fitted_render_orient),
        battery: [
            std::sync::atomic::AtomicI32::new(-1),
            std::sync::atomic::AtomicI32::new(-1),
        ],
        radio_wake: hid::Waker::new().expect("radio waker"),
        radio_on: AtomicBool::new(false),
        inject: Injection::default(),
        inputs: Mutex::new(inputs),
    });
    let endpoints = ControlEndpoints {
        runtime: sock_path.clone(),
        #[cfg(unix)]
        container: container_socket,
    };
    let control = serve_with_startup_recording(
        Arc::clone(&d),
        listeners,
        &endpoints,
        |rec| {
            record
                .as_deref()
                .map(|path| rec.start_in_background(path, shm::now_ns()))
                .transpose()
        },
        |within| shutdown.wait(within),
    );

    let dropout_writer = match flight_writer::Writer::start(platform::dirs::get().state, log) {
        Ok(writer) => Some(writer),
        Err(e) => {
            log!("could not start the dropout recording thread: {e}");
            None
        }
    };
    let engine_thread = {
        let d = Arc::clone(&d);
        thread::Builder::new()
            .name("cv1t-engine".into())
            .spawn(move || {
                watch_worker(&d.failed_worker, ENGINE_WORKER, false, || {
                    engine_main(Arc::clone(&d));
                });
            })
            .expect("engine thread")
    };
    let universe_thread = {
        let d = Arc::clone(&d);
        thread::Builder::new()
            .name("cv1t-universe".into())
            .spawn(move || universe_watch(d))
            .expect("universe thread")
    };
    log_running(&d, &shm_name, &sock_path, daemon_log::line);

    loop {
        exit_on_worker_failure(&d.failed_worker, &d.shm);
        // the daemon's liveness for the driver: opening the engine (the radio wait) blocks the
        // engine thread for seconds, which must not read as a dead daemon; stale poses are the
        // driver's own concern (sample age)
        d.shm.heartbeat(shm::now_ns());
        save_dropouts(&d, dropout_writer.as_ref());
        if let Some(why) = shutdown.wait(Duration::from_millis(200)) {
            log!("{why}: shutting down");
            break;
        }
        if d.stop_requested.load(Ordering::Acquire) {
            log!("asked to stop: shutting down");
            break;
        }
    }

    // nothing a client asks for runs while the daemon stops
    let _no_more_requests = control.stop();
    d.quit.store(true, Ordering::Release);
    let _ = engine_thread.join();
    let _ = universe_thread.join();
    finish_recording_shutdown(&d.rec);
    endpoints.remove();
    // the control threads still hold `d`, so retire the segment here rather than on its drop
    d.shm.retire();
    log!("stopped");
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[cfg(unix)]
    #[test]
    fn control_socket_paths_allow_107_bytes_and_reject_108_including_utf8() {
        for runtime in [
            format!("/{}", "r".repeat(93)),
            format!("/{}r", "é".repeat(46)),
        ] {
            assert!(control_socket_path(Path::new(&runtime), RuntimeSetting::Option).is_ok());
        }
        for runtime in [
            format!("/{}", "r".repeat(94)),
            format!("/{}", "é".repeat(47)),
        ] {
            let error =
                control_socket_path(Path::new(&runtime), RuntimeSetting::Environment).unwrap_err();
            assert_eq!(error.long.bytes, 108);
            assert_eq!(error.long.limit, 107);
            assert_eq!(error.setting, RuntimeSetting::Environment);
            assert_eq!(error.path, PathBuf::from(format!("{runtime}/control.sock")));
        }
    }

    #[test]
    fn a_superseded_unknown_window_breaks_a_pending_landmark_alert() {
        let d = status_daemon();
        let reference = [100.0, 500.0].map(|x| track::landmarks::Landmark {
            serial: "A".into(),
            x,
            y: 300.0,
            seen: 0.1,
        });
        let mut watch = LandmarkWatch {
            monitor: track::landmarks::BackgroundMonitor::new(
                reference.into(),
                LANDMARK_WINDOW_NS,
                20,
            )
            .unwrap(),
            alerts: Default::default(),
            missing: Default::default(),
        };
        for (window, shift) in [0.7, 0.0, 0.9, 1.1].into_iter().enumerate() {
            let window = window as u64;
            if window != 1 {
                for k in 0..100 {
                    watch.monitor.frame(
                        "A",
                        1 + window * LANDMARK_WINDOW_NS + k * LANDMARK_WINDOW_NS / 100,
                        [
                            nalgebra::Vector2::new(100.0 + shift, 300.0),
                            nalgebra::Vector2::new(500.0 + shift, 300.0),
                        ],
                    );
                }
            }
            let now = 1 + (window + 1) * LANDMARK_WINDOW_NS;
            if window == 2 {
                // Production ordering must supersede the held unknown before accepting
                // this movement, even if the worker completes it immediately.
                watch.poll(&d, now);
            } else {
                watch.monitor.tick(now);
            }
            // Gate publication after the real worker finishes. The second window has
            // no samples: hold its unknown until the following window is overdue.
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            if window == 2 {
                while !d
                    .landmark_shifts
                    .lock()
                    .unwrap()
                    .get("A")
                    .is_some_and(|s| (s.0 - 0.9).abs() < 1e-8)
                {
                    watch.poll(&d, now);
                    assert!(
                        std::time::Instant::now() < deadline,
                        "landmark worker timed out"
                    );
                    thread::sleep(Duration::from_millis(1));
                }
            } else {
                while watch.monitor.recycle() == 0 {
                    assert!(
                        std::time::Instant::now() < deadline,
                        "landmark worker timed out"
                    );
                    thread::sleep(Duration::from_millis(1));
                }
            }
            if window == 1 {
                assert_eq!(watch.alerts["A"].over, 1);
                assert!(!watch.missing.contains_key("A"));
                assert!(d.landmark_shifts.lock().unwrap().contains_key("A"));
                continue;
            }
            watch.poll(&d, now);
            if window < 3 {
                assert!(
                    watch.alerts["A"].active.is_none(),
                    "a movement alert crossed the unpublished unknown window"
                );
                assert_eq!(watch.alerts["A"].over, 1);
            } else {
                assert!(watch.alerts["A"].active.is_some());
            }
        }
    }

    #[test]
    fn an_overdue_unknown_window_prevents_a_delayed_recovery() {
        for delayed_frame in [false, true] {
            let d = status_daemon();
            let reference = [100.0, 500.0].map(|x| track::landmarks::Landmark {
                serial: "A".into(),
                x,
                y: 300.0,
                seen: 0.1,
            });
            let mut watch = LandmarkWatch {
                monitor: track::landmarks::BackgroundMonitor::new(
                    reference.into(),
                    LANDMARK_WINDOW_NS,
                    20,
                )
                .unwrap(),
                alerts: Default::default(),
                missing: Default::default(),
            };
            let alert = watch.alerts.entry("A".into()).or_default();
            alert.update(Some((1.2, 2)));
            alert.update(Some((1.2, 2)));
            d.landmark_shifts
                .lock()
                .unwrap()
                .insert("A".into(), (1.2, 0.0, 2));
            for k in 0..100 {
                watch.monitor.frame(
                    "A",
                    1 + k * LANDMARK_WINDOW_NS / 100,
                    [
                        nalgebra::Vector2::new(100.0, 300.0),
                        nalgebra::Vector2::new(500.0, 300.0),
                    ],
                );
            }
            watch.monitor.tick(1 + LANDMARK_WINDOW_NS);
            // Hold a completed zero-shift result without publishing it. This reclaims the
            // buffer only; LandmarkWatch's production poll controls result acceptance.
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            while watch.monitor.recycle() == 0 {
                assert!(
                    std::time::Instant::now() < deadline,
                    "landmark worker timed out"
                );
                thread::sleep(Duration::from_millis(1));
            }
            // The following window exceeds the documented 65,536-sample capacity.
            watch.monitor.frame(
                "A",
                2 + LANDMARK_WINDOW_NS,
                std::iter::repeat_n(nalgebra::Vector2::new(100.0, 300.0), 65_537),
            );
            if delayed_frame {
                let mut info = engine::FrameInfo::default();
                info.serial[0] = b'A' as _;
                info.frame_start_host_ns = 2 + LANDMARK_WINDOW_NS;
                watch.frame(&info, &[]);
                assert!(
                    watch.alerts["A"].active.is_some(),
                    "a delayed frame used its old timestamp to publish a recovery"
                );
            }
            watch.poll(&d, 1 + 2 * LANDMARK_WINDOW_NS);
            assert!(!d.landmark_shifts.lock().unwrap().contains_key("A"));
            assert!(
                watch.alerts["A"].active.is_some(),
                "stale recovery cleared the alert before the overdue unknown window"
            );
        }
    }

    #[test]
    fn unknown_landmark_matches_clear_status_without_clearing_an_alert() {
        let d = status_daemon();
        let reference = track::landmarks::Landmark {
            serial: "A".into(),
            x: 100.0,
            y: 300.0,
            seen: 0.1,
        };
        let mut watch = LandmarkWatch {
            monitor: track::landmarks::BackgroundMonitor::new(
                vec![reference],
                LANDMARK_WINDOW_NS,
                20,
            )
            .unwrap(),
            alerts: Default::default(),
            missing: Default::default(),
        };
        let alert = watch.alerts.entry("A".into()).or_default();
        alert.update(Some((1.2, 1)));
        alert.update(Some((1.2, 1)));
        let mut info = engine::FrameInfo::default();
        info.serial[0] = b'A' as _;
        for (window, xs) in [(0, vec![101.2]), (1, vec![99.4, 101.2])] {
            let blobs: Vec<_> = xs
                .into_iter()
                .map(|x| Blob {
                    x,
                    y: 300.0,
                    led_id: -1,
                    ..Default::default()
                })
                .collect();
            for k in 0..=100 {
                info.frame_start_host_ns =
                    1 + window * LANDMARK_WINDOW_NS + k * LANDMARK_WINDOW_NS / 100;
                watch.frame(&info, &blobs);
                watch.poll(&d, info.frame_start_host_ns);
            }
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            while if window == 0 {
                !d.landmark_shifts.lock().unwrap().contains_key("A")
            } else {
                !watch.missing.contains_key("A")
            } {
                watch.poll(&d, info.frame_start_host_ns);
                assert!(
                    std::time::Instant::now() < deadline,
                    "landmark worker timed out"
                );
                thread::sleep(Duration::from_millis(1));
            }
            assert_eq!(
                d.landmark_shifts.lock().unwrap().contains_key("A"),
                window == 0
            );
            assert!(watch.alerts["A"].active.is_some());
        }
    }

    #[test]
    fn landmark_visibility_changes_do_not_report_recovery() {
        let mut alert = LandmarkAlert::default();
        let notices: Vec<_> = [2, 2, 1, 2, 2, 1]
            .into_iter()
            .filter_map(|landmarks| alert.update(Some((0.7, landmarks))))
            .collect();
        assert_eq!(notices, [LandmarkNotice::Moved]);
        assert_eq!(alert.update(Some((0.1, 1))), None);
        assert_eq!(alert.update(Some((0.3, 2))), Some(LandmarkNotice::Back));
        assert_eq!(alert.update(Some((0.3, 2))), None);
        assert_eq!(alert.update(Some((0.7, 2))), None);
        assert_eq!(alert.update(Some((0.7, 2))), Some(LandmarkNotice::Moved));
    }

    #[test]
    fn missing_landmarks_break_a_pending_movement_streak() {
        let mut alert = LandmarkAlert::default();
        assert_eq!(alert.update(Some((0.7, 2))), None);
        for _ in 0..3 {
            assert_eq!(alert.update(None), None);
        }
        assert_eq!(alert.update(Some((0.7, 2))), None);
        assert_eq!(alert.update(Some((0.7, 2))), Some(LandmarkNotice::Moved));
    }

    #[test]
    fn an_active_landmark_alert_survives_missing_windows() {
        let mut alert = LandmarkAlert::default();
        assert_eq!(alert.update(Some((0.7, 2))), None);
        assert_eq!(alert.update(Some((0.7, 2))), Some(LandmarkNotice::Moved));
        for _ in 0..3 {
            assert_eq!(alert.update(None), None);
        }
        assert_eq!(alert.update(Some((0.7, 2))), None);
        assert_eq!(alert.update(Some((0.1, 1))), None);
        assert_eq!(alert.update(Some((0.3, 2))), Some(LandmarkNotice::Back));
        assert_eq!(alert.update(Some((0.3, 2))), None);
    }

    #[test]
    fn landmark_recovery_needs_the_activation_landmark_count() {
        let mut alert = LandmarkAlert::default();
        assert_eq!(alert.update(Some((0.7, 4))), None);
        assert_eq!(alert.update(Some((0.7, 4))), Some(LandmarkNotice::Moved));
        assert_eq!(alert.update(Some((0.1, 3))), None);
        assert_eq!(alert.update(Some((0.1, 4))), Some(LandmarkNotice::Back));
    }

    #[test]
    fn landmark_alert_can_tighten_but_fewer_landmarks_cannot_relax_it() {
        let mut alert = LandmarkAlert::default();
        assert_eq!(alert.update(Some((1.2, 1))), None);
        assert_eq!(alert.update(Some((1.2, 1))), Some(LandmarkNotice::Moved));
        assert_eq!(alert.update(Some((0.7, 2))), None);
        assert_eq!(alert.update(Some((0.1, 1))), None);
        assert_eq!(alert.update(Some((0.5, 2))), None);
        assert_eq!(alert.update(Some((0.3, 2))), Some(LandmarkNotice::Back));
    }

    #[test]
    fn landmark_movement_is_reported_once_until_recovery() {
        let mut alert = LandmarkAlert::default();
        assert_eq!(alert.update(Some((0.5, 2))), None);
        assert_eq!(alert.update(Some((0.5, 2))), Some(LandmarkNotice::Moved));
        for magnitude in [0.8, 0.6, 1.1] {
            assert_eq!(alert.update(Some((magnitude, 2))), None);
        }
        assert_eq!(alert.update(Some((0.1, 2))), Some(LandmarkNotice::Back));
        assert_eq!(alert.update(Some((0.1, 2))), None);
        assert_eq!(alert.update(Some((0.5, 2))), None);
        assert_eq!(alert.update(Some((0.5, 2))), Some(LandmarkNotice::Moved));
    }

    #[test]
    fn landmark_noise_needs_two_windows_over_the_applicable_threshold() {
        let mut alert = LandmarkAlert::default();
        for _ in 0..3 {
            assert_eq!(alert.update(Some((0.7, 1))), None);
        }
        assert_eq!(alert.update(Some((0.5, 2))), None);
        assert_eq!(alert.update(Some((0.3, 2))), None);
        assert_eq!(alert.update(Some((0.5, 2))), None);
        assert_eq!(alert.update(Some((0.5, 2))), Some(LandmarkNotice::Moved));
    }

    #[test]
    fn universe_watch_status_waits_for_the_first_check() {
        let d = status_daemon();
        assert!(handle(&d, "universe-watch").contains("waiting to check SteamVR"));
    }

    #[test]
    fn universe_watch_status_reports_the_state_and_remaining_wait() {
        let d = status_daemon();
        let start = std::time::Instant::now();
        for (state, seconds, text) in [
            (
                UniverseConnectionState::Stopped,
                10,
                "waiting for SteamVR to start; next check in 10 s",
            ),
            (
                UniverseConnectionState::CheckFailed,
                10,
                "cannot check whether SteamVR is running; next check in 10 s",
            ),
            (
                UniverseConnectionState::ConnectFailed,
                60,
                "retrying SteamVR; next in 60 s",
            ),
            (
                UniverseConnectionState::Connected,
                60,
                "SteamVR connection succeeded; checking once a minute, next in 60 s",
            ),
        ] {
            let connection = UniverseConnection {
                state,
                next_attempt: Some(start + Duration::from_secs(seconds)),
                ..Default::default()
            };
            assert_eq!(connection.status(start + Duration::from_millis(500)), text);
            *d.universe_connection.lock().unwrap() = connection;
            let reply = handle(&d, "universe-watch");
            let label = text.split(';').next().unwrap();
            assert!(reply.contains(label), "{reply}");
        }
        let connection = UniverseConnection {
            state: UniverseConnectionState::ConnectFailed,
            next_attempt: Some(start + Duration::from_secs(60)),
            ..Default::default()
        };
        assert_eq!(
            connection.status(start + Duration::from_secs(61)),
            "retrying SteamVR; next in 0 s"
        );
    }

    #[test]
    fn universe_watch_keeps_failed_retries_across_fresh_then_stale_driver_reports() {
        let mut connection = UniverseConnection::default();
        let start = std::time::Instant::now();
        let calls = std::cell::Cell::new(0);
        let messages = std::cell::RefCell::new(Vec::new());
        let stations = vec![("base-a".into(), nalgebra::Isometry3::identity())];
        let mut tick = |seconds, reported| {
            connection.stations(
                start + Duration::from_secs(seconds),
                seconds * 1_000_000_000,
                reported,
                || Ok(true),
                || {
                    calls.set(calls.get() + 1);
                    Err("SteamVR is still starting".into())
                },
                |s| messages.borrow_mut().push(s),
            )
        };
        assert_eq!(tick(0, None), None);
        assert_eq!(calls.get(), 1);
        let report = Some((1_000_000_000, stations.clone()));
        assert_eq!(tick(1, report.clone()), Some(stations.clone()));
        assert_eq!(tick(10, report.clone()), Some(stations.clone()));
        assert_eq!(tick(11, report), None);
        assert_eq!(calls.get(), 1, "a fresh report must not erase the deadline");
        tick(59, None);
        assert_eq!(calls.get(), 1);
        tick(60, None);
        assert_eq!(calls.get(), 2);
        let report = Some((61_000_000_000, stations.clone()));
        assert_eq!(tick(61, report.clone()), Some(stations));
        tick(71, report);
        tick(179, None);
        assert_eq!(
            calls.get(),
            2,
            "the longer backoff must survive reports too"
        );
        tick(180, None);
        assert_eq!(calls.get(), 3);
        assert_eq!(
            messages.borrow().len(),
            1,
            "reports must not repeat warnings"
        );
    }

    #[test]
    fn universe_fallback_checks_processes_only_when_due() {
        let mut connection = UniverseConnection::default();
        let start = std::time::Instant::now();
        let checks = std::cell::Cell::new(0);
        let calls = std::cell::Cell::new(0);
        let succeeds = std::cell::Cell::new(false);
        let mut tick = |seconds, running| {
            connection.connect(
                start + Duration::from_secs(seconds),
                || {
                    checks.set(checks.get() + 1);
                    Ok(running)
                },
                || -> Result<(), String> {
                    calls.set(calls.get() + 1);
                    if succeeds.get() {
                        Ok(())
                    } else {
                        Err("SteamVR is still starting".into())
                    }
                },
                |_| {},
            )
        };
        for seconds in 0..40 {
            tick(seconds, false);
            assert_eq!(checks.get(), seconds as usize / 10 + 1, "at {seconds} s");
            assert_eq!(calls.get(), 0);
        }
        tick(40, true);
        assert_eq!((checks.get(), calls.get()), (5, 1));
        for seconds in 41..100 {
            tick(seconds, true);
            assert_eq!((checks.get(), calls.get()), (5, 1));
        }
        tick(100, true);
        assert_eq!((checks.get(), calls.get()), (6, 2));
        for seconds in 101..220 {
            tick(seconds, false);
            assert_eq!((checks.get(), calls.get()), (6, 2));
        }
        tick(220, false);
        assert_eq!((checks.get(), calls.get()), (7, 2));
        for seconds in 221..230 {
            tick(seconds, true);
            assert_eq!((checks.get(), calls.get()), (7, 2));
        }
        tick(230, true);
        assert_eq!((checks.get(), calls.get()), (8, 3));
        tick(289, true);
        assert_eq!((checks.get(), calls.get()), (8, 3));
        tick(290, true);
        assert_eq!(
            (checks.get(), calls.get()),
            (9, 4),
            "a stop reset the backoff"
        );
        succeeds.set(true);
        tick(410, true);
        assert_eq!((checks.get(), calls.get()), (10, 5));
        for seconds in 411..470 {
            tick(seconds, false);
            assert_eq!((checks.get(), calls.get()), (10, 5));
        }
        tick(470, false);
        assert_eq!((checks.get(), calls.get()), (11, 5));
    }

    #[test]
    fn universe_fallback_never_connects_while_steamvr_is_stopped() {
        let mut connection = UniverseConnection::default();
        let start = std::time::Instant::now();
        let calls = std::cell::Cell::new(0);
        let messages = std::cell::RefCell::new(Vec::new());
        let mut tick = |seconds, running| {
            connection.connect(
                start + Duration::from_secs(seconds),
                || Ok(running),
                || {
                    calls.set(calls.get() + 1);
                    Ok(())
                },
                |s| messages.borrow_mut().push(s),
            )
        };
        for seconds in [0, 60, 3600, 86_400] {
            assert_eq!(tick(seconds, false), None);
        }
        assert_eq!(calls.get(), 0);
        assert_eq!(messages.borrow().len(), 1, "one message while stopped");
        assert_eq!(tick(86_401, true), None);
        assert_eq!(calls.get(), 0);
        assert_eq!(tick(86_410, true), Some(()));
        assert_eq!(calls.get(), 1, "a start is noticed at the next 10 s check");
    }

    #[test]
    fn universe_fallback_backs_off_and_resets_after_success_or_stop() {
        let mut connection = UniverseConnection::default();
        let start = std::time::Instant::now();
        let calls = std::cell::Cell::new(0);
        let messages = std::cell::RefCell::new(Vec::new());
        let mut tick = |seconds, running, succeeds| {
            connection.connect(
                start + Duration::from_secs(seconds),
                || Ok(running),
                || {
                    calls.set(calls.get() + 1);
                    if succeeds {
                        Ok(())
                    } else {
                        Err("SteamVR is still starting".to_string())
                    }
                },
                |s| messages.borrow_mut().push(s),
            )
        };
        let mut expected = 0;
        for (seconds, due) in [
            (0, true),
            (59, false),
            (60, true),
            (179, false),
            (180, true),
            (419, false),
            (420, true),
            (899, false),
            (900, true),
            (1799, false),
            (1800, true),
            (2699, false),
            (2700, true),
        ] {
            assert_eq!(tick(seconds, true, false), None);
            expected += usize::from(due);
            assert_eq!(calls.get(), expected, "at {seconds} s");
        }
        assert_eq!(
            messages.borrow().len(),
            1,
            "retries do not repeat a warning"
        );
        assert_eq!(tick(3600, true, true), Some(()));
        expected += 1;
        assert_eq!(tick(3659, true, false), None);
        assert_eq!(calls.get(), expected, "success keeps the minute interval");
        tick(3660, true, false);
        expected += 1;
        tick(3719, true, false);
        assert_eq!(calls.get(), expected);
        tick(3720, true, false);
        expected += 1;
        assert_eq!(
            calls.get(),
            expected,
            "success reset the failed retry delay"
        );
        tick(3721, false, false);
        assert_eq!(calls.get(), expected, "stopping never initializes OpenVR");
        tick(3839, true, false);
        assert_eq!(calls.get(), expected, "the stop is checked only when due");
        tick(3840, false, false);
        tick(3849, true, false);
        assert_eq!(calls.get(), expected);
        tick(3850, true, false);
        expected += 1;
        assert_eq!(calls.get(), expected, "stopping reset the pending retry");
        tick(3909, true, false);
        assert_eq!(calls.get(), expected);
        tick(3910, true, false);
        assert_eq!(calls.get(), expected + 1, "stopping reset the retry delay");
        assert_eq!(messages.borrow().len(), 5, "only state changes are logged");
    }

    #[test]
    fn universe_fallback_does_not_connect_when_processes_cannot_be_checked() {
        let mut connection = UniverseConnection::default();
        let start = std::time::Instant::now();
        let mut messages = Vec::new();
        for seconds in [0, 60, 3600] {
            assert_eq!(
                connection.connect(
                    start + Duration::from_secs(seconds),
                    || Err(io::Error::other("process list unavailable")),
                    || -> Result<(), String> { panic!("must not initialize OpenVR") },
                    |s| messages.push(s),
                ),
                None
            );
        }
        assert_eq!(messages.len(), 1);
    }

    #[test]
    fn placement_history_keeps_only_the_window_used_for_retrying() {
        let mut placements = std::collections::BTreeMap::new();
        placements.insert("sensor-old".into(), vec![0]);
        for n in 1..2000 {
            let now = n * (PLACEMENTS_WINDOW_NS + 1);
            prune_placements(&mut placements, now);
            let history = placements.entry("sensor-example".into()).or_default();
            history.push(now);
            assert_eq!(history, &[now]);
            assert!(!placements.contains_key("sensor-old"));
        }
        placements.insert("sensor-boundary".into(), vec![0, 1, PLACEMENTS_WINDOW_NS]);
        prune_placements(&mut placements, PLACEMENTS_WINDOW_NS + 1);
        assert_eq!(placements["sensor-boundary"], [1, PLACEMENTS_WINDOW_NS]);
    }

    #[test]
    fn an_engine_waiting_for_devices_is_live_but_its_exit_is_not() {
        let failed = Arc::new(AtomicU8::new(0));
        let (started, waiting) = std::sync::mpsc::channel();
        let (release, done) = std::sync::mpsc::channel();
        let flag = Arc::clone(&failed);
        let worker = thread::spawn(move || {
            watch_worker(&flag, ENGINE_WORKER, false, || {
                started.send(()).unwrap();
                done.recv().unwrap();
            })
        });
        waiting.recv().unwrap();
        assert_eq!(failed.load(Ordering::Acquire), 0);
        release.send(()).unwrap();
        worker.join().unwrap();
        assert_eq!(failed.load(Ordering::Acquire), ENGINE_WORKER);
    }

    #[test]
    fn expected_radio_returns_are_not_failures_but_panics_are() {
        let failed = AtomicU8::new(0);
        watch_worker(&failed, RADIO_WORKER, true, || {});
        assert_eq!(failed.load(Ordering::Acquire), 0);
        thread::scope(|scope| {
            let worker =
                scope.spawn(|| watch_worker(&failed, RADIO_WORKER, true, || panic!("radio test")));
            assert!(worker.join().is_err());
        });
        assert_eq!(failed.load(Ordering::Acquire), RADIO_WORKER);
    }

    struct BlockingTeardown {
        entered: std::sync::mpsc::Sender<()>,
        release: std::sync::mpsc::Receiver<()>,
    }

    impl Drop for BlockingTeardown {
        fn drop(&mut self) {
            self.entered.send(()).unwrap();
            self.release.recv().unwrap();
        }
    }

    fn failure_during_teardown(
        run: impl FnOnce(Arc<AtomicU8>, BlockingTeardown) + Send + 'static,
    ) -> (u8, bool) {
        let failed = Arc::new(AtomicU8::new(0));
        let flag = Arc::clone(&failed);
        let (entered, dropping) = std::sync::mpsc::channel();
        let (release, blocked) = std::sync::mpsc::channel();
        let worker = thread::spawn(move || {
            run(
                flag,
                BlockingTeardown {
                    entered,
                    release: blocked,
                },
            )
        });
        let reached_drop = dropping.recv_timeout(Duration::from_secs(5)).is_ok();
        let marked = failed.load(Ordering::Acquire);
        // Always release teardown before asserting, including with the guard moved too late.
        let _ = release.send(());
        let panicked = worker.join().is_err();
        assert!(reached_drop, "native teardown did not start");
        (marked, panicked)
    }

    #[test]
    fn a_radio_panic_is_marked_before_its_owned_device_can_block_in_drop() {
        let (marked, panicked) = failure_during_teardown(|failed, device| {
            watch_worker_resources(&failed, RADIO_WORKER, true, device, |_| {
                panic!("radio test")
            });
        });
        assert!(panicked);
        assert_eq!(
            marked, RADIO_WORKER,
            "panic was not marked before device teardown"
        );
    }

    #[test]
    fn an_engine_startup_panic_is_marked_before_its_owned_engine_can_block_in_drop() {
        let (marked, panicked) = failure_during_teardown(|failed, engine| {
            watch_worker_resources(&failed, ENGINE_WORKER, true, engine, |_| {
                panic!("engine startup test")
            });
        });
        assert!(panicked);
        assert_eq!(
            marked, ENGINE_WORKER,
            "startup panic waited for engine teardown"
        );
    }

    #[test]
    fn a_missing_decoder_never_opens_a_hid_device_that_would_block_on_drop() {
        let opened = Arc::new(AtomicBool::new(false));
        let did_open = Arc::clone(&opened);
        let (entered, dropping) = std::sync::mpsc::channel();
        let (release, blocked) = std::sync::mpsc::channel();
        let (completed, result) = std::sync::mpsc::channel();
        let worker = thread::spawn(move || {
            let resources = radio_resources(
                || None::<()>,
                || {
                    did_open.store(true, Ordering::Release);
                    Some(BlockingTeardown {
                        entered,
                        release: blocked,
                    })
                },
            );
            completed.send(resources.is_none()).unwrap();
        });
        let returned = result.recv_timeout(Duration::from_secs(5)).ok();
        let opened = opened.load(Ordering::Acquire);
        let cleanup_started = dropping.try_recv().is_ok();
        // Release an incorrectly opened device before asserting so the mutation cannot hang.
        let _ = release.send(());
        worker.join().unwrap();
        assert_eq!(returned, Some(true),
            "decoder failure waited for HID cancellation: opened={opened}, cleanup_started={cleanup_started}");
        assert!(!opened);
        assert!(!cleanup_started);
    }

    #[test]
    fn radio_resources_open_only_after_the_decoder_is_ready() {
        let order = std::cell::RefCell::new(Vec::new());
        let resources = radio_resources(
            || {
                order.borrow_mut().push("decoder");
                Some(1)
            },
            || {
                order.borrow_mut().push("HID");
                Some(2)
            },
        );
        assert_eq!(resources, Some((1, 2)));
        assert_eq!(*order.borrow(), ["decoder", "HID"]);
        assert!(radio_resources(|| Some(1), || None::<()>).is_none());
    }

    #[test]
    fn a_failed_spawn_marks_failure_before_its_abandoned_capture_can_block_in_drop() {
        let (marked, panicked) = failure_during_teardown(|failed, device| {
            // A failed OS spawn abandons its captured job before returning its error, without
            // entering the callback or beginning a panic. The job's marker must run first.
            let job = WorkerResources::starting(&failed, ENGINE_WORKER, device);
            let start = move || job.run_as(RADIO_WORKER, true, |_| {});
            let failure = Err::<(), _>((io::Error::other("thread creation failed"), start));
            drop(failure);
        });
        assert!(!panicked);
        assert_eq!(
            marked, ENGINE_WORKER,
            "failed spawn waited for captured device teardown"
        );
    }

    #[test]
    fn an_engine_panic_is_marked_before_its_scope_can_wait_on_teardown() {
        let failed = Arc::new(AtomicU8::new(0));
        let flag = Arc::clone(&failed);
        let (release, blocked) = std::sync::mpsc::channel();
        let worker = thread::spawn(move || {
            let _ = std::panic::catch_unwind(|| {
                engine_scope(&flag, |scope| {
                    scope.spawn(move || blocked.recv().unwrap());
                    panic!("engine startup test");
                })
            });
        });
        let until = std::time::Instant::now() + Duration::from_secs(5);
        while failed.load(Ordering::Acquire) == 0 && std::time::Instant::now() < until {
            thread::sleep(Duration::from_millis(1));
        }
        let marked = failed.load(Ordering::Acquire);
        release.send(()).unwrap();
        worker.join().unwrap();
        assert_eq!(marked, ENGINE_WORKER);
    }

    // An offline child of the failure-exit regression; it never opens an engine or USB device.
    #[test]
    #[ignore = "run by a_failed_worker_exits_nonzero_with_a_plain_reason"]
    fn failed_worker_exit_child() {
        let Some(which) = std::env::var_os("CV1TOUCH_TEST_FAILED_WORKER") else {
            return;
        };
        struct DeferredStderr;
        impl io::Write for DeferredStderr {
            fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
                // Make a missing fatal drain observable even if the writer is scheduled first.
                thread::sleep(Duration::from_millis(30));
                io::Write::write(&mut io::stderr(), bytes)
            }
            fn flush(&mut self) -> io::Result<()> {
                io::Write::flush(&mut io::stderr())
            }
        }
        let _logging = daemon_log::init_with_writer(DeferredStderr).unwrap();
        let worker = if which == "radio" {
            RADIO_WORKER
        } else {
            ENGINE_WORKER
        };
        let failed = AtomicU8::new(0);
        let state = Mutex::new(());
        thread::scope(|scope| {
            let panicking = scope.spawn(|| {
                watch_worker(&failed, worker, worker == RADIO_WORKER, || {
                    let _held = state.lock().unwrap();
                    panic!("worker test");
                })
            });
            assert!(panicking.join().is_err());
        });
        assert!(state.is_poisoned());
        let shared = ShmWriter::create(&format!("/cv1t-worker-exit-{}", process::id()), 1).unwrap();
        exit_on_worker_failure(&failed, &shared);
        panic!("a failed worker was treated as live");
    }

    #[test]
    fn a_failed_worker_exits_nonzero_with_a_plain_reason() {
        for (worker, reason) in [
            ("engine", "tracking engine thread"),
            ("radio", "radio thread"),
        ] {
            let child = process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "tests::failed_worker_exit_child",
                    "--ignored",
                    "--nocapture",
                ])
                .env("CV1TOUCH_TEST_FAILED_WORKER", worker)
                .output()
                .unwrap();
            assert_eq!(child.status.code(), Some(1));
            let stderr = String::from_utf8_lossy(&child.stderr);
            assert!(
                stderr.contains(&format!(
                    "{reason} stopped unexpectedly; exiting so the daemon can restart"
                )),
                "missing worker failure diagnostic: {stderr}"
            );
        }
    }

    #[cfg(unix)]
    #[test]
    fn a_failed_worker_exits_with_a_full_stderr_pipe() {
        use std::io::{Read, Write};
        use std::os::fd::{AsRawFd, FromRawFd};
        use std::process::Stdio;
        use std::time::Instant;

        for worker in ["engine", "radio"] {
            let mut fds = [-1; 2];
            // SAFETY: two descriptor slots, each then owned by one File.
            assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
            let mut read = unsafe { fs::File::from_raw_fd(fds[0]) };
            let mut write = unsafe { fs::File::from_raw_fd(fds[1]) };
            let fd = write.as_raw_fd();
            let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
            assert!(flags >= 0);
            assert_eq!(
                unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) },
                0
            );
            loop {
                match write.write(&[b'x'; 4096]) {
                    Ok(_) => {}
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                    Err(e) => panic!("fill stderr: {e}"),
                }
            }
            assert_eq!(unsafe { libc::fcntl(fd, libc::F_SETFL, flags) }, 0);
            let observed = write.try_clone().unwrap();
            let mut child = process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "tests::failed_worker_exit_child",
                    "--ignored",
                    "--nocapture",
                ])
                .env("CV1TOUCH_TEST_FAILED_WORKER", worker)
                .stdout(Stdio::null())
                .stderr(Stdio::from(write))
                .spawn()
                .unwrap();
            let start = Instant::now();
            let status = loop {
                if let Some(status) = child.try_wait().unwrap() {
                    break Some(status);
                }
                if start.elapsed() >= Duration::from_secs(2) {
                    child.kill().unwrap();
                    child.wait().unwrap();
                    break None;
                }
                thread::sleep(Duration::from_millis(5));
            };
            let after = unsafe { libc::fcntl(observed.as_raw_fd(), libc::F_GETFL) };
            drop(observed);
            read.read_to_end(&mut Vec::new()).unwrap();
            assert!(
                status.is_some(),
                "{worker} failure waited for a full stderr pipe"
            );
            assert_eq!(status.unwrap().code(), Some(1));
            assert_eq!(
                after, flags,
                "worker failure changed the pipe's blocking mode"
            );
        }
    }

    #[test]
    fn a_gap_recording_keeps_its_initial_use_priority_when_the_radio_returns() {
        let d = status_daemon();
        let last = 1_000_000_000;
        for (hand, moving) in [(0, true), (1, false)] {
            note_gap(
                &d,
                hand,
                GapEvent::Began {
                    last_sample_host_ns: last,
                    moving,
                },
            );
            note_gap(
                &d,
                hand,
                GapEvent::Ended {
                    last_sample_host_ns: last,
                    silent_ns: flight::SHORTEST_NS,
                },
            );
        }
        let due = d.flight.due(last + flight::AFTER_NS);
        assert_eq!(due.len(), 2);
        assert!(due[0].began_in_use);
        assert!(!due[1].began_in_use);
    }

    #[test]
    fn mixed_case_landmark_status_and_forget_join_device_and_file_names() {
        let d = status_daemon();
        let mut sensor = SensorStatus::default();
        shm::set_serial(&mut sensor, "CAMERA1");
        *d.sensors.lock().unwrap() = vec![sensor];
        d.landmark_shifts
            .lock()
            .unwrap()
            .insert("camera1".into(), (0.5, 0.0, 1));
        let value: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(value["sensors"][0]["landmark_shift"]["dx"], 0.5);
        let mut watch = LandmarkWatch {
            monitor: track::landmarks::BackgroundMonitor::new(vec![], 1, 1).unwrap(),
            alerts: Default::default(),
            missing: Default::default(),
        };
        watch.alerts.insert("camera1".into(), Default::default());
        watch.missing.insert("camera1".into(), 1);
        watch.forget(&d, "CAMERA1");
        assert!(watch.alerts.is_empty() && watch.missing.is_empty());
        assert!(d.landmark_shifts.lock().unwrap().is_empty());
    }

    fn status_daemon() -> Daemon {
        static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let name = format!(
            "/cv1t-dynamic-status-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        );
        Daemon {
            shm: ShmWriter::create(&name, 1).unwrap(),
            rec: Recorder::with_diagnostics(4096, |message| log(message)),
            record_command: Mutex::new(()),
            flight: FlightRecorder::new(),
            quit: AtomicBool::new(false),
            failed_worker: AtomicU8::new(0),
            stop_requested: AtomicBool::new(false),
            haptics: Mutex::new([None; 2]),
            session: Mutex::new(Default::default()),
            opts: Mutex::new(Default::default()),
            engine_requested: std::sync::atomic::AtomicU64::new(1),
            engine_applied: std::sync::atomic::AtomicU64::new(0),
            mode: Mutex::new("calibrated".into()),
            reopen: AtomicBool::new(false),
            tracker: TrackerKind::Rust,
            track_in: Mutex::new(FrameQueue::default()),
            sensors: Mutex::new(Vec::new()),
            sensor_streams: Mutex::new(Vec::new()),
            sensor_video: Mutex::new(Vec::new()),
            track_stats: Mutex::new([(Stats::default(), false); 2]),
            pending_frames_dropped: std::sync::atomic::AtomicU64::new(0),
            landmark_shifts: Mutex::new(Default::default()),
            sensor_poses: Mutex::new(Default::default()),
            sensor_agreement: Mutex::new(Default::default()),
            sensors_unsure: AtomicBool::new(false),
            calibration: Mutex::new(None),
            own_backups: Mutex::new(Vec::new()),
            universe_shift: Mutex::new(None),
            universe_recheck: AtomicBool::new(false),
            driver_stations: Mutex::new(None),
            universe_connection: Mutex::new(UniverseConnection::default()),
            universe_watch_on: AtomicBool::new(true),
            universe_following: Mutex::new(String::new()),
            shaping_on: AtomicBool::new(true),
            position_prediction: AtomicU32::new(100),
            rotation_prediction: AtomicU32::new(100),
            alignment_publish: Mutex::new(()),
            settings_path: std::env::temp_dir()
                .join(format!(
                    "cv1t-daemon-settings-{}",
                    name.trim_start_matches('/')
                ))
                .join("settings.json"),
            record_frames: AtomicBool::new(false),
            extra_latency_ns: std::sync::atomic::AtomicI64::new(0),
            tilt_mdeg: std::sync::atomic::AtomicI32::new(0),
            fitted_render_orient: Mutex::new([[0.0, 0.0, 0.0, 1.0]; 2]),
            battery: [
                std::sync::atomic::AtomicI32::new(-1),
                std::sync::atomic::AtomicI32::new(-1),
            ],
            radio_wake: hid::Waker::new().unwrap(),
            radio_on: AtomicBool::new(false),
            inject: Injection::default(),
            inputs: Mutex::new(Default::default()),
        }
    }

    /// `stop` asks the main loop to shut down, as a signal would.
    #[test]
    fn stop_asks_the_daemon_to_shut_down() {
        let d = status_daemon();
        assert!(!d.stop_requested.load(Ordering::Acquire));
        assert_eq!(handle(&d, "stop"), "ok stopping");
        assert!(d.stop_requested.load(Ordering::Acquire));
    }

    /// `prediction` is saved as it is set, for the next daemon to start from; showing it saves
    /// nothing, and while OpenHMD's own poses are published the reply says the dial does nothing.
    #[test]
    fn the_prediction_setting_is_saved_as_it_is_set() {
        let d = status_daemon();
        d.opts.lock().unwrap().fixed_poses.push(engine::SensorPose {
            serial: "WMTD000000001".into(),
            orient: [0.0, 0.0, 0.0, 1.0],
            pos: [0.0; 3],
        });
        assert_eq!(
            handle(&d, "prediction"),
            "ok prediction: SteamVR extrapolates with 100% of the tracked velocities"
        );
        assert!(!d.settings_path.exists());
        assert_eq!(
            handle(&d, "prediction 0"),
            "ok prediction: SteamVR extrapolates with 0% of the tracked velocities"
        );
        assert_eq!(
            settings::load(&d.settings_path).map(|s| s.prediction_percent),
            Ok(0)
        );
        // positions and orientations each on their own, both kept
        assert_eq!(
            handle(&d, "prediction position 100"),
            "ok prediction: SteamVR extrapolates positions with 100% of the tracked velocity and \
             orientations with 0%"
        );
        assert_eq!(
            (
                d.position_prediction.load(Ordering::Acquire),
                d.rotation_prediction.load(Ordering::Acquire)
            ),
            (100, 0)
        );
        assert_eq!(
            settings::load(&d.settings_path).map(|s| (s.prediction_percent, s.rotation_percent())),
            Ok((100, 0))
        );
        // the driver reads them from the alignment
        let published = d.shm.alignment().unwrap();
        assert_eq!(
            (published.prediction_position, published.prediction_rotation),
            (100, 0)
        );
        assert!(handle(&d, "prediction rotation 30").ends_with("orientations with 30%"));
        assert_eq!(
            settings::load(&d.settings_path).map(|s| (s.prediction_percent, s.rotation_percent())),
            Ok((100, 30))
        );
        // republished in place: a registration published meanwhile stays
        let mut moved = d.shm.alignment().unwrap();
        moved.lighthouse_pos = [1.0, 2.0, 3.0];
        d.shm.publish_alignment(&moved);
        handle(&d, "prediction 0");
        let published = d.shm.alignment().unwrap();
        assert_eq!(published.lighthouse_pos, [1.0, 2.0, 3.0]);
        assert_eq!(
            (published.prediction_position, published.prediction_rotation),
            (0, 0)
        );
        let _ = std::fs::remove_dir_all(d.settings_path.parent().unwrap());
    }

    /// `latency` is saved as it is set, for the next daemon to start from, and `prediction`,
    /// saved beside it, keeps it; the status says it, and the reply says which way the stamps
    /// move. Showing it saves nothing, and none is left out of the file.
    #[test]
    fn the_latency_adjustment_is_saved_as_it_is_set() {
        let d = status_daemon();
        assert_eq!(
            handle(&d, "latency"),
            "ok latency: published poses stamped 4.0 ms earlier than their IMU samples' arrival \
             time (4.0 ms measured, no adjustment)"
        );
        assert!(!d.settings_path.exists());
        assert_eq!(
            handle(&d, "latency -5.4"),
            "ok latency: published poses stamped 1.4 ms later than their IMU samples' arrival \
             time (4.0 ms measured, -5.4 ms adjustment)"
        );
        assert_eq!(d.extra_latency_ns.load(Ordering::Acquire), -5_400_000);
        assert_eq!(
            settings::load(&d.settings_path).map(|s| s.latency_ms()),
            Ok(-5.4)
        );
        handle(&d, "prediction 0");
        let s = settings::load(&d.settings_path).unwrap();
        assert_eq!((s.prediction_percent, s.latency_ms()), (0, -5.4));
        assert_eq!(start_latency_ns(&s), -5_400_000);
        let status: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(status["latency_adjustment_ms"], -5.4);
        assert_eq!(
            handle(&d, "latency 2"),
            "ok latency: published poses stamped 6.0 ms earlier than their IMU samples' arrival \
             time (4.0 ms measured, +2.0 ms adjustment)"
        );
        assert_eq!(
            handle(&d, "latency -4"),
            "ok latency: published poses stamped at their IMU samples' arrival time (4.0 ms \
             measured, -4.0 ms adjustment)"
        );
        handle(&d, "latency 0");
        assert_eq!(
            settings::load(&d.settings_path).map(|s| s.latency_adjustment_ms),
            Ok(None)
        );
        assert_eq!(
            handle(&d, "latency 40"),
            "err latency adjustment must be within +-30 ms"
        );
        assert_eq!(d.extra_latency_ns.load(Ordering::Acquire), 0);
        let _ = std::fs::remove_dir_all(d.settings_path.parent().unwrap());
    }

    /// `tilt` pitches the pose SteamVR and its applications see, through the render offset:
    /// applied once to the offsets as fitted (a republish for another setting does not add it
    /// again), saved as it is set, shown in the status, and 0 gives the fitted offsets back
    /// exactly.
    #[test]
    fn the_tilt_turns_the_render_offset_once_and_is_saved() {
        let d = status_daemon();
        // a fit like the real one, about all three axes: the pitch must be applied in the render
        // frame (fitted * pitch), which this tells apart from the LED-model frame (pitch * fitted)
        let axis_angle = |axis: nalgebra::Unit<nalgebra::Vector3<f64>>, deg: f64| {
            nalgebra::UnitQuaternion::from_axis_angle(&axis, deg.to_radians())
        };
        let fitted = axis_angle(nalgebra::Vector3::x_axis(), 42.5)
            * axis_angle(nalgebra::Vector3::y_axis(), 3.5)
            * axis_angle(nalgebra::Vector3::z_axis(), 2.0);
        let xyzw =
            |q: nalgebra::UnitQuaternion<f64>| [q.i as f32, q.j as f32, q.k as f32, q.w as f32];
        let unit = |q: [f32; 4]| {
            nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                q[3] as f64,
                q[0] as f64,
                q[1] as f64,
                q[2] as f64,
            ))
        };
        // the right hand has no fitted offset (its bit unset; all zeros, as the alignment comes
        // without a fit): left alone, not turned into NaN
        let a = shm::Alignment {
            render_orient: [xyzw(fitted), [0.0; 4]],
            valid: ffi::CV1T_ALIGN_RENDER_LEFT,
            ..shm::Alignment::default()
        };
        publish_alignment(&d, a);
        assert_eq!(
            handle(&d, "tilt"),
            "ok tilt: the controllers are shown as tracked (no tilt)"
        );
        assert_eq!(d.shm.alignment().unwrap().render_orient, a.render_orient);
        assert!(!d.settings_path.exists());
        assert_eq!(
            handle(&d, "tilt 10"),
            "ok tilt: the controllers are shown with the forward end pitched 10.0 deg up from the \
             tracked pose"
        );
        // what was applied, in the render frame: fitted^-1 * published
        let applied =
            |hand: usize| fitted.inverse() * unit(d.shm.alignment().unwrap().render_orient[hand]);
        let published = d.shm.alignment().unwrap();
        assert!(
            applied(0).angle_to(&axis_angle(nalgebra::Vector3::x_axis(), 10.0)) < 1e-4,
            "{:?}",
            applied(0).axis_angle()
        );
        assert_eq!(published.render_orient[1], [0.0; 4]);
        // another setting republishes: the tilt is not added again
        handle(&d, "prediction 0");
        assert_eq!(
            d.shm.alignment().unwrap().render_orient,
            published.render_orient
        );
        assert_eq!(
            settings::load(&d.settings_path).map(|s| (s.tilt_deg(), s.prediction_percent)),
            Ok((10.0, 0))
        );
        let status: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(status["tilt_deg"], 10.0);
        assert_eq!(
            handle(&d, "tilt -2.5"),
            "ok tilt: the controllers are shown with the forward end pitched 2.5 deg down from \
             the tracked pose"
        );
        assert!(applied(0).angle_to(&axis_angle(nalgebra::Vector3::x_axis(), -2.5)) < 1e-4);
        assert_eq!(
            handle(&d, "tilt 50"),
            "err tilt must be a number of degrees within +-45"
        );
        handle(&d, "tilt 0");
        assert_eq!(d.shm.alignment().unwrap().render_orient, a.render_orient);
        assert_eq!(
            settings::load(&d.settings_path).map(|s| s.tilt_deg),
            Ok(None)
        );
        let _ = std::fs::remove_dir_all(d.settings_path.parent().unwrap());
    }

    /// A source note stays within a page: the beginning and the end are kept.
    #[test]
    fn a_source_note_is_kept_to_a_page() {
        assert_eq!(note_in_source("a", "b"), "a; b");
        let long = "x".repeat(2000);
        let kept = note_in_source(&long, "placed again");
        assert!(
            kept.len() <= 1500 && kept.starts_with("xxx") && kept.ends_with("placed again"),
            "{}",
            kept.len()
        );
        assert!(kept.contains("[...]"));
    }

    /// The daemon thins only the backups its own writes made, oldest first.
    #[test]
    fn only_the_daemons_own_backups_are_thinned() {
        let dir = std::env::temp_dir().join(format!("cv1t-own-backups-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let theirs = dir.join("calibration.json.bak-1");
        std::fs::write(&theirs, "setup's").unwrap();
        let mut own = Vec::new();
        for k in 0..KEPT_BACKUPS + 2 {
            let b = dir.join(format!("calibration.json.bak-{}", 100 + k));
            std::fs::write(&b, "ours").unwrap();
            thin_own_backups(&mut own, b);
        }
        assert_eq!(own.len(), KEPT_BACKUPS);
        assert!(theirs.exists(), "another tool's backup stays");
        assert!(
            !dir.join("calibration.json.bak-100").exists()
                && !dir.join("calibration.json.bak-101").exists()
        );
        assert!(dir.join("calibration.json.bak-102").exists());
        std::fs::remove_dir_all(dir).unwrap();
    }

    fn serial_field(name: &str) -> [std::os::raw::c_char; 32] {
        let mut field = [0; 32];
        for (d, b) in field.iter_mut().zip(name.bytes()) {
            *d = b as std::os::raw::c_char;
        }
        field
    }

    fn calibrated_as(serial: &str) -> EngineOptions {
        EngineOptions {
            fixed_poses_only: true,
            fixed_poses: vec![engine::SensorPose {
                serial: serial.into(),
                orient: [0.0, 0.0, 0.0, 1.0],
                pos: [1.0, 2.0, 3.0],
            }],
            ..Default::default()
        }
    }

    /// The native engine places a camera whose calibration is spelled in another letter case than
    /// the device's, and the Rust tracker has to as well: it registers the camera under the
    /// device's spelling and uses its frames, rather than skipping them all.
    #[test]
    fn the_pipeline_uses_a_camera_calibrated_in_another_letter_case() {
        let opts = calibrated_as("camera1");
        let sensor = |name: &str| engine::Sensor {
            serial: serial_field(name),
            width: 1280,
            height: 960,
            fx: 700.0,
            fy: 700.0,
            cx: 640.0,
            cy: 480.0,
            ..Default::default()
        };
        let mut p = calibrated_pipeline(&opts, &[sensor("CAMERA1"), sensor("CAMERA10")]);
        assert!(
            p.camera_id("CAMERA1").is_some(),
            "registered as the device spells it"
        );
        assert!(p.camera_id("camera1").is_none());
        assert!(
            p.camera_id("CAMERA10").is_none(),
            "a longer serial is another sensor"
        );
        let pose = p
            .camera("CAMERA1")
            .unwrap()
            .world_from_cam
            .translation
            .vector;
        assert_eq!((pose.x, pose.y, pose.z), (1.0, 2.0, 3.0));
        p.hmd_clock_sample(5_000_000_000, 77_000);
        let frame = |name: &str| engine::FrameInfo {
            serial: serial_field(name),
            exposure_valid: 1,
            exposure_hmd_ts: 77_000,
            ..Default::default()
        };
        p.frame(&frame("CAMERA1"), &[]);
        assert_eq!(p.frames_skipped, 0, "its frames are not skipped");
        p.frame(&frame("CAMERA10"), &[]);
        assert_eq!(p.frames_skipped, 1);
    }

    /// A sensor placed again keeps its new pose in the options of a reopened engine whatever the
    /// letter case of the calibration's serial, and the options keep that spelling.
    #[test]
    fn a_new_pose_reaches_the_options_of_a_camera_calibrated_in_another_letter_case() {
        let mut opts = calibrated_as("camera1");
        let r = Reposed {
            serial: "CAMERA1".into(),
            orient_xyzw: [0.0, 0.0, 1.0, 0.0],
            pos: [0.5, 0.25, 0.125],
            median_px: 0.2,
            matches: 10,
            moved_mm: 50.0,
            turned_deg: 0.0,
            write: true,
        };
        adopt_reposed_pose(&mut opts, &r);
        assert_eq!(opts.fixed_poses[0].serial, "camera1");
        assert_eq!(opts.fixed_poses[0].orient, [0.0, 0.0, 1.0, 0.0]);
        assert_eq!(opts.fixed_poses[0].pos, [0.5, 0.25, 0.125]);
    }

    /// The new pose goes into the calibration's entry for a camera spelled in another letter case
    /// than the device's, rather than failing with "has no sensor"; the file's spelling stays.
    #[test]
    fn a_new_pose_is_written_to_a_calibration_entry_in_another_letter_case() {
        let dir = std::env::temp_dir().join(format!("cv1t-reposed-case-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let state = dir.join("state");
        std::fs::create_dir_all(&state).unwrap();
        let path = dir.join("calibration.json");
        let entry = |serial: &str| calib::file::SensorEntry {
            serial: serial.into(),
            orient_xyzw: [0.0, 0.0, 0.0, 1.0],
            pos: [0.0, 2.0, 1.0],
            rms_px: 0.3,
            observations: 100,
        };
        let cal = calib::file::CalibrationFile {
            format: calib::file::FORMAT.into(),
            version: calib::file::VERSION,
            created_unix_s: 1,
            world: "gravity".into(),
            source: "a test".into(),
            sensors: vec![entry("wmtd000000001"), entry("WMTD000000002")],
            stats: calib::file::Stats {
                reference: "test".into(),
                gravity_mps2: 9.81,
                gravity_spread_deg: None,
                motion_segments: 1,
                samples: 100,
                rejected_views: 0,
            },
        };
        calib::file::save(&path, &cal).unwrap();
        let loaded = LoadedCalibration::at(path.clone());
        let r = Reposed {
            serial: "WMTD000000001".into(),
            orient_xyzw: [0.0, 0.0, 0.0, 1.0],
            pos: [0.1, 2.0, 1.0],
            median_px: 0.25,
            matches: 4000,
            moved_mm: 100.0,
            turned_deg: 0.0,
            write: true,
        };
        write_reposed(&loaded, &r, &state).unwrap();
        let written = calib::file::load(&path).unwrap();
        assert_eq!(written.sensors[0].serial, "wmtd000000001");
        assert_eq!(written.sensors[0].pos, [0.1, 2.0, 1.0]);
        assert_eq!(written.sensors[0].observations, 4000);
        assert_eq!(written.sensors.len(), 2);
        std::fs::remove_dir_all(dir).unwrap();
    }

    /// A sensor's new pose is written into the loaded file only while it is still that file.
    #[test]
    fn a_new_pose_is_written_only_into_the_file_that_was_loaded() {
        let dir = std::env::temp_dir().join(format!("cv1t-reposed-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let state = dir.join("state");
        std::fs::create_dir_all(&state).unwrap();
        let path = dir.join("calibration.json");
        let cal = calib::file::CalibrationFile {
            format: calib::file::FORMAT.into(),
            version: calib::file::VERSION,
            created_unix_s: 1,
            world: "gravity".into(),
            source: "a test".into(),
            sensors: vec![calib::file::SensorEntry {
                serial: "WMTD000000001".into(),
                orient_xyzw: [0.0, 0.0, 0.0, 1.0],
                pos: [0.0, 2.0, 1.0],
                rms_px: 0.3,
                observations: 100,
            }],
            stats: calib::file::Stats {
                reference: "test".into(),
                gravity_mps2: 9.81,
                gravity_spread_deg: None,
                motion_segments: 1,
                samples: 100,
                rejected_views: 0,
            },
        };
        calib::file::save(&path, &cal).unwrap();
        let loaded = LoadedCalibration::at(path.clone());
        let r = Reposed {
            serial: "WMTD000000001".into(),
            orient_xyzw: [0.0, 0.0, 0.0, 1.0],
            pos: [0.1, 2.0, 1.0],
            median_px: 0.25,
            matches: 4000,
            moved_mm: 100.0,
            turned_deg: 0.0,
            write: true,
        };
        // changed since it was loaded: refused
        let mut other = cal.clone();
        other.source = "someone else's".into();
        calib::file::save(&path, &other).unwrap();
        let err = write_reposed(&loaded, &r, &state).unwrap_err();
        assert!(err.contains("changed since"), "{err}");
        // as loaded: written, with the pose, a note, and a backup
        let loaded = LoadedCalibration::at(path.clone());
        let journal = state.join("setup.json");
        let mut checkpoint = setup::Checkpoint::default();
        checkpoint.completed.insert(
            setup::Stage::Camera,
            setup::Binding {
                artifact_sha256: loaded.sha256.clone(),
                ..Default::default()
            },
        );
        setup::save(&journal, &checkpoint).unwrap();
        let lock = control::lock_single_instance(&state.join("setup.lock")).unwrap();
        let err = write_reposed(&loaded, &r, &state).unwrap_err();
        assert!(err.contains("cv1touch setup is running"), "{err}");
        assert_eq!(setup::fingerprint(&path).unwrap(), loaded.sha256);
        drop(lock);
        let (backup, sha) = write_reposed(&loaded, &r, &state).unwrap();
        assert!(backup.is_some_and(|b| b.exists()));
        let written = calib::file::load(&path).unwrap();
        assert_eq!(written.sensors[0].pos, [0.1, 2.0, 1.0]);
        assert_eq!(written.sensors[0].observations, 4000);
        assert!(written.source.contains("placed again by cv1touchd"));
        assert_eq!(setup::fingerprint(&path).unwrap(), sha);
        let rebound = setup::load(&journal).unwrap().unwrap();
        assert_eq!(
            rebound.completed[&setup::Stage::Camera].artifact_sha256,
            sha,
            "the checkpoint in the supplied state directory follows the new calibration"
        );
        std::fs::remove_dir_all(dir).unwrap();
    }

    /// A connected controller the tracker has no state for (it woke, or a wrong lock was dropped)
    /// is published where the tracker last had it, not at the engine's own position, without a
    /// position and not moving; with the engine tracking, the engine's pose is published as before.
    #[test]
    fn an_unfixed_controller_is_published_where_the_tracker_last_had_it() {
        let d = status_daemon();
        let mut pipe = Pipeline::new(pipeline::Options::default());
        // any body will do: the tracker never gets a frame to fix it with
        let leds = (0..8)
            .map(|i| {
                let a = i as f64 * std::f64::consts::FRAC_PI_4;
                let out = nalgebra::Vector3::new(a.cos(), a.sin(), 0.0);
                calib::capture::Led {
                    pos: out * 0.05,
                    dir: out,
                }
            })
            .collect();
        pipe.body(calib::DEVICE_LEFT, leds, nalgebra::Vector3::zeros());
        let mut shaper = VelocityShaper::new(ShapeParams::default());
        let now = shm::now_ns();
        let c = engine::Controller {
            calibrated: true,
            device_time_ns: 1_000_000,
            received_host_ns: now,
            view_pos: [3.0, 0.5, 4.0],
            view_orient: [0.0, 0.6, 0.0, 0.8],
            view_lin_vel: [1.0, 0.0, 0.0],
            view_ang_vel: [0.0, 2.0, 0.0],
            ..Default::default()
        };
        let mut memory = HandMemory {
            last_seen: Some([0.1, 1.0, -0.2]),
            ..HandMemory::default()
        };
        let mut hand = Hand::new(HAND_LEFT, HandConfig::default());
        publish_hand_state(
            &d,
            HAND_LEFT,
            &c,
            now,
            &mut hand,
            Some(&mut pipe),
            &mut shaper,
            &mut memory,
        );
        let st = d.shm.hand(HAND_LEFT).unwrap();
        assert_ne!(st.flags & ffi::CV1T_FLAG_CONNECTED, 0);
        assert_eq!(st.flags & ffi::CV1T_FLAG_POSITION, 0);
        assert_eq!((st.pos, st.orient), ([0.1, 1.0, -0.2], c.view_orient));
        assert_eq!(st.lin_vel, [0.0; 3]);
        assert_eq!(
            st.ang_vel, st.gyro_bias,
            "no turn for SteamVR to extrapolate"
        );

        let mut hand = Hand::new(HAND_LEFT, HandConfig::default());
        publish_hand_state(
            &d,
            HAND_LEFT,
            &c,
            now,
            &mut hand,
            None,
            &mut shaper,
            &mut memory,
        );
        assert_eq!(d.shm.hand(HAND_LEFT).unwrap().pos, c.view_pos);
    }

    /// A controller that goes silent while in use has the radio's traffic around its dropout
    /// recorded, the haptic commands sent meanwhile included; one that goes silent lying still
    /// (put down, asleep) has not.
    #[test]
    fn a_dropout_in_use_is_recorded() {
        for (gyro, recorded) in [([0.0, 1.0, 0.0], true), ([0.0, 0.01, 0.0], false)] {
            let d = status_daemon();
            let mut shaper = VelocityShaper::new(ShapeParams::default());
            let mut memory = HandMemory::default();
            let mut hand = Hand::new(HAND_RIGHT, HandConfig::default());
            let now = shm::now_ns();
            let c = engine::Controller {
                calibrated: true,
                device_time_ns: 1_000_000,
                received_host_ns: now,
                imu_gyro: gyro,
                ..Default::default()
            };
            // a radio report the engine read (no radio thread)
            d.report(1, now, &[0x0c; 64]);
            for at in [now, now + 200_000_000] {
                publish_hand_state(
                    &d,
                    HAND_RIGHT,
                    &c,
                    at,
                    &mut hand,
                    None,
                    &mut shaper,
                    &mut memory,
                );
            }
            assert_eq!(handle(&d, "haptic right 0.5 320 1"), "ok");
            let due = d.flight.due(now + flight::AFTER_NS);
            assert_eq!(due.len(), recorded as usize);
            if recorded {
                assert_eq!((due[0].hand, due[0].last_report_ns), (HAND_RIGHT, now));
                let types: Vec<u16> = due[0].records.iter().map(|r| r.0).collect();
                assert_eq!(types, [recorder::RAW_REPORT, recorder::HAPTIC]);
                assert_eq!(
                    due[0].records[1].2,
                    recorder::haptic_payload(HAND_RIGHT, 0.5, 320.0, 1.0)
                );
            }
        }
    }

    /// A gap that ends is recorded too, however still the controller was: held steady (aiming)
    /// it does not turn, but asleep it would not have come back.
    #[test]
    fn a_gap_that_ends_is_recorded() {
        let d = status_daemon();
        let mut shaper = VelocityShaper::new(ShapeParams::default());
        let mut memory = HandMemory::default();
        let mut hand = Hand::new(HAND_LEFT, HandConfig::default());
        let now = shm::now_ns();
        let mut c = engine::Controller {
            calibrated: true,
            device_time_ns: 1_000_000,
            received_host_ns: now,
            ..Default::default()
        };
        let mut publish = |c: &engine::Controller, at: u64| {
            publish_hand_state(
                &d,
                HAND_LEFT,
                c,
                at,
                &mut hand,
                None,
                &mut shaper,
                &mut memory,
            )
        };
        publish(&c, now);
        publish(&c, now + 300_000_000);
        assert!(d.flight.due(now + flight::AFTER_NS).is_empty(), "still");
        c.device_time_ns += 2_000_000;
        c.received_host_ns = now + 2_000_000_000;
        publish(&c, now + 2_000_000_000);
        let due = d.flight.due(now + flight::AFTER_NS);
        assert_eq!(due.len(), 1);
        assert_eq!((due[0].hand, due[0].last_report_ns), (HAND_LEFT, now));
    }

    /// A gap shorter than half a second that ends is a blip: not recorded, even in use.
    #[test]
    fn a_blip_is_not_recorded() {
        let d = status_daemon();
        let mut shaper = VelocityShaper::new(ShapeParams::default());
        let mut memory = HandMemory::default();
        let mut hand = Hand::new(HAND_RIGHT, HandConfig::default());
        let now = shm::now_ns();
        let mut c = engine::Controller {
            calibrated: true,
            device_time_ns: 1_000_000,
            received_host_ns: now,
            imu_gyro: [2.0, 0.0, 0.0],
            ..Default::default()
        };
        let mut publish = |c: &engine::Controller, at: u64| {
            publish_hand_state(
                &d,
                HAND_RIGHT,
                c,
                at,
                &mut hand,
                None,
                &mut shaper,
                &mut memory,
            )
        };
        publish(&c, now);
        publish(&c, now + 150_000_000);
        c.device_time_ns += 2_000_000;
        c.received_host_ns = now + 450_000_000;
        publish(&c, now + 450_000_000);
        assert!(d.flight.due(now + flight::AFTER_NS).is_empty());
    }

    /// Through a radio gap a controller is published as it last was while its samples arrived:
    /// its pose and its inputs (input calibration applied) as they were, connected, in the gap,
    /// without a position or motion; it is published live again when its samples come back.
    #[test]
    fn a_radio_gap_publishes_the_controller_as_it_was() {
        let d = status_daemon();
        let mut shaper = VelocityShaper::new(ShapeParams::default());
        let mut memory = HandMemory::default();
        let mut hand = Hand::new(HAND_RIGHT, HandConfig::default());
        let now = shm::now_ns();
        let mut c = engine::Controller {
            calibrated: true,
            device_time_ns: 1_000_000,
            received_host_ns: now,
            last_pose_fix_device_ns: 1_000_000,
            view_pos: [3.0, 0.5, 4.0],
            view_orient: [0.0, 0.6, 0.0, 0.8],
            view_lin_vel: [1.0, 0.0, 0.0],
            view_ang_vel: [0.0, 2.0, 0.0],
            gyro_bias: [0.01, 0.0, 0.0],
            trigger: 0.9,
            grip: 1.0,
            buttons: 0x1,
            ..Default::default()
        };
        let mut publish = |c: &engine::Controller, at: u64| {
            publish_hand_state(
                &d,
                HAND_RIGHT,
                c,
                at,
                &mut hand,
                None,
                &mut shaper,
                &mut memory,
            );
            d.shm.hand(HAND_RIGHT).unwrap()
        };
        let live = publish(&c, now);
        assert_eq!(
            live.flags & (ffi::CV1T_FLAG_POSITION | ffi::CV1T_FLAG_RADIO_GAP),
            ffi::CV1T_FLAG_POSITION
        );
        // the samples stop, and the engine's copy moves on regardless
        c.view_pos = [9.0; 3];
        c.trigger = 0.0;
        c.buttons = 0;
        let held = publish(&c, now + 500_000_000);
        assert_eq!(
            held.flags,
            ffi::CV1T_FLAG_CALIBRATED | ffi::CV1T_FLAG_CONNECTED | ffi::CV1T_FLAG_RADIO_GAP
        );
        assert_eq!((held.pos, held.orient), (live.pos, live.orient));
        assert_eq!(
            (held.trigger, held.grip, held.buttons),
            (live.trigger, live.grip, live.buttons)
        );
        assert_ne!(held.buttons, 0);
        assert_eq!(held.sample_host_ns, live.sample_host_ns);
        assert_eq!(held.publish_host_ns, now + 500_000_000);
        assert_eq!(held.lin_vel, [0.0; 3]);
        assert_eq!(held.ang_vel, held.gyro_bias);
        // back
        c.device_time_ns += 600_000_000;
        c.received_host_ns = now + 600_000_000;
        let back = publish(&c, now + 600_000_000);
        assert_eq!(back.flags & ffi::CV1T_FLAG_RADIO_GAP, 0);
        assert_eq!((back.pos, back.buttons), ([9.0; 3], 0));
    }

    /// The log names each loss of a connected controller's position and, when it comes back, how
    /// long the hand was without one, counted from its wake or its loss; sleep is no loss.
    #[test]
    fn the_log_says_how_long_regaining_a_position_took() {
        let ms = 1_000_000;
        let mut m = HandMemory::default();
        assert_eq!(m.observe(true, false, 0), None, "woke");
        assert_eq!(m.observe(true, false, 100 * ms), None);
        assert_eq!(
            m.observe(true, true, 340 * ms).as_deref(),
            Some("tracking after 340 ms without a position")
        );
        assert_eq!(m.observe(true, true, 400 * ms), None);
        assert_eq!(
            m.observe(true, false, 1000 * ms).as_deref(),
            Some("position lost")
        );
        assert_eq!(
            m.observe(true, true, 1250 * ms).as_deref(),
            Some("tracking after 250 ms without a position")
        );
        assert_eq!(m.observe(false, true, 9000 * ms), None, "asleep");
        assert_eq!(m.observe(true, false, 20_000 * ms), None, "woke again");
        assert_eq!(
            m.observe(true, true, 20_100 * ms).as_deref(),
            Some("tracking after 100 ms without a position")
        );
    }

    /// A tracked sample, turning and moving.
    fn tracked_sample() -> Stamped {
        use cv1touch::track::tracker::Output;
        use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

        let q = UnitQuaternion::from_euler_angles(0.3, -0.2, 1.1);
        let bias = Vector3::new(0.01, -0.02, 0.03);
        let rate = Vector3::new(1.0, 2.0, -3.0);
        Stamped {
            host_ns: 5_000_000_000,
            output: Output {
                t: 0,
                world_from_model: Isometry3::from_parts(Translation3::new(0.1, 1.2, -0.3), q),
                vel: Vector3::new(0.4, -0.5, 0.6),
                ang_vel_world: rate,
                gyro_bias: bias,
                accel_bias: Vector3::zeros(),
                tracking: true,
                last_fix: 0,
            },
            gyro: q.inverse() * rate + bias,
            gyro_bias: bias,
            last_fix_host_ns: 4_990_000_000,
        }
    }

    /// The tracked velocities are published as they are, shaped or not (the driver applies the
    /// prediction percents): the tools that carry a pose forward see the real motion.
    #[test]
    fn the_tracked_velocities_are_published_as_they_are() {
        use nalgebra::Vector3;

        let s = tracked_sample();
        let (q, rate) = (s.output.world_from_model.rotation, s.output.ang_vel_world);
        let v3 = |a: [f32; 3]| Vector3::new(a[0] as f64, a[1] as f64, a[2] as f64);
        let mut st = ControllerState::default();
        let mut shaper = VelocityShaper::new(ShapeParams::default());
        apply_tracked(&mut st, &s, 0, &mut shaper, false);
        // the body rate the driver hands SteamVR (the shaped world rate, in the body), and the
        // linear velocity
        assert!((q * (v3(st.ang_vel) - v3(st.gyro_bias)) - rate).norm() < 1e-5);
        assert!((v3(st.lin_vel) - s.output.vel).norm() < 1e-6);
        let mut shaped = ControllerState::default();
        apply_tracked(&mut shaped, &s, 0, &mut shaper, true);
        assert!(v3(shaped.lin_vel).norm() > 0.5 && v3(shaped.ang_vel).norm() > 0.5);
    }

    /// The percents a daemon starts with come from the settings, orientations' own or else the
    /// positions'.
    #[test]
    fn the_prediction_starts_as_saved() {
        let saved = |position, rotation| settings::Settings {
            prediction_percent: position,
            rotation_prediction_percent: rotation,
            ..settings::Settings::default()
        };
        assert_eq!(start_prediction(&saved(100, Some(0))), (100, 0));
        assert_eq!(start_prediction(&saved(30, None)), (30, 30));
        let alignment = with_prediction(shm::Alignment::default(), (100, 0));
        assert_eq!(
            (alignment.prediction_position, alignment.prediction_rotation),
            (100, 0)
        );
        assert_ne!(alignment.valid & ffi::CV1T_ALIGN_PREDICTION, 0);
    }

    /// While the controller is connected the tracker's pose is published and remembered, and once
    /// the tracker has no state (a wake, a dropped lock) the controller is held where it was,
    /// whatever the engine's own position; a sleeping one has no position.
    #[test]
    fn the_tracked_position_is_remembered_and_held_once_the_state_is_gone() {
        let s = tracked_sample();
        let mut memory = HandMemory::default();
        let mut state = ControllerState {
            flags: ffi::CV1T_FLAG_CONNECTED,
            pos: [3.0, 0.5, 4.0],
            ..Default::default()
        };
        let mut shaper = VelocityShaper::new(ShapeParams::default());
        apply_tracker(&mut state, Some(&s), &mut memory, |st, s| {
            apply_tracked(st, s, 0, &mut shaper, false)
        });
        let seen = s.output.world_from_model.translation.vector.cast::<f32>();
        assert_eq!(state.pos, [seen.x, seen.y, seen.z]);
        assert_eq!(memory.last_seen, Some(state.pos));
        // an output without a position (held, or none to show) is not where it was seen
        let mut untracked = s;
        untracked.output.tracking = false;
        untracked.output.world_from_model.translation.vector.x += 1.0;
        let mut st = state;
        apply_tracker(&mut st, Some(&untracked), &mut memory, |st, s| {
            apply_tracked(st, s, 0, &mut shaper, false)
        });
        assert_eq!(memory.last_seen, Some(state.pos));

        let mut engine = ControllerState {
            flags: ffi::CV1T_FLAG_CONNECTED | ffi::CV1T_FLAG_POSITION,
            pos: [3.0, 0.5, 4.0],
            ..Default::default()
        };
        apply_tracker(&mut engine, None, &mut memory, |_, _| unreachable!());
        assert_eq!(engine.pos, [seen.x, seen.y, seen.z]);
        assert_eq!(engine.flags, ffi::CV1T_FLAG_CONNECTED);

        let mut asleep = ControllerState {
            flags: ffi::CV1T_FLAG_POSITION,
            ..Default::default()
        };
        apply_tracker(&mut asleep, Some(&s), &mut memory, |_, _| unreachable!());
        assert_eq!(asleep.flags, 0);
    }

    /// OpenHMD's brute-force search leaves out a controller our tracker fitted within the last
    /// SEARCH_SKIP_FIX_NS, or one silent on the radio for SEARCH_SKIP_ASLEEP_NS, and searches for
    /// any other: never fitted, fits stopped, or a short radio dropout.
    #[test]
    fn the_search_skips_freshly_fitted_and_sleeping_controllers_only() {
        let ms = 1_000_000;
        assert!(search_needless(Some(0), Some(0)), "fitted this frame");
        assert!(search_needless(Some(100 * ms), Some(0)));
        assert!(!search_needless(Some(101 * ms), Some(0)), "fits stopped");
        assert!(!search_needless(None, Some(0)), "never fitted");
        assert!(!search_needless(None, Some(999 * ms)), "a radio dropout");
        assert!(search_needless(None, Some(1000 * ms)), "asleep");
        assert!(search_needless(None, None), "no sample yet");

        let mut skip = SearchSkip::default();
        assert_eq!(skip.changed(), Some([false, false]));
        assert_eq!(skip.changed(), None, "told only on change");
        skip.hands = [true, false];
        assert_eq!(skip.changed(), Some([true, false]));
    }

    #[test]
    fn idle_control_clients_release_all_sixteen_slots() {
        use std::io::{BufRead, BufReader, Read, Write};
        let fixture = socket_dir::SocketDir::new();
        let dir = fixture.path().to_path_buf();
        let endpoint = dir.join("control.sock");
        let second_endpoint = dir.join("private/control.sock");
        let listeners = vec![ipc::Listener::bind(&endpoint).unwrap()];
        #[cfg(unix)]
        let (listeners, _) = with_container_control(listeners, &second_endpoint);
        #[cfg(windows)]
        let listeners = {
            let mut listeners = listeners;
            listeners.push(ipc::Listener::bind(&second_endpoint).unwrap());
            listeners
        };
        let _server = serve_control(Arc::new(status_daemon()), listeners);
        let mut clients = Vec::new();
        for i in 0..MAX_CLIENTS {
            let path = if i % 2 == 0 {
                &endpoint
            } else {
                &second_endpoint
            };
            let mut conn = ipc::Connection::connect(path).unwrap();
            conn.set_timeout(Some(Duration::from_secs(5))).unwrap();
            conn.write_all(b"bogus\n").unwrap();
            let mut line = String::new();
            BufReader::new(&mut conn).read_line(&mut line).unwrap();
            assert_eq!(line, "err unknown command 'bogus'\n");
            clients.push(conn);
        }
        let closed = clients
            .iter_mut()
            .all(|conn| matches!(conn.read(&mut [0]), Ok(0)));
        drop(clients);
        let served = [&endpoint, &second_endpoint].iter().all(|endpoint| {
            (0..50).any(|_| {
                let answered = control::request(endpoint, "status", Duration::from_secs(2))
                    .is_ok_and(|reply| reply.starts_with("ok {"));
                if !answered {
                    thread::sleep(Duration::from_millis(20));
                }
                answered
            })
        });
        let _ = fs::remove_dir_all(dir);
        assert!(closed, "idle clients kept the control slots");
        assert!(
            served,
            "a new request was refused after idle clients expired"
        );
    }

    /// The control server answers each request line, whether split across reads or several in
    /// one, serves at most MAX_CLIENTS clients, drops one that sends 4 KiB without a newline,
    /// and answers nothing once stopped.
    #[test]
    fn the_control_server_answers_each_line_until_it_stops() {
        use std::io::{ErrorKind, Read, Write};
        let fixture = socket_dir::SocketDir::new();
        let dir = fixture.path().to_path_buf();
        let endpoint = dir.join("control.sock");
        let second_endpoint = dir.join("private/control.sock");
        let listeners = vec![ipc::Listener::bind(&endpoint).unwrap()];
        #[cfg(unix)]
        let (listeners, _) = with_container_control(listeners, &second_endpoint);
        #[cfg(windows)]
        let listeners = {
            let mut listeners = listeners;
            listeners.push(ipc::Listener::bind(&second_endpoint).unwrap());
            listeners
        };
        let control = serve_control(Arc::new(status_daemon()), listeners);
        let connect_on = |endpoint: &std::path::Path| {
            let mut conn = ipc::Connection::connect(endpoint).unwrap();
            conn.set_timeout(Some(Duration::from_secs(5))).unwrap();
            conn
        };
        #[derive(Debug, PartialEq)]
        enum Reply {
            Line(String),
            Closed,
            Silent,
        }
        let reply = |conn: &mut ipc::Connection| {
            let mut line = Vec::new();
            let mut byte = [0u8; 1];
            while byte[0] != b'\n' {
                match conn.read(&mut byte) {
                    Ok(1) => line.push(byte[0]),
                    Err(e) if matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::WouldBlock) => {
                        return Reply::Silent
                    }
                    _ => return Reply::Closed,
                }
            }
            Reply::Line(String::from_utf8(line).unwrap())
        };
        let answered =
            |reply: Reply| matches!(reply, Reply::Line(line) if line.starts_with("ok {"));

        let connect = || connect_on(&endpoint);
        let mut client = connect();
        client.write_all(b"sta").unwrap();
        thread::sleep(Duration::from_millis(20));
        client.write_all(b"tus\nbogus\nstatus\n").unwrap();
        assert!(answered(reply(&mut client)));
        assert_eq!(
            reply(&mut client),
            Reply::Line("err unknown command 'bogus'\n".into())
        );
        assert!(answered(reply(&mut client)));

        let mut second_client = connect_on(&second_endpoint);
        second_client.write_all(b"status\n").unwrap();
        assert!(answered(reply(&mut second_client)));
        let mut idle = Vec::new();
        for i in 2..MAX_CLIENTS {
            let mut next = connect_on(if i % 2 == 0 {
                &endpoint
            } else {
                &second_endpoint
            });
            next.write_all(b"status\n").unwrap();
            assert!(answered(reply(&mut next)));
            idle.push(next);
        }
        let mut refused = connect_on(&second_endpoint);
        let _ = refused.write_all(b"status\n");
        assert_eq!(
            reply(&mut refused),
            Reply::Closed,
            "one client over the limit"
        );
        idle.pop();
        // the freed slot serves the next client once its thread has seen the hang-up
        let served = (0..50).any(|_| {
            let mut next = connect();
            let ok = next.write_all(b"status\n").is_ok() && answered(reply(&mut next));
            if !ok {
                thread::sleep(Duration::from_millis(20));
            }
            ok
        });
        assert!(served);
        drop(idle);

        for endpoint in [&endpoint, &second_endpoint] {
            for terminated in [false, true] {
                let mut flood = connect_on(endpoint);
                let mut line = vec![b'x'; 5000];
                if terminated {
                    line.push(b'\n');
                }
                let _ = flood.write_all(&line);
                assert_eq!(reply(&mut flood), Reply::Closed, "4 KiB per request");
            }
        }

        drop(client);
        let mut client = connect();
        let _stopped = control.stop();
        client
            .set_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        client.write_all(b"status\n").unwrap();
        assert_eq!(reply(&mut client), Reply::Silent, "no answer once stopped");
        second_client
            .set_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        second_client.write_all(b"status\n").unwrap();
        assert_eq!(
            reply(&mut second_client),
            Reply::Silent,
            "both listeners stop"
        );
        let _ = fs::remove_dir_all(&dir);
    }

    #[cfg(unix)]
    #[test]
    fn container_control_is_not_published_after_either_instance_override() {
        let fixture = socket_dir::SocketDir::new();
        let root = fixture.path().to_path_buf();
        let mut cases = vec![("default", InstanceOptions::default(), true)];
        for (tag, option, value) in [
            (
                "runtime",
                "--runtime-dir",
                root.join("custom").to_string_lossy().into_owned(),
            ),
            ("shm", "--shm-name", "/cv1touch-example".to_string()),
            (
                "explicit-default-shm",
                "--shm-name",
                shm::DEFAULT_NAME.to_string(),
            ),
            (
                "explicit-default-runtime",
                "--runtime-dir",
                control::runtime_dir().to_string_lossy().into_owned(),
            ),
        ] {
            let mut instance = InstanceOptions::default();
            instance.set_override(option, value.clone());
            if option == "--runtime-dir" {
                assert_eq!(
                    instance.runtime_dir.as_deref(),
                    Some(std::path::Path::new(&value))
                );
                assert!(instance.shm_name.is_none());
            } else {
                assert_eq!(instance.shm_name.as_deref(), Some(value.as_str()));
                assert!(instance.runtime_dir.is_none());
            }
            cases.push((tag, instance, false));
        }
        for (tag, instance, publish) in cases {
            let dir = root.join(tag);
            fs::create_dir(&dir).unwrap();
            let legacy = dir.join("legacy.sock");
            let fallback = dir.join("private/control.sock");
            let (listeners, shared) = instance
                .with_container_control(vec![ipc::Listener::bind(&legacy).unwrap()], &fallback);
            assert_eq!(shared.is_some(), publish, "{tag}");
            assert_eq!(fallback.exists(), publish, "{tag}");
            let server = serve_control(Arc::new(status_daemon()), listeners);
            let reply = control::request(&legacy, "status", Duration::from_secs(5)).unwrap();
            assert!(
                reply.starts_with("ok {"),
                "{tag}: keep the instance's own endpoint"
            );
            let _stopped = server.stop();
        }
        fs::remove_dir_all(root).unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn an_unsafe_container_directory_keeps_the_legacy_control_server() {
        use std::os::unix::fs::{MetadataExt, PermissionsExt};
        let fixture = socket_dir::SocketDir::new();
        let dir = fixture.path().to_path_buf();
        let private = dir.join("private");
        fs::create_dir(&private).unwrap();
        fs::set_permissions(&private, fs::Permissions::from_mode(0o755)).unwrap();
        let legacy = dir.join("legacy.sock");
        let (listeners, shared) = with_container_control(
            vec![ipc::Listener::bind(&legacy).unwrap()],
            &private.join("control.sock"),
        );
        assert!(shared.is_none());
        assert_eq!(
            fs::symlink_metadata(&private).unwrap().mode() & 0o7777,
            0o755
        );
        let server = serve_control(Arc::new(status_daemon()), listeners);
        let reply = control::request(&legacy, "status", Duration::from_secs(5)).unwrap();
        assert!(reply.starts_with("ok {"));
        let _stopped = server.stop();
        fs::remove_dir_all(dir).unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn a_full_container_control_backlog_keeps_the_legacy_server_available() {
        use std::os::fd::AsRawFd;
        use std::os::unix::fs::PermissionsExt;
        use std::os::unix::net::{UnixListener, UnixStream};
        let fixture = socket_dir::SocketDir::new();
        let dir = fixture.path().to_path_buf();
        let private = dir.join("private");
        fs::create_dir(&private).unwrap();
        fs::set_permissions(&private, fs::Permissions::from_mode(0o700)).unwrap();
        let endpoint = private.join("control.sock");
        let busy = UnixListener::bind(&endpoint).unwrap();
        // SAFETY: a live listening socket; backlog zero admits one pending connection on Linux.
        assert_eq!(unsafe { libc::listen(busy.as_raw_fd(), 0) }, 0);
        let _queued = UnixStream::connect(&endpoint).unwrap();
        let legacy = dir.join("legacy.sock");
        let listeners = vec![ipc::Listener::bind(&legacy).unwrap()];
        let (send, recv) = std::sync::mpsc::channel();
        let worker = thread::spawn(move || {
            send.send(with_container_control(listeners, &endpoint))
                .unwrap();
        });
        let result = recv.recv_timeout(Duration::from_secs(2));
        let did_not_block = result.is_ok();
        // Release a blocking probe before failing, so the regression leaves no stuck thread.
        let (listeners, shared) = result.unwrap_or_else(|_| {
            busy.accept().unwrap();
            recv.recv_timeout(Duration::from_secs(5)).unwrap()
        });
        worker.join().unwrap();
        assert!(shared.is_none());
        let server = serve_control(Arc::new(status_daemon()), listeners);
        let reply = control::request(&legacy, "status", Duration::from_secs(5)).unwrap();
        assert!(reply.starts_with("ok {"));
        let _stopped = server.stop();
        fs::remove_dir_all(dir).unwrap();
        assert!(
            did_not_block,
            "a full secondary backlog blocked daemon startup"
        );
    }

    struct BlockedSink {
        entered: Option<std::sync::mpsc::Sender<()>>,
        release: std::sync::mpsc::Receiver<()>,
        sync: bool,
        header_written: bool,
        block_header: bool,
    }

    impl BlockedSink {
        fn block(&mut self) {
            if let Some(entered) = self.entered.take() {
                let _ = entered.send(());
                // Dropping the test's sender also releases storage on assertion failure.
                let _ = self.release.recv();
            }
        }
    }
    impl Write for BlockedSink {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            if (self.header_written || self.block_header) && !self.sync {
                self.block();
            }
            self.header_written = true;
            Ok(bytes.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    impl recorder::RecordingSink for BlockedSink {
        fn sync_all(&mut self) -> io::Result<()> {
            if self.sync {
                self.block();
            }
            Ok(())
        }
        fn disk_space(&self) -> io::Result<recorder::DiskSpace> {
            Ok(recorder::DiskSpace {
                available: 100 << 30,
                capacity: 200 << 30,
            })
        }
    }

    /// Both a blocked write and a blocked sync leave status/haptics/daemon stop responsive, and
    /// neither a premature success reply nor a replacement writer may escape while finishing.
    #[test]
    fn blocked_recording_storage_does_not_block_control_commands() {
        for (sync, scoped) in [(false, false), (true, false), (false, true), (true, true)] {
            let d = Arc::new(status_daemon());
            let fixture = socket_dir::SocketDir::new();
            let dir = fixture.path();
            let path = dir.join("capture.rec");
            let endpoint = dir.join("control.sock");
            let driver_endpoint = dir.join("driver.sock");
            let (entered_tx, entered) = std::sync::mpsc::channel();
            let (release, release_rx) = std::sync::mpsc::channel();
            d.rec
                .start_with_sink(
                    &path,
                    1,
                    false,
                    Box::new(BlockedSink {
                        entered: Some(entered_tx),
                        release: release_rx,
                        sync,
                        header_written: false,
                        block_header: false,
                    }),
                )
                .unwrap();
            d.rec.write(recorder::HAPTIC, 2, &[&[0u8; 16]]);
            if !sync {
                entered.recv_timeout(Duration::from_secs(2)).unwrap();
            }
            let _server = serve_control(
                Arc::clone(&d),
                vec![
                    ipc::Listener::bind(&endpoint).unwrap(),
                    ipc::Listener::bind(&driver_endpoint).unwrap(),
                ],
            );
            let stop_endpoint = endpoint.clone();
            let stop_command = if scoped {
                control::record_stop_if_line(&path).unwrap()
            } else {
                "record stop".into()
            };
            let (answered_tx, answered) = std::sync::mpsc::channel();
            let stop = thread::spawn(move || {
                // Old callers only checked `ok`, with a five-second request timeout.
                let result =
                    control::request(&stop_endpoint, &stop_command, Duration::from_secs(5));
                answered_tx.send(result).unwrap();
            });
            if sync {
                entered.recv_timeout(Duration::from_secs(2)).unwrap();
            }
            let deadline = std::time::Instant::now() + Duration::from_secs(2);
            while d.rec.active() && std::time::Instant::now() < deadline {
                thread::sleep(Duration::from_millis(1));
            }
            assert!(!d.rec.active(), "stop did not reach the recorder");
            let request = |line: &str| {
                control::request(&driver_endpoint, line, Duration::from_millis(200)).unwrap()
            };
            let reply = request("status");
            let value: serde_json::Value =
                serde_json::from_str(reply.strip_prefix("ok ").unwrap()).unwrap();
            assert_eq!(value["recording_status"]["state"], "finishing");
            assert_eq!(value["recording_status"]["finished"], false);
            assert_eq!(request("haptic left 0.1 80 0.5"), "ok");
            assert!(d.haptics.lock().unwrap()[HAND_LEFT].is_some());
            assert_eq!(request("stop"), "ok stopping");
            let start = control::record_start_line(&dir.join("replacement.rec"), false).unwrap();
            assert!(request(&start).starts_with("err "));
            assert!(!dir.join("replacement.rec").exists());
            let premature = answered.recv_timeout(Duration::from_millis(600));
            let pending = d.rec.has_session();
            let waited_for_storage = premature.is_err();
            // Always release before asserting so a mutation cannot strand a test writer.
            release.send(()).unwrap();
            let reply = match premature {
                Ok(reply) => reply,
                Err(_) => answered.recv_timeout(Duration::from_secs(2)).unwrap(),
            }
            .unwrap();
            stop.join().unwrap();
            assert!(pending);
            assert!(
                waited_for_storage,
                "record stop answered before storage completed"
            );
            assert!(
                reply.starts_with(if scoped {
                    "ok stopped records="
                } else {
                    "ok records="
                }),
                "a slow completed stop must keep the old success shape: {reply}"
            );
            assert!(d.rec.status().finished);
            assert!(!d.rec.has_session());
        }
    }

    #[test]
    fn a_recording_stop_timeout_is_bounded_while_sync_is_blocked() {
        let d = Arc::new(status_daemon());
        let (entered_tx, entered) = std::sync::mpsc::channel();
        let (release, release_rx) = std::sync::mpsc::channel();
        d.rec
            .start_with_sink(
                &std::env::temp_dir().join("cv1t-bounded-stop.rec"),
                1,
                false,
                Box::new(BlockedSink {
                    entered: Some(entered_tx),
                    release: release_rx,
                    sync: true,
                    header_written: false,
                    block_header: false,
                }),
            )
            .unwrap();
        let (done_tx, done) = std::sync::mpsc::channel();
        let stopping = Arc::clone(&d);
        let stopper = thread::spawn(move || {
            let result = stopping.rec.stop_timeout(None, Duration::from_millis(20));
            done_tx.send(result).unwrap();
        });
        entered.recv_timeout(Duration::from_secs(2)).unwrap();
        let timed_out = done.recv_timeout(Duration::from_millis(200));
        release.send(()).unwrap();
        stopper.join().unwrap();
        let status = timed_out
            .expect("a bounded stop waited indefinitely for storage")
            .unwrap();
        assert!(!status.finished);
    }

    #[test]
    fn scoped_stop_cancels_an_opening_recording_without_holding_up_status() {
        let d = Arc::new(status_daemon());
        let fixture = socket_dir::SocketDir::new();
        let dir = fixture.path();
        let path = dir.join("capture.rec");
        let endpoint = dir.join("control.sock");
        let (entered_tx, entered) = std::sync::mpsc::channel();
        let (release, release_rx) = std::sync::mpsc::channel();
        let start_daemon = Arc::clone(&d);
        let start_path = path.clone();
        let start = thread::spawn(move || {
            // The dispatcher holds the recording guard while start writes its header. Inject
            // that storage operation here; the stop/status requests use the real endpoint.
            let _record = start_daemon.record_command.lock().unwrap();
            start_daemon.rec.start_with_sink(
                &start_path,
                1,
                false,
                Box::new(BlockedSink {
                    entered: Some(entered_tx),
                    release: release_rx,
                    sync: false,
                    header_written: false,
                    block_header: true,
                }),
            )
        });
        entered.recv_timeout(Duration::from_secs(2)).unwrap();
        let _server = serve_control(
            Arc::clone(&d),
            vec![ipc::Listener::bind(&endpoint).unwrap()],
        );
        let request =
            |line: &str| control::request(&endpoint, line, Duration::from_millis(200)).unwrap();
        let reply = request("status");
        let value: serde_json::Value =
            serde_json::from_str(reply.strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(value["recording_status"]["state"], "starting");
        // Cleanup for another capture must never cancel this one.
        let other_stop = control::record_stop_if_line(&dir.join("another.rec")).unwrap();
        assert!(request(&other_stop).starts_with("err "));
        assert_eq!(d.rec.status().state, recorder::State::Starting);
        let stop = control::record_stop_if_line(&path).unwrap();
        assert!(request(&stop).starts_with("err "));
        release.send(()).unwrap();
        start.join().unwrap().unwrap();
        assert!(
            !d.rec.active(),
            "a timed-out start continued recording after its scoped cleanup"
        );
        let reply = control::request(&endpoint, &stop, Duration::from_secs(2)).unwrap();
        assert!(reply.starts_with("ok stopped records="), "{reply}");
    }

    #[test]
    fn startup_recording_failure_finishes_when_stderr_is_closed() {
        const CHILD: &str = "CV1TOUCH_TEST_RECORD_CLOSED_STDERR";
        if let Some(gate) = std::env::var_os(CHILD) {
            let _logging = daemon_log::init().unwrap();
            let gate = PathBuf::from(gate);
            let deadline = std::time::Instant::now() + Duration::from_secs(2);
            while !gate.exists() {
                assert!(std::time::Instant::now() < deadline);
                thread::sleep(Duration::from_millis(1));
            }
            for panicking in [false, true] {
                let mut d = status_daemon();
                if panicking {
                    d.rec = Recorder::with_diagnostics(4096, |message| {
                        log(message);
                        panic!("test diagnostic failed");
                    });
                }
                let starting = d
                    .rec
                    .start_in_background_with(&gate.with_extension("rec"), 1, || {
                        Err(io::Error::other(
                            "test recording open failed with closed stderr",
                        ))
                    })
                    .unwrap();
                let deadline = std::time::Instant::now() + Duration::from_secs(2);
                let outcome = loop {
                    if let Some(outcome) = starting.poll() {
                        break outcome;
                    }
                    assert!(std::time::Instant::now() < deadline);
                    thread::sleep(Duration::from_millis(1));
                };
                let status = d
                    .rec
                    .stop_timeout(None, Duration::from_millis(100))
                    .unwrap();
                println!("finished={} outcome={outcome:?}", status.finished);
                assert!(
                    status.finished,
                    "a diagnostic failure left the recording unfinished"
                );
                assert!(outcome
                    .unwrap_err()
                    .contains("test recording open failed with closed stderr"));
                assert_eq!(status.stats.write_errors, 1);
            }
            return;
        }
        let gate = std::env::temp_dir().join(format!(
            "cv1t-rec-stderr-{}-{}",
            std::process::id(),
            shm::now_ns()
        ));
        let mut child = process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "tests::startup_recording_failure_finishes_when_stderr_is_closed",
                "--nocapture",
            ])
            .env(CHILD, &gate)
            .stdout(process::Stdio::piped())
            .stderr(process::Stdio::piped())
            .spawn()
            .unwrap();
        // Gate the failure until the pipe reader is closed, rather than racing process startup.
        drop(child.stderr.take());
        fs::write(&gate, b"closed").unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        let timely = loop {
            if child.try_wait().unwrap().is_some() {
                break true;
            }
            if std::time::Instant::now() >= deadline {
                child.kill().unwrap();
                break false;
            }
            thread::sleep(Duration::from_millis(10));
        };
        let output = child.wait_with_output().unwrap();
        fs::remove_file(gate).unwrap();
        assert!(timely, "recording did not finish with closed stderr");
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stdout)
        );
        assert_eq!(
            String::from_utf8_lossy(&output.stdout)
                .matches("finished=true")
                .count(),
            2,
            "the ordinary and panicking diagnostic sinks must both finish"
        );
    }

    #[test]
    fn a_dispatched_recording_is_reserved_before_shutdown_can_pass_it() {
        let d = Arc::new(status_daemon());
        let fixture = socket_dir::SocketDir::new();
        let dir = fixture.path();
        let path = dir.join("capture.rec");
        let runtime = dir.join("control.sock");
        let listeners = vec![ipc::Listener::bind(&runtime).unwrap()];
        #[cfg(unix)]
        let (listeners, _) = with_container_control(listeners, &dir.join("container/control.sock"));
        let server = Arc::new(serve_control(Arc::clone(&d), listeners));
        let (paused_tx, paused) = std::sync::mpsc::channel();
        let (reserve_tx, reserve_rx) = std::sync::mpsc::channel();
        let (opening_tx, opening) = std::sync::mpsc::channel();
        let (open_tx, open_rx) = std::sync::mpsc::channel();
        let request_daemon = Arc::clone(&d);
        let request_lock = Arc::clone(&server.one_at_a_time);
        let check_lock = Arc::clone(&request_lock);
        let line = control::record_start_line(&path, false).unwrap();
        let request = thread::spawn(move || {
            handle_control_recording(
                &request_daemon,
                &line,
                &request_lock,
                |rec, path, frames| {
                    // Pause immediately before the actual reservation. Shutdown must still be
                    // excluded by the shared turn until that reservation has become visible.
                    paused_tx.send(check_lock.try_lock().is_err()).unwrap();
                    reserve_rx.recv().unwrap();
                    rec.reserve_with_frames(path, frames)
                },
                |reserved| {
                    opening_tx.send(()).unwrap();
                    open_rx.recv().unwrap();
                    reserved.start(shm::now_ns())
                },
            )
        });
        let protected = paused.recv_timeout(Duration::from_secs(2)).unwrap();
        let stopping = Arc::clone(&d);
        let stop_server = Arc::clone(&server);
        let (stopped_tx, stopped) = std::sync::mpsc::channel();
        let shutdown = thread::spawn(move || {
            let _turn = stop_server.stop();
            stopping.quit.store(true, Ordering::Release);
            finish_recording_shutdown(&stopping.rec);
            stopped_tx.send(()).unwrap();
        });
        if protected {
            reserve_tx.send(()).unwrap();
        }
        stopped.recv_timeout(Duration::from_secs(2)).unwrap();
        if !protected {
            reserve_tx.send(()).unwrap();
        }
        opening.recv_timeout(Duration::from_secs(2)).unwrap();
        open_tx.send(()).unwrap();
        let reply = request.join().unwrap();
        shutdown.join().unwrap();
        let created = path.exists();
        let _ = d.rec.stop_timeout(None, Duration::from_secs(2));
        d.shm.retire();
        assert!(protected, "reservation happened after releasing the shared dispatch lock: created={created}, reply={reply}");
        assert!(
            !created,
            "a recording file was created after shutdown finished: {reply}"
        );
        assert!(
            reply.starts_with("err ") && reply.contains("cancelled"),
            "{reply}"
        );
    }

    struct ReserveStopSink(std::sync::atomic::AtomicUsize);
    impl Write for ReserveStopSink {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            Ok(bytes.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    impl recorder::RecordingSink for ReserveStopSink {
        fn sync_all(&mut self) -> io::Result<()> {
            Ok(())
        }
        fn disk_space(&self) -> io::Result<recorder::DiskSpace> {
            Ok(recorder::DiskSpace {
                available: if self.0.fetch_add(1, Ordering::Relaxed) == 0 {
                    100 << 30
                } else {
                    1 << 30
                },
                capacity: 40 << 30,
            })
        }
    }

    fn recording_test_logger() -> daemon_log::Guard {
        struct DeferredStderr;
        impl Write for DeferredStderr {
            fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
                // A missing exit drain must lose the diagnostic even if the writer runs first.
                thread::sleep(Duration::from_millis(30));
                Write::write(&mut io::stderr(), bytes)
            }
            fn flush(&mut self) -> io::Result<()> {
                Write::flush(&mut io::stderr())
            }
        }
        daemon_log::init_with_writer(DeferredStderr).unwrap()
    }

    #[test]
    fn normal_shutdown_reports_each_recording_outcome_once() {
        const CHILD: &str = "CV1TOUCH_TEST_NORMAL_RECORD_SHUTDOWN";
        if let Ok(kind) = std::env::var(CHILD) {
            let _logging = recording_test_logger();
            static REPORTED: std::sync::atomic::AtomicUsize =
                std::sync::atomic::AtomicUsize::new(0);
            let mut d = status_daemon();
            d.rec = Recorder::with_diagnostics(4096, |message| {
                log(message);
                REPORTED.fetch_add(1, Ordering::Release);
            });
            let (entered_tx, entered) = std::sync::mpsc::channel();
            let (release, release_rx) = std::sync::mpsc::channel();
            let sink: Box<dyn recorder::RecordingSink> = match kind.as_str() {
                "error" => Box::new(FailedSink {
                    sync: false,
                    header_written: false,
                }),
                "reserve" => Box::new(ReserveStopSink(std::sync::atomic::AtomicUsize::new(0))),
                "pending" => Box::new(BlockedSink {
                    entered: Some(entered_tx),
                    release: release_rx,
                    sync: false,
                    header_written: false,
                    block_header: false,
                }),
                _ => unreachable!(),
            };
            d.rec
                .start_with_sink(std::path::Path::new("unused.rec"), 1, false, sink)
                .unwrap();
            d.rec.write(recorder::HAPTIC, 2, &[&[0u8; 16]]);
            if kind == "pending" {
                entered.recv_timeout(Duration::from_secs(2)).unwrap();
            } else {
                let deadline = std::time::Instant::now() + Duration::from_secs(2);
                while !d.rec.status().finished || REPORTED.load(Ordering::Acquire) == 0 {
                    assert!(std::time::Instant::now() < deadline);
                    thread::sleep(Duration::from_millis(1));
                }
            }
            finish_recording_shutdown(&d.rec);
            if kind == "pending" {
                release.send(()).unwrap();
                assert!(
                    d.rec
                        .stop_timeout(None, Duration::from_secs(2))
                        .unwrap()
                        .finished
                );
            }
            return;
        }
        for kind in ["error", "reserve", "pending"] {
            let output = process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "tests::normal_shutdown_reports_each_recording_outcome_once",
                    "--nocapture",
                ])
                .env(CHILD, kind)
                .output()
                .unwrap();
            let text = String::from_utf8_lossy(&output.stderr);
            assert!(output.status.success(), "{kind}: {text}");
            let diagnostics: Vec<_> = text
                .lines()
                .filter(|line| line.starts_with("cv1touchd: recording "))
                .collect();
            assert_eq!(diagnostics.len(), 1, "{kind}: {text}");
            if kind == "pending" {
                assert!(diagnostics[0].contains("file may be incomplete"), "{text}");
            }
        }
    }

    struct BlockedCloseSink {
        inner: FailedSink,
        release: std::sync::mpsc::Receiver<()>,
    }
    impl Write for BlockedCloseSink {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            self.inner.write(bytes)
        }
        fn flush(&mut self) -> io::Result<()> {
            self.inner.flush()
        }
    }
    impl recorder::RecordingSink for BlockedCloseSink {
        fn sync_all(&mut self) -> io::Result<()> {
            self.inner.sync_all()
        }
        fn disk_space(&self) -> io::Result<recorder::DiskSpace> {
            self.inner.disk_space()
        }
    }
    impl Drop for BlockedCloseSink {
        fn drop(&mut self) {
            let _ = self.release.recv();
        }
    }

    fn startup_test_listeners(dir: &std::path::Path) -> (Vec<ipc::Listener>, ControlEndpoints) {
        let runtime = dir.join("control.sock");
        let listeners = vec![ipc::Listener::bind(&runtime).unwrap()];
        #[cfg(unix)]
        let (listeners, container) =
            with_container_control(listeners, &dir.join("container/control.sock"));
        #[cfg(unix)]
        assert!(container.is_some());
        (
            listeners,
            ControlEndpoints {
                runtime,
                #[cfg(unix)]
                container,
            },
        )
    }

    /// Production startup serves status while its reserved open/header waits, and refuses a
    /// second start. No engine or USB device is opened by this orchestration.
    #[test]
    fn a_blocked_startup_recording_open_or_header_leaves_control_available() {
        for block_header in [false, true] {
            let d = Arc::new(status_daemon());
            let fixture = socket_dir::SocketDir::new();
            let dir = fixture.path();
            let path = dir.join("capture.rec");
            let (listeners, endpoints) = startup_test_listeners(dir);
            let endpoint = endpoints.runtime.clone();
            let (entered_tx, entered) = std::sync::mpsc::channel();
            let (release, release_rx) = std::sync::mpsc::channel();
            let start_daemon = Arc::clone(&d);
            let start_endpoints = endpoints.clone();
            let creator = thread::spawn(move || {
                serve_with_startup_recording(
                    start_daemon,
                    listeners,
                    &start_endpoints,
                    |rec| {
                        rec.start_in_background_with(&path, 1, move || {
                            if !block_header {
                                let _ = entered_tx.send(());
                                let _ = release_rx.recv();
                                return Ok(Box::new(FailedSink {
                                    sync: false,
                                    header_written: false,
                                })
                                    as Box<dyn recorder::RecordingSink>);
                            }
                            Ok(Box::new(BlockedSink {
                                entered: Some(entered_tx),
                                release: release_rx,
                                sync: false,
                                header_written: false,
                                block_header: true,
                            })
                                as Box<dyn recorder::RecordingSink>)
                        })
                        .map(Some)
                    },
                    |within| {
                        thread::sleep(within);
                        None
                    },
                )
            });
            entered.recv_timeout(Duration::from_secs(2)).unwrap();
            let request =
                |line: &str| control::request(&endpoint, line, Duration::from_millis(200));
            let status = request("status");
            #[cfg(unix)]
            let container_status = control::request(
                endpoints.container.as_ref().unwrap(),
                "status",
                Duration::from_millis(200),
            );
            let replacement =
                request(&control::record_start_line(&dir.join("other.rec"), false).unwrap());
            let haptic = request("haptic left 0.1 80 0.5");
            // Release before asserting, including when a mutation puts I/O before control.
            let _ = release.send(());
            let server = creator.join().unwrap();
            let value: serde_json::Value = serde_json::from_str(
                status
                    .expect("startup storage blocked status")
                    .strip_prefix("ok ")
                    .unwrap(),
            )
            .unwrap();
            #[cfg(unix)]
            assert!(container_status.unwrap().contains("\"state\":\"starting\""));
            assert_eq!(value["recording_status"]["state"], "starting");
            assert_eq!(value["recording_status"]["byte_limit"], 8u64 << 30);
            assert_eq!(replacement.unwrap(), "err already recording");
            assert!(!dir.join("other.rec").exists());
            assert_eq!(haptic.unwrap(), "ok");
            assert_eq!(
                finish_recording_startup(&d, &server, &endpoints, None, None),
                0
            );
        }
    }

    /// The child calls the production startup function, which owns its fatal exit and cleanup.
    /// An actual failed open and an injected failed header must both exit 1 and retire IPC.
    #[test]
    fn a_startup_recording_open_or_header_error_is_fatal_and_retires_ipc() {
        const CHILD: &str = "CV1TOUCH_TEST_RECORD_START_ERROR";
        if let Some(kind) = std::env::var_os(CHILD) {
            let _logging = recording_test_logger();
            let dir = PathBuf::from(std::env::var_os("CV1TOUCH_TEST_RECORD_START_DIR").unwrap());
            let d = Arc::new(status_daemon());
            fs::write(dir.join("shm-name"), d.shm.name()).unwrap();
            let (listeners, endpoints) = startup_test_listeners(&dir);
            let path = dir.join("missing").join("capture.rec");
            let _server = serve_with_startup_recording(
                d,
                listeners,
                &endpoints,
                |rec| {
                    if kind == "open" {
                        rec.start_in_background(&path, 1)
                    } else {
                        rec.start_in_background_with(&path, 1, || {
                            Ok(Box::new(FailedSink {
                                sync: false,
                                header_written: true,
                            }))
                        })
                    }
                    .map(Some)
                },
                |within| {
                    thread::sleep(within);
                    None
                },
            );
            panic!("production startup accepted a fatal recording error");
        }
        for kind in ["open", "header"] {
            let fixture = socket_dir::SocketDir::new();
            let dir = fixture.path();
            let mut child = process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "tests::a_startup_recording_open_or_header_error_is_fatal_and_retires_ipc",
                    "--nocapture",
                ])
                .env(CHILD, kind)
                .env("CV1TOUCH_TEST_RECORD_START_DIR", dir)
                .stdout(process::Stdio::piped())
                .stderr(process::Stdio::piped())
                .spawn()
                .unwrap();
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            let timely = loop {
                if child.try_wait().unwrap().is_some() {
                    break true;
                }
                if std::time::Instant::now() >= deadline {
                    child.kill().unwrap();
                    break false;
                }
                thread::sleep(Duration::from_millis(10));
            };
            let output = child.wait_with_output().unwrap();
            let text = format!(
                "{}{}",
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
            let name = fs::read_to_string(dir.join("shm-name")).unwrap();
            #[cfg(unix)]
            assert!(
                !dir.join("container/control.sock").exists(),
                "startup left its private socket behind"
            );
            assert!(
                !dir.join("control.sock").exists(),
                "startup left its socket behind"
            );
            assert!(
                shm::ShmReader::open(&name).is_err(),
                "startup left shared memory live"
            );
            assert!(
                timely,
                "startup did not report its failure promptly: {text}"
            );
            assert_eq!(output.status.code(), Some(1), "{text}");
            let diagnostics: Vec<_> = text
                .lines()
                .filter(|line| {
                    line.contains("recording failed:")
                        || line.starts_with("cv1touchd: open ")
                        || line.starts_with("cv1touchd: write ")
                })
                .collect();
            assert_eq!(diagnostics.len(), 1, "duplicate startup error: {text}");
            assert!(
                diagnostics[0].starts_with("cv1touchd: recording failed:"),
                "writer must own its diagnostic: {text}"
            );
            assert!(
                text.contains(if kind == "open" { "open " } else { "write " }),
                "{text}"
            );
            if kind == "header" {
                assert!(text.contains("test storage write failed"), "{text}");
            }
        }
    }

    /// Open is released only after production cleanup has requested the recorder drain.
    /// Its final storage wait must retain header errors, inspect sync failures and report a
    /// still-blocked open, while allowing a successful drain to exit cleanly.
    #[test]
    fn startup_recording_cleanup_reports_its_final_storage_outcome() {
        const CHILD: &str = "CV1TOUCH_TEST_RECORD_CLEANUP";
        if let Some(kind) = std::env::var_os(CHILD) {
            let _logging = recording_test_logger();
            let dir = PathBuf::from(std::env::var_os("CV1TOUCH_TEST_RECORD_CLEANUP_DIR").unwrap());
            let d = Arc::new(status_daemon());
            fs::write(dir.join("shm-name"), d.shm.name()).unwrap();
            let (listeners, endpoints) = startup_test_listeners(&dir);
            let path = dir.join("capture.rec");
            let (hold_open, released) = std::sync::mpsc::channel::<()>();
            let opening_daemon = Arc::clone(&d);
            let stopping_daemon = Arc::clone(&d);
            let replacing =
                kind == "replaced" || kind == "sync-replaced" || kind == "replacement-sync";
            let replacement_sync = kind == "replacement-sync";
            let replacement = dir.join("replacement.rec");
            let replacement_marker = dir.join("replacement-installed");
            let _server = serve_with_startup_recording(
                d,
                listeners,
                &endpoints,
                |rec| {
                    rec.start_in_background_with(&path, 1, move || {
                        if kind == "sync-replaced" || kind == "replacement-sync" {
                            let _ = released.recv();
                            return Ok(Box::new(FailedSink {
                                sync: kind == "sync-replaced",
                                header_written: false,
                            })
                                as Box<dyn recorder::RecordingSink>);
                        }
                        if kind == "pending" || kind == "replaced" {
                            let _ = released.recv();
                        }
                        if kind == "replaced" {
                            return Err(io::Error::other(
                                "test startup open failed before replacement",
                            ));
                        }
                        let deadline = std::time::Instant::now() + Duration::from_secs(2);
                        while !opening_daemon.quit.load(Ordering::Acquire)
                            || opening_daemon.rec.status().state != recorder::State::Finishing
                        {
                            assert!(
                                std::time::Instant::now() < deadline,
                                "cleanup never requested a drain"
                            );
                            thread::sleep(Duration::from_millis(1));
                        }
                        fs::write(dir.join("cleanup-wait-reached"), b"yes").unwrap();
                        if kind == "open" {
                            return Err(io::Error::other(
                                "test startup open failed during cleanup",
                            ));
                        }
                        if kind == "header-close" {
                            return Ok(Box::new(BlockedCloseSink {
                                inner: FailedSink {
                                    sync: false,
                                    header_written: true,
                                },
                                release: released,
                            })
                                as Box<dyn recorder::RecordingSink>);
                        }
                        Ok(Box::new(FailedSink {
                            sync: kind == "sync",
                            header_written: kind == "header",
                        })
                            as Box<dyn recorder::RecordingSink>)
                    })
                    .map(Some)
                },
                |_| {
                    if replacing {
                        if stopping_daemon.rec.status().state == recorder::State::Starting {
                            hold_open.send(()).unwrap();
                            let deadline = std::time::Instant::now() + Duration::from_secs(2);
                            while stopping_daemon.rec.status().state == recorder::State::Starting {
                                assert!(std::time::Instant::now() < deadline);
                                thread::sleep(Duration::from_millis(1));
                            }
                            stopping_daemon.rec.request_stop(None);
                        } else {
                            stopping_daemon.rec.request_stop(None);
                        }
                        let deadline = std::time::Instant::now() + Duration::from_secs(2);
                        while !stopping_daemon.rec.status().finished {
                            assert!(std::time::Instant::now() < deadline);
                            thread::sleep(Duration::from_millis(1));
                        }
                        let original = stopping_daemon
                            .rec
                            .stop_timeout(None, Duration::from_secs(2))
                            .unwrap();
                        assert_eq!(original.error.is_none(), replacement_sync);
                        stopping_daemon
                            .rec
                            .start_with_sink(
                                &replacement,
                                2,
                                false,
                                Box::new(FailedSink {
                                    sync: replacement_sync,
                                    header_written: false,
                                }),
                            )
                            .unwrap();
                        assert!(stopping_daemon.rec.status().error.is_none());
                        fs::write(&replacement_marker, b"yes").unwrap();
                    }
                    Some("test shutdown request".into())
                },
            );
            drop(hold_open);
            panic!("production startup returned after a shutdown request");
        }
        for (kind, expected_code, message) in [
            ("open", 1, "test startup open failed during cleanup"),
            ("header", 1, "test storage write failed"),
            ("header-close", 1, "test storage write failed"),
            ("replaced", 1, "test startup open failed before replacement"),
            ("sync-replaced", 1, "test storage sync failed"),
            ("replacement-sync", 1, "test storage sync failed"),
            ("sync", 1, "test storage sync failed"),
            (
                "pending",
                0,
                "startup recording is still waiting for storage; file may be incomplete",
            ),
            ("complete", 0, "cv1touchd: stopped"),
        ] {
            let fixture = socket_dir::SocketDir::new();
            let dir = fixture.path();
            let mut child = process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "tests::startup_recording_cleanup_reports_its_final_storage_outcome",
                    "--nocapture",
                ])
                .env(CHILD, kind)
                .env("CV1TOUCH_TEST_RECORD_CLEANUP_DIR", dir)
                .stdout(process::Stdio::piped())
                .stderr(process::Stdio::piped())
                .spawn()
                .unwrap();
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            let timely = loop {
                if child.try_wait().unwrap().is_some() {
                    break true;
                }
                if std::time::Instant::now() >= deadline {
                    child.kill().unwrap();
                    break false;
                }
                thread::sleep(Duration::from_millis(10));
            };
            let output = child.wait_with_output().unwrap();
            let text = format!(
                "{}{}",
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
            let name = fs::read_to_string(dir.join("shm-name")).unwrap();
            #[cfg(unix)]
            assert!(
                !dir.join("container/control.sock").exists(),
                "startup left its private socket behind"
            );
            assert!(
                timely,
                "startup cleanup did not finish promptly for {kind}: {text}"
            );
            assert!(
                !dir.join("control.sock").exists(),
                "startup cleanup left its socket behind"
            );
            assert!(
                shm::ShmReader::open(&name).is_err(),
                "startup cleanup left shared memory live"
            );
            if kind == "replaced" || kind == "sync-replaced" || kind == "replacement-sync" {
                assert!(
                    dir.join("replacement-installed").exists(),
                    "failed session was not replaced: {text}"
                );
            } else if kind != "pending" {
                assert!(
                    dir.join("cleanup-wait-reached").exists(),
                    "open did not wait for cleanup: {text}"
                );
            }
            assert_eq!(output.status.code(), Some(expected_code), "{kind}: {text}");
            let diagnostics: Vec<_> = text.lines().filter(|line| line.contains(message)).collect();
            assert_eq!(
                diagnostics.len(),
                1,
                "{kind}: expected one diagnostic: {text}"
            );
            if expected_code == 1 {
                assert!(
                    diagnostics[0].starts_with("cv1touchd: recording failed:"),
                    "{kind}: writer must report before close: {text}"
                );
            }
            if kind == "complete" {
                assert!(
                    !text.contains("file may be incomplete") && !text.contains("recording failed"),
                    "{text}"
                );
            }
        }
    }

    #[test]
    fn a_startup_error_queued_during_the_shutdown_wait_remains_fatal() {
        for signal in [false, true] {
            let d = Arc::new(status_daemon());
            let fixture = socket_dir::SocketDir::new();
            let dir = fixture.path();
            let (listeners, endpoints) = startup_test_listeners(dir);
            let control = serve_control(Arc::clone(&d), listeners);
            let (fail, fail_rx) = std::sync::mpsc::channel();
            let starting = d
                .rec
                .start_in_background_with(
                    &std::env::temp_dir().join("cv1t-startup-race.rec"),
                    1,
                    move || {
                        let _ = fail_rx.recv();
                        Err(io::Error::other("test startup open failed during shutdown"))
                    },
                )
                .unwrap();
            let mut starting = Some(starting);
            let mut fail = Some(fail);
            let result = wait_for_recording_start(&d, &mut starting, |_| {
                fail.take().unwrap().send(()).unwrap();
                let deadline = std::time::Instant::now() + Duration::from_secs(2);
                while !d.rec.status().finished {
                    assert!(std::time::Instant::now() < deadline);
                    thread::sleep(Duration::from_millis(1));
                }
                if signal {
                    Some("test shutdown request".into())
                } else {
                    d.stop_requested.store(true, Ordering::Release);
                    None
                }
            });
            assert!(!result.unwrap());
            assert!(starting.is_some(), "shutdown dropped its startup outcome");
            assert_eq!(
                finish_recording_startup(&d, &control, &endpoints, starting.as_ref(), None),
                1
            );
            assert!(!endpoints.runtime.exists());
            #[cfg(unix)]
            assert!(!endpoints.container.as_ref().unwrap().exists());
        }
    }

    /// Hardware opens only after the startup header lands, so the wait must notice the header
    /// within a short poll, not after sleeping out a whole heartbeat interval.
    #[test]
    fn tracking_starts_within_one_poll_of_the_startup_header() {
        let d = Arc::new(status_daemon());
        let path = std::env::temp_dir().join("cv1t-startup-poll.rec");
        let (release, release_rx) = std::sync::mpsc::channel::<()>();
        let (_unused, unused_rx) = std::sync::mpsc::channel::<()>();
        let mut starting = Some(
            d.rec
                .start_in_background_with(&path, 1, move || {
                    // Dropping the test's sender also releases the open on assertion failure.
                    let _ = release_rx.recv();
                    Ok(Box::new(BlockedSink {
                        entered: None,
                        release: unused_rx,
                        sync: false,
                        header_written: false,
                        block_header: false,
                    }) as Box<dyn recorder::RecordingSink>)
                })
                .unwrap(),
        );
        let mut waits = Vec::new();
        let mut release = Some(release);
        let ready = wait_for_recording_start(&d, &mut starting, |within| {
            waits.push(within);
            if let Some(release) = release.take() {
                release.send(()).unwrap();
            }
            let deadline = std::time::Instant::now() + Duration::from_secs(2);
            while d.rec.status().state != recorder::State::Recording {
                assert!(std::time::Instant::now() < deadline, "no header");
                thread::sleep(Duration::from_millis(1));
            }
            None
        });
        let _ = d.rec.stop_timeout(None, Duration::from_secs(2));
        d.shm.retire();
        assert_eq!(ready, Ok(true));
        assert!(starting.is_none());
        assert!(
            waits
                .iter()
                .all(|within| *within <= Duration::from_millis(50)),
            "waited {waits:?} for a header that was already written"
        );
    }

    /// The running message used to hold mode while taking opts, reversing status's order.
    /// Check at the real emission boundary before reading status, so a regression fails
    /// immediately rather than leaving a deadlocked test thread behind.
    #[test]
    fn startup_logging_releases_configuration_locks_before_emitting() {
        let d = status_daemon();
        *d.mode.lock().unwrap() = "calibrated".into();
        {
            let mut opts = d.opts.lock().unwrap();
            opts.hmd_leds_off = false;
            opts.ignore_sensors = vec!["camera1".into(), "camera2".into()];
        }
        let mut emitted = false;
        log_running(&d, "test-segment", Path::new("control.sock"), |message| {
            emitted = true;
            assert!(
                d.mode.try_lock().is_ok(),
                "startup logging still holds the mode lock"
            );
            assert!(
                d.opts.try_lock().is_ok(),
                "startup logging still holds the engine-options lock"
            );
            let message = message.to_string();
            assert!(message.contains("calibrated mode, hmd leds on"));
            assert!(message.contains("ignoring sensors [\"camera1\", \"camera2\"]"));
            let status: serde_json::Value =
                serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
            assert_eq!(status["mode"], "calibrated");
            assert_eq!(
                status["ignored_sensors"],
                serde_json::json!(["camera1", "camera2"])
            );
        });
        assert!(emitted, "startup message was not logged");
    }

    /// The driver reads the heartbeat while startup storage is still opening the file.
    #[test]
    fn the_heartbeat_keeps_beating_while_the_startup_header_is_pending() {
        let d = Arc::new(status_daemon());
        let path = std::env::temp_dir().join("cv1t-startup-beat.rec");
        let (release, release_rx) = std::sync::mpsc::channel::<()>();
        let (_unused, unused_rx) = std::sync::mpsc::channel::<()>();
        let mut starting = Some(
            d.rec
                .start_in_background_with(&path, 1, move || {
                    let _ = release_rx.recv();
                    Ok(Box::new(BlockedSink {
                        entered: None,
                        release: unused_rx,
                        sync: false,
                        header_written: false,
                        block_header: false,
                    }) as Box<dyn recorder::RecordingSink>)
                })
                .unwrap(),
        );
        let status: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        let reader = shm::ShmReader::open(status["shm"].as_str().unwrap()).unwrap();
        let mut seen = std::collections::BTreeSet::new();
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        let ready = wait_for_recording_start(&d, &mut starting, |within| {
            thread::sleep(within);
            seen.insert(reader.heartbeat_ns());
            (seen.len() >= 3 || std::time::Instant::now() >= deadline)
                .then(|| "test shutdown request".to_string())
        });
        drop(release);
        let _ = d.rec.stop_timeout(None, Duration::from_secs(2));
        d.shm.retire();
        assert_eq!(ready, Ok(false));
        assert!(
            seen.len() >= 3,
            "the heartbeat changed only {} times while the header was pending",
            seen.len()
        );
    }

    struct FailedSink {
        sync: bool,
        header_written: bool,
    }
    impl Write for FailedSink {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            if self.header_written && !self.sync {
                return Err(io::Error::other("test storage write failed"));
            }
            self.header_written = true;
            Ok(bytes.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    impl recorder::RecordingSink for FailedSink {
        fn sync_all(&mut self) -> io::Result<()> {
            if self.sync {
                Err(io::Error::other("test storage sync failed"))
            } else {
                Ok(())
            }
        }
        fn disk_space(&self) -> io::Result<recorder::DiskSpace> {
            Ok(recorder::DiskSpace {
                available: 100 << 30,
                capacity: 200 << 30,
            })
        }
    }

    #[test]
    fn failed_recording_write_or_sync_never_gets_a_success_reply() {
        for sync in [false, true] {
            let d = status_daemon();
            let path = std::env::temp_dir().join("cv1t-failed-sink.rec");
            d.rec
                .start_with_sink(
                    &path,
                    1,
                    false,
                    Box::new(FailedSink {
                        sync,
                        header_written: false,
                    }),
                )
                .unwrap();
            d.rec.write(recorder::HAPTIC, 2, &[&[0u8; 16]]);
            let reason = if sync {
                "test storage sync failed"
            } else {
                "test storage write failed"
            };
            if !sync {
                let deadline = std::time::Instant::now() + Duration::from_secs(2);
                while d.rec.status().error.is_none() && std::time::Instant::now() < deadline {
                    thread::sleep(Duration::from_millis(1));
                }
                let value: serde_json::Value =
                    serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
                assert_eq!(value["recording_status"]["state"], "failed");
                assert_eq!(value["recording_status"]["pending"], true);
                assert!(value["recording_status"]["error"]
                    .as_str()
                    .unwrap()
                    .contains(reason));
            }
            let reply = handle(&d, "record stop");
            assert!(
                reply.starts_with("err ") && reply.contains(reason),
                "{reply}"
            );
            let value: serde_json::Value =
                serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
            assert_eq!(value["recording_status"]["state"], "failed");
            assert!(value["recording_status"]["error"]
                .as_str()
                .unwrap()
                .contains(reason));
            assert_eq!(value["write_errors"], 1);
        }
    }

    struct ReserveSink {
        header_written: bool,
    }
    impl Write for ReserveSink {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            self.header_written = true;
            Ok(bytes.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    impl recorder::RecordingSink for ReserveSink {
        fn sync_all(&mut self) -> io::Result<()> {
            Ok(())
        }
        fn disk_space(&self) -> io::Result<recorder::DiskSpace> {
            Ok(recorder::DiskSpace {
                available: if self.header_written {
                    1 << 30
                } else {
                    100 << 30
                },
                capacity: 40 << 30,
            })
        }
    }

    #[test]
    fn reserve_ended_recording_reports_its_complete_file_but_not_a_successful_capture() {
        let d = status_daemon();
        let path = std::env::temp_dir().join("cv1t-reserve-sink.rec");
        d.rec
            .start_with_sink(
                &path,
                1,
                false,
                Box::new(ReserveSink {
                    header_written: false,
                }),
            )
            .unwrap();
        d.rec.write(recorder::HAPTIC, 2, &[&[0u8; 16]]);
        let reply = handle(&d, &control::record_stop_if_line(&path).unwrap());
        assert!(reply.starts_with("err recording ended early:"), "{reply}");
        assert!(
            reply.contains("1.00 GiB free; keeping 2.00 GiB free"),
            "{reply}"
        );
        assert!(reply.contains("file is complete and synced"), "{reply}");
        let value: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(value["recording_status"]["state"], "complete");
        assert_eq!(value["recording_status"]["finished"], true);
        assert!(value["recording_status"]["reason"].is_string());
        assert_eq!(value["write_errors"], 0);
    }

    #[test]
    fn scoped_record_stop_reports_outcome_and_preserves_unrelated_session() {
        let d = status_daemon();
        let path = std::env::temp_dir().join(format!("cv1touch-scoped-stop-{}.rec", shm::now_ns()));
        let other = path.with_extension("other.rec");
        let status: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(status["recording_scoped_stop"], true);
        assert_eq!(status["recording_durable_stop"], true);
        let stop = control::record_stop_if_line(&path).unwrap();
        assert_eq!(handle(&d, &stop), "ok not-matching");
        assert_eq!(
            handle(&d, &control::record_start_line(&path, true).unwrap()),
            "ok"
        );
        assert_eq!(d.rec.status().byte_limit, 16 << 30);
        assert_eq!(
            handle(&d, &control::record_stop_if_line(&other).unwrap()),
            "ok not-matching"
        );
        assert!(d.rec.active());
        assert!(d.record_frames.load(Ordering::Acquire));
        assert!(handle(&d, &stop).starts_with("ok stopped records="));
        assert!(!d.rec.has_session());
        assert!(!d.record_frames.load(Ordering::Acquire));
        assert_eq!(handle(&d, &stop), "ok not-matching");
        fs::remove_file(path).unwrap();
    }

    /// A control client's `record start` is what every capture sends. The recording must open
    /// for the sensors' pixels when asked, and a second recording in one daemon's lifetime must
    /// write its sensor and LED model records again.
    #[test]
    fn a_control_record_start_opens_the_file_and_prepares_its_session() {
        let d = Arc::new(status_daemon());
        let fixture = socket_dir::SocketDir::new();
        let dir = fixture.path();
        let path = dir.join("capture.rec");
        let endpoint = dir.join("control.sock");
        let _server = serve_control(
            Arc::clone(&d),
            vec![ipc::Listener::bind(&endpoint).unwrap()],
        );
        let request = |line: &str| control::request(&endpoint, line, Duration::from_secs(5));
        // The previous recording already wrote its session info.
        assert_eq!(
            d.session.lock().unwrap().plan(true, &[(7, true)]),
            (true, vec![7])
        );
        assert!(d.session.lock().unwrap().complete(1));
        assert!(!d.record_frames.load(Ordering::Acquire));

        let reply = request(&control::record_start_line(&path, true).unwrap()).unwrap();

        assert_eq!(reply, "ok");
        assert!(d.rec.active());
        assert!(d.record_frames.load(Ordering::Acquire));
        assert_eq!(d.rec.status().byte_limit, 16 << 30);
        assert_eq!(
            d.session.lock().unwrap().plan(true, &[(7, true)]),
            (true, vec![7]),
            "the new recording must get its own sensor and LED model records"
        );
        let stopped = request(&control::record_stop_if_line(&path).unwrap()).unwrap();
        let written = fs::metadata(&path).map(|m| m.len());
        d.shm.retire();
        assert!(stopped.starts_with("ok stopped records="), "{stopped}");
        assert!(written.unwrap() > 0, "the recording file has its header");
        assert!(!d.record_frames.load(Ordering::Acquire));
    }

    #[test]
    fn status_names_the_shared_memory_the_poses_are_in() {
        let d = status_daemon();
        let value: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        let name = value["shm"].as_str().unwrap();
        assert!(name.starts_with("/cv1t-dynamic-status-"), "{name}");
        assert!(shm::ShmReader::open(name).is_ok());
    }

    #[test]
    fn status_names_sensors_explicitly_left_closed() {
        let d = status_daemon();
        d.opts.lock().unwrap().ignore_sensors = vec!["IGNOREDCAMERA".into()];
        let value: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(
            value["ignored_sensors"],
            serde_json::json!(["IGNOREDCAMERA"])
        );
    }

    #[test]
    fn status_names_the_active_sensor_calibration_even_before_its_engine_opens() {
        let d = status_daemon();
        let opts = EngineOptions {
            fixed_poses: ["CAMERA1", "CAMERA2"]
                .map(|serial| engine::SensorPose {
                    serial: serial.into(),
                    orient: [0.0, 0.0, 0.0, 1.0],
                    pos: [0.0; 3],
                })
                .to_vec(),
            ..Default::default()
        };
        switch_mode(&d, opts.clone(), "calibrated");
        let value: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(
            value["calibrated_sensors"],
            serde_json::json!(["CAMERA1", "CAMERA2"])
        );
        switch_mode(&d, modes::capture(&opts), "capture");
        let value: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert!(value
            .as_object()
            .unwrap()
            .contains_key("calibrated_sensors"));
        assert!(value["calibrated_sensors"].is_null());
    }

    #[test]
    fn status_publishes_received_video_by_serial_at_full_u64_width() {
        let d = status_daemon();
        let mut sensor = SensorStatus {
            frames: 41,
            ..Default::default()
        };
        shm::set_serial(&mut sensor, "Camera1");
        let video = engine::SensorVideo {
            serial: "CAMERA1".into(),
            frames_received: 4_294_967_348,
        };
        publish_sensor_snapshot(&d, &[sensor], &[], &[video]);
        let value: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(value["sensors"][0]["frames"], 41);
        assert_eq!(value["sensors"][0]["frames_received"], 4_294_967_348u64);
        assert_eq!(std::mem::size_of::<ffi::ohmd_cv1touch_sensor_video>(), 40);
        publish_disconnected(&d);
        assert!(d.sensor_video.lock().unwrap().is_empty());
    }

    #[test]
    fn status_keeps_an_unavailable_received_counter_distinct_from_zero() {
        let d = status_daemon();
        let mut sensor = SensorStatus::default();
        shm::set_serial(&mut sensor, "CAMERA1");
        publish_sensor_snapshot(&d, &[sensor], &[], &[]);
        let value: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert!(value["sensors"][0]["frames_received"].is_null());
    }

    #[test]
    fn status_advertises_received_video_even_with_an_empty_sensor_list() {
        let d = status_daemon();
        let value: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(value["sensor_frames_received"], true);
        assert_eq!(value["sensors"], serde_json::json!([]));
    }

    #[test]
    fn status_keeps_failed_starts_and_actual_stream_settings_without_changing_sensor_counts() {
        let d = status_daemon();
        let mut failed = ffi::ohmd_cv1touch_sensor_stream {
            error_kind: ffi::OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH,
            error_code: -99,
            os_error: 28,
            ..Default::default()
        };
        for (out, byte) in failed.serial.iter_mut().zip(b"FAILED") {
            *out = *byte as _;
        }
        let mut active = ffi::ohmd_cv1touch_sensor_stream {
            alt_setting: 1,
            ..Default::default()
        };
        for (out, byte) in active.serial.iter_mut().zip(b"LOWER") {
            *out = *byte as _;
        }
        let mut sensor = SensorStatus::default();
        shm::set_serial(&mut sensor, "LOWER");
        publish_sensor_snapshot(&d, &[sensor], &[failed.into(), active.into()], &[]);
        let value: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(value["sensor_total"], 1);
        assert_eq!(
            value["sensor_streams"],
            serde_json::json!([
                {"scope": "last_start_attempt", "serial": "FAILED", "error_kind": "likely_usb_bandwidth", "error_code": -99, "os_error": 28, "alt_setting": null},
                {"scope": "last_start_attempt", "serial": "LOWER", "error_kind": "none", "error_code": 0, "os_error": 0, "alt_setting": 1}
            ])
        );
        assert_eq!(std::mem::size_of::<ffi::ohmd_cv1touch_sensor_stream>(), 64);
        publish_disconnected(&d);
        let value: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(value["sensor_streams"], serde_json::json!([]));
    }

    #[test]
    fn status_does_not_report_the_engine_ready_before_initialization() {
        let d = status_daemon();
        let reply = status_json(&d);
        let value: serde_json::Value =
            serde_json::from_str(reply.strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(value["engine_ready"], false);
    }

    #[test]
    fn reload_acknowledgement_cannot_confirm_old_engine_options() {
        let d = status_daemon();
        let (_, first) = requested_engine_options(&d);
        d.engine_applied.store(first, Ordering::Release);
        assert!(engine_is_ready(&d, &d.opts.lock().unwrap()));
        assert_eq!(
            switch_mode(&d, Default::default(), "calibrated"),
            "ok calibrated"
        );
        assert_eq!(*d.mode.lock().unwrap(), "calibrated");
        assert!(!engine_is_ready(&d, &d.opts.lock().unwrap()));
        // An older open can finish after a newer request without satisfying it.
        d.engine_applied.store(first, Ordering::Release);
        assert!(!engine_is_ready(&d, &d.opts.lock().unwrap()));
        let (_, latest) = requested_engine_options(&d);
        assert_ne!(latest, first);
        d.engine_applied.store(latest, Ordering::Release);
        assert!(engine_is_ready(&d, &d.opts.lock().unwrap()));
        publish_disconnected(&d);
        assert!(!engine_is_ready(&d, &d.opts.lock().unwrap()));
    }

    #[test]
    fn authoritative_status_reports_all_sensors_and_an_explicit_shm_prefix() {
        let d = status_daemon();
        let sensors: Vec<_> = (0..65)
            .map(|i| {
                let mut s = SensorStatus {
                    frames: i + 1,
                    ..Default::default()
                };
                shm::set_serial(&mut s, &format!("synthetic-camera-{i}"));
                s
            })
            .collect();
        publish_sensor_snapshot(&d, &sensors, &[], &[]);
        let reply = status_json(&d);
        let v: serde_json::Value =
            serde_json::from_str(reply.strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(v["sensors"].as_array().unwrap().len(), 65);
        assert_eq!(v["sensor_total"], 65);
        assert_eq!(v["sensor_shm_count"], 8);
        assert_eq!(v["sensor_shm_truncated"], true);
        assert_eq!(v["sensors"][64]["frames"], 65);
        assert_eq!(d.shm.sensors().unwrap().len(), 8);
        publish_disconnected(&d);
        let reply = status_json(&d);
        let v: serde_json::Value =
            serde_json::from_str(reply.strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(v["sensor_total"], 0);
        assert!(v["sensors"].as_array().unwrap().is_empty());
    }

    fn input_config() -> input_calibration::CalibrationFile {
        use input_calibration::{CalibrationFile, Endpoint, HandCalibration, StickAxis};
        let hand = |identity: String| HandCalibration {
            identity,
            trigger: Endpoint {
                released: -0.1,
                pressed: 0.9,
            },
            grip: Endpoint {
                released: -0.2,
                pressed: 0.8,
            },
            stick: [StickAxis {
                min: -1.0,
                center: 0.0,
                max: 1.0,
            }; 2],
        };
        CalibrationFile::new([hand("a".repeat(32)), hand("b".repeat(32))]).unwrap()
    }

    #[test]
    fn raw_buttons_preserve_physical_bits_before_injection() {
        let mut runtime = InputRuntime::default();
        runtime.set_identity(0, "a".repeat(32));
        let physical = cv1touch::hand::BTN_X | cv1touch::hand::BTN_STICK | (1 << 31);
        let mut state = ControllerState {
            flags: ffi::CV1T_FLAG_CONNECTED,
            buttons: physical,
            received_host_ns: 42,
            ..Default::default()
        };
        runtime.apply(0, &mut state);
        let injection = Injection::default();
        injection.arm(42);
        injection
            .press(
                0,
                control::Injected {
                    buttons: cv1touch::hand::BTN_Y,
                    ..Default::default()
                },
                100,
                42,
            )
            .unwrap();
        injection.apply(0, 42, &mut state);
        assert_ne!(state.buttons, physical);
        let snapshot = runtime.latest.clone();
        let json = serde_json::to_value(snapshot).unwrap();
        assert_eq!(json[0]["buttons"], physical);
        assert_eq!(json[0]["received_host_ns"], 42);
        assert_eq!(json[1]["buttons"], 0);
    }

    #[test]
    fn raw_buttons_reset_on_identity_change_and_disconnect() {
        let d = status_daemon();
        {
            let mut runtime = d.inputs.lock().unwrap();
            runtime.set_identity(0, "a".repeat(32));
            let mut state = ControllerState {
                flags: ffi::CV1T_FLAG_CONNECTED,
                buttons: cv1touch::hand::BTN_X,
                received_host_ns: 42,
                ..Default::default()
            };
            runtime.apply(0, &mut state);
            runtime.set_identity(0, "b".repeat(32));
            let json = serde_json::to_value(&runtime.latest).unwrap();
            assert_eq!(json[0]["buttons"], 0);
            assert_eq!(json[0]["received_host_ns"], 0);
            runtime.apply(0, &mut state);
        }
        publish_disconnected(&d);
        let runtime = d.inputs.lock().unwrap();
        let json = serde_json::to_value(&runtime.latest).unwrap();
        assert_eq!(json[0]["buttons"], 0);
        assert_eq!(json[0]["connected"], false);
        assert_eq!(json[0]["received_host_ns"], 0);
        assert_eq!(json[0]["identity"], "");
    }

    #[test]
    fn publication_clamps_defaults_and_keeps_raw_values_for_calibration() {
        let mut runtime = InputRuntime::default();
        let mut state = ControllerState {
            flags: ffi::CV1T_FLAG_CONNECTED,
            received_host_ns: 42,
            trigger: f32::INFINITY,
            grip: -0.2,
            stick: [1.2, f32::NAN],
            ..Default::default()
        };
        runtime.apply(0, &mut state);
        assert_eq!(
            (state.trigger, state.grip, state.stick),
            (0.0, 0.0, [1.0, 0.0])
        );
        assert!(runtime.latest[0].trigger.is_infinite());
        assert_eq!(runtime.latest[0].grip, -0.2);
        assert_eq!(runtime.latest[0].received_host_ns, 42);
        let raw = serde_json::to_value(&runtime.latest).unwrap();
        assert!(raw[0]["trigger"].is_null());
    }

    #[test]
    fn correction_preserves_measurement_and_dropout_releases_after_offset() {
        let mut runtime = InputRuntime {
            calibration: Some(input_config()),
            ..Default::default()
        };
        runtime.set_identity(0, "a".repeat(32));
        let mut state = ControllerState {
            flags: ffi::CV1T_FLAG_CONNECTED,
            trigger: 0.9,
            grip: 0.8,
            received_host_ns: 42,
            ..Default::default()
        };
        runtime.apply(0, &mut state);
        assert_eq!((state.trigger, state.grip), (1.0, 1.0));
        assert_eq!(
            (runtime.latest[0].trigger, runtime.latest[0].grip),
            (0.9, 0.8)
        );
        state.flags = 0;
        state.trigger = 0.0;
        state.grip = 0.0;
        runtime.apply(0, &mut state);
        assert_eq!(
            (state.trigger, state.grip, state.stick),
            (0.0, 0.0, [0.0; 2])
        );
        assert!(!runtime.latest[0].connected);
    }

    #[test]
    fn device_change_invalidates_old_sample_and_uses_factory_fallback() {
        let mut runtime = InputRuntime {
            calibration: Some(input_config()),
            ..Default::default()
        };
        runtime.set_identity(0, "a".repeat(32));
        runtime.latest[0].connected = true;
        runtime.latest[0].received_host_ns = 42;
        runtime.set_identity(0, "c".repeat(32));
        assert!(!runtime.latest[0].connected);
        assert_eq!(runtime.latest[0].received_host_ns, 0);
        assert!(!runtime.calibration_applies(0));
        let mut state = ControllerState {
            flags: ffi::CV1T_FLAG_CONNECTED,
            grip: 0.8,
            ..Default::default()
        };
        runtime.apply(0, &mut state);
        assert_eq!(state.grip, 0.8);
    }

    #[test]
    fn invalid_live_reload_retains_active_config_and_missing_file_resets_it() {
        let dir = std::env::temp_dir().join(format!(
            "cv1touch-input-reload-{}-{}",
            std::process::id(),
            shm::now_ns()
        ));
        cv1touch::platform::fs::create_private_dir_all(&dir).unwrap();
        let path = dir.join("input.json");
        let original = input_config();
        let runtime = Mutex::new(InputRuntime {
            calibration: Some(original.clone()),
            ..Default::default()
        });
        fs::write(&path, b"{broken}").unwrap();
        assert!(reload_inputs(&runtime, &path).is_err());
        assert_eq!(
            runtime.lock().unwrap().calibration.as_ref(),
            Some(&original)
        );
        fs::remove_file(&path).unwrap();
        reload_inputs(&runtime, &path).unwrap();
        assert!(runtime.lock().unwrap().calibration.is_none());
        fs::remove_dir(dir).unwrap();
    }

    #[test]
    fn an_armed_inject_presses_for_its_duration_only() {
        let inj = Injection::default();
        let x = control::Injected {
            buttons: cv1touch::hand::BTN_X,
            touches: cv1touch::hand::TOUCH_X,
            trigger: true,
            grip: false,
        };
        let pressed = |at: u64| {
            let mut st = ControllerState {
                buttons: cv1touch::hand::BTN_Y,
                trigger: 0.25,
                grip: 0.5,
                ..Default::default()
            };
            inj.apply(HAND_LEFT, at, &mut st);
            st
        };
        assert!(
            inj.press(HAND_LEFT, x, 200, 1_000).is_err(),
            "not while disarmed"
        );
        inj.arm(1_000);
        inj.press(HAND_LEFT, x, 200, 1_000).unwrap();
        let st = pressed(100_000_000);
        assert_eq!(
            (st.buttons, st.touches, st.trigger, st.grip),
            (
                cv1touch::hand::BTN_X | cv1touch::hand::BTN_Y,
                cv1touch::hand::TOUCH_X,
                1.0,
                0.5
            ),
            "ORed in, the rest kept"
        );
        assert_eq!(
            pressed(201_000_001).buttons,
            cv1touch::hand::BTN_Y,
            "released after 200 ms"
        );
        let mut right = ControllerState::default();
        inj.apply(HAND_RIGHT, 100_000_000, &mut right);
        assert_eq!(right.buttons, 0, "the other hand untouched");
        // a new inject for the hand replaces the old; disarming (or the arming running out) releases
        inj.press(
            HAND_LEFT,
            control::Injected {
                grip: true,
                ..Default::default()
            },
            2000,
            300_000_000,
        )
        .unwrap();
        assert_eq!(
            (pressed(400_000_000).buttons, pressed(400_000_000).grip),
            (cv1touch::hand::BTN_Y, 1.0)
        );
        inj.disarm();
        assert_eq!(pressed(400_000_000).grip, 0.5);
        inj.arm(0);
        inj.press(HAND_LEFT, x, 2000, INJECT_ARMED_NS - 1_000_000)
            .unwrap();
        assert_eq!(
            pressed(INJECT_ARMED_NS + 1).buttons,
            cv1touch::hand::BTN_Y,
            "armed for INJECT_ARMED_NS only"
        );
    }

    #[test]
    fn one_thread_at_a_time_publishes_a_hand() {
        let o = HandOwnership::default();
        // the engine publishes until the radio thread asks for the hand
        assert!(o.engine_publishes(HAND_LEFT) && !o.radio_publishes(HAND_LEFT));
        o.request(HAND_LEFT);
        assert!(
            !o.radio_publishes(HAND_LEFT),
            "not before the engine lets go"
        );
        // the engine lets go between two of its publishes, and only then may the radio thread write
        assert!(!o.engine_publishes(HAND_LEFT));
        assert!(o.radio_publishes(HAND_LEFT) && !o.engine_publishes(HAND_LEFT));
        assert!(
            o.engine_publishes(HAND_RIGHT),
            "the other hand stays the engine's"
        );
        // once the radio thread is gone (after its last publish) the engine has every hand again
        o.radio_exit();
        assert!(o.engine_publishes(HAND_LEFT) && o.engine_publishes(HAND_RIGHT));
    }

    #[test]
    fn default_affinity_preserves_a_restricted_thread() {
        thread::spawn(|| {
            let me = system::current_thread();
            let cpu = system::allowed_cpus(me).unwrap()[0];
            set_rt_affinity_with(me, Some(&cpu.to_string())).unwrap();
            set_rt_affinity_with(me, None).unwrap();
            assert_eq!(
                system::allowed_cpus(me).unwrap(),
                vec![cpu],
                "default affinity must not widen or replace the inherited mask"
            );
        })
        .join()
        .unwrap();
    }

    #[test]
    fn explicit_affinity_validates_ranges_and_the_allowed_mask() {
        let allowed = [0, 2, 3, 7];
        assert_eq!(
            affinity_cpus("0,2-3,7", &allowed).unwrap(),
            vec![0, 2, 3, 7]
        );
        for spec in [
            "",
            "1",
            "0-3",
            "7-2",
            "a",
            "0,,2",
            "2-",
            "1024",
            "0-999999999",
        ] {
            assert!(affinity_cpus(spec, &allowed).is_err(), "accepted {spec:?}");
        }
    }

    #[test]
    fn a_radio_sample_goes_out_over_the_engines_view_without_its_clock() {
        let view = engine::Controller {
            calibrated: true,
            device_time_ns: 9_000_000,
            received_host_ns: 1,
            last_pose_fix_device_ns: 8_990_000,
            last_orient_fix_device_ns: 8_995_000,
            view_pos: [0.1, 1.2, -0.3],
            device_id: 2,
            ..Default::default()
        };
        let s = engine::Controller {
            calibrated: true,
            device_time_ns: 42_000,
            received_host_ns: 77,
            imu_gyro: [1.0, 0.0, 0.0],
            imu_accel: [0.0, 9.8, 0.0],
            buttons: 1,
            trigger: 0.5,
            device_id: 2,
            ..Default::default()
        };
        let c = radio_sample(&view, &s);
        assert_eq!(
            (
                c.device_time_ns,
                c.received_host_ns,
                c.imu_gyro,
                c.imu_accel,
                c.buttons,
                c.trigger
            ),
            (42_000, 77, [1.0, 0.0, 0.0], [0.0, 9.8, 0.0], 1, 0.5)
        );
        assert!(
            c.calibrated && c.view_pos == [0.1, 1.2, -0.3] && c.device_id == 2,
            "the engine's view carries over"
        );
        assert_eq!(
            (c.last_pose_fix_device_ns, c.last_orient_fix_device_ns),
            (0, 0),
            "the engine's fix times are on another clock"
        );
    }

    #[cfg(target_os = "linux")]
    #[test]
    #[ignore = "needs rtkit headroom: rtkit grants a user at most 25 threads, and a running SteamVR holds 14"]
    fn realtime_request_makes_only_the_calling_thread_round_robin() {
        thread::spawn(|| {
            system::make_realtime(system::current_thread(), RADIO_RT_PRIORITY).unwrap();
            // SAFETY: plain queries of the calling thread (pid 0).
            let (policy, prio) = unsafe {
                let mut p: libc::sched_param = mem::zeroed();
                libc::sched_getparam(0, &mut p);
                (libc::sched_getscheduler(0), p.sched_priority)
            };
            assert_eq!(policy & !libc::SCHED_RESET_ON_FORK, libc::SCHED_RR);
            assert_eq!(prio as u32, RADIO_RT_PRIORITY);
            // threads it starts afterwards (OpenHMD's) stay normal
            let child = thread::spawn(|| unsafe { libc::sched_getscheduler(0) })
                .join()
                .unwrap();
            assert_eq!(child, libc::SCHED_OTHER);
        })
        .join()
        .unwrap();
        // SAFETY: as above.
        assert_eq!(
            unsafe { libc::sched_getscheduler(0) },
            libc::SCHED_OTHER,
            "the test's own thread stays normal"
        );
    }

    #[cfg(target_os = "linux")]
    #[test]
    #[ignore = "requires sufficient RLIMIT_MEMLOCK or CAP_IPC_LOCK for mlockall"]
    fn locked_memory_stays_resident_once_touched() {
        system::lock_memory().unwrap();
        let v = vec![1u8; 4 << 20];
        let status = std::fs::read_to_string("/proc/self/status").unwrap();
        let locked_kb: u64 = status
            .lines()
            .find_map(|l| l.strip_prefix("VmLck:"))
            .unwrap()
            .trim()
            .trim_end_matches(" kB")
            .parse()
            .unwrap();
        assert!(
            locked_kb >= 4 << 10,
            "VmLck {locked_kb} kB after touching 4 MB"
        );
        drop(v);
    }

    #[test]
    fn hold_summary_splits_the_hold_into_its_parts() {
        // 100 samples: held 1..=100 us; 10 us in OpenHMD's filter, 30 us in its whole update,
        // 5 us reading back; one sample stuck 4 ms in the filter
        let mut v: Vec<HoldParts> = (1..=100u64)
            .map(|i| HoldParts {
                held_ns: 1000 * i + 50_000,
                filter_ns: 10_000,
                update_ns: 30_000,
                read_ns: 5_000,
            })
            .collect();
        v[99] = HoldParts {
            held_ns: 4_216_000,
            filter_ns: 4_000_000,
            update_ns: 4_020_000,
            read_ns: 5_000,
        };
        assert_eq!(
            hold_summary("left", &mut v),
            "left: IMU sample held 0.10 ms median, 0.15 p99, 4.22 max (radio arrival to driver wake-up, 100 samples); \
             p99/max in OpenHMD's IMU filter 0.01/4.00, rest of its update 0.02/0.02, reading it back 0.01/0.01, ours 0.11/0.19"
        );
    }
}
