// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! cv1touchd: owns the Rift CV1 headset radio and sensors through the vendored OpenHMD engine and
//! publishes both Touch controllers' state to shared memory.

use std::fs;
use std::io::{Read, Write};
use std::mem;
use std::path::PathBuf;
use std::process;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use cv1touch::control::{self, Command};
use cv1touch::engine::{self, Blob, Engine, EngineOptions, EngineSink};
use cv1touch::frame_queue::FrameQueue;
use cv1touch::hand::{Hand, HandConfig};
use cv1touch::platform::{self, hid, ipc, shutdown, system};
use cv1touch::recorder::{self, Recorder};
use cv1touch::shm::{self, ControllerState, SensorStatus, ShmWriter, HAND_LEFT, HAND_RIGHT};
use cv1touch::track::pipeline::{self, Pipeline, Stamped};
use cv1touch::track::predict::{ShapeParams, VelocityShaper};
use cv1touch::track::tracker::Stats;
use cv1touch::{align, calib, ffi, input_calibration, modes, settings, supervise, track};

/// Disable headset tracking LEDs by default while retaining the headset radio for Touch.
const DEFAULT_HMD_LEDS_OFF: bool = true;
const IO_FAILURES_BEFORE_REOPEN: u32 = 50;
const SENSOR_PUBLISH_NS: u64 = 250_000_000;
const RECORDER_RING_BYTES: usize = 64 << 20;
/// Pixels at or below this are treated as sensor dark noise and are not recorded;
/// blobs start at the engine's threshold of 36, so their halos are kept.
const RAW_FRAME_FLOOR: u8 = 7;
const MAX_CLIENTS: usize = 16;

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
        raw.connected = state.flags & ffi::CV1T_FLAG_CONNECTED != 0;
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
    quit: AtomicBool,
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
    track_stats: Mutex<[(Stats, bool); 2]>,
    pending_frames_dropped: std::sync::atomic::AtomicU64,
    /// Per sensor serial: its landmarks' newest shift (px) and how many were measured.
    landmark_shifts: Mutex<std::collections::BTreeMap<String, (f64, f64, usize)>>,
    /// How the Lighthouse universe moved since the registration (see `universe_watch`), with
    /// the registration it was measured against (its `created_unix_s`).
    universe_shift: Mutex<Option<(u64, nalgebra::Isometry3<f64>)>>,
    /// Asks `universe_watch` to measure now rather than at its next check.
    universe_recheck: AtomicBool,
    /// The base stations' raw poses as the SteamVR driver last reported them (`stations`), with
    /// when they arrived (shm clock).
    driver_stations: Mutex<Option<TimedStationPoses>>,
    /// `universe-watch off` stops the watch (the registration stays as last followed).
    universe_watch_on: AtomicBool,
    /// `shaping off` publishes the tracked velocities as they are (see `track::predict`).
    shaping_on: AtomicBool,
    /// Percent of the velocities published for SteamVR's extrapolation (`prediction`), saved
    /// in `settings_path` on every change and read from it at start.
    prediction_percent: AtomicU32,
    settings_path: PathBuf,
    /// The current recording keeps the sensors' pixels (RAW_FRAME records).
    record_frames: AtomicBool,
    /// Extra stamp shift of published poses, ns (`latency` command; 0 by default).
    extra_latency_ns: std::sync::atomic::AtomicI64,
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
            log(format!(
                "{}: IMU biases from the last run (accelerometer {:+.3} {:+.3} {:+.3} m/s^2)",
                ["left", "right"][hand],
                a.x,
                a.y,
                a.z
            ));
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
        log(format!("IMU biases not saved: {e}"));
    }
}

/// Watches the landmarks (static infrared sources such as the base stations) for a moved sensor.
struct LandmarkWatch {
    monitor: track::landmarks::Monitor,
    sensors: std::collections::BTreeSet<String>,
    /// Per sensor: the last alert (shift magnitude), to report changes only.
    alerted: std::collections::BTreeMap<String, f64>,
    /// Consecutive windows without any landmark measured, per sensor.
    missing: std::collections::BTreeMap<String, u32>,
    /// Consecutive windows over the alert threshold, per sensor.
    over: std::collections::BTreeMap<String, u32>,
}

impl LandmarkWatch {
    fn load() -> Option<LandmarkWatch> {
        let path = track::landmarks::default_path();
        match track::landmarks::load(&path) {
            Ok(f) if !f.landmarks.is_empty() => {
                let sensors = f.landmarks.iter().map(|l| l.serial.clone()).collect();
                log(format!(
                    "watching {} landmarks from {} for moved sensors",
                    f.landmarks.len(),
                    path.display()
                ));
                Some(LandmarkWatch {
                    monitor: track::landmarks::Monitor::new(f.landmarks, LANDMARK_WINDOW_NS),
                    sensors,
                    alerted: Default::default(),
                    missing: Default::default(),
                    over: Default::default(),
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

    fn frame(&mut self, d: &Daemon, info: &engine::FrameInfo, blobs: &[Blob]) {
        let serial = engine::cstr_field(&info.serial);
        let unlabelled: Vec<nalgebra::Vector2<f64>> = blobs
            .iter()
            .filter(|b| b.led_id < 0)
            .map(|b| nalgebra::Vector2::new(b.x as f64, b.y as f64))
            .collect();
        let Some(shifts) = self
            .monitor
            .frame(&serial, info.frame_start_host_ns, &unlabelled, 20)
        else {
            return;
        };
        let mut out = d.landmark_shifts.lock().unwrap();
        for sensor in &self.sensors {
            match shifts.iter().find(|s| &s.serial == sensor) {
                Some(s) => {
                    self.missing.remove(sensor);
                    let m = s.shift.norm();
                    out.insert(sensor.clone(), (s.shift.x, s.shift.y, s.landmarks));
                    let last = self.alerted.get(sensor).copied().unwrap_or(0.0);
                    let threshold = if s.landmarks >= 2 {
                        LANDMARK_ALERT_PX
                    } else {
                        LANDMARK_ALERT_SINGLE_PX
                    };
                    let over = self.over.entry(sensor.clone()).or_default();
                    *over = if m >= threshold { *over + 1 } else { 0 };
                    if *over >= 2 && (m - last).abs() >= 0.2 {
                        log(format!("sensor {sensor}: its landmarks moved ({:+.2}, {:+.2}) px: it may have been knocked (cv1touch calibrate refine REC)", s.shift.x, s.shift.y));
                        self.alerted.insert(sensor.clone(), m);
                    } else if *over == 0 && last >= LANDMARK_ALERT_PX {
                        log(format!("sensor {sensor}: landmarks back in place"));
                        self.alerted.insert(sensor.clone(), m);
                    }
                }
                None => {
                    let n = self.missing.entry(sensor.clone()).or_default();
                    *n += 1;
                    if *n == 3 {
                        log(format!("sensor {sensor}: none of its landmarks seen for 3 minutes (moved more than a pixel, or the base stations are off)"));
                    }
                }
            }
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
        if self.rec.active() && !(source == 1 && self.radio_on.load(Ordering::Acquire)) {
            self.rec.write(
                recorder::RAW_REPORT,
                ts,
                &[&(source as u32).to_le_bytes(), data],
            );
        }
        if source == 1 {
            for (hand, pct) in cv1touch::hand::battery_from_radio(data) {
                // logged when it changes: it read 100 for hours on fresh cells, and the first
                // drop will show what the channel really is
                let was = self.battery[hand].swap(pct, Ordering::Relaxed);
                if was != pct {
                    log(format!(
                        "{} Touch battery {pct}%{}",
                        ["left", "right"][hand],
                        if was >= 0 {
                            format!(" (was {was}%)")
                        } else {
                            String::new()
                        }
                    ));
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

/// A line for the log. One that cannot be written is lost: under a supervisor that has ended,
/// standard error is a closed pipe, and the daemon still has to stop cleanly.
fn log(msg: impl AsRef<str>) {
    use std::io::Write;
    let _ = writeln!(std::io::stderr(), "cv1touchd: {}", msg.as_ref());
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
    publish_sensor_snapshot(d, &[]);
    d.shm.signal();
    d.shm.heartbeat(now);
}

fn publish_sensor_snapshot(d: &Daemon, sensors: &[SensorStatus]) {
    *d.sensors.lock().unwrap() = sensors.to_vec();
    d.shm.publish_sensors(sensors);
}

fn sensor_status(s: &engine::Sensor) -> SensorStatus {
    let mut out = SensorStatus::default();
    shm::set_serial(&mut out, &engine::sensor_serial(s));
    out.placed = s.have_camera_pose as u32;
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
    let mut p = Pipeline::new(pipeline::Options::default());
    for s in e.sensors() {
        let serial = engine::sensor_serial(&s);
        if let Some(pose) = opts.fixed_poses.iter().find(|f| f.serial == serial) {
            p.sensor(
                &serial,
                calib::camera::CameraModel::from_sensor(&s),
                calib::iso_from_xyzw(pose.orient, pose.pos),
            );
        }
    }
    d.track_in.lock().unwrap().clear();
    Some(p)
}

/// Puts the tracker's pose into the published state, stamped `extra_ns` earlier than the
/// pipeline's estimate of when the controller was there (the `latency` adjustment).
/// `shaper` turns the velocities into the ones SteamVR's constant-velocity prediction should use
/// (see `track::predict`), and `prediction_percent` of them is published: at 0 SteamVR shows the
/// pose where it was measured. The pose is published as tracked.
fn apply_tracked(
    state: &mut ControllerState,
    s: &Stamped,
    extra_ns: i64,
    shaper: &mut VelocityShaper,
    shaping: bool,
    prediction_percent: u32,
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
    let scale = f64::from(prediction_percent) / 100.0;
    let (w, v) = (w * scale, v.unwrap_or(s.output.vel) * scale);
    state.lin_vel = [v.x as f32, v.y as f32, v.z as f32];
    // the driver takes orient * (ang_vel - gyro_bias) as the world rate: the shaped one, in the body
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
/// How often the watch looks at the stations the driver reports (cheap: no SteamVR connection).
const UNIVERSE_DRIVER_CHECK_MS: u64 = 5_000;
/// The driver's report counts as current this long; past it, the watch connects to SteamVR itself
/// (an older driver that does not report), at most every `UNIVERSE_CHECK_MS`.
const DRIVER_STATIONS_FRESH_NS: u64 = 10_000_000_000;

/// Keeps the registration on the Lighthouse universe it was fitted in. A SteamVR restart can
/// re-bootstrap the universe from another base station and move it by centimetres (24 mm and 0.7
/// deg on 2026-10-08); the stations themselves do not move, so their raw poses, against those the
/// registration recorded, measure the move. Checked once a minute with a short-lived background
/// OpenVR connection (one that never keeps SteamVR from quitting).
fn universe_watch(d: Arc<Daemon>) {
    let mut first = true;
    let mut warned_disagree = false;
    let mut last_connect: Option<std::time::Instant> = None;
    while !d.quit.load(Ordering::Acquire) {
        sleep_until(
            &d,
            if first {
                5_000
            } else {
                UNIVERSE_DRIVER_CHECK_MS
            },
            || d.universe_recheck.swap(false, Ordering::AcqRel),
        );
        first = false;
        if !d.universe_watch_on.load(Ordering::Acquire) {
            continue;
        }
        let Ok(reg) = align::load_registration(&align::registration_path()) else {
            continue;
        };
        if reg.stations.len() < 2 {
            continue;
        }
        // the stations as the driver sees them inside SteamVR; else (an older driver) a short
        // SteamVR connection of our own, once a minute: each one costs vrserver an app connect
        let reported = d
            .driver_stations
            .lock()
            .unwrap()
            .clone()
            .filter(|(at, _)| shm::now_ns().saturating_sub(*at) < DRIVER_STATIONS_FRESH_NS);
        let now = match reported {
            Some((_, v)) => v,
            None => {
                if last_connect
                    .is_some_and(|t| t.elapsed() < Duration::from_millis(UNIVERSE_CHECK_MS))
                {
                    continue;
                }
                last_connect = Some(std::time::Instant::now());
                let Ok(client) = cv1touch::vr::Client::connect() else {
                    continue;
                };
                let v = client.stations();
                drop(client);
                v
            }
        };
        if now.len() < 2 {
            continue;
        }
        let Some(shift) = reg.follow(&now) else {
            if !warned_disagree {
                warned_disagree = true;
                log("universe watch: the base stations disagree on how the universe moved (one bumped?); registration left as it is");
            }
            continue;
        };
        warned_disagree = false;
        let s = shift.now_from_then;
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
        if moved {
            *d.universe_shift.lock().unwrap() = Some((reg.created_unix_s, s));
            let (alignment, _) = align::load_with(Some(&s));
            d.shm.publish_alignment(&alignment);
            if s != nalgebra::Isometry3::identity()
                || prev.is_some_and(|p| p != nalgebra::Isometry3::identity())
            {
                log(format!(
                    "universe watch: the Lighthouse universe moved {:.1} mm, {:.2} deg in the play area since the registration ({} stations, disagreeing there by {:.1} mm rms); registration follows",
                    (s * reg.play_area_point() - reg.play_area_point()).norm() * 1e3,
                    s.rotation.angle().to_degrees(),
                    shift.stations,
                    shift.residual_m * 1e3
                ));
            }
        }
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
        let mut e = match Engine::open(&opts, d.clone()) {
            Ok(e) => e,
            Err(err) => {
                if err != last_err {
                    log(format!("waiting for headset: {err}"));
                    last_err = err;
                }
                publish_disconnected(&d);
                sleep_interruptible(&d, 1000);
                continue;
            }
        };
        d.engine_applied.store(revision, Ordering::Release);
        last_err.clear();
        d.session.lock().unwrap().reset(); // sensors may differ after a reopen
        log(format!(
            "engine running with {} sensor(s)",
            e.sensors().len()
        ));
        make_usb_thread_realtime();
        let open: Vec<String> = e.sensors().iter().map(engine::sensor_serial).collect();
        d.track_in.lock().unwrap().register(open.iter().cloned());
        for serial in modes::unposed_sensors(&opts, &open) {
            log(format!("sensor {serial} has no calibrated pose: it only observes (run `cv1touch calibrate`)"));
        }
        let link = RadioLink::default();
        // the radio thread holds a decoder pointing into the engine: the scope joins it first
        thread::scope(|scope| {
            let radio = new_pipeline(&d, &opts, &e).and_then(|pipe| {
                let Some(path) = hid::headset_interface(1) else {
                    log("no HID interface for the headset's radio: publishing OpenHMD poses");
                    return None;
                };
                let file = match hid::Device::open(&path) {
                    Ok(f) => f,
                    Err(err) => {
                        log(format!("cannot read the radio at {}: {err}; publishing OpenHMD poses", path.display()));
                        return None;
                    }
                };
                // SAFETY: the decoder goes to the scoped radio thread, joined before the engine closes
                let dec = unsafe { e.radio_decoder() }?;
                log(format!("publishing cv1touch tracker poses from the radio thread ({}; OpenHMD acquires only)", path.display()));
                let watch = LandmarkWatch::load();
                let (d, link) = (&*d, &link);
                Some(thread::Builder::new().name("cv1t-radio".into()).spawn_scoped(scope, move || radio_main(d, link, dec, file, pipe, watch)).expect("radio thread"))
            });
            {
                let _stop = StopRadio { d: &d, link: &link };
                engine_loop(&d, &mut e, &link, radio.is_some());
            }
            if let Some(Err(_)) = radio.map(|r| r.join()) {
                log("radio thread panicked");
            }
        });
        if let Some(f) = link.biases.lock().unwrap().take() {
            save_biases(&f);
        }
        drop(e);
        publish_disconnected(&d);
    }
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
            log(format!(
                "reopening the engine for {} mode",
                d.mode.lock().unwrap()
            ));
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
                log("headset I/O failing; reopening");
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
        if radio {
            if bodies.find(e, link) {
                d.wake_radio();
            }
        } else {
            // no tracker reads the frames: keep the queue from growing
            d.track_in.lock().unwrap().clear();
        }
        if now >= next_sensors {
            let sensors: Vec<SensorStatus> = e.sensors().iter().map(sensor_status).collect();
            publish_sensor_snapshot(d, &sensors);
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
                        log(format!("{}: no factory IMU position cached for its LED model; publishing OpenHMD poses for it", ["left", "right"][h]));
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
    mut dec: engine::RadioDecoder,
    mut radio: hid::Device,
    mut pipe: Pipeline,
    mut watch: Option<LandmarkWatch>,
) {
    let me = system::current_thread();
    if let Err(err) = set_rt_affinity(me) {
        log(format!("radio thread keeps the process's CPUs ({err})"));
    }
    match system::make_realtime(me, RADIO_RT_PRIORITY) {
        Ok(how) => log(format!("radio thread real-time ({how})")),
        Err(err) => log(format!("radio thread stays normal priority ({err})")),
    }
    d.radio_on.store(true, Ordering::Release);
    let _exit = RadioExit { d, link };
    let mut biases = BiasMemory::load();
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
                    log(format!("radio read failed ({err}); reopening the engine"));
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
                    Some(&mut pipe),
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
            log(format!(
                "{}: tracking with {} LEDs, IMU at ({:.1}, {:.1}, {:.1}) mm",
                ["left", "right"][b.hand],
                b.leds.len(),
                b.imu.x * 1e3,
                b.imu.y * 1e3,
                b.imu.z * 1e3
            ));
            biases.seed(&mut pipe, b.hand, &b.leds);
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
                w.frame(d, &info, &blobs);
            }
            pipe.frame(&info, &blobs);
        }
        // a hand whose controller went quiet still gets its state refreshed (it times out)
        let now = shm::now_ns();
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
                    Some(&mut pipe),
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
        if let Some(f) = biases.update(&pipe, now) {
            *link.biases.lock().unwrap() = Some(f);
        }
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
    // the driver extrapolates the turn orient * (ang_vel - gyro_bias)
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
            log(format!(
                "{}: dropped a wrong lock (accelerometer bias ran off)",
                ["left", "right"][h]
            ));
        }
        let latest = p.latest(h);
        apply_tracker(&mut state, latest.as_ref(), memory, |state, s| {
            apply_tracked(
                state,
                s,
                d.extra_latency_ns.load(Ordering::Relaxed),
                shaper,
                d.shaping_on.load(Ordering::Relaxed),
                d.prediction_percent.load(Ordering::Relaxed),
            )
        });
        let connected = state.flags & ffi::CV1T_FLAG_CONNECTED != 0;
        let tracking = latest.is_some_and(|s| s.output.tracking);
        if let Some(change) = memory.observe(connected, tracking, now) {
            log(format!("{}: {change}", ["left", "right"][h]));
        }
    }
    d.inputs.lock().unwrap().apply(h, &mut state);
    d.inject.apply(h, now, &mut state);
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
                log(hold_summary(["left", "right"][h], v));
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
    let engine_ready = engine_is_ready(d);
    let hands = [HAND_LEFT, HAND_RIGHT].map(|h| d.shm.hand(h).unwrap_or_default());
    let sensors = d.sensors.lock().unwrap().clone();
    let st = d.rec.stats();
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
    let sensors_json: Vec<serde_json::Value> = sensors
        .iter()
        .map(|s| {
            let serial = shm::serial_str(s);
            let shift = shifts
                .get(&serial)
                .map(|(x, y, n)| serde_json::json!({"dx": x, "dy": y, "landmarks": n}));
            serde_json::json!({"serial": serial, "placed": s.placed, "frames": s.frames,
            "fixes": s.pose_fixes, "camera_orient": s.camera_orient, "camera_pos": s.camera_pos,
            "landmark_shift": shift})
        })
        .collect();
    drop(shifts);
    let mut queue = serde_json::to_value(d.track_in.lock().unwrap().stats()).unwrap();
    queue["pending_frames_dropped"] =
        serde_json::json!(d.pending_frames_dropped.load(Ordering::Relaxed));
    format!(
        "ok {{\"generation\":{},\"shm\":{},\"tracker\":\"{:?}\",\"mode\":\"{}\",\"engine_ready\":{},\"input_calibration\":{},\"alignment\":{{\"registration\":{},\"render\":[{},{}],\"universe\":{}}},\"recording_scoped_stop\":true,\"recording\":{},\"recorded\":{},\"dropped\":{},\"write_errors\":{},\"hands\":[{}],\"sensors\":{},\"sensor_total\":{},\"sensor_shm_count\":{},\"sensor_shm_truncated\":{},\"camera_queue\":{}}}",
        d.shm.generation(),
        serde_json::to_string(d.shm.name()).unwrap(),
        d.tracker,
        d.mode.lock().unwrap(),
        engine_ready,
        d.inputs.lock().unwrap().status(),
        al.valid & ffi::CV1T_ALIGN_LIGHTHOUSE != 0,
        al.valid & ffi::CV1T_ALIGN_RENDER_LEFT != 0,
        al.valid & ffi::CV1T_ALIGN_RENDER_RIGHT != 0,
        al.universe_id,
        d.rec.active(),
        st.records,
        st.dropped,
        st.write_errors,
        hands_json.join(","),
        serde_json::to_string(&sensors_json).unwrap(),
        sensors.len(),
        sensors.len().min(shm::MAX_SENSORS),
        sensors.len() > shm::MAX_SENSORS,
        serde_json::to_string(&queue).unwrap(),
    )
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
            d.haptics.lock().unwrap()[hand] = Some(Haptic {
                duration_s,
                frequency_hz,
                amplitude,
            });
            "ok".into()
        }
        Ok(Command::RecordStart(path, frames)) => match d.rec.start(&path, shm::now_ns()) {
            Ok(()) => {
                d.record_frames.store(frames, Ordering::Release);
                d.session.lock().unwrap().reset();
                "ok".into()
            }
            Err(e) => format!("err {e}"),
        },
        Ok(Command::RecordStop) => {
            if !d.rec.has_session() {
                return "err not recording".into();
            }
            d.record_frames.store(false, Ordering::Release);
            let s = d.rec.stop();
            format!(
                "ok records={} dropped={} write_errors={} bytes={}",
                s.records, s.dropped, s.write_errors, s.bytes
            )
        }
        Ok(Command::RecordStopIf(path)) => match d.rec.stop_if(&path) {
            Some(s) => {
                d.record_frames.store(false, Ordering::Release);
                format!(
                    "ok stopped records={} dropped={} write_errors={} bytes={}",
                    s.records, s.dropped, s.write_errors, s.bytes
                )
            }
            None => "ok not-matching".into(),
        },
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
            switch_mode(d, modes::capture(&current_opts(d)), "capture")
        }
        Ok(Command::CalibrationProvisional) => {
            switch_mode(d, modes::provisional(&current_opts(d)), "provisional")
        }
        Ok(Command::CalibrationLoad(path)) => match modes::calibrated(&current_opts(d), &path) {
            Ok((opts, n)) => {
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
                    log(format!(
                        "test input injection armed for {} s",
                        INJECT_ARMED_NS / 1_000_000_000
                    ));
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
        Ok(Command::Prediction(percent)) => {
            let mut unsaved = String::new();
            if let Some(p) = percent {
                d.prediction_percent.store(p, Ordering::Release);
                let s = settings::Settings {
                    prediction_percent: p,
                    ..settings::Settings::default()
                };
                if let Err(e) = settings::save(&d.settings_path, &s) {
                    unsaved = format!(" (not saved: {e})");
                }
            }
            // the dial scales what the tracker publishes; without a calibration, or with
            // `--tracker openhmd`, OpenHMD's own poses are published instead
            let engine_poses =
                d.tracker == TrackerKind::OpenHmd || d.opts.lock().unwrap().fixed_poses.is_empty();
            let idle = if engine_poses {
                " (no effect while OpenHMD's own poses are published)"
            } else {
                ""
            };
            format!(
                "ok prediction: SteamVR extrapolates with {}% of the tracked velocities{idle}{unsaved}",
                d.prediction_percent.load(Ordering::Acquire)
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
                _ => "no recent driver report: connecting to SteamVR once a minute".into(),
            };
            format!(
                "ok universe watch {} ({src})",
                if d.universe_watch_on.load(Ordering::Acquire) {
                    "on"
                } else {
                    "off"
                }
            )
        }
        Ok(Command::Latency(v)) => {
            if let Some(ms) = v {
                d.extra_latency_ns
                    .store((ms * 1e6) as i64, Ordering::Release);
            }
            format!(
                "ok latency: published poses stamped {:.1} ms earlier than their IMU samples' arrival time ({:.1} ms measured + {:+.1} ms adjustment)",
                (pipeline::DEFAULT_EXPOSURE_OFFSET_NS as f64 + d.extra_latency_ns.load(Ordering::Acquire) as f64) * 1e-6,
                pipeline::DEFAULT_EXPOSURE_OFFSET_NS as f64 * 1e-6,
                d.extra_latency_ns.load(Ordering::Acquire) as f64 * 1e-6
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
            d.shm.publish_alignment(&alignment);
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

fn engine_is_ready(d: &Daemon) -> bool {
    let _opts = d.opts.lock().unwrap();
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

/// OpenHMD logs through C's stdout, which is block-buffered when it is not a terminal: under
/// systemd its lines reached the journal in 4 KB batches, stamped with the flush time rather than
/// when they happened. Line-buffered, each lands as it is written. Must run before anything prints.
/// (On Windows the engine's DLL carries its own C runtime and stdout: ohmd_cv1touch_unbuffer_log.)
#[cfg(unix)]
fn line_buffer_stdout() {
    extern "C" {
        static mut stdout: *mut libc::FILE;
    }
    unsafe { libc::setvbuf(stdout, std::ptr::null_mut(), libc::_IOLBF, 0) };
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
        log("no cv1t-usb thread: OpenHMD's USB event thread stays as it is");
        return;
    };
    if let Err(err) = set_rt_affinity(tid) {
        log(format!("cv1t-usb keeps the process's CPUs ({err})"));
    }
    match system::make_realtime(tid, USB_RT_PRIORITY) {
        Ok(how) => log(format!("cv1t-usb real-time ({how})")),
        Err(err) => log(format!("cv1t-usb stays normal priority ({err})")),
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

/// Serves the control endpoint: one thread accepts, one thread per client (at most MAX_CLIENTS)
/// reads its request lines, and one request is handled at a time; each client's requests are
/// answered in order.
fn serve_control(d: Arc<Daemon>, mut listener: ipc::Listener) -> ControlServer {
    let clients = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let one_at_a_time = Arc::new(Mutex::new(()));
    let server = ControlServer {
        one_at_a_time: Arc::clone(&one_at_a_time),
    };
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
                            log(format!("control endpoint: {why}"));
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
                let spawned = thread::Builder::new()
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
    server
}

/// One client's request lines, each answered with one reply line. A client that stops reading
/// for a second, or sends 4 KiB without a newline, is dropped.
fn serve_client(d: &Daemon, mut conn: ipc::Connection, one_at_a_time: &Mutex<()>) {
    let mut buf = Vec::new();
    let mut chunk = [0u8; 1024];
    loop {
        match conn.read(&mut chunk) {
            Ok(0) | Err(_) => return,
            Ok(n) => buf.extend_from_slice(&chunk[..n]),
        }
        while let Some(pos) = buf.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = buf.drain(..=pos).collect();
            let reply = {
                let _turn = one_at_a_time.lock().unwrap_or_else(|e| e.into_inner());
                handle(d, String::from_utf8_lossy(&line).trim())
            };
            let _ = conn.set_timeout(Some(Duration::from_secs(1)));
            if conn.write_all(format!("{reply}\n").as_bytes()).is_err() {
                return;
            }
            let _ = conn.set_timeout(None);
        }
        if buf.len() > 4096 {
            return; // no newline in 4 KiB: not a protocol client
        }
    }
}

const USAGE: &str = "usage: cv1touchd [--tracker cv1touch|openhmd] [--radio-wait-ms N] [--hmd-leds on|off] [--ignore-sensor SERIAL]... [--calibration PATH | --provisional] [--record PATH] [--shm-name NAME] [--runtime-dir DIR]\n       cv1touchd --supervise [OPTIONS]   run it as a child process, restarted after a failure, its output in a log file\n       cv1touchd --help | --version\nAffinity: inherit by default; CV1TOUCH_CPUS=2,4-6 explicitly selects allowed CPUs.";

fn usage() -> ! {
    eprintln!("{USAGE}");
    process::exit(2);
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
    let runtime = args
        .iter()
        .rposition(|a| a == "--runtime-dir")
        .and_then(|i| args.get(i + 1))
        .map_or_else(control::runtime_dir, PathBuf::from);
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
    let endpoint = runtime.join("control.sock");
    let mut stopping = supervise::Stopping {
        requested: Box::new(|| requests.wait(Duration::ZERO)),
        ask: Box::new(|| {
            control::request(&endpoint, "stop", Duration::from_secs(2))
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
    let affinity = affinity_setting().and_then(|spec| {
        if let Some(spec) = spec {
            let allowed = system::allowed_cpus(system::current_thread())
                .map_err(|e| format!("CV1TOUCH_CPUS: {e}"))?;
            affinity_cpus(&spec, &allowed)?;
        }
        Ok(())
    });
    if let Err(e) = affinity {
        eprintln!("cv1touchd: {e}");
        process::exit(2);
    }
    // First, before any thread exists: on Linux the recorder and engine threads inherit the
    // blocked SIGINT and SIGTERM, which must reach only the shutdown requests, never a thread
    // whose default action kills us.
    let shutdown = shutdown::Requests::take().unwrap_or_else(|e| {
        log(format!("cannot take over shutdown requests: {e}"));
        process::exit(1);
    });
    #[cfg(unix)]
    line_buffer_stdout();
    #[cfg(windows)]
    // SAFETY: takes nothing; nothing has printed through the engine's stdout yet.
    unsafe {
        ffi::ohmd_cv1touch_unbuffer_log()
    };
    if let Err(e) = system::opt_out_of_throttling() {
        log(format!("power throttling stays as Windows sets it ({e})"));
    }
    match system::lock_memory() {
        Ok(()) => log("memory locked: pages stay in RAM once touched"),
        Err(e) => log(format!(
            "memory not locked ({e}): pages can be swapped out under pressure"
        )),
    }
    let mut opts = EngineOptions {
        radio_wait_ms: 2,
        hmd_leds_off: DEFAULT_HMD_LEDS_OFF,
        ..Default::default()
    };
    let mut record: Option<PathBuf> = None;
    let mut calibration: Option<PathBuf> = None;
    let mut provisional = false;
    let mut shm_name = shm::DEFAULT_NAME.to_string();
    let mut runtime_dir = control::runtime_dir();
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
            "--ignore-sensor" => opts
                .ignore_sensors
                .push(args.next().unwrap_or_else(|| usage())),
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
            "--shm-name" => shm_name = args.next().unwrap_or_else(|| usage()),
            "--runtime-dir" => runtime_dir = PathBuf::from(args.next().unwrap_or_else(|| usage())),
            _ => usage(),
        }
    }

    let (opts, mode) = if provisional {
        (modes::provisional(&opts), "provisional".to_string())
    } else {
        let path = calibration
            .clone()
            .unwrap_or_else(calib::file::default_path);
        if calibration.is_some() || path.exists() {
            match modes::calibrated(&opts, &path) {
                Ok((o, n)) => {
                    log(format!(
                        "calibrated: {n} sensor poses from {}",
                        path.display()
                    ));
                    (o, "calibrated".to_string())
                }
                Err(e) => {
                    log(format!(
                        "{e} (fix or remove it, or start with --provisional)"
                    ));
                    process::exit(1);
                }
            }
        } else {
            log(format!(
                "no calibration at {}: provisional placement (run `cv1touch calibrate`)",
                path.display()
            ));
            (modes::provisional(&opts), "provisional".to_string())
        }
    };

    if let Err(e) = cv1touch::platform::fs::create_private_dir_all(&runtime_dir) {
        log(format!("{}: {e}", runtime_dir.display()));
        process::exit(1);
    }
    // Lock before touching shared memory or the socket: a second instance must not clobber the first.
    let lock_path = runtime_dir.join("daemon.lock");
    let _lock = control::lock_single_instance(&lock_path).unwrap_or_else(|e| {
        log(format!(
            "another cv1touchd holds {} ({e})",
            lock_path.display()
        ));
        process::exit(1);
    });
    let shm = ShmWriter::create(&shm_name, random_u64()).unwrap_or_else(|e| {
        log(format!("shared memory {shm_name}: {e}"));
        process::exit(1);
    });
    let (alignment, notes) = align::load_default();
    for n in notes {
        log(n);
    }
    shm.publish_alignment(&alignment);
    let rec = Recorder::new(RECORDER_RING_BYTES);
    if let Some(path) = &record {
        if let Err(e) = rec.start(path, shm::now_ns()) {
            log(e);
            process::exit(1);
        }
    }
    let sock_path = runtime_dir.join("control.sock");
    // a stale socket goes (the lock guarantees no live owner); private to this user
    let listener = ipc::Listener::bind(&sock_path).unwrap_or_else(|e| {
        log(format!("control socket {}: {e}", sock_path.display()));
        process::exit(1);
    });

    let mut inputs = InputRuntime::default();
    match input_calibration::load(&input_calibration::default_path()) {
        Ok(calibration) => inputs.calibration = calibration,
        Err(error) => {
            log(format!("{error}; using factory input defaults"));
            inputs.error = Some(error);
        }
    }
    let settings_path = settings::default_path();
    let settings = settings::load(&settings_path).unwrap_or_else(|e| {
        log(format!("settings: {e}; using the defaults"));
        settings::Settings::default()
    });
    if settings.prediction_percent != 100 {
        log(format!(
            "prediction: SteamVR extrapolates with {}% of the tracked velocities ({})",
            settings.prediction_percent,
            settings_path.display()
        ));
    }
    let d = Arc::new(Daemon {
        shm,
        rec,
        quit: AtomicBool::new(false),
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
        track_stats: Mutex::new([(Stats::default(), false); 2]),
        pending_frames_dropped: std::sync::atomic::AtomicU64::new(0),
        landmark_shifts: Mutex::new(Default::default()),
        universe_shift: Mutex::new(None),
        universe_recheck: AtomicBool::new(false),
        driver_stations: Mutex::new(None),
        universe_watch_on: AtomicBool::new(true),
        shaping_on: AtomicBool::new(true),
        prediction_percent: AtomicU32::new(settings.prediction_percent),
        settings_path,
        record_frames: AtomicBool::new(false),
        extra_latency_ns: std::sync::atomic::AtomicI64::new(0),
        battery: [
            std::sync::atomic::AtomicI32::new(-1),
            std::sync::atomic::AtomicI32::new(-1),
        ],
        radio_wake: hid::Waker::new().expect("radio waker"),
        radio_on: AtomicBool::new(false),
        inject: Injection::default(),
        inputs: Mutex::new(inputs),
    });
    let engine_thread = {
        let d = Arc::clone(&d);
        thread::Builder::new()
            .name("cv1t-engine".into())
            .spawn(move || engine_main(d))
            .expect("engine thread")
    };
    let universe_thread = {
        let d = Arc::clone(&d);
        thread::Builder::new()
            .name("cv1t-universe".into())
            .spawn(move || universe_watch(d))
            .expect("universe thread")
    };
    log(format!(
        "running: shm {shm_name}, control {}, {} mode, hmd leds {}, ignoring sensors {:?}",
        sock_path.display(),
        d.mode.lock().unwrap(),
        if d.opts.lock().unwrap().hmd_leds_off {
            "off"
        } else {
            "on"
        },
        d.opts.lock().unwrap().ignore_sensors
    ));

    let control = serve_control(Arc::clone(&d), listener);
    loop {
        // the daemon's liveness for the driver: opening the engine (the radio wait) blocks the
        // engine thread for seconds, which must not read as a dead daemon; stale poses are the
        // driver's own concern (sample age)
        d.shm.heartbeat(shm::now_ns());
        if let Some(why) = shutdown.wait(Duration::from_millis(200)) {
            log(format!("{why}: shutting down"));
            break;
        }
        if d.stop_requested.load(Ordering::Acquire) {
            log("asked to stop: shutting down");
            break;
        }
    }

    // nothing a client asks for runs while the daemon stops
    let _no_more_requests = control.stop();
    d.quit.store(true, Ordering::Release);
    let _ = engine_thread.join();
    let _ = universe_thread.join();
    d.rec.stop();
    let _ = fs::remove_file(&sock_path);
    // the control threads still hold `d`, so retire the segment here rather than on its drop
    d.shm.retire();
    log("stopped");
}

#[cfg(test)]
mod tests {
    use super::*;

    fn status_daemon() -> Daemon {
        static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let name = format!(
            "/cv1t-dynamic-status-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        );
        Daemon {
            shm: ShmWriter::create(&name, 1).unwrap(),
            rec: Recorder::new(4096),
            quit: AtomicBool::new(false),
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
            track_stats: Mutex::new([(Stats::default(), false); 2]),
            pending_frames_dropped: std::sync::atomic::AtomicU64::new(0),
            landmark_shifts: Mutex::new(Default::default()),
            universe_shift: Mutex::new(None),
            universe_recheck: AtomicBool::new(false),
            driver_stations: Mutex::new(None),
            universe_watch_on: AtomicBool::new(true),
            shaping_on: AtomicBool::new(true),
            prediction_percent: AtomicU32::new(100),
            settings_path: std::env::temp_dir()
                .join(format!(
                    "cv1t-daemon-settings-{}",
                    name.trim_start_matches('/')
                ))
                .join("settings.json"),
            record_frames: AtomicBool::new(false),
            extra_latency_ns: std::sync::atomic::AtomicI64::new(0),
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
        // with OpenHMD's poses published, the reply says the dial does nothing: without a
        // calibration, and with `--tracker openhmd`
        let note = "(no effect while OpenHMD's own poses are published)";
        d.opts.lock().unwrap().fixed_poses.clear();
        assert!(handle(&d, "prediction 50").ends_with(note));
        let mut d = d;
        d.opts.lock().unwrap().fixed_poses.push(engine::SensorPose {
            serial: "WMTD000000001".into(),
            orient: [0.0, 0.0, 0.0, 1.0],
            pos: [0.0; 3],
        });
        d.tracker = TrackerKind::OpenHmd;
        assert!(handle(&d, "prediction 50").ends_with(note));
        let _ = std::fs::remove_dir_all(d.settings_path.parent().unwrap());
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

    /// `prediction` scales the velocities SteamVR extrapolates with, shaped or not, and nothing
    /// else: at 0 the driver hands SteamVR no motion at all, at 50 half the motion it does at 100,
    /// and the pose and its stamps are the same at every setting.
    #[test]
    fn prediction_scales_only_the_published_velocities() {
        use nalgebra::Vector3;

        let s = tracked_sample();
        let (q, rate) = (s.output.world_from_model.rotation, s.output.ang_vel_world);
        let v3 = |a: [f32; 3]| Vector3::new(a[0] as f64, a[1] as f64, a[2] as f64);
        for shaping in [false, true] {
            let publish = |percent| {
                let mut st = ControllerState::default();
                let mut shaper = VelocityShaper::new(ShapeParams::default());
                apply_tracked(&mut st, &s, 0, &mut shaper, shaping, percent);
                // what the driver hands SteamVR: the world rate and the linear velocity
                let world_rate = q * (v3(st.ang_vel) - v3(st.gyro_bias));
                (st, world_rate, v3(st.lin_vel))
            };
            let (full, full_rate, full_vel) = publish(100);
            assert!((full_rate - rate).norm() < 1e-5 && full_vel.norm() > 0.5);
            let (none, none_rate, none_vel) = publish(0);
            assert!(none_rate.norm() < 1e-6 && none_vel.norm() < 1e-6);
            let (half, half_rate, half_vel) = publish(50);
            assert!((half_rate - full_rate / 2.0).norm() < 1e-5);
            assert!((half_vel - full_vel / 2.0).norm() < 1e-6);
            for st in [none, half] {
                assert_eq!(
                    (
                        st.orient,
                        st.pos,
                        st.gyro_bias,
                        st.sample_host_ns,
                        st.last_position_fix_host_ns,
                        st.flags
                    ),
                    (
                        full.orient,
                        full.pos,
                        full.gyro_bias,
                        full.sample_host_ns,
                        full.last_position_fix_host_ns,
                        full.flags
                    )
                );
            }
        }
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
            apply_tracked(st, s, 0, &mut shaper, false, 100)
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
            apply_tracked(st, s, 0, &mut shaper, false, 100)
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

    /// The control server answers each request line, whether split across reads or several in
    /// one, serves at most MAX_CLIENTS clients, drops one that sends 4 KiB without a newline,
    /// and answers nothing once stopped.
    #[test]
    fn the_control_server_answers_each_line_until_it_stops() {
        use std::io::{ErrorKind, Read, Write};
        let dir = std::env::temp_dir().join(format!("cv1t-control-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        let endpoint = dir.join("control.sock");
        let control = serve_control(
            Arc::new(status_daemon()),
            ipc::Listener::bind(&endpoint).unwrap(),
        );
        let connect = || {
            let mut conn = ipc::Connection::connect(&endpoint).unwrap();
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

        let mut idle: Vec<_> = (1..MAX_CLIENTS).map(|_| connect()).collect();
        let mut refused = connect();
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

        let mut flood = connect();
        let _ = flood.write_all(&[b'x'; 5000]);
        assert_eq!(reply(&mut flood), Reply::Closed, "4 KiB without a newline");

        let _stopped = control.stop();
        client
            .set_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        client.write_all(b"status\n").unwrap();
        assert_eq!(reply(&mut client), Reply::Silent, "no answer once stopped");
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn scoped_record_stop_reports_outcome_and_preserves_unrelated_session() {
        let d = status_daemon();
        let path = std::env::temp_dir().join(format!("cv1touch-scoped-stop-{}.rec", shm::now_ns()));
        let other = path.with_extension("other.rec");
        let status: serde_json::Value =
            serde_json::from_str(status_json(&d).strip_prefix("ok ").unwrap()).unwrap();
        assert_eq!(status["recording_scoped_stop"], true);
        let stop = control::record_stop_if_line(&path).unwrap();
        assert_eq!(handle(&d, &stop), "ok not-matching");
        assert_eq!(
            handle(&d, &control::record_start_line(&path, true).unwrap()),
            "ok"
        );
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
        assert!(engine_is_ready(&d));
        assert_eq!(
            switch_mode(&d, Default::default(), "calibrated"),
            "ok calibrated"
        );
        assert_eq!(*d.mode.lock().unwrap(), "calibrated");
        assert!(!engine_is_ready(&d));
        // An older open can finish after a newer request without satisfying it.
        d.engine_applied.store(first, Ordering::Release);
        assert!(!engine_is_ready(&d));
        let (_, latest) = requested_engine_options(&d);
        assert_ne!(latest, first);
        d.engine_applied.store(latest, Ordering::Release);
        assert!(engine_is_ready(&d));
        publish_disconnected(&d);
        assert!(!engine_is_ready(&d));
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
        publish_sensor_snapshot(&d, &sensors);
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
