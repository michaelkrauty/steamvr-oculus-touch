// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Everything between the engine's raw inputs and the trackers' outputs: clock fits, exposure
//! timing, frame queueing and the per-controller trackers. The daemon feeds it live; `replay`
//! feeds it from a recording, in the order the daemon wrote it, so offline results carry over.
//!
//! Exposure time: the headset's hardware exposure stamp, mapped to host time through the headset
//! clock fit, then into each controller's clock through its own fit, plus a constant offset (the
//! difference of the two links' minimum transport delays and the exposure midpoint).

use std::collections::BTreeMap;

use nalgebra::{Isometry3, Vector2, Vector3};

use super::clock::{LinearClock, Unwrap32};
use super::tracker::{
    CameraAgreement, EnginePose, ImuSample, Outcome, Output, Params, Stats, Tracker,
};
use super::vision::{Blob, Body, Camera};
use crate::calib::camera::CameraModel;
use crate::calib::capture::{Led, MATCH_GOOD, MATCH_STRONG};
use crate::calib::{iso_from_xyzw, DEVICE_LEFT, DEVICE_RIGHT};
use crate::engine::{cstr_field, FrameInfo};
use crate::shm::{HAND_LEFT, HAND_RIGHT};

/// Report id of the headset's IMU report, which carries its clock and the exposure stamps.
pub const HMD_SENSORS_REPORT: u8 = 11;

/// How much later than the headset's the controllers' IMU samples are stamped: the radio link's
/// fastest delivery plus the IMU's own filtering, beyond the headset's report delay. Estimated from
/// development image alignment: over frames turning faster than 3 rad/s, the
/// predicted LEDs land nearest the blobs at +3.9 to 4.0 ms. Exposures are mapped this much later
/// into a controller's clock, and published poses are stamped this much earlier than their IMU
/// sample's arrival-based time, so that SteamVR predicts from when the controller was there.
pub const DEFAULT_EXPOSURE_OFFSET_NS: i64 = 4_000_000;

/// Wait this long past an exposure before probing it, so frames of it (and of the next exposure)
/// from the tracked sensors are in.
pub const PROBE_DELAY_NS: u64 = 80_000_000;
/// Probed frames waiting for the track to pass their exposure, per hand at most: about a second
/// of one sensor; a controller asleep drains none.
pub const PROBING_CAP: usize = 64;

/// The headset clock (microseconds) of one of its IMU reports.
pub fn hmd_report_timestamp(d: &[u8]) -> Option<u32> {
    (d.len() == 64 && d[0] == HMD_SENSORS_REPORT)
        .then(|| u32::from_le_bytes([d[8], d[9], d[10], d[11]]))
}

pub fn device_hand(device_id: i32) -> Option<usize> {
    match device_id {
        DEVICE_RIGHT => Some(HAND_RIGHT),
        DEVICE_LEFT => Some(HAND_LEFT),
        _ => None,
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    pub params: Params,
    /// Added to every exposure time mapped into a controller's clock.
    pub exposure_offset_ns: i64,
    /// Keep every frame's outcome in `Pipeline::events` and the trackers' miss diagnostics.
    pub trace: bool,
    /// These sensors' frames are not tracked with: each is kept in `Pipeline::probes` with the
    /// controller's pose at its exposure, once the other sensors' frames up to then are applied.
    /// A sensor being placed, or one that moved and is being re-placed (`Pipeline::probe`).
    pub probe_sensors: Vec<String>,
    /// Replay only: also keep each output's pose this long before, as the filter holds it once
    /// the frames that arrive in the meantime are in (`Replay::delayed`).
    pub delayed_ns: Option<u64>,
    /// Per hand, biases (gyro, accel) to start each new tracker from, as the daemon does with the
    /// remembered ones (see `track::biases`).
    pub seed_biases: [Option<(Vector3<f64>, Vector3<f64>)>; 2],
}

impl Default for Options {
    fn default() -> Self {
        Options {
            params: Params::default(),
            exposure_offset_ns: DEFAULT_EXPOSURE_OFFSET_NS,
            trace: false,
            probe_sensors: Vec::new(),
            delayed_ns: None,
            seed_biases: [None, None],
        }
    }
}

/// One frame's outcome for one controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameEvent {
    pub hand: usize,
    /// Stable sensor ID assigned when its serial is first registered.
    pub camera: usize,
    /// Exposure, controller clock and host clock.
    pub t_e: u64,
    pub exposure_host_ns: u64,
    /// The frame's start on the host clock (its FRAME_OBS / RAW_FRAME stamp).
    pub frame_start_host_ns: u64,
    pub outcome: Outcome,
    /// The engine's pose on this frame, and the tracker's at the exposure before it.
    pub engine: Option<EnginePose>,
    pub tracked: Option<Isometry3<f64>>,
}

/// A probed sensor's frame with the tracked pose of one controller at its exposure.
#[derive(Debug, Clone, PartialEq)]
pub struct Probe {
    /// The sensor's registration id (`Pipeline::serial_of`).
    pub camera: usize,
    pub hand: usize,
    /// The engine device id of that hand's controller, which its blob labels carry.
    pub device_id: i32,
    pub world_from_model: Isometry3<f64>,
    /// Position standard deviation of that pose, metres (the largest axis).
    pub pos_sd: f64,
    pub blobs: Vec<Vector2<f64>>,
    /// The engine's label of each blob, (device id << 8) | LED index, or -1.
    pub labels: Vec<i32>,
}

/// A tracker output stamped with its IMU sample's host time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stamped {
    pub host_ns: u64,
    pub output: Output,
    /// Body-frame gyro sample and the filter's gyro bias, rad/s.
    pub gyro: Vector3<f64>,
    pub gyro_bias: Vector3<f64>,
    /// Host time of the newest optical fix.
    pub last_fix_host_ns: u64,
}

struct Pending {
    t_e: u64,
    exposure_host_ns: u64,
    frame_start_host_ns: u64,
    camera: usize,
    cam: Camera,
    blobs: Vec<Blob>,
    engine: Option<EnginePose>,
}

#[derive(Clone, Copy)]
struct RegisteredCamera {
    id: usize,
    camera: Camera,
}

pub struct Pipeline {
    pub opts: Options,
    hmd_clock: LinearClock,
    hmd_stamps: Unwrap32,
    clocks: [LinearClock; 2],
    trackers: [Option<Tracker>; 2],
    cams: BTreeMap<String, RegisteredCamera>,
    /// Registration ids of the sensors whose frames are probed, not tracked with.
    probed: std::collections::BTreeSet<usize>,
    /// Registration ids of the sensors whose frames are dropped (given up on).
    ignored: std::collections::BTreeSet<usize>,
    pending: [Vec<Pending>; 2],
    /// Per hand: a probed sensor's frames (exposure, camera, blobs) waiting for the track to
    /// pass their exposure.
    probing: [Vec<(u64, usize, Vec<Blob>)>; 2],
    latest: [Option<Stamped>; 2],
    /// Frames before the headset clock had a fit, from sensors without a pose, or from sensors
    /// given up on (`ignore`).
    pub frames_skipped: u64,
    /// Pending frames evicted by the per-camera two-frame limit, summed across hands.
    pub frames_dropped: u64,
    pub events: Vec<FrameEvent>,
    pub probes: Vec<Probe>,
}

impl Pipeline {
    pub fn new(opts: Options) -> Pipeline {
        Pipeline {
            opts,
            hmd_clock: LinearClock::default(),
            hmd_stamps: Unwrap32::default(),
            clocks: [LinearClock::default(), LinearClock::default()],
            trackers: [None, None],
            cams: BTreeMap::new(),
            probed: Default::default(),
            ignored: Default::default(),
            pending: [Vec::new(), Vec::new()],
            probing: [Vec::new(), Vec::new()],
            latest: [None, None],
            frames_skipped: 0,
            frames_dropped: 0,
            events: Vec::new(),
            probes: Vec::new(),
        }
    }

    /// Registers a sensor and its calibrated pose (camera -> world, OpenCV axes).
    /// IDs remain stable across pose updates and later metadata, including replay registration.
    pub fn sensor(&mut self, serial: &str, model: CameraModel, world_from_cam: Isometry3<f64>) {
        let camera = Camera {
            model,
            world_from_cam,
        };
        if let Some(registered) = self.cams.get_mut(serial) {
            registered.camera = camera;
            return;
        }
        let id = self.cams.len();
        self.cams
            .insert(serial.to_string(), RegisteredCamera { id, camera });
        if self
            .opts
            .probe_sensors
            .iter()
            .any(|s| crate::status::same_sensor(s, serial))
        {
            self.probed.insert(id);
        }
        for tracker in self.trackers.iter_mut().flatten() {
            tracker.register_camera();
        }
    }

    /// Probes the sensor `serial` from now on (`on`), or tracks with it again: false when no
    /// such sensor is registered. Frames of it already queued for tracking are dropped.
    pub fn probe(&mut self, serial: &str, on: bool) -> bool {
        let Some(registered) = self.cams.get(serial) else {
            return false;
        };
        let id = registered.id;
        if on {
            self.probed.insert(id);
            for pending in &mut self.pending {
                pending.retain(|p| p.camera != id);
            }
        } else {
            self.probed.remove(&id);
            for probing in &mut self.probing {
                probing.retain(|p| p.1 != id);
            }
        }
        true
    }

    /// Drops the sensor `serial`'s frames from now on (`on`), neither tracked with nor probed, or
    /// takes them again: false when no such sensor is registered.
    pub fn ignore(&mut self, serial: &str, on: bool) -> bool {
        let Some(id) = self.cams.get(serial).map(|c| c.id) else {
            return false;
        };
        if on {
            self.ignored.insert(id);
            for probing in &mut self.probing {
                probing.retain(|p| p.1 != id);
            }
            for pending in &mut self.pending {
                pending.retain(|p| p.camera != id);
            }
        } else {
            self.ignored.remove(&id);
        }
        true
    }

    /// Forgets how the sensor `serial` agreed with the others, as when it was placed again.
    pub fn reset_agreement(&mut self, serial: &str) {
        let Some(id) = self.cams.get(serial).map(|c| c.id) else {
            return;
        };
        for tr in self.trackers.iter_mut().flatten() {
            if let Some(a) = tr.agreement.get_mut(id) {
                *a = CameraAgreement::default();
            }
        }
    }

    /// Whether the sensor `serial`'s frames are probed rather than tracked with.
    pub fn is_probed(&self, serial: &str) -> bool {
        self.cams
            .get(serial)
            .is_some_and(|c| self.probed.contains(&c.id))
    }

    /// The registration id of the sensor `serial`.
    pub fn camera_id(&self, serial: &str) -> Option<usize> {
        self.cams.get(serial).map(|c| c.id)
    }

    /// The serial of the sensor registered with id `camera`.
    pub fn serial_of(&self, camera: usize) -> Option<&str> {
        self.cams
            .iter()
            .find(|(_, c)| c.id == camera)
            .map(|(serial, _)| serial.as_str())
    }

    /// The sensor `serial`'s camera model and the pose it is tracked with.
    pub fn camera(&self, serial: &str) -> Option<&Camera> {
        self.cams.get(serial).map(|c| &c.camera)
    }

    pub fn has_body(&self, hand: usize) -> bool {
        self.trackers[hand].is_some()
    }

    /// A controller's LED model and factory IMU position; the first one per hand is kept.
    pub fn body(&mut self, device_id: i32, leds: Vec<Led>, imu_pos: Vector3<f64>) {
        let Some(hand) = device_hand(device_id) else {
            return;
        };
        if self.trackers[hand].is_none() {
            let mut tr = Tracker::new(
                device_id,
                Body { leds, imu_pos },
                self.opts.params,
                self.cams.len(),
            );
            tr.debug = self.opts.trace;
            if let Some((g, a)) = self.opts.seed_biases[hand] {
                tr.seed_biases(
                    g,
                    a,
                    super::biases::SEED_SD_GYRO,
                    super::biases::SEED_SD_ACCEL,
                );
            }
            self.trackers[hand] = Some(tr);
        }
    }

    /// Starts a hand's biases from an earlier run's (see `Tracker::seed_biases`, `track::biases`).
    pub fn seed_biases(&mut self, hand: usize, gyro: Vector3<f64>, accel: Vector3<f64>) {
        if let Some(tr) = self.trackers[hand].as_mut() {
            tr.seed_biases(
                gyro,
                accel,
                super::biases::SEED_SD_GYRO,
                super::biases::SEED_SD_ACCEL,
            );
        }
    }

    pub fn tracker(&self, hand: usize) -> Option<&Tracker> {
        self.trackers[hand].as_ref()
    }

    /// Each sensor (by serial) with its agreement with the others, both controllers' samples
    /// combined: the residual and bias weighted by samples, the counts summed.
    pub fn camera_agreement(&self) -> Vec<(String, CameraAgreement)> {
        self.cams
            .iter()
            .map(|(serial, camera)| {
                let mut all = CameraAgreement::default();
                for tr in self.trackers.iter().flatten() {
                    if let Some(a) = tr.agreement.get(camera.id) {
                        all.median_px += a.median_px * a.samples as f64;
                        all.bias_px += a.bias_px * a.samples as f64;
                        all.samples += a.samples;
                        all.checks += a.checks;
                        all.misses += a.misses;
                    }
                }
                let k = if all.samples > 0 {
                    1.0 / all.samples as f64
                } else {
                    f64::NAN
                };
                all.median_px *= k;
                all.bias_px *= k;
                (serial.clone(), all)
            })
            .collect()
    }

    pub fn stats(&self, hand: usize) -> Stats {
        self.trackers[hand]
            .as_ref()
            .map_or_else(Stats::default, |t| t.stats)
    }

    /// A raw report from the headset (source 0): its IMU reports carry the headset clock.
    pub fn hmd_report(&mut self, host_ns: u64, data: &[u8]) {
        if let Some(ts) = hmd_report_timestamp(data) {
            self.hmd_clock_sample(host_ns, ts);
        }
    }

    /// The headset clock (microseconds) of a report received at `host_ns`.
    pub fn hmd_clock_sample(&mut self, host_ns: u64, us: u32) {
        self.hmd_clock.observe(self.hmd_stamps.extend(us), host_ns);
    }

    /// One factory-calibrated IMU sample of a controller (repeats are ignored). Returns the
    /// tracker's output after it when the position is tracked.
    pub fn imu(
        &mut self,
        hand: usize,
        device_ns: u64,
        received_host_ns: u64,
        gyro: Vector3<f64>,
        accel: Vector3<f64>,
    ) -> Option<Stamped> {
        let tr = self.trackers[hand].as_mut()?;
        if device_ns == 0 || tr.newest_imu().is_some_and(|n| device_ns <= n) {
            return None;
        }
        self.clocks[hand].observe(device_ns, received_host_ns);
        let gaps = tr.stats.imu_gaps;
        tr.imu(ImuSample {
            t: device_ns,
            gyro,
            accel,
        });
        if tr.stats.imu_gaps != gaps {
            // the controller slept or dropped out: frames queued for its old clock are stale
            self.pending[hand].clear();
            self.probing[hand].clear();
        }
        self.drain(hand);
        let tr = self.trackers[hand].as_mut()?;
        while self.probing[hand]
            .first()
            .is_some_and(|p| p.0 + PROBE_DELAY_NS <= device_ns)
        {
            let (t_e, camera, blobs) = self.probing[hand].remove(0);
            if let Some(f) = tr.filter_at(t_e) {
                let pos_sd = (0..3)
                    .map(|i| f.cov[(3 + i, 3 + i)])
                    .fold(0.0, f64::max)
                    .sqrt();
                self.probes.push(Probe {
                    camera,
                    hand,
                    device_id: tr.device_id,
                    world_from_model: tr.body.world_from_model(&f.x),
                    pos_sd,
                    blobs: blobs.iter().map(|b| b.xy).collect(),
                    labels: blobs.iter().map(|b| b.led_id).collect(),
                });
            }
        }
        let Some(out) = tr.output() else {
            // no state yet, dropped, or a track no second sensor has confirmed with no position
            // held from before: nothing of ours to publish
            self.latest[hand] = None;
            return None;
        };
        // the clock fit stamps a sample at its fastest arrival; the motion it measured happened the
        // radio link's extra delay (the exposure offset) earlier
        let physical = |host: u64| {
            (host as i64)
                .saturating_sub(self.opts.exposure_offset_ns)
                .max(0) as u64
        };
        let host_ns = physical(self.clocks[hand].to_host(device_ns)?);
        let bg = out.gyro_bias;
        // a fix counts only while the track is published (see `Output::last_fix`)
        let last_fix_host_ns = match out.last_fix {
            0 => 0,
            t => self.clocks[hand].to_host(t).map_or(0, physical),
        };
        let s = Stamped {
            host_ns,
            output: out,
            gyro,
            gyro_bias: bg,
            last_fix_host_ns,
        };
        self.latest[hand] = Some(s);
        out.tracking.then_some(s)
    }

    /// The newest output for a hand, tracking or not.
    pub fn latest(&self, hand: usize) -> Option<Stamped> {
        self.latest[hand]
    }

    /// One sensor frame from the engine.
    pub fn frame(&mut self, info: &FrameInfo, blobs: &[crate::engine::Blob]) {
        let serial = cstr_field(&info.serial);
        let registered = self.cams.get(&serial).copied();
        let exposure = self
            .hmd_stamps
            .relative(info.exposure_hmd_ts)
            .and_then(|e| self.hmd_clock.to_host(e));
        let (Some(registered), Some(exposure), true) =
            (registered, exposure, info.exposure_valid != 0)
        else {
            self.frames_skipped += 1;
            return;
        };
        if self.ignored.contains(&registered.id) {
            self.frames_skipped += 1;
            return;
        }
        let camera = registered.id;
        let cam = registered.camera;
        let list: Vec<Blob> = blobs
            .iter()
            .map(|b| Blob {
                xy: Vector2::new(b.x as f64, b.y as f64),
                led_id: b.led_id,
            })
            .collect();
        let probe = self.probed.contains(&camera);
        for hand in 0..2 {
            let Some(tr) = self.trackers[hand].as_mut() else {
                continue;
            };
            let Some(t) = self.clocks[hand].to_device(exposure) else {
                continue;
            };
            let t_e = (t as i64 + self.opts.exposure_offset_ns).max(0) as u64;
            if probe {
                if self.probing[hand].len() >= PROBING_CAP {
                    self.probing[hand].remove(0);
                }
                self.probing[hand].push((t_e, camera, list.clone()));
                continue;
            }
            let engine = info.devices[..(info.n_devices as usize).min(info.devices.len())]
                .iter()
                .find(|d| {
                    d.device_id == tr.device_id
                        && d.pose_valid != 0
                        && d.match_flags & (MATCH_GOOD | MATCH_STRONG) != 0
                })
                .map(|d| EnginePose {
                    world_from_model: cam.world_from_cam * iso_from_xyzw(d.cam_orient, d.cam_pos),
                    strong: d.match_flags & MATCH_STRONG != 0,
                });
            // Bound each registered camera independently: a fast producer cannot evict another
            // camera's sole pending frame. Drop its oldest exposure, including a late incoming one.
            let at = self.pending[hand].partition_point(|p| p.t_e <= t_e);
            self.pending[hand].insert(
                at,
                Pending {
                    t_e,
                    exposure_host_ns: exposure,
                    frame_start_host_ns: info.frame_start_host_ns,
                    camera,
                    cam,
                    blobs: list.clone(),
                    engine,
                },
            );
            if self.pending[hand]
                .iter()
                .filter(|p| p.camera == camera)
                .count()
                > 2
            {
                let oldest = self.pending[hand]
                    .iter()
                    .position(|p| p.camera == camera)
                    .unwrap();
                self.pending[hand].remove(oldest);
                self.frames_dropped += 1;
            }
            self.drain(hand);
        }
    }

    /// Applies a hand's pending frames whose exposure its IMU has reached, each with the other
    /// controller's LEDs as tracked at the same exposure (in the other's clock).
    fn drain(&mut self, hand: usize) {
        let Pipeline {
            trackers,
            pending,
            clocks,
            events,
            opts,
            ..
        } = self;
        let (left, right) = trackers.split_at_mut(1);
        let (tr, other) = if hand == 0 {
            (left[0].as_mut(), right[0].as_ref())
        } else {
            (right[0].as_mut(), left[0].as_ref())
        };
        let Some(tr) = tr else { return };
        let pending = &mut pending[hand];
        let other_clock = &clocks[1 - hand];
        let min_facing = opts.params.fit.assoc.min_facing;
        while let Some(p) = pending.first() {
            let foreign = other
                .filter(|o| o.tracking())
                .and_then(|o| {
                    let t = (other_clock.to_device(p.exposure_host_ns)? as i64
                        + opts.exposure_offset_ns)
                        .max(0) as u64;
                    let f = o.filter_at(t)?;
                    Some(super::vision::predicted_pixels(
                        &p.cam, &o.body, &f.x, min_facing,
                    ))
                })
                .unwrap_or_default();
            let tracked = if opts.trace {
                tr.filter_at(p.t_e).map(|f| tr.body.world_from_model(&f.x))
            } else {
                None
            };
            let outcome = tr.frame(p.t_e, p.camera, &p.cam, &p.blobs, p.engine, &foreign);
            if outcome == Outcome::Early {
                break;
            }
            if opts.trace {
                events.push(FrameEvent {
                    hand,
                    camera: p.camera,
                    t_e: p.t_e,
                    exposure_host_ns: p.exposure_host_ns,
                    frame_start_host_ns: p.frame_start_host_ns,
                    outcome,
                    engine: p.engine,
                    tracked,
                });
            }
            pending.remove(0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::{self, Motion, Rng};
    use crate::engine::{Blob as EngineBlob, FrameInfo};
    use nalgebra::Point3;
    use std::os::raw::c_char;

    const H0: u64 = 5_000_000_000_000; // host clock at t = 0
    const C0: u64 = 4_000_000_000_000; // controller clock = host - C0
    const HMD_DELAY: u64 = 300_000; // fastest headset report
    const IMU_DELAY: u64 = 1_000_000; // fastest radio report

    fn hmd_us(host: u64) -> u32 {
        ((host - H0) / 1000 + 77_000) as u32
    }

    fn serial(i: usize) -> [c_char; 32] {
        let mut s = [0 as c_char; 32];
        for (d, b) in s.iter_mut().zip(format!("SENSOR{i}").bytes()) {
            *d = b as c_char;
        }
        s
    }

    enum Ev {
        Hmd(u64, u32),
        Imu(u64, u64, Vector3<f64>, Vector3<f64>),
        Frame(Box<FrameInfo>, Vec<EngineBlob>),
    }

    /// A Touch swept for `seconds`, its IMU silent in `gap`: every output while tracking (host
    /// time, position error), and whether a pose from before the gap was still published after it.
    fn run(seconds: f64, gap: Option<(f64, f64)>) -> (Vec<(f64, f64)>, bool) {
        run_with(seconds, gap, IMU_DELAY)
    }

    /// As `run`, with the radio link's fastest delay `imu_delay`. Errors are against the truth at
    /// the published stamp less the headset link's delay: stamps are meant to be the motion's own
    /// time, as near as the headset's clock can tell it.
    fn run_with(seconds: f64, gap: Option<(f64, f64)>, imu_delay: u64) -> (Vec<(f64, f64)>, bool) {
        let (errors, stale, _, _) =
            run_with_sensors(seconds, gap, imu_delay, 3, None, MATCH_GOOD | MATCH_STRONG);
        (errors, stale)
    }

    type Simulation = (
        Vec<(f64, f64)>,
        bool,
        Pipeline,
        crate::calib::file::CalibrationFile,
    );

    fn run_with_sensors(
        seconds: f64,
        gap: Option<(f64, f64)>,
        imu_delay: u64,
        camera_count: usize,
        recording: Option<&crate::recorder::Recorder>,
        match_flags: u32,
    ) -> Simulation {
        use crate::recorder::{self, as_bytes, slice_bytes};
        let mut rng = Rng::new(5);
        let motion = Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0));
        let leds = synth::touch_leds();
        let imu_pos = Vector3::new(0.015, -0.012, 0.008);
        let target = Vector3::new(0.0, 1.0, 0.0);
        let cams: Vec<Isometry3<f64>> = [
            (Vector3::new(-2.0, 2.3, 1.8), true),
            (Vector3::new(2.0, 2.3, 1.8), false),
            (Vector3::new(0.3, 2.4, -2.2), true),
            (Vector3::new(-1.5, 2.2, -1.7), false),
            (Vector3::new(1.5, 2.2, -1.7), true),
        ]
        .iter()
        .take(camera_count)
        .map(|&(p, flip)| synth::look_at(p, target, flip))
        .collect();
        let model = synth::camera_model();
        // the clocks map with each link's fastest delay, so the exposure lands that much off:
        // the offset makes it exact here
        let mut p = Pipeline::new(Options {
            exposure_offset_ns: imu_delay as i64 - HMD_DELAY as i64,
            trace: recording.is_some(),
            ..Options::default()
        });
        let mut calibration = crate::calib::file::CalibrationFile::example();
        calibration.sensors.clear();
        for (i, w) in cams.iter().enumerate() {
            p.sensor(&format!("SENSOR{i}"), model, *w);
            let (orient_xyzw, pos) = crate::calib::iso_to_xyzw(w);
            calibration.sensors.push(crate::calib::file::SensorEntry {
                serial: format!("SENSOR{i}"),
                orient_xyzw,
                pos,
                rms_px: 0.0,
                observations: 1,
            });
            if let Some(rec) = recording {
                let sensor = recorder::SensorInfo {
                    serial: serial(i),
                    width: model.width as i32,
                    height: model.height as i32,
                    fx: model.fx as f32,
                    fy: model.fy as f32,
                    cx: model.cx as f32,
                    cy: model.cy as f32,
                    k: model.k.map(|v| v as f32),
                    ..Default::default()
                };
                rec.write(recorder::SENSOR_INFO, H0, &[as_bytes(&sensor)]);
            }
        }
        if let Some(rec) = recording {
            let model: Vec<_> = leds
                .iter()
                .map(|l| recorder::LedEntry {
                    pos: l.pos.map(|v| v as f32).into(),
                    dir: l.dir.map(|v| v as f32).into(),
                })
                .collect();
            rec.write(
                recorder::LED_MODEL,
                H0,
                &[
                    &recorder::led_model_prefix(DEVICE_RIGHT, model.len()),
                    slice_bytes(&model),
                ],
            );
        }
        p.body(crate::calib::DEVICE_RIGHT, leds.clone(), imu_pos);
        let mut evs: Vec<(u64, Ev)> = Vec::new();
        let end = H0 + (seconds * 1e9) as u64;
        let mut h = H0;
        while h < end {
            let mut d = [0u8; 64];
            d[0] = HMD_SENSORS_REPORT;
            d[8..12].copy_from_slice(&hmd_us(h).to_le_bytes());
            evs.push((
                h + HMD_DELAY + rng.range(0.0, 1e6) as u64,
                Ev::Hmd(h + HMD_DELAY, hmd_us(h)),
            ));
            h += 1_000_000;
        }
        let mut h = H0 + 2_000_000;
        while h < end {
            let t = (h - H0) as f64 * 1e-9;
            if !gap.is_some_and(|(a, b)| t >= a && t < b) {
                let recv = h + imu_delay + rng.range(0.0, 1e6) as u64;
                let accel = motion.accel(t, &imu_pos, &Vector3::new(0.0, 9.80665, 0.0));
                evs.push((recv, Ev::Imu(h - C0, recv, motion.gyro(t), accel)));
            }
            h += 2_000_000;
        }
        let mut k = 0u32;
        let mut e = H0 + 10_000_000;
        while e < end {
            let t = (e - H0) as f64 * 1e-9;
            let pose = motion.pose(t);
            for (c, w) in cams.iter().enumerate() {
                let cam_from_obj = w.inverse() * pose;
                let blobs: Vec<EngineBlob> = leds
                    .iter()
                    .enumerate()
                    .filter_map(|(i, l)| {
                        let pc = cam_from_obj * Point3::from(l.pos);
                        if (cam_from_obj.rotation * l.dir).dot(&(-pc.coords)) / pc.coords.norm()
                            < 0.3
                        {
                            return None;
                        }
                        let uv = model.project(&pc.coords)?;
                        Some(EngineBlob {
                            x: (uv.x + rng.gauss() * 0.2) as f32,
                            y: (uv.y + rng.gauss() * 0.2) as f32,
                            width: 3,
                            height: 3,
                            area: 9,
                            led_id: (1 << 8) | i as i32,
                        })
                    })
                    .collect();
                let mut info = FrameInfo {
                    serial: serial(c),
                    frame_start_host_ns: e,
                    exposure_valid: 1,
                    exposure_count: k,
                    exposure_hmd_ts: hmd_us(e),
                    n_devices: 1,
                    ..Default::default()
                };
                let (o, q) = crate::calib::iso_to_xyzw(&cam_from_obj);
                info.devices[0].device_id = crate::calib::DEVICE_RIGHT;
                info.devices[0].match_flags = match_flags;
                info.devices[0].pose_valid = 1;
                info.devices[0].cam_orient = o;
                info.devices[0].cam_pos = q;
                evs.push((
                    e + 25_000_000 + c as u64 * 1_000_000,
                    Ev::Frame(Box::new(info), blobs),
                ));
            }
            k += 1;
            e += (1e9 / 52.0) as u64;
        }
        evs.sort_by_key(|e| e.0);
        let (mut errors, mut stale) = (Vec::new(), false);
        for (at, ev) in evs {
            match ev {
                Ev::Hmd(host, us) => {
                    if let Some(rec) = recording {
                        let mut report = [0u8; 64];
                        report[0] = HMD_SENSORS_REPORT;
                        report[8..12].copy_from_slice(&us.to_le_bytes());
                        rec.write(recorder::RAW_REPORT, host, &[&0u32.to_le_bytes(), &report]);
                    }
                    p.hmd_clock_sample(host, us);
                }
                Ev::Frame(info, blobs) => {
                    if let Some(rec) = recording {
                        rec.write(
                            recorder::FRAME_OBS,
                            at,
                            &[as_bytes(&*info), slice_bytes(&blobs)],
                        );
                    }
                    p.frame(&info, &blobs);
                }
                Ev::Imu(dev, recv, gyro, accel) => {
                    if let Some(rec) = recording {
                        let state = crate::shm::ControllerState {
                            device_time_ns: dev,
                            received_host_ns: recv,
                            imu_gyro: gyro.map(|v| v as f32).into(),
                            imu_accel: accel.map(|v| v as f32).into(),
                            ..Default::default()
                        };
                        rec.write(
                            recorder::CONTROLLER_STATE,
                            recv,
                            &[
                                &(HAND_RIGHT as u32).to_le_bytes(),
                                &0u32.to_le_bytes(),
                                as_bytes(&state),
                            ],
                        );
                    }
                    let s = p.imu(HAND_RIGHT, dev, recv, gyro, accel);
                    let t = (at - H0) as f64 * 1e-9;
                    // stale: something stamped before the gap still published after it
                    if let (Some((_, b)), Some(l)) = (gap, p.latest(HAND_RIGHT)) {
                        if t >= b && l.host_ns < H0 + (b * 1e9) as u64 {
                            stale = true;
                        }
                    }
                    if let Some(s) = s {
                        let truth = motion.pose((s.host_ns - HMD_DELAY - H0) as f64 * 1e-9);
                        errors.push((
                            t,
                            (s.output.world_from_model.translation.vector
                                - truth.translation.vector)
                                .norm(),
                        ));
                    }
                }
            }
        }
        (errors, stale, p, calibration)
    }

    /// The engine's poses reach the tracker rated as the engine rated them: one rated only good
    /// is taken too, and published once a second sensor confirms it.
    #[test]
    fn a_pose_the_engine_rates_only_good_is_tracked_once_confirmed() {
        let (errors, _, p, _) = run_with_sensors(3.0, None, IMU_DELAY, 3, None, MATCH_GOOD);
        let st = p.stats(HAND_RIGHT);
        assert!(
            st.good_acquisitions >= 1 && st.unconfirmed_acquisitions == 0,
            "{st:?}"
        );
        let late: Vec<f64> = errors.iter().filter(|e| e.0 > 1.0).map(|e| e.1).collect();
        assert!(late.len() > 900 && late.iter().all(|e| *e < 5e-3), "{st:?}");
    }

    /// What one sensor alone sees is never published: no tracking sample, and no fix time
    /// either (which the driver would report as a pose in range).
    #[test]
    fn nothing_one_sensor_alone_sees_is_published() {
        let (errors, _, p, _) = run_with_sensors(3.0, None, IMU_DELAY, 1, None, MATCH_GOOD);
        let st = p.stats(HAND_RIGHT);
        assert!(
            errors.is_empty() && st.unconfirmed_acquisitions > 0,
            "{st:?}"
        );
        assert!(
            p.latest(HAND_RIGHT)
                .is_none_or(|s| !s.output.tracking && s.last_fix_host_ns == 0),
            "{st:?}"
        );
    }

    #[test]
    fn dynamic_sensors_fifth_camera_survives_recording_and_replay() {
        let path = std::env::temp_dir().join(format!(
            "cv1t-five-sensor-replay-{}.rec",
            std::process::id()
        ));
        let _ = std::fs::remove_file(&path);
        let recorder = crate::recorder::Recorder::new(4 << 20);
        recorder.start(&path, H0).unwrap();
        let (_, _, p, calibration) = run_with_sensors(
            1.5,
            None,
            IMU_DELAY,
            5,
            Some(&recorder),
            MATCH_GOOD | MATCH_STRONG,
        );
        let stats = recorder.stop();
        assert_eq!(stats.dropped, 0);
        assert_eq!(stats.write_errors, 0);
        let participates = |event: &FrameEvent| {
            event.camera == 4 && matches!(event.outcome, Outcome::Updated(_) | Outcome::Acquired(_))
        };
        assert!(
            p.events.iter().any(participates),
            "fifth sensor never updated pipeline"
        );
        let replay = super::super::replay::replay(
            &path,
            &calibration,
            &|_| Some(Vector3::new(0.015, -0.012, 0.008)),
            &p.opts,
        )
        .unwrap();
        assert_eq!(replay.agreement.len(), 5);
        assert!(
            replay.events.iter().any(participates),
            "fifth sensor never updated replay"
        );
        assert!(!replay.hands[HAND_RIGHT].is_empty());
        std::fs::remove_file(path).unwrap();
    }

    /// The daemon places a camera whose calibration is spelled in another letter case than the
    /// device's, and so does the replay of its recording, under the recording's spelling.
    #[test]
    fn replay_places_sensors_calibrated_in_another_letter_case() {
        let path = std::env::temp_dir().join(format!(
            "cv1t-letter-case-replay-{}.rec",
            std::process::id()
        ));
        let _ = std::fs::remove_file(&path);
        let recorder = crate::recorder::Recorder::new(4 << 20);
        recorder.start(&path, H0).unwrap();
        let (_, _, p, mut calibration) = run_with_sensors(
            1.5,
            None,
            IMU_DELAY,
            3,
            Some(&recorder),
            MATCH_GOOD | MATCH_STRONG,
        );
        let stats = recorder.stop();
        assert_eq!((stats.dropped, stats.write_errors), (0, 0));
        for sensor in &mut calibration.sensors {
            sensor.serial = sensor.serial.to_ascii_lowercase();
        }
        let replay = super::super::replay::replay(
            &path,
            &calibration,
            &|_| Some(Vector3::new(0.015, -0.012, 0.008)),
            &p.opts,
        )
        .unwrap();
        let mut registered: Vec<_> = replay.agreement.iter().map(|(s, _)| s.as_str()).collect();
        registered.sort();
        assert_eq!(registered, ["SENSOR0", "SENSOR1", "SENSOR2"]);
        assert!(!replay.hands[HAND_RIGHT].is_empty());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn mixed_case_probe_options_keep_recorded_frames_out_of_tracking() {
        let path = std::env::temp_dir().join(format!("cv1t-probe-case-{}.rec", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let recorder = crate::recorder::Recorder::new(4 << 20);
        recorder.start(&path, H0).unwrap();
        let (_, _, p, mut cal) = run_with_sensors(
            2.0,
            None,
            IMU_DELAY,
            3,
            Some(&recorder),
            MATCH_GOOD | MATCH_STRONG,
        );
        let stats = recorder.stop();
        assert_eq!((stats.dropped, stats.write_errors), (0, 0));
        let options = Options {
            probe_sensors: vec!["sensor0".into()],
            ..p.opts
        };
        for unposed in [false, true] {
            if unposed {
                cal.remove_sensor("SENSOR0");
            }
            let replay = super::super::replay::replay(
                &path,
                &cal,
                &|_| Some(Vector3::new(0.015, -0.012, 0.008)),
                &options,
            )
            .unwrap();
            assert!(!replay.probes.is_empty(), "unposed={unposed}");
            assert!(replay.probes.iter().all(|p| p.camera == 0));
        }
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn the_whole_pipeline_tracks_to_a_millimetre() {
        let (errors, _) = run(5.0, None);
        let late: Vec<f64> = errors.iter().filter(|e| e.0 > 1.0).map(|e| e.1).collect();
        assert!(late.len() > 1500, "{} outputs", late.len());
        let mut v = late.clone();
        v.sort_by(|a, b| a.total_cmp(b));
        assert!(
            v[v.len() / 2] < 1e-3 && v[v.len() * 99 / 100] < 3e-3,
            "p50 {} p99 {}",
            v[v.len() / 2],
            v[v.len() * 99 / 100]
        );
    }

    /// A radio link 5 ms slower than the headset's: the published pose still carries the time the
    /// controller was there, not the time its report arrived, so SteamVR predicts from the right
    /// moment.
    #[test]
    fn a_slow_radio_link_does_not_delay_the_published_pose() {
        let (errors, _) = run_with(5.0, None, 5_000_000);
        let mut v: Vec<f64> = errors.iter().filter(|e| e.0 > 1.0).map(|e| e.1).collect();
        v.sort_by(|a, b| a.total_cmp(b));
        assert!(v[v.len() / 2] < 1e-3, "p50 {} m", v[v.len() / 2]);
    }

    /// The controller's IMU goes quiet for a second (sleep, radio dropout): nothing of the old
    /// track is published after it, and tracking comes back.
    #[test]
    fn nothing_stale_is_published_after_an_imu_gap() {
        let (errors, stale) = run(6.0, Some((3.0, 4.0)));
        assert!(!stale, "the pre-gap pose was still published after the gap");
        assert!(
            errors.iter().any(|e| e.0 > 4.5),
            "tracking did not come back"
        );
        let after: Vec<f64> = errors.iter().filter(|e| e.0 > 5.0).map(|e| e.1).collect();
        assert!(
            after.iter().all(|&e| e < 5e-3),
            "max error after re-acquisition {}",
            after.iter().copied().fold(0.0, f64::max)
        );
    }
    /// A sensor is probed when the options name it, or from the moment `probe` says so, and
    /// tracked with again when it says not; its frames queued the other way are dropped.
    #[test]
    fn probing_is_switched_live_per_sensor() {
        let mut p = Pipeline::new(Options {
            probe_sensors: vec!["B".to_string()],
            ..Options::default()
        });
        p.sensor("A", synth::camera_model(), Isometry3::identity());
        p.sensor("B", synth::camera_model(), Isometry3::identity());
        assert!(!p.is_probed("A") && p.is_probed("B"));
        assert_eq!(
            (p.serial_of(0), p.serial_of(1), p.serial_of(2)),
            (Some("A"), Some("B"), None)
        );
        assert!(p.probe("A", true) && p.probe("B", false));
        assert!(p.is_probed("A") && !p.is_probed("B"));
        assert!(!p.probe("X", true));
        p.probing[0].push((1, 1, Vec::new()));
        p.probing[0].push((2, 0, Vec::new()));
        assert!(p.probe("A", false));
        assert_eq!(p.probing[0].iter().map(|p| p.1).collect::<Vec<_>>(), [1]);
        assert!(p.camera("B").is_some() && p.camera("X").is_none());
        // ignored: its queued probes go, and its agreement can be forgotten
        p.body(DEVICE_RIGHT, synth::touch_leds(), Vector3::zeros());
        p.trackers[HAND_RIGHT].as_mut().unwrap().agreement[1].misses = 9;
        assert!(p.ignore("B", true) && !p.ignore("X", true));
        assert!(p.probing[0].is_empty());
        p.reset_agreement("B");
        assert_eq!(
            p.trackers[HAND_RIGHT].as_ref().unwrap().agreement[1].misses,
            0
        );
        assert!(p.ignore("B", false));
    }

    #[test]
    fn dynamic_sensors_registration_keeps_existing_identity() {
        let mut p = Pipeline::new(Options::default());
        p.sensor("Z", synth::camera_model(), Isometry3::identity());
        p.body(DEVICE_RIGHT, synth::touch_leds(), Vector3::zeros());
        p.trackers[HAND_RIGHT].as_mut().unwrap().agreement[0].samples = 7;
        p.sensor("A", synth::camera_model(), Isometry3::identity());
        assert_eq!(
            p.camera_agreement()
                .into_iter()
                .find(|a| a.0 == "Z")
                .unwrap()
                .1
                .samples,
            7
        );
        p.sensor(
            "Z",
            synth::camera_model(),
            Isometry3::translation(1.0, 0.0, 0.0),
        );
        assert_eq!(p.camera_agreement().len(), 2);
    }

    #[test]
    fn dynamic_sensors_pending_backlog_keeps_every_registered_camera() {
        let mut p = Pipeline::new(Options {
            trace: true,
            ..Options::default()
        });
        for camera in 0..65 {
            p.sensor(
                &format!("SENSOR{camera}"),
                synth::camera_model(),
                Isometry3::identity(),
            );
        }
        p.body(DEVICE_RIGHT, synth::touch_leds(), Vector3::zeros());
        p.hmd_clock_sample(H0, hmd_us(H0));
        p.imu(
            HAND_RIGHT,
            H0 - C0,
            H0,
            Vector3::zeros(),
            Vector3::y() * 9.8,
        );
        for camera in 0..65 {
            let info = FrameInfo {
                serial: serial(camera),
                exposure_valid: 1,
                exposure_hmd_ts: hmd_us(H0 + 10_000_000),
                ..Default::default()
            };
            p.frame(&info, &[]);
        }
        assert_eq!(p.pending[HAND_RIGHT].len(), 65);
        for _ in 0..100 {
            let info = FrameInfo {
                serial: serial(64),
                exposure_valid: 1,
                exposure_hmd_ts: hmd_us(H0 + 20_000_000),
                ..Default::default()
            };
            p.frame(&info, &[]);
        }
        for camera in 0..65 {
            let count = p.pending[HAND_RIGHT]
                .iter()
                .filter(|p| p.camera == camera)
                .count();
            assert!((1..=2).contains(&count), "camera {camera}: {count} queued");
        }
        assert_eq!(p.frames_dropped, 99);
        // An out-of-order old frame must not evict the newest observations for that sensor.
        let old = FrameInfo {
            serial: serial(64),
            exposure_valid: 1,
            exposure_hmd_ts: hmd_us(H0 + 5_000_000),
            ..Default::default()
        };
        p.frame(&old, &[]);
        assert_eq!(p.frames_dropped, 100);
        p.imu(
            HAND_RIGHT,
            H0 + 30_000_000 - C0,
            H0 + 30_000_000,
            Vector3::zeros(),
            Vector3::y() * 9.8,
        );
        assert!(p.pending[HAND_RIGHT].is_empty());
        let serviced: std::collections::BTreeSet<_> =
            p.events.iter().map(|event| event.camera).collect();
        assert_eq!(serviced.len(), 65);
    }
}
