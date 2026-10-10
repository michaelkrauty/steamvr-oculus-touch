// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

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
use super::tracker::{ImuSample, Outcome, Output, Params, Stats, Tracker};
use super::vision::{Blob, Body, Camera};
use crate::calib::camera::CameraModel;
use crate::calib::capture::{Led, MATCH_STRONG};
use crate::calib::{iso_from_xyzw, DEVICE_LEFT, DEVICE_RIGHT};
use crate::engine::{cstr_field, FrameInfo};
use crate::shm::{HAND_LEFT, HAND_RIGHT};

/// Report id of the headset's IMU report, which carries its clock and the exposure stamps.
pub const HMD_SENSORS_REPORT: u8 = 11;

/// How much later than the headset's the controllers' IMU samples are stamped: the radio link's
/// fastest delivery plus the IMU's own filtering, beyond the headset's report delay. Measured from
/// the images alone (eval-1791513601, 2026-10-08): over frames turning faster than 3 rad/s, the
/// predicted LEDs land nearest the blobs at +3.9 to 4.0 ms. Exposures are mapped this much later
/// into a controller's clock, and published poses are stamped this much earlier than their IMU
/// sample's arrival-based time, so that SteamVR predicts from when the controller was there.
pub const DEFAULT_EXPOSURE_OFFSET_NS: i64 = 4_000_000;

/// Wait this long past an exposure before probing it, so frames of it (and of the next exposure)
/// from the tracked sensors are in.
pub const PROBE_DELAY_NS: u64 = 80_000_000;

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
    /// This sensor's frames are not tracked with: each is kept in `Pipeline::probes` with the
    /// controller's pose at its exposure, once the other sensors' frames up to then are applied.
    pub probe_sensor: Option<String>,
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
            probe_sensor: None,
            delayed_ns: None,
            seed_biases: [None, None],
        }
    }
}

/// One frame's outcome for one controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameEvent {
    pub hand: usize,
    /// Index of the sensor in serial order.
    pub camera: usize,
    /// Exposure, controller clock and host clock.
    pub t_e: u64,
    pub exposure_host_ns: u64,
    /// The frame's start on the host clock (its FRAME_OBS / RAW_FRAME stamp).
    pub frame_start_host_ns: u64,
    pub outcome: Outcome,
    /// The engine's strong pose on this frame, and the tracker's at the exposure before it.
    pub engine: Option<Isometry3<f64>>,
    pub tracked: Option<Isometry3<f64>>,
}

/// A probed sensor's frame with the tracked pose of one controller at its exposure.
#[derive(Debug, Clone, PartialEq)]
pub struct Probe {
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
    engine: Option<Isometry3<f64>>,
}

pub struct Pipeline {
    pub opts: Options,
    hmd_clock: LinearClock,
    hmd_stamps: Unwrap32,
    clocks: [LinearClock; 2],
    trackers: [Option<Tracker>; 2],
    cams: BTreeMap<String, Camera>,
    pending: [Vec<Pending>; 2],
    probing: [Vec<(u64, Vec<Blob>)>; 2],
    latest: [Option<Stamped>; 2],
    /// Frames before the headset clock had a fit, or from sensors without a pose.
    pub frames_skipped: u64,
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
            pending: [Vec::new(), Vec::new()],
            probing: [Vec::new(), Vec::new()],
            latest: [None, None],
            frames_skipped: 0,
            events: Vec::new(),
            probes: Vec::new(),
        }
    }

    /// A sensor and its calibrated pose (camera -> world, OpenCV axes).
    pub fn sensor(&mut self, serial: &str, model: CameraModel, world_from_cam: Isometry3<f64>) {
        self.cams.insert(
            serial.to_string(),
            Camera {
                model,
                world_from_cam,
            },
        );
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
            let mut tr = Tracker::new(device_id, Body { leds, imu_pos }, self.opts.params);
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
    /// combined: (serial, median residual px, |bias| px, samples).
    pub fn camera_agreement(&self) -> Vec<(String, f64, f64, u64)> {
        self.cams
            .keys()
            .enumerate()
            .map(|(i, serial)| {
                let (mut sum, mut bias, mut n) = (0.0, Vector2::zeros(), 0u64);
                for tr in self.trackers.iter().flatten() {
                    if let Some(a) = tr.agreement.get(i) {
                        sum += a.median_px * a.samples as f64;
                        bias += a.bias_px * a.samples as f64;
                        n += a.samples;
                    }
                }
                let k = if n > 0 { 1.0 / n as f64 } else { f64::NAN };
                (serial.clone(), sum * k, (bias * k).norm(), n)
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
            let (t_e, blobs) = self.probing[hand].remove(0);
            if let Some(f) = tr.filter_at(t_e) {
                let pos_sd = (0..3)
                    .map(|i| f.cov[(3 + i, 3 + i)])
                    .fold(0.0, f64::max)
                    .sqrt();
                self.probes.push(Probe {
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
            // no state yet (or dropped after a gap): nothing of ours to publish
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
        let last_fix_host_ns = self.clocks[hand].to_host(tr.last_fix()).map_or(0, physical);
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
        let camera = self
            .cams
            .keys()
            .position(|s| *s == serial)
            .unwrap_or(usize::MAX);
        let cam = self.cams.get(&serial).copied();
        let exposure = self
            .hmd_stamps
            .relative(info.exposure_hmd_ts)
            .and_then(|e| self.hmd_clock.to_host(e));
        let (Some(cam), Some(exposure), true) = (cam, exposure, info.exposure_valid != 0) else {
            self.frames_skipped += 1;
            return;
        };
        let list: Vec<Blob> = blobs
            .iter()
            .map(|b| Blob {
                xy: Vector2::new(b.x as f64, b.y as f64),
                led_id: b.led_id,
            })
            .collect();
        let probe = self.opts.probe_sensor.as_deref() == Some(serial.as_str());
        for hand in 0..2 {
            let Some(tr) = self.trackers[hand].as_mut() else {
                continue;
            };
            let Some(t) = self.clocks[hand].to_device(exposure) else {
                continue;
            };
            let t_e = (t as i64 + self.opts.exposure_offset_ns).max(0) as u64;
            if probe {
                self.probing[hand].push((t_e, list.clone()));
                continue;
            }
            let engine = info.devices[..(info.n_devices as usize).min(info.devices.len())]
                .iter()
                .find(|d| {
                    d.device_id == tr.device_id
                        && d.pose_valid != 0
                        && d.match_flags & MATCH_STRONG != 0
                })
                .map(|d| cam.world_from_cam * iso_from_xyzw(d.cam_orient, d.cam_pos));
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
        // a frame whose exposure the IMU never reaches (the controller went to sleep) must not pile up
        if pending.len() > 64 {
            pending.drain(..pending.len() - 64);
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
        let mut rng = Rng::new(5);
        let motion = Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0));
        let leds = synth::touch_leds();
        let imu_pos = Vector3::new(0.015, -0.012, 0.008);
        let target = Vector3::new(0.0, 1.0, 0.0);
        let cams: Vec<Isometry3<f64>> = [
            (Vector3::new(-2.0, 2.3, 1.8), true),
            (Vector3::new(2.0, 2.3, 1.8), false),
            (Vector3::new(0.3, 2.4, -2.2), true),
        ]
        .iter()
        .map(|&(p, flip)| synth::look_at(p, target, flip))
        .collect();
        let model = synth::camera_model();
        // the clocks map with each link's fastest delay, so the exposure lands that much off:
        // the offset makes it exact here
        let mut p = Pipeline::new(Options {
            exposure_offset_ns: imu_delay as i64 - HMD_DELAY as i64,
            ..Options::default()
        });
        for (i, w) in cams.iter().enumerate() {
            p.sensor(&format!("SENSOR{i}"), model, *w);
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
                info.devices[0].match_flags = 3;
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
                Ev::Hmd(host, us) => p.hmd_clock_sample(host, us),
                Ev::Frame(info, blobs) => p.frame(&info, &blobs),
                Ev::Imu(dev, recv, gyro, accel) => {
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
        (errors, stale)
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
}
