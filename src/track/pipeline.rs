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
use super::epochs::{Checks, Evidence, State};
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

/// The inspection identity a frame's labels for `device_id` carry: 0 for ordinary matching,
/// None for an identified result without its epoch.
fn identity_epoch(info: &FrameInfo, device_id: i32) -> Option<u64> {
    info.devices[..(info.n_devices as usize).min(info.devices.len())]
        .iter()
        .find(|d| d.device_id == device_id)
        .map(|d| {
            if d.match_flags & crate::engine::MATCH_INSPECTION_IDENTITY == 0 {
                Some(0)
            } else {
                let epoch = u64::from(d.identity_epoch_lo) | (u64::from(d.identity_epoch_hi) << 32);
                (epoch != 0).then_some(epoch)
            }
        })
        .unwrap_or(Some(0))
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
    /// Native correspondence lifetime; zero denotes ordinary matching.
    pub identity_epoch: u64,
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

/// Keep counter overflow as conservative as lost lifecycle notices. Frames can arrive before
/// the radio loop's next mailbox extraction, so counting may reach the bound first.
fn count_observed(
    observed: &mut Evidence,
    lifetimes: &mut Evidence,
    agreement: &mut CameraAgreement,
    epoch: u64,
    checks: Checks,
) -> bool {
    let before = observed.forgotten_through;
    observed.count(epoch, checks);
    if observed.forgotten_through > before {
        lifetimes.lifecycle(observed);
        observed.lifecycle(lifetimes);
        *agreement = Default::default();
        true
    } else {
        false
    }
}

struct Pending {
    t_e: u64,
    exposure_host_ns: u64,
    frame_start_host_ns: u64,
    camera: usize,
    cam: Camera,
    blobs: Vec<Blob>,
    engine: Option<EnginePose>,
    identity_epoch: Option<u64>,
}

struct PendingProbe {
    t_e: u64,
    camera: usize,
    blobs: Vec<Blob>,
    identity_epoch: u64,
}

/// What an inspected or probed sensor's engine search reported about a hand since the
/// inspection or probing began.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct IdentityEvidence {
    /// A correspondence of this hand matched well enough to be checked against its IMU.
    pub seen: bool,
    /// The hand's own turn could establish (or had established) its identity: without it, a
    /// hand that was seen was too still, or moved together with the other one.
    pub judgeable: bool,
}

impl IdentityEvidence {
    /// The search found the requested hands but they never turned enough to be told apart:
    /// no verdict, only a reason to look again soon.
    pub fn starved(evidence: &[IdentityEvidence; 2], hands: [bool; 2]) -> bool {
        let requested = || {
            (0..2)
                .filter(|&hand| hands[hand])
                .map(|hand| evidence[hand])
        };
        requested().any(|e| e.seen) && !requested().any(|e| e.judgeable)
    }
}

struct InspectionExposure {
    camera: usize,
    frame_start_host_ns: u64,
    t_e: u64,
    until: u64,
    held: bool,
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
    camera_slots: usize,
    /// Registration ids of the sensors whose frames are probed, not tracked with.
    probed: std::collections::BTreeSet<usize>,
    /// Registration ids of the sensors whose frames are dropped (given up on).
    ignored: std::collections::BTreeSet<usize>,
    pending: [Vec<Pending>; 2],
    inspection_starts: [Vec<InspectionExposure>; 2],
    inspecting: std::collections::BTreeSet<usize>,
    /// Per inspected or probed sensor, by registration id.
    identity_evidence: BTreeMap<usize, [IdentityEvidence; 2]>,
    observed_evidence: BTreeMap<usize, [Evidence; 2]>,
    /// Sticky tombstones outlive inspection counters, until the engine acknowledges the
    /// sensor's restored pose (`retire_identities`). A new engine gets a new Pipeline, or a
    /// replay's `new_engine_session`; within it forgotten epochs stay behind the watermark.
    identity_lifetimes: BTreeMap<usize, [Evidence; 2]>,
    /// Lifecycle changes not yet handed to the movement and placement consumers.
    lifecycle_changes: BTreeMap<usize, [Evidence; 2]>,
    /// Per hand: a probed sensor's frames (exposure, camera, blobs) waiting for the track to
    /// pass their exposure.
    probing: [Vec<PendingProbe>; 2],
    latest: [Option<Stamped>; 2],
    /// Frames before the headset clock had a fit, from sensors without a pose, or from sensors
    /// given up on (`ignore`).
    pub frames_skipped: u64,
    /// Pending frames evicted by the per-camera two-frame limit, summed across hands.
    pub frames_dropped: u64,
    /// Lifecycle watermarks advanced by producer or consumer overflow.
    pub lifecycle_overflows: u64,
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
            camera_slots: 0,
            probed: Default::default(),
            ignored: Default::default(),
            pending: [Vec::new(), Vec::new()],
            inspection_starts: [Vec::new(), Vec::new()],
            inspecting: Default::default(),
            identity_evidence: BTreeMap::new(),
            observed_evidence: BTreeMap::new(),
            identity_lifetimes: BTreeMap::new(),
            lifecycle_changes: BTreeMap::new(),
            probing: [Vec::new(), Vec::new()],
            latest: [None, None],
            frames_skipped: 0,
            frames_dropped: 0,
            lifecycle_overflows: 0,
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
        let id = self.camera_slots;
        self.camera_slots += 1;
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

    /// Retire the camera and everything waiting to use it. IDs are never reused within this
    /// pipeline: a solver result still in flight must not apply to a later camera. Returns its
    /// retired ID so placement state can be forgotten under the same identity.
    pub fn remove_sensor(&mut self, serial: &str) -> Option<usize> {
        let serial = self
            .cams
            .keys()
            .find(|s| crate::status::same_sensor(s, serial))
            .cloned()?;
        self.ignore(&serial, true);
        let camera = self.cams.remove(&serial)?;
        let id = camera.id;
        self.probed.remove(&id);
        self.ignored.remove(&id);
        self.inspecting.remove(&id);
        self.identity_evidence.remove(&id);
        self.observed_evidence.remove(&id);
        self.identity_lifetimes.remove(&id);
        self.lifecycle_changes.remove(&id);
        for starts in &mut self.inspection_starts {
            starts.retain(|s| s.camera != id);
        }
        self.events.retain(|e| e.camera != id);
        self.probes.retain(|p| p.camera != id);
        for tracker in self.trackers.iter_mut().flatten() {
            tracker.forget_camera(id);
        }
        Some(id)
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
            self.identity_evidence.insert(id, Default::default());
            for pending in &mut self.pending {
                pending.retain(|p| p.camera != id);
            }
        } else {
            self.probed.remove(&id);
            if !self.inspecting.contains(&id) {
                self.identity_evidence.remove(&id);
            }
            for probing in &mut self.probing {
                probing.retain(|p| p.camera != id);
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
                probing.retain(|p| p.camera != id);
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
                self.camera_slots,
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

    /// Keeps the controllers separate while judging a sensor: agreement on one hand cannot
    /// establish the other hand's prior, especially when that hand is occluded.
    pub fn camera_agreement_by_hand(&self) -> Vec<(String, [CameraAgreement; 2])> {
        self.cams
            .iter()
            .map(|(serial, camera)| {
                let hands = [0, 1].map(|hand| {
                    self.trackers[hand]
                        .as_ref()
                        .and_then(|tr| tr.agreement.get(camera.id))
                        .copied()
                        .unwrap_or_default()
                });
                (serial.clone(), hands)
            })
            .collect()
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
                        all.observed_checks += a.observed_checks;
                        all.observed_misses += a.observed_misses;
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
            self.inspection_starts[hand].clear();
        }
        self.inspection_starts[hand].retain(|s| s.until > device_ns);
        self.drain(hand);
        let tr = self.trackers[hand].as_mut()?;
        while self.probing[hand]
            .first()
            .is_some_and(|p| p.t_e + PROBE_DELAY_NS <= device_ns)
        {
            let PendingProbe {
                t_e,
                camera,
                blobs,
                identity_epoch,
            } = self.probing[hand].remove(0);
            if self
                .identity_lifetimes
                .get(&camera)
                .is_some_and(|e| !e[hand].permits(identity_epoch))
            {
                continue;
            }
            if let Some(f) = tr.filter_at(t_e) {
                let pos_sd = (0..3)
                    .map(|i| f.cov[(3 + i, 3 + i)])
                    .fold(0.0, f64::max)
                    .sqrt();
                self.probes.push(Probe {
                    camera,
                    hand,
                    device_id: tr.device_id,
                    identity_epoch,
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

    /// A frame's exposure on the controller clock, using the same mapping for early search
    /// notifications and completed frames.
    pub fn exposure_time(&self, info: &FrameInfo, hand: usize) -> Option<u64> {
        if info.exposure_valid == 0 {
            return None;
        }
        let exposure = self
            .hmd_clock
            .to_host(self.hmd_stamps.relative(info.exposure_hmd_ts)?)?;
        let t = self.clocks.get(hand)?.to_device(exposure)?;
        Some((t as i64 + self.opts.exposure_offset_ns).max(0) as u64)
    }

    pub fn begin_inspection(&mut self, serial: &str) {
        if let Some(camera) = self.cams.get(serial) {
            self.inspecting.insert(camera.id);
            self.identity_evidence.insert(camera.id, Default::default());
            self.observed_evidence.insert(
                camera.id,
                self.identity_lifetimes
                    .get(&camera.id)
                    .cloned()
                    .unwrap_or_default(),
            );
        }
    }

    /// The inspection's actual observed counter deltas, tagged by their native identity epoch.
    pub fn observed_evidence(&self, serial: &str) -> [Evidence; 2] {
        self.camera_id(serial)
            .and_then(|id| self.observed_evidence.get(&id))
            .cloned()
            .unwrap_or_default()
    }

    pub fn inspection_identity(&mut self, event: &crate::engine::InspectionIdentity) {
        let Some(hand) = device_hand(event.device_id) else {
            return;
        };
        let Some(state) = State::from_native(event.state) else {
            return;
        };
        let mut states: [Evidence; 2] = Default::default();
        states[hand].note(event.epoch, state);
        let serial = cstr_field(&event.serial);
        self.inspection_lifecycle(&serial, &states);
    }

    /// Apply retained lifecycle notices before a movement decision. A normal end preserves
    /// earlier checks. An invalidation also clears the affected agreement's residual history,
    /// so the small-bias rule cannot keep using a disproved identity's fitted residuals.
    pub fn inspection_lifecycle(&mut self, serial: &str, states: &[Evidence; 2]) {
        let Some(camera) = self.camera_id(serial) else {
            return;
        };
        let lifetimes = self.identity_lifetimes.entry(camera).or_default();
        for hand in 0..2 {
            let invalidated = states[hand].invalidates(&lifetimes[hand]);
            let before = lifetimes[hand].forgotten_through;
            lifetimes[hand].lifecycle(&states[hand]);
            let overflow = lifetimes[hand].forgotten_through > before;
            if overflow {
                self.lifecycle_overflows += 1;
            }
            if let Some(observed) = self.observed_evidence.get_mut(&camera) {
                observed[hand].lifecycle(&lifetimes[hand]);
            }
            if invalidated || overflow {
                self.lifecycle_changes.entry(camera).or_default()[hand].lifecycle(&states[hand]);
                if let Some(tr) = &mut self.trackers[hand] {
                    tr.agreement[camera] = Default::default();
                }
                self.probing[hand]
                    .retain(|p| p.camera != camera || lifetimes[hand].permits(p.identity_epoch));
                self.probes.retain(|p| {
                    p.camera != camera
                        || p.hand != hand
                        || lifetimes[hand].permits(p.identity_epoch)
                });
            }
        }
    }

    /// A sensor's tombstones and watermark, kept through placement until its restored pose is
    /// acknowledged.
    pub fn identity_lifetimes(&self, camera: usize) -> [Evidence; 2] {
        self.identity_lifetimes
            .get(&camera)
            .cloned()
            .unwrap_or_default()
    }

    /// New tombstones and watermarks since the last call, by registration id: what the movement
    /// and placement consumers have not yet seen, never the whole history.
    pub fn take_lifecycle_changes(&mut self) -> Vec<(usize, [Evidence; 2])> {
        std::mem::take(&mut self.lifecycle_changes)
            .into_iter()
            .collect()
    }

    /// Whether a hand of this sensor frame carries an identity epoch its tombstones reject.
    pub fn retracts(&self, info: &FrameInfo) -> bool {
        let Some(lifetimes) = self
            .camera_id(&cstr_field(&info.serial))
            .and_then(|camera| self.identity_lifetimes.get(&camera))
        else {
            return false;
        };
        [(HAND_LEFT, DEVICE_LEFT), (HAND_RIGHT, DEVICE_RIGHT)]
            .into_iter()
            .any(|(hand, device)| {
                identity_epoch(info, device).is_some_and(|e| !lifetimes[hand].permits(e))
            })
    }

    /// The engine acknowledges a restored pose only after ending the sensor's identities and
    /// delivering every frame and lifecycle notice it tagged (`apply_pending_pose`,
    /// `rift_sensor_cv1touch_pose_applied`). The caller first drops that sensor's frames still
    /// queued that the tombstones retract (`retracts`); the rest carry no tombstoned epoch.
    /// Pending frames are checked again only when consumed, so those of an invalidated epoch
    /// go now; probing and probes lost theirs when each tombstone arrived. A search's counts
    /// went when it was cancelled at the restoration request, unless the sensor was still
    /// probed then (placed again or given up): those go now. The tombstones retire; the
    /// watermark stays. Movement and placement have finished with the search, and a new one
    /// takes new epochs.
    pub fn retire_identities(&mut self, serial: &str) {
        let Some(camera) = self.camera_id(serial) else {
            return;
        };
        self.observed_evidence.remove(&camera);
        let Some(lifetimes) = self.identity_lifetimes.get_mut(&camera) else {
            return;
        };
        for (pending, lifetimes) in self.pending.iter_mut().zip(lifetimes) {
            pending.retain(|p| {
                p.camera != camera || p.identity_epoch.is_none_or(|e| lifetimes.permits(e))
            });
            lifetimes.retire();
        }
    }

    /// The engine reopened and numbers its identities afresh, so nothing of the old engine's
    /// can judge the new one's: its searches, counts and tombstones go, with the frames and
    /// probes still tagged by its epochs. The daemon gives each engine session a new Pipeline;
    /// a replay keeps one across the reopen, for its tracking, and calls this.
    pub fn new_engine_session(&mut self) {
        self.inspecting.clear();
        self.identity_evidence.clear();
        self.observed_evidence.clear();
        self.identity_lifetimes.clear();
        self.lifecycle_changes.clear();
        for hand in 0..2 {
            self.inspection_starts[hand].clear();
            self.pending[hand].retain(|p| p.identity_epoch.is_none_or(|e| e == 0));
            self.probing[hand].retain(|p| p.identity_epoch == 0);
            if let Some(tr) = self.trackers[hand].as_mut() {
                for camera in self.cams.values() {
                    tr.cancel_inspection(camera.id);
                }
            }
        }
    }

    /// What the engine reported about each hand's identity on `serial` since its inspection or
    /// probing began (nothing for a sensor never inspected).
    pub fn identity_evidence(&self, serial: &str) -> [IdentityEvidence; 2] {
        self.cams
            .get(serial)
            .and_then(|camera| self.identity_evidence.get(&camera.id))
            .copied()
            .unwrap_or_default()
    }

    /// The native worker announces a search before it can outlive normal filter history.
    /// Keep only its outstanding capture-buffer exposures, independently of ordinary frames.
    pub fn hold_inspection(&mut self, info: &FrameInfo, remaining_ns: u64) {
        let serial = cstr_field(&info.serial);
        let Some(registered) = self.cams.get(&serial).copied() else {
            return;
        };
        if info.exposure_valid == 0
            || remaining_ns == 0
            || !(self.inspecting.contains(&registered.id) || self.probed.contains(&registered.id))
        {
            return;
        }
        for hand in 0..2 {
            let Some(t_e) = self.exposure_time(info, hand) else {
                continue;
            };
            let newest = self.trackers[hand]
                .as_ref()
                .and_then(|tr| tr.newest_imu())
                .unwrap_or(t_e);
            let starts = &mut self.inspection_starts[hand];
            starts.retain(|s| {
                s.camera != registered.id || s.frame_start_host_ns != info.frame_start_host_ns
            });
            if starts.iter().filter(|s| s.camera == registered.id).count() >= 4 {
                let k = starts
                    .iter()
                    .position(|s| s.camera == registered.id)
                    .unwrap();
                starts.remove(k);
            }
            starts.push(InspectionExposure {
                camera: registered.id,
                frame_start_host_ns: info.frame_start_host_ns,
                t_e,
                until: newest.saturating_add(remaining_ns),
                held: false,
            });
            self.drain(hand);
        }
    }

    pub fn cancel_inspection(&mut self, serial: &str) {
        if let Some(camera) = self.cams.get(serial) {
            self.inspecting.remove(&camera.id);
            if !self.probed.contains(&camera.id) {
                self.identity_evidence.remove(&camera.id);
                self.observed_evidence.remove(&camera.id);
            }
            for hand in 0..2 {
                self.inspection_starts[hand].retain(|s| s.camera != camera.id);
                if let Some(tr) = self.trackers[hand].as_mut() {
                    tr.cancel_inspection(camera.id);
                }
            }
        }
    }

    /// One sensor frame from the engine.
    pub fn frame(&mut self, info: &FrameInfo, blobs: &[crate::engine::Blob]) {
        let serial = cstr_field(&info.serial);
        let registered = self.cams.get(&serial).copied();
        if let Some(evidence) = registered.and_then(|r| self.identity_evidence.get_mut(&r.id)) {
            for d in &info.devices[..(info.n_devices as usize).min(info.devices.len())] {
                let Some(hand) = device_hand(d.device_id) else {
                    continue;
                };
                // An inspected hand's good match is exported only once identified; otherwise
                // its identity check withheld the pose. A pose exported without identity is
                // ordinary tracking from before the inspection reached the engine.
                let identified = d.match_flags & crate::engine::MATCH_INSPECTION_IDENTITY != 0;
                evidence[hand].seen |=
                    d.match_flags & MATCH_GOOD != 0 && (d.pose_valid == 0 || identified);
                evidence[hand].judgeable |=
                    identified || d.match_flags & crate::engine::MATCH_INSPECTION_MOTION != 0;
            }
        }
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
            // Clock fits can improve during a multi-second search. Match the stable frame ID,
            // and reuse the exposure mapping its retained prior actually belongs to.
            let saved = self.inspection_starts[hand]
                .iter()
                .position(|s| {
                    s.camera == camera && s.frame_start_host_ns == info.frame_start_host_ns
                })
                .map(|k| self.inspection_starts[hand].remove(k).t_e);
            let Some(t_e) = saved.or_else(|| self.exposure_time(info, hand)) else {
                continue;
            };
            let Some(tr) = self.trackers[hand].as_mut() else {
                continue;
            };
            let identity_epoch = identity_epoch(info, tr.device_id);
            if identity_epoch.is_some_and(|epoch| {
                self.identity_lifetimes
                    .get(&camera)
                    .is_some_and(|e| !e[hand].permits(epoch))
            }) {
                tr.take_inspection_prior(camera, t_e);
                continue;
            }
            if probe {
                let Some(identity_epoch) = identity_epoch else {
                    continue;
                };
                // Labels that arrive after their exposure left the filter history can only use
                // its held prior. Any other waits in `probing` until the IMU is past it.
                let held = tr.take_inspection_prior(camera, t_e);
                if tr.before_history(t_e) {
                    if let Some(f) = held {
                        let pos_sd = (0..3)
                            .map(|i| f.cov[(3 + i, 3 + i)])
                            .fold(0.0, f64::max)
                            .sqrt();
                        self.probes.push(Probe {
                            camera,
                            hand,
                            device_id: tr.device_id,
                            identity_epoch,
                            world_from_model: tr.body.world_from_model(&f.x),
                            pos_sd,
                            blobs: list.iter().map(|b| b.xy).collect(),
                            labels: list.iter().map(|b| b.led_id).collect(),
                        });
                    }
                    continue;
                }
                if self.probing[hand].len() >= PROBING_CAP {
                    self.probing[hand].remove(0);
                }
                self.probing[hand].push(PendingProbe {
                    t_e,
                    camera,
                    blobs: list.clone(),
                    identity_epoch,
                });
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
                    identified: d.match_flags & crate::engine::MATCH_INSPECTION_IDENTITY != 0,
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
                    identity_epoch,
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
            inspection_starts,
            observed_evidence,
            identity_lifetimes,
            lifecycle_changes,
            lifecycle_overflows,
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
        for start in &mut inspection_starts[hand] {
            if !start.held && tr.newest_imu().is_some_and(|t| t >= start.t_e) {
                tr.hold_inspection(
                    start.camera,
                    start.t_e,
                    start.until.saturating_sub(tr.newest_imu().unwrap()),
                );
                start.held = true;
            }
        }
        let pending = &mut pending[hand];
        let other_clock = &clocks[1 - hand];
        let min_facing = opts.params.fit.assoc.min_facing;
        while let Some(p) = pending.first() {
            if p.identity_epoch.is_some_and(|epoch| {
                identity_lifetimes
                    .get(&p.camera)
                    .is_some_and(|e| !e[hand].permits(epoch))
            }) {
                tr.take_inspection_prior(p.camera, p.t_e);
                pending.remove(0);
                continue;
            }
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
            let before = tr.agreement[p.camera];
            let outcome = tr.frame(p.t_e, p.camera, &p.cam, &p.blobs, p.engine, &foreign);
            if outcome == Outcome::Early {
                break;
            }
            if let (Some(epoch), Some(evidence)) =
                (p.identity_epoch, observed_evidence.get_mut(&p.camera))
            {
                let after = tr.agreement[p.camera];
                let lifetimes = &mut identity_lifetimes.entry(p.camera).or_default()[hand];
                if count_observed(
                    &mut evidence[hand],
                    lifetimes,
                    &mut tr.agreement[p.camera],
                    epoch,
                    Checks {
                        checks: after.observed_checks.saturating_sub(before.observed_checks),
                        misses: after.observed_misses.saturating_sub(before.observed_misses),
                    },
                ) {
                    *lifecycle_overflows += 1;
                    lifecycle_changes.entry(p.camera).or_default()[hand].lifecycle(lifetimes);
                }
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

    #[test]
    fn delayed_inspection_checks_survive_normal_end_but_not_invalidation() {
        for state in [State::Ended, State::Invalidated] {
            let (_, _, mut p, _) = run_with_inspection_lifecycle(
                3.5,
                None,
                IMU_DELAY,
                3,
                None,
                (
                    MATCH_GOOD | MATCH_STRONG | crate::engine::MATCH_INSPECTION_IDENTITY,
                    Some(1_000_000_000),
                ),
                Some((state, false, false)),
            );
            let evidence = p.observed_evidence("SENSOR2");
            let epoch = &evidence[HAND_RIGHT].epochs[&((1 << 32) | 1)];
            // A normal end keeps no lifecycle state: the late frame's checks simply count.
            assert_eq!(
                epoch.state,
                if state == State::Ended {
                    State::Started
                } else {
                    state
                }
            );
            assert_eq!(
                epoch.observed,
                if state == State::Ended {
                    Checks {
                        checks: 1,
                        misses: 1,
                    }
                } else {
                    Checks::default()
                }
            );
            assert!(evidence[HAND_LEFT].epochs.is_empty());
            assert_eq!(
                p.tracker(HAND_RIGHT).unwrap().agreement[2].observed_checks,
                u64::from(state == State::Ended),
                "an invalid late frame must not rebuild residual or miss statistics"
            );
            if state == State::Ended {
                let peers = p.tracker(HAND_RIGHT).unwrap().agreement[..2].to_vec();
                let pose = p.latest(HAND_RIGHT).unwrap().output.world_from_model;
                p.cancel_inspection("SENSOR2");
                p.inspection_identity(&crate::engine::InspectionIdentity {
                    serial: serial(2),
                    epoch: (1 << 32) | 1,
                    device_id: DEVICE_RIGHT,
                    state: crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED,
                });
                assert_eq!(
                    p.tracker(HAND_RIGHT).unwrap().agreement[2].observed_checks,
                    0
                );
                for (i, peer) in peers.iter().enumerate() {
                    let actual = &p.tracker(HAND_RIGHT).unwrap().agreement[i];
                    assert_eq!(actual.observed_checks, peer.observed_checks);
                    assert_eq!(actual.bias_px, peer.bias_px);
                }
                assert_eq!(p.latest(HAND_RIGHT).unwrap().output.world_from_model, pose);
            }
        }
    }

    #[test]
    fn invalidation_survives_cancellation_and_delayed_frames() {
        let (_, _, p, _) = run_with_inspection_lifecycle(
            3.5,
            None,
            IMU_DELAY,
            3,
            None,
            (
                MATCH_GOOD | MATCH_STRONG | crate::engine::MATCH_INSPECTION_IDENTITY,
                Some(100_000_000),
            ),
            Some((State::Invalidated, true, false)),
        );
        assert_eq!(
            p.tracker(HAND_RIGHT).unwrap().agreement[2].observed_checks,
            0
        );
    }

    #[test]
    fn invalidated_epochs_cannot_supply_delayed_placement_probes() {
        for delay in [0, 1_000_000_000] {
            for state in [State::Ended, State::Invalidated] {
                let (_, _, p, _) = run_with_inspection_lifecycle(
                    3.5,
                    None,
                    IMU_DELAY,
                    3,
                    None,
                    (
                        MATCH_GOOD | MATCH_STRONG | crate::engine::MATCH_INSPECTION_IDENTITY,
                        Some(delay),
                    ),
                    Some((state, false, true)),
                );
                let probes: Vec<_> = p.probes.iter().filter(|p| p.camera == 2).collect();
                assert_eq!(
                    probes.len(),
                    usize::from(state == State::Ended),
                    "{state:?}, {delay}"
                );
                if let Some(probe) = probes.first() {
                    assert_eq!(probe.identity_epoch, (1 << 32) | 1);
                }
            }
        }
    }

    #[test]
    fn invalidation_retracts_already_queued_placement_probes() {
        let (_, _, mut p, _) = run_with_inspection_lifecycle(
            3.5,
            None,
            IMU_DELAY,
            3,
            None,
            (
                MATCH_GOOD | MATCH_STRONG | crate::engine::MATCH_INSPECTION_IDENTITY,
                Some(1_000_000_000),
            ),
            Some((State::Ended, false, true)),
        );
        let probe = p.probes.iter().find(|p| p.camera == 2).unwrap().clone();
        p.probing[HAND_RIGHT].push(PendingProbe {
            t_e: 0,
            camera: 2,
            identity_epoch: probe.identity_epoch,
            blobs: probe
                .blobs
                .iter()
                .zip(&probe.labels)
                .map(|(&xy, &led_id)| Blob { xy, led_id })
                .collect(),
        });
        let mut event = crate::engine::InspectionIdentity {
            serial: serial(2),
            epoch: probe.identity_epoch,
            device_id: DEVICE_RIGHT,
            state: crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED,
        };
        p.inspection_identity(&event);
        assert_eq!(p.probing[HAND_RIGHT].len(), 1);
        assert_eq!(p.probes.len(), 1);
        event.state = crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED;
        p.inspection_identity(&event);
        assert!(p.probing[HAND_RIGHT].is_empty());
        assert!(p.probes.is_empty());
        p.cancel_inspection("SENSOR2");
        p.begin_inspection("SENSOR2");
        assert!(!p.identity_lifetimes(2)[HAND_RIGHT].permits(probe.identity_epoch));
    }

    #[test]
    fn lost_lifecycle_events_clear_residual_history() {
        let (_, _, mut p, _) = run_with_inspection_lifecycle(
            3.5,
            None,
            IMU_DELAY,
            3,
            None,
            (
                MATCH_GOOD | MATCH_STRONG | crate::engine::MATCH_INSPECTION_IDENTITY,
                Some(1_000_000_000),
            ),
            Some((State::Ended, false, false)),
        );
        assert_eq!(
            p.tracker(HAND_RIGHT).unwrap().agreement[2].observed_checks,
            1
        );
        let mut mailbox = super::super::epochs::Mailbox::default();
        mailbox.register(["SENSOR2".into()]);
        let epoch = (1 << 32) | 1;
        // A stalled consumer: normal ends leave nothing to deliver, so only invalidations of
        // later epochs can fill the mailbox, which forgets the oldest behind its watermark.
        for offset in 0..4096 {
            mailbox.note(&crate::engine::InspectionIdentity {
                serial: serial(2),
                epoch: epoch + offset,
                device_id: DEVICE_RIGHT,
                state: if offset < 2048 {
                    crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED
                } else {
                    crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED
                },
            });
        }
        let updates = mailbox.take();
        assert!(updates[0].1[HAND_RIGHT].forgotten_through >= epoch);
        p.inspection_lifecycle("SENSOR2", &updates[0].1);
        assert_eq!(
            p.tracker(HAND_RIGHT).unwrap().agreement[2].observed_checks,
            0
        );
    }

    #[test]
    fn counter_overflow_retracts_residual_history_and_leaves_a_tombstone() {
        let mut observed = Evidence::default();
        let mut lifetimes = Evidence::default();
        let mut agreement = CameraAgreement {
            observed_checks: 100,
            observed_misses: 0,
            samples: 100,
            median_px: 0.9,
            bias_px: Vector2::new(0.9, 0.0),
            ..Default::default()
        };
        for epoch in 1..=1024 {
            observed.count(
                epoch,
                Checks {
                    checks: 1,
                    misses: 0,
                },
            );
        }
        let overflow = count_observed(
            &mut observed,
            &mut lifetimes,
            &mut agreement,
            1025,
            Checks {
                checks: 1,
                misses: 0,
            },
        );
        assert!(overflow);
        assert_eq!(agreement.observed_checks, 0);
        assert_eq!(agreement.samples, 0);
        assert!(!lifetimes.permits(1));
        lifetimes.note(1, State::Started);
        assert!(
            !lifetimes.permits(1),
            "a delayed notice cannot revive lost evidence"
        );
        assert!(lifetimes.permits(1025));
    }

    #[test]
    fn locally_forgotten_lifetimes_clear_residual_history() {
        let (_, _, mut p, _) = run_with_inspection_lifecycle(
            3.5,
            None,
            IMU_DELAY,
            3,
            None,
            (
                MATCH_GOOD | MATCH_STRONG | crate::engine::MATCH_INSPECTION_IDENTITY,
                Some(1_000_000_000),
            ),
            Some((State::Ended, false, false)),
        );
        assert_eq!(
            p.tracker(HAND_RIGHT).unwrap().agreement[2].observed_checks,
            1
        );
        let epoch = (1 << 32) | 1;
        let notice = |offset, state| crate::engine::InspectionIdentity {
            serial: serial(2),
            epoch: epoch + offset,
            device_id: DEVICE_RIGHT,
            state,
        };
        // Normal ends can never be retracted, so any number of them keeps nothing.
        for offset in 1..=2048 {
            p.inspection_identity(&notice(offset, crate::ffi::OHMD_CV1TOUCH_IDENTITY_STARTED));
            p.inspection_identity(&notice(offset, crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED));
        }
        assert_eq!(p.identity_lifetimes(2)[HAND_RIGHT], Evidence::default());
        assert_eq!(
            p.tracker(HAND_RIGHT).unwrap().agreement[2].observed_checks,
            1
        );
        assert_eq!(p.lifecycle_overflows, 0);
        // Individual notices do not overflow the mailbox; one search with more tombstones
        // than the bound still forgets the oldest behind a watermark.
        for offset in 2049..=2049 + 1024 {
            p.inspection_identity(&notice(
                offset,
                crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED,
            ));
        }
        assert_eq!(
            p.identity_lifetimes(2)[HAND_RIGHT].forgotten_through,
            epoch + 2049
        );
        assert_eq!(
            p.tracker(HAND_RIGHT).unwrap().agreement[2].observed_checks,
            0
        );
        assert_eq!(p.lifecycle_overflows, 1);
    }

    /// A long engine session ends many correspondences normally, and none of them can be
    /// retracted any more. A knocked sensor whose identity starts afresh in each window must
    /// still be judged moved, without an overflow or a reset of its agreement.
    #[test]
    fn normally_ended_epochs_of_a_long_session_leave_no_lifecycle_state() {
        use super::super::repose;
        for hand in 0..2 {
            let device = if hand == HAND_LEFT {
                DEVICE_LEFT
            } else {
                DEVICE_RIGHT
            };
            let mut p = Pipeline::new(Default::default());
            for i in 0..3 {
                p.sensor(
                    &format!("SENSOR{i}"),
                    synth::camera_model(),
                    Isometry3::identity(),
                );
            }
            p.body(device, synth::touch_leds(), Vector3::zeros());
            let held = CameraAgreement {
                observed_checks: 300,
                samples: 300,
                median_px: 0.4,
                bias_px: Vector2::new(0.3, 0.0),
                ..Default::default()
            };
            p.trackers[hand].as_mut().unwrap().agreement[2] = held;
            let notice = |epoch, state| crate::engine::InspectionIdentity {
                serial: serial(2),
                epoch,
                device_id: device,
                state,
            };
            // Earlier inspections and placements: every break started and ended an epoch.
            for epoch in 1..=1100 {
                p.inspection_identity(&notice(epoch, crate::ffi::OHMD_CV1TOUCH_IDENTITY_STARTED));
                p.inspection_identity(&notice(epoch, crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED));
            }
            let mut monitor = repose::Monitor::new(repose::WINDOW_NS);
            let agreement = |w: u64, observed: (u64, u64)| {
                ["A", "B", "SENSOR2"]
                    .into_iter()
                    .map(|serial| {
                        let c = serial == "SENSOR2";
                        let mut hands = [CameraAgreement::default(); 2];
                        hands[hand] = CameraAgreement {
                            checks: 100 * w,
                            misses: if c { 100 * w } else { 0 },
                            observed_checks: if c { observed.0 } else { 100 * w },
                            observed_misses: if c { observed.1 } else { 0 },
                            samples: if c { 0 } else { 100 * w },
                            median_px: 0.1,
                            bias_px: Vector2::zeros(),
                        };
                        (serial.to_string(), hands)
                    })
                    .collect::<Vec<_>>()
            };
            let mut observed = (0, 0);
            for w in 0..=4 {
                let mut count = |p: &mut Pipeline, epoch, checks, misses| {
                    p.observed_evidence.get_mut(&2).unwrap()[hand]
                        .count(epoch, Checks { checks, misses });
                    observed.0 += checks;
                    observed.1 += misses;
                };
                if w == 3 {
                    // Ordinary matching continues beside the inspection's fresh identity.
                    count(&mut p, 0, 20, 0);
                    p.inspection_identity(&notice(
                        1101,
                        crate::ffi::OHMD_CV1TOUCH_IDENTITY_STARTED,
                    ));
                    count(&mut p, 1101, 50, 50);
                } else if w == 4 {
                    p.inspection_identity(&notice(1101, crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED));
                    p.inspection_identity(&notice(
                        1102,
                        crate::ffi::OHMD_CV1TOUCH_IDENTITY_STARTED,
                    ));
                    count(&mut p, 1102, 30, 30);
                }
                monitor.lifecycle("SENSOR2", &p.identity_lifetimes(2));
                monitor.observed_evidence("SENSOR2", &p.observed_evidence("SENSOR2"));
                let changes = monitor.update(1 + w * repose::WINDOW_NS, &agreement(w, observed));
                assert_eq!(
                    changes,
                    match w {
                        2 => vec![repose::Change::Inspect("SENSOR2".into())],
                        4 => {
                            // ordinary matching's 20 checks count beside both identities'
                            let mut decided = [None; 2];
                            decided[hand] = Some((
                                Checks {
                                    checks: 100,
                                    misses: 80,
                                },
                                2,
                            ));
                            vec![repose::Change::Moved {
                                serial: "SENSOR2".into(),
                                by: repose::MovedBy::Inspection(decided),
                            }]
                        }
                        _ => vec![],
                    },
                    "hand {hand}, window {w}"
                );
                if w == 2 {
                    p.begin_inspection("SENSOR2");
                }
            }
            assert_eq!(p.lifecycle_overflows, 0, "hand {hand}");
            assert_eq!(
                p.tracker(hand).unwrap().agreement[2],
                held,
                "a normal end cannot reset agreement"
            );
            assert_eq!(p.identity_lifetimes(2), <[Evidence; 2]>::default());
        }
    }

    /// The engine acknowledges a restored pose only after delivering every frame it tagged.
    /// The tombstone outlives those still queued then, however many epochs ended normally
    /// around it, and retires without moving the watermark.
    #[test]
    fn a_tombstone_outlives_the_frames_queued_at_restoration() {
        let (_, _, mut p, _) = run_with_inspection_lifecycle(
            3.5,
            None,
            IMU_DELAY,
            3,
            None,
            (
                MATCH_GOOD | MATCH_STRONG | crate::engine::MATCH_INSPECTION_IDENTITY,
                Some(1_000_000_000),
            ),
            Some((State::Invalidated, false, true)),
        );
        let invalid = (1 << 32) | 1;
        for epoch in (invalid - 1500..invalid).chain(invalid + 1..invalid + 1500) {
            for state in [
                crate::ffi::OHMD_CV1TOUCH_IDENTITY_STARTED,
                crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED,
            ] {
                p.inspection_identity(&crate::engine::InspectionIdentity {
                    serial: serial(2),
                    epoch,
                    device_id: DEVICE_RIGHT,
                    state,
                });
            }
        }
        let lifetimes = &p.identity_lifetimes(2)[HAND_RIGHT];
        assert_eq!(
            lifetimes.epochs.keys().copied().collect::<Vec<_>>(),
            [invalid]
        );
        assert_eq!(lifetimes.forgotten_through, 0);
        let host = p.latest(HAND_RIGHT).unwrap().host_ns;
        let frame = |epoch: u64, at: u64| {
            let mut info = FrameInfo {
                serial: serial(2),
                frame_start_host_ns: at,
                exposure_valid: 1,
                exposure_hmd_ts: hmd_us(at),
                n_devices: 1,
                ..Default::default()
            };
            info.devices[0].device_id = DEVICE_RIGHT;
            info.devices[0].match_flags = MATCH_GOOD | crate::engine::MATCH_INSPECTION_IDENTITY;
            info.devices[0].identity_epoch_lo = epoch as u32;
            info.devices[0].identity_epoch_hi = (epoch >> 32) as u32;
            info
        };
        let blobs = [EngineBlob {
            x: 600.0,
            y: 400.0,
            width: 3,
            height: 3,
            area: 9,
            led_id: (DEVICE_RIGHT << 8) | 1,
        }];
        let probing = |p: &Pipeline| {
            p.probing[HAND_RIGHT]
                .iter()
                .map(|q| q.identity_epoch)
                .collect::<Vec<_>>()
        };
        let before = probing(&p);
        assert!(!before.contains(&invalid));
        // A frame delivered before the acknowledgement is still rejected, and one of them
        // queued for tracking before the invalidation has not been consumed yet.
        p.frame(&frame(invalid, host - 200_000_000), &blobs);
        assert_eq!(probing(&p), before);
        let cam = *p.camera("SENSOR2").unwrap();
        let valid = invalid + 2000;
        for epoch in [valid, invalid] {
            p.pending[HAND_RIGHT].push(Pending {
                t_e: u64::MAX,
                exposure_host_ns: host,
                frame_start_host_ns: host,
                camera: 2,
                cam,
                blobs: Vec::new(),
                engine: None,
                identity_epoch: Some(epoch),
            });
        }
        // At the acknowledgement, a frame of each epoch still waits in the daemon's queue. The
        // retracted one goes before the tombstone retires: nothing would reject it after.
        let queued = [
            frame(invalid, host - 150_000_000),
            frame(valid, host - 100_000_000),
        ];
        let kept: Vec<_> = queued.iter().filter(|info| !p.retracts(info)).collect();
        assert_eq!(kept.len(), 1);
        p.retire_identities("SENSOR2");
        assert!(!p.retracts(&queued[0]));
        for info in kept {
            p.frame(info, &blobs);
        }
        assert_eq!(probing(&p), [before, vec![valid]].concat());
        assert_eq!(
            p.pending[HAND_RIGHT]
                .iter()
                .map(|q| q.identity_epoch)
                .collect::<Vec<_>>(),
            [Some(valid)]
        );
        assert_eq!(p.identity_lifetimes(2), <[Evidence; 2]>::default());
        assert!(p.observed_evidence("SENSOR2")[HAND_RIGHT].epochs.is_empty());
    }

    /// Consumers get each new tombstone or watermark once: nothing for starts, normal ends or
    /// a tombstone the pipeline already holds.
    #[test]
    fn lifecycle_changes_are_handed_over_once() {
        let mut p = Pipeline::new(Default::default());
        for i in 0..2 {
            p.sensor(
                &format!("SENSOR{i}"),
                synth::camera_model(),
                Isometry3::identity(),
            );
        }
        let notice = |epoch, state| crate::engine::InspectionIdentity {
            serial: serial(1),
            epoch,
            device_id: DEVICE_LEFT,
            state,
        };
        for epoch in 1..=100 {
            p.inspection_identity(&notice(epoch, crate::ffi::OHMD_CV1TOUCH_IDENTITY_STARTED));
            p.inspection_identity(&notice(epoch, crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED));
        }
        assert!(p.take_lifecycle_changes().is_empty());
        for _ in 0..2 {
            p.inspection_identity(&notice(50, crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED));
        }
        let changes = p.take_lifecycle_changes();
        assert_eq!(changes.len(), 1);
        assert_eq!(changes[0].0, 1);
        assert_eq!(
            changes[0].1[HAND_LEFT]
                .epochs
                .keys()
                .copied()
                .collect::<Vec<_>>(),
            [50]
        );
        assert!(changes[0].1[HAND_RIGHT].epochs.is_empty());
        p.inspection_identity(&notice(50, crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED));
        assert!(p.take_lifecycle_changes().is_empty());
    }

    #[test]
    fn invalidation_after_mailbox_extraction_prevents_movement() {
        use super::super::{epochs, repose};
        use std::sync::{Arc, Barrier, Mutex};
        for hand in 0..2 {
            let mut p = Pipeline::new(Default::default());
            p.sensor("SENSOR2", synth::camera_model(), Isometry3::identity());
            p.begin_inspection("SENSOR2");
            let mut monitor = repose::Monitor::new(repose::WINDOW_NS);
            let agreement = |w: u64, observed| {
                ["A", "B", "SENSOR2"]
                    .into_iter()
                    .map(|serial| {
                        let mut hands = [super::super::tracker::CameraAgreement::default(); 2];
                        hands[hand] = super::super::tracker::CameraAgreement {
                            checks: 100 * w,
                            misses: if serial == "SENSOR2" { 100 * w } else { 0 },
                            observed_checks: if serial == "SENSOR2" {
                                observed
                            } else {
                                100 * w
                            },
                            observed_misses: if serial == "SENSOR2" { observed } else { 0 },
                            samples: if serial == "SENSOR2" { 0 } else { 100 * w },
                            median_px: 0.1,
                            bias_px: Vector2::zeros(),
                        };
                        (serial.to_string(), hands)
                    })
                    .collect::<Vec<_>>()
            };
            for w in 0..=3 {
                if w == 3 {
                    p.observed_evidence.get_mut(&0).unwrap()[hand].count(
                        1,
                        Checks {
                            checks: 50,
                            misses: 50,
                        },
                    );
                }
                monitor.observed_evidence("SENSOR2", &p.observed_evidence("SENSOR2"));
                let changes = monitor.update(
                    1 + w * repose::WINDOW_NS,
                    &agreement(w, if w == 3 { 50 } else { 0 }),
                );
                assert_eq!(
                    changes,
                    if w == 2 {
                        vec![repose::Change::Inspect("SENSOR2".into())]
                    } else {
                        vec![]
                    }
                );
            }
            p.observed_evidence.get_mut(&0).unwrap()[hand].count(
                2,
                Checks {
                    checks: 30,
                    misses: 30,
                },
            );
            let mailbox = Arc::new(Mutex::new(epochs::Mailbox::default()));
            mailbox.lock().unwrap().register(["SENSOR2".into()]);
            // The radio loop extracted its mailbox, then tracking ran while a native callback arrived.
            assert!(mailbox.lock().unwrap().take().is_empty());
            let barrier = Arc::new(Barrier::new(2));
            std::thread::scope(|scope| {
                let (incoming, ready) = (Arc::clone(&mailbox), Arc::clone(&barrier));
                scope.spawn(move || {
                    incoming
                        .lock()
                        .unwrap()
                        .note(&crate::engine::InspectionIdentity {
                            serial: serial(2),
                            epoch: 2,
                            device_id: if hand == HAND_LEFT {
                                DEVICE_LEFT
                            } else {
                                DEVICE_RIGHT
                            },
                            state: crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED,
                        });
                    ready.wait();
                });
                barrier.wait();
                let changes = epochs::commit(&mailbox, &mut p, |pipe| {
                    monitor.observed_evidence("SENSOR2", &pipe.observed_evidence("SENSOR2"));
                    monitor.update(1 + 4 * repose::WINDOW_NS, &agreement(4, 80))
                });
                assert!(
                    changes.is_empty(),
                    "late invalidation must precede verdict: {changes:?}"
                );
                assert!(!monitor.handed_over("SENSOR2"));
                epochs::commit(&mailbox, &mut p, |_| {
                    assert!(
                        mailbox.try_lock().is_err(),
                        "lifecycle callbacks cannot interleave with commitment"
                    );
                });
            });
        }
    }

    enum Ev {
        Hmd(u64, u32),
        Imu(u64, u64, Vector3<f64>, Vector3<f64>),
        Frame(Box<FrameInfo>, Vec<EngineBlob>),
        Search(Box<FrameInfo>),
        Identity(crate::engine::InspectionIdentity),
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
        run_with_inspection(
            seconds,
            gap,
            imu_delay,
            camera_count,
            recording,
            (match_flags, None),
        )
    }

    fn run_with_inspection(
        seconds: f64,
        gap: Option<(f64, f64)>,
        imu_delay: u64,
        camera_count: usize,
        recording: Option<&crate::recorder::Recorder>,
        frame_mode: (u32, Option<u64>),
    ) -> Simulation {
        run_with_inspection_lifecycle(
            seconds,
            gap,
            imu_delay,
            camera_count,
            recording,
            frame_mode,
            None,
        )
    }

    fn run_with_inspection_lifecycle(
        seconds: f64,
        gap: Option<(f64, f64)>,
        imu_delay: u64,
        camera_count: usize,
        recording: Option<&crate::recorder::Recorder>,
        frame_mode: (u32, Option<u64>),
        lifecycle: Option<(State, bool, bool)>,
    ) -> Simulation {
        let (match_flags, inspection_delay) = frame_mode;
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
            let calibrated = if inspection_delay.is_some() && i == 2 {
                let mut moved = nalgebra::Translation3::new(0.4, 0.0, 0.0) * w;
                moved.rotation *= nalgebra::UnitQuaternion::from_axis_angle(
                    &Vector3::y_axis(),
                    20f64.to_radians(),
                );
                moved
            } else {
                *w
            };
            p.sensor(&format!("SENSOR{i}"), model, calibrated);
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
                if let Some(delay) = inspection_delay.filter(|_| c == 2) {
                    // A single outstanding cold search, after two peers establish the track.
                    if k == 104 {
                        assert!(
                            blobs.len() >= 5,
                            "a successful cold search needs labels: {}",
                            blobs.len()
                        );
                        info.devices[0].identity_epoch_lo = 1;
                        info.devices[0].identity_epoch_hi = 1;
                        if let Some((state, _, _)) = lifecycle {
                            evs.push((
                                e + 25_000_000 + delay - 1,
                                Ev::Identity(crate::engine::InspectionIdentity {
                                    serial: info.serial,
                                    epoch: (1 << 32) | 1,
                                    device_id: DEVICE_RIGHT,
                                    state: match state {
                                        State::Started => {
                                            crate::ffi::OHMD_CV1TOUCH_IDENTITY_STARTED
                                        }
                                        State::Ended => crate::ffi::OHMD_CV1TOUCH_IDENTITY_ENDED,
                                        State::Invalidated => {
                                            crate::ffi::OHMD_CV1TOUCH_IDENTITY_INVALIDATED
                                        }
                                    },
                                }),
                            ));
                        }
                        evs.push((e + 25_000_000, Ev::Search(Box::new(info))));
                        evs.push((e + 25_000_000 + delay, Ev::Frame(Box::new(info), blobs)));
                    }
                } else {
                    evs.push((
                        e + 25_000_000 + c as u64 * 1_000_000,
                        Ev::Frame(Box::new(info), blobs),
                    ));
                }
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
                Ev::Identity(event) => {
                    if let Some(rec) = recording {
                        rec.write(recorder::INSPECTION_IDENTITY, at, &[as_bytes(&event)]);
                    }
                    p.inspection_identity(&event);
                    if lifecycle.is_some_and(|(_, cancel, _)| cancel) {
                        p.cancel_inspection(&cstr_field(&event.serial));
                    }
                }
                Ev::Search(info) => {
                    p.begin_inspection(&cstr_field(&info.serial));
                    p.hold_inspection(&info, 30_000_000_000);
                    if lifecycle.is_some_and(|(_, _, probe)| probe) {
                        p.probe(&cstr_field(&info.serial), true);
                    }
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

    /// Cameras 0 and 1 see the right Touch from frame 80 on, the engine rating its poses only
    /// good: camera 0 acquires it alone, provisionally, and camera 1 confirms the track from
    /// the next frame. Probed camera 2 announces a cold search for frame `search`, if any, and
    /// delivers labels for frame `labelled` `delay_ns` after its capture. Radio reports sent
    /// during `stall` (host time from the start) arrive at its end. Returns the pipeline, and
    /// whether the track was still provisional when the search was announced.
    fn run_probed_search(
        search: Option<u32>,
        labelled: u32,
        delay_ns: u64,
        stall: Option<(u64, u64)>,
    ) -> (Pipeline, bool) {
        const FIRST: u32 = 80;
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
        let mut p = Pipeline::new(Options {
            exposure_offset_ns: IMU_DELAY as i64 - HMD_DELAY as i64,
            ..Options::default()
        });
        for (i, w) in cams.iter().enumerate() {
            p.sensor(&format!("SENSOR{i}"), model, *w);
        }
        p.body(DEVICE_RIGHT, leds.clone(), imu_pos);
        p.probe("SENSOR2", true);
        let exposure = |k: u32| H0 + 10_000_000 + (f64::from(k) * 1e9 / 52.0) as u64;
        let end = exposure(labelled) + delay_ns + 500_000_000;
        let mut evs: Vec<(u64, Ev)> = Vec::new();
        let mut h = H0;
        while h < end {
            evs.push((
                h + HMD_DELAY + rng.range(0.0, 1e6) as u64,
                Ev::Hmd(h + HMD_DELAY, hmd_us(h)),
            ));
            h += 1_000_000;
        }
        let mut h = H0 + 2_000_000;
        while h < end {
            let t = (h - H0) as f64 * 1e-9;
            let mut recv = h + IMU_DELAY + rng.range(0.0, 1e6) as u64;
            if let Some((from, to)) = stall {
                if (H0 + from..H0 + to).contains(&recv) {
                    recv = H0 + to;
                }
            }
            let accel = motion.accel(t, &imu_pos, &Vector3::new(0.0, 9.80665, 0.0));
            evs.push((recv, Ev::Imu(h - C0, recv, motion.gyro(t), accel)));
            h += 2_000_000;
        }
        let mut k = FIRST;
        while exposure(k) + 25_000_000 < end {
            let e = exposure(k);
            let pose = motion.pose((e - H0) as f64 * 1e-9);
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
                            led_id: (DEVICE_RIGHT << 8) | i as i32,
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
                info.devices[0].device_id = DEVICE_RIGHT;
                info.devices[0].match_flags = MATCH_GOOD;
                info.devices[0].pose_valid = 1;
                info.devices[0].cam_orient = o;
                info.devices[0].cam_pos = q;
                match c {
                    0 => evs.push((e + 25_000_000, Ev::Frame(Box::new(info), blobs))),
                    1 if k > FIRST => evs.push((e + 26_000_000, Ev::Frame(Box::new(info), blobs))),
                    2 => {
                        if search == Some(k) {
                            // after camera 0's frame of this exposure, before camera 1's
                            evs.push((e + 25_500_000, Ev::Search(Box::new(info))));
                        }
                        if k == labelled {
                            assert!(blobs.len() >= 5, "the search found the controller");
                            info.devices[0].match_flags |= crate::engine::MATCH_INSPECTION_IDENTITY;
                            info.devices[0].identity_epoch_lo = 1;
                            evs.push((e + 25_000_000 + delay_ns, Ev::Frame(Box::new(info), blobs)));
                        }
                    }
                    _ => {}
                }
            }
            k += 1;
        }
        evs.sort_by_key(|e| e.0);
        let mut provisional = false;
        for (_, ev) in evs {
            match ev {
                Ev::Hmd(host, us) => p.hmd_clock_sample(host, us),
                Ev::Imu(dev, recv, gyro, accel) => {
                    p.imu(HAND_RIGHT, dev, recv, gyro, accel);
                }
                Ev::Frame(info, blobs) => p.frame(&info, &blobs),
                Ev::Search(info) => {
                    let tr = p.tracker(HAND_RIGHT).unwrap();
                    provisional = tr.stats.good_acquisitions == 1 && !tr.tracking();
                    p.hold_inspection(&info, 30_000_000_000);
                }
                Ev::Identity(_) => unreachable!(),
            }
        }
        (p, provisional)
    }

    /// A held prior stands in for the history only for a track its peers confirmed, as for
    /// held evidence: a search announced while one sensor alone had the controller supplies no
    /// placement probe, even once a second sensor has confirmed that track.
    #[test]
    fn a_prior_held_while_provisional_supplies_no_probe() {
        for (search, was_provisional, probes) in [(80, true, 0), (90, false, 1)] {
            let (p, provisional) = run_probed_search(Some(search), search, 1_000_000_000, None);
            assert_eq!(provisional, was_provisional, "search at frame {search}");
            let tr = p.tracker(HAND_RIGHT).unwrap();
            assert!(tr.tracking() && tr.stats.unconfirmed_acquisitions == 0);
            assert_eq!(
                p.probes.iter().filter(|q| q.camera == 2).count(),
                probes,
                "search at frame {search}"
            );
        }
    }

    /// Labels whose exposure the controller's IMU has not reached yet wait for it: a radio stall
    /// does not lose them.
    #[test]
    fn probed_labels_ahead_of_the_imu_wait_for_it() {
        // frame 90 is captured 1.741 s in; its labels arrive in the stall's middle
        let stall = Some((1_720_000_000, 1_800_000_000));
        let (p, _) = run_probed_search(None, 90, 2_000_000, stall);
        let probes: Vec<_> = p.probes.iter().filter(|q| q.camera == 2).collect();
        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].identity_epoch, 1);
    }

    #[test]
    fn outstanding_searches_keep_exposure_priors_for_multi_second_results() {
        // Native identity survives a disputed camera rotation. The late labels may judge it,
        // but cannot move the live filter through that calibration.
        for delay in [1, 5, 20] {
            let (_, _, mut p, _) = run_with_inspection(
                delay as f64 + 3.0,
                None,
                IMU_DELAY,
                3,
                None,
                (
                    MATCH_GOOD | MATCH_STRONG | crate::engine::MATCH_INSPECTION_IDENTITY,
                    Some(delay * 1_000_000_000),
                ),
            );
            let tr = p.tracker(HAND_RIGHT).unwrap();
            let a = tr.agreement[2];
            assert_eq!(a.observed_checks, 1, "{delay}s: {a:?}");
            assert_eq!(a.observed_misses, 1, "{delay}s: {a:?}");
            assert_eq!(
                tr.stats.too_old, 1,
                "old labels judge the sensor without fusion"
            );
            assert!(p.inspection_starts[HAND_RIGHT].is_empty());
            p.cancel_inspection("SENSOR2");
            let host = p.latest(HAND_RIGHT).unwrap().host_ns + 20_000_000;
            let stale_start = FrameInfo {
                serial: serial(2),
                exposure_valid: 1,
                exposure_hmd_ts: hmd_us(host),
                ..Default::default()
            };
            p.hold_inspection(&stale_start, 30_000_000_000);
            assert!(
                p.inspection_starts[HAND_RIGHT].is_empty(),
                "cancelled callbacks cannot retain again"
            );
        }
    }

    /// Consume the actual native overlap/revocation outputs, rather than inventing a window
    /// of counters. The one ambiguous exposure cannot demote a sensor; the following rejected
    /// outputs supply no labelled evidence, and the sensor never enters probe collection.
    #[cfg(all(target_os = "linux", feature = "engine"))]
    #[test]
    fn a_crossing_revoked_by_native_identity_does_not_start_placement() {
        use crate::track::repose::{Change, Monitor, WINDOW_NS};
        let output = std::process::Command::new(
            std::path::Path::new(env!("CV1TOUCH_NATIVE_DIR")).join("tests/test_inspection_handoff"),
        )
        .arg("migration-trace")
        .output()
        .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let rows: Vec<Vec<u32>> = String::from_utf8(output.stdout)
            .unwrap()
            .lines()
            .filter_map(|line| line.strip_prefix("crossing "))
            .map(|line| {
                line.split_whitespace()
                    .map(|v| v.parse().unwrap())
                    .collect()
            })
            .filter(|row: &Vec<u32>| row[0] == DEVICE_RIGHT as u32)
            .collect();
        assert_eq!(rows.len(), 4);
        assert_eq!(
            rows.iter().map(|row| row[1]).collect::<Vec<_>>(),
            [1, 0, 0, 0]
        );
        let (_, _, mut p, _) =
            run_with_sensors(3.0, None, IMU_DELAY, 3, None, MATCH_GOOD | MATCH_STRONG);
        let before = p.tracker(HAND_RIGHT).unwrap().agreement[2];
        let mut monitor = Monitor::new(WINDOW_NS);
        assert!(monitor.update(1, &p.camera_agreement_by_hand()).is_empty());
        p.begin_inspection("SENSOR2");
        let start = p.tracker(HAND_RIGHT).unwrap().newest_imu().unwrap();
        for (k, row) in rows.iter().enumerate() {
            let at = start + (k as u64 + 1) * 20_000_000;
            let host = at + C0;
            p.hmd_clock_sample(host + HMD_DELAY, hmd_us(host));
            p.imu(
                HAND_RIGHT,
                at,
                host + IMU_DELAY,
                Vector3::zeros(),
                Vector3::new(0.0, 9.80665, 0.0),
            );
            // Fresh independent peers make every suspect exposure judgeable.
            for c in 0..3 {
                let tr = p.tracker(HAND_RIGHT).unwrap();
                let camera = p.cams[&format!("SENSOR{c}")].camera;
                let pose = tr.body.world_from_model(&tr.filter_at(at).unwrap().x);
                let camera_pose = camera.world_from_cam.inverse() * pose;
                let blobs: Vec<EngineBlob> = tr
                    .body
                    .leds
                    .iter()
                    .enumerate()
                    .filter_map(|(i, led)| {
                        let point = camera_pose * Point3::from(led.pos);
                        if (camera_pose.rotation * led.dir).dot(&(-point.coords))
                            / point.coords.norm()
                            < 0.3
                        {
                            return None;
                        }
                        let pixel = camera.model.project(&point.coords)?;
                        Some(EngineBlob {
                            x: (pixel.x + if c == 2 { 5.0 } else { 0.0 }) as f32,
                            y: pixel.y as f32,
                            width: 3,
                            height: 3,
                            area: 9,
                            led_id: if c != 2 || row[3] > 0 {
                                (DEVICE_RIGHT << 8) | i as i32
                            } else {
                                -1
                            },
                        })
                    })
                    .collect();
                assert!(blobs.len() >= 5);
                let mut info = FrameInfo {
                    serial: serial(c),
                    frame_start_host_ns: host,
                    exposure_valid: 1,
                    exposure_hmd_ts: hmd_us(host),
                    n_devices: 1,
                    ..Default::default()
                };
                let (orient, pos) = crate::calib::iso_to_xyzw(&camera_pose);
                info.devices[0].device_id = DEVICE_RIGHT;
                info.devices[0].pose_valid = if c == 2 { row[1] } else { 1 };
                info.devices[0].match_flags = if c == 2 {
                    row[2]
                } else {
                    MATCH_GOOD | MATCH_STRONG
                };
                info.devices[0].cam_orient = orient;
                info.devices[0].cam_pos = pos;
                p.frame(&info, &blobs);
                // Finish the exposure even if clock-fit rounding placed it just beyond the last
                // IMU sample, as normal radio delivery does after the camera callback.
                p.imu(
                    HAND_RIGHT,
                    at + 2_000_000,
                    host + IMU_DELAY + 2_000_000,
                    Vector3::zeros(),
                    Vector3::new(0.0, 9.80665, 0.0),
                );
            }
        }
        let after = p.tracker(HAND_RIGHT).unwrap().agreement[2];
        assert!(p.pending[HAND_RIGHT].is_empty());
        assert_eq!(
            after.observed_checks - before.observed_checks,
            1,
            "before {before:?}; after {after:?}"
        );
        for window in 1..=2 {
            let changes = monitor.update(1 + window * WINDOW_NS, &p.camera_agreement_by_hand());
            assert!(!changes
                .iter()
                .any(|change| matches!(change, Change::Moved { .. })));
        }
        assert!(!monitor.handed_over("SENSOR2"));
        assert!(!p.is_probed("SENSOR2"));
        assert!(p.probes.is_empty());
        assert!(p.probing.iter().all(Vec::is_empty));
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

    /// A recording outlives an engine reopen, and the new engine numbers its identities afresh.
    /// As the daemon's new pipeline does, replay forgets the old engine's tombstones at the
    /// reopen; it also retires a sensor's at its recorded acknowledgement. Without either, the
    /// old tombstone rejects a later frame of the same epoch.
    #[test]
    fn replay_follows_the_engine_sessions_identity_lifetimes() {
        use crate::recorder::{self, Reader, Recorder};
        let epoch = (1u64 << 32) | 1;
        for (marker, probes) in [
            (Some(recorder::ENGINE_SESSION), 1),
            (Some(recorder::IDENTITIES_RETIRED), 1),
            (None, 0),
        ] {
            let path = |name: &str| {
                std::env::temp_dir().join(format!(
                    "cv1t-replay-sessions-{name}-{marker:?}-{}.rec",
                    std::process::id()
                ))
            };
            let (first, second) = (path("first"), path("second"));
            let recording = Recorder::new(8 << 20);
            recording.start(&first, H0).unwrap();
            // Camera 2's labels of `epoch` are invalidated before they arrive.
            let (_, _, p, calibration) = run_with_inspection_lifecycle(
                3.5,
                None,
                IMU_DELAY,
                3,
                Some(&recording),
                (
                    MATCH_GOOD | MATCH_STRONG | crate::engine::MATCH_INSPECTION_IDENTITY,
                    Some(0),
                ),
                Some((State::Invalidated, false, false)),
            );
            assert_eq!(recording.stop().dropped, 0);
            // After the invalidation, the marker; half a second later, a frame of camera 2
            // carries the same epoch.
            let rewritten = Recorder::new(8 << 20);
            rewritten.start(&second, H0).unwrap();
            let mut retag_after = None;
            for r in Reader::open(&first).unwrap() {
                if r.rtype == recorder::FRAME_OBS && retag_after.is_some_and(|t| r.host_ns >= t) {
                    let (mut info, blobs) = recorder::decode_frame_obs(&r.payload).unwrap();
                    if cstr_field(&info.serial) == "SENSOR2" {
                        info.devices[0].identity_epoch_lo = epoch as u32;
                        info.devices[0].identity_epoch_hi = (epoch >> 32) as u32;
                        retag_after = None;
                        rewritten.write(
                            r.rtype,
                            r.host_ns,
                            &[recorder::as_bytes(&info), recorder::slice_bytes(&blobs)],
                        );
                        continue;
                    }
                }
                rewritten.write(r.rtype, r.host_ns, &[&r.payload]);
                if r.rtype == recorder::INSPECTION_IDENTITY {
                    match marker {
                        Some(recorder::IDENTITIES_RETIRED) => {
                            rewritten.write(recorder::IDENTITIES_RETIRED, r.host_ns, &[b"SENSOR2"])
                        }
                        Some(marker) => rewritten.write(marker, r.host_ns, &[]),
                        None => {}
                    }
                    retag_after = Some(r.host_ns + 500_000_000);
                }
            }
            assert_eq!(rewritten.stop().dropped, 0);
            let opts = Options {
                probe_sensors: vec!["SENSOR2".into()],
                ..p.opts.clone()
            };
            let replay = super::super::replay::replay(
                &second,
                &calibration,
                &|_| Some(Vector3::new(0.015, -0.012, 0.008)),
                &opts,
            )
            .unwrap();
            assert_eq!(
                replay
                    .probes
                    .iter()
                    .filter(|q| q.identity_epoch == epoch)
                    .count(),
                probes,
                "{marker:?}"
            );
            std::fs::remove_file(first).unwrap();
            std::fs::remove_file(second).unwrap();
        }
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
        p.probing[0].push(PendingProbe {
            t_e: 1,
            camera: 1,
            blobs: Vec::new(),
            identity_epoch: 0,
        });
        p.probing[0].push(PendingProbe {
            t_e: 2,
            camera: 0,
            blobs: Vec::new(),
            identity_epoch: 0,
        });
        assert!(p.probe("A", false));
        assert_eq!(
            p.probing[0].iter().map(|p| p.camera).collect::<Vec<_>>(),
            [1]
        );
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
    fn hotplug_removed_sensor_cannot_leave_probes_or_reuse_another_cameras_id() {
        let mut p = Pipeline::new(Options::default());
        for serial in ["SENSORALPHA", "SENSORBETA"] {
            p.sensor(serial, synth::camera_model(), Isometry3::identity());
        }
        p.body(DEVICE_RIGHT, synth::touch_leds(), Vector3::zeros());
        p.trackers[HAND_RIGHT].as_mut().unwrap().agreement[1].samples = 7;
        for (t_e, camera) in [(1, 0), (2, 1)] {
            p.probing[0].push(PendingProbe {
                t_e,
                camera,
                blobs: Vec::new(),
                identity_epoch: 0,
            });
        }
        p.probe("SENSORALPHA", true);
        let removed_camera = *p.camera("SENSORALPHA").unwrap();
        p.pending[0].push(Pending {
            camera: 0,
            cam: removed_camera,
            t_e: 1,
            exposure_host_ns: 1,
            frame_start_host_ns: 1,
            blobs: Vec::new(),
            engine: None,
            identity_epoch: None,
        });
        p.events.push(FrameEvent {
            camera: 0,
            hand: 0,
            t_e: 1,
            exposure_host_ns: 1,
            frame_start_host_ns: 1,
            outcome: Outcome::Waiting,
            engine: None,
            tracked: None,
        });
        p.probes.push(Probe {
            camera: 0,
            hand: 0,
            device_id: DEVICE_RIGHT,
            identity_epoch: 0,
            world_from_model: Isometry3::identity(),
            pos_sd: 0.0,
            blobs: Vec::new(),
            labels: Vec::new(),
        });
        assert!(p.remove_sensor("SENSORALPHA").is_some());
        assert!(p.pending[0].is_empty());
        assert!(p.events.is_empty());
        assert!(p.probes.is_empty());
        assert!(p.camera("SENSORALPHA").is_none());
        assert!(!p.is_probed("SENSORALPHA"));
        assert_eq!(p.serial_of(0), None);
        assert_eq!(
            p.probing[0].iter().map(|p| p.camera).collect::<Vec<_>>(),
            [1]
        );
        assert_eq!(p.camera_agreement().len(), 1);
        p.sensor("SENSORGAMMA", synth::camera_model(), Isometry3::identity());
        assert_eq!(p.camera_id("SENSORBETA"), Some(1));
        assert_eq!(p.camera_id("SENSORGAMMA"), Some(2));
        assert_eq!(
            p.trackers[HAND_RIGHT].as_ref().unwrap().agreement[1].samples,
            7
        );
        let mut early = Pipeline::new(Options::default());
        early.sensor("SENSORALPHA", synth::camera_model(), Isometry3::identity());
        early.sensor("SENSORBETA", synth::camera_model(), Isometry3::identity());
        assert!(early.remove_sensor("SENSORALPHA").is_some());
        early.body(DEVICE_RIGHT, synth::touch_leds(), Vector3::zeros());
        assert_eq!(
            early.trackers[HAND_RIGHT].as_ref().unwrap().agreement.len(),
            2
        );
    }

    /// A removed sensor's inspection, its held search exposures and its identity lifetimes go
    /// with it, including lifecycle changes no consumer has taken: none can be handed over for
    /// a camera that no longer has a serial. The other sensor keeps all of its own.
    #[test]
    fn hotplug_a_removed_sensor_leaves_no_inspection_or_identity_state() {
        let mut p = Pipeline::new(Options::default());
        for serial in ["SENSORALPHA", "SENSORBETA"] {
            p.sensor(serial, synth::camera_model(), Isometry3::identity());
            p.begin_inspection(serial);
            let mut invalidated: [Evidence; 2] = Default::default();
            invalidated[HAND_RIGHT].note(3, State::Invalidated);
            p.inspection_lifecycle(serial, &invalidated);
        }
        for (hand, camera) in [(HAND_LEFT, 0), (HAND_RIGHT, 0), (HAND_RIGHT, 1)] {
            p.inspection_starts[hand].push(InspectionExposure {
                camera,
                frame_start_host_ns: 1,
                t_e: 1,
                until: 2,
                held: false,
            });
        }
        assert!(p.remove_sensor("SENSORALPHA").is_some());
        let beta = [1];
        assert_eq!(p.inspecting.iter().copied().collect::<Vec<_>>(), beta);
        assert_eq!(
            p.identity_evidence.keys().copied().collect::<Vec<_>>(),
            beta
        );
        assert_eq!(
            p.observed_evidence.keys().copied().collect::<Vec<_>>(),
            beta
        );
        assert_eq!(
            p.identity_lifetimes.keys().copied().collect::<Vec<_>>(),
            beta
        );
        assert_eq!(
            p.take_lifecycle_changes()
                .into_iter()
                .map(|(camera, _)| camera)
                .collect::<Vec<_>>(),
            beta
        );
        assert!(p.inspection_starts[HAND_LEFT].is_empty());
        assert_eq!(
            p.inspection_starts[HAND_RIGHT]
                .iter()
                .map(|s| s.camera)
                .collect::<Vec<_>>(),
            beta
        );
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

    /// An inspection's outcome depends on what the engine reported about each hand's identity
    /// meanwhile: a good match whose pose it withheld was seen, motion or identity made it
    /// judgeable, and ordinary tracking from before the inspection reached the engine counts
    /// for nothing. Each inspection starts afresh.
    #[test]
    fn an_inspection_gathers_what_the_engine_could_tell_of_each_hand() {
        use crate::engine::{MATCH_INSPECTION_IDENTITY, MATCH_INSPECTION_MOTION};
        let mut p = Pipeline::new(Options::default());
        p.sensor("SENSOR0", synth::camera_model(), Isometry3::identity());
        let frame = |devices: &[(i32, u32, u32)]| {
            let mut info = FrameInfo {
                serial: serial(0),
                n_devices: devices.len() as u32,
                ..Default::default()
            };
            for (d, &(device_id, match_flags, pose_valid)) in info.devices.iter_mut().zip(devices) {
                d.device_id = device_id;
                d.match_flags = match_flags;
                d.pose_valid = pose_valid;
            }
            info
        };
        let none = [IdentityEvidence::default(); 2];
        let still = IdentityEvidence {
            seen: true,
            judgeable: false,
        };
        // Not inspected: nothing is gathered.
        p.frame(&frame(&[(DEVICE_RIGHT, MATCH_GOOD, 0)]), &[]);
        assert_eq!(p.identity_evidence("SENSOR0"), none);
        p.begin_inspection("SENSOR0");
        // Ordinary tracking, exported before the inspection applied: not a sighting of it.
        p.frame(&frame(&[(DEVICE_RIGHT, MATCH_GOOD | MATCH_STRONG, 1)]), &[]);
        assert_eq!(p.identity_evidence("SENSOR0"), none);
        // A withheld match: the right hand was seen, too still to identify.
        p.frame(
            &frame(&[(DEVICE_RIGHT, MATCH_GOOD, 0), (DEVICE_LEFT, 0, 0)]),
            &[],
        );
        assert_eq!(p.identity_evidence("SENSOR0")[HAND_RIGHT], still);
        assert_eq!(
            p.identity_evidence("SENSOR0")[HAND_LEFT],
            IdentityEvidence::default()
        );
        assert!(IdentityEvidence::starved(
            &p.identity_evidence("SENSOR0"),
            [false, true]
        ));
        // The left hand was neither seen nor judgeable: no verdict either way for it alone.
        assert!(!IdentityEvidence::starved(
            &p.identity_evidence("SENSOR0"),
            [true, false]
        ));
        p.frame(
            &frame(&[(DEVICE_LEFT, MATCH_GOOD | MATCH_INSPECTION_MOTION, 0)]),
            &[],
        );
        assert!(p.identity_evidence("SENSOR0")[HAND_LEFT].judgeable);
        assert!(!IdentityEvidence::starved(
            &p.identity_evidence("SENSOR0"),
            [true, true]
        ));
        p.cancel_inspection("SENSOR0");
        assert_eq!(p.identity_evidence("SENSOR0"), none);
        p.begin_inspection("SENSOR0");
        assert_eq!(p.identity_evidence("SENSOR0"), none);
        p.frame(
            &frame(&[(DEVICE_RIGHT, MATCH_GOOD | MATCH_INSPECTION_IDENTITY, 1)]),
            &[],
        );
        assert_eq!(
            p.identity_evidence("SENSOR0")[HAND_RIGHT],
            IdentityEvidence {
                seen: true,
                judgeable: true
            }
        );
        // A probation keeps its own, from when the sensor is probed until it is let go.
        p.cancel_inspection("SENSOR0");
        p.probe("SENSOR0", true);
        p.frame(&frame(&[(DEVICE_LEFT, MATCH_GOOD, 0)]), &[]);
        assert_eq!(p.identity_evidence("SENSOR0")[HAND_LEFT], still);
        p.probe("SENSOR0", false);
        assert_eq!(p.identity_evidence("SENSOR0"), none);
    }
}
