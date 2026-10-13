// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! One controller's tracker. The filter keeps a short history of states so that a sensor frame,
//! which arrives tens of milliseconds after its exposure, is applied at the exposure time and the
//! IMU samples since are replayed. Acquisition takes the engine's pose and labels; tracking then
//! associates blobs itself. Output is the newest IMU-propagated state, unsmoothed.

use std::collections::VecDeque;

use nalgebra::{Isometry3, UnitQuaternion, Vector2, Vector3};

use super::eskf::{Cov, Filter, Noise, Nominal, BA, BG, POS, ROT, VEL};
use super::search::{self, SearchParams};
use super::vision::{self, Blob, Body, Camera, FitParams};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    /// Controller clock, ns.
    pub t: u64,
    /// Body rate, rad/s, and specific force, m/s^2, factory-calibrated.
    pub gyro: Vector3<f64>,
    pub accel: Vector3<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Params {
    pub noise: Noise,
    pub fit: FitParams,
    /// m/s^2, along world -Y.
    pub gravity: f64,
    pub history_ns: u64,
    /// Without an accepted update for this long, the position is lost.
    pub lost_after_ns: u64,
    /// Fewest engine-labelled LEDs to acquire from.
    pub acquire_min_leds: usize,
    /// Standard deviations at acquisition (rad, m, m/s, rad/s, m/s^2); biases carried over from
    /// an earlier track keep their own.
    pub acquire_sd: [f64; 5],
    /// For this long after the last fix the propagated orientation is trusted to vet the engine's
    /// pose at re-acquisition, which may turn from it by at most `acquire_max_turn_rad`.
    pub orientation_memory_ns: u64,
    pub acquire_max_turn_rad: f64,
    /// IMU samples this far apart (normally 2 ms) are a discontinuity: the controller slept or
    /// dropped out, may have moved meanwhile, and its clock may have jumped. The state is dropped
    /// (the bias estimates kept) and the controller acquired afresh.
    pub max_imu_gap_ns: u64,
    /// After refusing the engine's poses for this long, the orientation memory is dropped: it is
    /// the one that is wrong.
    pub max_refusing_ns: u64,
    /// Blobs this close to an LED of the other controller (as it is tracked at the same exposure)
    /// are left to it, unless the engine labelled them as ours.
    pub foreign_px: f64,
    /// For this long after the last fix the dead-reckoned state is close enough for the
    /// tracker's own (shift-search) association to re-lock without the engine.
    pub relock_ns: u64,
    /// A wrong lock (a pose that fits the LEDs but is turned from the truth) leaks gravity into the
    /// filter, which drives the accelerometer bias off: beyond this from a remembered bias, or
    /// beyond `max_accel_bias` without one (m/s^2), the track is dropped.
    pub max_accel_bias_change: f64,
    pub max_accel_bias: f64,
    /// After a wrong lock, engine poses within `wrong_lock_turn_rad` of the dropped orientation
    /// (carried on by the gyro) are refused for this long: the engine tends to offer it again.
    pub wrong_lock_memory_ns: u64,
    pub wrong_lock_turn_rad: f64,
    /// At an acquisition with no orientation to check the engine's pose against, the pose is
    /// tested against its ring aliases (`vision::Alias`) on the frame. Each hypothesis costs its
    /// matched LEDs' squared residuals in units of `alias_sigma_px`, plus `alias_miss_cost` per
    /// LED it predicts facing the sensor with no blob near, and per blob in its footprint it
    /// leaves unexplained. The cheapest is taken when it leads by `alias_margin`; else the
    /// acquisition waits for a clearer frame (for up to `max_refusing_ns`).
    pub alias_sigma_px: f64,
    pub alias_miss_cost: f64,
    pub alias_margin: f64,
    /// While the accelerometer holds steady (over `steady_window_ns`, every reading within
    /// `steady_accel` of the mean, and the mean within `steady_accel` of g) it gives gravity's
    /// direction in the controller to a few degrees (its bias, uncorrected, adds up to ~3): an
    /// acquisition whose pose tilts further from it than `still_max_tilt_rad` is refused.
    pub still_max_tilt_rad: f64,
    pub steady_window_ns: u64,
    pub steady_accel: f64,
    /// The controller is still when, over the last `still_window_ns` of IMU, the gyro (less its
    /// bias) stays within `still_gyro` rad/s of zero and its samples within `still_gyro_spread`
    /// of their mean, and the accelerometer (less its bias) within `still_accel_spread` m/s^2 of
    /// its mean, a mean `still_accel_g` from g. Then every `zupt_every_ns` the velocity is
    /// measured as zero with `zupt_sigma` m/s: with too few LEDs to pin the depth, an unmodelled
    /// hundredth of a m/s^2 otherwise builds into a drift of centimetres along the line of sight.
    pub still_window_ns: u64,
    pub still_gyro: f64,
    pub still_gyro_spread: f64,
    pub still_accel_spread: f64,
    pub still_accel_g: f64,
    pub zupt_every_ns: u64,
    pub zupt_sigma: f64,
    /// The search for a lost controller's position with its kept orientation.
    pub search: SearchParams,
}

impl Default for Params {
    fn default() -> Self {
        Params {
            noise: Noise::default(),
            fit: FitParams::default(),
            gravity: 9.80665,
            history_ns: 400_000_000,
            lost_after_ns: 500_000_000,
            acquire_min_leds: 5,
            acquire_sd: [0.05, 0.03, 0.5, 0.02, 0.5],
            orientation_memory_ns: 30_000_000_000,
            acquire_max_turn_rad: 10f64.to_radians(),
            max_imu_gap_ns: 100_000_000,
            max_refusing_ns: 500_000_000,
            foreign_px: 2.0,
            relock_ns: 3_000_000_000,
            max_accel_bias_change: 0.5,
            max_accel_bias: 1.0,
            wrong_lock_memory_ns: 2_000_000_000,
            wrong_lock_turn_rad: 10f64.to_radians(),
            // true poses fit at ~0.1 px, ring aliases at 0.3-1 px (2026-10-08 recordings: the
            // true pose led every real alias by 23 or more)
            alias_sigma_px: 0.25,
            alias_miss_cost: 9.0,
            alias_margin: 10.0,
            // wrong poses taken while still were 9 and 15 degrees off it, the true ones 1 and 3
            still_max_tilt_rad: 6f64.to_radians(),
            steady_window_ns: 100_000_000,
            steady_accel: 0.5,
            still_window_ns: 200_000_000,
            still_gyro: 0.03,
            still_gyro_spread: 0.02,
            still_accel_spread: 0.2,
            still_accel_g: 0.15,
            zupt_every_ns: 20_000_000,
            zupt_sigma: 0.01,
            search: SearchParams::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Stats {
    pub frames: u64,
    pub updates: u64,
    pub leds_used: u64,
    /// Frames in which the state predicts no LED facing the sensor.
    pub not_visible: u64,
    /// Frames with LEDs expected but too few matches, or a failed fit, while tracking.
    pub too_few: u64,
    pub rejected: u64,
    pub acquisitions: u64,
    pub failed_acquisitions: u64,
    /// Engine poses refused at acquisition: their orientation disagrees with the gyro's.
    pub refused_acquisitions: u64,
    /// Frames applied behind a newer measurement (the later ones re-applied).
    pub out_of_order: u64,
    /// Later measurements that no longer fit after an out-of-order one, and were dropped.
    pub dropped_on_rebuild: u64,
    /// Frames older than the history.
    pub too_old: u64,
    /// IMU discontinuities (sleep, dropout) that reset the state.
    pub imu_gaps: u64,
    /// Re-locks after the position was lost, by the tracker's own association.
    pub relocks: u64,
    /// Tracks dropped for a runaway accelerometer bias.
    pub wrong_locks: u64,
    /// Acquisitions that took the engine's pose turned by a ring alias (see `vision::Alias`),
    /// which fitted the frame clearly better.
    pub alias_turns: u64,
    /// Acquisitions put off because the engine's pose and an alias fitted the frame about as well.
    pub ambiguous_acquisitions: u64,
    /// Acquisitions from an engine pose rated good but not strong, and from the tracker's own
    /// search with the kept orientation (see `search`; the engine offering no pose it can take),
    /// and those of either dropped because neither a second sensor nor a strong engine pose
    /// confirmed them in time (never published).
    pub good_acquisitions: u64,
    pub searched_acquisitions: u64,
    pub unconfirmed_acquisitions: u64,
    /// Frames searched with a kept orientation (see `SearchParams::every_ns`).
    pub searches: u64,
}

/// The engine's pose for a controller in one frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnginePose {
    pub world_from_model: Isometry3<f64>,
    /// The engine rated it strong (a very low error), not only good: only a strong pose is
    /// published at acquisition without a second sensor's confirmation (see
    /// `Tracker::provisional`). A good one is right most of the time, but on some recordings
    /// mostly wrong.
    pub strong: bool,
}

impl EnginePose {
    pub fn strong(world_from_model: Isometry3<f64>) -> EnginePose {
        EnginePose {
            world_from_model,
            strong: true,
        }
    }

    pub fn good(world_from_model: Isometry3<f64>) -> EnginePose {
        EnginePose {
            world_from_model,
            strong: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Output {
    /// Controller clock of the newest IMU sample.
    pub t: u64,
    pub world_from_model: Isometry3<f64>,
    /// At the model origin, m/s.
    pub vel: Vector3<f64>,
    pub ang_vel_world: Vector3<f64>,
    /// The filter's gyro bias, rad/s, body frame.
    pub gyro_bias: Vector3<f64>,
    /// The filter's accelerometer bias, m/s^2, body frame.
    pub accel_bias: Vector3<f64>,
    /// A recent optical fix holds the position. Without one the position is held where it was
    /// last tracked, at rest (dead reckoning only feeds the re-lock): never published moving.
    pub tracking: bool,
    /// Controller time of the newest optical fix while the position is tracked; 0 while it is
    /// not (a lost track, or one no second sensor has confirmed, has no fix to show).
    pub last_fix: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    Updated(usize),
    Acquired(usize),
    NotVisible,
    Missed,
    /// Not tracking and no usable engine pose.
    Waiting,
    TooOld,
    /// The exposure is newer than the newest IMU sample: retry after more IMU.
    Early,
    /// The camera is not part of this tracker's registered roster.
    UnknownCamera,
}

/// How well one sensor agrees with the others: an exponentially weighted mean of the median
/// pixel residual of its matched LEDs under a prior the other sensors set (another sensor updated
/// the track within `CROSS_CHECK_NS`). A sensor that moved since calibration stands out.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CameraAgreement {
    pub median_px: f64,
    /// Weighted mean of the per-frame mean residual vector: noise averages out of it, a moved
    /// sensor's consistent offset does not.
    pub bias_px: Vector2<f64>,
    pub samples: u64,
}

pub const CROSS_CHECK_NS: u64 = 25_000_000;
const AGREEMENT_WEIGHT: f64 = 1.0 / 500.0;

/// A measurement applied within the history, kept so that it can be applied again when an older
/// frame arrives after it.
#[derive(Debug, Clone)]
struct Applied {
    t: u64,
    cam: Camera,
    blobs: Vec<Blob>,
    pairs: Vec<(usize, usize)>,
}

#[derive(Debug, Clone)]
pub struct Tracker {
    pub params: Params,
    pub body: Body,
    /// The OpenHMD device id whose labels this controller's blobs carry.
    pub device_id: i32,
    history: VecDeque<(u64, Filter)>,
    imu: VecDeque<ImuSample>,
    /// Alongside `imu`: whether the controller was still at each sample (see `Params`), judged on
    /// arrival so that a rebuild replays the same zero-velocity updates.
    still: VecDeque<bool>,
    applied: VecDeque<Applied>,
    last_fix: u64,
    /// Controller time of the last accepted update or acquisition (for the orientation check).
    last_track: u64,
    /// Bias estimates (and their covariance) kept across an IMU discontinuity.
    saved_bias: Option<Filter>,
    /// First registered camera to update this acquisition, and whether another confirmed it.
    /// A track seen by one sensor may be off in depth: only a distinct second sensor vets it.
    track_camera: Option<usize>,
    track_confirmed: bool,
    /// Controller time of the newest update of a confirmed track.
    last_confirmed: u64,
    /// When the current run of refused acquisitions began (0: none).
    refusing_since: u64,
    /// The exposure of an acquisition no second sensor has confirmed yet, one from an engine pose
    /// rated only good or from the tracker's own search (`search_kept`): either can fit one view
    /// and be wrong. Until a second sensor confirms it the track takes updates but is not
    /// published; if none has within `SearchParams::confirm_within_ns`, the track is dropped, and
    /// with it the kept orientation.
    provisional: Option<u64>,
    /// The model origin where it was last tracked (held while the position is lost).
    held: Option<Vector3<f64>>,
    /// The accelerometer bias remembered from an earlier run (`seed_biases`).
    bias_ref: Option<Vector3<f64>>,
    /// The biases the current track started from, restored when it turns out a wrong lock.
    acquired_bias: Option<Filter>,
    /// A dropped wrong lock's orientation, carried on by the gyro: (controller time, orientation,
    /// refused until).
    wrong_lock: Option<(u64, UnitQuaternion<f64>, u64)>,
    /// The LED constellation's ring aliases, tested at acquisition.
    aliases: Vec<vision::Alias>,
    /// When the current run of acquisitions put off as ambiguous began (0: none).
    ambiguous_since: u64,
    /// The engine's labels for this controller come from a pose that disagrees with the track
    /// (the last engine pose shown did, or the track was acquired turned from it): ignored as
    /// hints until an engine pose agrees again. Its labels carry on from its own pose in frames
    /// where it shows none.
    engine_labels_off: bool,
    /// Controller time of each sensor's newest update.
    camera_last: Vec<u64>,
    /// Per registered camera, the exposure of its last frame searched with a kept orientation.
    searched_at: Vec<u64>,
    pub agreement: Vec<CameraAgreement>,
    pub stats: Stats,
    /// When set, every miss appends (LEDs predicted visible, of them with a blob within 3 px,
    /// matched by the association, engine labels) to `misses`.
    pub debug: bool,
    pub misses: Vec<(usize, usize, usize, usize)>,
    /// When `debug`: (matched, unexplained) of every accepted update.
    pub fits: Vec<(usize, usize)>,
}

impl Tracker {
    pub fn new(device_id: i32, body: Body, params: Params, camera_count: usize) -> Tracker {
        let aliases = vision::ring_aliases(&body.leds);
        Tracker {
            params,
            body,
            device_id,
            history: VecDeque::new(),
            imu: VecDeque::new(),
            still: VecDeque::new(),
            applied: VecDeque::new(),
            last_fix: 0,
            last_track: 0,
            saved_bias: None,
            bias_ref: None,
            acquired_bias: None,
            wrong_lock: None,
            aliases,
            ambiguous_since: 0,
            engine_labels_off: false,
            track_camera: None,
            track_confirmed: false,
            last_confirmed: 0,
            refusing_since: 0,
            provisional: None,
            held: None,
            camera_last: vec![0; camera_count],
            searched_at: vec![0; camera_count],
            agreement: vec![CameraAgreement::default(); camera_count],
            stats: Stats::default(),
            debug: false,
            misses: Vec::new(),
            fits: Vec::new(),
        }
    }

    /// Adds one camera admitted by the owning pipeline; arbitrary frame IDs never grow storage.
    pub(super) fn register_camera(&mut self) {
        self.camera_last.push(0);
        self.searched_at.push(0);
        self.agreement.push(CameraAgreement::default());
    }

    fn gravity(&self) -> Vector3<f64> {
        Vector3::new(0.0, -self.params.gravity, 0.0)
    }

    pub fn newest_imu(&self) -> Option<u64> {
        self.imu.back().map(|s| s.t)
    }

    /// Controller time of the newest optical fix while the position is tracked; 0 while it is not
    /// (lost, or a track no second sensor has confirmed yet).
    pub fn last_fix(&self) -> u64 {
        if self.tracking() {
            self.last_fix
        } else {
            0
        }
    }

    /// The position is tracked, and published: a fix holds it, and no second sensor is still
    /// awaited (see `provisional`).
    pub fn tracking(&self) -> bool {
        self.has_track() && self.provisional.is_none()
    }

    /// A fix holds the position, confirmed or not: the track takes updates.
    fn has_track(&self) -> bool {
        self.last_fix != 0
            && self
                .history
                .back()
                .is_some_and(|&(t, _)| t.saturating_sub(self.last_fix) <= self.params.lost_after_ns)
    }

    pub fn imu(&mut self, s: ImuSample) {
        if self.imu.back().is_some_and(|l| s.t <= l.t) {
            return;
        }
        if self
            .imu
            .back()
            .is_some_and(|l| s.t - l.t > self.params.max_imu_gap_ns)
        {
            self.stats.imu_gaps += 1;
            self.saved_bias = self.history.back().map(|h| h.1).or(self.saved_bias);
            self.history.clear();
            self.imu.clear();
            self.still.clear();
            self.applied.clear();
            self.last_fix = 0;
            self.last_track = 0;
            self.last_confirmed = 0;
            self.track_camera = None;
            self.track_confirmed = false;
            self.refusing_since = 0;
            self.ambiguous_since = 0;
            self.provisional = None;
            self.held = None;
        }
        if let Some((t, q, until)) = self.wrong_lock {
            let bg = self.saved_bias.map_or(Vector3::zeros(), |f| f.x.bg);
            self.wrong_lock = (s.t < until).then(|| {
                (
                    s.t,
                    q * UnitQuaternion::from_scaled_axis(
                        (s.gyro - bg) * (s.t.saturating_sub(t) as f64 * 1e-9),
                    ),
                    until,
                )
            });
        }
        let still = self.is_still(&s);
        self.imu.push_back(s);
        self.still.push_back(still);
        let g = self.gravity();
        if let Some(&(t, mut f)) = self.history.back() {
            f.propagate(
                &s.gyro,
                &s.accel,
                (s.t - t) as f64 * 1e-9,
                &g,
                &self.params.noise,
            );
            self.zupt(&mut f, t, s.t, still);
            self.history.push_back((s.t, f));
        }
        let cutoff = s.t.saturating_sub(self.params.history_ns);
        while self.history.len() > 1 && self.history[1].0 <= cutoff {
            self.history.pop_front();
        }
        let oldest = self.history.front().map_or(cutoff, |h| h.0.min(cutoff));
        while self.imu.len() > 1 && self.imu[0].t <= oldest {
            self.imu.pop_front();
            self.still.pop_front();
        }
        while self.applied.front().is_some_and(|a| a.t < oldest) {
            self.applied.pop_front();
        }
        self.note_held();
    }

    /// Whether the controller is still at sample `s`, from it and the samples before it.
    fn is_still(&self, s: &ImuSample) -> bool {
        let p = &self.params;
        let Some(first) = self.imu.front() else {
            return false;
        };
        if s.t.saturating_sub(first.t) < p.still_window_ns {
            return false;
        }
        let (bg, ba) = self
            .history
            .back()
            .map(|h| (h.1.x.bg, h.1.x.ba))
            .or(self.saved_bias.map(|f| (f.x.bg, f.x.ba)))
            .unwrap_or_default();
        let from = s.t.saturating_sub(p.still_window_ns);
        let win: Vec<&ImuSample> = self
            .imu
            .iter()
            .filter(|x| x.t >= from)
            .chain(std::iter::once(s))
            .collect();
        let n = win.len() as f64;
        let gm = win.iter().map(|x| x.gyro).sum::<Vector3<f64>>() / n;
        let am = win.iter().map(|x| x.accel).sum::<Vector3<f64>>() / n;
        (gm - bg).norm() < p.still_gyro
            && ((am - ba).norm() - p.gravity).abs() < p.still_accel_g
            && win.iter().all(|x| {
                (x.gyro - gm).norm() < p.still_gyro_spread
                    && (x.accel - am).norm() < p.still_accel_spread
            })
    }

    /// The zero-velocity update for the step from `t0` to `t1` (see `Params::zupt_sigma`).
    fn zupt(&self, f: &mut Filter, t0: u64, t1: u64, still: bool) {
        if still && t1 / self.params.zupt_every_ns != t0 / self.params.zupt_every_ns {
            f.update_velocity(&Vector3::zeros(), self.params.zupt_sigma);
        }
    }

    fn note_held(&mut self) {
        if self.tracking() {
            if let Some(&(_, f)) = self.history.back() {
                self.held = Some(self.body.world_from_model(&f.x).translation.vector);
            }
        }
    }

    /// Starts the biases from an earlier run's estimates, trusted to `sd_gyro` rad/s and `sd_accel`
    /// m/s^2: a controller's biases outlast a daemon restart, and learning the accelerometer's
    /// from scratch takes tens of seconds of varied orientation. Only before the first fix.
    pub fn seed_biases(
        &mut self,
        gyro: Vector3<f64>,
        accel: Vector3<f64>,
        sd_gyro: f64,
        sd_accel: f64,
    ) {
        if !self.history.is_empty() || self.saved_bias.is_some() {
            return;
        }
        let x = Nominal {
            q: UnitQuaternion::identity(),
            p: Vector3::zeros(),
            v: Vector3::zeros(),
            bg: gyro,
            ba: accel,
        };
        let mut cov = Cov::zeros();
        for i in 0..3 {
            cov[(BG + i, BG + i)] = sd_gyro * sd_gyro;
            cov[(BA + i, BA + i)] = sd_accel * sd_accel;
        }
        self.saved_bias = Some(Filter { x, cov });
        self.bias_ref = Some(accel);
    }

    /// Whether the newest state's accelerometer bias ran off (see `Params::max_accel_bias_change`).
    fn bias_ran_off(&self) -> bool {
        let Some(&(_, f)) = self.history.back() else {
            return false;
        };
        match self.bias_ref {
            Some(r) => (f.x.ba - r).norm() > self.params.max_accel_bias_change,
            None => f.x.ba.norm() > self.params.max_accel_bias,
        }
    }

    /// Drops the track as a wrong lock: the biases go back to what it started from, the
    /// orientation memory (the wrong orientation) is forgotten, and that orientation is refused
    /// for a while.
    fn drop_wrong_lock(&mut self) {
        let Some(&(t, f)) = self.history.back() else {
            return;
        };
        self.stats.wrong_locks += 1;
        self.wrong_lock = Some((t, f.x.q, t + self.params.wrong_lock_memory_ns));
        self.forget_track();
    }

    /// Gives up an unconfirmed track's fix but keeps what it carries (its orientation, its biases
    /// and the orientation memory): the next acquisition starts from it.
    fn give_up_fix(&mut self) {
        self.provisional = None;
        self.last_fix = 0;
        self.track_camera = None;
        self.track_confirmed = false;
    }

    /// Drops the track, and the orientation memory with it; the biases go back to what the track
    /// started from.
    fn forget_track(&mut self) {
        self.saved_bias = self.acquired_bias.or(self.saved_bias);
        self.history.clear();
        self.applied.clear();
        self.last_fix = 0;
        self.last_track = 0;
        self.last_confirmed = 0;
        self.track_camera = None;
        self.track_confirmed = false;
        self.refusing_since = 0;
        self.ambiguous_since = 0;
        self.provisional = None;
        self.held = None;
    }

    pub fn output(&self) -> Option<Output> {
        let &(t, f) = self.history.back()?;
        let s = self.imu.back()?;
        let w = f.x.q * (s.gyro - f.x.bg);
        let tracking = self.tracking();
        let mut pose = self.body.world_from_model(&f.x);
        let mut vel = f.x.v + w.cross(&(f.x.q * -self.body.imu_pos));
        if !tracking {
            // the gyro keeps the orientation; the position stays where it was last seen, and
            // without one (a track no second sensor has confirmed yet) there is none to show
            pose.translation.vector = self.held?;
            vel = Vector3::zeros();
        }
        Some(Output {
            t,
            world_from_model: pose,
            vel,
            ang_vel_world: w,
            gyro_bias: f.x.bg,
            accel_bias: f.x.ba,
            tracking,
            last_fix: if tracking { self.last_fix } else { 0 },
        })
    }

    /// The filter at time `t` from the history: the index of the newest snapshot at or before it,
    /// and that snapshot propagated to `t` with the IMU sample covering the gap. Every measurement
    /// at or before that snapshot's time is in it; every later one is after `t`.
    fn state_at(&self, t: u64) -> Option<(usize, Filter)> {
        let k = self.history.partition_point(|&(ts, _)| ts <= t);
        if k == 0 {
            return None;
        }
        let (tk, mut f) = self.history[k - 1];
        if t > tk {
            let s = self.imu.iter().find(|s| s.t > tk)?;
            f.propagate(
                &s.gyro,
                &s.accel,
                (t - tk) as f64 * 1e-9,
                &self.gravity(),
                &self.params.noise,
            );
        }
        Some((k - 1, f))
    }

    /// The filter at controller time `t`, when it is within the history and a fix holds the position.
    pub fn filter_at(&self, t: u64) -> Option<Filter> {
        (self.tracking() && t.saturating_sub(self.last_fix) <= self.params.lost_after_ns)
            .then(|| self.state_at(t).map(|(_, f)| f))
            .flatten()
    }

    /// Rebuilds the history after snapshot `k` with `first` (already fitted, at its time) and the
    /// logged measurements after it, re-fitted with their matches, merged with the IMU samples.
    fn rebuild(&mut self, k: usize, first: Applied, posterior: Filter) {
        let later: Vec<Applied> = self
            .applied
            .iter()
            .filter(|a| a.t > first.t)
            .cloned()
            .collect();
        self.applied.retain(|a| a.t <= first.t);
        self.history.truncate(k + 1);
        let (g, n) = (self.gravity(), self.params.noise);
        let (mut t, mut f) = (first.t, posterior);
        self.history.push_back((t, f));
        self.applied.push_back(first);
        let imu: Vec<(ImuSample, bool)> = self
            .imu
            .iter()
            .zip(&self.still)
            .filter(|(s, _)| s.t > t)
            .map(|(s, st)| (*s, *st))
            .collect();
        let (mut i, mut m) = (0, 0);
        loop {
            let next_m = later.get(m).map(|a| a.t);
            let next_s = imu.get(i).map(|s| s.0.t);
            match (next_m, next_s) {
                (Some(tm), Some(ts)) if tm <= ts => {
                    f.propagate(
                        &imu[i].0.gyro,
                        &imu[i].0.accel,
                        (tm - t) as f64 * 1e-9,
                        &g,
                        &n,
                    );
                    t = tm;
                    let a = &later[m];
                    match vision::fit_pairs(
                        &a.cam,
                        &self.body,
                        &f,
                        &a.blobs,
                        a.pairs.clone(),
                        &self.params.fit,
                    ) {
                        Ok(fit) => {
                            f = fit.updated.filter;
                            self.applied.push_back(Applied {
                                pairs: fit.pairs,
                                ..a.clone()
                            });
                        }
                        Err(_) => self.stats.dropped_on_rebuild += 1,
                    }
                    self.history.push_back((t, f));
                    m += 1;
                }
                (_, Some(ts)) => {
                    f.propagate(
                        &imu[i].0.gyro,
                        &imu[i].0.accel,
                        (ts - t) as f64 * 1e-9,
                        &g,
                        &n,
                    );
                    self.zupt(&mut f, t, ts, imu[i].1);
                    t = ts;
                    self.history.push_back((t, f));
                    i += 1;
                }
                // a measurement can't be newer than the newest IMU sample (frames wait for it)
                (Some(_), None) | (None, None) => break,
            }
        }
    }

    /// One sensor frame whose exposure was at controller time `t_e`. `engine` is the engine's
    /// world_from_model for this controller in this frame, when it found one it rates good or
    /// strong.
    /// `foreign`: where the other controller's LEDs are in this frame, as it is tracked.
    pub fn frame(
        &mut self,
        t_e: u64,
        camera: usize,
        cam: &Camera,
        blobs: &[Blob],
        engine: Option<EnginePose>,
        foreign: &[Vector2<f64>],
    ) -> Outcome {
        if camera >= self.camera_last.len() {
            return Outcome::UnknownCamera;
        }
        if self.newest_imu().is_none_or(|n| t_e > n) {
            return Outcome::Early;
        }
        self.stats.frames += 1;
        if self
            .provisional
            .is_some_and(|t| t_e.saturating_sub(t) > self.params.search.confirm_within_ns)
        {
            self.stats.unconfirmed_acquisitions += 1;
            self.forget_track();
        }
        // the other controller's blobs: labelled as another device, or on one of its LEDs
        let own: Vec<Blob> = blobs
            .iter()
            .filter(|b| {
                let ours = b.led_id >= 0 && b.led_id >> 8 == self.device_id;
                let theirs = b.led_id >= 0 && !ours;
                ours || (!theirs
                    && !foreign
                        .iter()
                        .any(|f| (f - b.xy).norm() < self.params.foreign_px))
            })
            .copied()
            .collect();
        let blobs = &own[..];
        let labelled: Vec<(usize, usize)> = blobs
            .iter()
            .enumerate()
            .filter(|(_, b)| {
                b.led_id >= 0
                    && b.led_id >> 8 == self.device_id
                    && ((b.led_id & 0xff) as usize) < self.body.leds.len()
            })
            .map(|(i, b)| ((b.led_id & 0xff) as usize, i))
            .collect();
        // a strong engine pose with labels enough to acquire from settles a track no second
        // sensor has confirmed: one that agrees with it confirms it, and one that does not
        // replaces it (taken below, from the orientation and biases the track carries), unless the
        // kept orientation the track was found with refuses its turn: then it is a flip
        let settling =
            engine.filter(|e| e.strong && labelled.len() >= self.params.acquire_min_leds);
        if let (Some(_), Some(e)) = (self.provisional, settling) {
            if let Some((_, prior)) = self.state_at(t_e) {
                let track = self.body.world_from_model(&prior.x);
                let turned = track.rotation.angle_to(&e.world_from_model.rotation)
                    > self.params.acquire_max_turn_rad;
                let moved = (track.translation.vector - e.world_from_model.translation.vector)
                    .norm()
                    > self.params.search.engine_agree_m;
                let remembered = self.last_confirmed != 0
                    && t_e.saturating_sub(self.last_confirmed) <= self.params.orientation_memory_ns;
                if !turned && !moved {
                    self.provisional = None;
                } else if !(turned && remembered) {
                    self.give_up_fix();
                }
            }
        }
        let relock = !self.has_track()
            && self.last_fix != 0
            && t_e.saturating_sub(self.last_fix) <= self.params.relock_ns;
        if self.has_track() || relock {
            let Some((k, prior)) = self.state_at(t_e) else {
                self.stats.too_old += 1;
                return Outcome::TooOld;
            };
            // the engine labels from its own pose: when that disagrees with the track's, they
            // are a flip or a mislabel (off by one LED around the ring, say), not hints
            if let Some(e) = engine {
                self.engine_labels_off = e.world_from_model.rotation.angle_to(&prior.x.q)
                    > self.params.acquire_max_turn_rad;
            }
            let hints: &[(usize, usize)] = if self.engine_labels_off {
                &[]
            } else {
                &labelled
            };
            match vision::fit_frame(cam, &self.body, &prior, blobs, hints, &self.params.fit) {
                Ok(fit) => {
                    if self.debug {
                        self.fits.push((fit.pairs.len(), fit.unexplained));
                    }
                    self.cross_check(t_e, camera, cam, blobs, &fit.pairs, &prior);
                    if self.applied.back().is_some_and(|a| a.t > t_e) {
                        self.stats.out_of_order += 1;
                    }
                    let n = fit.pairs.len();
                    self.rebuild(
                        k,
                        Applied {
                            t: t_e,
                            cam: *cam,
                            blobs: blobs.to_vec(),
                            pairs: fit.pairs,
                        },
                        fit.updated.filter,
                    );
                    self.last_fix = self.last_fix.max(t_e);
                    self.last_track = self.last_track.max(t_e);
                    if relock {
                        self.stats.relocks += 1;
                        self.track_camera = Some(camera);
                        self.track_confirmed = false;
                    } else {
                        self.track_confirmed |=
                            self.track_camera.is_some_and(|first| first != camera);
                        self.track_camera.get_or_insert(camera);
                    }
                    if self.track_confirmed {
                        self.last_confirmed = self.last_confirmed.max(t_e);
                        self.provisional = None;
                    }
                    self.refusing_since = 0;
                    if self.bias_ran_off() {
                        self.drop_wrong_lock();
                        return Outcome::Missed;
                    }
                    self.note_held();
                    self.stats.updates += 1;
                    self.stats.leds_used += n as u64;
                    return Outcome::Updated(n);
                }
                // a failed re-lock falls through to the engine's acquisition below
                Err(_) if relock => {}
                Err(_)
                    if vision::visible_leds(cam, &self.body, &prior.x, &self.params.fit.assoc)
                        == 0 =>
                {
                    self.stats.not_visible += 1;
                    return Outcome::NotVisible;
                }
                Err(miss) => {
                    if self.debug {
                        let (v, n, a) = vision::association_report(
                            cam,
                            &self.body,
                            &prior,
                            blobs,
                            self.params.fit.update.sigma_px,
                            &self.params.fit.assoc,
                            3.0,
                        );
                        self.misses.push((v, n, a, labelled.len()));
                    }
                    match miss {
                        vision::Miss::TooFew => self.stats.too_few += 1,
                        vision::Miss::Rejected => self.stats.rejected += 1,
                    }
                    return Outcome::Missed;
                }
            }
        }
        let engine = engine.filter(|_| labelled.len() >= self.params.acquire_min_leds);
        // the gyro keeps the orientation for a long while after the position is lost
        let remembered = self.last_confirmed != 0
            && t_e.saturating_sub(self.last_confirmed) <= self.params.orientation_memory_ns;
        if engine.is_none() && !remembered {
            return Outcome::Waiting;
        }
        let (k, carried) = match self.state_at(t_e) {
            Some((k, f)) => (k, Some(f)),
            None if self.history.is_empty() => (0, self.saved_bias),
            None => {
                self.stats.too_old += 1;
                return Outcome::TooOld;
            }
        };
        let memory = carried.filter(|_| remembered);
        let fit_params = FitParams {
            min_leds: self.params.acquire_min_leds,
            ..self.params.fit
        };
        // an engine pose that disagrees with the kept orientation is a flip or a mislabel
        let mut refused = false;
        if let (Some(e), Some(f)) = (engine, memory) {
            if f.x.q.angle_to(&e.world_from_model.rotation) > self.params.acquire_max_turn_rad {
                if self.refusing_since == 0 {
                    self.refusing_since = t_e;
                }
                if t_e.saturating_sub(self.refusing_since) < self.params.max_refusing_ns {
                    self.stats.refused_acquisitions += 1;
                    refused = true;
                } else {
                    self.last_confirmed = 0;
                }
            }
        }
        let (chosen, turned, unconfirmed) = match (engine.filter(|_| !refused), memory) {
            (Some(e), _) => {
                let world_from_model = e.world_from_model;
                if self.wrong_lock.is_some_and(|(_, q, _)| {
                    q.angle_to(&world_from_model.rotation) < self.params.wrong_lock_turn_rad
                }) {
                    self.stats.refused_acquisitions += 1;
                    return Outcome::Waiting;
                }
                self.refusing_since = 0;
                let Some(mut chosen) = self.attempt(
                    cam,
                    blobs,
                    &world_from_model,
                    labelled.clone(),
                    carried.as_ref(),
                    &fit_params,
                ) else {
                    self.stats.failed_acquisitions += 1;
                    return Outcome::Waiting;
                };
                // nothing to check the engine's orientation against (a pickup after sleep, a
                // first fix): a labelling off by one around the ring fits the frame almost as
                // well, so its turns are tried on this frame too
                let mut turned = false;
                if memory.is_none() && !self.aliases.is_empty() {
                    let mut costs = vec![self.acquisition_cost(cam, blobs, &chosen.1)];
                    let mut best: Option<(f64, (Filter, vision::Fit))> = None;
                    for a in &self.aliases {
                        let pairs: Vec<(usize, usize)> = labelled
                            .iter()
                            .filter_map(|&(i, b)| a.led_at[i].map(|j| (j, b)))
                            .collect();
                        if let Some(h) = self.attempt(
                            cam,
                            blobs,
                            &(world_from_model * a.turn),
                            pairs,
                            carried.as_ref(),
                            &fit_params,
                        ) {
                            let c = self.acquisition_cost(cam, blobs, &h.1);
                            if c < costs[0] && best.as_ref().is_none_or(|b| c < b.0) {
                                best = Some((c, h));
                            }
                            costs.push(c);
                        }
                    }
                    costs.sort_by(f64::total_cmp);
                    if costs.len() > 1 && costs[1] - costs[0] < self.params.alias_margin {
                        if self.ambiguous_since == 0 {
                            self.ambiguous_since = t_e;
                        }
                        if t_e.saturating_sub(self.ambiguous_since) < self.params.max_refusing_ns {
                            self.stats.ambiguous_acquisitions += 1;
                            return Outcome::Waiting;
                        }
                    }
                    if let Some((_, h)) = best {
                        chosen = h;
                        turned = true;
                    }
                }
                if !e.strong {
                    self.stats.good_acquisitions += 1;
                }
                (chosen, turned, !e.strong)
            }
            // no pose from the engine, or one refused: the kept orientation may find the
            // controller in this frame by itself
            (None, Some(f)) => {
                let p = &self.params.search;
                if blobs.len() > p.max_blobs
                    || self.searched_at[camera] != 0
                        && t_e.saturating_sub(self.searched_at[camera]) < p.every_ns
                {
                    return Outcome::Waiting;
                }
                self.searched_at[camera] = t_e;
                self.stats.searches += 1;
                match self.search_kept(cam, blobs, &f, carried.as_ref(), &fit_params) {
                    Some(h) => {
                        self.stats.searched_acquisitions += 1;
                        (h, false, true)
                    }
                    None => return Outcome::Waiting,
                }
            }
            (None, None) => return Outcome::Waiting,
        };
        self.ambiguous_since = 0;
        // lying still, the accelerometer knows which way is up
        if let Some(up) = self.steady_up(t_e, carried.map_or(Vector3::zeros(), |f| f.x.ba)) {
            if (chosen.0.x.q.inverse() * Vector3::y()).angle(&up) > self.params.still_max_tilt_rad {
                self.stats.refused_acquisitions += 1;
                return Outcome::Waiting;
            }
        }
        // a turned pose can be the orientation of a lock just dropped as wrong
        if turned
            && self.wrong_lock.is_some_and(|(_, q, _)| {
                q.angle_to(&chosen.0.x.q) < self.params.wrong_lock_turn_rad
            })
        {
            self.stats.refused_acquisitions += 1;
            return Outcome::Waiting;
        }
        if turned {
            self.stats.alias_turns += 1;
        }
        if unconfirmed {
            self.refusing_since = 0;
        }
        self.provisional = unconfirmed.then_some(t_e);
        // labels from an engine pose turned from the track's are a flip or a mislabel
        self.engine_labels_off = turned || refused;
        let (prior, fit) = chosen;
        let n = fit.pairs.len();
        let applied = Applied {
            t: t_e,
            cam: *cam,
            blobs: blobs.to_vec(),
            pairs: fit.pairs,
        };
        let fresh = self.history.is_empty();
        if fresh {
            self.history.push_back((t_e, prior));
        }
        // what a drop of this track (a wrong lock, or no second sensor confirming it) goes back to
        self.acquired_bias = carried;
        // later measurements were made under the lost state: start clean from here
        self.applied.retain(|a| a.t <= t_e);
        self.rebuild(k, applied, fit.updated.filter);
        if fresh {
            self.history.pop_front();
        }
        self.last_fix = self.last_fix.max(t_e);
        self.last_track = self.last_track.max(t_e);
        self.track_camera = Some(camera);
        self.track_confirmed = false;
        self.note_held();
        self.stats.acquisitions += 1;
        Outcome::Acquired(n)
    }

    /// One acquisition hypothesis: a pose, and the pairs that seed it (the engine's labels as they
    /// read under it, or a search's). Our own association under that fit adds what they missed,
    /// and the frame is fused once, from the acquisition prior. Returns the prior and the fit.
    fn attempt(
        &self,
        cam: &Camera,
        blobs: &[Blob],
        pose: &Isometry3<f64>,
        pairs: Vec<(usize, usize)>,
        carried: Option<&Filter>,
        fit_params: &FitParams,
    ) -> Option<(Filter, vision::Fit)> {
        let prior = self.acquisition_prior(pose, carried);
        let fit = vision::fit_pairs(cam, &self.body, &prior, blobs, pairs, fit_params).ok()?;
        let more = vision::merge_pairs(
            fit.pairs.clone(),
            &vision::associate(
                cam,
                &self.body,
                &fit.updated.filter,
                blobs,
                self.params.fit.update.sigma_px,
                &self.params.fit.assoc,
            ),
        );
        let fit = if more.len() > fit.pairs.len() {
            vision::fit_pairs(cam, &self.body, &prior, blobs, more, &self.params.fit).unwrap_or(fit)
        } else {
            fit
        };
        Some((prior, fit))
    }

    /// The controller in this frame from the kept orientation `kept` alone (see `search`): each
    /// position the frame supports is fitted, which corrects the orientation too. With nothing
    /// else to back it, a fit counts only if it explains the frame almost wholly
    /// (`explains_frame`); costed like an engine pose, the cheapest is taken, and a second sensor
    /// still has to confirm it (see `provisional`).
    fn search_kept(
        &self,
        cam: &Camera,
        blobs: &[Blob],
        kept: &Filter,
        carried: Option<&Filter>,
        fit_params: &FitParams,
    ) -> Option<(Filter, vision::Fit)> {
        let all: Vec<usize> = (0..blobs.len()).collect();
        let costed: Vec<(f64, (Filter, vision::Fit))> =
            search::search_position(cam, &self.body, &kept.x.q, blobs, &all, &self.params.search)
                .into_iter()
                .filter_map(|c| {
                    self.attempt(
                        cam,
                        blobs,
                        &c.world_from_model,
                        c.pairs,
                        carried,
                        fit_params,
                    )
                })
                .filter(|(_, fit)| self.explains_frame(cam, blobs, fit))
                .map(|h| (self.acquisition_cost(cam, blobs, &h.1), h))
                .collect();
        costed
            .into_iter()
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, h)| h)
    }

    /// A fit pairs enough LEDs, nearly all those it shows the camera, closely, and leaves next to
    /// no blob in its footprint unexplained (see `SearchParams`): a position found under a wrong
    /// orientation, or from blobs paired with the wrong LEDs, fits some LEDs loosely but not the
    /// frame.
    fn explains_frame(&self, cam: &Camera, blobs: &[Blob], fit: &vision::Fit) -> bool {
        let p = &self.params.search;
        let (visible, _, _) = vision::association_report(
            cam,
            &self.body,
            &fit.updated.filter,
            blobs,
            self.params.fit.update.sigma_px,
            &self.params.fit.assoc,
            3.0,
        );
        let n = fit.pairs.len();
        let residuals =
            vision::residuals(cam, &self.body, blobs, &fit.pairs, &fit.updated.filter.x);
        let rms = (residuals.iter().map(|o| o.r.norm_squared()).sum::<f64>()
            / residuals.len().max(1) as f64)
            .sqrt();
        n >= p.min_fit_leds
            && n as f64 >= p.min_fit_share * visible as f64
            && fit.unexplained <= p.max_unexplained
            && rms <= p.max_fit_rms_px
    }

    /// Updates the sensor's agreement with the others from this frame's residuals under the prior,
    /// when another sensor set that prior moments ago.
    fn cross_check(
        &mut self,
        t_e: u64,
        camera: usize,
        cam: &Camera,
        blobs: &[Blob],
        pairs: &[(usize, usize)],
        prior: &Filter,
    ) {
        if camera >= self.camera_last.len() {
            return;
        }
        let others = (0..self.camera_last.len()).any(|c| {
            c != camera
                && self.camera_last[c] != 0
                && t_e.saturating_sub(self.camera_last[c]) <= CROSS_CHECK_NS
                && self.camera_last[c] <= t_e
        });
        self.camera_last[camera] = self.camera_last[camera].max(t_e);
        if !others || pairs.len() < 4 {
            return;
        }
        let res = vision::residuals(cam, &self.body, blobs, pairs, &prior.x);
        let mean = res.iter().map(|o| o.r).sum::<Vector2<f64>>() / res.len() as f64;
        let mut r: Vec<f64> = res.iter().map(|o| o.r.norm()).collect();
        r.sort_by(|a, b| a.total_cmp(b));
        let median = r[r.len() / 2];
        let a = &mut self.agreement[camera];
        let w = if a.samples < 50 {
            1.0 / (a.samples + 1) as f64
        } else {
            AGREEMENT_WEIGHT
        };
        a.median_px += w * (median - a.median_px);
        a.bias_px += (mean - a.bias_px) * w;
        a.samples += 1;
    }

    /// Gravity's direction in the controller at controller time `t`, less the bias `ba`, when the
    /// accelerometer held steady before it (see `Params::still_max_tilt_rad`).
    fn steady_up(&self, t: u64, ba: Vector3<f64>) -> Option<Vector3<f64>> {
        let k = self.imu.partition_point(|s| s.t <= t);
        let from = t.saturating_sub(self.params.steady_window_ns);
        // the window must be covered: a sample at or before its start
        if k == 0 || self.imu.front().is_none_or(|s| s.t > from) {
            return None;
        }
        let win: Vec<&ImuSample> = self.imu.iter().take(k).filter(|s| s.t >= from).collect();
        let mean = win.iter().map(|s| s.accel).sum::<Vector3<f64>>() / win.len() as f64;
        let steady = win
            .iter()
            .all(|s| (s.accel - mean).norm() < self.params.steady_accel)
            && ((mean - ba).norm() - self.params.gravity).abs() < self.params.steady_accel;
        steady.then(|| (mean - ba).normalize())
    }

    /// How badly an acquisition hypothesis explains its frame (see `Params::alias_sigma_px`).
    fn acquisition_cost(&self, cam: &Camera, blobs: &[Blob], fit: &vision::Fit) -> f64 {
        let f = &fit.updated.filter;
        let chi: f64 = vision::residuals(cam, &self.body, blobs, &fit.pairs, &f.x)
            .iter()
            .map(|r| (r.r.norm() / self.params.alias_sigma_px).powi(2))
            .sum();
        let (visible, near, _) = vision::association_report(
            cam,
            &self.body,
            f,
            blobs,
            self.params.fit.update.sigma_px,
            &self.params.fit.assoc,
            3.0,
        );
        chi + self.params.alias_miss_cost * (visible.saturating_sub(near) + fit.unexplained) as f64
    }

    fn acquisition_prior(
        &self,
        world_from_model: &Isometry3<f64>,
        carried: Option<&Filter>,
    ) -> Filter {
        let q = world_from_model.rotation;
        let p = world_from_model * nalgebra::Point3::from(self.body.imu_pos);
        let [rot, pos, vel, bg, ba] = self.params.acquire_sd;
        let mut cov = Cov::zeros();
        let mut x = Nominal {
            q,
            p: p.coords,
            v: Vector3::zeros(),
            bg: Vector3::zeros(),
            ba: Vector3::zeros(),
        };
        for (block, sd) in [(ROT, rot), (POS, pos), (VEL, vel), (BG, bg), (BA, ba)] {
            for i in 0..3 {
                cov[(block + i, block + i)] = sd * sd;
            }
        }
        if let Some(f) = carried {
            x.bg = f.x.bg;
            x.ba = f.x.ba;
            cov.fixed_view_mut::<6, 6>(BG, BG)
                .copy_from(&f.cov.fixed_view::<6, 6>(BG, BG));
        }
        Filter { x, cov }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::{self, Motion, Rng};
    use nalgebra::{Point3, UnitQuaternion, Vector2};

    const G: f64 = 9.80665;

    struct World {
        cams: Vec<Camera>,
        body: Body,
        motion: Motion,
        ba: Vector3<f64>,
        bg: Vector3<f64>,
    }

    fn world(seed: u64) -> World {
        let target = Vector3::new(0.0, 1.0, 0.0);
        let cams = [
            (Vector3::new(-2.0, 2.3, 1.8), true),
            (Vector3::new(2.0, 2.3, 1.8), false),
            (Vector3::new(0.3, 2.4, -2.2), true),
        ]
        .iter()
        .map(|&(pos, flip)| Camera {
            model: synth::camera_model(),
            world_from_cam: synth::look_at(pos, target, flip),
        })
        .collect();
        let mut rng = Rng::new(seed);
        World {
            cams,
            body: Body {
                leds: synth::touch_leds(),
                imu_pos: Vector3::new(0.015, -0.012, 0.008),
            },
            motion: Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0)),
            ba: Vector3::new(0.02, 0.4, -0.05),
            bg: Vector3::new(0.01, -0.005, 0.003),
        }
    }

    /// `world` with a real Touch's LEDs: its ring is not symmetric, so only part of it lands on
    /// itself when turned (see `vision::ring_aliases`).
    fn touch_world(seed: u64) -> World {
        let mut w = world(seed);
        let (leds, imu_pos) = synth::cv1_touch_left();
        w.body = Body { leds, imu_pos };
        w
    }

    fn imu(w: &World, t: f64) -> ImuSample {
        ImuSample {
            t: (1e9 + t * 1e9) as u64,
            gyro: w.motion.gyro(t) + w.bg,
            accel: w
                .motion
                .accel(t, &w.body.imu_pos, &Vector3::new(0.0, G, 0.0))
                + w.ba,
        }
    }

    fn sight(w: &World, cam: &Camera, t: f64, rng: &mut Rng, noise: f64) -> Vec<Blob> {
        let pose = w.motion.pose(t);
        (0..w.body.leds.len())
            .filter_map(|i| {
                let pc = cam.world_from_cam.inverse() * (pose * Point3::from(w.body.leds[i].pos));
                let n =
                    cam.world_from_cam.inverse().rotation * (pose.rotation * w.body.leds[i].dir);
                if n.dot(&(-pc.coords)) / pc.coords.norm() < 0.3 {
                    return None;
                }
                let uv =
                    cam.model.project(&pc.coords)? + Vector2::new(rng.gauss(), rng.gauss()) * noise;
                Some(Blob {
                    xy: uv,
                    led_id: (1 << 8) | i as i32,
                })
            })
            .collect()
    }

    #[derive(Default)]
    struct Run {
        /// No frames in this span.
        blackout: Option<(f64, f64)>,
        /// Each frame arrives 20 to 45 ms after its exposure instead of 25 ms, so sensors overtake
        /// each other.
        shuffle: bool,
        /// The engine reports a pose turned 90 degrees in this span.
        engine_flipped: Option<(f64, f64)>,
        /// From this time on the engine gives neither poses nor labels.
        engine_silent_after: Option<f64>,
        /// Biases (gyro, accel) remembered from an earlier run.
        seed: Option<(Vector3<f64>, Vector3<f64>)>,
        /// From this time on the accelerometer reads this much more: the filter's bias runs off
        /// the way it does under a wrong lock.
        accel_jump: Option<(f64, Vector3<f64>)>,
        /// The track is dropped as a wrong lock at this time.
        drop_at: Option<f64>,
        /// At this time the kept state is turned by this rotation vector (rad), as a gyro gone
        /// wrong would leave it.
        turn_memory_at: Option<(f64, Vector3<f64>)>,
        /// The engine rates its poses good, never strong; but strong at the last of every this
        /// many exposures.
        engine_good: bool,
        strong_every: Option<usize>,
        /// From this time on only this many sensors (the first) deliver frames.
        cameras_after: Option<(f64, usize)>,
        /// From this time on every frame has this many stray blobs too.
        clutter_after: Option<(f64, usize)>,
    }

    /// Motion: IMU at 500 Hz, three sensors at 52 Hz, frames arriving 25 ms after exposure.
    /// Returns the position errors (m) of every output after the first second.
    fn run(
        w: &World,
        seconds: f64,
        pixel_noise: f64,
        drop_cams_between: Option<(f64, f64)>,
    ) -> (Tracker, Vec<f64>) {
        run_with(
            w,
            seconds,
            pixel_noise,
            &Run {
                blackout: drop_cams_between,
                ..Run::default()
            },
        )
    }

    fn run_with(w: &World, seconds: f64, pixel_noise: f64, opts: &Run) -> (Tracker, Vec<f64>) {
        let drop_cams_between = opts.blackout;
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        if let Some((bg, ba)) = opts.seed {
            tr.seed_biases(bg, ba, 0.002, 0.03);
        }
        let mut rng = Rng::new(11);
        // (exposure, arrival, camera, blobs)
        let mut pending: Vec<(f64, f64, usize, Vec<Blob>)> = Vec::new();
        let mut errors = Vec::new();
        let (dt_imu, dt_frame, delay) = (0.002, 1.0 / 52.0, 0.025);
        let mut next_frame = 0.01;
        let mut t = 0.0;
        let (mut dropped, mut turned) = (false, false);
        while t < seconds {
            t += dt_imu;
            let mut s = imu(w, t);
            if let Some((after, jump)) = opts.accel_jump.filter(|(after, _)| t >= *after) {
                let _ = after;
                s.accel += jump;
            }
            tr.imu(s);
            if opts.drop_at.is_some_and(|d| t >= d) && !dropped {
                dropped = true;
                tr.drop_wrong_lock();
            }
            assert_shows_nothing_unconfirmed(&tr);
            if let Some((_, turn)) = opts.turn_memory_at.filter(|(at, _)| t >= *at && !turned) {
                turned = true;
                let r = UnitQuaternion::from_scaled_axis(turn);
                for (_, f) in tr.history.iter_mut() {
                    f.x.q = r * f.x.q;
                }
            }
            while next_frame <= t {
                if !drop_cams_between.is_some_and(|(a, b)| next_frame >= a && next_frame < b) {
                    let cams = opts
                        .cameras_after
                        .filter(|(after, _)| next_frame >= *after)
                        .map_or(w.cams.len(), |(_, n)| n);
                    for (c, cam) in w.cams.iter().enumerate().take(cams) {
                        let late = if opts.shuffle {
                            rng.range(-0.005, 0.02)
                        } else {
                            0.0
                        };
                        pending.push((
                            next_frame,
                            next_frame + delay + late,
                            c,
                            sight(w, cam, next_frame, &mut rng, pixel_noise),
                        ));
                    }
                }
                next_frame += dt_frame;
            }
            let mut ready: Vec<_> = pending.iter().filter(|p| p.1 <= t).cloned().collect();
            ready.sort_by(|a, b| a.1.total_cmp(&b.1));
            pending.retain(|p| p.1 > t);
            for (te, _, c, mut blobs) in ready {
                if let Some((_, n)) = opts.clutter_after.filter(|(after, _)| te >= *after) {
                    for _ in 0..n {
                        let xy = Vector2::new(rng.range(0.0, 1280.0), rng.range(0.0, 960.0));
                        blobs.push(Blob { xy, led_id: -1 });
                    }
                }
                let mut engine = Some(w.motion.pose(te));
                if opts.engine_flipped.is_some_and(|(a, b)| te >= a && te < b) {
                    engine = engine.map(|mut e| {
                        e.rotation *= UnitQuaternion::from_scaled_axis(Vector3::new(
                            0.0,
                            std::f64::consts::FRAC_PI_2,
                            0.0,
                        ));
                        e
                    });
                }
                if opts.engine_silent_after.is_some_and(|a| te >= a) {
                    engine = None;
                    blobs.iter_mut().for_each(|b| b.led_id = -1);
                }
                tr.frame(
                    (1e9 + te * 1e9) as u64,
                    c,
                    &w.cams[c],
                    &blobs,
                    engine.map(|e| EnginePose {
                        world_from_model: e,
                        strong: !opts.engine_good
                            || opts.strong_every.is_some_and(|n| {
                                ((te - 0.01) * 52.0).round() as usize % n == n - 1
                            }),
                    }),
                    &[],
                );
            }
            if t > 1.0 {
                let Some(o) = tr.output() else { continue };
                if o.tracking && !opts.engine_flipped.is_some_and(|(a, b)| t >= a && t < b) {
                    errors.push(
                        (o.world_from_model.translation.vector
                            - w.motion.pose(t).translation.vector)
                            .norm(),
                    );
                }
            }
        }
        (tr, errors)
    }

    /// While no second sensor has confirmed a track, its output shows no position of it: none
    /// at all, or the one held from before, never tracking and with no fix.
    fn assert_shows_nothing_unconfirmed(tr: &Tracker) {
        if tr.provisional.is_none() {
            return;
        }
        assert_eq!(tr.last_fix(), 0);
        if let Some(o) = tr.output() {
            assert!(!o.tracking && o.last_fix == 0, "{o:?}");
            assert_eq!(
                Some(o.world_from_model.translation.vector),
                tr.held,
                "{o:?}"
            );
        }
    }

    fn pct(v: &[f64], p: f64) -> f64 {
        let mut s = v.to_vec();
        s.sort_by(|a, b| a.total_cmp(b));
        s[((s.len() - 1) as f64 * p) as usize]
    }

    #[test]
    fn tracks_a_swinging_controller_to_a_millimetre_with_late_frames() {
        let w = world(3);
        let (tr, errors) = run(&w, 10.0, 0.3, None);
        assert_eq!(tr.stats.acquisitions, 1, "{:?}", tr.stats);
        assert!(
            errors.len() > 4000,
            "tracked only {} outputs: {:?}",
            errors.len(),
            tr.stats
        );
        let (p50, p99) = (pct(&errors, 0.5), pct(&errors, 0.99));
        assert!(
            p50 < 1e-3 && p99 < 4e-3,
            "p50 {p50} p99 {p99} {:?}",
            tr.stats
        );
        let o = tr.output().unwrap();
        let truth_v = (w.motion.pose(10.0 + 1e-4).translation.vector
            - w.motion.pose(10.0 - 1e-4).translation.vector)
            / 2e-4;
        assert!(
            (o.vel - truth_v).norm() < 0.03,
            "velocity {:?} vs {truth_v:?}",
            o.vel
        );
    }

    #[test]
    fn coasts_through_a_short_blackout_and_relocks_without_reacquiring() {
        let w = world(4);
        let (tr, errors) = run(&w, 6.0, 0.3, Some((3.0, 3.3)));
        assert_eq!(tr.stats.acquisitions, 1, "{:?}", tr.stats);
        assert!(pct(&errors, 0.5) < 1e-3, "{:?}", tr.stats);
    }

    #[test]
    fn frames_overtaking_each_other_are_all_used() {
        let w = world(6);
        let (tr, errors) = run_with(
            &w,
            8.0,
            0.3,
            &Run {
                shuffle: true,
                ..Run::default()
            },
        );
        assert!(
            tr.stats.out_of_order > 50 && tr.stats.too_old == 0,
            "{:?}",
            tr.stats
        );
        assert!(
            tr.stats.dropped_on_rebuild < tr.stats.out_of_order / 20,
            "{:?}",
            tr.stats
        );
        assert!(
            pct(&errors, 0.5) < 1e-3 && pct(&errors, 0.99) < 4e-3,
            "p50 {} p99 {} {:?}",
            pct(&errors, 0.5),
            pct(&errors, 0.99),
            tr.stats
        );
    }

    /// A 1.5 s blackout loses the position; afterwards the engine helps no more (no poses, no
    /// labels). The gyro-held orientation and the dead-reckoned position re-lock by themselves.
    #[test]
    fn relocks_after_a_blackout_without_the_engine() {
        let w = world(9);
        let (tr, errors) = run_with(
            &w,
            6.0,
            0.3,
            &Run {
                blackout: Some((3.0, 4.5)),
                engine_silent_after: Some(3.0),
                ..Run::default()
            },
        );
        assert_eq!(tr.stats.acquisitions, 1, "{:?}", tr.stats);
        assert!(tr.stats.relocks >= 1, "{:?}", tr.stats);
        assert!(tr.tracking(), "{:?}", tr.stats);
        // the first updates after the re-lock pull in the dead-reckoning error
        assert!(
            pct(&errors, 0.5) < 1e-3 && pct(&errors, 0.99) < 15e-3,
            "p50 {} p99 {} {:?}",
            pct(&errors, 0.5),
            pct(&errors, 0.99),
            tr.stats
        );
    }

    /// A fit explains the frame with enough LEDs, nearly all those it shows the camera,
    /// reprojected closely, and at most one stray blob in its footprint.
    #[test]
    fn explains_frame_takes_only_a_close_nearly_whole_fit() {
        let w = touch_world(3);
        let tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let t = 2.0;
        let pose = w.motion.pose(t);
        let fit_params = FitParams {
            min_leds: 5,
            ..tr.params.fit
        };
        let mut rng = Rng::new(5);
        let (cam, blobs) = w
            .cams
            .iter()
            .map(|c| (c, sight(&w, c, t, &mut rng, 0.2)))
            .max_by_key(|(_, b)| b.len())
            .unwrap();
        let labels = blobs
            .iter()
            .enumerate()
            .map(|(i, b)| ((b.led_id & 0xff) as usize, i))
            .collect();
        let (_, fit) = tr
            .attempt(cam, &blobs, &pose, labels, None, &fit_params)
            .unwrap();
        assert!(fit.pairs.len() >= 8 && tr.explains_frame(cam, &blobs, &fit));
        let mut strays = fit.clone();
        strays.unexplained = 2;
        assert!(!tr.explains_frame(cam, &blobs, &strays));
        let mut part = fit.clone();
        part.pairs.truncate(fit.pairs.len() * 3 / 4);
        assert!(
            !tr.explains_frame(cam, &blobs, &part),
            "three quarters of the LEDs"
        );
        // the same fit against blobs a pixel off it, each its own way
        let loose: Vec<Blob> = blobs
            .iter()
            .map(|b| {
                let a = rng.range(0.0, std::f64::consts::TAU);
                Blob {
                    xy: b.xy + Vector2::new(a.cos(), a.sin()),
                    ..*b
                }
            })
            .collect();
        assert!(!tr.explains_frame(cam, &loose, &fit), "a pixel off");
    }

    /// An engine pose rated only good is taken, and published once a second sensor's frame
    /// confirms it: from the first exposure all three sensors see.
    #[test]
    fn a_good_engine_pose_is_published_once_a_second_sensor_confirms_it() {
        let w = world(3);
        let (tr, errors) = run_with(
            &w,
            4.0,
            0.3,
            &Run {
                engine_good: true,
                ..Run::default()
            },
        );
        assert!(tr.tracking(), "{:?}", tr.stats);
        assert_eq!(
            (
                tr.stats.good_acquisitions,
                tr.stats.unconfirmed_acquisitions
            ),
            (1, 0),
            "{:?}",
            tr.stats
        );
        assert!(
            errors.len() > 1490 && pct(&errors, 0.99) < 4e-3,
            "{:?}",
            tr.stats
        );
    }

    /// One sensor alone confirms nothing: an acquisition from a good engine pose, or from the
    /// kept orientation, that only one sensor sees is never published, and is dropped.
    #[test]
    fn an_acquisition_one_sensor_alone_sees_is_not_published() {
        let w = world(3);
        let (tr, errors) = run_with(
            &w,
            4.0,
            0.3,
            &Run {
                engine_good: true,
                cameras_after: Some((0.0, 1)),
                ..Run::default()
            },
        );
        assert!(errors.is_empty() && !tr.tracking(), "{:?}", tr.stats);
        assert!(
            tr.stats.unconfirmed_acquisitions > 0
                && tr.stats.unconfirmed_acquisitions + 1 >= tr.stats.good_acquisitions,
            "{:?}",
            tr.stats
        );

        let w = touch_world(9);
        let (tr, errors) = run_with(
            &w,
            9.0,
            0.3,
            &Run {
                blackout: Some((3.0, 6.5)),
                engine_silent_after: Some(3.0),
                cameras_after: Some((6.5, 1)),
                ..Run::default()
            },
        );
        assert!(!tr.tracking(), "{:?}", tr.stats);
        assert!(
            tr.stats.unconfirmed_acquisitions > 0
                && tr.stats.unconfirmed_acquisitions + 1 >= tr.stats.searched_acquisitions,
            "{:?}",
            tr.stats
        );
        // tracked from 1.0 s until lost 0.5 s into the blackout, and not again
        assert!(errors.len() < 1260, "{} tracked", errors.len());
    }

    /// After a blackout longer than the re-lock window, with the engine silent from then on (no
    /// poses, no labels), the orientation the gyro kept finds the controller again from the
    /// first frames it is back in, by itself.
    #[test]
    fn reacquires_from_the_kept_orientation_alone_after_a_long_blackout() {
        let w = touch_world(9);
        let (tr, errors) = run_with(
            &w,
            9.0,
            0.3,
            &Run {
                blackout: Some((3.0, 6.5)),
                engine_silent_after: Some(3.0),
                ..Run::default()
            },
        );
        assert!(tr.tracking(), "{:?}", tr.stats);
        assert_eq!(
            (tr.stats.acquisitions, tr.stats.searched_acquisitions),
            (2, 1),
            "{:?}",
            tr.stats
        );
        // tracked from 1.0 s until lost 0.5 s into the blackout, and again within 60 ms of its end
        assert!(
            errors.len() > ((2.5 + 2.44) * 500.0) as usize,
            "{} tracked",
            errors.len()
        );
        assert!(
            pct(&errors, 0.5) < 1e-3 && pct(&errors, 0.99) < 15e-3,
            "p50 {} p99 {} {:?}",
            pct(&errors, 0.5),
            pct(&errors, 0.99),
            tr.stats
        );
    }

    /// A kept orientation that is wrong (turned 60 degrees, about any axis, while the position
    /// was lost) acquires nothing: no candidate a search finds under it fits the frame well
    /// enough, and none is published.
    #[test]
    fn a_wrong_kept_orientation_publishes_nothing() {
        for seed in [4, 7, 9] {
            for axis in [Vector3::x(), Vector3::y(), Vector3::z()] {
                let w = touch_world(seed);
                let opts = Run {
                    blackout: Some((3.0, 6.5)),
                    engine_silent_after: Some(3.0),
                    turn_memory_at: Some((4.0, axis * 60f64.to_radians())),
                    ..Run::default()
                };
                let (tr, errors) = run_with(&w, 9.0, 0.3, &opts);
                // coasting into the blackout drifts about a centimetre; wrong poses are metres off
                assert!(
                    errors.iter().all(|e| *e < 0.03),
                    "seed {seed} {axis:?}: {:?}",
                    tr.stats
                );
                // a real Touch is not symmetric: the frame alone rules the wrong candidates out,
                // before a second sensor is needed to
                assert_eq!(
                    tr.stats.searched_acquisitions, 0,
                    "seed {seed} {axis:?}: {:?}",
                    tr.stats
                );
            }
        }
    }

    /// A strong engine pose settles a track no second sensor has confirmed: with one sensor and
    /// the engine rating every other pose strong, the track it agrees with is published at
    /// once, not dropped and taken again.
    #[test]
    fn a_strong_engine_pose_confirms_an_unconfirmed_track_it_agrees_with() {
        let w = world(3);
        let (tr, errors) = run_with(
            &w,
            2.5,
            0.3,
            &Run {
                engine_good: true,
                strong_every: Some(2),
                cameras_after: Some((0.0, 1)),
                ..Run::default()
            },
        );
        assert!(
            tr.tracking() && tr.stats.acquisitions == 1,
            "{:?}",
            tr.stats
        );
        // one sensor 2.9 m off measures depth to a centimetre or so
        assert!(
            tr.stats.good_acquisitions == 1 && errors.len() > 700 && pct(&errors, 0.5) < 0.02,
            "{} p50 {} {:?}",
            errors.len(),
            pct(&errors, 0.5),
            tr.stats
        );
    }

    /// An unconfirmed track a strong engine pose disagrees with is dropped, and the strong pose
    /// taken instead.
    #[test]
    fn a_strong_engine_pose_replaces_an_unconfirmed_track_it_disagrees_with() {
        let w = world(3);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let mut rng = Rng::new(3);
        let mut t = 0.0;
        let mut until = |tr: &mut Tracker, end: f64| {
            while t < end {
                t += 0.002;
                tr.imu(imu(&w, t));
            }
        };
        // one sensor, the one that sees most of it
        let c = (0..w.cams.len())
            .max_by_key(|&c| sight(&w, &w.cams[c], 0.45, &mut Rng::new(1), 0.0).len())
            .unwrap();
        let mut frame = |tr: &mut Tracker, te: f64, strong: bool| {
            let blobs = sight(&w, &w.cams[c], te, &mut rng, 0.3);
            let engine = EnginePose {
                world_from_model: w.motion.pose(te),
                strong,
            };
            tr.frame(
                (1e9 + te * 1e9) as u64,
                c,
                &w.cams[c],
                &blobs,
                Some(engine),
                &[],
            )
        };
        until(&mut tr, 0.5);
        assert!(matches!(frame(&mut tr, 0.45, false), Outcome::Acquired(_)));
        assert!(tr.provisional.is_some() && !tr.tracking());
        // the unconfirmed track turns out 30 cm off
        for (_, f) in tr.history.iter_mut() {
            f.x.p += Vector3::new(0.3, 0.0, 0.0);
        }
        let next = 0.45 + 1.0 / 52.0;
        until(&mut tr, next + 0.025);
        assert!(matches!(frame(&mut tr, next, true), Outcome::Acquired(_)));
        let o = tr.output().unwrap();
        assert!(o.tracking, "{:?}", tr.stats);
        let truth = w.motion.pose(t);
        // the strong pose's place, to what one sensor sees of depth, not the track 30 cm off
        let off = (o.world_from_model.translation.vector - truth.translation.vector).norm();
        assert!(off < 0.05, "{off} m off");
    }

    /// A strong engine pose with too few labelled LEDs to acquire from settles nothing: the
    /// unconfirmed track stays, rather than being dropped with nothing to take its place.
    #[test]
    fn a_strong_engine_pose_without_enough_labels_settles_nothing() {
        let w = world(3);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let c = (0..w.cams.len())
            .max_by_key(|&c| sight(&w, &w.cams[c], 0.45, &mut Rng::new(1), 0.0).len())
            .unwrap();
        let mut rng = Rng::new(3);
        let mut t = 0.0;
        while t < 0.5 {
            t += 0.002;
            tr.imu(imu(&w, t));
        }
        let blobs = sight(&w, &w.cams[c], 0.45, &mut rng, 0.3);
        let good = EnginePose::good(w.motion.pose(0.45));
        tr.frame(
            (1e9 + 0.45e9) as u64,
            c,
            &w.cams[c],
            &blobs,
            Some(good),
            &[],
        );
        assert!(tr.provisional.is_some());
        let next = 0.45 + 1.0 / 52.0;
        while t < next + 0.025 {
            t += 0.002;
            tr.imu(imu(&w, t));
        }
        // a strong pose 30 cm off, its blobs unlabelled
        let mut blobs = sight(&w, &w.cams[c], next, &mut rng, 0.3);
        blobs.iter_mut().for_each(|b| b.led_id = -1);
        let mut far = w.motion.pose(next);
        far.translation.vector.x += 0.3;
        let strong = EnginePose::strong(far);
        tr.frame(
            (1e9 + next * 1e9) as u64,
            c,
            &w.cams[c],
            &blobs,
            Some(strong),
            &[],
        );
        assert!(
            tr.provisional.is_some() && !tr.history.is_empty(),
            "{:?}",
            tr.stats
        );
    }

    /// A strong engine pose that replaces an unconfirmed track starts from the state it held: the
    /// orientation kept from the confirmed track before is not forgotten with it.
    #[test]
    fn replacing_an_unconfirmed_track_keeps_the_kept_orientation() {
        let w = world(3);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let mut rng = Rng::new(3);
        let (mut t, mut next) = (0.0, 0.01);
        // tracked and confirmed by all three sensors for a second
        while t < 1.0 {
            t += 0.002;
            tr.imu(imu(&w, t));
            while next <= t - 0.025 {
                for (c, cam) in w.cams.iter().enumerate() {
                    let blobs = sight(&w, cam, next, &mut rng, 0.3);
                    let engine = EnginePose::strong(w.motion.pose(next));
                    tr.frame((1e9 + next * 1e9) as u64, c, cam, &blobs, Some(engine), &[]);
                }
                next += 1.0 / 52.0;
            }
        }
        assert!(tr.tracking() && tr.last_confirmed != 0);
        // as if the track were an unconfirmed one, 30 cm off
        tr.provisional = Some(tr.last_fix);
        for (_, f) in tr.history.iter_mut() {
            f.x.p += Vector3::new(0.3, 0.0, 0.0);
        }
        let c = (0..w.cams.len())
            .max_by_key(|&c| sight(&w, &w.cams[c], next, &mut Rng::new(1), 0.0).len())
            .unwrap();
        while t < next + 0.025 {
            t += 0.002;
            tr.imu(imu(&w, t));
        }
        let blobs = sight(&w, &w.cams[c], next, &mut rng, 0.3);
        let engine = EnginePose::strong(w.motion.pose(next));
        let out = tr.frame(
            (1e9 + next * 1e9) as u64,
            c,
            &w.cams[c],
            &blobs,
            Some(engine),
            &[],
        );
        assert!(matches!(out, Outcome::Acquired(_)), "{out:?}");
        assert!(tr.tracking() && tr.last_confirmed != 0, "{:?}", tr.stats);
    }

    /// Dropping a track no second sensor confirmed keeps the biases learned before it: they go
    /// back to what that track started from, not to the first track's.
    #[test]
    fn an_unconfirmed_drop_keeps_the_biases_learned_before() {
        let w = touch_world(9);
        let (tr, _) = run_with(
            &w,
            9.0,
            0.3,
            &Run {
                seed: Some((Vector3::zeros(), Vector3::zeros())),
                blackout: Some((3.0, 6.5)),
                engine_silent_after: Some(3.0),
                cameras_after: Some((6.5, 1)),
                ..Run::default()
            },
        );
        assert!(tr.stats.unconfirmed_acquisitions > 0, "{:?}", tr.stats);
        let kept = tr.saved_bias.unwrap().x.ba;
        assert!(
            (kept - w.ba).norm() < w.ba.norm() / 2.0,
            "kept {kept:?}, truth {:?}",
            w.ba
        );
    }

    /// The search is bounded: each sensor's frames are searched at most every
    /// `SearchParams::every_ns`, and a frame with more blobs than `SearchParams::max_blobs` not
    /// at all.
    #[test]
    fn searches_are_rate_limited_and_skip_crowded_frames() {
        let w = touch_world(4);
        let lost = Run {
            blackout: Some((3.0, 6.5)),
            engine_silent_after: Some(3.0),
            turn_memory_at: Some((4.0, Vector3::x() * 60f64.to_radians())),
            ..Run::default()
        };
        // a kept orientation that finds nothing: searched again and again, three sensors
        let (tr, _) = run_with(&w, 9.0, 0.3, &lost);
        let every = Params::default().search.every_ns as f64 * 1e-9;
        let most = 3.0 * (2.5 / every + 1.0);
        let n = tr.stats.searches as f64;
        assert!(n > most / 2.0 && n <= most, "{n} searches, at most {most}");
        let crowded = Run {
            clutter_after: Some((6.5, Params::default().search.max_blobs + 1)),
            ..lost
        };
        let (tr, _) = run_with(&w, 9.0, 0.3, &crowded);
        assert_eq!(tr.stats.searches, 0);
    }

    /// A track ending, by a radio gap or by being dropped, restarts the wait on an ambiguous
    /// acquisition: a timer left from before would let the next ambiguous one through at once.
    #[test]
    fn a_track_ending_restarts_the_ambiguity_timer() {
        let w = world(3);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        tr.imu(imu(&w, 0.1));
        tr.ambiguous_since = 123;
        tr.imu(imu(&w, 0.5));
        assert_eq!((tr.stats.imu_gaps, tr.ambiguous_since), (1, 0));
        tr.ambiguous_since = 456;
        tr.forget_track();
        assert_eq!(tr.ambiguous_since, 0);
    }

    /// After a blackout longer than the re-lock window the engine first reports a flipped pose;
    /// the gyro-held orientation refuses it, and the right one is taken when it comes.
    #[test]
    fn a_flipped_engine_pose_is_refused_at_reacquisition() {
        let w = world(7);
        // longer than the re-lock window, so the engine's pose is needed
        let (tr, errors) = run_with(
            &w,
            10.0,
            0.3,
            &Run {
                blackout: Some((3.0, 6.5)),
                engine_flipped: Some((3.0, 7.0)),
                ..Run::default()
            },
        );
        assert!(tr.stats.refused_acquisitions > 0, "{:?}", tr.stats);
        assert_eq!(tr.stats.acquisitions, 2, "{:?}", tr.stats);
        assert!(pct(&errors, 0.99) < 5e-3, "{:?}", tr.stats);
    }

    /// The controller sleeps 20 s and is turned a quarter turn before it wakes (its clock jumps);
    /// the state is dropped rather than trusted, and the engine's pose is taken at once.
    #[test]
    fn a_wake_after_sleep_reacquires_at_once() {
        let w = world(8);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let mut rng = Rng::new(2);
        let feed =
            |tr: &mut Tracker, rng: &mut Rng, t0: f64, t1: f64, clock_shift: f64, turn: f64| {
                let mut t = t0;
                let mut next_frame = t0;
                let turned = |t: f64| {
                    let mut p = w.motion.pose(t);
                    p.rotation =
                        UnitQuaternion::from_scaled_axis(Vector3::new(0.0, turn, 0.0)) * p.rotation;
                    p
                };
                while t < t1 {
                    t += 0.002;
                    let mut s = imu(&w, t);
                    s.t = (s.t as f64 + clock_shift * 1e9) as u64;
                    s.gyro = w.motion.gyro(t) + w.bg; // body rates do not change with a turn of the world
                    s.accel = turned(t).rotation.inverse()
                        * (w.motion.pose(t).rotation * (s.accel - w.ba))
                        + w.ba;
                    tr.imu(s);
                    while next_frame <= t - 0.025 {
                        for (c, cam) in w.cams.iter().enumerate() {
                            let pose = turned(next_frame);
                            let blobs: Vec<Blob> = (0..w.body.leds.len())
                                .filter_map(|i| {
                                    let pc = cam.world_from_cam.inverse()
                                        * (pose * Point3::from(w.body.leds[i].pos));
                                    let n = cam.world_from_cam.inverse().rotation
                                        * (pose.rotation * w.body.leds[i].dir);
                                    (n.dot(&(-pc.coords)) / pc.coords.norm() > 0.3)
                                        .then(|| {
                                            cam.model.project(&pc.coords).map(|uv| Blob {
                                                xy: uv
                                                    + Vector2::new(rng.gauss(), rng.gauss()) * 0.3,
                                                led_id: (1 << 8) | i as i32,
                                            })
                                        })
                                        .flatten()
                                })
                                .collect();
                            tr.frame(
                                (1e9 + (next_frame + clock_shift) * 1e9) as u64,
                                c,
                                cam,
                                &blobs,
                                Some(EnginePose::strong(pose)),
                                &[],
                            );
                        }
                        next_frame += 1.0 / 52.0;
                    }
                }
            };
        feed(&mut tr, &mut rng, 0.0, 3.0, 0.0, 0.0);
        assert!(tr.tracking());
        // asleep from 3 s to 23 s; the clock comes back only 0.7 s on; meanwhile a quarter turn
        feed(
            &mut tr,
            &mut rng,
            23.0,
            24.0,
            -19.3,
            std::f64::consts::FRAC_PI_2,
        );
        assert_eq!(tr.stats.imu_gaps, 1, "{:?}", tr.stats);
        assert!(
            tr.tracking() && tr.stats.refused_acquisitions == 0,
            "{:?}",
            tr.stats
        );
        let o = tr.output().unwrap();
        let mut truth = w.motion.pose(24.0);
        truth.rotation =
            UnitQuaternion::from_scaled_axis(Vector3::new(0.0, std::f64::consts::FRAC_PI_2, 0.0))
                * truth.rotation;
        let err = o.world_from_model.rotation.angle_to(&truth.rotation);
        // (the turned scene's accelerometer ignores the turned lever-arm path: a degree is noise)
        assert!(
            err < 2f64.to_radians(),
            "rotation error {err} {:?}",
            tr.stats
        );
    }

    /// Picked up after sleeping (no orientation to check against), and the engine's labels are off
    /// by one LED around the ring: its pose is the real one turned ~28 degrees about the ring
    /// axis, and fits the blobs almost as well. The tracker tests the turn back on the frame and
    /// locks on the true pose instead of running on the wrong one until its bias runs off. Every
    /// other frame the engine has labels but no confident pose to show they are off.
    #[test]
    fn an_engine_pose_off_by_one_led_around_the_ring_is_turned_back() {
        let mut w = world(9);
        let (leds, imu_pos) = synth::cv1_touch_left();
        w.body = Body { leds, imu_pos };
        let alias = vision::ring_aliases(&w.body.leds)[0].clone();
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let mut rng = Rng::new(3);
        let (mut t, mut next_frame, mut worst_after_lock) = (0.0, 0.0, 0.0f64);
        while t < 4.0 {
            t += 0.002;
            tr.imu(imu(&w, t));
            while next_frame <= t - 0.025 {
                let truth = w.motion.pose(next_frame);
                let engine = truth * alias.turn.inverse();
                for (c, cam) in w.cams.iter().enumerate() {
                    // the true blobs, labelled as the LEDs the engine's turned pose puts there
                    let blobs: Vec<Blob> = sight(&w, cam, next_frame, &mut rng, 0.3)
                        .into_iter()
                        .map(|b| {
                            let j = (b.led_id & 0xff) as usize;
                            let i = alias.led_at.iter().position(|x| *x == Some(j));
                            Blob {
                                led_id: i.map_or(-1, |i| (1 << 8) | i as i32),
                                ..b
                            }
                        })
                        .collect();
                    let frame_no = (next_frame * 52.0).round() as usize;
                    tr.frame(
                        (1e9 + next_frame * 1e9) as u64,
                        c,
                        cam,
                        &blobs,
                        (frame_no % 2 == 0).then_some(EnginePose::strong(engine)),
                        &[],
                    );
                }
                next_frame += 1.0 / 52.0;
            }
            if let Some(o) = tr.output().filter(|o| o.tracking) {
                let truth = w.motion.pose((o.t as f64 - 1e9) * 1e-9);
                worst_after_lock =
                    worst_after_lock.max(o.world_from_model.rotation.angle_to(&truth.rotation));
            }
        }
        assert!(
            tr.tracking() && tr.stats.acquisitions >= 1,
            "{:?}",
            tr.stats
        );
        assert_eq!(tr.stats.wrong_locks, 0, "{:?}", tr.stats);
        assert!(tr.stats.alias_turns >= 1, "{:?}", tr.stats);
        assert!(
            worst_after_lock < 3f64.to_radians(),
            "worst {:.1} deg {:?}",
            worst_after_lock.to_degrees(),
            tr.stats
        );
    }

    /// Lost (no frames for 3 s, the engine silent, no re-lock): the published position stays
    /// where it was last tracked, at rest, while the orientation keeps following the gyro.
    #[test]
    fn a_lost_position_is_held_not_dead_reckoned() {
        let w = world(10);
        let mut tr = Tracker::new(
            1,
            w.body.clone(),
            Params {
                relock_ns: 0,
                ..Params::default()
            },
            w.cams.len(),
        );
        let mut rng = Rng::new(4);
        let mut t = 0.0;
        let mut next = 0.01;
        let mut held = None;
        while t < 6.0 {
            t += 0.002;
            tr.imu(imu(&w, t));
            while next <= t - 0.025 && next < 3.0 {
                for (c, cam) in w.cams.iter().enumerate() {
                    let blobs = sight(&w, cam, next, &mut rng, 0.3);
                    tr.frame(
                        (1e9 + next * 1e9) as u64,
                        c,
                        cam,
                        &blobs,
                        Some(EnginePose::strong(w.motion.pose(next))),
                        &[],
                    );
                }
                next += 1.0 / 52.0;
            }
            if t > 3.6 {
                let o = tr.output().unwrap();
                assert!(!o.tracking && o.vel.norm() == 0.0);
                let p = o.world_from_model.translation.vector;
                assert_eq!(*held.get_or_insert(p), p, "the held position moved");
                assert!(
                    o.world_from_model
                        .rotation
                        .angle_to(&w.motion.pose(t).rotation)
                        < 0.02,
                    "the orientation stopped following the gyro"
                );
            }
        }
    }

    #[test]
    fn an_accepted_update_ends_a_run_of_refusals() {
        let w = world(11);
        let (mut tr, _) = run_with(&w, 2.0, 0.3, &Run::default());
        tr.refusing_since = 123;
        let te = tr.newest_imu().unwrap() - 30_000_000;
        let tsec = (te as f64 - 1e9) * 1e-9;
        let mut rng = Rng::new(1);
        let blobs = sight(&w, &w.cams[0], tsec, &mut rng, 0.3);
        assert!(matches!(
            tr.frame(te, 0, &w.cams[0], &blobs, None, &[]),
            Outcome::Updated(_)
        ));
        assert_eq!(tr.refusing_since, 0);
    }

    /// Biases remembered from an earlier run are right from the first fix; learned from scratch,
    /// the accelerometer's are still off a few hundred milliseconds in.
    #[test]
    fn remembered_biases_hold_from_the_first_fix() {
        let w = world(5);
        let seeded = Run {
            seed: Some((
                w.bg + Vector3::new(5e-4, 0.0, -5e-4),
                w.ba + Vector3::new(0.01, -0.01, 0.0),
            )),
            ..Run::default()
        };
        let (tr, _) = run_with(&w, 0.3, 0.3, &seeded);
        let x = tr.history.back().unwrap().1.x;
        assert!(
            (x.ba - w.ba).norm() < 0.025 && (x.bg - w.bg).norm() < 2e-3,
            "seeded: bg {:?} ba {:?}",
            x.bg,
            x.ba
        );
        let (tr, _) = run(&w, 0.3, 0.3, None);
        let x = tr.history.back().unwrap().1.x;
        assert!(
            (x.ba - w.ba).norm() > 0.05,
            "learned from scratch already: ba {:?}",
            x.ba
        );
    }

    /// An accelerometer bias the filter drives far from where it started is a wrong lock (a pose
    /// that fits the LEDs but not gravity): the track is dropped and the biases it corrupted are not
    /// kept.
    #[test]
    fn a_runaway_accelerometer_bias_drops_the_track() {
        let w = world(5);
        // from a cold start the bias is free to run (acquisition sd 0.5 m/s^2)
        let opts = Run {
            accel_jump: Some((0.0, Vector3::new(3.0, 0.0, 0.0))),
            ..Run::default()
        };
        let (tr, _) = run_with(&w, 2.0, 0.3, &opts);
        assert!(tr.stats.wrong_locks >= 1, "{:?}", tr.stats);
        assert!(
            tr.saved_bias.is_none_or(|f| f.x.ba.norm() < 0.6),
            "kept the runaway bias {:?}",
            tr.saved_bias.map(|f| f.x.ba)
        );
    }

    /// After a wrong lock the engine tends to offer the same wrong pose again: poses agreeing with
    /// the dropped orientation (carried on by the gyro) are refused for a while.
    #[test]
    fn a_dropped_orientation_is_refused_for_a_while() {
        let w = world(5);
        let (tr, errors) = run_with(
            &w,
            4.0,
            0.3,
            &Run {
                drop_at: Some(1.5),
                ..Run::default()
            },
        );
        assert_eq!(tr.stats.wrong_locks, 1);
        assert!(tr.stats.refused_acquisitions > 0, "{:?}", tr.stats);
        assert_eq!(tr.stats.acquisitions, 2, "{:?}", tr.stats);
        assert!(tr.tracking(), "tracking again once the memory expired");
        // no tracked outputs between the drop and the memory's end
        assert!(
            errors.len()
                < ((4.0 - 1.0 - 2.0) / 0.002) as usize + 50 + ((1.5 - 1.0) / 0.002) as usize,
            "{} outputs",
            errors.len()
        );
    }

    /// A controller lying still does not drift while the sensors lose it, even with its
    /// accelerometer bias misjudged: the IMU itself says it is still.
    #[test]
    fn a_still_controller_holds_still_out_of_sight() {
        let mut w = world(5);
        w.motion = Motion::still(Vector3::new(0.1, 1.0, -0.2), [0.3, -0.5, 0.2]);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        tr.seed_biases(w.bg, w.ba + Vector3::new(0.1, 0.0, -0.05), 1e-4, 1e-3);
        let mut rng = Rng::new(3);
        let (dt, frame) = (0.002, 1.0 / 52.0);
        let (mut t, mut next) = (0.0, 0.01);
        while t < 3.5 {
            t += dt;
            tr.imu(imu(&w, t));
            // sensors see it for 2 s, then lose it
            while next <= t && next < 2.0 {
                for (c, cam) in w.cams.iter().enumerate() {
                    let blobs = sight(&w, cam, next, &mut rng, 0.3);
                    tr.frame(
                        (1e9 + next * 1e9) as u64,
                        c,
                        cam,
                        &blobs,
                        Some(EnginePose::strong(w.motion.pose(next))),
                        &[],
                    );
                }
                next += frame;
            }
        }
        let f = tr.history.back().unwrap().1;
        let drift = (tr.body.world_from_model(&f.x).translation.vector
            - w.motion.pose(t).translation.vector)
            .norm();
        assert!(
            drift < 0.005,
            "drifted {:.1} mm in 1.5 s out of sight",
            drift * 1e3
        );
    }

    /// Lying on its side, still: the synthetic ring looks exactly the same turned 45 degrees about
    /// its axis (three LEDs on), and the engine offers that turn with its labels to match. Nothing
    /// in a frame tells them apart; gravity does, and the turned pose is refused.
    #[test]
    fn a_still_controller_refuses_a_pose_that_disagrees_with_gravity() {
        let mut w = world(6);
        w.motion = Motion::still(
            Vector3::new(0.1, 1.0, -0.2),
            [0.0, 0.0, std::f64::consts::FRAC_PI_2],
        );
        let turned = |t: f64| {
            w.motion.pose(t)
                * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 45f64.to_radians())
        };
        let n = w.body.leds.len();
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let mut rng = Rng::new(4);
        let (mut t, mut next) = (0.0, 0.5);
        while t < 2.0 {
            t += 0.002;
            tr.imu(imu(&w, t));
            while next <= t {
                for (c, cam) in w.cams.iter().enumerate() {
                    let blobs: Vec<Blob> = sight(&w, cam, next, &mut rng, 0.3)
                        .into_iter()
                        .map(|b| Blob {
                            led_id: (1 << 8) | (((b.led_id & 0xff) as usize + 3) % n) as i32,
                            ..b
                        })
                        .collect();
                    tr.frame(
                        (1e9 + next * 1e9) as u64,
                        c,
                        cam,
                        &blobs,
                        Some(EnginePose::strong(turned(next))),
                        &[],
                    );
                }
                next += 1.0 / 52.0;
            }
        }
        assert_eq!(tr.stats.acquisitions, 0, "{:?}", tr.stats);
        assert!(tr.stats.refused_acquisitions > 0, "{:?}", tr.stats);
        // the true pose, still lying there, is taken at once
        while t < 2.5 {
            t += 0.002;
            tr.imu(imu(&w, t));
            while next <= t {
                for (c, cam) in w.cams.iter().enumerate() {
                    let blobs = sight(&w, cam, next, &mut rng, 0.3);
                    tr.frame(
                        (1e9 + next * 1e9) as u64,
                        c,
                        cam,
                        &blobs,
                        Some(EnginePose::strong(w.motion.pose(next))),
                        &[],
                    );
                }
                next += 1.0 / 52.0;
            }
        }
        assert!(
            tr.tracking() && tr.stats.acquisitions == 1 && tr.stats.wrong_locks == 0,
            "{:?}",
            tr.stats
        );
    }

    #[test]
    fn biases_converge() {
        let w = world(5);
        let (tr, _) = run(&w, 20.0, 0.3, None);
        let x = tr.history.back().unwrap().1.x;
        assert!(
            (x.bg - w.bg).norm() < 2e-3 && (x.ba - w.ba).norm() < 0.05,
            "bg {:?} ba {:?}",
            x.bg,
            x.ba
        );
    }
    #[test]
    fn dynamic_sensors_high_ids_confirm_and_receive_diagnostics() {
        let mut w = world(5);
        w.motion = Motion::still(Vector3::new(0.1, 1.0, -0.2), [0.3, -0.5, 0.2]);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), 65);
        let mut rng = Rng::new(3);
        let mut accepted = [0; 3];
        for tick in 1..400 {
            let t = tick as f64 * 0.002;
            tr.imu(imu(&w, t));
            if tick % 10 == 0 {
                for (slot, camera) in [8, 63, 64].into_iter().enumerate() {
                    let cam = &w.cams[slot];
                    let blobs = sight(&w, cam, t, &mut rng, 0.0);
                    if matches!(
                        tr.frame(
                            (1e9 + t * 1e9) as u64,
                            camera,
                            cam,
                            &blobs,
                            Some(EnginePose::strong(w.motion.pose(t))),
                            &[]
                        ),
                        Outcome::Acquired(_) | Outcome::Updated(_)
                    ) {
                        accepted[slot] += 1;
                    }
                }
            }
        }
        assert!(accepted.iter().all(|n| *n > 2), "{accepted:?}");
        for camera in [8, 63, 64] {
            assert!(
                tr.agreement.get(camera).is_some_and(|a| a.samples > 0),
                "camera {camera}"
            );
        }
        assert!(tr.last_confirmed > 0);
    }

    #[test]
    fn dynamic_sensors_63_and_64_are_distinct_confirmations() {
        let mut w = world(5);
        w.motion = Motion::still(Vector3::new(0.1, 1.0, -0.2), [0.3, -0.5, 0.2]);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), 65);
        let mut rng = Rng::new(3);
        for tick in 1..400 {
            let t = tick as f64 * 0.002;
            tr.imu(imu(&w, t));
            if tick % 10 == 0 {
                let slot = usize::from(tick >= 200);
                let camera = 63 + slot;
                let cam = &w.cams[slot];
                let blobs = sight(&w, cam, t, &mut rng, 0.0);
                tr.frame(
                    (1e9 + t * 1e9) as u64,
                    camera,
                    cam,
                    &blobs,
                    Some(EnginePose::strong(w.motion.pose(t))),
                    &[],
                );
            }
            if tick == 199 {
                assert!(tr.stats.updates > 2, "single sensor never tracked");
                assert_eq!(tr.last_confirmed, 0, "one sensor cannot confirm itself");
            }
        }
        assert!(
            tr.last_confirmed > 0,
            "second high-index sensor must confirm the first"
        );
    }

    #[test]
    fn dynamic_sensors_unknown_id_does_not_allocate_or_enter_tracking() {
        let w = world(5);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let count = tr.agreement.len();
        let out = tr.frame(1, usize::MAX, &w.cams[0], &[], None, &[]);
        assert_eq!(out, Outcome::UnknownCamera);
        assert_eq!(tr.agreement.len(), count);
        assert_eq!(tr.stats.frames, 0);
    }
}
