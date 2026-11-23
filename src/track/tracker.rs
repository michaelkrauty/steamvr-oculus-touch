// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! One controller's tracker. The filter keeps a short history of states so that a sensor frame,
//! which arrives tens of milliseconds after its exposure, is applied at the exposure time and the
//! IMU samples since are replayed. Acquisition takes the engine's pose and labels; tracking then
//! associates blobs itself. Output is the newest IMU-propagated state, unsmoothed.

use std::collections::VecDeque;

use nalgebra::{Isometry3, Matrix2, Matrix3, UnitQuaternion, Vector2, Vector3};

use super::eskf::{Cov, Filter, Noise, Nominal, BA, BG, POS, ROT, VEL};
use super::factory::{Gyro, GYRO_PER_COUNT};
use super::search::{self, SearchParams};
use super::vision::{self, Blob, Body, Camera, FitParams};

/// A first lock not yet told from its ring turns (see `Tracker::undecided`).
#[derive(Debug, Clone, PartialEq)]
struct Undecided {
    /// The exposure the track was first acquired from; replacing it keeps this.
    since: u64,
    /// The other hypotheses as turns of the track, in the model frame.
    turns: Vec<Isometry3<f64>>,
    /// Each one's cost less the track's, summed over the frames since `since`.
    lead: Vec<f64>,
}

/// The two cheapest costs in total_cmp order, including signed zero and NaNs.
fn retain_two_costs(costs: &mut [Option<f64>; 2], cost: f64) {
    if costs[0].is_none_or(|first| cost.total_cmp(&first).is_lt()) {
        costs[1] = costs[0];
        costs[0] = Some(cost);
    } else if costs[1].is_none_or(|second| cost.total_cmp(&second).is_lt()) {
        costs[1] = Some(cost);
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    /// Controller clock, ns.
    pub t: u64,
    /// Body rate, rad/s, and specific force, m/s^2, factory-calibrated.
    pub gyro: Vector3<f64>,
    pub accel: Vector3<f64>,
    /// Per gyro axis, the turn rate (rad/s, body frame) its reading at its rail may miss over
    /// the interval this sample ends, set by `Tracker::imu` (see `Tracker::own_rail`): none
    /// while no axis reads at its rail.
    pub hidden: Option<Matrix3<f64>>,
}

/// A gyro axis reading at its rail. Its reading approached the rail at `slope` counts/s and
/// crossed it at `crossed` (controller time, s), and the true rate is taken to rise on as fast.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Clip {
    crossed: f64,
    slope: f64,
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
    /// After refusing the engine's poses for this long, the orientation memory is dropped, once
    /// two sensors' refused poses agree (see `Tracker::same_hypothesis`): then it is the one that
    /// is wrong. One sensor's alone can be the other controller's blobs under this one's labels.
    pub max_refusing_ns: u64,
    /// Blobs this close to an LED of the other controller (as it is tracked at the same exposure)
    /// are left to it, unless the engine labelled them as ours (see `ForeignLed`).
    pub foreign_px: f64,
    /// For this long after the last fix the dead-reckoned state is close enough for the
    /// tracker's own (shift-search) association to re-lock without the engine.
    pub relock_ns: u64,
    /// A wrong lock (a pose that fits the LEDs but is turned from the truth) leaks gravity into the
    /// filter, which drives the accelerometer bias off: beyond this (m/s^2) the track is dropped,
    /// and no track starts from such a bias.
    /// Tested on the state that holds all of an exposure's fixes, once a later exposure's frame
    /// comes (or the IMU passes the exposure's last possible frame): between two sensors' fixes
    /// of one exposure the bias swings with the order they arrive in.
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
    /// acquisition waits for a clearer frame (for up to `max_refusing_ns`). The track taken is
    /// not published until it also leads the others by `alias_margin` over its frames from every
    /// sensor, each hypothesis matched from its own pose (see `Tracker::undecided`).
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
    /// IMU samples received, and those the radio lost between two received ones less than
    /// `max_imu_gap_ns` apart (a longer silence is counted in `imu_gaps`).
    pub imu_samples: u64,
    pub imu_lost: u64,
    /// Re-locks after the position was lost, by the tracker's own association.
    pub relocks: u64,
    /// Tracks dropped for a runaway accelerometer bias.
    pub wrong_locks: u64,
    /// Tracks refuted by the engine's strong poses (see `Tracker::refutes`) and acquired afresh.
    pub refuted: u64,
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
    /// Native camera/IMU relative motion identified this hand independently of calibration.
    /// Agreement may use it through a disputed camera pose; fusion still checks its attitude.
    pub identified: bool,
}

impl EnginePose {
    pub fn strong(world_from_model: Isometry3<f64>) -> EnginePose {
        EnginePose {
            world_from_model,
            strong: true,
            identified: false,
        }
    }

    pub fn good(world_from_model: Isometry3<f64>) -> EnginePose {
        EnginePose {
            world_from_model,
            strong: false,
            identified: false,
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
    /// Standard deviations from the filter's covariance (`published_stddev`): of the model
    /// origin's position along the world axes, m, and of the orientation about the model's own
    /// axes, rad. While the position is not `tracking`, the position's is that of the dead
    /// reckoning, not of the held position.
    pub pos_stddev: Vector3<f64>,
    pub rot_stddev: Vector3<f64>,
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

/// An LED of the other controller facing the sensor (as the association judges facing), where its
/// track puts it in the frame; only those within `AssocParams::max_px` of a blob. A match on a
/// blob within its association gate (`AssocParams::gate`, under its covariance) is evidence of
/// this controller only if this controller's prediction (its track, or the dead-reckoned state a
/// re-lock is fitted from) explains the blob better. Each prediction is judged by its own
/// covariance, so an uncertain one loses a near tie to an established one. An acquisition has no
/// prediction to judge the blobs by, since a pose fitted to them always explains them: there a
/// match on a blob within `foreign_px` of one is no evidence of this controller while the other
/// is in this sensor's view.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ForeignLed {
    pub uv: Vector2<f64>,
    /// The prediction's pixel covariance (see `vision::predicted_near`).
    pub cov: Matrix2<f64>,
    /// The other controller took an update from this sensor within the history. One hidden
    /// behind this controller on the sensor's line of sight did not, and its LEDs, predicted
    /// where this one's blobs are, say nothing of them.
    pub seen: bool,
}

/// A blob at `xy` under the prediction `uv` with covariance `cov`: its Mahalanobis^2, and its
/// negative log-likelihood (less a constant).
fn prediction_fit(uv: Vector2<f64>, cov: &Matrix2<f64>, xy: Vector2<f64>) -> (f64, f64) {
    let d = xy - uv;
    cov.try_inverse()
        .map_or((f64::INFINITY, f64::INFINITY), |s| {
            let d2 = (d.transpose() * s * d)[0];
            (d2, d2 + cov.determinant().ln())
        })
}

/// How well one sensor agrees with the others: an exponentially weighted mean of the median
/// pixel residual of its matched LEDs under a prior the other sensors set (another sensor updated
/// the track within `CROSS_CHECK_NS`). A sensor that moved since calibration stands out: a little,
/// and its residual and bias grow; a lot, and it matches nothing under that prior although the
/// controller is in its frame. `observed_checks` and `observed_misses` need matched LEDs or the
/// engine's labels to establish that presence. Unlabelled blobs where the track predicts LEDs
/// add only to `checks` and `misses`: a reason to ask for labels, not evidence of movement.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CameraAgreement {
    pub median_px: f64,
    /// Weighted mean of the per-frame mean residual vector: noise averages out of it, a moved
    /// sensor's consistent offset does not.
    pub bias_px: Vector2<f64>,
    pub samples: u64,
    /// Frames in which the controller was seen or predicted in view among enough blobs while
    /// the other sensors had just set the prior, and of those, the frames that matched nothing.
    pub checks: u64,
    pub misses: u64,
    /// Checks supported by actual LED correspondences: an accepted fit, or the engine's labels
    /// showing this controller even though its LEDs did not fit the prior. A frame full of
    /// unlabelled blobs alone is not evidence that the controller was visible.
    pub observed_checks: u64,
    pub observed_misses: u64,
}

pub const CROSS_CHECK_NS: u64 = 25_000_000;
const AGREEMENT_WEIGHT: f64 = 1.0 / 500.0;
/// A frame with this many blobs where the track puts this many LEDs in view may hold the
/// controller. Without matches or engine labels it warrants inspection, not a movement verdict.
const CHECK_MIN_BLOBS: usize = 5;
/// A controller nearer the sensor than this (metres) is no check of its pose: that close, blobs
/// merge and a millimetre of the prior is pixels.
const CHECK_MIN_DEPTH_M: f64 = 0.6;

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
    /// Where the controller's gyro readings end, from its factory calibration; without it no
    /// reading is taken for one at its rail.
    pub gyro_range: Option<Gyro>,
    /// Per gyro axis, the stretch it has read at its rail.
    clips: [Option<Clip>; 3],
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
    /// In that run, each sensor's newest refused pose, as its turn (world frame) from the kept
    /// orientation: the gyro carries the kept orientation, and with it the pose, so the turn holds
    /// while the controller moves.
    refused_by: Vec<(usize, UnitQuaternion<f64>)>,
    /// The exposure of an acquisition no second sensor has confirmed yet, one from an engine pose
    /// rated only good or from the tracker's own search (`search_kept`): either can fit one view
    /// and be wrong. Until a second sensor confirms it the track takes updates but is not
    /// published; if none has within `SearchParams::confirm_within_ns`, the track is dropped, and
    /// with it the kept orientation.
    provisional: Option<u64>,
    /// A track acquired with no orientation to check it against, while the engine's pose or one
    /// of its ring turns still explains the frames about as well as the track: one frame can
    /// favour an alias (the engine itself locks onto one), and gravity cannot settle it while the
    /// controller is being picked up. Until the track leads every other hypothesis by
    /// `alias_margin`, summed over the frames it takes, it takes updates but is not published; a
    /// hypothesis that leads it by as much takes its place. `max_refusing_ns` after the first
    /// acquisition it is published as it stands, as an ambiguous acquisition is taken.
    undecided: Option<Undecided>,
    /// The model origin where it was last tracked (held while the position is lost).
    held: Option<Vector3<f64>>,
    /// The newest exposure with fixes applied whose state the bias test has not yet seen.
    bias_pending: Option<u64>,
    /// The biases the current track started from, restored when it turns out a wrong lock.
    acquired_bias: Option<Filter>,
    /// A dropped wrong lock's orientation, carried on by the gyro: (controller time, orientation,
    /// refused until).
    wrong_lock: Option<(u64, UnitQuaternion<f64>, u64)>,
    /// The LED constellation's ring aliases, tested at acquisition.
    aliases: Vec<vision::Alias>,
    /// The last frame (sensor, exposure) that refuted the track (see `refutes`), and the fix it
    /// refuted.
    refuted: Option<(usize, u64, u64)>,
    /// When the current run of acquisitions put off as ambiguous began (0: none).
    ambiguous_since: u64,
    /// The engine's labels for this controller come from a pose that disagrees with the track
    /// (the last engine pose shown did, or the track was acquired turned from it): ignored as
    /// hints until an engine pose agrees again. Its labels carry on from its own pose in frames
    /// where it shows none.
    engine_labels_off: bool,
    /// Accepted exposure times per sensor within the filter history. Long-search frames may
    /// arrive after newer peer frames, so the newest timestamp alone cannot check their prior.
    camera_updates: Vec<VecDeque<u64>>,
    inspection_frames: VecDeque<InspectionFrame>,
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

/// Only native searches retain an exposure beyond the ordinary filter history. The native
/// sensor has four capture buffers; keeping four snapshots per camera bounds dropped callbacks.
#[derive(Debug, Clone)]
struct InspectionFrame {
    camera: usize,
    exposure: u64,
    until: u64,
    prior: Filter,
    peers: bool,
}

/// The controller's IMU sample period.
const IMU_PERIOD_NS: u64 = 2_000_000;

/// An interval this long or longer between consecutive IMU samples has at least one sample
/// missing. A lone 3 ms step is stamp jitter, paired with a 1 ms one, not a loss.
const HOLE_MIN_NS: u64 = 2 * IMU_PERIOD_NS - IMU_PERIOD_NS / 4;

/// The samples missing from an interval between two received IMU samples, by `HOLE_MIN_NS`'s
/// rule: a 3 ms step is jitter, a 4 ms one has lost a sample.
pub fn samples_lost_in(interval_ns: u64) -> u64 {
    ((interval_ns + IMU_PERIOD_NS / 4) / IMU_PERIOD_NS).saturating_sub(1)
}

/// What a stretch of lost IMU samples hides once it is bridged by the straight line between the
/// samples on either side: the measured p95 of the velocity (m/s) and rotation (rad) the lost
/// samples would have added beyond that line, by the stretch's length T (s) and by the rate of
/// change entering it, the jerk `ja` (m/s^3) and angular acceleration `jw` (rad/s^2) between the
/// two samples before it:
///   p95 |dv| = T^2 (HOLE_VEL[0] + HOLE_VEL[1] ja)
///   p95 |dθ| = T^2 (HOLE_ROT[0] + HOLE_ROT[1] jw) + T^3 (HOLE_ROT[2] + HOLE_ROT[3] jw)
/// The filter's per-axis sd for the stretch is that p95 over `P95_PER_AXIS`, the p95 of the
/// magnitude of an isotropic 3-D Gaussian in units of its per-axis sd.
const HOLE_VEL: [f64; 2] = [5.469, 0.1708];
const HOLE_ROT: [f64; 4] = [0.4293, 0.09208, 6.832, 3.9];
const P95_PER_AXIS: f64 = 2.795;

/// Per-axis variances of velocity (m^2/s^2) and rotation (rad^2) that the stretch from `prev`
/// to `s` hides (see `HOLE_VEL`), `before` being the sample before `prev`.
fn hole_variance(before: Option<&ImuSample>, prev: &ImuSample, s: &ImuSample) -> (f64, f64) {
    let span = (s.t - prev.t) as f64 * 1e-9;
    let (ja, jw) = before.filter(|b| b.t < prev.t).map_or((0.0, 0.0), |b| {
        let h = (prev.t - b.t) as f64 * 1e-9;
        (
            (prev.accel - b.accel).norm() / h,
            (prev.gyro - b.gyro).norm() / h,
        )
    });
    let t2 = span * span;
    let vel = t2 * (HOLE_VEL[0] + HOLE_VEL[1] * ja) / P95_PER_AXIS;
    let rot = (t2 * (HOLE_ROT[0] + HOLE_ROT[1] * jw)
        + t2 * span * (HOLE_ROT[2] + HOLE_ROT[3] * jw))
        / P95_PER_AXIS;
    (vel * vel, rot * rot)
}

/// Advances `f` from `t0` to `t1`, inside the interval that sample `s` ends; `prev` is the sample
/// before it and `before` the one before that. An interval of about one period holds `s` over
/// it, as `Filter::propagate` does. One with samples missing runs along the straight line from
/// `prev` to `s` over the missing part, then holds `s` over its own last period; and the filter
/// takes on what the missing part hides (`HOLE_VEL`), spread over the missing part.
/// Then it takes on the turn that gyro readings at their rails may hide (`ImuSample::hidden`).
#[allow(clippy::too_many_arguments)]
fn advance(
    f: &mut Filter,
    before: Option<&ImuSample>,
    prev: Option<&ImuSample>,
    s: &ImuSample,
    t0: u64,
    t1: u64,
    g: &Vector3<f64>,
    n: &Noise,
) {
    advance_readings(f, before, prev, s, t0, t1, g, n);
    if let Some(rates) = &s.hidden {
        let dt = t1.saturating_sub(t0) as f64 * 1e-9;
        f.add_hidden_turn(rates, &(s.accel - f.x.ba), dt);
    }
}

/// `advance` as the readings alone tell it.
#[allow(clippy::too_many_arguments)]
fn advance_readings(
    f: &mut Filter,
    before: Option<&ImuSample>,
    prev: Option<&ImuSample>,
    s: &ImuSample,
    t0: u64,
    t1: u64,
    g: &Vector3<f64>,
    n: &Noise,
) {
    let Some(p) = prev.filter(|p| s.t.saturating_sub(p.t) >= HOLE_MIN_NS) else {
        f.propagate(&s.gyro, &s.accel, t1.saturating_sub(t0) as f64 * 1e-9, g, n);
        return;
    };
    let split = s.t - IMU_PERIOD_NS;
    if t0 < split {
        let end = t1.min(split);
        let missing = (split - p.t) as f64;
        let at = |t: u64| (t.saturating_sub(p.t) as f64 / missing).clamp(0.0, 1.0);
        let along = |a: &Vector3<f64>, b: &Vector3<f64>, u: f64| a + (b - a) * u;
        let (u0, u1) = (at(t0), at(end));
        let dt = (end - t0) as f64 * 1e-9;
        // white noise over this piece, carried through it to its end
        let (vel, rot) = hole_variance(before, p, s);
        f.propagate_line(
            &along(&p.gyro, &s.gyro, u0),
            &along(&p.accel, &s.accel, u0),
            &along(&p.gyro, &s.gyro, u1),
            &along(&p.accel, &s.accel, u1),
            dt,
            g,
            n,
            (vel / (missing * 1e-9), rot / (missing * 1e-9)),
        );
    }
    if t1 > split {
        f.propagate(&s.gyro, &s.accel, (t1 - t0.max(split)) as f64 * 1e-9, g, n);
    }
}

impl Tracker {
    pub fn new(device_id: i32, body: Body, params: Params, camera_count: usize) -> Tracker {
        let aliases = vision::ring_aliases(&body.leds);
        Tracker {
            params,
            body,
            device_id,
            gyro_range: None,
            clips: [None; 3],
            history: VecDeque::new(),
            imu: VecDeque::new(),
            still: VecDeque::new(),
            applied: VecDeque::new(),
            last_fix: 0,
            last_track: 0,
            saved_bias: None,
            acquired_bias: None,
            wrong_lock: None,
            aliases,
            refuted: None,
            ambiguous_since: 0,
            engine_labels_off: false,
            track_camera: None,
            track_confirmed: false,
            last_confirmed: 0,
            refusing_since: 0,
            refused_by: Vec::new(),
            provisional: None,
            undecided: None,
            held: None,
            bias_pending: None,
            camera_updates: vec![VecDeque::new(); camera_count],
            inspection_frames: VecDeque::new(),
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
        self.camera_updates.push(VecDeque::new());
        self.searched_at.push(0);
        self.agreement.push(CameraAgreement::default());
    }

    pub(super) fn forget_camera(&mut self, id: usize) {
        self.camera_updates[id].clear();
        self.cancel_inspection(id);
        self.searched_at[id] = 0;
        self.agreement[id] = CameraAgreement::default();
        if self.track_camera == Some(id) {
            self.track_camera = None;
        }
    }

    fn gravity(&self) -> Vector3<f64> {
        Vector3::new(0.0, -self.params.gravity, 0.0)
    }

    pub fn newest_imu(&self) -> Option<u64> {
        self.imu.back().map(|s| s.t)
    }

    /// Whether sensor `camera` updated the track within the history before controller time `t`.
    pub fn seen_by(&self, camera: usize, t: u64) -> bool {
        self.camera_updates.get(camera).is_some_and(|u| {
            u.iter()
                .rev()
                .find(|&&x| x <= t)
                .is_some_and(|&x| t - x <= self.params.history_ns)
        })
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
        self.has_track() && self.provisional.is_none() && !self.undecided_now()
    }

    /// An undecided track (see `undecided`) is published at the latest `max_refusing_ns` after
    /// its acquisition, whatever happened since.
    fn undecided_now(&self) -> bool {
        self.undecided.as_ref().is_some_and(|u| {
            self.history
                .back()
                .is_some_and(|&(t, _)| t.saturating_sub(u.since) <= self.params.max_refusing_ns)
        })
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
        let gap = self
            .imu
            .back()
            .is_some_and(|l| s.t - l.t > self.params.max_imu_gap_ns);
        self.stats.imu_samples += 1;
        if let Some(l) = self.imu.back().filter(|_| !gap) {
            self.stats.imu_lost += samples_lost_in(s.t - l.t);
        }
        // past `history_ns` no frame of the pending exposure can come any more
        if self
            .bias_pending
            .is_some_and(|t| gap || s.t.saturating_sub(t) > self.params.history_ns)
        {
            self.test_pending_bias();
        }
        if gap {
            self.reset_imu_history();
        }
        let s = self.own_rail(s);
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
            let n = self.imu.len();
            let earlier = |k: usize| n.checked_sub(k).and_then(|i| self.imu.get(i));
            advance(
                &mut f,
                earlier(3),
                earlier(2),
                &s,
                t,
                s.t,
                &g,
                &self.params.noise,
            );
            self.zupt(&mut f, t, s.t, still);
            self.history.push_back((s.t, f));
        }
        self.inspection_frames.retain(|f| f.until > s.t);
        let cutoff = s.t.saturating_sub(self.params.history_ns);
        // a frame as old as the history looks back from its exposure for another sensor's update
        // (`others_set_prior`) or a sighting of the track (`seen_by`)
        let kept = cutoff.saturating_sub(self.params.history_ns.max(CROSS_CHECK_NS));
        for updates in &mut self.camera_updates {
            while updates.front().is_some_and(|t| *t < kept) {
                updates.pop_front();
            }
        }
        while self.history.len() > 1 && self.history[1].0 <= cutoff {
            self.history.pop_front();
        }
        let oldest = self.history.front().map_or(cutoff, |h| h.0.min(cutoff));
        // keep the two samples at or before the oldest snapshot: the interval propagating from
        // it ends at the next sample, and a hole there needs both (see `advance`)
        while self.imu.len() > 2 && self.imu[2].t <= oldest {
            self.imu.pop_front();
            self.still.pop_front();
        }
        while self.applied.front().is_some_and(|a| a.t < oldest) {
            self.applied.pop_front();
        }
        self.note_held();
    }

    /// `s` with the turn rate its gyro axes at their rails may miss (`ImuSample::hidden`). While
    /// an axis reads at its rail, the true rate is taken to go on rising as fast as the reading
    /// approached the rail over the two intervals before (with no sample before, by the whole
    /// range in one sample period, the fastest a reading can change), from where that approach
    /// crossed the rail. Each sample is held over the interval it ends, so the rate it misses is
    /// that excess at its time, along that axis's calibrated direction. The filter widens its
    /// rotation about that direction by the turn it hides (`Filter::add_hidden_turn`), so its
    /// uncertainty there is the most the rail can have hidden since the filter last knew the
    /// orientation, and the next fix corrects it at once. With no axis at its rail, nothing
    /// changes.
    fn own_rail(&mut self, mut s: ImuSample) -> ImuSample {
        let Some(gyro) = self.gyro_range else {
            return s;
        };
        let n = self.imu.len();
        let earlier = |k: usize| n.checked_sub(k).and_then(|i| self.imu.get(i));
        let (prev, before) = (earlier(1), earlier(2));
        let dt = prev.map_or(IMU_PERIOD_NS, |p| s.t - p.t) as f64 * 1e-9;
        let now = gyro.counts(&s.gyro);
        let (p, b) = (
            prev.map(|p| gyro.counts(&p.gyro)),
            before.map(|b| gyro.counts(&b.gyro)),
        );
        let mut rates = Matrix3::zeros();
        for k in 0..3 {
            let clip = &mut self.clips[k];
            let Some(end) = Gyro::rail(now[k]) else {
                *clip = None;
                continue;
            };
            let c = clip.get_or_insert_with(|| {
                let toward = end.signum();
                let (Some(prev), Some(p)) = (prev, p) else {
                    return Clip {
                        crossed: s.t as f64 * 1e-9,
                        slope: 65536.0 / (IMU_PERIOD_NS as f64 * 1e-9),
                    };
                };
                let last = toward * (end - p[k]) / dt;
                let slope = b.zip(before).map_or(last, |(b, before)| {
                    last.max(toward * (p[k] - b[k]) / ((prev.t - before.t) as f64 * 1e-9))
                });
                Clip {
                    crossed: prev.t as f64 * 1e-9 + toward * (end - p[k]) / slope,
                    slope,
                }
            });
            let excess = c.slope * (s.t as f64 * 1e-9 - c.crossed);
            rates.set_column(k, &(gyro.matrix.column(k) * (excess * GYRO_PER_COUNT)));
        }
        s.hidden = (rates != Matrix3::zeros()).then_some(rates);
        s
    }

    fn reset_imu_history(&mut self) {
        self.stats.imu_gaps += 1;
        self.clips = [None; 3];
        self.camera_updates.iter_mut().for_each(VecDeque::clear);
        self.inspection_frames.clear();
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
        self.undecided = None;
        self.held = None;
        self.bias_pending = None;
    }

    /// Host time exposed a discontinuity that the controller's elapsed time concealed.
    /// Keep calibration and bias estimates, but no pose or deadline from the old clock fit.
    pub(super) fn reset_clock(&mut self) {
        self.reset_imu_history();
        self.wrong_lock = None;
        self.searched_at.fill(0);
    }

    pub(super) fn clear_inspection_priors(&mut self) {
        self.inspection_frames.clear();
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
        let start = self.imu.partition_point(|x| x.t < from);
        let n = (self.imu.len() - start + 1) as f64;
        let mut win = self.imu.range(start..).chain(std::iter::once(s));
        let gm = win.clone().map(|x| x.gyro).sum::<Vector3<f64>>() / n;
        let am = win.clone().map(|x| x.accel).sum::<Vector3<f64>>() / n;
        #[cfg(test)]
        stillness_tests::observe_means(gm, am);
        (gm - bg).norm() < p.still_gyro
            && ((am - ba).norm() - p.gravity).abs() < p.still_accel_g
            && win.all(|x| {
                (x.gyro - gm).norm() < p.still_gyro_spread
                    && (x.accel - am).norm() < p.still_accel_spread
            })
    }

    /// The zero-velocity update for the step from `t0` to `t1` (see `Params::zupt_sigma`).
    /// The IMU cannot tell a steady glide from rest, so a zero that the filter's own velocity and
    /// its covariance rule out (past the chi-square 99.9 % point for three degrees of freedom) is
    /// not applied: otherwise a slow pure translation out of sight is braked to a stop.
    fn zupt(&self, f: &mut Filter, t0: u64, t1: u64, still: bool) {
        if !still || t1 / self.params.zupt_every_ns == t0 / self.params.zupt_every_ns {
            return;
        }
        let s = f.cov.fixed_view::<3, 3>(VEL, VEL)
            + Matrix3::identity() * self.params.zupt_sigma.powi(2);
        if s.try_inverse()
            .is_some_and(|s_inv| f.x.v.dot(&(s_inv * f.x.v)) < 16.27)
        {
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
    }

    /// Drops the track as a wrong lock when the accelerometer bias it was left with by the pending
    /// exposure's fixes ran off (see `Params::max_accel_bias`).
    fn test_pending_bias(&mut self) {
        self.bias_pending = None;
        if self
            .history
            .back()
            .is_some_and(|(_, f)| self.bias_ran_off(f))
        {
            self.drop_wrong_lock();
        }
    }

    /// Whether `f`'s accelerometer bias is beyond `Params::max_accel_bias`: a wrong lock's.
    fn bias_ran_off(&self, f: &Filter) -> bool {
        f.x.ba.norm() > self.params.max_accel_bias
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
        self.camera_updates.iter_mut().for_each(VecDeque::clear);
        self.inspection_frames.clear();
        self.provisional = None;
        self.undecided = None;
        self.last_fix = 0;
        self.track_camera = None;
        self.track_confirmed = false;
    }

    /// Drops the track, and the orientation memory with it; the biases go back to what the track
    /// started from.
    fn forget_track(&mut self) {
        self.camera_updates.iter_mut().for_each(VecDeque::clear);
        self.inspection_frames.clear();
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
        self.undecided = None;
        self.held = None;
        self.bias_pending = None;
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
        let (pos_stddev, rot_stddev) = published_stddev(&f, &self.body);
        Some(Output {
            t,
            world_from_model: pose,
            vel,
            ang_vel_world: w,
            gyro_bias: f.x.bg,
            accel_bias: f.x.ba,
            tracking,
            last_fix: if tracking { self.last_fix } else { 0 },
            pos_stddev,
            rot_stddev,
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
            let next = self.imu.partition_point(|s| s.t <= tk);
            let s = self.imu.get(next)?;
            let earlier = |k: usize| next.checked_sub(k).and_then(|i| self.imu.get(i));
            advance(
                &mut f,
                earlier(2),
                earlier(1),
                s,
                tk,
                t,
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
        let from = t;
        // the two samples before the first one replayed: the interval it ends, and the rate into it
        let at = self.imu.partition_point(|s| s.t <= from);
        let mut prev = at.checked_sub(1).and_then(|i| self.imu.get(i)).copied();
        let mut before = at.checked_sub(2).and_then(|i| self.imu.get(i)).copied();
        let mut imu = self
            .imu
            .iter()
            .zip(&self.still)
            .filter(|(s, _)| s.t > from)
            .peekable();
        let mut m = 0;
        loop {
            let next_m = later.get(m).map(|a| a.t);
            let next_s = imu.peek().map(|(s, _)| s.t);
            match (next_m, next_s) {
                (Some(tm), Some(ts)) if tm <= ts => {
                    let (s, _) = *imu.peek().unwrap();
                    advance(&mut f, before.as_ref(), prev.as_ref(), s, t, tm, &g, &n);
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
                    let (s, still) = imu.next().unwrap();
                    advance(&mut f, before.as_ref(), prev.as_ref(), s, t, ts, &g, &n);
                    (before, prev) = (prev, Some(*s));
                    self.zupt(&mut f, t, ts, *still);
                    t = ts;
                    self.history.push_back((t, f));
                }
                // a measurement can't be newer than the newest IMU sample (frames wait for it)
                (Some(_), None) | (None, None) => break,
            }
        }
    }

    /// Save the prior and peer confirmation before an outstanding native search outlives the
    /// ordinary 400 ms history. A cancellation calls `cancel_inspection`; frames release their
    /// snapshot on completion, and the native deadline also bounds lost callbacks.
    pub fn hold_inspection(&mut self, camera: usize, t_e: u64, remaining_ns: u64) {
        if camera >= self.camera_updates.len() || remaining_ns == 0 {
            return;
        }
        if let Some((_, prior)) = self.state_at(t_e) {
            self.inspection_frames
                .retain(|f| f.camera != camera || f.exposure != t_e);
            if self
                .inspection_frames
                .iter()
                .filter(|f| f.camera == camera)
                .count()
                >= 4
            {
                let oldest = self
                    .inspection_frames
                    .iter()
                    .position(|f| f.camera == camera)
                    .unwrap();
                self.inspection_frames.remove(oldest);
            }
            self.inspection_frames.push_back(InspectionFrame {
                camera,
                exposure: t_e,
                until: self
                    .newest_imu()
                    .unwrap_or(t_e)
                    .saturating_add(remaining_ns),
                prior,
                peers: self.provisional.is_none() && self.others_set_prior(camera, t_e),
            });
        }
    }

    /// A probed camera also needs the exposure's saved prior if its labels arrive late: only
    /// one its peers confirmed then, as `frame` requires of held evidence.
    pub fn take_inspection_prior(&mut self, camera: usize, t_e: u64) -> Option<Filter> {
        let k = self
            .inspection_frames
            .iter()
            .position(|f| f.camera == camera && f.exposure == t_e)?;
        let held = self.inspection_frames.remove(k)?;
        (held.peers && self.newest_imu()? < held.until).then_some(held.prior)
    }

    /// Whether controller time `t` is older than the filter history.
    pub fn before_history(&self, t: u64) -> bool {
        self.history.front().is_some_and(|&(oldest, _)| t < oldest)
    }

    pub fn cancel_inspection(&mut self, camera: usize) {
        self.inspection_frames.retain(|f| f.camera != camera);
    }

    /// One sensor frame whose exposure was at controller time `t_e`. `engine` is the engine's
    /// world_from_model for this controller in this frame, when it found one it rates good or
    /// strong.
    /// `foreign`: the other controller's LEDs in this frame, as it is tracked (see `ForeignLed`).
    pub fn frame(
        &mut self,
        t_e: u64,
        camera: usize,
        cam: &Camera,
        blobs: &[Blob],
        engine: Option<EnginePose>,
        foreign: &[ForeignLed],
    ) -> Outcome {
        if camera >= self.camera_updates.len() {
            return Outcome::UnknownCamera;
        }
        if self.newest_imu().is_none_or(|n| t_e > n) {
            return Outcome::Early;
        }
        if self.bias_pending.is_some_and(|t| t_e > t) {
            self.test_pending_bias();
        }
        let held = self
            .inspection_frames
            .iter()
            .position(|f| f.camera == camera && f.exposure == t_e)
            .and_then(|k| self.inspection_frames.remove(k));
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
                        .any(|f| (f.uv - b.xy).norm() < self.params.foreign_px))
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
        // Old search results can judge the camera, but can never rewind the live filter or
        // acquire a controller from a stale pose. Use exactly the prior its peers set then.
        if held.is_some() && self.state_at(t_e).is_none() {
            if let Some(held) = held.filter(|f| self.newest_imu().is_some_and(|t| t < f.until)) {
                let depth = (cam.world_from_cam.inverse()
                    * self.body.world_from_model(&held.prior.x))
                .translation
                .vector
                .z;
                let comparable = held.peers && depth >= CHECK_MIN_DEPTH_M;
                let labels_off = engine.is_some_and(|e| {
                    e.world_from_model.rotation.angle_to(&held.prior.x.q)
                        > self.params.acquire_max_turn_rad
                });
                let hints = if labels_off { &[][..] } else { &labelled };
                let fit =
                    vision::fit_frame(cam, &self.body, &held.prior, blobs, hints, &self.params.fit);
                let observed = comparable
                    && (engine.is_some()
                        && (engine.is_some_and(|e| e.identified) || !labels_off)
                        && labelled.len() >= self.params.acquire_min_leds
                        || fit.as_ref().is_ok_and(|f| f.pairs.len() >= 4));
                if observed {
                    self.check_camera(camera, fit.is_err(), true);
                    if let Ok(fit) = fit {
                        self.cross_check(camera, cam, blobs, &fit.pairs, &held.prior, comparable);
                    }
                }
            }
            self.stats.too_old += 1;
            return Outcome::TooOld;
        }
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
            let depth = (cam.world_from_cam.inverse() * self.body.world_from_model(&prior.x))
                .translation
                .vector
                .z;
            let comparable = camera < self.camera_updates.len()
                && self.provisional.is_none()
                && self.others_set_prior(camera, t_e)
                && depth >= CHECK_MIN_DEPTH_M;
            // Native relative motion can identify a hand through a rotated calibration.
            // Only camera evidence uses that identity; hints and fusion keep the attitude check.
            let labelled_here = engine.is_some()
                && (engine.is_some_and(|e| e.identified) || !self.engine_labels_off)
                && labelled.len() >= self.params.acquire_min_leds;
            let checked = comparable
                && (labelled_here
                    || (blobs.len() >= CHECK_MIN_BLOBS
                        && vision::visible_leds(
                            cam,
                            &self.body,
                            &prior.x,
                            &self.params.fit.assoc,
                        ) >= CHECK_MIN_BLOBS));
            let fit = vision::fit_frame(cam, &self.body, &prior, blobs, hints, &self.params.fit);
            let observed =
                comparable && (labelled_here || fit.as_ref().is_ok_and(|f| f.pairs.len() >= 4));
            if checked || observed {
                self.check_camera(camera, fit.is_err(), observed);
            }
            // the other controller's track can explain the matched blobs better (see `ForeignLed`)
            let fit = fit.and_then(|fit| {
                if self.evidence(cam, blobs, &fit.pairs, &prior, foreign) < self.params.fit.min_leds
                {
                    Err(vision::Miss::Rejected)
                } else {
                    Ok(fit)
                }
            });
            match fit {
                Ok(fit) => {
                    if self.debug {
                        self.fits.push((fit.pairs.len(), fit.unexplained));
                    }
                    self.note_camera_update(camera, t_e);
                    self.cross_check(camera, cam, blobs, &fit.pairs, &prior, comparable);
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
                    self.decide(t_e, cam, blobs);
                    self.bias_pending = Some(self.bias_pending.map_or(t_e, |t| t.max(t_e)));
                    self.note_held();
                    self.stats.updates += 1;
                    self.stats.leds_used += n as u64;
                    return Outcome::Updated(n);
                }
                // a failed re-lock falls through to the engine's acquisition below, and so does a
                // track the frames refute
                Err(_) if relock => {}
                // only a frame exposed after the newest fix can refute it: an older one, arriving
                // late, has a prior that the newer fixes have corrected since
                Err(_)
                    if t_e > self.last_fix
                        && self.refutes(cam, blobs, &prior, engine, &labelled) =>
                {
                    // one engine pose can be wrong: it takes a second frame refuting the same fix
                    if !self.refuted.is_some_and(|(c, t, fix)| {
                        (c != camera || t != t_e) && fix == self.last_fix
                    }) {
                        self.refuted = Some((camera, t_e, self.last_fix));
                        self.stats.rejected += 1;
                        return Outcome::Missed;
                    }
                    self.stats.refuted += 1;
                }
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
            // after an IMU reset only the samples since it can carry a state to the present
            None if self.history.is_empty() && self.imu.front().is_some_and(|s| s.t <= t_e) => {
                (0, self.saved_bias)
            }
            None => {
                self.stats.too_old += 1;
                return Outcome::TooOld;
            }
        };
        // A bias the wrong-lock test rejects is no start for a track: it would be dropped at its
        // next exposure and the next track restored to the same bias, for as long as the
        // controller is in view. A state can come by one without a wrong lock: a zero-velocity
        // update on a state dead-reckoned for seconds out of view. It is learned afresh.
        let carried = carried.map(|mut f| {
            if self.bias_ran_off(&f) {
                let sd = self.params.acquire_sd[4];
                f.x.ba = Vector3::zeros();
                f.cov.fixed_rows_mut::<3>(BA).fill(0.0);
                f.cov.fixed_columns_mut::<3>(BA).fill(0.0);
                f.cov.fixed_view_mut::<3, 3>(BA, BA).fill_diagonal(sd * sd);
            }
            f
        });
        let memory = carried.filter(|_| remembered);
        // the alias taken in place of the engine's pose, if one was
        let mut turned_by = None;
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
                    self.refused_by.clear();
                }
                let agreed = t_e.saturating_sub(self.refusing_since) >= self.params.max_refusing_ns
                    && self.refused_by.iter().any(|&(c, turn)| {
                        c != camera
                            && self.same_hypothesis(
                                cam,
                                blobs,
                                &e.world_from_model,
                                &(turn * f.x.q),
                            )
                    });
                self.refused_by.retain(|&(c, _)| c != camera);
                self.refused_by
                    .push((camera, e.world_from_model.rotation * f.x.q.inverse()));
                if agreed {
                    self.last_confirmed = 0;
                } else {
                    self.stats.refused_acquisitions += 1;
                    refused = true;
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
                    let original_cost = self.acquisition_cost(cam, blobs, &chosen.1);
                    let mut costs = [Some(original_cost), None];
                    let mut best: Option<(f64, (Filter, vision::Fit), usize)> = None;
                    for (k, a) in self.aliases.iter().enumerate() {
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
                            if c < original_cost && best.as_ref().is_none_or(|b| c < b.0) {
                                best = Some((c, h, k));
                            }
                            retain_two_costs(&mut costs, c);
                        }
                    }
                    if costs[1]
                        .is_some_and(|second| second - costs[0].unwrap() < self.params.alias_margin)
                    {
                        if self.ambiguous_since == 0 {
                            self.ambiguous_since = t_e;
                        }
                        if t_e.saturating_sub(self.ambiguous_since) < self.params.max_refusing_ns {
                            self.stats.ambiguous_acquisitions += 1;
                            return Outcome::Waiting;
                        }
                    }
                    if let Some((_, h, k)) = best {
                        chosen = h;
                        turned = true;
                        turned_by = Some(k);
                    }
                }
                // the other controller's blobs under our labels fit a pose as well as ours do
                if self.independent(blobs, &chosen.1.pairs, foreign) < fit_params.min_leds {
                    self.stats.refused_acquisitions += 1;
                    return Outcome::Waiting;
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
                match self.search_kept(cam, blobs, foreign, &f, carried.as_ref(), &fit_params) {
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
        self.undecided = None;
        if memory.is_none() && !self.aliases.is_empty() {
            // the engine's pose and its ring turns, as turns of the track
            let back = turned_by.map_or(Isometry3::identity(), |k| self.aliases[k].turn.inverse());
            let turns: Vec<Isometry3<f64>> = turned_by
                .map(|_| back)
                .into_iter()
                .chain(
                    (0..self.aliases.len())
                        .filter(|&k| turned_by != Some(k))
                        .map(|k| back * self.aliases[k].turn),
                )
                .collect();
            self.undecided = Some(Undecided {
                since: t_e,
                lead: vec![0.0; turns.len()],
                turns,
            });
            self.decide(t_e, cam, blobs);
        }
        self.note_held();
        self.stats.acquisitions += 1;
        Outcome::Acquired(n)
    }

    /// Adds a frame to an undecided track's count (see `undecided`).
    fn decide(&mut self, t_e: u64, cam: &Camera, blobs: &[Blob]) {
        let Some(mut u) = self.undecided.take() else {
            return;
        };
        let Some((_, f)) = self.state_at(t_e) else {
            self.undecided = Some(u);
            return;
        };
        let pose = self.body.world_from_model(&f.x);
        // each hypothesis matched afresh from its own pose; one that cannot be fitted at all
        // leaves unexplained all but three of the blobs the other matched
        let cost = |p: &Isometry3<f64>| self.hypothesis_cost(cam, blobs, p);
        let unfit =
            |(c, n): (f64, usize)| self.params.alias_miss_cost * n.saturating_sub(3) as f64 + c;
        let own = cost(&pose);
        for (lead, turn) in u.lead.iter_mut().zip(&u.turns) {
            *lead += match (own, cost(&(pose * turn))) {
                (Some(o), Some(t)) => t.0 - o.0,
                (Some(o), None) => unfit(o) - o.0,
                (None, Some(t)) => t.0 - unfit(t),
                (None, None) => 0.0,
            };
        }
        let m = self.params.alias_margin;
        let expired = t_e.saturating_sub(u.since) > self.params.max_refusing_ns;
        let Some(k) = (0..u.lead.len())
            .filter(|&k| u.lead[k] <= -m)
            .min_by(|&a, &b| u.lead[a].total_cmp(&u.lead[b]))
        else {
            if !expired && u.lead.iter().any(|&l| l < m) {
                self.undecided = Some(u);
            }
            return;
        };
        // the frames favour another hypothesis: the track starts over from it, as acquired now,
        // still to be published by the first acquisition's deadline
        let Some(&(t, f)) = self.history.back() else {
            return;
        };
        let mut x = self.acquisition_prior(
            &(self.body.world_from_model(&f.x) * u.turns[k]),
            self.acquired_bias.as_ref(),
        );
        x.x.v = f.x.v;
        self.history.clear();
        self.history.push_back((t, x));
        self.applied.clear();
        self.stats.alias_turns += 1;
        if !expired {
            self.undecided = Some(Undecided {
                since: u.since,
                lead: vec![0.0; self.aliases.len()],
                turns: self.aliases.iter().map(|a| a.turn).collect(),
            });
        }
    }

    /// A hypothesis's cost on the frame (see `acquisition_cost`), matched afresh from its own pose,
    /// and how many LEDs it matched; None when it cannot be fitted at all.
    fn hypothesis_cost(
        &self,
        cam: &Camera,
        blobs: &[Blob],
        pose: &Isometry3<f64>,
    ) -> Option<(f64, usize)> {
        let prior = self.acquisition_prior(pose, None);
        vision::fit_frame(cam, &self.body, &prior, blobs, &[], &self.params.fit)
            .ok()
            .map(|fit| (self.acquisition_cost(cam, blobs, &fit), fit.pairs.len()))
    }

    /// Whether the frame cannot tell `pose` from the same position turned to `rotation`: neither
    /// leads the other by `alias_margin`.
    fn same_hypothesis(
        &self,
        cam: &Camera,
        blobs: &[Blob],
        pose: &Isometry3<f64>,
        rotation: &UnitQuaternion<f64>,
    ) -> bool {
        let rival = Isometry3::from_parts(pose.translation, *rotation);
        match (
            self.hypothesis_cost(cam, blobs, pose),
            self.hypothesis_cost(cam, blobs, &rival),
        ) {
            (Some(a), Some(b)) => (a.0 - b.0).abs() < self.params.alias_margin,
            _ => false,
        }
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

    /// Whether the frame refutes the track's prior, which it gave no fix: the engine's strong pose,
    /// with labels enough to acquire from and turned from the track by no more than an
    /// acquisition allows (the gyro vouches for the orientation), fits the frame farther from the
    /// prior than both their covariances allow (`FitParams::max_correction`). The dead-reckoned
    /// position then ran off, as it can without fixes in fast motion: our association, gated by
    /// that prior, cannot find the controller, and without this the track would be shown where it
    /// ran to, as tracked, until `lost_after_ns`.
    fn refutes(
        &self,
        cam: &Camera,
        blobs: &[Blob],
        prior: &Filter,
        engine: Option<EnginePose>,
        labelled: &[(usize, usize)],
    ) -> bool {
        let Some(e) = engine.filter(|e| {
            e.strong
                && labelled.len() >= self.params.acquire_min_leds
                && e.world_from_model.rotation.angle_to(&prior.x.q)
                    <= self.params.acquire_max_turn_rad
        }) else {
            return false;
        };
        let fit_params = FitParams {
            min_leds: self.params.acquire_min_leds,
            ..self.params.fit
        };
        let Some((_, fit)) = self.attempt(
            cam,
            blobs,
            &e.world_from_model,
            labelled.to_vec(),
            Some(prior),
            &fit_params,
        ) else {
            return false;
        };
        let f = &fit.updated.filter;
        let d = f.x.minus(&prior.x).fixed_rows::<6>(0).into_owned();
        let s = prior.cov.fixed_view::<6, 6>(0, 0) + f.cov.fixed_view::<6, 6>(0, 0);
        s.cholesky()
            .is_some_and(|c| (d.transpose() * c.solve(&d))[0] > self.params.fit.max_correction)
    }

    /// The controller in this frame from the kept orientation `kept` alone (see `search`): each
    /// position the frame supports is fitted, which corrects the orientation too. With nothing
    /// else to back it, a fit counts only if it explains the frame almost wholly
    /// (`explains_frame`), with enough matches the other controller does not account for
    /// (`independent`); costed like an engine pose, the cheapest is taken, and a second sensor
    /// still has to confirm it (see `provisional`).
    fn search_kept(
        &self,
        cam: &Camera,
        blobs: &[Blob],
        foreign: &[ForeignLed],
        kept: &Filter,
        carried: Option<&Filter>,
        fit_params: &FitParams,
    ) -> Option<(Filter, vision::Fit)> {
        let all: Vec<usize> = (0..blobs.len()).collect();
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
            .filter(|(_, fit)| {
                self.independent(blobs, &fit.pairs, foreign) >= fit_params.min_leds
                    && self.explains_frame(cam, blobs, fit)
            })
            .map(|h| (self.acquisition_cost(cam, blobs, &h.1), h))
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, h)| h)
    }

    /// How many of `pairs` are evidence of this controller under `prior` (see `ForeignLed`): a
    /// blob within the association's limits (`AssocParams::max_px` and `gate`) of one of the other
    /// controller's LEDs counts only if
    /// `prior` explains it, as the LED it was matched to, better.
    fn evidence(
        &self,
        cam: &Camera,
        blobs: &[Blob],
        pairs: &[(usize, usize)],
        prior: &Filter,
        foreign: &[ForeignLed],
    ) -> usize {
        let p6 = prior.cov.fixed_view::<6, 6>(0, 0).into_owned();
        let var = Matrix2::identity() * self.params.fit.update.sigma_px.powi(2);
        pairs
            .iter()
            .filter(|&&(led, b)| {
                let xy = blobs[b].xy;
                let Some(theirs) = foreign
                    .iter()
                    .filter(|f| (f.uv - xy).norm() <= self.params.fit.assoc.max_px)
                    .map(|f| prediction_fit(f.uv, &f.cov, xy))
                    .filter(|&(d2, _)| d2 < self.params.fit.assoc.gate)
                    .map(|(_, cost)| cost)
                    .reduce(f64::min)
                else {
                    return true;
                };
                vision::project(cam, &self.body, &prior.x, led).is_some_and(|pr| {
                    prediction_fit(pr.uv, &(pr.j * p6 * pr.j.transpose() + var), xy).1 < theirs
                })
            })
            .count()
    }

    /// How many of `pairs` are evidence of this controller where there is no prediction to judge
    /// the blobs by (see `ForeignLed`): those not on an LED of the other controller in this
    /// sensor's view.
    fn independent(
        &self,
        blobs: &[Blob],
        pairs: &[(usize, usize)],
        foreign: &[ForeignLed],
    ) -> usize {
        pairs
            .iter()
            .filter(|&&(_, b)| {
                !foreign
                    .iter()
                    .any(|f| f.seen && (f.uv - blobs[b].xy).norm() < self.params.foreign_px)
            })
            .count()
    }

    /// A fit pairs enough LEDs, nearly all those it shows the camera, closely, and leaves next to
    /// no blob in its footprint unexplained (see `SearchParams`): a position found under a wrong
    /// orientation, or from blobs paired with the wrong LEDs, fits some LEDs loosely but not the
    /// frame.
    fn explains_frame(&self, cam: &Camera, blobs: &[Blob], fit: &vision::Fit) -> bool {
        let p = &self.params.search;
        let visible = vision::visible_leds(
            cam,
            &self.body,
            &fit.updated.filter.x,
            &self.params.fit.assoc,
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

    // Original predicate retained as an independent differential/benchmark oracle.
    #[cfg(test)]
    fn explains_frame_baseline(&self, cam: &Camera, blobs: &[Blob], fit: &vision::Fit) -> bool {
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
        camera: usize,
        cam: &Camera,
        blobs: &[Blob],
        pairs: &[(usize, usize)],
        prior: &Filter,
        comparable: bool,
    ) {
        if !comparable || pairs.len() < 4 {
            return;
        }
        let res = vision::pixel_residuals(cam, &self.body, blobs, pairs, &prior.x);
        let mean = res.iter().copied().sum::<Vector2<f64>>() / res.len() as f64;
        let mut r: Vec<f64> = res.iter().map(|o| o.norm()).collect();
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

    /// Whether another sensor updated the track within `CROSS_CHECK_NS` before `t_e`: the prior
    /// at `t_e` is then the other sensors' word, which this one can agree with or not.
    fn others_set_prior(&self, camera: usize, t_e: u64) -> bool {
        self.camera_updates.iter().enumerate().any(|(c, updates)| {
            c != camera
                && updates
                    .iter()
                    .rev()
                    .find(|&&t| t <= t_e)
                    .is_some_and(|&t| t_e - t <= CROSS_CHECK_NS)
        })
    }

    fn note_camera_update(&mut self, camera: usize, t_e: u64) {
        if let Some(updates) = self.camera_updates.get_mut(camera) {
            let k = updates.partition_point(|&t| t < t_e);
            if updates.get(k) != Some(&t_e) {
                updates.insert(k, t_e);
            }
        }
    }

    /// Counts the possible sighting, and separately the actual correspondences that can judge
    /// the sensor's pose. Missing an unlabelled hand may be occlusion or background blobs.
    fn check_camera(&mut self, camera: usize, missed: bool, observed: bool) {
        if let Some(a) = self.agreement.get_mut(camera) {
            a.checks += 1;
            a.misses += u64::from(missed);
            a.observed_checks += u64::from(observed);
            a.observed_misses += u64::from(observed && missed);
        }
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
        let start = self.imu.partition_point(|s| s.t < from);
        let mut win = self.imu.range(start..k);
        let mean = win.clone().map(|s| s.accel).sum::<Vector3<f64>>() / win.len() as f64;
        let steady = win.all(|s| (s.accel - mean).norm() < self.params.steady_accel)
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
        let (visible, near) =
            vision::visibility_counts(cam, &self.body, &f.x, blobs, &self.params.fit.assoc, 3.0);
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

/// The standard deviations of the pose `Body::world_from_model` puts at filter `f`: the model
/// origin's position along the world axes, which takes the orientation's error through the lever
/// arm from the IMU (q Exp(d0) r = q r - q [r]x d0 for small d0), and the orientation about the
/// model's own axes (the filter's local rotation error).
fn published_stddev(f: &Filter, body: &Body) -> (Vector3<f64>, Vector3<f64>) {
    // the pose block (rotation, position) alone, as the association uses it
    let p6 = f.cov.fixed_view::<6, 6>(ROT, ROT);
    let mut j = nalgebra::SMatrix::<f64, 3, 6>::zeros();
    j.fixed_view_mut::<3, 3>(0, ROT)
        .copy_from(&-(f.x.q.to_rotation_matrix().matrix() * super::eskf::skew(&-body.imu_pos)));
    j.fixed_view_mut::<3, 3>(0, POS)
        .copy_from(&Matrix3::identity());
    // the diagonal of j p6 j^T alone
    let pos = (j * p6).component_mul(&j).column_sum().map(f64::sqrt);
    let rot = p6.fixed_view::<3, 3>(ROT, ROT).diagonal().map(f64::sqrt);
    (pos, rot)
}

#[cfg(test)]
mod tests {
    mod hot_paths;
    use super::*;
    use crate::calib::synth::{self, Motion, Rng};
    use nalgebra::{Point3, Translation3, UnitQuaternion, Vector2};

    const G: f64 = 9.80665;

    /// The published position's uncertainty is the model origin's, not the IMU's: an
    /// orientation error about the model's z axis moves the origin 2 cm from the IMU across
    /// that lever arm, here along the world x axis.
    #[test]
    fn the_published_stddevs_are_the_published_points() {
        let body = Body {
            leds: Vec::new(),
            imu_pos: Vector3::new(0.02, 0.0, 0.0),
        };
        let mut cov = Cov::zeros();
        let (sr, sp) = (0.1, 0.003);
        cov[(ROT + 2, ROT + 2)] = sr * sr;
        for i in 0..3 {
            cov[(POS + i, POS + i)] = sp * sp;
        }
        let f = Filter {
            x: Nominal {
                q: UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2),
                p: Vector3::new(0.1, 1.0, -0.2),
                v: Vector3::zeros(),
                bg: Vector3::zeros(),
                ba: Vector3::zeros(),
            },
            cov,
        };
        let (pos, rot) = published_stddev(&f, &body);
        let across = (sp * sp + (sr * 0.02).powi(2)).sqrt();
        assert!(
            (pos - Vector3::new(across, sp, sp)).norm() < 1e-9,
            "{pos:?}"
        );
        assert!((rot - Vector3::new(0.0, 0.0, sr)).norm() < 1e-12, "{rot:?}");
    }

    /// A removed sensor's held search exposures and accepted updates go with it; the other
    /// cameras keep theirs.
    #[test]
    fn hotplug_a_forgotten_camera_leaves_no_held_search_or_updates() {
        let w = world(1);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let prior = Filter {
            x: Nominal {
                q: UnitQuaternion::identity(),
                p: Vector3::zeros(),
                v: Vector3::zeros(),
                bg: Vector3::zeros(),
                ba: Vector3::zeros(),
            },
            cov: Cov::identity(),
        };
        for camera in [0, 1] {
            tr.camera_updates[camera].push_back(10);
            tr.searched_at[camera] = 10;
            tr.inspection_frames.push_back(InspectionFrame {
                camera,
                exposure: 10,
                until: 20,
                prior,
                peers: true,
            });
        }
        tr.track_camera = Some(0);
        tr.forget_camera(0);
        assert!(tr.camera_updates[0].is_empty());
        assert_eq!(tr.camera_updates[1], [10]);
        assert_eq!(tr.searched_at, [0, 10, 0]);
        assert_eq!(
            tr.inspection_frames
                .iter()
                .map(|f| f.camera)
                .collect::<Vec<_>>(),
            [1]
        );
        assert_eq!(tr.track_camera, None);
    }

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
            hidden: None,
            gyro: w.motion.gyro(t) + w.bg,
            accel: w
                .motion
                .accel(t, &w.body.imu_pos, &Vector3::new(0.0, G, 0.0))
                + w.ba,
        }
    }

    fn sight(w: &World, cam: &Camera, t: f64, rng: &mut Rng, noise: f64) -> Vec<Blob> {
        sight_posed(w, cam, &w.motion.pose(t), rng, noise)
    }

    fn sight_posed(
        w: &World,
        cam: &Camera,
        pose: &Isometry3<f64>,
        rng: &mut Rng,
        noise: f64,
    ) -> Vec<Blob> {
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
        /// The accelerometer reads this much more over this span: an error the IMU model does not
        /// cover, as violent shaking can leave.
        accel_off: Option<(f64, f64, Vector3<f64>)>,
        /// The first sensor's first frame exposed at or after this time shows, with the engine's
        /// strong pose, a controller 20 cm from ours (as when the engine takes the other
        /// controller for this one).
        phantom_at: Option<f64>,
        /// The first two sensors' first frames exposed at or after this time show the same
        /// phantom, and arrive this much later than usual: after the third sensor's newer fixes.
        phantom_late: Option<(f64, f64)>,
        /// The IMU samples in this span are lost (the radio dropped them).
        imu_lost: Option<(f64, f64)>,
        /// The gyro reads no further than the ends of `Gyro::default`'s range, as a real one
        /// saturates; the tracker is told those ends when this is true.
        clipped: Option<bool>,
        /// Orientation errors (rad) are kept instead of position errors.
        rotation: bool,
        /// Position errors are kept from this time on (default 1 s).
        measure_after: Option<f64>,
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
        /// The tracker sees this sensor this far from where it is (world from where it is
        /// believed to be: a few mm and a fraction of a degree, as a knocked sensor).
        misplaced: Option<(usize, Isometry3<f64>)>,
        /// The engine_flipped span applies to the sensors from this one on (0: all).
        flip_from: usize,
        /// The other controller, from this time on (see `Other`).
        other: Option<(f64, Other)>,
    }

    /// The other controller, as tracked by its own tracker: it is handed to the tracker as
    /// `ForeignLed`s at its true pose.
    #[derive(Clone, Copy)]
    struct Other {
        /// Where it is, given this controller's pose and the first sensor's.
        place: fn(&Isometry3<f64>, &Camera) -> Isometry3<f64>,
        /// The frames show it and not this controller, under this controller's labels, and the
        /// engine gives its pose as this controller's (as when the engine takes it for this one).
        mislabelled: bool,
        /// The sensors from this one on have seen it (`ForeignLed::seen`).
        seen_from: usize,
        /// Its track's position sd, m.
        pos_sd: f64,
    }

    /// Beside this controller, 10 cm along the world x axis.
    fn beside(pose: &Isometry3<f64>, _: &Camera) -> Isometry3<f64> {
        Translation3::new(0.1, 0.0, 0.0) * pose
    }

    /// 10 cm behind this controller on the first sensor's line of sight: hidden by it from that
    /// sensor, its LEDs a pixel or less from this controller's in that sensor's image.
    fn behind(pose: &Isometry3<f64>, cam: &Camera) -> Isometry3<f64> {
        let ray = (pose.translation.vector - cam.world_from_cam.translation.vector).normalize();
        Translation3::from(ray * 0.1) * pose
    }

    /// `pose`'s LEDs that face `cam` near `blobs`, as a track of it known to `pos_sd` (and a fifth
    /// of a degree) predicts them.
    fn foreign_at(
        w: &World,
        cam: &Camera,
        pose: &Isometry3<f64>,
        blobs: &[Blob],
        seen: bool,
        pos_sd: f64,
    ) -> Vec<ForeignLed> {
        let p = Params::default();
        let mut cov = Cov::identity() * 1e-6;
        for k in 0..3 {
            cov[(ROT + k, ROT + k)] = 0.2f64.to_radians().powi(2);
            cov[(POS + k, POS + k)] = pos_sd * pos_sd;
        }
        let x = Nominal {
            q: pose.rotation,
            p: (pose * Point3::from(w.body.imu_pos)).coords,
            v: Vector3::zeros(),
            bg: Vector3::zeros(),
            ba: Vector3::zeros(),
        };
        vision::predicted_near(
            cam,
            &w.body,
            &Filter { x, cov },
            blobs,
            p.fit.assoc.max_px,
            p.fit.update.sigma_px,
            p.fit.assoc.min_facing,
        )
        .into_iter()
        .map(|(uv, cov)| ForeignLed { uv, cov, seen })
        .collect()
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
        tr.gyro_range = opts.clipped.filter(|&told| told).map(|_| Gyro::default());
        let seen: Vec<Camera> = w
            .cams
            .iter()
            .enumerate()
            .map(|(c, cam)| match opts.misplaced {
                Some((m, off)) if m == c => Camera {
                    world_from_cam: off * cam.world_from_cam,
                    ..*cam
                },
                _ => *cam,
            })
            .collect();
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
        let phantom = |c: usize, te: f64| {
            let at = |a: Option<f64>, cams: usize| {
                a.is_some_and(|a| c < cams && te >= a && te < a + dt_frame)
            };
            let shown = at(opts.phantom_at, 1) || at(opts.phantom_late.map(|p| p.0), 2);
            Translation3::new(if shown { 0.2 } else { 0.0 }, 0.0, 0.0)
        };
        let other_at = |te: f64| opts.other.filter(|(from, _)| te >= *from).map(|(_, o)| o);
        let held_back = |c: usize, te: f64| {
            opts.phantom_late
                .filter(|&(a, _)| c < 2 && te >= a && te < a + dt_frame)
                .map_or(0.0, |(_, extra)| extra)
        };
        while t < seconds {
            t += dt_imu;
            let mut s = imu(w, t);
            if let Some((after, jump)) = opts.accel_jump.filter(|(after, _)| t >= *after) {
                let _ = after;
                s.accel += jump;
            }
            if let Some((_, _, off)) = opts.accel_off.filter(|(a, b, _)| t >= *a && t < *b) {
                s.accel += off;
            }
            if opts.clipped.is_some() {
                let (lo, hi) = (f64::from(i16::MIN), f64::from(i16::MAX));
                s.gyro = s.gyro.map(|w| match w / GYRO_PER_COUNT {
                    c if c > hi => hi * GYRO_PER_COUNT,
                    c if c < lo => lo * GYRO_PER_COUNT,
                    _ => w,
                });
            }
            if !opts.imu_lost.is_some_and(|(a, b)| t > a && t < b) {
                tr.imu(s);
            }
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
                        } + held_back(c, next_frame);
                        let shown = match other_at(next_frame) {
                            Some(o) if o.mislabelled => {
                                (o.place)(&w.motion.pose(next_frame), &w.cams[0])
                            }
                            _ => phantom(c, next_frame) * w.motion.pose(next_frame),
                        };
                        pending.push((
                            next_frame,
                            next_frame + delay + late,
                            c,
                            sight_posed(w, cam, &shown, &mut rng, pixel_noise),
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
                let mut engine = Some(phantom(c, te) * w.motion.pose(te));
                let mut foreign = Vec::new();
                if let Some(o) = other_at(te) {
                    let pose = (o.place)(&w.motion.pose(te), &w.cams[0]);
                    foreign = foreign_at(w, &seen[c], &pose, &blobs, c >= o.seen_from, o.pos_sd);
                    if o.mislabelled {
                        engine = Some(pose);
                    }
                }
                if c >= opts.flip_from
                    && opts.engine_flipped.is_some_and(|(a, b)| te >= a && te < b)
                {
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
                    &seen[c],
                    &blobs,
                    engine.map(|e| EnginePose {
                        world_from_model: e,
                        identified: false,
                        strong: !opts.engine_good
                            || opts.strong_every.is_some_and(|n| {
                                ((te - 0.01) * 52.0).round() as usize % n == n - 1
                            }),
                    }),
                    &foreign,
                );
            }
            if t > opts.measure_after.unwrap_or(1.0) {
                let Some(o) = tr.output() else { continue };
                if o.tracking && !opts.engine_flipped.is_some_and(|(a, b)| t >= a && t < b) {
                    let truth = w.motion.pose(t);
                    errors.push(if opts.rotation {
                        o.world_from_model.rotation.angle_to(&truth.rotation)
                    } else {
                        (o.world_from_model.translation.vector - truth.translation.vector).norm()
                    });
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
        // the uncertainties are those of the newest state's published pose
        let newest = published_stddev(&tr.history.back().unwrap().1, &tr.body);
        assert_eq!((o.pos_stddev, o.rot_stddev), newest);
        assert!(newest.0.min() > 0.0 && newest.1.min() > 0.0, "{newest:?}");
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

    /// A controller woken and picked up while the engine locks onto one of its ring aliases: for
    /// the first 0.3 s the engine reports the pose turned by the alias, with its labels to match,
    /// and the right Touch's half-m/s^2 accelerometer bias is still to be learned. A single frame
    /// can favour the alias over the truth; the track may not be published until the frames have
    /// told them apart, and never shows the alias.
    #[test]
    fn a_first_lock_on_a_ring_alias_is_not_published() {
        // the cases of 80 (40 motions, either alias) that published the alias as tracked
        for (seed, k) in [(11, 0), (25, 0), (22, 1)] {
            let mut w = touch_world(seed);
            w.ba = Vector3::new(0.10, 0.44, -0.10);
            let alias = vision::ring_aliases(&w.body.leds).swap_remove(k);
            let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
            let mut rng = Rng::new(11);
            let (dt_frame, delay) = (1.0 / 52.0, 0.025);
            let mut pending: Vec<(f64, usize, Vec<Blob>)> = Vec::new();
            let (mut next_frame, mut t) = (0.01, 0.0);
            let (mut worst, mut published): (f64, usize) = (0.0, 0);
            while t < 0.6 {
                t += 0.002;
                tr.imu(imu(&w, t));
                while next_frame <= t {
                    for (c, cam) in w.cams.iter().enumerate() {
                        pending.push((next_frame, c, sight(&w, cam, next_frame, &mut rng, 0.3)));
                    }
                    next_frame += dt_frame;
                }
                let ready: Vec<_> = pending
                    .iter()
                    .filter(|p| p.0 + delay <= t)
                    .cloned()
                    .collect();
                pending.retain(|p| p.0 + delay > t);
                for (te, c, mut blobs) in ready {
                    let mut pose = w.motion.pose(te);
                    if te < 0.3 {
                        pose *= alias.turn;
                        for b in &mut blobs {
                            let i = (b.led_id & 0xff) as usize;
                            b.led_id = alias.led_at[i].map_or(-1, |j| (1 << 8) | j as i32);
                        }
                    }
                    let t_e = (1e9 + te * 1e9) as u64;
                    tr.frame(
                        t_e,
                        c,
                        &w.cams[c],
                        &blobs,
                        Some(EnginePose::strong(pose)),
                        &[],
                    );
                }
                if let Some(o) = tr.output().filter(|o| o.tracking) {
                    published += 1;
                    let truth = w.motion.pose(t).rotation;
                    worst = worst.max(o.world_from_model.rotation.angle_to(&truth));
                }
            }
            assert!(
                published > 0 && tr.tracking(),
                "seed {seed}: {:?}",
                tr.stats
            );
            assert!(
                worst < 5f64.to_radians(),
                "seed {seed}, alias {k}: published {:.1} deg off {:?}",
                worst.to_degrees(),
                tr.stats
            );
        }
    }

    /// An undecided first lock is shown at the latest `max_refusing_ns` after it was acquired,
    /// even when another hypothesis has taken its place meanwhile: here no hypothesis can ever
    /// lead by the margin but one forced past it 0.3 s in, so the track is replaced then and
    /// still has to be shown by the first acquisition's deadline.
    #[test]
    fn an_undecided_first_lock_is_shown_by_its_deadline_even_when_replaced() {
        let w = touch_world(5);
        let params = Params {
            alias_margin: 1e6,
            ..Params::default()
        };
        let mut tr = Tracker::new(1, w.body.clone(), params, w.cams.len());
        let mut rng = Rng::new(11);
        let (dt_frame, delay) = (1.0 / 52.0, 0.025);
        let mut pending: Vec<(f64, usize, Vec<Blob>)> = Vec::new();
        let (mut next_frame, mut t) = (0.01, 0.0);
        let (mut since, mut forced, mut shown) = (None, false, None);
        while t < 1.2 && shown.is_none() {
            t += 0.002;
            tr.imu(imu(&w, t));
            while next_frame <= t {
                for (c, cam) in w.cams.iter().enumerate() {
                    pending.push((next_frame, c, sight(&w, cam, next_frame, &mut rng, 0.3)));
                }
                next_frame += dt_frame;
            }
            let ready: Vec<_> = pending
                .iter()
                .filter(|p| p.0 + delay <= t)
                .cloned()
                .collect();
            pending.retain(|p| p.0 + delay > t);
            for (te, c, blobs) in ready {
                let engine = EnginePose::strong(w.motion.pose(te));
                tr.frame(
                    (1e9 + te * 1e9) as u64,
                    c,
                    &w.cams[c],
                    &blobs,
                    Some(engine),
                    &[],
                );
            }
            since = since.or(tr.undecided.as_ref().map(|u| u.since));
            if let (Some(s), Some(u)) = (since, tr.undecided.as_mut()) {
                if !forced && tr.history.back().is_some_and(|h| h.0 > s + 300_000_000) {
                    forced = true;
                    u.turns = vec![Isometry3::identity()];
                    u.lead = vec![-2e6];
                }
            }
            if tr.output().is_some_and(|o| o.tracking) {
                shown = tr.history.back().map(|h| h.0);
            }
        }
        let (since, shown) = (since.expect("acquired"), shown.expect("shown"));
        assert!(forced && tr.stats.alias_turns >= 1, "{:?}", tr.stats);
        let after = shown - since;
        assert!(
            after > 400_000_000 && after <= tr.params.max_refusing_ns + 2_000_000,
            "shown {} ms after acquisition {:?}",
            after as f64 * 1e-6,
            tr.stats
        );
    }

    /// Out of sight, an error the IMU model does not cover carries the dead-reckoned position off
    /// by decimetres. Once the sensors see the controller again, the engine's poses refute the
    /// track and it is acquired afresh: it is not shown where it ran off to, as tracked, until
    /// the position is lost.
    #[test]
    fn a_track_that_ran_off_out_of_sight_is_refuted() {
        let w = world(4);
        let (tr, errors) = run_with(
            &w,
            6.0,
            0.3,
            &Run {
                blackout: Some((4.0, 4.25)),
                accel_off: Some((4.0, 4.2, Vector3::new(15.0, 0.0, 0.0))),
                // the first frames after the blackout arrive at 4.275 s; the fresh track's
                // velocity then takes a few frames to settle
                measure_after: Some(4.35),
                ..Run::default()
            },
        );
        assert!(
            pct(&errors, 1.0) < 0.01,
            "{:.3} {:?}",
            pct(&errors, 1.0),
            tr.stats
        );
        assert!(tr.stats.refuted >= 1, "{:?}", tr.stats);
    }

    /// One frame whose engine pose and blobs put the controller 20 cm from the track refutes
    /// nothing: the other sensors' fixes hold the track.
    #[test]
    fn one_frame_alone_does_not_refute_a_track() {
        let w = world(4);
        let (tr, errors) = run_with(
            &w,
            6.0,
            0.3,
            &Run {
                phantom_at: Some(4.0),
                ..Run::default()
            },
        );
        assert_eq!(
            (tr.stats.acquisitions, tr.stats.refuted),
            (1, 0),
            "{:?}",
            tr.stats
        );
        assert!(pct(&errors, 1.0) < 0.01, "{:?}", tr.stats);
    }

    /// Two sensors' frames showing the controller 20 cm away arrive late, after the third sensor's
    /// newer fixes: those fixes have already placed the track, so the old frames refute nothing.
    #[test]
    fn frames_older_than_the_newest_fix_do_not_refute_it() {
        let w = world(4);
        let (tr, errors) = run_with(
            &w,
            6.0,
            0.3,
            &Run {
                phantom_late: Some((4.0, 0.06)),
                ..Run::default()
            },
        );
        assert_eq!(
            (tr.stats.acquisitions, tr.stats.refuted),
            (1, 0),
            "{:?}",
            tr.stats
        );
        assert!(pct(&errors, 1.0) < 0.01, "{:?}", tr.stats);
    }

    /// A sample stream state, its covariance as a converged track's.
    fn moving_filter() -> Filter {
        let x = Nominal {
            q: UnitQuaternion::from_euler_angles(0.3, -0.2, 1.1),
            p: Vector3::new(0.1, 1.0, -0.2),
            v: Vector3::new(0.4, -0.3, 0.2),
            bg: Vector3::new(0.01, -0.005, 0.003),
            ba: Vector3::new(0.02, 0.4, -0.05),
        };
        let mut cov = Cov::identity() * 1e-6;
        for k in 0..3 {
            cov[(POS + k, POS + k)] = 1e-7;
        }
        Filter { x, cov }
    }

    fn sample(t_ms: f64, k: f64) -> ImuSample {
        ImuSample {
            t: (t_ms * 1e6) as u64,
            hidden: None,
            gyro: Vector3::new(1.3 * k, -0.7, 2.1 * k),
            accel: Vector3::new(3.0 * k, 9.0 + k, -2.0),
        }
    }

    /// An interval of about one sample period (2 ms, or the 1 and 3 ms of stamp jitter) is the end
    /// sample held over it, exactly as `Filter::propagate` integrates it, whole or in pieces.
    #[test]
    fn a_complete_interval_is_integrated_as_before() {
        let (g, n) = (Vector3::new(0.0, -G, 0.0), Noise::default());
        for (t_prev, t_s) in [(10.0, 12.0), (10.0, 13.0), (10.0, 11.0)] {
            let (before, prev, s) = (sample(8.0, 0.9), sample(t_prev, 1.0), sample(t_s, 1.2));
            let (mut a, mut b) = (moving_filter(), moving_filter());
            advance(&mut a, Some(&before), Some(&prev), &s, prev.t, s.t, &g, &n);
            b.propagate(&s.gyro, &s.accel, (s.t - prev.t) as f64 * 1e-9, &g, &n);
            assert_eq!(a, b, "{t_prev} -> {t_s} ms");
            let mid = prev.t + 700_000;
            let (mut a, mut b) = (moving_filter(), moving_filter());
            advance(&mut a, Some(&before), Some(&prev), &s, prev.t, mid, &g, &n);
            advance(&mut a, Some(&before), Some(&prev), &s, mid, s.t, &g, &n);
            b.propagate(&s.gyro, &s.accel, (mid - prev.t) as f64 * 1e-9, &g, &n);
            b.propagate(&s.gyro, &s.accel, (s.t - mid) as f64 * 1e-9, &g, &n);
            assert_eq!(a, b, "{t_prev} -> {t_s} ms in two pieces");
        }
    }

    /// Through the tracker: a stream with no sample missing (stamp jitter included) leaves every
    /// history state, and a state between samples, bit for bit as holding each sample does.
    #[test]
    fn a_stream_without_missing_samples_is_integrated_as_before() {
        let w = world(4);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let start = moving_filter();
        tr.history.push_back((10_000_000, start));
        tr.imu.push_back(sample(10.0, 1.0));
        tr.still.push_back(false);
        let mut reference = vec![start];
        let mut t = 10.0;
        for (i, step) in [2.0, 2.0, 3.0, 1.0, 2.0, 2.0, 1.0, 3.0, 2.0]
            .iter()
            .enumerate()
        {
            t += step;
            let s = sample(t, 1.0 + 0.05 * i as f64);
            let mut f = *reference.last().unwrap();
            f.propagate(
                &s.gyro,
                &s.accel,
                step * 1e-3,
                &tr.gravity(),
                &tr.params.noise,
            );
            reference.push(f);
            tr.imu(s);
        }
        let got: Vec<Filter> = tr.history.iter().map(|&(_, f)| f).collect();
        assert_eq!(got, reference);
        // between the last two samples
        let s = tr.imu[tr.imu.len() - 1];
        let mid = s.t - 1_200_000;
        let mut f = reference[reference.len() - 2];
        f.propagate(
            &s.gyro,
            &s.accel,
            (mid - tr.imu[tr.imu.len() - 2].t) as f64 * 1e-9,
            &tr.gravity(),
            &tr.params.noise,
        );
        assert_eq!(tr.state_at(mid).map(|(_, f)| f), Some(f));
    }

    /// A track of a hard sway (4 Hz, up to 25 m/s^2) loses 60 ms of IMU samples where the
    /// acceleration curves. The state carried across the hole lies within the uncertainty the
    /// filter then claims: bridged by the straight line between the samples either side, which
    /// holding the last sample is not, and owning what that line cannot know.
    #[test]
    fn a_lost_stretch_is_bridged_within_its_uncertainty() {
        let mut w = world(4);
        w.motion = Motion::sway(
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.04, 0.03, 0.04),
            4.0,
            [0.4, -0.3, 0.2],
        );
        w.ba = Vector3::zeros();
        w.bg = Vector3::zeros();
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let state = |t: f64| {
            let imu_at = |t: f64| (w.motion.pose(t) * Point3::from(w.body.imu_pos)).coords;
            Nominal {
                q: w.motion.pose(t).rotation,
                p: imu_at(t),
                v: (imu_at(t + 1e-4) - imu_at(t - 1e-4)) / 2e-4,
                bg: Vector3::zeros(),
                ba: Vector3::zeros(),
            }
        };
        let t0 = 1.0;
        let mut start = moving_filter();
        start.x = state(t0);
        let first = imu(&w, t0);
        tr.history.push_back((first.t, start));
        tr.imu.push_back(first);
        tr.still.push_back(false);
        // the acceleration's curvature is largest at its peaks: the hole straddles one
        let (lost_from, lost_to) = (t0 + 0.040, t0 + 0.100);
        let mut t = t0;
        while t < lost_to + 0.0005 {
            t += 0.002;
            if t > lost_from + 1e-9 && t < lost_to - 1e-9 {
                continue;
            }
            tr.imu(imu(&w, t));
        }
        let (te, f) = *tr.history.back().unwrap();
        let truth = state((te as f64 * 1e-9) - 1.0);
        let e = f.x.minus(&truth);
        let ev = e.fixed_rows::<3>(VEL).into_owned();
        let cv = f.cov.fixed_view::<3, 3>(VEL, VEL).into_owned();
        let m2 = ev.dot(&(cv.try_inverse().unwrap() * ev));
        // chi-square 99.9 % for three degrees of freedom
        assert!(
            m2 < 16.27,
            "velocity off by {:.0} mm/s, {m2:.1} against chi-square 16.27",
            ev.norm() * 1e3
        );
        assert!(
            ev.norm() > 0.01,
            "the hole should have cost something: {:.1} mm/s",
            ev.norm() * 1e3
        );
    }

    /// The same sway, tracked: after the 60 ms hole the first frames are accepted, the track is
    /// neither rejected nor restarted, and it stays within a centimetre of the truth.
    #[test]
    fn the_first_fix_after_a_lost_stretch_is_accepted() {
        let mut w = world(4);
        w.motion = Motion::sway(
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.04, 0.03, 0.04),
            4.0,
            [0.4, -0.3, 0.2],
        );
        let run = |lost| {
            run_with(
                &w,
                5.0,
                0.3,
                &Run {
                    imu_lost: lost,
                    // from the first frames after the hole on (the output stands still meanwhile)
                    measure_after: Some(4.15),
                    ..Run::default()
                },
            )
        };
        let (base, _) = run(None);
        let (tr, errors) = run(Some((4.04, 4.10)));
        assert_eq!(
            (
                tr.stats.acquisitions,
                tr.stats.rejected,
                tr.stats.wrong_locks
            ),
            (1, base.stats.rejected, 0),
            "{:?}",
            tr.stats
        );
        assert!(
            pct(&errors, 1.0) < 0.01,
            "{:.1} mm {:?}",
            pct(&errors, 1.0) * 1e3,
            tr.stats
        );
    }

    /// The hole's uncertainty is white acceleration and turn rate over the missing part, as it
    /// stands at its end: from no covariance and no other noise, velocity variance V, position
    /// V Tm^2 / 3 and their covariance V Tm / 2, rotation R, per axis, with (V, R) from
    /// `hole_variance` and Tm the missing part; the same in two pieces. (No specific force, so the
    /// rotation's uncertainty does not reach the velocity.)
    #[test]
    fn a_holes_uncertainty_is_white_noise_over_its_missing_part() {
        let (g, quiet) = (
            Vector3::new(0.0, -G, 0.0),
            Noise {
                gyro: 0.0,
                accel: 0.0,
                gyro_bias: 0.0,
                accel_bias: 0.0,
                accel_per_accel: 0.0,
                accel_per_rate: 0.0,
            },
        );
        let at = |t_ms: f64, w: f64| ImuSample {
            t: (t_ms * 1e6) as u64,
            hidden: None,
            gyro: Vector3::new(w, -0.5 * w, 0.3),
            accel: Vector3::zeros(),
        };
        let (before, prev, s) = (at(8.0, 1.0), at(10.0, 3.0), at(54.0, 2.0));
        let mut start = moving_filter();
        start.x.ba = Vector3::zeros();
        start.cov = Cov::zeros();
        let split = s.t - IMU_PERIOD_NS;
        let tm = (split - prev.t) as f64 * 1e-9;
        let (v, r) = hole_variance(Some(&before), &prev, &s);
        let mut whole = start;
        advance(
            &mut whole,
            Some(&before),
            Some(&prev),
            &s,
            prev.t,
            split,
            &g,
            &quiet,
        );
        let mut pieces = start;
        let mid = prev.t + 17_000_000;
        advance(
            &mut pieces,
            Some(&before),
            Some(&prev),
            &s,
            prev.t,
            mid,
            &g,
            &quiet,
        );
        advance(
            &mut pieces,
            Some(&before),
            Some(&prev),
            &s,
            mid,
            split,
            &g,
            &quiet,
        );
        for (name, f) in [("whole", whole), ("pieces", pieces)] {
            for k in 0..3 {
                let close = |a: f64, b: f64| (a - b).abs() <= 1e-9 * b.abs();
                assert!(
                    close(f.cov[(VEL + k, VEL + k)], v),
                    "{name} velocity {:e} {v:e}",
                    f.cov[(VEL + k, VEL + k)]
                );
                assert!(
                    close(f.cov[(POS + k, POS + k)], v * tm * tm / 3.0),
                    "{name} position {:e} {:e}",
                    f.cov[(POS + k, POS + k)],
                    v * tm * tm / 3.0
                );
                assert!(
                    close(f.cov[(POS + k, VEL + k)], v * tm / 2.0),
                    "{name} position-velocity {:e} {:e}",
                    f.cov[(POS + k, VEL + k)],
                    v * tm / 2.0
                );
                assert!(
                    close(f.cov[(ROT + k, ROT + k)], r),
                    "{name} rotation {:e} {r:e}",
                    f.cov[(ROT + k, ROT + k)]
                );
            }
        }
    }

    /// A controller pushed steadily (gravity and about 3 g) loses 46 ms of samples. The hole's turn
    /// rate reaches the velocity and position through the specific force as it accumulates, so,
    /// from no covariance and no other noise, the covariance is the same to rounding whether the
    /// hole is integrated whole or in the pieces between frames.
    #[test]
    fn a_hole_is_integrated_alike_whole_or_in_pieces() {
        let (g, quiet) = (
            Vector3::new(0.0, -G, 0.0),
            Noise {
                gyro: 0.0,
                accel: 0.0,
                gyro_bias: 0.0,
                accel_bias: 0.0,
                accel_per_accel: 0.0,
                accel_per_rate: 0.0,
            },
        );
        let mut start = moving_filter();
        start.cov = Cov::zeros();
        let at = |t_ms: f64| ImuSample {
            t: (t_ms * 1e6) as u64,
            hidden: None,
            gyro: start.x.bg,
            accel: start.x.q.inverse() * -g + Vector3::new(18.0, -21.0, 9.0) + start.x.ba,
        };
        let (before, prev, s) = (at(8.0), at(10.0), at(58.0));
        let mut whole = start;
        advance(
            &mut whole,
            Some(&before),
            Some(&prev),
            &s,
            prev.t,
            s.t,
            &g,
            &quiet,
        );
        let mut pieces = start;
        let cuts = [prev.t, 13_100_000, 31_700_000, 32_000_000, 57_300_000, s.t];
        for c in cuts.windows(2) {
            advance(
                &mut pieces,
                Some(&before),
                Some(&prev),
                &s,
                c[0],
                c[1],
                &g,
                &quiet,
            );
        }
        let largest = whole.cov.amax();
        let worst = (whole.cov - pieces.cov).amax();
        assert!(
            worst <= 1e-12 * largest,
            "differs by {worst:e} of {largest:e}: velocity-rotation {:e} {:e}",
            whole.cov[(VEL, ROT + 2)],
            pieces.cov[(VEL, ROT + 2)]
        );
    }

    /// A hole that starts at the oldest snapshot the history keeps is still bridged as it was
    /// once the history has moved on: pruning keeps the sample at that snapshot and the one before
    /// it, so a late frame exposed inside the hole gets, bit for bit, the prior of a history that
    /// never moved on, the hole's variance with it.
    #[test]
    fn a_hole_at_the_oldest_snapshot_keeps_its_samples() {
        let w = world(4);
        let (before, prev, s) = (sample(998.0, 0.8), sample(1000.0, 1.0), sample(1060.0, 1.4));
        let fed = |history_ns: u64| {
            let params = Params {
                history_ns,
                ..Params::default()
            };
            let mut tr = Tracker::new(1, w.body.clone(), params, w.cams.len());
            tr.history.push_back((prev.t, moving_filter()));
            tr.imu.extend([before, prev]);
            tr.still.extend([false, false]);
            tr.imu(s);
            tr
        };
        let (kept, pruned) = (fed(400_000_000), fed(20_000_000));
        // the cutoff (1.040 s) is past the 1.000 s snapshot, which stays as the oldest one
        assert_eq!(pruned.history.front().map(|h| h.0), Some(prev.t));
        let inside = 1_030_000_000;
        let original = kept.state_at(inside).map(|(_, f)| f);
        assert!(
            original.is_some_and(|f| f.cov[(VEL, VEL)] > moving_filter().cov[(VEL, VEL)] * 10.0)
        );
        assert_eq!(pruned.state_at(inside).map(|(_, f)| f), original);
    }

    /// Holes past the fitted range (it was measured to 60 ms) up to the reset at `max_imu_gap_ns`
    /// keep the covariance finite, whatever the rates entering them.
    #[test]
    fn a_long_hole_keeps_the_covariance_finite() {
        let (g, n) = (Vector3::new(0.0, -G, 0.0), Noise::default());
        let rail = ImuSample {
            t: 8_000_000,
            hidden: None,
            gyro: Vector3::repeat(-35.0),
            accel: Vector3::repeat(-157.0),
        };
        let prev = ImuSample {
            t: 10_000_000,
            hidden: None,
            gyro: Vector3::repeat(35.0),
            accel: Vector3::repeat(157.0),
        };
        let s = ImuSample {
            t: 110_000_000,
            ..prev
        };
        let mut f = moving_filter();
        advance(&mut f, Some(&rail), Some(&prev), &s, prev.t, s.t, &g, &n);
        assert!(f.cov.iter().all(|c| c.is_finite()) && f.x.p.iter().all(|c| c.is_finite()));
        let (vel, rot) = hole_variance(Some(&rail), &prev, &s);
        assert!(vel.is_finite() && rot.is_finite() && vel > 0.0 && rot > 0.0);
    }

    #[test]
    fn imu_samples_lost_over_the_radio_are_counted_but_not_jitter_or_a_long_silence() {
        let w = world(8);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        // 3 ms and 1 ms is stamp jitter; 4 ms loses one sample, 10 ms four; 150 ms is a gap
        let mut ms = 0;
        tr.imu(imu(&w, 0.0));
        for step in [2, 2, 3, 1, 2, 4, 2, 10, 2, 150, 2, 2] {
            ms += step;
            tr.imu(imu(&w, f64::from(ms) * 1e-3));
        }
        assert_eq!(
            (tr.stats.imu_samples, tr.stats.imu_lost),
            (13, 5),
            "{:?}",
            tr.stats
        );
        assert_eq!(tr.stats.imu_gaps, 1);
    }

    /// A controller lying in view of three sensors is flicked at 2 s: a turn about its x axis
    /// whose rate peaks at 50 rad/s over 40 ms, of which the gyro, saturating at 34.9 rad/s,
    /// misses about 8 degrees. The orientation errors (rad) of the outputs from `after` seconds
    /// past the flick on, with the tracker told where the gyro's readings end and not told.
    fn after_a_flick(after: f64) -> [Vec<f64>; 2] {
        let mut w = world(3);
        w.motion =
            Motion::still(Vector3::new(0.0, 1.0, 0.0), [0.0; 3]).with_flick(2.0, 0.04, 0.0, 50.0);
        [true, false].map(|told| {
            let (tr, errors) = run_with(
                &w,
                2.4,
                0.3,
                &Run {
                    clipped: Some(told),
                    rotation: true,
                    measure_after: Some(2.04 + after),
                    ..Run::default()
                },
            );
            assert_eq!((tr.stats.acquisitions, tr.stats.wrong_locks), (1, 0));
            errors
        })
    }

    /// The turn the gyro's rail hides is taken out by the first fixes after it: told where the
    /// gyro's readings end, the shown orientation is within a degree of the truth from the first
    /// fix after the flick on and within a quarter of a degree 50 ms later; not told, the filter
    /// believes the clipped turn and is still more than 4 degrees off 0.35 s later.
    #[test]
    fn a_turn_beyond_the_gyros_rail_is_corrected_at_once() {
        let worst = |e: &[f64]| e.iter().fold(0.0f64, |a, &b| a.max(b)).to_degrees();
        // frames arrive 25 ms after their exposure: by 35 ms after the flick, two have landed
        let [told, not] = after_a_flick(0.035);
        assert!(worst(&told) < 1.0, "{:.2} deg", worst(&told));
        assert!(worst(&not) > 7.0, "{:.2} deg", worst(&not));
        let [told, not] = after_a_flick(0.085);
        assert!(worst(&told) < 0.25, "{:.2} deg", worst(&told));
        let best = not
            .iter()
            .fold(f64::INFINITY, |a, &b| a.min(b))
            .to_degrees();
        assert!(best > 4.0, "{best:.2} deg");
    }

    /// A spin held at 37 rad/s for 0.2 s, just past the gyro's rail of 34.9 rad/s, while fixes
    /// keep landing: each fix leaves the filter sure of the orientation, and the turn the rail
    /// hides after it is unknown again, so the uncertainty is taken on from the fix, not from
    /// where the reading reached the rail. Within a degree of the truth from 50 ms after the
    /// spin; if the uncertainty stopped growing once the rail could have hidden half a turn,
    /// the fixes left in the spin would leave several degrees.
    #[test]
    fn a_long_stretch_at_the_rail_is_owned_past_every_fix() {
        let mut w = world(3);
        w.motion =
            Motion::still(Vector3::new(0.0, 1.0, 0.0), [0.0; 3]).with_flick(2.0, 0.04, 0.2, 37.0);
        let (tr, errors) = run_with(
            &w,
            2.6,
            0.3,
            &Run {
                clipped: Some(true),
                rotation: true,
                measure_after: Some(2.29),
                ..Run::default()
            },
        );
        assert_eq!((tr.stats.acquisitions, tr.stats.wrong_locks), (1, 0));
        assert!(errors.len() > 100);
        let worst = errors.iter().fold(0.0f64, |a, &b| a.max(b)).to_degrees();
        assert!(worst < 1.0, "{worst:.2} deg");
    }

    /// While the gyro stays below its rails, telling the tracker where they are changes nothing,
    /// to the bit.
    #[test]
    fn below_its_rails_the_tracker_is_unchanged() {
        let w = world(5);
        let run = |clipped| {
            run_with(
                &w,
                3.0,
                0.3,
                &Run {
                    clipped,
                    ..Run::default()
                },
            )
        };
        let ((told, a), (untold, b)) = (run(Some(true)), run(None));
        assert!(told.gyro_range.is_some() && told.stats.acquisitions > 0);
        assert!(told.imu.iter().all(|s| s.hidden.is_none()));
        let bits = |e: &[f64]| e.iter().map(|e| e.to_bits()).collect::<Vec<_>>();
        assert_eq!(bits(&a), bits(&b));
        super::stillness_tests::assert_state(&told, &untold);
    }

    /// An axis at its rail misses, on that axis alone, a rate rising at the steeper approach
    /// over the two intervals before, from where it crossed the rail; each sample's excess is
    /// held over its interval. Leaving the rail ends it.
    #[test]
    fn a_reading_at_the_rail_owns_what_it_can_hide() {
        let w = world(1);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        tr.gyro_range = Some(Gyro::default());
        let at = |t_ms: u64, x: f64| ImuSample {
            t: t_ms * 1_000_000,
            gyro: Vector3::new(x * GYRO_PER_COUNT, 0.3, -0.2),
            accel: Vector3::new(0.0, G, 0.0),
            hidden: None,
        };
        let rail = f64::from(i16::MAX);
        let mut hidden = Vec::new();
        for s in [
            at(8, rail - 6000.0),
            at(10, rail - 2000.0),
            at(12, rail),
            at(14, rail),
            at(18, rail - 1.0),
            at(20, rail - 3000.0),
        ] {
            tr.imu(s);
            hidden.push(tr.imu.back().unwrap().hidden);
        }
        assert_eq!((hidden[0], hidden[1], hidden[5]), (None, None, None));
        // the approach: 2000 counts/ms, then 1000 to the rail, which it crossed at 11 ms; the
        // excess is 2000 counts at 12 ms, 6000 at 14 ms and 14000 at 18 ms
        for (h, excess) in [
            (hidden[2], 2000.0),
            (hidden[3], 6000.0),
            (hidden[4], 14000.0),
        ] {
            let h = h.unwrap();
            assert!(
                (h[(0, 0)] / (excess * GYRO_PER_COUNT) - 1.0).abs() < 1e-9,
                "{h}"
            );
            assert_eq!(h.iter().filter(|&&v| v != 0.0).count(), 1);
        }
    }

    /// The radio drops for 400 ms (an IMU reset) while the sensors keep exposing. The frames of
    /// the gap wait for IMU, as the daemon's pending frames do, and arrive with the first sample
    /// after it. None of them may start the new track: no IMU carries a state from their exposure
    /// across the gap, so one would show the controller turned by the whole gap's motion.
    #[test]
    fn frames_exposed_before_an_imu_reset_do_not_start_the_new_track() {
        let w = world(8);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let mut rng = Rng::new(7);
        let (gap, dt_frame) = ((2.0, 2.4), 1.0 / 52.0);
        let mut next_frame = 0.01;
        let mut t = 0.0;
        let mut worst: f64 = 0.0;
        while t < 3.0 {
            t += 0.002;
            if t > gap.0 && t < gap.1 {
                continue;
            }
            tr.imu(imu(&w, t));
            while next_frame <= t {
                for (c, cam) in w.cams.iter().enumerate() {
                    let blobs = sight(&w, cam, next_frame, &mut rng, 0.3);
                    let engine = EnginePose::strong(w.motion.pose(next_frame));
                    let t_e = (1e9 + next_frame * 1e9) as u64;
                    tr.frame(t_e, c, cam, &blobs, Some(engine), &[]);
                }
                next_frame += dt_frame;
            }
            if t > 1.0 {
                if let Some(o) = tr.output().filter(|o| o.tracking) {
                    let truth = w.motion.pose(t).rotation;
                    worst = worst.max(o.world_from_model.rotation.angle_to(&truth));
                }
            }
        }
        assert_eq!(tr.stats.imu_gaps, 1, "{:?}", tr.stats);
        assert!(tr.tracking(), "{:?}", tr.stats);
        assert!(
            worst < 3f64.to_radians(),
            "worst tracked rotation error {} deg {:?}",
            worst.to_degrees(),
            tr.stats
        );
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
        assert!(tr.explains_frame_baseline(cam, &blobs, &fit));
        let mut strays = fit.clone();
        strays.unexplained = 2;
        assert!(!tr.explains_frame(cam, &blobs, &strays));
        assert!(!tr.explains_frame_baseline(cam, &blobs, &strays));
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
        assert!(!tr.explains_frame_baseline(cam, &loose, &fit));
        assert!(!tr.explains_frame_baseline(cam, &blobs, &part));
    }

    #[test]
    fn visible_count_matches_association_report() {
        let mut checked = 0;
        for seed in 1..=8 {
            let w = touch_world(seed);
            let tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
            let mut rng = Rng::new(seed);
            for step in 0..100 {
                let t = step as f64 * 0.073;
                let f = tr.acquisition_prior(&w.motion.pose(t), None);
                for cam in &w.cams {
                    let blobs = sight(&w, cam, t, &mut rng, 0.2);
                    for facing in [-1.0, 0.0, 0.2, 0.9, 1.0] {
                        let mut p = tr.params.fit.assoc;
                        p.min_facing = facing;
                        assert_eq!(
                            vision::association_report(cam, &w.body, &f, &blobs, 1.0, &p, 3.0).0,
                            vision::visible_leds(cam, &w.body, &f.x, &p),
                            "seed={seed}, step={step}, facing={facing}"
                        );
                        checked += 1;
                    }
                }
            }
        }
        println!("equivalent cases: {checked}");
    }

    #[test]
    #[ignore]
    fn bench_explains_frame_visible_count() {
        use std::{hint::black_box, time::Instant};
        let w = touch_world(3);
        let tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let mut rng = Rng::new(5);
        let mut cases = Vec::new();
        for step in 0..100 {
            let t = step as f64 * 0.073;
            for cam in &w.cams {
                let blobs = sight(&w, cam, t, &mut rng, 0.2);
                let labels = blobs
                    .iter()
                    .enumerate()
                    .map(|(i, b)| ((b.led_id & 0xff) as usize, i))
                    .collect();
                if let Some((_, fit)) =
                    tr.attempt(cam, &blobs, &w.motion.pose(t), labels, None, &tr.params.fit)
                {
                    cases.push((*cam, blobs, fit));
                }
            }
        }
        assert!(cases.len() > 20);
        for (cam, blobs, fit) in &cases {
            assert_eq!(
                tr.explains_frame_baseline(cam, blobs, fit),
                tr.explains_frame(cam, blobs, fit)
            );
            let mut rejected = fit.clone();
            rejected.unexplained = usize::MAX;
            assert_eq!(
                tr.explains_frame_baseline(cam, blobs, &rejected),
                tr.explains_frame(cam, blobs, &rejected)
            );
        }
        println!("benchmark cases: {}", cases.len());
        for round in 0..9 {
            for new in if round % 2 == 0 {
                [false, true]
            } else {
                [true, false]
            } {
                let start = Instant::now();
                for _ in 0..200 {
                    for (cam, blobs, fit) in &cases {
                        let (cam, blobs, fit) =
                            (black_box(cam), black_box(blobs.as_slice()), black_box(fit));
                        black_box(if new {
                            tr.explains_frame(cam, blobs, fit)
                        } else {
                            tr.explains_frame_baseline(cam, blobs, fit)
                        });
                    }
                }
                println!(
                    "round={round} new={new} ns/call={:.1}",
                    start.elapsed().as_nanos() as f64 / (200 * cases.len()) as f64
                );
            }
        }
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
                identified: false,
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

    /// The engine takes the other controller, tracked 10 cm away, for this one, which no sensor
    /// sees: its blobs under this controller's labels, its pose rated strong. Fitted, they
    /// explain the frame as well as this controller's would; only the other's track tells them
    /// apart, and no track is started from them.
    #[test]
    fn the_other_controllers_blobs_under_this_ones_labels_start_no_track() {
        let w = world(3);
        let other = Other {
            place: beside,
            mislabelled: true,
            seen_from: 0,
            pos_sd: 0.0005,
        };
        let (tr, errors) = run_with(
            &w,
            3.0,
            0.3,
            &Run {
                other: Some((0.0, other)),
                ..Run::default()
            },
        );
        assert!(errors.is_empty() && !tr.tracking(), "{:?}", tr.stats);
        assert_eq!(tr.stats.acquisitions, 0, "{:?}", tr.stats);
        assert!(tr.stats.refused_acquisitions > 50, "{:?}", tr.stats);
    }

    /// As above with good poses, and with the first sensor not yet among those that see the other
    /// controller: there the blobs start a track, which waits for a second sensor. The others fit
    /// the same blobs, but the other controller's established track explains them better, and
    /// nothing confirms the track.
    #[test]
    fn a_second_sensor_fitting_the_other_controllers_blobs_confirms_nothing() {
        let w = world(3);
        let other = Other {
            place: beside,
            mislabelled: true,
            seen_from: 1,
            pos_sd: 0.0005,
        };
        let (tr, errors) = run_with(
            &w,
            3.0,
            0.3,
            &Run {
                engine_good: true,
                other: Some((0.0, other)),
                ..Run::default()
            },
        );
        assert!(errors.is_empty() && !tr.tracking(), "{:?}", tr.stats);
        assert!(
            tr.stats.good_acquisitions > 0
                && tr.stats.unconfirmed_acquisitions + 1 >= tr.stats.good_acquisitions,
            "{:?}",
            tr.stats
        );
    }

    /// The controller is lost from sight while the other moves in where it was, 4 cm off, under
    /// this controller's labels: within the re-lock window its dead-reckoned state finds them,
    /// but the other controller's track explains them better, and it does not re-lock.
    #[test]
    fn a_lost_controller_does_not_relock_onto_the_other_controllers_blobs() {
        fn near(pose: &Isometry3<f64>, _: &Camera) -> Isometry3<f64> {
            Translation3::new(0.04, 0.0, 0.0) * pose
        }
        let w = world(3);
        let other = Other {
            place: near,
            mislabelled: true,
            seen_from: 0,
            pos_sd: 0.0005,
        };
        let (tr, errors) = run_with(
            &w,
            5.0,
            0.3,
            &Run {
                other: Some((3.0, other)),
                ..Run::default()
            },
        );
        assert!(!tr.tracking(), "{:?}", tr.stats);
        assert_eq!(
            (tr.stats.acquisitions, tr.stats.relocks),
            (1, 0),
            "{:?}",
            tr.stats
        );
        // tracked from 1.0 s until lost 0.5 s after the other took its place
        assert!(errors.len() < 1260, "{} tracked", errors.len());
    }

    /// The other controller is held 10 cm behind this one on the first sensor's line of sight: its
    /// LEDs lie a pixel or less from this controller's in that sensor's image, nearly all within
    /// their association gates. This controller's track explains its own blobs better, and that
    /// sensor's frames keep updating it.
    #[test]
    fn a_controller_overlapping_the_other_in_the_image_keeps_its_track() {
        let w = world(3);
        let other = Other {
            place: behind,
            mislabelled: false,
            seen_from: 0,
            pos_sd: 0.0005,
        };
        let (tr, errors) = run_with(
            &w,
            5.0,
            0.3,
            &Run {
                other: Some((2.0, other)),
                ..Run::default()
            },
        );
        assert!(tr.tracking(), "{:?}", tr.stats);
        assert_eq!(tr.stats.acquisitions, 1, "{:?}", tr.stats);
        assert!(
            tr.seen_by(0, tr.newest_imu().unwrap()),
            "{:?}",
            tr.camera_updates
        );
        assert!(pct(&errors, 0.99) < 4e-3, "{:?}", tr.stats);
    }

    /// As above with the first sensor alone, and the other controller's track uncertain (5 mm):
    /// the blobs in its gates are better explained by this controller's track, and still count.
    #[test]
    fn a_contested_blob_our_track_explains_better_still_counts() {
        let w = world(3);
        let other = Other {
            place: behind,
            mislabelled: false,
            seen_from: 0,
            pos_sd: 0.005,
        };
        let (tr, errors) = run_with(
            &w,
            5.0,
            0.3,
            &Run {
                cameras_after: Some((2.0, 1)),
                other: Some((2.0, other)),
                ..Run::default()
            },
        );
        assert!(tr.tracking(), "{:?}", tr.stats);
        assert_eq!(tr.stats.acquisitions, 1, "{:?}", tr.stats);
        assert!(
            errors.len() > 1990,
            "{} tracked {:?}",
            errors.len(),
            tr.stats
        );
    }

    /// The kept orientation went wrong (60 degrees) while the controller was lost, and only one
    /// sensor sees it again, its engine pose refused by the kept orientation. One sensor's poses
    /// alone, however long refused, are no proof that the memory is wrong (they can be the other
    /// controller's LEDs under this one's labels): it is kept, and nothing is acquired.
    #[test]
    fn one_sensors_refused_poses_keep_the_orientation_memory() {
        let w = touch_world(7);
        let (tr, _) = run_with(
            &w,
            8.0,
            0.3,
            &Run {
                blackout: Some((3.0, 6.5)),
                turn_memory_at: Some((4.0, Vector3::y() * 60f64.to_radians())),
                cameras_after: Some((6.5, 1)),
                ..Run::default()
            },
        );
        assert!(tr.stats.refused_acquisitions > 50, "{:?}", tr.stats);
        assert_ne!(tr.last_confirmed, 0, "{:?}", tr.stats);
        assert_eq!(tr.stats.acquisitions, 1, "{:?}", tr.stats);
    }

    /// The kept orientation went wrong (60 degrees) while the controller was lost. Two sensors
    /// then show the controller, their engine poses the same and refused by it: past
    /// `max_refusing_ns` the memory is dropped and the controller taken where they put it.
    #[test]
    fn two_sensors_agreeing_drop_the_orientation_memory() {
        let w = touch_world(7);
        let (tr, errors) = run_with(
            &w,
            8.0,
            0.3,
            &Run {
                blackout: Some((3.0, 6.5)),
                turn_memory_at: Some((4.0, Vector3::y() * 60f64.to_radians())),
                cameras_after: Some((6.5, 2)),
                measure_after: Some(6.5),
                ..Run::default()
            },
        );
        assert!(tr.tracking(), "{:?}", tr.stats);
        assert_eq!(tr.stats.acquisitions, 2, "{:?}", tr.stats);
        // shown from max_refusing_ns after the sensors' first frames
        assert!(errors.len() > 400, "{} tracked", errors.len());
        assert!(pct(&errors, 0.5) < 1e-3, "{:?}", tr.stats);
    }

    /// As above, but the second sensor's engine pose is turned a quarter turn from the first's:
    /// both refused, they are not the same pose, and the memory is kept.
    #[test]
    fn two_sensors_disagreeing_keep_the_orientation_memory() {
        let w = touch_world(7);
        let (tr, _) = run_with(
            &w,
            8.0,
            0.3,
            &Run {
                blackout: Some((3.0, 6.5)),
                turn_memory_at: Some((4.0, Vector3::y() * 60f64.to_radians())),
                cameras_after: Some((6.5, 2)),
                engine_flipped: Some((6.5, 8.0)),
                flip_from: 1,
                ..Run::default()
            },
        );
        assert!(tr.stats.refused_acquisitions > 50, "{:?}", tr.stats);
        assert_ne!(tr.last_confirmed, 0, "{:?}", tr.stats);
        assert_eq!(tr.stats.acquisitions, 1, "{:?}", tr.stats);
    }

    /// The other controller's track is uncertain enough that its gate reaches past
    /// `AssocParams::max_px`: a blob farther than that from its LED is no candidate for it, as the
    /// association would not pair them either, and still counts for this controller.
    #[test]
    fn a_blob_beyond_max_px_of_the_other_controllers_led_counts() {
        let w = world(3);
        let tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let (cam, pose) = (&w.cams[0], w.motion.pose(1.0));
        let blobs = sight_posed(&w, cam, &pose, &mut Rng::new(1), 0.0);
        let pair = [((blobs[0].led_id & 0xff) as usize, 0)];
        let mut prior = tr.acquisition_prior(&pose, None);
        prior.cov *= 100.0;
        let p6 = prior.cov.fixed_view::<6, 6>(0, 0).into_owned();
        let pr = vision::project(cam, &w.body, &prior.x, pair[0].0).unwrap();
        let var = Matrix2::identity() * tr.params.fit.update.sigma_px.powi(2);
        let ours = prediction_fit(pr.uv, &(pr.j * p6 * pr.j.transpose() + var), blobs[0].xy);
        // just past max_px, at the variance that makes its likelihood there highest
        let off = tr.params.fit.assoc.max_px + 1.0;
        let theirs = ForeignLed {
            uv: blobs[0].xy + Vector2::new(off, 0.0),
            cov: Matrix2::identity() * (off * off / 2.0),
            seen: true,
        };
        let (d2, cost) = prediction_fit(theirs.uv, &theirs.cov, blobs[0].xy);
        assert!(
            d2 < tr.params.fit.assoc.gate && cost < ours.1,
            "{d2} {cost} {ours:?}"
        );
        assert_eq!(tr.evidence(cam, &blobs, &pair, &prior, &[theirs]), 1);
    }

    /// A frame arrives as late as the history allows: whether the other controller was seen by its
    /// sensor is looked up as far back from its exposure, and the update that shows it is kept.
    #[test]
    fn a_late_frame_still_finds_the_sensors_sighting() {
        let w = world(3);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        let ns = |t: f64| (1e9 + t * 1e9) as u64;
        tr.note_camera_update(0, ns(1.0));
        for k in 1..=725 {
            tr.imu(imu(&w, k as f64 * 0.002));
        }
        assert!(tr.seen_by(0, ns(1.15)), "{:?}", tr.camera_updates);
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

    /// A bias run off by an exposure's fixes drops the track when the next exposure's first frame
    /// comes, or once the IMU passes the last moment a frame of that exposure could still come.
    #[test]
    fn a_run_off_bias_is_tested_at_the_next_exposure() {
        let w = world(5);
        for by_frame in [true, false] {
            let (mut tr, _) = run_with(&w, 1.0, 0.3, &Run::default());
            assert!(tr.tracking() && tr.stats.wrong_locks == 0, "{:?}", tr.stats);
            let last = tr.applied.back().unwrap().t;
            tr.history.back_mut().unwrap().1.x.ba = Vector3::new(1.5, 0.0, 0.0);
            tr.bias_pending = Some(last);
            if by_frame {
                let te = last + 1_000_000_000 / 52;
                let pose = w.motion.pose((te - 1_000_000_000) as f64 * 1e-9);
                let blobs = sight_posed(&w, &w.cams[0], &pose, &mut Rng::new(3), 0.3);
                tr.frame(te, 0, &w.cams[0], &blobs, None, &[]);
                assert_eq!(tr.stats.wrong_locks, 1, "at the next exposure's frame");
            } else {
                let mut t = 1.0;
                while tr.stats.wrong_locks == 0 && t < 2.0 {
                    t += 0.002;
                    tr.imu(imu(&w, t));
                }
                let waited = (1e9 + t * 1e9) as u64 - last;
                assert!(
                    waited > tr.params.history_ns && waited <= tr.params.history_ns + 3_000_000,
                    "dropped {waited} ns after the exposure"
                );
            }
            assert!(!tr.tracking());
        }
    }

    /// With a sensor a few mm off its calibrated pose, each sensor's frame of one exposure pulls
    /// a cold track its own way while the controller moves fast, and the accelerometer bias swings
    /// with them past the limit and back. Only the state that holds all of an exposure's fixes is
    /// a wrong lock's evidence.
    #[test]
    fn a_bias_swing_between_the_frames_of_one_exposure_is_not_a_wrong_lock() {
        for (seed, sensor) in [(21, 2), (25, 0)] {
            let w = world(seed);
            let off = Isometry3::from_parts(
                Translation3::new(0.0048, 0.0064, 0.0),
                UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.25f64.to_radians(), 0.0)),
            );
            let opts = Run {
                misplaced: Some((sensor, off)),
                ..Run::default()
            };
            let (tr, _) = run_with(&w, 1.5, 0.3, &opts);
            assert_eq!(tr.stats.wrong_locks, 0, "seed {seed}: {:?}", tr.stats);
            assert!(tr.tracking(), "seed {seed}");
        }
    }

    /// An accelerometer bias the filter was started from is corrected by the fixes, however far
    /// off it was: moving away from it is not taken for a wrong lock (the track dropped and the
    /// stale bias restored for the next one to run off from again).
    #[test]
    fn a_stale_starting_accelerometer_bias_is_corrected_not_taken_for_a_wrong_lock() {
        let w = world(5);
        let stale = Run {
            seed: Some((w.bg, w.ba + Vector3::new(0.5, -0.4, 0.0))),
            ..Run::default()
        };
        let (tr, _) = run_with(&w, 6.0, 0.3, &stale);
        assert_eq!(tr.stats.wrong_locks, 0, "{:?}", tr.stats);
        assert!(tr.tracking());
        let ba = tr.history.back().unwrap().1.x.ba;
        assert!((ba - w.ba).norm() < 0.1, "ba {ba:?}, true {:?}", w.ba);
    }

    /// A track does not start from an accelerometer bias the wrong-lock test rejects, however the
    /// state it starts from came by it (a zero-velocity update on a state dead-reckoned for seconds
    /// out of view can leave one), and a drop does not restore it: each track would be dropped at
    /// its next exposure and the next one restored to the same bias, for as long as the controller
    /// is in view.
    #[test]
    fn a_track_never_starts_from_a_bias_the_wrong_lock_test_rejects() {
        let w = world(5);
        let ran_off = Run {
            seed: Some((w.bg, Vector3::new(1.3, 0.0, 0.0))),
            ..Run::default()
        };
        let (tr, _) = run_with(&w, 6.0, 0.3, &ran_off);
        assert_eq!(tr.stats.wrong_locks, 0, "{:?}", tr.stats);
        assert_eq!(tr.stats.acquisitions, 1, "{:?}", tr.stats);
        assert!(tr.tracking());
        let ba = tr.history.back().unwrap().1.x.ba;
        assert!((ba - w.ba).norm() < 0.1, "ba {ba:?}, true {:?}", w.ba);
        // a drop goes back to the bias the track started from: the fresh one
        let dropped = Run {
            drop_at: Some(2.0),
            ..ran_off
        };
        let (tr, _) = run_with(&w, 6.0, 0.3, &dropped);
        assert_eq!(tr.stats.wrong_locks, 1, "{:?}", tr.stats);
        assert!(!tr.bias_ran_off(&tr.saved_bias.unwrap()));
        assert!(tr.tracking(), "{:?}", tr.stats);
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

    /// A steady glide without a turn feels like rest to the IMU. Out of sight for 0.3 s, it keeps
    /// gliding instead of being braked by the zero-velocity update.
    #[test]
    fn a_steady_glide_out_of_sight_is_not_braked() {
        let mut w = world(5);
        w.motion = Motion::glide(
            Vector3::new(0.1, 1.0, -0.2),
            Vector3::new(0.1, 0.0, 0.0),
            [0.3, -0.5, 0.2],
        );
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
        tr.seed_biases(w.bg, w.ba, 1e-4, 1e-3);
        let mut rng = Rng::new(3);
        let (dt, frame) = (0.002, 1.0 / 52.0);
        let (mut t, mut next) = (0.0, 0.01);
        while t < 2.3 {
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
        assert!(tr.tracking());
        let f = tr.history.back().unwrap().1;
        let error = (tr.body.world_from_model(&f.x).translation.vector
            - w.motion.pose(t).translation.vector)
            .norm();
        assert!(
            error < 0.005,
            "{:.1} mm behind after 0.3 s out of sight",
            error * 1e3
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

    /// Frames of three sensors at 52 Hz over `seconds` of motion, the third sensor's blobs seen
    /// from `truly` while the tracker is told its calibrated pose: how each sensor agrees.
    fn agreement_with_third_sensor_at(truly: &Camera, seconds: f64) -> Vec<CameraAgreement> {
        agreement_with_third_sensor(truly, seconds, true)
    }

    /// As `agreement_with_third_sensor_at`; `labelled`: whether the engine labels the third
    /// sensor's blobs and rates a pose from it (it does not, for a controller our tracker holds
    /// that it cannot match itself).
    fn agreement_with_third_sensor(
        truly: &Camera,
        seconds: f64,
        labelled: bool,
    ) -> Vec<CameraAgreement> {
        let w = world(11);
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), 3);
        let mut rng = Rng::new(5);
        for tick in 1..(seconds * 500.0) as usize {
            let t = tick as f64 * 0.002;
            tr.imu(imu(&w, t));
            if tick % 10 == 0 {
                for camera in 0..3 {
                    let seen_from = if camera == 2 { truly } else { &w.cams[camera] };
                    let mut blobs = sight(&w, seen_from, t, &mut rng, 0.1);
                    let mut engine = Some(EnginePose::strong(
                        w.cams[camera].world_from_cam
                            * seen_from.world_from_cam.inverse()
                            * w.motion.pose(t),
                    ));
                    if camera == 2 && labelled {
                        engine.as_mut().unwrap().identified = true;
                    }
                    if camera == 2 && !labelled {
                        for b in &mut blobs {
                            b.led_id = -1;
                        }
                        engine = None;
                    }
                    tr.frame(
                        (1e9 + t * 1e9) as u64,
                        camera,
                        &w.cams[camera],
                        &blobs,
                        engine,
                        &[],
                    );
                }
            }
        }
        tr.agreement.clone()
    }

    /// A sensor moved a little since calibration still matches its LEDs, with a residual and a
    /// bias the sensors that stayed do not have.
    #[test]
    fn a_sensor_moved_a_little_shows_in_its_residual() {
        let w = world(11);
        let nudged = Camera {
            world_from_cam: Translation3::new(0.01, 0.0, 0.0) * w.cams[2].world_from_cam,
            ..w.cams[2]
        };
        let a = agreement_with_third_sensor_at(&nudged, 6.0);
        assert!(a[2].samples > 20, "{:?}", a[2]);
        assert!(
            a[2].median_px > 0.6 && a[2].bias_px.norm() > 0.5,
            "{:?}",
            a[2]
        );
        // the moved sensor pulls the track a little its way, so the others show some of it too,
        // but it stands out from them
        for stayed in &a[..2] {
            assert!(stayed.samples > 20, "{stayed:?}");
            assert!(
                a[2].median_px > 1.3 * stayed.median_px
                    && a[2].bias_px.norm() > 1.5 * stayed.bias_px.norm(),
                "{:?} vs {stayed:?}",
                a[2]
            );
        }
    }

    /// A sensor moved a lot sees the controller (the engine labels its LEDs) where the track,
    /// set by the other sensors, projects nothing near: no matches, so its residual stays silent,
    /// but every such frame is a check it missed.
    #[test]
    fn a_sensor_moved_a_lot_misses_while_the_engine_sees_the_controller() {
        let w = world(11);
        let moved = Camera {
            world_from_cam: Translation3::new(0.4, 0.0, 0.0) * w.cams[2].world_from_cam,
            ..w.cams[2]
        };
        let a = agreement_with_third_sensor_at(&moved, 6.0);
        assert!(a[2].observed_checks > 20, "{:?}", a[2]);
        assert!(
            a[2].observed_misses as f64 >= 0.8 * a[2].observed_checks as f64,
            "{:?}",
            a[2]
        );
        assert!(a[2].checks > 20, "{:?}", a[2]);
        assert!(a[2].misses as f64 >= 0.8 * a[2].checks as f64, "{:?}", a[2]);
        for stayed in &a[..2] {
            assert!(stayed.checks > 20, "{stayed:?}");
            assert!(
                (stayed.misses as f64) < 0.1 * stayed.checks as f64,
                "{stayed:?}"
            );
        }
    }

    #[test]
    fn camera_rotations_are_checked_through_the_disputed_calibration() {
        let w = world(11);
        for degrees in [5f64, 10.0, 20.0, 45.0] {
            let mut moved = w.cams[2];
            moved.world_from_cam.rotation *=
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), degrees.to_radians());
            let a = agreement_with_third_sensor_at(&moved, 6.0);
            assert!(a[2].observed_checks > 40, "{degrees} degrees: {:?}", a[2]);
            assert!(
                a[2].observed_misses > a[2].observed_checks * 3 / 5,
                "{degrees} degrees: {:?}",
                a[2]
            );
        }
    }

    /// The opposite hand's mirrored LED model can receive this hand's labels from a strong
    /// cold search. Its pose still disagrees with this hand's independent attitude.
    #[test]
    fn mirrored_controller_labels_are_unknown_for_the_other_hand() {
        for device in [1, 2] {
            for delay_s in [0, 1, 5, 20] {
                let mut w = touch_world(11);
                w.motion = Motion::still(Vector3::new(0.0, 1.0, 0.0), [0.3, -0.5, 0.2]);
                let mut other = touch_world(11);
                for led in &mut other.body.leds {
                    led.pos.x = -led.pos.x;
                    led.dir.x = -led.dir.x;
                }
                other.motion = Motion::still(
                    Vector3::new(0.24, 1.0, 0.0),
                    [0.3, -0.5 + 120f64.to_radians(), 0.2],
                );
                let mut tr = Tracker::new(device, w.body.clone(), Params::default(), 3);
                let mut rng = Rng::new(5);
                let mut delayed = Vec::new();
                let exposure = 3_000_000_000;
                for tick in 1..=1000 + delay_s * 500 {
                    let t = tick as f64 * 0.002;
                    tr.imu(imu(&w, t));
                    if tick % 10 == 0 {
                        for camera in 0..2 {
                            let mut blobs = sight(&w, &w.cams[camera], t, &mut rng, 0.1);
                            for b in &mut blobs {
                                b.led_id = (device << 8) | (b.led_id & 0xff);
                            }
                            tr.frame(
                                (1e9 + t * 1e9) as u64,
                                camera,
                                &w.cams[camera],
                                &blobs,
                                Some(EnginePose::strong(w.motion.pose(t))),
                                &[],
                            );
                        }
                    }
                    if tick == 1000 {
                        tr.hold_inspection(2, exposure, 30_000_000_000);
                        delayed = sight(&other, &w.cams[2], t, &mut rng, 0.1);
                        for b in &mut delayed {
                            b.led_id = (device << 8) | (b.led_id & 0xff);
                        }
                    }
                }
                assert!(delayed.len() >= tr.params.acquire_min_leds);
                assert!(tr
                    .inspection_frames
                    .iter()
                    .any(|f| f.camera == 2 && f.peers));
                tr.frame(
                    exposure,
                    2,
                    &w.cams[2],
                    &delayed,
                    Some(EnginePose::strong(other.motion.pose(2.0))),
                    &[],
                );
                let a = tr.agreement[2];
                assert_eq!(
                    a.observed_checks, 0,
                    "device {device}, delay {delay_s}s: {a:?}"
                );
                assert_eq!(
                    a.observed_misses, 0,
                    "device {device}, delay {delay_s}s: {a:?}"
                );
                assert_eq!(a.samples, 0, "a wrong hand is no matched evidence");
            }
        }
    }

    /// A recorded native search labelled the right hand while only the left was seen:
    /// 253 then 249 misses, with a 119.47--120.04 degree attitude error (first: 119.67375).
    /// Rebuild those windows with synthetic geometry: no room poses or raw captures are kept.
    #[test]
    fn a_replayed_wrong_hand_inspection_does_not_report_movement() {
        use crate::track::repose::{Change, Monitor, WINDOW_NS};
        let mut w = world(11);
        w.motion = Motion::still(Vector3::new(0.0, 1.0, 0.0), [0.3, -0.5, 0.2]);
        let other = Camera {
            world_from_cam: Translation3::new(0.238, 0.0, 0.0) * w.cams[2].world_from_cam,
            ..w.cams[2]
        };
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), 3);
        let mut rng = Rng::new(5);
        let mut m = Monitor::new(WINDOW_NS);
        let mut changes = Vec::new();
        for window in 0..=4 {
            let count = [50, 250, 250, 253, 249][window];
            for i in 1..=count {
                let t = window as f64 * 5.0 + i as f64 * 5.0 / count as f64;
                // All cameras see the same exposure; peers confirm this hand first.
                tr.imu(imu(&w, t));
                for camera in 0..3 {
                    let mut blobs = sight(
                        &w,
                        if camera == 2 { &other } else { &w.cams[camera] },
                        t,
                        &mut rng,
                        0.1,
                    );
                    let mut engine = Some(EnginePose::strong(w.motion.pose(t)));
                    if camera == 2 {
                        if window < 3 {
                            engine = None;
                            for b in &mut blobs {
                                b.led_id = -1;
                            }
                        } else {
                            let mut pose = w.motion.pose(t);
                            pose.rotation *= UnitQuaternion::from_axis_angle(
                                &Vector3::y_axis(),
                                119.67375f64.to_radians(),
                            );
                            engine = Some(EnginePose::strong(pose));
                        }
                    }
                    tr.frame(
                        (1e9 + t * 1e9) as u64,
                        camera,
                        &w.cams[camera],
                        &blobs,
                        engine,
                        &[],
                    );
                }
            }
            let agreement: Vec<_> = tr
                .agreement
                .iter()
                .enumerate()
                .map(|(i, a)| (format!("sensor-{i}"), [*a, CameraAgreement::default()]))
                .collect();
            changes.extend(m.update(1_000 + window as u64 * WINDOW_NS, &agreement));
        }
        assert_eq!(changes, [Change::Inspect("sensor-2".into())]);
        assert_eq!(tr.agreement[2].observed_misses, 0);
    }

    /// A labelled long-search frame arrives 100 ms after exposure, after both peers have
    /// delivered newer frames. The peer updates just before that exposure still check its pose.
    #[test]
    fn delayed_labelled_frames_use_peer_confirmation_at_exposure() {
        let w = world(11);
        let moved = Camera {
            world_from_cam: Translation3::new(0.4, 0.0, 0.0) * w.cams[2].world_from_cam,
            ..w.cams[2]
        };
        let mut tr = Tracker::new(1, w.body.clone(), Params::default(), 3);
        let mut rng = Rng::new(5);
        let mut delayed = VecDeque::new();
        for tick in 1..3000 {
            let t = tick as f64 * 0.002;
            tr.imu(imu(&w, t));
            if tick % 10 == 0 {
                for camera in 0..2 {
                    let blobs = sight(&w, &w.cams[camera], t, &mut rng, 0.1);
                    tr.frame(
                        (1e9 + t * 1e9) as u64,
                        camera,
                        &w.cams[camera],
                        &blobs,
                        Some(EnginePose::strong(w.motion.pose(t))),
                        &[],
                    );
                }
                delayed.push_back((tick, t, sight(&w, &moved, t, &mut rng, 0.1)));
            }
            if delayed
                .front()
                .is_some_and(|(captured, _, _)| tick >= captured + 50)
            {
                let (_, exposure, blobs) = delayed.pop_front().unwrap();
                tr.frame(
                    (1e9 + exposure * 1e9) as u64,
                    2,
                    &w.cams[2],
                    &blobs,
                    Some(EnginePose::strong(w.motion.pose(exposure))),
                    &[],
                );
            }
        }
        let a = tr.agreement[2];
        assert!(a.observed_checks > 100, "{a:?}");
        assert!(
            a.observed_misses as f64 > 0.8 * a.observed_checks as f64,
            "{a:?}"
        );
    }

    #[test]
    fn inspection_labels_delivered_seconds_late_still_check_the_exposure() {
        for delay_s in [1, 5, 20] {
            let w = world(11);
            let moved = Camera {
                world_from_cam: Translation3::new(0.4, 0.0, 0.0) * w.cams[2].world_from_cam,
                ..w.cams[2]
            };
            let mut tr = Tracker::new(1, w.body.clone(), Params::default(), 3);
            let mut rng = Rng::new(5);
            let exposure = 3_000_000_000;
            let mut delayed = Vec::new();
            for tick in 1..=1000 + delay_s * 500 {
                let t = tick as f64 * 0.002;
                tr.imu(imu(&w, t));
                if tick % 10 == 0 {
                    for camera in 0..2 {
                        let blobs = sight(&w, &w.cams[camera], t, &mut rng, 0.1);
                        tr.frame(
                            (1e9 + t * 1e9) as u64,
                            camera,
                            &w.cams[camera],
                            &blobs,
                            Some(EnginePose::strong(w.motion.pose(t))),
                            &[],
                        );
                    }
                }
                if tick == 1000 {
                    tr.hold_inspection(2, exposure, 30_000_000_000);
                    delayed = sight(&w, &moved, t, &mut rng, 0.1);
                }
            }
            assert!(
                tr.state_at(exposure).is_none(),
                "normal history stays short"
            );
            let last_fix = tr.last_fix;
            tr.frame(
                exposure,
                2,
                &w.cams[2],
                &delayed,
                Some(EnginePose::strong(w.motion.pose(2.0))),
                &[],
            );
            assert_eq!(tr.agreement[2].observed_checks, 1, "{delay_s}s delivery");
            assert_eq!(tr.agreement[2].observed_misses, 1, "{delay_s}s delivery");
            assert_eq!(
                tr.last_fix, last_fix,
                "old labels cannot rewind the live filter"
            );
            assert!(
                tr.inspection_frames.is_empty(),
                "completion releases its snapshot"
            );
            let newest = tr.newest_imu().unwrap();
            for _ in 0..8 {
                tr.hold_inspection(2, newest, 30_000_000_000);
            }
            assert_eq!(
                tr.inspection_frames.len(),
                1,
                "repeated notification is bounded"
            );
            for back in (0..8).rev() {
                tr.hold_inspection(2, newest - back * 2_000_000, 30_000_000_000);
            }
            assert_eq!(tr.inspection_frames.len(), 4, "native capture-buffer bound");
            tr.cancel_inspection(2);
            assert!(
                tr.inspection_frames.is_empty(),
                "cancellation releases its snapshots"
            );
            tr.hold_inspection(2, newest, 1);
            tr.imu(imu(&w, 2.0 + delay_s as f64 + 0.002));
            assert!(
                tr.inspection_frames.is_empty(),
                "deadline releases lost completions"
            );
        }
    }

    /// A controller held right at a sensor is no check of that sensor: its frames count neither
    /// as checks nor as misses, whatever the fit does, while the far sensors are checked as usual;
    /// a sensor a metre away is checked.
    #[test]
    fn a_controller_right_at_the_sensor_is_no_check() {
        let checks_with_third_sensor_at = |distance: f64| {
            let mut w = world(11);
            w.motion = Motion::still(Vector3::new(0.0, 1.0, 0.0), [0.3, -0.5, 0.2]);
            w.cams[2] = Camera {
                world_from_cam: synth::look_at(
                    Vector3::new(0.0, 1.0, distance),
                    Vector3::new(0.0, 1.0, 0.0),
                    false,
                ),
                ..w.cams[2]
            };
            let mut tr = Tracker::new(1, w.body.clone(), Params::default(), 3);
            let mut rng = Rng::new(5);
            for tick in 1..3000 {
                let t = tick as f64 * 0.002;
                tr.imu(imu(&w, t));
                if tick % 10 == 0 {
                    for camera in 0..3 {
                        let blobs = sight(&w, &w.cams[camera], t, &mut rng, 0.1);
                        tr.frame(
                            (1e9 + t * 1e9) as u64,
                            camera,
                            &w.cams[camera],
                            &blobs,
                            Some(EnginePose::strong(w.motion.pose(t))),
                            &[],
                        );
                    }
                }
            }
            (tr.agreement[2].checks, tr.agreement[0].checks)
        };
        let (near, far) = checks_with_third_sensor_at(0.4);
        assert_eq!(near, 0, "a sensor 0.4 m away");
        assert!(far > 20, "{far}");
        let (metre, _) = checks_with_third_sensor_at(1.0);
        assert!(metre > 20, "a sensor a metre away: {metre}");
    }

    /// The engine labels nothing in a sensor that moved a lot (it searches no further for a
    /// controller our tracker holds): a frame full of blobs where the track puts the controller
    /// requests an inspection, but its misses alone do not establish movement.
    #[test]
    fn a_sensor_moved_a_lot_misses_without_the_engines_labels() {
        let w = world(11);
        let moved = Camera {
            world_from_cam: Translation3::new(0.4, 0.0, 0.0) * w.cams[2].world_from_cam,
            ..w.cams[2]
        };
        let a = agreement_with_third_sensor(&moved, 6.0, false);
        assert_eq!(a[2].observed_misses, 0, "{:?}", a[2]);
        assert!(a[2].checks > 20, "{:?}", a[2]);
        assert!(a[2].misses as f64 >= 0.8 * a[2].checks as f64, "{:?}", a[2]);
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

#[cfg(test)]
pub(crate) mod stillness_tests;

#[cfg(test)]
#[path = "tracker/perf_tests.rs"]
mod perf_tests;
