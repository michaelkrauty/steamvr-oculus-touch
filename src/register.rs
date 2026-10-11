// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Quick registration of the cv1touch world to Lighthouse raw space: the user holds a Touch
//! against a Lighthouse-tracked controller and moves the pair.
//! With L_i the reference controller's raw pose and C_i the Touch's world pose at the same instant,
//! L_i * Y = X * C_i for X = lighthouse_from_world and Y = reference_from_touch (the hand-held
//! offset). Rotation from a hand-eye solve on relative motions, translation by linear least
//! squares, then Levenberg-Marquardt over both with Huber weights: a hand hold slips.

use nalgebra::{
    Isometry3, Matrix3, Matrix6, Rotation3, SMatrix, SVector, Translation3, UnitQuaternion,
    Vector3, Vector6,
};

use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::align::PoseEntry;
use crate::calib::motion::chordal_mean;
use crate::calib::{iso_from_xyzw, vec3};
use crate::shm::ControllerState;

#[derive(Debug, Clone, Copy)]
pub struct Options {
    /// Huber threshold on a pair's residual (m).
    pub huber_m: f64,
    /// Metres charged per radian of rotation disagreement.
    pub rot_weight_m: f64,
    pub min_pairs: usize,
    /// Second over first singular value of the hand-eye matrix: rotations must span two axes.
    pub min_axis_ratio: f64,
    /// Relative rotations used by the hand-eye solve are at least this large.
    pub min_relative_rad: f64,
    pub max_iterations: usize,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            huber_m: 0.005,
            rot_weight_m: 0.05,
            min_pairs: 1000,
            min_axis_ratio: 0.25,
            min_relative_rad: 0.35,
            max_iterations: 50,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Sample {
    pub t_ns: u64,
    /// The reference controller's raw-universe pose.
    pub reference: Isometry3<f64>,
    /// Each Touch's world pose (by hand index), when tracked.
    pub touch: [Option<Isometry3<f64>>; 2],
}

#[derive(Debug, Clone)]
pub struct Solution {
    pub lighthouse_from_world: Isometry3<f64>,
    pub reference_from_touch: Isometry3<f64>,
    pub pairs: usize,
    /// Pairs whose residual is within three Huber thresholds; the statistics below are theirs.
    pub inliers: usize,
    pub pos_rms_m: f64,
    pub rot_rms_deg: f64,
    pub axis_ratio: f64,
}

/// The Touch pose at `now_ns`: the published pose carried forward by its velocities (never back).
pub fn extrapolate(st: &ControllerState, now_ns: u64) -> Isometry3<f64> {
    let dt = ((now_ns as f64 - st.sample_host_ns as f64) * 1e-9).clamp(0.0, 0.05);
    let t = iso_from_xyzw(st.orient, st.pos);
    let w = vec3(st.ang_vel) - vec3(st.gyro_bias);
    Isometry3::from_parts(
        Translation3::from(t.translation.vector + vec3(st.lin_vel) * dt),
        t.rotation * UnitQuaternion::from_scaled_axis(w * dt),
    )
}

fn correlation(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len()) as f64;
    if n < 2.0 {
        return 0.0;
    }
    let (ma, mb) = (a.iter().sum::<f64>() / n, b.iter().sum::<f64>() / n);
    let (mut sab, mut saa, mut sbb) = (0.0, 0.0, 0.0);
    for (x, y) in a.iter().zip(b) {
        sab += (x - ma) * (y - mb);
        saa += (x - ma) * (x - ma);
        sbb += (y - mb) * (y - mb);
    }
    if saa > 0.0 && sbb > 0.0 {
        sab / (saa * sbb).sqrt()
    } else {
        0.0
    }
}

/// The Touch whose angular speed follows the reference's (correlation >= 0.7), if any.
pub fn pick_touch(samples: &[Sample]) -> Option<(usize, f64)> {
    let mut best: Option<(usize, f64)> = None;
    for hand in 0..2 {
        let (mut wr, mut wt) = (Vec::new(), Vec::new());
        for w in samples.windows(2) {
            let (Some(a), Some(b)) = (w[0].touch[hand], w[1].touch[hand]) else {
                continue;
            };
            let dt = (w[1].t_ns.saturating_sub(w[0].t_ns)) as f64 * 1e-9;
            if dt <= 0.0 {
                continue;
            }
            wr.push(w[0].reference.rotation.angle_to(&w[1].reference.rotation) / dt);
            wt.push(a.rotation.angle_to(&b.rotation) / dt);
        }
        let c = correlation(&wr, &wt);
        if c >= 0.7 && best.is_none_or(|(_, b)| c > b) {
            best = Some((hand, c));
        }
    }
    best
}

/// Angular speed between consecutive samples (rad/s), per sample index; NaN where a pose is missing.
fn angular_speeds(
    samples: &[Sample],
    pose: impl Fn(&Sample) -> Option<Isometry3<f64>>,
) -> Vec<f64> {
    samples
        .windows(2)
        .map(|w| {
            let dt = (w[1].t_ns.saturating_sub(w[0].t_ns)) as f64 * 1e-9;
            match (pose(&w[0]), pose(&w[1])) {
                (Some(a), Some(b)) if dt > 0.0 => a.rotation.angle_to(&b.rotation) / dt,
                _ => f64::NAN,
            }
        })
        .collect()
}

/// The sample shift that best lines the Touch stream up with the reference: Touch sample k + shift
/// shows what the reference shows at k (positive: the Touch stream lags). Searched over
/// +-max_shift samples by angular-speed correlation; returns the shift and its correlation.
pub fn estimate_lag(samples: &[Sample], hand: usize, max_shift: usize) -> (isize, f64) {
    let r = angular_speeds(samples, |s| Some(s.reference));
    let t = angular_speeds(samples, |s| s.touch[hand]);
    let mut best = (0isize, f64::MIN);
    for shift in -(max_shift as isize)..=(max_shift as isize) {
        let (mut a, mut b) = (Vec::new(), Vec::new());
        for (k, &reference_speed) in r.iter().enumerate() {
            let j = k as isize + shift;
            if j < 0 || j as usize >= t.len() || reference_speed.is_nan() || t[j as usize].is_nan()
            {
                continue;
            }
            a.push(reference_speed);
            b.push(t[j as usize]);
        }
        let c = correlation(&a, &b);
        if c > best.1 {
            best = (shift, c);
        }
    }
    best
}

/// (reference at k, Touch at k + shift) for every k where both exist.
pub fn shifted_pairs(
    samples: &[Sample],
    hand: usize,
    shift: isize,
) -> Vec<(Isometry3<f64>, Isometry3<f64>)> {
    (0..samples.len())
        .filter_map(|k| {
            let j = usize::try_from(k as isize + shift).ok()?;
            Some((samples[k].reference, samples.get(j)?.touch[hand]?))
        })
        .collect()
}

pub const SESSION_FORMAT: &str = "cv1touch-register-session";

#[derive(Serialize, Deserialize)]
struct SessionSample {
    t_ns: u64,
    reference: PoseEntry,
    touch: [Option<PoseEntry>; 2],
}

#[derive(Serialize, Deserialize)]
struct SessionFile {
    format: String,
    version: u32,
    reference: String,
    universe_id: u64,
    samples: Vec<SessionSample>,
}

/// Every sample of a registration session, so it can be solved again offline.
pub fn save_session(
    path: &Path,
    reference: &str,
    universe_id: u64,
    samples: &[Sample],
) -> Result<(), String> {
    let f = SessionFile {
        format: SESSION_FORMAT.into(),
        version: 1,
        reference: reference.into(),
        universe_id,
        samples: samples
            .iter()
            .map(|s| SessionSample {
                t_ns: s.t_ns,
                reference: PoseEntry::from_iso(&s.reference),
                touch: s.touch.map(|t| t.map(|t| PoseEntry::from_iso(&t))),
            })
            .collect(),
    };
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d).map_err(|e| format!("{}: {e}", d.display()))?;
    }
    std::fs::write(path, serde_json::to_string(&f).map_err(|e| e.to_string())?)
        .map_err(|e| format!("{}: {e}", path.display()))
}

/// (reference serial, universe id, samples)
pub fn load_session(path: &Path) -> Result<(String, u64, Vec<Sample>), String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let f: SessionFile =
        serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    if f.format != SESSION_FORMAT || f.version != 1 {
        return Err(format!(
            "{}: not a {SESSION_FORMAT} v1 file",
            path.display()
        ));
    }
    let samples = f
        .samples
        .iter()
        .map(|s| Sample {
            t_ns: s.t_ns,
            reference: s.reference.iso(),
            touch: s.touch.map(|t| t.map(|t| t.iso())),
        })
        .collect();
    Ok((f.reference, f.universe_id, samples))
}

/// R_Y from relative motions (axis(L_i^-1 L_j) = R_Y axis(C_i^-1 C_j)), and the axis ratio.
fn hand_eye(pairs: &[(Isometry3<f64>, Isometry3<f64>)], o: &Options) -> (UnitQuaternion<f64>, f64) {
    let mut h = Matrix3::zeros();
    let mut i = 0;
    while i + 1 < pairs.len() {
        let next = (i + 1..pairs.len().min(i + 200))
            .find(|&j| pairs[i].1.rotation.angle_to(&pairs[j].1.rotation) >= o.min_relative_rad);
        let Some(j) = next else {
            i += 1;
            continue;
        };
        let a = pairs[i].0.rotation.inverse() * pairs[j].0.rotation;
        let b = pairs[i].1.rotation.inverse() * pairs[j].1.rotation;
        if let (Some(aa), Some(ba)) = (a.axis(), b.axis()) {
            h += ba.into_inner() * aa.into_inner().transpose() * b.angle();
        }
        i = j;
    }
    let svd = h.svd(true, true);
    let (u, vt) = (svd.u.unwrap(), svd.v_t.unwrap());
    let mut d = Matrix3::identity();
    if (vt.transpose() * u.transpose()).determinant() < 0.0 {
        d[(2, 2)] = -1.0;
    }
    let r = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
        vt.transpose() * d * u.transpose(),
    ));
    let mut sv = [
        svd.singular_values[0],
        svd.singular_values[1],
        svd.singular_values[2],
    ];
    sv.sort_by(|a, b| b.total_cmp(a));
    (r, if sv[0] > 0.0 { sv[1] / sv[0] } else { 0.0 })
}

/// t_Y, t_X from L.R t_Y + L.t = R_X C.t + t_X (least squares).
fn translations(
    pairs: &[(Isometry3<f64>, Isometry3<f64>)],
    rx: &UnitQuaternion<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let mut n = Matrix6::zeros();
    let mut g = Vector6::zeros();
    for (l, c) in pairs {
        let mut a = SMatrix::<f64, 3, 6>::zeros();
        a.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(l.rotation.to_rotation_matrix().matrix());
        a.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(-Matrix3::identity()));
        let rhs = rx * c.translation.vector - l.translation.vector;
        n += a.transpose() * a;
        g += a.transpose() * rhs;
    }
    let z = n
        .cholesky()
        .map(|ch| ch.solve(&g))
        .unwrap_or_else(Vector6::zeros);
    (
        Vector3::new(z[0], z[1], z[2]),
        Vector3::new(z[3], z[4], z[5]),
    )
}

fn residual(
    x: &Isometry3<f64>,
    y: &Isometry3<f64>,
    l: &Isometry3<f64>,
    c: &Isometry3<f64>,
    rot_w: f64,
) -> Vector6<f64> {
    let (a, b) = (l * y, x * c);
    let dp = a.translation.vector - b.translation.vector;
    let dr = (b.rotation.inverse() * a.rotation).scaled_axis() * rot_w;
    Vector6::new(dp.x, dp.y, dp.z, dr.x, dr.y, dr.z)
}

fn retract(t: &Isometry3<f64>, d: &[f64]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(t.translation.vector + Vector3::new(d[0], d[1], d[2])),
        t.rotation * UnitQuaternion::from_scaled_axis(Vector3::new(d[3], d[4], d[5])),
    )
}

fn huber_cost(n: f64, d: f64) -> f64 {
    if n <= d {
        0.5 * n * n
    } else {
        d * (n - 0.5 * d)
    }
}

fn huber_weight(n: f64, d: f64) -> f64 {
    if n <= d {
        1.0
    } else {
        d / n
    }
}

fn refine(
    pairs: &[(Isometry3<f64>, Isometry3<f64>)],
    mut x: Isometry3<f64>,
    mut y: Isometry3<f64>,
    o: &Options,
) -> (Isometry3<f64>, Isometry3<f64>) {
    let cost = |x: &Isometry3<f64>, y: &Isometry3<f64>| {
        pairs
            .iter()
            .map(|(l, c)| huber_cost(residual(x, y, l, c, o.rot_weight_m).norm(), o.huber_m))
            .sum::<f64>()
    };
    let mut current = cost(&x, &y);
    let mut lambda = 1e-3;
    for _ in 0..o.max_iterations {
        let mut h = SMatrix::<f64, 12, 12>::zeros();
        let mut g = SVector::<f64, 12>::zeros();
        for (l, c) in pairs {
            let r = residual(&x, &y, l, c, o.rot_weight_m);
            let w = huber_weight(r.norm(), o.huber_m);
            let mut j = SMatrix::<f64, 6, 12>::zeros();
            for k in 0..12 {
                let mut d = [0.0; 12];
                d[k] = 1e-7;
                let rp = residual(
                    &retract(&x, &d[..6]),
                    &retract(&y, &d[6..]),
                    l,
                    c,
                    o.rot_weight_m,
                );
                j.set_column(k, &((rp - r) / 1e-7));
            }
            h += j.transpose() * j * w;
            g -= j.transpose() * r * w;
        }
        let mut improved = false;
        while lambda < 1e8 {
            let mut a = h;
            for k in 0..12 {
                a[(k, k)] += lambda * a[(k, k)].max(1e-12);
            }
            if let Some(step) = a.cholesky().map(|ch| ch.solve(&g)) {
                let (xn, yn) = (
                    retract(&x, &step.as_slice()[..6]),
                    retract(&y, &step.as_slice()[6..]),
                );
                let cn = cost(&xn, &yn);
                if cn < current {
                    let gain = (current - cn) / current.max(f64::MIN_POSITIVE);
                    (x, y, current) = (xn, yn, cn);
                    lambda = (lambda / 3.0).max(1e-12);
                    improved = gain > 1e-12;
                    break;
                }
            }
            lambda *= 4.0;
        }
        if !improved {
            break;
        }
    }
    (x, y)
}

pub fn solve(pairs: &[(Isometry3<f64>, Isometry3<f64>)], o: &Options) -> Result<Solution, String> {
    if pairs.len() < o.min_pairs {
        return Err(format!(
            "only {} paired samples (need {}): keep the Touch and the reference controller tracked (sensors and base stations) while moving",
            pairs.len(),
            o.min_pairs
        ));
    }
    let (ry, axis_ratio) = hand_eye(pairs, o);
    if axis_ratio < o.min_axis_ratio {
        return Err(format!(
            "turn the pair more: the rotations so far are almost all about one axis (ratio {axis_ratio:.2} < {:.2}); roll, pitch and yaw your wrist",
            o.min_axis_ratio
        ));
    }
    let rx = chordal_mean(
        &pairs
            .iter()
            .map(|(l, c)| l.rotation * ry * c.rotation.inverse())
            .collect::<Vec<_>>(),
    );
    let (ty, tx) = translations(pairs, &rx);
    let (x, y) = refine(
        pairs,
        Isometry3::from_parts(Translation3::from(tx), rx),
        Isometry3::from_parts(Translation3::from(ty), ry),
        o,
    );
    let res: Vec<Vector6<f64>> = pairs
        .iter()
        .map(|(l, c)| residual(&x, &y, l, c, o.rot_weight_m))
        .collect();
    let inl: Vec<&Vector6<f64>> = res
        .iter()
        .filter(|r| r.fixed_rows::<3>(0).norm() <= 3.0 * o.huber_m)
        .collect();
    let n = inl.len().max(1) as f64;
    Ok(Solution {
        lighthouse_from_world: x,
        reference_from_touch: y,
        pairs: pairs.len(),
        inliers: inl.len(),
        pos_rms_m: (inl
            .iter()
            .map(|r| r.fixed_rows::<3>(0).norm_squared())
            .sum::<f64>()
            / n)
            .sqrt(),
        rot_rms_deg: (inl
            .iter()
            .map(|r| (r.fixed_rows::<3>(3).norm() / o.rot_weight_m).powi(2))
            .sum::<f64>()
            / n)
            .sqrt()
            .to_degrees(),
        axis_ratio,
    })
}

/// With `lighthouse_from_world` fixed, each pair implies the held offset L_i^-1 X C_i; returns
/// their mean and spread (m, deg). A correct registration gives the hold's own spread.
pub fn check(
    pairs: &[(Isometry3<f64>, Isometry3<f64>)],
    x: &Isometry3<f64>,
) -> (Isometry3<f64>, f64, f64) {
    let ys: Vec<Isometry3<f64>> = pairs.iter().map(|(l, c)| l.inverse() * x * c).collect();
    let r = chordal_mean(&ys.iter().map(|y| y.rotation).collect::<Vec<_>>());
    let t = ys
        .iter()
        .map(|y| y.translation.vector)
        .sum::<Vector3<f64>>()
        / ys.len().max(1) as f64;
    let n = ys.len().max(1) as f64;
    let pos = (ys
        .iter()
        .map(|y| (y.translation.vector - t).norm_squared())
        .sum::<f64>()
        / n)
        .sqrt();
    let rot = (ys
        .iter()
        .map(|y| y.rotation.angle_to(&r).powi(2))
        .sum::<f64>()
        / n)
        .sqrt()
        .to_degrees();
    (Isometry3::from_parts(Translation3::from(t), r), pos, rot)
}

/// How `rigid_span` tells a Touch clamped to the reference from one merely near it.
#[derive(Debug, Clone, Copy)]
pub struct Rigid {
    /// Each pair is judged against its neighbours this far either side.
    pub half_window_ns: u64,
    /// A neighbour's held offset within this of the pair's counts as unchanged.
    pub tol_m: f64,
    pub tol_rad: f64,
    /// A stretch has to turn the reference through at least this much in all: a Touch and a
    /// headset resting on a desk are rigid too, and say nothing.
    pub min_turn_rad: f64,
}

impl Default for Rigid {
    fn default() -> Self {
        // loose enough for a registration a few cm off (its error turns with the pair, by up to
        // 2 cm in half a second of a brisk turn), tight against a hand leaving the headset
        Rigid {
            half_window_ns: 500_000_000,
            tol_m: 0.04,
            tol_rad: 6f64.to_radians(),
            min_turn_rad: 1.0,
        }
    }
}

/// The stretch of a capture in which the Touch moved rigidly with the reference, from pairs
/// (time, reference pose, Touch pose) in time order: the held offset L_i^-1 X0 C_i under a rough
/// registration `x0` stays put from moment to moment there (X0's own error only drifts it slowly
/// as the pair turns). Of the stretches found, the one turning the reference most; None when none
/// turns `min_turn_rad`. A pair is steady when three quarters of its neighbours share its offset,
/// so a stray pose does not break a stretch; stretches either side of a dropout join when their
/// offsets agree.
pub fn rigid_span(
    pairs: &[(u64, Isometry3<f64>, Isometry3<f64>)],
    x0: &Isometry3<f64>,
    o: &Rigid,
) -> Option<std::ops::Range<usize>> {
    let ys: Vec<Isometry3<f64>> = pairs.iter().map(|(_, l, c)| l.inverse() * x0 * c).collect();
    let close = |a: &Isometry3<f64>, b: &Isometry3<f64>, k: f64| {
        (a.translation.vector - b.translation.vector).norm() < o.tol_m * k
            && a.rotation.angle_to(&b.rotation) < o.tol_rad * k
    };
    let steady: Vec<bool> = (0..pairs.len())
        .map(|i| {
            let t = pairs[i].0;
            let lo = pairs.partition_point(|p| p.0 + o.half_window_ns < t);
            let hi = pairs.partition_point(|p| p.0 <= t + o.half_window_ns);
            (lo..hi).filter(|&j| close(&ys[i], &ys[j], 1.0)).count() * 4 >= (hi - lo) * 3
        })
        .collect();
    let mut runs: Vec<std::ops::Range<usize>> = Vec::new();
    for (i, _) in steady.iter().enumerate().filter(|(_, s)| **s) {
        match runs.last_mut() {
            Some(r) if r.end == i => r.end += 1,
            Some(r) if close(&ys[r.end - 1], &ys[i], 1.0) => r.end = i + 1,
            _ => runs.push(i..i + 1),
        }
    }
    let turn = |r: &std::ops::Range<usize>| {
        (r.start + 1..r.end)
            .map(|k| pairs[k - 1].1.rotation.angle_to(&pairs[k].1.rotation))
            .sum::<f64>()
    };
    runs.into_iter()
        .map(|r| (turn(&r), r))
        .filter(|(t, _)| *t >= o.min_turn_rad)
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, r)| r)
}

/// Find an attached, turning span without a previously registered coordinate frame.
/// Short hand-eye fits supply candidate frames; a full-span rigidity check rejects unrelated
/// motion and desk holds. A fresh capture may include picking up and putting down the Touches.
pub fn bootstrap_span(
    pairs: &[(u64, Isometry3<f64>, Isometry3<f64>)],
) -> Option<std::ops::Range<usize>> {
    if pairs.len() < 500 {
        return None;
    }
    let mut best = None;
    let mut best_turn = 0.0;
    let mut start = 0;
    while start + 500 <= pairs.len() {
        let end = pairs.partition_point(|p| p.0 <= pairs[start].0 + 25_000_000_000);
        let step = ((end - start) / 200).max(1);
        let seed: Vec<_> = pairs[start..end]
            .iter()
            .step_by(step)
            .map(|p| (p.1, p.2))
            .collect();
        let opts = Options {
            min_pairs: 100,
            min_axis_ratio: 0.1,
            max_iterations: 20,
            ..Options::default()
        };
        if let Ok(sol) = solve(&seed, &opts) {
            if sol.inliers * 5 >= sol.pairs * 4 && sol.pos_rms_m <= 0.010 && sol.rot_rms_deg <= 3.0
            {
                if let Some(r) = rigid_span(pairs, &sol.lighthouse_from_world, &Rigid::default()) {
                    let turn: f64 = pairs[r.clone()]
                        .windows(2)
                        .map(|w| w[0].1.rotation.angle_to(&w[1].1.rotation))
                        .sum();
                    if r.len() >= 500 && turn > best_turn {
                        best_turn = turn;
                        best = Some(r);
                    }
                }
            }
        }
        start = pairs
            .partition_point(|p| p.0 < pairs[start].0 + 5_000_000_000)
            .max(start + 1);
    }
    best
}

/// The stir: how far a Touch held fixed to the headset seems to circle about its place as the
/// wearer turns. From (heading, the Touch's position in the headset frame) samples, the first
/// harmonic in heading, p = a + b cos(heading) + c sin(heading) per axis by least squares; its
/// radius, sqrt((|b|^2 + |c|^2) / 2) (m). A world-fixed error e shows as a circle of radius |e|
/// across the horizontal. NaN when the headings do not go round.
pub fn stir_radius(samples: &[(f64, Vector3<f64>)]) -> f64 {
    let mut m = Matrix3::<f64>::zeros();
    let mut r = Matrix3::<f64>::zeros();
    for (heading, p) in samples {
        let f = Vector3::new(1.0, heading.cos(), heading.sin());
        m += f * f.transpose();
        r += f * p.transpose();
    }
    if m.symmetric_eigenvalues().min() < 0.05 * samples.len() as f64 {
        return f64::NAN;
    }
    let Some(k) = m.try_inverse() else {
        return f64::NAN;
    };
    let coef = k * r;
    ((coef.row(1).norm_squared() + coef.row(2).norm_squared()) / 2.0).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::Rng;

    /// A smooth hand-held path for `secs` at 100 Hz: rotation rates from sums of sines about the
    /// given axes, positions wandering through a 0.6 m box.
    fn path(secs: f64, rates: fn(f64) -> Vector3<f64>) -> Vec<Isometry3<f64>> {
        let dt = 0.01;
        let mut q = UnitQuaternion::from_scaled_axis(Vector3::new(0.2, 1.0, -0.3));
        (0..(secs / dt) as usize)
            .map(|k| {
                let t = k as f64 * dt;
                q *= UnitQuaternion::from_scaled_axis(rates(t) * dt);
                let p = Vector3::new(
                    0.3 * (0.4 * t).sin(),
                    1.2 + 0.2 * (0.6 * t).sin(),
                    0.3 * (0.5 * t).cos(),
                );
                Isometry3::from_parts(Translation3::from(p), q)
            })
            .collect()
    }

    fn all_axes(t: f64) -> Vector3<f64> {
        Vector3::new(
            1.5 * (0.7 * t).sin(),
            1.2 * (1.1 * t + 1.0).sin(),
            1.0 * (0.5 * t + 2.0).sin(),
        )
    }

    fn one_axis(t: f64) -> Vector3<f64> {
        Vector3::new(0.0, 0.0, 1.5 * (0.7 * t).sin())
    }

    fn x_true() -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(0.8, -0.1, 1.4),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.05, 2.3, -0.02)),
        )
    }

    fn y_true() -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(0.02, -0.06, 0.03),
            UnitQuaternion::from_scaled_axis(Vector3::new(1.2, 0.3, -0.4)),
        )
    }

    /// Reference poses L_i = X C_i Y_i^-1, with noise, a slipping hold and outliers.
    fn pairs(
        touch: &[Isometry3<f64>],
        noise_m: f64,
        noise_rad: f64,
        slip: bool,
        outliers: f64,
    ) -> Vec<(Isometry3<f64>, Isometry3<f64>)> {
        let mut rng = Rng::new(11);
        touch
            .iter()
            .enumerate()
            .map(|(k, c)| {
                let t = k as f64 * 0.01;
                let y = if slip {
                    y_true()
                        * Isometry3::from_parts(
                            Translation3::from(Vector3::new(
                                0.003 * (0.2 * t).sin(),
                                0.002 * (0.3 * t).cos(),
                                0.0,
                            )),
                            UnitQuaternion::from_scaled_axis(Vector3::new(
                                0.0,
                                0.017 * (0.25 * t).sin(),
                                0.0,
                            )),
                        )
                } else {
                    y_true()
                };
                let mut l = x_true() * c * y.inverse();
                l = Isometry3::from_parts(
                    Translation3::from(l.translation.vector + rng.gauss3(noise_m)),
                    l.rotation * UnitQuaternion::from_scaled_axis(rng.gauss3(noise_rad)),
                );
                if rng.uniform() < outliers {
                    l.translation.vector += Vector3::new(0.1, 0.0, -0.05);
                }
                (l, *c)
            })
            .collect()
    }

    fn assert_close(a: &Isometry3<f64>, b: &Isometry3<f64>, m: f64, deg: f64) {
        let dp = (a.translation.vector - b.translation.vector).norm();
        let da = a.rotation.angle_to(&b.rotation).to_degrees();
        assert!(dp < m && da < deg, "{:.5} mm {da:.5} deg", dp * 1e3);
    }

    #[test]
    fn recovers_an_exact_registration() {
        let s = solve(
            &pairs(&path(30.0, all_axes), 0.0, 0.0, false, 0.0),
            &Options::default(),
        )
        .unwrap();
        assert_close(&s.lighthouse_from_world, &x_true(), 1e-5, 1e-3);
        assert_close(&s.reference_from_touch, &y_true(), 1e-5, 1e-3);
        assert!(
            s.pos_rms_m < 1e-6 && s.axis_ratio > 0.25,
            "{} {}",
            s.pos_rms_m,
            s.axis_ratio
        );
    }

    #[test]
    fn tolerates_noise_a_slipping_hold_and_outliers() {
        let s = solve(
            &pairs(&path(60.0, all_axes), 0.001, 0.005, true, 0.03),
            &Options::default(),
        )
        .unwrap();
        assert_close(&s.lighthouse_from_world, &x_true(), 0.003, 0.3);
        assert!(
            s.inliers * 100 > s.pairs * 95,
            "{} of {}",
            s.inliers,
            s.pairs
        );
        assert!(s.pos_rms_m < 0.006, "{}", s.pos_rms_m);
    }

    #[test]
    fn rotation_about_one_axis_is_refused() {
        let err = solve(
            &pairs(&path(30.0, one_axis), 0.0, 0.0, false, 0.0),
            &Options::default(),
        )
        .unwrap_err();
        assert!(err.contains("turn the pair"), "{err}");
    }

    #[test]
    fn too_few_pairs_are_refused() {
        let err = solve(
            &pairs(&path(3.0, all_axes), 0.0, 0.0, false, 0.0),
            &Options::default(),
        )
        .unwrap_err();
        assert!(err.contains("paired samples"), "{err}");
    }

    #[test]
    fn picks_the_touch_that_moves_with_the_reference() {
        let held = path(20.0, all_axes);
        let other = path(20.0, |t| {
            Vector3::new(0.8 * (1.3 * t).cos(), 0.0, 0.5 * (0.9 * t).sin())
        });
        let ps = pairs(&held, 0.0, 0.0, false, 0.0);
        let samples: Vec<Sample> = ps
            .iter()
            .zip(&other)
            .enumerate()
            .map(|(k, ((l, c), o))| Sample {
                t_ns: k as u64 * 10_000_000,
                reference: *l,
                touch: [Some(*o), Some(*c)],
            })
            .collect();
        let (hand, corr) = pick_touch(&samples).unwrap();
        assert_eq!(hand, 1);
        assert!(corr > 0.9, "{corr}");
        let alone: Vec<Sample> = samples
            .iter()
            .map(|s| Sample {
                touch: [s.touch[0], None],
                ..*s
            })
            .collect();
        assert!(pick_touch(&alone).is_none());
    }

    /// The Touch stream runs 3 samples (30 ms) behind the reference: the lag is found, and pairing
    /// with it recovers the exact registration.
    #[test]
    fn finds_and_removes_a_stream_lag() {
        let held = path(30.0, all_axes);
        let ps = pairs(&held, 0.0, 0.0, false, 0.0);
        // touch sample k shows the pose of k - 3
        let samples: Vec<Sample> = (0..ps.len())
            .map(|k| Sample {
                t_ns: k as u64 * 10_000_000,
                reference: ps[k].0,
                touch: [None, (k >= 3).then(|| ps[k - 3].1)],
            })
            .collect();
        let (shift, corr) = estimate_lag(&samples, 1, 20);
        assert_eq!(shift, 3);
        assert!(corr > 0.99, "{corr}");
        let s = solve(&shifted_pairs(&samples, 1, shift), &Options::default()).unwrap();
        assert_close(&s.lighthouse_from_world, &x_true(), 1e-5, 1e-3);
        assert!(solve(&shifted_pairs(&samples, 1, 0), &Options::default())
            .map_or(true, |s| s.pos_rms_m > 1e-3));
    }

    #[test]
    fn sessions_round_trip() {
        let ps = pairs(&path(2.0, all_axes), 0.0, 0.0, false, 0.0);
        let samples: Vec<Sample> = ps
            .iter()
            .enumerate()
            .map(|(k, (l, c))| Sample {
                t_ns: k as u64,
                reference: *l,
                touch: [Some(*c), None],
            })
            .collect();
        let file = std::env::temp_dir().join(format!("cv1t-session-{}.json", std::process::id()));
        save_session(&file, "LHR-TEST-REFERENCE", 1772213422, &samples).unwrap();
        let (serial, universe, back) = load_session(&file).unwrap();
        assert_eq!(
            (serial.as_str(), universe, back.len()),
            ("LHR-TEST-REFERENCE", 1772213422, samples.len())
        );
        assert!(back[5].touch[1].is_none() && back[5].t_ns == 5);
        assert_close(&back[5].reference, &samples[5].reference, 1e-5, 1e-4);
        assert_close(
            &back[5].touch[0].unwrap(),
            &samples[5].touch[0].unwrap(),
            1e-5,
            1e-4,
        );
        std::fs::remove_file(&file).ok();
    }

    /// 90 s at 100 Hz: the Touch and the reference resting on a desk, picked up, clamped
    /// together and turned through every axis, let go, then still again apart.
    #[test]
    fn finds_the_stretch_in_which_the_touch_moved_with_the_reference() {
        let mut rng = Rng::new(12);
        let moving = path(90.0, all_axes);
        let desk = Isometry3::from_parts(
            Translation3::new(0.5, 0.8, -0.3),
            UnitQuaternion::identity(),
        );
        let wander = |t: f64| {
            Isometry3::from_parts(
                Translation3::new(
                    0.2 * (1.3 * t).sin(),
                    1.0 + 0.15 * (0.9 * t).cos(),
                    0.1 * (2.1 * t).sin(),
                ),
                UnitQuaternion::from_scaled_axis(Vector3::new(0.8 * (0.7 * t).sin(), 0.5 * t, 0.0)),
            )
        };
        let pairs: Vec<(u64, Isometry3<f64>, Isometry3<f64>)> = moving
            .iter()
            .enumerate()
            .map(|(k, c_moving)| {
                let t = k as f64 * 0.01;
                let (l, c) = match t {
                    t if t < 10.0 => (desk, desk * Isometry3::translation(0.3, 0.0, 0.0)),
                    t if t < 15.0 => (x_true() * c_moving * y_true().inverse(), wander(t)),
                    t if t < 75.0 => (x_true() * c_moving * y_true().inverse(), *c_moving),
                    t if t < 80.0 => (x_true() * c_moving * y_true().inverse(), wander(t)),
                    _ => (desk, desk * Isometry3::translation(0.0, 0.0, 0.4)),
                };
                let l = Isometry3::from_parts(
                    Translation3::from(l.translation.vector + rng.gauss3(0.001)),
                    l.rotation,
                );
                ((k as u64) * 10_000_000, l, c)
            })
            .collect();
        // the installed registration is off by 2 cm and half a degree
        let x0 = x_true()
            * Isometry3::from_parts(
                Translation3::new(0.012, -0.01, 0.012),
                UnitQuaternion::from_scaled_axis(Vector3::new(0.006, -0.005, 0.003)),
            );
        let r = rigid_span(&pairs, &x0, &Rigid::default()).unwrap();
        let (a, b) = (
            pairs[r.start].0 as f64 * 1e-9,
            pairs[r.end - 1].0 as f64 * 1e-9,
        );
        assert!(
            (a - 15.0).abs() < 0.6 && (b - 75.0).abs() < 0.6,
            "found {a:.2}..{b:.2} s"
        );
    }

    #[test]
    fn fresh_headset_registration_has_no_prior_frame() {
        let touch = path(60.0, all_axes);
        let ps = pairs(&touch, 0.0002, 0.0005, false, 0.0);
        // A completely different initial room frame; no identity or saved registration is usable.
        let timed: Vec<_> = ps
            .iter()
            .enumerate()
            .map(|(k, (l, c))| {
                let c = if !(1000..=5000).contains(&k) {
                    Isometry3::translation(0.4, 0.8, 0.2)
                } else {
                    *c
                };
                (k as u64 * 10_000_000, *l, c)
            })
            .collect();
        let r = bootstrap_span(&timed).expect("finds the on-headset span without a registration");
        assert!(r.start >= 900 && r.end <= 5100, "{r:?}");
        let ps: Vec<_> = timed[r].iter().map(|p| (p.1, p.2)).collect();
        let sol = solve(
            &ps,
            &Options {
                min_pairs: 500,
                min_axis_ratio: 0.1,
                ..Options::default()
            },
        )
        .unwrap();
        assert_close(&sol.lighthouse_from_world, &x_true(), 0.002, 0.2);
    }

    #[test]
    fn fresh_headset_registration_rejects_unrelated_motion() {
        let ref_path = path(30.0, all_axes);
        let other = path(30.0, |t| {
            Vector3::new(0.2 * t.cos(), 0.7 * (2.0 * t).sin(), 0.3)
        });
        let timed: Vec<_> = ref_path
            .iter()
            .zip(other)
            .enumerate()
            .map(|(k, (l, c))| (k as u64 * 10_000_000, *l, c))
            .collect();
        assert!(bootstrap_span(&timed).is_none());
    }

    #[test]
    fn a_capture_that_never_turns_has_no_rigid_stretch() {
        let still = Isometry3::translation(0.0, 1.0, 0.0);
        let pairs: Vec<_> = (0..3000u64)
            .map(|k| (k * 10_000_000, still, still))
            .collect();
        assert!(rigid_span(&pairs, &Isometry3::identity(), &Rigid::default()).is_none());
    }

    #[test]
    fn a_world_fixed_error_stirs_by_its_size() {
        // a Touch 12 cm in front of the eyes, drawn 1 cm east of where it is, wearer turning
        let e = Vector3::new(0.01, 0.0, 0.0);
        let held = Vector3::new(0.0, -0.05, -0.12);
        let samples: Vec<(f64, Vector3<f64>)> = (0..720)
            .map(|k| {
                let heading = (k as f64).to_radians();
                let head = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), heading);
                (heading, held + head.inverse() * e)
            })
            .collect();
        assert!(
            (stir_radius(&samples) - 0.01).abs() < 1e-6,
            "{}",
            stir_radius(&samples)
        );
        let still: Vec<(f64, Vector3<f64>)> = samples.iter().map(|(h, _)| (*h, held)).collect();
        assert!(stir_radius(&still) < 1e-9);
        assert!(
            stir_radius(&samples[..20]).is_nan(),
            "20 degrees of turn says nothing"
        );
    }

    #[test]
    fn check_measures_the_spread_of_the_hold() {
        let ps = pairs(&path(20.0, all_axes), 0.0, 0.0, false, 0.0);
        let (y, pos, rot) = check(&ps, &x_true());
        assert_close(&y, &y_true(), 1e-9, 1e-6);
        assert!(pos < 1e-9 && rot < 1e-6);
        let off = Isometry3::from_parts(
            Translation3::new(0.03, 0.0, 0.0),
            UnitQuaternion::identity(),
        ) * x_true();
        assert!(check(&ps, &off).1 > 0.01);
    }

    #[test]
    fn extrapolates_a_published_state() {
        let st = ControllerState {
            sample_host_ns: 1_000_000_000,
            orient: [0.0, 0.0, 0.0, 1.0],
            pos: [1.0, 2.0, 3.0],
            lin_vel: [1.0, 0.0, 0.0],
            ang_vel: [0.0, 0.0, 2.0],
            gyro_bias: [0.0, 0.0, 1.0],
            ..Default::default()
        };
        let t = extrapolate(&st, 1_010_000_000);
        assert!((t.translation.vector - Vector3::new(1.01, 2.0, 3.0)).norm() < 1e-6);
        assert!((t.rotation.scaled_axis() - Vector3::new(0.0, 0.0, 0.01)).norm() < 1e-6);
        assert_eq!(
            extrapolate(&st, 900_000_000).translation.vector,
            Vector3::new(1.0, 2.0, 3.0)
        ); // never backwards
    }
}
