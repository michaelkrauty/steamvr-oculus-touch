// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Bundle adjustment over sensor poses and per-sample controller poses, all in the reference
//! sensor's frame (fixed at identity: no gauge freedom; metric scale from the factory LED model).
//! Residuals: LED reprojection (Huber). Gravity is not part of it: the solve takes it from the
//! controllers' motion afterwards (see `motion`). Levenberg-Marquardt with numeric Jacobians;
//! per-sample blocks eliminated by Schur complement.

use std::collections::BTreeMap;

use nalgebra::{
    DMatrix, DVector, Isometry3, Matrix6, Point3, Translation3, UnitQuaternion, Vector2, Vector3,
    Vector6,
};

use super::camera::CameraModel;
use super::capture::{Led, WandSample};

/// Central-difference step for the numeric Jacobians (metres, radians).
const H: f64 = 1e-6;
/// Residual charged for an observation that no longer projects (pixels).
const LOST_PX: f64 = 100.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Options {
    pub sigma_px: f64,
    pub huber_px: f64,
    pub max_iterations: usize,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            sigma_px: 0.5,
            huber_px: 2.0,
            max_iterations: 50,
        }
    }
}

pub struct Problem<'a> {
    pub cameras: &'a [CameraModel],
    pub leds: &'a BTreeMap<i32, Vec<Led>>,
    pub samples: &'a [WandSample],
    pub reference: usize,
}

#[derive(Debug, Clone)]
pub struct State {
    pub ref_from_cam: Vec<Isometry3<f64>>,
    pub ref_from_obj: Vec<Option<Isometry3<f64>>>,
}

#[derive(Debug, Clone, Default)]
pub struct Report {
    pub iterations: usize,
    pub initial_cost: f64,
    pub final_cost: f64,
    pub camera_rms_px: Vec<f64>,
    pub camera_obs: Vec<usize>,
}

/// Global parameter layout: 6 per non-reference sensor.
struct Layout {
    cam: Vec<Option<usize>>,
    globals: usize,
}

impl Layout {
    fn new(p: &Problem) -> Layout {
        let mut cam = vec![None; p.cameras.len()];
        let mut next = 0;
        for (c, slot) in cam.iter_mut().enumerate() {
            if c != p.reference {
                *slot = Some(next);
                next += 6;
            }
        }
        Layout { cam, globals: next }
    }
}

/// Pose update: translation added in the outer frame, rotation applied on the right.
fn retract(t: &Isometry3<f64>, d: &[f64]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(t.translation.vector + Vector3::new(d[0], d[1], d[2])),
        t.rotation * UnitQuaternion::from_scaled_axis(Vector3::new(d[3], d[4], d[5])),
    )
}

fn unit<const N: usize>(a: usize, h: f64) -> [f64; N] {
    let mut d = [0.0; N];
    d[a] = h;
    d
}

fn reprojection(
    cam: &CameraModel,
    ref_from_cam: &Isometry3<f64>,
    ref_from_obj: &Isometry3<f64>,
    led: &Vector3<f64>,
    xy: &Vector2<f64>,
) -> Option<Vector2<f64>> {
    let p_ref = ref_from_obj * Point3::from(*led);
    cam.project(&ref_from_cam.inverse_transform_point(&p_ref).coords)
        .map(|uv| uv - xy)
}

fn huber_weight(n: f64, delta: f64) -> f64 {
    if n <= delta {
        1.0
    } else {
        delta / n
    }
}

fn huber_cost(n: f64, delta: f64) -> f64 {
    if n <= delta {
        0.5 * n * n
    } else {
        delta * (n - 0.5 * delta)
    }
}

/// Central-difference Jacobian columns of `f(param, step)` over `n` parameters.
fn numeric<const R: usize>(
    n: usize,
    mut f: impl FnMut(usize, f64) -> Option<[f64; R]>,
) -> Option<Vec<[f64; R]>> {
    (0..n)
        .map(|a| {
            let (plus, minus) = (f(a, H)?, f(a, -H)?);
            Some(std::array::from_fn(|row| {
                (plus[row] - minus[row]) / (2.0 * H)
            }))
        })
        .collect()
}

fn dot<const R: usize>(a: &[f64; R], b: &[f64; R]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Normal equations with the per-sample blocks kept separate for the Schur complement.
struct Normal {
    hww: Vec<Matrix6<f64>>,
    bw: Vec<Vector6<f64>>,
    hwg: Vec<DMatrix<f64>>,
    hgg: DMatrix<f64>,
    bg: DVector<f64>,
}

impl Normal {
    fn new(samples: usize, globals: usize) -> Normal {
        Normal {
            hww: vec![Matrix6::zeros(); samples],
            bw: vec![Vector6::zeros(); samples],
            hwg: vec![DMatrix::zeros(6, globals); samples],
            hgg: DMatrix::zeros(globals, globals),
            bg: DVector::zeros(globals),
        }
    }

    /// Adds residual block `r` of sample `i`: Jacobian columns `jw` (the sample's 6 pose
    /// parameters) and `jg` (the global parameters `cols`), weighted by `weight`.
    fn add<const R: usize>(
        &mut self,
        i: usize,
        r: &[f64; R],
        jw: &[[f64; R]],
        jg: &[[f64; R]],
        cols: &[usize],
        weight: f64,
    ) {
        for (a, ja) in jw.iter().enumerate() {
            self.bw[i][a] -= weight * dot(ja, r);
            for (b, jb) in jw.iter().enumerate() {
                self.hww[i][(a, b)] += weight * dot(ja, jb);
            }
            for (k, &g) in cols.iter().enumerate() {
                self.hwg[i][(a, g)] += weight * dot(ja, &jg[k]);
            }
        }
        for (k1, &g1) in cols.iter().enumerate() {
            self.bg[g1] -= weight * dot(&jg[k1], r);
            for (k2, &g2) in cols.iter().enumerate() {
                self.hgg[(g1, g2)] += weight * dot(&jg[k1], &jg[k2]);
            }
        }
    }
}

fn build(p: &Problem, s: &State, o: &Options, l: &Layout) -> Normal {
    let mut n = Normal::new(p.samples.len(), l.globals);
    let delta = o.huber_px / o.sigma_px;
    for (i, sample) in p.samples.iter().enumerate() {
        let (Some(w), Some(model)) = (s.ref_from_obj[i], p.leds.get(&sample.device_id)) else {
            continue;
        };
        for v in &sample.views {
            let (cam, t_cam) = (&p.cameras[v.camera], s.ref_from_cam[v.camera]);
            for ob in &v.obs {
                let Some(led) = model.get(ob.led) else {
                    continue;
                };
                let residual = |t_cam: &Isometry3<f64>, w: &Isometry3<f64>| {
                    reprojection(cam, t_cam, w, &led.pos, &ob.xy).map(|e| [e.x, e.y])
                };
                let Some(r) = residual(&t_cam, &w) else {
                    continue;
                };
                let Some(jw) = numeric(6, |a, h| residual(&t_cam, &retract(&w, &unit::<6>(a, h))))
                else {
                    continue;
                };
                let (jg, cols): (Vec<[f64; 2]>, Vec<usize>) = match l.cam[v.camera] {
                    Some(off) => {
                        match numeric(6, |a, h| residual(&retract(&t_cam, &unit::<6>(a, h)), &w)) {
                            Some(j) => (j, (off..off + 6).collect()),
                            None => continue,
                        }
                    }
                    None => (Vec::new(), Vec::new()),
                };
                let norm = (r[0] * r[0] + r[1] * r[1]).sqrt() / o.sigma_px;
                n.add(
                    i,
                    &r,
                    &jw,
                    &jg,
                    &cols,
                    huber_weight(norm, delta) / (o.sigma_px * o.sigma_px),
                );
            }
        }
    }
    n
}

type BundleStep = (Vec<Option<Vector6<f64>>>, DVector<f64>);

/// One damped step: eliminate the sample blocks, solve for the globals, back-substitute.
fn solve_step(n: &Normal, lambda: f64) -> Option<BundleStep> {
    let g = n.bg.len();
    let mut schur = n.hgg.clone();
    for k in 0..g {
        schur[(k, k)] += lambda * n.hgg[(k, k)].max(1e-9);
    }
    let mut rhs = n.bg.clone();
    let mut inverses = Vec::with_capacity(n.hww.len());
    for i in 0..n.hww.len() {
        let mut a = n.hww[i];
        if a.diagonal().iter().all(|&d| d == 0.0) {
            inverses.push(None);
            continue;
        }
        for k in 0..6 {
            a[(k, k)] += lambda * a[(k, k)].max(1e-9);
        }
        let Some(ai) = a.try_inverse() else {
            inverses.push(None);
            continue;
        };
        let ai_d = DMatrix::from_column_slice(6, 6, ai.as_slice());
        let hwg = &n.hwg[i];
        schur -= hwg.transpose() * (&ai_d * hwg);
        rhs -= hwg.transpose() * DVector::from_column_slice((ai * n.bw[i]).as_slice());
        inverses.push(Some(ai));
    }
    let dg = match schur.clone().cholesky() {
        Some(c) => c.solve(&rhs),
        None => schur.lu().solve(&rhs)?,
    };
    let dw = inverses
        .iter()
        .enumerate()
        .map(|(i, ai)| {
            ai.map(|ai| ai * (n.bw[i] - Vector6::from_column_slice((&n.hwg[i] * &dg).as_slice())))
        })
        .collect();
    Some((dw, dg))
}

fn apply(s: &State, l: &Layout, dw: &[Option<Vector6<f64>>], dg: &DVector<f64>) -> State {
    let mut out = s.clone();
    for (c, off) in l.cam.iter().enumerate() {
        if let Some(off) = *off {
            out.ref_from_cam[c] = retract(&s.ref_from_cam[c], &dg.as_slice()[off..off + 6]);
        }
    }
    for (i, d) in dw.iter().enumerate() {
        if let (Some(d), Some(w)) = (d, s.ref_from_obj[i]) {
            out.ref_from_obj[i] = Some(retract(&w, d.as_slice()));
        }
    }
    out
}

pub fn cost(p: &Problem, s: &State, o: &Options) -> f64 {
    let delta = o.huber_px / o.sigma_px;
    let mut total = 0.0;
    for (i, sample) in p.samples.iter().enumerate() {
        let (Some(w), Some(model)) = (s.ref_from_obj[i], p.leds.get(&sample.device_id)) else {
            continue;
        };
        for v in &sample.views {
            for ob in &v.obs {
                let Some(led) = model.get(ob.led) else {
                    continue;
                };
                let px = reprojection(
                    &p.cameras[v.camera],
                    &s.ref_from_cam[v.camera],
                    &w,
                    &led.pos,
                    &ob.xy,
                )
                .map_or(LOST_PX, |e| e.norm());
                total += huber_cost(px / o.sigma_px, delta);
            }
        }
    }
    total
}

/// Unweighted RMS reprojection error (px) of one sighting under the current state.
pub fn view_rms_px(p: &Problem, s: &State, sample: usize, view: usize) -> Option<f64> {
    let smp = p.samples.get(sample)?;
    let v = smp.views.get(view)?;
    let w = s.ref_from_obj.get(sample).copied().flatten()?;
    let model = p.leds.get(&smp.device_id)?;
    let (mut sum, mut n) = (0.0, 0usize);
    for ob in &v.obs {
        let led = model.get(ob.led)?;
        sum += reprojection(
            &p.cameras[v.camera],
            &s.ref_from_cam[v.camera],
            &w,
            &led.pos,
            &ob.xy,
        )
        .map_or(LOST_PX * LOST_PX, |e| e.norm_squared());
        n += 1;
    }
    (n > 0).then(|| (sum / n as f64).sqrt())
}

fn camera_stats(p: &Problem, s: &State) -> (Vec<f64>, Vec<usize>) {
    let n = p.cameras.len();
    let (mut sum, mut count) = (vec![0.0; n], vec![0usize; n]);
    for (i, sample) in p.samples.iter().enumerate() {
        let (Some(w), Some(model)) = (s.ref_from_obj[i], p.leds.get(&sample.device_id)) else {
            continue;
        };
        for v in &sample.views {
            for ob in &v.obs {
                let Some(led) = model.get(ob.led) else {
                    continue;
                };
                if let Some(e) = reprojection(
                    &p.cameras[v.camera],
                    &s.ref_from_cam[v.camera],
                    &w,
                    &led.pos,
                    &ob.xy,
                ) {
                    sum[v.camera] += e.norm_squared();
                    count[v.camera] += 1;
                }
            }
        }
    }
    (
        (0..n)
            .map(|c| {
                if count[c] > 0 {
                    (sum[c] / count[c] as f64).sqrt()
                } else {
                    f64::NAN
                }
            })
            .collect(),
        count,
    )
}

pub fn solve(p: &Problem, s: &mut State, o: &Options) -> Report {
    let l = Layout::new(p);
    let initial_cost = cost(p, s, o);
    let mut current = initial_cost;
    let mut lambda = 1e-4;
    let mut iterations = 0;
    'outer: while iterations < o.max_iterations {
        iterations += 1;
        let n = build(p, s, o, &l);
        loop {
            if let Some((dw, dg)) = solve_step(&n, lambda) {
                let candidate = apply(s, &l, &dw, &dg);
                let c = cost(p, &candidate, o);
                if c < current {
                    let gain = (current - c) / current.max(f64::MIN_POSITIVE);
                    *s = candidate;
                    current = c;
                    lambda = (lambda / 3.0).max(1e-12);
                    if gain < 1e-9 {
                        break 'outer;
                    }
                    break;
                }
            }
            lambda *= 4.0;
            if lambda > 1e12 {
                break 'outer;
            }
        }
    }
    let (camera_rms_px, camera_obs) = camera_stats(p, s);
    Report {
        iterations,
        initial_cost,
        final_cost: current,
        camera_rms_px,
        camera_obs,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::init;
    use crate::calib::synth::{self, Scene, SceneParams};

    fn setup(sc: &Scene) -> (usize, State) {
        let initial = init::initialize(&sc.samples, &sc.serials, 20).unwrap();
        let ref_from_obj = init::wand_poses(&sc.samples, &initial.ref_from_cam);
        (
            initial.reference,
            State {
                ref_from_cam: initial.ref_from_cam,
                ref_from_obj,
            },
        )
    }

    /// Truth in the reference frame: ref_from_cam per sensor.
    fn truth(sc: &Scene, reference: usize) -> Vec<Isometry3<f64>> {
        let world_from_ref = sc.world_from_cam[reference];
        sc.world_from_cam
            .iter()
            .map(|w| world_from_ref.inverse() * w)
            .collect()
    }

    fn run(sc: &Scene) -> (State, Report, Vec<Isometry3<f64>>) {
        let (reference, mut state) = setup(sc);
        let p = Problem {
            cameras: &sc.cameras,
            leds: &sc.leds,
            samples: &sc.samples,
            reference,
        };
        let report = solve(&p, &mut state, &Options::default());
        (state, report, truth(sc, reference))
    }

    fn assert_poses(state: &State, cams: &[Isometry3<f64>], max_mm: f64, max_deg: f64) {
        for (c, t) in cams.iter().enumerate() {
            let dp = (state.ref_from_cam[c].translation.vector - t.translation.vector).norm() * 1e3;
            let da = state.ref_from_cam[c]
                .rotation
                .angle_to(&t.rotation)
                .to_degrees();
            assert!(
                dp < max_mm && da < max_deg,
                "sensor {c}: {dp:.4} mm {da:.5} deg"
            );
        }
    }

    /// Exact data from a noisy start: the solver must land on the truth (no bias, full convergence).
    #[test]
    fn recovers_a_noise_free_room_exactly() {
        let sc = synth::scene(&SceneParams {
            pixel_noise: 0.0,
            ..Default::default()
        });
        let (state, report, cams) = run(&sc);
        assert_poses(&state, &cams, 1e-6, 1e-6);
        assert!(
            report.camera_rms_px.iter().all(|&r| r < 1e-6),
            "{:?}",
            report.camera_rms_px
        );
    }

    /// 0.3 px noise, 300 samples: the measured precision is about 1 mm and 0.015 deg per sensor;
    /// bounds leave ~2x margin.
    #[test]
    fn converges_to_the_true_sensor_poses() {
        let sc = synth::scene(&SceneParams::default());
        let (state, report, cams) = run(&sc);
        assert_poses(&state, &cams, 2.5, 0.05);
        assert!(report.final_cost < report.initial_cost);
        assert!(
            report.camera_rms_px.iter().all(|&r| r > 0.2 && r < 0.6),
            "{:?}",
            report.camera_rms_px
        );
    }
}
