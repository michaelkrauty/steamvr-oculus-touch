// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Joint calibration from a rig capture: the Touches fixed to a Lighthouse-tracked Index
//! controller and held still in many orientations (`cv1touch eval-capture --free`). In every hold
//! the Touches' poses follow from the Index's: world_from_touch = X^-1 * L * Y, with L the Index's
//! Lighthouse pose, X the registration (Lighthouse <- world) and Y the Touch's fixed offset on the
//! rig. The LED blobs each sensor sees then pin, jointly: the sensor poses (one sensor held fixed:
//! it carries the world frame and its gravity), X and every Y. Static holds take timing and rig
//! flex out; averaging a hold's frames takes the pixel noise out.
//!
//! Association is nearest-blob, so every prediction must start within about a pixel: a sensor
//! that moved further since the calibration is re-posed first (`track::refine`).

use std::collections::BTreeMap;

use nalgebra::{
    DMatrix, DVector, Isometry3, Point3, Translation3, UnitQuaternion, Vector2, Vector3,
};

use super::camera::CameraModel;
use super::capture::Led;
use crate::eval::TruthSample;

/// A small rigid motion from (rotation vector, translation).
fn exp6(v: &[f64]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(v[3], v[4], v[5]),
        UnitQuaternion::from_scaled_axis(Vector3::new(v[0], v[1], v[2])),
    )
}

/// One hold: the reference controller's Lighthouse pose and each sensor's frames during it.
#[derive(Debug, Clone)]
pub struct Hold {
    pub lighthouse_from_ref: Isometry3<f64>,
    /// Per sensor index: the blob positions of each frame.
    pub frames: Vec<Vec<Vec<Vector2<f64>>>>,
}

#[derive(Debug, Clone)]
pub struct Problem {
    pub cameras: Vec<CameraModel>,
    /// Sensor held fixed (its pose carries the world frame).
    pub fixed: usize,
    /// Per Touch: its LED model.
    pub touches: Vec<Vec<Led>>,
    pub holds: Vec<Hold>,
}

/// Unknowns: each sensor's cam_from_world, the registration, each Touch's ref_from_touch.
#[derive(Debug, Clone, PartialEq)]
pub struct State {
    pub cam_from_world: Vec<Isometry3<f64>>,
    pub lighthouse_from_world: Isometry3<f64>,
    pub ref_from_touch: Vec<Isometry3<f64>>,
}

impl State {
    fn params(&self, _p: &Problem) -> usize {
        6 * (self.cam_from_world.len() - 1) + 6 + 6 * self.ref_from_touch.len()
    }

    /// The state moved by `d` (layout: free sensors in order, registration, Touches).
    fn moved(&self, p: &Problem, d: &[f64]) -> State {
        let mut s = self.clone();
        let mut k = 0;
        for c in 0..s.cam_from_world.len() {
            if c == p.fixed {
                continue;
            }
            s.cam_from_world[c] = exp6(&d[k..k + 6]) * s.cam_from_world[c];
            k += 6;
        }
        s.lighthouse_from_world = exp6(&d[k..k + 6]) * s.lighthouse_from_world;
        k += 6;
        for t in 0..s.ref_from_touch.len() {
            s.ref_from_touch[t] *= exp6(&d[k..k + 6]);
            k += 6;
        }
        s
    }

    pub fn world_from_touch(&self, hold: &Hold, t: usize) -> Isometry3<f64> {
        self.lighthouse_from_world.inverse() * hold.lighthouse_from_ref * self.ref_from_touch[t]
    }
}

/// A hold's averaged sighting of one LED by one sensor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sighting {
    pub hold: usize,
    pub camera: usize,
    pub touch: usize,
    pub led: usize,
    pub xy: Vector2<f64>,
    pub frames: usize,
}

/// Matches every frame's blobs to the LEDs of all Touches at once (mutual nearest within `gate`,
/// runner-ups `margin` further) and averages each LED's matched positions over the hold. An LED
/// matched in fewer than half the frames of its sensor in that hold is left out.
pub fn associate(p: &Problem, s: &State, gate: f64, margin: f64) -> Vec<Sighting> {
    let mut out = Vec::new();
    for (k, hold) in p.holds.iter().enumerate() {
        for (c, frames) in hold.frames.iter().enumerate() {
            if frames.is_empty() {
                continue;
            }
            let cam = &p.cameras[c];
            let cfw = s.cam_from_world[c];
            let cam_pos = cfw.inverse().translation.vector;
            // predictions of every facing LED of every Touch
            let mut pred: Vec<(usize, usize, Vector2<f64>)> = Vec::new();
            for (t, leds) in p.touches.iter().enumerate() {
                let w = s.world_from_touch(hold, t);
                for (i, l) in leds.iter().enumerate() {
                    let pw = w * Point3::from(l.pos);
                    let n = w.rotation * l.dir;
                    if n.dot(&(cam_pos - pw.coords).normalize()) < 0.2 {
                        continue;
                    }
                    if let Some(uv) = cam.project(&(cfw * pw).coords) {
                        pred.push((t, i, uv));
                    }
                }
            }
            let mut acc: BTreeMap<(usize, usize), (Vector2<f64>, usize)> = BTreeMap::new();
            for blobs in frames {
                for (pi, &(t, i, uv)) in pred.iter().enumerate() {
                    let mut d: Vec<(f64, usize)> = blobs
                        .iter()
                        .enumerate()
                        .map(|(b, x)| ((x - uv).norm(), b))
                        .collect();
                    d.sort_by(|a, b| a.0.total_cmp(&b.0));
                    let Some(&(d1, b)) = d.first() else { continue };
                    if d1 > gate || d.get(1).is_some_and(|x| x.0 < d1 + margin) {
                        continue;
                    }
                    let rival = pred
                        .iter()
                        .enumerate()
                        .any(|(pj, q)| pj != pi && (blobs[b] - q.2).norm() < d1 + margin);
                    if rival {
                        continue;
                    }
                    let e = acc.entry((t, i)).or_insert((Vector2::zeros(), 0));
                    e.0 += blobs[b];
                    e.1 += 1;
                }
            }
            for ((t, i), (sum, n)) in acc {
                if 2 * n >= frames.len() {
                    out.push(Sighting {
                        hold: k,
                        camera: c,
                        touch: t,
                        led: i,
                        xy: sum / n as f64,
                        frames: n,
                    });
                }
            }
        }
    }
    out
}

fn residual(p: &Problem, s: &State, o: &Sighting) -> Option<Vector2<f64>> {
    let w = s.world_from_touch(&p.holds[o.hold], o.touch);
    let pc = s.cam_from_world[o.camera] * (w * Point3::from(p.touches[o.touch][o.led].pos));
    Some(o.xy - p.cameras[o.camera].project(&pc.coords)?)
}

/// Residual norms (px) of the sightings under a state.
pub fn residuals(p: &Problem, s: &State, obs: &[Sighting]) -> Vec<f64> {
    obs.iter()
        .filter_map(|o| residual(p, s, o).map(|r| r.norm()))
        .collect()
}

/// A weak pull of the free sensors toward their prior poses: px of residual per radian and per
/// metre. Holds of a compact rig leave each sensor free to trade a small turn for a sideways shift
/// that moves nothing inside the held volume; the prior (a room-wide calibration) settles those
/// directions and yields to anything the holds do show.
#[derive(Debug, Clone)]
pub struct Prior {
    pub cam_from_world: Vec<Isometry3<f64>>,
    pub px_per_rad: f64,
    pub px_per_m: f64,
}

impl Prior {
    pub fn around(s: &State) -> Prior {
        Prior {
            cam_from_world: s.cam_from_world.clone(),
            px_per_rad: 1.0 / 0.1f64.to_radians(),
            px_per_m: 200.0,
        }
    }

    /// Per free sensor: (its parameter offset, 6 weighted residuals).
    fn residuals(&self, p: &Problem, s: &State) -> Vec<(usize, [f64; 6])> {
        let mut out = Vec::new();
        let mut k = 0;
        for c in 0..s.cam_from_world.len() {
            if c == p.fixed {
                continue;
            }
            let d = s.cam_from_world[c] * self.cam_from_world[c].inverse();
            let (r, t) = (d.rotation.scaled_axis(), d.translation.vector);
            out.push((
                k,
                [
                    r.x * self.px_per_rad,
                    r.y * self.px_per_rad,
                    r.z * self.px_per_rad,
                    t.x * self.px_per_m,
                    t.y * self.px_per_m,
                    t.z * self.px_per_m,
                ],
            ));
            k += 6;
        }
        out
    }
}

/// Levenberg-Marquardt with Huber weights (`huber_px`) over all unknowns at once.
pub fn solve(
    p: &Problem,
    s: &State,
    obs: &[Sighting],
    huber_px: f64,
    iterations: usize,
    prior: Option<&Prior>,
) -> State {
    let n = s.params(p);
    let mut cur = s.clone();
    let mut lambda = 1e-3;
    let cost = |st: &State| -> f64 {
        let data: f64 = obs
            .iter()
            .filter_map(|o| residual(p, st, o))
            .map(|r| {
                let e = r.norm();
                if e <= huber_px {
                    0.5 * e * e
                } else {
                    huber_px * (e - 0.5 * huber_px)
                }
            })
            .sum();
        data + prior.map_or(0.0, |pr| {
            pr.residuals(p, st)
                .iter()
                .map(|(_, r)| 0.5 * r.iter().map(|x| x * x).sum::<f64>())
                .sum()
        })
    };
    let mut c0 = cost(&cur);
    for _ in 0..iterations {
        let (mut h, mut g) = (DMatrix::<f64>::zeros(n, n), DVector::<f64>::zeros(n));
        let eps = 1e-6;
        let shifted: Vec<State> = (0..n)
            .map(|j| {
                let mut d = vec![0.0; n];
                d[j] = eps;
                cur.moved(p, &d)
            })
            .collect();
        for o in obs {
            let Some(r0) = residual(p, &cur, o) else {
                continue;
            };
            let mut jac = DMatrix::<f64>::zeros(2, n);
            for (j, sj) in shifted.iter().enumerate() {
                let Some(rj) = residual(p, sj, o) else {
                    continue;
                };
                // r = observed - predicted: d(predicted)/dp = -(rj - r0)/eps
                let col = -(rj - r0) / eps;
                jac[(0, j)] = col.x;
                jac[(1, j)] = col.y;
            }
            let e = r0.norm();
            let w = if e <= huber_px { 1.0 } else { huber_px / e };
            let r = DVector::from_vec(vec![r0.x, r0.y]);
            h += jac.transpose() * &jac * w;
            g += jac.transpose() * r * w;
        }
        if let Some(pr) = prior {
            // the prior residual moves one to one with the sensor's own (left) perturbation
            for (k, r) in pr.residuals(p, &cur) {
                let scale = [
                    pr.px_per_rad,
                    pr.px_per_rad,
                    pr.px_per_rad,
                    pr.px_per_m,
                    pr.px_per_m,
                    pr.px_per_m,
                ];
                for i in 0..6 {
                    h[(k + i, k + i)] += scale[i] * scale[i];
                    g[k + i] -= scale[i] * r[i];
                }
            }
        }
        let mut improved = false;
        for _ in 0..8 {
            let mut hl = h.clone();
            for j in 0..n {
                hl[(j, j)] *= 1.0 + lambda;
            }
            let Some(step) = hl.cholesky().map(|ch| ch.solve(&g)) else {
                lambda *= 10.0;
                continue;
            };
            let next = cur.moved(p, step.as_slice());
            let c1 = cost(&next);
            if c1 < c0 {
                cur = next;
                c0 = c1;
                lambda = (lambda * 0.3).max(1e-9);
                improved = step.norm() > 1e-10;
                break;
            }
            lambda *= 10.0;
        }
        if !improved {
            break;
        }
    }
    cur
}

#[derive(Debug, Clone)]
pub struct Outcome {
    pub state: State,
    pub sightings: Vec<Sighting>,
}

/// Association rounds with a tightening gate, each followed by a solve. Holds where `use_hold`
/// is false are left out (for validation).
pub fn calibrate(p: &Problem, initial: &State, use_hold: &dyn Fn(usize) -> bool) -> Outcome {
    let debug = std::env::var_os("RIG_DEBUG").is_some();
    let med = |mut v: Vec<f64>| {
        v.sort_by(f64::total_cmp);
        v.get(v.len() / 2).copied().unwrap_or(f64::NAN)
    };
    let schedule = [(4.0, 1.5), (2.5, 1.0), (1.5, 1.0), (1.0, 0.8)];
    // first the registration and the rig offsets with the sensors held (a rough registration
    // misplaces the Touches by pixels in every sensor; free sensors would bend to wrong matches),
    // then everything, the sensors under their weak prior
    let frozen = Prior {
        px_per_rad: 1e9,
        px_per_m: 1e9,
        ..Prior::around(initial)
    };
    let weak = Prior::around(initial);
    let mut s = initial.clone();
    let mut obs = Vec::new();
    for (stage, prior) in [("offsets", &frozen), ("all", &weak)] {
        for (gate, margin) in schedule {
            obs = associate(p, &s, gate, margin)
                .into_iter()
                .filter(|o| use_hold(o.hold))
                .collect();
            let before = residuals(p, &s, &obs);
            s = solve(p, &s, &obs, 0.5, 30, Some(prior));
            if debug {
                eprintln!(
                    "{stage} gate {gate}: {} sightings, median residual {:.3} -> {:.3} px",
                    obs.len(),
                    med(before),
                    med(residuals(p, &s, &obs))
                );
            }
        }
    }
    Outcome {
        state: s,
        sightings: obs,
    }
}

/// Holds of a reference controller: spans of at least `min_ns` moving less than 2 mm and 0.3
/// degrees from their start, trimmed by a quarter second at both ends; with its mean pose.
pub fn holds(samples: &[TruthSample], min_ns: u64) -> Vec<(u64, u64, Isometry3<f64>)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < samples.len() {
        let s = samples[i];
        let mut j = i;
        while j + 1 < samples.len()
            && (samples[j + 1].pose.translation.vector - s.pose.translation.vector).norm() < 0.002
            && samples[j + 1].pose.rotation.angle_to(&s.pose.rotation) < 0.3f64.to_radians()
        {
            j += 1;
        }
        if samples[j].t_ns - s.t_ns >= min_ns {
            let (a, b) = (s.t_ns + 250_000_000, samples[j].t_ns - 250_000_000);
            let set: Vec<&TruthSample> = samples[i..=j]
                .iter()
                .filter(|x| x.t_ns >= a && x.t_ns <= b)
                .collect();
            let t = set
                .iter()
                .map(|x| x.pose.translation.vector)
                .sum::<Vector3<f64>>()
                / set.len() as f64;
            let r = super::motion::chordal_mean(
                &set.iter().map(|x| x.pose.rotation).collect::<Vec<_>>(),
            );
            out.push((a, b, Isometry3::from_parts(Translation3::from(t), r)));
            i = j + 1;
        } else {
            i += 1;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::{self, Rng};

    /// A synthetic rig: three sensors, two Touches on an Index, 30 holds. One sensor has moved
    /// since the calibration the solve starts from; the registration and the offsets are rough.
    #[test]
    fn recovers_sensors_registration_and_offsets() {
        let mut rng = Rng::new(9);
        let target = Vector3::new(0.0, 1.0, 0.0);
        let world_from_cam: Vec<Isometry3<f64>> = [
            (Vector3::new(-2.0, 2.3, 1.8), true),
            (Vector3::new(2.0, 2.3, 1.8), false),
            (Vector3::new(0.3, 2.4, -2.2), true),
        ]
        .iter()
        .map(|&(pos, flip)| synth::look_at(pos, target, flip))
        .collect();
        let x_true = Isometry3::from_parts(
            Translation3::new(0.1, -0.6, -2.3),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.002, 1.2, -0.003)),
        );
        let y_true = [
            Isometry3::from_parts(
                Translation3::new(0.03, -0.05, 0.08),
                UnitQuaternion::from_scaled_axis(Vector3::new(0.3, 0.1, 0.0)),
            ),
            Isometry3::from_parts(
                Translation3::new(-0.06, -0.04, 0.09),
                UnitQuaternion::from_scaled_axis(Vector3::new(-0.2, 2.9, 0.1)),
            ),
        ];
        let leds = synth::touch_leds();
        let cams = vec![synth::camera_model(); 3];
        let truth = State {
            cam_from_world: world_from_cam.iter().map(|w| w.inverse()).collect(),
            lighthouse_from_world: x_true,
            ref_from_touch: y_true.to_vec(),
        };
        let mut holds_v = Vec::new();
        for _ in 0..30 {
            let world_from_ref = Isometry3::from_parts(
                Translation3::new(
                    rng.range(-0.5, 0.5),
                    rng.range(0.7, 1.5),
                    rng.range(-0.5, 0.5),
                ),
                UnitQuaternion::from_scaled_axis(Vector3::new(
                    rng.range(-1.2, 1.2),
                    rng.range(-3.1, 3.1),
                    rng.range(-1.2, 1.2),
                )),
            );
            let hold0 = Hold {
                lighthouse_from_ref: x_true * world_from_ref,
                frames: vec![],
            };
            let frames = (0..3)
                .map(|c| {
                    (0..10)
                        .map(|_| {
                            let mut blobs = Vec::new();
                            for t in 0..2 {
                                let w = truth.world_from_touch(&hold0, t);
                                for l in &leds {
                                    let pw = w * Point3::from(l.pos);
                                    let cam_pos = world_from_cam[c].translation.vector;
                                    if (w.rotation * l.dir).dot(&(cam_pos - pw.coords).normalize())
                                        < 0.3
                                    {
                                        continue;
                                    }
                                    if let Some(uv) =
                                        cams[c].project(&(truth.cam_from_world[c] * pw).coords)
                                    {
                                        blobs.push(
                                            uv + Vector2::new(rng.gauss(), rng.gauss()) * 0.3,
                                        );
                                    }
                                }
                            }
                            blobs
                        })
                        .collect()
                })
                .collect();
            holds_v.push(Hold {
                lighthouse_from_ref: hold0.lighthouse_from_ref,
                frames,
            });
        }
        let p = Problem {
            cameras: cams,
            fixed: 0,
            touches: vec![leds.clone(), leds],
            holds: holds_v,
        };
        let bump = |a: &Isometry3<f64>, mm: f64, deg: f64| {
            exp6(&[
                deg.to_radians(),
                -deg.to_radians() * 0.5,
                deg.to_radians() * 0.3,
                mm * 1e-3,
                -mm * 0.7e-3,
                mm * 0.4e-3,
            ]) * a
        };
        // the old calibration is right except that sensor 2 has since drifted 2 mm and 0.05
        // degrees (a pixel: larger moves are `refine`'s, which aligns the whole pattern first);
        // the registration and the rig offsets are only roughly known
        let initial = State {
            cam_from_world: vec![
                truth.cam_from_world[0],
                truth.cam_from_world[1],
                bump(&truth.cam_from_world[2], 2.0, 0.05),
            ],
            lighthouse_from_world: bump(&x_true, 6.0, 0.3),
            ref_from_touch: vec![
                y_true[0] * exp6(&[0.005, 0.0, 0.004, 0.003, -0.002, 0.004]),
                y_true[1] * exp6(&[-0.004, 0.003, 0.0, -0.003, 0.004, 0.002]),
            ],
        };
        let out = calibrate(&p, &initial, &|_| true);
        let err = |a: &Isometry3<f64>, b: &Isometry3<f64>| {
            let d = a.inverse() * b;
            (
                d.translation.vector.norm() * 1e3,
                d.rotation.angle().to_degrees(),
            )
        };
        // a sensor is right where it reprojects what was held (every LED in every hold, in view)
        // like the true one; directions the holds cannot see are the prior's to keep
        for c in 1..3 {
            let mut worst: f64 = 0.0;
            for hold in &p.holds {
                for t in 0..2 {
                    for l in &p.touches[t] {
                        let q = truth.world_from_touch(hold, t) * Point3::from(l.pos);
                        let (a, b) = (
                            p.cameras[c].project(&(out.state.cam_from_world[c] * q).coords),
                            p.cameras[c].project(&(truth.cam_from_world[c] * q).coords),
                        );
                        if let (Some(a), Some(b)) = (a, b) {
                            if b.x >= 0.0 && b.y >= 0.0 && b.x < 1280.0 && b.y < 960.0 {
                                worst = worst.max((a - b).norm());
                            }
                        }
                    }
                }
            }
            assert!(
                worst < 0.3,
                "sensor {c} reprojects what was held {worst} px off"
            );
        }
        let (mm, deg) = err(&out.state.lighthouse_from_world, &x_true);
        assert!(mm < 1.0 && deg < 0.03, "registration: {mm} mm {deg} deg");
        for (t, expected) in y_true.iter().enumerate() {
            let (mm, deg) = err(&out.state.ref_from_touch[t], expected);
            assert!(mm < 1.0 && deg < 0.1, "offset {t}: {mm} mm {deg} deg");
        }
        let r = residuals(&p, &out.state, &out.sightings);
        assert!(
            r.iter().sum::<f64>() / (r.len() as f64) < 0.15,
            "mean residual {}",
            r.iter().sum::<f64>() / r.len() as f64
        );
    }
}
