// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Calibration solve: confident engine sightings -> gyro-consistency filter -> robust initial
//! poses -> vision-only bundle adjustment with a tightening rejection schedule -> gravity from the
//! controllers' motion -> world frame (gravity-aligned, +Y up) -> per-sensor world poses and a report.

use std::collections::BTreeMap;

use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3};

use super::ba::{self, Problem, State};
use super::camera::CameraModel;
use super::capture::{Capture, ImuSample, Led, WandSample};
use super::gyro::{self, FilterParams};
use super::init;
use super::motion::{self, MotionGravity, MotionParams, Subset};

#[derive(Debug, Clone)]
pub struct SolveOptions {
    /// Fewest inlier relative-pose samples for a sensor pair to link them.
    pub min_links: usize,
    /// Fewest usable LED observations per sensor after outlier rejection.
    pub min_observations: usize,
    /// Per round, the sightings in use are re-selected from every gyro-checked sighting: those whose
    /// RMS reprojection under the current state is within the round's limit (px). Rejection is not
    /// permanent, so sightings dropped while the state was still off return once it converges.
    /// After the schedule, the limit follows the noise: max(`final_floor_px`, `final_median_factor`
    /// x the median sighting RMS), repeated until the selection stops changing, for at most
    /// `rounds` rounds in all. A fixed limit is too loose for real sightings (median 0.11 px): a
    /// two-sensor sample with one misplaced sighting splits its error below it.
    pub reject_schedule_px: Vec<f64>,
    pub final_floor_px: f64,
    pub final_median_factor: f64,
    pub rounds: usize,
    pub gyro: FilterParams,
    pub motion: MotionParams,
    pub ba: ba::Options,
}

impl Default for SolveOptions {
    fn default() -> Self {
        SolveOptions {
            min_links: 30,
            min_observations: 200,
            reject_schedule_px: vec![8.0, 4.0, 2.0],
            final_floor_px: 0.5,
            final_median_factor: 4.0,
            rounds: 10,
            gyro: FilterParams::default(),
            motion: MotionParams::default(),
            ba: ba::Options::default(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct SensorResult {
    pub serial: String,
    pub world_from_cam: Isometry3<f64>,
    pub rms_px: f64,
    pub observations: usize,
    /// Gyro-checked sightings of this sensor, and how many of them reprojection left out.
    pub views: usize,
    pub rejected: usize,
}

#[derive(Debug, Clone)]
pub struct Solution {
    pub sensors: Vec<SensorResult>,
    pub reference: String,
    /// Gravity from the motion, in the world frame (+Y up by construction; kept for its magnitude,
    /// residuals and per-controller fits).
    pub gravity: MotionGravity,
    pub samples: usize,
    /// Sightings dropped before solving: no engine pose, or not a confident match.
    pub unposed_views: usize,
    /// Posed sightings the gyro contradicted.
    pub gyro_rejected_views: usize,
    /// Sightings rejected by reprojection during the bundle adjustment.
    pub rejected_views: usize,
    pub iterations: usize,
}

pub fn solve_capture(c: &Capture, o: &SolveOptions) -> Result<Solution, String> {
    solve_samples(
        &c.serials,
        &c.cameras,
        &c.leds,
        c.samples.clone(),
        &c.imu,
        o,
    )
}

/// Keeps the sightings the engine posed with a confident match; returns how many it dropped.
fn confident_views(samples: &mut Vec<WandSample>) -> usize {
    let before: usize = samples.iter().map(|s| s.views.len()).sum();
    for s in samples.iter_mut() {
        s.views.retain(|v| v.cam_from_obj.is_some() && v.strong);
    }
    samples.retain(|s| !s.views.is_empty());
    before - samples.iter().map(|s| s.views.len()).sum::<usize>()
}

pub fn solve_samples(
    serials: &[String],
    cameras: &[CameraModel],
    leds: &BTreeMap<i32, Vec<Led>>,
    mut samples: Vec<WandSample>,
    imu: &BTreeMap<i32, Vec<ImuSample>>,
    o: &SolveOptions,
) -> Result<Solution, String> {
    let unposed_views = confident_views(&mut samples);
    let gyro_rejected_views = gyro::filter_views(&mut samples, imu, &o.gyro).dropped;
    let initial = init::initialize(&samples, serials, o.min_links)?;
    let all = samples;
    let mut active = all.clone();
    let mut state = State {
        ref_from_obj: init::wand_poses(&active, &initial.ref_from_cam),
        ref_from_cam: initial.ref_from_cam,
    };
    let mut iterations = 0;
    let mut report = ba::Report::default();
    let last = o.reject_schedule_px.len();
    for round in 0..o.rounds.max(1) {
        let p = Problem {
            cameras,
            leds,
            samples: &active,
            reference: initial.reference,
        };
        report = ba::solve(&p, &mut state, &o.ba);
        iterations += report.iterations;
        if round + 1 == o.rounds {
            break;
        }
        let limit = match o.reject_schedule_px.get(round) {
            Some(&l) => l,
            None => o.final_floor_px.max(
                o.final_median_factor
                    * median_view_rms(
                        &Problem {
                            cameras,
                            leds,
                            samples: &active,
                            reference: initial.reference,
                        },
                        &state,
                    ),
            ),
        };
        let next = select_views(cameras, leds, &all, &state, initial.reference, limit);
        let same = |a: &WandSample, b: &WandSample| {
            a.views
                .iter()
                .map(|v| v.camera)
                .eq(b.views.iter().map(|v| v.camera))
        };
        let changed: Vec<usize> = (0..all.len())
            .filter(|&i| !same(&next[i], &active[i]))
            .collect();
        active = next;
        if changed.is_empty() && round >= last {
            break;
        }
        // a sample whose sightings changed restarts from its sightings' engine poses
        let fresh = init::wand_poses(&active, &state.ref_from_cam);
        for &i in &changed {
            state.ref_from_obj[i] = fresh[i];
        }
    }
    let count = |set: &[WandSample], c: usize| {
        set.iter()
            .flat_map(|s| &s.views)
            .filter(|v| v.camera == c)
            .count()
    };
    for (c, serial) in serials.iter().enumerate() {
        if report.camera_obs[c] < o.min_observations {
            return Err(format!(
                "sensor {serial} has only {} usable LED observations (need {}): wave the controllers where it can see them",
                report.camera_obs[c], o.min_observations
            ));
        }
    }
    let mut gravity = motion::estimate(&active, &state.ref_from_obj, imu, &o.motion)?;
    let points: Vec<Vector3<f64>> = active
        .iter()
        .zip(&state.ref_from_obj)
        .filter(|(s, _)| !s.views.is_empty())
        .filter_map(|(_, t)| Some(t.as_ref()?.translation.vector))
        .collect();
    let world = world_from_ref(&gravity.gravity, &points);
    gravity.gravity = world.rotation * gravity.gravity;
    for (_, g) in &mut gravity.independent {
        *g = world.rotation * *g;
    }
    Ok(Solution {
        sensors: serials
            .iter()
            .enumerate()
            .map(|(c, serial)| SensorResult {
                serial: serial.clone(),
                world_from_cam: world * state.ref_from_cam[c],
                rms_px: report.camera_rms_px[c],
                observations: report.camera_obs[c],
                views: count(&all, c),
                rejected: count(&all, c) - count(&active, c),
            })
            .collect(),
        reference: serials[initial.reference].clone(),
        gravity,
        samples: active.iter().filter(|s| !s.views.is_empty()).count(),
        unposed_views,
        gyro_rejected_views,
        rejected_views: all.iter().map(|s| s.views.len()).sum::<usize>()
            - active.iter().map(|s| s.views.len()).sum::<usize>(),
        iterations,
    })
}

fn median_view_rms(p: &Problem, s: &State) -> f64 {
    let mut r: Vec<f64> = p
        .samples
        .iter()
        .enumerate()
        .flat_map(|(i, smp)| (0..smp.views.len()).filter_map(move |v| ba::view_rms_px(p, s, i, v)))
        .collect();
    r.sort_by(f64::total_cmp);
    r.get(r.len() / 2).copied().unwrap_or(0.0)
}

/// Every sample with the sightings whose RMS reprojection under `state` is within `limit`. A sample
/// without a pose is judged from its sightings' engine poses. A sample that loses a sighting and
/// keeps fewer than two is dropped whole: two sightings that disagree cannot say which is wrong,
/// and a pose fitted to the wrong one would mislead the gravity estimate.
fn select_views(
    cameras: &[CameraModel],
    leds: &BTreeMap<i32, Vec<Led>>,
    all: &[WandSample],
    state: &State,
    reference: usize,
    limit: f64,
) -> Vec<WandSample> {
    let seeds = init::wand_poses(all, &state.ref_from_cam);
    let judged = State {
        ref_from_cam: state.ref_from_cam.clone(),
        ref_from_obj: state
            .ref_from_obj
            .iter()
            .zip(&seeds)
            .map(|(s, seed)| s.or(*seed))
            .collect(),
    };
    let p = Problem {
        cameras,
        leds,
        samples: all,
        reference,
    };
    all.iter()
        .enumerate()
        .map(|(i, s)| {
            let mut views: Vec<_> = s
                .views
                .iter()
                .enumerate()
                .filter(|(v, _)| ba::view_rms_px(&p, &judged, i, *v).is_some_and(|r| r <= limit))
                .map(|(_, v)| v.clone())
                .collect();
            if views.len() < s.views.len() && views.len() < 2 {
                views.clear();
            }
            WandSample { views, ..s.clone() }
        })
        .collect()
}

/// world_from_ref: +Y along `up` (given in the reference frame), the reference sensor's optical axis
/// projected onto the horizontal along -Z, origin at the centroid of `points` (the waved controller
/// positions, i.e. the middle of the play space).
pub fn world_from_ref(up: &Vector3<f64>, points: &[Vector3<f64>]) -> Isometry3<f64> {
    let y = up.normalize();
    let mut forward = Vector3::z() - y * y.z;
    if forward.norm() < 1e-6 {
        forward = Vector3::x() - y * y.x; // reference sensor looks straight down
    }
    let z = -forward.normalize();
    let x = y.cross(&z);
    let r = Rotation3::from_matrix_unchecked(Matrix3::from_rows(&[
        x.transpose(),
        y.transpose(),
        z.transpose(),
    ]));
    let rotation = UnitQuaternion::from_rotation_matrix(&r);
    let centroid = if points.is_empty() {
        Vector3::zeros()
    } else {
        points.iter().sum::<Vector3<f64>>() / points.len() as f64
    };
    Isometry3::from_parts(Translation3::from(-(rotation * centroid)), rotation)
}

#[derive(Debug, Clone, Copy)]
pub struct Acceptance {
    pub max_rms_px: f64,
    /// Plausible |G| (m/s^2): a gross check on timing, units and the accelerometer scale.
    pub gravity_range: (f64, f64),
    /// Largest angle between two independent gravity estimates.
    pub max_spread_deg: f64,
    /// Largest share of a sensor's gyro-checked sightings that reprojection may leave out: more
    /// means its sightings disagree with each other (a sensor bumped mid-capture).
    pub max_rejected_fraction: f64,
}

impl Default for Acceptance {
    fn default() -> Self {
        Acceptance {
            max_rms_px: 1.0,
            gravity_range: (9.6, 10.0),
            max_spread_deg: 0.5,
            max_rejected_fraction: 0.1,
        }
    }
}

pub fn check(sol: &Solution, a: &Acceptance) -> Result<(), String> {
    let mut problems: Vec<String> = sol
        .sensors
        .iter()
        // Incomparable values (NaN on either side) also fail acceptance.
        .filter(|s| {
            s.rms_px
                .partial_cmp(&a.max_rms_px)
                .is_none_or(|o| o.is_gt())
        })
        .map(|s| {
            format!(
                "{} reprojection {:.2} px > {:.2} px",
                s.serial, s.rms_px, a.max_rms_px
            )
        })
        .collect();
    let g = sol.gravity.gravity.norm();
    if !(g >= a.gravity_range.0 && g <= a.gravity_range.1) {
        problems.push(format!(
            "gravity magnitude {g:.3} m/s^2 outside {:.1}-{:.1}",
            a.gravity_range.0, a.gravity_range.1
        ));
    }
    for s in &sol.sensors {
        let share = s.rejected as f64 / s.views.max(1) as f64;
        if share
            .partial_cmp(&a.max_rejected_fraction)
            .is_none_or(|o| o.is_gt())
        {
            problems.push(format!(
                "{}: {} of {} sightings rejected by reprojection ({:.0}% > {:.0}%; was it moved during the capture?)",
                s.serial,
                s.rejected,
                s.views,
                share * 100.0,
                a.max_rejected_fraction * 100.0
            ));
        }
    }
    match sol.gravity.spread_deg() {
        Some(spread)
            if spread
                .partial_cmp(&a.max_spread_deg)
                .is_none_or(|o| o.is_gt()) =>
        {
            problems.push(format!(
                "independent gravity estimates disagree by {spread:.2} deg > {:.2} deg",
                a.max_spread_deg
            ))
        }
        Some(_) => {}
        None => problems.push(
            "gravity has no independent cross-check: wave both controllers, or one for longer"
                .into(),
        ),
    }
    if problems.is_empty() {
        Ok(())
    } else {
        Err(problems.join("; "))
    }
}

fn hand(device: i32) -> &'static str {
    match device {
        super::DEVICE_RIGHT => "right",
        super::DEVICE_LEFT => "left",
        _ => "controller",
    }
}

pub fn format_report(sol: &Solution) -> String {
    let g = &sol.gravity;
    let names: Vec<String> = g
        .independent
        .iter()
        .map(|(s, _)| match *s {
            Subset::Device(d) => hand(d).to_string(),
            Subset::FirstHalf(d) => format!("{} first half", hand(d)),
            Subset::SecondHalf(d) => format!("{} second half", hand(d)),
        })
        .collect();
    let spread = g
        .spread_deg()
        .map_or("no independent cross-check".to_string(), |d| {
            format!(
                "independent estimates ({}) within {d:.2} deg",
                names.join(", ")
            )
        });
    let mut out = format!(
        "reference sensor {}; {} controller samples; sightings dropped: {} unposed, {} gyro-inconsistent, {} by reprojection; {} iterations\n\
         gravity {:.3} m/s^2 from {} motion segments (residual median {:.1} mm, p90 {:.1} mm); {spread}\n",
        sol.reference,
        sol.samples,
        sol.unposed_views,
        sol.gyro_rejected_views,
        sol.rejected_views,
        sol.iterations,
        g.gravity.norm(),
        g.segments,
        g.residual_median_m * 1e3,
        g.residual_p90_m * 1e3
    );
    for d in &g.devices {
        out += &format!(
            "  {:5}  {} segments  accel bias ({:+.3}, {:+.3}, {:+.3}) m/s^2  IMU at ({:+.1}, {:+.1}, {:+.1}) mm\n",
            hand(d.device),
            d.segments,
            d.accel_bias.x,
            d.accel_bias.y,
            d.accel_bias.z,
            d.lever_arm.x * 1e3,
            d.lever_arm.y * 1e3,
            d.lever_arm.z * 1e3
        );
    }
    for s in &sol.sensors {
        let p = s.world_from_cam.translation.vector;
        let pitch = (s.world_from_cam.rotation * Vector3::z())
            .y
            .clamp(-1.0, 1.0)
            .asin()
            .to_degrees();
        out += &format!(
            "  {}  rms {:.3} px  obs {:6}  sightings {:5} ({} rejected)  pos ({:+.3}, {:+.3}, {:+.3}) m  looks {:+.1} deg\n",
            s.serial, s.rms_px, s.observations, s.views, s.rejected, p.x, p.y, p.z, pitch
        );
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::motion::DeviceFit;
    use crate::calib::synth::{self, TrajectoryParams, TrajectoryScene};
    use nalgebra::Point3;

    fn solve_trajectory(
        p: &TrajectoryParams,
        o: &SolveOptions,
    ) -> (TrajectoryScene, Result<Solution, String>) {
        let tr = synth::trajectory(p);
        let sol = solve_samples(
            &tr.scene.serials,
            &tr.scene.cameras,
            &tr.scene.leds,
            tr.scene.samples.clone(),
            &tr.imu,
            o,
        );
        (tr, sol)
    }

    fn short(p: TrajectoryParams) -> TrajectoryParams {
        TrajectoryParams {
            duration_s: 12.0,
            ..p
        }
    }

    /// The solved world differs from the scene's by a yaw about +Y and an offset: compare poses
    /// relative to sensor 0, and compare each sensor's tilt (yaw-invariant) directly.
    fn assert_matches_truth(
        tr: &TrajectoryScene,
        sol: &Solution,
        max_mm: f64,
        max_deg: f64,
        max_tilt: f64,
    ) {
        let sc = &tr.scene;
        let (w0, t0) = (sol.sensors[0].world_from_cam, sc.world_from_cam[0]);
        for (c, s) in sol.sensors.iter().enumerate() {
            assert_eq!(s.serial, sc.serials[c]);
            let (solved, truth) = (
                w0.inverse() * s.world_from_cam,
                t0.inverse() * sc.world_from_cam[c],
            );
            let dp = (solved.translation.vector - truth.translation.vector).norm();
            let da = solved.rotation.angle_to(&truth.rotation).to_degrees();
            assert!(
                dp * 1e3 < max_mm && da < max_deg,
                "{}: {:.4} mm {da:.5} deg",
                s.serial,
                dp * 1e3
            );
            for axis in [Vector3::x(), Vector3::z()] {
                let (a, b) = (
                    (s.world_from_cam.rotation * axis).y,
                    (sc.world_from_cam[c].rotation * axis).y,
                );
                assert!((a - b).abs() < max_tilt, "{} tilt {a} vs {b}", s.serial);
            }
        }
    }

    /// Exact sightings and engine poses: the whole pipeline lands on the truth, level included.
    #[test]
    fn solves_a_noise_free_room_exactly_and_level() {
        let p = short(TrajectoryParams {
            pixel_noise: 0.0,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            ..Default::default()
        });
        let (tr, sol) = solve_trajectory(&p, &SolveOptions::default());
        let sol = sol.unwrap();
        assert_matches_truth(&tr, &sol, 1e-3, 1e-5, 5e-4);
        assert_eq!(
            sol.rejected_views + sol.gyro_rejected_views + sol.unposed_views,
            0
        );
        assert!(
            (sol.gravity.gravity - Vector3::new(0.0, 9.80665, 0.0)).norm() < 5e-3,
            "{:?}",
            sol.gravity.gravity
        );
        check(&sol, &Acceptance::default()).unwrap();
    }

    /// 0.3 px LED noise and noisy engine poses: millimetres, hundredths of a degree, level to a
    /// tenth of a degree.
    #[test]
    fn solves_a_noisy_room_within_its_precision() {
        let (tr, sol) = solve_trajectory(
            &short(TrajectoryParams::default()),
            &SolveOptions::default(),
        );
        let sol = sol.unwrap();
        assert_matches_truth(&tr, &sol, 2.5, 0.05, 2e-3);
        check(&sol, &Acceptance::default()).unwrap();
    }

    /// Review fix 1: the cross-check compares independent estimates with each other. Two
    /// controllers whose gravity readings disagree by 0.7 deg must fail, not pass as two 0.35 deg
    /// halves of a joint estimate.
    #[test]
    fn disagreeing_controllers_fail_acceptance() {
        let p = short(TrajectoryParams {
            pixel_noise: 0.0,
            device_gravity_tilt: [
                Vector3::zeros(),
                Vector3::new(0.7f64.to_radians(), 0.0, 0.0),
            ],
            ..Default::default()
        });
        let (_, sol) = solve_trajectory(&p, &SolveOptions::default());
        let sol = sol.unwrap();
        let spread = sol.gravity.spread_deg().unwrap();
        assert!((spread - 0.7).abs() < 0.15, "{spread}");
        let err = check(&sol, &Acceptance::default()).unwrap_err();
        assert!(err.contains("disagree"), "{err}");
    }

    /// Review fix 1: with one controller, its first and second halves are the independent pair.
    #[test]
    fn one_controller_is_cross_checked_by_its_halves() {
        let tr = synth::trajectory(&TrajectoryParams {
            duration_s: 20.0,
            ..Default::default()
        });
        let right: BTreeMap<i32, Vec<ImuSample>> = tr
            .imu
            .iter()
            .filter(|(d, _)| **d == super::super::DEVICE_RIGHT)
            .map(|(d, s)| (*d, s.clone()))
            .collect();
        let sol = solve_samples(
            &tr.scene.serials,
            &tr.scene.cameras,
            &tr.scene.leds,
            tr.scene.samples.clone(),
            &right,
            &SolveOptions::default(),
        )
        .unwrap();
        let subsets: Vec<Subset> = sol.gravity.independent.iter().map(|(s, _)| *s).collect();
        assert_eq!(
            subsets,
            vec![
                Subset::FirstHalf(super::super::DEVICE_RIGHT),
                Subset::SecondHalf(super::super::DEVICE_RIGHT)
            ]
        );
        assert!(sol.gravity.spread_deg().unwrap() < 0.2);
        check(&sol, &Acceptance::default()).unwrap();
    }

    /// Review fix 2: a sensor bumped mid-capture fits either half at sub-pixel once the other half
    /// is rejected; the share of its sightings lost to reprojection must fail acceptance.
    #[test]
    fn a_sensor_bumped_mid_capture_fails_acceptance() {
        let tr = synth::trajectory(&short(TrajectoryParams {
            pixel_noise: 0.0,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            ..Default::default()
        }));
        let bump = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 2.35f64.to_radians());
        let mid = tr.scene.samples[tr.scene.samples.len() / 2].device_time_ns;
        let mut samples = tr.scene.samples.clone();
        for (i, s) in samples.iter_mut().enumerate() {
            if s.device_time_ns < mid {
                continue;
            }
            for v in s.views.iter_mut().filter(|v| v.camera == 0) {
                let truth = Isometry3::from_parts(Translation3::identity(), bump.inverse())
                    * tr.scene.world_from_cam[0].inverse()
                    * tr.scene.world_from_obj[i];
                reproject(&tr, v, &truth, s.device_id);
            }
        }
        let sol = solve_samples(
            &tr.scene.serials,
            &tr.scene.cameras,
            &tr.scene.leds,
            samples,
            &tr.imu,
            &SolveOptions::default(),
        )
        .unwrap();
        assert!(sol.sensors[0].rms_px < 0.5, "{}", sol.sensors[0].rms_px);
        let err = check(&sol, &Acceptance::default()).unwrap_err();
        assert!(
            err.contains("WMTD000000001") && err.contains("rejected"),
            "{err}"
        );
    }

    /// Re-renders a sighting from `cam_from_obj`: the engine's pose and every labelled LED move together.
    fn reproject(
        tr: &TrajectoryScene,
        v: &mut crate::calib::capture::View,
        cam_from_obj: &Isometry3<f64>,
        device: i32,
    ) {
        let leds = &tr.scene.leds[&device];
        for o in &mut v.obs {
            o.xy = tr.scene.cameras[v.camera]
                .project(&(cam_from_obj * Point3::from(leds[o.led].pos)).coords)
                .unwrap();
        }
        v.cam_from_obj = Some(*cam_from_obj);
    }

    /// Review fix 3: sightings with the right orientation but the wrong position (a depth or
    /// labelling error the gyro cannot see) are rejected by reprojection, not fitted.
    #[test]
    fn misplaced_sightings_are_rejected_by_reprojection() {
        let tr = synth::trajectory(&short(TrajectoryParams {
            pixel_noise: 0.0,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            ..Default::default()
        }));
        let mut samples = tr.scene.samples.clone();
        let mut corrupted = 0;
        for (i, s) in samples
            .iter_mut()
            .enumerate()
            .filter(|(i, s)| i % 15 == 0 && s.views.len() >= 2)
        {
            let v = &mut s.views[0];
            let truth = tr.scene.world_from_cam[v.camera].inverse() * tr.scene.world_from_obj[i];
            let moved = Isometry3::from_parts(
                Translation3::from(truth.translation.vector + Vector3::new(0.05, -0.03, 0.08)),
                truth.rotation,
            );
            reproject(&tr, v, &moved, s.device_id);
            corrupted += 1;
        }
        // one sample whose every sighting is garbage: its pose must not survive
        let g = samples.iter().position(|s| s.views.len() == 2).unwrap();
        for (k, v) in samples[g].views.iter_mut().enumerate() {
            for (j, o) in v.obs.iter_mut().enumerate() {
                o.xy.x += if (j + k) % 2 == 0 { 40.0 } else { -40.0 };
            }
        }
        let sol = solve_samples(
            &tr.scene.serials,
            &tr.scene.cameras,
            &tr.scene.leds,
            samples,
            &tr.imu,
            &SolveOptions::default(),
        )
        .unwrap();
        assert!(
            sol.rejected_views >= corrupted + 2,
            "{} rejected, {corrupted} + 2 corrupted",
            sol.rejected_views
        );
        assert_matches_truth(&tr, &sol, 0.01, 1e-4, 1e-3);
        check(&sol, &Acceptance::default()).unwrap();
    }

    /// Review Focus 2: wrong-but-confident sightings are rejected, not fitted.
    #[test]
    fn solve_rejects_mislabelled_views() {
        let p = short(TrajectoryParams {
            flipped_fraction: 0.15,
            pixel_noise: 0.0,
            ..Default::default()
        });
        let (tr, sol) = solve_trajectory(&p, &SolveOptions::default());
        let sol = sol.unwrap();
        assert!(sol.gyro_rejected_views > 0);
        assert_matches_truth(&tr, &sol, 0.1, 0.005, 1e-3);
    }

    /// Sightings without an engine pose, or without a confident match, never enter the solve.
    #[test]
    fn unposed_and_weak_sightings_are_dropped() {
        let tr = synth::trajectory(&short(TrajectoryParams {
            pixel_noise: 0.0,
            ..Default::default()
        }));
        let mut samples = tr.scene.samples.clone();
        let mut garbage = 0;
        for s in samples.iter_mut().step_by(5) {
            let mut v = s.views[0].clone();
            for o in &mut v.obs {
                o.led = (o.led + 5) % 24;
            }
            if garbage % 2 == 0 {
                v.cam_from_obj = None
            } else {
                v.strong = false
            }
            v.camera = (v.camera + 1) % 3;
            s.views.push(v);
            garbage += 1;
        }
        let sol = solve_samples(
            &tr.scene.serials,
            &tr.scene.cameras,
            &tr.scene.leds,
            samples,
            &tr.imu,
            &SolveOptions::default(),
        )
        .unwrap();
        assert_eq!(sol.unposed_views, garbage);
        assert_matches_truth(&tr, &sol, 0.1, 0.005, 1e-3);
    }

    /// Without IMU data the tilt is undetermined; say what to do.
    #[test]
    fn no_motion_data_is_an_error() {
        let tr = synth::trajectory(&short(TrajectoryParams::default()));
        let err = solve_samples(
            &tr.scene.serials,
            &tr.scene.cameras,
            &tr.scene.leds,
            tr.scene.samples.clone(),
            &BTreeMap::new(),
            &SolveOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("segments") && err.contains("wave"), "{err}");
    }

    /// Review Focus 1: a sensor without enough usable observations is named, not installed.
    #[test]
    fn sensor_without_enough_observations_is_an_error() {
        let (_, sol) = solve_trajectory(
            &short(TrajectoryParams::default()),
            &SolveOptions {
                min_observations: 1_000_000,
                ..Default::default()
            },
        );
        let err = sol.unwrap_err();
        assert!(
            err.contains("usable LED observations") && err.contains("WMTD"),
            "{err}"
        );
    }

    #[test]
    fn world_frame_conventions() {
        // A level, upright reference sensor: world up is image -y, and its optical axis maps to -Z.
        let w = world_from_ref(
            &Vector3::new(0.0, -1.0, 0.0),
            &[Vector3::new(0.0, 0.0, 2.0), Vector3::new(0.0, 0.0, 4.0)],
        );
        assert!((w.rotation * Vector3::new(0.0, -1.0, 0.0) - Vector3::y()).norm() < 1e-12);
        assert!((w.rotation * Vector3::z() + Vector3::z()).norm() < 1e-12);
        assert!((w * Point3::new(0.0, 0.0, 3.0)).coords.norm() < 1e-12); // centroid -> origin
    }

    fn solution(rms: [f64; 2], g: f64, own_deg: [f64; 2]) -> Solution {
        let sensor = |serial: &str, rms_px| SensorResult {
            serial: serial.into(),
            world_from_cam: Isometry3::identity(),
            rms_px,
            observations: 500,
            views: 400,
            rejected: 3,
        };
        let gravity = Vector3::new(0.0, g, 0.0);
        let device = |device| DeviceFit {
            device,
            segments: 80,
            accel_bias: Vector3::new(0.01, 0.47, -0.05),
            lever_arm: Vector3::new(0.0, -0.01, 0.005),
        };
        let tilted = |deg: f64| {
            UnitQuaternion::from_scaled_axis(Vector3::new(deg.to_radians(), 0.0, 0.0)) * gravity
        };
        Solution {
            sensors: vec![
                sensor("WMTD000000001", rms[0]),
                sensor("WMTD000000002", rms[1]),
            ],
            reference: "WMTD000000001".into(),
            gravity: {
                let devices = vec![
                    device(super::super::DEVICE_RIGHT),
                    device(super::super::DEVICE_LEFT),
                ];
                let independent = vec![
                    (
                        Subset::Device(super::super::DEVICE_RIGHT),
                        tilted(own_deg[0]),
                    ),
                    (
                        Subset::Device(super::super::DEVICE_LEFT),
                        tilted(own_deg[1]),
                    ),
                ];
                MotionGravity {
                    gravity,
                    segments: 160,
                    residual_median_m: 0.0038,
                    residual_p90_m: 0.02,
                    devices,
                    independent,
                }
            },
            samples: 100,
            unposed_views: 7,
            gyro_rejected_views: 3,
            rejected_views: 2,
            iterations: 12,
        }
    }

    #[test]
    fn acceptance_names_what_failed() {
        assert!(check(
            &solution([0.4, 0.5], 9.78, [0.1, 0.2]),
            &Acceptance::default()
        )
        .is_ok());
        let err = check(
            &solution([0.4, 1.5], 9.78, [0.1, 0.2]),
            &Acceptance::default(),
        )
        .unwrap_err();
        assert!(
            err.contains("WMTD000000002") && !err.contains("WMTD000000001 "),
            "{err}"
        );
        let err = check(
            &solution([0.4, 0.5], 9.1, [0.1, 0.2]),
            &Acceptance::default(),
        )
        .unwrap_err();
        assert!(err.contains("gravity magnitude"), "{err}");
        let err = check(
            &solution([0.4, 0.5], 9.78, [0.1, 0.9]),
            &Acceptance::default(),
        )
        .unwrap_err();
        assert!(err.contains("disagree"), "{err}");
    }

    #[test]
    fn report_lists_every_sensor_and_controller() {
        let r = format_report(&solution([0.4, 0.5], 9.78, [0.1, 0.2]));
        assert!(
            r.contains("WMTD000000001") && r.contains("WMTD000000002"),
            "{r}"
        );
        assert!(
            r.contains("9.780 m/s^2")
                && r.contains("right")
                && r.contains("left")
                && r.contains("+0.470"),
            "{r}"
        );
        assert!(r.contains("(right, left) within 0.10 deg"), "{r}");
    }

    /// End to end from a recording: frame observations, sensor info, LED models and both
    /// controllers' IMU streams through capture loading and the solve.
    #[test]
    fn solves_from_a_synthetic_recording() {
        use crate::calib::capture::from_records;
        use crate::calib::{iso_to_xyzw, DEVICE_LEFT};
        use crate::recorder::{
            self, as_bytes, led_model_prefix, slice_bytes, Blob, FrameInfo, LedEntry, Record,
            SensorInfo,
        };
        use crate::shm::{ControllerState, HAND_LEFT, HAND_RIGHT};
        use std::ffi::c_char;

        let tr = synth::trajectory(&short(TrajectoryParams {
            pixel_noise: 0.0,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            ..Default::default()
        }));
        let sc = &tr.scene;
        let cstr = |s: &str| {
            let mut f = [0 as c_char; 32];
            for (d, b) in f.iter_mut().zip(s.bytes()) {
                *d = b as c_char;
            }
            f
        };
        let mut recs = Vec::new();
        for (serial, cam) in sc.serials.iter().zip(&sc.cameras) {
            let info = SensorInfo {
                serial: cstr(serial),
                width: cam.width as i32,
                height: cam.height as i32,
                fx: cam.fx as f32,
                fy: cam.fy as f32,
                cx: cam.cx as f32,
                cy: cam.cy as f32,
                k: cam.k.map(|v| v as f32),
                ..Default::default()
            };
            recs.push(Record {
                rtype: recorder::SENSOR_INFO,
                host_ns: 1,
                payload: as_bytes(&info).to_vec(),
            });
        }
        for (device, model) in &sc.leds {
            let model: Vec<LedEntry> = model
                .iter()
                .map(|l| LedEntry {
                    pos: l.pos.map(|v| v as f32).into(),
                    dir: l.dir.map(|v| v as f32).into(),
                })
                .collect();
            recs.push(Record {
                rtype: recorder::LED_MODEL,
                host_ns: 1,
                payload: [
                    &led_model_prefix(*device, model.len())[..],
                    slice_bytes(&model),
                ]
                .concat(),
            });
        }
        // one frame per sensor per exposure, carrying every controller it sighted
        let mut frames: BTreeMap<(u64, usize), (FrameInfo, Vec<Blob>)> = BTreeMap::new();
        for smp in &sc.samples {
            for v in &smp.views {
                let exposure =
                    ((smp.device_time_ns - 1_000_000_000) as f64 * 52.0 / 1e9).round() as u32;
                let (info, blobs) =
                    frames
                        .entry((smp.device_time_ns, v.camera))
                        .or_insert_with(|| {
                            let info = FrameInfo {
                                serial: cstr(&sc.serials[v.camera]),
                                frame_start_host_ns: smp.device_time_ns,
                                exposure_valid: 1,
                                exposure_count: exposure % 65536,
                                ..Default::default()
                            };
                            (info, Vec::new())
                        });
                let d = &mut info.devices[info.n_devices as usize];
                info.n_devices += 1;
                let (q, p) = iso_to_xyzw(&v.cam_from_obj.unwrap());
                d.device_id = smp.device_id;
                d.device_time_ns = smp.device_time_ns;
                d.match_flags = 0x3;
                d.pose_valid = 1;
                d.cam_orient = q;
                d.cam_pos = p;
                blobs.extend(v.obs.iter().map(|o| Blob {
                    x: o.xy.x as f32,
                    y: o.xy.y as f32,
                    width: 3,
                    height: 3,
                    area: 9,
                    led_id: (smp.device_id << 8) | o.led as i32,
                }));
            }
        }
        for ((t, _), (info, blobs)) in &frames {
            recs.push(Record {
                rtype: recorder::FRAME_OBS,
                host_ns: *t,
                payload: [as_bytes(info), slice_bytes(blobs)].concat(),
            });
        }
        for (device, stream) in &tr.imu {
            let hand = if *device == DEVICE_LEFT {
                HAND_LEFT
            } else {
                HAND_RIGHT
            };
            for s in stream {
                let st = ControllerState {
                    device_time_ns: s.device_time_ns,
                    imu_accel: s.accel.map(|v| v as f32).into(),
                    imu_gyro: s.gyro.map(|v| v as f32).into(),
                    ..Default::default()
                };
                recs.push(Record {
                    rtype: recorder::CONTROLLER_STATE,
                    host_ns: s.device_time_ns,
                    payload: [&(hand as u32).to_le_bytes()[..], &[0u8; 4], as_bytes(&st)].concat(),
                });
            }
        }
        let cap = from_records(recs).unwrap();
        assert_eq!(cap.samples.len(), sc.samples.len());
        let sol = solve_capture(&cap, &SolveOptions::default()).unwrap();
        // f32 storage of the IMU and poses limits the precision slightly
        assert_matches_truth(&tr, &sol, 0.05, 1e-3, 1e-3);
        check(&sol, &Acceptance::default()).unwrap();
    }

    /// Real-scale timing (not run by default): `cargo test --release --lib real_scale -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn real_scale_benchmark() {
        let p = TrajectoryParams {
            duration_s: 90.0,
            flipped_fraction: 0.3,
            ..Default::default()
        };
        let t0 = std::time::Instant::now();
        let (tr, sol) = solve_trajectory(&p, &SolveOptions::default());
        let sol = sol.unwrap();
        eprintln!(
            "{} samples: {:.1} s, {} iterations, {} gyro-rejected, {} rejected",
            tr.scene.samples.len(),
            t0.elapsed().as_secs_f64(),
            sol.iterations,
            sol.gyro_rejected_views,
            sol.rejected_views
        );
        eprint!("{}", format_report(&sol));
        // one smooth trajectory constrains the geometry less than independent random poses:
        // ~1.6 mm measured at 0.3 px (statistical: the noise-free test is exact)
        assert_matches_truth(&tr, &sol, 2.5, 0.02, 2e-3);
    }
}
