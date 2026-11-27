// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Joint multi-camera sensor refit anchored to the prior calibration by rigid SE(3) alignment and
//! gated on chronological held-out reprojection RMS. Works from tracker-associated multi-camera
//! observations (no engine labels required) for any connected rig of two or more sensors.

use std::collections::{BTreeMap, BTreeSet};

use nalgebra::{Isometry3, Matrix2, Matrix3, Point3, UnitQuaternion, Vector2, Vector3};

use super::ba::{self, Problem, State};
use super::camera::CameraModel;
use super::capture::{Led, WandSample};

/// Orientation weight in the SE(3) rigid alignment cross-covariance (m^2, matching a 0.5 m arm).
/// Keeps camera centers exact at the centroid while resolving the roll degree of freedom of a
/// two-sensor rig.
const ALIGN_ROT_WEIGHT: f64 = 0.25;
const BA_MAX_ROUNDS: usize = 5;

/// Bounds for the adaptive per-view RMS threshold (px) used for symmetric corrupt-view skipping
/// during held-out comparison: a view worse than `limit` under both prior and candidate poses is
/// a corrupt frame (not sensor motion, which fits one of the two).
const HELDOUT_CORRUPT_MIN_PX: f64 = 1.0;
const HELDOUT_CORRUPT_MAX_PX: f64 = 4.0;
const HELDOUT_CORRUPT_MEDIAN_FACTOR: f64 = 6.0;

/// Leave-one-out outlier anchor thresholds: a single sensor excluded from the SE(3) gauge only
/// when the remaining sensors agree tightly and that one sensor stands out sharply.
const OUTLIER_INLIER_M: f64 = 0.005;
const OUTLIER_INLIER_RAD: f64 = 0.30 * std::f64::consts::PI / 180.0;
const OUTLIER_MIN_M: f64 = 0.010;
const OUTLIER_MIN_RAD: f64 = 0.25 * std::f64::consts::PI / 180.0;
const OUTLIER_SCORE_RATIO: f64 = 3.0;
const OUTLIER_IMPROVE_FACTOR: f64 = 0.45;

/// Pixel-evidence floor required before a pose-outlier sensor is classified as physically bumped
/// (so weakly observable depth drift never overwrites the saved calibration).
const BUMP_MIN_BEFORE_MEDIAN_PX: f64 = 0.40;
const BUMP_MIN_RMS_DROP_PX: f64 = 0.20;
const BUMP_MAX_AFTER_RATIO: f64 = 0.60;

/// One chronologically ordered multi-camera exposure with an initial controller pose seed
/// (`world_from_model`).
#[derive(Debug, Clone, PartialEq)]
pub struct RefitSample {
    pub sample: WandSample,
    pub seed: Isometry3<f64>,
}

/// Rigid SE(3) alignment of fitted sensor poses back onto the prior calibration frame.
#[derive(Debug, Clone, PartialEq)]
pub struct AlignResult {
    /// Rigid transform mapping the fitted BA frame into the prior world frame
    /// (`world_prior <- world_ba`).
    pub world_from_ba: Isometry3<f64>,
    /// Aligned `world_from_cam` poses, in input order.
    pub poses: Vec<Isometry3<f64>>,
    /// Per-sensor translation displacement from prior (metres).
    pub delta_m: Vec<f64>,
    /// Per-sensor rotation displacement from prior (radians).
    pub delta_rad: Vec<f64>,
    /// Index of a single sensor excluded from the gauge anchor because it moved clearly relative
    /// to the remaining sensors (only possible with >= 3 sensors).
    pub outlier: Option<usize>,
}

/// Fixed-camera, fresh-pose reprojection accuracy over multi-camera samples.
#[derive(Debug, Clone, PartialEq)]
pub struct HeldoutStats {
    pub pooled_rms_px: f64,
    pub total_obs: usize,
    pub samples: usize,
    pub camera_rms_px: Vec<f64>,
    pub camera_median_px: Vec<f64>,
    pub camera_obs: Vec<usize>,
}

#[derive(Debug, Clone)]
pub struct RefitOptions {
    /// Fraction of chronological samples used for fitting; the trailing remainder is held out.
    pub train_fraction: f64,
    /// Cap on training samples passed to BA (uniformly subsampled across the training span).
    pub max_train_samples: usize,
    /// Cap on held-out samples evaluated by the gate.
    pub max_heldout_samples: usize,
    /// Minimum multi-camera samples required in the training and held-out splits.
    pub min_train_samples: usize,
    pub min_heldout_samples: usize,
    /// Minimum LED observations per participating sensor in the training and held-out splits.
    pub min_camera_obs: usize,
    pub min_heldout_camera_obs: usize,
    /// Minimum multi-camera exposures per participating sensor in the training and held-out splits.
    pub min_camera_views: usize,
    pub min_heldout_camera_views: usize,
    /// Minimum co-observed multi-camera training samples required to link two sensors.
    pub min_pair_links: usize,
    /// Minimum controller motion variety across training samples (metres, metres, radians).
    pub min_spread_m: f64,
    pub min_side_m: f64,
    /// Minimum smallest 3D principal standard deviation of training controller positions (metres,
    /// rotation-invariant), preventing near-planar arcs from overfitting depth vs tilt.
    pub min_depth_std_m: f64,
    pub min_turn_rad: f64,
    /// Minimum 3D motion spread (2 * RMS radius, metres) in the chronological held-out split so
    /// validation does not test only stationary points.
    pub min_heldout_spread_m: f64,
    /// Minimum per-sensor 2D image-plane principal standard deviations (major, minor pixels) on
    /// the training split; sensors seen only in a narrow corner patch are skipped instead of
    /// fitting on an ill-conditioned depth/tilt ridge.
    pub min_camera_img_major_px: f64,
    pub min_camera_img_minor_px: f64,
    /// Optional minimum chronological duration spanned by the multi-sensor samples (nanoseconds;
    /// 0 disables the check for offline/short captures).
    pub min_span_ns: u64,
    /// Held-out acceptance thresholds: pooled RMS must improve by at least both `min_gain_px`
    /// and `min_rel_gain`, no sensor may regress beyond `max_cam_regress_px`, and final pooled
    /// RMS must stay within `max_after_rms_px`.
    pub min_gain_px: f64,
    pub min_rel_gain: f64,
    pub max_cam_regress_px: f64,
    pub max_after_rms_px: f64,
    /// Fail-closed upper sanity limits on any single sensor's shift from its anchor pose
    /// (metres, radians). Larger displacements belong to cold-search recovery, not a
    /// tracker-associated joint refit.
    pub max_sensor_delta_m: f64,
    pub max_sensor_delta_rad: f64,
    pub ba: ba::Options,
}

impl Default for RefitOptions {
    fn default() -> Self {
        RefitOptions {
            train_fraction: 0.70,
            max_train_samples: 600,
            max_heldout_samples: 300,
            min_train_samples: 60,
            min_heldout_samples: 25,
            min_camera_obs: 150,
            min_heldout_camera_obs: 30,
            min_camera_views: 20,
            min_heldout_camera_views: 8,
            min_pair_links: 10,
            min_spread_m: 0.30,
            min_side_m: 0.10,
            min_depth_std_m: 0.04,
            min_turn_rad: 15.0 * std::f64::consts::PI / 180.0,
            min_heldout_spread_m: 0.15,
            min_camera_img_major_px: 40.0,
            min_camera_img_minor_px: 15.0,
            min_span_ns: 0,
            min_gain_px: 0.015,
            min_rel_gain: 0.05,
            max_cam_regress_px: 0.020,
            max_after_rms_px: 0.50,
            max_sensor_delta_m: 0.10,
            max_sensor_delta_rad: 5.0 * std::f64::consts::PI / 180.0,
            ba: ba::Options {
                sigma_px: 0.5,
                huber_px: 2.0,
                max_iterations: 40,
            },
        }
    }
}

/// Result for one sensor participating in a joint refit.
#[derive(Debug, Clone, PartialEq)]
pub struct RefitSensor {
    /// Index into the caller's input `serials` / `cameras` / `prior` slices.
    pub camera: usize,
    pub serial: String,
    pub world_from_cam: Isometry3<f64>,
    pub delta_m: f64,
    pub delta_rad: f64,
    pub before_rms_px: f64,
    pub after_rms_px: f64,
    pub before_median_px: f64,
    pub after_median_px: f64,
    pub train_obs: usize,
    pub heldout_obs: usize,
    /// True if this sensor was excluded from the SE(3) gauge anchor as a single moved outlier.
    pub outlier: bool,
}

/// Outcome of a joint multi-camera refit and its chronological held-out validation gate.
/// `before` and `after` are measured on the exact same held-out observations against the
/// validated poses returned in `sensors`.
#[derive(Debug, Clone, PartialEq)]
pub struct RefitResult {
    pub accepted: bool,
    pub reject_reason: Option<String>,
    pub sensors: Vec<RefitSensor>,
    /// Sensors omitted from the fit for insufficient overlap, observations, or held-out coverage.
    pub skipped_serials: Vec<String>,
    pub before: HeldoutStats,
    pub after: HeldoutStats,
    pub train_samples: usize,
    pub heldout_samples: usize,
    pub train_rejected_views: usize,
    pub heldout_dropped_views: usize,
    pub outlier_serial: Option<String>,
}

impl RefitResult {
    /// Returns the single sensor that stood out as a pose outlier relative to its peers and also
    /// showed clear held-out pixel evidence of having moved before the refit.
    pub fn bumped_sensor(&self) -> Option<&RefitSensor> {
        if !self.accepted {
            return None;
        }
        let sensor = self.sensors.iter().find(|s| s.outlier)?;
        let drop = sensor.before_rms_px - sensor.after_rms_px;
        (sensor.before_median_px >= BUMP_MIN_BEFORE_MEDIAN_PX
            && drop >= BUMP_MIN_RMS_DROP_PX
            && sensor.after_rms_px <= BUMP_MAX_AFTER_RATIO * sensor.before_rms_px)
            .then_some(sensor)
    }
}

/// Optimal rigid SE(3) transform (`prior <- fitted`) aligning camera centers and orientations.
///
/// Minimises `sum_i ||R p_i + t - q_i||^2 + ALIGN_ROT_WEIGHT * ||R R_i - Q_i||_F^2`, giving the
/// exact camera-center centroid for `t` while keeping rotation well-conditioned even for 1 or 2
/// sensors (where camera centers alone are degenerate or collinear).
pub fn align_se3(fitted: &[Isometry3<f64>], prior: &[Isometry3<f64>]) -> Option<Isometry3<f64>> {
    if fitted.is_empty() || fitted.len() != prior.len() {
        return None;
    }
    let n = fitted.len() as f64;
    let p_bar: Vector3<f64> = fitted
        .iter()
        .map(|x| x.translation.vector)
        .sum::<Vector3<f64>>()
        / n;
    let q_bar: Vector3<f64> = prior
        .iter()
        .map(|x| x.translation.vector)
        .sum::<Vector3<f64>>()
        / n;
    let mut h = Matrix3::<f64>::zeros();
    for (a, b) in fitted.iter().zip(prior) {
        let dp = a.translation.vector - p_bar;
        let dq = b.translation.vector - q_bar;
        h += dq * dp.transpose();
        let ra = a.rotation.to_rotation_matrix().into_inner();
        let rb = b.rotation.to_rotation_matrix().into_inner();
        h += ALIGN_ROT_WEIGHT * (rb * ra.transpose());
    }
    if !h.iter().all(|v| v.is_finite()) {
        return None;
    }
    let svd = h.svd(true, true);
    let (u, vt) = (svd.u?, svd.v_t?);
    let d = if (u * vt).determinant() < 0.0 {
        -1.0
    } else {
        1.0
    };
    let r = u * Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, d)) * vt;
    let t = q_bar - r * p_bar;
    if !r.iter().all(|v| v.is_finite()) || !t.iter().all(|v| v.is_finite()) {
        return None;
    }
    Some(Isometry3::from_parts(
        t.into(),
        UnitQuaternion::from_matrix(&r),
    ))
}

fn pose_deltas(
    world_from_ba: &Isometry3<f64>,
    fitted: &[Isometry3<f64>],
    prior: &[Isometry3<f64>],
) -> (Vec<Isometry3<f64>>, Vec<f64>, Vec<f64>) {
    let mut poses = Vec::with_capacity(fitted.len());
    let mut delta_m = Vec::with_capacity(fitted.len());
    let mut delta_rad = Vec::with_capacity(fitted.len());
    for (f, p) in fitted.iter().zip(prior) {
        let aligned = world_from_ba * f;
        let d = p.inverse() * aligned;
        poses.push(aligned);
        delta_m.push(d.translation.vector.norm());
        delta_rad.push(d.rotation.angle());
    }
    (poses, delta_m, delta_rad)
}

fn normalized_score(dm: f64, dr: f64) -> f64 {
    (dm / OUTLIER_INLIER_M).max(dr / OUTLIER_INLIER_RAD)
}

/// Aligns `fitted` sensor poses to `prior` by rigid SE(3) Procrustes. When three or more sensors
/// are present and one sensor moved clearly against a tightly consistent remainder, anchors the
/// world frame to the unmoved remainder and reports the moved sensor in `outlier`.
pub fn align_to_prior(fitted: &[Isometry3<f64>], prior: &[Isometry3<f64>]) -> Option<AlignResult> {
    let t_all = align_se3(fitted, prior)?;
    let (poses_all, dm_all, dr_all) = pose_deltas(&t_all, fitted, prior);
    let worst_all = dm_all
        .iter()
        .zip(&dr_all)
        .map(|(&m, &r)| normalized_score(m, r))
        .fold(0.0_f64, f64::max);

    if fitted.len() >= 3 {
        let mut best: Option<(usize, f64, AlignResult)> = None;
        for skip in 0..fitted.len() {
            let sub_f: Vec<Isometry3<f64>> = fitted
                .iter()
                .enumerate()
                .filter(|&(i, _)| i != skip)
                .map(|(_, &p)| p)
                .collect();
            let sub_p: Vec<Isometry3<f64>> = prior
                .iter()
                .enumerate()
                .filter(|&(i, _)| i != skip)
                .map(|(_, &p)| p)
                .collect();
            let Some(t_sub) = align_se3(&sub_f, &sub_p) else {
                continue;
            };
            let (poses, dm, dr) = pose_deltas(&t_sub, fitted, prior);
            let mut worst_inlier_m = 0.0_f64;
            let mut worst_inlier_rad = 0.0_f64;
            let mut worst_inlier_score = 0.0_f64;
            for i in 0..fitted.len() {
                if i == skip {
                    continue;
                }
                worst_inlier_m = worst_inlier_m.max(dm[i]);
                worst_inlier_rad = worst_inlier_rad.max(dr[i]);
                worst_inlier_score = worst_inlier_score.max(normalized_score(dm[i], dr[i]));
            }
            let out_m = dm[skip];
            let out_r = dr[skip];
            let out_score = normalized_score(out_m, out_r);
            let valid_outlier = worst_inlier_m <= OUTLIER_INLIER_M
                && worst_inlier_rad <= OUTLIER_INLIER_RAD
                && (out_m >= OUTLIER_MIN_M || out_r >= OUTLIER_MIN_RAD)
                && out_score >= OUTLIER_SCORE_RATIO * worst_inlier_score.max(0.2)
                && worst_inlier_score <= OUTLIER_IMPROVE_FACTOR * worst_all;
            if valid_outlier
                && best
                    .as_ref()
                    .is_none_or(|(_, prev, _)| worst_inlier_score < *prev)
            {
                best = Some((
                    skip,
                    worst_inlier_score,
                    AlignResult {
                        world_from_ba: t_sub,
                        poses,
                        delta_m: dm,
                        delta_rad: dr,
                        outlier: Some(skip),
                    },
                ));
            }
        }
        if let Some((_, _, res)) = best {
            return Some(res);
        }
    }

    Some(AlignResult {
        world_from_ba: t_all,
        poses: poses_all,
        delta_m: dm_all,
        delta_rad: dr_all,
        outlier: None,
    })
}

fn finish_heldout_stats(
    n: usize,
    sum_sq: Vec<f64>,
    errs: Vec<Vec<f64>>,
    total_sq: f64,
    total_obs: usize,
    active_samples: usize,
) -> HeldoutStats {
    let camera_obs: Vec<usize> = errs.iter().map(Vec::len).collect();
    let camera_rms_px: Vec<f64> = (0..n)
        .map(|c| {
            if camera_obs[c] > 0 {
                (sum_sq[c] / camera_obs[c] as f64).sqrt()
            } else {
                f64::NAN
            }
        })
        .collect();
    let camera_median_px: Vec<f64> = errs
        .into_iter()
        .map(|mut v| {
            if v.is_empty() {
                f64::NAN
            } else {
                let mid = v.len() / 2;
                *v.select_nth_unstable_by(mid, f64::total_cmp).1
            }
        })
        .collect();
    let pooled_rms_px = if total_obs > 0 {
        (total_sq / total_obs as f64).sqrt()
    } else {
        f64::NAN
    };
    HeldoutStats {
        pooled_rms_px,
        total_obs,
        samples: active_samples,
        camera_rms_px,
        camera_median_px,
        camera_obs,
    }
}

fn heldout_stats_from_state(p: &Problem, s: &State) -> HeldoutStats {
    let n = p.cameras.len();
    let mut sum_sq = vec![0.0_f64; n];
    let mut errs = vec![Vec::<f64>::new(); n];
    let mut total_sq = 0.0_f64;
    let mut total_obs = 0usize;
    let mut active_samples = 0usize;
    for (i, sample) in p.samples.iter().enumerate() {
        let (Some(w), Some(model)) = (s.ref_from_obj[i], p.leds.get(&sample.device_id)) else {
            continue;
        };
        if sample.views.len() < 2 {
            continue;
        }
        active_samples += 1;
        for v in &sample.views {
            let (cam, t_cam) = (&p.cameras[v.camera], &s.ref_from_cam[v.camera]);
            for ob in &v.obs {
                let Some(led) = model.get(ob.led) else {
                    continue;
                };
                let p_ref = w * Point3::from(led.pos);
                let Some(uv) = cam.project(&t_cam.inverse_transform_point(&p_ref).coords) else {
                    continue;
                };
                let e2 = (uv - ob.xy).norm_squared();
                sum_sq[v.camera] += e2;
                errs[v.camera].push(e2.sqrt());
                total_sq += e2;
                total_obs += 1;
            }
        }
    }
    finish_heldout_stats(n, sum_sq, errs, total_sq, total_obs, active_samples)
}

/// Scores `s_prior` and `s_cand` on the exact same intersection of validly projected observations,
/// and counts every observation that `s_prior` projected but `s_cand` failed to project (so a
/// candidate can never improve its RMS by dropping failed projections).
fn heldout_stats_pair(
    p: &Problem,
    s_prior: &State,
    s_cand: &State,
) -> (HeldoutStats, HeldoutStats, usize) {
    let n = p.cameras.len();
    let mut p_sum_sq = vec![0.0_f64; n];
    let mut c_sum_sq = vec![0.0_f64; n];
    let mut p_errs = vec![Vec::<f64>::new(); n];
    let mut c_errs = vec![Vec::<f64>::new(); n];
    let mut p_total_sq = 0.0_f64;
    let mut c_total_sq = 0.0_f64;
    let mut total_obs = 0usize;
    let mut active_samples = 0usize;
    let mut cand_lost_obs = 0usize;

    for (i, sample) in p.samples.iter().enumerate() {
        let Some(model) = p.leds.get(&sample.device_id) else {
            continue;
        };
        if sample.views.len() < 2 {
            continue;
        }
        let (Some(wp), cand_w) = (s_prior.ref_from_obj[i], s_cand.ref_from_obj[i]) else {
            continue;
        };
        let Some(wc) = cand_w else {
            for v in &sample.views {
                let (cam, tp_cam) = (&p.cameras[v.camera], &s_prior.ref_from_cam[v.camera]);
                for ob in &v.obs {
                    if let Some(led) = model.get(ob.led) {
                        let p_ref = wp * Point3::from(led.pos);
                        if cam
                            .project(&tp_cam.inverse_transform_point(&p_ref).coords)
                            .is_some()
                        {
                            cand_lost_obs += 1;
                        }
                    }
                }
            }
            continue;
        };
        let mut sample_obs = 0usize;
        for v in &sample.views {
            let cam = &p.cameras[v.camera];
            let tp_cam = &s_prior.ref_from_cam[v.camera];
            let tc_cam = &s_cand.ref_from_cam[v.camera];
            for ob in &v.obs {
                let Some(led) = model.get(ob.led) else {
                    continue;
                };
                let uv_p = cam.project(
                    &tp_cam
                        .inverse_transform_point(&(wp * Point3::from(led.pos)))
                        .coords,
                );
                let uv_c = cam.project(
                    &tc_cam
                        .inverse_transform_point(&(wc * Point3::from(led.pos)))
                        .coords,
                );
                match (uv_p, uv_c) {
                    (Some(up), Some(uc)) => {
                        let ep2 = (up - ob.xy).norm_squared();
                        let ec2 = (uc - ob.xy).norm_squared();
                        p_sum_sq[v.camera] += ep2;
                        c_sum_sq[v.camera] += ec2;
                        p_errs[v.camera].push(ep2.sqrt());
                        c_errs[v.camera].push(ec2.sqrt());
                        p_total_sq += ep2;
                        c_total_sq += ec2;
                        total_obs += 1;
                        sample_obs += 1;
                    }
                    (Some(_), None) => {
                        cand_lost_obs += 1;
                    }
                    (None, _) => {}
                }
            }
        }
        if sample_obs > 0 {
            active_samples += 1;
        }
    }
    (
        finish_heldout_stats(n, p_sum_sq, p_errs, p_total_sq, total_obs, active_samples),
        finish_heldout_stats(n, c_sum_sq, c_errs, c_total_sq, total_obs, active_samples),
        cand_lost_obs,
    )
}

/// Evaluates fixed camera poses against multi-camera exposures by fitting a fresh 6-DoF
/// controller pose per exposure.
pub fn eval_heldout(
    cameras: &[CameraModel],
    leds: &BTreeMap<i32, Vec<Led>>,
    poses: &[Isometry3<f64>],
    samples: &[RefitSample],
    ba_opts: &ba::Options,
) -> HeldoutStats {
    let wand: Vec<WandSample> = samples.iter().map(|s| s.sample.clone()).collect();
    let p = Problem {
        cameras,
        leds,
        samples: &wand,
        reference: 0,
    };
    let mut state = State {
        ref_from_cam: poses.to_vec(),
        ref_from_obj: samples.iter().map(|s| Some(s.seed)).collect(),
    };
    ba::solve_poses(&p, &mut state, ba_opts);
    heldout_stats_from_state(&p, &state)
}

/// Evaluates `prior` and `candidate` on the same held-out exposures, symmetrically skipping any
/// view that exceeds an adaptive outlier threshold (`HELDOUT_CORRUPT_MEDIAN_FACTOR` times the
/// lower median, clamped to `HELDOUT_CORRUPT_MIN_PX..=HELDOUT_CORRUPT_MAX_PX`) under both poses
/// (corrupt frame, not motion).
fn compare_heldout(
    cameras: &[CameraModel],
    leds: &BTreeMap<i32, Vec<Led>>,
    prior: &[Isometry3<f64>],
    candidate: &[Isometry3<f64>],
    samples: &[RefitSample],
    ba_opts: &ba::Options,
) -> (HeldoutStats, HeldoutStats, usize, usize) {
    let wand: Vec<WandSample> = samples.iter().map(|s| s.sample.clone()).collect();
    let p = Problem {
        cameras,
        leds,
        samples: &wand,
        reference: 0,
    };
    let mut s_prior = State {
        ref_from_cam: prior.to_vec(),
        ref_from_obj: samples.iter().map(|s| Some(s.seed)).collect(),
    };
    let mut s_cand = State {
        ref_from_cam: candidate.to_vec(),
        ref_from_obj: samples.iter().map(|s| Some(s.seed)).collect(),
    };
    ba::solve_poses(&p, &mut s_prior, ba_opts);
    ba::solve_poses(&p, &mut s_cand, ba_opts);

    let med_p = median_view_rms(&p, &s_prior);
    let med_c = median_view_rms(&p, &s_cand);
    let limit = (HELDOUT_CORRUPT_MEDIAN_FACTOR * med_p.min(med_c))
        .clamp(HELDOUT_CORRUPT_MIN_PX, HELDOUT_CORRUPT_MAX_PX);

    let mut dropped = 0usize;
    let mut cleaned: Vec<RefitSample> = Vec::with_capacity(samples.len());
    for (i, rs) in samples.iter().enumerate() {
        let mut views = Vec::with_capacity(rs.sample.views.len());
        for (vi, v) in rs.sample.views.iter().enumerate() {
            let r_p = ba::view_rms_px(&p, &s_prior, i, vi).unwrap_or(f64::INFINITY);
            let r_c = ba::view_rms_px(&p, &s_cand, i, vi).unwrap_or(f64::INFINITY);
            if r_p > limit && r_c > limit {
                dropped += 1;
            } else {
                views.push(v.clone());
            }
        }
        if views.len() >= 2 {
            let mut s = rs.sample.clone();
            s.views = views;
            cleaned.push(RefitSample {
                sample: s,
                seed: rs.seed,
            });
        }
    }
    if dropped == 0 {
        let (before, after, cand_lost_obs) = heldout_stats_pair(&p, &s_prior, &s_cand);
        return (before, after, 0, cand_lost_obs);
    }
    let clean_wand: Vec<WandSample> = cleaned.iter().map(|s| s.sample.clone()).collect();
    let p_clean = Problem {
        cameras,
        leds,
        samples: &clean_wand,
        reference: 0,
    };
    let mut sp_clean = State {
        ref_from_cam: prior.to_vec(),
        ref_from_obj: cleaned.iter().map(|s| Some(s.seed)).collect(),
    };
    let mut sc_clean = State {
        ref_from_cam: candidate.to_vec(),
        ref_from_obj: cleaned.iter().map(|s| Some(s.seed)).collect(),
    };
    ba::solve_poses(&p_clean, &mut sp_clean, ba_opts);
    ba::solve_poses(&p_clean, &mut sc_clean, ba_opts);
    let (before, after, cand_lost_obs) = heldout_stats_pair(&p_clean, &sp_clean, &sc_clean);
    (before, after, dropped, cand_lost_obs)
}

fn pca_stds_3d(samples: &[RefitSample]) -> [f64; 3] {
    if samples.len() < 3 {
        return [0.0; 3];
    }
    let n = samples.len() as f64;
    let mean: Vector3<f64> = samples
        .iter()
        .map(|s| s.seed.translation.vector)
        .sum::<Vector3<f64>>()
        / n;
    let mut cov = Matrix3::<f64>::zeros();
    for s in samples {
        let d = s.seed.translation.vector - mean;
        cov += d * d.transpose();
    }
    cov /= n;
    let eig = cov.symmetric_eigenvalues();
    let mut stds = [
        eig.x.max(0.0).sqrt(),
        eig.y.max(0.0).sqrt(),
        eig.z.max(0.0).sqrt(),
    ];
    stds.sort_by(|a, b| b.total_cmp(a));
    stds
}

fn camera_img_pca_stds(samples: &[RefitSample], n_cams: usize) -> Vec<[f64; 2]> {
    let mut pts: Vec<Vec<Vector2<f64>>> = vec![Vec::new(); n_cams];
    for s in samples {
        for v in &s.sample.views {
            if v.camera < n_cams {
                for ob in &v.obs {
                    pts[v.camera].push(ob.xy);
                }
            }
        }
    }
    pts.into_iter()
        .map(|ps| {
            if ps.len() < 3 {
                return [0.0, 0.0];
            }
            let n = ps.len() as f64;
            let mean: Vector2<f64> = ps.iter().copied().sum::<Vector2<f64>>() / n;
            let mut cov = Matrix2::<f64>::zeros();
            for p in &ps {
                let d = p - mean;
                cov += d * d.transpose();
            }
            cov /= n;
            let eig = cov.symmetric_eigenvalues();
            let a = eig.x.max(0.0).sqrt();
            let b = eig.y.max(0.0).sqrt();
            if a >= b {
                [a, b]
            } else {
                [b, a]
            }
        })
        .collect()
}

fn varied(samples: &[RefitSample], opts: &RefitOptions) -> bool {
    let Some(first) = samples.first() else {
        return false;
    };
    let mut lo = Vector3::repeat(f64::INFINITY);
    let mut hi = Vector3::repeat(f64::NEG_INFINITY);
    let mut turned = 0.0_f64;
    for s in samples {
        let t = s.seed.translation.vector;
        lo = lo.inf(&t);
        hi = hi.sup(&t);
        turned = turned.max(first.seed.rotation.angle_to(&s.seed.rotation));
    }
    let mut sides = [hi.x - lo.x, hi.y - lo.y, hi.z - lo.z];
    sides.sort_by(|a, b| b.total_cmp(a));
    let pca = pca_stds_3d(samples);
    (hi - lo).norm() >= opts.min_spread_m
        && sides[1] >= opts.min_side_m
        && pca[2] >= opts.min_depth_std_m
        && turned >= opts.min_turn_rad
}

fn heldout_varied(samples: &[RefitSample], opts: &RefitOptions) -> bool {
    if opts.min_heldout_spread_m <= 0.0 {
        return true;
    }
    let pca = pca_stds_3d(samples);
    let spread = 2.0 * (pca[0] * pca[0] + pca[1] * pca[1] + pca[2] * pca[2]).sqrt();
    spread >= opts.min_heldout_spread_m
}

/// Largest connected component of cameras in `active` linked by at least `min_links` co-observed
/// multi-camera samples.
fn largest_component(
    n_cams: usize,
    active: &[bool],
    samples: &[RefitSample],
    min_links: usize,
) -> Vec<usize> {
    let mut links = vec![vec![0usize; n_cams]; n_cams];
    for s in samples {
        let cams: Vec<usize> = s
            .sample
            .views
            .iter()
            .map(|v| v.camera)
            .filter(|&c| c < n_cams && active[c])
            .collect();
        for i in 0..cams.len() {
            for j in (i + 1)..cams.len() {
                let (a, b) = (cams[i], cams[j]);
                if a != b {
                    links[a][b] += 1;
                    links[b][a] += 1;
                }
            }
        }
    }
    let mut visited = vec![false; n_cams];
    let mut best = Vec::new();
    for start in 0..n_cams {
        if !active[start] || visited[start] {
            continue;
        }
        let mut comp = Vec::new();
        let mut stack = vec![start];
        visited[start] = true;
        while let Some(u) = stack.pop() {
            comp.push(u);
            for v in 0..n_cams {
                if active[v] && !visited[v] && links[u][v] >= min_links {
                    visited[v] = true;
                    stack.push(v);
                }
            }
        }
        comp.sort_unstable();
        if comp.len() > best.len() {
            best = comp;
        }
    }
    best
}

fn filter_remap(samples: &[RefitSample], map: &[Option<usize>]) -> Vec<RefitSample> {
    let mut out = Vec::new();
    for rs in samples {
        let mut seen = BTreeSet::new();
        let mut views = Vec::new();
        for v in &rs.sample.views {
            let Some(Some(nc)) = map.get(v.camera) else {
                continue;
            };
            if !seen.insert(*nc) {
                continue;
            }
            let mut v = v.clone();
            v.camera = *nc;
            views.push(v);
        }
        if views.len() >= 2 {
            let mut s = rs.sample.clone();
            s.views = views;
            out.push(RefitSample {
                sample: s,
                seed: rs.seed,
            });
        }
    }
    out
}

fn count_obs_and_views(samples: &[RefitSample], n_cams: usize) -> (Vec<usize>, Vec<usize>) {
    let mut obs = vec![0usize; n_cams];
    let mut views = vec![0usize; n_cams];
    for s in samples {
        for v in &s.sample.views {
            if v.camera < n_cams {
                obs[v.camera] += v.obs.len();
                views[v.camera] += 1;
            }
        }
    }
    (obs, views)
}

/// Uniformly subsamples `samples` to at most `cap` while preserving chronological order and
/// ensuring rarer cameras keep representation.
fn subsample(
    samples: &[RefitSample],
    n_cams: usize,
    cap: usize,
    min_obs: usize,
) -> Vec<RefitSample> {
    if cap == 0 || samples.len() <= cap {
        return samples.to_vec();
    }
    let mut keep = vec![false; samples.len()];
    let mut kept = 0usize;
    // Uniformly spaced indices across [0, samples.len()).
    for k in 0..cap {
        let idx = k * samples.len() / cap;
        if !keep[idx] {
            keep[idx] = true;
            kept += 1;
        }
    }
    // Top up any camera that dropped below min_obs due to striding, replacing nothing past 1.25*cap.
    let hard_cap = cap + cap / 4;
    let mut cam_obs = vec![0usize; n_cams];
    for (i, s) in samples.iter().enumerate() {
        if keep[i] {
            for v in &s.sample.views {
                cam_obs[v.camera] += v.obs.len();
            }
        }
    }
    for c in 0..n_cams {
        if cam_obs[c] >= min_obs || kept >= hard_cap {
            continue;
        }
        let cand: Vec<usize> = samples
            .iter()
            .enumerate()
            .filter(|&(i, s)| !keep[i] && s.sample.views.iter().any(|v| v.camera == c))
            .map(|(i, _)| i)
            .collect();
        if cand.is_empty() {
            continue;
        }
        let stride = (cand.len() / 20).max(1);
        for &i in cand.iter().step_by(stride) {
            if cam_obs[c] >= min_obs || kept >= hard_cap {
                break;
            }
            keep[i] = true;
            kept += 1;
            for v in &samples[i].sample.views {
                cam_obs[v.camera] += v.obs.len();
            }
        }
    }
    samples
        .iter()
        .enumerate()
        .filter(|&(i, _)| keep[i])
        .map(|(_, s)| s.clone())
        .collect()
}

fn median_view_rms(p: &Problem, s: &State) -> f64 {
    let mut r: Vec<f64> = p
        .samples
        .iter()
        .enumerate()
        .flat_map(|(i, smp)| (0..smp.views.len()).filter_map(move |v| ba::view_rms_px(p, s, i, v)))
        .collect();
    if r.is_empty() {
        return 0.0;
    }
    let mid = r.len() / 2;
    *r.select_nth_unstable_by(mid, f64::total_cmp).1
}

/// Runs multi-round Huber BA with non-permanent view-level RMS rejection (re-selecting from `all`
/// each round, matching `calib::solve`), returning the solved camera poses, per-camera obs counts,
/// per-camera surviving view counts, rejected view count, and surviving connectivity.
fn solve_ba_rounds(
    cameras: &[CameraModel],
    leds: &BTreeMap<i32, Vec<Led>>,
    prior: &[Isometry3<f64>],
    samples: &[RefitSample],
    ba_opts: &ba::Options,
    min_pair_links: usize,
) -> (Vec<Isometry3<f64>>, Vec<usize>, Vec<usize>, usize, bool) {
    let n_cams = cameras.len();
    let (obs_init, _) = count_obs_and_views(samples, n_cams);
    let reference = (0..n_cams).max_by_key(|&c| obs_init[c]).unwrap_or(0);
    let all: Vec<WandSample> = samples.iter().map(|s| s.sample.clone()).collect();
    let seeds: Vec<Option<Isometry3<f64>>> = samples.iter().map(|s| Some(s.seed)).collect();

    let mut active = all.clone();
    let mut state = State {
        ref_from_cam: prior.to_vec(),
        ref_from_obj: seeds.clone(),
    };
    let schedule = [4.0_f64, 2.0_f64];
    let mut report = ba::Report::default();
    for round in 0..BA_MAX_ROUNDS {
        let p = Problem {
            cameras,
            leds,
            samples: &active,
            reference,
        };
        report = ba::solve(&p, &mut state, ba_opts);
        if round + 1 == BA_MAX_ROUNDS {
            break;
        }
        let limit = if round < schedule.len() {
            schedule[round]
        } else {
            (4.0 * median_view_rms(&p, &state)).max(0.5)
        };
        // Re-select from `all` using current state poses (or seed for dropped samples).
        let p_all = Problem {
            cameras,
            leds,
            samples: &all,
            reference,
        };
        let mut judged = State {
            ref_from_cam: state.ref_from_cam.clone(),
            ref_from_obj: state
                .ref_from_obj
                .iter()
                .zip(&seeds)
                .map(|(cur, seed)| cur.or(*seed))
                .collect(),
        };
        // Refresh poses for any previously dropped sample before judging its views.
        let needs_pose: Vec<usize> = active
            .iter()
            .enumerate()
            .filter(|(_, s)| s.views.is_empty())
            .map(|(i, _)| i)
            .collect();
        if !needs_pose.is_empty() {
            let sub_samples: Vec<WandSample> = needs_pose.iter().map(|&i| all[i].clone()).collect();
            let sub_p = Problem {
                cameras,
                leds,
                samples: &sub_samples,
                reference,
            };
            let mut sub_s = State {
                ref_from_cam: judged.ref_from_cam.clone(),
                ref_from_obj: needs_pose.iter().map(|&i| seeds[i]).collect(),
            };
            ba::solve_poses(&sub_p, &mut sub_s, ba_opts);
            for (k, &i) in needs_pose.iter().enumerate() {
                judged.ref_from_obj[i] = sub_s.ref_from_obj[k];
            }
        }
        let mut next = Vec::with_capacity(all.len());
        let mut changed = false;
        for (i, s) in all.iter().enumerate() {
            let mut views: Vec<_> = s
                .views
                .iter()
                .enumerate()
                .filter(|&(vi, _)| {
                    ba::view_rms_px(&p_all, &judged, i, vi).is_some_and(|r| r <= limit)
                })
                .map(|(_, v)| v.clone())
                .collect();
            if views.len() < 2 {
                views.clear();
            }
            if views.len() != active[i].views.len()
                || views
                    .iter()
                    .zip(&active[i].views)
                    .any(|(a, b)| a.camera != b.camera)
            {
                changed = true;
                state.ref_from_obj[i] = if views.is_empty() { None } else { seeds[i] };
            }
            let mut ns = s.clone();
            ns.views = views;
            next.push(ns);
        }
        active = next;
        if !changed && round >= schedule.len() {
            break;
        }
    }
    let total_views: usize = all.iter().map(|s| s.views.len()).sum();
    let kept_views: usize = active.iter().map(|s| s.views.len()).sum();
    // Check surviving per-camera views and graph connectivity (requiring at least half the
    // configured pair-link floor so two halves cannot hinge on a single noisy exposure).
    let min_surviving_links = (min_pair_links / 2).max(1);
    let mut links = vec![vec![0usize; n_cams]; n_cams];
    let mut surviving_views = vec![0usize; n_cams];
    for (i, s) in active.iter().enumerate() {
        if state.ref_from_obj[i].is_none() || s.views.len() < 2 {
            continue;
        }
        for v in &s.views {
            surviving_views[v.camera] += 1;
        }
        for a in 0..s.views.len() {
            for b in (a + 1)..s.views.len() {
                let (ca, cb) = (s.views[a].camera, s.views[b].camera);
                if ca != cb {
                    links[ca][cb] += 1;
                    links[cb][ca] += 1;
                }
            }
        }
    }
    let mut reached = vec![false; n_cams];
    let mut stack = vec![reference];
    reached[reference] = true;
    while let Some(u) = stack.pop() {
        for (v, seen) in reached.iter_mut().enumerate() {
            if !*seen && links[u][v] >= min_surviving_links {
                *seen = true;
                stack.push(v);
            }
        }
    }
    let connected = reached.into_iter().all(|r| r);
    (
        state.ref_from_cam,
        report.camera_obs,
        surviving_views,
        total_views.saturating_sub(kept_views),
        connected,
    )
}

/// Jointly refits connected sensors from chronologically ordered multi-camera observations,
/// anchors the result rigidly onto `prior` via all-sensor SE(3) alignment (with single-sensor
/// outlier leave-one-out), and validates improvement on chronological held-out exposures.
pub fn joint_refit(
    serials: &[String],
    cameras: &[CameraModel],
    prior: &[Isometry3<f64>],
    leds: &BTreeMap<i32, Vec<Led>>,
    samples: &[RefitSample],
    opts: &RefitOptions,
) -> Result<RefitResult, String> {
    joint_refit_against(serials, cameras, prior, prior, leds, samples, opts)
}

/// Like `joint_refit`, separating the world-frame `anchor` (e.g. the saved calibration on disk,
/// preventing cumulative gauge walk across repeated session refits) from the `current` poses
/// (initialisation and baseline for the held-out improvement gate).
pub fn joint_refit_against(
    serials: &[String],
    cameras: &[CameraModel],
    anchor: &[Isometry3<f64>],
    current: &[Isometry3<f64>],
    leds: &BTreeMap<i32, Vec<Led>>,
    samples: &[RefitSample],
    opts: &RefitOptions,
) -> Result<RefitResult, String> {
    let n_in = serials.len();
    if n_in < 2 || cameras.len() != n_in || anchor.len() != n_in || current.len() != n_in {
        return Err("joint refit needs at least two calibrated sensors".into());
    }
    let mut multi: Vec<RefitSample> = samples
        .iter()
        .filter(|s| s.sample.views.len() >= 2)
        .cloned()
        .collect();
    multi.sort_by_key(|s| s.sample.host_ns);
    if multi.len() < opts.min_train_samples + opts.min_heldout_samples {
        return Err(format!(
            "not enough multi-sensor exposures ({} collected, need at least {})",
            multi.len(),
            opts.min_train_samples + opts.min_heldout_samples
        ));
    }
    let span_ns = multi
        .last()
        .unwrap()
        .sample
        .host_ns
        .saturating_sub(multi[0].sample.host_ns);
    if span_ns < opts.min_span_ns {
        return Err(format!(
            "multi-sensor exposures span only {:.1} s (need at least {:.1} s)",
            span_ns as f64 * 1e-9,
            opts.min_span_ns as f64 * 1e-9
        ));
    }
    let split = ((multi.len() as f64 * opts.train_fraction.clamp(0.4, 0.85)).round() as usize)
        .clamp(
            opts.min_train_samples,
            multi.len().saturating_sub(opts.min_heldout_samples),
        );
    let (raw_train, raw_heldout) = (&multi[..split], &multi[split..]);

    // Iteratively prune cameras lacking train/heldout coverage or connectivity.
    let mut active = vec![true; n_in];
    let (component, train_full, heldout_full) = loop {
        let map: Vec<Option<usize>> = active
            .iter()
            .enumerate()
            .map(|(i, &on)| on.then_some(i))
            .collect();
        let tr = filter_remap(raw_train, &map);
        let ho = filter_remap(raw_heldout, &map);
        let (tr_obs, tr_views) = count_obs_and_views(&tr, n_in);
        let (ho_obs, ho_views) = count_obs_and_views(&ho, n_in);
        let tr_img = camera_img_pca_stds(&tr, n_in);
        let mut next_active = vec![false; n_in];
        for c in 0..n_in {
            next_active[c] = active[c]
                && tr_obs[c] >= opts.min_camera_obs
                && tr_views[c] >= opts.min_camera_views
                && ho_obs[c] >= opts.min_heldout_camera_obs
                && ho_views[c] >= opts.min_heldout_camera_views
                && tr_img[c][0] >= opts.min_camera_img_major_px
                && tr_img[c][1] >= opts.min_camera_img_minor_px;
        }
        let comp = largest_component(n_in, &next_active, &tr, opts.min_pair_links);
        let mut comp_mask = vec![false; n_in];
        for &c in &comp {
            comp_mask[c] = true;
        }
        if comp_mask == active {
            break (comp, tr, ho);
        }
        active = comp_mask;
    };
    if component.len() < 2 {
        return Err(
            "fewer than two sensors have enough overlapping multi-sensor exposures for a joint refit"
                .into(),
        );
    }
    let mut dense_map = vec![None; n_in];
    for (new_c, &old_c) in component.iter().enumerate() {
        dense_map[old_c] = Some(new_c);
    }
    let train_dense = filter_remap(&train_full, &dense_map);
    let heldout_dense = filter_remap(&heldout_full, &dense_map);
    if train_dense.len() < opts.min_train_samples {
        return Err(format!(
            "not enough overlapping training exposures ({}, need {})",
            train_dense.len(),
            opts.min_train_samples
        ));
    }
    if heldout_dense.len() < opts.min_heldout_samples {
        return Err(format!(
            "not enough overlapping held-out exposures ({}, need {})",
            heldout_dense.len(),
            opts.min_heldout_samples
        ));
    }
    if !varied(&train_dense, opts) {
        return Err(
            "controller motion in the training window is too small or narrow for a joint refit"
                .into(),
        );
    }
    if !heldout_varied(&heldout_dense, opts) {
        return Err(
            "controller motion in the held-out window is too small for a joint refit".into(),
        );
    }

    let k = component.len();
    let sub_cams: Vec<CameraModel> = component.iter().map(|&c| cameras[c]).collect();
    let sub_anchor: Vec<Isometry3<f64>> = component.iter().map(|&c| anchor[c]).collect();
    let sub_current: Vec<Isometry3<f64>> = component.iter().map(|&c| current[c]).collect();
    let train_ba = subsample(&train_dense, k, opts.max_train_samples, opts.min_camera_obs);
    let heldout_ba = subsample(
        &heldout_dense,
        k,
        opts.max_heldout_samples,
        opts.min_heldout_camera_obs,
    );

    let (fitted_raw, train_obs_after, train_views_after, train_rejected_views, connected) =
        solve_ba_rounds(
            &sub_cams,
            leds,
            &sub_current,
            &train_ba,
            &opts.ba,
            opts.min_pair_links,
        );
    if !connected {
        return Err("sensors disconnected after outlier rejection during joint refit".into());
    }
    let min_train_views_after = (opts.min_camera_views / 2).max(1);
    for (c, &obs) in train_obs_after.iter().enumerate() {
        if obs < opts.min_camera_obs / 2 {
            return Err(format!(
                "sensor {} kept only {obs} LED observations after outlier rejection",
                serials[component[c]]
            ));
        }
        if train_views_after[c] < min_train_views_after {
            return Err(format!(
                "sensor {} kept only {} training views after outlier rejection",
                serials[component[c]], train_views_after[c]
            ));
        }
    }

    let aligned = align_to_prior(&fitted_raw, &sub_anchor)
        .ok_or_else(|| "rigid SE(3) alignment of fitted sensors failed".to_string())?;
    let (before, after, heldout_dropped_views, cand_lost_obs) = compare_heldout(
        &sub_cams,
        leds,
        &sub_current,
        &aligned.poses,
        &heldout_ba,
        &opts.ba,
    );

    let mut reject_reason = None;
    if cand_lost_obs > 0 {
        reject_reason = Some(format!(
            "candidate sensor poses lost {cand_lost_obs} held-out LED projection(s)"
        ));
    } else if let Some(c) = (0..k).find(|&c| {
        aligned.delta_m[c] > opts.max_sensor_delta_m
            || aligned.delta_rad[c] > opts.max_sensor_delta_rad
    }) {
        reject_reason = Some(format!(
            "sensor {} shifted beyond sanity limit ({:.1} mm / {:.2} deg)",
            serials[component[c]],
            aligned.delta_m[c] * 1e3,
            aligned.delta_rad[c].to_degrees()
        ));
    } else if !before.pooled_rms_px.is_finite()
        || !after.pooled_rms_px.is_finite()
        || after.samples < opts.min_heldout_samples / 2
    {
        reject_reason = Some("held-out validation had too few usable observations".into());
    } else if after.pooled_rms_px > opts.max_after_rms_px {
        reject_reason = Some(format!(
            "held-out RMS remained too high ({:.3} px > {:.3} px)",
            after.pooled_rms_px, opts.max_after_rms_px
        ));
    } else {
        let gain = before.pooled_rms_px - after.pooled_rms_px;
        let rel = gain / before.pooled_rms_px.max(1e-9);
        if gain < opts.min_gain_px || rel < opts.min_rel_gain {
            reject_reason = Some(format!(
                "held-out RMS did not improve enough ({:.3} -> {:.3} px, {:+.3} px / {:+.1}%)",
                before.pooled_rms_px,
                after.pooled_rms_px,
                -gain,
                -rel * 100.0
            ));
        } else {
            let min_surviving_ho = (opts.min_heldout_camera_obs / 2).max(1);
            for c in 0..k {
                if after.camera_obs[c] < min_surviving_ho || !after.camera_rms_px[c].is_finite() {
                    reject_reason = Some(format!(
                        "sensor {} kept too few held-out observations ({})",
                        serials[component[c]], after.camera_obs[c]
                    ));
                    break;
                }
                if after.camera_rms_px[c] > opts.max_after_rms_px * 2.0 {
                    reject_reason = Some(format!(
                        "sensor {} held-out RMS remained too high ({:.3} px)",
                        serials[component[c]], after.camera_rms_px[c]
                    ));
                    break;
                }
                if before.camera_obs[c] >= min_surviving_ho
                    && after.camera_rms_px[c] > before.camera_rms_px[c] + opts.max_cam_regress_px
                {
                    reject_reason = Some(format!(
                        "sensor {} regressed on held-out exposures ({:.3} -> {:.3} px)",
                        serials[component[c]], before.camera_rms_px[c], after.camera_rms_px[c]
                    ));
                    break;
                }
            }
        }
    }

    let accepted = reject_reason.is_none();
    let sensors: Vec<RefitSensor> = component
        .iter()
        .enumerate()
        .map(|(c, &orig)| RefitSensor {
            camera: orig,
            serial: serials[orig].clone(),
            world_from_cam: aligned.poses[c],
            delta_m: aligned.delta_m[c],
            delta_rad: aligned.delta_rad[c],
            before_rms_px: before.camera_rms_px[c],
            after_rms_px: after.camera_rms_px[c],
            before_median_px: before.camera_median_px[c],
            after_median_px: after.camera_median_px[c],
            train_obs: train_obs_after[c],
            heldout_obs: after.camera_obs[c],
            outlier: aligned.outlier == Some(c),
        })
        .collect();
    let skipped_serials: Vec<String> = (0..n_in)
        .filter(|&c| dense_map[c].is_none())
        .map(|c| serials[c].clone())
        .collect();
    let outlier_serial = aligned.outlier.map(|c| serials[component[c]].clone());

    Ok(RefitResult {
        accepted,
        reject_reason,
        sensors,
        skipped_serials,
        before,
        after,
        train_samples: train_ba.len(),
        heldout_samples: heldout_ba.len(),
        train_rejected_views,
        heldout_dropped_views,
        outlier_serial,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::{self, SceneParams};
    use nalgebra::Translation3;

    fn iso(t: [f64; 3], rot_deg: [f64; 3]) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(t[0], t[1], t[2]),
            UnitQuaternion::from_scaled_axis(Vector3::new(
                rot_deg[0].to_radians(),
                rot_deg[1].to_radians(),
                rot_deg[2].to_radians(),
            )),
        )
    }

    #[test]
    fn align_se3_recovers_exact_rigid_transform_for_two_and_five_sensors() {
        let prior = [
            iso([-1.2, 1.5, -1.0], [10.0, 35.0, -5.0]),
            iso([1.2, 1.5, -1.0], [12.0, -35.0, 4.0]),
            iso([-1.0, 1.8, 1.2], [-8.0, 145.0, 2.0]),
            iso([1.0, 1.7, 1.3], [-10.0, -140.0, -3.0]),
            iso([0.0, 2.1, -1.6], [20.0, 0.0, 1.0]),
        ];
        let gauge = iso([0.35, -0.22, 0.41], [17.0, -28.0, 11.0]);
        let fitted: Vec<_> = prior.iter().map(|p| gauge * p).collect();

        for count in [1usize, 2, 3, 5] {
            let recovered = align_se3(&fitted[..count], &prior[..count]).unwrap();
            let err = recovered * gauge;
            assert!(
                err.translation.vector.norm() < 1e-12 && err.rotation.angle() < 1e-12,
                "count={count}: trans={} rot={}",
                err.translation.vector.norm(),
                err.rotation.angle()
            );
        }
    }

    #[test]
    fn align_to_prior_excludes_single_bumped_sensor_on_three_and_five_sensor_rigs() {
        let prior = [
            iso([-1.2, 1.5, -1.0], [10.0, 35.0, 0.0]),
            iso([1.2, 1.5, -1.0], [10.0, -35.0, 0.0]),
            iso([0.0, 1.8, 1.4], [-12.0, 180.0, 0.0]),
            iso([-1.1, 1.7, 1.1], [-8.0, 140.0, 0.0]),
            iso([1.1, 1.7, 1.1], [-8.0, -140.0, 0.0]),
        ];
        let gauge = iso([0.1, -0.05, 0.08], [3.0, -4.0, 2.0]);
        let bump = iso([0.018, -0.006, 0.004], [0.8, -0.4, 0.2]);

        for count in [3usize, 5] {
            let mut actual = prior[..count].to_vec();
            actual[0] *= bump;
            let fitted: Vec<_> = actual.iter().map(|p| gauge * p).collect();
            let res = align_to_prior(&fitted, &prior[..count]).unwrap();
            assert_eq!(
                res.outlier,
                Some(0),
                "count={count} deltas={:?}",
                res.delta_m
            );
            for i in 1..count {
                assert!(res.delta_m[i] < 1e-9, "inlier {i} moved {}", res.delta_m[i]);
                assert!(
                    res.delta_rad[i] < 1e-9,
                    "inlier {i} turned {}",
                    res.delta_rad[i]
                );
            }
            assert!((res.delta_m[0] - bump.translation.vector.norm()).abs() < 1e-9);
            assert!((res.delta_rad[0] - bump.rotation.angle()).abs() < 1e-9);
        }
    }

    #[test]
    fn align_to_prior_keeps_all_sensor_gauge_for_small_uniform_drift() {
        let prior = [
            iso([-1.2, 1.5, -1.0], [10.0, 35.0, 0.0]),
            iso([1.2, 1.5, -1.0], [10.0, -35.0, 0.0]),
            iso([0.0, 1.8, 1.4], [-12.0, 180.0, 0.0]),
            iso([-1.1, 1.7, 1.1], [-8.0, 140.0, 0.0]),
        ];
        let drifts = [
            iso([0.0015, -0.001, 0.0008], [0.05, -0.03, 0.02]),
            iso([-0.0012, 0.0009, -0.001], [-0.04, 0.04, -0.01]),
            iso([0.0008, 0.0014, 0.0005], [0.03, 0.02, -0.04]),
            iso([-0.001, -0.0007, 0.0012], [-0.02, -0.05, 0.03]),
        ];
        let fitted: Vec<_> = prior.iter().zip(&drifts).map(|(p, d)| p * d).collect();
        let res = align_to_prior(&fitted, &prior).unwrap();
        assert_eq!(res.outlier, None);
    }

    #[test]
    fn joint_refit_recovers_drifted_sensors_and_rejects_already_accurate_room() {
        let sc = synth::scene(&SceneParams {
            pixel_noise: 0.15,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            samples: 240,
            ..Default::default()
        });
        let samples: Vec<RefitSample> = sc
            .samples
            .iter()
            .zip(&sc.world_from_obj)
            .map(|(sample, &seed)| RefitSample {
                sample: sample.clone(),
                seed,
            })
            .collect();
        let opts = RefitOptions::default();

        // Unmoved room: held-out RMS is already at the noise floor, so the gate rejects.
        let exact = joint_refit(
            &sc.serials,
            &sc.cameras,
            &sc.world_from_cam,
            &sc.leds,
            &samples,
            &opts,
        )
        .unwrap();
        assert!(
            !exact.accepted,
            "already-calibrated room should not refit: {:.4} -> {:.4} px",
            exact.before.pooled_rms_px, exact.after.pooled_rms_px
        );

        // Two sensors drifted by realistic inter-session amounts (~4-6 mm, ~0.12-0.18 deg).
        let mut stale = sc.world_from_cam.clone();
        stale[0] *= iso([0.005, -0.003, 0.002], [0.15, -0.10, 0.05]);
        stale[1] *= iso([-0.004, 0.002, -0.003], [-0.12, 0.08, -0.06]);
        let res = joint_refit(&sc.serials, &sc.cameras, &stale, &sc.leds, &samples, &opts).unwrap();
        assert!(
            res.accepted,
            "drifted room should be accepted: {:?}",
            res.reject_reason
        );
        assert!(
            res.after.pooled_rms_px < res.before.pooled_rms_px * 0.5,
            "{:.3} -> {:.3} px",
            res.before.pooled_rms_px,
            res.after.pooled_rms_px
        );
        let truth_align = align_se3(
            &res.sensors
                .iter()
                .map(|s| s.world_from_cam)
                .collect::<Vec<_>>(),
            &sc.world_from_cam,
        )
        .unwrap();
        for s in &res.sensors {
            let d = sc.world_from_cam[s.camera].inverse() * (truth_align * s.world_from_cam);
            assert!(
                d.translation.vector.norm() < 0.002 && d.rotation.angle().to_degrees() < 0.05,
                "{}: {:.2} mm, {:.3} deg",
                s.serial,
                d.translation.vector.norm() * 1e3,
                d.rotation.angle().to_degrees()
            );
        }
    }

    #[test]
    fn joint_refit_recovers_drifted_two_sensor_rig() {
        let sc = synth::scene(&SceneParams {
            pixel_noise: 0.15,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            samples: 240,
            ..Default::default()
        });
        // Keep only the first two sensors and multi-camera samples visible to both.
        let serials = sc.serials[..2].to_vec();
        let cameras = sc.cameras[..2].to_vec();
        let truth = sc.world_from_cam[..2].to_vec();
        let samples: Vec<RefitSample> = sc
            .samples
            .iter()
            .zip(&sc.world_from_obj)
            .filter_map(|(sample, &seed)| {
                let mut s = sample.clone();
                s.views.retain(|v| v.camera < 2);
                (s.views.len() == 2).then_some(RefitSample { sample: s, seed })
            })
            .collect();
        let mut stale = truth.clone();
        stale[1] *= iso([0.006, -0.004, 0.003], [0.18, -0.12, 0.08]);
        let res = joint_refit(
            &serials,
            &cameras,
            &stale,
            &sc.leds,
            &samples,
            &RefitOptions::default(),
        )
        .unwrap();
        assert!(
            res.accepted,
            "2-sensor refit rejected: {:?}",
            res.reject_reason
        );
        assert_eq!(res.sensors.len(), 2);
        assert!(
            res.after.pooled_rms_px < res.before.pooled_rms_px * 0.5,
            "{:.3} -> {:.3} px",
            res.before.pooled_rms_px,
            res.after.pooled_rms_px
        );
        // Relative pose between the two sensors matches truth tightly.
        let rel_fit = res.sensors[0].world_from_cam.inverse() * res.sensors[1].world_from_cam;
        let rel_truth = truth[0].inverse() * truth[1];
        let d = rel_truth.inverse() * rel_fit;
        assert!(
            d.translation.vector.norm() < 0.003 && d.rotation.angle().to_degrees() < 0.08,
            "2-sensor rel err: {:.2} mm, {:.3} deg",
            d.translation.vector.norm() * 1e3,
            d.rotation.angle().to_degrees()
        );
    }

    #[test]
    fn joint_refit_detects_single_bumped_sensor_without_shifting_unmoved_peers() {
        let sc = synth::scene(&SceneParams {
            pixel_noise: 0.15,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            samples: 240,
            ..Default::default()
        });
        let samples: Vec<RefitSample> = sc
            .samples
            .iter()
            .zip(&sc.world_from_obj)
            .map(|(sample, &seed)| RefitSample {
                sample: sample.clone(),
                seed,
            })
            .collect();
        // Sensor 0 was bumped from `stale[0]` to `sc.world_from_cam[0]`; others stayed put.
        let mut stale = sc.world_from_cam.clone();
        stale[0] *= iso([0.015, -0.008, 0.005], [0.50, -0.30, 0.15]);
        let res = joint_refit(
            &sc.serials,
            &sc.cameras,
            &stale,
            &sc.leds,
            &samples,
            &RefitOptions::default(),
        )
        .unwrap();
        assert!(res.accepted, "{:?}", res.reject_reason);
        assert_eq!(res.outlier_serial.as_deref(), Some(sc.serials[0].as_str()));
        for s in &res.sensors {
            let err = sc.world_from_cam[s.camera].inverse() * s.world_from_cam;
            assert!(
                err.translation.vector.norm() < 0.002 && err.rotation.angle().to_degrees() < 0.05,
                "{}: {:.2} mm, {:.3} deg",
                s.serial,
                err.translation.vector.norm() * 1e3,
                err.rotation.angle().to_degrees()
            );
            if s.camera == 0 {
                assert!(s.outlier);
                assert!(s.delta_m > 0.012 && s.delta_rad.to_degrees() > 0.45);
            } else {
                assert!(!s.outlier);
                assert!(
                    s.delta_m < 0.002 && s.delta_rad.to_degrees() < 0.05,
                    "unmoved peer {} shifted {:.2} mm, {:.3} deg",
                    s.serial,
                    s.delta_m * 1e3,
                    s.delta_rad.to_degrees()
                );
            }
        }
    }

    #[test]
    fn joint_refit_handles_corrupt_views_in_train_and_heldout() {
        let sc = synth::scene(&SceneParams {
            pixel_noise: 0.15,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            samples: 240,
            ..Default::default()
        });
        let mut samples: Vec<RefitSample> = sc
            .samples
            .iter()
            .zip(&sc.world_from_obj)
            .map(|(sample, &seed)| RefitSample {
                sample: sample.clone(),
                seed,
            })
            .collect();
        // Corrupt one view in every 20th sample (both train and held-out) by 25 px.
        for (i, rs) in samples.iter_mut().enumerate() {
            if i % 20 == 0 {
                if let Some(v) = rs.sample.views.first_mut() {
                    for ob in &mut v.obs {
                        ob.xy.x += 25.0;
                    }
                }
            }
        }
        let mut stale = sc.world_from_cam.clone();
        stale[0] *= iso([0.005, -0.003, 0.002], [0.15, -0.10, 0.05]);
        let res = joint_refit(
            &sc.serials,
            &sc.cameras,
            &stale,
            &sc.leds,
            &samples,
            &RefitOptions::default(),
        )
        .unwrap();
        assert!(res.accepted, "{:?}", res.reject_reason);
        assert!(res.train_rejected_views > 0);
        assert!(res.heldout_dropped_views > 0);
        assert!(
            res.after.pooled_rms_px < 0.25,
            "corrupt views leaked into held-out RMS: {:.3} px",
            res.after.pooled_rms_px
        );
    }

    #[test]
    fn joint_refit_against_anchors_to_saved_frame_while_gating_on_current_poses() {
        let sc = synth::scene(&SceneParams {
            pixel_noise: 0.15,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            samples: 240,
            ..Default::default()
        });
        let samples: Vec<RefitSample> = sc
            .samples
            .iter()
            .zip(&sc.world_from_obj)
            .map(|(sample, &seed)| RefitSample {
                sample: sample.clone(),
                seed,
            })
            .collect();
        // Anchor is the saved calibration; current is slightly off internally and rigidly shifted.
        let shift = iso([0.04, -0.02, 0.03], [1.5, -1.0, 0.5]);
        let mut current: Vec<_> = sc.world_from_cam.iter().map(|p| shift * p).collect();
        current[0] *= iso([0.005, -0.003, 0.002], [0.15, -0.10, 0.05]);
        let res = joint_refit_against(
            &sc.serials,
            &sc.cameras,
            &sc.world_from_cam,
            &current,
            &sc.leds,
            &samples,
            &RefitOptions::default(),
        )
        .unwrap();
        assert!(res.accepted, "{:?}", res.reject_reason);
        // Poses must be anchored back into `sc.world_from_cam`, not shifted with `current`.
        for s in &res.sensors {
            let err = sc.world_from_cam[s.camera].inverse() * s.world_from_cam;
            assert!(
                err.translation.vector.norm() < 0.002 && err.rotation.angle().to_degrees() < 0.05,
                "{}: {:.2} mm, {:.3} deg",
                s.serial,
                err.translation.vector.norm() * 1e3,
                err.rotation.angle().to_degrees()
            );
        }
        // Once already at the fitted poses, gating against current rejects a redundant refit.
        let fitted: Vec<_> = res.sensors.iter().map(|s| s.world_from_cam).collect();
        let again = joint_refit_against(
            &sc.serials,
            &sc.cameras,
            &sc.world_from_cam,
            &fitted,
            &sc.leds,
            &samples,
            &RefitOptions::default(),
        )
        .unwrap();
        assert!(!again.accepted);
    }

    #[test]
    fn bumped_sensor_requires_both_pose_outlier_and_pixel_evidence() {
        let sc = synth::scene(&SceneParams {
            pixel_noise: 0.15,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            samples: 240,
            ..Default::default()
        });
        let samples: Vec<RefitSample> = sc
            .samples
            .iter()
            .zip(&sc.world_from_obj)
            .map(|(sample, &seed)| RefitSample {
                sample: sample.clone(),
                seed,
            })
            .collect();
        // A genuine single-sensor bump on a >= 3 sensor rig flags bumped_sensor().
        let mut bumped = sc.world_from_cam.clone();
        bumped[0] *= iso([0.008, -0.004, 0.003], [0.32, -0.15, 0.08]);
        let res = joint_refit(
            &sc.serials,
            &sc.cameras,
            &bumped,
            &sc.leds,
            &samples,
            &RefitOptions::default(),
        )
        .unwrap();
        assert!(res.accepted, "{:?}", res.reject_reason);
        assert_eq!(
            res.bumped_sensor().map(|s| s.serial.as_str()),
            Some(sc.serials[0].as_str())
        );

        // Small multi-sensor thermal drift improves held-out RMS but must NOT flag a physical bump.
        let mut drifted = sc.world_from_cam.clone();
        drifted[0] *= iso([0.003, -0.002, 0.001], [0.10, -0.06, 0.03]);
        drifted[1] *= iso([-0.002, 0.003, -0.001], [-0.08, 0.07, -0.04]);
        let res_drift = joint_refit(
            &sc.serials,
            &sc.cameras,
            &drifted,
            &sc.leds,
            &samples,
            &RefitOptions::default(),
        )
        .unwrap();
        assert!(res_drift.accepted, "{:?}", res_drift.reject_reason);
        assert_eq!(res_drift.bumped_sensor(), None);
    }

    #[test]
    fn returned_poses_match_reported_heldout_stats_and_denominators_match() {
        let sc = synth::scene(&SceneParams {
            pixel_noise: 0.15,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            samples: 240,
            ..Default::default()
        });
        let samples: Vec<RefitSample> = sc
            .samples
            .iter()
            .zip(&sc.world_from_obj)
            .map(|(sample, &seed)| RefitSample {
                sample: sample.clone(),
                seed,
            })
            .collect();
        let mut drifted = sc.world_from_cam.clone();
        drifted[0] *= iso([0.005, -0.003, 0.002], [0.18, -0.10, 0.05]);
        let opts = RefitOptions::default();
        let res = joint_refit(
            &sc.serials,
            &sc.cameras,
            &drifted,
            &sc.leds,
            &samples,
            &opts,
        )
        .unwrap();
        assert!(res.accepted, "{:?}", res.reject_reason);
        assert_eq!(res.before.total_obs, res.after.total_obs);
        assert_eq!(res.before.camera_obs, res.after.camera_obs);
        // Re-evaluating the exact returned poses on the held-out split must reproduce `after`.
        let mut ordered = samples.clone();
        ordered.sort_by_key(|s| s.sample.host_ns);
        let split = ((ordered.len() as f64 * opts.train_fraction).round() as usize).clamp(
            opts.min_train_samples,
            ordered.len() - opts.min_heldout_samples,
        );
        let returned: Vec<_> = res.sensors.iter().map(|s| s.world_from_cam).collect();
        let recheck = eval_heldout(
            &sc.cameras,
            &sc.leds,
            &returned,
            &ordered[split..],
            &opts.ba,
        );
        assert_eq!(recheck.total_obs, res.after.total_obs);
        assert!(
            (recheck.pooled_rms_px - res.after.pooled_rms_px).abs() < 1e-12,
            "recheck {} != reported {}",
            recheck.pooled_rms_px,
            res.after.pooled_rms_px
        );
    }

    #[test]
    fn compare_heldout_counts_lost_projections_and_sanity_limit_rejects_large_shift() {
        let sc = synth::scene(&SceneParams {
            pixel_noise: 0.15,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            samples: 240,
            ..Default::default()
        });
        let samples: Vec<RefitSample> = sc
            .samples
            .iter()
            .zip(&sc.world_from_obj)
            .map(|(sample, &seed)| RefitSample {
                sample: sample.clone(),
                seed,
            })
            .collect();
        // Flipping camera 0 180 deg causes its observations to fail projection under candidate.
        let mut bad = sc.world_from_cam.clone();
        bad[0] *= iso([0.0, 0.0, 0.0], [180.0, 0.0, 0.0]);
        let (before, after, _dropped, lost) = compare_heldout(
            &sc.cameras,
            &sc.leds,
            &sc.world_from_cam,
            &bad,
            &samples[..40],
            &RefitOptions::default().ba,
        );
        assert!(lost > 0, "flipped camera must report lost projections");
        assert_eq!(before.total_obs, after.total_obs);
        assert_eq!(before.camera_obs, after.camera_obs);

        // Tight sanity cap rejects an otherwise valid shift that exceeds the configured bound.
        let mut shifted = sc.world_from_cam.clone();
        shifted[0] *= iso([0.008, -0.004, 0.003], [0.25, -0.10, 0.05]);
        let opts = RefitOptions {
            max_sensor_delta_m: 0.004,
            ..Default::default()
        };
        let res = joint_refit(
            &sc.serials,
            &sc.cameras,
            &shifted,
            &sc.leds,
            &samples,
            &opts,
        )
        .unwrap();
        assert!(!res.accepted);
        assert!(res
            .reject_reason
            .as_deref()
            .is_some_and(|r| r.contains("sanity limit")));
    }

    #[test]
    fn rejects_planar_motion_and_skips_sensor_with_narrow_image_coverage() {
        let sc = synth::scene(&SceneParams {
            pixel_noise: 0.15,
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            samples: 240,
            ..Default::default()
        });
        let samples: Vec<RefitSample> = sc
            .samples
            .iter()
            .zip(&sc.world_from_obj)
            .map(|(sample, &seed)| RefitSample {
                sample: sample.clone(),
                seed,
            })
            .collect();
        // Flattening all seeds onto a 2D plane (zero Y depth) fails the 3D conditioning check.
        let mut planar = samples.clone();
        for rs in &mut planar {
            rs.seed.translation.vector.y = 1.2;
        }
        let err = joint_refit(
            &sc.serials,
            &sc.cameras,
            &sc.world_from_cam,
            &sc.leds,
            &planar,
            &RefitOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("too small or narrow"), "{err}");

        // Collapsing camera 2's training pixel coordinates into a tiny 10x10 px patch skips
        // camera 2 cleanly while the remaining well-conditioned sensors still refit.
        let mut corner = samples;
        let split = (corner.len() as f64 * 0.70).round() as usize;
        for rs in &mut corner[..split] {
            for v in &mut rs.sample.views {
                if v.camera == 2 {
                    for ob in &mut v.obs {
                        ob.xy =
                            Vector2::new(640.0 + (ob.led % 3) as f64, 480.0 + (ob.led / 3) as f64);
                    }
                }
            }
        }
        let mut drifted = sc.world_from_cam.clone();
        drifted[0] *= iso([0.005, -0.003, 0.002], [0.15, -0.10, 0.05]);
        let res = joint_refit(
            &sc.serials,
            &sc.cameras,
            &drifted,
            &sc.leds,
            &corner,
            &RefitOptions::default(),
        )
        .unwrap();
        assert!(res.accepted, "{:?}", res.reject_reason);
        assert_eq!(res.skipped_serials, vec![sc.serials[2].clone()]);
        assert_eq!(res.sensors.len(), 2);
    }
}
