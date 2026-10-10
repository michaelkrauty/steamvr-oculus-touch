// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Initial sensor poses from the engine's per-frame camera-relative poses. When two sensors see a
//! controller in the same exposure, cam_a_from_obj * (cam_b_from_obj)^-1 is one sample of their
//! relative pose. Wrong-but-confident matches (Phase 1's flipped placement) are outliers, so each
//! pair is averaged robustly, and sensors are chained to the reference along the best-supported
//! pairs (maximum spanning tree by inlier count).

use std::collections::BTreeMap;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3, Vector4};

use super::capture::WandSample;

/// Inlier gates around the rotation medoid / translation median. Wide enough for single-frame pose
/// noise at room scale (a 2 degree error at 3 m is 10 cm); a wrong match is off by tens of degrees.
pub const ROT_INLIER_RAD: f64 = 0.1396; // 8 degrees
pub const POS_INLIER_M: f64 = 0.30;

/// Relative-pose samples a_from_b for every sensor pair a < b seen in the same exposure with
/// confident (strong) engine poses.
pub fn pair_samples(samples: &[WandSample]) -> BTreeMap<(usize, usize), Vec<Isometry3<f64>>> {
    let mut out: BTreeMap<(usize, usize), Vec<Isometry3<f64>>> = BTreeMap::new();
    for s in samples {
        for (i, a) in s.views.iter().enumerate() {
            for b in &s.views[i + 1..] {
                let (Some(ta), Some(tb)) = (a.cam_from_obj, b.cam_from_obj) else {
                    continue;
                };
                if !(a.strong && b.strong) || a.camera == b.camera {
                    continue;
                }
                let (lo, hi, t) = if a.camera < b.camera {
                    (a.camera, b.camera, ta * tb.inverse())
                } else {
                    (b.camera, a.camera, tb * ta.inverse())
                };
                out.entry((lo, hi)).or_default().push(t);
            }
        }
    }
    out
}

/// Robust mean: the rotation medoid (over at most ~400 samples), the mean of rotations within
/// ROT_INLIER_RAD of it, and the mean translation of those within POS_INLIER_M of their per-axis
/// median. Returns the mean and its inlier count; None below 3 inliers.
pub fn robust_mean(samples: &[Isometry3<f64>]) -> Option<(Isometry3<f64>, usize)> {
    if samples.len() < 3 {
        return None;
    }
    let probe: Vec<&Isometry3<f64>> = samples
        .iter()
        .step_by((samples.len() / 400).max(1))
        .collect();
    let spread =
        |r: &UnitQuaternion<f64>| probe.iter().map(|o| r.angle_to(&o.rotation)).sum::<f64>();
    let medoid = probe
        .iter()
        .map(|p| p.rotation)
        .min_by(|a, b| spread(a).total_cmp(&spread(b)))?;
    let rot_in: Vec<&Isometry3<f64>> = samples
        .iter()
        .filter(|s| s.rotation.angle_to(&medoid) < ROT_INLIER_RAD)
        .collect();
    if rot_in.len() < 3 {
        return None;
    }
    let median = |axis: usize| {
        let mut v: Vec<f64> = rot_in.iter().map(|s| s.translation.vector[axis]).collect();
        v.sort_by(f64::total_cmp);
        v[v.len() / 2]
    };
    let t_med = Vector3::new(median(0), median(1), median(2));
    let inliers: Vec<&Isometry3<f64>> = rot_in
        .into_iter()
        .filter(|s| (s.translation.vector - t_med).norm() < POS_INLIER_M)
        .collect();
    if inliers.len() < 3 {
        return None;
    }
    let m = medoid.quaternion().coords;
    let q = inliers.iter().fold(Vector4::zeros(), |acc, s| {
        let c = s.rotation.quaternion().coords;
        acc + if c.dot(&m) < 0.0 { -c } else { c }
    });
    let t = inliers
        .iter()
        .fold(Vector3::zeros(), |acc, s| acc + s.translation.vector)
        / inliers.len() as f64;
    Some((
        Isometry3::from_parts(
            Translation3::from(t),
            UnitQuaternion::from_quaternion(Quaternion::from_vector(q)),
        ),
        inliers.len(),
    ))
}

#[derive(Debug, Clone)]
pub struct Initial {
    /// Camera index of the reference sensor (the one with the most confident sightings).
    pub reference: usize,
    /// ref_from_cam per sensor.
    pub ref_from_cam: Vec<Isometry3<f64>>,
    /// (from, to, inliers) links used to chain each sensor to the reference.
    pub links: Vec<(usize, usize, usize)>,
}

pub fn initialize(
    samples: &[WandSample],
    serials: &[String],
    min_links: usize,
) -> Result<Initial, String> {
    let n = serials.len();
    let mut confident = vec![0usize; n];
    for v in samples.iter().flat_map(|s| &s.views) {
        if v.strong && v.cam_from_obj.is_some() {
            confident[v.camera] += 1;
        }
    }
    let reference = (0..n)
        .max_by_key(|&c| (confident[c], std::cmp::Reverse(c)))
        .ok_or("no sensors in the capture")?;
    let edges: Vec<(usize, usize, Isometry3<f64>, usize)> = pair_samples(samples)
        .into_iter()
        .filter_map(|((a, b), poses)| {
            robust_mean(&poses)
                .filter(|&(_, k)| k >= min_links)
                .map(|(t, k)| (a, b, t, k))
        })
        .collect();
    let mut pose: Vec<Option<Isometry3<f64>>> = vec![None; n];
    pose[reference] = Some(Isometry3::identity());
    let mut links = Vec::new();
    loop {
        // Edge t is a_from_b: ref_from_b = ref_from_a * t, ref_from_a = ref_from_b * t^-1.
        let best = edges
            .iter()
            .filter_map(|&(a, b, t, k)| match (pose[a], pose[b]) {
                (Some(pa), None) => Some((k, a, b, pa * t)),
                (None, Some(pb)) => Some((k, b, a, pb * t.inverse())),
                _ => None,
            })
            .max_by_key(|e| e.0);
        let Some((k, from, to, t)) = best else { break };
        pose[to] = Some(t);
        links.push((from, to, k));
    }
    let missing: Vec<&str> = (0..n)
        .filter(|&c| pose[c].is_none())
        .map(|c| serials[c].as_str())
        .collect();
    if !missing.is_empty() {
        return Err(format!(
            "sensor(s) {} share too few confident controller sightings with the others (need {min_links} per link): \
             wave the controllers where those sensors' views overlap another sensor's",
            missing.join(", ")
        ));
    }
    Ok(Initial {
        reference,
        ref_from_cam: pose.into_iter().map(Option::unwrap).collect(),
        links,
    })
}

/// ref_from_obj per sample, from the sighting most others agree with (within the inlier gates), then
/// confident ones, then the one with the most labelled LEDs. A wrong-but-confident match can carry
/// the most LEDs; agreement outvotes it whenever two other sensors see the controller.
pub fn wand_poses(
    samples: &[WandSample],
    ref_from_cam: &[Isometry3<f64>],
) -> Vec<Option<Isometry3<f64>>> {
    samples
        .iter()
        .map(|s| {
            let candidates: Vec<(bool, usize, Isometry3<f64>)> = s
                .views
                .iter()
                .filter_map(|v| {
                    Some((
                        v.strong,
                        v.obs.len(),
                        ref_from_cam[v.camera] * v.cam_from_obj?,
                    ))
                })
                .collect();
            let support = |t: &Isometry3<f64>| {
                candidates
                    .iter()
                    .filter(|(_, _, o)| {
                        o.rotation.angle_to(&t.rotation) < ROT_INLIER_RAD
                            && (o.translation.vector - t.translation.vector).norm() < POS_INLIER_M
                    })
                    .count()
            };
            candidates
                .iter()
                .max_by_key(|(strong, n, t)| (support(t), *strong, *n))
                .map(|(_, _, t)| *t)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::{self, Scene, SceneParams};

    fn truth(sc: &Scene, reference: usize) -> Vec<Isometry3<f64>> {
        sc.world_from_cam
            .iter()
            .map(|w| sc.world_from_cam[reference].inverse() * w)
            .collect()
    }

    fn assert_close(init: &Initial, sc: &Scene) {
        for (c, t) in truth(sc, init.reference).iter().enumerate() {
            let dp = (init.ref_from_cam[c].translation.vector - t.translation.vector).norm();
            let da = init.ref_from_cam[c].rotation.angle_to(&t.rotation);
            assert!(
                dp < 0.03 && da < 0.02,
                "sensor {c}: {dp:.4} m {:.3} deg",
                da.to_degrees()
            );
        }
    }

    #[test]
    fn recovers_relative_sensor_poses() {
        let sc = synth::scene(&SceneParams::default());
        let init = initialize(&sc.samples, &sc.serials, 20).unwrap();
        assert_eq!(init.links.len(), 2);
        assert_close(&init, &sc);
    }

    /// Review Focus 2: wrong-but-confident sightings must not bend the initial poses.
    #[test]
    fn robust_to_flipped_views() {
        let sc = synth::scene(&SceneParams {
            flipped_fraction: 0.25,
            ..Default::default()
        });
        let init = initialize(&sc.samples, &sc.serials, 20).unwrap();
        assert_close(&init, &sc);
    }

    /// Review Focus 1: a sensor never seen together with another cannot be placed, and must be named.
    #[test]
    fn unlinked_sensor_is_an_error() {
        let mut sc = synth::scene(&SceneParams::default());
        for s in &mut sc.samples {
            if s.views.len() > 1 {
                s.views.retain(|v| v.camera != 2);
            }
        }
        let err = initialize(&sc.samples, &sc.serials, 20).unwrap_err();
        assert!(
            err.contains("share too few") && err.contains("WMTD000000003"),
            "{err}"
        );
    }

    #[test]
    fn robust_mean_ignores_a_minority_of_outliers() {
        let truth = Isometry3::from_parts(
            Translation3::new(1.0, 2.0, 3.0),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.1, 0.2, 0.3)),
        );
        let mut v: Vec<Isometry3<f64>> = (0..40).map(|_| truth).collect();
        v.extend((0..20).map(|i| {
            Isometry3::from_parts(
                Translation3::new(-1.0, i as f64, 0.0),
                UnitQuaternion::from_scaled_axis(Vector3::new(1.5, 0.0, i as f64 * 0.1)),
            )
        }));
        let (mean, inliers) = robust_mean(&v).unwrap();
        assert_eq!(inliers, 40);
        assert!(
            (mean.translation.vector - truth.translation.vector).norm() < 1e-9
                && mean.rotation.angle_to(&truth.rotation) < 1e-9
        );
    }

    #[test]
    fn wand_poses_follow_the_reference_frame() {
        let sc = synth::scene(&SceneParams {
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            ..Default::default()
        });
        let init = initialize(&sc.samples, &sc.serials, 20).unwrap();
        let wand = wand_poses(&sc.samples, &init.ref_from_cam);
        let ref_from_world = sc.world_from_cam[init.reference].inverse();
        for (w, truth) in wand.iter().zip(&sc.world_from_obj) {
            assert!(
                (w.unwrap().translation.vector - (ref_from_world * truth).translation.vector)
                    .norm()
                    < 1e-6
            );
        }
    }

    /// Review fix 10: a sample's pose is seeded from the sighting the others agree with, not from the
    /// one with the most LEDs (a wrong-but-confident match can have more).
    #[test]
    fn wand_pose_seeds_from_the_agreeing_sighting() {
        let sc = synth::scene(&SceneParams {
            pnp_rot_noise_rad: 0.0,
            pnp_pos_noise_m: 0.0,
            ..Default::default()
        });
        let init = initialize(&sc.samples, &sc.serials, 20).unwrap();
        let i = sc
            .samples
            .iter()
            .position(|s| s.views.len() == 3)
            .expect("a sample seen by all three sensors");
        let mut samples = sc.samples.clone();
        let v = &mut samples[i].views[0];
        let t = v.cam_from_obj.unwrap();
        v.cam_from_obj = Some(Isometry3::from_parts(
            t.translation,
            t.rotation * UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.0, 1.6)),
        ));
        let extra = v.obs.clone();
        v.obs.extend(extra);
        let got = wand_poses(&samples, &init.ref_from_cam)[i].unwrap();
        let truth = sc.world_from_cam[init.reference].inverse() * sc.world_from_obj[i];
        assert!(
            got.rotation.angle_to(&truth.rotation) < 1e-6
                && (got.translation.vector - truth.translation.vector).norm() < 1e-6
        );
    }
}
