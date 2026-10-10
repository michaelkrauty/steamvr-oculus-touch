// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Controller orientation from the gyro: a body-frame track per IMU stream, and a filter that keeps
//! an engine pose only when the controller's other poses in the same sensor agree with it through
//! the gyro. In capture mode the engine searches every frame blind (no sensor poses, so no
//! prediction), and many of its poses are self-consistent flips or mislabels; the gyro is the
//! arbiter that does not depend on the vision at all.

use std::collections::BTreeMap;

use nalgebra::{UnitQuaternion, Vector3};

use super::capture::{ImuSample, WandSample};

/// Body orientation over an IMU stream, relative to its first sample (trapezoidal integration).
pub struct Track {
    times: Vec<u64>,
    q: Vec<UnitQuaternion<f64>>,
    gyro: Vec<Vector3<f64>>,
}

/// Gaps longer than this are bridged with the rate before the gap rather than integrated blindly.
const MAX_STEP_S: f64 = 0.05;

impl Track {
    pub fn new(imu: &[ImuSample]) -> Track {
        let mut q = Vec::with_capacity(imu.len());
        let mut cur = UnitQuaternion::identity();
        for (k, s) in imu.iter().enumerate() {
            if k > 0 {
                let prev = &imu[k - 1];
                let dt = ((s.device_time_ns - prev.device_time_ns) as f64 * 1e-9).min(MAX_STEP_S);
                cur *= UnitQuaternion::from_scaled_axis((prev.gyro + s.gyro) * (0.5 * dt));
            }
            q.push(cur);
        }
        Track {
            times: imu.iter().map(|s| s.device_time_ns).collect(),
            q,
            gyro: imu.iter().map(|s| s.gyro).collect(),
        }
    }

    /// Orientation at device time `t` (body -> track frame); None outside the stream.
    pub fn at(&self, t: u64) -> Option<UnitQuaternion<f64>> {
        let (&first, &last) = (self.times.first()?, self.times.last()?);
        if t < first || t > last {
            return None;
        }
        let k = self.times.partition_point(|&x| x <= t) - 1;
        if self.times[k] == t || k + 1 == self.times.len() {
            return Some(self.q[k]);
        }
        // linear rate between the two samples, integrated to t
        let span = (self.times[k + 1] - self.times[k]) as f64 * 1e-9;
        let dt = (t - self.times[k]) as f64 * 1e-9;
        let rate_t = self.gyro[k] + (self.gyro[k + 1] - self.gyro[k]) * (dt / span);
        Some(
            self.q[k]
                * UnitQuaternion::from_scaled_axis(
                    (self.gyro[k] + rate_t) * (0.5 * dt.min(MAX_STEP_S)),
                ),
        )
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FilterParams {
    /// Largest disagreement (radians) for two poses to support each other.
    pub tolerance_rad: f64,
    /// Poses within this much device time of each other are compared.
    pub half_window_ns: u64,
    /// Fewest supporting poses.
    pub min_support: usize,
    /// A pose's agreeing fraction must reach this share of the best fraction within half the window.
    pub min_support_fraction: f64,
    /// Only poses at least this much body rotation away count as evidence: a pose compared with
    /// itself, unrotated, says nothing (a still controller's mislabel agrees with itself).
    pub min_rotation_rad: f64,
}

impl Default for FilterParams {
    fn default() -> Self {
        FilterParams {
            tolerance_rad: 5f64.to_radians(),
            half_window_ns: 1_000_000_000,
            min_support: 3,
            min_support_fraction: 0.7,
            min_rotation_rad: 10f64.to_radians(),
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FilterStats {
    pub kept: usize,
    pub dropped: usize,
}

/// Drops posed sightings the gyro contradicts (see the module doc), then samples left without
/// sightings. Sightings without an engine pose, and controllers without an IMU stream, are kept.
pub fn filter_views(
    samples: &mut Vec<WandSample>,
    imu: &BTreeMap<i32, Vec<ImuSample>>,
    p: &FilterParams,
) -> FilterStats {
    let mut stats = FilterStats::default();
    let cameras = samples
        .iter()
        .flat_map(|s| s.views.iter().map(|v| v.camera + 1))
        .max()
        .unwrap_or(0);
    let mut drop: Vec<(usize, usize)> = Vec::new();
    for (&device, stream) in imu {
        let track = Track::new(stream);
        for camera in 0..cameras {
            // (sample, view, time, camera-from-track orientation, track orientation): the third is
            // constant while the pose is right
            type OrientedSighting = (usize, usize, u64, UnitQuaternion<f64>, UnitQuaternion<f64>);
            let seq: Vec<OrientedSighting> = samples
                .iter()
                .enumerate()
                .filter(|(_, s)| s.device_id == device)
                .flat_map(|(i, s)| {
                    s.views
                        .iter()
                        .enumerate()
                        .filter(|(_, v)| v.camera == camera)
                        .map(move |(k, v)| (i, k, s.device_time_ns, v))
                })
                .filter_map(|(i, k, t, v)| {
                    let q = track.at(t)?;
                    Some((i, k, t, v.cam_from_obj?.rotation * q.inverse(), q))
                })
                .collect();
            // (agreeing, compared) among the rotated poses in each pose's own window: the fraction is
            // what neighbours compare, so a pose next to a coverage gap is not outvoted by a fuller window
            let support: Vec<(usize, usize)> = seq
                .iter()
                .map(|&(_, _, t, a, q)| {
                    let start = seq.partition_point(|x| x.2 < t.saturating_sub(p.half_window_ns));
                    let window = seq[start..]
                        .iter()
                        .take_while(|x| x.2 <= t + p.half_window_ns)
                        .filter(|x| x.4.angle_to(&q) >= p.min_rotation_rad);
                    window.fold((0, 0), |(n, all), x| {
                        (n + (x.3.angle_to(&a) < p.tolerance_rad) as usize, all + 1)
                    })
                })
                .collect();
            let frac = |m: usize| {
                if support[m].1 == 0 {
                    0.0
                } else {
                    support[m].0 as f64 / support[m].1 as f64
                }
            };
            for (j, &(i, k, t, _, _)) in seq.iter().enumerate() {
                let start = seq.partition_point(|x| x.2 < t.saturating_sub(p.half_window_ns / 2));
                let best = (start..seq.len())
                    .take_while(|&m| seq[m].2 <= t + p.half_window_ns / 2)
                    .map(frac)
                    .fold(0.0, f64::max);
                if support[j].0 >= p.min_support && frac(j) >= p.min_support_fraction * best {
                    stats.kept += 1;
                } else {
                    drop.push((i, k));
                }
            }
        }
    }
    stats.dropped = drop.len();
    drop.sort_unstable();
    for &(i, k) in drop.iter().rev() {
        samples[i].views.remove(k);
    }
    samples.retain(|s| !s.views.is_empty());
    stats
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::{self, TrajectoryParams};

    fn spin(rate: Vector3<f64>, hz: f64, secs: f64) -> Vec<ImuSample> {
        let n = (hz * secs) as u64 + 1;
        (0..n)
            .map(|i| ImuSample {
                device_time_ns: 1_000_000_000 + (i as f64 / hz * 1e9) as u64,
                accel: Vector3::zeros(),
                gyro: rate,
            })
            .collect()
    }

    #[test]
    fn track_integrates_a_constant_rate() {
        let rate = Vector3::new(0.0, 0.0, 2.0);
        let track = Track::new(&spin(rate, 500.0, 2.0));
        let q = track.at(1_000_000_000 + 1_000_000_000).unwrap();
        assert!(
            (q.scaled_axis() - rate).norm() < 1e-9,
            "{:?}",
            q.scaled_axis()
        );
        // between samples
        let q = track.at(1_000_000_000 + 1_001_000).unwrap();
        assert!((q.scaled_axis() - rate * 0.001001).norm() < 1e-9);
        assert!(track.at(999_999_999).is_none());
        assert!(track.at(3_000_000_001).is_none());
    }

    /// Integrating the generator's gyro reproduces the true relative orientation over a second.
    #[test]
    fn track_follows_a_trajectory() {
        let tr = synth::trajectory(&TrajectoryParams {
            duration_s: 3.0,
            ..Default::default()
        });
        let track = Track::new(&tr.imu[&crate::calib::DEVICE_RIGHT]);
        let m = &tr.motions[0];
        let (a, b) = (
            track.at(1_500_000_000).unwrap(),
            track.at(2_500_000_000).unwrap(),
        );
        let truth = m.pose(0.5).rotation.inverse() * m.pose(1.5).rotation;
        assert!(
            (a.inverse() * b).angle_to(&truth) < 2e-4,
            "{}",
            (a.inverse() * b).angle_to(&truth)
        );
    }

    fn is_flipped(tr: &synth::TrajectoryScene, i: usize, v: usize) -> bool {
        let s = &tr.scene.samples[i];
        let view = &s.views[v];
        let truth = tr.scene.world_from_cam[view.camera].inverse() * tr.scene.world_from_obj[i];
        view.cam_from_obj
            .unwrap()
            .rotation
            .angle_to(&truth.rotation)
            > 30f64.to_radians()
    }

    /// Random wrong-but-confident sightings are dropped; true ones survive.
    #[test]
    fn filter_drops_flipped_sightings() {
        let tr = synth::trajectory(&TrajectoryParams {
            duration_s: 20.0,
            flipped_fraction: 0.15,
            ..Default::default()
        });
        let mut flipped = 0;
        let mut good = 0;
        for (i, s) in tr.scene.samples.iter().enumerate() {
            for v in 0..s.views.len() {
                if is_flipped(&tr, i, v) {
                    flipped += 1
                } else {
                    good += 1
                }
            }
        }
        assert!(flipped > 300, "{flipped}");
        let mut samples = tr.scene.samples.clone();
        let stats = filter_views(&mut samples, &tr.imu, &FilterParams::default());
        // what survived, classified against the truth by time and sensor
        let (mut kept_flipped, mut kept_good) = (0, 0);
        for s in &samples {
            let i = tr
                .scene
                .samples
                .iter()
                .position(|o| o.device_id == s.device_id && o.device_time_ns == s.device_time_ns)
                .unwrap();
            for v in &s.views {
                let k = tr.scene.samples[i]
                    .views
                    .iter()
                    .position(|o| o.camera == v.camera)
                    .unwrap();
                if is_flipped(&tr, i, k) {
                    kept_flipped += 1
                } else {
                    kept_good += 1
                }
            }
        }
        assert_eq!(stats.kept, kept_flipped + kept_good);
        assert_eq!(stats.kept + stats.dropped, flipped + good);
        assert!(
            kept_flipped * 50 < flipped,
            "kept {kept_flipped} of {flipped} flipped"
        );
        assert!(
            kept_good * 100 > good * 97,
            "kept {kept_good} of {good} good"
        );
    }

    /// A persistent wrong labelling (the same flip for a whole second in one sensor) rotates
    /// against the gyro while the controller turns, and is dropped.
    #[test]
    fn filter_drops_a_persistent_mislabel() {
        let tr = synth::trajectory(&TrajectoryParams {
            duration_s: 10.0,
            pnp_rot_noise_rad: 0.01,
            ..Default::default()
        });
        let mut samples = tr.scene.samples.clone();
        let flip = UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.0, 1.6));
        let (from, to) = (4_000_000_000u64, 5_000_000_000u64);
        let mut injected = 0;
        for s in samples.iter_mut().filter(|s| {
            s.device_id == crate::calib::DEVICE_RIGHT
                && s.device_time_ns >= from
                && s.device_time_ns < to
        }) {
            for v in s.views.iter_mut().filter(|v| v.camera == 0) {
                let t = v.cam_from_obj.unwrap();
                v.cam_from_obj = Some(nalgebra::Isometry3::from_parts(
                    t.translation,
                    t.rotation * flip,
                ));
                injected += 1;
            }
        }
        assert!(injected > 20, "{injected}");
        filter_views(&mut samples, &tr.imu, &FilterParams::default());
        let left = samples
            .iter()
            .filter(|s| {
                s.device_id == crate::calib::DEVICE_RIGHT
                    && s.device_time_ns >= from
                    && s.device_time_ns < to
            })
            .flat_map(|s| s.views.iter())
            .filter(|v| v.camera == 0)
            .count();
        assert!(left * 10 < injected, "{left} of {injected} survived");
    }

    /// A controller lying still gives the gyro nothing to check: a static pose agrees with itself
    /// whether it is right or a mislabel, and a long rest would outvote the moving data. Such
    /// sightings are unverifiable and dropped.
    #[test]
    fn sightings_of_a_still_controller_are_unverifiable() {
        let imu = BTreeMap::from([(
            crate::calib::DEVICE_RIGHT,
            spin(Vector3::zeros(), 500.0, 4.0),
        )]);
        let pose = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(0.1, 0.2, 2.5),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.3, 0.1, 0.0)),
        );
        let view = crate::calib::capture::View {
            camera: 0,
            cam_from_obj: Some(pose),
            strong: true,
            obs: Vec::new(),
        };
        let mut samples: Vec<WandSample> = (0..150u64)
            .map(|i| WandSample {
                device_id: crate::calib::DEVICE_RIGHT,
                device_time_ns: 1_200_000_000 + i * 19_230_769,
                host_ns: 0,
                views: vec![view.clone()],
            })
            .collect();
        let stats = filter_views(&mut samples, &imu, &FilterParams::default());
        assert_eq!(stats.kept, 0);
        assert!(samples.is_empty());
    }

    /// Review fix 3: a wrong cluster that agrees with itself through the gyro (here a constant
    /// camera-frame offset on every third sighting) has support of its own; only its smaller share
    /// of the window gives it away.
    #[test]
    fn a_self_consistent_minority_cluster_is_dropped() {
        let tr = synth::trajectory(&TrajectoryParams {
            duration_s: 10.0,
            pnp_rot_noise_rad: 0.005,
            ..Default::default()
        });
        let mut samples = tr.scene.samples.clone();
        let offset = UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.5, 0.0));
        let mut injected = Vec::new();
        for s in samples
            .iter_mut()
            .filter(|s| s.device_id == crate::calib::DEVICE_RIGHT)
        {
            for v in s.views.iter_mut().filter(|v| v.camera == 1) {
                if (s.device_time_ns / 19_230_769) % 3 == 0 {
                    let t = v.cam_from_obj.unwrap();
                    v.cam_from_obj = Some(nalgebra::Isometry3::from_parts(
                        t.translation,
                        offset * t.rotation,
                    ));
                    injected.push(s.device_time_ns);
                }
            }
        }
        assert!(injected.len() > 40, "{}", injected.len());
        filter_views(&mut samples, &tr.imu, &FilterParams::default());
        let survived = samples
            .iter()
            .filter(|s| {
                s.device_id == crate::calib::DEVICE_RIGHT && injected.contains(&s.device_time_ns)
            })
            .filter(|s| s.views.iter().any(|v| v.camera == 1))
            .count();
        assert!(
            survived * 10 < injected.len(),
            "{survived} of {} survived",
            injected.len()
        );
    }

    #[test]
    fn controllers_without_an_imu_stream_are_left_alone() {
        let tr = synth::trajectory(&TrajectoryParams {
            duration_s: 2.0,
            flipped_fraction: 0.2,
            ..Default::default()
        });
        let mut samples = tr.scene.samples.clone();
        let stats = filter_views(&mut samples, &BTreeMap::new(), &FilterParams::default());
        assert_eq!(samples, tr.scene.samples);
        assert_eq!(stats.dropped, 0);
    }
}
