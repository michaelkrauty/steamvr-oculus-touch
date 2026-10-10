// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Gravity from motion. While a controller is tracked continuously, its camera-measured positions
//! must match its accelerometer integrated twice, rotated by its orientation, minus gravity:
//!
//!   p_k - p_0 + (R_k - R_0) r = v0 t_k + D_k - B_k b - G t_k^2 / 2
//!
//! with D_k the double integral of R f, B_k the double integral of R, b the accelerometer bias and
//! r the IMU's position in the LED model frame. Linear in v0 (per segment), G (shared) and b, r
//! (per controller); least squares with Huber reweighting, v0 eliminated per segment. Orientation
//! R(t) = A Q(t): the gyro track anchored per segment by the chordal mean of its camera
//! orientations. Needs no still moments: every orientation the controllers pass through counts.

use std::collections::BTreeMap;

use nalgebra::{DMatrix, DVector, Isometry3, Matrix3, Rotation3, UnitQuaternion, Vector3};

use super::capture::{ImuSample, WandSample};
use super::gyro::Track;

#[derive(Debug, Clone, Copy)]
pub struct MotionParams {
    /// Consecutive sightings further apart than this end a segment.
    pub max_gap_ns: u64,
    /// Segments are cut at this length: orientation error integrates into position quadratically.
    pub max_segment_ns: u64,
    pub min_segment_ns: u64,
    pub min_samples: usize,
    /// Fewest segments for an estimate (per controller for its own estimate).
    pub min_segments: usize,
    /// Position noise of a sighting (m), and the Huber threshold (m).
    pub sigma_m: f64,
    pub huber_m: f64,
    pub iterations: usize,
    /// Weak priors keeping the bias and lever arm bounded when the motion does not determine them.
    pub prior_bias: f64,
    pub prior_lever_m: f64,
}

impl Default for MotionParams {
    fn default() -> Self {
        MotionParams {
            max_gap_ns: 60_000_000,
            max_segment_ns: 400_000_000,
            min_segment_ns: 150_000_000,
            min_samples: 8,
            min_segments: 20,
            sigma_m: 0.002,
            huber_m: 0.005,
            iterations: 8,
            prior_bias: 1.0,
            prior_lever_m: 0.05,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DeviceFit {
    pub device: i32,
    pub segments: usize,
    pub accel_bias: Vector3<f64>,
    pub lever_arm: Vector3<f64>,
}

/// Which segments an independent gravity estimate came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Subset {
    Device(i32),
    FirstHalf(i32),
    SecondHalf(i32),
}

#[derive(Debug, Clone, PartialEq)]
pub struct MotionGravity {
    /// g * up, in the frame of the given poses.
    pub gravity: Vector3<f64>,
    pub segments: usize,
    pub residual_median_m: f64,
    pub residual_p90_m: f64,
    pub devices: Vec<DeviceFit>,
    /// Estimates from disjoint sets of segments: each controller alone, or with one controller its
    /// first and second halves. The cross-check.
    pub independent: Vec<(Subset, Vector3<f64>)>,
}

impl MotionGravity {
    /// Largest angle (degrees) between two independent estimates; None without a pair.
    pub fn spread_deg(&self) -> Option<f64> {
        let g = &self.independent;
        (0..g.len())
            .flat_map(|i| (i + 1..g.len()).map(move |j| g[i].1.angle(&g[j].1).to_degrees()))
            .reduce(f64::max)
    }
}

/// `ref_from_obj[i]` is sample i's controller pose (None: not posed).
pub fn estimate(
    samples: &[WandSample],
    ref_from_obj: &[Option<Isometry3<f64>>],
    imu: &BTreeMap<i32, Vec<ImuSample>>,
    p: &MotionParams,
) -> Result<MotionGravity, String> {
    let mut segs: Vec<Segment> = Vec::new();
    let mut devices: Vec<i32> = Vec::new();
    for (&device, stream) in imu {
        let found = segments(samples, ref_from_obj, device, stream, devices.len(), p);
        if !found.is_empty() {
            devices.push(device);
            segs.extend(found);
        }
    }
    if segs.len() < p.min_segments {
        return Err(format!(
            "only {} tracked segments of motion (need {}): wave the controllers for longer, turning them, where the sensors can see them",
            segs.len(),
            p.min_segments
        ));
    }
    let (x, residuals) = solve(&segs, devices.len(), p);
    let mut sorted = residuals.clone();
    sorted.sort_by(f64::total_cmp);
    let count = |d: usize| segs.iter().filter(|s| s.dev == d).count();
    let fits: Vec<DeviceFit> = devices
        .iter()
        .enumerate()
        .map(|(d, &device)| DeviceFit {
            device,
            segments: count(d),
            accel_bias: block(&x, 3 + 6 * d),
            lever_arm: block(&x, 6 + 6 * d),
        })
        .collect();
    let alone = |own: Vec<&Segment>| {
        let own: Vec<Segment> = own
            .into_iter()
            .map(|s| Segment {
                dev: 0,
                t0: s.t0,
                rows: s.rows.clone(),
            })
            .collect();
        block(&solve(&own, 1, p).0, 0)
    };
    let qualified: Vec<usize> = (0..devices.len())
        .filter(|&d| count(d) >= p.min_segments)
        .collect();
    let independent = if qualified.len() >= 2 {
        qualified
            .iter()
            .map(|&d| {
                (
                    Subset::Device(devices[d]),
                    alone(segs.iter().filter(|s| s.dev == d).collect()),
                )
            })
            .collect()
    } else {
        let d = (0..devices.len()).max_by_key(|&d| count(d)).unwrap_or(0);
        let mut own: Vec<&Segment> = segs.iter().filter(|s| s.dev == d).collect();
        own.sort_by_key(|s| s.t0);
        if own.len() >= 2 * p.min_segments {
            let second = own.split_off(own.len() / 2);
            vec![
                (Subset::FirstHalf(devices[d]), alone(own)),
                (Subset::SecondHalf(devices[d]), alone(second)),
            ]
        } else {
            Vec::new()
        }
    };
    Ok(MotionGravity {
        independent,
        gravity: block(&x, 0),
        segments: segs.len(),
        residual_median_m: sorted[sorted.len() / 2],
        residual_p90_m: sorted[sorted.len() * 9 / 10],
        devices: fits,
    })
}

#[derive(Debug, Clone)]
struct Row {
    tau: f64,
    /// Double integrals of R f and of R from the segment start.
    d: Vector3<f64>,
    b: Matrix3<f64>,
    /// R_k - R_0 (lever arm) and the measured displacement p_k - p_0.
    dr: Matrix3<f64>,
    y: Vector3<f64>,
}

struct Segment {
    dev: usize,
    /// Device time of the segment's first sighting.
    t0: u64,
    rows: Vec<Row>,
}

fn block(x: &DVector<f64>, at: usize) -> Vector3<f64> {
    Vector3::new(x[at], x[at + 1], x[at + 2])
}

/// Accelerometer reading at `t`, linearly interpolated.
fn accel_at(imu: &[ImuSample], t: u64) -> Option<Vector3<f64>> {
    let k = imu.partition_point(|s| s.device_time_ns <= t);
    if k == 0 || k == imu.len() {
        return (k > 0 && imu[k - 1].device_time_ns == t).then(|| imu[k - 1].accel);
    }
    let (a, b) = (&imu[k - 1], &imu[k]);
    let f = (t - a.device_time_ns) as f64 / (b.device_time_ns - a.device_time_ns) as f64;
    Some(a.accel + (b.accel - a.accel) * f)
}

/// One controller's tracked stretches, cut into segments, each integrated from its first sighting.
fn segments(
    samples: &[WandSample],
    ref_from_obj: &[Option<Isometry3<f64>>],
    device: i32,
    imu: &[ImuSample],
    dev: usize,
    p: &MotionParams,
) -> Vec<Segment> {
    let track = Track::new(imu);
    let mut seq: Vec<(u64, Isometry3<f64>)> = samples
        .iter()
        .zip(ref_from_obj)
        .filter(|(s, w)| s.device_id == device && w.is_some())
        .map(|(s, w)| (s.device_time_ns, w.unwrap()))
        .collect();
    seq.sort_by_key(|x| x.0);
    seq.dedup_by_key(|x| x.0);
    let mut out = Vec::new();
    let mut start = 0;
    while start < seq.len() {
        let mut end = start + 1;
        while end < seq.len()
            && seq[end].0 - seq[end - 1].0 <= p.max_gap_ns
            && seq[end].0 - seq[start].0 <= p.max_segment_ns
        {
            end += 1;
        }
        let seg = &seq[start..end];
        start = end;
        if seg.len() < p.min_samples || seg[seg.len() - 1].0 - seg[0].0 < p.min_segment_ns {
            continue;
        }
        if let Some(rows) = integrate(seg, &track, imu) {
            out.push(Segment {
                dev,
                t0: seg[0].0,
                rows,
            });
        }
    }
    out
}

fn integrate(seg: &[(u64, Isometry3<f64>)], track: &Track, imu: &[ImuSample]) -> Option<Vec<Row>> {
    let anchors: Vec<UnitQuaternion<f64>> = seg
        .iter()
        .map(|(t, w)| Some(w.rotation * track.at(*t)?.inverse()))
        .collect::<Option<_>>()?;
    let a = chordal_mean(&anchors);
    let rot = |t: u64| -> Option<Matrix3<f64>> {
        Some((a * track.at(t)?).to_rotation_matrix().into_inner())
    };
    let (t0, p0) = (seg[0].0, seg[0].1.translation.vector);
    let r0 = rot(t0)?;
    let (mut v, mut pos, mut vb, mut pb) = (
        Vector3::zeros(),
        Vector3::zeros(),
        Matrix3::zeros(),
        Matrix3::zeros(),
    );
    let mut ta = t0;
    let mut rows = Vec::with_capacity(seg.len() - 1);
    for &(tk, w) in &seg[1..] {
        while ta < tk {
            let next = imu.partition_point(|s| s.device_time_ns <= ta);
            let tb = imu.get(next).map_or(tk, |s| s.device_time_ns.min(tk));
            let mid = ta + (tb - ta) / 2;
            let dt = (tb - ta) as f64 * 1e-9;
            let r = rot(mid)?;
            let acc = r * accel_at(imu, mid)?;
            pos += v * dt + acc * (0.5 * dt * dt);
            v += acc * dt;
            pb += vb * dt + r * (0.5 * dt * dt);
            vb += r * dt;
            ta = tb;
        }
        rows.push(Row {
            tau: (tk - t0) as f64 * 1e-9,
            d: pos,
            b: pb,
            dr: rot(tk)? - r0,
            y: w.translation.vector - p0,
        });
    }
    Some(rows)
}

/// Global unknowns: G (3), then per controller accel bias (3) and lever arm (3). Returns them and
/// every row's residual norm (m).
fn solve(segs: &[Segment], ndev: usize, p: &MotionParams) -> (DVector<f64>, Vec<f64>) {
    let ng = 3 + 6 * ndev;
    let jacobian = |row: &Row, dev: usize| {
        let mut j = DMatrix::<f64>::zeros(3, ng);
        for c in 0..3 {
            j[(c, c)] = -0.5 * row.tau * row.tau;
        }
        j.view_mut((0, 3 + 6 * dev), (3, 3)).copy_from(&(-row.b));
        j.view_mut((0, 6 + 6 * dev), (3, 3)).copy_from(&(-(row.dr)));
        j
    };
    let rhs = |row: &Row| row.y - row.d;
    let mut weights: Vec<Vec<f64>> = segs.iter().map(|s| vec![1.0; s.rows.len()]).collect();
    let mut x = DVector::zeros(ng);
    let mut residuals = Vec::new();
    for _ in 0..p.iterations.max(1) {
        let mut hgg = DMatrix::<f64>::zeros(ng, ng);
        let mut bg = DVector::<f64>::zeros(ng);
        let mut per_seg = Vec::with_capacity(segs.len());
        for (s, ws) in segs.iter().zip(&weights) {
            let (mut hvv, mut hvg, mut bv) = (0.0, DMatrix::<f64>::zeros(3, ng), Vector3::zeros());
            for (row, wt) in s.rows.iter().zip(ws) {
                let w = wt / (p.sigma_m * p.sigma_m);
                let (j, y) = (jacobian(row, s.dev), rhs(row));
                hvv += w * row.tau * row.tau;
                hvg += &j * (w * row.tau);
                bv += y * (w * row.tau);
                hgg += j.transpose() * &j * w;
                bg += j.transpose() * DVector::from_column_slice(y.as_slice()) * w;
            }
            hgg -= hvg.transpose() * &hvg / hvv;
            bg -= hvg.transpose() * DVector::from_column_slice(bv.as_slice()) / hvv;
            per_seg.push((hvv, hvg, bv));
        }
        for d in 0..ndev {
            for c in 0..3 {
                hgg[(3 + 6 * d + c, 3 + 6 * d + c)] += 1.0 / (p.prior_bias * p.prior_bias);
                hgg[(6 + 6 * d + c, 6 + 6 * d + c)] += 1.0 / (p.prior_lever_m * p.prior_lever_m);
            }
        }
        x = match hgg.clone().cholesky() {
            Some(c) => c.solve(&bg),
            None => hgg.lu().solve(&bg).unwrap_or(x),
        };
        residuals.clear();
        for ((s, ws), (hvv, hvg, bv)) in segs.iter().zip(weights.iter_mut()).zip(&per_seg) {
            let v0 = (bv - Vector3::from_column_slice((hvg * &x).as_slice())) / *hvv;
            for (row, wt) in s.rows.iter().zip(ws.iter_mut()) {
                let e = v0 * row.tau
                    + Vector3::from_column_slice((jacobian(row, s.dev) * &x).as_slice())
                    - rhs(row);
                let n = e.norm();
                *wt = if n <= p.huber_m { 1.0 } else { p.huber_m / n };
                residuals.push(n);
            }
        }
    }
    (x, residuals)
}

/// The rotation nearest (Frobenius) to the mean of the rotation matrices.
pub fn chordal_mean(rs: &[UnitQuaternion<f64>]) -> UnitQuaternion<f64> {
    let m: Matrix3<f64> = rs.iter().map(|r| r.to_rotation_matrix().into_inner()).sum();
    let svd = m.svd(true, true);
    let (u, vt) = (svd.u.unwrap(), svd.v_t.unwrap());
    let mut d = Matrix3::identity();
    if (u * vt).determinant() < 0.0 {
        d[(2, 2)] = -1.0;
    }
    UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(u * d * vt))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::{self, Rng, TrajectoryParams, TrajectoryScene};
    use crate::calib::{DEVICE_LEFT, DEVICE_RIGHT};

    fn poses(tr: &TrajectoryScene) -> Vec<Option<Isometry3<f64>>> {
        tr.scene.world_from_obj.iter().copied().map(Some).collect()
    }

    fn check_devices(m: &MotionGravity, p: &TrajectoryParams, bias_tol: f64, lever_tol: f64) {
        for (k, d) in [DEVICE_RIGHT, DEVICE_LEFT].iter().enumerate() {
            let fit = m.devices.iter().find(|f| f.device == *d).unwrap();
            assert!(
                (fit.accel_bias - p.accel_bias[k]).norm() < bias_tol,
                "device {d} bias {:?}",
                fit.accel_bias
            );
            assert!(
                (fit.lever_arm - p.lever_arm[k]).norm() < lever_tol,
                "device {d} lever {:?}",
                fit.lever_arm
            );
        }
    }

    /// Exact poses: gravity, both accelerometer biases and both IMU lever arms come back.
    #[test]
    fn recovers_gravity_bias_and_lever_arm_from_exact_motion() {
        let p = TrajectoryParams {
            duration_s: 30.0,
            gravity: Vector3::new(0.3, 9.79, -0.4),
            ..Default::default()
        };
        let tr = synth::trajectory(&p);
        let m = estimate(
            &tr.scene.samples,
            &poses(&tr),
            &tr.imu,
            &MotionParams::default(),
        )
        .unwrap();
        assert!((m.gravity - p.gravity).norm() < 2e-3, "{:?}", m.gravity);
        assert!(m.residual_median_m < 1e-4, "{}", m.residual_median_m);
        check_devices(&m, &p, 0.01, 0.001);
        assert!(m.spread_deg().unwrap() < 0.02, "{:?}", m.spread_deg());
    }

    /// Sightings carry ~1 mm / ~0.3 deg of independent pose noise: the up direction stays within
    /// a tenth of a degree.
    #[test]
    fn tolerates_pose_noise() {
        let p = TrajectoryParams {
            duration_s: 40.0,
            ..Default::default()
        };
        let tr = synth::trajectory(&p);
        let mut rng = Rng::new(3);
        let noisy: Vec<Option<Isometry3<f64>>> = tr
            .scene
            .world_from_obj
            .iter()
            .map(|w| {
                Some(Isometry3::from_parts(
                    (w.translation.vector + rng.gauss3(0.001)).into(),
                    w.rotation * UnitQuaternion::from_scaled_axis(rng.gauss3(0.005)),
                ))
            })
            .collect();
        let m = estimate(&tr.scene.samples, &noisy, &tr.imu, &MotionParams::default()).unwrap();
        let err = m.gravity.angle(&p.gravity).to_degrees();
        assert!(err < 0.1, "{err} deg");
        assert!(
            (m.gravity.norm() - p.gravity.norm()).abs() < 0.05,
            "{}",
            m.gravity.norm()
        );
        assert!(m.spread_deg().unwrap() < 0.2, "{:?}", m.spread_deg());
    }

    /// A stretch of wrong positions (a mislabelled run the vision rejection missed) is outvoted.
    #[test]
    fn a_corrupted_stretch_is_outvoted() {
        let p = TrajectoryParams {
            duration_s: 30.0,
            ..Default::default()
        };
        let tr = synth::trajectory(&p);
        let mut ps = poses(&tr);
        for (i, s) in tr.scene.samples.iter().enumerate() {
            if s.device_time_ns > 5_000_000_000 && s.device_time_ns < 7_000_000_000 {
                let w = ps[i].unwrap();
                ps[i] = Some(Isometry3::from_parts(
                    (w.translation.vector + Vector3::new(0.08, -0.05, 0.02)).into(),
                    w.rotation,
                ));
            }
        }
        let m = estimate(&tr.scene.samples, &ps, &tr.imu, &MotionParams::default()).unwrap();
        assert!(
            m.gravity.angle(&p.gravity).to_degrees() < 0.05,
            "{}",
            m.gravity.angle(&p.gravity).to_degrees()
        );
    }

    /// Too little tracked motion: say what to do instead of guessing.
    #[test]
    fn too_little_motion_is_an_error() {
        let tr = synth::trajectory(&TrajectoryParams {
            duration_s: 1.5,
            ..Default::default()
        });
        let err = estimate(
            &tr.scene.samples,
            &poses(&tr),
            &tr.imu,
            &MotionParams::default(),
        )
        .unwrap_err();
        assert!(err.contains("segments") && err.contains("wave"), "{err}");
    }

    /// One controller alone still gives an estimate; the other has none to compare.
    #[test]
    fn one_controller_is_enough() {
        let p = TrajectoryParams {
            duration_s: 30.0,
            ..Default::default()
        };
        let tr = synth::trajectory(&p);
        let right: BTreeMap<i32, Vec<ImuSample>> = tr
            .imu
            .iter()
            .filter(|(d, _)| **d == DEVICE_RIGHT)
            .map(|(d, s)| (*d, s.clone()))
            .collect();
        let m = estimate(
            &tr.scene.samples,
            &poses(&tr),
            &right,
            &MotionParams::default(),
        )
        .unwrap();
        assert!((m.gravity - p.gravity).norm() < 2e-3);
        assert_eq!(m.devices.len(), 1);
    }

    #[test]
    fn chordal_mean_of_nearby_rotations() {
        let a = UnitQuaternion::from_scaled_axis(Vector3::new(0.1, 0.2, 0.3));
        let rs = [
            a * UnitQuaternion::from_scaled_axis(Vector3::new(0.01, 0.0, 0.0)),
            a * UnitQuaternion::from_scaled_axis(Vector3::new(-0.01, 0.0, 0.0)),
        ];
        assert!(chordal_mean(&rs).angle_to(&a) < 1e-4);
    }
}
