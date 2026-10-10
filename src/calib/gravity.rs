// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! The world's up from static holds. Over holds in many orientations a Touch's accelerometer
//! calibrates itself (every hold reads the same |g|), and then measures the direction of gravity
//! in the controller frame to about 0.1 degree. The tracked orientation carries that into the
//! world; where it disagrees with +Y on average, the calibration's world is tilted.

use nalgebra::{DMatrix, DVector, Matrix3, UnitQuaternion, Vector3};

pub const STANDARD_GRAVITY: f64 = 9.80665;

/// An accelerometer's correction from the holds alone: calibrated = d * (raw - b).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccelCal {
    /// Symmetric: per-axis scale and cross-axis coupling.
    pub d: Matrix3<f64>,
    pub b: Vector3<f64>,
    /// rms of |calibrated| - g over the holds, m/s^2.
    pub rms: f64,
}

impl AccelCal {
    pub fn apply(&self, raw: &Vector3<f64>) -> Vector3<f64> {
        self.d * (raw - self.b)
    }
}

/// Fits `AccelCal` to each hold's mean accelerometer reading so that every calibrated reading
/// has magnitude `g`. None with fewer than 12 holds, or when the holds do not span enough
/// orientations to separate the bias from the scale.
pub fn accel_cal(holds: &[Vector3<f64>], g: f64) -> Option<AccelCal> {
    if holds.len() < 12 {
        return None;
    }
    // the directions must fill all three axes, or the bias trades against the scale
    let dirs: Matrix3<f64> = holds
        .iter()
        .map(|a| a.normalize() * a.normalize().transpose())
        .sum::<Matrix3<f64>>()
        / holds.len() as f64;
    if dirs.symmetric_eigenvalues().min() < 0.05 {
        return None;
    }
    let unpack = |x: &[f64; 9]| {
        (
            Matrix3::new(x[0], x[3], x[4], x[3], x[1], x[5], x[4], x[5], x[2]),
            Vector3::new(x[6], x[7], x[8]),
        )
    };
    let mut x = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let residual = |x: &[f64; 9], a: &Vector3<f64>| {
        let (d, b) = unpack(x);
        (d * (a - b)).norm() - g
    };
    for _ in 0..20 {
        let mut jm = DMatrix::<f64>::zeros(holds.len(), 9);
        let mut r = DVector::<f64>::zeros(holds.len());
        for (k, a) in holds.iter().enumerate() {
            r[k] = residual(&x, a);
            for i in 0..9 {
                let mut y = x;
                y[i] += 1e-7;
                jm[(k, i)] = (residual(&y, a) - r[k]) / 1e-7;
            }
        }
        let step = (jm.transpose() * &jm)
            .cholesky()?
            .solve(&(jm.transpose() * -r));
        for i in 0..9 {
            x[i] += step[i];
        }
        if step.norm() < 1e-12 {
            break;
        }
    }
    let (d, b) = unpack(&x);
    let rms =
        (holds.iter().map(|a| residual(&x, a).powi(2)).sum::<f64>() / holds.len() as f64).sqrt();
    Some(AccelCal { d, b, rms })
}

/// The correction to the world, as a rotation vector with no component about +Y: the smallest
/// rotation taking the mean measured up, `world_from_body[k] * up_body[k]`, onto +Y. Corrected
/// world poses are this rotation times the old ones. With it, each hold's remaining angle (rad).
pub fn world_tilt(
    world_from_body: &[UnitQuaternion<f64>],
    up_body: &[Vector3<f64>],
) -> (Vector3<f64>, Vec<f64>) {
    let up: Vec<Vector3<f64>> = world_from_body
        .iter()
        .zip(up_body)
        .map(|(q, u)| q * u.normalize())
        .collect();
    let mean = up.iter().sum::<Vector3<f64>>();
    let y = Vector3::y();
    let correction =
        UnitQuaternion::rotation_between(&mean, &y).map_or(Vector3::zeros(), |c| c.scaled_axis());
    let c = UnitQuaternion::from_scaled_axis(correction);
    (
        Vector3::new(correction.x, 0.0, correction.z),
        up.iter().map(|u| (c * u).angle(&y)).collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::synth::Rng;

    fn random_rotation(rng: &mut Rng) -> UnitQuaternion<f64> {
        let v = rng.gauss3(1.0);
        UnitQuaternion::from_scaled_axis(v.normalize() * (rng.uniform() * std::f64::consts::PI))
    }

    /// 30 holds in random orientations through a biased, mis-scaled accelerometer.
    fn holds(
        rng: &mut Rng,
        d_true: &Matrix3<f64>,
        b_true: &Vector3<f64>,
        noise: f64,
    ) -> (Vec<UnitQuaternion<f64>>, Vec<Vector3<f64>>) {
        let up = Vector3::new(0.0, STANDARD_GRAVITY, 0.0);
        let rots: Vec<UnitQuaternion<f64>> = (0..30).map(|_| random_rotation(rng)).collect();
        let raw = rots
            .iter()
            .map(|q| {
                d_true.try_inverse().unwrap() * (q.inverse() * up) + b_true + rng.gauss3(noise)
            })
            .collect();
        (rots, raw)
    }

    #[test]
    fn the_accelerometer_calibrates_itself_from_holds() {
        let mut rng = Rng::new(7);
        let d_true = Matrix3::new(
            1.002, 0.0004, -0.0003, 0.0004, 0.998, 0.0002, -0.0003, 0.0002, 1.001,
        );
        let b_true = Vector3::new(0.018, 0.449, -0.081);
        let (_, raw) = holds(&mut rng, &d_true, &b_true, 0.001);
        let c = accel_cal(&raw, STANDARD_GRAVITY).unwrap();
        assert!((c.b - b_true).norm() < 0.005, "b {:?}", c.b);
        assert!((c.d - d_true).norm() < 1e-3, "d {:?}", c.d);
        assert!(c.rms < 0.002, "rms {}", c.rms);
    }

    #[test]
    fn holds_in_one_orientation_cannot_calibrate() {
        let mut rng = Rng::new(8);
        let up = Vector3::new(0.0, STANDARD_GRAVITY, 0.0);
        let raw: Vec<Vector3<f64>> = (0..30).map(|_| up + rng.gauss3(0.001)).collect();
        assert!(accel_cal(&raw, STANDARD_GRAVITY).is_none());
        assert!(accel_cal(&raw[..5], STANDARD_GRAVITY).is_none());
    }

    #[test]
    fn a_tilted_world_is_found_from_the_tracked_holds() {
        let mut rng = Rng::new(9);
        // the tracker's world is turned 0.6 degree from true: its "up" is off +Y
        let tilt = Vector3::new(0.6f64.to_radians(), 0.0, -0.11f64.to_radians());
        let true_from_world = UnitQuaternion::from_scaled_axis(tilt);
        let mut rots = vec![];
        let mut ups = vec![];
        for _ in 0..30 {
            let world_from_body = random_rotation(&mut rng);
            rots.push(world_from_body);
            // gravity is +Y up in the true frame; the body measures it there
            let up_true = Vector3::y();
            let body = (true_from_world * world_from_body).inverse() * up_true;
            ups.push((body + rng.gauss3(0.002)).normalize());
        }
        let (e, res) = world_tilt(&rots, &ups);
        assert!(
            (e - tilt).norm() < 0.05f64.to_radians(),
            "tilt {:?} deg",
            e.map(|x| x.to_degrees())
        );
        assert_eq!(e.y, 0.0);
        assert_eq!(res.len(), 30);
        let rms = (res.iter().map(|r| r * r).sum::<f64>() / 30.0).sqrt();
        assert!(
            rms < 0.3f64.to_radians(),
            "residual {} deg",
            rms.to_degrees()
        );
    }
}
