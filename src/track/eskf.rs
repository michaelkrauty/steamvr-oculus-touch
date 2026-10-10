// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Error-state Kalman filter for one controller: prediction from each IMU sample, and an iterated,
//! robust update from pixel residuals. No data association here (see `vision`).
//!
//! Nominal state: orientation q (world from the LED model frame, which the factory-calibrated IMU
//! axes share), IMU position p and velocity v in the world, gyro bias bg and accelerometer bias ba
//! (body frame). Error state (15): local rotation d0 (q = q^ Exp(d0)), dp, dv, dbg, dba.

use nalgebra::{Matrix3, SMatrix, SVector, UnitQuaternion, Vector2, Vector3};

pub type Cov = SMatrix<f64, 15, 15>;
pub type ErrVec = SVector<f64, 15>;
/// Jacobian of one pixel observation with respect to (local rotation, position).
pub type PoseJac = SMatrix<f64, 2, 6>;

pub const ROT: usize = 0;
pub const POS: usize = 3;
pub const VEL: usize = 6;
pub const BG: usize = 9;
pub const BA: usize = 12;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Nominal {
    pub q: UnitQuaternion<f64>,
    pub p: Vector3<f64>,
    pub v: Vector3<f64>,
    pub bg: Vector3<f64>,
    pub ba: Vector3<f64>,
}

impl Nominal {
    pub fn plus(&self, e: &ErrVec) -> Nominal {
        let part = |i: usize| Vector3::new(e[i], e[i + 1], e[i + 2]);
        Nominal {
            q: self.q * UnitQuaternion::from_scaled_axis(part(ROT)),
            p: self.p + part(POS),
            v: self.v + part(VEL),
            bg: self.bg + part(BG),
            ba: self.ba + part(BA),
        }
    }

    /// The error that takes `base` to `self`.
    pub fn minus(&self, base: &Nominal) -> ErrVec {
        let mut e = ErrVec::zeros();
        e.fixed_rows_mut::<3>(ROT)
            .copy_from(&(base.q.inverse() * self.q).scaled_axis());
        e.fixed_rows_mut::<3>(POS).copy_from(&(self.p - base.p));
        e.fixed_rows_mut::<3>(VEL).copy_from(&(self.v - base.v));
        e.fixed_rows_mut::<3>(BG).copy_from(&(self.bg - base.bg));
        e.fixed_rows_mut::<3>(BA).copy_from(&(self.ba - base.ba));
        e
    }
}

/// Continuous-time noise densities.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Noise {
    /// rad/s/sqrt(Hz)
    pub gyro: f64,
    /// m/s^2/sqrt(Hz)
    pub accel: f64,
    /// Bias random walks: rad/s/sqrt(s), m/s^2/sqrt(s).
    pub gyro_bias: f64,
    pub accel_bias: f64,
    /// Added to `accel` per m/s^2 of linear acceleration and per rad/s of rotation rate: model
    /// errors (timing, lever arm, scale) grow with the motion.
    pub accel_per_accel: f64,
    pub accel_per_rate: f64,
}

impl Default for Noise {
    fn default() -> Self {
        Noise {
            gyro: 2e-3,
            accel: 0.01,
            gyro_bias: 1e-4,
            accel_bias: 2e-4,
            accel_per_accel: 0.005,
            accel_per_rate: 0.0,
        }
    }
}

pub fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Filter {
    pub x: Nominal,
    pub cov: Cov,
}

/// One pixel observation linearised at a nominal state: residual (measured - predicted) and its
/// Jacobian with respect to (local rotation, position).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PixelResidual {
    pub r: Vector2<f64>,
    pub j: PoseJac,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateParams {
    /// Pixel standard deviation.
    pub sigma_px: f64,
    /// Huber threshold, in standard deviations of the 2-D residual norm.
    pub huber_k: f64,
    pub iterations: usize,
}

impl Default for UpdateParams {
    fn default() -> Self {
        UpdateParams {
            sigma_px: 0.5,
            huber_k: 2.0,
            iterations: 4,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Updated {
    pub filter: Filter,
    /// Post-fit residual norms, pixels.
    pub residuals_px: Vec<f64>,
}

impl Filter {
    /// Advances `dt` seconds holding the IMU sample constant: body angular rate `gyro` (rad/s) and
    /// specific force `accel` (m/s^2); `gravity` is the world gravity vector (0, -g, 0).
    pub fn propagate(
        &mut self,
        gyro: &Vector3<f64>,
        accel: &Vector3<f64>,
        dt: f64,
        gravity: &Vector3<f64>,
        n: &Noise,
    ) {
        if dt <= 0.0 {
            return;
        }
        let x = &mut self.x;
        let w = gyro - x.bg;
        let f = accel - x.ba;
        let r = x.q.to_rotation_matrix().into_inner();
        let a = r * f + gravity;
        x.p += x.v * dt + a * (0.5 * dt * dt);
        x.v += a * dt;
        x.q *= UnitQuaternion::from_scaled_axis(w * dt);
        x.q.renormalize();

        let mut phi = Cov::identity();
        let i3 = Matrix3::identity();
        let rf = r * skew(&f);
        phi.fixed_view_mut::<3, 3>(ROT, ROT).copy_from(
            &UnitQuaternion::from_scaled_axis(-w * dt)
                .to_rotation_matrix()
                .into_inner(),
        );
        phi.fixed_view_mut::<3, 3>(ROT, BG).copy_from(&(-i3 * dt));
        phi.fixed_view_mut::<3, 3>(POS, ROT)
            .copy_from(&(-rf * (0.5 * dt * dt)));
        phi.fixed_view_mut::<3, 3>(POS, VEL).copy_from(&(i3 * dt));
        phi.fixed_view_mut::<3, 3>(POS, BA)
            .copy_from(&(-r * (0.5 * dt * dt)));
        phi.fixed_view_mut::<3, 3>(VEL, ROT).copy_from(&(-rf * dt));
        phi.fixed_view_mut::<3, 3>(VEL, BA).copy_from(&(-r * dt));
        let mut q = Cov::zeros();
        let accel_density = n.accel + n.accel_per_accel * a.norm() + n.accel_per_rate * w.norm();
        for (block, density) in [
            (ROT, n.gyro),
            (VEL, accel_density),
            (BG, n.gyro_bias),
            (BA, n.accel_bias),
        ] {
            for k in 0..3 {
                q[(block + k, block + k)] = density * density * dt;
            }
        }
        self.cov = phi * self.cov * phi.transpose() + q;
        self.cov = (self.cov + self.cov.transpose()) * 0.5;
    }

    /// Fuses a direct measurement of the IMU's world velocity, `v` with standard deviation `sigma`
    /// (m/s) per axis: a zero-velocity update while the IMU says the controller is still.
    pub fn update_velocity(&mut self, v: &Vector3<f64>, sigma: f64) {
        let mut h = SMatrix::<f64, 3, 15>::zeros();
        h.fixed_view_mut::<3, 3>(0, VEL)
            .copy_from(&Matrix3::identity());
        let s = h * self.cov * h.transpose() + Matrix3::identity() * (sigma * sigma);
        let Some(s_inv) = s.try_inverse() else { return };
        let k = self.cov * h.transpose() * s_inv;
        let e: ErrVec = k * (v - self.x.v);
        self.x = self.x.plus(&e);
        let ikh = Cov::identity() - k * h;
        self.cov = ikh * self.cov * ikh.transpose() + k * (sigma * sigma) * k.transpose();
        self.cov = (self.cov + self.cov.transpose()) * 0.5;
    }

    /// Iterated, Huber-weighted update in information form. `measure` linearises the observations
    /// at a nominal state. None when the prior covariance is not positive definite or there are no
    /// observations. The filter is not changed: the caller commits `Updated::filter` if it accepts it.
    pub fn update<F>(&self, measure: F, p: &UpdateParams) -> Option<Updated>
    where
        F: Fn(&Nominal) -> Vec<PixelResidual>,
    {
        let info0 = self.cov.cholesky()?.inverse();
        let var = p.sigma_px * p.sigma_px;
        let mut e = ErrVec::zeros();
        let mut x = self.x;
        let mut cov = self.cov;
        for _ in 0..p.iterations.max(1) {
            let obs = measure(&x);
            if obs.is_empty() {
                return None;
            }
            let mut info = info0;
            let mut b = ErrVec::zeros();
            let e6 = e.fixed_rows::<6>(0).into_owned();
            for o in &obs {
                let norm = o.r.norm() / p.sigma_px;
                let w = if norm <= p.huber_k {
                    1.0
                } else {
                    p.huber_k / norm
                } / var;
                let jt = o.j.transpose();
                let mut block = info.fixed_view_mut::<6, 6>(0, 0);
                block += jt * o.j * w;
                let mut rows = b.fixed_rows_mut::<6>(0);
                rows += jt * (o.r + o.j * e6) * w;
            }
            let chol = info.cholesky()?;
            let next = chol.solve(&b);
            cov = chol.inverse();
            let step = (next - e).norm();
            e = next;
            x = self.x.plus(&e);
            if step < 1e-7 {
                break;
            }
        }
        let residuals_px = measure(&x).iter().map(|o| o.r.norm()).collect();
        Some(Updated {
            filter: Filter {
                x,
                cov: (cov + cov.transpose()) * 0.5,
            },
            residuals_px,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at_rest() -> Filter {
        let x = Nominal {
            q: UnitQuaternion::identity(),
            p: Vector3::new(0.0, 1.0, 0.0),
            v: Vector3::zeros(),
            bg: Vector3::zeros(),
            ba: Vector3::zeros(),
        };
        let mut cov = Cov::identity() * 1e-4;
        for k in 0..3 {
            cov[(BA + k, BA + k)] = 0.1;
        }
        Filter { x, cov }
    }

    const G: Vector3<f64> = Vector3::new(0.0, -9.80665, 0.0);

    /// The velocity's process noise grows with the motion: at rest only the base density, at 5 m/s^2
    /// and 2 rad/s the base plus both motion terms.
    #[test]
    fn velocity_noise_grows_with_the_motion() {
        let n = Noise {
            accel: 0.01,
            accel_per_accel: 0.004,
            accel_per_rate: 0.01,
            ..Noise::default()
        };
        let dt = 0.002;
        let growth = |gyro: Vector3<f64>, accel: Vector3<f64>| {
            let mut f = at_rest();
            f.cov = Cov::zeros();
            f.propagate(&gyro, &accel, dt, &G, &n);
            f.cov[(VEL + 1, VEL + 1)]
        };
        let rest = growth(Vector3::zeros(), Vector3::new(0.0, 9.80665, 0.0));
        assert!((rest - 0.01f64.powi(2) * dt).abs() < 1e-12, "{rest}");
        let moving = growth(
            Vector3::new(0.0, 0.0, 2.0),
            Vector3::new(0.0, 9.80665 + 5.0, 0.0),
        );
        let density = 0.01 + 0.004 * 5.0 + 0.01 * 2.0;
        assert!(
            (moving - density * density * dt).abs() < 1e-9,
            "{moving} vs {}",
            density * density * dt
        );
    }

    #[test]
    fn a_still_imu_feeling_gravity_stays_put() {
        let mut f = at_rest();
        for _ in 0..500 {
            f.propagate(
                &Vector3::zeros(),
                &Vector3::new(0.0, 9.80665, 0.0),
                0.002,
                &G,
                &Noise::default(),
            );
        }
        assert!((f.x.p - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-9 && f.x.v.norm() < 1e-9);
        assert!(
            f.cov[(POS, POS)] > 1e-4,
            "position uncertainty grows without observations"
        );
    }

    #[test]
    fn constant_rotation_and_acceleration_integrate_exactly_enough() {
        let mut f = at_rest();
        let w = Vector3::new(0.0, 1.0, 0.0); // 1 rad/s about +Y: specific force along body x rotates
        for _ in 0..1000 {
            // world accel (0.5, 0, 0) felt in the body frame, plus gravity
            let body = f.x.q.inverse() * (Vector3::new(0.5, 0.0, 0.0) - G);
            f.propagate(&w, &body, 0.001, &G, &Noise::default());
        }
        assert!((f.x.q.angle() - 1.0).abs() < 1e-9);
        assert!(
            (f.x.p.x - 0.25).abs() < 1e-3 && (f.x.v.x - 0.5).abs() < 1e-3,
            "{:?} {:?}",
            f.x.p,
            f.x.v
        );
    }

    /// Direct observations of position (as if pixels were metres): the update pulls the state to
    /// them, weighted by the prior, and a gross outlier is down-weighted by Huber.
    #[test]
    fn the_update_fuses_and_resists_an_outlier() {
        let f = at_rest();
        let target = Vector3::new(0.01, 1.0, 0.0);
        let pairs = |x: &Nominal, outlier: bool| -> Vec<PixelResidual> {
            let mut out = Vec::new();
            for axis in 0..3 {
                for k in 0..6 {
                    let mut j = PoseJac::zeros();
                    j[(0, POS + axis)] = 1.0;
                    let bad = outlier && axis == 0 && k == 0;
                    let z = if bad { 0.5 } else { target[axis] };
                    out.push(PixelResidual {
                        r: Vector2::new(z - x.p[axis], 0.0),
                        j,
                    });
                }
            }
            out
        };
        let p = UpdateParams {
            sigma_px: 1e-3,
            huber_k: 2.0,
            iterations: 10,
        };
        let clean = f.update(|x| pairs(x, false), &p).unwrap();
        assert!(
            (clean.filter.x.p - target).norm() < 2e-4,
            "{:?}",
            clean.filter.x.p
        );
        assert!(clean.filter.cov[(POS, POS)] < f.cov[(POS, POS)]);
        let robust = f.update(|x| pairs(x, true), &p).unwrap();
        assert!(
            (robust.filter.x.p.x - 0.01).abs() < 2e-3,
            "outlier pulled x to {}",
            robust.filter.x.p.x
        );
    }
}
