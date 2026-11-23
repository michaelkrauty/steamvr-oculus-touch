// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

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

        self.transition(&r, &f, &w, a.norm(), dt, n);
    }

    /// Advances `dt` seconds along the straight line from one IMU sample (`gyro0`, `accel0`) to
    /// another (`gyro1`, `accel1`): a stretch whose samples were lost, bridged by the samples on
    /// either side. The rotation turns by the mean rate; each specific force is turned with the
    /// orientation at its own end; the transition is taken at the midpoint. `white` is what the
    /// stretch hides, as densities of white acceleration and turn rate (see `add_white`).
    #[allow(clippy::too_many_arguments)]
    pub fn propagate_line(
        &mut self,
        gyro0: &Vector3<f64>,
        accel0: &Vector3<f64>,
        gyro1: &Vector3<f64>,
        accel1: &Vector3<f64>,
        dt: f64,
        gravity: &Vector3<f64>,
        n: &Noise,
        white: (f64, f64),
    ) {
        if dt <= 0.0 {
            return;
        }
        let x = &mut self.x;
        let w = ((gyro0 - x.bg) + (gyro1 - x.bg)) * 0.5;
        let (f0, f1) = (accel0 - x.ba, accel1 - x.ba);
        let r0 = x.q.to_rotation_matrix().into_inner();
        let q1 = x.q * UnitQuaternion::from_scaled_axis(w * dt);
        let r1 = q1.to_rotation_matrix().into_inner();
        let (a0, a1) = (r0 * f0 + gravity, r1 * f1 + gravity);
        x.p += x.v * dt + (a0 / 3.0 + a1 / 6.0) * (dt * dt);
        x.v += (a0 + a1) * (0.5 * dt);
        x.q = q1;
        x.q.renormalize();
        let rm = (self.x.q * UnitQuaternion::from_scaled_axis(-w * (0.5 * dt)))
            .to_rotation_matrix()
            .into_inner();
        let f = (f0 + f1) * 0.5;
        self.transition(&rm, &f, &w, ((a0 + a1) * 0.5).norm(), dt, n);
        self.add_white(&(rm * skew(&f)), white.0, white.1, dt);
    }

    /// Adds white acceleration of density `accel` (m^2/s^3) and white turn rate of density
    /// `rate` (rad^2/s) over `dt` seconds, integrated through the dynamics `transition` steps:
    /// the rotation's error reaches the velocity through the turned specific force `rf` (R [f]x)
    /// as it accumulates, and from there the position. For constant inputs this is exact, so a
    /// stretch takes on the same whole or in pieces.
    fn add_white(&mut self, rf: &Matrix3<f64>, accel: f64, rate: f64, dt: f64) {
        let (t2, t3) = (dt * dt, dt * dt * dt);
        let i3 = Matrix3::identity();
        let leak = rf * rf.transpose() * rate;
        for (r, c, block) in [
            (ROT, ROT, i3 * (rate * dt)),
            (VEL, ROT, -rf * (rate * t2 / 2.0)),
            (POS, ROT, -rf * (rate * t3 / 6.0)),
            (VEL, VEL, i3 * (accel * dt) + leak * (t3 / 3.0)),
            (POS, VEL, i3 * (accel * t2 / 2.0) + leak * (t2 * t2 / 8.0)),
            (POS, POS, i3 * (accel * t3 / 3.0) + leak * (t2 * t3 / 20.0)),
        ] {
            let mut at = self.cov.fixed_view_mut::<3, 3>(r, c);
            at += block;
            if r != c {
                let mut at = self.cov.fixed_view_mut::<3, 3>(c, r);
                at += block.transpose();
            }
        }
    }

    /// Takes on the turn that gyro readings at their rails may hide over the `dt` seconds just
    /// propagated, `rates` holding per gyro axis the turn rate its reading may miss (rad/s, body
    /// frame; zero off the rail). The missed turn has one sign, so the standard deviation of the
    /// rotation about that direction grows by all of it from what the filter holds now, up to
    /// half a turn, beyond which an orientation about it is unknown. A fix in between has
    /// shrunk that deviation, and the growth starts again from there. The growth is taken on as
    /// white turn rate over `dt`, integrated through the dynamics as `add_white` does, with `f`
    /// the specific force less its bias.
    pub fn add_hidden_turn(&mut self, rates: &Matrix3<f64>, f: &Vector3<f64>, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let rf = self.x.q.to_rotation_matrix().into_inner() * skew(f);
        let (t2, t3) = (dt * dt, dt * dt * dt);
        for rate in rates.column_iter().filter(|r| *r != Vector3::zeros()) {
            let axis = rate.normalize();
            let held = (axis.transpose() * self.cov.fixed_view::<3, 3>(ROT, ROT) * axis)[0];
            let sd = (held.sqrt() + rate.norm() * dt).min(std::f64::consts::PI);
            if sd * sd <= held {
                continue;
            }
            let rot = axis * axis.transpose() * ((sd * sd - held) / dt);
            let leak = rf * rot * rf.transpose();
            for (row, col, block) in [
                (ROT, ROT, rot * dt),
                (VEL, ROT, -rf * rot * (t2 / 2.0)),
                (POS, ROT, -rf * rot * (t3 / 6.0)),
                (VEL, VEL, leak * (t3 / 3.0)),
                (POS, VEL, leak * (t2 * t2 / 8.0)),
                (POS, POS, leak * (t2 * t3 / 20.0)),
            ] {
                let mut at = self.cov.fixed_view_mut::<3, 3>(row, col);
                at += block;
                if row != col {
                    let mut at = self.cov.fixed_view_mut::<3, 3>(col, row);
                    at += block.transpose();
                }
            }
        }
    }

    fn transition(
        &mut self,
        r: &Matrix3<f64>,
        f: &Vector3<f64>,
        w: &Vector3<f64>,
        a_norm: f64,
        dt: f64,
        n: &Noise,
    ) {
        let mut phi = Cov::identity();
        let i3 = Matrix3::identity();
        let rf = r * skew(f);
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
        let accel_density = n.accel + n.accel_per_accel * a_norm + n.accel_per_rate * w.norm();
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
        self.update_with_final_residuals(
            &measure,
            |x| measure(x).iter().map(|o| o.r.norm()).collect(),
            p,
        )
    }

    /// Same update, with a cheaper post-fit norm evaluation when Jacobians are unnecessary.
    /// Called only after a successful solve; an empty final vector still returns Some(Updated).
    pub(super) fn update_with_final_residuals<F, G>(
        &self,
        measure: F,
        final_residuals: G,
        p: &UpdateParams,
    ) -> Option<Updated>
    where
        F: Fn(&Nominal) -> Vec<PixelResidual>,
        G: Fn(&Nominal) -> Vec<f64>,
    {
        let info0 = self.cov.cholesky()?.inverse();
        let var = p.sigma_px * p.sigma_px;
        let mut e = ErrVec::zeros();
        let mut x = self.x;
        let mut cov = self.cov;
        for iteration in 0..p.iterations.max(1) {
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
            let step = (next - e).norm();
            e = next;
            x = self.x.plus(&e);
            if step < 1e-7 || iteration + 1 == p.iterations.max(1) {
                // Only the final accepted factor contributes the returned covariance.
                cov = chol.inverse();
                break;
            }
        }
        let residuals_px = final_residuals(&x);
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

    /// A turn a gyro's rail hides widens the rotation about that direction by all of the turn,
    /// from what the filter holds: after a fix has narrowed it, from there. Never past half a
    /// turn, and not about any other direction.
    #[test]
    fn a_hidden_turn_widens_the_rotation_from_what_the_filter_holds() {
        let dt = 0.002;
        let mut rates = Matrix3::zeros();
        rates.set_column(1, &Vector3::new(6.0, 8.0, 0.0));
        let (along, across) = (Vector3::new(0.6, 0.8, 0.0), Vector3::new(-0.8, 0.6, 0.0));
        let sd = |f: &Filter, a: &Vector3<f64>| {
            (a.transpose() * f.cov.fixed_view::<3, 3>(ROT, ROT) * a)[0].sqrt()
        };
        let mut f = at_rest();
        let force = Vector3::new(0.0, 9.80665, 0.0);
        f.add_hidden_turn(&rates, &force, dt);
        assert!((sd(&f, &along) - (0.01 + 10.0 * dt)).abs() < 1e-12);
        assert!((sd(&f, &across) - 0.01).abs() < 1e-12);
        assert!((sd(&f, &Vector3::z()) - 0.01).abs() < 1e-12);
        // a fix narrows it; the turn hidden after it widens it from there
        f.cov
            .fixed_view_mut::<3, 3>(ROT, ROT)
            .copy_from(&(Matrix3::identity() * 0.005f64.powi(2)));
        f.add_hidden_turn(&rates, &force, dt);
        assert!((sd(&f, &along) - (0.005 + 10.0 * dt)).abs() < 1e-12);
        f.add_hidden_turn(&(rates * 1e4), &force, dt);
        assert!((sd(&f, &along) - std::f64::consts::PI).abs() < 1e-12);
        f.add_hidden_turn(&rates, &force, dt);
        assert!((sd(&f, &along) - std::f64::consts::PI).abs() < 1e-12);
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

#[cfg(test)]
mod equivalence;
