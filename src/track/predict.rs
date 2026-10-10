// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Velocities shaped for SteamVR's prediction. SteamVR extrapolates a pose to the time a frame is
//! shown (some 15-35 ms after the IMU sample) at constant velocity: its docs say it ignores the
//! accelerations a driver reports. During a flick that leaves the pose behind by half the
//! acceleration times the horizon squared. Publishing, instead of the velocity at the sample, the
//! mean velocity over a typical horizon H (v + a H/2) makes SteamVR's own extrapolation land on
//! the constant-acceleration prediction there; the pose itself is untouched. The acceleration
//! comes from a least-squares slope over the last few milliseconds of rates, and only above aiming
//! speeds: a steady hand gets exactly the velocities it had.
//!
//! Measured on recorded play (eval-1791516631), 25 ms ahead while moving: rotation error p90
//! 2.02 -> 1.26 deg, p99 5.84 -> 4.08; position p90 2.69 -> 1.29 mm, p99 7.94 -> 3.64; better
//! than plain at every horizon 15-35 ms and every percentile through p99.9.

use std::collections::VecDeque;

use nalgebra::{UnitQuaternion, Vector3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapeParams {
    /// The horizon the velocities are averaged over (s): SteamVR's typical prediction horizon.
    pub horizon_s: f64,
    /// The rates the accelerations are fitted over (s).
    pub window_s: f64,
    /// Angular speeds (rad/s) between which the shaping fades in: below the first, none.
    pub fade_rad_s: (f64, f64),
    /// Linear speeds (m/s) likewise.
    pub fade_m_s: (f64, f64),
    /// Samples further apart than this start the fit afresh (s).
    pub max_gap_s: f64,
    /// The largest angular (rad/s^2) and linear (m/s^2) accelerations used: a shake reverses
    /// within the horizon, and a larger estimate overshoots more than it gains.
    pub max_ang_accel: f64,
    pub max_lin_accel: f64,
    /// A track younger than this (s) is not shaped linearly: its velocity is still converging.
    pub settle_s: f64,
}

impl Default for ShapeParams {
    fn default() -> Self {
        // H 20 ms: better than plain velocities at real horizons of 15-35 ms. Caps 200 rad/s^2 and
        // 40 m/s^2: a fast shake read as more overshoots; with them every percentile through
        // p99.9 beats plain on four recordings, at the cost of 0.2 deg at p99
        ShapeParams {
            horizon_s: 0.020,
            window_s: 0.010,
            fade_rad_s: (0.5, 1.5),
            fade_m_s: (0.1, 0.3),
            max_gap_s: 0.020,
            max_ang_accel: 200.0,
            max_lin_accel: 40.0,
            settle_s: 0.2,
        }
    }
}

type ShapedVelocity = (Vector3<f64>, Option<Vector3<f64>>);

#[derive(Debug, Clone)]
pub struct VelocityShaper {
    pub params: ShapeParams,
    /// (time s, angular velocity in the body rad/s, linear velocity world m/s or None when not
    /// tracked). Body rates: a re-lock that turns the tracked orientation does not show in them.
    history: VecDeque<(f64, Vector3<f64>, Option<Vector3<f64>>)>,
    /// When the current run of tracked samples began (s).
    tracked_since: Option<f64>,
    /// The newest sample's time (ns) and what it was shaped to.
    last: Option<(u64, ShapedVelocity)>,
}

fn smoothstep(x: f64, (a, b): (f64, f64)) -> f64 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The least-squares slope of `v` against `t`; None with fewer than 3 points.
fn slope(pts: impl Iterator<Item = (f64, Vector3<f64>)> + Clone) -> Option<Vector3<f64>> {
    let n = pts.clone().count();
    if n < 3 {
        return None;
    }
    let tm = pts.clone().map(|p| p.0).sum::<f64>() / n as f64;
    let vm = pts.clone().map(|p| p.1).sum::<Vector3<f64>>() / n as f64;
    let stt: f64 = pts.clone().map(|p| (p.0 - tm).powi(2)).sum();
    (stt > 0.0).then(|| pts.map(|p| (p.1 - vm) * (p.0 - tm)).sum::<Vector3<f64>>() / stt)
}

impl VelocityShaper {
    pub fn new(params: ShapeParams) -> Self {
        VelocityShaper {
            params,
            history: VecDeque::new(),
            tracked_since: None,
            last: None,
        }
    }

    /// The newest output at time `t_ns`: its orientation (world from body), the angular velocity
    /// in the body (gyro less bias, rad/s) and the linear velocity in the world (None while the
    /// position is not tracked). Returns the angular velocity in the world and the linear one to
    /// publish. The angular acceleration is the body rates' slope turned into the world: the
    /// derivative of q w_b is q dw_b/dt, exactly.
    pub fn shape_body(
        &mut self,
        t_ns: u64,
        q: &UnitQuaternion<f64>,
        w_body: Vector3<f64>,
        v: Option<Vector3<f64>>,
    ) -> (Vector3<f64>, Option<Vector3<f64>>) {
        if let Some((t_last, out)) = self.last {
            if t_last == t_ns {
                return out; // the same sample again: no new information
            }
        }
        let p = self.params;
        let t = t_ns as f64 * 1e-9;
        if self
            .history
            .back()
            .is_some_and(|b| t <= b.0 || t - b.0 > p.max_gap_s)
        {
            self.history.clear();
            self.tracked_since = None;
        }
        self.tracked_since = match v {
            Some(_) => Some(self.tracked_since.unwrap_or(t)),
            None => None,
        };
        self.history.push_back((t, w_body, v));
        while self.history.front().is_some_and(|f| f.0 < t - p.window_s) {
            self.history.pop_front();
        }
        let half = 0.5 * p.horizon_s;
        let cap = |a: Vector3<f64>, max: f64| {
            if a.norm() > max {
                a * (max / a.norm())
            } else {
                a
            }
        };
        let w = q * w_body;
        let gw = smoothstep(w.norm(), p.fade_rad_s);
        let w_out = if gw > 0.0 {
            slope(self.history.iter().map(|h| (h.0, h.1)))
                .map_or(w, |a| w + cap(q * a, p.max_ang_accel) * (half * gw))
        } else {
            w
        };
        let settled = self.tracked_since.is_some_and(|s| t - s >= p.settle_s);
        let v_out = v.map(|v| {
            let gv = smoothstep(v.norm(), p.fade_m_s);
            if gv > 0.0 && settled && self.history.iter().all(|h| h.2.is_some()) {
                slope(self.history.iter().map(|h| (h.0, h.2.unwrap_or_default())))
                    .map_or(v, |a| v + cap(a, p.max_lin_accel) * (half * gv))
            } else {
                v
            }
        });
        self.last = Some((t_ns, (w_out, v_out)));
        (w_out, v_out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 500 Hz samples of a rate that changes at a constant `accel`, from `start`.
    fn feed(
        s: &mut VelocityShaper,
        start: Vector3<f64>,
        accel: Vector3<f64>,
        n: usize,
    ) -> (Vector3<f64>, Option<Vector3<f64>>, Vector3<f64>) {
        let mut out = (Vector3::zeros(), None);
        let mut w = start;
        for k in 0..n {
            w = start + accel * (k as f64 * 0.002);
            out = s.shape_body(
                1_000_000_000 + k as u64 * 2_000_000,
                &UnitQuaternion::identity(),
                w,
                Some(w * 0.5),
            );
        }
        (out.0, out.1, w)
    }

    #[test]
    fn a_fast_turn_publishes_the_mean_velocity_over_the_horizon() {
        let mut s = VelocityShaper::new(ShapeParams::default());
        let accel = Vector3::new(0.0, 60.0, 0.0); // rad/s^2, a brisk flick, under the cap
        let (w, v, raw) = feed(
            &mut s,
            Vector3::new(0.0, 3.0, 0.0) - accel * 0.2,
            accel,
            120,
        );
        let expect = raw + accel * 0.010;
        assert!((w - expect).norm() < 1e-9, "{w:?} vs {expect:?}");
        // the linear velocity (here half the rate) likewise
        let v = v.unwrap();
        assert!(
            (v - (raw * 0.5 + accel * 0.5 * 0.010)).norm() < 1e-9,
            "{v:?}"
        );
    }

    /// A shake read as a huge acceleration reverses within the horizon: the estimate is capped.
    #[test]
    fn a_violent_shake_is_capped() {
        let mut s = VelocityShaper::new(ShapeParams::default());
        let accel = Vector3::new(1000.0, 0.0, 0.0);
        let (w, _, raw) = feed(&mut s, Vector3::new(4.0, 0.0, 0.0), accel, 20);
        assert!(
            ((w - raw).norm() - 200.0 * 0.010).abs() < 1e-9,
            "{:?}",
            w - raw
        );
    }

    /// A re-lock turns the tracked orientation 30 deg in one sample while the hand turns steadily:
    /// the world rate jumps, the body rate does not, and nothing is added.
    #[test]
    fn an_orientation_jump_is_not_an_acceleration() {
        let mut s = VelocityShaper::new(ShapeParams::default());
        let wb = Vector3::new(0.0, 3.0, 0.0);
        let mut q = UnitQuaternion::identity();
        for k in 0..10u64 {
            if k == 5 {
                q = UnitQuaternion::from_scaled_axis(Vector3::new(0.52, 0.0, 0.0)) * q;
            }
            let (w, _) = s.shape_body(1_000_000_000 + k * 2_000_000, &q, wb, None);
            assert!((w - q * wb).norm() < 1e-9, "sample {k}: {w:?}");
        }
    }

    /// Just acquired, the filter's velocity is still converging from zero: not a motion to
    /// extrapolate, so the linear shaping waits.
    #[test]
    fn a_fresh_track_is_not_shaped_linearly() {
        let mut s = VelocityShaper::new(ShapeParams::default());
        let q = UnitQuaternion::identity();
        let mut last = None;
        for k in 0..60u64 {
            // not tracked for 20 ms, then a velocity climbing 30 m/s^2 as the filter converges
            let v = (k >= 10).then(|| Vector3::new(0.3 + 0.06 * (k - 10) as f64, 0.0, 0.0));
            let (_, vo) = s.shape_body(1_000_000_000 + k * 2_000_000, &q, Vector3::zeros(), v);
            if (10..100).contains(&k) {
                assert_eq!(
                    vo, v,
                    "sample {k}: shaped within the first 200 ms of the track"
                );
            }
            last = vo;
        }
        assert!(last.is_some());
    }

    /// The engine loop shapes both hands on every pass, also a hand whose newest sample it shaped
    /// on the pass before (about one pass in fifteen): that is the same output again, not a fresh
    /// start that throws away the slope and the settle time.
    #[test]
    fn a_repeated_sample_changes_nothing() {
        let p = ShapeParams::default();
        let (mut once, mut twice) = (VelocityShaper::new(p), VelocityShaper::new(p));
        let q = UnitQuaternion::identity();
        let mut last = (Vector3::zeros(), None, None);
        for k in 0..200u64 {
            // 400 ms of a hand speeding up: 60 rad/s^2 and 10 m/s^2
            let w = Vector3::new(0.0, 2.0 + 0.12 * k as f64, 0.0);
            let v = Some(Vector3::new(0.5 + 0.02 * k as f64, 0.0, 0.0));
            let t = 1_000_000_000 + k * 2_000_000;
            let a = once.shape_body(t, &q, w, v);
            assert_eq!(twice.shape_body(t, &q, w, v), a, "sample {k}");
            assert_eq!(twice.shape_body(t, &q, w, v), a, "sample {k}, shaped again");
            last = (a.0, a.1, v);
        }
        assert!(
            last.1.unwrap().x > last.2.unwrap().x + 0.05,
            "settled and still speeding up: shaped linearly"
        );
    }

    #[test]
    fn a_steady_hand_gets_exactly_its_own_velocities() {
        let mut s = VelocityShaper::new(ShapeParams::default());
        // slow and accelerating (aiming adjustments): below the fade, nothing is added
        let (w, v, raw) = feed(
            &mut s,
            Vector3::new(0.1, 0.0, 0.0),
            Vector3::new(5.0, 0.0, 0.0),
            10,
        );
        assert!(raw.norm() < 0.5);
        assert_eq!(w, raw);
        assert_eq!(v, Some(raw * 0.5));
    }

    #[test]
    fn a_gap_or_an_untracked_position_starts_afresh() {
        let mut s = VelocityShaper::new(ShapeParams::default());
        let accel = Vector3::new(0.0, 0.0, 80.0);
        feed(&mut s, Vector3::new(0.0, 0.0, 4.0), accel, 10);
        // 50 ms later: one sample is no slope
        let w = Vector3::new(0.0, 0.0, 9.0);
        assert_eq!(
            s.shape_body(
                1_000_000_000 + 70_000_000,
                &UnitQuaternion::identity(),
                w,
                Some(w)
            ),
            (w, Some(w))
        );
        // the position lost within the window: linear left alone, angular still shaped
        let mut s = VelocityShaper::new(ShapeParams::default());
        for k in 0..6u64 {
            let w = Vector3::new(0.0, 0.0, 4.0 + 0.16 * k as f64);
            let v = (k != 2).then_some(Vector3::new(1.0, 0.0, 0.0) * (1.0 + 0.1 * k as f64));
            let (wo, vo) = s.shape_body(
                1_000_000_000 + k * 2_000_000,
                &UnitQuaternion::identity(),
                w,
                v,
            );
            if k == 5 {
                assert!((wo - w).norm() > 0.5, "angular shaped: {wo:?}");
                assert_eq!(vo, v, "linear untouched across the lost fix");
            }
        }
    }
}
