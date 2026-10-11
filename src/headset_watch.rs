// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! Watching a headset-registration capture as it happens: whether a controller held against the
//! headset stays put, and when the person has turned, nodded and tilted enough to finish.
//!
//! A controller fixed to the headset turns exactly as the headset does: in their own body frames,
//! each rotation of the controller is the headset's rotation seen through one fixed offset X
//! (b = X⁻¹·a·X). That holds whatever error the current registration has, since a fixed world
//! transform cancels out of body-frame increments. X is fitted from the increments so far (the
//! hand-eye problem); a controller whose increments stop agreeing with it has slipped.

use std::collections::VecDeque;

use nalgebra::{Matrix4, Quaternion, SymmetricEigen, UnitQuaternion, Vector3, Vector4};

/// A controller that shifts against the headset by more than this (degrees of rotation) has
/// slipped: well above tracking noise and the timing mismatch of two devices at a person's turning
/// speed (about a degree together), well below what spoils a fit.
pub const SLIP_DEG: f64 = 3.0;

/// Orientation samples are taken every this many ns.
const STEP_NS: u64 = 100_000_000;
/// The offset is fitted from increments this many samples (0.5 s) long...
const FIT_SPAN: usize = 5;
/// ...and each window this many samples (1 s) long is judged against it: a shift within the
/// window shows up whole. Longer windows turn further, and after a shift too small to stop for,
/// the fit made before it is off by that much, which a large turn magnifies.
const JUDGE_SPAN: usize = 10;
/// Increments outside this range (degrees) are left out of the fit: too little rotation to inform
/// it, or so much that the quaternion's sign, and with it the constraint, is ambiguous.
const MIN_TURN_DEG: f64 = 5.0;
const MAX_TURN_DEG: f64 = 120.0;
/// Each increment joining the fit fades the earlier ones by this much, so the fit follows the
/// last few seconds of motion (an increment's weight halves after 35 newer ones): hands holding
/// controllers to a turning head give a little, and a fit of the whole minute would judge that
/// creep, magnified by every large turn, as a slip.
const FADE: f64 = 0.98;
/// Rotation (degrees) the fit needs before windows are judged.
const WARMUP_DEG: f64 = 90.0;
/// Consecutive judged windows (one per sample) over `SLIP_DEG` that make a slip: 0.8 s, so a
/// brief tracking glitch, which comes back, is not one.
const PERSIST: usize = 8;

/// Quaternion as [w, x, y, z], the scalar part made non-negative (q and -q are one rotation).
fn wxyz(q: &UnitQuaternion<f64>) -> Vector4<f64> {
    let v = Vector4::new(q.w, q.i, q.j, q.k);
    if q.w < 0.0 {
        -v
    } else {
        v
    }
}

/// p⊗q = left(p)·q, for quaternions as [w, x, y, z].
fn left(p: &Vector4<f64>) -> Matrix4<f64> {
    let (w, x, y, z) = (p[0], p[1], p[2], p[3]);
    Matrix4::new(w, -x, -y, -z, x, w, -z, y, y, z, w, -x, z, -y, x, w)
}

/// q⊗p = right(p)·q.
fn right(p: &Vector4<f64>) -> Matrix4<f64> {
    let (w, x, y, z) = (p[0], p[1], p[2], p[3]);
    Matrix4::new(w, -x, -y, -z, x, w, z, -y, y, -z, w, x, z, y, -x, w)
}

/// The fixed offset X best explaining increments accumulated as `m` (a·X = X·b for each).
fn fit(m: &Matrix4<f64>) -> UnitQuaternion<f64> {
    let eig = SymmetricEigen::new(*m);
    let (i, _) = eig
        .eigenvalues
        .iter()
        .enumerate()
        .fold(
            (0, f64::INFINITY),
            |best, (i, &v)| if v < best.1 { (i, v) } else { best },
        );
    let v = eig.eigenvectors.column(i);
    UnitQuaternion::from_quaternion(Quaternion::new(v[0], v[1], v[2], v[3]))
}

fn constraint(a: &UnitQuaternion<f64>, b: &UnitQuaternion<f64>) -> Matrix4<f64> {
    let d = left(&wxyz(a)) - right(&wxyz(b));
    d.transpose() * d
}

/// Whether an increment turning this far (degrees) can inform the fit.
fn usable(turn: f64) -> bool {
    (MIN_TURN_DEG..=MAX_TURN_DEG).contains(&turn)
}

/// Whether one controller stays fixed to the headset during a capture.
///
/// Each 1 s window is judged against the offset fitted from what came before it: with A and B
/// the headset's and the controller's turn over the window (body frames), (X⁻¹·A·X)⁻¹·B is the
/// controller's shift against the headset within the window. The fit leaves the window's own
/// increments out until the window has passed, so a slip can't pull the fit along before it is
/// judged; the window's own increment does join the fit when judging it, which pins what the
/// past can't (after turning only about one axis, the offset's twist about it is still open).
pub struct RigidWatch {
    samples: VecDeque<(UnitQuaternion<f64>, UnitQuaternion<f64>)>,
    next_ns: u64,
    count: usize,
    /// Fit increments waiting for their window to pass: (sample count when they join, the
    /// constraint, degrees turned).
    pending: VecDeque<(usize, Matrix4<f64>, f64)>,
    m: Matrix4<f64>,
    turned_deg: f64,
    over: usize,
    worst_deg: f64,
}

impl Default for RigidWatch {
    fn default() -> Self {
        Self::new()
    }
}

impl RigidWatch {
    pub fn new() -> RigidWatch {
        RigidWatch {
            samples: VecDeque::new(),
            next_ns: 0,
            count: 0,
            pending: VecDeque::new(),
            m: Matrix4::zeros(),
            turned_deg: 0.0,
            over: 0,
            worst_deg: 0.0,
        }
    }

    /// Whether a shift is showing but not yet held long enough to count as a slip.
    pub fn unsettled(&self) -> bool {
        (1..PERSIST).contains(&self.over)
    }

    /// The headset's and the controller's orientation at the same instant `t_ns`. Some(degrees)
    /// once the controller has shifted against the headset by more than `SLIP_DEG` for
    /// `PERSIST` samples in a row; then on every later call.
    pub fn push(
        &mut self,
        t_ns: u64,
        hmd: UnitQuaternion<f64>,
        touch: UnitQuaternion<f64>,
    ) -> Option<f64> {
        if self.over >= PERSIST {
            return Some(self.worst_deg);
        }
        if t_ns < self.next_ns {
            return None;
        }
        self.next_ns = t_ns + STEP_NS;
        self.count += 1;
        self.samples.push_back((hmd, touch));
        if self.samples.len() > JUDGE_SPAN + 1 {
            self.samples.pop_front();
        }
        while let Some((due, c, turn)) = self.pending.front() {
            if *due > self.count {
                break;
            }
            self.m = self.m * FADE + c;
            self.turned_deg += turn / FIT_SPAN as f64;
            self.pending.pop_front();
        }
        let n = self.samples.len();
        let increment = |from: usize| {
            let (h0, c0) = self.samples[from];
            (h0.inverse() * hmd, c0.inverse() * touch)
        };
        if n > FIT_SPAN {
            let (a, b) = increment(n - 1 - FIT_SPAN);
            let turn = a.angle().to_degrees();
            if usable(turn) {
                self.pending
                    .push_back((self.count + JUDGE_SPAN, constraint(&a, &b), turn));
            }
        }
        if n <= JUDGE_SPAN || self.turned_deg < WARMUP_DEG {
            return None;
        }
        let (a, b) = increment(0);
        let x = if usable(a.angle().to_degrees()) {
            fit(&(self.m + constraint(&a, &b)))
        } else {
            fit(&self.m)
        };
        let shift = ((x.inverse() * a * x).inverse() * b).angle().to_degrees();
        if shift > SLIP_DEG {
            self.over += 1;
            self.worst_deg = self.worst_deg.max(shift);
            return (self.over >= PERSIST).then_some(self.worst_deg);
        }
        self.over = 0;
        self.worst_deg = 0.0;
        None
    }
}

/// Turning (degrees, each way) and nodding/tilting (degrees of range) a capture needs: two full
/// turns each way less some slack, and a clear nod and tilt.
pub const TURN_DEG: f64 = 660.0;
pub const NOD_DEG: f64 = 30.0;

/// How far the headset has turned each way and nodded and tilted (SteamVR: +Y up, -Z forward).
#[derive(Default)]
pub struct TurnCoverage {
    yaw: Option<f64>,
    left_deg: f64,
    right_deg: f64,
    pitch: Option<(f64, f64)>,
    roll: Option<(f64, f64)>,
}

fn span(range: Option<(f64, f64)>) -> f64 {
    range.map_or(0.0, |(lo, hi)| hi - lo)
}

fn widen(range: &mut Option<(f64, f64)>, v: f64) {
    let r = range.get_or_insert((v, v));
    r.0 = r.0.min(v);
    r.1 = r.1.max(v);
}

impl TurnCoverage {
    pub fn push(&mut self, hmd: UnitQuaternion<f64>) {
        let forward = hmd * Vector3::new(0.0, 0.0, -1.0);
        let right = hmd * Vector3::x();
        widen(
            &mut self.pitch,
            forward.y.clamp(-1.0, 1.0).asin().to_degrees(),
        );
        widen(&mut self.roll, right.y.clamp(-1.0, 1.0).asin().to_degrees());
        if forward.y.abs() > 0.9 {
            // looking straight up or down: no heading to measure
            self.yaw = None;
            return;
        }
        let yaw = (-forward.x).atan2(-forward.z).to_degrees();
        if let Some(last) = self.yaw {
            let d = (yaw - last + 540.0).rem_euclid(360.0) - 180.0;
            if d > 0.0 {
                self.left_deg += d;
            } else {
                self.right_deg -= d;
            }
        }
        self.yaw = Some(yaw);
    }

    pub fn turns_done(&self) -> bool {
        self.left_deg >= TURN_DEG && self.right_deg >= TURN_DEG
    }

    pub fn done(&self) -> bool {
        self.turns_done() && span(self.pitch) >= NOD_DEG && span(self.roll) >= NOD_DEG
    }
}

/// What a headset capture tells the person.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Cue {
    /// Keep going.
    Recording,
    /// Turned enough: now nod and tilt. Given once.
    Turned,
    /// Turned, nodded and tilted enough: the capture can stop.
    Done,
    /// The controller in this hand shifted against the headset by this many degrees.
    Slipped(usize, f64),
}

/// A headset capture as it happens: how far the person has got, and (with the slip check)
/// whether a controller has moved on the headset.
pub struct HeadsetCapture {
    watch: Option<[RigidWatch; 2]>,
    coverage: TurnCoverage,
    turned: bool,
    slipped: Option<(usize, f64)>,
}

impl HeadsetCapture {
    pub fn new(slip_check: bool) -> HeadsetCapture {
        HeadsetCapture {
            watch: slip_check.then(Default::default),
            coverage: TurnCoverage::default(),
            turned: false,
            slipped: None,
        }
    }

    /// The headset's and each controller's orientation (when tracked) at one instant.
    pub fn push(
        &mut self,
        t_ns: u64,
        hmd: UnitQuaternion<f64>,
        touch: [Option<UnitQuaternion<f64>>; 2],
    ) {
        self.coverage.push(hmd);
        let Some(watch) = self.watch.as_mut() else {
            return;
        };
        for (hand, (w, touch)) in watch.iter_mut().zip(touch).enumerate() {
            if let (None, Some(touch)) = (self.slipped, touch) {
                self.slipped = w.push(t_ns, hmd, touch).map(|deg| (hand, deg));
            }
        }
    }

    /// The cue for now: a slip first, then done (once no shift is pending), then (once) turned.
    pub fn cue(&mut self) -> Cue {
        if let Some((hand, deg)) = self.slipped {
            return Cue::Slipped(hand, deg);
        }
        let unsettled = self.watch.iter().flatten().any(RigidWatch::unsettled);
        if self.coverage.done() && !unsettled {
            return Cue::Done;
        }
        if !self.turned && self.coverage.turns_done() {
            self.turned = true;
            return Cue::Turned;
        }
        Cue::Recording
    }

    pub fn slipped(&self) -> bool {
        self.slipped.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rot(axis: Vector3<f64>, deg: f64) -> UnitQuaternion<f64> {
        UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), deg.to_radians())
    }

    /// A person's head at time t, turning at `speed` degrees/s: two turns one way, two back,
    /// with a little nod and tilt (`wobble`) while turning, then nodding and tilting `nod` degrees
    /// each way.
    fn head_with(t: f64, speed: f64, wobble: f64, nod: f64) -> UnitQuaternion<f64> {
        let turning = 720.0 / speed;
        let yaw = if t < turning {
            speed * t
        } else if t < 2.0 * turning {
            speed * (2.0 * turning - t)
        } else {
            0.0
        };
        let mut nod_deg = 3.0 * wobble * (0.7 * t).sin();
        let mut tilt = 2.0 * wobble * (0.5 * t).cos();
        if t >= 2.0 * turning {
            let since = t - 2.0 * turning;
            nod_deg += nod * (2.0 * since).sin();
            tilt += nod * (1.3 * since).sin();
        }
        rot(Vector3::y(), yaw) * rot(Vector3::x(), nod_deg) * rot(Vector3::z(), tilt)
    }

    fn head_at(t: f64, speed: f64, wobble: f64) -> UnitQuaternion<f64> {
        head_with(t, speed, wobble, 20.0)
    }

    /// At 45 degrees/s: turning until 32 s, then nodding and tilting.
    fn head(t: f64) -> UnitQuaternion<f64> {
        head_at(t, 45.0, 1.0)
    }

    /// Small tracking noise, deterministic.
    fn jitter(i: u64) -> UnitQuaternion<f64> {
        let s = |k: u64| ((i * 2654435761 + k * 40503) % 1000) as f64 / 1000.0 - 0.5;
        rot(Vector3::new(s(1), s(2), s(3)), 0.3)
    }

    /// Runs a capture of `secs` at 500 Hz; `slip(t)` is the controller's rotation against the
    /// headset at time t. The world error stands for a wrong registration, the offset for how
    /// the controller sits.
    fn run_with(
        head: impl Fn(f64) -> UnitQuaternion<f64>,
        secs: f64,
        slip: impl Fn(f64) -> UnitQuaternion<f64>,
    ) -> Option<(f64, f64)> {
        let world = rot(Vector3::new(0.3, 1.0, -0.2), 25.0);
        let offset = rot(Vector3::new(1.0, 0.2, 0.5), 70.0);
        let mut w = RigidWatch::new();
        let mut i = 0u64;
        while (i as f64) / 500.0 < secs {
            let t = i as f64 / 500.0;
            // the controller's pose arrives 10 ms behind the headset's
            let c = world * head(t - 0.01) * offset * slip(t) * jitter(i);
            if let Some(deg) = w.push((t * 1e9) as u64, head(t), c) {
                return Some((t, deg));
            }
            i += 1;
        }
        None
    }

    fn run(secs: f64, slip: impl Fn(f64) -> UnitQuaternion<f64>) -> Option<(f64, f64)> {
        run_with(head, secs, slip)
    }

    fn still(_: f64) -> UnitQuaternion<f64> {
        UnitQuaternion::identity()
    }

    #[test]
    fn a_controller_held_still_against_the_headset_never_slips() {
        assert_eq!(run(48.0, still), None);
    }

    #[test]
    fn turning_about_one_axis_and_then_nodding_is_not_a_slip() {
        // turning without any wobble leaves the offset's twist about the vertical open until
        // the first nod
        assert_eq!(run_with(|t| head_at(t, 45.0, 0.0), 48.0, still), None);
    }

    #[test]
    fn turning_fast_is_not_a_slip_and_a_slip_is_still_caught() {
        let fast = |t| head_at(t, 90.0, 1.0);
        assert_eq!(run_with(fast, 24.0, still), None);
        let slip = |t: f64| {
            if t >= 6.0 {
                rot(Vector3::x(), 6.0)
            } else {
                UnitQuaternion::identity()
            }
        };
        let (t, _) = run_with(fast, 24.0, slip).expect("missed a slip while turning fast");
        assert!((6.0..8.0).contains(&t), "caught at {t} s");
    }

    #[test]
    fn a_controller_that_shifts_a_few_degrees_is_caught_within_two_seconds() {
        for axis in [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Vector3::new(1.0, -1.0, 0.5),
        ] {
            for at in [10.0, 25.0, 40.0] {
                let found = run(48.0, |t| {
                    if t >= at {
                        rot(axis, 6.0)
                    } else {
                        UnitQuaternion::identity()
                    }
                });
                let (t, deg) =
                    found.unwrap_or_else(|| panic!("missed a slip about {axis:?} at {at} s"));
                assert!(
                    t >= at && t < at + 2.0,
                    "caught at {t} s for a slip at {at} s"
                );
                assert!(deg > SLIP_DEG, "{deg}");
            }
        }
    }

    #[test]
    fn a_slip_soon_after_the_offset_is_first_known_is_caught() {
        // the fit is still small then: had it taken in the increments after the slip at once,
        // they would have pulled it along
        let slip = |t: f64| {
            if t >= 4.0 {
                rot(Vector3::new(1.0, -1.0, 0.5), 6.0)
            } else {
                UnitQuaternion::identity()
            }
        };
        let (t, _) = run_with(|t| head_at(t, 90.0, 1.0), 24.0, slip).expect("missed the slip");
        assert!((4.0..6.0).contains(&t), "caught at {t} s");
    }

    #[test]
    fn a_tiny_settle_is_tolerated_but_five_degrees_is_not() {
        let shift = |deg: f64| {
            move |t: f64| {
                if t >= 20.0 {
                    rot(Vector3::new(0.3, 1.0, 0.4), deg)
                } else {
                    UnitQuaternion::identity()
                }
            }
        };
        let two = run(48.0, shift(2.0));
        let five = run(48.0, shift(5.0));
        assert!(two.is_none() && five.is_some(), "2°: {two:?}, 5°: {five:?}");
    }

    #[test]
    fn a_hold_that_creeps_slowly_is_followed_not_stopped() {
        // hands holding controllers to a turning head give a little over a minute (here about
        // 10°, more than real holds do), which a fit of the whole capture no longer explains
        // once the head nods and tilts far
        let creep = |t: f64| rot(Vector3::new(1.0, 0.5, -0.3), 0.2 * t);
        assert_eq!(
            run_with(|t| head_with(t, 45.0, 1.0, 50.0), 48.0, creep),
            None
        );
    }

    #[test]
    fn a_brief_tracking_glitch_is_not_a_slip() {
        let glitch = |t: f64| {
            if (20.0..20.15).contains(&t) {
                rot(Vector3::x(), 8.0)
            } else {
                UnitQuaternion::identity()
            }
        };
        assert_eq!(run(48.0, glitch), None);
    }

    #[test]
    fn the_fitted_offset_matches_how_the_controller_sits() {
        let offset = rot(Vector3::new(1.0, 0.2, 0.5), 70.0);
        let mut m = Matrix4::zeros();
        for k in 0..40 {
            let a = rot(
                Vector3::new((k as f64).sin(), 1.0, (k as f64 * 0.7).cos()),
                20.0 + k as f64,
            );
            let b = offset.inverse() * a * offset;
            m += constraint(&a, &b);
        }
        assert!(fit(&m).angle_to(&offset).to_degrees() < 0.01);
        // the multiplication matrices are what they claim
        let (p, q) = (rot(Vector3::x(), 30.0), rot(Vector3::y(), 50.0));
        assert!((left(&wxyz(&p)) * wxyz(&q) - wxyz(&(p * q))).norm() < 1e-12);
        assert!((right(&wxyz(&q)) * wxyz(&p) - wxyz(&(p * q))).norm() < 1e-12);
    }

    /// The cues a capture of `secs` gives, with when, for a controller pair slipping by
    /// `slip(t)` (left, right).
    fn cues(
        slip_check: bool,
        secs: f64,
        slip: impl Fn(f64) -> [UnitQuaternion<f64>; 2],
    ) -> Vec<(f64, Cue)> {
        let world = rot(Vector3::new(0.3, 1.0, -0.2), 25.0);
        let offsets = [
            rot(Vector3::new(1.0, 0.2, 0.5), 70.0),
            rot(Vector3::new(-1.0, 0.3, 0.5), 80.0),
        ];
        let mut capture = HeadsetCapture::new(slip_check);
        let mut out = Vec::new();
        let mut i = 0u64;
        while (i as f64) / 500.0 < secs {
            let t = i as f64 / 500.0;
            let slipped = slip(t);
            let touch = [0, 1].map(|hand| {
                Some(
                    world
                        * head(t - 0.01)
                        * offsets[hand]
                        * slipped[hand]
                        * jitter(i + hand as u64),
                )
            });
            capture.push((t * 1e9) as u64, head(t), touch);
            if i % 10 == 0 {
                let cue = capture.cue();
                if cue != Cue::Recording {
                    out.push((t, cue));
                }
                if matches!(cue, Cue::Done | Cue::Slipped(..)) {
                    break;
                }
            }
            i += 1;
        }
        out
    }

    #[test]
    fn a_capture_cues_once_when_turned_and_again_when_done() {
        let id = UnitQuaternion::identity();
        let got = cues(true, 48.0, |_| [id, id]);
        assert_eq!(got.len(), 2, "{got:?}");
        assert_eq!(got[0].1, Cue::Turned);
        assert!((30.0..32.5).contains(&got[0].0), "{got:?}");
        assert_eq!(got[1].1, Cue::Done);
        assert!((32.0..36.0).contains(&got[1].0), "{got:?}");
    }

    #[test]
    fn a_capture_names_the_controller_that_slipped() {
        let id = UnitQuaternion::identity();
        let slip = |t: f64| {
            [
                id,
                if t >= 10.0 {
                    rot(Vector3::z(), 6.0)
                } else {
                    id
                },
            ]
        };
        let got = cues(true, 48.0, slip);
        let (t, cue) = *got.last().unwrap();
        assert!(
            matches!(cue, Cue::Slipped(1, deg) if deg > SLIP_DEG),
            "{got:?}"
        );
        assert!((10.0..12.0).contains(&t), "{got:?}");
        // without the check, the same capture runs to the end
        let got = cues(false, 48.0, slip);
        assert_eq!(got.last().unwrap().1, Cue::Done, "{got:?}");
    }

    #[test]
    fn a_slip_just_before_the_finish_is_still_reported() {
        let id = UnitQuaternion::identity();
        let (done, _) = *cues(true, 48.0, |_| [id, id]).last().unwrap();
        let at = done - 0.2;
        let slip = move |t: f64| [id, if t >= at { rot(Vector3::x(), 6.0) } else { id }];
        let got = cues(true, 48.0, slip);
        assert!(
            matches!(got.last().unwrap().1, Cue::Slipped(1, _)),
            "{got:?}"
        );
    }

    #[test]
    fn coverage_needs_two_turns_each_way_then_a_nod_and_a_tilt() {
        let mut c = TurnCoverage::default();
        let mut t = 0.0;
        while t < 16.0 {
            c.push(head(t));
            t += 0.01;
        }
        assert!(c.left_deg > 700.0 && c.right_deg < 10.0 && !c.turns_done());
        while t < 32.0 {
            c.push(head(t));
            t += 0.01;
        }
        assert!(
            c.turns_done() && !c.done(),
            "turned, not yet nodded and tilted"
        );
        while t < 40.0 {
            c.push(head(t));
            t += 0.01;
        }
        assert!(c.done());
    }
}
