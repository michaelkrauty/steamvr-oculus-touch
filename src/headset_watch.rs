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

use crate::register::{Options, RelativeTurns, SEED_EVERY_NS, SEED_MIN_PAIRS, SEED_WINDOW_NS};

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
/// turns each way less some slack, and a clear nod and tilt, each at least half the range to
/// each side of level (a nod up and a nod down; a tilt toward each shoulder).
pub const TURN_DEG: f64 = 660.0;
pub const NOD_DEG: f64 = 30.0;
/// The quick capture's turn each way: a look over each shoulder. With the nod and tilt it turns
/// the headset about all three axes, which the fit needs; it needs no more turning than that.
pub const QUICK_TURN_DEG: f64 = 90.0;
/// The shortest capture that can end. The fit takes 500 pairs of headset and controller poses
/// (one in ten of a controller's published motion samples), which come at 41 to 46 a second in
/// recorded headset captures, so 11 to 12 s held; the stretch it finds as held loses a little at
/// each end, and the shortest piece of those captures it accepted was 12.2 s long.
pub const MIN_CAPTURE_NS: u64 = 15_000_000_000;
/// How often the quick capture takes the headset's rotation for the fit's rotation test: the
/// fit's pairs are one in ten of a controller's published motion samples, which come at 41 to 46
/// a second in recorded headset captures.
const PAIR_NS: u64 = 23_000_000;
/// How often the quick capture runs the fit's rotation test once its other targets are met.
const TEST_NS: u64 = 250_000_000;

/// The rotations a headset capture has given the fit so far, and whether the fit's rotation test
/// passes on them. The fit tests twice: the search for the stretch on the headset fits windows of
/// the capture (`register::bootstrap_span`), one of which must pass, and the final fit tests the
/// whole stretch, about the whole capture when the controllers stay on the headset.
///
/// The headset's rotation stands in for each controller's: a controller held to the headset
/// turns by the headset's relative rotations seen through its hold (b = Y⁻¹·a·Y), by the same
/// angles about axes turned by one fixed Y, which leaves the fit's axis ratio as it is; and the
/// headset is tracked throughout.
struct FitRotations {
    pairs: usize,
    last_ns: Option<u64>,
    all: RelativeTurns,
    /// The search's windows: where each starts (ns, pairs before it) and its rotations so far.
    windows: Vec<(u64, usize, RelativeTurns)>,
    /// When the test last ran, and its result.
    tested: Option<(u64, bool)>,
}

impl FitRotations {
    fn new() -> FitRotations {
        FitRotations {
            pairs: 0,
            last_ns: None,
            all: turns(),
            windows: Vec::new(),
            tested: None,
        }
    }

    fn push(&mut self, t_ns: u64, hmd: UnitQuaternion<f64>) {
        if self.last_ns.is_some_and(|last| t_ns < last + PAIR_NS) {
            return;
        }
        self.last_ns = Some(t_ns);
        if self
            .windows
            .last()
            .is_none_or(|(start, ..)| t_ns >= start + SEED_EVERY_NS)
        {
            self.windows.push((t_ns, self.pairs, turns()));
        }
        for (start, _, window) in &mut self.windows {
            if t_ns <= *start + SEED_WINDOW_NS {
                window.push(hmd, hmd);
            }
        }
        self.all.push(hmd, hmd);
        self.pairs += 1;
    }

    /// Whether the fit's rotation test passes on the rotations so far, run at most every
    /// `TEST_NS`.
    fn pass(&mut self) -> bool {
        let now = self.last_ns.unwrap_or(0);
        if let Some((_, pass)) = self.tested.filter(|(at, _)| now < at + TEST_NS) {
            return pass;
        }
        let enough =
            |turns: &RelativeTurns| turns.axis_ratio() >= Options::headset().min_axis_ratio;
        let pass = enough(&self.all)
            && self
                .windows
                .iter()
                .any(|(_, before, window)| before + SEED_MIN_PAIRS <= self.pairs && enough(window));
        self.tested = Some((now, pass));
        pass
    }
}

fn turns() -> RelativeTurns {
    RelativeTurns::new(Options::headset().min_relative_rad)
}

/// How far the headset has turned each way and nodded and tilted (SteamVR: +Y up, -Z forward).
pub struct TurnCoverage {
    /// How far (degrees) to turn each way.
    turn_deg: f64,
    yaw: Option<f64>,
    left_deg: f64,
    right_deg: f64,
    pitch: Option<(f64, f64)>,
    roll: Option<(f64, f64)>,
}

/// Whether the range reaches half of `NOD_DEG` to each side of level: both nods, both tilts.
fn both_ways(range: Option<(f64, f64)>) -> bool {
    range.is_some_and(|(lo, hi)| lo <= -NOD_DEG / 2.0 && hi >= NOD_DEG / 2.0)
}

fn widen(range: &mut Option<(f64, f64)>, v: f64) {
    let r = range.get_or_insert((v, v));
    r.0 = r.0.min(v);
    r.1 = r.1.max(v);
}

impl TurnCoverage {
    pub fn new(turn_deg: f64) -> TurnCoverage {
        TurnCoverage {
            turn_deg,
            yaw: None,
            left_deg: 0.0,
            right_deg: 0.0,
            pitch: None,
            roll: None,
        }
    }

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
        self.left_deg >= self.turn_deg && self.right_deg >= self.turn_deg
    }

    pub fn done(&self) -> bool {
        self.turns_done() && both_ways(self.pitch) && both_ways(self.roll)
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
    /// The quick capture's rotations, for the fit's rotation test.
    fit: Option<FitRotations>,
    /// The first and the latest sample's time.
    span_ns: Option<(u64, u64)>,
    turned: bool,
    slipped: Option<(usize, f64)>,
    /// The instants pushed so far.
    pairs: usize,
}

/// How far a headset capture has got, for a display while it records.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct Progress {
    /// Headset poses taken, each with the controllers' at the same instant.
    pub pairs: usize,
    /// Seconds from the first pose to the latest.
    pub seconds: f64,
    /// Degrees turned to the left and to the right, and how far each needs.
    pub turn_left_deg: f64,
    pub turn_right_deg: f64,
    pub needed_deg: f64,
    /// The lowest and highest pitch (looking down, up) and roll (tilting right, left) so far,
    /// degrees from level, and how far each side of level both need.
    pub pitch_deg: Option<(f64, f64)>,
    pub roll_deg: Option<(f64, f64)>,
    pub nod_needed_deg: f64,
}

impl HeadsetCapture {
    /// `quick`: a look over each shoulder for the turns, given in any order, so no cue between
    /// them and the nods; the fit's own rotation test decides when the nods and tilts are enough.
    pub fn new(slip_check: bool, quick: bool) -> HeadsetCapture {
        HeadsetCapture {
            watch: slip_check.then(Default::default),
            coverage: TurnCoverage::new(if quick { QUICK_TURN_DEG } else { TURN_DEG }),
            fit: quick.then(FitRotations::new),
            span_ns: None,
            turned: quick,
            slipped: None,
            pairs: 0,
        }
    }

    pub fn progress(&self) -> Progress {
        let c = &self.coverage;
        Progress {
            pairs: self.pairs,
            seconds: self.span_ns.map_or(0.0, |(first, last)| {
                last.saturating_sub(first) as f64 * 1e-9
            }),
            turn_left_deg: c.left_deg,
            turn_right_deg: c.right_deg,
            needed_deg: c.turn_deg,
            pitch_deg: c.pitch,
            roll_deg: c.roll,
            nod_needed_deg: NOD_DEG / 2.0,
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
        self.pairs += 1;
        self.span_ns = Some((self.span_ns.map_or(t_ns, |s| s.0), t_ns));
        if let Some(fit) = self.fit.as_mut() {
            fit.push(t_ns, hmd);
        }
        let Some(watch) = self.watch.as_mut() else {
            return;
        };
        for (hand, (w, touch)) in watch.iter_mut().zip(touch).enumerate() {
            if let (None, Some(touch)) = (self.slipped, touch) {
                self.slipped = w.push(t_ns, hmd, touch).map(|deg| (hand, deg));
            }
        }
    }

    /// The cue for now: a slip first, then done (once no shift is pending, the capture is long
    /// enough for the fit and, quick, turned enough for the fit's rotation test), then (once)
    /// turned.
    pub fn cue(&mut self) -> Cue {
        if let Some((hand, deg)) = self.slipped {
            return Cue::Slipped(hand, deg);
        }
        let unsettled = self.watch.iter().flatten().any(RigidWatch::unsettled);
        let long_enough = self
            .span_ns
            .is_some_and(|(first, last)| last.saturating_sub(first) >= MIN_CAPTURE_NS);
        if self.coverage.done()
            && !unsettled
            && long_enough
            && self.fit.as_mut().is_none_or(FitRotations::pass)
        {
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

    /// After the turns, nodding both ways but tilting toward one shoulder only (either shoulder),
    /// or the other way round: the capture does not end on a one-sided motion, however far it
    /// goes (35 degrees one way covers more range than two 15-degree sides would).
    #[test]
    fn one_sided_nods_or_tilts_do_not_finish_the_capture() {
        for (one_sided_nod, sign) in [(true, 1.0), (true, -1.0), (false, 1.0), (false, -1.0)] {
            let mut c = TurnCoverage::new(TURN_DEG);
            let turning = 720.0 / 45.0;
            let mut t = 0.0;
            while t < 2.0 * turning + 20.0 {
                let mut hmd = head_with(t, 45.0, 0.0, 0.0);
                if t >= 2.0 * turning {
                    let since = t - 2.0 * turning;
                    let nod = 35.0 * (2.0 * since).sin();
                    let tilt = 35.0 * (1.3 * since).sin();
                    let one_way = |v: f64| sign * v.abs();
                    let (nod, tilt) = if one_sided_nod {
                        (one_way(nod), tilt)
                    } else {
                        (nod, one_way(tilt))
                    };
                    hmd = rot(Vector3::x(), nod) * rot(Vector3::z(), tilt);
                }
                c.push(hmd);
                t += 0.01;
            }
            assert!(c.turns_done());
            assert!(!c.done(), "one_sided_nod {one_sided_nod} sign {sign}");
        }
        // both ways each: done
        let mut c = TurnCoverage::new(TURN_DEG);
        let mut t = 0.0;
        while t < 2.0 * 720.0 / 45.0 + 20.0 {
            c.push(head_with(t, 45.0, 0.0, 20.0));
            t += 0.01;
        }
        assert!(c.done());
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
        cues_of(false, head, slip_check, secs, slip)
    }

    /// As `cues`, for the head's motion `head` in a full or quick capture.
    fn cues_of(
        quick: bool,
        head: impl Fn(f64) -> UnitQuaternion<f64>,
        slip_check: bool,
        secs: f64,
        slip: impl Fn(f64) -> [UnitQuaternion<f64>; 2],
    ) -> Vec<(f64, Cue)> {
        let world = rot(Vector3::new(0.3, 1.0, -0.2), 25.0);
        let offsets = [
            rot(Vector3::new(1.0, 0.2, 0.5), 70.0),
            rot(Vector3::new(-1.0, 0.3, 0.5), 80.0),
        ];
        let mut capture = HeadsetCapture::new(slip_check, quick);
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

    /// The progress shown while recording counts the poses and follows the turns and nods.
    #[test]
    fn progress_follows_the_head() {
        for (quick, needed) in [(false, TURN_DEG), (true, QUICK_TURN_DEG)] {
            let mut capture = HeadsetCapture::new(false, quick);
            assert_eq!(capture.progress().pairs, 0);
            assert_eq!(capture.progress().pitch_deg, None);
            // 90 degrees to the left over 3 s, looking 20 degrees down
            for i in 0..=1500u64 {
                let t = i as f64 / 500.0;
                let hmd = rot(Vector3::y(), 30.0 * t) * rot(Vector3::x(), -20.0);
                capture.push(i * 2_000_000, hmd, [None; 2]);
            }
            let p = capture.progress();
            assert_eq!(p.pairs, 1501);
            assert!((p.seconds - 3.0).abs() < 1e-9, "{p:?}");
            assert!((p.turn_left_deg - 90.0).abs() < 0.1, "{p:?}");
            assert!(p.turn_right_deg < 0.1, "{p:?}");
            assert_eq!(p.needed_deg, needed);
            let (low, high) = p.pitch_deg.unwrap();
            assert!(
                (low + 20.0).abs() < 0.1 && (high + 20.0).abs() < 0.1,
                "{p:?}"
            );
            assert_eq!(p.nod_needed_deg, NOD_DEG / 2.0);
        }
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
        let mut c = TurnCoverage::new(TURN_DEG);
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

    /// The quick capture's motion, `speed` times as fast as a calm one: looking down and up and
    /// tilting toward each shoulder by `deg` degrees (4 s), then looking over each shoulder, 100
    /// degrees each way (6 s), then still.
    fn quick_head_by(t: f64, speed: f64, deg: f64) -> UnitQuaternion<f64> {
        let t = t * speed;
        let (mut nod, mut tilt, mut yaw) = (0.0, 0.0, 0.0);
        if t < 2.0 {
            nod = deg * (std::f64::consts::PI * t).sin();
        } else if t < 4.0 {
            tilt = deg * (std::f64::consts::PI * (t - 2.0)).sin();
        } else if t < 10.0 {
            yaw = 100.0 * (std::f64::consts::PI * (t - 4.0) / 3.0).sin();
        }
        rot(Vector3::y(), yaw) * rot(Vector3::x(), nod) * rot(Vector3::z(), tilt)
    }

    fn quick_head(t: f64, speed: f64) -> UnitQuaternion<f64> {
        quick_head_by(t, speed, 20.0)
    }

    /// What the fit makes of `secs` of the head's motion, paired as the fit pairs a headset
    /// capture: the headset's pose and a controller's held to it, one in ten of the controller's
    /// samples, 43 a second; the stretch on the headset is searched for first, as setup does.
    fn fit_rotation_test(
        head: impl Fn(f64) -> UnitQuaternion<f64>,
        secs: f64,
    ) -> Result<(), String> {
        use crate::register::{bootstrap_span, solve};
        let iso = |x: f64, y: f64, z: f64, r: UnitQuaternion<f64>| {
            nalgebra::Isometry3::from_parts(nalgebra::Translation3::new(x, y, z), r)
        };
        let world = iso(0.3, -0.2, 0.5, rot(Vector3::new(0.3, 1.0, -0.2), 25.0));
        let hold = iso(0.08, 0.02, -0.05, rot(Vector3::new(1.0, 0.2, 0.5), 70.0));
        let timed: Vec<_> = (0..)
            .map(|k| k as f64 / 43.0)
            .take_while(|&t| t < secs)
            .map(|t| {
                let headset = iso(0.0, 1.7, 0.0, head(t));
                ((t * 1e9) as u64, headset, world.inverse() * headset * hold)
            })
            .collect();
        let span = bootstrap_span(&timed).map_err(|e| e.unwrap_or("no stretch found".into()))?;
        let pairs: Vec<_> = timed[span].iter().map(|p| (p.1, p.2)).collect();
        solve(&pairs, &Options::headset()).map(|_| ())
    }

    #[test]
    fn a_quick_capture_ends_only_on_rotations_the_fit_accepts() {
        let id = UnitQuaternion::identity();
        // nods and tilts of 16 or 20 degrees, a second each way, turn the fit's chain of
        // relative rotations of 20 degrees about the vertical only
        for (deg, speed, ends) in [
            (16.0, 1.0, false),
            (20.0, 1.0, false),
            (20.0, 0.6, true),
            (30.0, 1.0, true),
            (30.0, 2.0, true),
        ] {
            let head = move |t: f64| quick_head_by(t, speed, deg);
            let got = cues_of(true, head, true, 60.0, |_| [id, id]);
            if ends {
                let &(t, cue) = got.last().unwrap();
                assert_eq!(cue, Cue::Done, "{deg} deg at {speed}: {got:?}");
                let fit = fit_rotation_test(head, t);
                assert!(
                    fit.is_ok(),
                    "{deg} deg at {speed}: done at {t} s, then {fit:?}"
                );
            } else {
                assert!(got.is_empty(), "{deg} deg at {speed}: {got:?}");
                assert!(
                    fit_rotation_test(head, 60.0).is_err(),
                    "{deg} deg at {speed}"
                );
            }
        }
    }

    #[test]
    fn a_quick_capture_needs_the_rotations_within_one_window_of_the_fits_search() {
        let id = UnitQuaternion::identity();
        // a small nod and tilt, the looks over each shoulder, and only half a minute later a
        // deep nod: the whole capture turns about two axes, no 25 s of it does
        let head = |t: f64| {
            let swing = |from: f64, deg: f64| {
                if (from..from + 1.0).contains(&t) {
                    deg * (2.0 * std::f64::consts::PI * (t - from)).sin()
                } else {
                    0.0
                }
            };
            let yaw = if (2.0..8.0).contains(&t) {
                100.0 * (std::f64::consts::PI * (t - 2.0) / 3.0).sin()
            } else {
                0.0
            };
            rot(Vector3::y(), yaw)
                * rot(Vector3::x(), swing(0.0, 16.0) + swing(32.0, 40.0))
                * rot(Vector3::z(), swing(1.0, 16.0))
        };
        assert!(fit_rotation_test(head, 60.0).is_err());
        assert!(cues_of(true, head, true, 60.0, |_| [id, id]).is_empty());
    }

    #[test]
    fn a_quick_capture_needs_the_rotations_over_the_whole_capture() {
        let id = UnitQuaternion::identity();
        // looking over each shoulder again and again for 40 s, then nodding and tilting: the
        // last 25 s turn about all three axes, the whole capture about the vertical almost only
        let head = |t: f64| {
            if t < 40.0 {
                rot(Vector3::y(), 100.0 * (std::f64::consts::PI * t / 3.0).sin())
            } else {
                quick_head_by(t - 40.0, 1.0, 25.0)
            }
        };
        assert!(fit_rotation_test(head, 60.0).is_err());
        assert!(cues_of(true, head, true, 60.0, |_| [id, id]).is_empty());
    }

    #[test]
    fn a_quick_capture_ends_soon_after_the_fit_would_accept_it() {
        let id = UnitQuaternion::identity();
        // nods and tilts too small and brisk for the fit, then at 20 s a deeper nod
        let head = |t: f64| {
            let deeper = if (20.0..22.0).contains(&t) {
                30.0 * (std::f64::consts::PI * (t - 20.0)).sin()
            } else {
                0.0
            };
            rot(Vector3::x(), deeper) * quick_head(t, 1.0)
        };
        let accepted = (0..)
            .map(|k| k as f64 * 0.05)
            .find(|&t| t > 60.0 || fit_rotation_test(head, t).is_ok())
            .unwrap();
        assert!((20.0..22.0).contains(&accepted), "{accepted}");
        let got = cues_of(true, head, true, 60.0, |_| [id, id]);
        assert_eq!(got.len(), 1, "{got:?}");
        assert_eq!(got[0].1, Cue::Done);
        assert!(
            (accepted - 0.1..accepted + 0.5).contains(&got[0].0),
            "done at {} s, the fit accepts from {accepted} s",
            got[0].0
        );
    }

    #[test]
    fn a_quick_capture_ends_after_a_nod_a_tilt_and_a_look_over_each_shoulder() {
        let id = UnitQuaternion::identity();
        // calm: done once the second look ends, with no cue between the nods and the turns
        let calm = cues_of(true, |t| quick_head(t / 2.4, 1.0), true, 30.0, |_| [id, id]);
        assert_eq!(calm.len(), 1, "{calm:?}");
        assert_eq!(calm[0].1, Cue::Done);
        assert!((16.3..16.9).contains(&calm[0].0), "{calm:?}");
        // brisk and deeper: the motion is over in 5 s, but the fit needs longer
        let brisk = cues_of(
            true,
            |t| quick_head_by(t, 2.0, 30.0),
            true,
            30.0,
            |_| [id, id],
        );
        assert_eq!(brisk.len(), 1, "{brisk:?}");
        assert_eq!(brisk[0].1, Cue::Done);
        let min_s = MIN_CAPTURE_NS as f64 * 1e-9;
        assert!(
            (min_s - 0.01..min_s + 0.1).contains(&brisk[0].0),
            "{brisk:?}"
        );
        // the full capture asks two turns each way of the same motion: it keeps recording
        assert!(cues_of(false, |t| quick_head(t, 1.0), true, 30.0, |_| [id, id]).is_empty());
        // without the looks over the shoulders the quick capture does not end either
        let nods_only = |t: f64| quick_head(t.min(4.0), 1.0);
        assert!(cues_of(true, nods_only, true, 30.0, |_| [id, id]).is_empty());
    }

    #[test]
    fn a_quick_capture_names_the_controller_that_slipped() {
        let id = UnitQuaternion::identity();
        let slip = |t: f64| [if t >= 6.0 { rot(Vector3::x(), 6.0) } else { id }, id];
        let got = cues_of(true, |t| quick_head(t / 1.4, 1.0), true, 30.0, slip);
        assert!(
            matches!(got.last().unwrap().1, Cue::Slipped(0, _)),
            "{got:?}"
        );
    }
}
