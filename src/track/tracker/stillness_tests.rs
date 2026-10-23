// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Independent pre-optimization oracle, bitwise differential tests, and an opt-in CPU benchmark.
use super::*;
use crate::calib::synth::{Motion, Rng};
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::hint::black_box;
use std::time::Instant;

fn vector_bits(v: Vector3<f64>) -> [u64; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

type Means = ([u64; 3], [u64; 3]);
thread_local! {
    static CAPTURE: Cell<bool> = const { Cell::new(false) };
    static MEANS: Cell<Option<Means>> = const { Cell::new(None) };
    static ALLOCATIONS: Cell<Option<(usize, usize, usize)>> = const { Cell::new(None) };
}

// Count only the calling thread, only inside `counted`; other parallel tests are unaffected.
struct CountingAllocator;
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let _ = ALLOCATIONS.try_with(|c| {
            if let Some((a, d, bytes)) = c.get() {
                c.set(Some((a + 1, d, bytes + layout.size())));
            }
        });
        unsafe { System.alloc(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let _ = ALLOCATIONS.try_with(|c| {
            if let Some((a, d, bytes)) = c.get() {
                c.set(Some((a + 1, d, bytes + new_size)));
            }
        });
        unsafe { System.realloc(ptr, layout, new_size) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let _ = ALLOCATIONS.try_with(|c| {
            if let Some((a, d, bytes)) = c.get() {
                c.set(Some((a + 1, d, bytes + layout.size())));
            }
        });
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        let _ = ALLOCATIONS.try_with(|c| {
            if let Some((a, d, bytes)) = c.get() {
                c.set(Some((a, d + 1, bytes)));
            }
        });
        unsafe { System.dealloc(ptr, layout) }
    }
}

fn counted<T>(f: impl FnOnce() -> T) -> (T, (usize, usize, usize)) {
    struct Reset;
    impl Drop for Reset {
        fn drop(&mut self) {
            ALLOCATIONS.with(|c| c.set(None));
        }
    }
    ALLOCATIONS.with(|c| {
        assert!(c.get().is_none());
        c.set(Some((0, 0, 0)));
    });
    let reset = Reset;
    let result = f();
    let counts = ALLOCATIONS.with(|c| c.get().unwrap());
    drop(reset);
    (result, counts)
}

// Test-only tap after the real detector's reductions; never compiled into production builds.
pub(super) fn observe_means(gm: Vector3<f64>, am: Vector3<f64>) {
    if CAPTURE.with(Cell::get) {
        MEANS.with(|c| c.set(Some((vector_bits(gm), vector_bits(am)))));
    }
}

fn evaluated(f: impl FnOnce() -> bool) -> (bool, Option<Means>) {
    CAPTURE.with(|c| c.set(true));
    MEANS.with(|c| c.set(None));
    let result = f();
    CAPTURE.with(|c| c.set(false));
    (result, MEANS.with(Cell::get))
}

impl Tracker {
    fn legacy_still(&self, s: &ImuSample) -> bool {
        let p = &self.params;
        let Some(first) = self.imu.front() else {
            return false;
        };
        if s.t.saturating_sub(first.t) < p.still_window_ns {
            return false;
        }
        let (bg, ba) = self
            .history
            .back()
            .map(|h| (h.1.x.bg, h.1.x.ba))
            .or(self.saved_bias.map(|f| (f.x.bg, f.x.ba)))
            .unwrap_or_default();
        let from = s.t.saturating_sub(p.still_window_ns);
        let win: Vec<&ImuSample> = self
            .imu
            .iter()
            .filter(|x| x.t >= from)
            .chain(std::iter::once(s))
            .collect();
        let n = win.len() as f64;
        let gm = win.iter().map(|x| x.gyro).sum::<Vector3<f64>>() / n;
        let am = win.iter().map(|x| x.accel).sum::<Vector3<f64>>() / n;
        observe_means(gm, am);
        (gm - bg).norm() < p.still_gyro
            && ((am - ba).norm() - p.gravity).abs() < p.still_accel_g
            && win.iter().all(|x| {
                (x.gyro - gm).norm() < p.still_gyro_spread
                    && (x.accel - am).norm() < p.still_accel_spread
            })
    }

    fn legacy_imu(&mut self, s: ImuSample) {
        if self.imu.back().is_some_and(|l| s.t <= l.t) {
            return;
        }
        if self
            .imu
            .back()
            .is_some_and(|l| s.t - l.t > self.params.max_imu_gap_ns)
        {
            self.stats.imu_gaps += 1;
            self.camera_updates.iter_mut().for_each(VecDeque::clear);
            self.inspection_frames.clear();
            self.saved_bias = self.history.back().map(|h| h.1).or(self.saved_bias);
            self.history.clear();
            self.imu.clear();
            self.still.clear();
            self.applied.clear();
            self.last_fix = 0;
            self.last_track = 0;
            self.last_confirmed = 0;
            self.track_camera = None;
            self.track_confirmed = false;
            self.refusing_since = 0;
            self.ambiguous_since = 0;
            self.provisional = None;
            self.held = None;
        }
        if let Some((t, q, until)) = self.wrong_lock {
            let bg = self.saved_bias.map_or(Vector3::zeros(), |f| f.x.bg);
            self.wrong_lock = (s.t < until).then(|| {
                (
                    s.t,
                    q * UnitQuaternion::from_scaled_axis(
                        (s.gyro - bg) * (s.t.saturating_sub(t) as f64 * 1e-9),
                    ),
                    until,
                )
            });
        }
        let still = self.legacy_still(&s);
        self.imu.push_back(s);
        self.still.push_back(still);
        let g = self.gravity();
        if let Some(&(t, mut f)) = self.history.back() {
            f.propagate(
                &s.gyro,
                &s.accel,
                (s.t - t) as f64 * 1e-9,
                &g,
                &self.params.noise,
            );
            self.zupt(&mut f, t, s.t, still);
            self.history.push_back((s.t, f));
        }
        self.inspection_frames.retain(|f| f.until > s.t);
        let cutoff = s.t.saturating_sub(self.params.history_ns);
        for updates in &mut self.camera_updates {
            while updates
                .front()
                .is_some_and(|t| *t < cutoff.saturating_sub(CROSS_CHECK_NS))
            {
                updates.pop_front();
            }
        }
        while self.history.len() > 1 && self.history[1].0 <= cutoff {
            self.history.pop_front();
        }
        let oldest = self.history.front().map_or(cutoff, |h| h.0.min(cutoff));
        while self.imu.len() > 1 && self.imu[0].t <= oldest {
            self.imu.pop_front();
            self.still.pop_front();
        }
        while self.applied.front().is_some_and(|a| a.t < oldest) {
            self.applied.pop_front();
        }
        self.note_held();
    }
}

fn tracker(params: Params) -> Tracker {
    Tracker::new(
        1,
        Body {
            leds: Vec::new(),
            imu_pos: Vector3::new(0.015, -0.012, 0.008),
        },
        params,
        3,
    )
}

fn sample(t: u64) -> ImuSample {
    ImuSample {
        t,
        gyro: Vector3::zeros(),
        accel: Vector3::new(0.0, 9.80665, 0.0),
    }
}

fn fixture(rate: u64, wrapped: bool, window: u64) -> (Tracker, ImuSample) {
    let mut tr = tracker(Params {
        still_window_ns: window,
        ..Params::default()
    });
    let dt = 1_000_000_000 / rate;
    let len = (tr.params.history_ns / dt) as usize;
    tr.imu = VecDeque::with_capacity(len + 7);
    if wrapped {
        // Advance the head without clearing it, then cross the physical storage boundary.
        for _ in 0..tr.imu.capacity() - 3 {
            tr.imu.push_back(sample(0));
            tr.imu.pop_front();
        }
    }
    for i in 0..len {
        let mut s = sample(1_000_000_000 + i as u64 * dt);
        let phase = i as f64 * 0.37;
        s.gyro = Vector3::new(phase.sin(), phase.cos(), (phase * 0.7).sin()) * 0.001;
        s.accel += Vector3::new(phase.cos(), phase.sin(), (phase * 0.3).cos()) * 0.01;
        tr.imu.push_back(s);
    }
    assert_eq!(!tr.imu.as_slices().1.is_empty(), wrapped);
    tr.still.resize(tr.imu.len(), false);
    let next = sample(1_000_000_000 + len as u64 * dt);
    (tr, next)
}

fn equivalent(tr: &Tracker, s: &ImuSample) -> bool {
    let expected = evaluated(|| tr.legacy_still(s));
    let actual = evaluated(|| tr.is_still(s));
    assert_eq!(actual, expected, "sample {s:?}, params {:?}", tr.params);
    actual.0
}

#[test]
fn stillness_window_edges_and_bias_priority_match_the_original() {
    let mut tr = tracker(Params::default());
    assert!(!equivalent(&tr, &sample(0)));
    tr.imu.push_back(sample(0));
    assert!(!equivalent(&tr, &sample(199_999_999)));
    assert!(equivalent(&tr, &sample(200_000_000)));
    // Only the boundary sample moves: excluding it reverses the decision.
    tr.imu[0].gyro.x = 0.05;
    assert!(!equivalent(&tr, &sample(200_000_000)));
    assert!(equivalent(&tr, &sample(200_000_001))); // empty selected suffix, new sample once
    tr.params.still_window_ns = 0;
    assert!(equivalent(&tr, &sample(1)));
    tr.params.still_window_ns = 17;
    assert!(!equivalent(&tr, &sample(16)));
    assert!(equivalent(&tr, &sample(18)));

    let bg = Vector3::new(0.1, -0.03, 0.04);
    let ba = Vector3::new(0.4, 0.7, -0.3);
    tr.params.still_window_ns = 200_000_000;
    tr.imu[0] = ImuSample {
        gyro: bg,
        accel: sample(0).accel + ba,
        ..sample(0)
    };
    let s = ImuSample {
        t: 200_000_000,
        ..tr.imu[0]
    };
    assert!(!equivalent(&tr, &s));
    tr.seed_biases(bg, ba, 0.001, 0.01);
    assert!(equivalent(&tr, &s));
    let mut f = tr.saved_bias.unwrap();
    f.x.bg = Vector3::zeros();
    f.x.ba = Vector3::zeros();
    tr.history.push_back((0, f));
    assert!(!equivalent(&tr, &s)); // history overrides a matching saved bias
    tr.history.back_mut().unwrap().1 = tr.saved_bias.unwrap();
    tr.saved_bias.as_mut().unwrap().x.bg = Vector3::zeros();
    assert!(equivalent(&tr, &s)); // matching history overrides the wrong saved bias
}

#[test]
fn stillness_means_and_decisions_match_for_wrapping_growth_and_thresholds() {
    for rate in [250, 500, 1000] {
        for wrapped in [false, true] {
            for window in [0, 17_000_000, 200_000_000, 600_000_000] {
                let (mut tr, s) = fixture(rate, wrapped, window);
                equivalent(&tr, &s);
                let mut rng = Rng::new(rate + window);
                for i in 0..257 {
                    let mut next = sample(s.t + i * 2_000_000);
                    next.gyro = rng.gauss3(0.03);
                    next.accel += rng.gauss3(0.2);
                    equivalent(&tr, &next);
                    tr.imu.push_back(next); // forces growth as well as wrapped logical iteration
                }
            }
        }
    }
    // Adjacent representable values at each strict threshold, with no spread in the window.
    for kind in 0..4 {
        for delta in [-1i64, 0, 1] {
            let mut tr = tracker(Params::default());
            let limit = match kind {
                0 => tr.params.still_gyro,
                1 => tr.params.still_accel_g,
                2 => tr.params.still_gyro_spread,
                _ => tr.params.still_accel_spread,
            };
            let value = f64::from_bits((limit.to_bits() as i64 + delta) as u64);
            let mut a = sample(0);
            let mut b = sample(200_000_000);
            match kind {
                0 => {
                    a.gyro.x = value;
                    b.gyro.x = value;
                }
                1 => {
                    a.accel.y += value;
                    b.accel.y += value;
                }
                2 => {
                    a.gyro.x = -value;
                    b.gyro.x = value;
                }
                _ => {
                    a.accel.x = -value;
                    b.accel.x = value;
                }
            }
            tr.imu.push_back(a);
            equivalent(&tr, &b);
        }
    }
    // Deliberate cancellation makes forward/reverse summation observably different in bits.
    let mut tr = tracker(Params {
        still_window_ns: 3,
        ..Params::default()
    });
    for (t, x) in [(0, 1e16), (1, -1e16), (2, 1.0)] {
        let mut s = sample(t);
        s.gyro.x = x;
        s.accel.x = x;
        tr.imu.push_back(s);
    }
    equivalent(&tr, &sample(3));
}

fn filter_bits(f: &Filter) -> Vec<u64> {
    f.x.q
        .coords
        .iter()
        .chain(f.x.p.iter())
        .chain(f.x.v.iter())
        .chain(f.x.bg.iter())
        .chain(f.x.ba.iter())
        .chain(f.cov.iter())
        .map(|x| x.to_bits())
        .collect()
}

// The handler oracle must track upstream lifecycle changes; only detector selection differs.
#[test]
fn legacy_imu_copies_the_current_handler_except_for_detector_selection() {
    fn body<'a>(source: &'a str, signature: &str) -> &'a str {
        source
            .split_once(signature)
            .unwrap()
            .1
            .split_once("\n    }")
            .unwrap()
            .0
    }
    let actual = body(
        include_str!("../tracker.rs"),
        "    pub fn imu(&mut self, s: ImuSample) {",
    );
    let legacy = body(
        include_str!("stillness_tests.rs"),
        "    fn legacy_imu(&mut self, s: ImuSample) {",
    );
    assert_eq!(
        actual.replace("self.is_still(&s)", "self.legacy_still(&s)"),
        legacy
    );
}

fn assert_state(a: &Tracker, b: &Tracker) {
    assert_eq!(a.still, b.still);
    assert_eq!(a.imu, b.imu);
    assert_eq!(a.stats, b.stats);
    assert_eq!(a.history.len(), b.history.len());
    for ((ta, fa), (tb, fb)) in a.history.iter().zip(&b.history) {
        assert_eq!(ta, tb);
        assert_eq!(filter_bits(fa), filter_bits(fb));
    }
    for (fa, fb) in [
        (a.saved_bias.as_ref(), b.saved_bias.as_ref()),
        (a.acquired_bias.as_ref(), b.acquired_bias.as_ref()),
    ] {
        assert_eq!(fa.map(filter_bits), fb.map(filter_bits));
    }
    assert_eq!(a.held.map(vector_bits), b.held.map(vector_bits));
    assert_eq!(a.bias_ref.map(vector_bits), b.bias_ref.map(vector_bits));
    assert_eq!(
        (a.last_fix, a.last_track, a.last_confirmed),
        (b.last_fix, b.last_track, b.last_confirmed)
    );
    assert_eq!(a.wrong_lock, b.wrong_lock);
    assert_eq!(a.track_camera, b.track_camera);
    assert_eq!(a.track_confirmed, b.track_confirmed);
    assert_eq!(a.refusing_since, b.refusing_since);
    assert_eq!(a.ambiguous_since, b.ambiguous_since);
    assert_eq!(a.provisional, b.provisional);
    assert_eq!(a.engine_labels_off, b.engine_labels_off);
    assert_eq!(a.camera_updates, b.camera_updates);
    assert_eq!(a.searched_at, b.searched_at);
    assert_eq!(a.agreement, b.agreement);
    assert_eq!(a.debug, b.debug);
    assert_eq!(a.misses, b.misses);
    assert_eq!(a.fits, b.fits);
    assert_eq!(a.applied.len(), b.applied.len());
    for (fa, fb) in a.applied.iter().zip(&b.applied) {
        assert_eq!(fa.t, fb.t);
        assert_eq!(fa.cam, fb.cam);
        assert_eq!(fa.blobs, fb.blobs);
        assert_eq!(fa.pairs, fb.pairs);
    }
    assert_eq!(a.inspection_frames.len(), b.inspection_frames.len());
    for (fa, fb) in a.inspection_frames.iter().zip(&b.inspection_frames) {
        assert_eq!(
            (fa.camera, fa.exposure, fa.until, fa.peers),
            (fb.camera, fb.exposure, fb.until, fb.peers)
        );
        assert_eq!(filter_bits(&fa.prior), filter_bits(&fb.prior));
    }
}

// Synthetic accepted exposures and outstanding searches, without camera I/O or frame fitting.
fn populate_camera_bookkeeping(tr: &mut Tracker, now: u64) {
    let prior = tr.history.back().unwrap().1;
    for camera in 0..tr.camera_updates.len() {
        let updates = &mut tr.camera_updates[camera];
        updates.clear();
        // Ordinary 50 Hz exposures over the retained 400 ms plus cross-check margin.
        for age in (0..=22).rev() {
            updates.push_back(now - age * 20_000_000 - camera as u64);
        }
        tr.searched_at[camera] = now - 20_000_000;
        for k in 0..4 {
            let mut held_prior = prior;
            held_prior.x.p.x += (camera * 4 + k) as f64 * 0.001;
            tr.inspection_frames.push_back(InspectionFrame {
                camera,
                exposure: now - (k as u64 + 1) * 20_000_000,
                until: now + [1, 40_000_000, 100_000_000, 500_000_000][k],
                prior: held_prior,
                peers: (camera + k) % 2 == 0,
            });
        }
    }
}

#[test]
fn stillness_through_imu_preserves_camera_expiry_priors_and_gap_cleanup() {
    let (mut a, first) = fixture(500, true, 200_000_000);
    let prior = a.acquisition_prior(&Isometry3::identity(), None);
    a.history.push_back((first.t - 2_000_000, prior));
    a.still.resize(a.imu.len(), false);
    a.last_fix = first.t - 2_000_000;
    a.last_track = a.last_fix;
    a.last_confirmed = a.last_fix;
    a.track_camera = Some(1);
    a.track_confirmed = true;
    a.refusing_since = first.t - 4_000_000;
    a.ambiguous_since = first.t - 6_000_000;
    a.provisional = Some(first.t - 8_000_000);
    populate_camera_bookkeeping(&mut a, first.t);
    // Exact strict-cutoff boundary, with expired and retained entries on both sides.
    let cutoff = first.t - a.params.history_ns - CROSS_CHECK_NS;
    a.camera_updates[0] = VecDeque::from([cutoff - 1, cutoff, cutoff + 1]);
    let mut b = a.clone();
    let original_frames = a.inspection_frames.clone();
    a.imu(first);
    b.legacy_imu(first);
    assert_state(&a, &b);
    assert_eq!(a.camera_updates[0], [cutoff, cutoff + 1]);
    assert_eq!(a.inspection_frames.len(), 12);
    let exact_deadline = sample(first.t + 1);
    a.imu(exact_deadline);
    b.legacy_imu(exact_deadline);
    assert_state(&a, &b);
    assert_eq!(a.camera_updates[0], [cutoff + 1]);
    assert_eq!(a.inspection_frames.len(), 9);
    for held in &a.inspection_frames {
        let old = original_frames
            .iter()
            .find(|f| f.camera == held.camera && f.exposure == held.exposure)
            .unwrap();
        assert_eq!(filter_bits(&held.prior), filter_bits(&old.prior));
        assert_eq!((held.until, held.peers), (old.until, old.peers));
    }
    // Rejected samples do not even expire camera state.
    let before = a.clone();
    for rejected in [first, exact_deadline] {
        a.imu(rejected);
        b.legacy_imu(rejected);
        assert_state(&a, &before);
        assert_state(&a, &b);
    }
    // A gap must clear unexpired snapshots and camera times that ordinary expiry would retain.
    let gap = sample(exact_deadline.t + a.params.max_imu_gap_ns + 1);
    assert!(gap.t < a.inspection_frames.back().unwrap().until);
    assert!(
        a.camera_updates[1].back().unwrap()
            > &gap.t.saturating_sub(a.params.history_ns + CROSS_CHECK_NS)
    );
    a.imu(gap);
    b.legacy_imu(gap);
    assert_state(&a, &b);
    assert!(a.inspection_frames.is_empty());
    assert!(a.camera_updates.iter().all(VecDeque::is_empty));
    assert!(a.history.is_empty());
    assert_eq!(a.imu.len(), 1);
    assert_eq!(a.stats.imu_gaps, 1);
}

#[test]
fn stillness_through_imu_preserves_every_filter_bit_and_rejects_old_samples() {
    for seed in [1, 7, 93] {
        let mut rng = Rng::new(seed);
        let motion = Motion::random(&mut rng, Vector3::new(0.0, 1.0, 0.0));
        let mut a = tracker(Params::default());
        let bg = Vector3::new(0.01, -0.005, 0.003);
        let ba = Vector3::new(0.02, 0.1, -0.05);
        a.seed_biases(bg, ba, 0.002, 0.03);
        let mut b = a.clone();
        let mut t = 1_000_000_000;
        let mut still_count = 0;
        for i in 0..2000 {
            t += if i == 1200 {
                a.params.max_imu_gap_ns + 1
            } else {
                2_000_000
            };
            // Simulated acquisition after startup and after the discontinuity.
            if i == 1 || i == 1201 {
                for tr in [&mut a, &mut b] {
                    let f = tr.acquisition_prior(&Isometry3::identity(), tr.saved_bias.as_ref());
                    tr.history.push_back((t - 2_000_000, f));
                    tr.last_fix = t - 2_000_000;
                }
            }
            if i % 100 == 2 || i == 1199 {
                for tr in [&mut a, &mut b] {
                    tr.inspection_frames.clear();
                    populate_camera_bookkeeping(tr, t - 2_000_000);
                }
            }
            let sec = i as f64 * 0.002;
            let mut s = sample(t);
            if (400..1100).contains(&i) {
                s.gyro = motion.gyro(sec);
                s.accel = motion.accel(sec, &a.body.imu_pos, &Vector3::new(0.0, 9.80665, 0.0));
            }
            s.gyro += bg + rng.gauss3(0.0001);
            s.accel += ba + rng.gauss3(0.001);
            a.imu(s);
            b.legacy_imu(s);
            assert_state(&a, &b);
            if i == 1200 {
                assert_eq!(a.imu.len(), 1);
                assert_eq!(a.still.front(), Some(&false));
                assert!(a.history.is_empty());
            }
            still_count += usize::from(*a.still.back().unwrap());
            if i % 31 == 0 {
                let before = a.clone();
                for rejected in [s, ImuSample { t: t - 1, ..s }] {
                    a.imu(rejected);
                    b.legacy_imu(rejected);
                    assert_state(&a, &before);
                    assert_state(&a, &b);
                }
            }
        }
        assert!(still_count > 500);
        assert_eq!(a.stats.imu_gaps, 1);
    }
}

#[test]
fn stillness_detector_does_not_allocate() {
    for rate in [250, 500, 1000] {
        for wrapped in [false, true] {
            for window in [0, 17_000_000, 200_000_000, 600_000_000] {
                let (tr, s) = fixture(rate, wrapped, window);
                let (actual, counts) = counted(|| black_box(tr.is_still(black_box(&s))));
                assert_eq!(actual, tr.legacy_still(&s));
                assert_eq!(
                    counts,
                    (0, 0, 0),
                    "rate={rate}, wrapped={wrapped}, window={window}"
                );
            }
        }
    }
}

#[cfg(target_os = "linux")]
fn cpu_ns() -> u128 {
    let mut time = std::mem::MaybeUninit::<libc::timespec>::uninit();
    // A thread CPU clock excludes time descheduled by other workers or game holds.
    assert_eq!(
        unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, time.as_mut_ptr()) },
        0
    );
    let time = unsafe { time.assume_init() };
    time.tv_sec as u128 * 1_000_000_000 + time.tv_nsec as u128
}

fn summary(mut x: Vec<f64>) -> (f64, f64, f64) {
    x.sort_by(f64::total_cmp);
    (x[x.len() / 2], x[0], x[x.len() - 1])
}

#[test]
#[ignore = "CPU benchmark: run in release with --ignored --exact --nocapture --test-threads=1"]
fn stillness_cpu_benchmark() {
    println!("case,operation,implementation,median_ns,min_ns,max_ns,allocation_requests,deallocs,requested_bytes");
    for (name, rate, wrapped, window, startup, moving) in [
        ("250-contiguous", 250, false, 200_000_000, false, false),
        ("250-wrapped", 250, true, 200_000_000, false, false),
        ("500-contiguous", 500, false, 200_000_000, false, false),
        ("500-wrapped", 500, true, 200_000_000, false, false),
        ("1000-contiguous", 1000, false, 200_000_000, false, false),
        ("1000-wrapped", 1000, true, 200_000_000, false, false),
        (
            "500-contiguous-cameras",
            500,
            false,
            200_000_000,
            false,
            false,
        ),
        ("500-wrapped-cameras", 500, true, 200_000_000, false, false),
        ("startup", 500, false, 200_000_000, true, false),
        ("short", 500, true, 20_000_000, false, false),
        ("zero", 500, true, 0, false, false),
        ("custom", 500, true, 300_000_000, false, false),
        ("moving", 500, true, 200_000_000, false, true),
    ] {
        let (mut tr, s) = fixture(rate, wrapped, window);
        if startup {
            tr.imu.truncate(1);
            tr.still.truncate(1);
            tr.imu[0].t = s.t - 2_000_000;
        }
        if moving {
            for x in &mut tr.imu {
                x.gyro.x += 0.5;
            }
        }
        equivalent(&tr, &s);
        let cameras = name.ends_with("-cameras");
        for full in [false, true] {
            // Ten independent 400 ms batches keep camera bookkeeping populated; cloning and
            // fixture injection are outside timing, with the same 2,000 total full calls.
            let chunks = if full && cameras { 10 } else { 1 };
            let calls = if full && cameras {
                200
            } else if full {
                2000
            } else {
                30_000
            };
            let samples: Vec<_> = (0..calls)
                .map(|i| {
                    let phase = i as f64 * 0.037;
                    let mut next = ImuSample {
                        t: s.t + i * (1_000_000_000 / rate),
                        ..s
                    };
                    if (calls / 4..3 * calls / 4).contains(&i) {
                        next.gyro = Vector3::new(phase.sin(), phase.cos(), 0.1) * 0.4;
                        next.accel += Vector3::new(phase.cos(), phase.sin(), 0.2) * 0.3;
                    }
                    next
                })
                .collect();
            let prepare = || {
                let mut work = tr.clone();
                if full {
                    let f = work.acquisition_prior(&Isometry3::identity(), None);
                    work.history.push_back((s.t - 1_000_000_000 / rate, f));
                    work.last_fix = s.t;
                    if cameras {
                        populate_camera_bookkeeping(&mut work, s.t - 1_000_000_000 / rate);
                    }
                }
                work
            };
            if full {
                let (mut original, mut actual) = (prepare(), prepare());
                for &next in &samples {
                    original.legacy_imu(next);
                    actual.imu(next);
                    assert_state(&actual, &original);
                }
            }
            let mut times = [Vec::new(), Vec::new()];
            let mut counts = [(0, 0, 0); 2];
            for round in 0..21 {
                for implementation in if round % 2 == 0 { [0, 1] } else { [1, 0] } {
                    let mut works: Vec<_> = (0..chunks).map(|_| prepare()).collect();
                    // Allocation probe is separate from timing; both paths consume real output.
                    counts[implementation] = if full {
                        let mut probe = works[0].clone();
                        counted(|| {
                            if implementation == 0 {
                                probe.legacy_imu(s)
                            } else {
                                probe.imu(s)
                            }
                        })
                        .1
                    } else {
                        counted(|| {
                            black_box(if implementation == 0 {
                                works[0].legacy_still(&s)
                            } else {
                                works[0].is_still(&s)
                            })
                        })
                        .1
                    };
                    let start = Instant::now();
                    #[cfg(target_os = "linux")]
                    let cpu_start = cpu_ns();
                    for work in &mut works {
                        for &next in &samples {
                            if full {
                                if implementation == 0 {
                                    work.legacy_imu(black_box(next));
                                } else {
                                    work.imu(black_box(next));
                                }
                                black_box((
                                    &work.history,
                                    &work.still,
                                    &work.stats,
                                    &work.camera_updates,
                                    &work.inspection_frames,
                                ));
                            } else {
                                black_box(if implementation == 0 {
                                    black_box(&work).legacy_still(black_box(&s))
                                } else {
                                    black_box(&work).is_still(black_box(&s))
                                });
                            }
                        }
                    }
                    let wall_ns =
                        start.elapsed().as_nanos() as f64 / (calls as usize * chunks) as f64;
                    #[cfg(target_os = "linux")]
                    let elapsed_ns =
                        (cpu_ns() - cpu_start) as f64 / (calls as usize * chunks) as f64;
                    #[cfg(not(target_os = "linux"))]
                    let elapsed_ns = wall_ns;
                    times[implementation].push(elapsed_ns);
                    println!(
                        "raw,{name},{full},{round},{implementation},{elapsed_ns:.2},{wall_ns:.2}"
                    );
                    black_box(works);
                }
            }
            for implementation in 0..2 {
                let (median, min, max) = summary(std::mem::take(&mut times[implementation]));
                let (a, d, bytes) = counts[implementation];
                println!(
                    "{name},{},{},{median:.2},{min:.2},{max:.2},{a},{d},{bytes}",
                    if full { "imu" } else { "detector" },
                    if implementation == 0 {
                        "original"
                    } else {
                        "actual"
                    }
                );
            }
        }
    }
}
