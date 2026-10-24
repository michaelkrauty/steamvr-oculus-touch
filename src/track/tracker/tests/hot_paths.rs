// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Pre-batch oracles deliberately retain their original algorithms. Changes here require
//! checking equivalence against the baseline, not copying the optimized implementation.
use super::super::stillness_tests::{assert_state, counted, filter_bits};
use super::*;

impl Tracker {
    fn original_state_at(&self, t: u64) -> Option<(usize, Filter)> {
        let k = self.history.partition_point(|&(ts, _)| ts <= t);
        if k == 0 {
            return None;
        }
        let (tk, mut f) = self.history[k - 1];
        if t > tk {
            let s = self.imu.iter().find(|s| s.t > tk)?;
            f.propagate(
                &s.gyro,
                &s.accel,
                (t - tk) as f64 * 1e-9,
                &self.gravity(),
                &self.params.noise,
            );
        }
        Some((k - 1, f))
    }

    fn original_steady_up(&self, t: u64, ba: Vector3<f64>) -> Option<Vector3<f64>> {
        let k = self.imu.partition_point(|s| s.t <= t);
        let from = t.saturating_sub(self.params.steady_window_ns);
        // the window must be covered: a sample at or before its start
        if k == 0 || self.imu.front().is_none_or(|s| s.t > from) {
            return None;
        }
        let win: Vec<&ImuSample> = self.imu.iter().take(k).filter(|s| s.t >= from).collect();
        let mean = win.iter().map(|s| s.accel).sum::<Vector3<f64>>() / win.len() as f64;
        let steady = win
            .iter()
            .all(|s| (s.accel - mean).norm() < self.params.steady_accel)
            && ((mean - ba).norm() - self.params.gravity).abs() < self.params.steady_accel;
        steady.then(|| (mean - ba).normalize())
    }

    fn original_others_set_prior(&self, camera: usize, t_e: u64) -> bool {
        self.camera_updates.iter().enumerate().any(|(c, updates)| {
            c != camera
                && updates
                    .iter()
                    .rev()
                    .any(|&t| t <= t_e && t_e - t <= CROSS_CHECK_NS)
        })
    }

    fn original_rebuild(&mut self, k: usize, first: Applied, posterior: Filter) {
        let later: Vec<Applied> = self
            .applied
            .iter()
            .filter(|a| a.t > first.t)
            .cloned()
            .collect();
        self.applied.retain(|a| a.t <= first.t);
        self.history.truncate(k + 1);
        let (g, n) = (self.gravity(), self.params.noise);
        let (mut t, mut f) = (first.t, posterior);
        self.history.push_back((t, f));
        self.applied.push_back(first);
        let imu: Vec<(ImuSample, bool)> = self
            .imu
            .iter()
            .zip(&self.still)
            .filter(|(s, _)| s.t > t)
            .map(|(s, st)| (*s, *st))
            .collect();
        let (mut i, mut m) = (0, 0);
        loop {
            let next_m = later.get(m).map(|a| a.t);
            let next_s = imu.get(i).map(|s| s.0.t);
            match (next_m, next_s) {
                (Some(tm), Some(ts)) if tm <= ts => {
                    f.propagate(
                        &imu[i].0.gyro,
                        &imu[i].0.accel,
                        (tm - t) as f64 * 1e-9,
                        &g,
                        &n,
                    );
                    t = tm;
                    let a = &later[m];
                    match vision::fit_pairs(
                        &a.cam,
                        &self.body,
                        &f,
                        &a.blobs,
                        a.pairs.clone(),
                        &self.params.fit,
                    ) {
                        Ok(fit) => {
                            f = fit.updated.filter;
                            self.applied.push_back(Applied {
                                pairs: fit.pairs,
                                ..a.clone()
                            });
                        }
                        Err(_) => self.stats.dropped_on_rebuild += 1,
                    }
                    self.history.push_back((t, f));
                    m += 1;
                }
                (_, Some(ts)) => {
                    f.propagate(
                        &imu[i].0.gyro,
                        &imu[i].0.accel,
                        (ts - t) as f64 * 1e-9,
                        &g,
                        &n,
                    );
                    self.zupt(&mut f, t, ts, imu[i].1);
                    t = ts;
                    self.history.push_back((t, f));
                    i += 1;
                }
                // a measurement can't be newer than the newest IMU sample (frames wait for it)
                (Some(_), None) | (None, None) => break,
            }
        }
    }

    fn original_acquisition_cost(&self, cam: &Camera, blobs: &[Blob], fit: &vision::Fit) -> f64 {
        let f = &fit.updated.filter;
        let chi: f64 = vision::residuals(cam, &self.body, blobs, &fit.pairs, &f.x)
            .iter()
            .map(|r| (r.r.norm() / self.params.alias_sigma_px).powi(2))
            .sum();
        let (visible, near, _) = vision::tests::hot_paths::original_association_report(
            cam,
            &self.body,
            f,
            blobs,
            self.params.fit.update.sigma_px,
            &self.params.fit.assoc,
            3.0,
        );
        chi + self.params.alias_miss_cost * (visible.saturating_sub(near) + fit.unexplained) as f64
    }

    fn original_search_kept(
        &self,
        cam: &Camera,
        blobs: &[Blob],
        kept: &Filter,
        carried: Option<&Filter>,
        fit_params: &FitParams,
    ) -> Option<(Filter, vision::Fit)> {
        let all: Vec<usize> = (0..blobs.len()).collect();
        let costed: Vec<(f64, (Filter, vision::Fit))> =
            search::search_position(cam, &self.body, &kept.x.q, blobs, &all, &self.params.search)
                .into_iter()
                .filter_map(|c| {
                    self.attempt(
                        cam,
                        blobs,
                        &c.world_from_model,
                        c.pairs,
                        carried,
                        fit_params,
                    )
                })
                .filter(|(_, fit)| self.explains_frame(cam, blobs, fit))
                .map(|h| (self.acquisition_cost(cam, blobs, &h.1), h))
                .collect();
        costed
            .into_iter()
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, h)| h)
    }
}

fn wrap<T: Clone>(q: &mut VecDeque<T>) {
    let values: Vec<_> = q.iter().cloned().collect();
    *q = VecDeque::with_capacity(values.len());
    q.extend(values.iter().cloned());
    for _ in 0..values.len() / 2 {
        let v = q.pop_front().unwrap();
        q.push_back(v);
    }
    for (slot, value) in q.iter_mut().zip(values) {
        *slot = value;
    }
}

#[test]
fn history_queries_match_original_at_boundaries_and_across_wraps() {
    let w = world(3);
    let mut tr = Tracker::new(1, w.body, Params::default(), 3);
    let f = tr.acquisition_prior(&Isometry3::identity(), None);
    for n in [0, 1, 2, 17, 200] {
        tr.imu.clear();
        tr.history.clear();
        for i in 0..n {
            let t = 1_000_000 + i * 2_000_000;
            tr.imu.push_back(ImuSample {
                t,
                gyro: Vector3::new(0.1, 0.2, 0.3),
                accel: Vector3::new(0.0, G, 0.0),
            });
            tr.history.push_back((t, f));
        }
        wrap(&mut tr.imu);
        wrap(&mut tr.history);
        for window in [0, 1, 100_000_000, u64::MAX] {
            tr.params.steady_window_ns = window;
            for t in [
                0,
                1_000_000,
                1_000_001,
                50_000_000,
                99_000_000,
                100_000_000,
                500_000_000,
                u64::MAX,
            ] {
                let bits = |r: Option<(usize, Filter)>| r.map(|(i, f)| (i, filter_bits(&f)));
                assert_eq!(bits(tr.state_at(t)), bits(tr.original_state_at(t)));
                let bits = |r: Option<Vector3<f64>>| r.map(|v| v.map(f64::to_bits));
                let (up, allocations) = counted(|| tr.steady_up(t, Vector3::zeros()));
                assert_eq!(bits(up), bits(tr.original_steady_up(t, Vector3::zeros())));
                assert_eq!(allocations, (0, 0, 0));
            }
        }
    }
}

#[test]
fn prior_check_matches_original_for_all_boundary_subsets() {
    let stamps = [
        0,
        1,
        CROSS_CHECK_NS - 1,
        CROSS_CHECK_NS,
        CROSS_CHECK_NS + 1,
        u64::MAX - CROSS_CHECK_NS,
        u64::MAX,
    ];
    for mask in 0..1 << stamps.len() {
        let mut tr = Tracker::new(1, world(1).body, Params::default(), 3);
        for (i, &t) in stamps.iter().enumerate().rev() {
            if mask & (1 << i) != 0 {
                tr.note_camera_update(i % 3, t);
                tr.note_camera_update(i % 3, t);
            }
        }
        for updates in &mut tr.camera_updates {
            wrap(updates);
        }
        for camera in 0..4 {
            for &t in &stamps {
                assert_eq!(
                    tr.others_set_prior(camera, t),
                    tr.original_others_set_prior(camera, t)
                );
            }
        }
    }
}

#[test]
fn streamed_rebuild_matches_original_with_optical_updates_and_empty_ranges() {
    let mut cases = 0;
    let mut savings = 0;
    let mut dropped = 0;
    for stationary in [false, true] {
        let mut w = world(3);
        if stationary {
            w.motion = Motion::still(Vector3::new(0.0, 1.0, 0.0), [0.0; 3]);
        }
        let (tr, _) = run_with(
            &w,
            2.0,
            0.2,
            &Run {
                shuffle: true,
                ..Default::default()
            },
        );
        assert!(!tr.applied.is_empty());
        for idx in [0, tr.applied.len() / 2, tr.applied.len() - 1] {
            let first = tr.applied[idx].clone();
            let (k, posterior) = tr.state_at(first.t).unwrap();
            for mode in 0..7 {
                let mut base = tr.clone();
                let mut first = first.clone();
                match mode {
                    1 => base.still.iter_mut().for_each(|x| *x = true),
                    2 => base.still.iter_mut().for_each(|x| *x = false),
                    3 => {
                        for a in &mut base.applied {
                            if a.t > first.t {
                                a.pairs.clear();
                            }
                        }
                    }
                    4 => {
                        base.imu.clear();
                        base.still.clear();
                    }
                    5 => first.t = base.imu.back().unwrap().t,
                    6 => {
                        let mut a = first.clone();
                        a.t = base.imu.back().unwrap().t + 1;
                        base.applied.push_back(a);
                    }
                    _ => {}
                }
                let mut a = base.clone();
                let mut b = base.clone();
                wrap(&mut a.imu);
                wrap(&mut a.still);
                wrap(&mut b.imu);
                wrap(&mut b.still);
                let fa = first.clone();
                let fb = first.clone();
                let (_, old) = counted(|| a.original_rebuild(k, fa, posterior));
                let (_, new) = counted(|| b.rebuild(k, fb, posterior));
                assert_state(&a, &b);
                assert!(new.0 <= old.0);
                assert!(new.2 <= old.2);
                savings += old.0 - new.0;
                dropped += usize::from(a.stats.dropped_on_rebuild > base.stats.dropped_on_rebuild);
                cases += 1;
            }
        }
    }
    assert_eq!(cases, 42);
    assert!(savings > 0);
    assert!(dropped > 0);
}

#[test]
fn acquisition_and_streamed_search_match_original() {
    let w = touch_world(3);
    let mut tr = Tracker::new(1, w.body.clone(), Params::default(), w.cams.len());
    let mut rng = Rng::new(99);
    let mut found = 0;
    let bits = |r: Option<(Filter, vision::Fit)>| {
        r.map(|(f, fit)| {
            (
                filter_bits(&f),
                filter_bits(&fit.updated.filter),
                fit.updated
                    .residuals_px
                    .iter()
                    .map(|x| x.to_bits())
                    .collect::<Vec<_>>(),
                fit.pairs,
                fit.unexplained,
            )
        })
    };
    for step in 0..12 {
        let t = 0.2 + step as f64 * 0.11;
        let kept = tr.acquisition_prior(&w.motion.pose(t), None);
        for cam in &w.cams {
            let blobs = sight(&w, cam, t, &mut rng, 0.2);
            for cap in [0, 1, 4] {
                tr.params.search.max_candidates = cap;
                let old = tr.original_search_kept(cam, &blobs, &kept, None, &tr.params.fit);
                let new = tr.search_kept(cam, &blobs, &kept, None, &tr.params.fit);
                if let Some((_, fit)) = &new {
                    assert_eq!(
                        tr.acquisition_cost(cam, &blobs, fit).to_bits(),
                        tr.original_acquisition_cost(cam, &blobs, fit).to_bits()
                    );
                    found += 1;
                }
                assert_eq!(bits(old), bits(new));
            }
        }
    }
    assert!(found > 0);
}

/// Offline, whole-function timing; no assertions about noisy wall-clock speedups.
/// Fixture preparation and tracker cloning are outside each timed interval.
#[test]
#[ignore = "offline CPU benchmark; run explicitly with --nocapture --test-threads=1"]
fn benchmark_tracking_hot_paths() {
    use std::{hint::black_box, time::Instant};
    let w = touch_world(3);
    let (base, _) = run_with(
        &w,
        2.0,
        0.2,
        &Run {
            shuffle: true,
            ..Default::default()
        },
    );
    let first = base.applied.front().unwrap().clone();
    let (k, posterior) = base.state_at(first.t).unwrap();
    let fit = vision::fit_pairs(
        &first.cam,
        &base.body,
        &posterior,
        &first.blobs,
        first.pairs.clone(),
        &base.params.fit,
    )
    .unwrap();
    assert_eq!(
        base.original_acquisition_cost(&first.cam, &first.blobs, &fit)
            .to_bits(),
        base.acquisition_cost(&first.cam, &first.blobs, &fit)
            .to_bits(),
    );
    let (_, old_allocs) =
        counted(|| base.original_acquisition_cost(&first.cam, &first.blobs, &fit));
    let (_, new_allocs) = counted(|| base.acquisition_cost(&first.cam, &first.blobs, &fit));
    eprintln!("acquisition alloc/realloc calls, frees, cumulative bytes: original={old_allocs:?}, new={new_allocs:?}");
    for rebuild in [false, true] {
        let mut times = [Vec::new(), Vec::new()];
        let iterations = if rebuild { 100 } else { 10_000 };
        for repeat in 0..12 {
            for variant in [repeat % 2, 1 - repeat % 2] {
                let mut work: Vec<_> = if rebuild {
                    (0..iterations)
                        .map(|_| (base.clone(), Some(first.clone())))
                        .collect()
                } else {
                    Vec::new()
                };
                let start = Instant::now();
                if rebuild {
                    for (tr, first) in &mut work {
                        let first = first.take().unwrap();
                        if variant == 0 {
                            black_box(tr).original_rebuild(k, first, posterior);
                        } else {
                            black_box(tr).rebuild(k, first, posterior);
                        }
                    }
                } else {
                    for _ in 0..iterations {
                        let tr = black_box(&base);
                        let cost = if variant == 0 {
                            tr.original_acquisition_cost(
                                black_box(&first.cam),
                                black_box(&first.blobs),
                                black_box(&fit),
                            )
                        } else {
                            tr.acquisition_cost(
                                black_box(&first.cam),
                                black_box(&first.blobs),
                                black_box(&fit),
                            )
                        };
                        black_box(cost);
                    }
                }
                times[variant].push(start.elapsed().as_secs_f64() * 1e6 / iterations as f64);
                black_box(&work);
            }
        }
        for (variant, times) in times.iter_mut().enumerate() {
            times.sort_by(f64::total_cmp);
            eprintln!("rebuild={rebuild}, variant={variant} (0 original, 1 new): us/call min/median/max {:.3}/{:.3}/{:.3}", times[0], times[6], times[11]);
        }
    }
}
