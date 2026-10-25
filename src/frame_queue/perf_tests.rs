// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

use super::*;
use crate::track::tracker::stillness_tests::counted;

#[test]
fn bounded_insertion_matches_sorted_reference() {
    let mut q = FrameQueue::default();
    q.register(["camera".into()]);
    let mut reference = Vec::new();
    let mut seed = 42u64;
    let mut dropped = 0;
    for value in 0..20_000 {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        let time = (seed >> 32) % 100;
        q.push_frame("camera", time, value);
        if let Some(i) = reference.iter().position(|&(t, _)| t == time) {
            reference.remove(i);
            dropped += 1;
        }
        reference.push((time, value));
        reference.sort_by_key(|p| p.0);
        if reference.len() > FRAMES_PER_SENSOR {
            reference.remove(0);
            dropped += 1;
        }
        assert_eq!(q.stats().dropped_frames, dropped);
        assert_eq!(
            q.frames["camera"].iter().copied().collect::<Vec<_>>(),
            reference
        );
        if value % 7 == 0 {
            assert_eq!(q.pop_frame(), Some(reference.remove(0).1));
        }
        if value % 97 == 0 {
            q.clear();
            reference.clear();
        }
    }
}

#[test]
fn selection_visits_once_in_order_and_retains_storage() {
    for mask in 0..4 {
        let mut q = FrameQueue::default();
        q.register(["a".into(), "b".into()]);
        // Move the deque head so removal is also tested across its wrap.
        for time in 0..3 {
            q.push_frame("a", time, time);
            q.frames.get_mut("a").unwrap().pop_front();
        }
        q.push_frame("a", 20, 1);
        q.push_frame("a", 10, 0);
        q.push_frame("b", 15, 2);
        let capacity = q.frames["a"].capacity();
        let mut visited = Vec::new();
        let taken = q.take_sensor_frames("a", |&value| {
            visited.push(value);
            mask & (1 << value) != 0
        });
        assert_eq!(visited, [0, 1]);
        assert_eq!(
            taken,
            (0..2).filter(|i| mask & (1 << i) != 0).collect::<Vec<_>>()
        );
        assert_eq!(q.frames["a"].capacity(), capacity);
        assert_eq!(q.stats().dropped_frames, 0);
        let kept: Vec<_> = (0..2).filter(|i| mask & (1 << i) == 0).collect();
        let mut expected = kept.clone();
        expected.insert(usize::from(!kept.is_empty()), 2);
        let actual: Vec<_> = std::iter::from_fn(|| q.pop_frame()).collect();
        assert_eq!(actual, expected);
    }
}

#[test]
fn registered_frames_and_empty_selections_do_not_allocate() {
    let mut q = FrameQueue::default();
    q.register(["camera".into()]);
    let (_, counts) = counted(|| {
        for time in 0..1000 {
            q.push_frame("camera", time, time);
            q.push_frame("camera", time, time); // replacement
            q.push_frame("camera", 0, 0); // old exposure when full
            assert!(q.take_sensor_frames("camera", |_| false).is_empty());
            if time % 5 == 0 {
                q.pop_frame();
            }
        }
    });
    assert_eq!(counts, (0, 0, 0));
    assert!(!q.take_sensor_frames("camera", |_| true).is_empty());
    // Taking every frame must not discard the ingress queue's allocation.
    let (_, counts) = counted(|| q.push_frame("camera", 2000, 2000));
    assert_eq!(counts, (0, 0, 0));
}

#[test]
fn reusable_clock_batch_clears_old_samples_and_handles_wrap_and_overflow() {
    let mut q = FrameQueue::<()>::default();
    let mut batch = Vec::new();
    for t in 0..CLOCK_SAMPLES as u64 {
        q.push_clock(t, t as u32);
    }
    q.take_clocks_into(&mut batch);
    let (_, counts) = counted(|| {
        for n in [0, 1, 31, 32, 33, 64, 5, 0] {
            for t in 0..n {
                q.push_clock(t, t as u32);
            }
            q.take_clocks_into(&mut batch);
            assert_eq!(batch.len(), n.min(CLOCK_SAMPLES as u64) as usize);
            for (i, &sample) in batch.iter().enumerate() {
                let t = n.saturating_sub(CLOCK_SAMPLES as u64) + i as u64;
                assert_eq!(sample, (t, t as u32));
            }
            assert!(q.is_empty());
        }
    });
    assert_eq!(counts, (0, 0, 0));
    assert_eq!(q.stats().dropped_clocks, 33);
}

#[test]
#[ignore = "offline release benchmark; single threaded with --nocapture"]
fn benchmark_ingress_clock_batch() {
    use std::{hint::black_box, time::Instant};
    for n in [1, 8, 32] {
        let mut q = FrameQueue::<()>::default();
        let mut batch = Vec::new();
        for t in 0..32 {
            q.push_clock(t, t as u32);
        }
        q.take_clocks_into(&mut batch);
        let mut times = [Vec::new(), Vec::new()];
        for round in 0..12 {
            for which in [round % 2, 1 - round % 2] {
                let start = Instant::now();
                for _ in 0..100_000 {
                    for t in 0..n {
                        q.push_clock(black_box(t), t as u32);
                    }
                    if which == 0 {
                        // Baseline 985f278: a new output allocation for each nonempty batch.
                        black_box(q.clocks.drain(..).collect::<Vec<_>>());
                    } else {
                        q.take_clocks_into(&mut batch);
                        black_box(&batch);
                    }
                }
                times[which].push(start.elapsed().as_secs_f64() * 1e9 / 100_000.0);
            }
        }
        for (name, t) in ["baseline", "reused"].into_iter().zip(&mut times) {
            t.sort_by(f64::total_cmp);
            eprintln!(
                "{n} clocks {name} ns/batch min/median/max: {:.1}/{:.1}/{:.1}",
                t[0],
                (t[5] + t[6]) / 2.0,
                t[11]
            );
        }
    }
}
