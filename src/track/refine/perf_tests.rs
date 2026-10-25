// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

use super::*;
use crate::calib::synth::{camera_model, Rng};
use crate::track::tracker::stillness_tests::counted;

// Frozen matching algorithm from 985f278; do not share the optimized selection with this oracle.
fn baseline_matches(
    model: &CameraModel,
    cam_from_world: &Isometry3<f64>,
    s: &Sighting,
    gate: f64,
    margin: f64,
) -> Vec<(Vector3<f64>, Vector2<f64>)> {
    let uv: Vec<Option<Vector2<f64>>> = s
        .leds
        .iter()
        .map(|w| model.project(&(cam_from_world * Point3::from(*w)).coords))
        .collect();
    let mut out = Vec::new();
    for (i, u) in uv.iter().enumerate() {
        let Some(u) = u else { continue };
        let mut d: Vec<(f64, usize)> = s
            .blobs
            .iter()
            .enumerate()
            .map(|(b, x)| ((x - u).norm(), b))
            .collect();
        d.sort_by(|a, b| a.0.total_cmp(&b.0));
        let Some(&(d1, b)) = d.first() else { continue };
        if d1 > gate || d.get(1).is_some_and(|x| x.0 < d1 + margin) {
            continue;
        }
        let rival = uv
            .iter()
            .enumerate()
            .any(|(j, v)| j != i && v.is_some_and(|v| (s.blobs[b] - v).norm() < d1 + margin));
        if !rival {
            out.push((s.leds[i], s.blobs[b]));
        }
    }
    out
}

fn bits(pairs: Vec<(Vector3<f64>, Vector2<f64>)>) -> Vec<[u64; 5]> {
    // Outputs are copies of the fixture, not floating-point arithmetic results: compare exactly,
    // even for a selected non-finite input.
    pairs
        .into_iter()
        .map(|(w, p)| [w.x, w.y, w.z, p.x, p.y].map(f64::to_bits))
        .collect()
}

fn fixture(n: usize, clutter: usize) -> Sighting {
    let model = camera_model();
    let leds: Vec<_> = (0..n)
        .map(|i| {
            Vector3::new(
                (i % 6) as f64 * 0.035 - 0.1,
                (i / 6) as f64 * 0.035 - 0.05,
                1.5,
            )
        })
        .collect();
    let mut blobs: Vec<_> = leds
        .iter()
        .map(|w| model.project(w).unwrap() + Vector2::new(0.2, -0.1))
        .collect();
    let mut rng = Rng::new(19);
    blobs.extend((0..clutter).map(|_| Vector2::new(rng.uniform() * 1280.0, rng.uniform() * 960.0)));
    Sighting { leds, blobs }
}

#[test]
fn nearest_two_matches_stable_sort() {
    let model = camera_model();
    let mut rng = Rng::new(21);
    for seed in 0..2000 {
        let mut s = fixture(seed % 25, seed % 65);
        for p in &mut s.blobs {
            *p += Vector2::new(rng.gauss(), rng.gauss()) * 2.0;
        }
        if seed % 3 == 0 {
            s.blobs.extend_from_within(..); // ties and ambiguous matches
        }
        if seed % 7 == 0 {
            s.leds.extend_from_within(..); // competing LEDs
        }
        if seed % 11 == 0 {
            s.blobs.reverse();
        }
        let cfw = Isometry3::from_parts(
            Translation3::new(rng.gauss() * 0.001, rng.gauss() * 0.001, 0.0),
            UnitQuaternion::from_euler_angles(0.0, rng.gauss() * 0.001, 0.0),
        );
        for (gate, margin) in [(4.0, 1.5), (2.5, 1.0), (1.5, 1.0), (1.0, 1.0), (10.0, 0.0)] {
            assert_eq!(
                bits(matches(&model, &cfw, &s, gate, margin)),
                bits(baseline_matches(&model, &cfw, &s, gate, margin)),
                "case {seed}"
            );
        }
    }
}

#[test]
fn nearest_two_preserves_boundaries_ties_and_nonfinite_inputs() {
    let model = camera_model();
    let cfw = Isometry3::identity();
    let center = Vector2::new(model.cx, model.cy);
    let mut s = Sighting {
        leds: vec![Vector3::new(0.0, 0.0, 1.0)],
        blobs: Vec::new(),
    };
    let values = [
        -f64::NAN,
        f64::NEG_INFINITY,
        -4.0,
        -1.0,
        -0.0,
        0.0,
        1.0,
        4.0,
        f64::INFINITY,
        f64::NAN,
    ];
    for a in values {
        for b in values {
            s.blobs = vec![center + Vector2::new(a, 0.0), center + Vector2::new(b, 0.0)];
            for gate in [-1.0, 0.0, 1.0, 4.0, f64::INFINITY, f64::NAN] {
                for margin in [-1.0, 0.0, 1.0, f64::INFINITY, f64::NAN] {
                    assert_eq!(
                        bits(matches(&model, &cfw, &s, gate, margin)),
                        bits(baseline_matches(&model, &cfw, &s, gate, margin))
                    );
                }
            }
        }
    }
    for z in [
        f64::NEG_INFINITY,
        -1.0,
        0.0,
        1e-6,
        1.000001e-6,
        f64::INFINITY,
        f64::NAN,
    ] {
        s.leds = vec![Vector3::new(0.0, 0.0, z)];
        for n in 0..=2 {
            s.blobs = vec![center; n];
            assert_eq!(
                bits(matches(&model, &cfw, &s, 4.0, 1.5)),
                bits(baseline_matches(&model, &cfw, &s, 4.0, 1.5))
            );
        }
    }
}

#[test]
fn median_selection_preserves_upper_median_bits() {
    assert!(median(Vec::new()).is_nan());
    let mut rng = Rng::new(42);
    for n in 1..256 {
        let mut v: Vec<_> = (0..n).map(|_| rng.gauss()).collect();
        v.extend([
            -f64::NAN,
            f64::NEG_INFINITY,
            -0.0,
            0.0,
            f64::INFINITY,
            f64::NAN,
            f64::from_bits(0x7ff8_0000_0000_0001),
        ]);
        for _ in 0..3 {
            let mut sorted = v.clone();
            sorted.sort_by(f64::total_cmp);
            assert_eq!(
                median(v.clone()).to_bits(),
                sorted[sorted.len() / 2].to_bits()
            );
            v.reverse();
            v.truncate(v.len() / 2);
        }
    }
}

#[test]
fn matching_removes_per_led_allocations() {
    let s = fixture(24, 64);
    let model = camera_model();
    let cfw = Isometry3::identity();
    let (old, oc) = counted(|| baseline_matches(&model, &cfw, &s, 4.0, 1.5));
    let (new, nc) = counted(|| matches(&model, &cfw, &s, 4.0, 1.5));
    assert_eq!(new.len(), 24);
    assert_eq!(new, old);
    assert!(oc.0 >= nc.0 + 24, "{oc:?} vs {nc:?}");
}

#[test]
#[ignore = "offline release benchmark; single threaded with --nocapture"]
fn benchmark_recalibration_matching() {
    use std::{hint::black_box, time::Instant};
    let model = camera_model();
    let cfw = Isometry3::identity();
    for clutter in [0, 64] {
        let s = fixture(24, clutter);
        let mut times = [Vec::new(), Vec::new()];
        for round in 0..12 {
            for which in [round % 2, 1 - round % 2] {
                let start = Instant::now();
                for _ in 0..5000 {
                    let f = if which == 0 {
                        baseline_matches
                    } else {
                        matches
                    };
                    black_box(f(
                        black_box(&model),
                        black_box(&cfw),
                        black_box(&s),
                        4.0,
                        1.5,
                    ));
                }
                times[which].push(start.elapsed().as_secs_f64() * 1e6 / 5000.0);
            }
        }
        for (name, t) in ["baseline", "nearest-two"].into_iter().zip(&mut times) {
            t.sort_by(f64::total_cmp);
            eprintln!(
                "24 LEDs + {clutter} clutter {name} us/call min/median/max: {:.3}/{:.3}/{:.3}",
                t[0],
                (t[5] + t[6]) / 2.0,
                t[11]
            );
        }
    }
}
