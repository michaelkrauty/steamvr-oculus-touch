// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Independent pre-optimization oracle and opt-in full-association benchmark.
use super::*;
use std::hint::black_box;
use std::time::Instant;

// Keep the original collecting/sorting implementation independent of production.
#[allow(clippy::too_many_arguments)]
fn baseline(
    cam: &Camera,
    body: &Body,
    f: &Filter,
    blobs: &[Blob],
    sigma_px: f64,
    p: &AssocParams,
    snap_px: f64,
    min_leds: usize,
    margin: usize,
) -> Vec<(usize, usize)> {
    let (w, h) = (cam.model.width as f64, cam.model.height as f64);
    let visible: Vec<Projection> = (0..body.leds.len())
        .filter_map(|i| project(cam, body, &f.x, i))
        .filter(|pr| {
            pr.facing > p.min_facing
                && pr.uv.x >= -50.0
                && pr.uv.y >= -50.0
                && pr.uv.x < w + 50.0
                && pr.uv.y < h + 50.0
        })
        .collect();
    if visible.len() < min_leds || blobs.len() < min_leds {
        return Vec::new();
    }
    let p6 = f.cov.fixed_view::<6, 6>(0, 0).into_owned();
    // the pattern moves as a whole: the largest per-LED spread bounds the search
    let reach = visible
        .iter()
        .map(|pr| (pr.j * p6 * pr.j.transpose()).trace().sqrt())
        .fold(0.0, f64::max)
        * 3.0
        + 3.0 * sigma_px;
    let place = |d: &Vector2<f64>| -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        for pr in &visible {
            let at = pr.uv + d;
            let mut near: Vec<(f64, usize)> = blobs
                .iter()
                .enumerate()
                .map(|(b, x)| ((x.xy - at).norm(), b))
                .filter(|x| x.0 < 2.0 * snap_px)
                .collect();
            near.sort_by(|a, b| a.0.total_cmp(&b.0));
            if near.first().is_some_and(|x| x.0 < snap_px)
                && near.get(1).is_none_or(|x| x.0 >= snap_px)
                && !out.iter().any(|o: &(usize, usize)| o.1 == near[0].1)
            {
                out.push((pr.led, near[0].1));
            }
        }
        out
    };
    let mut scored: Vec<(usize, Vector2<f64>)> = Vec::new();
    for pr in &visible {
        for b in blobs {
            let d = b.xy - pr.uv;
            if d.norm() <= reach.min(p.max_px * 3.0) {
                scored.push((place(&d).len(), d));
            }
        }
    }
    scored.sort_by_key(|a| std::cmp::Reverse(a.0));
    let Some(&(best, d)) = scored.first() else {
        return Vec::new();
    };
    let rival = scored
        .iter()
        .find(|s| (s.1 - d).norm() > 2.0 * snap_px)
        .map_or(0, |s| s.0);
    if best < min_leds || best < rival + margin {
        return Vec::new();
    }
    let mut pairs = place(&d);
    pairs.sort();
    pairs
}

fn fixture() -> (Camera, Body, Filter, Vec<Blob>) {
    let (cam, body, truth) = (camera(), body(), state());
    let list = blobs(&cam, &body, &truth).iter().map(|s| s.1).collect();
    let prior = filter(
        Nominal {
            p: truth.p + cam.world_from_cam.rotation * Vector3::new(0.02, -0.015, 0.0),
            ..truth
        },
        0.01,
        0.03,
    );
    (cam, body, prior, list)
}

#[test]
fn unique_scan_matches_sorted_shift_search() {
    let (cam, body, prior, base) = fixture();
    let p = AssocParams::default();
    let check = |list: &[Blob], snap| {
        assert_eq!(
            associate_shifted(&cam, &body, &prior, list, 0.5, &p, snap, 4, 2),
            baseline(&cam, &body, &prior, list, 0.5, &p, snap, 4, 2),
            "snap={snap}, blobs={list:?}"
        );
    };
    check(&base, 1.5);
    assert!(associate_shifted(&cam, &body, &prior, &base, 0.5, &p, 1.5, 4, 2).len() >= 5);
    let mut seed = 42u64;
    let mut random = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / ((1u64 << 53) as f64)
    };
    for i in 0..2000 {
        let mut list = base.clone();
        for b in &mut list {
            b.xy += Vector2::new((random() - 0.5) * 4.0, (random() - 0.5) * 4.0);
        }
        list.truncate(i % (base.len() + 1));
        if i % 3 == 0 {
            list.extend(list.clone());
        }
        for _ in 0..i % 12 {
            list.push(Blob {
                xy: Vector2::new(random() * 640.0, random() * 480.0),
                led_id: -1,
            });
        }
        check(
            &list,
            [0.0, -1.0, 0.1, 1.5, 10.0, f64::NAN, f64::INFINITY][i % 7],
        );
    }
    // Exact duplicates are ambiguous, and input order must not select a winner.
    let mut duplicates = base.clone();
    duplicates.extend(base.clone());
    check(&duplicates, 1.5);
    assert!(associate_shifted(&cam, &body, &prior, &duplicates, 0.5, &p, 1.5, 4, 2).is_empty());
    duplicates.reverse();
    check(&duplicates, 1.5);
    // Distances at and immediately around the strict gate, plus non-finite blobs.
    for offset in [
        f64::from_bits(1.5f64.to_bits() - 1),
        1.5,
        f64::from_bits(1.5f64.to_bits() + 1),
        3.0,
        f64::INFINITY,
        f64::NAN,
    ] {
        let mut list = base.clone();
        for b in &base {
            list.push(Blob {
                xy: b.xy + Vector2::new(offset, 0.0),
                led_id: -1,
            });
        }
        check(&list, 1.5);
    }
}

#[test]
#[ignore = "opt-in CPU benchmark; synthetic recovery workload, not live latency"]
fn bench_shift_association() {
    let (cam, body, prior, blobs) = fixture();
    let p = AssocParams::default();
    let call = |fast| {
        let f = if fast { associate_shifted } else { baseline };
        f(
            black_box(&cam),
            black_box(&body),
            black_box(&prior),
            black_box(&blobs),
            0.5,
            &p,
            1.5,
            4,
            2,
        )
    };
    assert_eq!(call(false), call(true));
    for _ in 0..1000 {
        black_box(call(false));
        black_box(call(true));
    }
    println!("{} blobs, {} matched pairs", blobs.len(), call(true).len());
    for round in 0..7 {
        for fast in if round % 2 == 0 {
            [false, true]
        } else {
            [true, false]
        } {
            let start = Instant::now();
            for _ in 0..5000 {
                black_box(call(fast));
            }
            println!(
                "round={round} fast={fast} ns/call={}",
                start.elapsed().as_nanos() / 5000
            );
        }
    }
}
