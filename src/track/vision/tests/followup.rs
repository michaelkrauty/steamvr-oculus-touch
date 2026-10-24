// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! Frozen baseline projection and focused follow-up performance regressions.
use super::*;

fn original_project(cam: &Camera, body: &Body, x: &Nominal, i: usize) -> Option<Projection> {
    let led = &body.leds[i];
    let arm = led.pos - body.imu_pos;
    let r = x.q.to_rotation_matrix().into_inner();
    let rcw = cam
        .world_from_cam
        .rotation
        .inverse()
        .to_rotation_matrix()
        .into_inner();
    let pc = rcw * (x.p + r * arm - cam.world_from_cam.translation.vector);
    let (uv, duv) = project_with_jacobian(&cam.model, &pc)?;
    let mut dpc = SMatrix::<f64, 3, 6>::zeros();
    dpc.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(rcw * (-r * skew(&arm))));
    dpc.fixed_view_mut::<3, 3>(0, 3).copy_from(&rcw);
    let facing = (rcw * (r * led.dir)).dot(&(-pc)) / pc.norm();
    Some(Projection {
        led: i,
        uv,
        j: duv * dpc,
        facing,
    })
}

fn projection_bits(p: Option<Projection>) -> Option<Vec<u64>> {
    p.map(|p| {
        std::iter::once(p.led as u64)
            .chain(
                p.uv.iter()
                    .chain(p.j.iter())
                    .chain(std::iter::once(&p.facing))
                    .map(|v| v.to_bits()),
            )
            .collect()
    })
}

#[test]
fn batched_projection_matches_baseline_bits() {
    let mut cam = camera();
    let b = body();
    for seed in 0..128 {
        let mut x = state();
        x.q = UnitQuaternion::from_euler_angles(
            seed as f64 * 0.13,
            seed as f64 * 0.23,
            seed as f64 * 0.31,
        );
        x.p.z += (seed as f64 - 64.0) * 0.1;
        cam.world_from_cam.rotation =
            UnitQuaternion::from_euler_angles(0.1, seed as f64 * 0.07, -0.2);
        let projector = Projector::new(&cam, &b, &x);
        for i in 0..b.leds.len() {
            assert_eq!(
                projection_bits(projector.project(i)),
                projection_bits(original_project(&cam, &b, &x, i))
            );
        }
    }
}

#[test]
#[ignore = "offline timing; run release, single threaded, with --nocapture"]
fn benchmark_followup_projection() {
    use std::{hint::black_box, time::Instant};
    let (cam, b, x) = (camera(), body(), state());
    let mut times = [Vec::new(), Vec::new(), Vec::new()];
    for round in 0..12 {
        for which in [round % 3, (round + 1) % 3, (round + 2) % 3] {
            let start = Instant::now();
            for _ in 0..5000 {
                let (cam, b, x) = (black_box(&cam), black_box(&b), black_box(&x));
                if which == 0 {
                    for i in 0..b.leds.len() {
                        black_box(original_project(cam, b, x, i));
                    }
                } else {
                    let projector = Projector::new(cam, b, x);
                    for i in 0..b.leds.len() {
                        if which == 1 {
                            black_box(projector.project(i));
                        } else {
                            black_box(projector.pixel(i));
                        }
                    }
                }
            }
            times[which].push(start.elapsed().as_secs_f64() * 1e6 / 5000.0);
        }
    }
    for (name, t) in ["baseline", "batched", "pixel-only"]
        .into_iter()
        .zip(&mut times)
    {
        t.sort_by(f64::total_cmp);
        eprintln!(
            "{name} projection us/batch min/median/max: {:.3}/{:.3}/{:.3}",
            t[0], t[6], t[11]
        );
    }
}

#[test]
fn pixel_only_matches_baseline_including_camera_plane_and_nonfinite() {
    let mut cam = camera();
    cam.world_from_cam = Isometry3::identity();
    let mut b = body();
    b.imu_pos = Vector3::zeros();
    b.leds.truncate(1);
    b.leds[0].pos = Vector3::zeros();
    let mut x = state();
    x.q = UnitQuaternion::identity();
    for z in [
        -1.0,
        0.0,
        1e-6,
        1.0001e-6,
        1.001e-6,
        2e-6,
        1.0,
        1e100,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NAN,
    ] {
        for a in [0.0, 0.1, 1.0, 1e100, f64::INFINITY, f64::NAN] {
            x.p = Vector3::new(a, -a, z);
            let actual = Projector::new(&cam, &b, &x).pixel(0);
            let expected = original_project(&cam, &b, &x, 0).map(|p| (p.uv, p.facing));
            let bits = |p: Option<(Vector2<f64>, f64)>| {
                p.map(|(uv, f)| [uv.x.to_bits(), uv.y.to_bits(), f.to_bits()])
            };
            assert_eq!(bits(actual), bits(expected), "a={a}, z={z}");
        }
    }
    let b = body();
    for seed in 0..128 {
        x.p = Vector3::new(0.1, -0.2, seed as f64 * 0.03);
        x.q = UnitQuaternion::from_euler_angles(seed as f64 * 0.1, 0.5, -0.2);
        for i in 0..b.leds.len() {
            let p = Projector::new(&cam, &b, &x).pixel(i);
            let q = original_project(&cam, &b, &x, i);
            assert_eq!(
                p.map(|(uv, f)| [uv.x.to_bits(), uv.y.to_bits(), f.to_bits()]),
                q.map(|p| [p.uv.x.to_bits(), p.uv.y.to_bits(), p.facing.to_bits()])
            );
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn baseline_shifted(
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
    let projector = Projector::new(cam, body, &f.x);
    let (w, h) = (cam.model.width as f64, cam.model.height as f64);
    let visible: Vec<Projection> = (0..body.leds.len())
        .filter_map(|i| projector.project(i))
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
            // Only a unique blob strictly inside snap_px is usable. A second
            // match proves ambiguity; sorting a wider neighbourhood adds nothing.
            let mut near = blobs
                .iter()
                .enumerate()
                .filter(|(_, x)| (x.xy - at).norm() < snap_px);
            if let Some((b, _)) = near.next() {
                if near.next().is_none() && !out.iter().any(|o: &(usize, usize)| o.1 == b) {
                    out.push((pr.led, b));
                }
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

#[test]
fn shifted_scratch_matches_baseline_and_reduces_allocations() {
    use crate::track::tracker::stillness_tests::counted;
    let (cam, b, x) = (camera(), body(), state());
    let f = filter(x, 0.01, 0.03);
    let base: Vec<_> = blobs(&cam, &b, &x).into_iter().map(|(_, b)| b).collect();
    let p = AssocParams::default();
    let (new, nc) = counted(|| associate_shifted(&cam, &b, &f, &base, 0.5, &p, 1.5, 4, 2));
    let (old, oc) = counted(|| baseline_shifted(&cam, &b, &f, &base, 0.5, &p, 1.5, 4, 2));
    assert!(!new.is_empty());
    assert_eq!(new, old);
    assert!(nc.0 < oc.0, "{nc:?} vs {oc:?}");
    for n in 0..=base.len() {
        let mut list = base[..n].to_vec();
        if n % 2 == 0 {
            list.extend_from_within(..);
        }
        for snap in [-1.0, 0.0, 0.5, 1.5, 20.0, f64::INFINITY, f64::NAN] {
            for margin in [0, 1, 2, 100] {
                assert_eq!(
                    associate_shifted(&cam, &b, &f, &list, 0.5, &p, snap, 4, margin),
                    baseline_shifted(&cam, &b, &f, &list, 0.5, &p, snap, 4, margin)
                );
            }
        }
    }
}
