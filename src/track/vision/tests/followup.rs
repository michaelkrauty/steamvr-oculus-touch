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

#[test]
#[ignore = "offline release benchmark"]
fn benchmark_followup_shifted() {
    use std::{hint::black_box, time::Instant};
    let (cam, b, x) = (camera(), body(), state());
    let f = filter(x, 0.01, 0.03);
    let list: Vec<_> = blobs(&cam, &b, &x).into_iter().map(|(_, b)| b).collect();
    let p = AssocParams::default();
    let mut times = [Vec::new(), Vec::new()];
    for round in 0..12 {
        for which in [round % 2, 1 - round % 2] {
            let start = Instant::now();
            for _ in 0..2000 {
                let fun = if which == 0 {
                    baseline_shifted
                } else {
                    associate_shifted
                };
                black_box(fun(black_box(&cam), &b, &f, &list, 0.5, &p, 1.5, 4, 2));
            }
            times[which].push(start.elapsed().as_secs_f64() * 1e6 / 2000.0);
        }
    }
    for (name, t) in ["baseline", "scratch + selection"]
        .into_iter()
        .zip(&mut times)
    {
        t.sort_by(f64::total_cmp);
        eprintln!(
            "{name} shifted us/call min/median/max: {:.3}/{:.3}/{:.3}",
            t[0], t[6], t[11]
        );
    }
}

#[test]
fn diagnostic_residuals_match_full_projection_bits() {
    use crate::track::tracker::stillness_tests::counted;
    let (cam, b, mut x) = (camera(), body(), state());
    let list: Vec<_> = (0..b.leds.len())
        .map(|i| Blob {
            xy: Vector2::new(i as f64 * 40.0, 300.0),
            led_id: -1,
        })
        .collect();
    let pairs: Vec<_> = (0..b.leds.len()).rev().map(|i| (i, i)).collect();
    for z in [0.0, 1.0, 2.0, 4.0, f64::NAN] {
        x.p.z = z;
        let (actual, nc) = counted(|| pixel_residuals(&cam, &b, &list, &pairs, &x));
        let (full, oc) = counted(|| residuals(&cam, &b, &list, &pairs, &x));
        let expected: Vec<_> = pairs
            .iter()
            .filter_map(|&(led, blob)| {
                original_project(&cam, &b, &x, led).map(|p| list[blob].xy - p.uv)
            })
            .collect();
        let bits = |v: Vec<Vector2<f64>>| {
            v.into_iter()
                .map(|v| [v.x.to_bits(), v.y.to_bits()])
                .collect::<Vec<_>>()
        };
        assert_eq!(bits(actual), bits(expected));
        if !full.is_empty() {
            assert!(nc.2 < oc.2, "smaller diagnostic storage: {nc:?} vs {oc:?}");
        }
    }
}

fn baseline_gate_pairs(
    cam: &Camera,
    body: &Body,
    f: &Filter,
    blobs: &[Blob],
    pairs: &[(usize, usize)],
    sigma_px: f64,
    p: &AssocParams,
) -> Vec<(usize, usize)> {
    let projector = Projector::new(cam, body, &f.x);
    let p6 = f.cov.fixed_view::<6, 6>(0, 0).into_owned();
    let var = Matrix2::identity() * (sigma_px * sigma_px);
    pairs
        .iter()
        .copied()
        .filter(|&(led, b)| {
            let Some(pr) = projector.project(led) else {
                return false;
            };
            let Some(s_inv) = (pr.j * p6 * pr.j.transpose() + var).try_inverse() else {
                return false;
            };
            let d = blobs[b].xy - pr.uv;
            pr.facing > p.min_facing
                && d.norm() <= p.max_px
                && (d.transpose() * s_inv * d)[0] < p.gate
        })
        .collect()
}

#[test]
fn cheap_pair_gates_preserve_matches() {
    let (cam, b, x) = (camera(), body(), state());
    let list: Vec<_> = (0..b.leds.len())
        .map(|i| Blob {
            xy: original_project(&cam, &b, &x, i).unwrap().uv,
            led_id: -1,
        })
        .collect();
    let pairs: Vec<_> = (0..b.leds.len()).map(|i| (i, i)).collect();
    for facing in [-1.0, 0.2, 1.0, f64::NAN] {
        for cap in [-1.0, 0.0, 1.0, f64::INFINITY, f64::NAN] {
            for covariance in [0.0, 1e-4, f64::NAN] {
                let mut f = filter(x, 0.01, 0.03);
                f.cov.fill(covariance);
                let p = AssocParams {
                    min_facing: facing,
                    max_px: cap,
                    ..Default::default()
                };
                for sigma in [0.0, 0.5, f64::NAN] {
                    assert_eq!(
                        gate_pairs(&cam, &b, &f, &list, &pairs, sigma, &p),
                        baseline_gate_pairs(&cam, &b, &f, &list, &pairs, sigma, &p)
                    );
                }
            }
        }
    }
}

fn baseline_residuals(
    cam: &Camera,
    body: &Body,
    blobs: &[Blob],
    pairs: &[(usize, usize)],
    x: &Nominal,
) -> Vec<PixelResidual> {
    let projector = Projector::new(cam, body, x);
    pairs
        .iter()
        .filter_map(|&(led, b)| {
            projector.project(led).map(|pr| PixelResidual {
                r: blobs[b].xy - pr.uv,
                j: pr.j,
            })
        })
        .collect()
}

#[test]
fn residual_capacity_removes_growth_without_changing_bits() {
    use crate::track::tracker::stillness_tests::counted;
    let (cam, b, mut x) = (camera(), body(), state());
    let list: Vec<_> = (0..b.leds.len())
        .map(|i| Blob {
            xy: Vector2::new(i as f64, 300.0),
            led_id: -1,
        })
        .collect();
    let pairs: Vec<_> = (0..b.leds.len()).map(|i| (i, i)).collect();
    let bits = |v: Vec<PixelResidual>| {
        v.into_iter()
            .flat_map(|p| {
                p.r.iter()
                    .chain(p.j.iter())
                    .map(|v| v.to_bits())
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>()
    };
    for z in [0.0, 1.0, 2.0, 100.0, f64::NAN] {
        x.p.z = z;
        for n in 0..=pairs.len() {
            let (new, nc) = counted(|| residuals(&cam, &b, &list, &pairs[..n], &x));
            let (old, oc) = counted(|| baseline_residuals(&cam, &b, &list, &pairs[..n], &x));
            assert_eq!(nc.0, usize::from(!new.is_empty()));
            if old.len() > 4 {
                assert!(nc.0 < oc.0, "{nc:?} vs {oc:?}");
            }
            assert_eq!(bits(new), bits(old));
        }
    }
}

#[test]
fn shifted_search_with_no_hypotheses_does_not_allocate_scratch() {
    use crate::track::tracker::stillness_tests::counted;
    let (cam, b, x) = (camera(), body(), state());
    let f = filter(x, 0.01, 0.03);
    let list: Vec<_> = blobs(&cam, &b, &x).into_iter().map(|(_, b)| b).collect();
    let p = AssocParams {
        max_px: -1.0,
        ..Default::default()
    };
    let (new, nc) = counted(|| associate_shifted(&cam, &b, &f, &list, 0.5, &p, 1.5, 0, 0));
    let (old, oc) = counted(|| baseline_shifted(&cam, &b, &f, &list, 0.5, &p, 1.5, 0, 0));
    assert_eq!(new, old);
    assert!(new.is_empty());
    assert_eq!(nc, oc);
}

#[test]
#[ignore = "offline release benchmark"]
fn benchmark_followup_residuals_and_gates() {
    use std::{hint::black_box, time::Instant};
    let (cam, b, x) = (camera(), body(), state());
    let f = filter(x, 0.01, 0.03);
    let list: Vec<_> = (0..b.leds.len())
        .map(|i| Blob {
            xy: original_project(&cam, &b, &x, i).unwrap().uv,
            led_id: -1,
        })
        .collect();
    let pairs: Vec<_> = (0..b.leds.len()).map(|i| (i, i)).collect();
    for scenario in 0..4 {
        let p = AssocParams {
            min_facing: if scenario == 3 { 2.0 } else { -2.0 },
            ..Default::default()
        };
        let mut times = [Vec::new(), Vec::new()];
        for round in 0..12 {
            for which in [round % 2, 1 - round % 2] {
                let start = Instant::now();
                for _ in 0..5000 {
                    let cam = black_box(&cam);
                    match (scenario, which) {
                        (0, 0) | (1, 0) => {
                            black_box(baseline_residuals(cam, &b, &list, &pairs, &x));
                        }
                        (0, _) => {
                            black_box(residuals(cam, &b, &list, &pairs, &x));
                        }
                        (1, _) => {
                            black_box(pixel_residuals(cam, &b, &list, &pairs, &x));
                        }
                        (_, 0) => {
                            black_box(baseline_gate_pairs(cam, &b, &f, &list, &pairs, 0.5, &p));
                        }
                        (_, _) => {
                            black_box(gate_pairs(cam, &b, &f, &list, &pairs, 0.5, &p));
                        }
                    }
                }
                times[which].push(start.elapsed().as_secs_f64() * 1e6 / 5000.0);
            }
        }
        for (name, t) in ["baseline", "optimized"].into_iter().zip(&mut times) {
            t.sort_by(f64::total_cmp);
            eprintln!(
                "{} {name} us/call min/median/max: {:.3}/{:.3}/{:.3}",
                [
                    "residual capacity",
                    "diagnostic residuals",
                    "accepted gates",
                    "rejected gates"
                ][scenario],
                t[0],
                t[6],
                t[11]
            );
        }
    }
}

// A complete pixel-only caller: LLVM can eliminate unused Jacobian fields in the
// baseline too. Do not infer its speedup from a benchmark consuming full Jacobians.
fn baseline_predicted_pixels(
    cam: &Camera,
    body: &Body,
    x: &Nominal,
    min_facing: f64,
) -> Vec<Vector2<f64>> {
    (0..body.leds.len())
        .filter_map(|i| original_project(cam, body, x, i))
        .filter(|pr| pr.facing > min_facing)
        .map(|pr| pr.uv)
        .collect()
}

#[test]
#[ignore = "offline release benchmark"]
fn benchmark_followup_predicted_pixels() {
    use std::{hint::black_box, time::Instant};
    let (cam, b, x) = (camera(), body(), state());
    assert_eq!(
        predicted_pixels(&cam, &b, &x, 0.2),
        baseline_predicted_pixels(&cam, &b, &x, 0.2)
    );
    let mut times = [Vec::new(), Vec::new()];
    for round in 0..12 {
        for which in [round % 2, 1 - round % 2] {
            let start = Instant::now();
            for _ in 0..10000 {
                let f = if which == 0 {
                    baseline_predicted_pixels
                } else {
                    predicted_pixels
                };
                black_box(f(black_box(&cam), &b, &x, 0.2));
            }
            times[which].push(start.elapsed().as_secs_f64() * 1e6 / 10000.0);
        }
    }
    for (name, t) in ["baseline", "pixel-only"].into_iter().zip(&mut times) {
        t.sort_by(f64::total_cmp);
        eprintln!(
            "{name} predicted_pixels us/call min/median/max: {:.3}/{:.3}/{:.3}",
            t[0], t[6], t[11]
        );
    }
}
