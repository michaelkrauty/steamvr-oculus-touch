// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
use super::*;
use crate::track::tracker::stillness_tests::counted;
fn baseline_score(
    cam: &Camera,
    offsets: &[Vector3<f64>],
    normals: &[Vector3<f64>],
    t: &Vector3<f64>,
    blobs: &[Blob],
    rays: &[(usize, Vector3<f64>)],
    p: &SearchParams,
) -> Option<Scored> {
    let centre = cam.world_from_cam.translation.vector;
    let cam_from_world = cam.world_from_cam.rotation.inverse();
    let mut near: Vec<(f64, usize, usize)> = Vec::new();
    let mut shown = 0;
    for (k, (o, n)) in offsets.iter().zip(normals).enumerate() {
        let w = t + o;
        let to_cam = centre - w;
        if n.dot(&to_cam) <= p.min_facing * to_cam.norm() {
            continue;
        }
        let Some(uv) = cam.model.project(&(cam_from_world * -to_cam)) else {
            continue;
        };
        if uv.x < 0.0
            || uv.y < 0.0
            || uv.x >= cam.model.width as f64
            || uv.y >= cam.model.height as f64
        {
            continue;
        }
        shown += 1;
        let nearest = rays
            .iter()
            .map(|&(b, _)| ((blobs[b].xy - uv).norm_squared(), b))
            .min_by(|x, y| x.0.total_cmp(&y.0));
        if let Some((d2, b)) = nearest.filter(|(d2, _)| *d2 <= p.inlier_px * p.inlier_px) {
            near.push((d2, k, b));
        }
    }
    if near.len() < p.min_leds {
        return None;
    }
    near.sort_by(|x, y| x.0.total_cmp(&y.0));
    let (mut leds, mut used) = (HashSet::new(), HashSet::new());
    let mut pairs = Vec::new();
    let mut sq_px = 0.0;
    for (d2, k, b) in near {
        if leds.insert(k) && used.insert(b) {
            pairs.push((k, b));
            sq_px += d2;
        }
    }
    (pairs.len() >= p.min_leds && pairs.len() as f64 >= p.min_share * shown as f64).then_some(
        Scored {
            pairs,
            shown,
            sq_px,
        },
    )
}

#[allow(clippy::type_complexity)]
fn fixture() -> (
    Camera,
    Vec<Vector3<f64>>,
    Vec<Vector3<f64>>,
    Vector3<f64>,
    Vec<Blob>,
    Vec<(usize, Vector3<f64>)>,
) {
    let cam = Camera {
        model: crate::calib::synth::camera_model(),
        world_from_cam: Isometry3::identity(),
    };
    let offsets: Vec<_> = (0..24)
        .map(|i| Vector3::new((i % 6) as f64 * 0.02, (i / 6) as f64 * 0.02, 0.0))
        .collect();
    let normals = vec![Vector3::new(0.0, 0.0, -1.0); offsets.len()];
    let t = Vector3::new(-0.06, -0.04, 1.0);
    let blobs: Vec<_> = offsets
        .iter()
        .map(|o| Blob {
            xy: cam.model.project(&(t + o)).unwrap(),
            led_id: -1,
        })
        .collect();
    let rays = blobs
        .iter()
        .enumerate()
        .map(|(i, b)| (i, cam.model.unproject(&b.xy).unwrap()))
        .collect();
    (cam, offsets, normals, t, blobs, rays)
}

#[test]
fn scoring_without_hash_sets_matches_baseline_and_allocates_less() {
    let (cam, offsets, normals, t, mut blobs, rays) = fixture();
    let p = SearchParams::default();
    let (new, nc) = counted(|| score(&cam, &offsets, &normals, &t, &blobs, &rays, &p));
    let (old, oc) = counted(|| baseline_score(&cam, &offsets, &normals, &t, &blobs, &rays, &p));
    assert!(new.is_some());
    assert!(nc.0 < oc.0, "{nc:?} vs {oc:?}");
    let bits = |s: Option<Scored>| s.map(|s| (s.pairs, s.shown, s.sq_px.to_bits()));
    assert_eq!(bits(new), bits(old));
    for n in 0..=rays.len() {
        for radius in [0.0, 1.0, 4.0, 30.0, f64::NAN, f64::INFINITY] {
            let p = SearchParams {
                inlier_px: radius,
                ..p
            };
            assert_eq!(
                bits(score(&cam, &offsets, &normals, &t, &blobs, &rays[..n], &p)),
                bits(baseline_score(
                    &cam,
                    &offsets,
                    &normals,
                    &t,
                    &blobs,
                    &rays[..n],
                    &p
                ))
            );
        }
        // Duplicate coordinates cause multiple LEDs to compete for one blob.
        if n < blobs.len() {
            blobs[n].xy = blobs[0].xy;
        }
    }
}

#[test]
#[ignore = "offline release benchmark"]
fn benchmark_followup_search_score() {
    use std::{hint::black_box, time::Instant};
    let (cam, offsets, normals, t, blobs, rays) = fixture();
    let p = SearchParams::default();
    let mut times = [Vec::new(), Vec::new()];
    for round in 0..12 {
        for which in [round % 2, 1 - round % 2] {
            let start = Instant::now();
            for _ in 0..5000 {
                let f = if which == 0 { baseline_score } else { score };
                black_box(f(
                    black_box(&cam),
                    &offsets,
                    &normals,
                    &t,
                    &blobs,
                    &rays,
                    &p,
                ));
            }
            times[which].push(start.elapsed().as_secs_f64() * 1e6 / 5000.0);
        }
    }
    for (name, t) in ["baseline", "pair scan"].into_iter().zip(&mut times) {
        t.sort_by(f64::total_cmp);
        eprintln!(
            "{name} score us/call min/median/max: {:.3}/{:.3}/{:.3}",
            t[0], t[6], t[11]
        );
    }
}

#[test]
fn streamed_ray_fit_matches_collecting_baseline() {
    let centre = Vector3::new(0.1, -0.5, 0.3);
    let (_, offsets, _, _, _, rays) = fixture();
    let baseline = |pairs: Vec<Option<(Vector3<f64>, Vector3<f64>)>>| {
        let pairs: Option<Vec<_>> = pairs.into_iter().collect();
        pairs.and_then(|pairs| {
            let mut a = nalgebra::Matrix3::zeros();
            let mut rhs = Vector3::zeros();
            for (o, d) in &pairs {
                let across = nalgebra::Matrix3::identity() - d * d.transpose();
                a += across;
                rhs += across * (centre - o);
            }
            a.try_inverse().map(|inv| inv * rhs)
        })
    };
    for n in 0..=offsets.len() {
        let mut pairs: Vec<_> = offsets
            .iter()
            .zip(&rays)
            .take(n)
            .map(|(&o, &(_, d))| Some((o, d)))
            .collect();
        for missing in 0..=n {
            if missing > 0 {
                pairs[missing - 1] = None;
            }
            let (actual, counts) = counted(|| closest_to_rays(&centre, pairs.iter().copied()));
            let expected = baseline(pairs.clone());
            let bits = |v: Option<Vector3<f64>>| v.map(|v| v.map(f64::to_bits));
            assert_eq!(bits(actual), bits(expected));
            assert_eq!(counts.0, 0);
        }
    }
}

fn sweep_baseline_score(
    cam: &Camera,
    offsets: &[Vector3<f64>],
    normals: &[Vector3<f64>],
    t: &Vector3<f64>,
    blobs: &[Blob],
    rays: &[(usize, Vector3<f64>)],
    p: &SearchParams,
) -> Option<Scored> {
    let centre = cam.world_from_cam.translation.vector;
    let cam_from_world = cam.world_from_cam.rotation.inverse();
    let mut near: Vec<(f64, usize, usize)> = Vec::new();
    let mut shown = 0;
    for (k, (o, n)) in offsets.iter().zip(normals).enumerate() {
        let w = t + o;
        let to_cam = centre - w;
        if n.dot(&to_cam) <= p.min_facing * to_cam.norm() {
            continue;
        }
        let Some(uv) = cam.model.project(&(cam_from_world * -to_cam)) else {
            continue;
        };
        if uv.x < 0.0
            || uv.y < 0.0
            || uv.x >= cam.model.width as f64
            || uv.y >= cam.model.height as f64
        {
            continue;
        }
        shown += 1;
        let nearest = rays
            .iter()
            .map(|&(b, _)| ((blobs[b].xy - uv).norm_squared(), b))
            .min_by(|x, y| x.0.total_cmp(&y.0));
        if let Some((d2, b)) = nearest.filter(|(d2, _)| *d2 <= p.inlier_px * p.inlier_px) {
            near.push((d2, k, b));
        }
    }
    if near.len() < p.min_leds {
        return None;
    }
    near.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut pairs = Vec::new();
    let mut sq_px = 0.0;
    for (d2, k, b) in near {
        // Each LED contributes at most one entry to near. Only blobs can repeat;
        // scan the short output instead of allocating and hashing two sets.
        if !pairs.iter().any(|&(_, used)| used == b) {
            pairs.push((k, b));
            sq_px += d2;
        }
    }
    (pairs.len() >= p.min_leds && pairs.len() as f64 >= p.min_share * shown as f64).then_some(
        Scored {
            pairs,
            shown,
            sq_px,
        },
    )
}

#[test]
fn sweep_score_capacity_eliminates_growth() {
    let (cam, offsets, normals, t, blobs, rays) = fixture();
    let p = SearchParams::default();
    let bits = |s: Option<Scored>| s.map(|s| (s.pairs, s.shown, s.sq_px.to_bits()));
    for n in 0..=rays.len() {
        let (new, nc) = counted(|| score(&cam, &offsets, &normals, &t, &blobs, &rays[..n], &p));
        let (old, oc) =
            counted(|| sweep_baseline_score(&cam, &offsets, &normals, &t, &blobs, &rays[..n], &p));
        assert_eq!(bits(new), bits(old));
        assert!(nc.0 <= oc.0);
        if n == rays.len() {
            assert!(nc.0 < oc.0, "{nc:?} vs {oc:?}");
            assert_eq!(nc.0, 2, "one candidate buffer and one match buffer");
        }
    }
}
