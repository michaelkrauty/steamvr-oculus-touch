// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! Frozen 5a949b8 algorithms for the additional hot-path sweep.
use super::*;
use crate::track::tracker::stillness_tests::filter_bits;

fn baseline_fit_pairs(
    cam: &Camera,
    body: &Body,
    prior: &Filter,
    blobs: &[Blob],
    mut pairs: Vec<(usize, usize)>,
    p: &FitParams,
) -> Result<Fit, Miss> {
    if pairs.len() < p.min_leds {
        return Err(Miss::TooFew);
    }
    let mut updated = prior
        .update(|x| residuals(cam, body, blobs, &pairs, x), &p.update)
        .ok_or(Miss::Rejected)?;
    // per pair (a pair that no longer projects counts as an outlier, keeping pairs and residuals aligned)
    let pair_residual = |x: &Nominal, &(led, b): &(usize, usize)| {
        project(cam, body, x, led).map_or(f64::INFINITY, |pr| (blobs[b].xy - pr.uv).norm())
    };
    let original_len = pairs.len();
    pairs.retain(|pr| pair_residual(&updated.filter.x, pr) <= p.inlier_px);
    if pairs.len() < p.min_leds
        || (pairs.len() as f64) < p.min_inlier_fraction * original_len as f64
    {
        return Err(Miss::Rejected);
    }
    if pairs.len() < original_len {
        updated = prior
            .update(|x| residuals(cam, body, blobs, &pairs, x), &p.update)
            .ok_or(Miss::Rejected)?;
        if pairs
            .iter()
            .any(|pr| pair_residual(&updated.filter.x, pr) > p.inlier_px)
        {
            return Err(Miss::Rejected);
        }
    }
    let e = updated
        .filter
        .x
        .minus(&prior.x)
        .fixed_rows::<6>(0)
        .into_owned();
    let p6 = prior.cov.fixed_view::<6, 6>(0, 0).into_owned();
    if p6
        .cholesky()
        .is_none_or(|c| (e.transpose() * c.solve(&e))[0] > p.max_correction)
    {
        return Err(Miss::Rejected);
    }
    let unexplained = unexplained_blobs(
        cam,
        body,
        &updated.filter.x,
        blobs,
        &pairs,
        p.assoc.min_facing,
        3.0,
    );
    if unexplained >= p.min_unexplained_reject
        && unexplained as f64 >= p.unexplained_ratio * pairs.len() as f64
    {
        return Err(Miss::Rejected);
    }
    Ok(Fit {
        updated,
        pairs,
        unexplained,
    })
}

fn assert_fit_equal(a: Result<Fit, Miss>, b: Result<Fit, Miss>) {
    match (a, b) {
        (Ok(a), Ok(b)) => {
            assert_eq!(a.pairs, b.pairs);
            assert_eq!(a.unexplained, b.unexplained);
            assert_eq!(
                filter_bits(&a.updated.filter),
                filter_bits(&b.updated.filter)
            );
            assert_eq!(
                a.updated
                    .residuals_px
                    .iter()
                    .map(|v| v.to_bits())
                    .collect::<Vec<_>>(),
                b.updated
                    .residuals_px
                    .iter()
                    .map(|v| v.to_bits())
                    .collect::<Vec<_>>()
            );
        }
        (Err(a), Err(b)) => assert_eq!(a, b),
        (a, b) => panic!("different outcomes: {a:?} vs {b:?}"),
    }
}

#[test]
fn sweep_outlier_checks_match_full_projection() {
    let (cam, body, x) = (camera(), body(), state());
    let seen = blobs(&cam, &body, &x);
    let pairs: Vec<_> = seen
        .iter()
        .enumerate()
        .map(|(b, (led, _))| (*led, b))
        .collect();
    let list: Vec<_> = seen.iter().map(|(_, b)| *b).collect();
    let prior = filter(x, 0.1, 0.1);
    assert!(fit_pairs(
        &cam,
        &body,
        &prior,
        &list,
        pairs.clone(),
        &FitParams::default()
    )
    .is_ok());
    for n in 0..=pairs.len() {
        for delta in [0.0, 0.5, 4.0, 30.0, f64::NAN] {
            let mut list = list.clone();
            if let Some(blob) = list.first_mut() {
                blob.xy.x += delta;
            }
            for inlier_px in [0.0, 0.5, 3.0, f64::INFINITY, f64::NAN] {
                let p = FitParams {
                    inlier_px,
                    ..FitParams::default()
                };
                assert_fit_equal(
                    fit_pairs(&cam, &body, &prior, &list, pairs[..n].to_vec(), &p),
                    baseline_fit_pairs(&cam, &body, &prior, &list, pairs[..n].to_vec(), &p),
                );
            }
        }
    }
}

#[test]
#[ignore = "offline release benchmark"]
fn benchmark_sweep_fit() {
    use std::{hint::black_box, time::Instant};
    let (cam, body, x) = (camera(), body(), state());
    let seen = blobs(&cam, &body, &x);
    let pairs: Vec<_> = seen
        .iter()
        .enumerate()
        .map(|(b, (led, _))| (*led, b))
        .collect();
    let list: Vec<_> = seen.iter().map(|(_, b)| *b).collect();
    let prior = filter(x, 0.1, 0.1);
    let p = FitParams::default();
    let mut times = [Vec::new(), Vec::new()];
    for round in 0..12 {
        for which in [round % 2, 1 - round % 2] {
            let f = if which == 0 {
                baseline_fit_pairs
            } else {
                fit_pairs
            };
            let start = Instant::now();
            for _ in 0..1000 {
                black_box(f(black_box(&cam), &body, &prior, &list, pairs.clone(), &p)).ok();
            }
            times[which].push(start.elapsed().as_secs_f64() * 1e6 / 1000.0);
        }
    }
    for (name, t) in ["baseline fit", "pixel fit"].into_iter().zip(&mut times) {
        t.sort_by(f64::total_cmp);
        eprintln!(
            "{name}: {:.3}/{:.3}/{:.3} us min/median/max",
            t[0], t[6], t[11]
        );
    }
}

fn baseline_associate_visible(
    visible: &[Projection],
    f: &Filter,
    blobs: &[Blob],
    sigma_px: f64,
    p: &AssocParams,
) -> Vec<(usize, usize)> {
    let p6 = f.cov.fixed_view::<6, 6>(0, 0).into_owned();
    let var = Matrix2::identity() * (sigma_px * sigma_px);
    // (d2, visible index, blob)
    let mut cand: Vec<(f64, usize, usize)> = Vec::new();
    for (k, pr) in visible.iter().enumerate() {
        let Some(s_inv) = (pr.j * p6 * pr.j.transpose() + var).try_inverse() else {
            continue;
        };
        for (b, blob) in blobs.iter().enumerate() {
            let d = blob.xy - pr.uv;
            if d.norm() > p.max_px {
                continue;
            }
            let d2 = (d.transpose() * s_inv * d)[0];
            if d2 < p.gate {
                cand.push((d2, k, b));
            }
        }
    }
    cand.sort_by(|a, b| a.0.total_cmp(&b.0));
    let lone = |k: usize| {
        visible
            .iter()
            .enumerate()
            .all(|(m, o)| m == k || (o.uv - visible[k].uv).norm() > p.merge_px)
    };
    let (mut led_done, mut blob_done) = (vec![false; visible.len()], vec![false; blobs.len()]);
    let mut out = Vec::new();
    for &(d2, k, b) in &cand {
        if led_done[k] || blob_done[b] {
            continue;
        }
        led_done[k] = true;
        blob_done[b] = true;
        let rival = cand.iter().any(|&(e2, k2, b2)| {
            ((b2 == b && k2 != k) || (k2 == k && b2 != b)) && e2 < d2 + p.margin
        });
        if !rival && lone(k) {
            out.push((visible[k].led, b));
        }
    }
    out.sort();
    out
}

fn association_fixture() -> (Vec<Projection>, Filter, Vec<Blob>) {
    let (cam, b, x) = (camera(), body(), state());
    let visible: Vec<_> = (0..b.leds.len())
        .filter_map(|i| project(&cam, &b, &x, i))
        .collect();
    let blobs = visible
        .iter()
        .map(|pr| Blob {
            xy: pr.uv,
            led_id: -1,
        })
        .collect();
    (visible, filter(x, 0.1, 0.1), blobs)
}

#[test]
fn sweep_association_matches_baseline() {
    let (visible, f, blobs) = association_fixture();
    let mut cases = vec![blobs.clone(), Vec::new()];
    cases.push(
        blobs
            .iter()
            .map(|b| Blob {
                xy: b.xy + Vector2::repeat(1000.0),
                ..*b
            })
            .collect(),
    );
    cases.push(vec![blobs[0]; 8]);
    cases.push(vec![Blob {
        xy: Vector2::repeat(f64::NAN),
        led_id: -1,
    }]);
    for list in cases {
        for max_px in [-1.0, 0.0, 1.0, 30.0, f64::INFINITY, f64::NAN] {
            for margin in [-1.0, 0.0, 6.0, f64::INFINITY, f64::NAN] {
                for sigma in [0.0, 0.5, f64::NAN] {
                    for cov in [f.cov, Cov::zeros()] {
                        let f = Filter { cov, ..f.clone() };
                        let p = AssocParams {
                            max_px,
                            margin,
                            ..AssocParams::default()
                        };
                        assert_eq!(
                            associate_visible(&visible, &f, &list, sigma, &p),
                            baseline_associate_visible(&visible, &f, &list, sigma, &p)
                        );
                    }
                }
            }
        }
    }
}

#[test]
#[ignore = "offline release benchmark"]
fn benchmark_sweep_association() {
    use std::{hint::black_box, time::Instant};
    let (visible, f, list) = association_fixture();
    let p = AssocParams::default();
    let f = filter(f.x, 0.001, 0.001);
    for (label, list) in [
        ("near", list.clone()),
        (
            "far",
            list.iter()
                .map(|b| Blob {
                    xy: b.xy + Vector2::repeat(1000.0),
                    ..*b
                })
                .collect(),
        ),
    ] {
        let mut times = [Vec::new(), Vec::new()];
        for round in 0..12 {
            for which in [round % 2, 1 - round % 2] {
                let fun = if which == 0 {
                    baseline_associate_visible
                } else {
                    associate_visible
                };
                let start = Instant::now();
                for _ in 0..5000 {
                    black_box(fun(black_box(&visible), &f, &list, 0.5, &p));
                }
                times[which].push(start.elapsed().as_secs_f64() * 1e6 / 5000.0);
            }
        }
        for (name, t) in ["baseline", "optimized"].into_iter().zip(&mut times) {
            t.sort_by(f64::total_cmp);
            eprintln!(
                "{label} {name}: {:.3}/{:.3}/{:.3} us min/median/max",
                t[0], t[6], t[11]
            );
        }
    }
}

#[test]
fn sweep_no_candidates_avoids_masks() {
    use crate::track::tracker::stillness_tests::counted;
    let (visible, f, list) = association_fixture();
    let p = AssocParams {
        gate: 0.0,
        ..AssocParams::default()
    };
    let (new, nc) = counted(|| associate_visible(&visible, &f, &list, 0.5, &p));
    let (old, oc) = counted(|| baseline_associate_visible(&visible, &f, &list, 0.5, &p));
    assert_eq!(new, old);
    assert_eq!(nc.0, 0);
    assert_eq!(oc.0, 2);
}

#[test]
fn sweep_empty_blobs_skip_projection_allocation() {
    use crate::track::tracker::stillness_tests::counted;
    let (cam, b, x) = (camera(), body(), state());
    let f = filter(x, 0.1, 0.1);
    let p = AssocParams::default();
    let (new, nc) = counted(|| associate(&cam, &b, &f, &[], 0.5, &p));
    let (old, oc) = counted(|| super::hot_paths::original_associate(&cam, &b, &f, &[], 0.5, &p));
    assert_eq!(new, old);
    assert_eq!(nc.0, 0);
    assert!(oc.0 > 0);
}

fn baseline_pixel_residuals(
    cam: &Camera,
    body: &Body,
    blobs: &[Blob],
    pairs: &[(usize, usize)],
    x: &Nominal,
) -> Vec<Vector2<f64>> {
    let projector = Projector::new(cam, body, x);
    pairs
        .iter()
        .filter_map(|&(led, b)| projector.pixel(led).map(|(uv, _)| blobs[b].xy - uv))
        .collect()
}

#[test]
fn sweep_diagnostic_capacity_preserves_bits_and_avoids_growth() {
    use crate::track::tracker::stillness_tests::counted;
    let (cam, b, mut x) = (camera(), body(), state());
    let seen = blobs(&cam, &b, &x);
    let list: Vec<_> = seen.iter().map(|s| s.1).collect();
    let pairs: Vec<_> = seen.iter().enumerate().map(|(i, s)| (s.0, i)).collect();
    let bits = |v: Vec<Vector2<f64>>| {
        v.into_iter()
            .map(|p| [p.x.to_bits(), p.y.to_bits()])
            .collect::<Vec<_>>()
    };
    for n in 0..=pairs.len() {
        let (new, nc) = counted(|| pixel_residuals(&cam, &b, &list, &pairs[..n], &x));
        let (old, oc) = counted(|| baseline_pixel_residuals(&cam, &b, &list, &pairs[..n], &x));
        assert_eq!(bits(new), bits(old));
        assert_eq!(nc.0, usize::from(n > 0));
        assert!(nc.0 <= oc.0);
        if n > 4 {
            assert!(nc.0 < oc.0);
        }
    }
    x.p = cam.world_from_cam.translation.vector
        + cam.world_from_cam.rotation * Vector3::new(0.0, 0.0, -10.0);
    let (new, nc) = counted(|| pixel_residuals(&cam, &b, &list, &pairs, &x));
    assert!(new.is_empty());
    assert_eq!(nc.0, 0);
}

fn sweep_baseline_predicted_pixels(
    cam: &Camera,
    body: &Body,
    x: &Nominal,
    min_facing: f64,
) -> Vec<Vector2<f64>> {
    let projector = Projector::new(cam, body, x);
    (0..body.leds.len())
        .filter_map(|i| projector.pixel(i))
        .filter(|(_, facing)| *facing > min_facing)
        .map(|(uv, _)| uv)
        .collect()
}

#[test]
fn sweep_predicted_capacity_preserves_pixels_and_avoids_growth() {
    use crate::track::tracker::stillness_tests::counted;
    let (cam, b, x) = (camera(), body(), state());
    for min_facing in [-1.0, 0.0, 0.2, 1.0, f64::NAN] {
        let (new, nc) = counted(|| predicted_pixels(&cam, &b, &x, min_facing));
        let (old, oc) = counted(|| sweep_baseline_predicted_pixels(&cam, &b, &x, min_facing));
        assert_eq!(new, old);
        assert_eq!(nc.0, usize::from(!new.is_empty()));
        assert!(nc.0 <= oc.0);
        if min_facing == -1.0 {
            assert!(nc.0 < oc.0);
        }
    }
}

fn sweep_baseline_gate_pairs(
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
            let d = blobs[b].xy - pr.uv;
            let within = pr.facing > p.min_facing && d.norm() <= p.max_px;
            if !within {
                return false;
            }
            let Some(s_inv) = (pr.j * p6 * pr.j.transpose() + var).try_inverse() else {
                return false;
            };
            (d.transpose() * s_inv * d)[0] < p.gate
        })
        .collect()
}

#[test]
fn sweep_hint_capacity_preserves_matches_and_avoids_growth() {
    use crate::track::tracker::stillness_tests::counted;
    let (cam, b, x) = (camera(), body(), state());
    let seen = blobs(&cam, &b, &x);
    let list: Vec<_> = seen.iter().map(|s| s.1).collect();
    let pairs: Vec<_> = seen.iter().enumerate().map(|(i, s)| (s.0, i)).collect();
    let f = filter(x, 0.1, 0.1);
    for n in 0..=pairs.len() {
        for min_facing in [-1.0, 0.2, 1.0, f64::NAN] {
            let p = AssocParams {
                min_facing,
                ..AssocParams::default()
            };
            let (new, nc) = counted(|| gate_pairs(&cam, &b, &f, &list, &pairs[..n], 0.5, &p));
            let (old, oc) =
                counted(|| sweep_baseline_gate_pairs(&cam, &b, &f, &list, &pairs[..n], 0.5, &p));
            assert_eq!(new, old);
            assert_eq!(nc.0, usize::from(!new.is_empty()));
            assert!(nc.0 <= oc.0);
            if n > 4 && min_facing == -1.0 {
                assert!(nc.0 < oc.0);
            }
        }
    }
}
