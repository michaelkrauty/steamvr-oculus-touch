// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Independent pre-batch algorithms; do not update these to mirror optimized code.
use super::*;
use crate::track::tracker::stillness_tests::{counted, filter_bits};

pub(crate) fn original_association_report(
    cam: &Camera,
    body: &Body,
    f: &Filter,
    blobs: &[Blob],
    sigma_px: f64,
    p: &AssocParams,
    near_px: f64,
) -> (usize, usize, usize) {
    let (w, h) = (cam.model.width as f64, cam.model.height as f64);
    let visible: Vec<Projection> = (0..body.leds.len())
        .filter_map(|i| project(cam, body, &f.x, i))
        .filter(|pr| {
            pr.facing > p.min_facing
                && pr.uv.x >= 0.0
                && pr.uv.y >= 0.0
                && pr.uv.x < w
                && pr.uv.y < h
        })
        .collect();
    let near = visible
        .iter()
        .filter(|pr| blobs.iter().any(|b| (b.xy - pr.uv).norm() <= near_px))
        .count();
    (
        visible.len(),
        near,
        original_associate(cam, body, f, blobs, sigma_px, p).len(),
    )
}

pub(crate) fn original_associate(
    cam: &Camera,
    body: &Body,
    f: &Filter,
    blobs: &[Blob],
    sigma_px: f64,
    p: &AssocParams,
) -> Vec<(usize, usize)> {
    let (w, h) = (cam.model.width as f64, cam.model.height as f64);
    let visible: Vec<Projection> = (0..body.leds.len())
        .filter_map(|i| project(cam, body, &f.x, i))
        .filter(|pr| {
            pr.facing > p.min_facing
                && pr.uv.x >= 0.0
                && pr.uv.y >= 0.0
                && pr.uv.x < w
                && pr.uv.y < h
        })
        .collect();
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

pub(crate) fn original_unexplained_blobs(
    cam: &Camera,
    body: &Body,
    x: &Nominal,
    blobs: &[Blob],
    used: &[(usize, usize)],
    min_facing: f64,
    margin: f64,
) -> usize {
    let uv: Vec<Vector2<f64>> = (0..body.leds.len())
        .filter_map(|i| project(cam, body, x, i))
        .filter(|pr| pr.facing > min_facing)
        .map(|pr| pr.uv)
        .collect();
    if uv.is_empty() {
        return 0;
    }
    let (lo, hi) = uv
        .iter()
        .fold((uv[0], uv[0]), |(lo, hi), p| (lo.inf(p), hi.sup(p)));
    blobs
        .iter()
        .enumerate()
        .filter(|(i, b)| {
            !used.iter().any(|u| u.1 == *i)
                && b.xy.x >= lo.x - margin
                && b.xy.y >= lo.y - margin
                && b.xy.x <= hi.x + margin
                && b.xy.y <= hi.y + margin
        })
        .count()
}

pub(crate) fn original_fit_pairs(
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
    let keep: Vec<(usize, usize)> = pairs
        .iter()
        .filter(|pr| pair_residual(&updated.filter.x, pr) <= p.inlier_px)
        .copied()
        .collect();
    if keep.len() < p.min_leds || (keep.len() as f64) < p.min_inlier_fraction * pairs.len() as f64 {
        return Err(Miss::Rejected);
    }
    if keep.len() < pairs.len() {
        pairs = keep;
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
    let unexplained = original_unexplained_blobs(
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

#[test]
fn counts_association_and_bounds_match_original() {
    let cam = camera();
    let b = body();
    for seed in 0..24 {
        let mut x = state();
        x.q = UnitQuaternion::from_euler_angles(
            seed as f64 * 0.13,
            seed as f64 * 0.23,
            seed as f64 * 0.31,
        );
        let f = filter(x, 0.005, 0.002);
        let mut list: Vec<_> = predicted_pixels(&cam, &b, &x, -2.0)
            .into_iter()
            .map(|xy| Blob { xy, led_id: -1 })
            .collect();
        list.push(Blob {
            xy: Vector2::new(f64::NAN, 0.0),
            led_id: -1,
        });
        for facing in [-2.0, 0.2, 2.0, f64::NAN] {
            let p = AssocParams {
                min_facing: facing,
                ..Default::default()
            };
            assert_eq!(
                associate(&cam, &b, &f, &list, 1.0, &p),
                original_associate(&cam, &b, &f, &list, 1.0, &p)
            );
            for distance in [-1.0, 0.0, 3.0, f64::INFINITY, f64::NAN] {
                let expected = original_association_report(&cam, &b, &f, &list, 1.0, &p, distance);
                assert_eq!(
                    association_report(&cam, &b, &f, &list, 1.0, &p, distance),
                    expected
                );
                let (counts, allocs) =
                    counted(|| visibility_counts(&cam, &b, &x, &list, &p, distance));
                assert_eq!(counts, (expected.0, expected.1));
                assert_eq!(allocs, (0, 0, 0));
                let (count, allocs) =
                    counted(|| unexplained_blobs(&cam, &b, &x, &list, &[], facing, distance));
                assert_eq!(
                    count,
                    original_unexplained_blobs(&cam, &b, &x, &list, &[], facing, distance)
                );
                assert_eq!(allocs, (0, 0, 0));
            }
        }
    }
    let empty = Body {
        leds: Vec::new(),
        imu_pos: Vector3::zeros(),
    };
    assert_eq!(
        visibility_counts(&cam, &empty, &state(), &[], &AssocParams::default(), 3.0),
        (0, 0)
    );
}

#[test]
fn retained_pairs_match_original_including_successful_refits() {
    let cam = camera();
    let b = body();
    let mut refits = 0;
    let mut saved = 0;
    let bits = |r: &Result<Fit, Miss>| {
        r.as_ref()
            .map(|f| {
                (
                    filter_bits(&f.updated.filter),
                    f.updated
                        .residuals_px
                        .iter()
                        .map(|x| x.to_bits())
                        .collect::<Vec<_>>(),
                    f.pairs.clone(),
                    f.unexplained,
                )
            })
            .map_err(|e| *e)
    };
    for seed in 0..12 {
        let mut x = state();
        x.q = UnitQuaternion::from_euler_angles(
            seed as f64 * 0.11,
            seed as f64 * 0.17,
            seed as f64 * 0.21,
        );
        let prior = filter(x, 0.005, 0.002);
        let seen = blobs(&cam, &b, &x);
        let pairs: Vec<_> = seen.iter().enumerate().map(|(i, s)| (s.0, i)).collect();
        for outliers in [0, 1, seen.len()] {
            let mut list: Vec<_> = seen.iter().map(|s| s.1).collect();
            for blob in list.iter_mut().take(outliers) {
                blob.xy.x += 10.0;
            }
            for gate in [0.0, 2.0, f64::NAN] {
                let p = FitParams {
                    inlier_px: gate,
                    ..Default::default()
                };
                let mut a = Vec::with_capacity(pairs.len() + 64);
                a.extend_from_slice(&pairs);
                let mut z = Vec::with_capacity(pairs.len() + 64);
                z.extend_from_slice(&pairs);
                let (old, oc) = counted(|| original_fit_pairs(&cam, &b, &prior, &list, a, &p));
                let (new, nc) = counted(|| fit_pairs(&cam, &b, &prior, &list, z, &p));
                assert_eq!(bits(&old), bits(&new));
                assert!(nc.0 <= oc.0);
                saved += oc.0 - nc.0;
                if let Ok(fit) = new {
                    refits += usize::from(fit.pairs.len() < pairs.len());
                }
            }
        }
    }
    assert!(refits > 0);
    assert!(saved > 0);
}
