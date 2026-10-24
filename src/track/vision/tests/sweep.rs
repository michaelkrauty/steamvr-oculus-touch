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
