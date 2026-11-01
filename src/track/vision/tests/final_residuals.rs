// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Frozen pre-final-residual fit orchestration. Keep the oracle independent of the optimized
//! fit paths; Filter::update retains the legacy full-measurement final callback.
//! Normal count/parity tests: `cargo test --no-default-features --lib final_residual`.
//! Uninstrumented benchmark: `CV1TOUCH_NO_PROJECTION_COUNT=1 cargo test --no-default-features --lib final_residual_benchmark -- --ignored --nocapture --test-threads=1`.
use super::*;
use crate::calib::camera::projection_count::PROJECTIONS;
use crate::track::eskf::{ErrVec, PixelResidual};
use std::{
    cell::Cell,
    hint::black_box,
    time::{Duration, Instant},
};

fn legacy_fit_frame(
    cam: &Camera,
    body: &Body,
    prior: &Filter,
    blobs: &[Blob],
    hints: &[(usize, usize)],
    p: &FitParams,
) -> Result<Fit, Miss> {
    let sigma = p.update.sigma_px;
    let hints = gate_pairs(cam, body, prior, blobs, hints, sigma, &p.assoc);
    let mut pairs = merge_pairs(associate(cam, body, prior, blobs, sigma, &p.assoc), &hints);
    if pairs.len() < p.min_leds {
        pairs = merge_pairs(
            associate_shifted(
                cam,
                body,
                prior,
                blobs,
                sigma,
                &p.assoc,
                p.snap_px,
                p.shift_min_leds,
                p.shift_margin,
            ),
            &hints,
        );
    }
    if pairs.len() < p.min_leds {
        return Err(Miss::TooFew);
    }
    let first = prior
        .update(|x| residuals(cam, body, blobs, &pairs, x), &p.update)
        .ok_or(Miss::Rejected)?;
    let again = merge_pairs(
        associate(cam, body, &first.filter, blobs, sigma, &p.assoc),
        &hints,
    );
    if again.len() > pairs.len() {
        pairs = again;
    }
    legacy_fit_pairs(cam, body, prior, blobs, pairs, p)
}

/// The update for given matches, with the outlier refit and the acceptance test.
fn legacy_fit_pairs(
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
    let pair_residual = |projector: &Projector<'_>, &(led, b): &(usize, usize)| {
        projector
            .pixel(led)
            .map_or(f64::INFINITY, |(uv, _)| (blobs[b].xy - uv).norm())
    };
    let original_len = pairs.len();
    let projector = Projector::new(cam, body, &updated.filter.x);
    pairs.retain(|pr| pair_residual(&projector, pr) <= p.inlier_px);
    if pairs.len() < p.min_leds
        || (pairs.len() as f64) < p.min_inlier_fraction * original_len as f64
    {
        return Err(Miss::Rejected);
    }
    if pairs.len() < original_len {
        updated = prior
            .update(|x| residuals(cam, body, blobs, &pairs, x), &p.update)
            .ok_or(Miss::Rejected)?;
        let projector = Projector::new(cam, body, &updated.filter.x);
        if pairs
            .iter()
            .any(|pr| pair_residual(&projector, pr) > p.inlier_px)
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

fn result_bits(r: &Result<Fit, Miss>) -> Result<(Vec<u64>, Vec<(usize, usize)>, usize), Miss> {
    r.as_ref()
        .map(|f| {
            let x = &f.updated.filter.x;
            let bits =
                x.q.coords
                    .iter()
                    .chain(x.p.iter())
                    .chain(x.v.iter())
                    .chain(x.bg.iter())
                    .chain(x.ba.iter())
                    .chain(f.updated.filter.cov.iter())
                    .chain(f.updated.residuals_px.iter())
                    .map(|x| x.to_bits())
                    .collect();
            (bits, f.pairs.clone(), f.unexplained)
        })
        .map_err(|e| *e)
}

fn fixture() -> (Camera, Body, Filter, Vec<Blob>, Vec<(usize, usize)>) {
    let (cam, b, truth) = (camera(), body(), state());
    let seen = blobs(&cam, &b, &truth);
    let list = seen.iter().map(|s| s.1).collect();
    let hints = seen.iter().enumerate().map(|(i, s)| (s.0, i)).collect();
    let prior_x = truth.plus(&ErrVec::from_iterator([
        0.004, -0.003, 0.002, 0.001, -0.0008, 0.0012, 0., 0., 0., 0., 0., 0., 0., 0., 0.,
    ]));
    (cam, b, filter(prior_x, 0.005, 0.002), list, hints)
}

#[test]
fn final_residual_counts() {
    assert!(
        option_env!("CV1TOUCH_NO_PROJECTION_COUNT").is_none(),
        "count regression requires the default test instrumentation"
    );
    let (cam, b, prior, list, hints) = fixture();
    let p = FitParams::default();
    PROJECTIONS.with(|n| n.set(0));
    let old = legacy_fit_frame(&cam, &b, &prior, &list, &hints, &p);
    let old_count = PROJECTIONS.with(Cell::get);
    PROJECTIONS.with(|n| n.set(0));
    let new = fit_frame(&cam, &b, &prior, &list, &hints, &p);
    let new_count = PROJECTIONS.with(Cell::get);
    assert_eq!(result_bits(&old), result_bits(&new));
    assert_eq!(new.unwrap().pairs.len(), 12);
    // Two successful 12-pair final evaluations; no metrics or outlier pass skipped.
    assert_eq!((old_count, new_count), (1128, 984));
    assert_eq!(old_count - new_count, 144);
    println!("12-pair fit camera projections: {old_count} -> {new_count}, saved 144");
}

#[test]
fn final_residual_equivalence() {
    let (cam, b) = (camera(), body());
    let (mut ok, mut too_few, mut rejected, mut refits) = (0, 0, 0, 0);
    for seed in 0..24 {
        let mut truth = state();
        truth.q = UnitQuaternion::from_euler_angles(
            seed as f64 * 0.11,
            seed as f64 * 0.17,
            seed as f64 * 0.21,
        );
        let seen = blobs(&cam, &b, &truth);
        let hints: Vec<_> = seen.iter().enumerate().map(|(i, s)| (s.0, i)).collect();
        for iterations in [0, 1, 4, 8] {
            for outliers in [0, 1, seen.len()] {
                let mut list: Vec<_> = seen.iter().map(|s| s.1).collect();
                for blob in list.iter_mut().take(outliers) {
                    blob.xy.x += 10.0;
                }
                let mut px = truth;
                px.p.x += 0.001 * (seed % 4) as f64;
                let prior = filter(px, 0.005, 0.002);
                for mode in 0..6 {
                    let mut p = FitParams::default();
                    p.update.iterations = iterations;
                    match mode {
                        1 => p.inlier_px = 0.0,
                        2 => p.max_correction = 0.0,
                        3 => p.inlier_px = f64::NAN,
                        4 => p.min_leds = seen.len() + 1,
                        _ => (),
                    }
                    let supplied = if mode == 5 { &[][..] } else { &hints[..] };
                    let old = legacy_fit_frame(&cam, &b, &prior, &list, supplied, &p);
                    let new = fit_frame(&cam, &b, &prior, &list, supplied, &p);
                    assert_eq!(
                        result_bits(&old),
                        result_bits(&new),
                        "seed={seed}, iterations={iterations}, outliers={outliers}, mode={mode}"
                    );
                    match old {
                        Ok(_) => ok += 1,
                        Err(Miss::TooFew) => too_few += 1,
                        Err(Miss::Rejected) => rejected += 1,
                    }
                    let old = legacy_fit_pairs(&cam, &b, &prior, &list, hints.clone(), &p);
                    let new = fit_pairs(&cam, &b, &prior, &list, hints.clone(), &p);
                    assert_eq!(result_bits(&old), result_bits(&new));
                    if let Ok(f) = old {
                        refits += usize::from(f.pairs.len() < hints.len());
                    }
                }
            }
        }
    }
    assert!(ok > 0 && too_few > 0 && rejected > 0 && refits > 0);
    println!("1728 fit_frame and 1728 fit_pairs comparisons: frame ok={ok}, too_few={too_few}, rejected={rejected}; accepted refits={refits}");
}

#[test]
fn final_residual_projection_edges_and_failures() {
    let mut cam = camera();
    cam.world_from_cam = Isometry3::identity();
    let mut b = body();
    b.imu_pos = Vector3::zeros();
    b.leds.truncate(1);
    b.leds[0].pos = Vector3::zeros();
    let mut x = state();
    x.q = UnitQuaternion::identity();
    let list = [Blob {
        xy: Vector2::new(1.0, 2.0),
        led_id: -1,
    }];
    let pairs = [(0, 0)];
    let bits = |v: Vec<f64>| v.into_iter().map(f64::to_bits).collect::<Vec<_>>();
    let mut cases = 0;
    for z in [
        -1.0,
        0.0,
        1e-6,
        1.0000001e-6,
        1.001e-6,
        2e-6,
        1.0,
        f64::INFINITY,
        f64::NAN,
    ] {
        for a in [0.0, 0.1, 1e9, f64::NAN, f64::INFINITY] {
            x.p = Vector3::new(a, -a, z);
            let old: Vec<_> = residuals(&cam, &b, &list, &pairs, &x)
                .iter()
                .map(|r| r.r.norm())
                .collect();
            let new = final_residual_norms(&cam, &b, &list, &pairs, &x);
            assert_eq!(bits(old), bits(new), "point={:?}", x.p);
            cases += 1;
        }
    }
    // Filtering must preserve the order of surviving pairs, even with a NaN norm.
    x.p = Vector3::new(0.0, 0.0, 1.0);
    b.leds = [Vector3::zeros(), Vector3::new(0.0, 0.0, -2.0), Vector3::x()]
        .into_iter()
        .map(|pos| Led {
            pos,
            dir: Vector3::z(),
        })
        .collect();
    let list = [
        Blob {
            xy: Vector2::new(3.0, 4.0),
            led_id: -1,
        },
        Blob {
            xy: Vector2::new(f64::NAN, 1.0),
            led_id: -1,
        },
    ];
    let pairs = [(2, 0), (1, 0), (0, 1), (0, 0)];
    let expected: Vec<_> = residuals(&cam, &b, &list, &pairs, &x)
        .iter()
        .map(|o| o.r.norm())
        .collect();
    let actual = final_residual_norms(&cam, &b, &list, &pairs, &x);
    assert_eq!(actual.len(), 3);
    assert!(actual[1].is_nan());
    assert_eq!(bits(expected), bits(actual));

    let (cam, b, mut prior, list, pairs) = fixture();
    for mode in 0..3 {
        if mode == 2 {
            prior.cov.fill(0.0);
        }
        let measure = |x: &Nominal| {
            if mode == 1 {
                vec![]
            } else {
                residuals(&cam, &b, &list, &pairs, x)
            }
        };
        let old = prior.update(measure, &UpdateParams::default());
        let new = prior.update_with_final_residuals(
            measure,
            |x| {
                assert_eq!(mode, 0, "failed updates must not evaluate final norms");
                final_residual_norms(&cam, &b, &list, &pairs, x)
            },
            &UpdateParams::default(),
        );
        assert_eq!(old, new);
        assert_eq!(old.is_none(), mode != 0);
    }
    let prior = filter(state(), 0.005, 0.002);
    let indefinite_info = |_: &Nominal| {
        vec![PixelResidual {
            r: Vector2::new(1.0, 0.0),
            j: PoseJac::repeat(1e6),
        }]
    };
    let p = UpdateParams {
        huber_k: -1.0,
        ..UpdateParams::default()
    };
    assert!(prior.update(indefinite_info, &p).is_none());
    assert!(prior
        .update_with_final_residuals(
            indefinite_info,
            |_| panic!("information Cholesky failure must not evaluate final norms"),
            &p,
        )
        .is_none());
    let calls = Cell::new(0);
    let old = prior
        .update(
            |_| {
                calls.set(calls.get() + 1);
                if calls.get() == 1 {
                    vec![PixelResidual {
                        r: Vector2::zeros(),
                        j: PoseJac::zeros(),
                    }]
                } else {
                    vec![]
                }
            },
            &UpdateParams::default(),
        )
        .unwrap();
    let new = prior
        .update_with_final_residuals(
            |_| {
                vec![PixelResidual {
                    r: Vector2::zeros(),
                    j: PoseJac::zeros(),
                }]
            },
            |_| vec![],
            &UpdateParams::default(),
        )
        .unwrap();
    assert_eq!(old, new);
    println!("{cases} camera-plane/nonfinite residual cases, normal/empty/non-PD update and empty final measurement equivalent");
}

#[test]
#[ignore = "manual synthetic whole-fit benchmark (without projection counters)"]
fn final_residual_benchmark() {
    assert!(
        option_env!("CV1TOUCH_NO_PROJECTION_COUNT").is_some(),
        "compile with CV1TOUCH_NO_PROJECTION_COUNT=1 to benchmark without operation counters"
    );
    let (cam, b, prior, list, hints) = fixture();
    let p = FitParams::default();
    let run = |new: bool, count: usize| {
        let start = Instant::now();
        for _ in 0..count {
            let result = if new {
                fit_frame(
                    black_box(&cam),
                    black_box(&b),
                    black_box(&prior),
                    black_box(&list),
                    black_box(&hints),
                    black_box(&p),
                )
            } else {
                legacy_fit_frame(
                    black_box(&cam),
                    black_box(&b),
                    black_box(&prior),
                    black_box(&list),
                    black_box(&hints),
                    black_box(&p),
                )
            };
            black_box(result.unwrap());
        }
        start.elapsed()
    };
    run(false, 2000);
    run(true, 2000);
    let count = 10000;
    let mut old = Vec::new();
    let mut new = Vec::new();
    for round in 0..12 {
        let (a, b) = if round % 2 == 0 {
            (run(false, count), run(true, count))
        } else {
            let b = run(true, count);
            (run(false, count), b)
        };
        println!(
            "round={round}, baseline_ns={:.1}, pixel_final_ns={:.1}, saving_pct={:.2}",
            a.as_nanos() as f64 / count as f64,
            b.as_nanos() as f64 / count as f64,
            100.0 * (1.0 - b.as_secs_f64() / a.as_secs_f64())
        );
        old.push(a);
        new.push(b);
    }
    old.sort();
    new.sort();
    let median = |v: &[Duration]| (v[5].as_secs_f64() + v[6].as_secs_f64()) / 2.0;
    println!(
        "median baseline_ns={:.1}, pixel_final_ns={:.1}, saving_pct={:.2}",
        median(&old) * 1e9 / count as f64,
        median(&new) * 1e9 / count as f64,
        100.0 * (1.0 - median(&new) / median(&old))
    );
}
