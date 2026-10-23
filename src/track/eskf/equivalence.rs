// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Bitwise oracle and opt-in release timing for the iterated covariance update.

use super::*;
use crate::calib::{camera::CameraModel, capture::Led};
use crate::track::vision::{self, Body, Camera};
use nalgebra::Isometry3;
use std::cell::RefCell;

// Frozen before deferring the posterior inverse. Keep its arithmetic independent of update.
impl Filter {
    fn original_update<F>(&self, measure: F, p: &UpdateParams) -> Option<Updated>
    where
        F: Fn(&Nominal) -> Vec<PixelResidual>,
    {
        let info0 = self.cov.cholesky()?.inverse();
        let var = p.sigma_px * p.sigma_px;
        let mut e = ErrVec::zeros();
        let mut x = self.x;
        let mut cov = self.cov;
        for _ in 0..p.iterations.max(1) {
            let obs = measure(&x);
            if obs.is_empty() {
                return None;
            }
            let mut info = info0;
            let mut b = ErrVec::zeros();
            let e6 = e.fixed_rows::<6>(0).into_owned();
            for o in &obs {
                let norm = o.r.norm() / p.sigma_px;
                let w = if norm <= p.huber_k {
                    1.0
                } else {
                    p.huber_k / norm
                } / var;
                let jt = o.j.transpose();
                let mut block = info.fixed_view_mut::<6, 6>(0, 0);
                block += jt * o.j * w;
                let mut rows = b.fixed_rows_mut::<6>(0);
                rows += jt * (o.r + o.j * e6) * w;
            }
            let chol = info.cholesky()?;
            let next = chol.solve(&b);
            cov = chol.inverse();
            let step = (next - e).norm();
            e = next;
            x = self.x.plus(&e);
            if step < 1e-7 {
                break;
            }
        }
        let residuals_px = measure(&x).iter().map(|o| o.r.norm()).collect();
        Some(Updated {
            filter: Filter {
                x,
                cov: (cov + cov.transpose()) * 0.5,
            },
            residuals_px,
        })
    }
}

struct Frame {
    filter: Filter,
    camera: Camera,
    body: Body,
    pixels: Vec<Vector2<f64>>,
}

impl Frame {
    fn new(seed: usize, outlier: bool) -> Self {
        let camera = Camera {
            model: CameraModel {
                width: 1280,
                height: 960,
                fx: 600.0,
                fy: 600.0,
                cx: 640.0,
                cy: 480.0,
                k: [0.01, -0.001, 0.0, 0.0],
            },
            world_from_cam: Isometry3::identity(),
        };
        let body = Body {
            imu_pos: Vector3::new(0.01, -0.02, 0.005),
            leds: (0..12)
                .map(|i| {
                    let a = i as f64 * std::f64::consts::TAU / 12.0;
                    Led {
                        pos: Vector3::new(0.045 * a.cos(), 0.045 * a.sin(), 0.01 * (2.0 * a).sin()),
                        dir: -Vector3::z(),
                    }
                })
                .collect(),
        };
        let t = seed as f64 * 0.07;
        let truth = Nominal {
            q: UnitQuaternion::from_scaled_axis(Vector3::new(0.1 + t, -0.2, 0.03)),
            p: Vector3::new(0.1, 0.2, 1.2 + t),
            v: Vector3::new(0.2, -0.03, 0.1),
            bg: Vector3::new(0.001, -0.002, 0.003),
            ba: Vector3::new(0.02, 0.01, -0.03),
        };
        let mut pixels: Vec<_> = (0..body.leds.len())
            .map(|i| vision::project(&camera, &body, &truth, i).unwrap().uv)
            .collect();
        if outlier {
            pixels[0] += Vector2::new(8.0, -6.0);
        }
        let e = ErrVec::from_iterator((0..15).map(|i| {
            if i < 6 {
                (i as f64 + 1.0) * 0.0004
            } else {
                0.0
            }
        }));
        // Dense positive-definite 15x15 prior: pose observations also update velocity and biases.
        let l = Cov::from_fn(|r, c| match r.cmp(&c) {
            std::cmp::Ordering::Equal => [0.015, 0.004, 0.03, 0.002, 0.02][r / 3],
            std::cmp::Ordering::Greater => 0.0003 * ((r + c + seed) as f64).sin(),
            std::cmp::Ordering::Less => 0.0,
        });
        Self {
            filter: Filter {
                x: truth.plus(&e),
                cov: l * l.transpose(),
            },
            camera,
            body,
            pixels,
        }
    }

    fn measure(&self, x: &Nominal) -> Vec<PixelResidual> {
        self.pixels
            .iter()
            .enumerate()
            .map(|(i, uv)| {
                let p = vision::project(&self.camera, &self.body, x, i).unwrap();
                PixelResidual {
                    r: uv - p.uv,
                    j: p.j,
                }
            })
            .collect()
    }
}

fn state_bits(x: &Nominal) -> Vec<u64> {
    x.q.coords
        .iter()
        .chain(x.p.iter())
        .chain(x.v.iter())
        .chain(x.bg.iter())
        .chain(x.ba.iter())
        .map(|v| v.to_bits())
        .collect()
}

fn updated_bits(out: &Option<Updated>) -> Option<Vec<u64>> {
    out.as_ref().map(|u| {
        let mut bits = state_bits(&u.filter.x);
        bits.extend(u.filter.cov.iter().map(|v| v.to_bits()));
        bits.extend(u.residuals_px.iter().map(|v| v.to_bits()));
        bits
    })
}

fn compare(
    f: &Filter,
    p: &UpdateParams,
    measure: impl Fn(usize, &Nominal) -> Vec<PixelResidual>,
) -> (Option<Updated>, usize) {
    let before = (state_bits(&f.x), f.cov.map(f64::to_bits));
    let run = |original: bool| {
        let calls = RefCell::new(Vec::new());
        let callback = |x: &Nominal| {
            let i = calls.borrow().len();
            calls.borrow_mut().push(state_bits(x));
            measure(i, x)
        };
        let out = if original {
            f.original_update(callback, p)
        } else {
            f.update(callback, p)
        };
        (out, calls.into_inner())
    };
    let (expected, expected_calls) = run(true);
    let (actual, calls) = run(false);
    assert_eq!(
        updated_bits(&actual),
        updated_bits(&expected),
        "returned state/covariance/residual bits"
    );
    assert_eq!(
        calls, expected_calls,
        "measurement order, input states and count"
    );
    assert_eq!(
        (state_bits(&f.x), f.cov.map(f64::to_bits)),
        before,
        "prior unchanged"
    );
    (actual, calls.len())
}

#[test]
fn iterated_update_matches_original_bits() {
    for seed in 0..16 {
        for outlier in [false, true] {
            let frame = Frame::new(seed, outlier);
            for iterations in [0, 1, UpdateParams::default().iterations, 10] {
                let p = UpdateParams {
                    iterations,
                    ..UpdateParams::default()
                };
                let (out, calls) = compare(&frame.filter, &p, |_, x| frame.measure(x));
                let out = out.unwrap();
                assert_eq!(out.residuals_px.len(), 12);
                assert!((2..=iterations.max(1) + 1).contains(&calls));
                if iterations <= 1 {
                    assert_eq!(calls, 2);
                }
                // Covariance coupling exercises all nominal components, not just pose.
                assert_ne!(
                    state_bits(&out.filter.x)[7..],
                    state_bits(&frame.filter.x)[7..]
                );
            }
        }
    }
}

fn observation(r: f64, j: f64) -> Vec<PixelResidual> {
    let mut jac = PoseJac::zeros();
    jac[(0, 0)] = j;
    vec![PixelResidual {
        r: Vector2::new(r, 0.0),
        j: jac,
    }]
}

#[test]
fn update_exits_and_failures_match_original() {
    let frame = Frame::new(0, false);
    let f = frame.filter;
    let p = UpdateParams {
        iterations: 10,
        ..UpdateParams::default()
    };
    let (out, calls) = compare(&f, &p, |_, _| observation(0.0, 1.0));
    assert!(out.is_some());
    assert_eq!(
        calls, 2,
        "converges on the first iteration plus final residual evaluation"
    );
    for iterations in [0, 1, 4, 10] {
        let p = UpdateParams { iterations, ..p };
        let (out, calls) = compare(&f, &p, |i, _| {
            observation(if i % 2 == 0 { 1.0 } else { -1.0 }, 100.0)
        });
        assert!(out.is_some());
        assert_eq!(
            calls,
            iterations.max(1) + 1,
            "exhausted cap plus final residual evaluation"
        );
    }
    let (out, calls) = compare(
        &Filter {
            cov: Cov::zeros(),
            ..f
        },
        &p,
        |_, _| panic!("invalid prior must not measure"),
    );
    assert!(out.is_none());
    assert_eq!(calls, 0);
    for empty_at in [0, 1] {
        let (out, calls) = compare(&f, &p, |i, _| {
            if i == empty_at {
                vec![]
            } else {
                observation(1.0, 100.0)
            }
        });
        assert!(out.is_none());
        assert_eq!(calls, empty_at + 1);
    }
    let (out, calls) = compare(&f, &UpdateParams { iterations: 1, ..p }, |i, _| {
        if i == 1 {
            vec![]
        } else {
            observation(1.0, 100.0)
        }
    });
    assert!(out.unwrap().residuals_px.is_empty());
    assert_eq!(calls, 2, "empty final residual evaluation remains accepted");
    // A deliberately invalid negative Huber weight makes the second information matrix indefinite.
    let (out, calls) = compare(
        &f,
        &UpdateParams {
            sigma_px: 1.0,
            huber_k: -1.0,
            ..p
        },
        |i, _| observation(1.0, if i == 0 { 0.01 } else { 1e6 }),
    );
    assert!(out.is_none());
    assert_eq!(
        calls, 2,
        "later factorization fails after an accepted first factor"
    );
}

/// Run with --release --ignored --exact track::eskf::equivalence::covariance_update_timing --nocapture.
/// Outputs paired raw ns/update samples, alternating order, and the actual iteration histogram.
#[test]
#[ignore = "opt-in release timing; no hardware"]
fn covariance_update_timing() {
    use std::{collections::BTreeMap, hint::black_box, time::Instant};
    let mode = std::env::var("CV1TOUCH_COVARIANCE_BENCH").unwrap_or_else(|_| "both".into());
    let frames: Vec<_> = (0..16).map(|s| Frame::new(s, s % 2 == 0)).collect();
    for (name, iterations, converged) in [
        ("zero", 0, false),
        ("one", 1, false),
        ("default", 4, false),
        ("capped", 10, false),
        ("converged", 4, true),
    ] {
        let p = UpdateParams {
            iterations,
            ..UpdateParams::default()
        };
        let mut histogram = BTreeMap::new();
        for f in &frames {
            let (_, calls) = compare(&f.filter, &p, |_, x| {
                if converged {
                    observation(0.0, 1.0)
                } else {
                    f.measure(x)
                }
            });
            *histogram.entry(calls - 1).or_insert(0usize) += 1;
        }
        println!("iterations,{name},{histogram:?}");
        let time = |original: bool, n: usize| {
            let start = Instant::now();
            for i in 0..n {
                let f = black_box(&frames[i % frames.len()]);
                let measure = |x: &Nominal| {
                    if converged {
                        observation(0.0, 1.0)
                    } else {
                        f.measure(x)
                    }
                };
                let out = if original {
                    f.filter.original_update(measure, black_box(&p))
                } else {
                    f.filter.update(measure, black_box(&p))
                };
                black_box(out);
            }
            start.elapsed().as_nanos() as f64 / n as f64
        };
        time(true, 256);
        time(false, 256);
        for sample in 0..9 {
            for original in if sample % 2 == 0 {
                [true, false]
            } else {
                [false, true]
            } {
                if (mode == "baseline" && !original) || (mode == "new" && original) {
                    continue;
                }
                let ns = time(original, 4096);
                println!(
                    "timing,{name},{sample},{},{ns:.3}",
                    if original { "baseline" } else { "new" }
                );
            }
        }
    }
}
