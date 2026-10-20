// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Seeded scenes exercise the production collector, extraction, and matcher together.
//! Ordinary scenes bound each raw sample's error. Split-emitter scenes instead
//! bound recurring mode centroids by 0.4 px while allowing broader within-mode scatter.
//! Their symmetric tails and thinning keep collected mode centroids bounded, including
//! when collection clips tails. Two extracted spots may be observations of one emitter.
//! Spurious modes are persistent, separated from genuine modes, and independently
//! rejected if they counterfeit a learned pair (including genuine plus spurious modes).
//! No matcher output is used to select scenes. The lone-emitter test documents counterfeits.

use super::*;
use std::time::Instant;

struct Random(u64);
impl Random {
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / ((1u64 << 53) as f64)
    }
    fn pick(&mut self, n: usize) -> usize {
        (self.unit() * n as f64) as usize
    }
    fn disk(&mut self, radius: f64) -> Vector2<f64> {
        let angle = self.unit() * std::f64::consts::TAU;
        let r = radius * self.unit().sqrt();
        Vector2::new(r * angle.cos(), r * angle.sin())
    }
}

fn refs(xs: &[Vector2<f64>]) -> Vec<Landmark> {
    xs.iter()
        .map(|p| Landmark {
            serial: "A".into(),
            x: p.x,
            y: p.y,
            seen: 0.1,
        })
        .collect()
}

struct Scene {
    reference: Vec<Landmark>,
    samples: Vec<Vec<Vector2<f64>>>,
    truth: Vector2<f64>,
    rejected: usize,
}

fn split_scene(seed: u64, rng: &mut Random) -> Scene {
    let kind = seed % 64;
    let positive = kind == 9;
    let many = kind >= 12;
    let dx = match kind {
        8 => 1.5,
        9 => 1.1,
        _ => [0.0, 0.7, 1.1, 1.5, 1.9, 3.0, 10.0, 20.0][rng.pick(8)],
    };
    let error = if kind <= 9 || rng.pick(2) == 0 {
        0.3999999999999
    } else {
        0.31 + rng.unit() * 0.08
    };
    let sigma = if kind <= 9 {
        0.4
    } else {
        [0.12, 0.3, 0.4][rng.pick(3)]
    };
    let mut xs = if positive {
        vec![0.0, 100.0]
    } else {
        vec![0.0, 1.6]
    };
    if many {
        xs.extend([10.0, 11.6]);
    }
    let reference: Vec<_> = xs.iter().map(|x| Vector2::new(*x, 300.25)).collect();
    let mut modes = Vec::new();
    for base in if many { &[0.0, 10.0][..] } else { &[0.0][..] } {
        for sign in [-1.0, 1.0] {
            let hits = if kind <= 9 {
                101
            } else {
                [0, 10, 21, 51, 101][rng.pick(5)]
            };
            modes.push((base + dx + sign * error, sigma, hits));
        }
    }
    if positive {
        modes.push((100.0 + dx + error, 0.12, 101));
    }
    // Central mass exceeds each tail; both tails contain the same number of hits.
    // Collection is symmetric in y about every reference, so it retains both tails
    // or neither. Every extracted mode still has y at the true emitter's centre.
    let samples_by_mode: Vec<Vec<_>> = modes
        .into_iter()
        .map(|(x, sigma, hits)| {
            if hits == 0 {
                return Vec::new();
            }
            let tail = (hits as f64 * 0.32).floor() as usize;
            let amplitude = sigma * (hits as f64 / (2 * tail) as f64).sqrt();
            (0..hits)
                .map(|k| {
                    Vector2::new(
                        x,
                        300.25
                            + if k < tail {
                                -amplitude
                            } else if k < 2 * tail {
                                amplitude
                            } else {
                                0.0
                            },
                    )
                })
                .collect()
        })
        .collect();
    let mut samples = Vec::new();
    // At most one observed mode in each frame: the source changes mode over time.
    for k in 0..101 {
        for mode in &samples_by_mode {
            if let Some(p) = mode.get(k) {
                samples.push(vec![*p]);
            }
        }
    }
    if samples.is_empty() {
        samples.push(Vec::new());
    }
    Scene {
        reference: refs(&reference),
        samples,
        truth: Vector2::new(dx, 0.0),
        rejected: 0,
    }
}

fn scene(seed: u64) -> Scene {
    let mut rng = Random(seed.wrapping_add(1).wrapping_mul(0x9e3779b97f4a7c15));
    if (8..=15).contains(&(seed % 64)) {
        return split_scene(seed, &mut rng);
    }
    // Fixed seed classes preserve the two clipping regressions in the property suite.
    if seed % 64 < 2 {
        let clipped = seed % 64 == 1;
        let xs = [
            100.0,
            101.8,
            if clipped { 104.38 } else { 104.3 },
            110.0,
            111.8,
        ];
        let reference: Vec<_> = xs.iter().map(|x| Vector2::new(*x, 300.0)).collect();
        let samples = (0..100)
            .map(|k| {
                vec![
                    Vector2::new(110.0, 300.0),
                    Vector2::new(111.8, 300.0),
                    Vector2::new(
                        if !clipped {
                            114.3
                        } else if k < 10 {
                            113.84
                        } else {
                            114.44
                        },
                        300.0,
                    ),
                ]
            })
            .collect();
        // The asymmetric third mode in the exact regression has a 0.54 px tail, but
        // is below min_hits after clipping and never supports a published fit.
        return Scene {
            reference: refs(&reference),
            samples,
            truth: Vector2::new(10.0, 0.0),
            rejected: 0,
        };
    }
    // Numerical-boundary families include the review witnesses, errors exactly at
    // 0.4 and one ULP below it, and coordinates far beyond a camera's normal range.
    // They are chosen before matching, just like the random physical scenes below.
    if (2..=7).contains(&(seed % 64)) {
        let kind = seed % 64;
        let (xs, points, truth) = if kind == 2 {
            (
                vec![100.39999999999999, 105.60000000000001, 1124.0, 1130.0],
                vec![1124.0, 1130.0],
                1024.0,
            )
        } else if kind == 3 {
            (
                vec![100.0000000000001, 102.9999999999999, 1200.4, 1202.6],
                vec![1200.4, 1202.6],
                1100.0,
            )
        } else if kind == 5 {
            let large = 1_073_741_824.0;
            (
                vec![
                    100.39999999999999,
                    105.60000000000001,
                    large + 100.0,
                    large + 106.0,
                ],
                vec![large + 100.0, large + 106.0],
                large,
            )
        } else {
            let base = [0.0, 1024.0, 1_048_576.0, 1_000_000_000.0][rng.pick(4)];
            if kind == 6 {
                // A common +0.4 centroid bias is measurable and reaches the bound,
                // rather than exercising only rejection of alternative mappings.
                let xs = vec![base + 100.0, base + 107.0, base + 117.0];
                let points = xs.iter().map(|x| x + 1.5).collect();
                (xs, points, 1.1)
            } else {
                let error = match rng.pick(4) {
                    0 => 0.4,
                    1 => f64::from_bits(0.4_f64.to_bits() - 1),
                    2 => 0.4 * (1.0 - rng.unit() * 1e-9),
                    _ => rng.unit() * 0.4,
                };
                let points = vec![base + 1124.0, base + 1130.0];
                let original = [points[0] - 1024.0, points[1] - 1024.0];
                let mut first = original[0] + error;
                let mut second = original[1] - error;
                // Quantizing a boundary draw at large coordinates can round outside
                // the model's bound. Snap inward by one ULP before matching, without
                // consulting matcher output. Nearby subtraction is exact here.
                if first - original[0] > CENTROID_ERROR_PX {
                    first = f64::from_bits(first.to_bits() - 1);
                }
                if original[1] - second > CENTROID_ERROR_PX {
                    second = f64::from_bits(second.to_bits() + 1);
                }
                assert!(
                    first - original[0] <= CENTROID_ERROR_PX
                        && original[1] - second <= CENTROID_ERROR_PX
                );
                let xs = vec![first, second, points[0], points[1]];
                (xs, points, 1024.0)
            }
        };
        let reference: Vec<_> = xs.iter().map(|x| Vector2::new(*x, 300.0)).collect();
        let samples = vec![points.iter().map(|x| Vector2::new(*x, 300.0)).collect(); 100];
        return Scene {
            reference: refs(&reference),
            samples,
            truth: Vector2::new(truth, 0.0),
            rejected: 0,
        };
    }
    let group_count = 1 + rng.pick(3);
    let shape = [
        Vector2::new(0.0, 0.0),
        Vector2::new(1.6 + rng.unit() * 2.0, 0.0),
        Vector2::new(4.0 + rng.unit() * 2.0, 1.6 + rng.unit() * 2.0),
    ];
    let mut reference = Vec::new();
    let mut base_x = 100.0;
    for _ in 0..group_count {
        let base = Vector2::new(base_x, 300.0);
        base_x += 8.0 + rng.unit() * 12.0;
        let n = 2 + rng.pick(2);
        for p in shape.iter().take(n) {
            reference.push(base + p);
        }
    }
    // Include both ordinary separated stations and repeated nearby groups.
    if rng.pick(2) == 0 {
        reference.push(Vector2::new(500.0, 700.0));
    }
    assert!(reference
        .iter()
        .enumerate()
        .all(|(i, a)| reference.iter().skip(i + 1).all(|b| (a - b).norm() >= 1.6)));
    let angle = if rng.pick(2) == 0 {
        0.0
    } else {
        rng.unit() * std::f64::consts::TAU
    };
    let distance = if seed % 4 == 2 {
        rng.unit() * 1.9
    } else {
        match rng.pick(4) {
            0 => rng.unit() * 1.9,
            1 => 1.9 + rng.unit() * 1.0,
            _ => rng.unit() * 20.0,
        }
    };
    let truth = Vector2::new(angle.cos() * distance, angle.sin() * distance);
    let centres: Vec<_> = reference
        .iter()
        .map(|r| r + truth + rng.disk(if seed % 4 == 2 { 0.06 } else { 0.18 }))
        .collect();
    let hits: Vec<_> = centres
        .iter()
        .map(|_| {
            if seed % 4 == 2 {
                100
            } else {
                [0, 10, 25, 60, 100][rng.pick(5)]
            }
        })
        .collect();
    let mut spurious = Vec::new();
    let mut rejected = 0;
    for _ in 0..rng.pick(4) {
        let p = if rng.pick(4) == 0 {
            reference[rng.pick(reference.len())] + rng.disk(2.6)
        } else {
            Vector2::new(rng.unit() * 1280.0, rng.unit() * 960.0)
        };
        let separated = centres
            .iter()
            .chain(&spurious)
            .all(|q| (p - q).norm() > 1.6);
        let counterfeit = centres.iter().chain(&spurious).any(|q| {
            reference.iter().enumerate().any(|(i, a)| {
                reference.iter().enumerate().any(|(j, b)| {
                    i != j && ((p - q) - (a - b)).norm() <= 2.0 * CENTROID_ERROR_PX + 0.26
                })
            })
        });
        if !separated || counterfeit {
            rejected += 1;
        } else {
            spurious.push(p);
        }
    }
    let samples = (0..100)
        .map(|k| {
            let mut frame = Vec::new();
            for (p, hits) in centres.iter().zip(&hits) {
                if k < *hits {
                    frame.push(p + rng.disk(0.18));
                }
            }
            for p in &spurious {
                frame.push(*p + rng.disk(0.08));
            }
            frame
        })
        .collect();
    Scene {
        reference: refs(&reference),
        samples,
        truth,
        rejected,
    }
}

#[test]
fn seeded_scenes_never_publish_a_false_displacement() {
    let count = std::env::var("CV1TOUCH_LANDMARK_PROPERTY_SCENES")
        .ok()
        .map(|s| s.parse::<usize>().expect("scene count"))
        .unwrap_or(256);
    assert!((2..=200_000).contains(&count));
    let mut unknown = 0;
    let mut rejected = 0;
    let mut violations = 0;
    let mut worst = std::time::Duration::ZERO;
    for seed in 0..count as u64 {
        let Scene {
            reference,
            samples,
            truth,
            rejected: excluded,
        } = scene(seed);
        rejected += excluded;
        let window_ns = samples.len() as u64 * 10;
        let mut monitor = Monitor::new(reference, window_ns);
        for (k, frame) in samples.iter().enumerate() {
            assert!(monitor.frame("A", 1 + k as u64 * 10, frame, 20).is_none());
        }
        let start = Instant::now();
        let out = monitor.frame("A", 1 + window_ns, &[], 20).unwrap();
        worst = worst.max(start.elapsed());
        if out.is_empty() {
            unknown += 1;
        }
        for shift in out {
            if (shift.shift - truth).norm() > CENTROID_ERROR_PX + 1e-8 {
                violations += 1;
                if violations <= 20 {
                    eprintln!("seed {seed}, truth {truth:?}, false shift {shift:?}");
                }
            }
        }
    }
    println!("property scenes={count} unknown={unknown} noncounterfeit_rejections={rejected} violations={violations} worst_ms={:.6}",worst.as_secs_f64()*1000.0);
    assert_eq!(
        violations, 0,
        "published shifts must satisfy the centroid-error bound"
    );
}
