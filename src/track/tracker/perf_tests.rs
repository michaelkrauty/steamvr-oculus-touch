// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
use super::stillness_tests::counted;
use super::*;

#[test]
fn two_costs_match_sorted_baseline_without_allocating() {
    let values = [
        f64::NEG_INFINITY,
        -1.0,
        -0.0,
        0.0,
        1.0,
        f64::INFINITY,
        f64::NAN,
        -f64::NAN,
        f64::from_bits(0x7ff8000000000001),
    ];
    for a in values {
        for b in values {
            for c in values {
                for input in [&[][..], &[a], &[a, b], &[a, b, c]] {
                    let (actual, counts) = counted(|| {
                        let mut result = [None, None];
                        for &c in input {
                            retain_two_costs(&mut result, c);
                        }
                        result.map(|v| v.map(f64::to_bits))
                    });
                    let mut sorted = input.to_vec();
                    sorted.sort_by(f64::total_cmp);
                    assert_eq!(
                        actual,
                        [
                            sorted.first().copied().map(f64::to_bits),
                            sorted.get(1).copied().map(f64::to_bits)
                        ]
                    );
                    assert_eq!(counts.0, 0);
                }
            }
        }
    }
}
