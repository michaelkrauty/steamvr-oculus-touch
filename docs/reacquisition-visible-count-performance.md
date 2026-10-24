# Visible-LED count in reacquisition

`Tracker::search_kept` calls `explains_frame` for each successfully fitted
remembered-orientation acquisition candidate. The predicate previously called
`association_report` but used only the visible-LED count, discarding the nearby
blob count and full association result. It now calls the existing `visible_leds`
helper. Both use identical projections and visibility predicates. No threshold,
acceptance rule, state, cache, dependency, or search cadence changes.

This is a radio-thread reacquisition optimization, not a normal successful-frame
or IMU-path optimization. Searches are limited by default to once per 60 ms per
camera and skip frames with more than 24 blobs. The measured saving applies per
candidate checked, not to every incoming frame.

## Correctness and reproduction

```sh
cargo test --locked --no-default-features --release --lib visible_count_matches_association_report
cargo test --locked --no-default-features --release --lib explains_frame_takes_only_a_close_nearly_whole_fit
cargo test --locked --no-default-features --release --lib bench_explains_frame_visible_count -- --ignored --nocapture --test-threads=1
```

The test-only oracle retains the entire original `explains_frame` implementation,
including the discarded tuple fields, so the compiler may eliminate any unused
work it can normally eliminate. The existing acceptance regression tests both
versions on a valid fit, excess unexplained blobs, too few pairs, and pixel errors.
A differential test compares the count helpers over 12,000 cases: eight synthetic
worlds, 100 poses, three cameras, and five facing thresholds. The benchmark also
checks whole-predicate equality for 273 synthetic fits and variants rejected for
excess unexplained blobs. All geometry is generated from public test fixtures.

## Local CPU measurement

Measured against baseline `cdccb78` on Linux x86-64, Intel Core i9-14900KS,
Rust 1.86.0, using Cargo's optimized release profile without native-engine features.
The benchmark times the complete predicate, including its unchanged residual/RMS
calculation. It uses 273 precomputed fits, 200 repetitions per round, nine rounds,
alternating baseline/optimized order, with `black_box` on inputs and results.
Fixture construction and equivalence assertions are outside the timed region.

| Whole predicate, round-average time per call | Baseline | Count-only |
| --- | ---: | ---: |
| Median over nine rounds | 5.557 µs | 2.624 µs |
| Range across rounds | 5.411–5.799 µs | 2.558–2.681 µs |

The fixture took approximately 53% less time in this predicate, saving 2.93 µs
per candidate at the medians. Ranges are variation among round averages, not
confidence intervals or per-call latency percentiles. The machine was not
isolated or frequency-pinned, and no separate warm-up phase was used. These
measurements do not establish live fallback frequency, whole-search speedup,
total tracking CPU savings, or end-to-end motion-to-display latency.
