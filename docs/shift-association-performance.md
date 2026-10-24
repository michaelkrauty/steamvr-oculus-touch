# Unique-blob scan in shifted association

`fit_frame` calls `associate_shifted` when normal association plus engine hints
find too few pairs. This recovery fallback scores candidate shifts, placing each
visible LED against all observed blobs for each candidate. It is not necessarily
called on every camera frame.

Previously each placement collected distances below `2 * snap_px` and sorted
them, only to accept a blob when exactly one distance was strictly below
`snap_px`. Scanning until the second match implements that same condition without
the temporary vector or sorting. The existing duplicate-use check and LED order
remain unchanged. There is no new cache, retained memory, dependency, or tuning.

## Correctness and reproduction

```sh
cargo test --locked --no-default-features --lib track::vision
cargo test --locked --no-default-features --lib bench_shift_association -- --ignored --nocapture --test-threads=1
```

The test-only oracle retains the entire old implementation, including its sort.
A deterministic differential test compares complete returned pairs for 2,000
cases with missing blobs, jitter, duplicate blobs, clutter, and varied thresholds
(including zero, negative, NaN, and infinity). Additional cases exercise duplicated
constellations, reversed input order, non-finite blobs, and offsets at/around the
strict gate. The existing re-lock test checks the subsequent filter correction.
The fixture also requires a successful association, so agreement cannot pass
solely through both versions rejecting everything.

For nonnegative finite distances and positive finite `snap_px`, the original
first/second sorted-distance checks mean exactly one distance is below the gate.
Distances between the gate and twice the gate cannot affect that decision. With
nonpositive or NaN gates both versions reject; with an infinite gate both count
finite distances. NaN distances fail either filter. Distances are still computed
with the same `norm()` operation; this change does not substitute squared norms
or change threshold rounding.

## Local CPU measurement

Measured on Linux x86-64, Intel Core i9-14900KS, Rust 1.86.0, using the commands
above and the repository's optimized test profile (`opt-level = 2`). This is a
synthetic 12-blob re-lock fixture yielding the same 10 pairs. After 1,000 warm-up
calls per implementation, the benchmark uses seven batches of 5,000 calls each,
alternates baseline/optimized order, and uses `black_box` for inputs and results.
No allocation counting is enabled during timing; the library test harness retains
its existing dormant thread-local allocation hook.

| Whole association, batch-average time | Baseline | Unique scan |
| --- | ---: | ---: |
| Median over seven batches | 37.642 µs | 27.894 µs |
| Range across batches | 36.753–43.316 µs | 27.280–30.094 µs |

This fixture took about 26% less CPU wall time per call (9.7 µs at the medians).
The ranges describe run-to-run variation, not confidence intervals or per-call
latency percentiles. The machine was not isolated or frequency-pinned. This does
not measure live fallback frequency, total tracking CPU, or motion-to-display
latency. Normal association, candidate-shift ordering, and final matching policy
are unchanged.
