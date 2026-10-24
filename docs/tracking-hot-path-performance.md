# Tracking hot-path reductions

This batch preserves tracking decisions while removing redundant work. The baseline
is `052f5ce`; its already-merged count-only reacquisition check is not duplicated.
No formats, thresholds, cadence, tracking policy, or driver behavior change.

## Changes and limits

- Clock observations retain a new open-bin minimum without refitting unchanged
  closed bins once the clock has warmed up. Startup and rollover still refit.
- Position search moves accepted pair buffers rather than cloning them.
- Historical state lookup binary-searches the first IMU sample after its snapshot.
  This can be slower than a scan for matches near the start of short histories.
- Gravity checks borrow the sorted IMU window; history rebuilds stream paired IMU
  samples/stillness flags. The rebuild cutoff remains the original exposure time,
  not the mutable replay timestamp. Optical measurements still win equal timestamps.
- Candidate selection reduces directly to the cheapest fit, retaining candidate
  order and `total_cmp` tie semantics.
- Peer-camera checks stop once their newest eligible update is already stale.
- Unexplained-blob bounds are reduced directly from projected points in the same
  order, including the original first-point operation.
- Full association reports reuse their projected list for matching. Acquisition
  costs instead request only visible/nearby counts, without allocating that list
  or running matching. These two optimizations coexist: full reports do not route
  through the count-only helper and then repeat projection.
- Inlier filtering retains survivors in the owned pair buffer. Rejection fractions
  and the decision to refit use the original pair count.

Moving/retaining owned pair buffers can retain more spare capacity than creating
new buffers. This is an allocation/copying trade-off, not a guarantee of less
retained memory. Timing improvements are workload-dependent. Several isolated
candidates removed allocations without a convincing full-function timing change.
The attempted round-robin modulo rewrite is **not** included: optimized machine
code already performed increment-and-wrap.

## Durable verification

Tests keep independent pre-batch algorithms, rather than deriving the oracle from
optimized implementations:

- `src/track/clock.rs`: original observation/refit behavior, including warmup,
  window sizes, bin minima, rollover, and resets.
- `src/track/tracker/tests/hot_paths.rs`: timestamp boundaries and wrapped queues;
  original-versus-new state/filter bits; sorted camera-update subsets; rebuilds
  with optical/IMU interleaving, forced stillness, rejected measurements and empty
  replay ranges; acquisition cost bits and complete selected fits.
- `src/track/vision/tests/hot_paths.rs`: full reports, ordered matches, count-only
  results, projected bounds, and fit results including successful outlier refits.
  Empty inputs, non-finite thresholds/coordinates and overallocated buffers are
  exercised. Filter/covariance/residual comparisons use floating-point bits.

The existing thread-local counting allocator measures only the calling test's
explicit interval. Allocation assertions check savings/no increases; they do not
assert timing. Existing synthetic late-frame, reacquisition, and vision tests also
run against the combined implementation.

```sh
cargo test --locked --no-default-features --lib hot_paths -- --test-threads=1
cargo test --locked --no-default-features --lib skipped_refits_match_original
cargo test --locked --all-targets
```

## Reproducing an offline timing comparison

```sh
cargo test --locked --release --lib benchmark_tracking_hot_paths \
  -- --ignored --nocapture --test-threads=1
```

The ignored benchmark compares complete acquisition-cost and history-rebuild
functions with their pre-batch versions, using a synthetic Touch controller and
shuffled delayed frames. It alternates execution order over twelve repetitions,
uses `black_box`, prepares fixtures/owned inputs outside timing, and reports
minimum/median/maximum microseconds per call. Acquisition allocation counts are
reported separately; counting is disabled during timing (the allocator wrapper
remains installed). Rebuild destruction is outside timing. Unchanged dependencies
of the copied oracles use the current implementation, so this isolates the named
functions rather than comparing two full daemon builds.

Record compiler, build profile, CPU and variability with any published numbers.
Exploratory checks used Rust 1.86.0 and an Intel Core i9-14900KS; do not generalize
those synthetic timings to other machines or to motion-to-photon latency. There
is no claimed combined live latency improvement or statistical confidence bound.
