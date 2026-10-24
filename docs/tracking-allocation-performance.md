# Additional tracking hot-path reductions

Baseline: `5a949b8`, following the [previous sweep](tracking-followup-performance.md).
Ten independently committed production changes preserve matching order, numerical
acceptance rules, tracking policy, cadence, and all public formats:

1. Outlier rejection shares a projector per fitted state and requests only pixels,
   not unused Jacobians. Refit checks build a new projector for the new state.
2. Association allocates no LED/blob masks when no candidate survives gating.
3. Empty blob frames return from association before projecting LEDs.
4. Diagnostic residual vectors allocate once, after the first surviving projection.
5. Acquisition score match vectors reserve their candidate-count upper bound.
6. Acquisition nearest-blob vectors reserve their LED-count upper bound, lazily.
7. Predicted-pixel vectors reserve their LED-count upper bound, lazily.
8. Accepted hint vectors reserve their input-pair-count upper bound, lazily.
9. Visible projection batches share one collector with a lazy LED-count reservation.
10. Frame dispatch moves the original blob buffer to hand 1, the final consumer,
    instead of cloning it. Hand 0 still receives an independent copy when needed.
    Both ordinary pending frames and pending placement probes use this rule.

The reservations remove growth allocations, not every allocation. Empty outputs
still allocate nothing in the lazy collectors. Reserving an upper bound may retain
more capacity when most points are rejected; this is a small bounded-memory tradeoff
for ordinary CV1 LED patterns, not a universally lower-memory algorithm. For example,
one matching ray in the 24-LED acquisition fixture requests 576 bytes instead of
96 bytes, with no allocation-count benefit. No larger search budget, caching policy,
new runtime dependency, or approximation is introduced.

## Verification

`src/track/vision/tests/sweep.rs` freezes the pre-change algorithms. The acquisition
oracle in `src/track/search/perf_tests.rs` freezes the immediate pre-change score,
not the older hash-set version. Oracles share unchanged lower-level dependencies,
so these isolate the changed functions rather than two whole daemon builds.

Tests compare exact ordered outputs and filter/residual floating-point bits. They
cover empty inputs, rejected points, duplicates, singular covariance, NaN/infinite
parameters, varying gates and inlier cutoffs, and accepted/rejected fits. Existing
camera-plane, non-finite projection, successful outlier-refit, 2,000-case shifted
association, and synthetic complete-pipeline regressions run against the combined
changes. Thread-local allocation counts prove eliminated growth/mask allocations;
the final-hand test also checks pointer identity, zero allocation on transfer,
and independence of the first hand's buffer. Timing thresholds are not CI tests.

```sh
cargo test --locked --no-default-features --lib track::
RUST_TEST_THREADS=4 scripts/check.sh
cargo test --locked --release --no-default-features --lib benchmark_sweep \
  -- --ignored --nocapture --test-threads=1
```

The full check script covers formatting, Clippy, all Rust targets, intentional-abort
handling, release binaries, native regressions, driver contract, installer/runtime/
package/setup/report integration, and repeats socket suites under a long TMPDIR.
Hardware-dependent ignored tests remain explicitly opt-in. Four test threads bound
concurrent socket-fixture load, as in the previous sweep.

## Local measurements and limits

Representative synthetic measurements on an Intel Core i9-14900KS, Rust 1.86.0,
release profile, Linux, unpinned on a shared host. Each committed benchmark alternates
baseline/optimized order for twelve rounds, prepares fixtures outside timing, and
uses `black_box`. Returned values are destroyed inside timing; the complete fit
benchmark includes cloning its input pair vector in both versions. Allocation
counting is disabled while timing (the counting allocator wrapper remains installed).
Numbers are microseconds per call, minimum / median / maximum, not confidence bounds.

| Operation | Baseline | Optimized |
| --- | --- | --- |
| Complete fit | 15.553 / 15.593 / 15.679 | 14.165 / 14.230 / 14.286 |
| Acquisition score, 24 rays | 2.359 / 2.394 / 2.454 | 2.183 / 2.211 / 2.223 |
| Diagnostic residuals | 0.502 / 0.512 / 0.519 | 0.399 / 0.404 / 0.410 |
| Predicted pixels | 0.882 / 0.891 / 0.905 | 0.787 / 0.796 / 0.806 |
| Accepted hints | 2.102 / 2.109 / 2.139 | 2.028 / 2.035 / 2.054 |
| Association with nearby blobs (excluding projection) | 4.074 / 4.117 / 4.150 | 4.091 / 4.120 / 4.143 |
| Association with distant blobs (excluding projection) | 1.829 / 1.841 / 1.858 | 1.817 / 1.829 / 1.844 |

The last two ranges overlap: no association timing improvement is claimed from mask
elision alone. The allocation reduction is exact. Acquisition with zero rays showed
no improvement, and one-ray scoring had a small overlapping timing difference
(0.693 / 0.698 / 0.707 versus 0.702 / 0.705 / 0.730 microseconds). The useful score
case removes six growth allocation events (eight to two). Diagnostic, predicted-pixel,
and accepted-hint fixtures each remove two (three to one). No-candidate association
removes both mask allocations; final-hand frame transfer removes one nonempty-vector
clone. Counts include reallocations. They are workload-specific, not universal totals.

Experiments that did not meet the bar were discarded: delaying covariance inversion
slowed nearby-blob association, sorted rival early-exit did not improve representative
fixtures, and fusing velocity-fit passes regressed longer histories. None ships here.

These results establish local CPU/allocation savings, not a controlled live
motion-to-photon latency improvement. Live deployment verification checks binary
identity, service readiness and advancing sensor frames separately.
