# Tracking hot-path follow-up

Baseline: `fc842f4` (the preceding [tracking sweep](tracking-hot-path-performance.md)).
This follow-up changes no tracking thresholds, association tie rules, search
budgets, recording formats, or shared-memory ABI. It contains ten small production
changes:

1. Build camera/body rotation matrices once per LED batch instead of once per LED.
   Per-LED multiplication order is unchanged; rotations are not pre-multiplied.
2. Visibility, predicted-pixel and footprint queries calculate pixels/facing without
   unused Jacobians. The original central-difference camera-plane rejection is
   retained, including its behavior for non-finite coordinates.
3. Shift hypotheses reuse their placement vector instead of allocating one for
   every hypothesis. Scratch allocates lazily, so an empty search does not pay for it.
4. Acquisition scoring uses its short output to detect already-used blobs. Each
   LED already contributes at most one nearest-blob candidate, so the separate LED
   hash set is redundant; neither hash set is needed.
5. Alias ambiguity checks retain only the two cheapest costs, without allocating
   and sorting a cost vector. `total_cmp`, signed-zero and NaN ordering are retained;
   choosing a replacement fit still compares against the original fit's cost.
6. Shift association selects the first highest-scoring hypothesis and the highest
   eligible rival directly, instead of stable-sorting every hypothesis. First-tie
   selection and the rival-distance predicate are unchanged.
7. Ray fitting consumes matched ray/offset pairs directly instead of collecting
   an intermediate vector. Missing rays and singular fits still return no result.
8. Camera agreement diagnostics keep pixel residuals, not unused Jacobians.
9. Hint gating checks facing and pixel distance before forming/inverting projected
   covariance. Valid-pair arithmetic and all acceptance predicates are unchanged.
10. Optical-update residual vectors reserve their pair-count upper bound after the
    first successful projection, avoiding growth reallocations. No successful
    projections still means no allocation.

## Verification

The new tests are in `src/track/vision/tests/followup.rs`,
`src/track/search/perf_tests.rs`, and `src/track/tracker/perf_tests.rs`.
They retain pre-change algorithms, rather than using the optimized implementation
as its own oracle. The projection oracle is from the baseline; later oracles use
unchanged dependencies at their immediate pre-optimization state, isolating each
change rather than timing two complete daemon builds.

Coverage includes bitwise projection/Jacobian/residual comparisons, camera-plane
boundaries, NaNs/infinities, varied orientations, empty and rejected outputs,
duplicate blobs, hypothesis ties, covariance failures, missing rays, singular
fits, and exact ordered match lists. Thread-local allocation assertions verify
reductions and absence of scratch allocations when no hypothesis can be tried.
The pixel-only non-finite oracle compares NaN classification rather than NaN sign
or payload: Rust arithmetic does not guarantee those bits, and Windows stable CI
for PR #84 produced opposite-sign NaNs from the two equivalent paths. Every non-NaN
component still compares bitwise, including signed zero and infinities; `Some`/`None`
projection results must also match. A regression includes both exact CI bit patterns,
alternate payloads, and checks that non-NaN differences remain distinguishable.

The existing 2,000-case shifted-association differential test and synthetic
tracking/acquisition/camera-agreement tests run against the combined code too.

```sh
cargo test --locked --no-default-features --lib track::
scripts/check.sh
```

The latter checks formatting, Clippy, all Rust targets, expected-abort handling,
release binaries, native regressions, the driver contract, installer and CLI
integration, and repeats socket-related checks with a long temporary directory.
Hardware-dependent ignored tests are not substituted with passing stubs.

## Offline measurements

```sh
cargo test --locked --release --no-default-features --lib benchmark_followup \
  -- --ignored --nocapture --test-threads=1
```

The committed ignored benchmarks alternate baseline/optimized order over twelve
rounds, use `black_box`, and construct fixtures outside timing. Returned vectors
are destroyed inside timing for both versions. The thread-local counting allocator
is installed but counting is disabled during timing. Timings report
minimum/median/maximum, not a confidence interval. A separate complete
`predicted_pixels` benchmark allows the compiler to eliminate unused baseline
Jacobian fields too; comparing a full-Jacobian result against a pixel-only result
alone would overstate that caller's improvement.

A representative run on an Intel Core i9-14900KS, Rust 1.86.0, release profile,
with the test process pinned to logical CPU 16, produced the following microseconds
per call (per LED batch for projection). These are synthetic local measurements,
not a statistical guarantee; the host was not otherwise isolated. To reproduce
CPU pinning, choose an allowed CPU on your host and prefix the command with
`taskset -c CPU` on Linux.

| Operation | Baseline min / median / max (µs) | Optimized min / median / max (µs) |
| --- | --- | --- |
| Acquisition score, 24-point pattern | 3.753 / 3.859 / 4.308 | 2.362 / 2.448 / 2.989 |
| Complete `predicted_pixels` | 3.609 / 3.700 / 3.924 | 0.896 / 0.908 / 0.938 |
| Full-Jacobian projection batch | 3.461 / 3.495 / 3.508 | 3.312 / 3.344 / 3.365 |
| Residual vector capacity | 3.435 / 3.466 / 3.491 | 3.351 / 3.373 / 3.389 |
| Diagnostic residuals | 3.440 / 3.453 / 3.476 | 0.949 / 0.961 / 0.976 |
| All hints pass cheap gates | 4.075 / 4.095 / 4.137 | 4.093 / 4.121 / 4.162 |
| All hints fail facing gate | 3.637 / 3.666 / 3.673 | 3.329 / 3.351 / 3.373 |
| Complete shifted association | 64.662 / 65.054 / 65.374 | 62.492 / 63.011 / 63.251 |

The all-accepted gate case shows no improvement (its ranges overlap); only rejected
hints avoid covariance work. The small rotation/capacity/shift timing differences
are less robust than the large pixel-only/scoring savings. Earlier unpinned
rotation-only runs even changed the sign of that small difference. Allocation
assertions, not wall-clock thresholds, are the durable regression checks for the
buffer changes. Alias-cost and ray-fit changes are verified as allocation
reductions without a separate timing claim.

Synthetic fixtures use the vision tests' controller geometry and a 24-point
planar acquisition pattern. Results depend on geometry, clutter, compiler,
allocator and CPU scheduling. Hash-set removal is intended for the CV1's small
LED patterns; scanning the output is quadratic for arbitrary much larger bodies.
A reused placement vector may retain capacity from a larger earlier hypothesis;
a residual vector may reserve more than needed if some matched points fail to
project. These are small allocation/copying trade-offs, not promises of lower
retained memory in every input.

No combined live motion-to-photon latency improvement is claimed. Live deployment
verification establishes which binary is running and that the service responds;
it is not a controlled headset latency experiment.
