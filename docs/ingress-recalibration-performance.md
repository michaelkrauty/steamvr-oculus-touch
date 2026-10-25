# Ingress and sensor-recalibration CPU work

Baseline: `985f278`, immediately before the v0.3.0 release pass. These changes
preserve tracking thresholds, clock fitting, exposure ordering, queue limits,
round-robin service, and calibration matching decisions. They do not change the
recording format or shared-memory ABI relative to that baseline (v0.3.0 as a whole
does update the ABI relative to v0.2.0).

## Changes

- Register each sensor with capacity for its two queued frames. When full, reject
  an older incoming exposure or evict the oldest retained exposure **before**
  inserting the new one. Duplicate timestamps still replace their earlier frame
  and count as one drop. The queue never temporarily needs a third slot. This
  eagerly reserves two payload slots even for an admitted but idle sensor; it
  does not preallocate the blobs owned by a frame.
- Remove selected frames in place. Previously, even selecting no frames rebuilt
  the retained deque; now an empty selection allocates nothing, and selecting all
  frames does not discard the ingress queue's storage. Only returned frames need
  an output vector. This is bounded removal from a two-element queue, not a
  quadratic scan of an unbounded backlog.
- Reuse the radio worker's output clock vector. `take_clocks_into` clears the old
  batch and drains the new one while keeping the ingress and consumer buffers.
  Fitting still happens **after** releasing the ingress mutex. The consumer keeps
  a small bounded buffer for at most 32 samples, plus `Vec`'s spare capacity;
  the first batch or growth to a larger batch can still allocate.
- Sensor refinement needs only the nearest and next-nearest blob, not a sorted
  list for each projected LED. A linear scan replaces those temporary distance
  vectors and stable sorts. Strict `total_cmp` comparisons preserve first-tie
  selection, including non-finite ordering. Residual medians use selection instead
  of a full sort, keeping the same upper-median definition.

Sensor refinement runs on the background solver, not on every IMU publication.
Reducing its CPU demand is useful during recalibration, but is not evidence that
normal motion-to-photon latency changed. The ingress savings likewise remove
allocator work; they do not lower the radio or camera transport latency floor.

## Verification

`src/frame_queue/perf_tests.rs` tests 20,000 insertions against an independent
sorted-vector reference, duplicate and late exposures, wrapping queues, every
two-frame selection, preserved service order, clock overflow and empty batches.
Thread-local allocation counters verify allocation-free registered frame
insertion/empty selection and warmed clock handoff, including overflow. Existing
hotplug and 65-sensor fairness tests continue to apply.

`src/track/refine/perf_tests.rs` retains the baseline matching algorithm. It checks
2,000 synthetic scenes across five gates/margins, clutter, duplicate blobs and
LEDs, order reversal, perturbed camera poses, exact gate/margin boundaries,
empty/singleton inputs and non-finite coordinates. Match outputs compare their
input bits in order. Median tests compare selected values against a sorted
reference, including signed zeros and NaN payloads. The 24-LED allocation fixture
removes at least 24 allocation requests per matching call. Existing end-to-end
synthetic sensor placement/refinement tests cover the solver using these matches.

```sh
cargo test --locked --no-default-features --lib frame_queue::
cargo test --locked --no-default-features --lib track::refine::
scripts/check.sh
```

## Offline measurements

```sh
cargo test --locked --release --no-default-features --lib benchmark_ingress_clock_batch \
  -- --ignored --nocapture --test-threads=1
cargo test --locked --release --no-default-features --lib benchmark_recalibration_matching \
  -- --ignored --nocapture --test-threads=1
```

The committed benchmarks alternate baseline/optimized order over 12 rounds and
use `black_box`. Fixtures and warm-up are outside timing. The clock benchmark
includes insertion, draining and disposal of each old output batch (100,000
batches per round); refinement includes result destruction (5,000 calls per
round). Allocation counting is disabled during timing. Both implementations run
in the same optimized binary; these isolate the changed operations rather than
comparing two complete daemons. No elapsed-time threshold gates CI.

Measurements below use an Intel Core i9-14900KS, Rust 1.86.0, release profile on
Linux, pinned to logical CPU 16 with `taskset -c 16`. The host is not otherwise
isolated. Min/median/max describe the 12 rounds, not a confidence interval.
Synthetic geometry and allocator/CPU/compiler differences limit generalization;
there is no new hardware tracking or display-latency measurement in this pass.

| Clock batch | Baseline min / median / max (ns) | Reused min / median / max (ns) |
| --- | --- | --- |
| 1 sample | 20.7 / 21.2 / 22.5 | 3.6 / 3.8 / 4.1 |
| 8 samples | 36.0 / 44.4 / 45.2 | 23.6 / 24.2 / 27.7 |
| 32 samples | 115.1 / 115.9 / 118.1 | 116.6 / 117.1 / 119.5 |

Small clock batches benefit; full-backlog timing does not improve and its ranges
overlap. Removing per-batch allocations is the durable regression claim, not a
speedup for every backlog size. These savings are nanoseconds, not milliseconds.

| Sensor-refinement matching | Baseline min / median / max (µs) | Nearest-two min / median / max (µs) |
| --- | --- | --- |
| 24 LEDs, 24 blobs | 7.662 / 7.772 / 7.855 | 3.694 / 3.724 / 3.772 |
| 24 LEDs, 24 blobs + 64 clutter blobs | 29.003 / 29.334 / 30.735 | 7.936 / 8.066 / 8.129 |

Matching took about 52% and 73% less time respectively in these fixtures. This
does not measure the complete nonlinear solver, where projection and matrix work
remain; median selection is verified for equivalence without a separate timing
claim.
