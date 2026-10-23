# Iterated optical covariance update

The posterior covariance is consumed only after `Filter::update` finishes. Its information matrices, solves and state updates still run in the original order; only the final accepted factor is inverted. The prior inverse and final symmetrization are unchanged. An update taking I iterations now performs one posterior inverse instead of I, including when iteration zero requests the minimum of one iteration. A later failure returns `None` without computing a discarded posterior covariance.

## Reproduce the focused check and timing

The independent original implementation is frozen in `src/track/eskf/equivalence.rs` from `6dc19fa52c67069b993bf748bacc538f8da0a966`; the method and Cargo.lock are unchanged in the measured baseline `6ceab21ece35d0f3faf4bfe62ad8f59b6de67be3`. The measured candidate source was `da18166a6d6255e8b487a19376b9d80fca4bc03a`, before this evidence update. Run the ordinary equivalence tests, then the optional release timing test:

```sh
nice -n 19 cargo +1.99.0 test --locked --lib track::eskf::equivalence
nice -n 19 cargo +1.99.0 test --locked --release --lib track::eskf::equivalence::covariance_update_timing -- --ignored --exact --nocapture
```

Use a dedicated target directory and the site's game-aware build-slot wrapper when sharing a VR workstation. The benchmark consumes complete outputs with `black_box`, uses 16 dense positive-definite 15x15 priors with 12 projected LED observations, and alternates original/new order across nine samples of 4,096 updates each, after warmup. Half the frames have a gross pixel outlier. It includes caps 0, 1, 4 and 10, plus a first-iteration convergence workload. The histogram is eight 3-iteration and eight 4-iteration optical fits at caps 4 and 10; half the default-cap fits exhaust the cap. The other workloads take one iteration. The output label `capped` denotes cap 10, not ten executed iterations.

## Measured 2026-10-23

Linux, Intel Core i9-14900KS efficiency cores, four-core build slot, nice 19, four build jobs, Rust 1.99.0 (`b940084d7`), default release profile. No load generator or live hardware. Two consecutive release runs yielded these ns/update medians and sample ranges:

| Workload / run | Original median [min, max] | Deferred inverse median [min, max] | Reduction |
| --- | ---: | ---: | ---: |
| Cap 4 / 1 | 28,136 [27,932, 28,439] | 20,315 [20,129, 20,721] | 27.80% |
| Cap 4 / 2 | 27,275 [27,011, 27,506] | 19,587 [19,482, 20,295] | 28.19% |
| Cap 10 / 1 | 28,247 [27,423, 28,571] | 20,197 [19,705, 20,655] | 28.50% |
| Cap 10 / 2 | 27,852 [27,193, 28,278] | 20,011 [19,484, 20,418] | 28.15% |

Single-iteration median changes stayed within 0.85% across caps zero and one and immediate convergence, with overlapping sample ranges and opposite directions between the two runs. Before the production change, timing the two identical implementations gave median differences within 0.65%. The small single-iteration variation is not a demonstrated regression or speedup.

Temporary isolated counters at the posterior `chol.inverse()` sites verified I inverses in the original against one in production for every successful fixture at every cap, and one against zero for a later failed factorization. The initial prior inverse was not instrumented or changed. Instrumentation was removed byte-for-byte before the final checks; no counters or timing hooks enter production.

## Equivalence and limits

The tests compare the complete returned `Option`, all quaternion/position/velocity/bias floating-point bits, all 225 covariance entries, residual bits and callback input-state order/count. They cover clean/outlier frames, zero/one/default/ten iteration caps, early convergence, forced cap exhaustion, invalid prior, first/later empty observations, empty final residual evaluation, and a constructible later Cholesky failure using an invalid negative Huber weight. The oracle tests pass before the optimization as expected. Removing the convergence inverse, mishandling cap zero, or adding an extra measurement callback each makes them fail; restoring production makes them pass.

The same existing 30-second recording and matching historical calibration were replayed by fresh release binaries from the exact baseline and candidate above, with trace and 20 ms delayed outputs enabled. The input/config snapshot was frozen and hashed. Two rounds of five alternating pairs reversed their initial order; one warmup per binary was retained separately. Replay elapsed time excludes calibration loading and complete dump formatting/flushing.

| Replay timing set | Baseline seconds median [min, max] | Candidate seconds median [min, max] | Reduction |
| --- | ---: | ---: | ---: |
| Round 1 | 1.080340 [1.072115, 1.089250] | 0.977576 [0.972078, 1.026631] | 9.51% |
| Round 2 | 1.093915 [1.075515, 1.114597] | 0.994400 [0.986688, 0.999941] | 9.10% |
| Combined ten pairs | 1.086865 [1.072115, 1.114597] | 0.990232 [0.972078, 1.026631] | 8.89% |

Every pair favored the candidate; paired reductions ranged from 4.35% to 10.76%. All 22 complete `Replay` Debug dumps, including warmups, matched by direct byte comparison and SHA-256: 14,012 / 14,098 published samples, 4,873 / 4,755 fits and 12,496 events. This includes poses, velocities, biases, delayed outputs, statistics, misses, fits, dropped/skipped frames, events, probes and camera agreement. Debug output does not serialize NaN payload bits or internal filter covariance; the per-update bitwise oracle covers covariance. Current dump SHA-256: `8ac52e03638fa41238f969ca88d16da20b820c530d053a50d6f74f6f94d908d7`. Recording SHA-256: `aa81fe54885d8e766c15972db4af78c4d227478f9ef13449a6c769e45ecb62c3`; calibration SHA-256: `075d65d163a2981353ea71c12a36c2e27c4cbca54fbbfde82086e59ca2d94d63`. Earlier old-base measurements and a different dump hash are retained as historical artifacts; upstream pipeline changes make them unsuitable as the current-base comparison.

One initial old-base dump attempt exited 101 with a partial artifact while disk space was tight. The initial runner did not retain its captured stderr, so the cause is unconfirmed. A direct retry and all ten subsequent old-base paired runs passed. All 22 fresh current-base runs passed on their first attempt, with distinct retained stdout/stderr/dump files. Those successes do not establish the cause of the earlier failure.

Complete `scripts/check.sh`, strict Clippy on Rust 1.99/1.85 and both Windows MSVC type-check modes passed on the frozen rebased candidate. This includes both ordinary and long-TMPDIR Rust/native/integration runs, 129 native tests in each mode, expected-abort audit and driver contracts. Source manifests stayed identical throughout checks and measurements. Native Windows execution remains unverified locally.

These are elapsed filter/replay processing times. It does not establish display latency, judder, live tracking quality, hardware behavior, or every pipeline lifecycle path. Synthetic updates repeat independent priors; real iteration distributions depend on the recording and tracking conditions. The private replay fixture is not distributed with the repository. No prediction, tracking thresholds, association, iteration limit or public API changes.
