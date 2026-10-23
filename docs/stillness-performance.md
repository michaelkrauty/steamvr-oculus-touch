# IMU stillness-window performance

The detector finds the first retained timestamp at or after the window boundary with `VecDeque::partition_point`, then chains that forward suffix with the incoming sample. `Tracker::imu` rejects non-increasing timestamps and clears history after a discontinuity. Under that invariant, the old filter and the new range enumerate exactly the same references in the same order, including wrapped storage and an empty suffix. The count is the suffix length plus one. Both vector sums retain their zero-initialized forward reduction, normalization, bias priority, strict thresholds and short-circuit order. There are no running sums or new persistent state.

## Reproduce

Choose a cache output directory (`OUT`) and the build-slot wrapper (`BUILDSLOT`) used on the development machine. Every heavy command must run through the wrapper, which honors game holds. From the repository root:

```sh
export CARGO_TARGET_DIR="$OUT/target"
export CARGO_BUILD_JOBS=4
export CARGO_INCREMENTAL=0
"$BUILDSLOT" nice -n 19 cargo +1.99.0 test --locked --release --lib track::tracker::stillness_tests
"$BUILDSLOT" nice -n 19 cargo +1.99.0 test --locked --release --lib track::tracker::stillness_tests::stillness_cpu_benchmark -- --ignored --exact --nocapture --test-threads=1 > "$OUT/stillness-benchmark.log" 2>&1
```

`original` uses the independent original detector inside the complete IMU handler from base `6ceab21ece35d0f3faf4bfe62ad8f59b6de67be3`. `actual` calls the real detector and `Tracker::imu`. A source comparison verifies that the legacy handler differs only in its method name and detector call; a committed test keeps that check current. The original detector is unchanged between the earlier `6dc19fa` base and this base, and the retained oracle matches it except for its name and the test-only mean tap. The tap observes actual reductions for exact comparisons and is absent from production builds. Both timed detectors have the same inactive tap. The allocator forwards to `System`, including reallocation, and only counts an explicitly armed interval on the current thread. Counting is disabled during timing; allocation probes run separately. Inactive allocator/tap checks remain in this release test executable, so timings describe this harness.

## Equivalence and regression evidence

Two detector differential tests compare actual decisions and all six mean bit patterns with the original detector: startup, inclusive boundary, empty selected suffix, zero/custom windows, deque wrapping/growth, seeded biases and history-over-saved bias priority, still/moving samples, adjacent representable threshold values, and cancellation-sensitive forward sums. A third test sends three deterministic rest/motion/rest trajectories (6,000 samples) through the actual and original IMU paths. Acquisition priors are seeded synthetically before propagation and after the gap. Three camera queues and outstanding inspection snapshots are populated repeatedly, including immediately before the gap. Every step compares retained nominal filters, all 225 covariance entries, saved/acquired bias filters and held inspection priors by bits, their camera/exposure/deadline/peer metadata and order, camera update queues, IMU/stillness histories, applied measurements, tracking/reset metadata, agreement and diagnostics. Duplicate and out-of-order samples leave this state unchanged.

A separate populated lifecycle test checks camera timestamps immediately below, at and above the cutoff, inspection retention up to its exact deadline, unchanged retained prior bits and metadata, and max-gap clearing of entries that ordinary expiry would retain. These are synthetic bookkeeping fixtures; no camera I/O, image fitting or native search runs. Existing tracker/pipeline tests supplement them. Both populated comparisons and the source-copy test fail with the stale copied handler, and pass after refreshing it. No production handler change was needed.

Equivalence tests also pass with the original detector restored in the current handler. They are not a performance regression. `stillness_detector_does_not_allocate` fails with that original detector and passes with the new one. It measures only detector calls, after fixture construction, across three illustrative rates, both deque layouts, and four window lengths. It does not claim the complete tracker is allocation-free.

The original six separately compiled detector mutations failed: exclusive boundary, reversed gyro sum, reversed accelerometer sum, selecting the whole history, incoming sample twice, and omitting the incoming sample from the count. Their detector lines and the two differential detector tests and allocation regression remain unchanged, so those historical mutation artifacts are retained. Six new production-handler mutations each fail the populated lifecycle comparison alone (excluding the source-copy test): dropping camera gap clearing, dropping inspection gap clearing, dropping inspection expiry, dropping camera expiry, making camera cutoff expiry inclusive, and retaining inspections at the exact deadline. Restoring the candidate passes all six focused tests.

## Measurements

2026-10-23, Linux x86-64, Intel Core i9-14900KS efficiency cores assigned by buildslot, Rust 1.99.0 / LLVM 23.1.1, release defaults. Two complete runs each use 21 rounds with old/new order alternating. The table is the second run; each cell is median [minimum, maximum] nanoseconds. Detector rounds consume 30,000 repeated evaluations; full-call rounds consume 2,000 increasing samples, with stationary first/last quarters and motion in between. Setup/cloning, camera fixture injection, allocation probes and comparisons are outside timing. Results and full tracker state are consumed, and complete state is compared at every sample outside timing for each full-call workload. Linux uses the calling thread's CPU clock, excluding time descheduled by other work or game holds; raw output also records elapsed wall time. The portable fallback measures elapsed time.

Default fixtures retain 400 ms and select the last 200 ms. Rates are illustrative, not live measurements. All fixtures configure three cameras. Ordinary rows leave their bookkeeping empty. The two `-cameras` rows time ten independently prepared 400 ms batches (200 calls each at 500 Hz), each starting with 23 accepted exposure timestamps per camera at 50 Hz and four outstanding inspection snapshots per camera. Exposure queues expire gradually; deadlines remove snapshots during each batch and leave one per camera throughout its end. All snapshots and their metadata are compared as well as the filters. Camera fixture construction and resets are outside timing; the handler's actual retention and expiry work is inside. Detector measurements for these rows are the same detector-only workload as ordinary rows, since camera state is not read by the detector. `startup` has only 2 ms of history; `short`, `zero`, and `custom` use 20, 0, and 300 ms windows at 500 Hz; `moving` fails the gyro-mean gate. Full-call batches mature even when starting from `startup`.

| Case | Original detector ns | New detector ns | Original full IMU ns | New full IMU ns |
| --- | ---: | ---: | ---: | ---: |
| 250-contiguous | 537.3 [512.4, 554.1] | 387.2 [384.3, 391.9] | 2057.7 [1990.0, 2142.0] | 1826.7 [1800.2, 1856.0] |
| 250-wrapped | 571.7 [547.7, 581.9] | 389.4 [384.4, 392.4] | 1977.5 [1938.8, 2009.3] | 1754.2 [1729.2, 1789.3] |
| 500-contiguous | 1035.4 [1010.5, 1053.1] | 720.7 [709.9, 730.3] | 2313.0 [2290.5, 2340.4] | 1919.3 [1902.8, 1951.9] |
| 500-wrapped | 1028.9 [1010.0, 1041.9] | 717.4 [709.8, 729.6] | 2286.8 [2246.6, 2462.4] | 1929.3 [1896.8, 1986.6] |
| 1000-contiguous | 1885.3 [1836.3, 1924.3] | 1414.5 [1398.0, 1463.4] | 2989.0 [2953.0, 3140.1] | 2361.8 [2338.4, 2479.3] |
| 1000-wrapped | 1885.8 [1859.5, 1907.3] | 1409.6 [1393.6, 1420.7] | 3003.7 [2933.0, 3089.7] | 2355.8 [2312.0, 2380.1] |
| 500-contiguous-cameras | 1024.4 [1006.4, 1049.7] | 722.9 [709.3, 745.0] | 3025.3 [2751.5, 3185.7] | 2589.7 [2330.8, 2797.0] |
| 500-wrapped-cameras | 1028.4 [1011.7, 1039.9] | 725.1 [719.4, 732.5] | 2982.8 [2607.3, 3049.7] | 2560.7 [2521.2, 2628.5] |
| startup | 3.8 [3.5, 4.0] | 4.2 [4.0, 4.6] | 2257.2 [2206.1, 2287.7] | 1897.3 [1859.8, 1939.2] |
| short | 266.4 [261.7, 278.6] | 123.0 [121.8, 126.7] | 1890.5 [1864.4, 1966.2] | 1650.8 [1632.3, 1686.6] |
| zero | 120.2 [119.2, 122.6] | 39.1 [38.7, 44.3] | 1769.9 [1726.7, 1838.1] | 1633.0 [1599.1, 1672.8] |
| custom | 1459.5 [1422.8, 1496.2] | 1073.8 [1063.1, 1081.9] | 2595.4 [2552.7, 2645.6] | 2127.6 [2097.9, 2174.0] |
| moving | 482.3 [451.2, 504.6] | 171.8 [168.8, 174.2] | 2297.3 [2254.2, 2409.2] | 1902.3 [1889.6, 1951.1] |

Across both runs, default 500 Hz contiguous detector medians improve by 30.4% and full-call medians by 17.0–17.6%; wrapped full calls improve by 15.6–15.9%. With populated cameras, full calls improve by 13.9–14.4% across both layouts and runs. Startup detector medians are 0.45 and 0.48 ns slower (3.52 to 3.97 ns and 3.76 to 4.24 ns), despite an unchanged early-return path; this small harness regression is retained in the results. Startup full-call batches still improve after maturing. No material full-call regression appears in these workloads.

At 250/500/1,000 Hz with the default window, the original detector makes 5/6/7 allocator requests (allocation plus reallocations), frees one buffer, and cumulatively requests 992/2,016/4,064 bytes. These byte counts sum requested sizes, not peak live memory. The new detector makes zero requests and zero frees in every measured case. Startup was already allocation-free. Full-handler probes include history/storage allocations: at 500 Hz, both empty and populated cases fall from nine requests to three and from 28,688 to 26,672 cumulatively requested bytes per probed call. The six removed requests and 2,016 bytes match the detector-only probe; three deque growth requests remain. Only the integrated detector allocation removal is claimed.

The previous measurements on `6dc19fa` remain historical cache artifacts. Their copied full-IMU baseline omitted camera-update and inspection cleanup added upstream, so those old full-call numbers did not prove current-handler equivalence. This table replaces them using the refreshed handler and populated camera coverage. The original detector measurements and six mutation artifacts are retained without rewriting them. An earlier mutation compilation hit a full filesystem; that historical infrastructure failure remains recorded. This refresh uses no incremental compilation and preserves the recording free-space guard.

These are synthetic CPU and allocation measurements. No recording replay, live hardware, display-latency or judder improvement is established. Raw timing rounds, allocation probes, mutation failures and full gate logs are retained outside the repository in the task's build cache.
