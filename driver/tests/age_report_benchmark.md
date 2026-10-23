# Exact pose-age percentile selection

`test_pose` compares the entire report against the original sorting implementation in `age_report_oracle.h`: 2,176 cases, sizes 1–10,000, odd/even and p99 index boundaries, six distributions, five seeds, empty/singleton/partial/full handoffs, uint64 extremes, rounding and truncated serial/report text. It does not require any ordering of the mutated vectors. The only production caller clears both vectors immediately after logging.

`bench_age_report.cpp` compares that oracle with the actual `age_report` and, separately, its private selection helper (included from `pose.cpp`). Each timed batch has 32 independent inputs copied before the timer starts. Returned strings are consumed after timing; formatting and result construction are timed, destruction and input copies are not. Both implementations see identical input bytes. Two warmup rounds precede nine measured rounds per case, with alternating old/new order. Full strings are checked before timing. `AGE_REPORT_BASELINE` selects sorting for the selection-only comparator when measuring the unchanged baseline source.

Run from the repository root; choose a cache directory for output and your machine's build-slot wrapper. Every compilation and benchmark must run through that wrapper, which honors game holds. For example, with `BUILDSLOT` and `OUT` set:

```sh
mkdir -p "$OUT"
CARGO_BUILD_JOBS=4 "$BUILDSLOT" nice -n 19 g++ -std=c++20 -O3 -DNDEBUG \
  -Idriver/src -Icommon -isystem third_party/openvr/headers \
  driver/tests/bench_age_report.cpp -o "$OUT/bench_age_report"
CARGO_BUILD_JOBS=4 "$BUILDSLOT" nice -n 19 "$OUT/bench_age_report" \
  > "$OUT/age_report.csv" 2> "$OUT/age_report-checksum.log"
```

Repeat the executable three times with separate output files. CSV rows contain ns/report (or ns for selection over the same one/two vectors). Summarize medians and min/max per size/distribution/handoff/operation, retaining every raw row. Do not time a debug build or count input copying as part of the helper.

## Measured on 2026-10-23

Baseline: `6dc19fa52c67069b993bf748bacc538f8da0a966`. Linux x86-64, Core i9-14900KS, GCC/libstdc++ 16.2.1, `-O3 -DNDEBUG`, nice 19, restricted to a four-efficiency-core build slot on a shared machine. The pre-change run compared original production sorting with the preserved oracle. Three candidate runs provided 27 measured batches per case. These are synthetic sizes, not observed live reporting windows.

For random ages with equally sized, independently generated handoffs, pooled medians in microseconds:

| Samples per vector | Full report, sort → select | Selection only, sort → select | Full-report speedup |
| --- | --- | --- | --- |
| 1 | 0.889 → 0.969 | 0.010 → 0.017 | 0.92× |
| 2 | 0.870 → 0.877 | 0.014 → 0.023 | 0.99× |
| 3 | 0.918 → 0.937 | 0.019 → 0.032 | 0.98× |
| 100 | 4.505 → 3.068 | 3.522 → 2.045 | 1.47× |
| 1,000 | 57.256 → 19.967 | 56.393 → 19.014 | 2.87× |
| 10,000 | 795.916 → 191.255 | 795.315 → 205.252 | 4.16× |

The candidate's full-report random-input ranges across those batches were 0.860–1.714, 0.858–1.154, 0.912–1.250, 2.941–3.809, 19.312–25.495 and 185.833–249.977 µs respectively. Shared-host scheduling noise is visible; the selection-only and full-report measurements are independent, so subtracting their medians does not isolate formatting cost.

All six distributions improved at 1,000 and 10,000 samples with handoffs (1.19–2.87× and 1.64–4.16×). Ordered 100-sample inputs were roughly flat (1.326 → 1.288 µs); reversed 100-sample inputs regressed by 0.110 µs (1.318 → 1.428 µs), and constant ones by 0.027 µs. Some tiny-input report medians regressed by about 0.09 µs; selection itself costs an extra 7–13 ns for the random 1–3 sample cases. These absolute costs at the existing ten-second cadence do not justify another algorithm or size threshold. Empty handoffs are also benchmarked.

Keep the two simple `nth_element` calls and `max_element`: the large-window savings justify the small helper. This is average-linear selection, not a worst-case latency bound. Sampling, reporting cadence, synchronous logging and pose submission are unchanged. No end-to-end latency, judder or live-hardware claim is made.

## Regression sensitivity

Changing the index from `q * (size - 1)` to `q * size` fails the complete-string comparison at two ages. Selecting p99 at 1.0 instead of 0.99 also fails at two ages. Retaining p50 by reference instead of copying its scalar before the second selection fails at four handoffs. Mutant source copies were compiled outside the checkout; the restored production implementation passes. None of these failures was worked around in the tests.

## Validation and limits

Focused native driver tests passed 10/10. After rebasing onto `6ceab21`, the complete `scripts/check.sh` passed with Rust 1.99.0: formatting, strict Clippy, 1,247 Rust tests (11 ignored), 129 native tests in both ordinary and long-TMPDIR runs, expected-abort audit, driver contracts and CLI/package integration suites. Strict Clippy on 1.85.0 and both Windows MSVC type-check modes also passed. The Windows pose-test and full driver cross-builds passed with MinGW; the latter emitted an existing `%zu` format warning in untouched `shm_client.cpp`. Native Windows execution was unavailable locally.

The first check-script attempt on the older baseline failed ten long-TMPDIR recording/replay tests because the disk had only 14–16 GiB free and their unchanged recording guard requires 20 GiB. The first full Windows link also ran out of space; its retry succeeded. Removing inactive build caches recovered 36.9 GiB, after which the complete check-script rerun passed on the rebased source. No code, test, threshold or assertion was changed to bypass either failure. Both runs used a dedicated Cargo target, incremental compilation off and dev/test debug information off.

Sources stayed frozen during compilation, tests and measurements; only evidence and changelog wording were finalized afterward. All checks were offline.
