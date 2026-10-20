# Tests that deliberately abort

The fatal-path tests keep their expected `SIGABRT`, but disable core dumps only in the subprocess about to fail. Production crashes retain their normal reporting. Run the ordinary Rust and Meson suites through `scripts/check.sh`; their abort fixtures check their own resource limits and, on Linux, dumpability before triggering the failure. On Linux, `scripts/check.sh` also runs the focused audit below.

A zero `RLIMIT_CORE` prevents systemd-coredump from storing a core, but Linux still invokes a piped `core_pattern` handler. The resulting journal event can also wake a desktop crash handler. Linux fixtures therefore also set `PR_SET_DUMPABLE=0` immediately before the fatal operation. See [core(5)](https://man7.org/linux/man-pages/man5/core.5.html) and [PR_SET_DUMPABLE(2)](https://man7.org/linux/man-pages/man2/PR_SET_DUMPABLE.2const.html).

Do not apply this to a whole test suite or before exec. Non-dumpable processes restrict ptrace and access to `/proc/<pid>/fd`, including the short paths used by SocketDir fixtures. None of the intentional-abort children below owns a SocketDir view: they only use anonymous pipes, a poisoned mutex, logger callbacks or a loaded math-assertion bridge. Setup completes before dumpability changes.

| Source | Cases | Deliberate aborts per suite run |
| --- | --- | --- |
| `src/bin/cv1touchd/callback_tests.rs` | `frame_panic_aborts_with_a_full_pipe`, `search_panic_aborts_with_a_full_pipe`, `identity_panic_aborts_with_a_full_pipe`, `image_panic_aborts_with_a_full_pipe`, `report_panic_aborts_with_a_full_pipe`, `activity_panic_aborts_with_a_full_pipe`; each also runs with a healthy pipe | 12 |
| `src/bin/cv1touchd/log.rs` | `a_native_assertion_aborts_with_a_full_stderr_pipe`, `a_native_assertion_reaches_a_healthy_stderr_pipe`; real `OHMD_ASSERT` through `native_assert_bridge.c` | 2 |
| `tests/test_native_log_reentry.c` via `check_native_log_reentry.py` | `short`, `truncated`, `fatal`, `partial` | 4 |
| `tests/test_native_assert.cpp` via `check_native_assert.py` | `fail` in the routed and standalone builds | 2 |

The `abort()` calls in `tests/test_search_cancel.c` and `tests/test_inspection_handoff.c` are failures on an unexpected stub call or an unknown case name, not expected crashes; the stubs in `tests/test_search_skip.c` fail through `CHECK` (exit status 1, no core). Driver tests expect no aborts: `test_control_disconnect.cpp` restores default `SIGPIPE` to catch a broken socket send, then requires normal exit; SIGPIPE does not generate a core. The worker-failure tests in `cv1touchd.rs` expect exit status 1 after a caught panic. `tests/daemon_logging.rs` exercises the fatal log drain through an invalid option and expects exit status 2. These normal exits need no core suppression.

## Focused Linux audit

`tests/check_expected_aborts.py` takes an already built `cv1touchd` **test executable** and the native build directory. `scripts/check.sh` runs it on Linux after `cargo test`. It runs the exact abort suites and requires twenty distinct snapshots of each aborting process's own `/proc/self/limits` and `PR_GET_DUMPABLE` value: six callback cases, each with a full pipe and a healthy pipe, two math assertions, four native reentry modes and two C++ assertion builds. The callback and math-assertion names must be exactly the tests that executable lists. Each snapshot must have zero soft/hard core limits and dumpability zero. When `coredumpctl` is installed, it also requires no journal entries for those twenty PIDs since the run began. Temporary snapshots stay under the caller's TMPDIR and are removed on exit.

Find the test executable in Cargo's `--message-format=json` compiler artifacts (`target.name=cv1touchd`, `profile.test=true`, bin kind). Its native directory is the `CV1TOUCH_NATIVE_DIR` in the same output's `build-script-executed` message for this package, the build the harness was compiled against; the profile's `cv1touch-native-dir` marker names only the latest build script run. `scripts/check.sh` takes both from that output and fails if the native directory is missing `tests/test_native_log_reentry` or either `test_native_assert_*` binary. The audit does not build anything or access hardware or installed configuration.

Both helpers check dumpability before reaching the fatal operation, so removing either Linux `PR_SET_DUMPABLE` call fails even when the runner already imposed a zero hard core limit. A native setup failure exits with status 80; a Rust setup failure fails the child test. Neither is accepted as the expected SIGABRT. The optional snapshots allow the separate audit to reject missing fixture calls as well.
