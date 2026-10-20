# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty

"""Audit all intentional Linux aborts using snapshots taken by the crashing processes.

Usage: python3 tests/check_expected_aborts.py DAEMON_TEST_EXECUTABLE NATIVE_BUILD_DIR
The executables must already be built. No hardware or installed state is used.
"""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time


# Each callback test aborts twice (full pipe and healthy pipe). The other suites abort once.
# Names come from the suites, so a new callback test fails the list check instead of passing
# under a stale total. Snapshot files stay keyed by pid: coredumpctl matches COREDUMP_PID.
CALLBACK_TESTS = (
    "callback_tests::activity_panic_aborts_with_a_full_pipe",
    "callback_tests::frame_panic_aborts_with_a_full_pipe",
    "callback_tests::identity_panic_aborts_with_a_full_pipe",
    "callback_tests::image_panic_aborts_with_a_full_pipe",
    "callback_tests::report_panic_aborts_with_a_full_pipe",
    "callback_tests::search_panic_aborts_with_a_full_pipe",
)
ASSERTION_TESTS = (
    "daemon_log::assertion_tests::a_native_assertion_aborts_with_a_full_stderr_pipe",
    "daemon_log::assertion_tests::a_native_assertion_reaches_a_healthy_stderr_pipe",
)
REENTRY_MODES = ("short", "truncated", "fatal", "partial")
ASSERT_BUILDS = ("routed", "standalone")


def listed_tests(executable, test_filter, env):
    result = subprocess.run(
        [str(executable), test_filter, "--list"],
        capture_output=True, text=True, timeout=30, env=env,
    )
    assert result.returncode == 0, (test_filter, result.returncode, result.stderr)
    suffix = ": test"
    return [line[:-len(suffix)] for line in result.stdout.splitlines() if line.endswith(suffix)]


def main():
    if sys.platform != "linux":
        raise SystemExit("This audit requires Linux /proc and PR_GET_DUMPABLE")
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    daemon, native = (Path(arg).resolve() for arg in sys.argv[1:])
    tests = Path(__file__).resolve().parent
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = os.pathsep.join(
        part for part in (str(native / "third_party/openhmd"), env.get("LD_LIBRARY_PATH")) if part
    )
    callbacks = listed_tests(daemon, "callback_tests::", env)
    assertions = listed_tests(daemon, "daemon_log::assertion_tests::", env)
    assert sorted(callbacks) == sorted(CALLBACK_TESTS), (
        f"callback tests: {callbacks}, expected {list(CALLBACK_TESTS)}"
    )
    assert sorted(assertions) == sorted(ASSERTION_TESTS), (
        f"assertion tests: {assertions}, expected {list(ASSERTION_TESTS)}"
    )
    # Each callback case with a full and a healthy pipe, each math assertion, each native
    # reentry mode and each C++ assertion build: one distinct child apiece.
    expected = (
        len(CALLBACK_TESTS) * 2 + len(ASSERTION_TESTS) + len(REENTRY_MODES) + len(ASSERT_BUILDS)
    )
    started = str(int(time.time()))
    with tempfile.TemporaryDirectory(prefix="cv1t-aborts-") as directory:
        env["CV1TOUCH_TEST_ABORT_SNAPSHOTS"] = directory
        commands = [
            [str(daemon), "callback_tests::", "--test-threads=1"],
            [str(daemon), "daemon_log::assertion_tests::", "--test-threads=1"],
            [sys.executable, str(tests / "check_native_log_reentry.py"),
             str(native / "tests/test_native_log_reentry")],
            [sys.executable, str(tests / "check_native_assert.py"),
             str(native / "tests/test_native_assert_routed"), "routed"],
            [sys.executable, str(tests / "check_native_assert.py"),
             str(native / "tests/test_native_assert_standalone"), "standalone"],
        ]
        failures = []
        for command in commands:
            result = subprocess.run(command, env=env, timeout=30)
            if result.returncode:
                failures.append((command, result.returncode))
        assert not failures, failures
        snapshots = list(Path(directory).glob("*.limits"))
        assert len(snapshots) == expected, (
            f"expected {expected} abort snapshots, got {len(snapshots)}"
        )
        for path in snapshots:
            lines = path.read_text().splitlines()
            assert lines[0] == "dumpable: 0", (path.name, lines[0])
            core = next(line for line in lines if line.startswith("Max core file size"))
            assert core.split()[4:] == ["0", "0", "bytes"], (path.name, core)
        print(f"{expected} aborting children: own /proc limits = 0/0, dumpable = 0", flush=True)
        if shutil.which("coredumpctl"):
            result = subprocess.run(
                ["coredumpctl", "--no-pager", "--no-legend", "--since", "@" + started,
                 "list", *("COREDUMP_PID=" + path.stem for path in snapshots)],
                capture_output=True, text=True, timeout=30,
                env={**env, "LC_ALL": "C"},
            )
            assert result.returncode == 1 and "No coredumps found" in result.stderr, result
            print(f"No coredumpctl entries for any of the {expected} children", flush=True)


if __name__ == "__main__":
    main()
