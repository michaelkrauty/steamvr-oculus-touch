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
        # Three callback cases with full/healthy pipes, two math assertions, four
        # native reentry modes and two C++ assertion builds: fourteen distinct children.
        assert len(snapshots) == 14, f"expected 14 abort snapshots, got {len(snapshots)}"
        for path in snapshots:
            lines = path.read_text().splitlines()
            assert lines[0] == "dumpable: 0", (path.name, lines[0])
            core = next(line for line in lines if line.startswith("Max core file size"))
            assert core.split()[4:] == ["0", "0", "bytes"], (path.name, core)
        print("14 aborting children: own /proc limits = 0/0, dumpable = 0", flush=True)
        if shutil.which("coredumpctl"):
            result = subprocess.run(
                ["coredumpctl", "--no-pager", "--no-legend", "--since", "@" + started,
                 "list", *("COREDUMP_PID=" + path.stem for path in snapshots)],
                capture_output=True, text=True, timeout=30,
                env={**env, "LC_ALL": "C"},
            )
            assert result.returncode == 1 and "No coredumps found" in result.stderr, result
            print("No coredumpctl entries for any of the 14 children", flush=True)


if __name__ == "__main__":
    main()
