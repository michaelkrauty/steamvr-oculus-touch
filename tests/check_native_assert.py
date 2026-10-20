# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty

"""A header assertion still evaluates once and aborts in routed and standalone C++ builds."""
import os
import signal
import subprocess
import sys

if os.name == "posix":
    import resource

    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

exe, mode = sys.argv[1:]
subprocess.run([exe], check=True, timeout=2)
result = subprocess.run([exe, "fail"], capture_output=True, timeout=2)
result.stderr = result.stderr.replace(b"\r\n", b"\n")
assert result.returncode != 0, result
if os.name == "posix":
    assert result.returncode == -signal.SIGABRT, result
assert b"argc == 0" in result.stderr, result.stderr
assert b"test_native_assert.cpp" in result.stderr, result.stderr
assert b".cpp:" in result.stderr or b"line" in result.stderr, result.stderr
if mode == "routed":
    assert b"fatal drain called\n" in result.stderr, result.stderr
