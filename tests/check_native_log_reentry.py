# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty

"""Recursive receivers/fatal drains abort silently, including with a full truncated TLS line."""
import os
import signal
import subprocess
import sys

for mode in ("short", "truncated", "fatal", "partial"):
    result = subprocess.run([sys.argv[1], mode], capture_output=True, timeout=2)
    assert result.returncode != 0, (mode, result)
    if os.name == "posix":
        assert result.returncode == -signal.SIGABRT, (mode, result)
    else:
        assert result.returncode not in (77, 78, 79), (mode, result)
    assert not result.stdout and not result.stderr, (mode, result)
