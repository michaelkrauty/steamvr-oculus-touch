#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty

"""Offline actual Pipeline -> daemon apply_tracked -> driver make_pose regression.

Only unit harnesses and a CPU-only stdin consumer run; no daemon, hardware or IPC objects.
Run with the Meson-built driver/test_rust_clock_pose executable as the sole argument.
"""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

with tempfile.TemporaryDirectory(prefix="cv1touch-clock-") as tmp:
    env = dict(os.environ, CV1TOUCH_CLOCK_PUBLICATIONS=str(Path(tmp) / "publications.json"))
    commands = [
        ["cargo", "test", "--locked", "--lib",
         "fifo_receive_stalls_preserve_physical_state_and_fix_age", "--", "--nocapture"],
        ["cargo", "test", "--locked", "--bin", "cv1touchd",
         "rust_clock_pipeline_publications_reach_driver_unchanged", "--", "--nocapture"],
    ]
    results = []
    for i, command in enumerate(commands):
        if i == 1:
            if not Path(env["CV1TOUCH_CLOCK_PUBLICATIONS"]).is_file():
                raise SystemExit("Pipeline harness did not produce publications")
            env["CV1TOUCH_CLOCK_DRIVER"] = str(Path(sys.argv[1]).resolve())
        results.append(subprocess.run(command, env=env, check=False).returncode)
    if any(results):
        raise SystemExit(f"Rust clock boundary failed: {results}")
