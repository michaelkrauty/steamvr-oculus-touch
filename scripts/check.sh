#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
# Build and run hardware-independent Rust and native regressions.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
python3 -m unittest discover -s tests -p test_install.py
export CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-4}"
cargo fmt --all --check
cargo clippy --locked --all-targets -- -D warnings
cargo test --locked --all-targets
messages=$(mktemp)
socket_tests=
trap 'rm -f -- "$messages"; rm -rf -- "${socket_tests:-}"' EXIT
cargo build --locked --release --bins --message-format=json > "$messages"
# Cargo reports the actual executable location, including custom target directories
# and an explicitly selected native target triple.
daemon=$(python3 - "$messages" <<'PY'
import json, sys
from pathlib import Path
matches = set()
for line in Path(sys.argv[1]).read_text().splitlines():
    event = json.loads(line)
    if (event.get("reason") == "compiler-artifact"
            and event.get("target", {}).get("name") == "cv1touchd"
            and "bin" in event.get("target", {}).get("kind", [])
            and event.get("executable")):
        matches.add(event["executable"])
if len(matches) != 1:
    raise SystemExit("Cargo did not report exactly one cv1touchd executable")
print(matches.pop())
PY
)
native=$(cat "$(dirname "$daemon")/cv1touch-native-dir")
meson test -C "$native" --no-rebuild --print-errorlogs
# The SteamVR driver against the daemon's own shared-memory and control code.
cargo run --locked --release --example driver_contract -- "$native/driver/test_daemon_contract"
readelf -d "$daemon" | grep -F '$ORIGIN/../lib/cv1touch' >/dev/null
CV1TOUCH_TEST_DAEMON="$daemon" CV1TOUCH_TEST_BINARY="$(dirname "$daemon")/cv1touch" CV1TOUCH_TEST_LIBRARY_PATH="$native/third_party/openhmd" python3 -m unittest discover -s tests -p test_runtime_dir.py
CV1TOUCH_TEST_BINARY="$(dirname "$daemon")/cv1touch" CV1TOUCH_TEST_LIBRARY_PATH="$native/third_party/openhmd" python3 -m unittest discover -s tests -p test_driver_package.py
CV1TOUCH_TEST_BINARY="$(dirname "$daemon")/cv1touch" CV1TOUCH_TEST_LIBRARY_PATH="$native/third_party/openhmd" python3 -m unittest discover -s tests -p test_setup.py
CV1TOUCH_TEST_BINARY="$(dirname "$daemon")/cv1touch" CV1TOUCH_TEST_LIBRARY_PATH="$native/third_party/openhmd" python3 -m unittest discover -s tests -p test_report.py
# Exercise every socket fixture with a long base even when the caller's TMPDIR is short.
# All files stay in a private tree under this build's output and are removed by the trap.
socket_tests=$(mktemp -d "$(dirname "$daemon")/socket-tests.XXXXXX")
chmod 700 "$socket_tests"
long_tmp="$socket_tests/$(printf '%080d' 0)"
mkdir -m 700 "$long_tmp"
(
    export TMPDIR="$long_tmp"
    echo "Checking socket suites with a long TMPDIR"
    cargo test --locked --all-targets
    meson test -C "$native" --no-rebuild --print-errorlogs
    cargo run --locked --release --example driver_contract -- "$native/driver/test_daemon_contract"
    CV1TOUCH_TEST_DAEMON="$daemon" CV1TOUCH_TEST_BINARY="$(dirname "$daemon")/cv1touch" CV1TOUCH_TEST_LIBRARY_PATH="$native/third_party/openhmd" python3 -m unittest discover -s tests -p test_runtime_dir.py
    CV1TOUCH_TEST_BINARY="$(dirname "$daemon")/cv1touch" CV1TOUCH_TEST_LIBRARY_PATH="$native/third_party/openhmd" python3 -m unittest discover -s tests -p test_setup.py
    CV1TOUCH_TEST_BINARY="$(dirname "$daemon")/cv1touch" CV1TOUCH_TEST_LIBRARY_PATH="$native/third_party/openhmd" python3 -m unittest discover -s tests -p test_report.py
)
