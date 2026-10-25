#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
# Build and run hardware-independent Rust and native regressions.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
python3 -m unittest discover -s tests -p test_release.py
python3 -m unittest discover -s tests -p test_windows_bundle.py
python3 -m unittest discover -s tests -p test_install.py
export CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-4}"
cargo fmt --all --check
cargo clippy --locked --all-targets -- -D warnings
cargo test --locked --all-targets
messages=$(mktemp)
socket_tests=
trap 'rm -f -- "$messages"; rm -rf -- "${socket_tests:-}"' EXIT
# On Linux, audit every intentional abort. The cargo test above already built the
# test harness; --no-run only reprints its artifact path and the native directory
# its build script compiled in (CV1TOUCH_NATIVE_DIR), so the audit runs the native
# abort fixtures from the same engine build the harness uses.
if [ "$(uname -s)" = Linux ]; then
  cargo test --locked --all-targets --no-run --message-format=json > "$messages"
  audit=$(python3 - "$messages" <<'PY'
import json, sys
from pathlib import Path
harnesses, natives = set(), set()
for line in Path(sys.argv[1]).read_text().splitlines():
    event = json.loads(line)
    target = event.get("target", {})
    if (event.get("reason") == "compiler-artifact"
            and target.get("name") == "cv1touchd"
            and "bin" in target.get("kind", [])
            and event.get("profile", {}).get("test") is True
            and event.get("executable")):
        harnesses.add(event["executable"])
    if event.get("reason") == "build-script-executed":
        natives.update(value for name, value in event.get("env", [])
                       if name == "CV1TOUCH_NATIVE_DIR")
if len(harnesses) != 1:
    raise SystemExit("Cargo did not report exactly one cv1touchd test executable")
if len(natives) != 1:
    raise SystemExit("Cargo did not report exactly one native directory for the tests")
print(harnesses.pop())
print(natives.pop())
PY
)
  daemon_test=${audit%%$'\n'*}
  native_test=${audit#*$'\n'}
  for native_bin in \
    tests/test_native_log_reentry \
    tests/test_native_assert_routed \
    tests/test_native_assert_standalone
  do
    if [ ! -x "$native_test/$native_bin" ]; then
      echo "expected-abort audit: missing $native_test/$native_bin" >&2
      exit 1
    fi
  done
  python3 tests/check_expected_aborts.py "$daemon_test" "$native_test"
fi
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
