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
trap 'rm -f "$messages"' EXIT
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
readelf -d "$daemon" | grep -F '$ORIGIN/../lib/cv1touch' >/dev/null
