# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""Exercise the actual report and rig callers without native acquisition.

Run with python3 tests/test_eval_no_data.py. Cargo's target directory and build
limits can be supplied by the caller. All generated sources and JSON use a
TemporaryDirectory; production functions are extracted unchanged, not copied.
The caller crate reuses the production dependency lock and Cargo target directory.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def production_function(source, name):
    start = source.index(f"fn {name}(")
    # Top-level function closing braces are unindented.
    end = source.index("\n}", start) + 2
    return source[start:end]


class EvaluationNoDataTests(unittest.TestCase):
    def test_report_and_rig_missing_truth_and_valid_controls(self):
        source = (ROOT / "src/bin/cv1touch.rs").read_text()
        with tempfile.TemporaryDirectory(prefix="cv1touch-eval-test-", dir=ROOT) as directory:
            root = Path(directory)
            (root / "src").mkdir()
            (root / "Cargo.toml").write_text(
                '[package]\nname = "eval-caller-regression"\nversion = "0.0.0"\nedition = "2021"\n'
                '[dependencies]\ncv1touch = { path = ' + json.dumps(str(ROOT)) +
                ', default-features = false }\nnalgebra = "0.33"\nserde_json = "1"\n'
            )
            # Cargo adds this caller package while retaining production dependency versions.
            (root / "Cargo.lock").write_bytes((ROOT / "Cargo.lock").read_bytes())
            fixture = (ROOT / "tests/data/eval_no_data.rs").read_text()
            (root / "src/main.rs").write_text(
                fixture + "\n" + production_function(source, "eval_report_with") +
                "\n" + production_function(source, "rig_pair")
            )
            env = dict(os.environ)
            env.setdefault("CARGO_TARGET_DIR", str(ROOT / "target"))
            result = subprocess.run(
                ["cargo", "run", "--offline", "--quiet", "--manifest-path", str(root / "Cargo.toml"),
                 "--", str(root)], capture_output=True, text=True, env=env, cwd=ROOT,
                timeout=900,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            print(result.stdout)
            sections = {}
            for section in result.stdout.split("CASE ")[1:]:
                name, output = section.split("\n", 1)
                sections[name] = output
            for name in ["short", "no-moving", "rig-missing"]:
                output = sections[name]
                with self.subTest(case=name):
                    self.assertIn("lag: unavailable", output)
                    self.assertNotIn("lag removed:", output)
                    self.assertNotIn("cv1touch latency", output)
                    self.assertNotIn("Touch stayed fixed", output)
                    self.assertNotIn("best-fit registration differs", output)
            for name in ["continuous", "partial"]:
                output = sections[name]
                with self.subTest(case=name):
                    self.assertNotIn("unavailable", output)
                    self.assertRegex(output, r"lag: -?0\.0 ms")
                    self.assertIn("moving, lag removed:", output)
                    self.assertNotIn("cv1touch latency", output)
                    self.assertNotIn("Touch stayed fixed", output)
            self.assertIn("(6000 poses)", sections["continuous"])
            self.assertIn("(1001 poses)", sections["partial"])
            self.assertRegex(sections["rig-valid"], r"lag\s+[+-]0\.0 ms")
            self.assertNotIn("unavailable", sections["rig-valid"])


if __name__ == "__main__":
    unittest.main()
