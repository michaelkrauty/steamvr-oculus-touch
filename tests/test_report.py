# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""`cv1touch report` with private directories, its zip read back by Python's own zipfile."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import zipfile

BINARY = os.environ.get("CV1TOUCH_TEST_BINARY")
LIBRARY_PATH = os.environ.get("CV1TOUCH_TEST_LIBRARY_PATH")


@unittest.skipUnless(BINARY and LIBRARY_PATH, "requires built CLI and native library path")
class ReportTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="cv1touch-report-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.env = dict(os.environ, HOME=str(self.root), XDG_CONFIG_HOME=str(self.root / "config"),
                        XDG_STATE_HOME=str(self.root / "state"), XDG_RUNTIME_DIR=str(self.root / "run"),
                        XDG_DATA_HOME=str(self.root / "data"), CV1TOUCH_STEAM_ROOT=str(self.root / "steam"),
                        CV1TOUCH_STEAMVR=str(self.root / "steamvr"), LD_LIBRARY_PATH=LIBRARY_PATH)
        self.config = self.root / "config/cv1touch"
        self.state = self.root / "state/cv1touch"
        self.config.mkdir(parents=True)
        (self.state / "logs").mkdir(parents=True)
        (self.root / "steam/logs").mkdir(parents=True)

    def report(self, out, success=True):
        result = subprocess.run([BINARY, "report", "--out", str(out)], env=self.env, cwd=self.root,
                                capture_output=True, text=True, timeout=120)
        self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
        return result

    def test_logs_settings_and_a_summary_in_one_zip(self):
        (self.config / "settings.json").write_text('{"prediction_percent": 0}\n')
        (self.config / "calibration.json.bak-1").write_text("an old calibration\n")
        (self.state / "imu-biases.json").write_text("{}\n")
        (self.state / "capture.rec").write_bytes(b"\0" * 1000)
        (self.state / "logs/cv1touchd.log").write_text("2026-10-14T17:05:03.123Z cv1touchd: started\n")
        (self.state / "logs/cv1touchd.1.log").write_text("an earlier run\n")
        (self.root / "steam/logs/vrserver.txt").write_text("driver_cv1touch: connected\n")
        out = self.root / "report.zip"
        printed = self.report(out).stdout
        self.assertIn("Send that file to whoever is helping you", printed)
        self.assertIn("your network addresses", printed)
        self.assertEqual(out.stat().st_mode & 0o777, 0o600)
        with zipfile.ZipFile(out) as archive:
            self.assertIsNone(archive.testzip())
            names = set(archive.namelist())
            for name in ("report.txt", "config/settings.json", "state/imu-biases.json",
                         "logs/cv1touchd.log", "logs/cv1touchd.1.log", "steamvr/vrserver.txt"):
                self.assertIn(name, names)
            # backups and recordings are listed, not sent
            self.assertFalse(any("bak" in n or n.endswith(".rec") for n in names), names)
            self.assertEqual(archive.read("config/settings.json"), b'{"prediction_percent": 0}\n')
            self.assertEqual(archive.read("steamvr/vrserver.txt"), b"driver_cv1touch: connected\n")
            summary = archive.read("report.txt").decode()
        for section in ("Checks (cv1touch doctor)", "Oculus USB devices (vendor 2833)", "Programs",
                        "Daemon status", "Setup (cv1touch setup --check --json)"):
            self.assertIn(section, summary)
        self.assertIn("FAIL daemon", summary)
        self.assertIn("cv1touchd: not answering", summary)
        self.assertIn("calibration.json.bak-1  19 bytes", summary)
        self.assertIn("capture.rec  1000 bytes", summary)
        # this boot's journal of the daemon (the real user's: journalctl ignores HOME), or why not
        self.assertTrue("logs/journal.txt" in names or "the daemon's journal:" in summary)

    def test_an_existing_file_is_left_alone(self):
        out = self.root / "report.zip"
        out.write_bytes(b"someone else's file")
        error = self.report(out, success=False).stderr
        self.assertIn(f"There is already a file at {out}", error)
        self.assertIn("--out", error)
        self.assertEqual(out.read_bytes(), b"someone else's file")


if __name__ == "__main__":
    unittest.main()
