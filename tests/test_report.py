# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""`cv1touch report` with private directories, its zip read back by Python's own zipfile."""
import os
from pathlib import Path
import struct
import subprocess
from socket_dir import SocketDirectory
import unittest
from unittest import mock
import zipfile

BINARY = os.environ.get("CV1TOUCH_TEST_BINARY")
LIBRARY_PATH = os.environ.get("CV1TOUCH_TEST_LIBRARY_PATH")


@unittest.skipUnless(BINARY and LIBRARY_PATH, "requires built CLI and native library path")
class ReportTests(unittest.TestCase):
    def setUp(self):
        temporary = SocketDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.journal = b"fixture: cv1touchd started\nfixture: controller waiting\n"
        self.mock_bin = self.root / "bin"
        self.mock_bin.mkdir()
        journalctl = self.mock_bin / "journalctl"
        journalctl.write_text("#!/bin/sh\nprintf '%s\\n' 'fixture: cv1touchd started' "
                              "'fixture: controller waiting'\n")
        journalctl.chmod(0o755)
        pgrep = self.mock_bin / "pgrep"
        pgrep.write_text("#!/bin/sh\nexit 1\n")
        pgrep.chmod(0o755)
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
        env = dict(self.env, PATH=str(self.mock_bin))
        result = subprocess.run([BINARY, "report", "--out", str(out)], env=env, cwd=self.root,
                                capture_output=True, text=True, timeout=120)
        self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
        return result

    def test_logs_settings_and_a_summary_in_one_zip(self):
        (self.config / "settings.json").write_text('{"prediction_percent": 0}\n')
        (self.config / "calibration.json.bak-1").write_text("an old calibration\n")
        (self.state / "imu-biases.json").write_text("{}\n")
        (self.state / "capture.rec").write_bytes(b"\0" * 1000)
        for t in (1000, 1001, 1002, 1003):
            (self.state / f"dropout-{t}-right.rec").write_bytes(b"CV1TREC\0" + bytes([t % 256]) * 100)
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
            # backups and recordings are listed, not sent: but for the newest three dropouts'
            dropouts = {f"state/dropout-{t}-right.rec" for t in (1001, 1002, 1003)}
            self.assertTrue(dropouts <= names, names)
            self.assertEqual(archive.read("state/dropout-1003-right.rec"), b"CV1TREC\0" + bytes([1003 % 256]) * 100)
            self.assertFalse(any("bak" in n or (n.endswith(".rec") and n not in dropouts) for n in names), names)
            self.assertEqual(archive.read("config/settings.json"), b'{"prediction_percent": 0}\n')
            self.assertEqual(archive.read("steamvr/vrserver.txt"), b"driver_cv1touch: connected\n")
            summary = archive.read("report.txt").decode()
            self.assertIn("logs/journal.txt", names)
            self.assertTrue(archive.read("logs/journal.txt") == self.journal,
                            "the archive must contain the private journal fixture")
        for section in ("Checks (cv1touch doctor)", "Oculus USB devices (vendor 2833)", "Programs",
                        "Daemon status", "Setup (cv1touch setup --check --json)"):
            self.assertIn(section, summary)
        self.assertIn("FAIL daemon", summary)
        self.assertIn("sensor_streams: skipped: daemon isn't running", summary)
        self.assertIn("cv1touchd: not answering", summary)
        self.assertIn("SteamVR (vrserver): not running", summary)
        self.assertIn("calibration.json.bak-1  19 bytes", summary)
        self.assertIn("capture.rec  1000 bytes", summary)

    def test_an_interrupted_log_rotation_is_listed_by_name_and_size(self):
        logs = self.state / "logs"
        staging = logs / "cv1touchd.log.rotation"
        staging.mkdir()
        (staging / "cv1touchd.log").write_text("an old log\n")
        (staging / "unrelated").write_text("not a log")
        (logs / "cv1touchd.log").write_text("the current log\n")
        out = self.root / "report.zip"
        self.report(out)
        with zipfile.ZipFile(out) as archive:
            summary = archive.read("report.txt").decode()
            # names and sizes only: the staged files are not sent
            self.assertFalse(any("rotation" in n or "unrelated" in n for n in archive.namelist()))
        self.assertIn(f"{logs}:", summary)
        self.assertIn("  cv1touchd.log  16 bytes", summary)
        self.assertIn("  cv1touchd.log.rotation/", summary)
        self.assertIn(f"{staging}:", summary)
        self.assertIn("  cv1touchd.log  11 bytes", summary)
        self.assertIn("  unrelated  9 bytes", summary)
        self.assertNotIn("not a log", summary)

    def test_a_missing_or_non_executable_stub_cannot_reach_the_host_journal(self):
        host_bin = self.root / "host-bin"
        host_bin.mkdir()
        decoy = host_bin / "journalctl"
        decoy.write_text("#!/bin/sh\nprintf '%s\\n' called > \"${0%/*}/ran\"\n"
                         "printf '%s\\n' 'decoy journal'\n")
        decoy.chmod(0o755)
        marker = host_bin / "ran"
        stub = self.mock_bin / "journalctl"
        contents = stub.read_bytes()
        for broken in ("missing", "not-executable"):
            with self.subTest(stub=broken):
                marker.unlink(missing_ok=True)
                if broken == "missing":
                    stub.unlink()
                else:
                    stub.chmod(0o600)
                try:
                    out = self.root / f"{broken}.zip"
                    with mock.patch.dict(os.environ, PATH=str(host_bin)):
                        self.report(out)
                    self.assertFalse(marker.exists(), "the host journal command must never run")
                    with zipfile.ZipFile(out) as archive:
                        self.assertNotIn("logs/journal.txt", archive.namelist())
                        self.assertIn("the daemon's journal:", archive.read("report.txt").decode())
                finally:
                    stub.write_bytes(contents)
                    stub.chmod(0o755)

    def test_recording_summaries_read_old_and_skip_count_headers(self):
        path = self.root / "synthetic.rec"
        payload = struct.pack("<I", 1) + bytes([12]) + bytes(63)
        record = struct.pack("<HHIQ", 2, 0, len(payload), 1000) + payload
        for skipped in (0, 7):
            with self.subTest(skipped=skipped):
                path.write_bytes(struct.pack("<8sIIQQ", b"CV1TREC\0", 2, 32, 1000, skipped) + record)
                result = subprocess.run([BINARY, "rec-summary", str(path)], env=self.env,
                                        capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("{2: 1}", result.stdout)
                if skipped:
                    self.assertIn("flight recorder: 7 reports or commands skipped in the saved window", result.stdout)
                else:
                    self.assertNotIn("skipped", result.stdout)

    def test_an_unknown_skip_count_is_labelled_in_summaries(self):
        path = self.root / "unknown-skips.rec"
        path.write_bytes(struct.pack("<8sIIQQ", b"CV1TREC\0", 2, 32, 1000, (1 << 64) - 1))
        result = subprocess.run([BINARY, "rec-summary", str(path)], env=self.env,
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("flight recorder: some reports or commands may be missing from the saved window", result.stdout)
        self.assertNotIn(str((1 << 64) - 1), result.stdout)

    def test_an_existing_file_is_left_alone(self):
        out = self.root / "report.zip"
        out.write_bytes(b"someone else's file")
        error = self.report(out, success=False).stderr
        self.assertIn(f"There is already a file at {out}", error)
        self.assertIn("--out", error)
        self.assertEqual(out.read_bytes(), b"someone else's file")


if __name__ == "__main__":
    unittest.main()
