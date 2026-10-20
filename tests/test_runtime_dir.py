# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""Exercise startup diagnostics before the daemon creates state or opens hardware."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import zipfile

BINARY = os.environ.get("CV1TOUCH_TEST_DAEMON")
CLI = os.environ.get("CV1TOUCH_TEST_BINARY")
LIBRARY_PATH = os.environ.get("CV1TOUCH_TEST_LIBRARY_PATH")


@unittest.skipUnless(BINARY and LIBRARY_PATH, "requires built daemon and native library path")
class ControlSocketPathTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="cv1touch-path-test-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.env = dict(os.environ, HOME=str(self.root / "home"),
                        XDG_CONFIG_HOME=str(self.root / "config"),
                        XDG_STATE_HOME=str(self.root / "state"),
                        XDG_RUNTIME_DIR=str(self.root / "run"), LD_LIBRARY_PATH=LIBRARY_PATH)
        self.env.pop("CV1TOUCH_CPUS", None)

    def check_startup(self, endpoint, arguments, env, setting, other, supervise=False):
        # If path validation regresses, this invalid name still stops startup before
        # a shared-memory segment or USB device can be opened. All state stays private.
        mode = ["--supervise"] if supervise else []
        result = subprocess.run([BINARY, *mode, "--provisional", "--shm-name", "/invalid/test", *arguments],
                                env=env, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        pieces = []
        for line in result.stderr.splitlines():
            for marker in ("control socket path: ", "control socket path (continued): "):
                if marker in line:
                    pieces.append(line.split(marker, 1)[1])
                    break
        self.assertEqual("".join(pieces), str(endpoint), result.stderr)
        self.assertIn(f"{len(os.fsencode(endpoint))} bytes", result.stderr)
        self.assertIn("at most 107 bytes", result.stderr)
        self.assertIn("too long", result.stderr)
        self.assertIn("cv1touchd: cannot start: the path of its control socket", result.stderr)
        # It names the setting that produced the path, not the one that did not.
        self.assertIn(setting, result.stderr)
        self.assertNotIn(other, result.stderr)
        self.assertNotIn("SUN_LEN", result.stderr)
        self.assertEqual(list(self.root.iterdir()), [], "invalid path created runtime state")

    def test_a_long_runtime_override_explains_how_to_shorten_the_socket_path(self):
        for name in ("r" * 95, "é" * 48):
            with self.subTest(name=name):
                runtime = self.root / name
                self.check_startup(runtime / "control.sock", ["--runtime-dir", str(runtime)], self.env,
                                   "--runtime-dir", "XDG_RUNTIME_DIR")

    def test_a_long_xdg_runtime_dir_explains_how_to_shorten_the_socket_path(self):
        for name in ("r" * 95, "é" * 48):
            with self.subTest(name=name):
                runtime = self.root / name
                env = dict(self.env, XDG_RUNTIME_DIR=str(runtime))
                self.check_startup(runtime / "cv1touch/control.sock", [], env,
                                   "XDG_RUNTIME_DIR", "--runtime-dir")

    def test_a_long_supervisor_runtime_override_is_rejected_before_state_is_created(self):
        # Oversized components also stop an old supervisor before it can launch a child.
        for name in ("r" * 256, "é" * 1100):
            with self.subTest(name=name):
                runtime = self.root / name
                self.check_startup(runtime / "control.sock", ["--runtime-dir", str(runtime)],
                                   self.env, "--runtime-dir", "XDG_RUNTIME_DIR", supervise=True)

    def test_a_long_supervisor_xdg_runtime_dir_is_rejected_before_state_is_created(self):
        for name in ("r" * 256, "é" * 1100):
            with self.subTest(name=name):
                runtime = self.root / name
                env = dict(self.env, XDG_RUNTIME_DIR=str(runtime))
                self.check_startup(runtime / "cv1touch/control.sock", [], env,
                                   "XDG_RUNTIME_DIR", "--runtime-dir", supervise=True)

    def test_a_rejected_supervisor_path_does_not_wait_for_stderr(self):
        read, write = os.pipe()
        self.addCleanup(os.close, read)
        self.addCleanup(os.close, write)
        os.set_blocking(write, False)
        while True:
            try:
                os.write(write, b"x" * 4096)
            except BlockingIOError:
                break
        os.set_blocking(write, True)
        runtime = self.root / ("r" * 256)
        # The invalid shared-memory name also prevents hardware access if validation regresses.
        child = subprocess.Popen(
            [BINARY, "--supervise", "--provisional", "--shm-name", "/invalid/test",
             "--runtime-dir", str(runtime)], env=self.env,
            stdout=subprocess.DEVNULL, stderr=write)
        try:
            status = child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
            self.fail("rejecting the path waited for the supervisor's stderr")
        self.assertEqual(status, 1)
        self.assertTrue(os.get_blocking(write), "the supervisor changed its stderr mode")
        self.assertEqual(list(self.root.iterdir()), [], "invalid path created runtime state")

    def test_a_large_runtime_environment_value_is_logged_in_full(self):
        # Below Linux's 128 KiB per-string argument/environment limit, but larger
        # than the logger queue if the path is split into unnecessarily small lines.
        runtime = self.root / ("r" * 120_000)
        env = dict(self.env, XDG_RUNTIME_DIR=str(runtime))
        self.check_startup(runtime / "cv1touch/control.sock", [], env,
                                   "XDG_RUNTIME_DIR", "--runtime-dir")


@unittest.skipUnless(CLI and LIBRARY_PATH, "requires built CLI and native library path")
class CliSocketPathTests(unittest.TestCase):
    """With a runtime directory too long for a socket, the commands that talk to the daemon say
    why they cannot, in words a person can act on. Nothing here can reach a running daemon."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="cv1touch-cli-path-test-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.runtime = self.root / ("r" * 95)
        self.endpoint = self.runtime / "cv1touch/control.sock"
        self.env = dict(os.environ, HOME=str(self.root / "home"),
                        XDG_CONFIG_HOME=str(self.root / "config"),
                        XDG_DATA_HOME=str(self.root / "data"),
                        XDG_STATE_HOME=str(self.root / "state"),
                        XDG_RUNTIME_DIR=str(self.runtime), LD_LIBRARY_PATH=LIBRARY_PATH,
                        CV1TOUCH_STEAM_ROOT=str(self.root / "steam"),
                        CV1TOUCH_STEAMVR=str(self.root / "steamvr"),
                        CV1TOUCH_ASSETS=str(self.root / "assets"))

    def run_cli(self, *arguments, input=""):
        try:
            result = subprocess.run([CLI, *arguments], env=self.env, input=input,
                                    capture_output=True, text=True, timeout=5)
        except subprocess.TimeoutExpired:
            self.fail(f"{arguments} waited instead of explaining the unaddressable socket")
        self.assertEqual(list(self.root.iterdir()), [], "the command created state")
        return result

    def assert_explains(self, text):
        self.assertIn("too long", text)
        self.assertIn(f"{len(os.fsencode(self.endpoint))} bytes", text)
        self.assertIn("at most 107 bytes", text)
        self.assertIn("XDG_RUNTIME_DIR", text)
        self.assertIn(str(self.endpoint), text)
        self.assertNotIn("invalid control socket path", text)

    def test_commands_that_ask_the_daemon_explain_the_path(self):
        # Include each request-sending command family, including path-carrying requests.
        for command in (
                ["status"], ["status", "--shm", "/unused-test-segment"], ["status", "--json"],
                ["stop"], ["inputs", "--json"], ["inputs", "reload"], ["alignment", "reload"],
                ["calibration", "capture"], ["calibration", "provisional"],
                ["calibration", "load", str(self.root / "calibration.json")],
                ["haptic", "left", "0.1", "160", "0.5"], ["test-inject"],
                ["inject", "left", "x", "10"],
                ["record", "start", str(self.root / "capture.rec")],
                ["record", "start", str(self.root / "capture.rec"), "--frames"], ["record", "stop"],
                ["shaping"], ["universe-watch"], ["latency"], ["prediction"],
                ["prediction", "position", "100"], ["prediction", "rotation", "100"], ["tilt"],
                ["calibrate"], ["calibrate", "--out", str(self.root / "calibration.json")],
                ["calibrate", "inputs"], ["calibrate", "inputs", "--reset"],
                ["calibrate", "inputs", "--out", str(self.root / "inputs.json")],
                ["register"], ["register", "--check"], ["register", "--force"],
                ["register", "--jig"], ["register", "--headset"],
                ["eval-capture"], ["eval-capture", "--free", "1"]):
            with self.subTest(command=command):
                # Input calibration asks for Enter before its first raw-input request.
                result = self.run_cli(*command, input="\n")
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assert_explains(result.stderr)

    def test_setup_preserves_the_daemon_error_in_text_and_json_checks(self):
        for command in (["setup", "--check"], ["setup", "--check", "--json"]):
            with self.subTest(command=command):
                result = self.run_cli(*command)
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assert_explains(result.stdout)

    def test_multibyte_runtime_directory_counts_bytes_in_the_cli(self):
        self.runtime = self.root / ("é" * 48)
        self.endpoint = self.runtime / "cv1touch/control.sock"
        self.env["XDG_RUNTIME_DIR"] = str(self.runtime)
        result = self.run_cli("inputs", "--json")
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assert_explains(result.stderr)

    def test_report_keeps_the_daemon_error(self):
        path = self.root / "report.zip"
        result = subprocess.run([CLI, "report", "--out", str(path)], env=self.env,
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(list(self.root.iterdir()), [path], "the report created other state")
        with zipfile.ZipFile(path) as report:
            self.assert_explains(report.read("report.txt").decode())

    def test_doctor_explains_the_failed_daemon_check(self):
        for command in (["doctor"], ["doctor", "--json"]):
            with self.subTest(command=command):
                result = self.run_cli(*command)
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assert_explains(result.stdout)
