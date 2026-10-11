# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""Setup CLI regressions using only isolated files and a mock control socket."""
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import threading
import time
import pty
import unittest

BINARY = os.environ.get("CV1TOUCH_TEST_BINARY")
LIBRARY_PATH = os.environ.get("CV1TOUCH_TEST_LIBRARY_PATH")


@unittest.skipUnless(BINARY and LIBRARY_PATH, "requires built CLI and native library path")
class SetupTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="cv1touch-setup-cli-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.env = dict(os.environ, HOME=str(self.root), XDG_CONFIG_HOME=str(self.root / "config"),
                        XDG_STATE_HOME=str(self.root / "state"), XDG_RUNTIME_DIR=str(self.root / "run"),
                        XDG_DATA_HOME=str(self.root / "data"), XDG_CACHE_HOME=str(self.root / "cache"),
                        CV1TOUCH_STEAM_ROOT=str(self.root / "steam"),
                        CV1TOUCH_STEAMVR=str(self.root / "steamvr"), CV1TOUCH_ASSETS=str(self.root / "assets"),
                        LD_LIBRARY_PATH=LIBRARY_PATH)
        self.commands = []
        self.configured = False
        self.loaded_configuration = True
        self.engine_ready = True
        self.awake = False
        self.counter = 10
        self.extra_sensor = False
        self.mutation_checkpoints = []
        self.socket_path = self.root / "run/cv1touch/control.sock"
        self.socket_path.parent.mkdir(parents=True)
        self.server = socket.socket(socket.AF_UNIX)
        self.server.bind(str(self.socket_path))
        self.server.listen()
        self.server.settimeout(0.1)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.serve)
        self.thread.start()
        self.addCleanup(self.shutdown)

    def shutdown(self):
        self.stop.set()
        self.thread.join(timeout=3)
        self.server.close()

    def serve(self):
        while not self.stop.is_set():
            try:
                connection, _ = self.server.accept()
            except socket.timeout:
                continue
            with connection:
                command = connection.makefile("rb").readline().decode().strip()
                self.commands.append(command)
                if command == "status":
                    self.counter += 1
                    sensors = [{"serial": "CAMTEST", "placed": 1, "frames": self.counter,
                                "fixes": [0, 0]}] if self.configured else []
                    if self.extra_sensor:
                        sensors.append({"serial": "CAMNEW", "placed": 0, "frames": self.counter,
                                        "fixes": [0, 0]})
                    reply = {"generation": 1, "engine_ready": self.engine_ready, "mode": "calibrated" if self.configured else "provisional",
                             "recording_scoped_stop": True, "recording": False, "sensor_total": len(sensors), "sensors": sensors, "hands": [],
                             "alignment": {"registration": self.configured and self.loaded_configuration,
                                           "render": [self.configured and self.loaded_configuration] * 2},
                             "input_calibration": {"calibrated": [self.configured and self.loaded_configuration] * 2}}
                elif command == "inputs":
                    reply = {"hands": [{"identity": str(hand) * 32, "connected": self.awake,
                                        "received_host_ns": time.monotonic_ns() if self.awake else 0, "trigger": 0, "grip": 0,
                                        "stick": [0, 0]} for hand in [1, 2]]}
                else:
                    journal = self.root / "state/cv1touch/setup.json"
                    self.mutation_checkpoints.append(json.loads(journal.read_text()) if journal.exists() else None)
                    connection.sendall(b"err forbidden mutation\n")
                    continue
                connection.sendall(("ok " + json.dumps(reply) + "\n").encode())

    def cli(self, *args):
        return subprocess.run([BINARY, "setup", *args], env=self.env, cwd=self.root,
                              input="", capture_output=True, text=True, timeout=15)

    def terminal_guide(self, answers="", expected_code=0):
        master, slave = pty.openpty()
        try:
            process = subprocess.Popen([BINARY, "setup"], env=self.env, cwd=self.root,
                                       stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if answers:
                os.write(master, answers.encode())
            try:
                stdout, stderr = process.communicate(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
                self.fail("guide did not complete without additional captures")
            self.assertEqual(process.returncode, expected_code, stdout + stderr)
            return stdout + stderr
        finally:
            os.close(master)
            os.close(slave)

    def fixtures(self):
        self.configured = self.awake = True
        directory = self.root / "config/cv1touch"
        directory.mkdir(parents=True)
        pose = {"orient_xyzw": [0, 0, 0, 1], "pos": [0, 0, 0]}
        camera = {"format": "cv1touch-calibration", "version": 1, "created_unix_s": 1,
                  "world": "test", "source": "/deleted/exact-test.rec",
                  "sensors": [{"serial": "CAMTEST", **pose, "rms_px": 0.1, "observations": 100}],
                  "stats": {"reference": "CAMTEST", "gravity_mps2": 9.81, "gravity_spread_deg": 0.1,
                            "motion_segments": 10, "samples": 1000, "rejected_views": 0}}
        registration = {"format": "cv1touch-registration", "version": 1, "created_unix_s": 1,
                        "lighthouse_from_world": pose, "universe_id": 1, "touch": "left", "reference": "hmd",
                        "pairs": 1000, "pos_rms_mm": 1, "rot_rms_deg": 0.1, "stations": [], "play_area": None}
        render = {"format": "cv1touch-render-offsets", "version": 1, "created_unix_s": 1, "source": "test",
                  "hands": [{"hand": hand, "model_from_render": pose, "rms_mm": 0.1, "leds": 24}
                            for hand in ["left", "right"]]}
        endpoint = {"released": 0, "pressed": 1}
        stick = {"min": -1, "center": 0, "max": 1}
        inputs = {"format": "cv1touch-input-calibration", "version": 1,
                  "analog_deadzone": 0.05, "stick_deadzone": 0.05,
                  "hands": [{"identity": str(hand) * 32, "trigger": endpoint, "grip": endpoint,
                             "stick": [stick, stick]} for hand in [1, 2]]}
        for name, value in [("calibration", camera), ("registration", registration),
                            ("render-offsets", render), ("input-calibration", inputs)]:
            (directory / (name + ".json")).write_text(json.dumps(value))
        runtime = self.root / "steamvr"
        for name in ["bin/linux64/libopenvr_api.so", "drivers/oculus/resources/input/touch_profile.json"]:
            path = runtime / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"test fixture")
        for hand in ["left", "right"]:
            path = runtime / f"resources/rendermodels/oculus_cv1_controller_{hand}/oculus_cv1_controller_{hand}_body.obj"
            path.parent.mkdir(parents=True)
            path.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
        assets = self.root / "assets"
        assets.mkdir()
        for name in ["driver_cv1touch.so", "driver.vrdrivermanifest"]:
            (assets / name).write_bytes(b"test fixture")
        return directory

    def test_check_is_read_only_and_explains_missing_stages(self):
        before = {p: p.read_bytes() for p in self.root.rglob("*") if p.is_file()}
        result = self.cli("--check", "--json")
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual([stage["stage"] for stage in report["stages"]],
                         ["camera", "gravity", "render", "inputs", "registration"])
        self.assertFalse(report["ready"])
        self.assertTrue(self.commands)
        self.assertTrue(set(self.commands) <= {"status", "inputs"}, self.commands)
        after = {p: p.read_bytes() for p in self.root.rglob("*") if p.is_file()}
        self.assertEqual(before, after)
        self.assertFalse((self.root / "state").exists())

    def test_noninteractive_guide_fails_before_any_daemon_command_or_write(self):
        result = self.cli()
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn("interactive terminal", result.stderr)
        self.assertEqual(self.commands, [])
        self.assertFalse((self.root / "state").exists())

    def test_corrupt_checkpoint_is_not_overwritten_or_silently_ignored(self):
        journal = self.root / "state/cv1touch/setup.json"
        journal.parent.mkdir(parents=True)
        journal.write_text('{"version":999,"completed":{},"pending":[],"recordings":{}}')
        before = journal.read_bytes()
        result = self.cli("--check", "--json")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("setup journal", result.stderr)
        self.assertEqual(journal.read_bytes(), before)
        self.assertEqual(self.commands, [])

    def test_legacy_adoption_and_unchanged_resume_only_write_the_journal(self):
        directory = self.fixtures()
        before = {p: p.read_bytes() for p in directory.iterdir()}
        result = self.cli("--check", "--json")
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        stages = json.loads(result.stdout)["stages"]
        self.assertEqual([r["state"] for r in stages], ["unknown"] * 3 + ["ready", "unknown"])
        self.assertEqual({p: p.read_bytes() for p in directory.iterdir()}, before)
        self.assertFalse((self.root / "state").exists())
        self.terminal_guide("keep\n" * 4)
        journal = self.root / "state/cv1touch/setup.json"
        self.assertEqual(journal.stat().st_mode & 0o777, 0o600)
        self.assertEqual({p: p.read_bytes() for p in directory.iterdir()}, before)
        result = self.cli("--check", "--json")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue(json.loads(result.stdout)["ready"])
        self.assertIn("Saved setup is complete", self.terminal_guide())
        self.assertTrue(set(self.commands) <= {"status", "inputs"}, self.commands)
        self.assertEqual({p: p.read_bytes() for p in directory.iterdir()}, before)

    def test_external_world_change_and_added_sensor_cannot_claim_ready(self):
        directory = self.fixtures()
        self.terminal_guide("keep\n" * 4)
        path = directory / "calibration.json"
        value = json.loads(path.read_text())
        value["sensors"][0]["pos"] = [0.1, 0, 0]
        path.write_text(json.dumps(value))
        report = json.loads(self.cli("--check", "--json").stdout)
        self.assertEqual([r["state"] for r in report["stages"]],
                         ["stale", "stale", "ready", "ready", "stale"])
        self.extra_sensor = True
        report = json.loads(self.cli("--check", "--json").stdout)
        self.assertFalse(report["ready"])
        self.assertIn("need placement", report["stages"][0]["reason"])
        self.assertEqual(next(r for r in report["stages"] if r["stage"] == "registration")["state"], "blocked")
        self.assertTrue(set(self.commands) <= {"status", "inputs"}, self.commands)

    def test_sleeping_hands_reuse_valid_files_without_capture_or_erasure(self):
        directory = self.fixtures()
        self.terminal_guide("keep\n" * 4)
        inputs = (directory / "input-calibration.json").read_bytes()
        self.awake = False
        report = json.loads(self.cli("--check", "--json").stdout)
        self.assertEqual([r["state"] for r in report["stages"]], ["ready"] * 5)
        self.assertFalse(report["ready"])
        self.assertIn("Saved setup is complete", self.terminal_guide())
        self.assertEqual((directory / "input-calibration.json").read_bytes(), inputs)
        self.assertTrue(set(self.commands) <= {"status", "inputs"}, self.commands)

    def test_failed_camera_stage_persists_dependency_invalidation_before_mutation(self):
        directory = self.fixtures()
        self.terminal_guide("keep\n" * 4)
        path = directory / "calibration.json"
        value = json.loads(path.read_text())
        value["sensors"][0]["pos"] = [0.1, 0, 0]
        path.write_text(json.dumps(value))
        before = {p: p.read_bytes() for p in directory.iterdir()}
        output = self.terminal_guide("new\nq\n", expected_code=1)
        self.assertIn("cannot enter camera capture mode", output)
        mutations = [command for command in self.commands if command not in {"status", "inputs"}]
        self.assertEqual(mutations, ["calibration capture",
                                     "calibration load " + json.dumps(str(path)),
                                     "calibration provisional"])
        self.assertIn("calibration reload failed", output)
        self.assertEqual(set(self.mutation_checkpoints[0]["pending"]),
                         {"camera", "gravity", "registration"})
        journal = self.root / "state/cv1touch/setup.json"
        persisted = journal.read_bytes()
        report = json.loads(self.cli("--check", "--json").stdout)
        self.assertEqual([r["state"] for r in report["stages"]],
                         ["stale", "stale", "ready", "ready", "stale"])
        self.assertEqual(journal.read_bytes(), persisted)
        self.assertEqual({p: p.read_bytes() for p in directory.iterdir()}, before)

    def test_invalid_camera_failure_restores_provisional_without_rewriting_artifacts(self):
        directory = self.fixtures()
        self.terminal_guide("keep\n" * 4)
        (directory / "calibration.json").write_text("{malformed camera")
        before = {p: p.read_bytes() for p in directory.iterdir()}
        output = self.terminal_guide("new\nq\n", expected_code=1)
        self.assertIn("cannot enter camera capture mode", output)
        mutations = [command for command in self.commands if command not in {"status", "inputs"}]
        self.assertEqual(mutations, ["calibration capture", "calibration provisional"])
        self.assertEqual({p: p.read_bytes() for p in directory.iterdir()}, before)
        journal = json.loads((self.root / "state/cv1touch/setup.json").read_text())
        self.assertEqual(set(journal["pending"]), {"camera", "gravity", "registration"})

    def test_saved_stages_require_daemon_loaded_configuration_for_live_readiness(self):
        directory = self.fixtures()
        self.terminal_guide("keep\n" * 4)
        before = {p: p.read_bytes() for p in directory.iterdir()}
        journal = self.root / "state/cv1touch/setup.json"
        checkpoint = journal.read_bytes()
        self.loaded_configuration = False
        result = self.cli("--check", "--json")
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual([r["state"] for r in report["stages"]], ["ready"] * 5)
        self.assertFalse(report["ready"])
        loaded = next(check for check in report["checks"] if check["name"] == "tracking_configuration")
        self.assertFalse(loaded["ok"])
        self.assertEqual(journal.read_bytes(), checkpoint)
        self.assertEqual({p: p.read_bytes() for p in directory.iterdir()}, before)
        self.assertTrue(set(self.commands) <= {"status", "inputs"}, self.commands)

    def test_guide_does_not_claim_completion_while_the_daemon_ignores_the_saved_setup(self):
        self.fixtures()
        self.terminal_guide("keep\n" * 4)
        self.loaded_configuration = False
        output = self.terminal_guide("q\n", expected_code=1)
        self.assertIn("not using the saved setup", output)
        self.assertNotIn("Saved setup is complete", output)

    def test_pending_engine_reload_blocks_capture_despite_advancing_old_streams(self):
        directory = self.fixtures()
        self.terminal_guide("keep\n" * 4)
        path = directory / "calibration.json"
        value = json.loads(path.read_text())
        value["sensors"][0]["pos"] = [0.1, 0, 0]
        path.write_text(json.dumps(value))
        before = {p: p.read_bytes() for p in directory.iterdir()}
        journal = self.root / "state/cv1touch/setup.json"
        checkpoint = journal.read_bytes()
        self.engine_ready = False
        result = self.cli("--check", "--json")
        report = json.loads(result.stdout)
        self.assertEqual(report["stages"][0]["state"], "blocked")
        self.assertIn("initializing or reloading", report["stages"][0]["reason"])
        output = self.terminal_guide("q\n", expected_code=1)
        self.assertIn("initializing or reloading", output)
        self.assertEqual(journal.read_bytes(), checkpoint)
        self.assertEqual({p: p.read_bytes() for p in directory.iterdir()}, before)
        self.assertTrue(set(self.commands) <= {"status", "inputs"}, self.commands)

    def test_legacy_daemon_checks_work_but_guided_reload_requires_ready_marker(self):
        directory = self.fixtures()
        self.engine_ready = None
        self.terminal_guide("keep\n" * 4)
        result = self.cli("--check", "--json")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        path = directory / "calibration.json"
        value = json.loads(path.read_text())
        value["sensors"][0]["pos"] = [0.1, 0, 0]
        path.write_text(json.dumps(value))
        before = {p: p.read_bytes() for p in directory.iterdir()}
        journal = self.root / "state/cv1touch/setup.json"
        checkpoint = journal.read_bytes()
        output = self.terminal_guide("new\nq\n", expected_code=1)
        self.assertIn("matching cv1touchd", output)
        self.assertIn("no capture was started", output)
        self.assertEqual(journal.read_bytes(), checkpoint)
        self.assertEqual({p: p.read_bytes() for p in directory.iterdir()}, before)
        self.assertTrue(set(self.commands) <= {"status", "inputs"}, self.commands)

    def test_human_actions_are_bold_when_forced_but_json_and_no_color_stay_plain(self):
        self.fixtures()
        self.env.pop("NO_COLOR", None)
        self.env.update(TERM="xterm-256color", CLICOLOR_FORCE="1")
        checked = self.cli("--check", "--json")
        self.assertNotIn("\x1b", checked.stdout)
        output = self.terminal_guide("keep\n" * 4)
        self.assertIn("\x1b[1mSensor positions (~2 min)", output)
        self.assertIn("Sensor positions    saved earlier, not verified", output)
        self.env["NO_COLOR"] = "1"
        self.assertNotIn("\x1b", self.terminal_guide())
        self.env.pop("NO_COLOR")
        self.env["TERM"] = "dumb"
        self.assertNotIn("\x1b", self.terminal_guide())

    def test_malformed_journal_is_read_only_and_never_contacts_daemon(self):
        journal = self.root / "state/cv1touch/setup.json"
        journal.parent.mkdir(parents=True)
        for payload in ["{", "[]", '{"version":1,"completed":{},"pending":[],"recordings":{},"unexpected":true}']:
            with self.subTest(payload=payload):
                journal.write_text(payload)
                result = self.cli("--check", "--json")
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn("setup journal", result.stderr)
                self.assertEqual(journal.read_text(), payload)
                self.assertEqual(self.commands, [])


if __name__ == "__main__":
    unittest.main()
