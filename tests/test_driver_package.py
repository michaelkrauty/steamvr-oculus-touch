# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""Exercise the built CLI and installer with private paths and stubbed SteamVR tools."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

BINARY = os.environ.get("CV1TOUCH_TEST_BINARY")
LIBRARY_PATH = os.environ.get("CV1TOUCH_TEST_LIBRARY_PATH")
ROOT = Path(__file__).resolve().parents[1]
# A made-up input profile laid out as SteamVR's Oculus Touch profile is (not SteamVR's own file).
STEAMVR_PROFILE = """{
  "jsonid" : "input_profile",
  "controller_type" : "oculus_touch",
  "resource_root" : "example",
  "driver_name" : "example",
  "remapping" : "{example}/input/example_remapping.json",
  "input_source" :
  {
    "/input/a" : { "type" : "button", "side" : "right", "binding_image_point" : [ 7, 8 ], "click" : true }
  },
  "default_bindings" : [ { "app_key" : "example.dashboard", "binding_url" : "example_dashboard.json" } ]
}
"""


def driver_profile(text):
    """SteamVR's profile `text` made over into the controllers' own, written out by hand: the
    edits outside input_source that `cv1touch steamvr install` makes, and no others."""
    return (text.replace('"controller_type" : "oculus_touch",', '"controller_type" : "cv1touch_touch",\n  "compatibility_mode_controller_type" : "oculus_touch",', 1)
            .replace('"resource_root" : "example"', '"resource_root" : "cv1touch"', 1)
            .replace('"driver_name" : "example"', '"driver_name" : "cv1touch"', 1)
            .replace('  "remapping" : "{example}/input/example_remapping.json",\n', "", 1))


@unittest.skipUnless(BINARY and LIBRARY_PATH, "requires built CLI and native library path")
class DriverPackageTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="cv1touch-driver-test-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.assets = self.root / "assets"
        self.assets.mkdir()
        (self.assets / "driver_cv1touch.so").write_bytes(b"original driver")
        (self.assets / "driver.vrdrivermanifest").write_text('{"name":"cv1touch"}')
        mock_bin = self.root / "mock-bin"
        mock_bin.mkdir()
        (mock_bin / "pgrep").write_text("#!/bin/sh\nexit 1\n")
        (mock_bin / "pgrep").chmod(0o755)
        runtime_bin = self.root / "steamvr/bin"
        runtime_bin.mkdir(parents=True)
        stub = runtime_bin / "vrpathreg.sh"
        stub.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$CV1TOUCH_STUB_LOG"\n')
        stub.chmod(0o755)
        self.log = self.root / "vrpathreg.log"
        self.env = dict(os.environ, HOME=str(self.root), XDG_CONFIG_HOME=str(self.root / "config"),
                        XDG_DATA_HOME=str(self.root / "data"), XDG_STATE_HOME=str(self.root / "state"),
                        XDG_RUNTIME_DIR=str(self.root / "run"), CV1TOUCH_ASSETS=str(self.assets),
                        CV1TOUCH_STEAMVR=str(self.root / "steamvr"), CV1TOUCH_STEAM_ROOT=str(self.root / "steam"),
                        CV1TOUCH_STUB_LOG=str(self.log), LD_LIBRARY_PATH=LIBRARY_PATH,
                        PATH=str(mock_bin) + ":" + os.environ["PATH"])
        self.package = self.root / "data/cv1touch/steamvr/cv1touch"
        self.steamvr_input = self.root / "steamvr/drivers/oculus/resources/input"
        self.steamvr_input.mkdir(parents=True)
        (self.steamvr_input / "touch_profile.json").write_text(STEAMVR_PROFILE)
        (self.steamvr_input / "example_dashboard.json").write_text('{"bindings": {}}')

    def cli(self, *args, success=True):
        result = subprocess.run([BINARY, "steamvr", *args], env=self.env, cwd=self.root, capture_output=True, text=True)
        self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
        return result

    def log_text(self):
        return self.log.read_text() if self.log.exists() else ""

    def make_legacy(self):
        """Turns the package into one an early cv1touch left: library and manifest, no record."""
        (self.package / ".cv1touch-owned.json").unlink()
        shutil.rmtree(self.package / "resources")

    def doctor_profile_check(self):
        result = subprocess.run([BINARY, "doctor", "--json"], env=self.env, cwd=self.root, capture_output=True, text=True, timeout=30)
        return {c["name"]: c for c in json.loads(result.stdout)["checks"]}["input_profile"]

    def test_install_makes_the_input_profile_from_steamvrs_and_uninstall_removes_it(self):
        self.cli("install")
        profile = (self.package / "resources/input/touch_profile.json").read_text()
        made = json.loads(profile)
        # the controllers' own type, which uses games' Oculus Touch bindings; SteamVR ignores
        # compatibility mode in a profile with a remapping
        self.assertEqual(made["controller_type"], "cv1touch_touch")
        self.assertEqual(made["compatibility_mode_controller_type"], "oculus_touch")
        self.assertNotIn("remapping", made)
        self.assertEqual((made["resource_root"], made["driver_name"]), ("cv1touch", "cv1touch"))
        added = made["input_source"]["/input/home"]
        self.assertEqual(added, {"type": "button", "side": "right", "binding_image_point": [72, 39], "click": True, "order": 0})
        # SteamVR's text with those edits and the one source added, and the binding it names beside it
        start = profile.index('\n    "/input/home"')
        end = profile.index("},", start) + len("},")
        self.assertEqual(profile[:start] + profile[end:], driver_profile(STEAMVR_PROFILE))
        self.assertEqual((self.package / "resources/input/example_dashboard.json").read_text(), '{"bindings": {}}')
        names = json.loads((self.package / "resources/localization/localization.json").read_text())
        self.assertEqual(names, [{"language_tag": "en_US", "cv1touch_touch": "Oculus Touch (cv1touch)",
                                  "/input/home": "Oculus Button", "/input/thumbrest": "Thumbrest"}])
        self.assertTrue(self.doctor_profile_check()["ok"])
        self.cli("uninstall")
        self.assertFalse(self.package.exists())

    def test_an_existing_right_home_button_is_kept_and_its_bindings_copied(self):
        home = '    "/input/home" : { "type" : "button", "side" : "right", "click" : true, "binding_image_point" : [ 11, 12 ] },\n'
        text = STEAMVR_PROFILE.replace('  {\n    "/input/a"', "  {\n" + home + '    "/input/a"', 1)
        (self.steamvr_input / "touch_profile.json").write_text(text)
        self.cli("check")
        self.cli("install")
        installed = (self.package / "resources/input/touch_profile.json").read_text()
        self.assertEqual(installed, driver_profile(text))
        self.assertEqual((self.package / "resources/input/example_dashboard.json").read_text(), '{"bindings": {}}')
        self.assertTrue(self.doctor_profile_check()["ok"])

    def test_an_undrivable_home_button_refuses_before_external_calls(self):
        text = STEAMVR_PROFILE.replace(
            '"/input/a" : { "type" : "button", "side" : "right", "binding_image_point" : [ 7, 8 ], "click" : true }',
            '"/input/home" : { "type" : "button", "side" : "left", "click" : true }',
        )
        (self.steamvr_input / "touch_profile.json").write_text(text)
        for command in ("check", "install"):
            with self.subTest(command=command):
                result = self.cli(command, success=False)
                self.assertIn("already has /input/home", result.stderr)
                self.assertIn("/input/home/click", result.stderr)
                self.assertNotIn("scripts/install.py", result.stderr)
        self.assertEqual(self.log_text(), "")
        self.assertFalse(self.package.exists())

    def test_another_controllers_profile_refuses_before_external_calls(self):
        (self.steamvr_input / "touch_profile.json").write_text(STEAMVR_PROFILE.replace('"oculus_touch"', '"example_touch"', 1))
        for command in ("check", "install"):
            with self.subTest(command=command):
                result = self.cli(command, success=False)
                self.assertIn('its controller_type is "example_touch", not oculus_touch', result.stderr)
                self.assertIn("update cv1touch", result.stderr)
        self.assertEqual(self.log_text(), "")
        self.assertFalse(self.package.exists())

    def test_uninstall_works_without_steamvrs_profile(self):
        self.cli("install")
        (self.steamvr_input / "touch_profile.json").unlink()
        self.cli("uninstall")
        self.assertFalse(self.package.exists())

    def test_home_button_uses_the_menu_buttons_image_point(self):
        text = STEAMVR_PROFILE.replace(
            '"/input/a" : { "type" : "button", "side" : "right", "binding_image_point" : [ 7, 8 ], "click" : true }',
            '"/input/system" : { "type" : "button", "side" : "left", "binding_image_point" : [ 75, 40 ], "click" : true }',
        )
        (self.steamvr_input / "touch_profile.json").write_text(text)
        self.cli("install")
        added = json.loads((self.package / "resources/input/touch_profile.json").read_text())["input_source"]["/input/home"]
        self.assertEqual(added["binding_image_point"], [75, 40])

    def test_missing_steamvr_profile_refuses_install_before_external_calls(self):
        (self.steamvr_input / "touch_profile.json").unlink()
        for command in ("check", "install"):
            with self.subTest(command=command):
                result = self.cli(command, success=False)
                self.assertIn("SteamVR's Oculus Touch input profile is missing", result.stderr)
                self.assertIn("install SteamVR, or have Steam verify SteamVR's files", result.stderr)
                self.assertIn("run `cv1touch steamvr install`", result.stderr)
                self.assertNotIn("scripts/install.py", result.stderr)
        self.assertEqual(self.log_text(), "")
        self.assertFalse(self.package.exists())
        check = self.doctor_profile_check()
        self.assertFalse(check["ok"])
        self.assertIn("SteamVR's Oculus Touch input profile is missing", check["detail"])

    def test_missing_steamvr_runtime_names_the_folder(self):
        shutil.rmtree(self.root / "steamvr")
        for command in ("check", "install"):
            with self.subTest(command=command):
                result = self.cli(command, success=False)
                self.assertIn(f"SteamVR was not found at {self.root / 'steamvr'}", result.stderr)
                self.assertIn("install it from Steam", result.stderr)
                self.assertIn("start it once", result.stderr)
                self.assertIn("CV1TOUCH_STEAMVR", result.stderr)
                self.assertIn("run `cv1touch steamvr install`", result.stderr)
                self.assertNotIn("verify SteamVR's files", result.stderr)
                self.assertNotIn("vrpathreg missing", result.stderr)
                self.assertNotIn("scripts/install.py", result.stderr)
        self.assertEqual(self.log_text(), "")
        self.assertFalse(self.package.exists())
        check = self.doctor_profile_check()
        self.assertFalse(check["ok"])
        self.assertIn("SteamVR was not found at", check["detail"])
        self.assertIn("install it from Steam", check["detail"])
        self.assertNotIn("verify SteamVR's files", check["detail"])
        _, command, env = self.installer()
        installed = subprocess.run(command, env=env, capture_output=True, text=True)
        self.assertNotEqual(installed.returncode, 0, installed.stdout + installed.stderr)
        self.assertIn(f"SteamVR was not found at {self.root / 'steamvr'}", installed.stderr)
        self.assertIn("install it from Steam", installed.stderr)
        self.assertIn("CV1TOUCH_STEAMVR", installed.stderr)
        self.assertIn("start it once", installed.stderr)
        self.assertIn("run `python3 scripts/install.py` again", installed.stderr)
        self.assertNotIn("verify SteamVR's files", installed.stderr)
        self.assertNotIn("run `cv1touch steamvr install`", installed.stderr)
        self.assertFalse((self.root / "prefix/bin/cv1touch").exists())
        self.assertFalse(self.package.exists())
        self.assertEqual(self.log_text(), "")

    def test_doctor_says_when_the_input_profile_is_missing_or_out_of_date(self):
        check = self.doctor_profile_check()
        self.assertFalse(check["ok"])
        self.assertIn("is missing", check["detail"])
        self.assertIn("run `cv1touch steamvr install`", check["detail"])
        self.cli("install")
        (self.steamvr_input / "touch_profile.json").write_text(STEAMVR_PROFILE.replace("[ 7, 8 ]", "[ 9, 10 ]"))
        check = self.doctor_profile_check()
        self.assertFalse(check["ok"])
        self.assertIn("is out of date", check["detail"])
        self.assertIn("SteamVR's Oculus Touch profile, or cv1touch, changed", check["detail"])
        self.assertNotIn("dashboard bindings", check["detail"])
        self.cli("install")
        (self.steamvr_input / "example_dashboard.json").write_text('{"bindings": {"changed": true}}')
        check = self.doctor_profile_check()
        self.assertFalse(check["ok"])
        self.assertIn("SteamVR's dashboard bindings for Oculus Touch changed", check["detail"])
        self.assertNotIn("profile, or cv1touch, changed", check["detail"])
        self.cli("install")
        self.assertTrue(self.doctor_profile_check()["ok"])

    def test_modified_files_refuse_upgrade_and_unregister_before_external_calls(self):
        for name in ("bin/linux64/driver_cv1touch.so", "driver.vrdrivermanifest"):
            with self.subTest(name=name):
                self.cli("install")
                path = self.package / name
                original = path.read_bytes()
                path.write_bytes(b'{"name":"cv1touch","local":"edit"}')
                before = self.log_text()
                self.cli("install", success=False)
                self.cli("uninstall", success=False)
                self.assertEqual(self.log_text(), before)
                self.assertEqual(path.read_bytes(), b'{"name":"cv1touch","local":"edit"}')
                path.write_bytes(original)
                self.cli("uninstall")

    def test_unknown_content_and_symlink_refuse_before_external_calls(self):
        self.cli("install")
        extra = self.package / "personal"
        extra.write_bytes(b"keep")
        before = self.log_text()
        self.cli("install", success=False)
        self.cli("uninstall", success=False)
        self.assertEqual(self.log_text(), before)
        self.assertEqual(extra.read_bytes(), b"keep")
        extra.unlink()
        library = self.package / "bin/linux64/driver_cv1touch.so"
        library.unlink()
        library.symlink_to(self.assets / "driver_cv1touch.so")
        self.cli("install", success=False)
        self.cli("uninstall", success=False)
        self.assertEqual(self.log_text(), before)
        self.assertTrue(library.is_symlink())

    def test_repeated_install_remove_and_exact_legacy_adoption(self):
        self.cli("install")
        self.cli("install")
        self.make_legacy()
        self.cli("check")
        self.cli("install")
        self.assertEqual(json.loads((self.package / ".cv1touch-owned.json").read_text())["version"], 2)
        self.cli("uninstall")
        self.cli("uninstall")
        self.assertFalse(self.package.exists())

    def installer(self):
        """The real installer's command for a bundle of the built CLI, and its environment."""
        bundle = self.root / "bundle"
        bundle.mkdir()
        shutil.copy2(BINARY, bundle / "cv1touch")
        (bundle / "cv1touchd").write_bytes(b"daemon placeholder; never launched")
        for name in ("driver_cv1touch.so", "driver.vrdrivermanifest"):
            shutil.copy2(self.assets / name, bundle / name)
        for library in Path(LIBRARY_PATH.split(":")[0]).glob("libopenhmd.so*"):
            if library.is_file():
                shutil.copy2(library, bundle / library.name)
        env = dict(self.env)
        env.pop("CV1TOUCH_ASSETS")
        command = [sys.executable, str(ROOT / "scripts/install.py"), "--prefix", str(self.root / "prefix"), "--artifacts", str(bundle), "--no-service"]
        return bundle, command, env

    def test_installer_without_steamvrs_profile_changes_nothing(self):
        (self.steamvr_input / "touch_profile.json").unlink()
        _, command, env = self.installer()
        result = subprocess.run(command, env=env, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("SteamVR's Oculus Touch input profile is missing", result.stderr)
        self.assertIn("run `python3 scripts/install.py` again", result.stderr)
        self.assertNotIn("run `cv1touch steamvr install`", result.stderr)
        self.assertFalse((self.root / "prefix/bin/cv1touch").exists())
        self.assertFalse(self.package.exists())
        self.assertEqual(self.log_text(), "")

    def test_default_installer_conflict_leaves_prefix_and_registration_unchanged(self):
        bundle, command, env = self.installer()
        prefix = self.root / "prefix"
        first = subprocess.run(command, env=env, capture_output=True, text=True)
        self.assertEqual(first.returncode, 0, first.stdout + first.stderr)
        self.assertIn('"/input/home"', (self.package / "resources/input/touch_profile.json").read_text())
        registered = self.package / "bin/linux64/driver_cv1touch.so"
        registered.write_bytes(b"user changed registered driver")
        before_log = self.log_text()
        before_manifest = (prefix / "lib/cv1touch/install-manifest.json").read_bytes()
        (bundle / "cv1touchd").write_bytes(b"upgraded daemon placeholder")
        upgrade = subprocess.run(command, env=env, capture_output=True, text=True)
        self.assertNotEqual(upgrade.returncode, 0)
        self.assertEqual((prefix / "bin/cv1touchd").read_bytes(), b"daemon placeholder; never launched")
        self.assertEqual((prefix / "lib/cv1touch/install-manifest.json").read_bytes(), before_manifest)
        self.assertEqual(self.log_text(), before_log)
        removal = subprocess.run(command + ["--uninstall"], env=env, capture_output=True, text=True)
        self.assertNotEqual(removal.returncode, 0)
        self.assertTrue((prefix / "bin/cv1touch").exists())
        self.assertEqual(registered.read_bytes(), b"user changed registered driver")
        self.assertEqual(self.log_text(), before_log)
        # A byte-identical legacy package can still upgrade after the prefix payload changes.
        registered.write_bytes(b"original driver")
        self.make_legacy()
        (bundle / "driver_cv1touch.so").write_bytes(b"new trusted driver")
        repaired = subprocess.run(command, env=env, capture_output=True, text=True)
        self.assertEqual(repaired.returncode, 0, repaired.stdout + repaired.stderr)
        self.assertEqual(registered.read_bytes(), b"new trusted driver")
        self.assertEqual((prefix / "bin/cv1touchd").read_bytes(), b"upgraded daemon placeholder")
        removed = subprocess.run(command + ["--uninstall"], env=env, capture_output=True, text=True)
        self.assertEqual(removed.returncode, 0, removed.stdout + removed.stderr)
        self.assertFalse(self.package.exists())

    def test_installer_uninstall_works_without_steamvrs_profile(self):
        _, command, env = self.installer()
        first = subprocess.run(command, env=env, capture_output=True, text=True)
        self.assertEqual(first.returncode, 0, first.stdout + first.stderr)
        self.assertTrue(self.package.is_dir())
        (self.steamvr_input / "touch_profile.json").unlink()
        removed = subprocess.run(command + ["--uninstall"], env=env, capture_output=True, text=True)
        self.assertEqual(removed.returncode, 0, removed.stdout + removed.stderr)
        self.assertFalse(self.package.exists())


if __name__ == "__main__":
    unittest.main()
