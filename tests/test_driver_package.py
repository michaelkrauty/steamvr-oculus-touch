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

    def cli(self, *args, success=True):
        result = subprocess.run([BINARY, "steamvr", *args], env=self.env, cwd=self.root, capture_output=True, text=True)
        self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
        return result

    def log_text(self):
        return self.log.read_text() if self.log.exists() else ""

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
        (self.package / ".cv1touch-owned.json").unlink()
        self.cli("check")
        self.cli("install")
        self.assertEqual(json.loads((self.package / ".cv1touch-owned.json").read_text())["version"], 1)
        self.cli("uninstall")
        self.cli("uninstall")
        self.assertFalse(self.package.exists())

    def test_default_installer_conflict_leaves_prefix_and_registration_unchanged(self):
        bundle = self.root / "bundle"
        bundle.mkdir()
        shutil.copy2(BINARY, bundle / "cv1touch")
        (bundle / "cv1touchd").write_bytes(b"daemon placeholder; never launched")
        for name in ("driver_cv1touch.so", "driver.vrdrivermanifest"):
            shutil.copy2(self.assets / name, bundle / name)
        for library in Path(LIBRARY_PATH.split(":")[0]).glob("libopenhmd.so*"):
            if library.is_file():
                shutil.copy2(library, bundle / library.name)
        prefix = self.root / "prefix"
        env = dict(self.env)
        env.pop("CV1TOUCH_ASSETS")
        command = [sys.executable, str(ROOT / "scripts/install.py"), "--prefix", str(prefix), "--artifacts", str(bundle), "--no-service"]
        first = subprocess.run(command, env=env, capture_output=True, text=True)
        self.assertEqual(first.returncode, 0, first.stdout + first.stderr)
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
        (self.package / ".cv1touch-owned.json").unlink()
        (bundle / "driver_cv1touch.so").write_bytes(b"new trusted driver")
        repaired = subprocess.run(command, env=env, capture_output=True, text=True)
        self.assertEqual(repaired.returncode, 0, repaired.stdout + repaired.stderr)
        self.assertEqual(registered.read_bytes(), b"new trusted driver")
        self.assertEqual((prefix / "bin/cv1touchd").read_bytes(), b"upgraded daemon placeholder")
        removed = subprocess.run(command + ["--uninstall"], env=env, capture_output=True, text=True)
        self.assertEqual(removed.returncode, 0, removed.stdout + removed.stderr)
        self.assertFalse(self.package.exists())


if __name__ == "__main__":
    unittest.main()
