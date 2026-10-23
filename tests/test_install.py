"""Exercise the real installer with isolated files and no desktop/device effects."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/install.py"
SPEC = importlib.util.spec_from_file_location("cv1touch_install", SCRIPT)
INSTALLER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(INSTALLER)


class InstallTests(unittest.TestCase):
    def test_installed_service_matches_the_checked_in_restart_policy(self):
        template = (INSTALLER.ROOT / "systemd/cv1touchd.service").read_text()
        binary = Path('/example path/cv1touchd')
        service = INSTALLER.service_text(binary)
        self.assertEqual(service, template.replace('ExecStart=%h/.local/bin/cv1touchd', 'ExecStart=:"/example path/cv1touchd"'))
        self.assertIn('ExecStart=:"/example path/cv1touchd"\n', service)
        self.assertIn("Restart=on-failure\nRestartSec=2\n", service)
        self.assertNotIn("StartLimit", service)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="cv1touch-install-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bundle = self.root / "bundle"
        self.bundle.mkdir()
        for name in ("cv1touch", "cv1touchd", "libopenhmd.so", "libopenhmd.so.0", "driver_cv1touch.so", "driver.vrdrivermanifest"):
            (self.bundle / name).write_bytes(f"fake-{name}".encode())
        self.prefix = self.root / "prefix with spaces"
        self.env = dict(os.environ, HOME=str(self.root), XDG_CONFIG_HOME=str(self.root / "config"))

    def invoke(self, *args, service=False):
        command = [sys.executable, str(SCRIPT), "--prefix", str(self.prefix), "--artifacts", str(self.bundle), "--no-steamvr"]
        if not service:
            command.append("--no-service")
        return subprocess.run(command + list(args), env=self.env, capture_output=True, text=True)

    def test_install_upgrade_and_remove_preserves_user_data(self):
        data = self.root / "config/cv1touch/calibration.json"
        data.parent.mkdir(parents=True)
        data.write_text("personal calibration")
        self.assertEqual(self.invoke().returncode, 0)
        self.assertEqual((self.prefix / "bin/cv1touch").read_bytes(), b"fake-cv1touch")
        (self.bundle / "cv1touch").write_bytes(b"upgraded")
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((self.prefix / "bin/cv1touch").read_bytes(), b"upgraded")
        self.assertEqual(self.invoke("--uninstall").returncode, 0)
        self.assertFalse((self.prefix / "bin/cv1touch").exists())
        self.assertEqual(data.read_text(), "personal calibration")

    def test_conflicting_file_is_not_overwritten(self):
        path = self.prefix / "bin/cv1touch"
        path.parent.mkdir(parents=True)
        path.write_text("another program")
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(path.read_text(), "another program")
        self.assertFalse((self.prefix / "bin/cv1touchd").exists())

    def test_missing_artifact_leaves_no_partial_installation(self):
        (self.bundle / "driver_cv1touch.so").unlink()
        self.assertNotEqual(self.invoke().returncode, 0)
        self.assertFalse((self.prefix / "bin/cv1touch").exists())

    def test_missing_soname_leaves_no_partial_installation(self):
        (self.bundle / "libopenhmd.so.0").unlink()
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("SONAME", result.stderr)
        self.assertFalse((self.prefix / "bin/cv1touch").exists())

    def test_conflicting_service_leaves_no_partial_installation(self):
        service = self.root / "config/systemd/user/cv1touchd.service"
        service.parent.mkdir(parents=True)
        service.write_text("existing user service")
        result = self.invoke(service=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(service.read_text(), "existing user service")
        self.assertFalse((self.prefix / "bin/cv1touch").exists())

    def test_remove_keeps_modified_files_and_manifest(self):
        self.assertEqual(self.invoke().returncode, 0)
        path = self.prefix / "bin/cv1touchd"
        path.write_text("locally changed")
        result = self.invoke("--uninstall")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(path.read_text(), "locally changed")
        self.assertTrue((self.prefix / "lib/cv1touch/install-manifest.json").exists())

    def test_path_traversal_in_manifest_cannot_remove_external_file(self):
        self.assertEqual(self.invoke().returncode, 0)
        outside = self.root / "keep"
        outside.write_text("keep me")
        manifest = self.prefix / "lib/cv1touch/install-manifest.json"
        record = json.loads(manifest.read_text())
        record["files"]["../keep"] = hashlib.sha256(outside.read_bytes()).hexdigest()
        manifest.write_text(json.dumps(record))
        self.assertNotEqual(self.invoke("--uninstall").returncode, 0)
        self.assertEqual(outside.read_text(), "keep me")

    def test_failed_upgrade_restores_files_and_can_retry(self):
        self.assertEqual(self.invoke().returncode, 0)
        manifest = self.prefix / "lib/cv1touch/install-manifest.json"
        before = manifest.read_bytes()
        (self.bundle / "cv1touch").write_bytes(b"upgraded-cli")
        (self.bundle / "cv1touchd").write_bytes(b"upgraded-daemon")
        args = mock.Mock(prefix=self.prefix, artifacts=self.bundle, no_build=True, no_service=True, no_steamvr=True)
        original = INSTALLER.atomic_copy
        failed = False

        def fail_once(source, destination):
            nonlocal failed
            if destination == self.prefix / "bin/cv1touchd" and not failed:
                failed = True
                raise OSError("simulated disk failure")
            original(source, destination)

        with mock.patch.dict(os.environ, self.env), mock.patch.object(INSTALLER, "atomic_copy", side_effect=fail_once):
            with self.assertRaises(OSError):
                INSTALLER.install(args)
        self.assertEqual((self.prefix / "bin/cv1touch").read_bytes(), b"fake-cv1touch")
        self.assertEqual((self.prefix / "bin/cv1touchd").read_bytes(), b"fake-cv1touchd")
        self.assertEqual(manifest.read_bytes(), before)
        self.assertEqual(self.invoke().returncode, 0)

    def test_partial_remove_records_successful_unregister_before_removing_cli(self):
        self.assertEqual(self.invoke().returncode, 0)
        manifest = self.prefix / "lib/cv1touch/install-manifest.json"
        record = json.loads(manifest.read_text())
        record["steamvr"] = True
        manifest.write_text(json.dumps(record))
        daemon = self.prefix / "bin/cv1touchd"
        daemon.write_text("modified")
        args = mock.Mock(prefix=self.prefix, no_service=True, no_steamvr=False)
        with mock.patch.object(INSTALLER, "run") as run:
            with self.assertRaises(RuntimeError):
                INSTALLER.uninstall(args)
            run.assert_called_once()
        self.assertFalse(json.loads(manifest.read_text())["steamvr"])
        self.assertFalse((self.prefix / "bin/cv1touch").exists())

    def test_interrupted_upgrade_is_recovered_before_retry(self):
        self.assertEqual(self.invoke().returncode, 0)
        (self.bundle / "cv1touch").write_bytes(b"upgraded-cli")
        (self.bundle / "cv1touchd").write_bytes(b"upgraded-daemon")
        args = mock.Mock(prefix=self.prefix, artifacts=self.bundle, no_build=True, no_service=True, no_steamvr=True)
        original = INSTALLER.atomic_copy

        def interrupt(source, destination):
            if destination == self.prefix / "bin/cv1touchd":
                raise KeyboardInterrupt()
            original(source, destination)

        with mock.patch.dict(os.environ, self.env), mock.patch.object(INSTALLER, "atomic_copy", side_effect=interrupt):
            with self.assertRaises(KeyboardInterrupt):
                INSTALLER.install(args)
        self.assertTrue((self.prefix / ".cv1touch-transaction/journal.json").exists())
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse((self.prefix / ".cv1touch-transaction").exists())
        self.assertEqual((self.prefix / "bin/cv1touch").read_bytes(), b"upgraded-cli")
        self.assertEqual((self.prefix / "bin/cv1touchd").read_bytes(), b"upgraded-daemon")

    def test_driver_preflight_failure_precedes_prefix_changes(self):
        args = mock.Mock(prefix=self.prefix, artifacts=self.bundle, no_build=True, no_service=True, no_steamvr=False)
        with mock.patch.dict(os.environ, self.env), mock.patch.object(INSTALLER.subprocess, "run", return_value=mock.Mock(returncode=1)), mock.patch.object(INSTALLER, "run", side_effect=RuntimeError("modified registered driver")) as run:
            with self.assertRaisesRegex(RuntimeError, "modified registered driver"):
                INSTALLER.install(args)
        self.assertEqual(run.call_args.args[0][1:3], ["steamvr", "check"])
        self.assertEqual(run.call_args.args[0][-1], "--install-script")
        self.assertFalse((self.prefix / "bin/cv1touch").exists())
        self.assertFalse((self.prefix / "lib/cv1touch/install-manifest.json").exists())

    def test_driver_upgrade_uses_trusted_old_assets_for_preflight_and_install(self):
        self.assertEqual(self.invoke().returncode, 0)
        old_driver = (self.prefix / "lib/cv1touch/driver_cv1touch.so").read_bytes()
        (self.bundle / "driver_cv1touch.so").write_bytes(b"new driver")
        args = mock.Mock(prefix=self.prefix, artifacts=self.bundle, no_build=True, no_service=True, no_steamvr=False)
        observed = []
        def capture(command, **kwargs):
            observed.append(command)
            self.assertEqual(command[2], "check" if len(observed) == 1 else "install")
            self.assertEqual(command[3], "--legacy-assets")
            if len(observed) == 1:
                self.assertEqual(command[-1], "--install-script")
            else:
                self.assertNotIn("--install-script", command)
            self.assertEqual(Path(command[4]).read_bytes(), old_driver)
        with mock.patch.dict(os.environ, self.env), mock.patch.object(INSTALLER.subprocess, "run", return_value=mock.Mock(returncode=1)), mock.patch.object(INSTALLER, "run", side_effect=capture):
            INSTALLER.install(args)
        self.assertEqual(len(observed), 2)
        self.assertEqual((self.prefix / "lib/cv1touch/driver_cv1touch.so").read_bytes(), b"new driver")

    def test_prepared_bundle_requires_notices_without_source_fallback(self):
        args = mock.Mock(prefix=self.prefix, artifacts=self.bundle, no_build=True, no_service=True, no_steamvr=True)
        with mock.patch.object(INSTALLER, "ROOT", self.root / "missing-source"):
            with self.assertRaisesRegex(RuntimeError, "licensing file"):
                INSTALLER.install(args)
        self.assertFalse((self.prefix / "bin/cv1touch").exists())


if __name__ == "__main__":
    unittest.main()
