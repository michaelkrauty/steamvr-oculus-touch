"""Exercise the real installer with isolated files and no desktop/device effects."""

from contextlib import contextmanager
import errno
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


FAKE_INTEGRATION = r'''
import json
import os
from pathlib import Path
import shutil
import sys
root = Path(os.environ["FAKE_ROOT"])
name, args = Path(sys.argv[0]).name, sys.argv[1:]
with (root / "calls").open("a") as log:
    log.write(json.dumps([name, *args]) + "\n")
if name == "pgrep":
    sys.exit(1)
if name == "systemctl":
    if "enable" in args:
        (root / "enabled").write_text("enabled")
    if "disable" in args:
        (root / "enabled").unlink()
    sys.exit(0)
assert name == "cv1touch" and args[0] == "steamvr", sys.argv
driver = root / "registered-driver"
if args[1] == "install":
    driver.mkdir()
    (driver / "driver_cv1touch.so").write_bytes(b"independent driver copy")
    (driver / ".cv1touch-owned.json").write_text("fake independent ownership record")
    (root / "registration").write_text(str(driver))
elif args[1] == "uninstall":
    shutil.rmtree(driver)
    (root / "registration").unlink()
else:
    assert args[1] == "check", sys.argv
'''


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

    def invoke(self, *args, service=False, steamvr=False):
        command = [sys.executable, str(SCRIPT), "--prefix", str(self.prefix), "--artifacts", str(self.bundle)]
        if not steamvr:
            command.append("--no-steamvr")
        if not service:
            command.append("--no-service")
        return subprocess.run(command + list(args), env=self.env, capture_output=True, text=True)

    def fake_integrations(self):
        fakebin = self.root / "fakebin"
        fakebin.mkdir()
        for path in (self.bundle / "cv1touch", fakebin / "pgrep", fakebin / "systemctl"):
            path.write_text(f"#!{sys.executable}\n" + FAKE_INTEGRATION)
            path.chmod(0o755)
        self.env.update(FAKE_ROOT=str(self.root), PATH=str(fakebin))

    def assert_skip_refused(self, *flags):
        self.fake_integrations()
        installed = self.invoke(service=True, steamvr=True)
        self.assertEqual(installed.returncode, 0, installed.stderr)
        manifest = self.prefix / "lib/cv1touch/install-manifest.json"
        record = json.loads(manifest.read_text())
        unit = Path(record["service"])
        self.assertIn(str(self.prefix / "bin/cv1touchd"), unit.read_text())
        self.assertTrue(record["steamvr"])
        self.assertTrue((self.root / "enabled").exists())
        self.assertTrue((self.root / "registration").exists())

        def snapshot():
            return {str(path.relative_to(self.root)): (path.read_bytes(), path.stat().st_mode)
                    for path in self.root.rglob("*") if path.is_file()}

        before = snapshot()
        result = self.invoke("--uninstall", *flags, service=True, steamvr=True)
        after = snapshot()
        self.assertEqual(result.returncode, 1, result.stderr)
        for flag in flags:
            self.assertIn(flag, result.stderr)
        self.assertIn("unchanged", result.stderr)
        self.assertIn("omit", result.stderr)
        # Includes payload, manifest bytes, unit, enablement, registration,
        # independent fake driver ownership, and the integration command log.
        self.assertEqual(after, before)

        retry = self.invoke("--uninstall", service=True, steamvr=True)
        self.assertEqual(retry.returncode, 0, retry.stderr)
        self.assertFalse(manifest.exists())
        for relative in record["files"]:
            self.assertFalse((self.prefix / relative).exists(), relative)
        for path in (unit, self.root / "enabled", self.root / "registration", self.root / "registered-driver"):
            self.assertFalse(path.exists(), str(path))

    def test_uninstall_refuses_skipping_recorded_service(self):
        self.assert_skip_refused("--no-service")

    def test_uninstall_refuses_skipping_recorded_steamvr(self):
        self.assert_skip_refused("--no-steamvr")

    def test_uninstall_refuses_skipping_both_recorded_integrations(self):
        self.assert_skip_refused("--no-service", "--no-steamvr")

    def test_uninstall_allows_skip_flags_without_recorded_integrations(self):
        self.fake_integrations()
        for flags in (("--no-service",), ("--no-steamvr",), ("--no-service", "--no-steamvr")):
            with self.subTest(flags=flags):
                installed = self.invoke()
                self.assertEqual(installed.returncode, 0, installed.stderr)
                manifest = self.prefix / "lib/cv1touch/install-manifest.json"
                record = json.loads(manifest.read_text())
                self.assertFalse(record.get("service"))
                self.assertFalse(record.get("steamvr"))
                result = self.invoke("--uninstall", *flags, service=True, steamvr=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertFalse(manifest.exists())
                for relative in record["files"]:
                    self.assertFalse((self.prefix / relative).exists(), relative)
                self.assertFalse((self.root / "calls").exists())

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

    def test_journal_prepublication_failures_leave_destinations_unchanged_and_allow_retry(self):
        original_open = Path.open
        original_replace = os.replace
        for stage in ("write", "flush", "fsync", "rename"):
            with self.subTest(stage=stage):
                self.prefix = self.root / stage
                result = self.invoke()
                self.assertEqual(result.returncode, 0, result.stderr)
                before = {path: path.read_bytes() for path in self.prefix.rglob("*") if path.is_file()}
                upgraded = f"upgraded-{stage}".encode()
                (self.bundle / "cv1touch").write_bytes(upgraded)
                transaction = self.prefix / ".cv1touch-transaction"
                journal = transaction / "journal.json"
                args = mock.Mock(prefix=self.prefix, artifacts=self.bundle, no_build=True, no_service=True, no_steamvr=True)
                injected = []

                def fail(*args, **kwargs):
                    injected.append(stage)
                    raise OSError(errno.ENOSPC, f"injected journal {stage} failure")

                @contextmanager
                def journal_open(path, *args, **kwargs):
                    with original_open(path, *args, **kwargs) as stream:
                        if stage == "write":
                            write = stream.write

                            def partial_write(text):
                                write(text[:1])
                                stream.flush()
                                fail()

                            with mock.patch.object(stream, "write", side_effect=partial_write):
                                yield stream
                        elif stage == "flush":
                            with mock.patch.object(stream, "flush", side_effect=fail):
                                yield stream
                        elif stage == "fsync":
                            with mock.patch.object(INSTALLER.os, "fsync", side_effect=fail):
                                yield stream
                        else:
                            yield stream

                def intercepted_open(path, *args, **kwargs):
                    # Match the journal writer whether it uses the final or a staging name.
                    if path.parent == transaction and args == ("w",):
                        return journal_open(path, *args, **kwargs)
                    return original_open(path, *args, **kwargs)

                def interrupted_publish(source, destination):
                    if Path(destination) == journal:
                        injected.append(stage)
                        raise KeyboardInterrupt()
                    return original_replace(source, destination)

                with mock.patch.dict(os.environ, self.env), mock.patch.object(Path, "open", intercepted_open):
                    with mock.patch.object(INSTALLER.os, "replace", side_effect=interrupted_publish):
                        with mock.patch.object(INSTALLER, "atomic_copy") as replacement:
                            with self.assertRaises(KeyboardInterrupt if stage == "rename" else OSError) as failure:
                                INSTALLER.install(args)
                            if stage != "rename":
                                self.assertEqual(failure.exception.errno, errno.ENOSPC)
                            replacement.assert_not_called()
                self.assertEqual(injected, [stage])
                self.assertFalse(journal.exists())
                if stage == "write":
                    self.assertTrue(any(path.read_bytes() == b"[" for path in transaction.iterdir()))
                for path, contents in before.items():
                    self.assertEqual(path.read_bytes(), contents, str(path))
                result = self.invoke()
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertFalse(transaction.exists())
                self.assertEqual((self.prefix / "bin/cv1touch").read_bytes(), upgraded)

    def test_malformed_published_journal_is_retained(self):
        transaction = self.prefix / ".cv1touch-transaction"
        transaction.mkdir(parents=True, mode=0o700)
        journal = transaction / "journal.json"
        journal.write_bytes(b"[")
        for _ in range(2):
            with self.assertRaises(json.JSONDecodeError):
                INSTALLER.recover_transaction(self.prefix)
            self.assertEqual(journal.read_bytes(), b"[")

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
        before = {path: path.read_bytes() for path in self.prefix.rglob("*") if path.is_file()}
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
        journal = self.prefix / ".cv1touch-transaction/journal.json"
        entries = json.loads(journal.read_text())
        self.assertEqual({entry["destination"] for entry in entries}, {str(path) for path in before if path.name != ".cv1touch-install.lock"})
        self.assertEqual((self.prefix / "bin/cv1touch").read_bytes(), b"upgraded-cli")
        INSTALLER.recover_transaction(self.prefix)
        for path, contents in before.items():
            self.assertEqual(path.read_bytes(), contents, str(path))
        # Repeat the interruption to also exercise recovery in a fresh installer.
        with mock.patch.dict(os.environ, self.env), mock.patch.object(INSTALLER, "atomic_copy", side_effect=interrupt):
            with self.assertRaises(KeyboardInterrupt):
                INSTALLER.install(args)
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
