# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""The Windows bundle audit must not accept the build host's optional runtimes."""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("check_bundle", Path(__file__).resolve().parents[1] / "windows/check_bundle.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def imports(*names):
    return "Format: COFF-x86-64\n" + "".join(f"Import {{\n  Name: {name}\n}}\n" for name in names)


class WindowsBundleTests(unittest.TestCase):
    def test_inbox_libraries_but_not_optional_runtimes(self):
        for name in ("KERNEL32.dll", "setupapi.dll", "api-ms-win-crt-runtime-l1-1-0.dll"):
            self.assertTrue(audit.system_dll(name), name)
        for name in ("VCRUNTIME140.dll", "MSVCP140.dll", "concrt140.dll", "libusb-1.0.dll"):
            self.assertFalse(audit.system_dll(name), name)

    def test_both_regular_and_delayed_imports(self):
        output = imports("kernel32.dll") + "DelayImport {\n  Name: VCRUNTIME140.dll\n}\n"
        self.assertEqual(audit.dependencies(output), ["kernel32.dll", "VCRUNTIME140.dll"])

    def test_transitive_dependencies_must_be_adjacent_not_somewhere_on_the_host(self):
        with tempfile.TemporaryDirectory() as temporary, contextlib.redirect_stdout(io.StringIO()):
            bundle = Path(temporary)
            (bundle / "bin").mkdir()
            (bundle / "bin/tool.exe").touch()
            with patch.object(audit.subprocess, "check_output", return_value=imports("kernel32.dll", "engine.dll")):
                with self.assertRaisesRegex(ValueError, "unshipped engine.dll"):
                    audit.check(bundle, "readobj")
                (bundle / "engine.dll").touch()
                with self.assertRaisesRegex(ValueError, "unshipped engine.dll"):
                    audit.check(bundle, "readobj")
                (bundle / "engine.dll").unlink()
            (bundle / "bin/engine.dll").touch()
            def readobj(command, **_):
                return imports("kernel32.dll", "engine.dll") if command[-1].endswith(".exe") else imports("VCRUNTIME140.dll")
            with patch.object(audit.subprocess, "check_output", side_effect=readobj):
                with self.assertRaisesRegex(ValueError, "engine.dll requires unshipped VCRUNTIME140.dll"):
                    audit.check(bundle, "readobj")
            with patch.object(audit.subprocess, "check_output", return_value=imports("KERNEL32.dll")):
                audit.check(bundle, "readobj")

    def test_missing_imports_or_wrong_architecture_cannot_silently_pass(self):
        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary)
            with self.assertRaisesRegex(ValueError, "no executables"):
                audit.check(bundle, "readobj")
            (bundle / "tool.exe").touch()
            for output in ("Format: COFF-x86-64\n", imports("kernel32.dll").replace("x86-64", "i386")):
                with patch.object(audit.subprocess, "check_output", return_value=output):
                    with self.assertRaises(ValueError):
                        audit.check(bundle, "readobj")


if __name__ == "__main__":
    unittest.main()
