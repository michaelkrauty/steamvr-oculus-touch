#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""Reject Windows bundles that depend on unshipped, non-system DLLs.

Windows CI images have development runtimes installed, so executing the bundle alone
does not prove a stock Windows installation can load it. Inspect every PE's imports,
including delay imports, without consulting the build machine's DLL search path.
Requires llvm-readobj (part of the LLVM toolchain used to build cv1touch).
"""

import argparse
from pathlib import Path
import re
import subprocess


# Inbox on supported Windows 11 x64, not optional redistributables. Keep this explicit:
# a newly imported DLL needs either a reviewed system dependency or a bundled copy.
SYSTEM_DLLS = frozenset(name.lower() + ".dll" for name in (
    "advapi32", "bcrypt", "bcryptprimitives", "cfgmgr32", "hid", "kernel32",
    "newdev", "ntdll", "ole32", "setupapi", "shell32", "user32", "winmm",
))


def dependencies(output):
    return re.findall(r"^\s+Name: (\S+)\s*$", output, re.MULTILINE)


def system_dll(name):
    return name.lower() in SYSTEM_DLLS or bool(re.fullmatch(
        r"api-ms-win-(?:core|crt)-[a-z0-9-]+\.dll", name, re.IGNORECASE))


def check(bundle, readobj):
    binaries = sorted(p for p in bundle.rglob("*") if p.is_file() and p.suffix.lower() in (".dll", ".exe"))
    if not binaries:
        raise ValueError("bundle has no executables or DLLs")
    failures = []
    for binary in binaries:
        output = subprocess.check_output([readobj, "--coff-imports", str(binary)], text=True)
        if not re.search(r"^Format: COFF-x86-64$", output, re.MULTILINE):
            raise ValueError(f"{binary} is not a Windows x64 binary")
        imports = dependencies(output)
        if not imports:
            raise ValueError(f"no imports found for {binary}; cannot validate the bundle")
        adjacent = {p.name.lower() for p in binary.parent.iterdir() if p.is_file()}
        for name in imports:
            if not system_dll(name) and name.lower() not in adjacent:
                failures.append(f"{binary.relative_to(bundle)} requires unshipped {name}")
        print(f"{binary.relative_to(bundle)}: {', '.join(imports)}")
    if failures:
        raise ValueError("\n".join(failures))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--readobj", default="llvm-readobj")
    args = parser.parse_args()
    try:
        check(args.bundle, args.readobj)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Windows bundle: {error}\n")
