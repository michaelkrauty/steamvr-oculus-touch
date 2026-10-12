#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""The Windows twin of check_exports.sh: the driver DLL exports only HmdDriverFactory and needs no
C/C++ runtime DLL from the host, since vrserver loads it beside its own (Review Focus 3)."""
import re
import struct
import sys

# the dynamic MSVC, Universal CRT and LLVM/GNU C++ runtimes
RUNTIME = re.compile(r"^(msvcp|vcruntime|concrt|api-ms-win-crt-|ucrtbase|libc\+\+|libstdc\+\+|libgcc)", re.I)


def parse(data):
    """(exported names, number of exported functions, imported DLL names) of a PE32+ image."""
    pe = struct.unpack_from("<I", data, 0x3C)[0]
    if data[pe:pe + 4] != b"PE\0\0":
        raise ValueError("not a PE image")
    sections, optional_size = struct.unpack_from("<2xH12xH", data, pe + 4)
    optional = pe + 24
    if struct.unpack_from("<H", data, optional)[0] != 0x20B:
        raise ValueError("not PE32+ (64-bit)")
    directories = struct.unpack_from("<4I", data, optional + 112)  # export rva, size, import rva, size
    table = optional + optional_size
    spans = []
    for i in range(sections):
        vsize, va, raw_size, raw = struct.unpack_from("<4I", data, table + 40 * i + 8)
        spans.append((va, max(vsize, raw_size), raw))

    def offset(rva):
        for va, size, raw in spans:
            if va <= rva < va + size:
                return raw + rva - va
        raise ValueError(f"rva {rva:#x} outside every section")

    def cstr(rva):
        start = offset(rva)
        return data[start:data.index(b"\0", start)].decode("ascii")

    names, functions = [], 0
    if directories[0]:
        functions, count, _, name_table = struct.unpack_from("<4I", data, offset(directories[0]) + 20)
        names = [cstr(struct.unpack_from("<I", data, offset(name_table) + 4 * i)[0]) for i in range(count)]
    imports = []
    if directories[2]:
        at = offset(directories[2])
        while True:
            entry = struct.unpack_from("<5I", data, at)
            if not any(entry):
                break
            imports.append(cstr(entry[3]))
            at += 20
    return names, functions, imports


def main(path):
    with open(path, "rb") as f:
        names, functions, imports = parse(f.read())
    if names != ["HmdDriverFactory"] or functions != 1:
        print(f"unexpected exports: {names} ({functions} functions)")
        return 1
    runtime = [dll for dll in imports if RUNTIME.match(dll)]
    if runtime:
        print(f"links a runtime DLL: {runtime}")
        return 1
    print(f"exports ok; imports {', '.join(imports)}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
