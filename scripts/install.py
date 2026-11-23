#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""Build and install cv1touch for the current user, or remove managed files."""

import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
LICENSE_FILES = ("LICENSE", "NOTICE", "THIRD_PARTY_NOTICES.md", "third_party/openhmd/LICENSE", "third_party/openhmd/LICENSE-OpenCV", "third_party/openhmd/src/drv_oculus_rift/rift-sensor-camera.c", "third_party/openhmd/src/drv_oculus_rift/rift-sensor-pose.c", "third_party/licenses/nxjson-MIT.txt", "third_party/licenses/miniz-Unlicense.txt", "third_party/openvr/LICENSE", "third_party/openvr/PROVENANCE.md")


OCULUS_USB_VENDOR = "2833"

# Directories udev reads rules from, local administrator's first.
UDEV_RULE_DIRS = (Path("/etc/udev/rules.d"), Path("/run/udev/rules.d"), Path("/usr/lib/udev/rules.d"), Path("/lib/udev/rules.d"))

# What `cv1touch steamvr install` prints when it turned the setting on.
ENABLED_MULTIPLE_DRIVERS = "activateMultipleDrivers enabled (previous settings kept as "


def run(args, **kwargs):
    # The output is kept for the error message: each stage prints one result line.
    return subprocess.run([str(a) for a in args], check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, **kwargs).stdout


@contextmanager
def stage(header):
    print(f"{header}...", end="", flush=True)
    try:
        yield
    except BaseException:
        print(" failed", flush=True)
        raise
    print(" done", flush=True)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_copy(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=".cv1touch-", dir=destination.parent)
    os.close(fd)
    try:
        shutil.copy2(source, name)
        with open(name, "rb") as staged:
            os.fsync(staged.fileno())
        os.replace(name, destination)
        sync_directory(destination.parent)
    finally:
        Path(name).unlink(missing_ok=True)


def service_text(binary):
    if any(character in str(binary) for character in ("\n", "\r")):
        raise RuntimeError("service executable path contains a line break")
    # systemd expands percent specifiers even inside a quoted ExecStart.
    executable = str(binary).replace("\\", "\\\\").replace('"', '\\"').replace("%", "%%")
    return (
        "[Unit]\nDescription=Rift CV1 Touch controller tracking\n"
        "Documentation=https://github.com/michaelkrauty/steamvr-oculus-touch\n\n"
        f'[Service]\nExecStart=:"{executable}"\n'
        "Restart=on-failure\nRestartSec=2\n\n[Install]\nWantedBy=default.target\n"
    )


def native_build(target, profile):
    marker = target / profile / "cv1touch-native-dir"
    if not marker.is_file():
        raise RuntimeError(f"missing {marker}; run cargo build --release first")
    native = Path(marker.read_text().strip())
    if not native.is_dir():
        raise RuntimeError(f"native build directory no longer exists: {native}")
    return native


def artifacts(args, staging):
    if args.artifacts:
        return args.artifacts.resolve()
    metadata = json.loads(subprocess.check_output(
        ["cargo", "metadata", "--format-version=1", "--no-deps"], cwd=ROOT, text=True))
    target = args.target_dir.resolve() if args.target_dir else Path(metadata["target_directory"])
    profile = None
    if not args.no_build:
        command = ["cargo", "build", "--locked", "--release", "--bins", "--message-format=json-render-diagnostics"]
        if args.target_dir:
            command += ["--target-dir", str(args.target_dir)]
        log = target / "cv1touch-install.log"
        with stage("Building cv1touch (this takes a few minutes)"):
            log.parent.mkdir(parents=True, exist_ok=True)
            with log.open("w") as diagnostics:
                build = subprocess.run(command, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=diagnostics)
            if build.returncode:
                raise RuntimeError(f"the build failed; the compiler's messages are in {log}")
        for line in build.stdout.splitlines():
            message = json.loads(line)
            if message.get("reason") == "compiler-artifact" and message.get("target", {}).get("name") == "cv1touchd" and message.get("executable"):
                profile = Path(message["executable"]).parent
    if profile is None:
        markers = [p for p in [target / "release/cv1touch-native-dir", *target.glob("*/release/cv1touch-native-dir")] if p.is_file()]
        if len(markers) != 1:
            raise RuntimeError("cannot select one existing release build; omit --no-build or use --artifacts")
        profile = markers[0].parent
    native = native_build(profile.parent, "release")
    for name in ("cv1touch", "cv1touchd"):
        shutil.copy2(profile / name, staging / name)
    for library in (native / "third_party/openhmd").glob("libopenhmd.so*"):
        if not library.is_file():
            continue
        # Dereference the SONAME links so every installed name is independently usable.
        shutil.copy2(library, staging / library.name)
    shutil.copy2(native / "driver/driver_cv1touch.so", staging / "driver_cv1touch.so")
    shutil.copy2(ROOT / "driver/cv1touch/driver.vrdrivermanifest", staging / "driver.vrdrivermanifest")
    for name in LICENSE_FILES:
        destination = staging / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / name, destination)
    return staging


def manifest_path(prefix):
    return prefix / "lib/cv1touch/install-manifest.json"


def read_manifest(prefix):
    path = manifest_path(prefix)
    return json.loads(path.read_text()) if path.exists() else {"files": {}}


def write_manifest(prefix, record):
    with tempfile.TemporaryDirectory(prefix="cv1touch-manifest-") as temporary:
        source = Path(temporary) / "manifest"
        source.write_text(json.dumps(record, indent=2) + "\n")
        atomic_copy(source, manifest_path(prefix))


def recover_transaction(prefix):
    transaction = prefix / ".cv1touch-transaction"
    if not transaction.exists():
        return
    if transaction.is_symlink() or transaction.stat().st_uid != os.geteuid():
        raise RuntimeError(f"unsafe transaction directory: {transaction}")
    journal = transaction / "journal.json"
    if not journal.exists():
        # No destination is changed until the complete journal is durable.
        shutil.rmtree(transaction)
        return
    entries = json.loads(journal.read_text())
    for entry in reversed(entries):
        destination = Path(entry["destination"])
        if destination.is_symlink():
            raise RuntimeError(f"transaction destination changed, recovery retained at {transaction}")
        current = digest(destination) if destination.exists() else None
        if current not in (entry["old_sha256"], entry["new_sha256"], None):
            raise RuntimeError(f"modified during upgrade, recovery retained at {transaction}: {destination}")
        if entry["backup"] is None:
            destination.unlink(missing_ok=True)
        else:
            atomic_copy(transaction / entry["backup"], destination)
    shutil.rmtree(transaction)
    sync_directory(prefix)


def replace_transaction(prefix, sources):
    transaction = prefix / ".cv1touch-transaction"
    transaction.mkdir(mode=0o700)
    entries = []
    for index, (destination, source) in enumerate(sources.items()):
        backup = None
        old = None
        if destination.exists():
            backup = f"backup-{index}"
            shutil.copy2(destination, transaction / backup)
            with (transaction / backup).open("rb") as saved:
                os.fsync(saved.fileno())
            old = digest(destination)
        entries.append({"destination": str(destination), "backup": backup, "old_sha256": old, "new_sha256": digest(source)})
    # Recovery must never see a partially written journal as a recovery marker.
    staged_journal = transaction / "journal.json.tmp"
    with staged_journal.open("w") as journal:
        json.dump(entries, journal)
        journal.flush()
        os.fsync(journal.fileno())
    os.replace(staged_journal, transaction / "journal.json")
    sync_directory(transaction)
    sync_directory(prefix)
    try:
        for destination, source in sources.items():
            atomic_copy(source, destination)
    except Exception:
        recover_transaction(prefix)
        raise
    shutil.rmtree(transaction)
    sync_directory(prefix)


def owned_destination(prefix, relative):
    path = Path(relative)
    if path.is_absolute() or ".." in path.parts:
        raise RuntimeError(f"invalid managed path: {relative}")
    destination = prefix / path
    # Do not follow a parent directory outside the selected install prefix.
    if not destination.parent.resolve().is_relative_to(prefix.resolve()):
        raise RuntimeError(f"managed directory escapes install prefix: {relative}")
    return destination


def install(args):
    prefix = args.prefix.resolve()
    recover_transaction(prefix)
    previous = read_manifest(prefix)
    unit_path = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "systemd/user/cv1touchd.service"
    unit = service_text(prefix / "bin/cv1touchd")
    if not args.no_service and (unit_path.exists() or unit_path.is_symlink()):
        expected = previous.get("service_sha256")
        if unit_path.is_symlink() or not expected or digest(unit_path) != expected:
            raise RuntimeError(f"refusing to replace an unmanaged service: {unit_path}; use --no-service or back it up")
    if not args.no_steamvr:
        active = subprocess.run(["pgrep", "-x", "vrserver"], stdout=subprocess.DEVNULL)
        if active.returncode == 0:
            raise RuntimeError("quit SteamVR before installing; it rewrites settings when exiting")
        if active.returncode != 1:
            raise RuntimeError("could not check whether SteamVR is running")
    with tempfile.TemporaryDirectory(prefix="cv1touch-install-") as temporary:
        bundle = artifacts(args, Path(temporary))
        with stage(f"Installing in {prefix}"):
            names = ["cv1touch", "cv1touchd", "driver_cv1touch.so", "driver.vrdrivermanifest"]
            libraries = sorted(p for p in bundle.glob("libopenhmd.so*") if p.is_file())
            if not (bundle / "libopenhmd.so.0").is_file():
                raise RuntimeError("bundle is missing the required libopenhmd.so.0 SONAME")
            sources = {f"bin/{name}": bundle / name for name in names[:2]}
            sources.update({f"lib/cv1touch/{name}": bundle / name for name in names[2:]})
            sources.update({f"lib/cv1touch/{p.name}": p for p in libraries})
            sources["lib/cv1touch/install.py"] = Path(__file__).resolve()
            for name in LICENSE_FILES:
                source = bundle / name if (bundle / name).is_file() else ROOT / name
                if not source.is_file():
                    raise RuntimeError(f"bundle is missing required licensing file: {name}")
                sources[f"share/cv1touch/{name}"] = source
            for relative, source in sources.items():
                if not source.is_file():
                    raise RuntimeError(f"bundle is missing {source.name}")
                destination = owned_destination(prefix, relative)
                if destination.exists() or destination.is_symlink():
                    expected = previous["files"].get(relative)
                    if not expected or destination.is_symlink() or digest(destination) != expected:
                        raise RuntimeError(f"refusing to replace an unmanaged or modified file: {destination}; "
                                               "move it aside or delete it, then run the installer again")
            # Validate the registered copy before committing any prefix/service changes. Legacy
            # adoption must compare against the trusted OLD source assets, not the upgrade payload.
            legacy_args = []
            if not args.no_steamvr:
                old_assets = ("lib/cv1touch/driver_cv1touch.so", "lib/cv1touch/driver.vrdrivermanifest")
                if all(relative in previous["files"] for relative in old_assets):
                    snapshots = []
                    for index, relative in enumerate(old_assets):
                        old = owned_destination(prefix, relative)
                        if not old.is_file() or old.is_symlink() or digest(old) != previous["files"][relative]:
                            raise RuntimeError(f"untrusted previous driver asset: {old}")
                        snapshot = Path(temporary) / f"previous-driver-{index}"
                        shutil.copy2(old, snapshot)
                        snapshots.append(str(snapshot))
                    legacy_args = ["--legacy-assets", *snapshots]
                check_env = dict(os.environ)
                check_env["LD_LIBRARY_PATH"] = str(bundle) + (":" + check_env["LD_LIBRARY_PATH"] if check_env.get("LD_LIBRARY_PATH") else "")
                # --install-script: cv1touch is not installed yet, so a failure must say to run
                # this installer again, not `cv1touch steamvr install`.
                run([bundle / "cv1touch", "steamvr", "check", *legacy_args, "--install-script"], env=check_env)
            # Keep old-only payloads owned for uninstall and later bundle rollback.
            # Preserve their original digests rather than adopting local changes.
            managed = dict(previous["files"])
            managed.update({relative: digest(source) for relative, source in sources.items()})
            replacements = {owned_destination(prefix, relative): source for relative, source in sources.items()}
            if not args.no_service:
                source = Path(temporary) / "cv1touchd.service"
                source.write_text(unit)
                replacements[unit_path] = source
            record = {"files": managed, "steamvr": previous.get("steamvr", False)}
            # A failed/partial CLI attempt cannot establish ownership. Keep cleanup tools
            # until a retry succeeds, including when publication after CLI success fails.
            if previous.get("steamvr_pending") or (not args.no_steamvr and not record["steamvr"]):
                record["steamvr_pending"] = True
            if not args.no_service:
                record.update(service=str(unit_path), service_sha256=hashlib.sha256(unit.encode()).hexdigest())
            elif previous.get("service"):
                record.update({key: previous[key] for key in ("service", "service_sha256")})
            manifest = Path(temporary) / "install-manifest.json"
            manifest.write_text(json.dumps(record, indent=2) + "\n")
            replacements[manifest_path(prefix)] = manifest
            prefix.mkdir(parents=True, exist_ok=True)
            replace_transaction(prefix, replacements)
        if not args.no_steamvr:
            with stage("Registering the driver with SteamVR"):
                registered = run([prefix / "bin/cv1touch", "steamvr", "install", *legacy_args])
            # The one change to the user's own settings: say it, with where the old ones are.
            for line in registered.splitlines():
                if line.startswith(ENABLED_MULTIPLE_DRIVERS):
                    print(f"SteamVR's activateMultipleDrivers setting is now on, so that SteamVR loads cv1touch's driver "
                          f"next to the headset's. Your settings as they were are kept in {line[len(ENABLED_MULTIPLE_DRIVERS):-1]}")
            record["steamvr"] = True
            record.pop("steamvr_pending", None)
            write_manifest(prefix, record)
    if not args.no_service:
        with stage("Starting the cv1touchd service"):
            run(["systemctl", "--user", "daemon-reload"])
            run(["systemctl", "--user", "enable", "--now", "cv1touchd.service"])
    print("\n".join(next_steps(prefix, args.no_service)))


def rule_lines(text):
    return {line.strip() for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")}


def udev_rule_installed(directories=UDEV_RULE_DIRS):
    # cv1touch's own rule under any file name: a rule that only mentions the vendor may be a
    # comment or cover other products.
    wanted = rule_lines((ROOT / "udev/70-cv1touch.rules").read_text())
    for directory in directories:
        for rule in Path(directory).glob("*.rules"):
            try:
                if wanted <= rule_lines(rule.read_text(errors="replace")):
                    return True
            except OSError:
                continue
    return False


def devices_usable(sysfs=Path("/sys"), dev=Path("/dev"), usable=lambda node: os.access(node, os.R_OK | os.W_OK)):
    # Whether this user can open every connected Oculus USB device and its HID nodes now, as a
    # rule other than cv1touch's may allow. False when none is connected: nothing shows it then.
    nodes = []
    for device in (sysfs / "bus/usb/devices").glob("*"):
        try:
            if (device / "idVendor").read_text().strip() == OCULUS_USB_VENDOR:
                bus, number = (int((device / name).read_text()) for name in ("busnum", "devnum"))
                nodes.append(dev / f"bus/usb/{bus:03d}/{number:03d}")
        except (OSError, ValueError):
            continue
    for hid in (sysfs / "class/hidraw").glob("hidraw*"):
        parent = (hid / "device").resolve()
        while parent != parent.parent and not (parent / "idVendor").is_file():
            parent = parent.parent
        try:
            if (parent / "idVendor").read_text().strip() == OCULUS_USB_VENDOR:
                nodes.append(dev / hid.name)
        except OSError:
            continue
    return bool(nodes) and all(usable(node) for node in nodes)


def next_steps(prefix, no_service, path=None, rule_dirs=UDEV_RULE_DIRS, usable=devices_usable):
    lines = [""]
    binaries = prefix / "bin"
    path = os.environ.get("PATH", "") if path is None else path
    if str(binaries) not in path.split(os.pathsep):
        lines += [f"{binaries} is not on your PATH, so your shell will not find the cv1touch command.",
                  "Add this line to ~/.bashrc (or your shell's startup file), then open a new terminal:",
                  f'  export PATH={shlex.quote(str(binaries))}:"$PATH"', ""]
    if not (udev_rule_installed(rule_dirs) or usable()):
        rule = ROOT / "udev/70-cv1touch.rules"
        lines += ["cv1touch needs permission to use the headset and sensors over USB.",
                  "Run these two commands once (they ask for your password):",
                  f"  sudo install -m 0644 {shlex.quote(str(rule))} /etc/udev/rules.d/70-cv1touch.rules",
                  "  sudo udevadm control --reload-rules",
                  "Then unplug the headset and the sensors and plug them back in.", ""]
    if no_service:
        lines.append("Start cv1touchd yourself; setup needs it running.")
    lines.append("Next: run cv1touch setup, which walks you through calibration. "
                 "If something stops working later, cv1touch doctor says what is wrong.")
    return lines


def uninstall(args):
    prefix = args.prefix.resolve()
    recover_transaction(prefix)
    manifest = manifest_path(prefix)
    if not manifest.exists():
        raise RuntimeError(f"no managed installation at {prefix}")
    record = read_manifest(prefix)
    if record.get("steamvr_pending"):
        raise RuntimeError("SteamVR registration ownership is unresolved; installation unchanged; "
                           "retry this installer without --no-steamvr before uninstalling")
    skipped = []
    if record.get("service") and args.no_service:
        skipped.append("--no-service")
    if record.get("steamvr") and args.no_steamvr:
        skipped.append("--no-steamvr")
    if skipped:
        raise RuntimeError(f"managed integrations remain; installation unchanged; omit {' and '.join(skipped)} to uninstall")
    if record.get("steamvr") and not args.no_steamvr:
        run([prefix / "bin/cv1touch", "steamvr", "uninstall"])
        record["steamvr"] = False
        write_manifest(prefix, record)
    if record.get("service") and not args.no_service:
        run(["systemctl", "--user", "disable", "--now", "cv1touchd.service"])
        unit = Path(record["service"])
        if unit.is_file() and not unit.is_symlink() and digest(unit) == record["service_sha256"]:
            unit.unlink()
        elif unit.exists():
            raise RuntimeError(f"service changed since installation, keeping it: {unit}")
        run(["systemctl", "--user", "daemon-reload"])
        record.pop("service", None)
        record.pop("service_sha256", None)
        write_manifest(prefix, record)
    kept = {}
    for relative, expected in record["files"].items():
        path = owned_destination(prefix, relative)
        if path.is_symlink() or (path.exists() and digest(path) != expected):
            kept[relative] = expected
            print(f"Keeping modified file: {path}", file=sys.stderr)
        else:
            path.unlink(missing_ok=True)
    if kept:
        record["files"] = kept
        write_manifest(prefix, record)
        raise RuntimeError("modified files remain; installation manifest retained")
    manifest.unlink()
    print("Uninstalled managed files. Calibration, recordings, and SteamVR settings are retained.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", type=Path, default=Path.home() / ".local")
    parser.add_argument("--no-build", action="store_true", help="install an existing release build")
    parser.add_argument("--target-dir", type=Path, help="Cargo target directory")
    parser.add_argument("--artifacts", type=Path, help="install a prepared flat bundle instead of building")
    parser.add_argument("--no-service", action="store_true", help="do not manage a systemd user service")
    parser.add_argument("--no-steamvr", action="store_true", help="do not register/unregister the SteamVR driver")
    parser.add_argument("--uninstall", action="store_true", help="remove files owned by this installer")
    args = parser.parse_args()
    try:
        prefix = args.prefix.resolve()
        prefix.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(prefix / ".cv1touch-install.lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        with os.fdopen(descriptor, "r+") as lock:
            if os.fstat(lock.fileno()).st_uid != os.geteuid():
                raise RuntimeError("installation lock is not owned by this user")
            fcntl.flock(lock, fcntl.LOCK_EX)
            (uninstall if args.uninstall else install)(args)
    except subprocess.CalledProcessError as error:
        # The command's own output says what went wrong; its exit status does not.
        print(error.output.rstrip() if error.output else f"cv1touch install: {error}", file=sys.stderr)
        return 1
    except (OSError, ValueError, RuntimeError) as error:
        print(f"cv1touch install: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
