#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""Exercise the shipped installer on a disposable Windows runner, without USB hardware.

This intentionally uses the runner's per-user install, Start menu and Run registration.
Never run it on a user's configured PC. It validates missing prerequisites and recovery,
not calibration, real SteamVR, UAC interaction, or USB streaming.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import winreg
import zipfile


def run(args, expected=0, timeout=90, **kwargs):
    result = subprocess.run([str(a) for a in args], capture_output=True, text=True,
                            encoding="utf-8", errors="replace", timeout=timeout, **kwargs)
    if result.returncode != expected:
        raise AssertionError(f"{args}: exit {result.returncode}, expected {expected}\n{result.stdout}\n{result.stderr}")
    return result.stdout


def until(predicate, explanation, timeout=30):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError(explanation)
        time.sleep(0.2)


def run_entry():
    with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"Software\Microsoft\Windows\CurrentVersion\Run") as key:
        try:
            return winreg.QueryValueEx(key, "cv1touch")[0]
        except FileNotFoundError:
            return None


def powershell(script, **kwargs):
    return run(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command",
                "[Console]::OutputEncoding = [Text.UTF8Encoding]::new(); " + script], **kwargs)


def status(tool):
    result = subprocess.run([str(tool), "status", "--json"], capture_output=True, timeout=10)
    return json.loads(result.stdout) if result.returncode == 0 else None


def start(quiet):
    # A quiet launcher detaches a long-lived supervisor; never give it capture pipes
    # whose EOF would then depend on stopping that supervisor.
    subprocess.run([str(quiet)], check=True, timeout=15,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def shortcut(folder, name):
    path = folder / (name + ".lnk")
    assert path.is_file(), path
    env = dict(os.environ, CV1TOUCH_TEST_SHORTCUT=str(path))
    return json.loads(powershell(
        "$s = (New-Object -ComObject WScript.Shell).CreateShortcut($env:CV1TOUCH_TEST_SHORTCUT); "
        "@{target=$s.TargetPath; arguments=$s.Arguments; directory=$s.WorkingDirectory} | ConvertTo-Json", env=env))


def run_shortcut(link, expected=0):
    # /k is for a person to read the result; /c closes the test shell afterward. Preserve
    # the installed command string verbatim otherwise, including cmd's nested quoting.
    arguments = link["arguments"].replace("/k ", "/c ", 1)
    command = f'"{link["target"]}" {arguments}'
    result = subprocess.run(command, cwd=link["directory"], capture_output=True,
                            timeout=90, text=True, encoding="utf-8", errors="replace")
    assert result.returncode == expected, (command, result.returncode, result.stdout, result.stderr)
    return result.stdout


def main(installer, work):
    local = Path(os.environ["LOCALAPPDATA"])
    data = local / "cv1touch"
    assert not data.exists(), "refusing to change existing user data; use a fresh disposable runner"
    config = data
    config.mkdir(parents=True)
    settings = config / "settings.json"
    saved = b'{"prediction_percent":75}\n'
    settings.write_bytes(saved)
    logs = data / "state/logs"
    shortcuts = Path(os.environ["APPDATA"]) / "Microsoft/Windows/Start Menu/Programs/cv1touch"
    flags = ["/VERYSILENT", "/SUPPRESSMSGBOXES", "/NORESTART"]
    paths = [local / "Programs/cv1touch", local / "Programs/cv1touch test ü & spaces"]
    for cycle, app in enumerate(paths):
        print(f"Install/upgrade/uninstall cycle {cycle + 1}: {app}", flush=True)
        tool = app / "bin/cv1touch.exe"
        quiet = app / "bin/cv1touchd-quiet.exe"
        uninstall = app / "unins000.exe"
        install_args = [installer, *flags, f"/DIR={app}", f"/LOG={work / f'install-{cycle}.log'}"]
        run(install_args, timeout=180)
        try:
            assert "cv1touch " in run([tool, "--version"])
            assert run_entry() == f'"{quiet}"'
            until(lambda: status(tool) is not None, "installed daemon did not answer")
            assert settings.read_bytes() == saved
            assert (logs / "finish-setup.log").is_file()
            assert (logs / "installer.log").is_file()
            assert "Setup is incomplete" in (work / f"install-{cycle}.log").read_text(encoding="utf-8-sig")

            # Check the actual Start-menu command quoting, not a hand-recreated CLI command.
            for name in ("Set up cv1touch", "Check cv1touch", "Finish setting up cv1touch",
                         "Send a cv1touch report", "Start cv1touch", "Stop cv1touch"):
                shortcut(shortcuts, name)
            check = run_shortcut(shortcut(shortcuts, "Check cv1touch"), expected=1)
            assert "FAIL" in check and "sensor" in check
            repair = run_shortcut(shortcut(shortcuts, "Finish setting up cv1touch"), expected=1)
            assert "Setup is incomplete" in repair, repair
            until(lambda: status(tool) is not None, "repair did not restart the daemon")
            assert (logs / "finish-setup.previous.log").is_file()

            # Repeated launch must not create another daemon/supervisor pair.
            generation = status(tool)["generation"]
            for _ in range(2):
                start(quiet)
            assert status(tool)["generation"] == generation
            run_shortcut(shortcut(shortcuts, "Stop cv1touch"))
            until(lambda: status(tool) is None, "Stop shortcut did not stop the daemon")
            start(quiet)
            until(lambda: status(tool) is not None, "Start did not restart the daemon")

            setup = json.loads(run([tool, "setup", "--check", "--json"], expected=1))
            assert setup, "missing setup diagnostic JSON"
            report = work / f"report ü {cycle}.zip"
            run([tool, "report", "--out", report])
            with zipfile.ZipFile(report) as archive:
                assert archive.testzip() is None
                for name in ("report.txt", "logs/cv1touchd.log", "logs/installer.log", "logs/finish-setup.log"):
                    assert name in archive.namelist(), name
                assert archive.read("config/settings.json") == saved

            # Real package assembly against synthetic SteamVR files and a registration stub.
            # This covers Windows rename/quoting/resource handling without claiming the
            # proprietary runtime or a headset was exercised.
            runtime = work / "SteamVR test ü & spaces"
            resources = runtime / "drivers/oculus/resources/input"
            resources.mkdir(parents=True, exist_ok=True)
            profile = {"jsonid": "input_profile", "controller_type": "oculus_touch",
                       "resource_root": "oculus", "driver_name": "oculus",
                       "input_source": {"/input/a": {"type": "button", "side": "right", "click": True}},
                       "default_bindings": []}
            (resources / "touch_profile.json").write_text(json.dumps(profile))
            (runtime / "bin/win64").mkdir(parents=True, exist_ok=True)
            compiler = Path(os.environ["WINDIR"]) / "Microsoft.NET/Framework64/v4.0.30319/csc.exe"
            source = work / "register.cs"
            source.write_text('class Register { static void Main(string[] a) { System.IO.File.AppendAllText(System.Environment.GetEnvironmentVariable("CV1TOUCH_TEST_REGISTRATION_LOG"), string.Join("|", a) + "\\n"); } }')
            run([compiler, "/nologo", f"/out:{runtime / 'bin/win64/vrpathreg.exe'}", source])
            registration_log = work / f"registration-{cycle}.log"
            env = dict(os.environ, CV1TOUCH_STEAMVR=str(runtime), CV1TOUCH_STEAM_ROOT=str(work / "Steam"),
                       CV1TOUCH_TEST_REGISTRATION_LOG=str(registration_log))
            run([tool, "steamvr", "install"], env=env)
            package = data / "data/steamvr/cv1touch"
            assert (package / "bin/win64/driver_cv1touch.dll").read_bytes() == (app / "lib/cv1touch/driver_cv1touch.dll").read_bytes()
            made = json.loads((package / "resources/input/touch_profile.json").read_text())
            assert made["controller_type"] == "cv1touch_touch"
            assert made["compatibility_mode_controller_type"] == "oculus_touch"
            run([tool, "steamvr", "check"], env=env)
            run([tool, "steamvr", "install"], env=env)  # replace an existing managed directory
            run([tool, "steamvr", "uninstall"], env=env)
            assert not package.exists()
            assert registration_log.read_text(encoding="utf-8").splitlines() == [f"adddriver|{package}", f"adddriver|{package}", f"removedriver|{package}"]

            # Upgrade with the previous daemon running, preserving the same user settings.
            generation = status(tool)["generation"]
            run(install_args, timeout=180)
            until(lambda: (s := status(tool)) is not None and s["generation"] != generation,
                  "upgrade left the old daemon running")
            assert settings.read_bytes() == saved

            # A fake vrserver exercises the real process preflight. It is a test-only .NET
            # sleeper, not SteamVR and not evidence that the real runtime loads our driver.
            source = work / "sleeper.cs"
            source.write_text("class Sleeper { static void Main() { System.Threading.Thread.Sleep(120000); } }")
            fake = work / "vrserver.exe"
            run([compiler, "/nologo", f"/out:{fake}", source])
            with subprocess.Popen([str(fake)]) as process:
                try:
                    time.sleep(0.5)
                    before = hashlib.sha256(tool.read_bytes()).digest()
                    run([tool, "finish-setup"], expected=1)
                    run(install_args, expected=7, timeout=120)
                    assert hashlib.sha256(tool.read_bytes()).digest() == before
                    assert status(tool) is not None, "blocked upgrade stopped tracking"
                    # Cancellation must leave the install in place, including the Run entry.
                    result = subprocess.run([str(uninstall), *flags], timeout=60)
                    assert result.returncode != 0 and tool.is_file() and run_entry()
                finally:
                    process.terminate()
                    process.wait(timeout=10)
        finally:
            if uninstall.exists():
                run([uninstall, *flags, f"/LOG={work / f'uninstall-{cycle}.log'}"], timeout=120)
                until(lambda: not app.exists(), "uninstaller left application files behind")
        assert run_entry() is None
        assert settings.read_bytes() == saved, "uninstaller removed personal settings"
        assert not shortcuts.exists(), "uninstaller left Start-menu shortcuts"
        assert (logs / "uninstaller.log").is_file()
    print("Windows install, repair, upgrade, shortcuts, diagnostics and uninstall checks passed.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("installer", type=Path)
    parser.add_argument("--disposable-runner", action="store_true", required=True)
    args = parser.parse_args()
    if os.name != "nt":
        parser.error("Windows only")
    # CI retains this folder on failure for diagnosing installer/shortcut errors.
    work = Path("target/windows-install-test").resolve()
    work.mkdir(parents=True, exist_ok=True)
    main(args.installer.resolve(), work)
