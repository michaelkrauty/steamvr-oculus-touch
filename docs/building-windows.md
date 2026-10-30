# Building on Windows

This page is for developers. Testers and players want [cv1touch on Windows](windows.md): one installer, built by CI from the bundle `windows\bundle.ps1` collects, with the Inno Setup script `windows\installer\cv1touch.iss`. The SteamVR driver, the native engine, the daemon and the `cv1touch` command build and pass their tests on Windows 11 x64.

Like the Linux build, the Windows build uses no Oculus software. The Rift headset needs no driver. The sensors run with Windows' own WinUSB driver, which `cv1touch sensors install` gives them (it asks for permission once); `cv1touch sensors` says whether each sensor plugged in has it, and `cv1touch sensors remove` takes it off again. No driver package of our own and no certificate is involved.

## Tools

- Visual Studio 2022 Build Tools with "Desktop development with C++" (MSVC x64 and a Windows 11 SDK)
- LLVM (`clang-cl` and `libclang`), for example `winget install LLVM.LLVM`
- Python 3 with `pip install meson ninja`
- Rust for `x86_64-pc-windows-msvc`, from [rustup](https://www.rust-lang.org/tools/install)
- [vcpkg](https://github.com/microsoft/vcpkg), for the engine's dependencies

Build with `clang-cl`. The shared-memory helpers and the engine use GNU C builtins and C11 atomics that `cl.exe` lacks.

## SteamVR driver only

From PowerShell, in the repository:

```powershell
$env:CC = 'clang-cl'; $env:CXX = 'clang-cl'
meson setup build-driver --vsenv
meson test -C build-driver --print-errorlogs
```

This builds `build-driver\driver\driver_cv1touch.dll` and its tests. The DLL carries its own C and C++ runtime and exports only `HmdDriverFactory`.

To check the driver against the daemon's own shared-memory and control code:

```powershell
cargo run --no-default-features --example driver_contract -- build-driver\driver\test_daemon_contract.exe
```

## Engine

`vcpkg.json` pins the dependencies: hidapi, libusb, OpenCV (calib3d) and libjpeg-turbo. The `x64-windows-cv1touch` triplet in `windows\triplets` builds them as static libraries on the static C runtime. The exception is libusb, which is LGPL-2.1 and ships as its own DLL.

```powershell
vcpkg install --x-manifest-root=. --overlay-triplets=windows\triplets --triplet x64-windows-cv1touch --x-install-root=vcpkg_installed
$env:CV1TOUCH_DEPS = "$PWD\vcpkg_installed\x64-windows-cv1touch"
cargo test --lib --bins
```

The first `vcpkg install` compiles OpenCV, which takes a long time. `build.rs` builds the engine with meson, links `openhmd`, and copies `openhmd-0.dll` and `libusb-1.0.dll` next to the executables Cargo builds.

The repository's Cargo configuration statically links the MSVC runtime into the Rust executables, just as the native libraries do. Do not override it with a dynamic CRT build: the installer does not require or ship the Visual C++ Redistributable. `python windows/check_bundle.py target/bundle` uses `llvm-readobj` to reject unshipped DLL dependencies, including dependencies a CI runner happens to have installed.

On a disposable hosted runner, `windows/test_standard_user.ps1` creates a temporary non-administrator account and runs `windows/test_install.py` against the actual installer. It checks upgrade, repair, Start-menu command quoting, daemon stop/start, report creation, canceled uninstall and removal. It repeats with spaces, Unicode and an ampersand in the installation path, preserves a settings fixture, loads the packaged driver DLL, and checks that a synthetic process named `vrserver.exe` prevents replacement/removal. The account and its profile are removed afterward. Do not run this on a configured PC. These tests cover missing prerequisites and synthetic SteamVR assets, not actual SteamVR or physical calibration. A separate fast job compiles the Inno script with placeholder payloads; those placeholders are never run or published.

The hosted Windows library job also creates a uniquely named synthetic ROOT device, selects the inbox WinUSB driver through the production SetupAPI path, checks the binding, and removes the node. This opt-in test requires an elevated disposable runner; it never rebinds physical devices. A synthetic device cannot stream USB data, so it does not validate sensor transfers, real-device recovery, or the UAC interaction.

## Bundle and installer

From a release build with the engine, `windows\bundle.ps1` collects what a Windows install is into `target\bundle` (it reads the vcpkg ports' licences and versions from `CV1TOUCH_DEPS`), and Inno Setup 6 packs the installer into `target\installer`:

```powershell
cargo build --locked --release --bins
powershell -ExecutionPolicy Bypass -File windows\bundle.ps1
& "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe" /DVersion=0.3.2 "/DBundle=$PWD\target\bundle" windows\installer\cv1touch.iss
```

CI does both on every push and pull request (`.github\workflows\windows.yml`), installs and uninstalls the result silently on a clean runner, and attaches it to the release of a tag `v*`.

## Type-checking without the engine

With `CV1TOUCH_CHECK_ONLY=1` the build script does not build the engine, so `cargo check` and `cargo clippy` work without its dependencies, on Windows or from Linux with the `x86_64-pc-windows-msvc` target installed (`libclang` is still needed, for the bindings). A build that links fails. CI's hosted Windows jobs check the daemon and the command this way:

```sh
CV1TOUCH_CHECK_ONLY=1 cargo clippy --locked --target x86_64-pc-windows-msvc --all-targets -- -D warnings
```
