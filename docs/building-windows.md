# Building on Windows (work in progress)

Windows support is being built. Today the SteamVR driver and the native engine build and pass their tests on Windows 11 x64. The daemon and the `cv1touch` command do not run on Windows yet, so there is nothing to install. This page is for developers.

Like the Linux build, the Windows build uses no Oculus software. The Rift headset needs no driver. The sensors will need a WinUSB driver package of our own, which is not written yet.

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
cargo test --lib
```

The first `vcpkg install` compiles OpenCV, which takes a long time. `build.rs` builds the engine with meson, links `openhmd`, and copies `openhmd-0.dll` and `libusb-1.0.dll` next to the executables Cargo builds.
