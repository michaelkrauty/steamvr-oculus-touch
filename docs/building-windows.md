# Building on Windows

For developers. To install, use the installer: [cv1touch on Windows](windows.md).

The Windows build uses no Oculus software. The sensors use Windows' WinUSB driver, which `cv1touch sensors install` assigns; no driver package or certificate of ours is involved.

## Tools

- Visual Studio 2022 Build Tools with "Desktop development with C++" (MSVC x64 and a Windows 11 SDK)
- LLVM (`clang-cl` and `libclang`), for example `winget install LLVM.LLVM`, in a version the installed MSVC standard library supports (MSVC 14.52 needs Clang 22 or newer)
- Python 3 with `pip install meson ninja`
- Rust for `x86_64-pc-windows-msvc`, from [rustup](https://www.rust-lang.org/tools/install)
- [vcpkg](https://github.com/microsoft/vcpkg)

Build the C and C++ code with `clang-cl`: it uses GNU builtins and C11 atomics that `cl.exe` lacks.

## SteamVR driver

From PowerShell in the repository:

```powershell
$env:CC = 'clang-cl'; $env:CXX = 'clang-cl'
meson setup build-driver --vsenv
meson test -C build-driver --print-errorlogs
cargo run --no-default-features --example driver_contract -- build-driver\driver\test_daemon_contract.exe
```

This builds `build-driver\driver\driver_cv1touch.dll`, which carries its own C and C++ runtime and exports only `HmdDriverFactory`. The last command checks the driver against the daemon's shared-memory and control code.

## Engine, daemon and tool

`vcpkg.json` pins hidapi, libusb and libjpeg-turbo. The `x64-windows-cv1touch` triplet in `windows\triplets` builds them as static libraries with the static C runtime, except libusb (LGPL-2.1), which ships as its own DLL.

```powershell
vcpkg install --x-manifest-root=. --overlay-triplets=windows\triplets --triplet x64-windows-cv1touch --x-install-root=vcpkg_installed
$env:CV1TOUCH_DEPS = "$PWD\vcpkg_installed\x64-windows-cv1touch"
cargo test --lib --bins
```

`build.rs` builds the engine with Meson and copies `openhmd-0.dll` and `libusb-1.0.dll` next to the built binaries. The Rust executables link the MSVC runtime statically, like the native libraries; do not override that, since the installer does not ship the Visual C++ Redistributable.

## Installer

```powershell
cargo build --locked --release --bins
powershell -ExecutionPolicy Bypass -File windows\bundle.ps1
& "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe" /DVersion=0.6.0 "/DBundle=$PWD\target\bundle" windows\installer\cv1touch.iss
```

`windows\bundle.ps1` collects the install into `target\bundle`, and Inno Setup 6 writes the installer to `target\installer`. `python windows/check_bundle.py target/bundle` rejects DLL dependencies the bundle does not ship.

CI (`.github\workflows\windows.yml`) builds the installer on every push and pull request, tests install, repair, upgrade and uninstall as a standard user on a clean hosted runner (`windows\test_standard_user.ps1`, which runs `windows\test_install.py`), and attaches the installer to the release of a `v*` tag. Do not run those tests on a configured PC.

## Type-checking without the engine

With `CV1TOUCH_CHECK_ONLY=1`, the build script skips the engine, so `cargo check` and `cargo clippy` work without its dependencies, on Windows or from Linux with the `x86_64-pc-windows-msvc` target (`libclang` is still needed). Linking fails in this mode.

```sh
CV1TOUCH_CHECK_ONLY=1 cargo clippy --locked --target x86_64-pc-windows-msvc --all-targets -- -D warnings
```
