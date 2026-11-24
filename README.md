# SteamVR Oculus Touch

Use first-generation Oculus Rift CV1 Touch controllers in SteamVR on Linux and Windows, without the Oculus runtime.

SteamVR Oculus Touch combines a Rust optical/inertial tracker and daemon, a native SteamVR driver, calibration tools, and a modified OpenHMD acquisition engine in one repository. SteamVR receives controller poses, buttons, capacitive touch, triggers, sticks, and haptics through its normal driver interfaces.

**Status:** v0.6.0. Tested and working on Linux. Untested on Windows, but it should work. This project is independent of Meta, Oculus, and Valve.

## Hardware and software

- Original **Rift CV1 Touch** controllers, already paired to a CV1 headset. Rift S and Quest Touch controllers are unsupported. Pairing is not implemented here.
- A **CV1 headset connected over USB**, used as the controllers' radio receiver. The CV1 display is not used by this driver.
- Two or more CV1 tracking sensors with overlapping views of the play area ([adding sensors](docs/setup.md#add-or-remove-sensors)).
- A working native Linux SteamVR installation and an independently tracked SteamVR headset for coordinate registration. A separate reference controller is optional.
- Linux x86-64, Rust 1.85 or newer, and the native build dependencies below. The default service setup uses systemd user services and logind device access.

This repository supplies controllers, not a CV1 display/headset driver. Sandboxed Steam installations are untested. Windows has an installer: [cv1touch on Windows](docs/windows.md).

## Quick start

**Windows:** download `cv1touch-setup-0.6.0.exe` from the [v0.6.0 release](https://github.com/michaelkrauty/steamvr-oculus-touch/releases/tag/v0.6.0), then follow [cv1touch on Windows](docs/windows.md). The installer is unsigned; sensor-driver setup asks for elevation separately.

**Linux:**

Start SteamVR once, so that it records where it is installed, then quit it before installing. Install the build dependencies from [setup](docs/setup.md#build-dependencies), then:

```sh
git clone --branch v0.6.0 --depth 1 https://github.com/michaelkrauty/steamvr-oculus-touch.git
cd steamvr-oculus-touch
python3 scripts/install.py
```

Linux v0.6.0 is distributed as source, including [tagged source archives](https://github.com/michaelkrauty/steamvr-oculus-touch/releases/tag/v0.6.0). Native dependencies are dynamically linked; build locally on the distribution where you will run it. For development on `main`, see [Contributing](CONTRIBUTING.md).

The installer builds with the locked dependency graph, installs into `~/.local`, registers the SteamVR driver, backs up SteamVR settings before enabling multiple drivers, and starts a user service. It never runs sudo. When it ends, it prints what is still to do: the two commands that install the [udev rule](docs/setup.md#device-access) when that rule is not installed and the headset and sensors cannot be opened yet, then reconnecting the headset and sensors, and the line that puts `~/.local/bin` on your PATH if it is not there.

Then wake both controllers by pressing a button on each, and run:

```sh
cv1touch setup
```

It walks you through every calibration step, one at a time, and remembers the finished ones if you stop ([guided setup](docs/setup.md#guided-setup)). Its last step needs SteamVR running with your headset tracking. Calibration is specific to your sensor placement, so none is shipped. The daemon the installer started keeps running, starts again each time you log in, and reconnects to the devices. If something stops working later, `cv1touch doctor` says what is wrong.

## Documentation

- [Setup, calibration, upgrades, and removal](docs/setup.md)
- [Windows installation](docs/windows.md)
- [Trigger, grip, and thumbstick calibration](docs/input-calibration.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Architecture, coordinates, and expert controls](docs/architecture.md)
- [Validation and measurement limits](docs/validation.md)
- [Contributing](CONTRIBUTING.md)

The command-line tools are named `cv1touch` and `cv1touchd`. Run `cv1touch help expert` for the full command list. You can run the binaries directly, use a custom installation prefix, control the daemon over its local socket, and replay recordings offline. Service management is optional.

## Limitations

Tracking needs camera visibility. Occlusion, too few visible LEDs, or insufficient overlapping views reduce tracking quality. The radio and USB transport set a latency floor; local software timing is not end-to-end motion-to-photon latency. Registration can drift when SteamVR changes its tracking-space solution; base-station following handles approximately rigid changes, while significant disagreement requires repeating registration. A headset that base stations do not track (such as a Rift S or a Quest) gives nothing to follow: if the controllers are no longer where your hands are, redo Headset alignment in `cv1touch setup` (on Windows, in **Set up cv1touch**). Battery reports come from controller firmware and may not predict a weak battery or intermittent contact reliably.

## License and credits

Original SteamVR Oculus Touch work is available under the [Apache License 2.0](LICENSE). Vendored components retain their own licenses; see [third-party notices](THIRD_PARTY_NOTICES.md). In particular, this project builds on the substantial CV1 acquisition and tracking work in [OpenHMD](https://github.com/OpenHMD/OpenHMD) and [Jan Schmidt's OpenHMD fork](https://github.com/thaytan/OpenHMD/tree/rift-kalman-filter), and uses Valve's [OpenVR SDK](https://github.com/ValveSoftware/openvr). SteamVR's installed profiles and render models are used at runtime and are not redistributed.
