# SteamVR Oculus Touch

Use first-generation Oculus Rift CV1 Touch controllers in SteamVR on Linux, without the Oculus runtime.

SteamVR Oculus Touch combines a Rust optical/inertial tracker and daemon, a native SteamVR driver, calibration tools, and a modified OpenHMD acquisition engine in one repository. SteamVR receives controller poses, buttons, capacitive touch, triggers, sticks, and haptics through its normal driver interfaces.

**Status:** early release, tested on one Linux installation with one left/right controller pair and up to five CV1 sensors. The tracking stack has been used in VR games; broader hardware, distribution, headset, and application compatibility still needs testing. This project is independent of Meta, Oculus, and Valve.

## Hardware and software

- Original **Rift CV1 Touch** controllers, already paired to a CV1 headset. Rift S and Quest Touch controllers are unsupported. Pairing is not implemented here.
- A **CV1 headset connected over USB**, used as the controllers' radio receiver. The CV1 display is not used by this driver.
- CV1 tracking sensors. Use at least two with overlapping views for reliable position tracking. There is no fixed sensor-count cap; USB bandwidth, CPU capacity, and available memory determine the usable count. One sensor has weaker depth observability and is not the recommended configuration. See [adding sensors](docs/setup.md#add-or-remove-sensors).
- A working native Linux SteamVR installation and an independently tracked SteamVR headset for coordinate registration. A separate reference controller is optional.
- Linux x86-64, Rust 1.85 or newer, and the native build dependencies below. The default service setup uses systemd user services and logind device access.

This repository supplies controllers, not a CV1 display/headset driver. Sandboxed Steam installations and Windows are not validated.

## Quick start

Quit SteamVR before installing. Install the build dependencies from [setup](docs/setup.md#build-dependencies), then:

```sh
git clone https://github.com/michaelkrauty/steamvr-oculus-touch.git
cd steamvr-oculus-touch
python3 scripts/install.py
```

The installer builds with the locked dependency graph, installs into `~/.local`, registers the SteamVR driver, backs up SteamVR settings before enabling multiple drivers, and starts a user service. It never runs sudo. Put `~/.local/bin` on your PATH.

Install the supplied [udev rule](docs/setup.md#device-access) once, reconnect the USB devices, then run:

```sh
cv1touch doctor
cv1touch status --json
```

The initial doctor check reports missing calibration/alignment until setup is complete. Follow [setup and calibration](docs/setup.md#calibrate-and-align). Calibration is specific to your camera placement; no personal room calibration is shipped. Setup includes camera calibration, gravity levelling, registration to your SteamVR headset, and guided trigger/grip/thumbstick calibration. Once complete, the daemon starts automatically and reconnects to devices.

## Documentation

- [Setup, calibration, upgrades, and removal](docs/setup.md)
- [Trigger, grip, and thumbstick calibration](docs/input-calibration.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Architecture, coordinates, and expert controls](docs/architecture.md)
- [Validation and measurement limits](docs/validation.md)
- [Contributing](CONTRIBUTING.md)

The command-line tools are named `cv1touch` and `cv1touchd`. Run `cv1touch --help` for the full command list. You can run the binaries directly, use a custom installation prefix, control the daemon over its local socket, and replay recordings offline. Service management is optional.

## Limitations

Tracking needs camera visibility. Occlusion, too few visible LEDs, or insufficient overlapping views reduce tracking quality. The radio and USB transport set a latency floor; local software timing is not end-to-end motion-to-photon latency. Registration can drift when SteamVR changes its tracking-space solution; base-station following handles approximately rigid changes, while significant disagreement requires repeating registration. Battery reports come from controller firmware and may not predict a weak battery or intermittent contact reliably.

## License and credits

Original SteamVR Oculus Touch work is available under the [Apache License 2.0](LICENSE). Vendored components retain their own licenses; see [third-party notices](THIRD_PARTY_NOTICES.md). In particular, this project builds on the substantial CV1 acquisition and tracking work in [OpenHMD](https://github.com/OpenHMD/OpenHMD) and [Jan Schmidt's OpenHMD fork](https://github.com/thaytan/OpenHMD/tree/rift-kalman-filter), and uses Valve's [OpenVR SDK](https://github.com/ValveSoftware/openvr). SteamVR's installed profiles and render models are used at runtime and are not redistributed.
