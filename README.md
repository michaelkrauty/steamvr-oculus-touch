# SteamVR Oculus Touch

Use original Oculus Rift CV1 Touch controllers in SteamVR on Linux and Windows, without Oculus software.

A Rust daemon tracks the controllers from the CV1 sensors and the controllers' IMUs, and a native SteamVR driver gives SteamVR their poses, buttons, capacitive touch, triggers, sticks and haptics.

**Status:** v0.7.0. Tested and working on Linux. Untested on Windows, but it should work. This project is independent of Meta, Oculus and Valve.

## What you need

- Rift CV1 Touch controllers, already paired to a CV1 headset. Rift S and Quest controllers are unsupported, and pairing is not implemented.
- The CV1 headset connected over USB. It is only the controllers' radio receiver: its display is not used.
- Two or more CV1 sensors with overlapping views of the play area.
- SteamVR with another headset that SteamVR already tracks.
- Linux x86-64 or Windows 11 x64. On Linux the default install runs the daemon as a systemd user service.

## Install

**Windows:** download `cv1touch-setup-0.7.0.exe` from the [v0.7.0 release](https://github.com/michaelkrauty/steamvr-oculus-touch/releases/tag/v0.7.0) and follow [cv1touch on Windows](docs/windows.md).

**Linux:** start SteamVR once so it writes where it is installed, quit it, install the [build dependencies](docs/setup.md#build-dependencies), then:

```sh
git clone --branch v0.7.0 --depth 1 https://github.com/michaelkrauty/steamvr-oculus-touch.git
cd steamvr-oculus-touch
python3 scripts/install.py
```

The installer builds from source, installs into `~/.local`, registers the SteamVR driver and starts a user service. It never runs sudo. When it finishes, it prints anything left to do, such as installing the [udev rule](docs/setup.md#device-access) or adding `~/.local/bin` to your PATH.

Then wake both controllers and run:

```sh
cv1touch setup
```

The guide walks through each calibration step and remembers finished ones. Its last step needs SteamVR running with your headset tracking. If something stops working later, run `cv1touch doctor`.

## Documentation

- [Setup, upgrades and removal](docs/setup.md)
- [cv1touch on Windows](docs/windows.md)
- [Input calibration](docs/input-calibration.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Architecture and expert controls](docs/architecture.md)
- [Building on Windows](docs/building-windows.md)
- [Contributing](CONTRIBUTING.md)

## Limitations

- Position needs the sensors to see the controller's LEDs. Rotation continues through occlusion from the IMU.
- If SteamVR re-solves its base stations differently, the controllers can drift from your hands. cv1touch follows the base station SteamVR tracks the headset from; if the controllers are still off, redo Headset alignment in `cv1touch setup`.
- A headset that base stations do not track, such as a Rift S or a Quest, gives cv1touch nothing to follow: redo Headset alignment when the controllers no longer line up.
- Battery levels come from controller firmware and may not warn of a weak battery.

## License and credits

Apache License 2.0 ([LICENSE](LICENSE)). Vendored components keep their own licenses ([third-party notices](THIRD_PARTY_NOTICES.md)). This project builds on the CV1 tracking work in [OpenHMD](https://github.com/OpenHMD/OpenHMD) and [Jan Schmidt's OpenHMD fork](https://github.com/thaytan/OpenHMD/tree/rift-kalman-filter), and uses Valve's [OpenVR SDK](https://github.com/ValveSoftware/openvr). SteamVR's input profiles and render models are used at runtime and not redistributed.
