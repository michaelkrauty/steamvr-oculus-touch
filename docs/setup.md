# Setup

## Build dependencies

The supported build target is Linux x86-64. Rust 1.85 or newer and Cargo are required; install a stable toolchain using [the official Rust instructions](https://www.rust-lang.org/tools/install) if your distribution packages are older. The OpenVR headers and modified OpenHMD sources are vendored; the build does not need a system OpenVR SDK. Native dependencies are dynamically linked, so build on the distribution where you will run it.

On Debian/Ubuntu with a suitable Rust toolchain:

```sh
sudo apt install build-essential pkg-config meson ninja-build clang libclang-dev libhidapi-dev libusb-1.0-0-dev libopencv-dev libjpeg-dev libpipewire-0.3-dev libspa-0.2-dev python3 procps
```

On Arch Linux:

```sh
sudo pacman -S --needed base-devel rust meson ninja clang hidapi libusb opencv libjpeg-turbo pipewire glib2 gstreamer gst-plugins-base python procps-ng
```

Check the CI workflow for the declared CI dependency set. Package availability and SteamVR compatibility vary by distribution. Source builds require internet access for Cargo's locked dependencies; native dependencies are resolved from the system.

## Device access

Run the daemon as your normal desktop user. Do not run it as root. Install the rule once:

```sh
sudo install -m 0644 udev/70-cv1touch.rules /etc/udev/rules.d/70-cv1touch.rules
sudo udevadm control --reload-rules
```

Reconnect the CV1 headset and sensors after reloading the rules. The rule grants the active local seat user access through logind ACLs to Oculus USB and hidraw devices. A remote-only or headless session may need administrator-managed device-group permissions instead; the provided rule intentionally does not make devices world-writable.

Use USB ports/controllers with enough bandwidth. If streams repeatedly stop or report incomplete frames, check the kernel USB log and try distributing sensors between USB controllers. Additional sensors do not help when their transfers cannot be sustained.

## Install

Quit SteamVR, then run `python3 scripts/install.py` from the clone. It installs:

| Location | Purpose |
| --- | --- |
| `~/.local/bin/cv1touch`, `cv1touchd` | CLI and daemon |
| `~/.local/lib/cv1touch` | Runtime library, driver package, installer manifest |
| `$XDG_DATA_HOME/cv1touch/steamvr/cv1touch` | Driver registered with SteamVR; defaults to `~/.local/share` |
| `$XDG_CONFIG_HOME/systemd/user/cv1touchd.service` | User service; defaults to `~/.config` |

The installer refuses to overwrite unmanaged or modified installed files. For a manual deployment, use `--no-service` and/or `--no-steamvr`; `--prefix PATH` changes the binary/library installation prefix. `--no-build` reuses a release build, `--target-dir PATH` selects its Cargo output, and `--artifacts DIR` installs a prepared bundle. Run `python3 scripts/install.py --help` for all options.

SteamVR is discovered from its OpenVR path registry and common native Steam paths. Set `CV1TOUCH_STEAMVR` to the SteamVR runtime directory if needed, and `CV1TOUCH_STEAM_ROOT` to the Steam directory containing `config/steamvr.vrsettings` if discovery is ambiguous. Export these for installation and CLI commands; add them to a systemd user-service override when the daemon needs custom paths.

## Calibrate and align

Camera calibration estimates the sensors' poses in a shared world. Registration then maps that world into the SteamVR tracking space. These are separate operations. A previously paired CV1 headset supplies the radio link; your other SteamVR headset supplies the reference pose.

1. Start the service and wake both controllers. `cv1touch status --json` should show every connected sensor receiving frames. Ensure every sensor has a useful view of the play area and overlaps another sensor. `cv1touch status` is a continuously updating dashboard; Ctrl+C exits it.
2. Run `cv1touch calibrate`. Follow the timed 60-second capture: move both controllers slowly throughout the space and turn them through varied orientations. Spend time in overlapping views. The solver rejects insufficient or inconsistent observations and preserves the previous calibration when it succeeds. A rejected solve is a request for a better capture, not a reason to use `--force` routinely.
3. Level the world with gravity. Start a recording, then hold each controller still for at least 2 seconds in **at least 12 substantially different orientations**, including upside down and sideways. Move gently between holds, keeping optical visibility. A minute or two is usually enough. Use a new absolute path for each recording:

   ```sh
   cv1touch record start /tmp/cv1touch-gravity.rec
   # Make the static holds, then:
   cv1touch record stop
   cv1touch calibrate gravity /tmp/cv1touch-gravity.rec --write
   systemctl --user restart cv1touchd
   ```

   The gravity solver needs diverse, observable holds and reports an error if it cannot fit them. Keep the recording for diagnosis. Repeat gravity levelling after a new camera calibration.
4. Start SteamVR with its own headset tracking normally. Run `cv1touch fit-render` using the calibration recording, or supply a recording path explicitly. This computes the visual-model offsets from the installed SteamVR meshes and the controllers' factory LED geometry.
5. Run `cv1touch register --headset` and follow its capture instructions; it records, fits, saves, and reloads the alignment. For explicit control of the recording and fit, use:

   ```sh
   cv1touch eval-capture --free 150
   ```

   During the capture, hold or securely attach both Touch controllers to the headset so their relationship to it stays rigid. Follow the command's instructions about when to attach, move, and detach; the fitter isolates a rigid span from the surrounding handling. With the headset tracking, turn through two circles in each direction, then nod and tilt; keep the controllers visible to the CV1 sensors. The capture prints its output base path. Use that exact base, without guessing the newest recording:

   ```sh
   cv1touch register --follow BASE --reference hmd --write
   cv1touch alignment reload
   ```

   Replace `BASE` with the printed path. The fit must have sufficient motion and agreement. If the two controllers move relative to the headset, repeat the capture. No separate tracked controller is required for this path.
6. Run `cv1touch calibrate inputs`. For each controller, the guide captures released triggers/grips and the centered stick, full trigger and grip presses, then full thumbstick circles. Follow the prompts, press firmly at the normal mechanical stop, and move the stick against its gate through several complete circles. Settings apply immediately and survive restarts; SteamVR can stay open. See [input calibration](input-calibration.md) for reset and expert controls.
7. Verify position and orientation in SteamVR. Run `cv1touch input-test` and follow its button prompts with a focused VR app running. The tool checks the exact supported SteamVR inputs and reports menu/thumb-rest delivery as a manual app check. Test haptics with a game or `cv1touch haptic left 0.1 100 0.5`. Check occlusion recovery, sleeping/waking, both hands, and a daemon restart before relying on the setup.

Sensor movement requires recalibration or `calibrate refine`/`calibrate place` with a new observable capture. A new SteamVR room setup or a substantial non-rigid base-station re-solve can require repeating headset registration. See [troubleshooting](troubleshooting.md).

## Upgrade

Quit SteamVR, update the clone, run `python3 scripts/install.py`, then restart the service:

```sh
systemctl --user restart cv1touchd
```

The installer replaces managed binaries and keeps calibration/state. A running process can continue using its previous mapped code until restarted. SteamVR loads a new driver on its next start. Keep a known-good release available if you need rollback; newer configuration schemas may need corresponding backups.

## Remove

Quit SteamVR, then:

```sh
python3 scripts/install.py --uninstall
```

Use the same prefix and environment overrides as installation. The installer unregisters the driver, disables/removes its managed service, and removes unmodified managed files. Calibration, recordings, and SteamVR settings remain. `activateMultipleDrivers` remains enabled because other drivers may need it. Modified installed files are preserved and reported. Remove the udev rule manually only if no other Oculus tooling needs it.

## Optional sensor viewer

`tools/sensor-view` displays live raw sensor streams; `--annotated` shows blob labels. It requires PipeWire support at build time and GStreamer at runtime (`gst-launch-1.0`, PipeWire source, text overlay, and video sink plugins). Install `gstreamer1.0-tools gstreamer1.0-pipewire gstreamer1.0-plugins-base gstreamer1.0-x` on Debian/Ubuntu, or `gst-plugin-pipewire gst-plugins-base gst-plugins-good` on Arch. The viewer connects to streams only while open.
