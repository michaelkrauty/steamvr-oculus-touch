# Setup

## Build dependencies

The supported build target is Linux x86-64. Rust 1.85 or newer, Cargo, Meson 1.3 or newer, and Python 3.9 or newer are required; install a stable toolchain using [the official Rust instructions](https://www.rust-lang.org/tools/install) if your distribution packages are older. The OpenVR headers and modified OpenHMD sources are vendored; the build does not need a system OpenVR SDK. Native dependencies are dynamically linked, so build on the distribution where you will run it.

On Debian/Ubuntu with a suitable Rust toolchain:

```sh
sudo apt install build-essential pkg-config meson ninja-build clang libclang-dev libhidapi-dev libusb-1.0-0-dev libopencv-dev libjpeg-dev libpipewire-0.3-dev libspa-0.2-dev python3 procps
```

On Arch Linux:

```sh
sudo pacman -S --needed base-devel rust meson ninja clang hidapi libusb opencv libjpeg-turbo pipewire glib2 gstreamer gst-plugins-base python procps-ng
```

Hosted CI builds on Ubuntu 24.04; this identifies the build environment, not a SteamVR runtime compatibility guarantee. Check the CI workflow for the declared CI dependency set. Package availability and SteamVR compatibility vary by distribution. Source builds require internet access for Cargo's locked dependencies; native dependencies are resolved from the system.

## Device access

Run the daemon as your normal desktop user. Do not run it as root. Install the rule once:

```sh
sudo install -m 0644 udev/70-cv1touch.rules /etc/udev/rules.d/70-cv1touch.rules
sudo udevadm control --reload-rules
```

Reconnect the CV1 headset and sensors after reloading the rules. The rule grants the active local seat user access through logind ACLs to Oculus USB and hidraw devices. A remote-only or headless session may need administrator-managed device-group permissions instead; the provided rule intentionally does not make devices world-writable.

Use USB ports/controllers with enough bandwidth. If streams repeatedly stop or report incomplete frames, check the kernel USB log and try distributing sensors between USB controllers. Additional sensors do not help when their transfers cannot be sustained.

## Install

Quit SteamVR, then run `python3 scripts/install.py` from the repository root. A clone of the `v0.1.0` tag or an extracted [v0.1.0 source archive](https://github.com/michaelkrauty/steamvr-oculus-touch/releases/tag/v0.1.0) works; Git is not required to install from an archive. The installer builds locally against your system's native dependencies. It installs:

| Location | Purpose |
| --- | --- |
| `~/.local/bin/cv1touch`, `cv1touchd` | CLI and daemon |
| `~/.local/lib/cv1touch` | Runtime library, driver package, installer manifest |
| `$XDG_DATA_HOME/cv1touch/steamvr/cv1touch` | Driver registered with SteamVR; defaults to `~/.local/share` |
| `$XDG_CONFIG_HOME/systemd/user/cv1touchd.service` | User service; defaults to `~/.config` |

The installer refuses to overwrite unmanaged or modified installed files, including the registered SteamVR driver package. Driver ownership is recorded with SHA-256 hashes; a legacy package without ownership metadata is adopted only when both files exactly match the trusted previous installation. Existing driver conflicts are checked before the installer replaces prefix files; back up local changes and remove the conflicting package manually before retrying. For a manual deployment, use `--no-service` and/or `--no-steamvr`; `--prefix PATH` changes the binary/library installation prefix. `--no-build` reuses a release build, `--target-dir PATH` selects its Cargo output, and `--artifacts DIR` installs a prepared bundle. Run `python3 scripts/install.py --help` for all options. `cv1touch steamvr check` checks registered-driver ownership without changing files or registration.

SteamVR is discovered from its OpenVR path registry and common native Steam paths. Set `CV1TOUCH_STEAMVR` to the SteamVR runtime directory if needed, and `CV1TOUCH_STEAM_ROOT` to the Steam directory containing `config/steamvr.vrsettings` if discovery is ambiguous. Export these for installation and CLI commands; add them to a systemd user-service override when the daemon needs custom paths.

## Guided setup (development `main`)

The following commands are unreleased and available on development `main`, not in the v0.1.0 quick start:

```sh
cv1touch setup --check
cv1touch setup --check --json
cv1touch setup
```

The check commands are read-only: they report camera, gravity, render, input, and registration stages as ready, missing, invalid, unknown, stale, or blocked. They do not start recordings or change configuration. A ready stage means its saved dependencies are reusable; it does not certify current physical tracking accuracy. Use the status dashboard and the final SteamVR checks below to verify operation.

Complete installation and device access first, install matching CLI and daemon builds, start the daemon, wake both already-paired controllers, and check that every expected sensor receives frames. Captures require an interactive terminal. SteamVR and its independently tracked headset must be available for headset registration. Setup does not pair controllers, reset hardware, change firmware, grant permissions, or start/restart services for you.

Physical instructions and required actions are bold in interactive terminals, while diagnostic output stays plain. `NO_COLOR` or `TERM=dumb` disables emphasis; JSON reports remain plain.

The guide runs only the stages that need work and keeps a private completion journal at `$XDG_STATE_HOME/cv1touch/setup.json` (default `~/.local/state/cv1touch/setup.json`). It keeps exact recording paths and checks configuration dependencies. Rerunning with the same completed configuration reuses it without another capture. Changes to a full camera-world calibration require gravity and registration review; render and input settings have their own dependencies. Input reuse is bound to the physical controllers' factory identities. Interrupted or rejected work does not count as completed; fix the reported prerequisite and rerun the guide.

Existing camera, gravity, render, and registration files without journal evidence are **unknown**, rather than silently trusted or discarded. The guide lets you deliberately adopt them after visual verification or perform a new capture. Before adopting, check overlapping sensor coverage, level tracking, correct rendered controller offsets, and controller position/orientation relative to the SteamVR headset throughout the play area. Adoption records your verification and retains the existing calibration files; it does not perform a new accuracy measurement. If you cannot verify a stage, recapture it. Identity-bound valid input settings can be reused independently.

Follow each physical prompt fully: move both controllers slowly across overlapping camera views and varied orientations, then keep both controllers rigid against the headset while turning, nodding, and tilting for registration. An accepted fresh camera solve also establishes gravity alignment, so the guide completes both stages from that capture. The separate `calibrate gravity` command remains available for additional levelling refinement; when the guide needs a separate gravity capture, hold the controllers still in at least 12 substantially different orientations. Keep both controllers awake while fitting and tracking reloads; press a face button if they sleep. The guide retains the existing solver acceptance gates. It cannot infer a reliable headset-space transform from unrelated head and hand movement. Input calibration uses four shared phases for both hands: neutral, full trigger, full grip, then full stick circles, totalling 16 seconds of timed capture plus confirmations.

Input calibration precedes registration, leaving headset movement as the final physical step. The guide first shows a separate controls and haptic check while the headset stays off. It checks X on the left controller and A on the right controller and asks you to confirm each controller's haptic pulse with another button press. Confirm only if you felt the buzz; cancel and fix haptics if it is missing.

After both confirmations, the guide shows the headset preparation and motion instructions in short groups, before you put on the headset or start recording. Keep SteamVR running with its headset tracking, put the headset and both controllers within reach, and read the attachment instructions. Put on the headset, then release and press A on the right controller to start a ten-second positioning countdown conveyed by pulses in both controllers. Hold both controllers rigidly against the headset without covering its tracking sensors or the Touch rings. A distinct double pulse signals recording has started; make two slow turns each way, then nod and tilt repeatedly.

Release and press A on the right controller again to finish the capture, or B on the right controller to cancel without installing a fit; Enter and Ctrl+C remain available from the terminal. Recording stops after 150 seconds at most, with a longer gentle buzz signalling it has stopped. Quitting or interrupting preparation starts no registration recording. If a capture is interrupted or rejected, rerun setup to resume the incomplete stage; earlier completed stages remain reusable.

After setup, verify both hands in SteamVR, input delivery, haptics, occlusion recovery, sleeping/waking, and a daemon restart as described below. The guided flow has automated coverage; live hardware validation of this unreleased flow is still pending. Linux remains the supported target; Windows is unsupported.

The manual commands below remain available for explicit recordings, inspection, alternative registration paths, and expert control.

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
6. Run `cv1touch calibrate inputs`. In v0.1.0, the guide captures each controller separately. On development `main`, both hands share four phases: released triggers/grips and centered sticks, full trigger presses, full grip presses, then full thumbstick circles. Follow the prompts, press firmly at the normal mechanical stop, and move the stick against its gate through several complete circles. Settings apply immediately and survive restarts; SteamVR can stay open. See [input calibration](input-calibration.md) for reset and expert controls.
7. Verify position and orientation in SteamVR. Run `cv1touch input-test` and follow its button prompts with a focused VR app running. The tool checks the exact supported SteamVR inputs and reports menu/thumb-rest delivery as a manual app check. Test haptics with a game or `cv1touch haptic left 0.1 100 0.5`. Check occlusion recovery, sleeping/waking, both hands, and a daemon restart before relying on the setup.

Sensor movement requires recalibration or `calibrate refine`/`calibrate place` with a new observable capture. A new SteamVR room setup or a substantial non-rigid base-station re-solve can require repeating headset registration. See [troubleshooting](troubleshooting.md).

## Add or remove sensors

Connect each new sensor, then restart `cv1touchd`. Check `cv1touch status --json`: `sensor_total` should match the number admitted, every sensor's `frames` counter should advance, and a new uncalibrated sensor has `placed: 0`. There is no fixed sensor-count limit. USB transport, memory, and processing capacity still apply; distribute cameras across available USB controllers and check the logs if an open or stream start fails. To leave particular cameras closed, pass `--ignore-sensor SERIAL` to the daemon; the option is repeatable.

For a fresh layout, repeat camera calibration, gravity levelling, and SteamVR registration as above. To add just one sensor while retaining an existing calibrated room, capture both controllers moving in the new sensor's view while the calibrated sensors also see them:

```sh
cv1touch record start /tmp/cv1touch-add-sensor.rec
# Move both controllers with varied orientations in overlapping views, then:
cv1touch record stop
cv1touch calibrate place SERIAL /tmp/cv1touch-add-sensor.rec
```

Replace `SERIAL` with the new sensor's value from status. Inspect the fit first; repeat with `--write` only when the fit succeeds, then restart the daemon. This adds the sensor in the existing coordinate frame and preserves the other camera poses. Keep the capture and inspect tracking after the change. Removing a temporary sensor does not require deleting its calibration; disconnected entries remain available if you reconnect it. Restart after changing attached hardware.

The live status board lists every sensor. `camera_queue.dropped_frames` counts evictions from the bounded camera callback queues. `camera_queue.pending_frames_dropped` counts deferred frame copies evicted separately for each hand while waiting for its IMU samples; it can rise normally when a controller is idle or asleep, even with no callback backlog. Compare counter changes while both controllers are awake and moving. If drops persist during motion, inspect active-hand tracking, IMU/radio timing, and CPU load; stopped sensor frame counters also warrant checking USB transport.

## Upgrade

Quit SteamVR and select the desired release before running the installer. For a tagged shallow checkout, run these commands from the repository root, replacing `v0.1.0` with the desired published tag:

```sh
git fetch origin --tags
git checkout --detach v0.1.0
python3 scripts/install.py
systemctl --user restart cv1touchd
```

For a source-archive installation, extract the desired release into a new directory and run `python3 scripts/install.py` there, then restart the service.

The installer replaces managed binaries and keeps calibration/state. A running process can continue using its previous mapped code until restarted. SteamVR loads a new driver on its next start. Keep a known-good release available if you need rollback; newer configuration schemas may need corresponding backups.

## Remove

Quit SteamVR, then:

```sh
python3 scripts/install.py --uninstall
```

Use the same prefix and environment overrides as installation. The installer unregisters the driver, disables/removes its managed service, and removes unmodified managed files. Calibration, recordings, and SteamVR settings remain. `activateMultipleDrivers` remains enabled because other drivers may need it. Modified installed files are preserved and reported. Remove the udev rule manually only if no other Oculus tooling needs it.

## Optional sensor viewer

`tools/sensor-view` displays live raw sensor streams; `--annotated` shows blob labels. It requires PipeWire support at build time and GStreamer at runtime (`gst-launch-1.0`, PipeWire source, text overlay, and video sink plugins). Install `gstreamer1.0-tools gstreamer1.0-pipewire gstreamer1.0-plugins-base gstreamer1.0-x` on Debian/Ubuntu, or `gst-plugin-pipewire gst-plugins-base gst-plugins-good` on Arch. The viewer connects to streams only while open.
