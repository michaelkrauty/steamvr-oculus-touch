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

Quit SteamVR, then run `python3 scripts/install.py` from the repository root. A clone of the `v0.2.0` tag or an extracted [v0.2.0 source archive](https://github.com/michaelkrauty/steamvr-oculus-touch/releases/tag/v0.2.0) works; Git is not required to install from an archive. The installer builds locally against your system's native dependencies. It installs:

| Location | Purpose |
| --- | --- |
| `~/.local/bin/cv1touch`, `cv1touchd` | CLI and daemon |
| `~/.local/lib/cv1touch` | Runtime library, driver package, installer manifest |
| `$XDG_DATA_HOME/cv1touch/steamvr/cv1touch` | Driver registered with SteamVR; defaults to `~/.local/share` |
| `$XDG_CONFIG_HOME/systemd/user/cv1touchd.service` | User service; defaults to `~/.config` |

The installer refuses to overwrite unmanaged or modified installed files, including the registered SteamVR driver package. Driver ownership is recorded with SHA-256 hashes; a legacy package without ownership metadata is adopted only when both files exactly match the trusted previous installation. Existing driver conflicts are checked before the installer replaces prefix files; back up local changes and remove the conflicting package manually before retrying. For a manual deployment, use `--no-service` and/or `--no-steamvr`; `--prefix PATH` changes the binary/library installation prefix. `--no-build` reuses a release build, `--target-dir PATH` selects its Cargo output, and `--artifacts DIR` installs a prepared bundle. Run `python3 scripts/install.py --help` for all options. `cv1touch steamvr check` checks registered-driver ownership without changing files or registration.

SteamVR is discovered from its OpenVR path registry and common native Steam paths. Set `CV1TOUCH_STEAMVR` to the SteamVR runtime directory if needed, and `CV1TOUCH_STEAM_ROOT` to the Steam directory containing `config/steamvr.vrsettings` if discovery is ambiguous. Export these for installation and CLI commands; add them to a systemd user-service override when the daemon needs custom paths.

## Guided setup

After installing, calibrate with the guide:

```sh
cv1touch setup --check
cv1touch setup --check --json
cv1touch setup
cv1touch setup --no-slip-check
```

`setup --check` is read-only: it lists each stage as done, not done, saved but not verified, needing redoing, or waiting on something, and starts nothing. A done stage means its saved settings still fit together; it does not measure tracking accuracy. Check that in SteamVR afterwards.

Before you start: install the CLI and daemon from the same build, start the daemon, wake both already-paired controllers, and make sure every sensor shows frames. Headset alignment needs SteamVR running with the headset tracking. Setup never pairs controllers, resets hardware, changes firmware or permissions, or restarts services.

`cv1touch setup` shows the checks (only the failing ones are explained), the stage list, and a prompt for the next stage: Enter starts it, `q` quits. It runs only the stages that need work and remembers finished ones in `$XDG_STATE_HOME/cv1touch/setup.json` (default `~/.local/state/cv1touch/setup.json`), so rerunning it continues where you left off. Each finished stage prints one line with its headline result; solver output goes to a log next to the recordings, shown when a stage fails. A failed stage offers to try again right away; earlier stages stay done. Instructions are bold in a terminal; `NO_COLOR` or `TERM=dumb` turns that off (and a dumb terminal gets ASCII marks), and JSON reports stay plain.

Settings saved before you used the guide show as saved but not verified. `k` keeps such a stage as it is: do that only if tracking already looks right (sensor coverage, level, controller models, and the controllers' position against the SteamVR headset across the play area). Keeping records your judgement; it measures nothing. When in doubt, redo the stage.

Whenever your hands are full, A on the right controller does what Enter does, and timed steps are marked by a haptic pulse in both controllers and a beep from the speakers: one starts, two end. The stages:

- **Sensor positions**: works out where each of your Rift sensors is in the room. The sensors find their places by watching the lights in the controllers' rings, and moments when two sensors see a controller at once matter most. For 60 seconds, slowly wave both controllers around the whole play area: high and low, close to each sensor and far from it, turning your wrists so the rings face every way. A good result also levels the floor.
- **Floor level** (only when the sensor positions did not already do it): finds which way is down, so the floor in VR is level. Each controller can feel gravity, but only while held perfectly still, and only accurately in many different positions. Twelve times: turn and tilt both controllers into a new position with their rings in view of the sensors, hold them still, press A, and keep them still until the double beep (3 seconds).
- **Controller model**: lines up the controller you see in VR with the real one. Each controller reports where its lights are, and the guide moves SteamVR's 3D model to match. Nothing to aim or hold: keep both controllers awake for 5 seconds.
- **Buttons and sticks**: measures how far each trigger, grip and thumbstick moves, so in SteamVR a full pull reads as a full pull. Let go of everything, pull both triggers all the way, squeeze both grips all the way, then circle both thumbsticks around their edges. Each step starts as soon as both hands are in position (Enter also starts it) and ends at the double beep.
- **Headset alignment**: lines up the controllers' tracking with the headset's, so in VR the controllers appear exactly where your hands are. First a button and pulse check (press X, feel the pulse, press X again; the same with A), because with the headset on you will press A and go by haptic pulses. Then the guide shows the whole procedure, since you will not see the screen with the headset on: put on the headset; press the real controllers firmly against the outside of it, left controller on the left side, right on the right, hands off the rings and the headset's sensors, and from then on do not let them slide or twist on the headset (if one does, the calibration fails and you start the step again); press A on the right controller, and you feel a 10-second countdown via controller haptic pulses, one a second, while nothing is recorded yet: use it to press the controllers back into place in case pressing A moved them; two quick pulses start the recording: slowly turn your whole body around on the spot, two full circles to your left and then two to your right, 5 to 10 seconds per circle, until a single pulse; then look up and down and tilt your head toward each shoulder, as far as is comfortable, a few times each, until one long pulse, and take the headset off. What you see in the headset during this step does not matter. The recording ends by itself; you never press anything while holding the controllers to the headset, except B to cancel. While recording, the guide watches each controller against the headset: if one turns on it by more than about 3 degrees within a second, the recording stops with three quick pulses (as it does after B or any other problem; the screen says which), and the guide offers to try again with or without this slip check (`setup --no-slip-check` turns it off from the start, for when it keeps stopping careful recordings). A hold that shifts very slowly is tolerated rather than stopped, and a controller sliding without turning is not noticed; both still make the calibration fail at the end if the controllers did not stay put. If a controller falls asleep before the countdown, the guide names it and waits; once the countdown starts, a sleeping controller cancels the step.

After setup, check both hands in SteamVR: position, buttons, haptics, recovery from occlusion and from sleep, and a daemon restart (see below). The guide has been validated on one installation with five sensors and one controller pair. Linux is the supported target; Windows is not.

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
6. Run `cv1touch calibrate inputs`. Both hands share four phases: released triggers/grips and centered sticks, full trigger presses, full grip presses, then full thumbstick circles. Follow the prompts, press firmly at the normal mechanical stop, and move the stick against its gate through several complete circles. Settings apply immediately and survive restarts; SteamVR can stay open. See [input calibration](input-calibration.md) for reset and expert controls.
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

Quit SteamVR and select the desired release before running the installer. For a tagged shallow checkout, run these commands from the repository root, replacing `v0.2.0` with the desired published tag:

```sh
git fetch origin --tags
git checkout --detach v0.2.0
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
