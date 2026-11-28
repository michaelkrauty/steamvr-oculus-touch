# Setup

This guide covers Linux. For Windows, see [cv1touch on Windows](windows.md); the calibration guide is the same on both.

## Build dependencies

You need Rust 1.85 or newer, Meson 1.3 or newer and Python 3.9 or newer. If your distribution's Rust is older, install it with [rustup](https://www.rust-lang.org/tools/install). The OpenVR headers and the modified OpenHMD engine are vendored.

Debian or Ubuntu:

```sh
sudo apt install build-essential pkg-config meson ninja-build clang libclang-dev libhidapi-dev libusb-1.0-0-dev libjpeg-dev libpipewire-0.3-dev libspa-0.2-dev python3 procps
```

Arch Linux:

```sh
sudo pacman -S --needed base-devel rust meson ninja clang hidapi libusb libjpeg-turbo pipewire glib2 gstreamer gst-plugins-base python procps-ng
```

Fedora:

```sh
sudo dnf install gcc-c++ libstdc++-static cargo meson ninja-build clang hidapi-devel libusb1-devel libjpeg-turbo-devel pipewire-devel libubsan python3 procps-ng
```

CI builds on Ubuntu 24.04 and Fedora 44. Native libraries are linked dynamically, so build on the system you run it on. Cargo needs internet access to fetch the locked crates.

## Device access

Run the daemon as your desktop user, never as root. Install the udev rule once:

```sh
sudo install -m 0644 udev/70-cv1touch.rules /etc/udev/rules.d/70-cv1touch.rules
sudo udevadm control --reload-rules
```

Then reconnect the headset and sensors. The rule gives the user at the local seat access to the Oculus USB and hidraw devices through logind. A remote or headless session needs group permissions set up by an administrator instead.

Spread the sensors across USB controllers. If a sensor's video keeps stopping, `cv1touch doctor` says when it ran out of USB bandwidth.

## Install

Start SteamVR once so it writes where it is installed, quit it, then run `python3 scripts/install.py` from a clone of the `v0.7.0` tag or an extracted [v0.7.0 source archive](https://github.com/michaelkrauty/steamvr-oculus-touch/releases/tag/v0.7.0). It installs:

| Location | Contents |
| --- | --- |
| `~/.local/bin/cv1touch`, `cv1touchd` | Command-line tool and daemon |
| `~/.local/lib/cv1touch` | Engine library, driver package, install manifest |
| `$XDG_DATA_HOME/cv1touch/steamvr/cv1touch` | The driver SteamVR loads (default `~/.local/share`) |
| `$XDG_CONFIG_HOME/systemd/user/cv1touchd.service` | User service (default `~/.config`) |

It also turns on SteamVR's `activateMultipleDrivers` setting, keeping a backup of the previous settings. It refuses to overwrite files it did not install or that were changed since.

Options (`python3 scripts/install.py --help`):

- `--prefix PATH`: install the binaries and library somewhere other than `~/.local`.
- `--no-service`: do not install or restart the systemd user service.
- `--no-steamvr`: do not register the SteamVR driver.
- `--no-build`, `--target-dir PATH`: reuse an existing release build.
- `--artifacts DIR`: install a prepared bundle instead of building.
- `--uninstall`: remove what the installer installed.

If the installer cannot find SteamVR, set `CV1TOUCH_STEAMVR` to the SteamVR folder, and `CV1TOUCH_STEAM_ROOT` to the Steam folder that holds `config/steamvr.vrsettings`. If it says SteamVR's Oculus Touch input profile is missing, have Steam verify SteamVR's files and run it again.

The driver's input profile is built at install time from SteamVR's own Oculus Touch profile. SteamVR shows the controllers as **Oculus Touch (cv1touch)**, which uses each game's Oculus Touch bindings and adds the right controller's Oculus button.

## Pair controllers

Each controller must be paired to your CV1 headset; ones bought with the headset usually are. `cv1touch pair` checks and pairs them with no Oculus software. Stop the daemon first, since pairing uses the headset's radio:

```sh
cv1touch stop
cv1touch pair --check
cv1touch pair
systemctl --user start cv1touchd
```

`--check` shows what is paired and changes nothing (add `--json` for machine-readable output); wake each controller first (press any button). `cv1touch pair` restarts the headset in pairing mode and lists each controller that asks to be paired; to make one ask, hold Menu (☰) and Y on the left controller, or the Oculus button and B on the right one, until its light blinks. Pairing writes the headset's radio address into the controller without unpairing any previous controller, so if you own spare controllers paired to the same headset, keep the unused pair asleep or remove its batteries so two controllers for the same hand do not transmit at once. `--listen` lists the controllers that ask without pairing any, and `--normal` returns a headset left in pairing mode back to normal mode. The headset returns to normal mode at the end, or on Ctrl+C, and every run saves what was paired before and after in `$XDG_STATE_HOME/cv1touch/pairing/` (on Windows, `%LOCALAPPDATA%\cv1touch\pairing\`).

## Guided setup

Wake both controllers and make sure the daemon is running, then:

```sh
cv1touch setup
```

Press Enter to start each stage, or `q` to quit. With your hands full, A on the right controller works as Enter. Timed steps start with one haptic pulse and beep and end with two. Setup remembers finished stages in `~/.local/state/cv1touch/setup.json`, so running it again continues where you left off. It never pairs controllers, changes firmware or permissions, or restarts services.

1. **Sensor positions** (60 s): slowly wave both controllers around the whole play area, high and low, near and far from each sensor, turning your wrists so the rings face every way. Views shared by two sensors matter most. A good result also levels the floor.
2. **Floor level** (only if the previous stage did not level it): 12 times, turn both controllers into a new position with the rings in view, hold still, press A, and keep still until the double beep (3 s).
3. **Controller model** (5 s): keep both controllers awake. This lines up the controller drawn in VR with the real one.
4. **Buttons and sticks**: release everything, pull both triggers fully, squeeze both grips fully, then circle both sticks against their edges. See [input calibration](input-calibration.md).
5. **Headset alignment**: needs SteamVR running with your headset tracking. The guide checks the buttons and haptics first, then explains the whole procedure, since you will not see the screen:
   1. Put on the headset and press the controllers firmly against its sides, left on the left, right on the right, without covering its sensors or cameras. Keep them from sliding or twisting.
   2. Press A. A 10-second countdown of pulses follows; use it to settle the controllers back in place.
   3. After two quick pulses, slowly turn on the spot: two full circles to the left, then two to the right, 5 to 10 s per circle.
   4. After one pulse, look up and down and tilt your head toward each shoulder a few times.
   5. One long pulse ends logging. B cancels at any time.

   If a controller turns on the headset by more than about 3 degrees, logging stops with three pulses and the guide offers to try again. `cv1touch setup --no-slip-check` turns that check off.

Other forms:

- `cv1touch setup --check` lists each stage's state without starting anything (`--json` for scripts).
- `cv1touch setup --events` runs the same guide for a graphical front end: one JSON object per line on standard output and line commands such as `run sensors` or `quit` on standard input.
- `cv1touch setup --redo STAGE` redoes `sensors`, `floor`, `model`, `buttons` or `headset`. Once setup is complete, plain `cv1touch setup` also offers each stage by number. Redoing Sensor positions also requires Headset alignment again.
- `cv1touch setup --redo headset --quick` is a 15-second alignment for a new headset or when the headset's tracking has moved: controllers anywhere on the headset, then look down and up, tilt toward each shoulder and look over each shoulder.

A stage saved before you used the guide shows as "saved earlier, not verified". Press `k` to keep it only if tracking already looks right; otherwise redo it.

Headset alignment measures the base station SteamVR tracks the headset from. When SteamVR sees more than one station, the driver sets SteamVR's `driver_lighthouse.primarybasestation` to that station from the next SteamVR start, and removes the setting when SteamVR closes. It never changes a value you set. To turn this off, run `cv1touch pin-base-station off` (or add `"pin_base_station": false` to `~/.config/cv1touch/settings.json`); `cv1touch pin-base-station on` turns it back on.

### Headsets without base stations

Any headset that works in SteamVR can be used, including Quest over Steam Link, Virtual Desktop or ALVR, Rift S, PSVR2 and Windows Mixed Reality. Keep the headset's own controllers asleep or out of the way while playing with CV1 Touch, and avoid covering the headset's tracking sensors while holding Touch controllers against it during alignment.

Because there are no fixed base stations, cv1touch follows the headset's live SteamVR universe ID so the controllers stay visible if the headset starts a new tracking space (for example after a boundary reset or a new wireless session). If the tracking space changed, `cv1touch doctor`, `cv1touch status` and `cv1touch-gui` warn you so you can run the 15-second quick alignment if your hands appear offset.

On Quest, use SteamVR's seated or standing recenter rather than the headset's own recenter button when possible; in Virtual Desktop, enabling **Stage Tracking** keeps the play space anchored across sessions.

After setup, check both hands in SteamVR. `cv1touch input-test` checks the buttons, triggers and sticks while a VR app has focus.

## Bind the right Oculus button

Nothing is bound to the right controller's Oculus button until you bind it, game by game:

1. If the game uses OpenXR, start it first; otherwise its binding page says "Unable to load action manifest".
2. In SteamVR, open **Settings**, **Controllers**, **Manage Controller Bindings**.
3. Choose the game and edit its current binding.
4. On the right hand, choose **Oculus Button** and pick an action. Save.

The left Menu button stays SteamVR's dashboard button.

## Sensors that move

A sensor that is bumped or moved is placed again automatically while at least two other sensors stay put. Keep playing with the controllers in its view; it takes about a minute of movement. The new pose is saved to the calibration with a backup of the old file. Redo Sensor positions only when `cv1touch doctor` says to.

## Add or remove sensors

Plug sensors in or out while the daemon runs; it notices within about two seconds. `cv1touch status` and `cv1touch doctor` say why a new sensor is waiting and what to do. There is no fixed sensor limit, only USB bandwidth and CPU. To keep a sensor closed, pass `--ignore-sensor SERIAL` to `cv1touchd` (repeatable).

To add a sensor to a calibrated room, log both controllers moving where it and the calibrated sensors can see them, then place it:

```sh
dir="${XDG_STATE_HOME:-$HOME/.local/state}/cv1touch"
cv1touch record start "$dir/add-sensor.rec"
# Move both controllers through varied orientations, then:
cv1touch record stop
cv1touch calibrate place SERIAL "$dir/add-sensor.rec"
```

`SERIAL` is the sensor's serial from `cv1touch status`. If the fit looks good, run the same command with `--write` and restart the daemon. For a new layout, redo Sensor positions instead. An unplugged sensor keeps its calibration for when it returns.

## Manual calibration

`cv1touch help expert` lists every command. The stages of `cv1touch setup` map to these:

| Stage | Command |
| --- | --- |
| Sensor positions | `cv1touch calibrate` (60 s capture and solve) |
| Floor level | `cv1touch calibrate gravity LOG --write` on a tracking log of at least 12 still holds of 2 s or more |
| Controller model | `cv1touch fit-render [REC]` |
| Buttons and sticks | `cv1touch calibrate inputs` |
| Headset alignment | `cv1touch register --headset` |

Solvers reject captures that are too short or inconsistent and keep the previous calibration. `--force` skips those checks; it is for diagnosis only. Restart the daemon after writing a calibration from the command line: `systemctl --user restart cv1touchd`.

## Upgrade

Quit SteamVR, select the release, and run the installer again:

```sh
git fetch origin --tags
git checkout --detach v0.7.0
python3 scripts/install.py
```

For a source archive, extract the new release into a new folder and run the installer there. Calibration and settings are kept. The installer restarts the service so the new daemon runs (with `--no-service` it tells you how). Always update the daemon and the driver together.

After a SteamVR update, run `cv1touch doctor`. If its `input_profile` check says the profile is out of date, quit SteamVR and run `cv1touch steamvr install`.

To go back to a release from before the right Oculus button could be bound, first quit SteamVR and run `cv1touch steamvr uninstall` with the newer one.

## Remove

Quit SteamVR, then run with the same prefix and environment as the install:

```sh
python3 scripts/install.py --uninstall
```

This unregisters the driver, removes the service and the files the installer owns, and removes the driver's base-station setting from SteamVR. It keeps calibration, tracking logs, changed files and `activateMultipleDrivers`. Remove the udev rule by hand if nothing else needs it.

## Sensor viewer

`tools/sensor-view` shows live sensor views; `--annotated` labels the blobs. It needs GStreamer: `gstreamer1.0-tools gstreamer1.0-pipewire gstreamer1.0-plugins-base gstreamer1.0-x` on Debian or Ubuntu, or `gst-plugin-pipewire gst-plugins-base gst-plugins-good` on Arch.
