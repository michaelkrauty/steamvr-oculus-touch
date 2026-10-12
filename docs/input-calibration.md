# Input calibration

Run this with the daemon running and both Touch controllers awake:

```sh
cv1touch calibrate inputs
```

SteamVR can stay open; no VR application or daemon restart is required for calibration. Follow four timed prompts with both hands simultaneously: leave both triggers and grips released with both thumbsticks centered for 2 seconds, hold both triggers fully pulled for 2 seconds, hold both grips fully squeezed for 2 seconds, then move both thumbsticks against their gates through several complete circles for 10 seconds. Timed collection totals 16 seconds, plus your time confirming each prompt. Use normal firm pressure at the mechanical stop. Keep the other inputs released during each press capture.

The guide measures the factory-normalized values before any previous user correction. Each hand independently requires fresh samples, a consistent physical-controller identity, and enough travel in every required direction. A fresh hand cannot compensate for stale or disconnected input from the other hand. Incomplete captures do not replace your current settings. After a successful save, the daemon applies the new settings immediately.

## What changes

The file is `$XDG_CONFIG_HOME/cv1touch/input-calibration.json`, or `~/.config/cv1touch/input-calibration.json` by default. It contains trigger/grip released and pressed endpoints and each stick axis's minimum, center, and maximum. Each hand is bound to its controller's factory-calibration identity; replacement controllers use factory defaults until you calibrate them. The file is personal device configuration and is not included in the repository.

Measured trigger/grip travel maps to 0–1. The default deadzone is 5% at each endpoint, so small resting noise stays released and ordinary full pressure reaches 1. Stick axes are centered and scaled separately, then use a 5% radial center deadzone; a round gate maps to a unit circle while preserving direction after axis correction. Final output is finite and bounded to SteamVR's normalized ranges. Disconnected controllers release their inputs.

This corrects endpoint drift or uneven travel. It does not change tracking, camera poses, SteamVR registration, button/capacitive mappings, or haptics. Recalibrate if a controller's resting or fully pressed values change.

## Verify and reset

With a focused VR app running, use:

```sh
cv1touch input-test
```

The tool checks the specific supported inputs delivered to SteamVR and asks whether each haptic pulse was felt. Menu/thumb-rest delivery needs a manual check in an app because those paths are not exposed by Valve's legacy controller-state binding. Check that both controllers' rendered triggers/grips reach their stops and their sticks return to center.

To remove this input correction and reload factory defaults:

```sh
cv1touch calibrate inputs --reset
```

Reset affects only input calibration. Other calibration and alignment files remain in place.

## Expert controls

To capture a file for inspection without applying it:

```sh
cv1touch calibrate inputs --out /tmp/cv1touch-input-calibration.json
```

The versioned JSON schema uses `format`, `version`, `analog_deadzone`, `stick_deadzone`, and two `hands` entries in left/right order. Each hand has an `identity`, `trigger` and `grip` endpoints (`released`, `pressed`), and two `stick` axis entries (`min`, `center`, `max`). Coordinates are in the factory-normalized space, so a released endpoint can be negative. All coordinates must be finite within [-4, 4]; analog span and each stick center-to-edge span must be at least 0.5. Deadzones may be set from 0 through 0.2; zero disables that deadzone. Unknown fields and unsupported format versions are rejected.

Inspect factory-normalized live values or reload a manually edited valid file:

```sh
cv1touch inputs --json
cv1touch inputs reload
```

Reload validates the entire file before replacing the active configuration; a malformed file leaves the previous configuration active. The wizard saves with mode 0600 and replaces the file atomically. A failed live reload restores the previous file. Restarting the daemon reloads valid settings; invalid startup settings are reported and factory defaults are used.
