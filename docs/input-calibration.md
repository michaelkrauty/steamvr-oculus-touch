# Input calibration

Input calibration makes a full trigger pull or grip squeeze read as 1 and a stick reach its edge. It is the Buttons and sticks stage of `cv1touch setup`, and can be run on its own with the daemon running and both controllers awake:

```sh
cv1touch calibrate inputs
```

SteamVR can stay open. Follow four timed prompts, with both hands at once:

1. Release both triggers and grips and leave both sticks centred (2 s).
2. Pull both triggers fully (2 s).
3. Squeeze both grips fully (2 s).
4. Circle both sticks against their edges (10 s).

Settings apply at once and survive restarts. An incomplete capture keeps the previous settings.

## How it works

The settings are in `~/.config/cv1touch/input-calibration.json` (`$XDG_CONFIG_HOME/cv1touch`; `%LOCALAPPDATA%\cv1touch` on Windows). They hold each trigger's and grip's released and pressed values and each stick axis's minimum, centre and maximum, measured from the factory-calibrated values. Each hand is tied to its controller, so a different controller uses factory values until it is calibrated.

Triggers and grips are mapped to 0 to 1 with a 5% deadzone at each end. Sticks are centred and scaled per axis, with a 5% deadzone at the centre. Tracking, bindings and haptics are not affected.

## Check and reset

With a VR app focused, run `cv1touch input-test`. It checks each input SteamVR receives and asks whether you felt each haptic pulse. Check the menu button, thumb rests and right Oculus button in a game instead: the test cannot see them, and the Oculus button does nothing until you [bind it](setup.md#bind-the-right-oculus-button).

To return to factory values:

```sh
cv1touch calibrate inputs --reset
```

## Expert controls

- `cv1touch calibrate inputs --out PATH` writes a calibration to a file without applying it.
- `cv1touch inputs --json` shows the live factory-calibrated values.
- `cv1touch inputs reload` loads an edited file. An invalid file is rejected and the current settings stay.

The file has `format`, `version`, `analog_deadzone`, `stick_deadzone` and two `hands` (left, then right). Each hand has an `identity`, `trigger` and `grip` (`released`, `pressed`) and two `stick` axes (`min`, `center`, `max`). Values must be finite within [-4, 4], with at least 0.5 of travel; deadzones range from 0 (off) to 0.2. Unknown fields are rejected.
