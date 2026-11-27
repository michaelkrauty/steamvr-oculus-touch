# Troubleshooting

Start with `cv1touch doctor`: each failing check says what is wrong and what to do. `cv1touch status` shows the sensors and controllers live (Ctrl+C exits). On Windows, **Check cv1touch** in the Start menu runs `doctor`.

The daemon's log:

- Linux: `journalctl --user _COMM=cv1touchd -b`
- Windows: `%LOCALAPPDATA%\cv1touch\state\logs\cv1touchd.log`

## Reporting a problem

`cv1touch report` (on Windows, **Send a cv1touch report**) saves one zip file with a summary of the system and the `doctor` checks, the ends of the daemon's and SteamVR's logs, cv1touch's settings and calibration, and the newest three dropout logs. Attach it to your issue with what you were doing. Review it first: calibration describes your room, and SteamVR's log can name your devices, user folder and network addresses.

## Symptoms

| Symptom | Fix |
| --- | --- |
| `cv1touchd isn't running` | Linux: `systemctl --user start cv1touchd`. Windows: **Start cv1touch**. If it stops again, the log says why. |
| `error while loading shared libraries` after a system update | Quit SteamVR, then in the source folder run `cargo clean` and `python3 scripts/install.py`. Calibration and settings are kept. |
| The controllers don't appear | Press a button on each to wake it, and check the batteries. They must be paired to the CV1 headset that is plugged in. `cv1touch status` shows whether each is awake. |
| A sensor is missing or sends no video | `cv1touch doctor` names it. Check its cable, or move it to a different USB controller (not another port on the same hub). On Windows, `cv1touch sensors` checks its driver. |
| A new sensor is not used | `cv1touch status` says what it is waiting for. Once it sends video, [add it to the calibration](setup.md#add-or-remove-sensors). |
| A sensor was bumped or moved | Keep playing with the controllers in its view; it is [placed again automatically](setup.md#sensors-that-move). Redo Sensor positions only if `cv1touch doctor` says to. |
| Rotation works but position is lost | The sensors cannot see the controller. At least two sensors should see the play area. |
| A controller freezes for a moment mid-game | Its radio dropped out. It is held where it was, buttons included, for up to 10 s. A controller that does not return needs a button press to wake. |
| `dropout-<time>-<hand>.rec` files appear | cv1touch saved the radio traffic from 20 s before to 10 s after a dropout. It keeps the newest ten. |
| The log says `tracking engine reopening` or `tracking thread stalled` | The daemon paused for a few seconds. The controllers are held for up to 10 s and carry on. |
| The log says `controller radio calibration: ... retrying in N s` | The controller's factory calibration could not be read; cv1touch retries, waiting up to a minute. If it keeps happening, send a report. |
| The log says `Could not save the calibration to ...` | Tracking works, but the next start reads the controller's calibration again. Free disk space or fix the named folder's permissions. |
| The controllers are offset or rotated from your hands | `cv1touch doctor` warns when SteamVR tracks the headset from a base station no alignment measured, and says whether to restart SteamVR or redo Headset alignment. `cv1touch universe` shows which station SteamVR uses. |
| With a Rift S, Quest or other headset without base stations, the controllers drift from your hands | Redo Headset alignment: `cv1touch setup --redo headset --quick`, or on Windows **Line up the controllers with your headset**. |
| SteamVR does not track the headset after starting | It is waiting for the base station cv1touch pinned. Switch that station on, or restart SteamVR: cv1touch removes its setting when SteamVR closes. |
| The controller drawn in VR is offset from your hand | Redo Controller model in `cv1touch setup`. |
| A game holds a tool at the wrong angle | `cv1touch tilt DEGREES` (positive: front up, at most 45) tilts the controllers as games see them. `cv1touch tilt 0` undoes it. |
| A game's controls do nothing | The game may have bindings only for other controllers. Make one for Oculus Touch (cv1touch) in **Manage Controller Bindings**. |
| The right Oculus button does nothing | [Bind it](setup.md#bind-the-right-oculus-button) for that game. If the binding page has no **Oculus Button**, run `cv1touch doctor` and check `input_profile`. |
| A binding page says "Unable to load action manifest" | The game uses OpenXR: start it, then open the binding page again. |
| `doctor` says `input_profile` is missing or out of date | SteamVR was updated. Quit SteamVR and run `cv1touch steamvr install` (Windows: **Finish setting up cv1touch**). If SteamVR's own profile is missing, have Steam verify SteamVR's files first. |
| A trigger, grip or stick does not reach its end or centre | Redo Buttons and sticks in `cv1touch setup`. See [input calibration](input-calibration.md). |
| Controllers track but haptics fail in Steam's Linux container | Update the daemon and driver to the same build and restart both. |
| The log says real-time scheduling was denied | Nothing to do: normal scheduling works. |
| The installer or `doctor` cannot find SteamVR | Start SteamVR once, or set `CV1TOUCH_STEAMVR` to its folder and `CV1TOUCH_STEAM_ROOT` to Steam's folder. |
| A calibration solver rejects a capture | Read the reason and capture the missing views or orientations again. |

## Files

| | Linux | Windows |
| --- | --- | --- |
| Settings and calibration | `~/.config/cv1touch` | `%LOCALAPPDATA%\cv1touch` |
| Tracking logs, setup state, learned IMU state | `~/.local/state/cv1touch` | `%LOCALAPPDATA%\cv1touch\state` |

Sensor calibration and alignment writes keep timestamped backups. Do not copy another user's calibration: it describes their room.
