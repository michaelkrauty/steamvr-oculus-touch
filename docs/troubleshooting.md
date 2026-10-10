# Troubleshooting

Start with `cv1touch doctor`, `cv1touch status --json`, and `journalctl --user -u cv1touchd -b`. Include version, distribution, sensor count, build/test results, and the relevant error when reporting a problem. Remove serial numbers, room coordinates, personal paths, and unrelated logs before sharing.

| Symptom | Checks and recovery |
| --- | --- |
| Daemon not reachable | Check `systemctl --user status cv1touchd`; for manual runs check the runtime directory and socket permissions. Only one daemon may run per user. |
| No controllers | Wake them with a button, check batteries, and confirm they are paired to the connected CV1 headset. Headset USB and hidraw access are both required. |
| No sensor frames | Check device ACLs, cables, USB bandwidth, and kernel transfer errors. Restart the daemon after fixing USB transport; do not calibrate with a missing sensor. |
| Rotation works, position is lost | Check sensor views, optical occlusion, calibration, and overlapping coverage. Rotation-only output is not a valid position fix. |
| Controllers translated/rotated from the headset space | Repeat headset registration after checking gravity levelling and headset tracking. `cv1touch universe` reports base-station disagreement. |
| Rendered controller is offset despite good tracking | Run `fit-render` with a recording containing both factory LED models, then reload alignment. Check that native SteamVR contains the CV1 models. |
| Buttons visible locally but unavailable to an app | SteamVR focus and action bindings matter. Run a focused scene app/game and check its Oculus Touch bindings; an inactive action is not proof that a physical button was lost. |
| Haptics unavailable | Check the controller is awake/connected, the driver is loaded, and the game's haptic binding is active. |
| Real-time scheduling denied | The daemon falls back to normal scheduling. Use the reported reason to diagnose rtkit/session policy; normal scheduling remains supported. No permissive polkit rule is required by this project. |
| Custom Steam library not found | Set `CV1TOUCH_STEAMVR` to the actual runtime directory and `CV1TOUCH_STEAM_ROOT` for its Steam config root. Flatpak/sandboxed Steam is not validated. |

Configuration is under `$XDG_CONFIG_HOME/cv1touch` (default `~/.config/cv1touch`); recordings and learned IMU state are under `$XDG_STATE_HOME/cv1touch` (default `~/.local/state/cv1touch`). Do not copy another user's camera calibration: it encodes their room. Successful writes preserve timestamped backups. Raw recordings can contain camera imagery if `--frames` was used; inspect them before sharing.

If a calibration solver fails, read its rejection reason and capture the missing views or orientations. `--force` bypasses acceptance checks and can install unsuitable results; it is an expert diagnostic option, not part of normal setup.
