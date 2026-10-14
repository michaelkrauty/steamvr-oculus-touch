# Changelog

All notable user-facing changes are recorded here. Version numbers follow semantic versioning; changes before 1.0 may alter interfaces.

## Unreleased

### Added

- `cv1touchd --supervise` runs the daemon as a child process with each line of its output stamped with the time in `logs/cv1touchd.log` under the state directory, keeping four earlier logs. After a failure it starts the daemon again 2 s later, as the systemd unit does, unless the daemon refused its arguments or failed five times in a row within 30 s of starting. It is meant for Windows, where no service manager does this.
- `cv1touch sensors` says whether each Rift sensor plugged in can be used: on Linux, that the udev rules let it be opened. On Windows, where the engine reads the sensors through WinUSB and Windows gives them no driver of its own, `cv1touch sensors install` gives each sensor's two interfaces Windows' own WinUSB driver (the Microsoft-signed "WinUsb Device", as choosing it in Device Manager does; no other driver or certificate is installed), asking Windows for permission once, and `cv1touch sensors remove` takes off what it gave, leaving a WinUSB someone else set up alone. Both say so, and exit with code 3, when a program was using a sensor and Windows must be restarted to finish. `cv1touch doctor` checks it on Windows.
- `cv1touch report` saves one zip file with what someone helping you needs to see what went wrong: a summary (version, system, the `doctor` checks, the Oculus USB devices and the drivers they run with, the daemon's status and setup's state), the ends of the daemon's and SteamVR's logs, and cv1touch's settings and calibration. On Windows it goes on the desktop and Explorer shows it; elsewhere it goes in the current directory, or wherever `--out` says. It says what it holds: SteamVR's log also names your other VR devices, your user folder and your network addresses.
- `cv1touch stop` stops the daemon cleanly, as SIGTERM or Ctrl+C does, and with it a `cv1touchd --supervise` running it. The supervisor itself stops the same way: Ctrl+C or SIGTERM asks the daemon to stop, and ends it if it has not stopped 10 s later. Closing the supervisor's console window leaves the daemon 5 s, as Windows then ends the supervisor, and logging off ends both at once; `cv1touch stop` is the clean way.
- `cv1touch prediction [PERCENT]` sets how much of the tracked motion SteamVR extrapolates poses with: 100 (the default) as before, 0 none, so each pose is shown where it was measured. `cv1touch prediction position PERCENT` and `cv1touch prediction rotation PERCENT` set positions and orientations separately: positions as tracked keep the controllers with the body on a sidestep, while orientations shown where they were measured keep them from tilting during fast turns. Settings apply at once and are kept for later runs in `settings.json` in the configuration directory (`rotation_prediction_percent` for orientations; a file without it means the same for both). The driver applies them, whichever poses are published (also OpenHMD's): daemon and driver must be updated together (shared-memory ABI 4).

### Changed

- Controllers are tracked again sooner after waking up or losing their position. The tracker also starts from OpenHMD poses rated good, not only strong, and after a loss it searches for the controller's position itself with the orientation the gyro kept. A track started either way is shown only once a second sensor, or a strong OpenHMD pose that agrees with it, confirms it. Replaying recordings from 2026-10-08, wake-ups that had taken 6.6 s and 2.9 s took 0.24 s and 1.5 s and the other three 43-63% less time, with no new wrong locks. `cv1touch status --json` counts these acquisitions (`good`, `searched`, `unconfirmed`).
- `cv1touchd` no longer runs OpenHMD's brute-force pose search for a controller its own tracker fitted within the last 100 ms, or one asleep, except on a sensor that has no pose yet. That search took most of the daemon's CPU time during play. The per-sensor OpenHMD fix counts in `cv1touch status` are lower for tracked controllers as a result.

### Fixed

- A controller that has just woken up, or whose wrong lock was just dropped, no longer appears at OpenHMD's own position estimate (which could be metres off) until it is tracked again: SteamVR shows it where it was last tracked, turning but not moving. The daemon log now says how long each controller went without a position when it gets one back, and logs that after a wake too.

## [0.2.0](https://github.com/michaelkrauty/steamvr-oculus-touch/releases/tag/v0.2.0) - 2026-10-11

### Added

- `cv1touch setup` guide for camera, gravity, render, input, and headset-registration stages, with a private resumable journal and dependency-aware reuse.
- Read-only `setup --check` and `setup --check --json` reports; explicit visual-verification/adoption of legacy calibration without discarding its files.
- Controller-button and haptic checks before headset registration, with a button-started haptic countdown and B on the right controller to cancel the capture.
- Headset registration ends its recording by itself once the person has turned, nodded and tilted enough, and stops it with three quick haptic pulses when a controller turns on the headset by more than about 3 degrees within a second (or on any other problem); the retry prompt, or `setup --no-slip-check`, turns the slip check off.
- `cv1touchd` status names its shared-memory segment (`shm`).

### Changed

- Guided input calibration captures both hands in four shared phases, reducing timed capture to 16 seconds while retaining freshness, identity, travel, and save/reload checks.
- An accepted fresh guided camera solve completes gravity alignment from the same motion capture; separate still-hold gravity refinement remains available.
- Guided input calibration precedes headset registration, leaving headset movement as the final physical setup step.
- The setup guide reads like a checklist: plain stage names (sensor positions, floor level, controller model, buttons and sticks, headset alignment), checks explained only when they fail, Enter/`q` (and `k` to keep unverified saved settings) instead of typed words, one result line per stage with solver output in a log, and an immediate retry after a failure. Captures start with A on the right controller as well as Enter, timed steps are marked by a controller haptic pulse and a speaker beep, and input calibration records each step as soon as both hands are in position. Instructions say what each step is for and exactly how to do it, and failures say in plain words what happened and that the previous calibration is unchanged.

### Fixed

- Failed camera captures recover provisional tracking when the previous calibration is malformed; rejected reloads attempt recovery and report failure.
- Cancelled render, registration, gravity, placement, and refinement fits do not install their computed settings.
- Guided registration bootstraps a fresh fit after camera-world changes instead of depending on an obsolete saved transform.
- Guided camera/gravity handoffs wait for the requested engine configuration, advancing sensor streams, and fresh data from the same controllers before continuing.
- Pending setup stages retain missing or invalid artifact diagnostics instead of offering adoption of unavailable settings.
- Guided headset registration explains preparation before recording and waits for a deliberate button press and haptic countdown, so users can get into position without watching the terminal.
- Guided capture validates recording acknowledgements and scopes cleanup to its own recording when a start reply is lost.
- Guided calibration prompts identify buttons by controller, state when input sampling begins, and separate the control checks from headset instructions.
- A controller falling asleep before the registration countdown (during the button checks or while reading the headset instructions) no longer aborts setup: the guide names the controller to wake, waits up to five minutes, and then needs a fresh release and press. A dropout during the countdown or recording still cancels without installing a fit.

## [0.1.0](https://github.com/michaelkrauty/steamvr-oculus-touch/releases/tag/v0.1.0) - 2026-10-10

### Added

- Complete Linux CV1 Touch stack: Rust daemon/tracker, camera calibration, SteamVR-space registration, native SteamVR driver, haptics, recording and replay.
- User-local installer, service setup, USB/hidraw access rules, documented setup and expert paths.
- Pinned OpenVR SDK, attributed modified OpenHMD source, build and test automation.
- Dynamic sensor discovery, calibration, tracking, and complete diagnostics without a fixed camera-count cap.
- Guided per-controller input calibration for trigger/grip endpoints and thumbstick centers/full travel, with live reload and factory reset.

### Changed

- Source documentation and recording diagnostics describe current behavior without unpublished development notes or phase labels.
- Original-code copyright notices and software citation identify the maintainer by GitHub username.
- Upstream Linux CI runs maintainer changes automatically and requires explicit workflow approval for external fork revisions on the dedicated runner; manual branch checks and independent GitHub-hosted checks in forks are available.
- Project and repository renamed to SteamVR Oculus Touch (`steamvr-oculus-touch`); command names and configuration paths remain compatible.

### Fixed

- Registered SteamVR driver upgrades and removal now preserve modified or unmanaged files; verified ownership metadata and atomic package replacement protect installation changes.
- Installation and runtime dependencies on a developer's checkout and CPU topology.
- First-time headset registration without an existing alignment file.
- Concurrent alignment writes, stale controller inputs, and SIGPIPE on driver control sends.
- USB camera shutdown deadlock that could stall daemon restarts.
- Multi-camera teardown blocked by a peer's USB retry loop; fully parked streams now recover through bounded engine-side retries.
- Camera allocation/startup failure cleanup and frame-buffer lifetime during shutdown.
- Fixed-size cross-camera state, high-index camera aliasing, and camera queue starvation under load.
- Out-of-range analog inputs after factory calibration drift; SteamVR output now stays within its normalized ranges.
- Input diagnostics accepting unrelated axes or missing updates during driver delivery lag.
- Linux OpenVR controller-state packing that could prevent input diagnostics from reading SteamVR buttons and axes correctly.
