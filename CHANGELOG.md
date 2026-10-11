# Changelog

All notable user-facing changes are recorded here. Version numbers follow semantic versioning; changes before 1.0 may alter interfaces.

## Unreleased

### Added

- Development `cv1touch setup` guide for camera, gravity, render, input, and headset-registration stages, with a private resumable journal and dependency-aware reuse.
- Read-only `setup --check` and `setup --check --json` reports; explicit visual-verification/adoption of legacy calibration without discarding its files.
- Controller-button and haptic checks before headset registration, with a button-started haptic countdown and B on the right controller to cancel the capture.
- Headset registration ends its recording by itself once the person has turned, nodded and tilted enough, and stops it with three quick haptic pulses when a controller turns on the headset by more than about 3 degrees within a second (or on any other problem); the retry prompt, or `setup --no-slip-check`, turns the slip check off.
- `cv1touchd` status names its shared-memory segment (`shm`).

### Changed

- Guided input calibration captures both hands in four shared phases, reducing timed capture to 16 seconds while retaining freshness, identity, travel, and save/reload checks.
- An accepted fresh guided camera solve completes gravity alignment from the same motion capture; separate still-hold gravity refinement remains available.
- Guided input calibration precedes headset registration, leaving headset movement as the final physical setup step.
- The setup guide reads like a checklist: plain stage names (sensor positions, floor level, controller model, buttons and sticks, headset alignment), checks explained only when they fail, Enter/`q` (and `k` to keep unverified saved settings) instead of typed words, one result line per stage with solver output in a log, and an immediate retry after a failure. Captures start with A on the right controller as well as Enter, timed steps are marked by a controller haptic pulse and a speaker beep, and input calibration records each step as soon as both hands are in position.

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
