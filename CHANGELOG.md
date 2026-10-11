# Changelog

All notable user-facing changes are recorded here. Version numbers follow semantic versioning; changes before 1.0 may alter interfaces.

## Unreleased

### Added

- Complete Linux CV1 Touch stack: Rust daemon/tracker, camera calibration, SteamVR-space registration, native SteamVR driver, haptics, recording and replay.
- User-local installer, service setup, USB/hidraw access rules, documented setup and expert paths.
- Pinned OpenVR SDK, attributed modified OpenHMD source, build and test automation.
- Dynamic sensor discovery, calibration, tracking, and complete diagnostics without a fixed camera-count cap.
- Guided per-controller input calibration for trigger/grip endpoints and thumbstick centers/full travel, with live reload and factory reset.

### Changed

- Linux CI uses a dedicated runner for trusted private-repository changes and GitHub-hosted runners for public repositories and fork pull requests.
- Project and repository renamed to SteamVR Oculus Touch (`steamvr-oculus-touch`); command names and configuration paths remain compatible.

### Fixed

- Installation and runtime dependencies on a developer's checkout and CPU topology.
- First-time headset registration without an existing alignment file.
- Concurrent alignment writes, stale controller inputs, and SIGPIPE on driver control sends.
- USB camera shutdown deadlock that could stall daemon restarts.
- Camera allocation/startup failure cleanup and frame-buffer lifetime during shutdown.
- Fixed-size cross-camera state, high-index camera aliasing, and camera queue starvation under load.
- Out-of-range analog inputs after factory calibration drift; SteamVR output now stays within its normalized ranges.
- Input diagnostics accepting unrelated axes or missing updates during driver delivery lag.
- Linux OpenVR controller-state packing that could prevent input diagnostics from reading SteamVR buttons and axes correctly.
