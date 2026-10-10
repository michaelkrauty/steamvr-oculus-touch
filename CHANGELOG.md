# Changelog

All notable user-facing changes are recorded here. Version numbers follow semantic versioning; changes before 1.0 may alter interfaces.

## Unreleased

### Added

- Complete Linux CV1 Touch stack: Rust daemon/tracker, camera calibration, SteamVR-space registration, native SteamVR driver, haptics, recording and replay.
- User-local installer, service setup, USB/hidraw access rules, documented setup and expert paths.
- Pinned OpenVR SDK, attributed modified OpenHMD source, build and test automation.

### Fixed

- Installation and runtime dependencies on a developer's checkout and CPU topology.
- First-time headset registration without an existing alignment file.
- Concurrent alignment writes, stale controller inputs, and SIGPIPE on driver control sends.
