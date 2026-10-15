# Architecture

```text
CV1 headset radio / USB sensors
          │
          ▼
Modified OpenHMD acquisition engine (C/C++)
  radio reports, IMU, factory calibration, images, blobs
          │
          ▼
cv1touchd (Rust)
  optical association + error-state Kalman filter
  bias estimation, recovery, prediction shaping
          │
          ├── local control socket / recording and replay
          ▼
Versioned shared memory (seqlocks + futex notification)
          │
          ▼
Native SteamVR driver (C++)
  poses, inputs, capacitive touch, battery, haptics
```

OpenHMD owns device discovery and USB/camera acquisition. The Rust tracker fuses gyroscope and accelerometer data with multi-camera LED observations; OpenHMD poses provide acquisition priors/fallback. A pose it rates strong starts a published track at once; one it rates only good starts a provisional track, shown only once a second sensor's frame (or a strong pose) confirms it. After a loss, while the gyro still holds the controller's orientation, the tracker also searches for its position by itself (`track::search`), with a bounded cost per frame, and treats what it finds as provisional too. The headset is used as a radio receiver and does not become a SteamVR display device. The driver is deliberately thin and shares the C ABI in `common/cv1touch_shm.h` with the daemon.

## Coordinates and timestamps

Factory LED geometry, per-camera optical coordinates, the calibrated tracking world, the SteamVR tracking universe, and the rendered controller model are different frames. Camera calibration supplies world-from-camera transforms; registration supplies SteamVR-from-world; render fitting supplies the controller-model offset. Orientation serialization uses XYZW quaternions. See the ABI headers and `src/calib` for units and transform direction.

Host/device clock mapping and exposure-time compensation place measurements on a monotonic timeline. Published poses carry their sample time and velocities. The driver reports pose age to SteamVR, which performs its own prediction with the share of those velocities `cv1touch prediction` sets (positions and orientations separately). SteamVR integrates the angular velocity a driver reports in the frame of the reported orientation, not in driver world space as its driver documentation says, and takes the linear velocity in driver world space, rotated by the world-from-driver alignment; the driver therefore hands over the body rate (`ang_vel - gyro_bias`) and the world velocity. This was measured in 2026-10 with `examples/steamvr_prediction_probe.rs`, which publishes a synthetic controller and reads back what SteamVR predicts; run it again when SteamVR changes. Published stamps are checked against a Lighthouse-tracked controller fixed to the same jig by `cv1touch eval-capture` and `cv1touch eval`, which reports the lag between the two (negative: the Touch ahead) and the `cv1touch latency` adjustment that puts the Touch on the reference's timeline; the driver submits a pose stamped after the submission itself as current, so stamping later than the samples' arrival has no further effect; the daemon keeps that adjustment in `settings.json` with the prediction percents. Software delivery time, hardware radio cadence, optical exposure age, and display latency must not be combined into one misleading latency number.

## Reliability boundaries

Shared-memory slots use single-writer seqlocks. Alignment writers are serialized; hand ownership transfers between radio and fallback publishing. Driver readers reject incompatible mappings and stale heartbeats. Stale or disconnected controller samples release application inputs. Haptic/control sends use bounded local IPC and cannot deliver SIGPIPE to SteamVR. A separate daemon keeps device reconnects and tracking computation out of the SteamVR server.

Optional [input calibration](input-calibration.md) maps measured trigger/grip endpoints and stick travel before publication. The wizard samples factory-normalized inputs before user correction, so repeated calibration does not compound corrections. A stable factory-calibration identity binds each hand's settings to its controller; mismatches use finite, bounded factory values. Live reload validates the complete configuration before replacing it under the publication lock. Disconnect release bypasses the transform, since a raw zero can represent a partial press after endpoint correction.

The daemon attempts bounded real-time priority through rtkit for selected transport threads and falls back with a diagnostic if policy denies it. By default it inherits the process's allowed CPUs. `CV1TOUCH_CPUS` is an optional explicit CPU-list override; invalid or unavailable CPUs are rejected rather than applying a development-machine topology. Avoid changing scheduling policy without measuring it under your real workload.

## Expert use

Build with `cargo build --locked --release`; install without services using `python3 scripts/install.py --no-service`. Run `cv1touchd` directly as your desktop user. The service is just a wrapper around this daemon. Use `--help` for diagnostic commands, calibration alternatives, explicit recording files, offline `track` replay, and predictor controls.

Input injection is disabled by default. `test-inject on` permits bounded injection for ten minutes; `test-inject off` disarms it immediately. It is for integration tests and can affect applications currently receiving controller input. The socket and shared memory are local to the user's runtime environment; they are not network APIs.

The acquisition engine discovers sensors dynamically and supports one controller pair. Sensor identities and tracking state are dynamically sized; there is no fixed camera-count cap. Two pending frames per admitted camera and a bounded clock queue prevent a stalled consumer from accumulating unlimited work, with round-robin camera service and visible drop counters. Other hardware drivers remain in the attributed upstream source tree but are not built or offered as supported cv1touch hardware. Future changes to the ABI or recording schema must update compatibility checks, readers, tests, and the changelog together.

## Sensor diagnostics and compatibility

`cv1touch status --json` returns every admitted sensor in `sensors`, with `sensor_total` and camera queue/drop counters. The live status board uses this complete list. A newly attached sensor needs a daemon restart; a sensor without a calibrated pose can record observations but does not contribute to the Rust pose estimate until calibrated. Failed USB opens/starts are logged, and stalled streams are visible as non-advancing frame counters.

The controller/alignment shared-memory ABI is version 4: the published velocities are the tracked ones, and the alignment block carries the prediction percents the driver applies. Its eight sensor slots are a legacy diagnostic prefix, not a tracking limit. `sensor_shm_count` and `sensor_shm_truncated` describe that prefix explicitly. Use control JSON for the full list; the status board warns when it must fall back to shared memory. Recording records already identify sensors individually by serial, so the recording schema is unchanged.
