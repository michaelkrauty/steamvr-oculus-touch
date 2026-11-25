# Architecture

```text
CV1 headset radio / USB sensors
          │
          ▼
Modified OpenHMD engine (C, third_party/openhmd)
  radio reports, IMU, factory calibration, sensor frames, LED blobs
          │
          ▼
cv1touchd (Rust)
  LED matching, error-state Kalman filter, sensor checks
          │
          ├── control socket, tracking logs
          ▼
Shared memory (common/cv1touch_shm.h)
          │
          ▼
SteamVR driver (C++, driver/)
  poses, inputs, touch, battery, haptics
```

- **Engine.** Opens the headset's radio and the sensors, reads IMU samples and factory calibration from the controllers, captures sensor frames and finds the LED blobs in them. The changes to upstream OpenHMD are listed in [its vendor notes](../third_party/openhmd/CV1TOUCH-VENDOR.md).
- **Daemon.** `cv1touchd` matches blobs to the controllers' LEDs, fuses them with the IMU in an error-state Kalman filter (`src/track`), and publishes poses. It runs outside SteamVR so device reconnects and tracking work never stall SteamVR's server.
- **Driver.** A thin SteamVR driver that reads the shared memory and sends haptics and base-station reports back over the control socket.
- **Tool.** `cv1touch` runs setup, calibration, diagnostics and offline replay, talking to the daemon over the control socket.

## Tracking

A track starts from an OpenHMD pose. A pose rated strong is shown at once; one rated good, or one the daemon found by searching with the orientation the gyro kept after a loss, is shown only once a second sensor or a strong pose confirms it.

A controller whose samples stop for over 100 ms is held where it was, inputs included, for up to 10 s, then released. A controller that announced sleep is marked asleep. A dropout while the controller was in use is saved as `dropout-<time>-<hand>.rec`, from the last minute of radio traffic the daemon keeps in memory.

Battery levels arrive only from an awake controller; `cv1touch status --json` shows `battery_percent` as null while it sleeps.

## Coordinates and time

Several frames are involved: each controller's factory LED geometry, each sensor, the calibrated tracking world, SteamVR's tracking space, and the rendered controller model.

- Sensor calibration gives world-from-sensor transforms.
- Registration (headset alignment) gives SteamVR-from-world.
- Render fitting gives the offset of SteamVR's controller model.

SteamVR tracks Lighthouse devices relative to the base station it started from, and solves the stations' relative poses differently each session. A registration therefore measures where one station is in the tracking world, and the daemon follows the station SteamVR tracks the headset from, which it reads from SteamVR's log. To keep that station the same, the driver sets `driver_lighthouse.primarybasestation` while SteamVR runs (see [guided setup](setup.md#guided-setup)).

Published poses are the newest measurement, stamped with the time the controller was there. cv1touch does not predict: the driver reports each pose's age with the body-frame angular velocity and world-frame linear velocity, and SteamVR does its own prediction. Do not mix software delivery time, radio cadence, exposure age and display latency into one latency figure.

## Shared memory and control

- Poses are published in shared memory (`/cv1touch`, ABI version 4) using single-writer seqlocks, with a futex on Linux and an event on Windows to wake the driver. The driver rejects a mapping with another ABI version or a stale heartbeat. Its eight sensor slots are a diagnostic prefix, not a sensor limit.
- The control socket is `$XDG_RUNTIME_DIR/cv1touch/control.sock` on Linux and a named pipe on Windows. Inside Steam's pressure-vessel container, which hides `$XDG_RUNTIME_DIR`, the driver uses a second socket at `/dev/shm/cv1touch-control-<uid>/control.sock`. Starting the daemon with `--runtime-dir` or `--shm-name` turns that second socket off.
- `preview [SERIAL] [2|4]` replies `ok` and JSON with each sensor's latest image, max-pooled by 4 (or by 2 for the named sensor) so that a single-pixel LED still shows, zlib-compressed and base64-encoded, along with its brightest pixel, bright-pixel share, blobs in full-image pixels and each device's match counts. Each request holds a 3 s lease (up to about 15 fps per sensor, or 30 fps for one named sensor); `preview stop` ends it immediately. Without a preview lease a frame costs one atomic load, and sensor threads never block on a reply.
- Neither is a network interface. Any change to the ABI or the tracking log format must update both readers and writers, their compatibility tests and the changelog together.

## Sensors during play

The daemon compares each sensor with the others. When a sensor's view of the LEDs disagrees, it first confirms which controller the sensor sees, which needs each controller to turn about 30 degrees on its own in view. If the sensor moved, the daemon places it again from the other sensors and uses it once two batches of frames agree, then writes the new pose to the calibration and keeps a backup. A sensor that keeps moving (three placements in half an hour), or that no pose fits, is left out until Sensor positions is redone. `cv1touch status --json` shows each sensor's state under `pose`.

Once static infrared light sources in the sensors' views have been learned with `cv1touch landmarks learn REC --write`, the daemon watches them and warns when a sensor's view of them shifts for two one-minute windows in a row; this only reports, and changes no tracking.

Sensors are added and removed while the daemon runs. Each USB attachment gets one video-start attempt per engine; a sensor that failed to start needs to be plugged in again.

## Tracking logs

`cv1touch record start PATH [--frames]` logs radio reports, sensor observations and haptic commands; `--frames` adds raw images. A tracking log stops at 8 GiB (16 GiB with frames) and keeps 5% of the disk free, between 2 and 20 GiB. Replay one offline with `cv1touch track BASE`, and summarise it with `cv1touch rec-summary PATH`. Readers skip entry types they do not know.

## Expert controls

`cv1touch help expert` lists every command; `cv1touchd --help` lists the daemon's options:

```text
cv1touchd [--tracker cv1touch|openhmd] [--radio-wait-ms N] [--hmd-leds on|off]
          [--ignore-sensor SERIAL]... [--calibration PATH | --provisional]
          [--record PATH] [--shm-name NAME] [--runtime-dir DIR]
cv1touchd --supervise [OPTIONS]
```

- The systemd service runs `cv1touchd` directly. `--supervise` runs it as a child that is restarted after a failure, with its output in a log file; the Windows install uses this.
- `CV1TOUCH_CPUS=2,4-6` restricts the daemon to those CPUs; by default it inherits the process's CPUs. On Linux it asks rtkit for real-time priority on its transport threads and runs normally if refused.
- `settings.json` in the configuration folder holds `tilt_deg` (set by `cv1touch tilt`) and `pin_base_station`.
- `cv1touch test-inject on` allows synthetic input (`cv1touch inject`) for ten minutes, for automated tests. It reaches whatever application has focus.
- `cv1touch status --json` gives the full state: hands, every sensor with its frame and drop counters, startup and hot-plug status, and logging status.
