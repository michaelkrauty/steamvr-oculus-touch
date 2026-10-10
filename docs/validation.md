# Validation

## What has been exercised

The original development setup used a Linux x86-64 desktop, one CV1 Touch pair, and four CV1 sensors alongside a Lighthouse-tracked SteamVR headset. The controllers were used in VR games; the owner confirmed positional/orientational alignment and working haptics. Those observations establish feasibility for that configuration, not a compatibility guarantee for every game or distribution.

The release work adds clean-build and relocated-install checks, regression tests for runtime failure paths, and a documented first-time setup. CI checks the deterministic software paths without USB devices or a running SteamVR server. Hardware behavior cannot be inferred from CI alone.

## Measurement interpretation

Development captures compared tracked controller poses to a rigidly attached SteamVR reference and checked cross-camera/Touch-to-Touch consistency. A reference's own errors, non-rigid attachment, changed room setup, clock alignment, and fit/test overlap can dominate the residual. Tracking is not established as more accurate than another controller merely because a fitted residual is small.

Radio-arrival-to-publication measurements characterize added software delay. They exclude the radio's sampling/delivery interval, camera exposure, SteamVR prediction, application rendering, and display scan-out. No universal sub-millisecond end-to-end claim is made here. Timing knobs and prediction shaping are expert controls that need out-of-sample checks.

Raw development recordings and personal calibration are not included. Consequently, historical numerical measurements are not presented as reproducible public benchmarks. A publishable benchmark should specify hardware, software commit, sensor layout, capture/clock procedure, reference quality, held-out data, and uncertainty, and distribute appropriately sanitized data with permission.

## Before relying on a new setup

Exercise every button and axis, capacitive input, both hands, and haptics in a focused SteamVR app. Test sleep/wake, disconnect/reconnect, daemon restart, optical occlusion and relock, poor coverage, and a SteamVR restart. Check long sessions for stopped camera streams or recurring USB failures. Re-check registration after room setup changes. Report failures with the exact version and conditions.
