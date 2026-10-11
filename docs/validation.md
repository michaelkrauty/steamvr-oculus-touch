# Validation

## What has been exercised

The original development setup used a Linux x86-64 desktop, one CV1 Touch pair, and four CV1 sensors alongside a Lighthouse-tracked SteamVR headset. The controllers were used in VR games; the owner confirmed positional/orientational alignment and working haptics. Those observations establish feasibility for that configuration, not a compatibility guarantee for every game or distribution.

The release work adds clean-build and relocated-install checks, regression tests for runtime failure paths, and a documented first-time setup. CI checks the deterministic software paths without USB devices or a running SteamVR server. Hardware behavior cannot be inferred from CI alone.

Input calibration regressions cover endpoint and stick normalization, controller identity changes, stale or incomplete captures, disconnect release, malformed configuration, and save/reload rollback. The input test checks the corresponding SteamVR axis or button instead of accepting unrelated movement. Validate measured endpoints and in-app delivery on your own controllers after calibration.

Dynamic-sensor regressions exercise acquisition/configuration through 65 synthetic sensors, high-index camera tracking and distinct-camera confirmation, complete diagnostics beyond the shared-memory prefix, bounded fair work queues, and allocation/shutdown failure paths. These software tests do not establish physical capacity at those counts; real USB bandwidth, CPU load, and camera coverage remain setup-dependent.

A hardware-free USB regression exercises peer-camera shutdown and recovery with every transfer parked; that isolated test also passes AddressSanitizer/UndefinedBehaviorSanitizer and ThreadSanitizer, without establishing physical USB recovery behavior.

A fifth physical sensor was added to the calibrated four-sensor room. All five recorded concurrent camera streams, and the added camera was calibrated in the existing coordinate frame without modifying the original four camera poses. The owner confirmed alignment and smooth in-app movement with five sensors; diagnostics observed all 29 daemon inputs, all 26 inputs supported by the legacy SteamVR interface, and both haptic responses. Its first USB hub placement exceeded available bandwidth; moving it to a separate USB path allowed all five streams to run. This validates that configuration, not a universal five-camera USB capacity guarantee.

## Measurement interpretation

Development captures compared tracked controller poses to a rigidly attached SteamVR reference and checked cross-camera/Touch-to-Touch consistency. A reference's own errors, non-rigid attachment, changed room setup, clock alignment, and fit/test overlap can dominate the residual. Tracking is not established as more accurate than another controller merely because a fitted residual is small.

Radio-arrival-to-publication measurements characterize added software delay. They exclude the radio's sampling/delivery interval, camera exposure, SteamVR prediction, application rendering, and display scan-out. No universal sub-millisecond end-to-end claim is made here. Timing knobs and prediction shaping are expert controls that need out-of-sample checks.

Raw development recordings and personal calibration are not included. Consequently, historical numerical measurements are not presented as reproducible public benchmarks. A publishable benchmark should specify hardware, software commit, sensor layout, capture/clock procedure, reference quality, held-out data, and uncertainty, and distribute appropriately sanitized data with permission.

## Before relying on a new setup

Exercise every button and axis, capacitive input, both hands, and haptics in a focused SteamVR app. Test sleep/wake, disconnect/reconnect, daemon restart, optical occlusion and relock, poor coverage, and a SteamVR restart. Check long sessions for stopped camera streams or recurring USB failures. Re-check registration after room setup changes. Report failures with the exact version and conditions.
