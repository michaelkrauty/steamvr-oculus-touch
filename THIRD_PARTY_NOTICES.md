# Third-party notices

The root Apache 2.0 license covers original SteamVR Oculus Touch code and local modifications only to the extent the author may license them. It does not replace third-party licenses or remove their copyright notices. Source headers retain individual authorship.

## Vendored source

| Component | Origin and revision | Applicable terms |
| --- | --- | --- |
| OpenHMD acquisition engine | [thaytan/OpenHMD](https://github.com/thaytan/OpenHMD), `a4ebf094e8be5c53c43c5f68d92e87afa5bdf0c8`, modified | [Boost 1.0](third_party/openhmd/LICENSE), including the Boost option in dual-license files; exceptions below |
| LambdaTwist headers | Bundled by that OpenHMD revision; retained authorship in each header | Boost option of GPL-2.0-or-later/Boost dual license |
| nxjson | Bundled by OpenHMD; Copyright 2018 Yaroslav Stavnichiy | [MIT](third_party/licenses/nxjson-MIT.txt); small UTF-8 encoder independently replaced |
| miniz | Bundled by OpenHMD; authors and history in source | [Unlicense](third_party/licenses/miniz-Unlicense.txt); original dedication retained in source |
| OpenVR SDK headers | [ValveSoftware/openvr](https://github.com/ValveSoftware/openvr), pinned revision documented [here](third_party/openvr/PROVENANCE.md) | [BSD 3-Clause](third_party/openvr/LICENSE), Copyright Valve Corporation |

The unused upstream GStreamer plugin and 3Glasses driver are not distributed in this snapshot. OpenHMD files bearing both LGPL and Boost alternatives are used under Boost, with their original notices intact.

Local OpenHMD changes match configured, discovered and calibrated sensor serials without regard to ASCII letter case, using bounded comparisons.

The local OpenHMD log callback forwards the daemon's native diagnostics to its bounded logging queue; other clients retain the engine's original stdout/stderr behavior for ordinary logs. The libusb callback covers its normal logging implementation; special libusb build options can bypass it.

Native assertion sites use a project macro and fixed-buffer failure callback, so the daemon can drain briefly and abort without its C runtime's blocking assertion printer. Assertions remain enabled in production. Without an installed receiver, engine assertions abort without synchronous output; standalone unity harnesses retain their ordinary assertions.

Fatal records are independent of the native printf accumulator. Reentered log receivers and fatal callbacks abort silently without invoking callbacks or draining again.

Local OpenHMD changes count received camera frames atomically before analysis locks or queues and expose these 64-bit counters through the separate `ohmd_cv1touch_get_sensor_video` extension. Existing native sensor samples and controller shared memory retain their 32-bit released-buffer counter and layout.

Local OpenHMD changes also retain per-sensor USB video startup diagnostics through cleanup, expose them through a separate `ohmd_cv1touch_get_sensor_stream` extension, and record the setting used by the last start attempt. The existing sensor sample and controller shared-memory layouts are unchanged; the diagnostics do not select another USB setting or retry a failed stream.

Local OpenHMD changes release parser copies, cached Touch calibration and temporary LED models on failure, propagate parse errors, and clean up failed POSIX calibration-file reads. Malformed cached calibration falls back to the radio by its observed factory hash; allocation failures keep the cache eligible. Terminal failures back off from I/O completion, retain history through silence, and report sanitized failure transitions from the second consecutive failure. Pending calibration advances from the engine loop, with a five-second deadline per radio command, refreshed only when a flash block arrives and checked once more before the command is given up. An abandoned command, including a haptics write whose status could not be read, is drained before one replacement is sent; the radio has no abort or transaction tag. Shared drain polls wait 100 ms from completion, with one warning after 10 s still waiting. Short replies cannot confirm identity or haptics. HIDAPI failures map to input/output errors. nxjson clears numeric conversion errno and unwinds allocation failures with a per-call error. Calibration requires usable roots, IMU/LED geometry, whole-number increasing input ranges and nonzero consumed capacitive ranges; missing dead zones and offsets retain defaults, and unused capacitive slots are ignored. The upstream revision and source license headers remain unchanged.

Local OpenHMD cache writes use a consistent negative-error convention, publish a written and closed sibling temporary file, and retain complete cached data on failed writes. POSIX saves create the temporary file exclusively with the mode an ordinary write would give and rename it into place. Windows saves use an exclusive temporary file and MoveFileExW replacement, skip UNC/extended path roots during directory creation, normalize joined path separators, and map Win32 errors to their specific errno. Neither syncs to disk, because the save runs on the tracking thread. Failed saves log one warning naming the file, and valid controller calibration remains usable when its optional save fails. New path/error helpers retain Boost licensing.

## Build and runtime dependencies

Rust dependency versions and checksums are pinned in `Cargo.lock`. Their license metadata is available from Cargo/their package sources: libc (MIT/Apache-2.0), miniz_oxide (MIT/Zlib/Apache-2.0), nalgebra (BSD-3-Clause), serde/serde_json (MIT/Apache-2.0), bindgen (BSD-3-Clause), sha2 (MIT/Apache-2.0), and their transitive dependencies. These packages are not vendored here. Native system dependencies include hidapi, libusb, OpenCV, JPEG, and optionally PipeWire/SPA; their upstream licenses continue to apply. Distributing a prebuilt bundle requires reviewing the licenses of every included library, not merely this source repository.

## Windows bundle

The Windows installer ships binaries, which Linux leaves to the distribution's packages, so `windows\bundle.ps1` puts the terms of every library they carry into `third_party\licenses` beside the program, as vcpkg built them: `windows-bundle.txt` names each port and its version, `<port>-copyright.txt` holds its terms, `rust-crates.txt` lists the Rust crates the build uses for this target (linked into the executables, or building them) with the licence each declares, and `rust\<crate>-<version>\` holds each crate's own licence files.

| Component | How it ships | Applicable terms |
| --- | --- | --- |
| libusb | `libusb-1.0.dll`, its own DLL, so that it can be replaced by any compatible build; its source is at [libusb/libusb](https://github.com/libusb/libusb) under the version `windows-bundle.txt` names | LGPL-2.1-or-later (`libusb-copyright.txt`) |
| OpenCV (calib3d and what it needs) | statically linked into `openhmd-0.dll` | Apache-2.0 (`opencv4-copyright.txt`) |
| libjpeg-turbo | statically linked into `openhmd-0.dll` | IJG, BSD-3-Clause and zlib (`libjpeg-turbo-copyright.txt`) |
| hidapi | statically linked into `openhmd-0.dll` | BSD-3-Clause option (`hidapi-copyright.txt`) |
| zlib | statically linked, through the above | Zlib (`zlib-copyright.txt`) |
| Rust crates | statically linked into `cv1touchd.exe`, `cv1touch.exe` and `cv1touchd-quiet.exe`, or used to build them | each crate's own, listed in `rust-crates.txt` with the texts in `rust\` (MIT, Apache-2.0, BSD-3-Clause and the like) |

SteamVR is installed separately. Its Oculus input profiles and controller meshes are referenced from that installation; no Oculus firmware, proprietary runtime, mesh, texture, input-profile asset, or SteamVR binary is redistributed here. Oculus, Touch, Meta, Steam, SteamVR, and OpenVR names are used to identify interoperability targets and do not imply endorsement.
