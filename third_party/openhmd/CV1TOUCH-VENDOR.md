# Modified OpenHMD acquisition engine

Source: [Jan Schmidt's OpenHMD fork](https://github.com/thaytan/OpenHMD), branch `rift-kalman-filter`, commit [`a4ebf094e8be5c53c43c5f68d92e87afa5bdf0c8`](https://github.com/thaytan/OpenHMD/commit/a4ebf094e8be5c53c43c5f68d92e87afa5bdf0c8), dated 2026-07-29 ("rift: Backport pose metric tweaks from Monado"). Imported as source, without upstream history; this is a modified snapshot, not an unmodified upstream release.

Most source is under Boost Software License 1.0; dual-license files are used under their Boost option. `nxjson` retains MIT, and `miniz` retains Unlicense/public-domain terms. See the individual source headers, this directory's `LICENSE`, and the root `THIRD_PARTY_NOTICES.md`. The unused upstream GStreamer plugin and 3Glasses driver are omitted; other upstream hardware sources are retained but only the Rift driver is configured by cv1touch.

Local changes expose a bounded CV1 acquisition API, controller/radio decoding, factory geometry, timestamps, raw/labelled observations, optional PipeWire views, fixed calibrated camera poses, radio-only headset operation, and more resilient camera transfers. Tracking for cv1touch is performed in Rust. Native dependencies are resolved from the system; OpenCV 4 and 5 discovery is supported. The small nxjson UTF-8 encoder is replaced with an independently written Unicode-scalar encoder and boundary tests.

Compare this snapshot against the pinned upstream commit to inspect the complete modification set. Subsequent release changes are tracked in this repository. Preserve upstream notices when modifying or extracting code.
