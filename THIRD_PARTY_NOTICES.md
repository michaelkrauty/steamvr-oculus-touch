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

## Build and runtime dependencies

Rust dependency versions and checksums are pinned in `Cargo.lock`. Their license metadata is available from Cargo/their package sources: libc (MIT/Apache-2.0), nalgebra (BSD-3-Clause), serde/serde_json (MIT/Apache-2.0), bindgen (BSD-3-Clause), sha2 (MIT/Apache-2.0), and their transitive dependencies. These packages are not vendored here. Native system dependencies include hidapi, libusb, OpenCV, JPEG, and optionally PipeWire/SPA; their upstream licenses continue to apply. Distributing a prebuilt bundle requires reviewing the licenses of every included library, not merely this source repository.

SteamVR is installed separately. Its Oculus input profiles and controller meshes are referenced from that installation; no Oculus firmware, proprietary runtime, mesh, texture, input-profile asset, or SteamVR binary is redistributed here. Oculus, Touch, Meta, Steam, SteamVR, and OpenVR names are used to identify interoperability targets and do not imply endorsement.
