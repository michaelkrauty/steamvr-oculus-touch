// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#[path = "../build_support.rs"]
mod build_support;

use std::path::Path;

#[test]
fn profile_directory_does_not_depend_on_package_directory_depth() {
    for profile in [
        "target/debug",
        "custom-target/release",
        "target/x86_64-pc-windows-msvc/debug",
        "build/custom-target/custom-profile",
    ] {
        for package in ["cv1touch-012345/out", "cv1touch/012345/out"] {
            let out = Path::new(profile).join("build").join(package);
            assert_eq!(build_support::profile_dir(&out), Path::new(profile));
        }
    }
}

#[test]
#[should_panic(expected = "Cargo OUT_DIR must be inside a profile's build directory")]
fn missing_build_directory_is_an_error() {
    build_support::profile_dir(Path::new("target/debug/cv1touch/out"));
}
