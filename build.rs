// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Build the pinned native engine in Cargo's profile-specific output directory.
use std::{
    env, fs,
    path::{Path, PathBuf},
    process::Command,
};

fn run(cmd: &mut Command) {
    let status = cmd
        .status()
        .unwrap_or_else(|e| panic!("failed to run {cmd:?}: {e}"));
    assert!(status.success(), "{cmd:?} failed: {status}");
}

fn main() {
    let root = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    for p in [
        "third_party/openhmd",
        "third_party/openvr",
        "common",
        "meson.build",
        "tests",
        "driver",
    ] {
        println!("cargo:rerun-if-changed={}", root.join(p).display());
    }
    if env::var_os("CARGO_FEATURE_ENGINE").is_some() {
        build_engine(&root, &out);
    }
    generate_engine(&root, &out);
    generate_openvr(&root, &out);
}

fn cross_target_args() -> Vec<&'static str> {
    let windows = env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows");
    if env::var("HOST") == env::var("TARGET") && !windows {
        Vec::new()
    } else {
        vec!["-ffreestanding"]
    }
}

/// Builds the pinned native engine with meson and links it.
fn build_engine(root: &Path, out: &Path) {
    assert_eq!(
        env::var("CARGO_CFG_TARGET_OS").as_deref(),
        Ok("linux"),
        "the native engine builds on Linux only so far"
    );
    assert_eq!(
        env::var("CARGO_CFG_TARGET_ARCH").as_deref(),
        Ok("x86_64"),
        "cv1touch supports x86_64 only"
    );
    assert_eq!(
        env::var("HOST"),
        env::var("TARGET"),
        "cross compilation of the native engine is not supported"
    );
    let native = out.join("native");
    let mut meson = Command::new("meson");
    meson.arg("setup");
    if native.join("build.ninja").exists() {
        meson.arg("--reconfigure");
    }
    run(meson.arg("--wrap-mode=nodownload").arg(&native).arg(root));
    let jobs = env::var("CARGO_BUILD_JOBS")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .filter(|&n| n > 0)
        .unwrap_or(4);
    run(Command::new("ninja")
        .arg("-C")
        .arg(&native)
        .arg("-j")
        .arg(jobs.to_string()));
    println!("cargo:rerun-if-env-changed=CARGO_BUILD_JOBS");
    let lib_dir = native.join("third_party/openhmd");
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=dylib=openhmd");
    // Installed executables live in bin, with the native engine in lib/cv1touch.
    // Cargo supplies the native search path for `cargo run` and `cargo test`.
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../lib/cv1touch");
    println!("cargo:rustc-env=CV1TOUCH_NATIVE_DIR={}", native.display());
    let profile_dir = out
        .ancestors()
        .nth(3)
        .expect("Cargo OUT_DIR profile directory");
    fs::write(
        profile_dir.join("cv1touch-native-dir"),
        format!("{}\n", native.display()),
    )
    .expect("write native artifact marker");
}

fn generate_engine(root: &Path, out: &Path) {
    bindgen::Builder::default()
        .header(root.join("common/cv1touch_ffi.h").to_str().unwrap())
        .clang_arg(format!(
            "-I{}",
            root.join("third_party/openhmd/include").display()
        ))
        .clang_arg(format!("-I{}", root.join("common").display()))
        .clang_arg("-DCV1T_SHM_TYPES_ONLY")
        // another target, or Windows (whose C headers clang would have to find in a Visual Studio
        // installation): the bindings need only clang's own headers (stdint, stddef, stdbool)
        .clang_args(cross_target_args())
        .allowlist_function("ohmd_.*")
        .allowlist_type("(ohmd|cv1t)_.*")
        .allowlist_var("(OHMD|CV1T)_.*")
        .prepend_enum_name(false)
        .derive_default(true)
        .layout_tests(true)
        .generate()
        .expect("bindgen engine headers failed")
        .write_to_file(out.join("bindings.rs"))
        .expect("write engine bindings");
}

fn generate_openvr(root: &Path, out: &Path) {
    bindgen::Builder::default()
        .header(root.join("third_party/openvr/headers/openvr_capi.h").to_str().unwrap())
        .allowlist_type("VR_IVRSystem_FnTable|TrackedDevicePose_t|HmdMatrix34_t")
        .allowlist_var("(ETrackedDeviceClass|ETrackingUniverseOrigin|EVRApplicationType|ETrackedDeviceProperty|ETrackingResult)_.*")
        .allowlist_type("ETrackedDeviceClass|ETrackingUniverseOrigin|EVRApplicationType|ETrackedDeviceProperty|ETrackedPropertyError|EVRInitError|ETrackingResult")
        .prepend_enum_name(false).derive_default(true).layout_tests(false)
        .generate().expect("bindgen pinned OpenVR header failed")
        .write_to_file(out.join("openvr.rs")).expect("write OpenVR bindings");
}
