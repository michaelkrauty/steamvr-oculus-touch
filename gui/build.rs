// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Gives the Windows program its icon, drawn by the same code as the tray's.

#[allow(dead_code)]
#[path = "src/icon.rs"]
mod icon;
#[allow(dead_code)]
#[path = "src/png.rs"]
mod png;
#[path = "src/res.rs"]
mod res;

fn main() {
    for path in ["build.rs", "src/icon.rs", "src/png.rs", "src/res.rs"] {
        println!("cargo:rerun-if-changed={path}");
    }
    let var = |name| std::env::var(name).unwrap_or_default();
    // link.exe takes a .res as an input; GNU ld needs it converted by windres first, and
    // releases use MSVC.
    if var("CARGO_CFG_TARGET_OS") == "windows" && var("CARGO_CFG_TARGET_ENV") == "msvc" {
        let path = std::path::Path::new(&var("OUT_DIR")).join("cv1touch-gui.res");
        std::fs::write(&path, res::icon()).expect("writing the icon resource");
        println!("cargo:rustc-link-arg-bin=cv1touch-gui={}", path.display());
    }
}
