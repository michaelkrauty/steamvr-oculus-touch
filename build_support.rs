// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

use std::path::Path;

pub fn profile_dir(out: &Path) -> &Path {
    out.ancestors()
        .find(|path| path.file_name().is_some_and(|name| name == "build"))
        .and_then(Path::parent)
        .expect("Cargo OUT_DIR must be inside a profile's build directory")
}
