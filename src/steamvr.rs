// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Installing the cv1touch SteamVR driver: the driver directory (manifest + library), its
//! registration with SteamVR's `vrpathreg`, and `activateMultipleDrivers` in steamvr.vrsettings
//! (backed up first) so it runs beside the Lighthouse driver.

use std::path::{Path, PathBuf};

fn home() -> PathBuf {
    PathBuf::from(std::env::var_os("HOME").unwrap_or_default())
}

/// Native Steam configuration root; may differ from the library containing SteamVR.
pub fn steam_root() -> PathBuf {
    std::env::var_os("CV1TOUCH_STEAM_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| home().join(".steam/steam"))
}

fn registry_path() -> PathBuf {
    let config = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|| home().join(".config"));
    config.join("openvr/openvrpaths.vrpath")
}

fn path_registry() -> Option<serde_json::Value> {
    std::fs::read_to_string(registry_path())
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
}

pub fn driver_registered(dir: &Path) -> Result<bool, String> {
    let text = match std::fs::read_to_string(registry_path()) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(false),
        Err(e) => return Err(format!("OpenVR path registry: {e}")),
    };
    let value: serde_json::Value =
        serde_json::from_str(&text).map_err(|e| format!("OpenVR path registry: {e}"))?;
    Ok(value["external_drivers"].as_array().is_some_and(|paths| {
        paths.iter().any(|p| {
            p.as_str().is_some_and(|p| {
                Path::new(p) == dir
                    || (std::fs::canonicalize(p)
                        .ok()
                        .zip(std::fs::canonicalize(dir).ok())
                        .is_some_and(|(a, b)| a == b))
            })
        })
    }))
}

fn registered_path(key: &str) -> Option<PathBuf> {
    path_registry()?[key]
        .as_array()?
        .iter()
        .filter_map(|p| p.as_str().map(PathBuf::from))
        .find(|p| p.is_dir())
}

/// The SteamVR runtime directory, not the Steam installation root.
pub fn runtime_root() -> PathBuf {
    std::env::var_os("CV1TOUCH_STEAMVR")
        .map(PathBuf::from)
        .or_else(|| registered_path("runtime"))
        .unwrap_or_else(|| steam_root().join("steamapps/common/SteamVR"))
}

pub fn vrsettings_path() -> PathBuf {
    if std::env::var_os("CV1TOUCH_STEAM_ROOT").is_some() {
        return steam_root().join("config/steamvr.vrsettings");
    }
    registered_path("config")
        .unwrap_or_else(|| steam_root().join("config"))
        .join("steamvr.vrsettings")
}

pub fn vrpathreg() -> PathBuf {
    runtime_root().join("bin/vrpathreg.sh")
}

fn installed_assets(exe: &Path) -> PathBuf {
    exe.parent()
        .unwrap_or_else(|| Path::new("."))
        .join("../lib/cv1touch")
}

/// Driver payload is installed beside the engine; development builds use Cargo's native output.
pub fn driver_assets() -> Result<(PathBuf, PathBuf), String> {
    let explicit = std::env::var_os("CV1TOUCH_ASSETS").map(PathBuf::from);
    let dir = explicit
        .clone()
        .unwrap_or_else(|| installed_assets(&std::env::current_exe().unwrap_or_default()));
    let so = dir.join("driver_cv1touch.so");
    let manifest = dir.join("driver.vrdrivermanifest");
    if so.is_file() && manifest.is_file() {
        return Ok((so, manifest));
    }
    if explicit.is_some() {
        return Err(format!(
            "{}: needs driver_cv1touch.so and driver.vrdrivermanifest",
            dir.display()
        ));
    }
    let native = PathBuf::from(env!("CV1TOUCH_NATIVE_DIR"));
    let so = native.join("driver/driver_cv1touch.so");
    let manifest =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("driver/cv1touch/driver.vrdrivermanifest");
    if so.is_file() && manifest.is_file() {
        Ok((so, manifest))
    } else {
        Err(format!("driver assets missing: install the complete runtime or set CV1TOUCH_ASSETS (checked {})", dir.display()))
    }
}

pub fn driver_dir() -> PathBuf {
    std::env::var_os("XDG_DATA_HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|| home().join(".local/share"))
        .join("cv1touch/steamvr/cv1touch")
}

fn copy_atomic(from: &Path, to: &Path) -> Result<(), String> {
    if let Some(d) = to.parent() {
        std::fs::create_dir_all(d).map_err(|e| format!("{}: {e}", d.display()))?;
    }
    let tmp = to.with_extension("tmp");
    std::fs::copy(from, &tmp).map_err(|e| format!("{}: {e}", from.display()))?;
    std::fs::rename(&tmp, to).map_err(|e| format!("{}: {e}", to.display()))
}

pub fn assemble(built_so: &Path, manifest: &Path, dir: &Path) -> Result<(), String> {
    copy_atomic(built_so, &dir.join("bin/linux64/driver_cv1touch.so"))?;
    copy_atomic(manifest, &dir.join("driver.vrdrivermanifest"))
}

/// Remove only the package layout assembled by cv1touch; retain any unknown content.
pub fn remove_owned_driver(dir: &Path) -> Result<(), String> {
    if !dir.exists() {
        return Ok(());
    }
    let manifest = dir.join("driver.vrdrivermanifest");
    let value: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(&manifest).map_err(|e| format!("{}: {e}", manifest.display()))?,
    )
    .map_err(|e| e.to_string())?;
    if value["name"] != "cv1touch" {
        return Err(format!(
            "{}: not a cv1touch manifest; retained",
            dir.display()
        ));
    }
    let known = [
        "driver.vrdrivermanifest",
        "bin",
        "bin/linux64",
        "bin/linux64/driver_cv1touch.so",
    ];
    fn check(root: &Path, dir: &Path, known: &[&str]) -> Result<(), String> {
        if std::fs::symlink_metadata(dir)
            .map_err(|e| e.to_string())?
            .file_type()
            .is_symlink()
        {
            return Err(format!("{}: symlink retained", dir.display()));
        }
        for entry in std::fs::read_dir(dir).map_err(|e| e.to_string())? {
            let entry = entry.map_err(|e| e.to_string())?;
            let path = entry.path();
            let relative = path.strip_prefix(root).map_err(|e| e.to_string())?;
            if !known.iter().any(|k| relative == Path::new(k)) {
                return Err(format!("{}: unknown file retained", path.display()));
            }
            let kind = entry.file_type().map_err(|e| e.to_string())?;
            if kind.is_symlink() {
                return Err(format!("{}: symlink retained", path.display()));
            }
            if kind.is_dir() {
                check(root, &path, known)?;
            }
        }
        Ok(())
    }
    check(dir, dir, &known)?;
    std::fs::remove_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))
}

/// Read-only preflight before driver registration or payload replacement.
pub fn validate_settings(path: &Path) -> Result<(), String> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(()),
        Err(e) => return Err(format!("{}: {e}", path.display())),
    };
    let value: serde_json::Value = serde_json::from_str(&text)
        .map_err(|e| format!("{}: {e} (not touching it)", path.display()))?;
    if !value.is_object() || value.get("steamvr").is_some_and(|v| !v.is_object()) {
        return Err(format!(
            "{}: settings and steamvr must be JSON objects",
            path.display()
        ));
    }
    Ok(())
}

/// Sets steamvr.activateMultipleDrivers; keeps every other setting. A missing file is created.
pub fn enable_multiple_drivers(path: &Path) -> Result<Option<PathBuf>, String> {
    let existing = match std::fs::read_to_string(path) {
        Ok(t) => Some(t),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
        Err(e) => return Err(format!("{}: {e}", path.display())),
    };
    let mut v: serde_json::Value = match &existing {
        Some(t) => serde_json::from_str(t)
            .map_err(|e| format!("{}: {e} (not touching it)", path.display()))?,
        None => serde_json::json!({}),
    };
    if v["steamvr"]["activateMultipleDrivers"] == serde_json::Value::Bool(true) {
        return Ok(None);
    }
    let obj = v
        .as_object_mut()
        .ok_or(format!("{}: not a JSON object", path.display()))?;
    let steamvr = obj
        .entry("steamvr")
        .or_insert_with(|| serde_json::json!({}));
    steamvr
        .as_object_mut()
        .ok_or(format!("{}: \"steamvr\" is not an object", path.display()))?
        .insert("activateMultipleDrivers".into(), true.into());
    let backup = match existing {
        Some(t) => {
            let now = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_nanos());
            let b = path.with_file_name(format!("steamvr.vrsettings.bak-{now}"));
            std::fs::write(&b, t).map_err(|e| format!("{}: {e}", b.display()))?;
            Some(b)
        }
        None => None,
    };
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d).map_err(|e| format!("{}: {e}", d.display()))?;
    }
    let tmp = path.with_extension("tmp");
    std::fs::write(
        &tmp,
        serde_json::to_string_pretty(&v).map_err(|e| e.to_string())? + "\n",
    )
    .map_err(|e| format!("{}: {e}", tmp.display()))?;
    std::fs::rename(&tmp, path).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(backup)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("cv1t-steamvr-{}-{tag}", std::process::id()));
        std::fs::remove_dir_all(&d).ok();
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn uninstall_removes_owned_payload_and_retains_unknown_content() {
        let d = tmp("remove-owned");
        std::fs::write(d.join("lib.so"), b"ELF").unwrap();
        std::fs::write(d.join("manifest"), br#"{"name":"cv1touch"}"#).unwrap();
        let owned = d.join("driver");
        assemble(&d.join("lib.so"), &d.join("manifest"), &owned).unwrap();
        std::fs::write(owned.join("personal-file"), b"keep").unwrap();
        assert!(remove_owned_driver(&owned).is_err());
        assert_eq!(std::fs::read(owned.join("personal-file")).unwrap(), b"keep");
        std::fs::remove_file(owned.join("personal-file")).unwrap();
        remove_owned_driver(&owned).unwrap();
        assert!(!owned.exists());
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn installed_payload_follows_relocated_prefix() {
        assert_eq!(
            installed_assets(Path::new("/opt/moved/bin/cv1touch")),
            Path::new("/opt/moved/bin/../lib/cv1touch")
        );
    }

    #[test]
    fn enables_multiple_drivers_keeping_everything_else() {
        let d = tmp("settings");
        let p = d.join("steamvr.vrsettings");
        std::fs::write(
            &p,
            r#"{"steamvr":{"motionSmoothing":false},"driver_lighthouse":{"enable":true}}"#,
        )
        .unwrap();
        let backup = enable_multiple_drivers(&p).unwrap().expect("a backup");
        assert_eq!(
            std::fs::read_to_string(&backup).unwrap(),
            r#"{"steamvr":{"motionSmoothing":false},"driver_lighthouse":{"enable":true}}"#
        );
        let v: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&p).unwrap()).unwrap();
        assert_eq!(v["steamvr"]["activateMultipleDrivers"], true);
        assert_eq!(v["steamvr"]["motionSmoothing"], false);
        assert_eq!(v["driver_lighthouse"]["enable"], true);
        assert!(enable_multiple_drivers(&p).unwrap().is_none()); // already on: untouched
    }

    #[test]
    fn creates_a_missing_settings_file() {
        let d = tmp("missing");
        let p = d.join("config").join("steamvr.vrsettings");
        assert!(enable_multiple_drivers(&p).unwrap().is_none());
        let v: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&p).unwrap()).unwrap();
        assert_eq!(v["steamvr"]["activateMultipleDrivers"], true);
    }

    #[test]
    fn refuses_a_broken_settings_file() {
        let d = tmp("broken");
        let p = d.join("steamvr.vrsettings");
        std::fs::write(&p, "{ not json").unwrap();
        assert!(enable_multiple_drivers(&p)
            .unwrap_err()
            .contains("steamvr.vrsettings"));
        assert_eq!(std::fs::read_to_string(&p).unwrap(), "{ not json");
    }

    #[test]
    fn assembles_the_driver_directory() {
        let d = tmp("assemble");
        std::fs::write(d.join("lib.so"), b"ELF").unwrap();
        std::fs::write(d.join("driver.vrdrivermanifest"), b"{}").unwrap();
        let out = d.join("cv1touch");
        assemble(&d.join("lib.so"), &d.join("driver.vrdrivermanifest"), &out).unwrap();
        assert_eq!(
            std::fs::read(out.join("bin/linux64/driver_cv1touch.so")).unwrap(),
            b"ELF"
        );
        assert_eq!(
            std::fs::read(out.join("driver.vrdrivermanifest")).unwrap(),
            b"{}"
        );
        assert!(assemble(
            &d.join("missing.so"),
            &d.join("driver.vrdrivermanifest"),
            &out
        )
        .unwrap_err()
        .contains("missing.so"));
    }
}
