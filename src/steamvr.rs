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

const OWNERSHIP_FILE: &str = ".cv1touch-owned.json";
const DRIVER_LIBRARY: &str = "bin/linux64/driver_cv1touch.so";
const DRIVER_MANIFEST: &str = "driver.vrdrivermanifest";

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct DriverOwnership {
    format: String,
    version: u32,
    files: std::collections::BTreeMap<String, String>,
}

fn fingerprint(path: &Path) -> Result<String, String> {
    use sha2::{Digest, Sha256};
    let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(format!("{:x}", Sha256::digest(bytes)))
}

fn ownership(so: &Path, manifest: &Path) -> Result<DriverOwnership, String> {
    Ok(DriverOwnership {
        format: "cv1touch-driver-ownership".into(),
        version: 1,
        files: [
            (DRIVER_LIBRARY.into(), fingerprint(so)?),
            (DRIVER_MANIFEST.into(), fingerprint(manifest)?),
        ]
        .into(),
    })
}

/// Read-only validation. Legacy files are trusted only when exactly equal to the supplied
/// previously installed source assets; a matching driver name alone does not establish ownership.
pub fn check_driver(dir: &Path, legacy: Option<(&Path, &Path)>) -> Result<(), String> {
    match std::fs::symlink_metadata(dir) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(()),
        Err(e) => return Err(format!("{}: {e}", dir.display())),
        Ok(m) if !m.is_dir() || m.file_type().is_symlink() => {
            return Err(format!(
                "{}: not a plain driver directory; retained",
                dir.display()
            ))
        }
        Ok(_) => {}
    }
    fn check_tree(root: &Path, dir: &Path) -> Result<(), String> {
        for entry in std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))? {
            let entry = entry.map_err(|e| e.to_string())?;
            let path = entry.path();
            let relative = path.strip_prefix(root).map_err(|e| e.to_string())?;
            let kind = entry.file_type().map_err(|e| e.to_string())?;
            let directory = ["bin", "bin/linux64"]
                .iter()
                .any(|p| relative == Path::new(p));
            let file = [DRIVER_LIBRARY, DRIVER_MANIFEST, OWNERSHIP_FILE]
                .iter()
                .any(|p| relative == Path::new(p));
            if kind.is_symlink() || !(directory && kind.is_dir() || file && kind.is_file()) {
                return Err(format!(
                    "{}: unknown content or symlink retained",
                    path.display()
                ));
            }
            if directory {
                check_tree(root, &path)?;
            }
        }
        Ok(())
    }
    check_tree(dir, dir)?;
    let record_path = dir.join(OWNERSHIP_FILE);
    let record = if record_path.exists() {
        let record: DriverOwnership =
            serde_json::from_slice(&std::fs::read(&record_path).map_err(|e| e.to_string())?)
                .map_err(|e| {
                    format!(
                        "{}: invalid ownership metadata: {e}; retained",
                        record_path.display()
                    )
                })?;
        if record.format != "cv1touch-driver-ownership"
            || record.version != 1
            || record.files.len() != 2
            || !record.files.contains_key(DRIVER_LIBRARY)
            || !record.files.contains_key(DRIVER_MANIFEST)
        {
            return Err(format!(
                "{}: unsupported ownership metadata; retained",
                record_path.display()
            ));
        }
        record
    } else {
        let (so, manifest) =
            legacy.ok_or_else(|| format!("{}: unmanaged driver; retained", dir.display()))?;
        ownership(so, manifest)?
    };
    for (name, expected) in record.files {
        let path = dir.join(name);
        if fingerprint(&path)? != expected {
            return Err(format!("{}: modified or unmanaged driver file; retained (back up local changes and remove the driver package {} before retrying)", path.display(), dir.display()));
        }
    }
    Ok(())
}

fn staging_dir(dir: &Path) -> Result<PathBuf, String> {
    let parent = dir.parent().ok_or("driver directory has no parent")?;
    std::fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(|e| e.to_string())?
        .as_nanos();
    let stage = parent.join(format!(".cv1touch-stage-{}-{nanos}", std::process::id()));
    std::fs::create_dir(&stage).map_err(|e| format!("{}: {e}", stage.display()))?;
    Ok(stage)
}

fn sync_directory(path: &Path) -> Result<(), String> {
    std::fs::File::open(path)
        .and_then(|f| f.sync_all())
        .map_err(|e| format!("{}: {e}", path.display()))
}

/// Linux-only atomic directory publication keeps ownership metadata and both payloads together.
/// Unsupported rename operations leave the existing package untouched and report the stage.
fn publish_directory(
    stage: &Path,
    dir: &Path,
    exists: bool,
    legacy: Option<(&Path, &Path)>,
) -> Result<(), String> {
    use std::os::unix::ffi::OsStrExt;
    let from = std::ffi::CString::new(stage.as_os_str().as_bytes()).map_err(|e| e.to_string())?;
    let to = std::ffi::CString::new(dir.as_os_str().as_bytes()).map_err(|e| e.to_string())?;
    let flags = if exists {
        libc::RENAME_EXCHANGE
    } else {
        libc::RENAME_NOREPLACE
    };
    let result = unsafe {
        libc::renameat2(
            libc::AT_FDCWD,
            from.as_ptr(),
            libc::AT_FDCWD,
            to.as_ptr(),
            flags,
        )
    };
    if result != 0 {
        return Err(format!(
            "could not atomically publish {}: {}; existing package retained, staged files at {}",
            dir.display(),
            std::io::Error::last_os_error(),
            stage.display()
        ));
    }
    sync_directory(dir.parent().ok_or("driver directory has no parent")?)?;
    if exists {
        // If an external writer raced the final check, retain its changes as recovery evidence.
        check_driver(stage, legacy).map_err(|e| {
            format!(
                "new package installed; previous package retained at {}: {e}",
                stage.display()
            )
        })?;
        std::fs::remove_dir_all(stage).map_err(|e| {
            format!(
                "new package installed; old package retained at {}: {e}",
                stage.display()
            )
        })?;
    }
    Ok(())
}

pub fn assemble_with_legacy(
    built_so: &Path,
    manifest: &Path,
    dir: &Path,
    legacy: Option<(&Path, &Path)>,
) -> Result<(), String> {
    let record = ownership(built_so, manifest)?;
    check_driver(dir, legacy)?;
    let stage = staging_dir(dir)?;
    let prepare = || -> Result<(), String> {
        std::fs::create_dir_all(stage.join("bin/linux64")).map_err(|e| e.to_string())?;
        for (source, name) in [(built_so, DRIVER_LIBRARY), (manifest, DRIVER_MANIFEST)] {
            let destination = stage.join(name);
            std::fs::copy(source, &destination)
                .map_err(|e| format!("{}: {e}", source.display()))?;
            std::fs::File::open(&destination)
                .and_then(|f| f.sync_all())
                .map_err(|e| e.to_string())?;
        }
        let metadata = stage.join(OWNERSHIP_FILE);
        std::fs::write(
            &metadata,
            serde_json::to_vec_pretty(&record).map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        std::fs::File::open(&metadata)
            .and_then(|f| f.sync_all())
            .map_err(|e| e.to_string())?;
        sync_directory(&stage.join("bin/linux64"))?;
        sync_directory(&stage.join("bin"))?;
        sync_directory(&stage)?;
        check_driver(&stage, None)?;
        // Revalidate after staging, before replacing any existing payload.
        check_driver(dir, legacy)?;
        publish_directory(&stage, dir, dir.exists(), legacy)
    };
    prepare().map_err(|e| format!("{e} (staging path: {})", stage.display()))
}

pub fn assemble(built_so: &Path, manifest: &Path, dir: &Path) -> Result<(), String> {
    assemble_with_legacy(built_so, manifest, dir, Some((built_so, manifest)))
}

pub fn remove_owned_driver(dir: &Path) -> Result<(), String> {
    let assets = driver_assets().ok();
    let legacy = assets
        .as_ref()
        .map(|(so, manifest)| (so.as_path(), manifest.as_path()));
    check_driver(dir, legacy)?;
    if !dir.exists() {
        return Ok(());
    }
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
    #[test]
    fn modified_driver_payload_is_preserved_before_upgrade_or_remove() {
        for name in ["bin/linux64/driver_cv1touch.so", "driver.vrdrivermanifest"] {
            let d = tmp(&format!("modified-{}", name.replace('/', "-")));
            let so = d.join("source.so");
            let manifest = d.join("source.json");
            std::fs::write(&so, b"original driver").unwrap();
            std::fs::write(&manifest, br#"{"name":"cv1touch"}"#).unwrap();
            let package = d.join("package");
            assemble(&so, &manifest, &package).unwrap();
            let changed = package.join(name);
            std::fs::write(&changed, b"user modification").unwrap();
            assert!(assemble(&so, &manifest, &package).is_err());
            assert!(remove_owned_driver(&package).is_err());
            assert_eq!(std::fs::read(&changed).unwrap(), b"user modification");
            std::fs::remove_dir_all(d).unwrap();
        }
    }

    #[test]
    fn unknown_content_and_symlinks_block_install_before_any_replacement() {
        use std::os::unix::fs::symlink;
        for symbolic in [false, true] {
            let d = tmp(if symbolic {
                "install-symlink"
            } else {
                "install-unknown"
            });
            let so = d.join("source.so");
            let manifest = d.join("source.json");
            std::fs::write(&so, b"old").unwrap();
            std::fs::write(&manifest, br#"{"name":"cv1touch"}"#).unwrap();
            let package = d.join("package");
            assemble(&so, &manifest, &package).unwrap();
            if symbolic {
                std::fs::remove_file(package.join("bin/linux64/driver_cv1touch.so")).unwrap();
                symlink(&so, package.join("bin/linux64/driver_cv1touch.so")).unwrap();
            } else {
                std::fs::write(package.join("personal"), b"keep").unwrap();
            }
            std::fs::write(&so, b"new").unwrap();
            assert!(assemble(&so, &manifest, &package).is_err());
            assert!(remove_owned_driver(&package).is_err());
            if !symbolic {
                assert_eq!(
                    std::fs::read(package.join("bin/linux64/driver_cv1touch.so")).unwrap(),
                    b"old"
                );
            }
            std::fs::remove_dir_all(d).unwrap();
        }
    }

    #[test]
    fn legacy_package_requires_exact_trusted_payload() {
        let d = tmp("legacy");
        let so = d.join("source.so");
        let manifest = d.join("source.json");
        std::fs::write(&so, b"old").unwrap();
        std::fs::write(&manifest, br#"{"name":"cv1touch"}"#).unwrap();
        let package = d.join("package");
        std::fs::create_dir_all(package.join("bin/linux64")).unwrap();
        std::fs::copy(&so, package.join("bin/linux64/driver_cv1touch.so")).unwrap();
        std::fs::copy(&manifest, package.join("driver.vrdrivermanifest")).unwrap();
        assert!(check_driver(&package, None).is_err());
        check_driver(&package, Some((&so, &manifest))).unwrap();
        std::fs::write(package.join("bin/linux64/driver_cv1touch.so"), b"changed").unwrap();
        assert!(check_driver(&package, Some((&so, &manifest))).is_err());
        std::fs::write(package.join("bin/linux64/driver_cv1touch.so"), b"old").unwrap();
        assemble(&so, &manifest, &package).unwrap();
        std::fs::write(&so, b"new").unwrap();
        assemble(&so, &manifest, &package).unwrap();
        assemble(&so, &manifest, &package).unwrap();
        remove_owned_driver(&package).unwrap();
        remove_owned_driver(&package).unwrap();
        std::fs::remove_dir_all(d).unwrap();
    }
    #[test]
    fn legacy_upgrade_can_validate_old_assets_and_install_new_assets() {
        let d = tmp("legacy-upgrade");
        let old = d.join("old.so");
        let new = d.join("new.so");
        let manifest = d.join("manifest");
        std::fs::write(&old, b"old").unwrap();
        std::fs::write(&new, b"new").unwrap();
        std::fs::write(&manifest, br#"{"name":"cv1touch"}"#).unwrap();
        let package = d.join("package");
        std::fs::create_dir_all(package.join("bin/linux64")).unwrap();
        std::fs::copy(&old, package.join(DRIVER_LIBRARY)).unwrap();
        std::fs::copy(&manifest, package.join(DRIVER_MANIFEST)).unwrap();
        assert!(assemble(&new, &manifest, &package).is_err());
        assemble_with_legacy(&new, &manifest, &package, Some((&old, &manifest))).unwrap();
        assert_eq!(std::fs::read(package.join(DRIVER_LIBRARY)).unwrap(), b"new");
        check_driver(&package, None).unwrap();
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn failed_atomic_publish_keeps_existing_directory_and_stage() {
        let d = tmp("publish-failure");
        let package = d.join("package");
        let stage = d.join("stage");
        std::fs::create_dir(&package).unwrap();
        std::fs::create_dir(&stage).unwrap();
        std::fs::write(package.join("original"), b"keep").unwrap();
        // A new-install operation cannot replace a destination that appeared meanwhile.
        assert!(publish_directory(&stage, &package, false, None).is_err());
        assert_eq!(std::fs::read(package.join("original")).unwrap(), b"keep");
        assert!(stage.is_dir());
        std::fs::remove_dir_all(d).unwrap();
    }
}
