// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Installing the cv1touch SteamVR driver: the driver directory (manifest, library, the input
//! profile derived from SteamVR's own Oculus Touch profile and the names SteamVR shows for the
//! controllers), its registration with SteamVR's
//! `vrpathreg`, and `activateMultipleDrivers` in steamvr.vrsettings (backed up first) so it runs
//! beside the Lighthouse driver.

use std::path::{Path, PathBuf};

#[cfg(unix)]
fn default_steam_root() -> PathBuf {
    PathBuf::from(std::env::var_os("HOME").unwrap_or_default()).join(".steam/steam")
}

/// Steam's default location; the OpenVR path registry names the real ones once SteamVR has run.
#[cfg(windows)]
fn default_steam_root() -> PathBuf {
    std::env::var_os("ProgramFiles(x86)")
        .map_or_else(|| PathBuf::from(r"C:\Program Files (x86)"), PathBuf::from)
        .join("Steam")
}

/// Native Steam configuration root; may differ from the library containing SteamVR.
pub fn steam_root() -> PathBuf {
    std::env::var_os("CV1TOUCH_STEAM_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(default_steam_root)
}

fn registry_path() -> PathBuf {
    crate::platform::dirs::get()
        .config_home
        .join("openvr/openvrpaths.vrpath")
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
        .unwrap_or_else(|| {
            steam_root()
                .join("steamapps")
                .join("common")
                .join("SteamVR")
        })
}

/// Where SteamVR writes its logs (vrserver.txt holds the driver's lines).
pub fn log_dir() -> PathBuf {
    registered_path("log").unwrap_or_else(|| steam_root().join("logs"))
}

pub fn vrsettings_path() -> PathBuf {
    if std::env::var_os("CV1TOUCH_STEAM_ROOT").is_some() {
        return steam_root().join("config/steamvr.vrsettings");
    }
    registered_path("config")
        .unwrap_or_else(|| steam_root().join("config"))
        .join("steamvr.vrsettings")
}

fn vrpathreg_rel() -> &'static str {
    if cfg!(windows) {
        "bin/win64/vrpathreg.exe"
    } else {
        "bin/vrpathreg.sh"
    }
}

/// SteamVR's driver registration tool.
pub fn vrpathreg() -> PathBuf {
    runtime_root().join(vrpathreg_rel())
}

/// `runtime` is a SteamVR installation when it is a directory that holds vrpathreg.
/// Either missing means SteamVR has not been found there: it may not be installed, or
/// it may be in another library and not started yet, so the path registry has no
/// runtime and the usual folder is empty.
pub fn steamvr_present(runtime: &Path) -> bool {
    runtime.is_dir() && runtime.join(vrpathreg_rel()).is_file()
}

/// What to say when [`steamvr_present`] is false. Names both causes, then `reinstall`.
pub fn steamvr_not_found(runtime: &Path, reinstall: &str) -> String {
    format!(
        "SteamVR was not found at {}: install it from Steam; if it is installed, start it once so it records where it is installed, or set CV1TOUCH_STEAMVR to its folder; then {reinstall}",
        runtime.display()
    )
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
    let so = dir.join(DRIVER_LIBRARY.1);
    let manifest = dir.join(DRIVER_MANIFEST);
    if so.is_file() && manifest.is_file() {
        return Ok((so, manifest));
    }
    if explicit.is_some() {
        return Err(format!(
            "{}: needs {} and {DRIVER_MANIFEST}",
            dir.display(),
            DRIVER_LIBRARY.1
        ));
    }
    let native = PathBuf::from(
        option_env!("CV1TOUCH_NATIVE_DIR")
            .ok_or("this build has no native driver; build with the engine feature")?,
    );
    let so = native.join("driver").join(DRIVER_LIBRARY.1);
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("driver/cv1touch")
        .join(DRIVER_MANIFEST);
    if so.is_file() && manifest.is_file() {
        Ok((so, manifest))
    } else {
        Err(format!("driver assets missing: install the complete runtime or set CV1TOUCH_ASSETS (checked {})", dir.display()))
    }
}

pub fn driver_dir() -> PathBuf {
    crate::platform::dirs::get().data.join("steamvr/cv1touch")
}

const OWNERSHIP_FILE: &str = ".cv1touch-owned.json";
/// The directory SteamVR loads this platform's driver library from, and the library.
#[cfg(unix)]
const DRIVER_LIBRARY: (&str, &str) = ("bin/linux64", "driver_cv1touch.so");
#[cfg(windows)]
const DRIVER_LIBRARY: (&str, &str) = ("bin/win64", "driver_cv1touch.dll");
const DRIVER_MANIFEST: &str = "driver.vrdrivermanifest";
/// The driver's name in its manifest. SteamVR resolves `{cv1touch}` to its `resources`.
const DRIVER_NAME: &str = "cv1touch";
/// The driver's input resources, which SteamVR names `{cv1touch}/input/...`.
const DRIVER_INPUT: &str = "resources/input";
/// The driver's names for its controller type and inputs, which SteamVR reads from
/// `{cv1touch}/localization/localization.json`.
const DRIVER_LOCALIZATION: &str = "resources/localization/localization.json";
/// The directories of the package's resources, which an ownership record of version 2 may own.
const DRIVER_RESOURCE_DIRS: [&str; 2] = [DRIVER_INPUT, "resources/localization"];

/// The controllers' SteamVR controller type, `CV1T_CONTROLLER_TYPE` in common/cv1touch_steamvr.h,
/// which the driver sets. SteamVR keeps one input profile per controller type, the first it loads,
/// and for `oculus_touch` that is its own, which has no right Oculus button: the button needs a
/// controller type of its own. SteamVR saves the bindings a user makes per controller type, so this
/// must never change.
pub const CONTROLLER_TYPE: &str =
    match std::ffi::CStr::from_bytes_with_nul(crate::ffi::CV1T_CONTROLLER_TYPE) {
        Ok(name) => match name.to_str() {
            Ok(name) => name,
            Err(_) => panic!("CV1T_CONTROLLER_TYPE is not UTF-8"),
        },
        Err(_) => panic!("CV1T_CONTROLLER_TYPE is not one C string"),
    };
/// SteamVR's Oculus Touch controller type, which games ship bindings for. A game with no binding
/// of its own for [`CONTROLLER_TYPE`] uses its binding for this one.
const OCULUS_TOUCH: &str = "oculus_touch";
/// What SteamVR's binding pages call [`CONTROLLER_TYPE`].
const CONTROLLER_TYPE_NAME: &str = "Oculus Touch (cv1touch)";
/// What they call the right Oculus button's source.
pub const HOME_NAME: &str = "Oculus Button";

/// SteamVR's Oculus Touch input profile, which the driver's own is derived from.
const TOUCH_PROFILE: &str = "drivers/oculus/resources/input/touch_profile.json";
/// The derived profile, `{cv1touch}/input/touch_profile.json` to SteamVR.
const DRIVER_PROFILE: &str = "resources/input/touch_profile.json";
/// The right Touch's Oculus button, which SteamVR's profile leaves out: `/input/system` is
/// SteamVR's dashboard button, which applications never see. "home" is the name SteamVR's CV1
/// render model gives this button (`home_button`) and an OpenXR standard identifier. Its place
/// on the binding picture is the left Menu button's, read from `/input/system` in the profile
/// SteamVR has installed, because the right controller's picture is the left one's mirror. When
/// that point is missing, [72, 39] is used: that is the Menu button's point in SteamVR's own
/// profile. No touch: the button has no touch sensor.
const HOME_SOURCE: &str = "/input/home";
/// The Menu button's image point in SteamVR's Oculus Touch profile, used when a profile has none.
const HOME_IMAGE_POINT: [i64; 2] = [72, 39];

fn home_entry(point: [i64; 2]) -> String {
    format!(
        r#"
    "/input/home" : {{
        "type" : "button",
        "side" : "right",
        "binding_image_point" : [ {x}, {y} ],
        "click" : true,
        "order" : 0
    }}"#,
        x = point[0],
        y = point[1],
    )
}

/// `/input/system`'s image point, or [`HOME_IMAGE_POINT`] when it is missing or not two integers.
fn menu_image_point(system: Option<&serde_json::Value>) -> [i64; 2] {
    let Some(coords) = system
        .and_then(|entry| entry.get("binding_image_point"))
        .and_then(|point| point.as_array())
        .filter(|coords| coords.len() == 2)
    else {
        return HOME_IMAGE_POINT;
    };
    let mut point = HOME_IMAGE_POINT;
    for (slot, value) in point.iter_mut().zip(coords) {
        let Some(n) = value.as_i64() else {
            return HOME_IMAGE_POINT;
        };
        *slot = n;
    }
    point
}

/// The driver library's path in the driver directory, as the ownership record names it.
fn driver_library() -> String {
    format!("{}/{}", DRIVER_LIBRARY.0, DRIVER_LIBRARY.1)
}

/// Version 1 owns the library and the manifest; version 2 also the files under
/// [`DRIVER_RESOURCE_DIRS`].
#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct DriverOwnership {
    format: String,
    version: u32,
    files: std::collections::BTreeMap<String, String>,
}

fn digest(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    format!("{:x}", Sha256::digest(bytes))
}

fn fingerprint(path: &Path) -> Result<String, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(digest(&bytes))
}

/// A path of plain names, relative: one that cannot leave the directory it is joined to.
fn plain_relative(name: &str) -> bool {
    let path = Path::new(name);
    !name.is_empty()
        && path
            .components()
            .all(|c| matches!(c, std::path::Component::Normal(_)))
}

/// Files the driver package holds beside its library and manifest: path in the package, contents.
pub type DriverResources = Vec<(String, Vec<u8>)>;

fn ownership(
    so: &Path,
    manifest: &Path,
    resources: &[(String, Vec<u8>)],
) -> Result<DriverOwnership, String> {
    let mut files: std::collections::BTreeMap<_, _> = [
        (driver_library(), fingerprint(so)?),
        (DRIVER_MANIFEST.into(), fingerprint(manifest)?),
    ]
    .into();
    files.extend(
        resources
            .iter()
            .map(|(name, bytes)| (name.clone(), digest(bytes))),
    );
    Ok(DriverOwnership {
        format: "cv1touch-driver-ownership".into(),
        version: 2,
        files,
    })
}

/// A previously installed package's files, before ownership records (version 1's two).
fn legacy_ownership(so: &Path, manifest: &Path) -> Result<DriverOwnership, String> {
    Ok(DriverOwnership {
        version: 1,
        ..ownership(so, manifest, &[])?
    })
}

fn supported(record: &DriverOwnership) -> bool {
    let owned = |name: &str| {
        name == driver_library()
            || name == DRIVER_MANIFEST
            || record.version == 2
                && DRIVER_RESOURCE_DIRS.iter().any(|dir| {
                    name.strip_prefix(dir)
                        .and_then(|rest| rest.strip_prefix('/'))
                        .is_some_and(plain_relative)
                })
    };
    record.format == "cv1touch-driver-ownership"
        && matches!(record.version, 1 | 2)
        && record.files.contains_key(&driver_library())
        && record.files.contains_key(DRIVER_MANIFEST)
        && record.files.keys().all(|name| owned(name))
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
        if !supported(&record) {
            return Err(format!(
                "{}: unsupported ownership metadata; retained",
                record_path.display()
            ));
        }
        record
    } else {
        let (so, manifest) =
            legacy.ok_or_else(|| format!("{}: unmanaged driver; retained", dir.display()))?;
        legacy_ownership(so, manifest)?
    };
    // Exactly the owned files and the directories holding them, nothing else.
    let mut files: Vec<PathBuf> = record.files.keys().map(PathBuf::from).collect();
    files.push(OWNERSHIP_FILE.into());
    let directories: Vec<&Path> = files
        .iter()
        .flat_map(|file| file.ancestors().skip(1))
        .filter(|d| !d.as_os_str().is_empty())
        .collect();
    fn check_tree(
        root: &Path,
        dir: &Path,
        files: &[PathBuf],
        directories: &[&Path],
    ) -> Result<(), String> {
        for entry in std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))? {
            let entry = entry.map_err(|e| e.to_string())?;
            let path = entry.path();
            let relative = path.strip_prefix(root).map_err(|e| e.to_string())?;
            let kind = entry.file_type().map_err(|e| e.to_string())?;
            let directory = directories.contains(&relative);
            let file = files.iter().any(|p| relative == p);
            if kind.is_symlink() || !(directory && kind.is_dir() || file && kind.is_file()) {
                return Err(format!(
                    "{}: unknown content or symlink retained",
                    path.display()
                ));
            }
            if directory {
                check_tree(root, &path, files, directories)?;
            }
        }
        Ok(())
    }
    check_tree(dir, dir, &files, &directories)?;
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
    crate::platform::fs::sync_dir(path).map_err(|e| format!("{}: {e}", path.display()))
}

/// Linux-only atomic directory publication keeps ownership metadata and both payloads together.
/// Unsupported rename operations leave the existing package untouched and report the stage.
fn publish_directory(
    stage: &Path,
    dir: &Path,
    exists: bool,
    legacy: Option<(&Path, &Path)>,
) -> Result<(), String> {
    crate::platform::fs::swap_dir(stage, dir, exists).map_err(|e| {
        format!(
            "could not atomically publish {}: {e}; existing package retained, staged files at {}",
            dir.display(),
            stage.display()
        )
    })?;
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

/// Puts the library, the manifest and `resources` (paths under [`DRIVER_RESOURCE_DIRS`]) in `dir`.
pub fn assemble_with_legacy(
    built_so: &Path,
    manifest: &Path,
    resources: &[(String, Vec<u8>)],
    dir: &Path,
    legacy: Option<(&Path, &Path)>,
) -> Result<(), String> {
    let record = ownership(built_so, manifest, resources)?;
    if !supported(&record) {
        return Err(
            "driver resources must be files under resources/input or resources/localization".into(),
        );
    }
    check_driver(dir, legacy)?;
    let stage = staging_dir(dir)?;
    let prepare = || -> Result<(), String> {
        std::fs::create_dir_all(stage.join(DRIVER_LIBRARY.0)).map_err(|e| e.to_string())?;
        for (source, name) in [
            (built_so, driver_library()),
            (manifest, DRIVER_MANIFEST.into()),
        ] {
            let destination = stage.join(name);
            std::fs::copy(source, &destination)
                .map_err(|e| format!("{}: {e}", source.display()))?;
            crate::platform::fs::sync_file(&destination).map_err(|e| e.to_string())?;
        }
        let mut directories = std::collections::BTreeSet::new();
        for (name, bytes) in resources {
            let destination = stage.join(name);
            let parent = destination.parent().ok_or("resource without a directory")?;
            std::fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
            std::fs::write(&destination, bytes)
                .map_err(|e| format!("{}: {e}", destination.display()))?;
            crate::platform::fs::sync_file(&destination).map_err(|e| e.to_string())?;
            directories.extend(
                Path::new(name)
                    .ancestors()
                    .skip(1)
                    .filter(|d| !d.as_os_str().is_empty())
                    .map(|d| stage.join(d)),
            );
        }
        let metadata = stage.join(OWNERSHIP_FILE);
        std::fs::write(
            &metadata,
            serde_json::to_vec_pretty(&record).map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        crate::platform::fs::sync_file(&metadata).map_err(|e| e.to_string())?;
        // deepest first, so that each directory's entry is durable in its parent
        for d in directories.iter().rev() {
            sync_directory(d)?;
        }
        sync_directory(&stage.join(DRIVER_LIBRARY.0))?;
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
    assemble_with_legacy(built_so, manifest, &[], dir, Some((built_so, manifest)))
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

/// How to put the driver package's input profile right, once SteamVR has what it needs.
#[cfg(windows)]
pub const REINSTALL: &str = "quit SteamVR and choose Finish setting up cv1touch in the Start menu";
#[cfg(not(windows))]
pub const REINSTALL: &str = "quit SteamVR and run `cv1touch steamvr install`";
/// `scripts/install.py` runs `steamvr check` before cv1touch is on PATH, so the command above
/// would run an older install, or none. The installer itself has to be run again.
#[cfg(not(windows))]
const INSTALL_SCRIPT_REINSTALL: &str = "quit SteamVR and run `python3 scripts/install.py` again";

/// SteamVR's Oculus Touch input profile in the SteamVR installation `runtime`.
pub fn touch_profile(runtime: &Path) -> PathBuf {
    runtime.join(TOUCH_PROFILE)
}

/// The driver package's input resources, made from the Oculus Touch profile of the SteamVR
/// installation `runtime` (cv1touch ships none of SteamVR's files): that profile made over into
/// [`CONTROLLER_TYPE`]'s, with the right Oculus button ([`driver_profile`]); the names SteamVR's
/// binding pages show for that type and button; and the default bindings the profile names, which
/// SteamVR looks for beside it, copied as they are. A binding file that is not there is left out:
/// SteamVR cannot load it from its own profile either. The other files it names (legacy bindings,
/// pictures) it names by `{oculus}` paths, which SteamVR resolves to its Oculus driver's resources
/// from anywhere. When SteamVR itself is not at `runtime`, the error says to install it, or, if
/// it is installed, to start it once or set `CV1TOUCH_STEAMVR`. When it is there but the profile
/// is missing, the error says to verify SteamVR's files, then what to run.
pub fn derive_resources(runtime: &Path) -> Result<DriverResources, String> {
    derive_resources_with(runtime, REINSTALL)
}

/// [`derive_resources`] for `scripts/install.py`'s preflight. That check runs before cv1touch is
/// installed, so the failure says to run the installer again rather than `cv1touch steamvr install`.
#[cfg(not(windows))]
pub fn derive_resources_for_install_script(runtime: &Path) -> Result<DriverResources, String> {
    derive_resources_with(runtime, INSTALL_SCRIPT_REINSTALL)
}

fn derive_resources_with(runtime: &Path, reinstall: &str) -> Result<DriverResources, String> {
    let source = touch_profile(runtime);
    let text = match std::fs::read_to_string(&source) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            if !steamvr_present(runtime) {
                return Err(steamvr_not_found(runtime, reinstall));
            }
            return Err(format!(
                "SteamVR's Oculus Touch input profile is missing ({}), and cv1touch's profile is made from it: install SteamVR, or have Steam verify SteamVR's files (its Properties, Installed Files); then {reinstall}",
                source.display()
            ));
        }
        Err(e) => return Err(format!("{}: {e}", source.display())),
    };
    let (profile, bindings) =
        driver_profile(&text).map_err(|e| format!("{}: {e}", source.display()))?;
    let mut resources = vec![
        (DRIVER_PROFILE.to_string(), profile.into_bytes()),
        (
            DRIVER_LOCALIZATION.to_string(),
            localization()?.into_bytes(),
        ),
    ];
    let dir = source.parent().unwrap_or(runtime);
    for name in bindings {
        let path = dir.join(&name);
        match std::fs::read(&path) {
            Ok(bytes) => resources.push((format!("{DRIVER_INPUT}/{name}"), bytes)),
            // SteamVR cannot load a binding that is not beside its own profile either.
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => {
                return Err(format!(
                    "{} (a default binding SteamVR's Oculus Touch profile names): {e}",
                    path.display()
                ))
            }
        }
    }
    Ok(resources)
}

/// The names SteamVR's binding pages show for the controller type and the right Oculus button,
/// which they look up under the profile's `resource_root`. Its other inputs keep the names
/// SteamVR gives them; only the thumb rest's came from the Oculus driver's names.
fn localization() -> Result<String, String> {
    let names = serde_json::json!([{
        "language_tag": "en_US",
        CONTROLLER_TYPE: CONTROLLER_TYPE_NAME,
        HOME_SOURCE: HOME_NAME,
        "/input/thumbrest": "Thumbrest",
    }]);
    Ok(serde_json::to_string_pretty(&names).map_err(|e| e.to_string())? + "\n")
}

/// Default binding files named beside the profile, not `{driver}` paths or absolute paths.
fn binding_names(profile: &serde_json::Value) -> Result<Vec<String>, String> {
    let mut bindings = Vec::new();
    for binding in profile["default_bindings"].as_array().into_iter().flatten() {
        let name = binding["binding_url"]
            .as_str()
            .ok_or("a default binding has no binding_url")?;
        // SteamVR resolves these the same wherever the profile is
        if name.starts_with('{') || Path::new(name).is_absolute() {
            continue;
        }
        if !plain_relative(name) {
            return Err(format!(
                "its default binding {name} is not beside it, which this cv1touch does not expect: update cv1touch"
            ));
        }
        if !bindings.iter().any(|b| b == name) {
            bindings.push(name.to_string());
        }
    }
    Ok(bindings)
}

/// The driver publishes `/input/home/click` for the right hand and no other home component.
/// A touch beside that click is still this button: SteamVR's own `/input/system` declares
/// one, and the driver publishes only the click.
fn home_click_drivable(entry: &serde_json::Value) -> bool {
    entry.get("type").and_then(|v| v.as_str()) == Some("button")
        && entry.get("side").and_then(|v| v.as_str()) == Some("right")
        && entry.get("click").and_then(|v| v.as_bool()) == Some(true)
}

/// A member of a JSON object, by where its key's and its value's text are.
struct Member {
    key: String,
    key_text: std::ops::Range<usize>,
    value: std::ops::Range<usize>,
}

/// The members of the JSON object that is `text`, in their order, located in the text itself.
fn members(text: &str) -> Result<Vec<Member>, String> {
    use serde::Deserialize;
    let bytes = text.as_bytes();
    let skip = |mut at: usize| {
        while bytes.get(at).is_some_and(|b| b" \t\r\n".contains(b)) {
            at += 1;
        }
        at
    };
    // the text of the one JSON value starting at `at`
    let value_text = |at: usize| -> Result<std::ops::Range<usize>, String> {
        let mut json = serde_json::Deserializer::from_str(&text[at..]);
        let raw = <&serde_json::value::RawValue>::deserialize(&mut json)
            .map_err(|e| format!("not an input profile: {e}"))?;
        Ok(at..at + raw.get().len())
    };
    let mut at = skip(0);
    if bytes.get(at) != Some(&b'{') {
        return Err("not an input profile: not a JSON object".into());
    }
    at = skip(at + 1);
    let mut members = Vec::new();
    if bytes.get(at) == Some(&b'}') {
        return Ok(members);
    }
    loop {
        let key_text = value_text(at)?;
        let key = serde_json::from_str(&text[key_text.clone()])
            .map_err(|e| format!("not an input profile: {e}"))?;
        at = skip(key_text.end);
        if bytes.get(at) != Some(&b':') {
            return Err("not an input profile: a key without a value".into());
        }
        let value = value_text(skip(at + 1))?;
        at = skip(value.end);
        members.push(Member {
            key,
            key_text,
            value,
        });
        match bytes.get(at) {
            Some(b',') => at = skip(at + 1),
            Some(b'}') => return Ok(members),
            _ => return Err("not an input profile: a member is not followed by , or }".into()),
        }
    }
}

/// The driver's input profile made from SteamVR's Oculus Touch `profile`, and the default binding
/// files it names beside itself. Every byte of `profile` is kept, except that:
/// - `controller_type` is [`CONTROLLER_TYPE`] instead of `oculus_touch`, and
///   `compatibility_mode_controller_type` is `oculus_touch`, so a game with no binding of its own
///   for the controllers uses its Oculus Touch binding;
/// - `remapping` is left out, because SteamVR ignores `compatibility_mode_controller_type` in a
///   profile that has one. That file converts bindings made for other controllers, for games
///   with none for Oculus Touch;
/// - `resource_root` and `driver_name` are this driver's, so that the binding pages name the
///   controller type and the button from its localization;
/// - the right Oculus button's source is first in `input_source`, at the Menu button's image point,
///   unless the profile has it as a right-hand button with click already. Any other `/input/home`
///   is refused.
///
/// Members it adds go after `controller_type`, on lines of their own indented as it is, and every
/// added line has the profile's line endings (SteamVR's are CRLF).
fn driver_profile(profile: &str) -> Result<(String, Vec<String>), String> {
    let original: serde_json::Value =
        serde_json::from_str(profile).map_err(|e| format!("not an input profile: {e}"))?;
    let sources = original["input_source"]
        .as_object()
        .ok_or("not an input profile: input_source is not an object")?;
    if original["controller_type"] != OCULUS_TOUCH {
        return Err(format!(
            "its controller_type is {}, not {OCULUS_TOUCH}, which this cv1touch does not expect: update cv1touch",
            original["controller_type"]
        ));
    }
    let add_home = match sources.get(HOME_SOURCE) {
        None => true,
        Some(entry) if home_click_drivable(entry) => false,
        Some(_) => {
            return Err(format!(
                "it already has {HOME_SOURCE}, but not as a right-hand button with click, which is all this driver's {HOME_SOURCE}/click can drive: update cv1touch"
            ))
        }
    };
    let members = members(profile)?;
    let find = |key: &str| members.iter().position(|m| m.key == key);
    let newline = if profile.contains("\r\n") {
        "\r\n"
    } else {
        "\n"
    };
    let quoted = |s: &str| serde_json::Value::from(s).to_string();
    // (the text replaced, its replacement); none overlap
    let mut edits = Vec::new();

    let controller = &members[find("controller_type").ok_or("it has no controller_type")?];
    let line = &profile[..controller.key_text.start];
    let indent = &line[line.rfind('\n').map_or(0, |n| n + 1)..];
    let lead = if indent.trim().is_empty() {
        format!("{newline}{indent}")
    } else {
        String::new()
    };
    // between the key and its value, as `controller_type` has it
    let colon = &profile[controller.key_text.end..controller.value.start];
    let mut replacement = quoted(CONTROLLER_TYPE);
    for (key, value) in [
        ("compatibility_mode_controller_type", OCULUS_TOUCH),
        ("resource_root", DRIVER_NAME),
        ("driver_name", DRIVER_NAME),
    ] {
        match find(key) {
            Some(i) => edits.push((members[i].value.clone(), quoted(value))),
            None => replacement += &format!(",{lead}{}{colon}{}", quoted(key), quoted(value)),
        }
    }
    edits.push((controller.value.clone(), replacement));

    if let Some(i) = find("remapping") {
        // with the comma after it, or the one before it when it is the last member
        let text = match (members.get(i + 1), i.checked_sub(1)) {
            (Some(next), _) => members[i].key_text.start..next.key_text.start,
            (None, Some(before)) => members[before].value.end..members[i].value.end,
            (None, None) => members[i].key_text.start..members[i].value.end,
        };
        edits.push((text, String::new()));
    }

    let mut expected_home = None;
    if add_home {
        let at = members[find("input_source").ok_or("it has no input_source")?]
            .value
            .start
            + 1;
        let separator = if sources.is_empty() { "" } else { "," };
        let entry = home_entry(menu_image_point(sources.get("/input/system")));
        let parsed: serde_json::Value = serde_json::from_str(&format!("{{{entry}}}"))
            .map_err(|e| format!("could not add the right Oculus button: {e}"))?;
        expected_home = Some(parsed[HOME_SOURCE].clone());
        edits.push((at..at, entry.replace('\n', newline) + separator));
    }

    // last first, so that each edit's place is still where it was found
    edits.sort_by_key(|(text, _)| std::cmp::Reverse((text.start, text.end)));
    let mut derived = profile.to_string();
    for (text, replacement) in edits {
        derived.replace_range(text, &replacement);
    }

    // what the edits must have made, compared value by value
    let mut expected = original.clone();
    let object = expected.as_object_mut().ok_or("not an input profile")?;
    object.insert("controller_type".into(), CONTROLLER_TYPE.into());
    object.insert(
        "compatibility_mode_controller_type".into(),
        OCULUS_TOUCH.into(),
    );
    object.insert("resource_root".into(), DRIVER_NAME.into());
    object.insert("driver_name".into(), DRIVER_NAME.into());
    object.remove("remapping");
    let sources = object
        .get_mut("input_source")
        .and_then(|s| s.as_object_mut());
    if let (Some(home), Some(sources)) = (expected_home, sources) {
        sources.insert(HOME_SOURCE.into(), home);
    }
    let made: Result<serde_json::Value, _> = serde_json::from_str(&derived);
    if made.ok().as_ref() != Some(&expected) {
        return Err("could not make cv1touch's input profile from it".into());
    }
    Ok((derived, binding_names(&original)?))
}

/// Digests the ownership record names, when `dir` has a supported one.
///
/// A missing or unreadable record is not a hand edit: the byte comparison still applies.
fn recorded_digests(dir: &Path) -> Option<std::collections::BTreeMap<String, String>> {
    let bytes = std::fs::read(dir.join(OWNERSHIP_FILE)).ok()?;
    let record: DriverOwnership = serde_json::from_slice(&bytes).ok()?;
    supported(&record).then_some(record.files)
}

/// Whether the driver package in `dir` holds the input resources made from the Oculus Touch
/// profile SteamVR has now (the derived profile's path), and what to do if not.
///
/// A file the ownership record names, whose digest is not the one recorded, was changed after
/// cv1touch installed it. Check all recorded input resources, including bindings SteamVR no longer
/// names, before any byte comparison: install will not replace a changed file, so the out-of-date
/// advice would send the user to a command that refuses.
pub fn check_input_profile(dir: &Path, runtime: &Path) -> Result<PathBuf, String> {
    let resources = derive_resources(runtime)?;
    if let Some(recorded) = recorded_digests(dir) {
        for (name, expected) in recorded {
            if !DRIVER_RESOURCE_DIRS
                .iter()
                .any(|resource| Path::new(&name).starts_with(resource))
            {
                continue;
            }
            let path = dir.join(name);
            match std::fs::read(&path) {
                Ok(installed) if digest(&installed) != expected => {
                    return Err(format!(
                        "{} was changed after cv1touch installed it; back up local changes and remove the driver package {} before retrying",
                        path.display(),
                        dir.display()
                    ));
                }
                Ok(_) => {}
                // Leave missing-file diagnostics to the freshness comparison below.
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                Err(e) => return Err(format!("{}: {e}", path.display())),
            }
        }
    }
    for (name, bytes) in resources {
        let path = dir.join(&name);
        // Freshness compares the profile first: a later file is stale, or missing, only when
        // the profile is as it should be. The names are cv1touch's alone. A copied binding is
        // SteamVR's own file, byte for byte, which goes out of date only when SteamVR changes it,
        // and is missing only when SteamVR has it now and this package, installed without it,
        // does not.
        let (missing, stale) = if name == DRIVER_PROFILE {
            (
                "the SteamVR driver is not installed, or an older cv1touch installed it",
                "SteamVR's Oculus Touch profile, or cv1touch, changed since the SteamVR driver was installed",
            )
        } else if name == DRIVER_LOCALIZATION {
            (
                "an older cv1touch installed the SteamVR driver",
                "cv1touch changed since the SteamVR driver was installed",
            )
        } else {
            (
                "SteamVR's Oculus Touch profile names it and the installed driver does not have it",
                "SteamVR's dashboard bindings for Oculus Touch changed since the SteamVR driver was installed",
            )
        };
        match std::fs::read(&path) {
            Ok(installed) => {
                if installed != bytes {
                    return Err(format!(
                        "{} is out of date: {stale}; {REINSTALL}",
                        path.display()
                    ));
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                return Err(format!(
                    "{} is missing: {missing}; {REINSTALL}",
                    path.display()
                ))
            }
            Err(e) => return Err(format!("{}: {e}", path.display())),
        }
    }
    Ok(dir.join(DRIVER_PROFILE))
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
        assert_eq!(std::fs::read(out.join(driver_library())).unwrap(), b"ELF");
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
        for name in [driver_library(), DRIVER_MANIFEST.into()] {
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
    #[cfg(unix)]
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
                std::fs::remove_file(package.join(driver_library())).unwrap();
                symlink(&so, package.join(driver_library())).unwrap();
            } else {
                std::fs::write(package.join("personal"), b"keep").unwrap();
            }
            std::fs::write(&so, b"new").unwrap();
            assert!(assemble(&so, &manifest, &package).is_err());
            assert!(remove_owned_driver(&package).is_err());
            if !symbolic {
                assert_eq!(
                    std::fs::read(package.join(driver_library())).unwrap(),
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
        std::fs::create_dir_all(package.join(DRIVER_LIBRARY.0)).unwrap();
        std::fs::copy(&so, package.join(driver_library())).unwrap();
        std::fs::copy(&manifest, package.join(DRIVER_MANIFEST)).unwrap();
        assert!(check_driver(&package, None).is_err());
        check_driver(&package, Some((&so, &manifest))).unwrap();
        std::fs::write(package.join(driver_library()), b"changed").unwrap();
        assert!(check_driver(&package, Some((&so, &manifest))).is_err());
        std::fs::write(package.join(driver_library()), b"old").unwrap();
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
        std::fs::create_dir_all(package.join(DRIVER_LIBRARY.0)).unwrap();
        std::fs::copy(&old, package.join(driver_library())).unwrap();
        std::fs::copy(&manifest, package.join(DRIVER_MANIFEST)).unwrap();
        assert!(assemble(&new, &manifest, &package).is_err());
        assemble_with_legacy(&new, &manifest, &[], &package, Some((&old, &manifest))).unwrap();
        assert_eq!(
            std::fs::read(package.join(driver_library())).unwrap(),
            b"new"
        );
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

    /// A made-up input profile laid out as SteamVR's are (not SteamVR's own), spaced unevenly so
    /// that whether its text is kept shows.
    const FIXTURE_PROFILE: &str = r#"{
  "jsonid" : "input_profile",
  "controller_type": "oculus_touch",
  "resource_root" : "example",
  "driver_name" : "example",
  "legacy_binding" : "{example}/input/example_legacy.json",
  "remapping" : "{example}/input/example_remapping.json",
  "input_bindingui_right" : { "image": "{example}/icons/example_right.svg" },
  "input_source" :
  {
	"/input/trigger" : { "type" : "trigger", "binding_image_point" : [ 1, 2 ], "value" : true },
    "/input/system" : {
        "type" : "button", "side" : "left",
        "binding_image_point" : [ 3, 4 ], "click" : true
    }
  },
  "default_bindings": [
    { "app_key": "example.app", "binding_url": "example_bindings.json" },
    { "app_key": "example.other", "binding_url": "{example}/input/elsewhere.json" }
  ]
}
"#;

    /// The edits [`driver_profile`] makes outside `input_source` to a profile laid out as
    /// [`FIXTURE_PROFILE`] is, written out by hand.
    fn top_level_edits(profile: &str) -> String {
        profile
            .replacen(
                r#""controller_type": "oculus_touch","#,
                "\"controller_type\": \"cv1touch_touch\",\n  \"compatibility_mode_controller_type\": \"oculus_touch\",",
                1,
            )
            .replacen(
                r#""resource_root" : "example""#,
                r#""resource_root" : "cv1touch""#,
                1,
            )
            .replacen(
                r#""driver_name" : "example""#,
                r#""driver_name" : "cv1touch""#,
                1,
            )
            .replacen(
                "  \"remapping\" : \"{example}/input/example_remapping.json\",\n",
                "",
                1,
            )
    }

    /// [`FIXTURE_PROFILE`] as the driver's profile: every edit, and nothing else.
    fn fixture_driver_profile() -> String {
        top_level_edits(FIXTURE_PROFILE).replacen(
            "\"input_source\" :\n  {",
            &format!("\"input_source\" :\n  {{{},", home_entry([3, 4])),
            1,
        )
    }

    /// A profile of SteamVR's Oculus Touch controller type with `rest` beside its controller_type.
    fn touch(rest: &str) -> String {
        format!(r#"{{"controller_type": "oculus_touch", {rest}}}"#)
    }

    /// A SteamVR directory that has vrpathreg and none of the Oculus input files.
    fn installed_runtime(d: &Path) -> PathBuf {
        let runtime = d.join("runtime");
        std::fs::create_dir_all(runtime.join(if cfg!(windows) { "bin/win64" } else { "bin" }))
            .unwrap();
        std::fs::write(runtime.join(vrpathreg_rel()), b"#!/bin/sh\n").unwrap();
        runtime
    }

    /// A SteamVR installation in `d` holding only the fixture profile and the binding it names.
    fn fixture_runtime(d: &Path) -> PathBuf {
        let runtime = installed_runtime(d);
        let profile = touch_profile(&runtime);
        std::fs::create_dir_all(profile.parent().unwrap()).unwrap();
        std::fs::write(&profile, FIXTURE_PROFILE).unwrap();
        std::fs::write(
            profile.with_file_name("example_bindings.json"),
            b"example bindings",
        )
        .unwrap();
        runtime
    }

    fn fixture_payload(d: &Path) -> (PathBuf, PathBuf) {
        std::fs::write(d.join("lib.so"), b"ELF").unwrap();
        std::fs::write(d.join("manifest"), br#"{"name":"cv1touch"}"#).unwrap();
        (d.join("lib.so"), d.join("manifest"))
    }

    #[test]
    fn the_profile_becomes_the_controllers_own_and_keeps_every_other_byte() {
        let d = tmp("derive");
        let resources = derive_resources(&fixture_runtime(&d)).unwrap();
        let names: Vec<&str> = resources.iter().map(|(name, _)| name.as_str()).collect();
        // a binding named by an {driver} path is found from anywhere: nothing to copy
        assert_eq!(
            names,
            [
                DRIVER_PROFILE,
                DRIVER_LOCALIZATION,
                "resources/input/example_bindings.json"
            ]
        );
        let text = std::str::from_utf8(&resources[0].1).unwrap();
        assert_eq!(text, fixture_driver_profile());
        let profile: serde_json::Value = serde_json::from_str(text).unwrap();
        // the controllers' own type, which uses games' Oculus Touch bindings
        assert_eq!(profile["controller_type"], CONTROLLER_TYPE);
        assert_eq!(
            profile["compatibility_mode_controller_type"],
            "oculus_touch"
        );
        // SteamVR ignores compatibility mode in a profile with a remapping
        assert!(profile.get("remapping").is_none(), "{text}");
        assert_eq!(profile["resource_root"], DRIVER_NAME);
        assert_eq!(profile["driver_name"], DRIVER_NAME);
        assert_eq!(
            profile["input_source"][HOME_SOURCE],
            serde_json::json!({"type": "button", "side": "right", "binding_image_point": [3, 4], "click": true, "order": 0})
        );
        // legacy bindings and pictures stay SteamVR's
        assert_eq!(
            profile["legacy_binding"],
            "{example}/input/example_legacy.json"
        );
        assert_eq!(resources[2].1, b"example bindings");
        // CRLF lines stay CRLF lines, the added ones too
        let crlf = FIXTURE_PROFILE.replace('\n', "\r\n");
        let (derived, _) = driver_profile(&crlf).unwrap();
        assert_eq!(derived, fixture_driver_profile().replace('\n', "\r\n"));
        assert_eq!(
            derived.matches('\n').count(),
            derived.matches("\r\n").count()
        );
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn the_edits_follow_the_profiles_own_layout() {
        // On one line, a remapping last and no resource_root or driver_name: those are added
        // after controller_type, and the remapping goes with the comma before it.
        let minified =
            r#"{"controller_type":"oculus_touch","input_source":{},"remapping":"r.json"}"#;
        assert_eq!(
            driver_profile(minified).unwrap().0,
            format!(
                r#"{{"controller_type":"cv1touch_touch","compatibility_mode_controller_type":"oculus_touch","resource_root":"cv1touch","driver_name":"cv1touch","input_source":{{{}}}}}"#,
                home_entry(HOME_IMAGE_POINT)
            )
        );
        // a remapping right after controller_type, first of the rest, a compatibility mode
        // already there (its value replaced) and tab indents
        let tabs = "{\n\t\"controller_type\" : \"oculus_touch\",\n\t\"remapping\" : \"r.json\",\n\t\"compatibility_mode_controller_type\" : \"x\",\n\t\"input_source\" : { \"/input/a\" : {} }\n}";
        assert_eq!(
            driver_profile(tabs).unwrap().0,
            format!(
                "{{\n\t\"controller_type\" : \"cv1touch_touch\",\n\t\"resource_root\" : \"cv1touch\",\n\t\"driver_name\" : \"cv1touch\",\n\t\"compatibility_mode_controller_type\" : \"oculus_touch\",\n\t\"input_source\" : {{{}, \"/input/a\" : {{}} }}\n}}",
                home_entry(HOME_IMAGE_POINT)
            )
        );
        // with no remapping, resource_root or driver_name, nothing goes and those are added
        let (derived, _) = driver_profile(&touch(r#""input_source": {}"#)).unwrap();
        let derived: serde_json::Value = serde_json::from_str(&derived).unwrap();
        assert_eq!(derived.as_object().unwrap().len(), 5, "{derived}");
    }

    #[test]
    fn the_binding_pages_name_the_type_and_the_button() {
        let names: serde_json::Value = serde_json::from_str(&localization().unwrap()).unwrap();
        assert_eq!(
            names,
            serde_json::json!([{
                "language_tag": "en_US",
                "cv1touch_touch": "Oculus Touch (cv1touch)",
                "/input/home": "Oculus Button",
                "/input/thumbrest": "Thumbrest"
            }])
        );
        // SteamVR looks them up under the profile's resource_root, in the localization file
        // of the driver of that name
        let (profile, _) = driver_profile(FIXTURE_PROFILE).unwrap();
        let profile: serde_json::Value = serde_json::from_str(&profile).unwrap();
        assert_eq!(profile["resource_root"], DRIVER_NAME);
        assert_eq!(
            DRIVER_LOCALIZATION,
            "resources/localization/localization.json"
        );
    }

    #[test]
    fn a_missing_steamvr_profile_says_what_to_do() {
        let d = tmp("no-profile");
        let runtime = installed_runtime(&d);
        let e = derive_resources(&runtime).unwrap_err();
        assert!(e.starts_with("SteamVR's Oculus Touch input profile is missing"));
        assert!(e.contains(&touch_profile(&runtime).display().to_string()));
        assert!(e.contains("verify SteamVR's files"), "{e}");
        assert!(!e.contains("was not found"), "{e}");
        assert!(e.ends_with(REINSTALL));
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn steamvr_that_is_not_installed_is_named() {
        let d = tmp("no-steamvr");
        let runtime = d.join("runtime");
        let e = derive_resources(&runtime).unwrap_err();
        assert!(
            e.starts_with(&format!("SteamVR was not found at {}", runtime.display())),
            "{e}"
        );
        assert!(
            e.contains("install it from Steam")
                && e.contains("start it once")
                && e.contains("CV1TOUCH_STEAMVR"),
            "{e}"
        );
        assert!(
            e.ends_with(REINSTALL) && !e.contains("verify SteamVR's files"),
            "{e}"
        );
        // The folder can exist and still not be SteamVR, when vrpathreg is not in it.
        std::fs::create_dir_all(runtime.join("drivers/oculus/resources/input")).unwrap();
        let e = derive_resources(&runtime).unwrap_err();
        assert!(
            e.starts_with("SteamVR was not found at")
                && e.contains("install it from Steam")
                && !e.contains("verify SteamVR's files"),
            "{e}"
        );
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_missing_default_binding_is_left_out() {
        let d = tmp("dangling-binding");
        let runtime = fixture_runtime(&d);
        let binding = touch_profile(&runtime).with_file_name("example_bindings.json");
        std::fs::remove_file(&binding).unwrap();
        let resources = derive_resources(&runtime).unwrap();
        assert_eq!(
            resources
                .iter()
                .map(|(name, _)| name.as_str())
                .collect::<Vec<_>>(),
            [DRIVER_PROFILE, DRIVER_LOCALIZATION]
        );
        let profile = std::str::from_utf8(&resources[0].1).unwrap();
        assert!(profile.contains("example_bindings.json"), "{profile}");
        // The file showing up later is one the installed package does not have.
        let (so, manifest) = fixture_payload(&d);
        let package = d.join("package");
        assemble_with_legacy(&so, &manifest, &resources, &package, None).unwrap();
        check_input_profile(&package, &runtime).unwrap();
        std::fs::write(&binding, b"example bindings").unwrap();
        let e = check_input_profile(&package, &runtime).unwrap_err();
        assert!(
            e.contains("example_bindings.json")
                && e.contains("does not have it")
                && e.ends_with(REINSTALL),
            "{e}"
        );
        assert!(!e.contains("not installed"), "{e}");
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn the_image_point_follows_the_menu_button() {
        let (copied, _) = driver_profile(FIXTURE_PROFILE).unwrap();
        let copied: serde_json::Value = serde_json::from_str(&copied).unwrap();
        assert_eq!(
            copied["input_source"][HOME_SOURCE]["binding_image_point"],
            serde_json::json!([3, 4])
        );
        for sources in [
            r#""input_source": {}"#,
            r#""input_source": {"/input/system": {"type": "button"}}"#,
            r#""input_source": {"/input/system": {"binding_image_point": [1]}}"#,
            r#""input_source": {"/input/system": {"binding_image_point": "no"}}"#,
        ] {
            let (derived, _) = driver_profile(&touch(sources)).unwrap();
            let derived: serde_json::Value = serde_json::from_str(&derived).unwrap();
            assert_eq!(
                derived["input_source"][HOME_SOURCE]["binding_image_point"],
                serde_json::to_value(HOME_IMAGE_POINT).unwrap(),
                "{sources}"
            );
        }
    }

    #[cfg(not(windows))]
    #[test]
    fn the_install_script_is_told_to_run_itself_again() {
        let d = tmp("install-script");
        let e = derive_resources_for_install_script(&d.join("runtime")).unwrap_err();
        assert!(e.starts_with("SteamVR was not found at"), "{e}");
        assert!(
            e.contains("install it from Steam")
                && e.contains("CV1TOUCH_STEAMVR")
                && !e.contains("verify SteamVR's files"),
            "{e}"
        );
        assert!(e.ends_with(INSTALL_SCRIPT_REINSTALL), "{e}");
        assert!(!e.contains("cv1touch steamvr install"), "{e}");
        let plain = derive_resources(&d.join("runtime")).unwrap_err();
        assert!(
            plain.starts_with("SteamVR was not found at")
                && plain.ends_with(REINSTALL)
                && plain.contains("cv1touch steamvr install"),
            "{plain}"
        );
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn profiles_this_cannot_make_the_driver_profile_from_are_refused() {
        let (empty, _) = driver_profile(&touch(r#""input_source": {}"#)).unwrap();
        let empty: serde_json::Value = serde_json::from_str(&empty).unwrap();
        assert_eq!(empty["input_source"].as_object().unwrap().len(), 1);
        for (profile, says) in [
            (
                touch(r#""input_source": {"/input/home": {}}"#),
                "already has",
            ),
            (
                touch(
                    r#""input_source": {"/input/home": {"type": "button", "side": "left", "click": true}}"#,
                ),
                "/input/home/click",
            ),
            (touch(r#""input_source": []"#), "not an object"),
            ("not json".into(), "not an input profile"),
            (
                touch(r#""input_source": {}, "default_bindings": [{"binding_url": "../x.json"}]"#),
                "not beside it",
            ),
            (
                touch(r#""input_source": {}, "default_bindings": [{"app_key": "a"}]"#),
                "no binding_url",
            ),
            // another controller's profile, or none, is not what the driver's is made from
            (
                r#"{"controller_type": "example_touch", "input_source": {}}"#.into(),
                "its controller_type is \"example_touch\", not oculus_touch",
            ),
            (
                r#"{"input_source": {}}"#.into(),
                "its controller_type is null, not oculus_touch",
            ),
        ] {
            let e = driver_profile(&profile).unwrap_err();
            assert!(e.contains(says), "{profile}: {e}");
        }
    }

    #[test]
    fn a_right_home_button_already_in_the_profile_is_kept() {
        let d = tmp("home-kept");
        let runtime = fixture_runtime(&d);
        let path = touch_profile(&runtime);
        let text = FIXTURE_PROFILE.replacen(
            "\"input_source\" :\n  {",
            "\"input_source\" :\n  {\n    \"/input/home\" : { \"type\" : \"button\", \"side\" : \"right\", \"click\" : true },",
            1,
        );
        std::fs::write(&path, &text).unwrap();
        let resources = derive_resources(&runtime).unwrap();
        // the profile is still made the controllers' own; its input sources stay as they are
        assert_eq!(
            std::str::from_utf8(&resources[0].1).unwrap(),
            top_level_edits(&text)
        );
        assert_eq!(
            resources[2],
            (
                "resources/input/example_bindings.json".to_string(),
                b"example bindings".to_vec()
            )
        );
        // SteamVR's own buttons declare touch beside click. The driver publishes only the click.
        let with_touch = touch(
            r#""input_source": {"/input/home": {"type": "button", "side": "right", "click": true, "touch": true}}"#,
        );
        let (derived, _) = driver_profile(&with_touch).unwrap();
        let derived: serde_json::Value = serde_json::from_str(&derived).unwrap();
        let original: serde_json::Value = serde_json::from_str(&with_touch).unwrap();
        assert_eq!(derived["input_source"], original["input_source"]);
        for sources in [
            r#""input_source": {"/input/home": {"type": "button", "click": true}}"#,
            r#""input_source": {"/input/home": {"type": "button", "side": "right", "click": false}}"#,
            r#""input_source": {"/input/home": {"type": "button", "side": "right"}}"#,
            r#""input_source": {"/input/home": {"type": "trigger", "side": "right", "click": true}}"#,
        ] {
            let e = driver_profile(&touch(sources)).unwrap_err();
            assert!(
                e.contains("already has") && e.contains("/input/home/click"),
                "{sources}: {e}"
            );
        }
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn the_package_owns_its_input_resources() {
        let d = tmp("resources");
        let resources = derive_resources(&fixture_runtime(&d)).unwrap();
        let (so, manifest) = fixture_payload(&d);
        let package = d.join("package");
        assemble_with_legacy(&so, &manifest, &resources, &package, None).unwrap();
        for (name, bytes) in &resources {
            assert_eq!(&std::fs::read(package.join(name)).unwrap(), bytes);
        }
        let record: DriverOwnership =
            serde_json::from_slice(&std::fs::read(package.join(OWNERSHIP_FILE)).unwrap()).unwrap();
        assert_eq!((record.version, record.files.len()), (2, 5));
        check_driver(&package, None).unwrap();
        // an edited profile or names, and an unknown file beside them, are kept, and block
        // upgrade and removal
        for (name, edit) in [
            (DRIVER_PROFILE, &b"edited"[..]),
            (DRIVER_LOCALIZATION, b"edited"),
            ("resources/input/personal.json", b"keep"),
            ("resources/localization/personal.json", b"keep"),
        ] {
            let path = package.join(name);
            let before = std::fs::read(&path).ok();
            std::fs::write(&path, edit).unwrap();
            assert!(assemble_with_legacy(&so, &manifest, &resources, &package, None).is_err());
            assert!(remove_owned_driver(&package).is_err());
            assert_eq!(std::fs::read(&path).unwrap(), edit);
            match before {
                Some(bytes) => std::fs::write(&path, bytes).unwrap(),
                None => std::fs::remove_file(&path).unwrap(),
            }
        }
        // removing the driver removes what was made for it
        remove_owned_driver(&package).unwrap();
        assert!(!package.exists());
        // a resource outside resources/input and resources/localization is refused before
        // anything is written
        for name in [
            "bin/extra.json",
            "resources/other/extra.json",
            "resources/input/../../extra.json",
            "resources/localization/../../extra.json",
        ] {
            let extra = [(name.to_string(), b"x".to_vec())];
            assert!(assemble_with_legacy(&so, &manifest, &extra, &package, None).is_err());
            assert!(!package.exists());
        }
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_package_an_earlier_cv1touch_installed_upgrades() {
        let d = tmp("version-1");
        let resources = derive_resources(&fixture_runtime(&d)).unwrap();
        let (so, manifest) = fixture_payload(&d);
        let package = d.join("package");
        std::fs::create_dir_all(package.join(DRIVER_LIBRARY.0)).unwrap();
        std::fs::copy(&so, package.join(driver_library())).unwrap();
        std::fs::copy(&manifest, package.join(DRIVER_MANIFEST)).unwrap();
        let version_1 = |files: &[&str]| {
            let files: std::collections::BTreeMap<_, _> = files
                .iter()
                .map(|name| (name.to_string(), fingerprint(&package.join(name)).unwrap()))
                .collect();
            std::fs::write(
                package.join(OWNERSHIP_FILE),
                serde_json::json!({"format": "cv1touch-driver-ownership", "version": 1, "files": files}).to_string(),
            )
            .unwrap();
        };
        version_1(&[&driver_library(), DRIVER_MANIFEST]);
        check_driver(&package, None).unwrap();
        assemble_with_legacy(&so, &manifest, &resources, &package, None).unwrap();
        check_input_profile(&package, &d.join("runtime")).unwrap();
        // version 1 never owned anything but the library and the manifest
        for owned in [DRIVER_PROFILE, DRIVER_LOCALIZATION] {
            version_1(&[&driver_library(), DRIVER_MANIFEST, owned]);
            assert!(check_driver(&package, None)
                .unwrap_err()
                .contains("unsupported ownership metadata"));
        }
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn the_doctor_check_compares_with_the_profile_steamvr_has_now() {
        let d = tmp("doctor-profile");
        let runtime = fixture_runtime(&d);
        let (so, manifest) = fixture_payload(&d);
        let package = d.join("package");
        let install = || {
            let resources = derive_resources(&runtime).unwrap();
            assemble_with_legacy(&so, &manifest, &resources, &package, None).unwrap();
        };
        let e = check_input_profile(&package, &runtime).unwrap_err();
        assert!(
            e.contains("is missing") && e.contains("not installed") && e.ends_with(REINSTALL),
            "{e}"
        );
        install();
        assert_eq!(
            check_input_profile(&package, &runtime).unwrap(),
            package.join(DRIVER_PROFILE)
        );
        // A hand edit no longer matches the ownership record. Install refuses to replace it, so
        // this must not say the file is out of date or to run install as it stands.
        let changed_after_install = |name: &str| {
            let path = package.join(name);
            let owned = std::fs::read(&path).unwrap();
            std::fs::write(&path, b"hand edited").unwrap();
            let e = check_input_profile(&package, &runtime).unwrap_err();
            let advice = format!(
                "back up local changes and remove the driver package {} before retrying",
                package.display()
            );
            assert!(
                e.contains(&format!(
                    "{} was changed after cv1touch installed it",
                    path.display()
                )) && e.contains(&advice)
                    && !e.contains("out of date")
                    && !e.contains(REINSTALL),
                "{e}"
            );
            std::fs::write(&path, owned).unwrap();
        };
        changed_after_install(DRIVER_PROFILE);
        changed_after_install(DRIVER_LOCALIZATION);
        changed_after_install("resources/input/example_bindings.json");
        check_input_profile(&package, &runtime).unwrap();
        // The digest is checked before the bytes. A file that matches what would be made now,
        // but not what the record stored, was still changed after install.
        let record_path = package.join(OWNERSHIP_FILE);
        let record_bytes = std::fs::read(&record_path).unwrap();
        let mut record: serde_json::Value = serde_json::from_slice(&record_bytes).unwrap();
        record["files"][DRIVER_PROFILE] = serde_json::Value::from("not-the-installed-digest");
        std::fs::write(&record_path, serde_json::to_vec(&record).unwrap()).unwrap();
        let e = check_input_profile(&package, &runtime).unwrap_err();
        assert!(
            e.contains(&format!(
                "{} was changed after cv1touch installed it",
                package.join(DRIVER_PROFILE).display()
            )) && !e.contains("out of date"),
            "{e}"
        );
        std::fs::write(&record_path, record_bytes).unwrap();
        check_input_profile(&package, &runtime).unwrap();
        // SteamVR updated its profile, or a binding it names: ours is out of date until installed again.
        // A copied binding is not the profile, so the cause must not say the profile or cv1touch changed.
        let profile = touch_profile(&runtime);
        for (path, changed, cause, absent) in [
            (
                profile.clone(),
                FIXTURE_PROFILE.replace("[ 1, 2 ]", "[ 5, 6 ]"),
                "SteamVR's Oculus Touch profile, or cv1touch, changed",
                "dashboard bindings",
            ),
            (
                profile.with_file_name("example_bindings.json"),
                "changed bindings".into(),
                "SteamVR's dashboard bindings for Oculus Touch changed",
                "profile, or cv1touch, changed",
            ),
        ] {
            std::fs::write(&path, changed).unwrap();
            let e = check_input_profile(&package, &runtime).unwrap_err();
            assert!(
                e.contains("is out of date") && e.contains(cause) && e.ends_with(REINSTALL),
                "{e}"
            );
            assert!(!e.contains(absent), "{e}");
            install();
            check_input_profile(&package, &runtime).unwrap();
        }
        // Names an older cv1touch installed still match that install's record, and differ from
        // what this one makes. The same bytes with the record left as this install's are a hand
        // edit, covered above, not "cv1touch changed".
        let names = package.join(DRIVER_LOCALIZATION);
        let made = std::fs::read(&names).unwrap();
        let record_path = package.join(OWNERSHIP_FILE);
        let record_bytes = std::fs::read(&record_path).unwrap();
        let mut record: serde_json::Value = serde_json::from_slice(&record_bytes).unwrap();
        record["files"][DRIVER_LOCALIZATION] = serde_json::Value::from(digest(b"[]"));
        std::fs::write(&record_path, serde_json::to_vec(&record).unwrap()).unwrap();
        std::fs::write(&names, b"[]").unwrap();
        let e = check_input_profile(&package, &runtime).unwrap_err();
        assert!(
            e.contains("is out of date: cv1touch changed") && e.ends_with(REINSTALL),
            "{e}"
        );
        std::fs::write(&record_path, &record_bytes).unwrap();
        std::fs::remove_file(&names).unwrap();
        let e = check_input_profile(&package, &runtime).unwrap_err();
        assert!(
            e.contains("is missing: an older cv1touch installed the SteamVR driver")
                && e.ends_with(REINSTALL),
            "{e}"
        );
        assert!(
            !e.contains("SteamVR's Oculus Touch profile names it"),
            "{e}"
        );
        std::fs::write(&names, made).unwrap();
        check_input_profile(&package, &runtime).unwrap();
        std::fs::remove_file(&profile).unwrap();
        assert!(check_input_profile(&package, &runtime)
            .unwrap_err()
            .starts_with("SteamVR's Oculus Touch input profile is missing"));
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn the_doctor_checks_edits_before_a_stale_profile() {
        let d = tmp("doctor-stale-profile-edit");
        let runtime = fixture_runtime(&d);
        let (so, manifest) = fixture_payload(&d);
        let package = d.join("package");
        let resources = derive_resources(&runtime).unwrap();
        assemble_with_legacy(&so, &manifest, &resources, &package, None).unwrap();
        std::fs::write(
            touch_profile(&runtime),
            FIXTURE_PROFILE.replace("[ 1, 2 ]", "[ 5, 6 ]"),
        )
        .unwrap();
        for name in [DRIVER_LOCALIZATION, "resources/input/example_bindings.json"] {
            let path = package.join(name);
            let original = std::fs::read(&path).unwrap();
            std::fs::write(&path, b"hand edited").unwrap();
            assert!(check_driver(&package, None).is_err());
            let e = check_input_profile(&package, &runtime).unwrap_err();
            assert!(
                e.starts_with(&format!(
                    "{} was changed after cv1touch installed it",
                    path.display()
                )) && e.contains("back up local changes and remove the driver package")
                    && !e.contains(REINSTALL),
                "{e}"
            );
            std::fs::write(&path, original).unwrap();
        }
        let e = check_input_profile(&package, &runtime).unwrap_err();
        assert!(
            e.contains("is out of date") && e.ends_with(REINSTALL),
            "{e}"
        );
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn the_doctor_checks_edits_to_a_binding_steamvr_no_longer_names() {
        let d = tmp("doctor-old-binding-edit");
        let runtime = fixture_runtime(&d);
        let (so, manifest) = fixture_payload(&d);
        let package = d.join("package");
        let resources = derive_resources(&runtime).unwrap();
        assemble_with_legacy(&so, &manifest, &resources, &package, None).unwrap();
        let mut profile: serde_json::Value = serde_json::from_str(FIXTURE_PROFILE).unwrap();
        profile["default_bindings"]
            .as_array_mut()
            .unwrap()
            .remove(0);
        std::fs::write(
            touch_profile(&runtime),
            serde_json::to_vec(&profile).unwrap(),
        )
        .unwrap();
        let name = "resources/input/example_bindings.json";
        assert!(derive_resources(&runtime)
            .unwrap()
            .iter()
            .all(|(derived, _)| derived != name));
        let path = package.join(name);
        let original = std::fs::read(&path).unwrap();
        std::fs::write(&path, b"hand edited").unwrap();
        assert!(check_driver(&package, None).is_err());
        let e = check_input_profile(&package, &runtime).unwrap_err();
        assert!(
            e.starts_with(&format!(
                "{} was changed after cv1touch installed it",
                path.display()
            )) && e.contains("back up local changes and remove the driver package")
                && !e.contains(REINSTALL),
            "{e}"
        );
        std::fs::write(&path, original).unwrap();
        let e = check_input_profile(&package, &runtime).unwrap_err();
        assert!(
            e.contains("is out of date") && e.ends_with(REINSTALL),
            "{e}"
        );
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn the_driver_names_the_profile_type_and_source_made_here() {
        let manifest: serde_json::Value =
            serde_json::from_str(include_str!("../driver/cv1touch/driver.vrdrivermanifest"))
                .unwrap();
        // {cv1touch} is the resources directory of the driver of that name
        assert_eq!(manifest["name"], DRIVER_NAME);
        let device = include_str!("../driver/src/touch_device.cpp");
        let profile = DRIVER_PROFILE.strip_prefix("resources/").unwrap();
        assert!(device.contains(&format!("\"{{cv1touch}}/{profile}\"")));
        // the controller type the driver gives its devices is the one the profile is made for
        assert!(
            device.contains("Prop_ControllerType_String, CV1T_CONTROLLER_TYPE)"),
            "touch_device.cpp sets another controller type"
        );
        // and SteamVR keeps users' bindings under it, so it never changes
        assert_eq!(CONTROLLER_TYPE, "cv1touch_touch");
        assert!(
            include_str!("../driver/src/inputs.cpp").contains(&format!("\"{HOME_SOURCE}/click\""))
        );
    }
}
