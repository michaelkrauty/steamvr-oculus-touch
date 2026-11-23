// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Settings changed while the daemon runs that outlast it: the pitch the controllers are shown
//! with (`cv1touch tilt`), and whether SteamVR is kept starting from a measured base station
//! (`pin_base_station`, set by hand; the daemon reads it again when the file changes). The
//! daemon saves them on every change and reads them when it starts; without a usable file it
//! goes on with the defaults. Fields of older versions (the prediction percents and the latency
//! adjustment) are ignored; the position percent is still written, as 100, because 0.4.3 and
//! earlier refuse a file without it.

use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

pub const FORMAT: &str = "cv1touch-settings";

/// Raw settings input resource policy, not a bound implied by the JSON schema. Normal saved
/// settings are a few hundred bytes; 1 MiB leaves generous room for future fields.
const MAX_FILE_BYTES: u64 = 1024 * 1024;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Settings {
    pub format: String,
    pub version: u32,
    /// Always 100, for 0.4.3 and earlier, which refuse a file without it; ignored when read.
    #[serde(skip_deserializing, default = "full_prediction")]
    pub prediction_percent: u32,
    /// How far the controllers are shown pitched from their tracked pose, degrees, positive with
    /// the forward end up (`cv1touch tilt`; for applications that place a tool in the hand at a
    /// fixed angle). Within +-`MAX_TILT_DEG`; left out when none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tilt_deg: Option<f64>,
    /// Whether the SteamVR driver keeps SteamVR starting from a base station a headset
    /// registration measured (`base_pin`); left out when on.
    #[serde(default = "on", skip_serializing_if = "is_on")]
    pub pin_base_station: bool,
}

fn full_prediction() -> u32 {
    100
}

fn on() -> bool {
    true
}

fn is_on(v: &bool) -> bool {
    *v
}

/// The largest tilt accepted, degrees, either way.
pub const MAX_TILT_DEG: f64 = 45.0;

impl Settings {
    /// The tilt, degrees (0 when none).
    pub fn tilt_deg(&self) -> f64 {
        self.tilt_deg.unwrap_or(0.0)
    }
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            format: FORMAT.into(),
            version: 1,
            prediction_percent: full_prediction(),
            tilt_deg: None,
            pin_base_station: true,
        }
    }
}

/// settings.json in the configuration directory, beside the default calibration (see
/// `calib::file::default_path`).
pub fn default_path() -> PathBuf {
    crate::calib::file::default_path().with_file_name("settings.json")
}

/// The settings at `path`: the defaults when there is no file, and an error when it is
/// unreadable, not a regular file, over the 1 MiB resource limit, malformed, of another format
/// or out of range (the caller goes on with the defaults). Symlinks to regular files are allowed.
pub fn load(path: &Path) -> Result<Settings, String> {
    // On Unix, O_NONBLOCK lets a FIFO reach the type check without waiting for a writer.
    // This does not make regular-file or Windows I/O nonblocking.
    let file = match crate::platform::fs::open_read_nonblocking(path) {
        Ok(file) => file,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Settings::default()),
        Err(e) => return Err(format!("{}: {e}", path.display())),
    };
    let metadata = file
        .metadata()
        .map_err(|e| format!("{}: {e}", path.display()))?;
    if !metadata.is_file() {
        return Err(format!(
            "{}: settings must be a regular file",
            path.display()
        ));
    }
    if metadata.len() > MAX_FILE_BYTES {
        return Err(size_error(path));
    }
    let bytes = read_bounded(file, path)?;
    let text = std::str::from_utf8(&bytes).map_err(|e| format!("{}: {e}", path.display()))?;
    let s: Settings = serde_json::from_str(text).map_err(|e| format!("{}: {e}", path.display()))?;
    if s.format != FORMAT {
        return Err(format!("{}: not a {FORMAT} file", path.display()));
    }
    if !s.tilt_deg().is_finite() || s.tilt_deg().abs() > MAX_TILT_DEG {
        return Err(format!(
            "{}: a tilt outside +-{MAX_TILT_DEG} degrees",
            path.display()
        ));
    }
    Ok(s)
}

fn size_error(path: &Path) -> String {
    format!(
        "{}: settings exceeds {MAX_FILE_BYTES} byte limit",
        path.display()
    )
}

fn read_bounded(file: std::fs::File, path: &Path) -> Result<Vec<u8>, String> {
    // Metadata alone is insufficient: another writer may grow the opened file after the check.
    let mut bytes = Vec::new();
    file.take(MAX_FILE_BYTES + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    if bytes.len() as u64 > MAX_FILE_BYTES {
        return Err(size_error(path));
    }
    Ok(bytes)
}

/// Writes `s` to `path` through a temporary file, synced before and after the rename, so that
/// neither a crash nor a power cut leaves half a file.
pub fn save(path: &Path, s: &Settings) -> Result<(), String> {
    use crate::platform::fs;
    let dir = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    fs::create_private_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let tmp = path.with_extension("json.tmp");
    // a temporary file left by an interrupted save
    let _ = std::fs::remove_file(&tmp);
    let text = serde_json::to_string_pretty(s).map_err(|e| e.to_string())?;
    let write = || -> std::io::Result<()> {
        let mut f = fs::create_private(&tmp)?;
        f.write_all(text.as_bytes())?;
        f.sync_all()?;
        fs::rename_replacing(&tmp, path)?;
        fs::sync_dir(dir)
    };
    write().map_err(|e| {
        let _ = std::fs::remove_file(&tmp);
        format!("{}: {e}", path.display())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounded_read_rejects_growth_after_metadata_check() {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("config/settings.json");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, b"").unwrap();
        let file = crate::platform::fs::open_read_nonblocking(&path).unwrap();
        let metadata = file.metadata().unwrap();
        assert!(metadata.is_file());
        assert!(metadata.len() <= MAX_FILE_BYTES);
        // Deterministically reproduce growth between the loader's metadata check and read.
        let writer = std::fs::OpenOptions::new().write(true).open(&path).unwrap();
        writer.set_len(MAX_FILE_BYTES + 1).unwrap();
        drop(writer);
        let error = read_bounded(file, &path).unwrap_err();
        assert!(error.contains("byte limit"), "{error}");
    }

    #[test]
    fn saved_settings_load_back() {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("config/settings.json");
        let s = Settings {
            tilt_deg: Some(3.0),
            ..Settings::default()
        };
        save(&path, &s).unwrap();
        assert_eq!(load(&path), Ok(s));
    }

    #[test]
    fn no_file_gives_the_defaults_and_a_bad_one_an_error() {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("config/settings.json");
        assert_eq!(load(&path), Ok(Settings::default()), "missing");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        for text in [
            "not json",
            r#"{"format":"something-else","version":1}"#,
            r#"{"format":"cv1touch-settings","version":1,"tilt_deg":46}"#,
        ] {
            std::fs::write(&path, text).unwrap();
            assert!(load(&path).is_err(), "{text}");
        }
    }

    /// The tilt (`cv1touch tilt`) is kept with its sign and left out when none.
    #[test]
    fn the_tilt_is_kept_and_none_by_default() {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("config/settings.json");
        let s = Settings {
            tilt_deg: Some(-7.5),
            ..Settings::default()
        };
        save(&path, &s).unwrap();
        assert_eq!(load(&path), Ok(s));
        assert_eq!(load(&path).unwrap().tilt_deg(), -7.5);
        assert_eq!(Settings::default().tilt_deg(), 0.0);
        save(&path, &Settings::default()).unwrap();
        assert!(!std::fs::read_to_string(&path).unwrap().contains("tilt_deg"));
    }

    /// Base-station pinning is on unless the file turns it off, and written only when off.
    #[test]
    fn pinning_is_on_unless_turned_off() {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("config/settings.json");
        save(&path, &Settings::default()).unwrap();
        assert!(!std::fs::read_to_string(&path)
            .unwrap()
            .contains("pin_base_station"));
        assert!(load(&path).unwrap().pin_base_station);
        std::fs::write(
            &path,
            r#"{"format":"cv1touch-settings","version":1,"pin_base_station":false}"#,
        )
        .unwrap();
        let s = load(&path).unwrap();
        assert!(!s.pin_base_station);
        save(&path, &s).unwrap();
        assert_eq!(load(&path), Ok(s));
    }

    /// A file from an older version keeps its tilt; its prediction percents and latency
    /// adjustment are ignored, whatever they were, and only the position percent is written
    /// back, as 100, so that 0.4.3 and earlier still load the file.
    #[test]
    fn an_older_file_keeps_only_the_tilt() {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("config/settings.json");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(
            &path,
            r#"{"format":"cv1touch-settings","version":1,"prediction_percent":0,"rotation_prediction_percent":101,"latency_adjustment_ms":-31,"tilt_deg":-4.0}"#,
        )
        .unwrap();
        let s = load(&path).unwrap();
        assert_eq!(
            s,
            Settings {
                tilt_deg: Some(-4.0),
                ..Settings::default()
            }
        );
        save(&path, &s).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let v: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(v["prediction_percent"], 100, "{text}");
        assert!(
            !text.contains("rotation") && !text.contains("latency"),
            "{text}"
        );
    }
}
