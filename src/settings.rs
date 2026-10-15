// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Settings changed while the daemon runs that outlast it: how much of the tracked motion SteamVR
//! extrapolates positions and orientations with (`cv1touch prediction`), and the stamp adjustment
//! of published poses (`cv1touch latency`), and the pitch the controllers are shown with
//! (`cv1touch tilt`). The daemon saves them on every change and reads them when it starts;
//! without a usable file it goes on with the defaults.

use std::io::Write;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

pub const FORMAT: &str = "cv1touch-settings";

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Settings {
    pub format: String,
    pub version: u32,
    /// Percent of the tracked linear velocity SteamVR extrapolates positions with: 100 as
    /// tracked, 0 none.
    pub prediction_percent: u32,
    /// Percent of the tracked angular velocity SteamVR extrapolates orientations with; a file
    /// without it (written before positions and orientations had their own) means the same as
    /// for positions.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rotation_prediction_percent: Option<u32>,
    /// How much earlier than measured published poses are stamped, ms, on top of the pipeline's
    /// own estimate (`cv1touch latency`; `cv1touch eval` measures what is left). Within
    /// +-`MAX_LATENCY_ADJUSTMENT_MS`; left out when none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub latency_adjustment_ms: Option<f64>,
    /// How far the controllers are shown pitched from their tracked pose, degrees, positive with
    /// the forward end up (`cv1touch tilt`; for applications that place a tool in the hand at a
    /// fixed angle). Within +-`MAX_TILT_DEG`; left out when none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tilt_deg: Option<f64>,
}

/// The largest latency adjustment accepted, ms, either way.
pub const MAX_LATENCY_ADJUSTMENT_MS: f64 = 30.0;
/// The largest tilt accepted, degrees, either way.
pub const MAX_TILT_DEG: f64 = 45.0;

impl Settings {
    /// The percent for orientations.
    pub fn rotation_percent(&self) -> u32 {
        self.rotation_prediction_percent
            .unwrap_or(self.prediction_percent)
    }

    /// The latency adjustment, ms (0 when none).
    pub fn latency_ms(&self) -> f64 {
        self.latency_adjustment_ms.unwrap_or(0.0)
    }

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
            prediction_percent: 100,
            rotation_prediction_percent: None,
            latency_adjustment_ms: None,
            tilt_deg: None,
        }
    }
}

/// settings.json in the configuration directory, beside the default calibration (see
/// `calib::file::default_path`).
pub fn default_path() -> PathBuf {
    crate::calib::file::default_path().with_file_name("settings.json")
}

/// The settings at `path`: the defaults when there is no file, and an error when it is
/// unreadable, malformed, of another format or out of range (the caller goes on with the
/// defaults).
pub fn load(path: &Path) -> Result<Settings, String> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Settings::default()),
        Err(e) => return Err(format!("{}: {e}", path.display())),
    };
    let s: Settings =
        serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    if s.format != FORMAT {
        return Err(format!("{}: not a {FORMAT} file", path.display()));
    }
    if s.prediction_percent > 100 || s.rotation_percent() > 100 {
        return Err(format!("{}: a prediction percent over 100", path.display()));
    }
    if !s.latency_ms().is_finite() || s.latency_ms().abs() > MAX_LATENCY_ADJUSTMENT_MS {
        return Err(format!(
            "{}: a latency adjustment outside +-{MAX_LATENCY_ADJUSTMENT_MS} ms",
            path.display()
        ));
    }
    if !s.tilt_deg().is_finite() || s.tilt_deg().abs() > MAX_TILT_DEG {
        return Err(format!(
            "{}: a tilt outside +-{MAX_TILT_DEG} degrees",
            path.display()
        ));
    }
    Ok(s)
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

    fn temp(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("cv1t-settings-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        dir.join("settings.json")
    }

    #[test]
    fn saved_settings_load_back() {
        let path = temp("round-trip");
        let s = Settings {
            prediction_percent: 100,
            rotation_prediction_percent: Some(0),
            ..Settings::default()
        };
        save(&path, &s).unwrap();
        assert_eq!(load(&path), Ok(s));
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }

    /// A file written before orientations had their own percent sets both.
    #[test]
    fn an_older_file_sets_positions_and_orientations_alike() {
        let path = temp("older");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(
            &path,
            r#"{"format":"cv1touch-settings","version":1,"prediction_percent":0}"#,
        )
        .unwrap();
        let s = load(&path).unwrap();
        assert_eq!((s.prediction_percent, s.rotation_percent()), (0, 0));
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }

    #[test]
    fn no_file_gives_the_defaults_and_a_bad_one_an_error() {
        let path = temp("bad");
        assert_eq!(load(&path), Ok(Settings::default()), "missing");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        for text in [
            "not json",
            r#"{"format":"cv1touch-settings","version":1,"prediction_percent":101}"#,
            r#"{"format":"cv1touch-settings","version":1,"prediction_percent":0,"rotation_prediction_percent":101}"#,
            r#"{"format":"something-else","version":1,"prediction_percent":0}"#,
            r#"{"format":"cv1touch-settings","version":1,"prediction_percent":100,"latency_adjustment_ms":31}"#,
            r#"{"format":"cv1touch-settings","version":1,"prediction_percent":100,"latency_adjustment_ms":-30.5}"#,
            r#"{"format":"cv1touch-settings","version":1,"prediction_percent":100,"tilt_deg":46}"#,
        ] {
            std::fs::write(&path, text).unwrap();
            assert!(load(&path).is_err(), "{text}");
        }
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }

    /// The tilt (`cv1touch tilt`) is kept with its sign and left out when none.
    #[test]
    fn the_tilt_is_kept_and_none_by_default() {
        let path = temp("tilt");
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
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }

    /// The latency adjustment (`cv1touch latency`) is kept with its sign; a file from before it
    /// had one, and one that leaves it out, mean none.
    #[test]
    fn the_latency_adjustment_is_kept_and_none_by_default() {
        let path = temp("latency");
        let s = Settings {
            latency_adjustment_ms: Some(-5.4),
            ..Settings::default()
        };
        save(&path, &s).unwrap();
        assert_eq!(load(&path), Ok(s));
        assert_eq!(load(&path).unwrap().latency_ms(), -5.4);
        std::fs::write(
            &path,
            r#"{"format":"cv1touch-settings","version":1,"prediction_percent":100}"#,
        )
        .unwrap();
        assert_eq!(load(&path).unwrap().latency_ms(), 0.0);
        assert_eq!(Settings::default().latency_ms(), 0.0);
        // none is left out of the file, not written as 0
        save(&path, &Settings::default()).unwrap();
        assert!(!std::fs::read_to_string(&path)
            .unwrap()
            .contains("latency_adjustment_ms"));
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }
}
