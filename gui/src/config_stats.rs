// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Fit quality read from the saved calibration files. Read only; a missing or
//! unreadable file is simply absent.

use serde_json::Value;
use std::path::{Path, PathBuf};

#[derive(Clone, Default)]
pub struct ConfigStats {
    /// `calibration.json`: the sensors' room calibration.
    pub calibration: Option<Value>,
    /// `registration.json`: the alignment with SteamVR's headset tracking.
    pub registration: Option<Value>,
    /// `render-offsets.json`: where the controller models sit on the tracked LEDs.
    pub render: Option<Value>,
    /// `settings.json`: the controller angle and base-station pinning the daemon starts with.
    pub settings: Option<Value>,
}

impl ConfigStats {
    pub fn load() -> Self {
        dir().map(|dir| Self::load_from(&dir)).unwrap_or_default()
    }

    pub fn load_from(dir: &Path) -> Self {
        let read = |name: &str, format: &str| {
            std::fs::read(dir.join(name))
                .ok()
                .and_then(|bytes| serde_json::from_slice::<Value>(&bytes).ok())
                .filter(|value| value["format"] == format)
        };
        Self {
            calibration: read("calibration.json", "cv1touch-calibration"),
            registration: read("registration.json", "cv1touch-registration"),
            render: read("render-offsets.json", "cv1touch-render-offsets"),
            settings: read("settings.json", "cv1touch-settings"),
        }
    }

    /// The saved controller angle, degrees; the daemon's default without a file.
    pub fn tilt_deg(&self) -> f64 {
        self.settings
            .as_ref()
            .and_then(|s| s["tilt_deg"].as_f64())
            .unwrap_or(0.0)
    }

    /// The saved base-station pinning: the file leaves it out when it is on.
    pub fn pin_base_station(&self) -> bool {
        self.settings
            .as_ref()
            .and_then(|s| s["pin_base_station"].as_bool())
            .unwrap_or(true)
    }

    /// The median of the sensors' reprojection RMS in pixels, and how many sensors.
    pub fn sensor_fit(&self) -> Option<(f64, usize)> {
        let mut rms: Vec<f64> = self.calibration.as_ref()?["sensors"]
            .as_array()?
            .iter()
            .filter_map(|sensor| sensor["rms_px"].as_f64())
            .collect();
        rms.sort_by(f64::total_cmp);
        let n = rms.len();
        (n > 0).then(|| {
            let median = if n % 2 == 1 {
                rms[n / 2]
            } else {
                (rms[n / 2 - 1] + rms[n / 2]) / 2.0
            };
            (median, n)
        })
    }

    /// One sensor's calibration entry, matched by serial.
    pub fn sensor(&self, serial: &str) -> Option<&Value> {
        self.calibration.as_ref()?["sensors"]
            .as_array()?
            .iter()
            .find(|s| {
                s["serial"]
                    .as_str()
                    .is_some_and(|s| s.eq_ignore_ascii_case(serial))
            })
    }

    /// The headset alignment's position (mm) and rotation (degrees) RMS.
    pub fn headset_fit(&self) -> Option<(f64, f64)> {
        let r = self.registration.as_ref()?;
        Some((r["pos_rms_mm"].as_f64()?, r["rot_rms_deg"].as_f64()?))
    }

    /// The controller model offsets' RMS (mm) and LED count for each hand.
    pub fn render_fit(&self, hand: &str) -> Option<(f64, u64)> {
        let h = self.render.as_ref()?["hands"]
            .as_array()?
            .iter()
            .find(|h| h["hand"] == hand)?;
        Some((h["rms_mm"].as_f64()?, h["leds"].as_u64().unwrap_or(0)))
    }
}

/// cv1touch's configuration directory, as the daemon resolves it.
fn dir() -> Option<PathBuf> {
    let var = |name| {
        std::env::var_os(name)
            .map(PathBuf::from)
            .filter(|p| p.is_absolute())
    };
    if cfg!(windows) {
        var("LOCALAPPDATA").map(|p| p.join("cv1touch"))
    } else {
        var("XDG_CONFIG_HOME")
            .or_else(|| var("HOME").map(|home| home.join(".config")))
            .map(|p| p.join("cv1touch"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_fit_quality_and_ignores_missing_or_foreign_files() {
        let dir = std::env::temp_dir().join(format!("cv1touch-config-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        assert!(ConfigStats::load_from(&dir).calibration.is_none());
        let write = |name: &str, body: &str| std::fs::write(dir.join(name), body).unwrap();
        write(
            "calibration.json",
            r#"{"format":"cv1touch-calibration","sensors":[{"serial":"SENSOR-A","rms_px":0.3},{"serial":"SENSOR-B","rms_px":0.1},{"serial":"SENSOR-C","rms_px":0.2},{"serial":"SENSOR-D","rms_px":0.4}],"stats":{"gravity_spread_deg":0.1}}"#,
        );
        write(
            "registration.json",
            r#"{"format":"cv1touch-registration","pos_rms_mm":2.0,"rot_rms_deg":0.9,"pairs":100}"#,
        );
        write(
            "render-offsets.json",
            r#"{"format":"cv1touch-render-offsets","hands":[{"hand":"left","rms_mm":0.18,"leds":22},{"hand":"right","rms_mm":0.21,"leds":21}]}"#,
        );
        let stats = ConfigStats::load_from(&dir);
        let (median, n) = stats.sensor_fit().unwrap();
        assert!((median - 0.25).abs() < 1e-9);
        assert_eq!(n, 4);
        assert_eq!(stats.sensor("sensor-b").unwrap()["rms_px"], 0.1);
        assert_eq!(stats.headset_fit(), Some((2.0, 0.9)));
        assert_eq!(stats.render_fit("right"), Some((0.21, 21)));
        assert_eq!(stats.render_fit("other"), None);
        write("calibration.json", "{not json");
        write(
            "registration.json",
            r#"{"format":"something-else","pos_rms_mm":1}"#,
        );
        let stats = ConfigStats::load_from(&dir);
        assert!(stats.calibration.is_none() && stats.registration.is_none());
        assert!(stats.sensor_fit().is_none() && stats.headset_fit().is_none());
        std::fs::remove_dir_all(dir).unwrap();
    }
}
