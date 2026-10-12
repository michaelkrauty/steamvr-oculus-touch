// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Explicit input calibration in factory-normalized coordinates; raw values stay available
//! to the calibration wizard. Stored identities bind each hand to its physical controller.

use serde::{Deserialize, Serialize};
use std::fs;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

pub const FORMAT: &str = "cv1touch-input-calibration";
pub const VERSION: u32 = 1;
pub const DEFAULT_DEADZONE: f32 = 0.05;
pub const MAX_FILE_BYTES: usize = 16 * 1024;
const MIN_TRAVEL: f64 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Endpoint {
    pub released: f32,
    pub pressed: f32,
}
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StickAxis {
    pub min: f32,
    pub center: f32,
    pub max: f32,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HandCalibration {
    pub identity: String,
    pub trigger: Endpoint,
    pub grip: Endpoint,
    pub stick: [StickAxis; 2],
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CalibrationFile {
    pub format: String,
    pub version: u32,
    pub analog_deadzone: f32,
    pub stick_deadzone: f32,
    pub hands: [HandCalibration; 2],
}

fn valid_coordinate(x: f32) -> bool {
    x.is_finite() && x.abs() <= 4.0
}
fn valid_deadzone(x: f32) -> bool {
    x.is_finite() && (0.0..=0.2).contains(&x)
}
fn finite_clamp(x: f32, lo: f32, hi: f32) -> f32 {
    if x.is_finite() {
        x.clamp(lo, hi)
    } else {
        0.0
    }
}

impl Endpoint {
    pub fn validate(&self) -> Result<(), String> {
        if !valid_coordinate(self.released)
            || !valid_coordinate(self.pressed)
            || (self.pressed as f64 - self.released as f64) < MIN_TRAVEL
        {
            return Err("analog endpoints must be finite within [-4,4] and span at least 0.5 from released to pressed".into());
        }
        Ok(())
    }
    fn apply(&self, value: f32, deadzone: f32) -> f32 {
        if !value.is_finite() {
            return 0.;
        }
        let unit =
            (value as f64 - self.released as f64) / (self.pressed as f64 - self.released as f64);
        ((unit - deadzone as f64) / (1. - 2. * deadzone as f64)).clamp(0., 1.) as f32
    }
}
impl StickAxis {
    pub fn validate(&self) -> Result<(), String> {
        if ![self.min, self.center, self.max]
            .into_iter()
            .all(valid_coordinate)
            || (self.center as f64 - self.min as f64) < MIN_TRAVEL
            || (self.max as f64 - self.center as f64) < MIN_TRAVEL
        {
            return Err("stick endpoints must be finite within [-4,4], ordered min < center < max, with at least 0.5 travel on each side".into());
        }
        Ok(())
    }
    fn apply(&self, value: f32) -> f64 {
        let delta = value as f64 - self.center as f64;
        delta
            / if delta < 0. {
                self.center as f64 - self.min as f64
            } else {
                self.max as f64 - self.center as f64
            }
    }
}
impl HandCalibration {
    pub fn validate(&self) -> Result<(), String> {
        if self.identity.len() != 32
            || !self
                .identity
                .bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
        {
            return Err("controller identity must be 32 lowercase hexadecimal characters".into());
        }
        self.trigger.validate()?;
        self.grip.validate()?;
        for axis in &self.stick {
            axis.validate()?;
        }
        Ok(())
    }
}
impl CalibrationFile {
    pub fn new(hands: [HandCalibration; 2]) -> Result<Self, String> {
        let value = Self {
            format: FORMAT.into(),
            version: VERSION,
            analog_deadzone: DEFAULT_DEADZONE,
            stick_deadzone: DEFAULT_DEADZONE,
            hands,
        };
        value.validate()?;
        Ok(value)
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.format != FORMAT || self.version != VERSION {
            return Err("unsupported input calibration format or version".into());
        }
        if !valid_deadzone(self.analog_deadzone) || !valid_deadzone(self.stick_deadzone) {
            return Err("input deadzones must be finite between 0 and 0.2".into());
        }
        for (i, hand) in self.hands.iter().enumerate() {
            hand.validate().map_err(|e| format!("hand {i}: {e}"))?;
        }
        if self.hands[0].identity == self.hands[1].identity {
            return Err("controller identities must be distinct".into());
        }
        Ok(())
    }
    /// Unknown identities and invalid in-memory configurations use bounded factory values.
    /// The calibrated stick uses a circular gate: diagonal square-gate overshoot is clipped
    /// radially, preserving direction rather than clamping each axis separately.
    pub fn apply(
        &self,
        hand: usize,
        identity: &str,
        trigger: f32,
        grip: f32,
        stick: [f32; 2],
    ) -> (f32, f32, [f32; 2]) {
        let Some(h) = self.hands.get(hand) else {
            return factory_values(trigger, grip, stick);
        };
        if h.identity != identity || self.validate().is_err() {
            return factory_values(trigger, grip, stick);
        }
        let mut out = [0.; 2];
        if stick.into_iter().all(f32::is_finite) {
            let x = h.stick[0].apply(stick[0]);
            let y = h.stick[1].apply(stick[1]);
            let length = x.hypot(y);
            if length > self.stick_deadzone as f64 {
                let magnitude = ((length - self.stick_deadzone as f64)
                    / (1. - self.stick_deadzone as f64))
                    .min(1.);
                out = [
                    (x / length * magnitude) as f32,
                    (y / length * magnitude) as f32,
                ];
            }
        }
        (
            h.trigger.apply(trigger, self.analog_deadzone),
            h.grip.apply(grip, self.analog_deadzone),
            out,
        )
    }
}

/// Keep factory behavior when no matching calibration exists, but never publish NaN or
/// values outside the declared one-sided analog / two-sided stick input ranges.
pub fn factory_values(trigger: f32, grip: f32, stick: [f32; 2]) -> (f32, f32, [f32; 2]) {
    (
        finite_clamp(trigger, 0., 1.),
        finite_clamp(grip, 0., 1.),
        stick.map(|v| finite_clamp(v, -1., 1.)),
    )
}
pub fn default_path() -> PathBuf {
    crate::platform::dirs::get()
        .config
        .join("input-calibration.json")
}
pub fn load(path: &Path) -> Result<Option<CalibrationFile>, String> {
    let file = match crate::platform::fs::open_read_nofollow(path) {
        Ok(f) => f,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(e.to_string()),
    };
    let meta = file.metadata().map_err(|e| e.to_string())?;
    if !meta.is_file() || meta.len() > MAX_FILE_BYTES as u64 {
        return Err("input calibration must be a regular file of at most 16 KiB".into());
    }
    let mut bytes = Vec::new();
    file.take((MAX_FILE_BYTES + 1) as u64)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > MAX_FILE_BYTES {
        return Err("input calibration exceeds 16 KiB".into());
    }
    let value: CalibrationFile = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    value.validate()?;
    Ok(Some(value))
}
pub fn save(path: &Path, value: &CalibrationFile) -> Result<(), String> {
    value.validate()?;
    let bytes = serde_json::to_vec_pretty(value).map_err(|e| e.to_string())?;
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    crate::platform::fs::create_private_dir_all(parent).map_err(|e| e.to_string())?;
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let temp = parent.join(format!(
        ".input-calibration-{}-{}.tmp",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let mut file = crate::platform::fs::create_private(&temp).map_err(|e| e.to_string())?;
    let result = (|| {
        file.write_all(&bytes)?;
        file.sync_all()?;
        crate::platform::fs::rename_replacing(&temp, path)?;
        crate::platform::fs::sync_dir(parent)
    })()
    .map_err(|e: std::io::Error| e.to_string());
    if result.is_err() {
        let _ = fs::remove_file(temp);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    fn example() -> CalibrationFile {
        CalibrationFile::new(std::array::from_fn(|i| HandCalibration {
            identity: format!("{:032x}", i + 1),
            trigger: Endpoint {
                released: -0.1,
                pressed: 0.95,
            },
            grip: Endpoint {
                released: -0.18,
                pressed: 0.82,
            },
            stick: [StickAxis {
                min: -0.9,
                center: 0.1,
                max: 0.8,
            }; 2],
        }))
        .unwrap()
    }
    #[test]
    fn measured_endpoints_deadzone_and_monotonic_travel() {
        let c = example();
        let id = &c.hands[0].identity;
        assert_eq!(c.apply(0, id, -0.1, -0.18, [0.1; 2]), (0., 0., [0.; 2]));
        assert_eq!(c.apply(0, id, 0.95, 0.82, [0.1; 2]), (1., 1., [0.; 2]));
        assert_eq!(c.apply(0, id, 0.92, 0.79, [0.1; 2]).0, 1.);
        let mut last = 0.;
        for i in 0..1001 {
            let x = -0.3 + i as f32 / 800.;
            let v = c.apply(0, id, x, x, [0.1; 2]).1;
            assert!(v >= last && (0.0..=1.0).contains(&v));
            last = v;
        }
    }
    #[test]
    fn sticks_use_asymmetric_center_then_radial_deadzone_and_round_gate() {
        let c = example();
        let id = &c.hands[0].identity;
        assert_eq!(c.apply(0, id, 0., 0., [0.8, 0.1]).2, [1., 0.]);
        assert_eq!(c.apply(0, id, 0., 0., [-0.9, 0.1]).2, [-1., 0.]);
        assert_eq!(c.apply(0, id, 0., 0., [0.11, 0.11]).2, [0., 0.]);
        let s = c.apply(0, id, 0., 0., [0.8, 0.8]).2;
        assert!((s[0] - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-6 && s[0] == s[1]);
        let s = c.apply(0, id, 0., 0., [0.45, 0.8]).2;
        assert!((s[0] / s[1] - 0.5).abs() < 1e-6 && (s[0].hypot(s[1]) - 1.).abs() < 1e-6);
    }
    #[test]
    fn fallback_and_nonfinite_samples_are_safe() {
        let c = example();
        for hand in [0, 2, usize::MAX] {
            assert_eq!(
                c.apply(
                    hand,
                    "unknown",
                    f32::NAN,
                    f32::INFINITY,
                    [f32::NEG_INFINITY, 3.]
                ),
                (0., 0., [0., 1.])
            );
        }
        assert_eq!(
            c.apply(
                0,
                &c.hands[0].identity,
                f32::NAN,
                f32::INFINITY,
                [f32::NAN, 1.]
            ),
            (0., 0., [0., 0.])
        );
    }
    #[test]
    fn rejects_bad_ranges_deadzones_and_identities() {
        let mut c = example();
        c.hands[0].grip.pressed = 0.1;
        assert!(c.validate().is_err());
        let mut c = example();
        c.hands[0].stick[0].min = 0.;
        assert!(c.validate().is_err());
        for x in [f32::NAN, f32::INFINITY, f32::MAX, -5.] {
            let mut c = example();
            c.hands[0].trigger.released = x;
            assert!(c.validate().is_err());
        }
        for x in [-0.1, 0.201, f32::NAN] {
            let mut c = example();
            c.analog_deadzone = x;
            assert!(c.validate().is_err());
        }
        let mut c = example();
        c.hands[1].identity = c.hands[0].identity.clone();
        assert!(c.validate().is_err());
        let mut c = example();
        c.hands[0].identity = "not-an-identity".into();
        assert!(c.validate().is_err());
    }
    #[test]
    fn extreme_samples_stay_finite_and_radial_direction_is_preserved() {
        let c = example();
        let id = &c.hands[0].identity;
        for value in [-f32::MAX, -4.0, 0.0, 4.0, f32::MAX] {
            let (t, g, stick) = c.apply(0, id, value, value, [value, value]);
            assert!(t.is_finite() && g.is_finite() && stick.into_iter().all(f32::is_finite));
            assert!((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&g));
            assert!(stick[0].hypot(stick[1]) <= 1.000001);
        }
        for i in 0..360 {
            let angle = (i as f32).to_radians();
            let direction = [angle.cos(), angle.sin()];
            let raw = direction.map(|v| 0.1 + v * if v < 0.0 { 1.0 } else { 0.7 });
            let actual = c.apply(0, id, 0.0, 0.0, raw).2;
            for axis in 0..2 {
                assert!((actual[axis] - direction[axis]).abs() < 1e-5);
            }
        }
    }
    #[test]
    fn malformed_schema_and_version_are_rejected() {
        let mut v = serde_json::to_value(example()).unwrap();
        v["version"] = 999.into();
        assert!(serde_json::from_value::<CalibrationFile>(v.clone())
            .unwrap()
            .validate()
            .is_err());
        v["version"] = VERSION.into();
        v["unknown"] = true.into();
        assert!(serde_json::from_value::<CalibrationFile>(v).is_err());
        let mut v = serde_json::to_value(example()).unwrap();
        v["hands"][0]["grip"]["pressed"] = serde_json::Value::Null;
        assert!(serde_json::from_value::<CalibrationFile>(v).is_err());
        let c = example();
        let left = c.apply(0, &c.hands[1].identity, 0.2, 0.3, [0.4, 0.5]);
        assert_eq!(left, (0.2, 0.3, [0.4, 0.5]));
    }
    #[test]
    #[cfg(unix)]
    fn bounded_atomic_private_storage_and_bad_loads() {
        use std::os::unix::fs::PermissionsExt;
        let dir = std::env::temp_dir().join(format!("cv1touch-input-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir(&dir).unwrap();
        let p = dir.join("calibration.json");
        let c = example();
        assert_eq!(load(&p).unwrap(), None);
        save(&p, &c).unwrap();
        assert_eq!(load(&p).unwrap(), Some(c.clone()));
        assert_eq!(
            std::fs::metadata(&p).unwrap().permissions().mode() & 0o777,
            0o600
        );
        let before = std::fs::read(&p).unwrap();
        let mut invalid = c.clone();
        invalid.hands[0].grip.pressed = 0.;
        assert!(save(&p, &invalid).is_err());
        assert_eq!(std::fs::read(&p).unwrap(), before);
        for bytes in [
            b"{".to_vec(),
            vec![b' '; MAX_FILE_BYTES + 1],
            b"{\"format\":\"unknown\"}".to_vec(),
        ] {
            std::fs::write(&p, bytes).unwrap();
            assert!(load(&p).is_err());
        }
        std::fs::remove_file(&p).unwrap();
        std::os::unix::fs::symlink("/dev/zero", &p).unwrap();
        assert!(load(&p).is_err());
        std::fs::remove_dir_all(dir).unwrap();
    }
}
