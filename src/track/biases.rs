// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Each controller's IMU biases, remembered across daemon restarts. The gyro bias always seeds
//! a new run. The accelerometer bias is seeded only from `verified_accel` — an estimate verified
//! across varied orientations (camera calibration or online multi-orientation tracking), never an
//! unsettled or single-orientation snapshot. Legacy files without `verified_accel` start cold.
//! Controllers are told apart by hand and LED model, so a replaced controller starts fresh.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::calib::capture::Led;

pub const FORMAT: &str = "cv1touch-imu-biases";

/// How far a remembered gyro bias is trusted when seeding a new run: its turn-on spread.
pub const SEED_SD_GYRO: f64 = 0.005;

/// Prior standard deviation on a verified accelerometer bias (m/s^2) when it seeds a new tracker:
/// matches cold acquisition's `acquire_sd[4]` so a verified bias stays centered on the hardware
/// offset without slowing wrong-lock detection or adaptation to thermal drift.
pub const SEED_SD_ACCEL: f64 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Entry {
    /// rad/s, body frame.
    pub gyro: [f64; 3],
    /// m/s^2, body frame (last observed while tracking, for diagnostics).
    pub accel: [f64; 3],
    /// m/s^2, body frame: verified across varied orientations (camera calibration or online
    /// multi-orientation tracking), safe to seed into a new tracker. Older files omit this field
    /// and start accelerometer bias cold.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub verified_accel: Option<[f64; 3]>,
    pub updated_unix_s: u64,
}

impl Entry {
    pub fn gyro(&self) -> Vector3<f64> {
        Vector3::from(self.gyro)
    }

    pub fn verified_accel(&self) -> Option<Vector3<f64>> {
        self.verified_accel.map(Vector3::from)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BiasFile {
    pub format: String,
    pub version: u32,
    pub controllers: BTreeMap<String, Entry>,
}

impl Default for BiasFile {
    fn default() -> Self {
        BiasFile {
            format: FORMAT.into(),
            version: 1,
            controllers: BTreeMap::new(),
        }
    }
}

/// imu-biases.json in cv1touch's state directory (platform::dirs).
pub fn default_path() -> PathBuf {
    crate::platform::dirs::get().state.join("imu-biases.json")
}

/// "left-1a2b3c4d": the hand and a fingerprint of the LED model at 0.1 mm.
pub fn key(hand: usize, leds: &[Led]) -> String {
    let mut h: u32 = 0x811c_9dc5;
    for l in leds {
        for v in l.pos.iter() {
            for b in ((v * 1e4).round() as i64).to_le_bytes() {
                h = (h ^ b as u32).wrapping_mul(0x0100_0193);
            }
        }
    }
    format!(
        "{}-{h:08x}",
        ["left", "right"].get(hand).copied().unwrap_or("other")
    )
}

/// The file at `path`; an empty one when it is missing or unreadable.
pub fn load(path: &Path) -> BiasFile {
    std::fs::read_to_string(path)
        .ok()
        .and_then(|t| serde_json::from_str::<BiasFile>(&t).ok())
        .filter(|f| f.format == FORMAT)
        .unwrap_or_default()
}

static NEXT_TMP_ID: AtomicU64 = AtomicU64::new(0);

fn lock_path(path: &Path) -> PathBuf {
    let mut os = path.as_os_str().to_os_string();
    os.push(".lock");
    PathBuf::from(os)
}

fn with_lock<T>(path: &Path, f: impl FnOnce() -> Result<T, String>) -> Result<T, String> {
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    let lock = lock_path(path);
    let _guard =
        crate::platform::fs::lock_file(&lock).map_err(|e| format!("{}: {e}", lock.display()))?;
    f()
}

fn save_unlocked(path: &Path, f: &BiasFile) -> Result<(), String> {
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    let seq = NEXT_TMP_ID.fetch_add(1, Ordering::Relaxed);
    let name = path
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("imu-biases.json");
    let tmp = path.with_file_name(format!(".{name}.{}.{seq}.tmp", std::process::id()));
    let text = serde_json::to_string_pretty(f).map_err(|e| e.to_string())?;
    if let Err(e) = std::fs::write(&tmp, text) {
        let _ = std::fs::remove_file(&tmp);
        return Err(format!("{}: {e}", tmp.display()));
    }
    if let Err(e) = crate::platform::fs::rename_replacing(&tmp, path) {
        let _ = std::fs::remove_file(&tmp);
        return Err(format!("{}: {e}", path.display()));
    }
    Ok(())
}

/// Writes `f` to `path` through a unique temporary file under the bias-file lock, so overlapping
/// writers cannot collide and a crash never leaves half a file.
pub fn save(path: &Path, f: &BiasFile) -> Result<(), String> {
    with_lock(path, || save_unlocked(path, f))
}

/// Updates `verified_accel` in `path` for each `(key, bias)` without disturbing saved gyro biases
/// or other controllers. Serialized across processes with `merge_updates`.
pub fn save_verified_accel(
    path: &Path,
    entries: &[(String, Vector3<f64>)],
    updated_unix_s: u64,
) -> Result<(), String> {
    if entries.is_empty() {
        return Ok(());
    }
    with_lock(path, || {
        let mut file = load(path);
        for (k, a) in entries {
            let accel = [a.x, a.y, a.z];
            file.controllers
                .entry(k.clone())
                .and_modify(|e| {
                    e.verified_accel = Some(accel);
                    e.updated_unix_s = updated_unix_s;
                })
                .or_insert(Entry {
                    gyro: [0.0; 3],
                    accel,
                    verified_accel: Some(accel),
                    updated_unix_s,
                });
        }
        save_unlocked(path, &file)
    })
}

/// One controller's bias update from the daemon's radio thread, merged into the on-disk file so an
/// external calibration's `verified_accel` (`save_verified_accel`) is never overwritten.
#[derive(Debug, Clone, PartialEq)]
pub struct Update {
    /// Current gyro and accelerometer bias from a tracking hand, if tracking.
    pub tracking: Option<([f64; 3], [f64; 3])>,
    /// `Some(Some(v))` = newly verified in this run; `Some(None)` = invalidated `invalidated_from`
    /// in this run; `None` = preserve whatever `verified_accel` is on disk.
    pub verified_accel: Option<Option<[f64; 3]>>,
    /// The `verified_accel` this run originally seeded, checked before applying `Some(None)` so a
    /// newer calibration written to disk is not cleared.
    pub invalidated_from: Option<[f64; 3]>,
    pub updated_unix_s: u64,
}

/// Merges daemon bias updates into `path` without clobbering untouched controllers or an externally
/// written `verified_accel`.
pub fn merge_updates(path: &Path, updates: &BTreeMap<String, Update>) -> Result<(), String> {
    if updates.is_empty() {
        return Ok(());
    }
    with_lock(path, || {
        let mut file = load(path);
        let mut changed = false;
        for (k, u) in updates {
            let disk = file.controllers.get(k).copied();
            let disk_verified = disk.and_then(|e| e.verified_accel);
            let verified_accel = match u.verified_accel {
                Some(Some(v)) => Some(v),
                Some(None) => {
                    if disk_verified == u.invalidated_from {
                        None
                    } else {
                        disk_verified
                    }
                }
                None => disk_verified,
            };
            if let Some((gyro, accel)) = u.tracking {
                file.controllers.insert(
                    k.clone(),
                    Entry {
                        gyro,
                        accel,
                        verified_accel,
                        updated_unix_s: u.updated_unix_s,
                    },
                );
                changed = true;
            } else if let Some(mut e) = disk {
                if e.verified_accel != verified_accel {
                    e.verified_accel = verified_accel;
                    e.updated_unix_s = u.updated_unix_s;
                    file.controllers.insert(k.clone(), e);
                    changed = true;
                }
            } else if let Some(v) = verified_accel {
                file.controllers.insert(
                    k.clone(),
                    Entry {
                        gyro: [0.0; 3],
                        accel: v,
                        verified_accel: Some(v),
                        updated_unix_s: u.updated_unix_s,
                    },
                );
                changed = true;
            }
        }
        if changed {
            save_unlocked(path, &file)?;
        }
        Ok(())
    })
}
#[cfg(test)]
mod tests {
    use super::*;

    fn leds(dx: f64) -> Vec<Led> {
        vec![
            Led {
                pos: Vector3::new(0.03 + dx, -0.008, -0.011),
                dir: Vector3::y(),
            },
            Led {
                pos: Vector3::new(-0.02, 0.01, 0.0),
                dir: Vector3::y(),
            },
        ]
    }

    #[test]
    fn controllers_are_told_apart_by_hand_and_led_model() {
        assert_eq!(key(0, &leds(0.0)), key(0, &leds(0.0)));
        assert_eq!(
            key(0, &leds(0.0)),
            key(0, &leds(1e-6)),
            "below the 0.1 mm fingerprint"
        );
        assert_ne!(key(0, &leds(0.0)), key(1, &leds(0.0)));
        assert_ne!(key(0, &leds(0.0)), key(0, &leds(1e-3)));
        assert!(key(1, &leds(0.0)).starts_with("right-"));
    }

    #[test]
    fn biases_round_trip_and_a_missing_file_is_empty() {
        let tmp = tempfile::tempdir().unwrap();
        let dir = tmp.path();
        let path = dir.join("sub/imu-biases.json");
        assert_eq!(load(&path), BiasFile::default());
        let mut f = BiasFile::default();
        f.controllers.insert(
            key(1, &leds(0.0)),
            Entry {
                gyro: [0.01, -0.005, 0.003],
                accel: [0.018, 0.449, -0.081],
                verified_accel: Some([0.018, 0.449, -0.081]),
                updated_unix_s: 1_791_500_000,
            },
        );
        save(&path, &f).unwrap();
        assert_eq!(load(&path), f);
        std::fs::write(&path, "{ not json").unwrap();
        assert_eq!(load(&path), BiasFile::default());
    }

    #[test]
    fn legacy_file_without_verified_accel_loads_as_none() {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("imu-biases.json");
        std::fs::write(
            &path,
            r#"{"format":"cv1touch-imu-biases","version":1,"controllers":{"left-abcd":{"gyro":[0.01,-0.02,0.03],"accel":[0.1,1.9,0.0],"updated_unix_s":100}}}"#,
        )
        .unwrap();
        let f = load(&path);
        let e = f.controllers.get("left-abcd").unwrap();
        assert_eq!(e.gyro, [0.01, -0.02, 0.03]);
        assert_eq!(e.accel, [0.1, 1.9, 0.0]);
        assert_eq!(e.verified_accel, None);
    }

    #[test]
    fn save_verified_accel_and_merge_updates_preserve_each_other() {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("imu-biases.json");
        let mut init = BiasFile::default();
        init.controllers.insert(
            "left-1111".into(),
            Entry {
                gyro: [0.01, 0.02, 0.03],
                accel: [0.0, 0.2, 0.0],
                verified_accel: None,
                updated_unix_s: 100,
            },
        );
        init.controllers.insert(
            "right-2222".into(),
            Entry {
                gyro: [-0.01, 0.0, 0.01],
                accel: [0.0, 0.9, 0.0],
                verified_accel: Some([0.0, 0.9, 0.0]),
                updated_unix_s: 100,
            },
        );
        save(&path, &init).unwrap();

        // Calibration saves verified_accel for left without disturbing gyro or right.
        save_verified_accel(
            &path,
            &[("left-1111".into(), Vector3::new(-0.04, 1.90, 0.10))],
            200,
        )
        .unwrap();
        let after_cal = load(&path);
        assert_eq!(after_cal.controllers["left-1111"].gyro, [0.01, 0.02, 0.03]);
        assert_eq!(
            after_cal.controllers["left-1111"].verified_accel,
            Some([-0.04, 1.90, 0.10])
        );
        assert_eq!(
            after_cal.controllers["right-2222"].verified_accel,
            Some([0.0, 0.9, 0.0])
        );

        // Daemon flushes tracking update without verified change: keeps external verified_accel.
        let mut updates = BTreeMap::new();
        updates.insert(
            "left-1111".into(),
            Update {
                tracking: Some(([0.02, 0.03, 0.04], [-0.05, 1.88, 0.09])),
                verified_accel: None,
                invalidated_from: None,
                updated_unix_s: 210,
            },
        );
        merge_updates(&path, &updates).unwrap();
        let after_track = load(&path);
        assert_eq!(
            after_track.controllers["left-1111"].gyro,
            [0.02, 0.03, 0.04]
        );
        assert_eq!(
            after_track.controllers["left-1111"].verified_accel,
            Some([-0.04, 1.90, 0.10])
        );

        // Stale invalidation from an older seed does not erase the newer calibration's bias.
        updates.clear();
        updates.insert(
            "left-1111".into(),
            Update {
                tracking: None,
                verified_accel: Some(None),
                invalidated_from: Some([0.0, 0.0, 0.0]),
                updated_unix_s: 220,
            },
        );
        merge_updates(&path, &updates).unwrap();
        assert_eq!(
            load(&path).controllers["left-1111"].verified_accel,
            Some([-0.04, 1.90, 0.10])
        );

        // Matching invalidation clears verified_accel cleanly.
        updates.clear();
        updates.insert(
            "left-1111".into(),
            Update {
                tracking: None,
                verified_accel: Some(None),
                invalidated_from: Some([-0.04, 1.90, 0.10]),
                updated_unix_s: 230,
            },
        );
        merge_updates(&path, &updates).unwrap();
        assert_eq!(load(&path).controllers["left-1111"].verified_accel, None);

        // Newly verified controller that went to sleep before its first tracking save still persists.
        updates.clear();
        updates.insert(
            "right-3333".into(),
            Update {
                tracking: None,
                verified_accel: Some(Some([0.0, 1.85, 0.0])),
                invalidated_from: None,
                updated_unix_s: 240,
            },
        );
        merge_updates(&path, &updates).unwrap();
        let new_entry = load(&path).controllers["right-3333"];
        assert_eq!(new_entry.verified_accel, Some([0.0, 1.85, 0.0]));
        assert_eq!(new_entry.accel, [0.0, 1.85, 0.0]);
    }

    #[test]
    fn lock_exclusive_serializes_calibration_and_daemon_merge() {
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::Arc;
        use std::time::Duration;

        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("imu-biases.json");
        let mut init = BiasFile::default();
        init.controllers.insert(
            "left-1111".into(),
            Entry {
                gyro: [0.01, 0.0, 0.0],
                accel: [0.0, 0.2, 0.0],
                verified_accel: None,
                updated_unix_s: 100,
            },
        );
        save(&path, &init).unwrap();

        let guard = crate::platform::fs::lock_file(&lock_path(&path)).unwrap();
        let finished = Arc::new(AtomicBool::new(false));
        let finished_t = finished.clone();
        let path_t = path.clone();
        let daemon = std::thread::spawn(move || {
            let mut updates = BTreeMap::new();
            updates.insert(
                "left-1111".into(),
                Update {
                    tracking: Some(([0.02, 0.03, 0.04], [0.0, 1.88, 0.0])),
                    verified_accel: None,
                    invalidated_from: None,
                    updated_unix_s: 210,
                },
            );
            merge_updates(&path_t, &updates).unwrap();
            finished_t.store(true, Ordering::Release);
        });
        std::thread::sleep(Duration::from_millis(40));
        assert!(
            !finished.load(Ordering::Acquire),
            "merge_updates must wait for the exclusive lock"
        );
        let mut cal = load(&path);
        cal.controllers.get_mut("left-1111").unwrap().verified_accel = Some([-0.04, 1.90, 0.10]);
        save_unlocked(&path, &cal).unwrap();
        drop(guard);

        daemon.join().unwrap();
        assert!(finished.load(Ordering::Acquire));
        let final_entry = load(&path).controllers["left-1111"];
        assert_eq!(final_entry.gyro, [0.02, 0.03, 0.04]);
        assert_eq!(final_entry.verified_accel, Some([-0.04, 1.90, 0.10]));
    }

    #[test]
    fn concurrent_calibration_and_daemon_updates_never_collide_or_lose_verified_accel() {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("imu-biases.json");
        let cal_bias = Vector3::new(-0.04, 1.90, 0.10);
        save_verified_accel(&path, &[("left-1111".into(), cal_bias)], 100).unwrap();

        let p1 = path.clone();
        let p2 = path.clone();
        let t1 = std::thread::spawn(move || {
            for i in 0..40u64 {
                save_verified_accel(&p1, &[("left-1111".into(), cal_bias)], 200 + i).unwrap();
            }
        });
        let t2 = std::thread::spawn(move || {
            for i in 0..40u64 {
                let mut updates = BTreeMap::new();
                updates.insert(
                    "left-1111".into(),
                    Update {
                        tracking: Some(([0.01, 0.02, 0.03], [-0.04, 1.89, 0.10])),
                        verified_accel: None,
                        invalidated_from: None,
                        updated_unix_s: 300 + i,
                    },
                );
                updates.insert(
                    "right-2222".into(),
                    Update {
                        tracking: Some(([-0.01, 0.0, 0.01], [0.0, 0.3, 0.0])),
                        verified_accel: None,
                        invalidated_from: None,
                        updated_unix_s: 300 + i,
                    },
                );
                merge_updates(&p2, &updates).unwrap();
            }
        });
        t1.join().unwrap();
        t2.join().unwrap();

        let final_file = load(&path);
        assert_eq!(
            final_file.controllers["left-1111"].verified_accel,
            Some([-0.04, 1.90, 0.10])
        );
        assert_eq!(final_file.controllers["left-1111"].gyro, [0.01, 0.02, 0.03]);
        assert_eq!(
            final_file.controllers["right-2222"].gyro,
            [-0.01, 0.0, 0.01]
        );
    }
}
