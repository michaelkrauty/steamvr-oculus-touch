// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Alignment of the tracked controllers with SteamVR: the quick Lighthouse registration
//! (registration.json, from `cv1touch register`) and the render-model offsets
//! (render-offsets.json, from `cv1touch fit-render`). The daemon publishes both through shared
//! memory (`cv1t_alignment`); the SteamVR driver applies them.

use std::path::{Path, PathBuf};

use nalgebra::{Isometry3, Point3, UnitQuaternion};
use serde::{de::DeserializeOwned, Deserialize, Serialize};

use crate::calib::{iso_from_xyzw, iso_to_xyzw};
use crate::ffi;
use crate::shm::{Alignment, HAND_LEFT, HAND_RIGHT};

pub const REGISTRATION_FORMAT: &str = "cv1touch-registration";
pub const RENDER_FORMAT: &str = "cv1touch-render-offsets";
pub const VERSION: u32 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PoseEntry {
    pub orient_xyzw: [f32; 4],
    pub pos: [f32; 3],
}

impl PoseEntry {
    pub fn from_iso(t: &Isometry3<f64>) -> PoseEntry {
        let (orient_xyzw, pos) = iso_to_xyzw(t);
        PoseEntry { orient_xyzw, pos }
    }

    pub fn iso(&self) -> Isometry3<f64> {
        iso_from_xyzw(self.orient_xyzw, self.pos)
    }

    fn validate(&self, what: &str) -> Result<(), String> {
        let n = self.orient_xyzw.iter().map(|v| v * v).sum::<f32>().sqrt();
        if !(n > 0.99 && n < 1.01 && self.pos.iter().all(|v| v.is_finite())) {
            return Err(format!(
                "{what}: quaternion norm {n} or position not finite"
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Registration {
    pub format: String,
    pub version: u32,
    pub created_unix_s: u64,
    /// Lighthouse raw universe <- cv1touch world.
    pub lighthouse_from_world: PoseEntry,
    /// `Prop_CurrentUniverseId_Uint64` of the reference controller.
    pub universe_id: u64,
    /// The Touch that was held ("left"/"right") and the reference controller's serial.
    pub touch: String,
    pub reference: String,
    pub pairs: usize,
    pub pos_rms_mm: f64,
    pub rot_rms_deg: f64,
    /// The base stations' raw poses when it was fitted: the Lighthouse universe it belongs to.
    #[serde(default)]
    pub stations: Vec<StationPose>,
    /// Where it was fitted (raw, m): the reference's mean position, about where the wearer plays.
    /// The universe is followed there (see `universe_shift`).
    #[serde(default)]
    pub play_area: Option<[f64; 3]>,
}

impl Registration {
    /// `play_area`, or for a registration without one, 1.2 m below the stations' middle.
    pub fn play_area_point(&self) -> Point3<f64> {
        if let Some(p) = self.play_area {
            return Point3::from(p);
        }
        let n = self.stations.len().max(1) as f64;
        let c = self
            .stations
            .iter()
            .map(|s| s.pose.iso().translation.vector)
            .sum::<nalgebra::Vector3<f64>>()
            / n;
        Point3::new(c.x, c.y - 1.2, c.z)
    }

    /// The universe's move since this registration, as of the stations `now` (see
    /// `universe_shift`).
    pub fn follow(&self, now: &[(String, Isometry3<f64>)]) -> Option<Shift> {
        let then: Vec<(String, Isometry3<f64>)> = self
            .stations
            .iter()
            .map(|s| (s.serial.clone(), s.pose.iso()))
            .collect();
        universe_shift(&then, now, &self.play_area_point(), STATIONS_AGREE_M)
    }
}

/// Stations whose moves differ by more than this over the play area (rms) do not agree: one was
/// bumped, or SteamVR's layout changed beyond following.
pub const STATIONS_AGREE_M: f64 = 0.04;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StationPose {
    pub serial: String,
    pub pose: PoseEntry,
}

/// How the Lighthouse universe moved since a registration: the transform taking poses in the old
/// universe to the new one, where it matters, in the play area.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Shift {
    pub now_from_then: Isometry3<f64>,
    /// The stations it comes from.
    pub stations: usize,
    /// How far those stations still disagree in the play area (rms, m): SteamVR re-solved its
    /// layout, so how a device is tracked now depends on which stations see it.
    pub residual_m: f64,
}

/// Half the side of the box about the play area over which stations' moves are compared (m).
const PLAY_AREA_HALF_M: f64 = 0.7;

/// Points over the play area: a 3x3x3 grid about `centre`.
fn play_area_points(centre: &Point3<f64>) -> Vec<Point3<f64>> {
    let h = PLAY_AREA_HALF_M;
    let mut v = Vec::with_capacity(27);
    for x in [-h, 0.0, h] {
        for y in [-h, 0.0, h] {
            for z in [-h, 0.0, h] {
                v.push(centre + nalgebra::Vector3::new(x, y, z));
            }
        }
    }
    v
}

/// The universe's move since `then` (the stations' raw poses when the registration was fitted)
/// as of `now`. Lighthouse devices are tracked by angles measured in each station's own frame,
/// and the stations do not move: a device a station sees moves with that station's change,
/// D_i = now_i * then_i^-1. When SteamVR just moved its universe these all agree; when it
/// re-solved its layout they differ (a station's believed pose turned a degree about itself, say,
/// which its position does not show, swings what it tracks by centimetres). The shift is the rigid
/// move closest to the D_i over a box about `play_area` (raw, then), from the largest group of
/// stations agreeing there within `agree_m` rms (a station bumped on its own is left out). None
/// with fewer than two stations in that group.
pub fn universe_shift(
    then: &[(String, Isometry3<f64>)],
    now: &[(String, Isometry3<f64>)],
    play_area: &Point3<f64>,
    agree_m: f64,
) -> Option<Shift> {
    use nalgebra::{Matrix3, Vector3};
    let each: Vec<Isometry3<f64>> = now
        .iter()
        .filter_map(|(s, n)| {
            then.iter()
                .find(|(t, _)| t == s)
                .map(|(_, t)| n * t.inverse())
        })
        .collect();
    let pts = play_area_points(play_area);
    let apart = |a: &Isometry3<f64>, b: &Isometry3<f64>| {
        (pts.iter()
            .map(|p| (a * p - b * p).norm_squared())
            .sum::<f64>()
            / pts.len() as f64)
            .sqrt()
    };
    let group: Vec<&Isometry3<f64>> = each
        .iter()
        .map(|a| {
            each.iter()
                .filter(|b| apart(a, b) < agree_m)
                .collect::<Vec<_>>()
        })
        .max_by_key(|g| g.len())?;
    if group.len() < 2 {
        return None;
    }
    // Kabsch from the play area's points to where the group's stations put them
    let src: Vec<Vector3<f64>> = group
        .iter()
        .flat_map(|_| pts.iter().map(|p| p.coords))
        .collect();
    let dst: Vec<Vector3<f64>> = group
        .iter()
        .flat_map(|d| pts.iter().map(move |p| (*d * p).coords))
        .collect();
    let k = src.len() as f64;
    let (ca, cb) = (
        src.iter().sum::<Vector3<f64>>() / k,
        dst.iter().sum::<Vector3<f64>>() / k,
    );
    let h: Matrix3<f64> = src
        .iter()
        .zip(&dst)
        .map(|(a, b)| (b - cb) * (a - ca).transpose())
        .sum();
    let svd = h.svd(true, true);
    let (u, vt) = (svd.u?, svd.v_t?);
    let d = if (u * vt).determinant() < 0.0 {
        -1.0
    } else {
        1.0
    };
    let r = u * Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, d)) * vt;
    let t = cb - r * ca;
    let residual = (src
        .iter()
        .zip(&dst)
        .map(|(a, b)| (r * a + t - b).norm_squared())
        .sum::<f64>()
        / k)
        .sqrt();
    Some(Shift {
        now_from_then: Isometry3::from_parts(t.into(), UnitQuaternion::from_matrix(&r)),
        stations: group.len(),
        residual_m: residual,
    })
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RenderOffset {
    pub hand: String,
    /// LED model frame <- SteamVR render model frame.
    pub model_from_render: PoseEntry,
    pub rms_mm: f64,
    /// LEDs the fit used (under windows the mesh models).
    pub leds: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RenderOffsets {
    pub format: String,
    pub version: u32,
    pub created_unix_s: u64,
    pub source: String,
    pub hands: Vec<RenderOffset>,
}

fn config_dir() -> PathBuf {
    crate::calib::file::default_path()
        .parent()
        .map(Path::to_path_buf)
        .unwrap_or_default()
}

pub fn registration_path() -> PathBuf {
    config_dir().join("registration.json")
}

pub fn render_path() -> PathBuf {
    config_dir().join("render-offsets.json")
}

fn read<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

pub fn load_registration(path: &Path) -> Result<Registration, String> {
    let r: Registration = read(path)?;
    if r.format != REGISTRATION_FORMAT || r.version != VERSION {
        return Err(format!(
            "{}: not a {REGISTRATION_FORMAT} v{VERSION} file",
            path.display()
        ));
    }
    r.lighthouse_from_world
        .validate(&path.display().to_string())?;
    Ok(r)
}

pub fn load_render(path: &Path) -> Result<RenderOffsets, String> {
    let r: RenderOffsets = read(path)?;
    if r.format != RENDER_FORMAT || r.version != VERSION {
        return Err(format!(
            "{}: not a {RENDER_FORMAT} v{VERSION} file",
            path.display()
        ));
    }
    for h in &r.hands {
        if h.hand != "left" && h.hand != "right" {
            return Err(format!(
                "{}: hand must be left or right, not {:?}",
                path.display(),
                h.hand
            ));
        }
        h.model_from_render.validate(&path.display().to_string())?;
    }
    Ok(r)
}

/// Writes atomically (temp file + rename); an existing file is first renamed to
/// `<name>.bak-<unix nanoseconds>`, which is returned.
pub fn save<T: Serialize>(path: &Path, value: &T) -> Result<Option<PathBuf>, String> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    let text = serde_json::to_string_pretty(value).map_err(|e| e.to_string())? + "\n";
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, text).map_err(|e| format!("{}: {e}", tmp.display()))?;
    let backup = if path.exists() {
        let mut name = path.file_name().unwrap_or_default().to_os_string();
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos());
        name.push(format!(".bak-{now}"));
        let b = path.with_file_name(name);
        std::fs::rename(path, &b).map_err(|e| format!("{}: {e}", b.display()))?;
        Some(b)
    } else {
        None
    };
    std::fs::rename(&tmp, path).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(backup)
}

pub fn alignment(reg: Option<&Registration>, render: Option<&RenderOffsets>) -> Alignment {
    let mut a = Alignment::default();
    if let Some(r) = reg {
        a.valid |= ffi::CV1T_ALIGN_LIGHTHOUSE;
        a.universe_id = r.universe_id;
        a.lighthouse_orient = r.lighthouse_from_world.orient_xyzw;
        a.lighthouse_pos = r.lighthouse_from_world.pos;
    }
    for h in render.map_or(&[][..], |r| &r.hands[..]) {
        let (hand, bit) = if h.hand == "left" {
            (HAND_LEFT, ffi::CV1T_ALIGN_RENDER_LEFT)
        } else {
            (HAND_RIGHT, ffi::CV1T_ALIGN_RENDER_RIGHT)
        };
        a.valid |= bit;
        a.render_orient[hand] = h.model_from_render.orient_xyzw;
        a.render_pos[hand] = h.model_from_render.pos;
    }
    a
}

fn optional<T>(
    path: PathBuf,
    load: fn(&Path) -> Result<T, String>,
    notes: &mut Vec<String>,
) -> Option<T> {
    if !path.exists() {
        return None;
    }
    load(&path)
        .map_err(|e| notes.push(format!("{e} (ignored)")))
        .ok()
}

/// Both default files: a missing file is simply absent; a broken one is reported and ignored.
pub fn load_default() -> (Alignment, Vec<String>) {
    load_with(None)
}

/// As `load_default`, with the registration carried by `shift` (the Lighthouse universe's move
/// since it was fitted, see `universe_shift`).
pub fn load_with(shift: Option<&Isometry3<f64>>) -> (Alignment, Vec<String>) {
    let mut notes = Vec::new();
    let mut reg = optional(registration_path(), load_registration, &mut notes);
    if let (Some(r), Some(s)) = (reg.as_mut(), shift) {
        r.lighthouse_from_world = PoseEntry::from_iso(&(s * r.lighthouse_from_world.iso()));
    }
    let render = optional(render_path(), load_render, &mut notes);
    notes.push(format!(
        "alignment: registration {}, render offsets {}",
        reg.as_ref().map_or("none".to_string(), |r| format!(
            "{} ({:.1} mm rms)",
            registration_path().display(),
            r.pos_rms_mm
        )),
        render.as_ref().map_or("none".to_string(), |r| r
            .hands
            .iter()
            .map(|h| h.hand.as_str())
            .collect::<Vec<_>>()
            .join("+"))
    ));
    (alignment(reg.as_ref(), render.as_ref()), notes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ffi;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};

    fn pose(x: f64) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(x, -0.5, 2.0),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.1, x, -0.2)),
        )
    }

    fn stations(shift: &Isometry3<f64>) -> Vec<(String, Isometry3<f64>)> {
        [
            ("LHB-A", (1.9, 2.3, -2.6, 0.1)),
            ("LHB-B", (-1.5, 2.2, -2.5, -0.4)),
            ("LHB-C", (0.0, 2.3, -3.4, 2.9)),
            ("LHB-D", (0.1, 2.4, 3.1, 1.3)),
        ]
        .iter()
        .map(|&(s, (x, y, z, yaw))| {
            (
                s.to_string(),
                shift
                    * Isometry3::from_parts(
                        Translation3::new(x, y, z),
                        UnitQuaternion::from_scaled_axis(Vector3::new(0.3, yaw, 0.05)),
                    ),
            )
        })
        .collect()
    }

    /// Where the play area is in these tests: in front of and below the stations.
    fn play() -> nalgebra::Point3<f64> {
        nalgebra::Point3::new(0.1, 1.0, -0.5)
    }

    /// A re-bootstrapped universe moves every station alike: the shift comes back exactly; a
    /// station that physically moved on its own is outvoted; one station alone, or two that
    /// disagree, are not trusted.
    #[test]
    fn the_universe_shift_comes_from_the_stations() {
        let then = stations(&Isometry3::identity());
        let shift = Isometry3::from_parts(
            Translation3::new(0.018, -0.006, 0.014),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.002, 0.009, -0.011)),
        );
        let mut now = stations(&shift);
        let s = universe_shift(&then, &now, &play(), 0.04).unwrap();
        let d = s.now_from_then.inverse() * shift;
        assert!(
            d.translation.vector.norm() < 1e-6 && d.rotation.angle() < 1e-6,
            "{d:?}"
        );
        assert_eq!(s.stations, 4);
        assert!(s.residual_m < 1e-6);
        // one station bumped by 10 cm
        now[2].1 = Isometry3::translation(0.1, 0.0, 0.0) * now[2].1;
        let s = universe_shift(&then, &now, &play(), 0.04).unwrap();
        assert_eq!(s.stations, 3);
        assert!(
            (s.now_from_then.inverse() * shift)
                .translation
                .vector
                .norm()
                < 1e-6
        );
        // nothing moved
        assert!(
            universe_shift(&then, &then, &play(), 0.04)
                .unwrap()
                .now_from_then
                .translation
                .vector
                .norm()
                < 1e-9
        );
        // one station in common is not enough
        assert!(universe_shift(&then[..1], &now[..1], &play(), 0.04).is_none());
        // two that disagree are not enough either
        let split = vec![now[0].clone(), now[2].clone()];
        assert!(universe_shift(&then, &split, &play(), 0.04).is_none());
    }

    /// SteamVR re-solving its layout can turn a station's believed pose about itself: its
    /// position stays, so the stations' positions say nothing moved, but everything that station
    /// tracks, three metres off, swings by centimetres. The follow is the stations' changes as
    /// they act on the play area, and how far they disagree there is reported.
    #[test]
    fn a_station_turned_in_place_moves_the_play_area_by_its_share() {
        let then = stations(&Isometry3::identity());
        let mut now = then.clone();
        let c = now[2].1;
        let turn = Isometry3::from_parts(
            Translation3::from(c.translation.vector),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.5f64.to_radians(), 0.0)),
        ) * Isometry3::from_parts(
            Translation3::from(-c.translation.vector),
            UnitQuaternion::identity(),
        );
        now[2].1 = turn * c;
        let s = universe_shift(&then, &now, &play(), 0.04).unwrap();
        assert_eq!(s.stations, 4);
        // a quarter of the turned station's swing at the play area
        let expected = (turn * play() - play()) / 4.0;
        let got = s.now_from_then * play() - play();
        assert!(
            (got - expected).norm() < 0.001,
            "got {got:?} expected {expected:?}"
        );
        assert!(expected.norm() > 0.005);
        assert!(
            s.residual_m > 0.005 && s.residual_m < 0.04,
            "{}",
            s.residual_m
        );
    }

    fn registration() -> Registration {
        Registration {
            format: REGISTRATION_FORMAT.into(),
            version: VERSION,
            created_unix_s: 1_791_480_000,
            lighthouse_from_world: PoseEntry::from_iso(&pose(0.3)),
            universe_id: 1_634_567_890_123,
            touch: "left".into(),
            reference: "LHR-TEST-HMD".into(),
            pairs: 5400,
            pos_rms_mm: 3.1,
            rot_rms_deg: 0.4,
            stations: Vec::new(),
            play_area: None,
        }
    }

    fn render() -> RenderOffsets {
        RenderOffsets {
            format: RENDER_FORMAT.into(),
            version: VERSION,
            created_unix_s: 1_791_480_000,
            source: "/tmp/x.rec".into(),
            hands: vec![RenderOffset {
                hand: "right".into(),
                model_from_render: PoseEntry::from_iso(&pose(-0.1)),
                rms_mm: 1.2,
                leds: 22,
            }],
        }
    }

    #[test]
    fn alignment_carries_what_the_files_say() {
        let a = alignment(Some(&registration()), Some(&render()));
        assert_eq!(
            a.valid,
            ffi::CV1T_ALIGN_LIGHTHOUSE | ffi::CV1T_ALIGN_RENDER_RIGHT
        );
        assert_eq!(a.universe_id, 1_634_567_890_123);
        let (q, p) = iso_to_xyzw(&pose(0.3));
        assert_eq!(a.lighthouse_orient, q);
        assert_eq!(a.lighthouse_pos, p);
        let (q, p) = iso_to_xyzw(&pose(-0.1));
        assert_eq!(a.render_orient[crate::shm::HAND_RIGHT], q);
        assert_eq!(a.render_pos[crate::shm::HAND_RIGHT], p);
        assert_eq!(alignment(None, None).valid, 0);
    }

    #[test]
    fn files_round_trip_and_validate() {
        let dir = std::env::temp_dir().join(format!("cv1t-align-{}", std::process::id()));
        let path = dir.join("registration.json");
        assert!(save(&path, &registration()).unwrap().is_none());
        assert_eq!(load_registration(&path).unwrap(), registration());
        assert!(save(&path, &registration()).unwrap().is_some()); // backup of the previous file
        let rpath = dir.join("render-offsets.json");
        save(&rpath, &render()).unwrap();
        assert_eq!(load_render(&rpath).unwrap(), render());
        let mut bad = registration();
        bad.lighthouse_from_world.orient_xyzw = [0.0, 0.0, 0.0, 0.0];
        save(&path, &bad).unwrap();
        assert!(load_registration(&path).unwrap_err().contains("quaternion"));
        let mut wrong = render();
        wrong.hands[0].hand = "middle".into();
        save(&rpath, &wrong).unwrap();
        assert!(load_render(&rpath).unwrap_err().contains("hand"));
        std::fs::remove_dir_all(&dir).ok();
    }
}
