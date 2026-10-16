// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Alignment of the tracked controllers with SteamVR: the quick Lighthouse registration
//! (registration.json, from `cv1touch register`) and the render-model offsets
//! (render-offsets.json, from `cv1touch fit-render`). The daemon publishes both through shared
//! memory (`cv1t_alignment`); the SteamVR driver applies them.

use std::collections::{BTreeMap, BTreeSet};
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
    /// The base station SteamVR tracked the reference from throughout the capture (its log names
    /// it; see `steamvr_log`), when known: the registration measured where that station is.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tracking_base: Option<String>,
    /// Base stations' poses in the cv1touch world as registrations measured them: this one's
    /// tracking base, and those earlier registrations against the same calibration measured.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub measured: Vec<MeasuredStation>,
}

/// A base station's pose in the cv1touch world, measured by a registration: SteamVR tracked the
/// reference from that station during the capture, so the fit put the station where it is.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeasuredStation {
    pub serial: String,
    /// cv1touch world <- station.
    pub world_from_station: PoseEntry,
    /// When the registration that measured it was fitted.
    pub registered_unix_s: u64,
    /// The `created_unix_s` of the calibration it was measured against (its world frame).
    pub calibration_unix_s: u64,
}

impl MeasuredStation {
    /// The base station SteamVR tracked the reference from during a capture, as the registration
    /// fitted to that capture measures it: where `lighthouse_from_world` puts the station's raw
    /// pose `raw` (SteamVR's, during the capture) in the world.
    pub fn from_fit(
        serial: &str,
        lighthouse_from_world: &Isometry3<f64>,
        raw: &Isometry3<f64>,
        registered_unix_s: u64,
        calibration_unix_s: u64,
    ) -> MeasuredStation {
        MeasuredStation {
            serial: serial.to_string(),
            world_from_station: PoseEntry::from_iso(&(lighthouse_from_world.inverse() * raw)),
            registered_unix_s,
            calibration_unix_s,
        }
    }
}

/// Two poses this close (0.1 mm, 0.01 degrees) are the same: a layout SteamVR reported again.
fn same_pose(a: &Isometry3<f64>, b: &Isometry3<f64>) -> bool {
    let d = a.inverse() * b;
    d.translation.vector.norm() < 1e-4 && d.rotation.angle() < 0.01f64.to_radians()
}

/// The measurements a new registration keeps: `new` (its own tracking base) and those of
/// `previous` against the same calibration for other stations; stations and sensors stay where
/// they are mounted, so a measurement holds until the calibration changes. A registration that
/// measured no station keeps none: they would be followed instead of its own fit.
pub fn merge_measured(
    previous: &[MeasuredStation],
    new: Option<MeasuredStation>,
    calibration_unix_s: u64,
) -> Vec<MeasuredStation> {
    let Some(new) = new else {
        return Vec::new();
    };
    let mut v: Vec<MeasuredStation> = previous
        .iter()
        .filter(|m| m.calibration_unix_s == calibration_unix_s && m.serial != new.serial)
        .cloned()
        .collect();
    v.push(new);
    v.sort_by(|a, b| a.serial.cmp(&b.serial));
    v
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
        universe_shift(
            &self.fit_layout(),
            now,
            &self.play_area_point(),
            STATIONS_AGREE_M,
        )
    }

    /// The base stations' raw poses when it was fitted.
    pub fn fit_layout(&self) -> Vec<(String, Isometry3<f64>)> {
        self.stations
            .iter()
            .map(|s| (s.serial.clone(), s.pose.iso()))
            .collect()
    }

    /// Where each base station is in the cv1touch world. A measured station is where its
    /// registration measured it. Another is placed by SteamVR's layouts (`layouts`, beside the one
    /// this was fitted in), each of which puts it relative to the measured stations; their mean
    /// is kept, since one layout's relation between two stations is off by 2 to 17 cm over the
    /// play area and the mean of seven by 1 cm. With none measured (a registration from before
    /// they were recorded), each station is where the fit's own layout puts it, which is right
    /// for the one SteamVR tracked the reference from.
    pub fn station_map(
        &self,
        layouts: &[Vec<(String, Isometry3<f64>)>],
    ) -> BTreeMap<String, StationWorld> {
        let world_from_lh = self.lighthouse_from_world.iso().inverse();
        let mut map: BTreeMap<String, StationWorld> = self
            .measured
            .iter()
            .map(|m| {
                (
                    m.serial.clone(),
                    StationWorld {
                        world_from_station: m.world_from_station.iso(),
                        placement: Placement::Measured,
                    },
                )
            })
            .collect();
        let fit = self.fit_layout();
        if map.is_empty() {
            for (s, p) in &fit {
                map.insert(
                    s.clone(),
                    StationWorld {
                        world_from_station: world_from_lh * p,
                        placement: Placement::Fit,
                    },
                );
            }
            return map;
        }
        let measured: Vec<(String, Isometry3<f64>)> = map
            .iter()
            .map(|(s, w)| (s.clone(), w.world_from_station))
            .collect();
        let all = std::iter::once(&fit).chain(layouts);
        let serials: BTreeSet<&String> = all.clone().flatten().map(|(s, _)| s).collect();
        for serial in serials {
            if map.contains_key(serial) {
                continue;
            }
            // one candidate per layout and measured station in it; the same relation seen twice
            // (the fit's layout is often in `layouts` too) counts once
            let mut cands: Vec<Isometry3<f64>> = Vec::new();
            let mut used = 0;
            for layout in all.clone() {
                let Some((_, st)) = layout.iter().find(|(s, _)| s == serial) else {
                    continue;
                };
                let before = cands.len();
                for (k, wk) in &measured {
                    let Some((_, sk)) = layout.iter().find(|(s, _)| s == k) else {
                        continue;
                    };
                    let c = wk * sk.inverse() * st;
                    if !cands.iter().any(|o| same_pose(o, &c)) {
                        cands.push(c);
                    }
                }
                used += usize::from(cands.len() > before);
            }
            if cands.is_empty() {
                continue;
            }
            let r = crate::calib::motion::chordal_mean(
                &cands.iter().map(|c| c.rotation).collect::<Vec<_>>(),
            );
            let t = cands
                .iter()
                .map(|c| c.translation.vector)
                .sum::<nalgebra::Vector3<f64>>()
                / cands.len() as f64;
            map.insert(
                serial.clone(),
                StationWorld {
                    world_from_station: Isometry3::from_parts(t.into(), r),
                    placement: Placement::Layouts(used),
                },
            );
        }
        map
    }

    /// How the registration applies now, with `base` the station SteamVR tracks the headset from
    /// (`layouts` place the stations no registration measured): through that station when the
    /// map places it; through the stations' moves together when there is no base or the map has
    /// no place for it. None when SteamVR does not report the base now (the registration is left
    /// as it is meanwhile, rather than followed another way for a moment), or the stations do not
    /// agree on a move.
    pub fn follow_now(
        &self,
        base: Option<&str>,
        now: &[(String, Isometry3<f64>)],
        layouts: &[Vec<(String, Isometry3<f64>)>],
    ) -> Option<Follow> {
        let map = self.station_map(layouts);
        let unplaced = match base {
            Some(b) if map.contains_key(b) => {
                return self.follow_base(b, now, &map).map(|f| Follow::Base {
                    base: b.to_string(),
                    follow: f,
                })
            }
            Some(b) => Some(b.to_string()),
            None => None,
        };
        self.follow(now)
            .map(|shift| Follow::Universe { shift, unplaced })
    }

    /// The registration as it applies while SteamVR tracks the headset from base station `base`
    /// (see `steamvr_log`): SteamVR tracks it in that station's frame, so the cv1touch world sits
    /// where that station's raw pose `now` and its world pose in `map` put it, whatever SteamVR
    /// has done to the other stations' poses. None when `now` or `map` lacks the station.
    pub fn follow_base(
        &self,
        base: &str,
        now: &[(String, Isometry3<f64>)],
        map: &BTreeMap<String, StationWorld>,
    ) -> Option<BaseFollow> {
        let (_, raw) = now.iter().find(|(s, _)| s == base)?;
        let w = map.get(base)?;
        let lighthouse_from_world = raw * w.world_from_station.inverse();
        Some(BaseFollow {
            now_from_then: lighthouse_from_world * self.lighthouse_from_world.iso().inverse(),
            placement: w.placement,
        })
    }
}

/// What the registration is followed through now (`Registration::follow_now`).
#[derive(Debug, Clone, PartialEq)]
pub enum Follow {
    /// The base station SteamVR tracks the headset from.
    Base { base: String, follow: BaseFollow },
    /// The stations' moves together: SteamVR's log names no station, or (`unplaced`) one the
    /// registration has no place for.
    Universe {
        shift: Shift,
        unplaced: Option<String>,
    },
}

impl Follow {
    /// Raw universe now <- raw universe when the registration was fitted.
    pub fn now_from_then(&self) -> Isometry3<f64> {
        match self {
            Follow::Base { follow, .. } => follow.now_from_then,
            Follow::Universe { shift, .. } => shift.now_from_then,
        }
    }

    /// The same way of following (station and how it is placed, or the universe with as many
    /// stations): what a change of is worth saying.
    pub fn same_way(&self, other: &Follow) -> bool {
        match (self, other) {
            (Follow::Base { base: a, follow: f }, Follow::Base { base: b, follow: g }) => {
                a == b && f.placement == g.placement
            }
            (
                Follow::Universe {
                    shift: a,
                    unplaced: x,
                },
                Follow::Universe {
                    shift: b,
                    unplaced: y,
                },
            ) => a.stations == b.stations && x == y,
            _ => false,
        }
    }
}

impl std::fmt::Display for Follow {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Follow::Base { base, follow } => write!(
                f,
                "base station {base}, which SteamVR tracks the headset from ({})",
                follow.placement
            ),
            Follow::Universe { shift, unplaced } => write!(
                f,
                "the stations' moves together ({} stations, disagreeing in the play area by {:.1} mm rms; {})",
                shift.stations,
                shift.residual_m * 1e3,
                match unplaced {
                    Some(b) => format!("SteamVR tracks the headset from {b}, which the registration has no place for"),
                    None => "SteamVR's log names no station it tracks the headset from".to_string(),
                }
            ),
        }
    }
}

/// How a base station's world pose in a `station_map` is known.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Placement {
    /// A registration measured it (SteamVR tracked the reference from it).
    Measured,
    /// The mean over this many of SteamVR's layouts of where they put it relative to the
    /// measured stations.
    Layouts(usize),
    /// Where the layout the registration was fitted in puts it (no station measured).
    Fit,
}

impl std::fmt::Display for Placement {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Placement::Measured => write!(f, "measured by a registration"),
            Placement::Layouts(1) => write!(f, "placed by one of SteamVR's layouts"),
            Placement::Layouts(n) => write!(f, "placed by {n} of SteamVR's layouts"),
            Placement::Fit => write!(f, "placed by the layout the registration was fitted in"),
        }
    }
}

/// A base station's pose in the cv1touch world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StationWorld {
    /// cv1touch world <- station.
    pub world_from_station: Isometry3<f64>,
    pub placement: Placement,
}

/// The registration followed through the base station SteamVR tracks the headset from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BaseFollow {
    /// Raw universe now <- raw universe when it was fitted (`Shift::now_from_then`).
    pub now_from_then: Isometry3<f64>,
    pub placement: Placement,
}

/// Base stations' raw poses by serial: a layout as SteamVR reports it.
pub type Stations = Vec<(String, Isometry3<f64>)>;

/// How long SteamVR keeps a layout before it counts as one of its estimates (s). It re-solves the
/// layout in steps while tracking starts, a few seconds apart, and the layouts the stations were
/// validated with (`Registration::station_map`) were each one SteamVR had settled on.
pub const SETTLED_S: u64 = 120;

/// Watches the layouts SteamVR reports for one it settles on (`SETTLED_S`).
#[derive(Debug, Default)]
pub struct Settling {
    current: Option<(Stations, u64)>,
    given: bool,
}

impl Settling {
    /// `stations` as SteamVR reports them at `now_s` (unix s); the layout, once, when SteamVR has
    /// kept it `SETTLED_S`.
    pub fn see(&mut self, stations: &[(String, Isometry3<f64>)], now_s: u64) -> Option<Stations> {
        let same = self
            .current
            .as_ref()
            .is_some_and(|(l, _)| same_layout(l, stations));
        if !same {
            self.current = Some((stations.to_vec(), now_s));
            self.given = false;
        }
        let (l, since) = self.current.as_ref()?;
        if self.given || now_s.saturating_sub(*since) < SETTLED_S {
            return None;
        }
        self.given = true;
        Some(l.clone())
    }
}

pub const LAYOUTS_FORMAT: &str = "cv1touch-station-layouts";
pub const LAYOUTS_VERSION: u32 = 1;
/// Layouts kept per universe; the oldest go first.
const MAX_LAYOUTS: usize = 64;

/// The base station layouts SteamVR has reported (each time it re-solves them, it reports a new
/// one), which place the stations no registration measured (`Registration::station_map`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Layouts {
    pub format: String,
    pub version: u32,
    pub universes: Vec<UniverseLayouts>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UniverseLayouts {
    pub universe_id: u64,
    pub layouts: Vec<Layout>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Layout {
    pub seen_unix_s: u64,
    pub stations: Vec<StationPose>,
}

impl Default for Layouts {
    fn default() -> Self {
        Layouts {
            format: LAYOUTS_FORMAT.into(),
            version: LAYOUTS_VERSION,
            universes: Vec::new(),
        }
    }
}

/// Whether two layouts put every station within 0.1 mm and 0.01 deg of each other.
fn same_layout(a: &[(String, Isometry3<f64>)], b: &[(String, Isometry3<f64>)]) -> bool {
    a.len() == b.len()
        && a.iter()
            .all(|(s, p)| b.iter().any(|(t, q)| s == t && same_pose(p, q)))
}

impl Layouts {
    /// Universe `universe_id`'s layouts, oldest first.
    pub fn of(&self, universe_id: u64) -> Vec<Vec<(String, Isometry3<f64>)>> {
        self.universes
            .iter()
            .filter(|u| u.universe_id == universe_id)
            .flat_map(|u| &u.layouts)
            .map(|l| {
                l.stations
                    .iter()
                    .map(|s| (s.serial.clone(), s.pose.iso()))
                    .collect()
            })
            .collect()
    }

    /// Adds `stations` (two or more) as universe `universe_id`'s newest layout unless it has the
    /// layout already (SteamVR started again without re-solving it is the same layout); true when
    /// added.
    pub fn record(
        &mut self,
        universe_id: u64,
        seen_unix_s: u64,
        stations: &[(String, Isometry3<f64>)],
    ) -> bool {
        if stations.len() < 2 {
            return false;
        }
        let i = match self
            .universes
            .iter()
            .position(|u| u.universe_id == universe_id)
        {
            Some(i) => i,
            None => {
                self.universes.push(UniverseLayouts {
                    universe_id,
                    layouts: Vec::new(),
                });
                self.universes.len() - 1
            }
        };
        let u = &mut self.universes[i];
        if u.layouts.iter().any(|l| {
            let l: Vec<(String, Isometry3<f64>)> = l
                .stations
                .iter()
                .map(|s| (s.serial.clone(), s.pose.iso()))
                .collect();
            same_layout(&l, stations)
        }) {
            return false;
        }
        let mut sorted: Vec<StationPose> = stations
            .iter()
            .map(|(s, p)| StationPose {
                serial: s.clone(),
                pose: PoseEntry::from_iso(p),
            })
            .collect();
        sorted.sort_by(|a, b| a.serial.cmp(&b.serial));
        u.layouts.push(Layout {
            seen_unix_s,
            stations: sorted,
        });
        if u.layouts.len() > MAX_LAYOUTS {
            u.layouts.remove(0);
        }
        true
    }
}

pub fn layouts_path() -> PathBuf {
    crate::platform::dirs::get()
        .state
        .join("station-layouts.json")
}

/// The layouts on file (none when there is no file); an error when it cannot be read or is not
/// a layouts file this version reads.
pub fn read_layouts(path: &Path) -> Result<Layouts, String> {
    if !path.exists() {
        return Ok(Layouts::default());
    }
    let l: Layouts = read(path)?;
    if l.format != LAYOUTS_FORMAT || l.version != LAYOUTS_VERSION {
        return Err(format!(
            "{}: not a {LAYOUTS_FORMAT} v{LAYOUTS_VERSION} file",
            path.display()
        ));
    }
    Ok(l)
}

/// The layouts on file to place stations with: none when there is no file or it cannot be read.
pub fn load_layouts(path: &Path) -> Layouts {
    read_layouts(path).unwrap_or_default()
}

/// Writes atomically (a temp file of this process's own, renamed over it), with no backup: the
/// file only ever grows from what SteamVR reports.
pub fn save_layouts(path: &Path, layouts: &Layouts) -> Result<(), String> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    let text = serde_json::to_string(layouts).map_err(|e| e.to_string())? + "\n";
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_nanos());
    let tmp = path.with_extension(format!("json.tmp-{}-{nanos}", std::process::id()));
    std::fs::write(&tmp, text).map_err(|e| format!("{}: {e}", tmp.display()))?;
    std::fs::rename(&tmp, path).map_err(|e| {
        std::fs::remove_file(&tmp).ok();
        format!("{}: {e}", path.display())
    })
}

/// Adds `stations` to the layouts on file at `path` (as `Layouts::record`), reading the file
/// afresh so what another process added stays; true when it was new. Returns the layouts on file.
/// A file that cannot be read is left as it is (an error), not written over.
pub fn record_layout(
    path: &Path,
    universe_id: u64,
    seen_unix_s: u64,
    stations: &[(String, Isometry3<f64>)],
) -> Result<(bool, Layouts), String> {
    let mut l = read_layouts(path)?;
    if !l.record(universe_id, seen_unix_s, stations) {
        return Ok((false, l));
    }
    save_layouts(path, &l)?;
    Ok((true, l))
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
    for m in &r.measured {
        m.world_from_station
            .validate(&format!("{}: {}", path.display(), m.serial))?;
    }
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

    fn iso(t: (f64, f64, f64), r: (f64, f64, f64)) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(t.0, t.1, t.2),
            UnitQuaternion::from_scaled_axis(Vector3::new(r.0, r.1, r.2)),
        )
    }

    /// How far apart two world-to-raw transforms put the play area's box (rms, m).
    fn apart(a: &Isometry3<f64>, b: &Isometry3<f64>) -> f64 {
        let pts = play_area_points(&play());
        (pts.iter()
            .map(|p| (a * p - b * p).norm_squared())
            .sum::<f64>()
            / pts.len() as f64)
            .sqrt()
    }

    /// The stations' true poses in the world, and a registration fitted while SteamVR tracked
    /// the headset from LHB-A: its raw universe then is `universe_then` for LHB-A, while SteamVR's
    /// layout has the other stations a little off (`errs`, in their own frames).
    fn fitted(
        universe_then: &Isometry3<f64>,
        errs: &[Isometry3<f64>],
    ) -> (Vec<(String, Isometry3<f64>)>, Registration) {
        let world = stations(&Isometry3::identity());
        let then: Vec<(String, Isometry3<f64>)> = world
            .iter()
            .zip(std::iter::once(Isometry3::identity()).chain(errs.iter().copied()))
            .map(|((s, w), e)| (s.clone(), universe_then * w * e))
            .collect();
        let mut r = registration();
        r.lighthouse_from_world = PoseEntry::from_iso(universe_then);
        r.stations = then
            .iter()
            .map(|(s, p)| StationPose {
                serial: s.clone(),
                pose: PoseEntry::from_iso(p),
            })
            .collect();
        r.play_area = Some(play().coords.into());
        r.tracking_base = Some("LHB-A".into());
        r.measured = vec![MeasuredStation::from_fit(
            "LHB-A",
            universe_then,
            &then[0].1,
            r.created_unix_s,
            7,
        )];
        (world, r)
    }

    /// SteamVR tracks the headset in the frame of the station it bootstrapped from: when it
    /// re-solves the layout, the headset moves with that station alone. The registration follows
    /// that station exactly, where a fit to all the stations' moves (`universe_shift`) is
    /// centimetres out.
    #[test]
    fn the_registration_follows_the_station_steamvr_tracks_the_headset_from() {
        let u_then = iso((0.3, -0.7, -2.4), (0.0, 2.3, 0.0));
        let (world, reg) = fitted(
            &u_then,
            &[
                iso((0.03, 0.0, -0.02), (0.0, 0.01, 0.0)),
                iso((0.0, 0.02, 0.0), (0.005, 0.0, 0.0)),
                iso((-0.02, 0.0, 0.01), (0.0, 0.0, -0.01)),
            ],
        );
        let map = reg.station_map(&[]);
        // the measurement is where the station is, and the capture's own layout is no move
        let d = map["LHB-A"].world_from_station.inverse() * world[0].1;
        assert!(d.translation.vector.norm() < 1e-5 && d.rotation.angle() < 1e-5);
        let own = reg.follow_base("LHB-A", &reg.fit_layout(), &map).unwrap();
        assert!(apart(&own.now_from_then, &Isometry3::identity()) < 1e-5);
        // a new session: the universe sits elsewhere and the others are re-solved differently
        let u_now = iso((0.25, -0.68, -2.45), (0.01, 2.33, -0.02));
        let now: Vec<(String, Isometry3<f64>)> = world
            .iter()
            .zip([
                Isometry3::identity(),
                iso((-0.05, 0.01, 0.0), (0.0, -0.02, 0.0)),
                iso((0.0, 0.0, 0.04), (0.01, 0.0, 0.0)),
                iso((0.02, -0.03, 0.0), (0.0, 0.0, 0.015)),
            ])
            .map(|((s, w), e)| (s.clone(), u_now * w * e))
            .collect();
        let f = reg.follow_base("LHB-A", &now, &map).unwrap();
        assert_eq!(f.placement, Placement::Measured);
        let followed = f.now_from_then * reg.lighthouse_from_world.iso();
        assert!(
            apart(&followed, &u_now) < 1e-5,
            "{}",
            apart(&followed, &u_now)
        );
        let blended = reg.follow(&now).unwrap().now_from_then * reg.lighthouse_from_world.iso();
        assert!(
            apart(&blended, &u_now) > 0.01,
            "{}",
            apart(&blended, &u_now)
        );
        // a station SteamVR does not report, or one the map lacks: no follow
        assert!(reg.follow_base("LHB-A", &now[1..], &map).is_none());
        assert!(reg.follow_base("LHB-X", &now, &map).is_none());
        // what the watch follows: the station when it can; the stations' moves together when
        // there is no base or the registration has no place for it; nothing for a moment when
        // SteamVR does not report the base, rather than another way
        let by_base = reg.follow_now(Some("LHB-A"), &now, &[]).unwrap();
        assert!(matches!(&by_base, Follow::Base { base, .. } if base == "LHB-A"));
        assert_eq!(by_base.now_from_then(), f.now_from_then);
        let moved = iso((0.01, 0.0, -0.02), (0.0, 0.005, 0.0));
        let agreeing: Vec<(String, Isometry3<f64>)> = reg
            .fit_layout()
            .iter()
            .map(|(s, p)| (s.clone(), moved * p))
            .collect();
        for (base, unplaced) in [(None, None), (Some("LHB-X"), Some("LHB-X".to_string()))] {
            match reg.follow_now(base, &agreeing, &[]) {
                Some(u @ Follow::Universe { .. }) => {
                    assert!(
                        matches!(&u, Follow::Universe { shift, unplaced: x } if shift.stations == 4 && *x == unplaced)
                    );
                    assert!(apart(&u.now_from_then(), &moved) < 1e-5);
                    assert!(!u.same_way(&by_base));
                }
                other => panic!("{base:?}: {other:?}"),
            }
        }
        assert!(reg.follow_now(Some("LHB-A"), &agreeing[1..], &[]).is_none());
    }

    /// A station no registration measured is placed by SteamVR's layouts relative to the measured
    /// one: each layout's relation is off, by errors that scatter about the truth, and their mean
    /// is far closer than any one of them. A layout seen twice counts once.
    #[test]
    fn a_station_not_measured_is_placed_by_the_mean_of_steamvrs_layouts() {
        let u = iso((0.3, -0.7, -2.4), (0.0, 2.3, 0.0));
        let e = |k: f64| {
            [
                iso((0.06 * k, 0.0, 0.0), (0.0, 0.03 * k, 0.0)),
                iso((0.0, -0.04 * k, 0.0), (0.02 * k, 0.0, 0.0)),
                iso((0.0, 0.0, 0.05 * k), (0.0, 0.0, -0.025 * k)),
            ]
        };
        let (world, reg) = fitted(&u, &e(1.0));
        let layout = |k: f64, at: &Isometry3<f64>| -> Vec<(String, Isometry3<f64>)> {
            world
                .iter()
                .zip(std::iter::once(Isometry3::identity()).chain(e(k)))
                .map(|((s, w), x)| (s.clone(), at * w * x))
                .collect()
        };
        let elsewhere = iso((-0.1, 0.05, 0.2), (0.0, -0.4, 0.01));
        // the fit's layout (+1), its mirror image (-1), and the fit's layout again
        let layouts = vec![layout(-1.0, &elsewhere), layout(1.0, &u)];
        let map = reg.station_map(&layouts);
        for (i, (s, w)) in world.iter().enumerate().skip(1) {
            let m = map[s];
            assert_eq!(m.placement, Placement::Layouts(2), "{s}");
            let d = m.world_from_station.inverse() * w;
            let single = reg.station_map(&[]);
            let ds = single[s].world_from_station.inverse() * w;
            assert!(
                d.translation.vector.norm() < 0.003 && d.rotation.angle() < 0.003,
                "{s}: {d:?}"
            );
            assert!(
                ds.translation.vector.norm() > 0.03 || ds.rotation.angle() > 0.015,
                "{i}"
            );
        }
        assert_eq!(map["LHB-A"].placement, Placement::Measured);
        // with two stations measured, each layout places the others twice: still two layouts
        let mut two = reg.clone();
        two.measured.push(MeasuredStation {
            serial: world[1].0.clone(),
            world_from_station: PoseEntry::from_iso(&world[1].1),
            registered_unix_s: 0,
            calibration_unix_s: 7,
        });
        let map = two.station_map(&layouts);
        assert_eq!(map[&world[1].0].placement, Placement::Measured);
        assert_eq!(map[&world[2].0].placement, Placement::Layouts(2));
    }

    /// A registration from before the tracking base was recorded measured none: every station is
    /// where the fit's own layout puts it, so following one is following its own change since.
    #[test]
    fn with_no_station_measured_each_is_where_the_fit_layout_puts_it() {
        let u = iso((0.3, -0.7, -2.4), (0.0, 2.3, 0.0));
        let (_, mut reg) = fitted(&u, &[iso((0.03, 0.0, 0.0), (0.0, 0.01, 0.0)); 3]);
        reg.measured.clear();
        reg.tracking_base = None;
        let map = reg.station_map(&[]);
        assert!(map.values().all(|m| m.placement == Placement::Fit));
        let then = reg.fit_layout();
        let moved = iso((0.01, 0.0, -0.02), (0.0, 0.005, 0.0));
        let now: Vec<(String, Isometry3<f64>)> =
            then.iter().map(|(s, p)| (s.clone(), moved * p)).collect();
        let f = reg.follow_base("LHB-C", &now, &map).unwrap();
        assert_eq!(f.placement, Placement::Fit);
        assert!(apart(&f.now_from_then, &moved) < 1e-5);
    }

    #[test]
    fn measurements_hold_until_the_calibration_changes() {
        let m = |s: &str, cal: u64, at: u64| MeasuredStation {
            serial: s.into(),
            world_from_station: PoseEntry::from_iso(&pose(at as f64 * 0.01)),
            registered_unix_s: at,
            calibration_unix_s: cal,
        };
        let previous = [m("LHB-A", 1, 1), m("LHB-B", 1, 2), m("LHB-C", 0, 3)];
        let merged = merge_measured(&previous, Some(m("LHB-A", 1, 9)), 1);
        assert_eq!(merged, vec![m("LHB-A", 1, 9), m("LHB-B", 1, 2)]);
        assert_eq!(
            merge_measured(&previous, Some(m("LHB-D", 1, 9)), 1),
            vec![m("LHB-A", 1, 1), m("LHB-B", 1, 2), m("LHB-D", 1, 9)]
        );
        assert_eq!(
            merge_measured(&previous, Some(m("LHB-D", 5, 9)), 5),
            vec![m("LHB-D", 5, 9)]
        );
        // measuring none, a registration keeps none
        assert_eq!(merge_measured(&previous, None, 1), vec![]);
    }

    #[test]
    fn a_layout_counts_once_steamvr_has_kept_it() {
        let a = stations(&Isometry3::identity());
        let mut b = a.clone();
        b[0].1 = Isometry3::translation(0.11, 0.0, 0.0) * b[0].1;
        let mut s = Settling::default();
        // re-solved within seconds while tracking starts: neither counts
        assert_eq!(s.see(&a, 1000), None);
        assert_eq!(s.see(&b, 1003), None);
        assert_eq!(s.see(&b, 1003 + SETTLED_S - 1), None);
        assert_eq!(s.see(&b, 1003 + SETTLED_S), Some(b.clone()));
        assert_eq!(s.see(&b, 1003 + 5 * SETTLED_S), None);
        assert_eq!(s.see(&a, 2000), None);
        assert_eq!(s.see(&a, 2000 + SETTLED_S), Some(a.clone()));
    }

    #[test]
    fn each_new_layout_is_recorded_once() {
        let mut l = Layouts::default();
        let a = stations(&Isometry3::identity());
        assert!(l.record(1, 10, &a));
        assert!(!l.record(1, 11, &a));
        assert!(!l.record(1, 12, &a[..1]));
        let mut b = a.clone();
        b[1].1 = Isometry3::translation(0.0, 0.0, 0.001) * b[1].1;
        assert!(l.record(1, 13, &b));
        // back to an earlier layout: not new
        assert!(!l.record(1, 14, &a));
        assert!(l.record(2, 14, &a));
        assert_eq!(l.of(1).len(), 2);
        assert_eq!(l.of(2).len(), 1);
        for k in 0..MAX_LAYOUTS + 5 {
            let mut c = a.clone();
            c[0].1 = Isometry3::translation(0.01 * k as f64, 0.0, 0.0) * c[0].1;
            l.record(1, 100 + k as u64, &c);
        }
        assert_eq!(l.of(1).len(), MAX_LAYOUTS);
        let dir = std::env::temp_dir().join(format!("cv1t-layouts-{}", std::process::id()));
        let path = dir.join("station-layouts.json");
        assert_eq!(load_layouts(&path), Layouts::default());
        save_layouts(&path, &l).unwrap();
        assert_eq!(load_layouts(&path), l);
        // recorded through the file: what is on file stays, a layout on file is not new
        let (added, on_file) = record_layout(&path, 3, 1, &a).unwrap();
        assert!(added && on_file.of(1) == l.of(1) && on_file.of(3).len() == 1);
        assert!(!record_layout(&path, 3, 2, &a).unwrap().0);
        // a file it cannot read is not written over
        std::fs::write(&path, "{ not json").unwrap();
        assert!(record_layout(&path, 3, 3, &b).is_err());
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "{ not json");
        assert_eq!(load_layouts(&path), Layouts::default());
        std::fs::remove_dir_all(&dir).ok();
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
            tracking_base: None,
            measured: Vec::new(),
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

        // with no tracking base or measurements, the file is as before they existed
        assert!(!std::fs::read_to_string(&path).unwrap().contains("measured"));
        let (_, measured) = fitted(&pose(0.2), &[Isometry3::identity(); 3]);
        save(&path, &measured).unwrap();
        assert_eq!(load_registration(&path).unwrap(), measured);
        let mut broken = measured.clone();
        broken.measured[0].world_from_station.orient_xyzw = [0.0; 4];
        save(&path, &broken).unwrap();
        assert!(load_registration(&path).unwrap_err().contains("LHB-A"));
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
