// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Which base station SteamVR starts tracking the headset from. A headset registration measures
//! where the station SteamVR tracked the headset from is (`align::Registration::measured`); any
//! other is placed only by SteamVR's own layouts, about a centimetre off. SteamVR picks the
//! station itself on each start, unless its setting `driver_lighthouse.primarybasestation`
//! (the station's serial as a signed 32-bit number) names one: then it starts from that one
//! only, and waits for it when it is not in sight ("Saw N bases and none are the preferred base
//! X"). The Lighthouse driver reads the setting once, when SteamVR starts, so a change applies
//! from the next start; a change during a session leaves that session alone.
//!
//! The SteamVR driver reports the setting and cv1touch's two records of it in SteamVR's settings
//! every 2 s and writes what `decide` answers: a station a registration measured and SteamVR sees
//! now, never with a single station, and never over a value someone else set.

use std::time::{Duration, Instant};

/// SteamVR's setting, and cv1touch's records in its own driver section.
pub const SECTION: &str = "driver_lighthouse";
pub const KEY: &str = "primarybasestation";
pub const OWN_SECTION: &str = "driver_cv1touch";
/// The value cv1touch set: the setting is cv1touch's while it holds this value.
pub const OURS_KEY: &str = "pinned_base";
/// The station whose pin was removed because SteamVR waited for it (see `at_quit`).
pub const UNPINNED_KEY: &str = "unpinned_base";

/// How long SteamVR waits for the pinned station, with the headset untracked, before `status` and
/// `doctor` say to restart SteamVR: quitting it removes the pin (`at_quit`).
pub const NOTE_AFTER: Duration = Duration::from_secs(30);

/// The setting and the two records, 0 when not set.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Keys {
    pub pin: i32,
    pub ours: i32,
    pub unpinned: i32,
}

impl Keys {
    /// Whether the setting holds a value cv1touch did not set.
    pub fn users(&self) -> bool {
        self.pin != 0 && self.pin != self.ours
    }
}

/// The setting's value for a base station's serial (`LHB-` and 8 hex digits).
pub fn value(serial: &str) -> Option<i32> {
    let hex = serial.strip_prefix("LHB-")?;
    if hex.len() != 8 {
        return None;
    }
    u32::from_str_radix(hex, 16)
        .ok()
        .map(|v| v as i32)
        .filter(|&v| v != 0)
}

/// The base station's serial for a value of the setting.
pub fn serial(value: i32) -> String {
    format!("LHB-{:08X}", value as u32)
}

/// What the decision looks at besides the keys.
#[derive(Debug, Clone, Copy)]
pub struct Seen<'a> {
    /// Pinning is on (cv1touch's setting `pin_base_station`).
    pub enabled: bool,
    /// The base stations SteamVR sees now.
    pub stations: &'a [String],
    /// The stations a headset registration measured, newest first.
    pub measured: &'a [String],
    /// The station SteamVR tracks the headset from now, if it does.
    pub headset_base: Option<&'a str>,
    /// Whether SteamVR started tracking the headset from that station by its own choice, rather
    /// than because the setting named it.
    pub chosen: bool,
    /// The station SteamVR has been waiting for, in a session that has not tracked the headset
    /// (`Watch`), and for how long.
    pub waited: Option<(&'a str, Duration)>,
}

/// How long SteamVR has waited for cv1touch's pin `k`, if it waits for it.
fn waited_for_ours(k: Keys, s: &Seen) -> Option<Duration> {
    s.waited
        .filter(|(p, _)| k.pin != 0 && k.pin == k.ours && value(p) == Some(k.pin))
        .map(|(_, w)| w)
}

/// cv1touch's pin `k` removed, its station remembered.
fn unpin(k: Keys) -> Keys {
    Keys {
        pin: 0,
        ours: 0,
        unpinned: k.pin,
    }
}

/// What the setting and the records should be. A value someone else set is never touched; a record
/// of a value the setting no longer holds is dropped. Turned off: cv1touch's pin goes. A pin
/// SteamVR waits for stays until SteamVR quits (`at_quit`); a station whose pin was removed that
/// way is not pinned again until SteamVR starts from it by its own choice, so a headset kept out
/// of its sight does not swing between pinned and not. While SteamVR tracks the headset and sees
/// more than one station: the measured, seen station it tracks the headset from, else the current
/// pin if it still qualifies, else the newest measured station seen; none qualifying, no pin.
pub fn decide(keys: Keys, s: &Seen) -> Keys {
    let mut k = keys;
    if k.ours != k.pin {
        k.ours = 0;
    }
    if k.users() {
        return k;
    }
    if s.chosen && s.headset_base.and_then(value) == Some(k.unpinned) {
        k.unpinned = 0;
    }
    if !s.enabled {
        return Keys::default();
    }
    let Some(base) = s.headset_base else {
        return k;
    };
    if s.stations.len() < 2 {
        return k;
    }
    let current = (k.pin != 0).then(|| serial(k.pin));
    let pin = std::iter::once(base)
        .chain(current.as_deref())
        .chain(s.measured.iter().map(String::as_str))
        .filter(|x| s.measured.iter().any(|m| m == x) && s.stations.iter().any(|t| t == x))
        .filter_map(value)
        .find(|&v| v != k.unpinned)
        .unwrap_or(0);
    Keys {
        pin,
        ours: pin,
        unpinned: k.unpinned,
    }
}

/// The answer to the driver's last report, as SteamVR quits: when SteamVR is waiting for
/// cv1touch's pin in a session that has not tracked the headset, the pin goes and its station is
/// remembered (see `decide`), however short the wait, so the next start is free; otherwise
/// nothing changes.
pub fn at_quit(keys: Keys, s: &Seen) -> Keys {
    if waited_for_ours(keys, s).is_some() {
        unpin(keys)
    } else {
        keys
    }
}

/// The stations a registration measured (`Registration::measured`), newest first.
pub fn measured(stations: &[crate::align::MeasuredStation]) -> Vec<String> {
    let mut m: Vec<&crate::align::MeasuredStation> = stations.iter().collect();
    m.sort_by_key(|m| std::cmp::Reverse(m.registered_unix_s));
    m.into_iter().map(|m| m.serial.clone()).collect()
}

/// What the daemon follows between the driver's reports: SteamVR's log as it says how the
/// headset is tracked, and since when SteamVR waits for a station in a session that has not
/// tracked the headset (a wait after the headset was tracked is the headset out of sight, not a
/// pin that keeps SteamVR from tracking).
#[derive(Debug, Default)]
pub struct Watch {
    pub enabled: bool,
    pub measured: Vec<String>,
    pub headset_base: Option<String>,
    pub chosen: bool,
    waiting: Option<(String, Instant)>,
}

impl Watch {
    pub fn new(enabled: bool) -> Watch {
        Watch {
            enabled,
            ..Default::default()
        }
    }

    /// Takes in what SteamVR's log says now (`steamvr_log::Bases`), seen at `now`.
    pub fn see(&mut self, bases: &crate::steamvr_log::Bases, now: Instant) {
        self.headset_base = bases.of_headset().map(str::to_string);
        self.chosen = self.headset_base.is_some() && !bases.headset_preferred;
        let waiting = bases
            .waiting_for
            .as_ref()
            .filter(|_| !bases.headset_tracked());
        match (waiting, &self.waiting) {
            (Some(p), Some((q, _))) if p == q => {}
            (Some(p), _) => self.waiting = Some((p.clone(), now)),
            (None, _) => self.waiting = None,
        }
    }

    /// The station SteamVR waits for and how long it has, at `now`.
    pub fn waited(&self, now: Instant) -> Option<(&str, Duration)> {
        self.waiting
            .as_ref()
            .map(|(p, since)| (p.as_str(), now.saturating_duration_since(*since)))
    }

    pub fn seen<'a>(&'a self, stations: &'a [String], now: Instant) -> Seen<'a> {
        Seen {
            enabled: self.enabled,
            stations,
            measured: &self.measured,
            headset_base: self.headset_base.as_deref(),
            chosen: self.chosen,
            waited: self.waited(now),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: &str = "LHB-0000000A";
    const B: &str = "LHB-0000000B";
    const C: &str = "LHB-0000000C";
    const X: &str = "LHB-0000000F";

    fn keys(pin: &str, ours: &str, unpinned: &str) -> Keys {
        let v = |s: &str| value(s).unwrap_or(0);
        Keys {
            pin: v(pin),
            ours: v(ours),
            unpinned: v(unpinned),
        }
    }

    #[test]
    fn serials_and_values() {
        assert_eq!(value("LHB-2B1973A2"), Some(723088290));
        assert_eq!(value("LHB-C1E62875"), Some(-1041880971));
        assert_eq!(serial(-1041880971), "LHB-C1E62875");
        assert_eq!(serial(305419896), "LHB-12345678");
        for bad in [
            "LHB-0000000",
            "LHB-00000000",
            "LHR-2B1973A2",
            "LHB-2B1973AZ",
            "",
        ] {
            assert_eq!(value(bad), None, "{bad}");
        }
    }

    #[test]
    fn the_decision() {
        let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        let all = s(&[A, B, C]);
        let ab = s(&[A, B]);
        let ac = s(&[A, C]);
        let cx = s(&[C, X]);
        let a = s(&[A]);
        let measured = s(&[B, A]); // B newest
        let none: Vec<String> = Vec::new();
        let long = NOTE_AFTER;
        let longer = Duration::from_secs(3600);
        let seen = |stations, headset_base, chosen, waited| Seen {
            enabled: true,
            stations,
            measured: &measured,
            headset_base,
            chosen,
            waited,
        };
        #[rustfmt::skip]
        let table: Vec<(&str, Keys, Seen, Keys)> = vec![
            ("tracked from a measured, seen station: pinned", keys("", "", ""), seen(&all, Some(A), true, None), keys(A, A, "")),
            ("tracked from an unmeasured one: the newest measured seen", keys("", "", ""), seen(&all, Some(C), true, None), keys(B, B, "")),
            ("the newest measured not seen: the next", keys("", "", ""), seen(&ac, Some(C), true, None), keys(A, A, "")),
            ("our pin still qualifies: kept over the newest", keys(A, A, ""), seen(&all, Some(C), false, None), keys(A, A, "")),
            ("our pin, tracked from another measured: that one", keys(A, A, ""), seen(&all, Some(B), true, None), keys(B, B, "")),
            ("no measured station seen: our pin removed", keys(A, A, ""), seen(&cx, Some(C), true, None), keys("", "", "")),
            ("no registration measured one: nothing", keys("", "", ""), Seen { measured: &none, ..seen(&all, Some(A), true, None) }, keys("", "", "")),
            ("one station in all: nothing written", keys("", "", ""), seen(&a, Some(A), true, None), keys("", "", "")),
            ("one station, our pin: left", keys(B, B, ""), seen(&a, Some(A), true, None), keys(B, B, "")),
            ("headset not tracked: nothing", keys("", "", ""), seen(&all, None, false, None), keys("", "", "")),
            ("the user's value: never touched", keys(X, "", ""), seen(&all, Some(A), true, None), keys(X, "", "")),
            ("the user's value, waited for: never touched", keys(X, "", ""), seen(&all, None, false, Some((X, long))), keys(X, "", "")),
            ("the user changed ours: theirs now, the record goes", keys(C, A, ""), seen(&all, Some(A), true, None), keys(C, "", "")),
            ("the user removed ours: the record goes, then pinned again", keys("", A, ""), seen(&all, Some(A), true, None), keys(A, A, "")),
            ("waited 30 s for our pin in a session: kept until SteamVR quits", keys(A, A, ""), seen(&ab, None, false, Some((A, long))), keys(A, A, "")),
            ("waited an hour for our pin in a session: kept until SteamVR quits", keys(A, A, ""), seen(&ab, None, false, Some((A, longer))), keys(A, A, "")),
            ("waited for a station that is not our pin now: kept", keys(B, B, ""), seen(&ab, None, false, Some((A, long))), keys(B, B, "")),
            ("a removed pin, tracked from it because it was preferred: not again", keys("", "", A), seen(&all, Some(A), false, None), keys(B, B, A)),
            ("a removed pin, the only measured seen: none", keys("", "", A), seen(&ac, Some(A), false, None), keys("", "", A)),
            ("a removed pin, SteamVR's own choice: pinned again", keys("", "", A), seen(&all, Some(A), true, None), keys(A, A, "")),
            ("turned off: our pin and records go", keys(A, A, B), Seen { enabled: false, ..seen(&all, Some(A), true, None) }, keys("", "", "")),
            ("turned off, the user's value: left", keys(X, "", ""), Seen { enabled: false, ..seen(&all, Some(A), true, None) }, keys(X, "", "")),
        ];
        for (what, k, s, want) in table {
            assert_eq!(decide(k, &s), want, "{what}");
        }
        // the driver's last report, as SteamVR quits (`waited` is only ever set in a session that
        // has not tracked the headset: `Watch`)
        let brief = Duration::from_secs(3);
        #[rustfmt::skip]
        let at_quit_table: Vec<(&str, Keys, Seen, Keys)> = vec![
            ("waited 3 s for our pin, then quit: removed and remembered", keys(A, A, ""), seen(&ab, None, false, Some((A, brief))), keys("", "", A)),
            ("waited for our pin, then quit, turned off: removed and remembered", keys(A, A, ""), Seen { enabled: false, ..seen(&ab, None, false, Some((A, brief))) }, keys("", "", A)),
            ("quit while not waiting: nothing, though another station would be pinned", keys(A, A, ""), seen(&all, Some(B), true, None), keys(A, A, "")),
            ("quit while waiting for a station that is not our pin: nothing", keys(B, B, ""), seen(&ab, None, false, Some((A, brief))), keys(B, B, "")),
            ("the user's value at quit, waited for: untouched", keys(X, "", ""), seen(&all, None, false, Some((X, brief))), keys(X, "", "")),
            ("the user's value equal to a remembered station at quit: untouched", keys(A, "", A), seen(&all, None, false, Some((A, brief))), keys(A, "", A)),
        ];
        for (what, k, s, want) in at_quit_table {
            assert_eq!(at_quit(k, &s), want, "{what}");
        }
    }

    #[test]
    fn measured_newest_first() {
        use crate::align::{MeasuredStation, PoseEntry};
        let pose = PoseEntry::from_iso(&nalgebra::Isometry3::identity());
        let m = |serial: &str, at: u64| MeasuredStation {
            serial: serial.into(),
            world_from_station: pose,
            registered_unix_s: at,
            calibration_unix_s: 1,
        };
        assert_eq!(measured(&[]), Vec::<String>::new());
        assert_eq!(measured(&[m(A, 10), m(B, 30), m(C, 20)]), [B, C, A]);
    }

    #[test]
    fn the_watch_times_the_wait_from_when_the_log_first_says_it() {
        let mut bases = crate::steamvr_log::Bases::default();
        let mut w = Watch::default();
        let t0 = Instant::now();
        let line = |s: &str| crate::steamvr_log::parse_line(s).unwrap();
        for e in [
            "x lighthouse: LHR-HEADSET H: Saw 1 bases and none are the preferred base 0000000A",
            "x lighthouse: LHR-HEADSET H: Saw 3 bases and none are the preferred base 0000000A",
        ]
        .iter()
        .enumerate()
        {
            bases.apply(&line(e.1));
            w.see(&bases, t0 + Duration::from_secs(5 * e.0 as u64));
        }
        assert_eq!(
            w.waited(t0 + Duration::from_secs(31)),
            Some((A, Duration::from_secs(31)))
        );
        assert_eq!(w.headset_base, None);
        bases.apply(&line(
            "x lighthouse: LHR-HEADSET H: ----- BOOTSTRAPPED base 0000000A (preferred) distance 3.31m",
        ));
        w.see(&bases, t0 + Duration::from_secs(32));
        assert_eq!(w.waited(t0 + Duration::from_secs(40)), None);
        assert_eq!((w.headset_base.as_deref(), w.chosen), (Some(A), false));
        bases.apply(&line(
            "x lighthouse: LHR-HEADSET H: ----- BOOTSTRAPPED base 0000000B (best) distance 3.31m",
        ));
        w.see(&bases, t0 + Duration::from_secs(50));
        assert_eq!((w.headset_base.as_deref(), w.chosen), (Some(B), true));
    }

    #[test]
    fn the_watch_counts_a_wait_only_in_a_session_that_has_not_tracked_the_headset() {
        let line = |s: &str| crate::steamvr_log::parse_line(s).unwrap();
        let wait = line(
            "x lighthouse: LHR-HEADSET H: Saw 3 bases and none are the preferred base 0000000A",
        );
        let t0 = Instant::now();
        let later = t0 + Duration::from_secs(40);
        // tracked earlier in this log, then lost and waiting: the headset is out of sight
        let mut bases = crate::steamvr_log::Bases::default();
        bases.apply(&line(
            "x lighthouse: LHR-HEADSET H: ----- BOOTSTRAPPED base 0000000A (preferred) distance 3.31m",
        ));
        bases.apply(&line("x lighthouse: LHR-HEADSET H: Resetting tracking: x"));
        bases.apply(&wait);
        let mut w = Watch::default();
        w.see(&bases, t0);
        assert_eq!(w.waited(later), None);
        // SteamVR's next session, appended to the same log, has not tracked it: the wait counts
        bases.apply(&line("x lighthouse: version 2.18.4"));
        bases.apply(&wait);
        w.see(&bases, t0);
        assert_eq!(w.waited(later), Some((A, Duration::from_secs(40))));
        // skipped history may have held the headset's bootstrap: counted as tracked
        let mut bases = crate::steamvr_log::Bases::default();
        bases.apply(&crate::steamvr_log::Event::Skipped);
        bases.apply(&wait);
        w.see(&bases, t0);
        assert_eq!(w.waited(later), None);
        // the next session in the same log starts untracked, whatever was skipped before it
        bases.apply(&line("x lighthouse: version 2.18.4"));
        bases.apply(&wait);
        w.see(&bases, t0);
        assert_eq!(w.waited(later), Some((A, Duration::from_secs(40))));
        // a new log, never tracked: the wait counts
        let mut bases = crate::steamvr_log::Bases::default();
        bases.apply(&wait);
        w.see(&bases, t0);
        assert_eq!(w.waited(later), Some((A, Duration::from_secs(40))));
    }
}
