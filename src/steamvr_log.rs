// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Which base station SteamVR tracks each Lighthouse device from, read from its server log
//! (vrserver.txt). SteamVR's Lighthouse driver bootstraps a device's tracking from one base station
//! and from then on tracks it in that station's frame. As the device sees more stations it
//! re-solves their poses relative to each other ("RELATIONSHIP"), moving the one farther from the
//! universe's origin, which can be the device's own base ("Moving the base for tracking too"): the
//! device moves with its base, and a move of any other station does not move it. SteamVR names the
//! base when tracking starts ("BOOTSTRAPPED base") and says when tracking is lost; nothing in
//! OpenVR's API tells it.

use std::collections::{BTreeMap, BTreeSet};
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

/// What a line of the log says about a device's tracking base.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    /// `device` is tracked from base station `base` (OpenVR serial) from now on; `headset` when
    /// SteamVR logs the device as the headset.
    Bootstrapped {
        device: String,
        headset: bool,
        base: String,
    },
    /// `device` lost its tracking: until it bootstraps again it has no base.
    Lost { device: String },
    /// Every device lost its tracking: the universe was dropped, or the Lighthouse driver started.
    AllLost,
}

impl Event {
    fn device(&self) -> Option<&str> {
        match self {
            Event::Bootstrapped { device, .. } | Event::Lost { device } => Some(device),
            Event::AllLost => None,
        }
    }
}

/// The log line's event, if it is one: the Lighthouse driver's
/// `lighthouse: LHR-XXXXXXXX H: ----- BOOTSTRAPPED base 2B1973A2 (best) ...`,
/// `... H: Resetting tracking: ...`, `... H: Calibration failed: ...`,
/// `lighthouse: Stopped tracking with universe N` and `lighthouse: version X`.
pub fn parse_line(line: &str) -> Option<Event> {
    let (_, rest) = line.split_once("lighthouse: ")?;
    if rest.starts_with("Stopped tracking with universe") || rest.starts_with("version ") {
        return Some(Event::AllLost);
    }
    let (who, what) = rest.split_once(": ")?;
    let mut words = who.split(' ');
    let device = words.next().filter(|d| d.starts_with("LHR-"))?.to_string();
    let headset = match (words.next(), words.next()) {
        (None, _) => false,
        (Some(tag), None) => tag == "H",
        (Some(_), Some(_)) => return None,
    };
    if let Some(after) = what.strip_prefix("----- BOOTSTRAPPED base ") {
        let hex = after.get(..8)?;
        if !hex.bytes().all(|b| b.is_ascii_hexdigit()) || !after[8..].starts_with(' ') {
            return None;
        }
        return Some(Event::Bootstrapped {
            device,
            headset,
            base: format!("LHB-{}", hex.to_ascii_uppercase()),
        });
    }
    if what.starts_with("Resetting tracking") || what.starts_with("Calibration failed") {
        return Some(Event::Lost { device });
    }
    None
}

/// Each device's tracking base, as the log has told it so far.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Bases {
    /// Device serial -> base station serial, for the devices tracked now.
    pub by_device: BTreeMap<String, String>,
    /// The device SteamVR logs as the headset, once one has bootstrapped.
    pub headset: Option<String>,
    /// Device serial -> the base station it last bootstrapped from, kept while it is without
    /// tracking (until SteamVR starts a new log).
    pub last: BTreeMap<String, String>,
}

impl Bases {
    pub fn apply(&mut self, e: &Event) {
        match e {
            Event::Bootstrapped {
                device,
                headset,
                base,
            } => {
                if *headset {
                    self.headset = Some(device.clone());
                }
                self.by_device.insert(device.clone(), base.clone());
                self.last.insert(device.clone(), base.clone());
            }
            Event::Lost { device } => {
                self.by_device.remove(device);
            }
            Event::AllLost => self.by_device.clear(),
        }
    }

    /// The base station `device` is tracked from now.
    pub fn of(&self, device: &str) -> Option<&str> {
        self.by_device.get(device).map(String::as_str)
    }

    /// The base station the headset is tracked from now.
    pub fn of_headset(&self) -> Option<&str> {
        self.of(self.headset.as_deref()?)
    }

    /// The base station the headset was last tracked from in this log, through a moment without
    /// tracking; `reference`'s when the log has named no headset (a log from before SteamVR
    /// tagged it).
    pub fn last_of_headset(&self, reference: &str) -> Option<&str> {
        let device = self.headset.as_deref().unwrap_or(reference);
        self.last.get(device).map(String::as_str)
    }
}

/// The devices whose tracking base was the same from `start` to `end` with nothing in `events`
/// (what the log said in between) touching it: device serial -> base.
pub fn steady(start: &Bases, end: &Bases, events: &[Event]) -> BTreeMap<String, String> {
    if events.contains(&Event::AllLost) {
        return BTreeMap::new();
    }
    let touched: BTreeSet<&str> = events.iter().filter_map(Event::device).collect();
    start
        .by_device
        .iter()
        .filter(|(d, b)| !touched.contains(d.as_str()) && end.of(d) == Some(b.as_str()))
        .map(|(d, b)| (d.clone(), b.clone()))
        .collect()
}

/// SteamVR's server log.
pub fn server_log() -> PathBuf {
    crate::steamvr::log_dir().join("vrserver.txt")
}

/// The log as it grows. SteamVR starts a new one each time it starts (the old one becomes
/// vrserver.previous.txt): a file that no longer begins as it did, or is shorter than what was
/// read, is read again from its start.
pub struct Tail {
    path: PathBuf,
    /// Whether a poll has read the file yet.
    started: bool,
    offset: u64,
    head: Vec<u8>,
    partial: Vec<u8>,
}

/// What a poll read: whether the file was read from its start (the first poll, or a new log),
/// and its complete lines since the last poll.
#[derive(Debug, Default, PartialEq)]
pub struct Polled {
    pub restarted: bool,
    pub lines: Vec<String>,
}

/// Bytes compared to tell a new log from the one being read (its first line has the date and
/// time to the microsecond).
const HEAD_BYTES: usize = 128;

impl Tail {
    pub fn new(path: &Path) -> Tail {
        Tail {
            path: path.to_path_buf(),
            started: false,
            offset: 0,
            head: Vec::new(),
            partial: Vec::new(),
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn poll(&mut self) -> std::io::Result<Polled> {
        let mut f = std::fs::File::open(&self.path)?;
        let len = f.metadata()?.len();
        let mut head = vec![0u8; HEAD_BYTES.min(len as usize)];
        f.read_exact(&mut head)?;
        // the log read so far, grown or not, still begins as it did
        let same = self.started
            && len >= self.offset
            && head.len() >= self.head.len()
            && head[..self.head.len()] == self.head[..];
        let restarted = !same;
        if restarted {
            self.started = true;
            self.offset = 0;
            self.partial.clear();
        }
        if restarted || head.len() > self.head.len() {
            self.head = head;
        }
        f.seek(SeekFrom::Start(self.offset))?;
        let mut buf = Vec::new();
        f.read_to_end(&mut buf)?;
        self.offset += buf.len() as u64;
        self.partial.extend_from_slice(&buf);
        let mut lines = Vec::new();
        if let Some(end) = self.partial.iter().rposition(|&b| b == b'\n') {
            let complete: Vec<u8> = self.partial.drain(..=end).collect();
            lines = complete
                .split(|&b| b == b'\n')
                .filter(|l| !l.is_empty())
                .map(|l| {
                    String::from_utf8_lossy(l)
                        .trim_end_matches('\r')
                        .to_string()
                })
                .collect();
        }
        Ok(Polled { restarted, lines })
    }
}

/// Follows the tracking bases through a log: `poll` brings them up to date.
pub struct Watch {
    tail: Tail,
    pub bases: Bases,
}

impl Watch {
    pub fn new(path: &Path) -> Watch {
        Watch {
            tail: Tail::new(path),
            bases: Bases::default(),
        }
    }

    pub fn path(&self) -> &Path {
        self.tail.path()
    }

    /// Reads what the log added; returns the events it held (a new log starts from no bases,
    /// and counts as every device losing its tracking; a log that cannot be read leaves none).
    pub fn poll(&mut self) -> std::io::Result<Vec<Event>> {
        let p = match self.tail.poll() {
            Ok(p) => p,
            Err(e) => {
                // what the log said no longer stands for what SteamVR does now; once it can be
                // read again it is read from its start
                self.bases = Bases::default();
                self.tail = Tail::new(&self.tail.path.clone());
                return Err(e);
            }
        };
        let mut events = Vec::new();
        if p.restarted {
            self.bases = Bases::default();
            events.push(Event::AllLost);
        }
        for e in p.lines.iter().filter_map(|l| parse_line(l)) {
            self.bases.apply(&e);
            events.push(e);
        }
        Ok(events)
    }
}

/// The tracking bases over a stretch of time, a capture: `finish` names the devices that kept
/// theirs throughout (see `steady`), none when the log could not be read.
pub struct Span {
    watch: Watch,
    start: Option<Bases>,
}

impl Span {
    pub fn begin(path: &Path) -> Span {
        let mut watch = Watch::new(path);
        let start = watch.poll().ok().map(|_| watch.bases.clone());
        Span { watch, start }
    }

    pub fn finish(mut self) -> BTreeMap<String, String> {
        let Some(start) = self.start else {
            return BTreeMap::new();
        };
        match self.watch.poll() {
            Ok(events) => steady(&start, &self.watch.bases, &events),
            Err(_) => BTreeMap::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BOOT: &str = "Thu Oct 15 2026 17:14:51.820611 [Info] - lighthouse: LHR-D6E2112A H: ----- BOOTSTRAPPED base 2B1973A2 (best) distance 3.28m velocity 0.44m/s base pitch ~135.0 deg roll ~-7.5 deg -----";

    #[test]
    fn the_lines_that_name_or_end_a_tracking_base() {
        assert_eq!(
            parse_line(BOOT),
            Some(Event::Bootstrapped {
                device: "LHR-D6E2112A".into(),
                headset: true,
                base: "LHB-2B1973A2".into()
            })
        );
        assert_eq!(
            parse_line("Thu Oct 15 2026 17:26:24.882110 [Info] - lighthouse: LHR-D6E2112A H: ----- BOOTSTRAPPED base c1e62875 (delayed) distance 2.06m velocity 0.02m/s -----"),
            Some(Event::Bootstrapped {
                device: "LHR-D6E2112A".into(),
                headset: true,
                base: "LHB-C1E62875".into()
            })
        );
        // a controller: no headset tag
        assert_eq!(
            parse_line("x [Info] - lighthouse: LHR-9A21C00A: ----- BOOTSTRAPPED base FB22E36C (best) distance 2.1m -----"),
            Some(Event::Bootstrapped {
                device: "LHR-9A21C00A".into(),
                headset: false,
                base: "LHB-FB22E36C".into()
            })
        );
        assert_eq!(
            parse_line("x [Info] - lighthouse: LHR-D6E2112A H: Resetting tracking: no optical samples for 2000ms"),
            Some(Event::Lost {
                device: "LHR-D6E2112A".into()
            })
        );
        assert_eq!(
            parse_line("x [Info] - lighthouse: LHR-D6E2112A H: Calibration failed: no optical samples from base 2B1973A2 for 2000ms"),
            Some(Event::Lost {
                device: "LHR-D6E2112A".into()
            })
        );
        assert_eq!(
            parse_line("x [Info] - lighthouse: Stopped tracking with universe 1772213422"),
            Some(Event::AllLost)
        );
        assert_eq!(
            parse_line("x [Info] - lighthouse: version 2.17.10"),
            Some(Event::AllLost)
        );
        for other in [
            "x [Info] - lighthouse: LHR-D6E2112A H: ----- SECONDARY base 2B1973A2 distance 3.25m -----",
            "x [Info] - lighthouse: Moving base 2B1973A2 88mm and 1.6 deg because of relationship with FB22E36C, which is closer to the origin",
            "x [Info] - lighthouse: LHR-D6E2112A H: ----- BOOTSTRAPPED base 2B19 (best) -----",
            "x [Info] - lighthouse: LHR-D6E2112A H: ----- BOOTSTRAPPED base 2B1973A2X (best) -----",
            "x [Info] - lighthouse: 80CFF8E11C-RYB: Triggered keepalive (succeeded)",
            "x [Info] - New Connect message from /x/vrstartup",
        ] {
            assert_eq!(parse_line(other), None, "{other}");
        }
    }

    fn boot(device: &str, headset: bool, base: &str) -> Event {
        Event::Bootstrapped {
            device: device.into(),
            headset,
            base: base.into(),
        }
    }

    #[test]
    fn a_device_keeps_its_base_until_it_loses_tracking() {
        let mut b = Bases::default();
        b.apply(&boot("LHR-H", true, "LHB-A"));
        b.apply(&boot("LHR-C", false, "LHB-B"));
        assert_eq!(b.of_headset(), Some("LHB-A"));
        assert_eq!(b.of("LHR-C"), Some("LHB-B"));
        b.apply(&Event::Lost {
            device: "LHR-H".into(),
        });
        assert_eq!(b.of_headset(), None);
        assert_eq!(b.of("LHR-C"), Some("LHB-B"));
        b.apply(&boot("LHR-H", true, "LHB-C"));
        assert_eq!(b.of_headset(), Some("LHB-C"));
        b.apply(&Event::AllLost);
        assert_eq!(b.of_headset(), None);
        assert_eq!(b.headset.as_deref(), Some("LHR-H"));
        // the station it was last tracked from stays, for following through a dropout
        assert_eq!(b.last_of_headset("LHR-H"), Some("LHB-C"));
        // a controller as the reference does not stand in for a headset the log names
        assert_eq!(b.last_of_headset("LHR-C"), Some("LHB-C"));
        let mut untagged = Bases::default();
        untagged.apply(&boot("LHR-C", false, "LHB-B"));
        assert_eq!(untagged.last_of_headset("LHR-C"), Some("LHB-B"));
        assert_eq!(untagged.last_of_headset("LHR-X"), None);
    }

    #[test]
    fn a_base_is_steady_only_when_nothing_touched_it() {
        let mut start = Bases::default();
        start.apply(&boot("LHR-H", true, "LHB-A"));
        start.apply(&boot("LHR-C", false, "LHB-B"));
        let end = start.clone();
        assert_eq!(steady(&start, &end, &[]).len(), 2);
        // lost and bootstrapped again from the same station in between: not steady
        let events = [
            Event::Lost {
                device: "LHR-H".into(),
            },
            boot("LHR-H", true, "LHB-A"),
        ];
        assert_eq!(
            steady(&start, &end, &events),
            BTreeMap::from([("LHR-C".to_string(), "LHB-B".to_string())])
        );
        assert!(steady(&start, &end, &[Event::AllLost]).is_empty());
        let mut moved = end.clone();
        moved.apply(&boot("LHR-C", false, "LHB-D"));
        assert_eq!(steady(&start, &moved, &[]).len(), 1);
    }

    #[test]
    fn the_tail_reads_what_was_added_and_a_new_log_from_its_start() {
        let dir = std::env::temp_dir().join(format!("cv1t-vrlog-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("vrserver.txt");
        std::fs::write(
            &path,
            "Thu Oct 15 2026 first line\n".to_string() + BOOT + "\npart",
        )
        .unwrap();
        let mut w = Watch::new(&path);
        let e = w.poll().unwrap();
        assert_eq!(e.len(), 2, "{e:?}"); // the start of a log, and the bootstrap
        assert_eq!(w.bases.of_headset(), Some("LHB-2B1973A2"));
        // the rest of the half-written line arrives
        let mut f = std::fs::OpenOptions::new()
            .append(true)
            .open(&path)
            .unwrap();
        use std::io::Write;
        writeln!(f, "ial [Info] - lighthouse: LHR-D6E2112A H: Resetting tracking: no optical samples for 2000ms").unwrap();
        drop(f);
        assert_eq!(
            w.poll().unwrap(),
            vec![Event::Lost {
                device: "LHR-D6E2112A".into()
            }]
        );
        assert_eq!(w.bases.of_headset(), None);
        assert!(w.poll().unwrap().is_empty());
        // SteamVR restarts: a new, longer log that begins differently
        let mut text = "Fri Oct 16 2026 a new log\n".to_string();
        text += &BOOT.replace("2B1973A2", "FB22E36C");
        text += "\n";
        text += &"padding line\n".repeat(20);
        std::fs::write(&path, text).unwrap();
        let e = w.poll().unwrap();
        assert_eq!(e.first(), Some(&Event::AllLost));
        assert_eq!(w.bases.of_headset(), Some("LHB-FB22E36C"));
        assert_eq!(w.bases.last_of_headset("x"), Some("LHB-FB22E36C"));
        // and from then on it is followed as it grows, not read again from its start
        assert_eq!(w.poll().unwrap(), vec![]);
        let mut f = std::fs::OpenOptions::new()
            .append(true)
            .open(&path)
            .unwrap();
        writeln!(f, "x [Info] - lighthouse: LHR-D6E2112A H: Resetting tracking: no optical samples for 2000ms").unwrap();
        drop(f);
        assert_eq!(
            w.poll().unwrap(),
            vec![Event::Lost {
                device: "LHR-D6E2112A".into()
            }]
        );
        assert_eq!(w.poll().unwrap(), vec![]);
        // and a shorter one that begins the same way: a new session, no station remembered
        std::fs::write(&path, "Fri Oct 16 2026 a new log\n").unwrap();
        assert_eq!(w.poll().unwrap(), vec![Event::AllLost]);
        assert_eq!(w.bases.of_headset(), None);
        assert_eq!(w.bases.last_of_headset("LHR-D6E2112A"), None);
        // nor once the log cannot be read
        std::fs::write(&path, format!("Fri Oct 16 2026 a new log\n{BOOT}\n")).unwrap();
        w.poll().unwrap();
        assert!(w.bases.last_of_headset("LHR-D6E2112A").is_some());
        std::fs::remove_file(&path).unwrap();
        assert!(w.poll().is_err());
        assert_eq!(w.bases.last_of_headset("LHR-D6E2112A"), None);
        // the same log readable again: read from its start, so the base is known again
        std::fs::write(&path, format!("Fri Oct 16 2026 a new log\n{BOOT}\n")).unwrap();
        w.poll().unwrap();
        assert_eq!(
            w.bases.last_of_headset("LHR-D6E2112A"),
            Some("LHB-2B1973A2")
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_short_log_that_grows_is_the_same_log() {
        let dir = std::env::temp_dir().join(format!("cv1t-vrgrow-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("vrserver.txt");
        std::fs::write(&path, "").unwrap();
        let mut w = Watch::new(&path);
        assert_eq!(w.poll().unwrap(), vec![Event::AllLost]);
        assert_eq!(w.poll().unwrap(), vec![]);
        std::fs::write(&path, "Thu Oct 15 2026 21:04:24.280468 [Info] - start\n").unwrap();
        assert_eq!(w.poll().unwrap(), vec![]);
        std::fs::write(
            &path,
            format!("Thu Oct 15 2026 21:04:24.280468 [Info] - start\n{BOOT}\n"),
        )
        .unwrap();
        assert_eq!(w.poll().unwrap().len(), 1);
        assert_eq!(w.poll().unwrap(), vec![]);
        assert_eq!(w.bases.of_headset(), Some("LHB-2B1973A2"));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_span_names_the_bases_kept_throughout() {
        let dir = std::env::temp_dir().join(format!("cv1t-vrspan-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("vrserver.txt");
        std::fs::write(&path, format!("Thu Oct 15 2026 first line\n{BOOT}\n")).unwrap();
        let s = Span::begin(&path);
        std::fs::write(
            &path,
            format!("Thu Oct 15 2026 first line\n{BOOT}\nx [Info] - lighthouse: 80CFF8E11C-RYB: keepalive\n"),
        )
        .unwrap();
        assert_eq!(
            s.finish(),
            BTreeMap::from([("LHR-D6E2112A".to_string(), "LHB-2B1973A2".to_string())])
        );
        let s = Span::begin(&path);
        std::fs::write(
            &path,
            format!("Thu Oct 15 2026 first line\n{BOOT}\nx [Info] - lighthouse: LHR-D6E2112A H: Resetting tracking: no optical samples for 2000ms\n{BOOT}\n"),
        )
        .unwrap();
        assert!(s.finish().is_empty());
        assert!(Span::begin(&dir.join("missing.txt")).finish().is_empty());
        std::fs::remove_dir_all(&dir).ok();
    }
}
