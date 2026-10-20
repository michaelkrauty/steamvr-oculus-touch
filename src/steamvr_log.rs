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

use crate::log_lines::{LineBuffer, READ_BYTES};

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
    /// Log input or event history was discarded; earlier bases cannot be trusted.
    Skipped,
}

impl Event {
    fn device(&self) -> Option<&str> {
        match self {
            Event::Bootstrapped { device, .. } | Event::Lost { device } => Some(device),
            Event::AllLost | Event::Skipped => None,
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
    /// tracking (until SteamVR starts a new log or input is skipped).
    pub last: BTreeMap<String, String>,
    /// Skipped history makes an unnamed headset base unknown, rather than known to be lost.
    skipped: bool,
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
            Event::Skipped => {
                self.by_device.clear();
                self.last.clear();
                self.skipped = true;
            }
        }
    }

    /// Whether skipped input left the headset's base unknown. A later bootstrap names it again.
    pub fn tracking_unknown(&self, reference: &str) -> bool {
        self.skipped && self.last_of_headset(reference).is_none()
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
    /// tagged it), provided no input has been skipped.
    pub fn last_of_headset(&self, reference: &str) -> Option<&str> {
        let device = match self.headset.as_deref() {
            Some(device) => device,
            None if self.skipped => return None,
            None => reference,
        };
        self.last.get(device).map(String::as_str)
    }
}

/// The devices whose tracking base was the same from `start` to `end` with nothing in `events`
/// (what the log said in between) touching it: device serial -> base.
pub fn steady(start: &Bases, end: &Bases, events: &[Event]) -> BTreeMap<String, String> {
    if events.contains(&Event::AllLost) || events.contains(&Event::Skipped) {
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
struct Tail {
    path: PathBuf,
    /// Whether a poll has read the file yet.
    started: bool,
    offset: u64,
    head: Vec<u8>,
    partial: LineBuffer,
}

enum LogLine<'a> {
    Restart,
    Text(&'a [u8]),
    Discarded,
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
            partial: LineBuffer::default(),
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    fn poll(&mut self, mut line: impl FnMut(LogLine<'_>)) -> std::io::Result<()> {
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
            self.partial = LineBuffer::default();
            line(LogLine::Restart);
        }
        if restarted || head.len() > self.head.len() {
            self.head = head;
        }
        f.seek(SeekFrom::Start(self.offset))?;
        // Read only the file as it stood when opened, in fixed chunks. A writer appending
        // continuously cannot keep this poll running forever.
        let mut remaining = len.saturating_sub(self.offset);
        let mut buf = [0u8; READ_BYTES];
        while remaining != 0 {
            let want = remaining.min(buf.len() as u64) as usize;
            let n = match f.read(&mut buf[..want]) {
                Ok(0) => break,
                Ok(n) => n,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e),
            };
            self.offset += n as u64;
            remaining -= n as u64;
            self.partial.push(&buf[..n], |text, truncated| {
                // A shortened prefix could look like a valid tracking event: discard the
                // whole overlong line, including its suffix through the next newline.
                if !truncated {
                    line(LogLine::Text(text));
                } else {
                    line(LogLine::Discarded);
                }
            });
            if self.partial.is_truncated() {
                line(LogLine::Discarded);
            }
        }
        Ok(())
    }
}

const MAX_EVENTS: usize = 1024;

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
    /// More than `MAX_EVENTS` events return only `Skipped`, so captures cannot mistake missing
    /// history for steady tracking. Earlier bases are forgotten; later complete events still
    /// update the current bases and a later bootstrap can name the headset's base again.
    pub fn poll(&mut self) -> std::io::Result<Vec<Event>> {
        let mut events = Vec::new();
        let mut overflow = false;
        let result = self.tail.poll(|line| {
            let event = match line {
                LogLine::Restart => {
                    self.bases = Bases::default();
                    Some(Event::AllLost)
                }
                LogLine::Discarded => Some(Event::Skipped),
                LogLine::Text(text) => parse_line(&String::from_utf8_lossy(text)),
            };
            if let Some(e) = event {
                if !overflow && events.len() == MAX_EVENTS {
                    self.bases.apply(&Event::Skipped);
                    events.clear();
                    events.push(Event::Skipped);
                    overflow = true;
                }
                self.bases.apply(&e);
                if !overflow {
                    events.push(e);
                }
            }
        });
        if let Err(e) = result {
            self.bases = Bases::default();
            self.tail = Tail::new(&self.tail.path.clone());
            return Err(e);
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

    #[test]
    fn a_burst_of_events_is_bounded_without_leaving_the_bases_out_of_date() {
        let dir = std::env::temp_dir().join(format!("cv1t-vrlog-burst-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("vrserver.txt");
        let boot = "x lighthouse: LHR-EXAMPLE H: ----- BOOTSTRAPPED base 1234ABCD (best)\n";
        let later = "x lighthouse: LHR-EXAMPLE H: ----- BOOTSTRAPPED base 8765DCBA (best)\nx lighthouse: LHR-CONTROLLER: ----- BOOTSTRAPPED base 11112222 (best)\nx lighthouse: LHR-CONTROLLER: Resetting tracking: no samples\nx lighthouse: LHR-EXAMPLE H: Resetting tracking: no samples\n";
        std::fs::write(&path, boot.repeat(1100) + later).unwrap();
        let mut w = Watch::new(&path);
        assert_eq!(w.poll().unwrap(), [Event::Skipped]);
        assert_eq!(w.bases.of_headset(), None);
        assert_eq!(
            w.bases.last_of_headset("LHR-REFERENCE"),
            Some("LHB-8765DCBA")
        );
        assert_eq!(w.bases.of("LHR-CONTROLLER"), None);
        assert_eq!(
            w.bases.last.get("LHR-CONTROLLER").map(String::as_str),
            Some("LHB-11112222")
        );
        assert!(w.poll().unwrap().is_empty());
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn a_skipped_log_cannot_use_the_reference_before_identifying_the_headset() {
        use std::io::Write;
        let dir =
            std::env::temp_dir().join(format!("cv1t-vrlog-unidentified-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("vrserver.txt");
        for overflow in [false, true] {
            std::fs::write(
                &path,
                "x lighthouse: LHR-REFERENCE: ----- BOOTSTRAPPED base 1234ABCD (best)\n",
            )
            .unwrap();
            let mut watch = Watch::new(&path);
            watch.poll().unwrap();
            assert_eq!(
                watch.bases.last_of_headset("LHR-REFERENCE"),
                Some("LHB-1234ABCD")
            );
            let mut file = std::fs::OpenOptions::new()
                .append(true)
                .open(&path)
                .unwrap();
            if overflow {
                write!(
                    file,
                    "{}",
                    "x lighthouse: LHR-REFERENCE: Resetting tracking: no samples\n"
                        .repeat(MAX_EVENTS + 1)
                )
                .unwrap();
            } else {
                writeln!(
                    file,
                    "x lighthouse: LHR-HEADSET H: ----- BOOTSTRAPPED base 8765DCBA (best){}",
                    "x".repeat(8192)
                )
                .unwrap();
            }
            writeln!(
                file,
                "x lighthouse: LHR-REFERENCE: ----- BOOTSTRAPPED base 11112222 (best)"
            )
            .unwrap();
            assert!(watch.poll().unwrap().contains(&Event::Skipped));
            assert_eq!(watch.bases.headset, None);
            assert_eq!(watch.bases.of("LHR-REFERENCE"), Some("LHB-11112222"));
            assert_eq!(watch.bases.last_of_headset("LHR-REFERENCE"), None);
            assert!(watch.bases.tracking_unknown("LHR-REFERENCE"));
            writeln!(
                file,
                "x lighthouse: LHR-HEADSET H: ----- BOOTSTRAPPED base 8765DCBA (best)"
            )
            .unwrap();
            watch.poll().unwrap();
            assert_eq!(
                watch.bases.last_of_headset("LHR-REFERENCE"),
                Some("LHB-8765DCBA")
            );
            assert!(!watch.bases.tracking_unknown("LHR-REFERENCE"));
        }
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn skipped_history_keeps_the_headset_identity() {
        let mut bases = Bases::default();
        bases.apply(&boot("LHR-HEADSET", true, "LHB-1234ABCD"));
        bases.apply(&boot("LHR-CONTROLLER", false, "LHB-11112222"));
        bases.apply(&Event::Skipped);
        bases.apply(&boot("LHR-CONTROLLER", false, "LHB-8765DCBA"));
        assert_eq!(bases.last_of_headset("LHR-CONTROLLER"), None);
        assert!(bases.tracking_unknown("LHR-CONTROLLER"));
        bases.apply(&boot("LHR-HEADSET", false, "LHB-55556666"));
        assert_eq!(
            bases.last_of_headset("LHR-CONTROLLER"),
            Some("LHB-55556666")
        );
        assert!(!bases.tracking_unknown("LHR-CONTROLLER"));
    }

    #[test]
    fn skipped_bootstraps_forget_the_remembered_headset_base_until_named_again() {
        use std::io::Write;
        let dir =
            std::env::temp_dir().join(format!("cv1t-vrlog-remembered-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("vrserver.txt");
        for tag in [" H", ""] {
            let a = format!(
                "x lighthouse: LHR-EXAMPLE{tag}: ----- BOOTSTRAPPED base 1234ABCD (best)\n"
            );
            let b =
                format!("x lighthouse: LHR-EXAMPLE{tag}: ----- BOOTSTRAPPED base 8765DCBA (best)");
            for ending in ["\n", ""] {
                std::fs::write(&path, &a).unwrap();
                let mut watch = Watch::new(&path);
                watch.poll().unwrap();
                assert_eq!(
                    watch.bases.last_of_headset("LHR-EXAMPLE"),
                    Some("LHB-1234ABCD")
                );
                let mut file = std::fs::OpenOptions::new()
                    .append(true)
                    .open(&path)
                    .unwrap();
                write!(file, "{b}{}{ending}", "x".repeat(8192)).unwrap();
                watch.poll().unwrap();
                assert_eq!(
                    watch.bases.last_of_headset("LHR-EXAMPLE"),
                    None,
                    "a discarded bootstrap could have changed the base"
                );
                write!(file, "\n{b}\n").unwrap();
                watch.poll().unwrap();
                if tag.is_empty() {
                    assert_eq!(watch.bases.last_of_headset("LHR-EXAMPLE"), None);
                    assert!(watch.bases.tracking_unknown("LHR-EXAMPLE"));
                    writeln!(
                        file,
                        "x lighthouse: LHR-EXAMPLE H: ----- BOOTSTRAPPED base 8765DCBA (best)"
                    )
                    .unwrap();
                    watch.poll().unwrap();
                }
                assert_eq!(
                    watch.bases.last_of_headset("LHR-EXAMPLE"),
                    Some("LHB-8765DCBA")
                );
                watch.bases.apply(&Event::Lost {
                    device: "LHR-EXAMPLE".into(),
                });
                assert_eq!(
                    watch.bases.last_of_headset("LHR-EXAMPLE"),
                    Some("LHB-8765DCBA"),
                    "ordinary loss retains the named base"
                );
            }
        }
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn overflow_forgets_the_remembered_base_until_a_later_bootstrap() {
        use std::io::Write;
        let dir = std::env::temp_dir().join(format!("cv1t-vrlog-overflow-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("vrserver.txt");
        std::fs::write(
            &path,
            "x lighthouse: LHR-EXAMPLE H: ----- BOOTSTRAPPED base 1234ABCD (best)\n",
        )
        .unwrap();
        let mut watch = Watch::new(&path);
        watch.poll().unwrap();
        let mut file = std::fs::OpenOptions::new()
            .append(true)
            .open(&path)
            .unwrap();
        write!(
            file,
            "{}",
            "x lighthouse: LHR-CONTROLLER: Resetting tracking: no samples\n".repeat(MAX_EVENTS + 1)
        )
        .unwrap();
        assert!(!watch.poll().unwrap().is_empty());
        assert_eq!(watch.bases.last_of_headset("LHR-EXAMPLE"), None);
        writeln!(
            file,
            "x lighthouse: LHR-EXAMPLE H: ----- BOOTSTRAPPED base 8765DCBA (best)"
        )
        .unwrap();
        watch.poll().unwrap();
        assert_eq!(
            watch.bases.last_of_headset("LHR-EXAMPLE"),
            Some("LHB-8765DCBA")
        );
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn an_unterminated_line_stays_bounded_and_is_not_parsed_after_truncation() {
        let dir = std::env::temp_dir().join(format!("cv1t-vrlog-long-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("vrserver.txt");
        let boot = "x lighthouse: LHR-EXAMPLE H: ----- BOOTSTRAPPED base 1234ABCD (best)";
        std::fs::write(&path, format!("header\n{boot}{}", "x".repeat(32 * 1024))).unwrap();
        let mut w = Watch::new(&path);
        assert!(w
            .poll()
            .unwrap()
            .iter()
            .all(|e| matches!(e, Event::AllLost | Event::Skipped)));
        assert!(w.tail.partial.len() <= 8192);
        use std::io::Write;
        let mut file = std::fs::OpenOptions::new()
            .append(true)
            .open(&path)
            .unwrap();
        writeln!(file, "\n{boot}").unwrap();
        let events = w.poll().unwrap();
        assert_eq!(events.len(), 2);
        assert_eq!(
            events[0],
            Event::Skipped,
            "the overlong line must be discarded"
        );
        assert_eq!(w.bases.of_headset(), Some("LHB-1234ABCD"));
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn an_overlong_line_cannot_hide_a_tracking_change_during_a_capture() {
        let dir = std::env::temp_dir().join(format!("cv1t-vrlog-discard-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("vrserver.txt");
        for ending in ["\n", ""] {
            std::fs::write(
                &path,
                "x lighthouse: LHR-EXAMPLE H: ----- BOOTSTRAPPED base 1234ABCD (best)\n",
            )
            .unwrap();
            let span = Span::begin(&path);
            use std::io::Write;
            let mut log = std::fs::OpenOptions::new()
                .append(true)
                .open(&path)
                .unwrap();
            write!(
                log,
                "x lighthouse: LHR-EXAMPLE H: Resetting tracking: {}{ending}",
                "x".repeat(8192)
            )
            .unwrap();
            assert!(
                span.finish().is_empty(),
                "discarding a line lost its tracking change"
            );
        }
        std::fs::remove_dir_all(dir).unwrap();
    }

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
        assert!(steady(&start, &start, &[Event::Skipped]).is_empty());
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
