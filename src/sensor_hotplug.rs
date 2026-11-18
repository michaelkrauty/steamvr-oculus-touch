// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! USB presence changes, debounced independently per sensor. A stream failure is not an
//! attachment change; a definite device removal can retire a handle even between USB scans.
//! A running engine adds a settled, ready sensor itself (`take_adds`); without one, the next
//! engine start opens it (`before_open`).

use std::collections::{BTreeMap, BTreeSet};
use std::io;

use crate::platform::usb::Sensor;
use crate::status::{same_sensor, sensor_identity};

pub const DEBOUNCE_NS: u64 = 1_000_000_000;
/// A live add that found no device, or one in use, is tried this many times more, this far apart:
/// uvcvideo can hold a sensor's interfaces for a moment after it appears.
pub const ADD_RETRIES: u32 = 3;
pub const ADD_RETRY_NS: u64 = 1_000_000_000;

/// libusb's error codes a live add can end with.
pub const LIBUSB_ERROR_ACCESS: i32 = -3;
pub const LIBUSB_ERROR_NO_DEVICE: i32 = -4;
pub const LIBUSB_ERROR_NOT_FOUND: i32 = -5;
pub const LIBUSB_ERROR_BUSY: i32 = -6;

#[derive(Debug, Default, PartialEq, Eq)]
pub struct Change {
    pub plugged: Vec<Sensor>,
    pub unplugged: Vec<String>,
    pub ready: Vec<String>,
    /// Connected and ready, but the engine that opened did not open them.
    pub unopened: Vec<String>,
}

#[derive(serde::Serialize)]
pub struct PendingSensor {
    pub serial: String,
    pub state: &'static str,
    pub detail: String,
}

/// What the engine thread does with a finished live add (`Monitor::add_finished`).
#[derive(Debug, PartialEq, Eq)]
pub enum AddVerdict {
    /// The sensor joined the engine: it is in use from now on.
    Use,
    /// The watch no longer waits for this add (`closed`, `before_open`): release it at once.
    Release,
    /// Its USB device went away while it was being opened, as a native NO_DEVICE says: retire it
    /// (`Change::unplugged`); it gets one recovery once two scans confirm it is back.
    Gone(Change),
    /// Tried again once `ADD_RETRY_NS` have passed.
    Retry { attempt: u32 },
    /// Not tried again before it is plugged in again; `pending` says why, with what to do.
    Failed(String),
    /// Nothing more to do: a failed start is recorded with the engine's start records, as at an
    /// engine start, and is not tried again before the next attachment.
    Done,
}

/// The watcher's view of the sensors and of the current engine's sensor objects. One rule
/// decides what a USB observation can do to an object:
/// - While the engine holds a live object for a serial, the serial is in use: nothing the
///   watcher sees makes it pending, other than as opening while the engine opens.
/// - Only evidence about the device an object opened retires it: its own NO_DEVICE, or a view
///   settled from listings read after the object started that rules that device out. A serial
///   names one device, so the serial being gone rules out whichever device the object opened, and
///   so does the departure of an attachment such a listing showed for it: the object's device
///   was that one, or had left before it came. A serial no such listing has shown is retired only
///   by its absence or by the object's NO_DEVICE.
/// - The listing engine_session reads after the open is read after every start; a watcher scan
///   is, from the second one applied after the roster is known (scans run one at a time, and the
///   first may have been read before the starts). From `before_open` until then, nothing
///   settles: no departure can be blamed on an object yet unknown, and nothing logged or
///   reported about a sensor the engine may open claims what it will do with it.
/// - A running engine adds a settled, ready sensor itself (`take_adds`), and the object it starts
///   joins the roster as one the open started does: from its adoption the serial is in use, by
///   no attachment yet. From `take_adds` until the add ends nothing about that serial settles,
///   and the first scan applied after its adoption, which may have been read before its start,
///   settles nothing about it either; the next ones are read after its start. So only those
///   listings or its own NO_DEVICE retire it.
/// - The roster rebuilds the attempts: each attachment gets one per engine, at its open or in an
///   add. A start, whether its video started or not, spends it until the next engine start or a
///   replug; a device that was not there yet or was busy is tried again, a few times; a
///   NO_DEVICE gets the one recovery a native NO_DEVICE gets, held against the attachment it was
///   reported for. Without a running engine a ready sensor waits for the next engine start, and
///   so does one the running engine refused, with the reason.
#[derive(Clone, Debug, Hash)]
pub struct Monitor {
    present: BTreeMap<String, Sensor>,
    membership: BTreeMap<String, (Option<Sensor>, u64)>,
    readiness: BTreeMap<String, (String, bool, u64)>,
    attempted: BTreeMap<String, String>,
    /// Detached serials: their native objects stay excluded until the engine has released them
    /// (`released`), or until the next engine start.
    detached: BTreeSet<String>,
    gone_seen: BTreeMap<String, String>,
    /// One recovery of an attachment whose native handle reported NO_DEVICE: once back, the
    /// running engine adds it again (or the next engine start opens it). A new OS generation
    /// resets this budget; the same attachment failing again asks for a replug.
    gone_retried: BTreeMap<String, String>,
    back: BTreeMap<String, Option<(String, u64)>>,
    opening: bool,
    /// From `before_open` until the engine's roster is known: any sensor may be one it opens.
    discovering: bool,
    /// A scan applied now was read after the current engine's starts (see the rule above).
    settling: bool,
    opening_serials: BTreeSet<String>,
    /// Attachment identities observed before this engine began discovery.
    opening_attachments: BTreeMap<String, String>,
    /// The last engine open failed (no headset): its sensors wait for it, not for the engine.
    engine_waiting: bool,
    /// The current engine's live sensor objects, with the attachment a listing read after the
    /// start last showed for that serial, if one has.
    in_use: BTreeMap<String, Option<String>>,
    /// An engine is open and past its first update: it adds settled, ready sensors itself.
    running: bool,
    /// Live adds the engine is working on: serial -> attachment.
    adding: BTreeMap<String, String>,
    /// Serials whose object joined the running engine after the last scan applied was read.
    fresh: BTreeSet<String>,
    /// Live adds that found no device, or one in use: serial -> (attachment, tries so far, not
    /// before).
    retry: BTreeMap<String, (String, u32, u64)>,
    /// Live adds that failed for good: serial -> (attachment, the reason with what to do).
    add_failed: BTreeMap<String, (String, String)>,
    /// Attachments the running engine could not be asked to add: they wait for its next start.
    refused: BTreeMap<String, String>,
    /// When a scan first listed each attachment: serial -> (attachment, host ns).
    seen: BTreeMap<String, (String, u64)>,
    scan_error: Option<String>,
}

/// Why a present sensor is not ready to open on this operating system.
fn usb_access_detail() -> &'static str {
    if cfg!(windows) {
        "This sensor's driver interfaces aren't active yet."
    } else {
        "This sensor's USB device isn't accessible yet (permissions)."
    }
}

/// A replacement, removal or newly granted access that has not settled yet.
const CHECKING_DETAIL: &str = "Its USB connection just changed; the daemon is checking it again.";

/// A sensor the opening engine may still open.
const OPENING_DETAIL: &str = "Waiting for the engine to finish opening.";

/// A sensor the engine is opening, or that the running engine opens on its next pass.
const ADDING_DETAIL: &str = "The engine is opening this sensor.";

/// A ready sensor no running engine adds: the next engine start opens it.
fn engine_start_detail() -> &'static str {
    if cfg!(windows) {
        "This sensor will be used the next time the tracking engine starts. Restart the daemon to use it now (choose Stop cv1touch, then Start cv1touch in the Start menu)."
    } else {
        "This sensor will be used the next time the tracking engine starts. Restart the daemon to use it now (systemctl --user restart cv1touchd)."
    }
}

/// Why a live add failed for good, in plain words, with what to do.
pub fn add_failure_detail(outcome: crate::engine::AddOutcome, error_code: i32) -> String {
    use crate::engine::AddOutcome;
    let replug = "unplug the sensor and plug it back in";
    match (outcome, error_code) {
        (AddOutcome::Rejected, _) => format!(
            "The engine turned this sensor down: it reported another serial, or it is set to be ignored. Check the daemon log, then {replug}."
        ),
        (AddOutcome::NotFound | AddOutcome::Stale, _) | (_, LIBUSB_ERROR_NOT_FOUND) => format!(
            "Its USB connection kept dropping while the engine opened it. Check its cable, then {replug}."
        ),
        (_, LIBUSB_ERROR_BUSY) => format!(
            "Another program or driver is using this sensor's USB device. Close it, then {replug}."
        ),
        (_, LIBUSB_ERROR_ACCESS) if cfg!(windows) => format!(
            "Windows didn't let the daemon open this sensor's USB device. Run cv1touch sensors to check its driver, then {replug}."
        ),
        (_, LIBUSB_ERROR_ACCESS) => format!(
            "The daemon isn't allowed to use this sensor's USB device (permissions). Run cv1touch sensors to check USB access, then {replug}."
        ),
        (_, code) => format!(
            "The engine could not open this sensor (USB error {code}). {}; if that keeps happening, try another USB port.",
            capitalized(replug)
        ),
    }
}

/// The running engine could not be asked to open a sensor at all: only an engine start opens it.
fn add_refused_detail() -> String {
    format!(
        "The running engine could not take this sensor. {}",
        engine_start_detail()
    )
}

fn capitalized(text: &str) -> String {
    let mut chars = text.chars();
    chars
        .next()
        .map(|c| c.to_ascii_uppercase().to_string() + chars.as_str())
        .unwrap_or_default()
}

fn listing(sensors: Vec<Sensor>, ignored: &[String]) -> BTreeMap<String, Sensor> {
    sensors
        .into_iter()
        .filter(|s| !ignored.iter().any(|i| same_sensor(i, &s.serial)))
        .map(|s| (sensor_identity(&s.serial), s))
        .collect()
}

impl Monitor {
    pub fn new(initial: io::Result<Vec<Sensor>>, ignored: &[String]) -> Self {
        let (initial, scan_error) = match initial {
            Ok(devices) => (devices, None),
            Err(e) => (Vec::new(), Some(e.to_string())),
        };
        let present = listing(initial, ignored);
        let attempted = present
            .iter()
            .filter(|(_, s)| s.ready)
            .map(|(serial, s)| (serial.clone(), s.attachment.clone()))
            .collect();
        Self {
            present,
            attempted,
            membership: BTreeMap::new(),
            readiness: BTreeMap::new(),
            detached: BTreeSet::new(),
            gone_seen: BTreeMap::new(),
            gone_retried: BTreeMap::new(),
            back: BTreeMap::new(),
            opening: false,
            discovering: false,
            settling: true,
            opening_serials: BTreeSet::new(),
            opening_attachments: BTreeMap::new(),
            engine_waiting: false,
            in_use: BTreeMap::new(),
            running: false,
            adding: BTreeMap::new(),
            fresh: BTreeSet::new(),
            retry: BTreeMap::new(),
            add_failed: BTreeMap::new(),
            refused: BTreeMap::new(),
            seen: BTreeMap::new(),
            scan_error,
        }
    }

    fn ready_since(&mut self, serial: &str, sensor: &Sensor, now: u64) -> bool {
        let candidate = self
            .readiness
            .entry(serial.to_string())
            .or_insert_with(|| (sensor.attachment.clone(), sensor.ready, now));
        if candidate.0 != sensor.attachment || candidate.1 != sensor.ready {
            *candidate = (sensor.attachment.clone(), sensor.ready, now);
        }
        now.saturating_sub(candidate.2) >= DEBOUNCE_NS
    }

    fn retire(&mut self, serial: &str) {
        self.detached.insert(serial.to_string());
        self.attempted.remove(serial);
        self.in_use.remove(serial);
    }

    pub fn update(
        &mut self,
        now: u64,
        listed: io::Result<Vec<Sensor>>,
        ignored: &[String],
    ) -> Option<Change> {
        // Whether this listing can settle anything, and whether the next one can. Nothing about
        // a sensor settles while the engine adds it, nor by the first listing applied after it
        // joined: that one may have been read before its start.
        let settles_any = self.settling;
        self.settling = !self.discovering;
        let fresh = std::mem::take(&mut self.fresh);
        let mut next = match listed {
            Ok(devices) => {
                self.scan_error = None;
                listing(devices, ignored)
            }
            Err(e) => {
                self.scan_error = Some(e.to_string());
                self.membership.clear();
                self.readiness.clear();
                for candidate in self.back.values_mut() {
                    *candidate = None;
                }
                return None;
            }
        };
        // A failed Windows arrival-property query makes the node unready, not a new generation.
        let mut unknown = BTreeSet::new();
        for (serial, sensor) in &mut next {
            if sensor.attachment.is_empty() {
                if let Some(old) = self.present.get(serial) {
                    sensor.attachment.clone_from(&old.attachment);
                }
                sensor.ready = false;
                unknown.insert(serial.clone());
            }
        }
        self.seen.retain(|serial, _| next.contains_key(serial));
        for (serial, sensor) in &next {
            if !sensor.attachment.is_empty()
                && self.seen.get(serial).map(|(a, _)| a) != Some(&sensor.attachment)
            {
                self.seen
                    .insert(serial.clone(), (sensor.attachment.clone(), now));
            }
        }
        // A new serial that disappears before admission must start its debounce again.
        self.membership
            .retain(|serial, _| self.present.contains_key(serial) || next.contains_key(serial));
        self.readiness
            .retain(|serial, _| self.present.contains_key(serial) || next.contains_key(serial));
        let serials: BTreeSet<_> = self.present.keys().chain(next.keys()).cloned().collect();
        let mut change = Change::default();
        for serial in serials {
            let settles =
                settles_any && !fresh.contains(&serial) && !self.adding.contains_key(&serial);
            let old = self.present.get(&serial).cloned();
            let new = next.get(&serial);
            // Filling in the first arrival property is not an unplug/replug.
            let same = match (&old, new) {
                (Some(a), Some(b)) => a.attachment.is_empty() || a.attachment == b.attachment,
                (None, None) => true,
                _ => false,
            };
            if same {
                self.membership.remove(&serial);
                if let (Some(old), Some(new)) = (old, new) {
                    if old.attachment.is_empty() && !new.attachment.is_empty() {
                        if self.attempted.get(&serial) == Some(&old.attachment) {
                            self.attempted
                                .insert(serial.clone(), new.attachment.clone());
                        }
                        self.present
                            .get_mut(&serial)
                            .unwrap()
                            .attachment
                            .clone_from(&new.attachment);
                    }
                    if settles && !unknown.contains(&serial) {
                        if let Some(seen @ None) = self.in_use.get_mut(&serial) {
                            *seen = Some(new.attachment.clone());
                        }
                    }
                    if !new.ready {
                        // Loss of access is immediate; only becoming ready needs to settle.
                        self.present.get_mut(&serial).unwrap().ready = false;
                        self.readiness.remove(&serial);
                    } else if !old.ready {
                        if self.ready_since(&serial, new, now) && settles {
                            self.present.get_mut(&serial).unwrap().ready = new.ready;
                            if new.ready
                                && self.attempted.get(&serial) != Some(&new.attachment)
                                && !self.in_use.contains_key(&serial)
                            {
                                change.ready.push(new.serial.clone());
                            }
                        }
                    } else {
                        self.readiness.remove(&serial);
                    }
                }
                continue;
            }
            // The old attachment is not eligible while a removal/replacement settles.
            if let Some(old) = self.present.get_mut(&serial) {
                old.ready = false;
            }
            if let Some(sensor) = new {
                self.ready_since(&serial, sensor, now);
            } else {
                self.readiness.remove(&serial);
            }
            let candidate = self
                .membership
                .entry(serial.clone())
                .or_insert_with(|| (new.cloned(), now));
            if candidate.0.as_ref().map(|s| &s.attachment) != new.map(|s| &s.attachment) {
                *candidate = (new.cloned(), now);
            }
            if !settles || now.saturating_sub(candidate.1) < DEBOUNCE_NS {
                continue;
            }
            self.membership.remove(&serial);
            // The engine's object stays in use unless this view rules out the device it opened.
            let seen = self.in_use.get(&serial).cloned();
            let keeps = match (&seen, new) {
                (Some(None), Some(_)) => true,
                (Some(Some(at)), Some(s)) => s.attachment == *at,
                _ => false,
            };
            if keeps {
                if let Some(s) = new.filter(|s| !s.attachment.is_empty()) {
                    self.in_use
                        .insert(serial.clone(), Some(s.attachment.clone()));
                }
            } else if let Some(old) = old {
                self.retire(&serial);
                if self.gone_seen.get(&serial) != Some(&old.attachment) {
                    change.unplugged.push(old.serial);
                }
            }
            self.present.remove(&serial);
            self.gone_retried.remove(&serial);
            self.back.remove(&serial);
            if let Some(sensor) = new {
                let mut sensor = sensor.clone();
                let stable = self.ready_since(&serial, &sensor, now);
                sensor.ready &= stable;
                self.present.insert(serial.clone(), sensor.clone());
                if !keeps {
                    change.plugged.push(sensor);
                }
            } else {
                self.readiness.remove(&serial);
            }
            // The attachment this listing has is the only one a live add may still remember.
            self.drop_unlisted_add(&serial);
        }
        // A native NO_DEVICE is evidence the old handle died. Confirm a ready USB attachment
        // on two later scans, even if a platform reused its persistent device instance ID.
        let mut returned = Vec::new();
        for (serial, candidate) in &mut self.back {
            if let Some(sensor) = next
                .get(serial)
                .filter(|s| s.ready && !s.attachment.is_empty())
            {
                match candidate {
                    Some((generation, since)) if *generation == sensor.attachment => {
                        if now.saturating_sub(*since) >= DEBOUNCE_NS {
                            returned.push(serial.clone());
                        }
                    }
                    _ => *candidate = Some((sensor.attachment.clone(), now)),
                }
            } else {
                *candidate = None;
            }
        }
        for serial in returned {
            self.back.remove(&serial);
            if let Some(sensor) = self.present.get(&serial) {
                change.plugged.push(sensor.clone());
            }
        }
        (change != Change::default()).then_some(change)
    }

    /// Only a definite native NO_DEVICE retires a handle; bandwidth and ordinary I/O errors do not.
    pub fn native_gone(&mut self, serial: &str) -> Option<Change> {
        let serial = sensor_identity(serial);
        // Whatever attachment it was opened at, this engine's handle is no longer in use.
        self.in_use.remove(&serial);
        if self.detached.contains(&serial) {
            return None;
        }
        let generation = self.present.get(&serial)?.attachment.clone();
        // Reported already: an object still in the roster reports it again on every snapshot
        // until it is released. Another attachment's object is another report.
        if self.gone_seen.get(&serial) == Some(&generation) {
            return None;
        }
        self.gone_seen
            .insert(serial.to_string(), generation.clone());
        let can_retry = self.gone_retried.get(&serial) != Some(&generation);
        if can_retry {
            self.gone_retried.insert(serial.to_string(), generation);
            self.back.insert(serial.to_string(), None);
            self.retire(&serial);
        } else {
            self.detached.insert(serial.to_string());
        }
        Some(Change {
            unplugged: vec![self.present[&serial].serial.clone()],
            ..Default::default()
        })
    }

    /// Consume this ready snapshot once. Actual native start records also reconcile attachments
    /// discovered during the open, including streams whose failed sensor object was freed.
    pub fn before_open(&mut self) {
        self.detached.clear();
        self.gone_seen.clear();
        self.back.clear();
        // This engine's roster replaces the preceding engine's attempts. An unready sensor
        // omitted now must remain eligible when access returns, unless native discovery tried it.
        self.attempted.clear();
        self.in_use.clear();
        // An engine start opens every ready attachment again, including failed live adds.
        self.running = false;
        self.adding.clear();
        self.fresh.clear();
        self.retry.clear();
        self.add_failed.clear();
        self.refused.clear();
        self.opening = true;
        self.discovering = true;
        self.settling = false;
        self.opening_serials.clear();
        self.opening_attachments = self
            .present
            .iter()
            .map(|(serial, s)| (serial.clone(), s.attachment.clone()))
            .collect();
        for (serial, (candidate, _)) in &self.membership {
            if let Some(sensor) = candidate {
                self.opening_attachments
                    .insert(serial.clone(), sensor.attachment.clone());
            }
        }
        for (serial, s) in &self.present {
            if s.ready && !self.membership.contains_key(serial) {
                self.attempted.insert(serial.clone(), s.attachment.clone());
                self.opening_serials.insert(serial.clone());
            }
        }
    }

    /// The engine could not open (the headset is missing or unusable): the sensors it would
    /// have opened wait for the headset until an open succeeds, through each retry.
    pub fn open_failed(&mut self) {
        self.opening = false;
        self.discovering = false;
        self.settling = true;
        self.engine_waiting = true;
        self.running = false;
    }

    /// The engine opened; it still initializes until `opened`.
    pub fn open_succeeded(&mut self) {
        self.engine_waiting = false;
    }

    /// The opened engine's first update is done: its sensors are in its roster, and from now on
    /// it adds sensors plugged in while it runs.
    pub fn opened(&mut self) {
        self.opening = false;
        self.discovering = false;
        self.opening_serials.clear();
        self.running = true;
    }

    /// The engine closed: none of its objects is in use any more, and the adds it was working
    /// on, or could not finish, went with it: the next engine start opens their sensors.
    pub fn closed(&mut self) {
        self.in_use.clear();
        self.opening = false;
        self.discovering = false;
        self.settling = true;
        self.opening_serials.clear();
        self.running = false;
        let unfinished = self
            .adding
            .iter()
            .chain(self.add_failed.iter().map(|(s, (a, _))| (s, a)))
            .chain(&self.refused);
        for (serial, attachment) in unfinished {
            if self.attempted.get(serial) == Some(attachment) {
                self.attempted.remove(serial);
            }
        }
        self.adding.clear();
        self.fresh.clear();
        self.retry.clear();
        self.add_failed.clear();
        self.refused.clear();
    }

    /// Whether a settled, ready attachment is due its first attempt in this engine, by an add.
    fn wants_add(&self, serial: &str, sensor: &Sensor, now: u64) -> bool {
        sensor.ready
            && !sensor.attachment.is_empty()
            && !self.membership.contains_key(serial)
            && !self.back.contains_key(serial)
            && !self.in_use.contains_key(serial)
            && !self.adding.contains_key(serial)
            // its old object is released first
            && !self.detached.contains(serial)
            && self.attempted.get(serial) != Some(&sensor.attachment)
            && self
                .retry
                .get(serial)
                .is_none_or(|(a, _, not_before)| *a != sensor.attachment || *not_before <= now)
    }

    /// The sensors the running engine is to add now, each marked as being added, its attempt
    /// spent. A detached serial waits until the engine has released its old object (`released`).
    pub fn take_adds(&mut self, now: u64) -> Vec<Sensor> {
        if !self.running || self.scan_error.is_some() {
            return Vec::new();
        }
        let due: Vec<(String, Sensor)> = self
            .present
            .iter()
            .filter(|(serial, sensor)| self.wants_add(serial, sensor, now))
            .map(|(serial, sensor)| (serial.clone(), sensor.clone()))
            .collect();
        for (serial, sensor) in &due {
            self.attempted
                .insert(serial.clone(), sensor.attachment.clone());
            self.adding
                .insert(serial.clone(), sensor.attachment.clone());
        }
        due.into_iter().map(|(_, sensor)| sensor).collect()
    }

    /// A removal or replacement has settled. A retry, a failed add or a refusal for an
    /// attachment this listing no longer has went with that attachment: the engine loop must
    /// not keep servicing it, and status must not keep its reason.
    fn drop_unlisted_add(&mut self, serial: &str) {
        let listed = self
            .present
            .get(serial)
            .map(|sensor| sensor.attachment.clone());
        let unlisted = |attachment: &String| listed.as_deref() != Some(attachment.as_str());
        if self
            .retry
            .get(serial)
            .is_some_and(|(attachment, ..)| unlisted(attachment))
        {
            self.retry.remove(serial);
        }
        if self
            .add_failed
            .get(serial)
            .is_some_and(|(attachment, _)| unlisted(attachment))
        {
            self.add_failed.remove(serial);
        }
        if self.refused.get(serial).is_some_and(unlisted) {
            self.refused.remove(serial);
        }
    }

    /// When the next live-add retry is due, if one is waiting for an attachment that is
    /// present and ready. Anything else is not a retry the engine loop waits on: a time in
    /// the past would make it service sensors on every pass.
    pub fn next_retry(&self) -> Option<u64> {
        self.retry
            .iter()
            .filter(|(serial, (attachment, _, _))| {
                self.present.get(*serial).is_some_and(|sensor| {
                    sensor.ready
                        && !sensor.attachment.is_empty()
                        && sensor.attachment == *attachment
                })
            })
            .map(|(_, (_, _, at))| *at)
            .min()
    }

    /// Every retry, failed add and refusal names the attachment still listed for its serial,
    /// and [`Self::next_retry`] names only one that is present and ready.
    pub fn live_add_records_current(&self) -> bool {
        let listed = |serial: &String, attachment: &String| {
            self.present
                .get(serial)
                .is_some_and(|sensor| sensor.attachment == *attachment)
        };
        let records = self
            .retry
            .iter()
            .all(|(serial, (attachment, _, _))| listed(serial, attachment))
            && self
                .add_failed
                .iter()
                .all(|(serial, (attachment, _))| listed(serial, attachment))
            && self
                .refused
                .iter()
                .all(|(serial, attachment)| listed(serial, attachment));
        let due = self.next_retry().is_none_or(|at| {
            self.retry.iter().any(|(serial, (attachment, _, when))| {
                *when == at
                    && self
                        .present
                        .get(serial)
                        .is_some_and(|sensor| sensor.ready && sensor.attachment == *attachment)
            })
        });
        records && due
    }

    /// A finished live add of `attachment`, and what the engine thread does with it. Called with
    /// the watch held while the engine adopts it: nothing about the serial settled while it was
    /// added, so the sensor it started is in use now, as one an open started, by no attachment
    /// yet; scans read after its start decide what its device is.
    pub fn add_finished(
        &mut self,
        serial: &str,
        attachment: &str,
        outcome: crate::engine::AddOutcome,
        error_code: i32,
        now: u64,
    ) -> AddVerdict {
        use crate::engine::AddOutcome;
        let serial = sensor_identity(serial);
        if self.adding.get(&serial).map(String::as_str) != Some(attachment) {
            return if outcome == AddOutcome::Started {
                AddVerdict::Release
            } else {
                AddVerdict::Done
            };
        }
        self.adding.remove(&serial);
        let transient = matches!(outcome, AddOutcome::NotFound | AddOutcome::Stale)
            || (outcome == AddOutcome::OpenFailed
                && matches!(
                    error_code,
                    LIBUSB_ERROR_ACCESS | LIBUSB_ERROR_BUSY | LIBUSB_ERROR_NOT_FOUND
                ));
        match outcome {
            AddOutcome::Started => {
                self.retry.remove(&serial);
                // a recovered attachment is open again: losing it again is a removal to report
                self.gone_seen.remove(&serial);
                self.in_use.insert(serial.clone(), None);
                self.fresh.insert(serial);
                AddVerdict::Use
            }
            // Recorded with the engine's start records, where a NO_DEVICE start is retired like
            // any other (`native_gone`); not tried again before the next attachment.
            AddOutcome::StartFailed => {
                self.retry.remove(&serial);
                AddVerdict::Done
            }
            AddOutcome::OpenFailed if error_code == LIBUSB_ERROR_NO_DEVICE => {
                self.retry.remove(&serial);
                self.native_gone(&serial)
                    .map_or(AddVerdict::Done, AddVerdict::Gone)
            }
            _ if transient => {
                let tries = match self.retry.get(&serial) {
                    Some((a, tries, _)) if a == attachment => *tries + 1,
                    _ => 1,
                };
                if tries > ADD_RETRIES {
                    self.retry.remove(&serial);
                    let detail = add_failure_detail(outcome, error_code);
                    self.add_failed
                        .insert(serial, (attachment.to_string(), detail.clone()));
                    return AddVerdict::Failed(detail);
                }
                self.retry.insert(
                    serial.clone(),
                    (attachment.to_string(), tries, now + ADD_RETRY_NS),
                );
                self.attempted.remove(&serial);
                AddVerdict::Retry { attempt: tries }
            }
            _ => {
                self.retry.remove(&serial);
                let detail = add_failure_detail(outcome, error_code);
                self.add_failed
                    .insert(serial, (attachment.to_string(), detail.clone()));
                AddVerdict::Failed(detail)
            }
        }
    }

    /// The engine already holds a live object of this serial: it is in use, as an add's would be.
    pub fn already_open(&mut self, serial: &str, attachment: &str) {
        let serial = sensor_identity(serial);
        if self.adding.get(&serial).map(String::as_str) == Some(attachment) {
            self.adding.remove(&serial);
            self.in_use.entry(serial.clone()).or_insert(None);
            self.fresh.insert(serial);
        }
    }

    /// The running engine could not be asked to add this sensor: the next engine start opens it.
    pub fn add_refused(&mut self, serial: &str, attachment: &str) {
        let serial = sensor_identity(serial);
        if self.adding.get(&serial).map(String::as_str) == Some(attachment) {
            self.adding.remove(&serial);
            self.refused.insert(serial, attachment.to_string());
        }
    }

    /// The engine released the detached serial's native object and forgot its start record: a
    /// new attachment of it can be added.
    pub fn released(&mut self, serial: &str) {
        self.detached.remove(&sensor_identity(serial));
    }

    /// When a scan first listed this attachment, if it still lists it.
    pub fn plugged_at(&self, serial: &str, attachment: &str) -> Option<u64> {
        self.seen
            .get(&sensor_identity(serial))
            .filter(|(a, _)| a == attachment)
            .map(|(_, at)| *at)
    }

    /// Whether an engine runs and adds plugged-in sensors itself.
    pub fn running(&self) -> bool {
        self.running
    }

    /// Whether an engine opens a sensor plugged in now: one that runs, or one opening, which
    /// opens it or adds it once it runs.
    pub fn engine_adds(&self) -> bool {
        self.running || self.opening
    }

    /// Whether the engine has sensor work: an old object to release (a retirement need not say
    /// unplugged twice), or, running, a sensor to add, now or when its retry is due.
    pub fn engine_work(&self) -> bool {
        !self.detached.is_empty() || self.addable()
    }

    /// Whether the running engine has a sensor to add, now or when its retry is due.
    pub fn addable(&self) -> bool {
        self.running
            && self.scan_error.is_none()
            && self
                .present
                .iter()
                .any(|(serial, sensor)| self.wants_add(serial, sensor, u64::MAX))
    }

    /// Whether a live add is under way.
    pub fn adding(&self) -> bool {
        !self.adding.is_empty()
    }

    pub fn pending(&self) -> Vec<PendingSensor> {
        self.present.iter().filter_map(|(serial, sensor)| {
            let waiting = !self.in_use.contains_key(serial)
                && self.attempted.get(serial) != Some(&sensor.attachment);
            // A replacement or removal still settling, a NO_DEVICE being confirmed on a ready
            // attachment, or access that the OS has just granted.
            let checking = self.membership.contains_key(serial)
                || if sensor.ready {
                    self.back.contains_key(serial)
                } else {
                    self.readiness.get(serial).is_some_and(|(attachment, ready, _)| {
                        *ready && *attachment == sensor.attachment
                    })
                };
            // Why this attachment's add failed, unless a change of it is being checked now, or USB
            // cannot be read to tell.
            let known = !checking && self.scan_error.is_none();
            let ours = |attachment: &String| known && *attachment == sensor.attachment;
            let failed = self.add_failed.get(serial).filter(|(a, _)| ours(a));
            let refused = self.refused.get(serial).is_some_and(ours);
            let (state, detail) = if self.engine_waiting && self.opening_serials.contains(serial) {
                ("waiting_headset", "Waiting for the headset; this sensor is opened with it.")
            } else if (self.opening && self.opening_serials.contains(serial)) || self.adding.contains_key(serial) {
                ("opening", ADDING_DETAIL)
            } else if let Some((_, detail)) = failed {
                return Some(PendingSensor { serial: sensor.serial.clone(), state: "open_failed", detail: detail.clone() });
            } else if refused {
                return Some(PendingSensor { serial: sensor.serial.clone(), state: "waiting_engine_start", detail: add_refused_detail() });
            } else if self.gone_seen.get(serial) == Some(&sensor.attachment) && !waiting {
                ("waiting_replug", "Its USB device went away; unplug this sensor and plug it back in.")
            } else if !waiting { return None; }
            // Until its roster is known, the opening engine may be using it.
            else if self.discovering { ("opening", OPENING_DETAIL) }
            else if self.scan_error.is_some() {
                ("waiting_usb", "USB information is unavailable; the daemon is still checking for this sensor.")
            } else if checking { ("checking_usb", CHECKING_DETAIL) }
            else if !sensor.ready {
                // Linux access can stay closed (no udev rule, or not the active seat). Windows
                // waits until the driver interfaces are active. Say which one this build is.
                ("waiting_usb", usb_access_detail())
            } else if self.opening { ("opening", OPENING_DETAIL) }
            else if self.engine_waiting { ("waiting_headset", "Waiting for the headset; this sensor is opened with it.") }
            // the running engine adds it on its next pass, once its old object is released, or
            // when its retry is due
            else if self.running { ("opening", ADDING_DETAIL) }
            else { ("waiting_engine_start", engine_start_detail()) };
            Some(PendingSensor { serial: sensor.serial.clone(), state, detail: detail.to_string() })
        }).collect()
    }

    /// The engine's last-start table is its roster, including failed starts, and replaces what
    /// `before_open` expected it to try. `inventory` is the listing read after the open, or
    /// `None` if it failed. Every stream that started is in use, known by the attachment that
    /// listing shows for its serial. A failed start spends at most one attachment's attempt,
    /// remembering native discoveries before the watcher admits them, without spending a
    /// replacement's attempt on its preceding attachment's result.
    pub fn reconcile_open(
        &mut self,
        now: u64,
        starts: &[crate::engine::SensorStreamStart],
        inventory: Option<&[Sensor]>,
    ) -> Change {
        let mut change = Change::default();
        let inventoried = inventory.is_some();
        let mut inventory = listing(inventory.unwrap_or_default().to_vec(), &[]);
        let listed: BTreeSet<_> = inventory
            .iter()
            .filter(|(_, s)| !s.attachment.is_empty())
            .map(|(serial, _)| serial.clone())
            .collect();
        for (serial, sensor) in &mut inventory {
            if sensor.attachment.is_empty() {
                if let Some(generation) = self.known_attachment(serial) {
                    sensor.attachment.clone_from(generation);
                }
            }
        }
        // A sensor that listing lacks has left, as a watcher scan would show it: it is not ready,
        // and its removal settles like any other. Nothing says it waits for the next start.
        if inventoried {
            for (serial, sensor) in &mut self.present {
                if !inventory.contains_key(serial) {
                    sensor.ready = false;
                    self.readiness.remove(serial);
                    let candidate = self.membership.entry(serial.clone()).or_insert((None, now));
                    if candidate.0.is_some() {
                        *candidate = (None, now);
                    }
                }
            }
        }
        let expected = std::mem::take(&mut self.attempted);
        self.opening_serials.clear();
        for start in starts {
            let serial = sensor_identity(&start.serial);
            if start.error_kind == "none" {
                let seen = inventory.get(&serial).filter(|_| listed.contains(&serial));
                match seen {
                    Some(sensor) => {
                        let sensor = sensor.clone();
                        self.observe(&serial, &sensor, &mut change);
                    }
                    // Unlisted: it is in use all the same. Only its absence or its NO_DEVICE
                    // can tell it left.
                    None if !self.present.contains_key(&serial) => {
                        let unknown = Sensor {
                            serial: start.serial.clone(),
                            attachment: String::new(),
                            ready: false,
                        };
                        self.present.insert(serial.clone(), unknown.clone());
                        change.plugged.push(unknown);
                    }
                    None => {}
                }
                self.in_use
                    .insert(serial.clone(), seen.map(|s| s.attachment.clone()));
                self.opening_serials.insert(serial);
                continue;
            }
            // A failed start spends one attachment's attempt: the one its record names, as the
            // listing after the open shows it or, unlisted, as the watcher knows it. A record
            // that names none (a failed Linux pin) spends the attempt `before_open` expected,
            // unless the listing after the open shows another attachment.
            let sensor = inventory.get(&serial).cloned();
            let shown = sensor
                .as_ref()
                .filter(|_| listed.contains(&serial))
                .map(|s| s.attachment.clone());
            let attachment = match &start.attachment {
                Some(named) => sensor
                    .as_ref()
                    .map(|s| s.attachment.clone())
                    .filter(|a| !a.is_empty())
                    .or_else(|| self.known_attachment(&serial).cloned())
                    .filter(|a| a == named),
                None => expected
                    .get(&serial)
                    .filter(|a| shown.as_ref().is_none_or(|s| s == *a))
                    .cloned(),
            };
            let Some(attachment) = attachment else {
                continue;
            };
            match self.membership.get(&serial) {
                // The watcher saw yet another attachment: this start may be the stale one.
                Some((Some(s), _)) if !s.attachment.is_empty() && s.attachment != attachment => {
                    continue
                }
                Some((Some(_), _)) => {
                    self.membership.remove(&serial);
                }
                // A removal still settling decides for itself.
                Some((None, _)) | None => {}
            }
            if let Some(sensor) = sensor {
                self.observe(&serial, &sensor, &mut change);
            }
            self.attempted.insert(serial.clone(), attachment);
            self.opening_serials.insert(serial);
        }
        // What `before_open` expected it to try but its roster does not account for (access went
        // away for a moment, or the listing after the open contradicts an unnamed failure) waits
        // for the next start.
        for (serial, attachment) in expected {
            if let Some(sensor) = self.present.get(&serial).filter(|s| {
                s.ready
                    && s.attachment == attachment
                    && !self.in_use.contains_key(&serial)
                    && !self.attempted.contains_key(&serial)
            }) {
                change.unopened.push(sensor.serial.clone());
            }
        }
        self.discovering = false;
        change
    }

    /// The attachment the watcher knows for `serial`, or knew when this engine began opening.
    fn known_attachment(&self, serial: &str) -> Option<&String> {
        self.present
            .get(serial)
            .map(|s| &s.attachment)
            .filter(|generation| !generation.is_empty())
            .or_else(|| self.opening_attachments.get(serial))
    }

    /// Native discovery attempted this current USB attachment. Remember it even before the
    /// watcher has admitted it, or in place of the attachment it replaced, so detach and
    /// NO_DEVICE can retire it.
    fn observe(&mut self, serial: &str, sensor: &Sensor, change: &mut Change) {
        match self.present.get_mut(serial) {
            Some(old) if old.attachment.is_empty() => old.attachment.clone_from(&sensor.attachment),
            Some(old) if old.attachment == sensor.attachment => {}
            old => {
                // A serial names one device: the attachment it replaced left.
                if let Some(old) = old.filter(|old| !old.attachment.is_empty()) {
                    change.unplugged.push(old.serial.clone());
                }
                let mut observed = sensor.clone();
                observed.ready = false;
                self.present.insert(serial.to_string(), observed);
                self.gone_retried.remove(serial);
                change.plugged.push(sensor.clone());
            }
        }
    }

    /// Whether the current engine holds a live object for `serial`.
    pub fn in_use(&self, serial: &str) -> bool {
        self.in_use.contains_key(&sensor_identity(serial))
    }

    /// Whether a sensor waits for the next engine start (see `pending`).
    #[cfg(test)]
    pub(crate) fn waits_for_engine_start(&self) -> bool {
        self.pending()
            .iter()
            .any(|p| p.state == "waiting_engine_start")
    }

    /// Whether a settled, ready attachment waits for its first attempt: a running engine adds it
    /// (`take_adds`) once its old object is released, or the next engine start opens it.
    #[cfg(test)]
    pub(crate) fn wants_open(&self) -> bool {
        self.scan_error.is_none()
            && self.present.iter().any(|(serial, sensor)| {
                let mut released = self.clone();
                released.detached.remove(serial);
                released.wants_add(serial, sensor, u64::MAX)
            })
    }

    pub fn scan_error(&self) -> Option<&str> {
        self.scan_error.as_deref()
    }
    pub fn admitted(&self, serial: &str) -> bool {
        !self.detached.iter().any(|s| same_sensor(s, serial))
    }
    pub fn detached(&self) -> &BTreeSet<String> {
        &self.detached
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sensor(serial: &str, attachment: &str, ready: bool) -> Sensor {
        Sensor {
            serial: serial.into(),
            attachment: attachment.into(),
            ready,
        }
    }
    fn change(m: &mut Monitor, at: u64, devices: Vec<Sensor>) -> Option<Change> {
        assert!(m.update(at, Ok(devices.clone()), &[]).is_none());
        m.update(at + DEBOUNCE_NS, Ok(devices), &[])
    }
    fn states(m: &Monitor) -> Vec<&'static str> {
        m.pending().iter().map(|p| p.state).collect()
    }

    #[test]
    fn an_unready_sensor_names_this_platforms_usb_wait() {
        let m = Monitor::new(Ok(vec![sensor("SENSORALPHA", "port-a-1", false)]), &[]);
        let pending = m.pending();
        assert_eq!(pending.len(), 1);
        assert_eq!(pending[0].state, "waiting_usb");
        let detail = pending[0].detail.as_str();
        if cfg!(windows) {
            assert_eq!(detail, "This sensor's driver interfaces aren't active yet.");
            assert!(!detail.to_ascii_lowercase().contains("permission"));
        } else {
            assert_eq!(
                detail,
                "This sensor's USB device isn't accessible yet (permissions)."
            );
            assert!(detail.contains("permissions"));
        }
        assert!(!detail.contains("Windows"));
    }

    use crate::engine::AddOutcome;

    /// A running engine with no sensors yet.
    fn running() -> Monitor {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        m.before_open();
        m.opened();
        m
    }

    /// A sensor plugged into a running engine is added by it, which status shows as the engine
    /// opening it; once added it is in use, and nothing reopens the engine.
    #[test]
    fn hotplug_a_running_engine_adds_a_plugged_in_sensor_itself() {
        let mut m = running();
        let a = sensor("SENSORALPHA", "port-a-1", true);
        assert!(m.update(10, Ok(vec![a.clone()]), &[]).is_none());
        // still settling: not yet
        assert!(m.take_adds(10).is_empty() && !m.addable());
        assert!(m.pending().is_empty());
        m.update(10 + DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
            .unwrap();
        assert_eq!(m.plugged_at(&a.serial, &a.attachment), Some(10));
        // ready and settled: the engine takes it on its next pass
        assert!(m.addable());
        assert_eq!(details(&m), [("opening", ADDING_DETAIL.to_string())]);
        assert_eq!(m.take_adds(10 + DEBOUNCE_NS), std::slice::from_ref(&a));
        assert!(m.adding() && m.take_adds(10 + DEBOUNCE_NS).is_empty() && !m.addable());
        assert_eq!(states(&m), ["opening"]);
        assert_eq!(
            m.add_finished("sensoralpha", &a.attachment, AddOutcome::Started, 0, 0),
            AddVerdict::Use
        );
        assert!(m.in_use(&a.serial));
        assert!(!m.adding() && m.pending().is_empty() && !m.wants_open());
        // no reopen, and the next scans leave it alone
        assert!(m
            .update(5 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
            .is_none());
        assert!(m.take_adds(5 * DEBOUNCE_NS).is_empty());
        assert!(m.in_use(&a.serial));
    }

    /// No engine runs (no headset, one closing for a mode change, or before the first): the next
    /// engine start opens the sensor, and only then is the next start the reason it waits.
    #[test]
    fn hotplug_without_a_running_engine_the_next_start_opens_the_sensor() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        let mut m = running();
        m.closed();
        change(&mut m, 1, vec![a.clone()]).unwrap();
        assert!(m.take_adds(2 * DEBOUNCE_NS).is_empty() && !m.addable());
        let pending = m.pending();
        assert_eq!(pending[0].state, "waiting_engine_start");
        let detail = pending[0].detail.as_str();
        assert!(
            detail
                .starts_with("This sensor will be used the next time the tracking engine starts."),
            "{detail}"
        );
        if cfg!(windows) {
            assert!(detail.contains("Stop cv1touch, then Start cv1touch"));
        } else {
            assert!(detail.contains("systemctl --user restart cv1touchd"));
        }
        m.before_open();
        m.open_failed();
        assert_eq!(states(&m), ["waiting_headset"]);
        assert!(m.take_adds(3 * DEBOUNCE_NS).is_empty());
        m.before_open();
        m.open_succeeded();
        assert_eq!(states(&m), ["opening"]);
        m.opened();
        assert!(m.pending().is_empty());
        assert!(
            m.take_adds(4 * DEBOUNCE_NS).is_empty(),
            "the start opened it"
        );
    }

    /// An engine that closes during an add leaves the sensor to the next engine start, which
    /// opens it; the closed engine's result is never adopted.
    #[test]
    fn hotplug_an_add_the_closing_engine_took_with_it_is_opened_at_the_next_start() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        let mut m = running();
        change(&mut m, 1, vec![a.clone()]).unwrap();
        assert_eq!(m.take_adds(2 * DEBOUNCE_NS), std::slice::from_ref(&a));
        m.closed();
        assert!(!m.adding());
        assert_eq!(states(&m), ["waiting_engine_start"]);
        assert_eq!(
            m.add_finished(&a.serial, &a.attachment, AddOutcome::Started, 0, 0),
            AddVerdict::Release
        );
        assert!(!m.in_use(&a.serial));
        m.before_open();
        assert_eq!(states(&m), ["opening"]);
        m.opened();
        assert!(m.pending().is_empty());
    }

    /// Unplugged, or replugged elsewhere, while it is added: nothing about it settles until its
    /// add ends, so nothing logged or shown claims what the add will do. The sensor the engine
    /// started is in use, by no connection yet: a departure retires it only once listings read
    /// after its start settle it, from the second scan applied after its adoption, and a
    /// replacement does not (the engine may have opened that one), until its own NO_DEVICE.
    #[test]
    fn hotplug_a_sensor_unplugged_during_its_add_is_judged_by_listings_read_after_its_start() {
        for replaced in [false, true] {
            let a = sensor("SENSORALPHA", "port-a-1", true);
            let mut m = running();
            change(&mut m, 1, vec![a.clone()]).unwrap();
            assert_eq!(m.take_adds(2 * DEBOUNCE_NS), std::slice::from_ref(&a));
            let mut now_listed = vec![];
            if replaced {
                now_listed.push(sensor("SENSORALPHA", "port-a-2", true));
            }
            assert!(change(&mut m, 3 * DEBOUNCE_NS, now_listed.clone()).is_none());
            assert!(m.admitted(&a.serial));
            assert_eq!(states(&m), ["opening"]);
            assert_eq!(
                m.add_finished(
                    &a.serial,
                    &a.attachment,
                    AddOutcome::Started,
                    0,
                    4 * DEBOUNCE_NS
                ),
                AddVerdict::Use
            );
            assert!(m.in_use(&a.serial));
            assert!(
                m.update(5 * DEBOUNCE_NS, Ok(now_listed.clone()), &[])
                    .is_none(),
                "read before its start, maybe"
            );
            let settled = m.update(6 * DEBOUNCE_NS, Ok(now_listed.clone()), &[]);
            if replaced {
                assert!(settled.is_none(), "{settled:?}");
                assert!(m.in_use(&a.serial) && m.admitted(&a.serial));
                assert!(m.pending().is_empty());
                // its own NO_DEVICE: retired; the replacement is added once two scans confirm it
                assert_eq!(
                    m.native_gone(&a.serial).unwrap().unplugged,
                    std::slice::from_ref(&a.serial)
                );
                m.released(&a.serial);
                m.update(7 * DEBOUNCE_NS, Ok(now_listed.clone()), &[]);
                m.update(8 * DEBOUNCE_NS, Ok(now_listed.clone()), &[])
                    .unwrap();
                assert_eq!(m.take_adds(8 * DEBOUNCE_NS), now_listed);
            } else {
                assert_eq!(settled.unwrap().unplugged, std::slice::from_ref(&a.serial));
                assert!(!m.in_use(&a.serial) && !m.admitted(&a.serial));
                m.released(&a.serial);
                assert!(m.pending().is_empty() && m.take_adds(7 * DEBOUNCE_NS).is_empty());
            }
        }
    }

    /// Replugged faster than a scan settles: one change unplugs and plugs it; the old object is
    /// released before the new connection is added.
    #[test]
    fn hotplug_a_quick_replug_releases_before_it_adds() {
        let mut a = sensor("SENSORALPHA", "port-a-1", true);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        m.before_open();
        m.reconcile_open(
            1,
            &[stream_start(&a.serial, &a.attachment, false)],
            Some(&[a.clone()]),
        );
        m.opened();
        a.attachment = "port-a-2".into();
        assert!(m.update(2, Ok(vec![a.clone()]), &[]).is_none());
        let c = change(&mut m, 3, vec![a.clone()]).unwrap();
        assert_eq!((c.unplugged.len(), c.plugged.len()), (1, 1));
        assert!(m.take_adds(2 * DEBOUNCE_NS).is_empty());
        assert_eq!(m.detached().iter().collect::<Vec<_>>(), ["SENSORALPHA"]);
        assert_eq!(states(&m), ["opening"]);
        m.released(&a.serial);
        assert_eq!(m.take_adds(2 * DEBOUNCE_NS), [a]);
    }

    /// A device that is not there yet, or busy for a moment (uvcvideo letting go of it), is
    /// tried again a second later, up to three times; then the reason is shown, until a replug.
    #[test]
    fn hotplug_transient_open_failures_are_retried_three_times() {
        for (outcome, code) in [
            (AddOutcome::OpenFailed, LIBUSB_ERROR_BUSY),
            (AddOutcome::OpenFailed, LIBUSB_ERROR_ACCESS),
            (AddOutcome::OpenFailed, LIBUSB_ERROR_NOT_FOUND),
            (AddOutcome::NotFound, 0),
            (AddOutcome::Stale, 0),
        ] {
            let a = sensor("SENSORALPHA", "port-a-1", true);
            let mut m = running();
            change(&mut m, 1, vec![a.clone()]).unwrap();
            let mut now = 2 * DEBOUNCE_NS;
            for attempt in 1..=ADD_RETRIES {
                assert_eq!(
                    m.take_adds(now),
                    std::slice::from_ref(&a),
                    "{outcome:?} {attempt}"
                );
                assert_eq!(
                    m.add_finished(&a.serial, &a.attachment, outcome, code, now),
                    AddVerdict::Retry { attempt }
                );
                assert_eq!(states(&m), ["opening"]);
                assert!(m.take_adds(now + ADD_RETRY_NS - 1).is_empty(), "too early");
                assert_eq!(m.next_retry(), Some(now + ADD_RETRY_NS));
                now += ADD_RETRY_NS;
            }
            assert_eq!(m.take_adds(now), std::slice::from_ref(&a));
            let AddVerdict::Failed(detail) =
                m.add_finished(&a.serial, &a.attachment, outcome, code, now)
            else {
                panic!("{outcome:?} {code} was retried a fourth time");
            };
            assert_eq!(m.next_retry(), None);
            assert!(
                detail.ends_with("unplug the sensor and plug it back in."),
                "{detail}"
            );
            assert_eq!(states(&m), ["open_failed"]);
            assert_eq!(m.pending()[0].detail, detail);
            for later in 1..4 {
                assert!(m.take_adds(now + later * ADD_RETRY_NS).is_empty());
            }
            // plugged in again: a new attachment, a new attempt
            let b = sensor("SENSORALPHA", "port-a-2", true);
            change(&mut m, now + 10 * DEBOUNCE_NS, vec![b.clone()]).unwrap();
            m.released(&b.serial);
            assert_eq!(m.take_adds(now + 12 * DEBOUNCE_NS), [b]);
            assert_eq!(states(&m), ["opening"]);
        }
    }

    /// Other failures are not retried: a failed start is recorded with the engine's start
    /// records (doctor gives its advice), and an open that failed otherwise says why. A sensor
    /// the engine could not be asked to add waits for the next engine start, which opens it.
    #[test]
    fn hotplug_lasting_failures_are_not_retried() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        for (outcome, code, state) in [
            (AddOutcome::StartFailed, -1, None),
            (AddOutcome::StartFailed, LIBUSB_ERROR_NO_DEVICE, None),
            (AddOutcome::OpenFailed, -99, Some("open_failed")),
            (AddOutcome::Rejected, 0, Some("open_failed")),
        ] {
            let mut m = running();
            change(&mut m, 1, vec![a.clone()]).unwrap();
            assert_eq!(m.take_adds(2 * DEBOUNCE_NS), std::slice::from_ref(&a));
            let verdict = m.add_finished(&a.serial, &a.attachment, outcome, code, 2 * DEBOUNCE_NS);
            assert_eq!(
                states(&m),
                state.into_iter().collect::<Vec<_>>(),
                "{outcome:?}"
            );
            assert_eq!(matches!(verdict, AddVerdict::Failed(_)), state.is_some());
            assert!(!m.in_use(&a.serial));
            for later in 3..8 {
                m.update(later * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]);
                assert!(
                    m.take_adds(later * DEBOUNCE_NS).is_empty() && !m.addable(),
                    "{outcome:?} looped"
                );
            }
            // a closed engine leaves it to the next start, which tries again
            let mut closed = m.clone();
            closed.closed();
            if state.is_some() {
                assert_eq!(states(&closed), ["waiting_engine_start"]);
            }
            closed.before_open();
            closed.opened();
            assert!(closed.pending().is_empty());
        }
        let mut m = running();
        change(&mut m, 1, vec![a.clone()]).unwrap();
        m.take_adds(2 * DEBOUNCE_NS);
        m.add_refused(&a.serial, &a.attachment);
        assert_eq!(states(&m), ["waiting_engine_start"]);
        let detail = m.pending()[0].detail.clone();
        assert!(
            detail.starts_with("The running engine could not take this sensor. This sensor will be used the next time the tracking engine starts."),
            "{detail}"
        );
        assert!(
            detail.contains(if cfg!(windows) {
                "Start cv1touch"
            } else {
                "systemctl --user restart cv1touchd"
            }),
            "{detail}"
        );
        assert!(!m.adding() && !m.addable());
        // the next engine start tries everything again
        m.before_open();
        m.opened();
        assert!(m.pending().is_empty());
    }

    /// A USB device that went away while it was being opened is retired like a native removal:
    /// once two scans confirm it is back, it is added again, once.
    #[test]
    fn hotplug_no_device_while_opening_gets_one_bounded_recovery() {
        let a = sensor("SENSORALPHA", "same-instance", true);
        let mut m = running();
        change(&mut m, 1, vec![a.clone()]).unwrap();
        assert_eq!(m.take_adds(2 * DEBOUNCE_NS), std::slice::from_ref(&a));
        let AddVerdict::Gone(gone) = m.add_finished(
            &a.serial,
            &a.attachment,
            AddOutcome::OpenFailed,
            LIBUSB_ERROR_NO_DEVICE,
            2 * DEBOUNCE_NS,
        ) else {
            panic!("NO_DEVICE was not a removal");
        };
        assert_eq!(gone.unplugged, std::slice::from_ref(&a.serial));
        assert!(!m.admitted(&a.serial));
        m.released(&a.serial);
        assert!(
            m.take_adds(3 * DEBOUNCE_NS).is_empty(),
            "not before two scans"
        );
        assert!(m
            .update(3 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
            .is_none());
        assert_eq!(
            m.update(4 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
                .unwrap()
                .plugged,
            std::slice::from_ref(&a)
        );
        assert_eq!(m.take_adds(4 * DEBOUNCE_NS), std::slice::from_ref(&a));
        // the same attachment failing again asks for a replug
        assert_eq!(
            m.add_finished(
                &a.serial,
                &a.attachment,
                AddOutcome::OpenFailed,
                LIBUSB_ERROR_NO_DEVICE,
                4 * DEBOUNCE_NS,
            ),
            AddVerdict::Done
        );
        for later in 5..9 {
            m.update(later * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]);
            assert!(m.take_adds(later * DEBOUNCE_NS).is_empty());
        }
        assert_eq!(states(&m), ["waiting_replug"]);
        // after a recovery that worked, losing it again is reported, and asks for a replug
        let mut m = running();
        change(&mut m, 1, vec![a.clone()]).unwrap();
        m.take_adds(2 * DEBOUNCE_NS);
        assert!(matches!(
            m.add_finished(
                &a.serial,
                &a.attachment,
                AddOutcome::OpenFailed,
                LIBUSB_ERROR_NO_DEVICE,
                0
            ),
            AddVerdict::Gone(_)
        ));
        m.released(&a.serial);
        m.update(3 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]);
        m.update(4 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]).unwrap();
        assert_eq!(m.take_adds(4 * DEBOUNCE_NS), std::slice::from_ref(&a));
        assert_eq!(
            m.add_finished(&a.serial, &a.attachment, AddOutcome::Started, 0, 0),
            AddVerdict::Use
        );
        assert_eq!(
            m.native_gone(&a.serial).unwrap().unplugged,
            std::slice::from_ref(&a.serial)
        );
        m.released(&a.serial);
        m.update(5 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]);
        assert_eq!(states(&m), ["waiting_replug"]);
        assert!(m.take_adds(6 * DEBOUNCE_NS).is_empty());
    }

    /// A NO_DEVICE reported for the replacement of a sensor whose own NO_DEVICE was handled is a
    /// report of its own, with its own recovery: the first one's is not held against it.
    #[test]
    fn hotplug_a_replacements_no_device_is_its_own_report() {
        let a1 = sensor("SENSORALPHA", "port-a-1", true);
        let a2 = sensor("SENSORALPHA", "port-a-2", true);
        let mut m = Monitor::new(Ok(vec![a1.clone()]), &[]);
        m.before_open();
        m.reconcile_open(
            1,
            &[stream_start(&a1.serial, &a1.attachment, false)],
            Some(std::slice::from_ref(&a1)),
        );
        m.opened();
        assert!(m.native_gone(&a1.serial).is_some());
        assert!(m.native_gone(&a1.serial).is_none(), "the same report again");
        m.released(&a1.serial);
        change(&mut m, 2 * DEBOUNCE_NS, vec![a2.clone()]).unwrap();
        m.released(&a2.serial);
        assert_eq!(m.take_adds(3 * DEBOUNCE_NS), std::slice::from_ref(&a2));
        // its start failed with NO_DEVICE: the start record reports it
        assert_eq!(
            m.add_finished(
                &a2.serial,
                &a2.attachment,
                AddOutcome::StartFailed,
                LIBUSB_ERROR_NO_DEVICE,
                0
            ),
            AddVerdict::Done
        );
        let gone = m
            .native_gone(&a2.serial)
            .expect("the replacement's NO_DEVICE was ignored");
        assert_eq!(gone.unplugged, std::slice::from_ref(&a2.serial));
        assert_eq!(states(&m), ["checking_usb"], "it gets its own recovery");
    }

    /// The engine already holds a live object of the serial: it is in use, as an add's is.
    #[test]
    fn hotplug_a_sensor_the_engine_already_has_is_in_use() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        let mut m = running();
        change(&mut m, 1, vec![a.clone()]).unwrap();
        assert_eq!(m.take_adds(2 * DEBOUNCE_NS), std::slice::from_ref(&a));
        m.already_open(&a.serial, &a.attachment);
        assert!(m.in_use(&a.serial) && !m.adding() && m.pending().is_empty());
        assert!(m.take_adds(3 * DEBOUNCE_NS).is_empty());
    }

    /// A settled reopen and repeated add cycles leave nothing behind: what an add, a retry or a
    /// failure kept goes with the engine, or with the sensor's listing. An unplug while a retry
    /// waits does too, and a sensor that is not ready does not keep a retry due.
    #[test]
    fn hotplug_live_adds_keep_nothing_once_their_sensor_is_gone() {
        let mut m = running();
        m.update(0, Ok(vec![]), &[]);
        let settled = format!("{m:?}");
        for round in 0..3u64 {
            let at = |tick: u64| (10 * round + tick) * DEBOUNCE_NS;
            let a = sensor("SENSORALPHA", &format!("port-a-{round}"), true);
            change(&mut m, at(1), vec![a.clone()]).unwrap();
            assert_eq!(m.take_adds(at(2)), std::slice::from_ref(&a));
            m.add_finished(
                &a.serial,
                &a.attachment,
                AddOutcome::OpenFailed,
                LIBUSB_ERROR_BUSY,
                at(2),
            );
            assert_eq!(m.take_adds(at(3)), std::slice::from_ref(&a));
            m.add_finished(&a.serial, &a.attachment, AddOutcome::Started, 0, at(3));
            change(&mut m, at(4), vec![]).unwrap();
            m.update(at(6), Ok(vec![]), &[]);
            m.released(&a.serial);
        }
        assert_eq!(format!("{m:?}"), settled);

        // Access drops while a retry waits. The entry stays, for when the sensor is ready
        // again, but it is not due: the engine loop would otherwise service on every pass.
        let a = sensor("SENSORALPHA", "port-a-retry", true);
        let t0 = 100 * DEBOUNCE_NS;
        change(&mut m, t0, vec![a.clone()]).unwrap();
        let now = t0 + DEBOUNCE_NS;
        assert_eq!(m.take_adds(now), std::slice::from_ref(&a));
        assert_eq!(
            m.add_finished(
                &a.serial,
                &a.attachment,
                AddOutcome::OpenFailed,
                LIBUSB_ERROR_BUSY,
                now
            ),
            AddVerdict::Retry { attempt: 1 }
        );
        let due = now + ADD_RETRY_NS;
        assert_eq!(m.next_retry(), Some(due));
        let mut dark = a.clone();
        dark.ready = false;
        assert!(m.update(due - 1, Ok(vec![dark]), &[]).is_none());
        assert_eq!(m.next_retry(), None, "an unready sensor left a retry due");
        assert!(m.live_add_records_current());
        assert!(m.update(due - 1, Ok(vec![a.clone()]), &[]).is_none());
        assert_eq!(m.next_retry(), None, "access came back before it settled");
        m.update(due - 1 + DEBOUNCE_NS, Ok(vec![a.clone()]), &[]);
        assert_eq!(
            m.next_retry(),
            Some(due),
            "the retry was dropped while it waited"
        );

        // Unplugged before that retry runs. The first scan clears readiness, so it is not
        // due while the removal settles; once it has settled the entry is gone.
        let leaving = due + DEBOUNCE_NS;
        assert!(m.update(leaving, Ok(vec![]), &[]).is_none());
        assert_eq!(
            m.next_retry(),
            None,
            "a sensor on its way out left a retry due"
        );
        assert!(m.live_add_records_current());
        m.update(leaving + DEBOUNCE_NS, Ok(vec![]), &[]).unwrap();
        m.released(&a.serial);
        assert_eq!(m.next_retry(), None);
        assert!(m.live_add_records_current());
        assert_eq!(format!("{m:?}"), settled, "a retry outlived its sensor");

        // A failed add and a refusal are the same: nothing remains once the sensor is gone.
        for (kind, fail) in [("failed", true), ("refused", false)] {
            let b = sensor("SENSORALPHA", &format!("port-a-{kind}"), true);
            let base = (300 + if fail { 0 } else { 20 }) * DEBOUNCE_NS;
            change(&mut m, base, vec![b.clone()]).unwrap();
            assert_eq!(m.take_adds(base + DEBOUNCE_NS), std::slice::from_ref(&b));
            if fail {
                m.add_finished(
                    &b.serial,
                    &b.attachment,
                    AddOutcome::Rejected,
                    0,
                    base + DEBOUNCE_NS,
                );
                assert_eq!(states(&m), ["open_failed"]);
            } else {
                m.add_refused(&b.serial, &b.attachment);
                assert_eq!(states(&m), ["waiting_engine_start"]);
            }
            change(&mut m, base + 3 * DEBOUNCE_NS, vec![]).unwrap();
            m.released(&b.serial);
            assert_eq!(m.next_retry(), None);
            assert!(m.live_add_records_current());
            assert_eq!(format!("{m:?}"), settled, "{kind} outlived its sensor");
        }
    }

    #[test]
    fn hotplug_sensors_wait_for_the_headset_while_the_engine_cannot_open() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        m.before_open();
        assert_eq!(states(&m), ["opening"]);
        // the headset is missing: every retry fails, and the sensor waits for it meanwhile
        for _ in 0..3 {
            m.open_failed();
            assert_eq!(states(&m), ["waiting_headset"]);
            m.before_open();
            assert_eq!(states(&m), ["waiting_headset"]);
        }
        // one plugged in between two attempts waits for the headset too
        m.open_failed();
        let b = sensor("SENSORBRAVO", "port-b-1", true);
        change(&mut m, 1, vec![a.clone(), b]).unwrap();
        assert_eq!(states(&m), ["waiting_headset", "waiting_headset"]);
        m.before_open();
        m.open_succeeded();
        assert_eq!(states(&m), ["opening", "opening"]);
        m.opened();
        assert!(m.pending().is_empty());
    }

    #[test]
    fn hotplug_a_failed_initial_listing_recovers_without_a_retry_loop() {
        let mut m = Monitor::new(Err(io::Error::other("listing interrupted")), &[]);
        let a = sensor("SENSORALPHA", "port-a-1", true);
        change(&mut m, 1, vec![a.clone()]).unwrap();
        assert!(m.waits_for_engine_start());
        m.before_open();
        m.opened();
        assert!(m.update(3 * DEBOUNCE_NS, Ok(vec![a]), &[]).is_none());
        assert!(!m.wants_open());
    }

    #[test]
    fn hotplug_new_sensor_is_attempted_once() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        let a = sensor("SENSORALPHA", "port-a-1", true);
        let c = change(&mut m, 1, vec![a.clone()]).unwrap();
        assert_eq!(c.plugged, std::slice::from_ref(&a));
        assert!(m.waits_for_engine_start());
        m.before_open();
        m.opened();
        for now in [2, 10, 100, 3600].map(|s| s * DEBOUNCE_NS) {
            assert!(m.update(now, Ok(vec![a.clone()]), &[]).is_none());
            assert!(
                !m.wants_open(),
                "a failed stream cannot cause another attempt"
            );
        }
    }

    #[test]
    fn hotplug_removal_retires_the_serial_without_reopening() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        let b = sensor("SENSORBETA", "port-b-1", true);
        let mut m = Monitor::new(Ok(vec![a.clone(), b.clone()]), &[]);
        let c = change(&mut m, 1, vec![b.clone()]).unwrap();
        assert_eq!(c.unplugged, std::slice::from_ref(&a.serial));
        assert!(!m.admitted(&a.serial));
        assert!(m.admitted(&b.serial));
        assert!(!m.waits_for_engine_start());
        // Reattaching the same serial keeps the old native object excluded until the next open.
        change(&mut m, 3 * DEBOUNCE_NS, vec![a.clone(), b]).unwrap();
        assert!(!m.admitted(&a.serial));
        assert!(m.waits_for_engine_start());
        m.before_open();
        assert!(m.admitted(&a.serial));
    }

    #[test]
    fn hotplug_access_can_arrive_seconds_after_boot_without_repeating_a_failed_start() {
        let mut a = sensor("SENSORALPHA", "port-a-1", false);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        m.before_open();
        m.opened();
        assert!(!m.wants_open());
        a.ready = true;
        let c = change(&mut m, 5 * DEBOUNCE_NS, vec![a.clone()]).unwrap();
        assert_eq!(c.ready, [a.serial.clone()]);
        assert!(c.plugged.is_empty());
        assert!(m.wants_open());
        m.before_open();
        m.opened();
        for (now, ready) in [(10, false), (20, true), (30, false), (40, true)] {
            a.ready = ready;
            change(&mut m, now * DEBOUNCE_NS, vec![a.clone()]);
            assert!(!m.wants_open());
        }
    }

    #[test]
    fn hotplug_enumeration_bounces_and_errors_do_not_look_like_unplugging() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        assert!(m.update(1, Ok(vec![]), &[]).is_none());
        assert!(m
            .update(
                DEBOUNCE_NS,
                Err(io::Error::other("listing interrupted")),
                &[]
            )
            .is_none());
        assert!(m.update(2 * DEBOUNCE_NS, Ok(vec![]), &[]).is_none());
        assert!(m
            .update(2 * DEBOUNCE_NS + DEBOUNCE_NS - 1, Ok(vec![]), &[])
            .is_none());
        assert!(m.update(3 * DEBOUNCE_NS, Ok(vec![a]), &[]).is_none());
        assert!(m.admitted("SENSORALPHA"));
    }

    #[test]
    fn hotplug_ignored_sensors_and_listing_order_do_not_trigger_changes() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        let b = sensor("SENSORBETA", "port-b-1", true);
        let ignore = [b.serial.clone()];
        let mut m = Monitor::new(Ok(vec![a.clone()]), &ignore);
        for now in 1..=4 {
            assert!(m
                .update(now * DEBOUNCE_NS, Ok(vec![b.clone(), a.clone()]), &ignore)
                .is_none());
            assert!(!m.waits_for_engine_start());
        }
    }

    #[test]
    fn hotplug_a_new_address_at_the_same_port_is_one_new_attempt() {
        let mut a = sensor("SENSORALPHA", "port-a-1", true);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        a.attachment = "port-a-2".into();
        let c = change(&mut m, 1, vec![a.clone()]).unwrap();
        assert_eq!(c.unplugged, [a.serial.clone()]);
        assert_eq!(c.plugged, [a.clone()]);
        assert!(m.waits_for_engine_start());
        m.before_open();
        m.opened();
        assert!(!m.wants_open());
    }

    #[test]
    fn hotplug_an_unplugged_pending_sensor_stops_waiting() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        change(&mut m, 1, vec![sensor("SENSORALPHA", "port-a-1", true)]).unwrap();
        assert!(m.waits_for_engine_start());
        change(&mut m, 2 * DEBOUNCE_NS, vec![]).unwrap();
        assert!(!m.waits_for_engine_start());
    }

    #[test]
    fn hotplug_an_engine_open_consumes_only_the_attachments_already_known() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        m.before_open();
        // A sensor that arrives while native opening is blocked still needs an attempt. Its
        // plug settles once the roster shows the engine did not open it.
        let a = sensor("SENSORALPHA", "port-a-1", true);
        assert!(change(&mut m, 1, vec![a.clone()]).is_none());
        assert!(m.pending().is_empty());
        m.reconcile_open(1 + DEBOUNCE_NS, &[], None);
        m.opened();
        assert_eq!(
            change(&mut m, 3 * DEBOUNCE_NS, vec![a.clone()])
                .unwrap()
                .plugged,
            [a]
        );
        assert!(m.wants_open());
    }
    #[test]
    fn hotplug_debounces_each_sensor_while_a_peer_flaps() {
        let a = sensor("SENSORALPHA", "a-1", true);
        let b = sensor("SENSORBRAVO", "b-1", true);
        let c = sensor("SENSORCHARLIE", "c-1", true);
        let mut m = Monitor::new(Ok(vec![a, b.clone()]), &[]);
        let mut peer = b;
        peer.ready = false;
        assert!(m
            .update(1, Ok(vec![peer.clone(), c.clone()]), &[])
            .is_none());
        peer.attachment = "b-2".into();
        peer.ready = true;
        let change = m
            .update(DEBOUNCE_NS + 1, Ok(vec![peer.clone(), c.clone()]), &[])
            .unwrap();
        assert_eq!(change.unplugged, ["SENSORALPHA"]);
        assert_eq!(change.plugged, std::slice::from_ref(&c));
        assert!(!m.admitted("SENSORALPHA"));
        assert!(m.waits_for_engine_start());
        m.before_open();
        m.opened();
        for i in 2..8 {
            peer.attachment = format!("b-{}", i + 1);
            peer.ready = i % 2 == 0;
            assert!(m
                .update(i * DEBOUNCE_NS + 1, Ok(vec![peer.clone(), c.clone()]), &[])
                .is_none());
            assert!(!m.wants_open());
        }
    }

    #[test]
    fn hotplug_readiness_flaps_do_not_delay_membership() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        let mut a = sensor("SENSORALPHA", "a-1", false);
        assert!(m.update(1, Ok(vec![a.clone()]), &[]).is_none());
        a.ready = true;
        let change = m.update(DEBOUNCE_NS + 1, Ok(vec![a.clone()]), &[]).unwrap();
        assert_eq!(change.plugged.len(), 1);
        assert!(!change.plugged[0].ready);
        assert!(!m.waits_for_engine_start());
        let change = m.update(2 * DEBOUNCE_NS + 1, Ok(vec![a]), &[]).unwrap();
        assert_eq!(change.ready, ["SENSORALPHA"]);
        assert!(m.waits_for_engine_start());
    }

    #[test]
    fn hotplug_native_gone_allows_one_recovery_without_a_seen_absence() {
        let a = sensor("SENSORALPHA", "same-instance", true);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        assert_eq!(
            m.native_gone(&a.serial).unwrap().unplugged,
            std::slice::from_ref(&a.serial)
        );
        assert!(!m.admitted(&a.serial));
        assert!(m.native_gone(&a.serial).is_none());
        assert!(!m.waits_for_engine_start());
        assert!(m.update(1, Ok(vec![a.clone()]), &[]).is_none());
        assert!(!m.waits_for_engine_start());
        assert_eq!(
            m.update(DEBOUNCE_NS + 1, Ok(vec![a.clone()]), &[])
                .unwrap()
                .plugged,
            std::slice::from_ref(&a)
        );
        assert!(m.waits_for_engine_start());
        m.before_open();
        m.opened();
        // A repeated NO_DEVICE on that OS generation asks for a replug.
        m.native_gone(&a.serial).unwrap();
        for i in 2..5 {
            assert!(m
                .update(i * DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
                .is_none());
            assert!(!m.wants_open());
        }
        assert_eq!(m.pending()[0].state, "waiting_replug");
        let b = sensor("SENSORALPHA", "new-arrival", true);
        let change = change(&mut m, 6 * DEBOUNCE_NS, vec![b.clone()]).unwrap();
        assert!(
            change.unplugged.is_empty(),
            "native removal was already reported"
        );
        assert_eq!(change.plugged, [b]);
        assert!(m.wants_open());
    }

    #[test]
    fn hotplug_unknown_generation_waits_for_metadata_without_a_false_detach() {
        let a = sensor("SENSORALPHA", "arrival-1", true);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        let unknown = sensor("SENSORALPHA", "", false);
        assert!(change(&mut m, 1, vec![unknown]).is_none());
        assert!(m.admitted(&a.serial));
        assert!(!m.present[&a.serial].ready);
        assert!(change(&mut m, 2 * DEBOUNCE_NS, vec![a.clone()]).is_none());
        assert!(m.present[&a.serial].ready);
        assert!(m.admitted(&a.serial));
        assert!(!m.waits_for_engine_start());
    }
    #[test]
    fn hotplug_late_native_gone_cannot_retire_the_replacement_attachment() {
        let a = sensor("SENSORALPHA", "a-1", true);
        let mut m = Monitor::new(Ok(vec![a]), &[]);
        let returned = sensor("SENSORALPHA", "a-2", true);
        let changed = change(&mut m, 1, vec![returned]).unwrap();
        assert_eq!(changed.unplugged, ["SENSORALPHA"]);
        assert!(
            m.native_gone("SENSORALPHA").is_none(),
            "the old native object is already retired"
        );
        assert!(m.waits_for_engine_start());
    }
    #[test]
    fn hotplug_an_unadmitted_sensor_disappearing_breaks_its_debounce() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        let a = sensor("SENSORALPHA", "a-1", true);
        assert!(m.update(1, Ok(vec![a.clone()]), &[]).is_none());
        assert!(m.update(DEBOUNCE_NS + 1, Ok(vec![]), &[]).is_none());
        assert!(
            m.update(3 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
                .is_none(),
            "an old candidate must not count as the first of two later observations"
        );
        assert!(!m.waits_for_engine_start());
        let change = m.update(4 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]).unwrap();
        assert_eq!(change.plugged, [a]);
        assert!(m.waits_for_engine_start());
    }

    #[test]
    fn hotplug_lost_usb_readiness_blocks_an_unconsumed_attempt_immediately() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        let mut a = sensor("SENSORALPHA", "a-1", true);
        change(&mut m, 1, vec![a.clone()]).unwrap();
        assert!(m.waits_for_engine_start());
        a.ready = false;
        assert!(m
            .update(2 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
            .is_none());
        assert!(
            !m.waits_for_engine_start(),
            "a failed status/interface/access query must not consume the attempt"
        );
        a.ready = true;
        assert!(m
            .update(3 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
            .is_none());
        assert!(!m.waits_for_engine_start());
        assert_eq!(
            m.update(4 * DEBOUNCE_NS, Ok(vec![a]), &[]).unwrap().ready,
            ["SENSORALPHA"]
        );
        assert!(m.waits_for_engine_start());
        m.before_open();
        m.opened();
        assert!(!m.wants_open());
    }
    #[test]
    fn hotplug_a_changing_attachment_blocks_old_readiness_immediately() {
        for ready in [false, true] {
            let mut m = Monitor::new(Ok(vec![]), &[]);
            let a = sensor("SENSORALPHA", "a-1", true);
            change(&mut m, 1, vec![a]).unwrap();
            let b = sensor("SENSORALPHA", "a-2", ready);
            assert!(m
                .update(2 * DEBOUNCE_NS, Ok(vec![b.clone()]), &[])
                .is_none());
            assert!(!m.present["SENSORALPHA"].ready);
            assert!(!m.waits_for_engine_start());
            m.update(3 * DEBOUNCE_NS, Ok(vec![b]), &[]);
            assert_eq!(m.waits_for_engine_start(), ready);
        }
    }

    #[test]
    fn hotplug_missing_sensor_stops_waiting_during_membership_debounce() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        let a = sensor("SENSORALPHA", "a-1", true);
        change(&mut m, 1, vec![a]).unwrap();
        assert!(m.waits_for_engine_start());
        assert!(m.update(2 * DEBOUNCE_NS, Ok(vec![]), &[]).is_none());
        assert!(!m.present["SENSORALPHA"].ready);
        assert!(!m.waits_for_engine_start());
    }

    #[test]
    fn hotplug_absence_resets_readiness_even_without_debounced_removal() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        let mut a = sensor("SENSORALPHA", "a-1", false);
        change(&mut m, 1, vec![a.clone()]).unwrap();
        a.ready = true;
        m.update(2 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]);
        m.update(2 * DEBOUNCE_NS + 1, Ok(vec![]), &[]);
        m.update(3 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]);
        assert!(!m.waits_for_engine_start());
        m.update(4 * DEBOUNCE_NS, Ok(vec![a]), &[]);
        assert!(m.waits_for_engine_start());
    }
    fn stream_start(
        serial: &str,
        attachment: &str,
        failed: bool,
    ) -> crate::engine::SensorStreamStart {
        let mut raw = crate::ffi::ohmd_cv1touch_sensor_stream::default();
        for (out, byte) in raw.serial.iter_mut().zip(serial.bytes()) {
            *out = byte as std::ffi::c_char;
        }
        if failed {
            raw.error_kind = crate::ffi::OHMD_CV1TOUCH_STREAM_ERROR_LIKELY_USB_BANDWIDTH;
            raw.error_code = -11;
        } else {
            raw.alt_setting = 2;
        }
        let mut start: crate::engine::SensorStreamStart = raw.into();
        start.attachment = Some(attachment.to_owned());
        start
    }

    #[test]
    fn a_failed_start_cannot_consume_a_replacement_before_reconciliation() {
        use crate::ffi;
        for (kind, old, new, replacement) in [
            (
                ffi::OHMD_CV1TOUCH_ATTACHMENT_SYSFS,
                [1 | (7 << 8), 0x1234, 0],
                [1 | (8 << 8), 0x1235, 0],
                "sysfs:1:8@0000000000001235",
            ),
            (
                ffi::OHMD_CV1TOUCH_ATTACHMENT_WINDOWS,
                [0x9abcdef0, 0x12345678, 0],
                [0x9abcdef1, 0x12345678, 0],
                r"USB\VID_2833&PID_0211\SENSORALPHA@123456789abcdef1",
            ),
        ] {
            let mut m = Monitor::new(Ok(vec![]), &[]);
            m.before_open();
            // Native tried A and freed its failed non-NO_DEVICE object. The watcher
            // never admitted A; B replaced it before the completion scan.
            let mut raw = ffi::ohmd_cv1touch_sensor_stream::default();
            for (out, byte) in raw.serial.iter_mut().zip(b"SENSORALPHA") {
                *out = *byte as std::ffi::c_char;
            }
            raw.error_kind = ffi::OHMD_CV1TOUCH_STREAM_ERROR_OTHER;
            raw.error_code = -1;
            raw.attachment = ffi::ohmd_cv1touch_sensor_attachment { kind, data: old };
            let b = sensor("SENSORALPHA", replacement, true);
            m.reconcile_open(1, &[raw.into()], Some(std::slice::from_ref(&b)));
            change(&mut m, 1, vec![b.clone()]);
            m.opened();
            assert_eq!(m.pending().len(), 1, "B lost its pending reason");
            assert!(m.wants_open(), "A's failed start consumed B's attempt");
            // The next engine start attempts B; even another ordinary start failure must
            // consume this matching attachment once, without a retry loop.
            m.before_open();
            raw.attachment.data = new;
            m.reconcile_open(
                1 + DEBOUNCE_NS,
                &[raw.into()],
                Some(std::slice::from_ref(&b)),
            );
            m.opened();
            assert!(m.pending().is_empty());
        }
    }

    #[test]
    fn unknown_discovery_identity_leaves_the_attachment_pending() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        m.before_open();
        let a = sensor("SENSORALPHA", "port-a-1", true);
        let mut unknown = stream_start(&a.serial, &a.attachment, true);
        unknown.attachment = None;
        m.reconcile_open(1, &[unknown], Some(std::slice::from_ref(&a)));
        change(&mut m, 1, vec![a]);
        m.opened();
        assert_eq!(m.pending().len(), 1);
        assert!(m.wants_open());
    }

    #[test]
    fn hotplug_doctor_reconciles_discovery_including_failed_starts() {
        for failed in [false, true] {
            let mut m = Monitor::new(Ok(vec![]), &[]);
            m.before_open();
            // This attachment was absent from before_open but native discovery attempted it.
            let a = sensor("SENSORALPHA", "port-a-1", true);
            let found = m.reconcile_open(
                1,
                &[stream_start(&a.serial, "port-a-1", failed)],
                Some(std::slice::from_ref(&a)),
            );
            assert_eq!(found.plugged, std::slice::from_ref(&a));
            assert!(
                change(&mut m, 1, vec![a]).is_none(),
                "discovery was already reported"
            );
            m.opened();
            assert!(
                !m.wants_open(),
                "a native attempt, including failure, is not attempted again"
            );
            assert!(m.pending().is_empty());
        }
    }

    #[test]
    fn hotplug_doctor_leaves_arrivals_after_discovery_pending() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        m.before_open();
        // The same completion inventory cannot consume an attachment absent from native starts.
        let a = sensor("SENSORALPHA", "port-a-1", true);
        m.reconcile_open(1, &[], Some(std::slice::from_ref(&a)));
        change(&mut m, 1, vec![a]).unwrap();
        m.opened();
        assert!(m.wants_open());
    }

    #[test]
    fn hotplug_doctor_does_not_consume_a_replacement_from_an_old_start() {
        for failed in [true, false] {
            let mut a = sensor("SENSORALPHA", "port-a-1", true);
            let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
            m.before_open();
            a.attachment = "port-a-2".into();
            assert!(change(&mut m, 1, vec![a.clone()]).is_none());
            m.reconcile_open(
                1 + DEBOUNCE_NS,
                &[stream_start(&a.serial, "port-a-1", failed)],
                Some(std::slice::from_ref(&a)),
            );
            m.opened();
            assert!(
                !m.attempted.contains_key(&a.serial),
                "an old generation's start cannot spend the replacement's attempt"
            );
            if failed {
                change(&mut m, 3 * DEBOUNCE_NS, vec![a.clone()]).unwrap();
            } else {
                // A stream that started is in use, on the replacement or on the old device,
                // until its own NO_DEVICE says which.
                assert!(m.pending().is_empty());
                m.native_gone(&a.serial).unwrap();
                m.update(3 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]);
                m.update(4 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]).unwrap();
            }
            assert!(m.wants_open());
        }
    }

    #[test]
    fn hotplug_doctor_keeps_unsettled_discovery_and_readiness_debounced() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        let mut a = sensor("SENSORALPHA", "port-a-1", true);
        m.update(1, Ok(vec![a.clone()]), &[]);
        m.before_open();
        m.reconcile_open(
            1,
            &[stream_start(&a.serial, "port-a-1", false)],
            Some(std::slice::from_ref(&a)),
        );
        a.ready = false;
        m.update(DEBOUNCE_NS + 1, Ok(vec![a.clone()]), &[]);
        assert!(!m.present[&a.serial].ready);
        m.opened();
        a.ready = true;
        change(&mut m, 3 * DEBOUNCE_NS, vec![a]);
        assert!(!m.wants_open());
    }

    #[test]
    fn hotplug_doctor_serial_identity_matches_ignores_starts_and_removals() {
        let a = sensor("SensorAlpha", "port-a-1", true);
        let ignored = Monitor::new(Ok(vec![a.clone()]), &["sensoralpha".into()]);
        assert!(
            ignored.present.is_empty(),
            "ignore matching must follow native identity"
        );
        let mut m = Monitor::new(Ok(vec![]), &[]);
        m.before_open();
        let found = m.reconcile_open(
            1,
            &[stream_start("sensoralpha", "port-a-1", false)],
            Some(std::slice::from_ref(&a)),
        );
        assert_eq!(found.plugged, std::slice::from_ref(&a));
        assert!(change(&mut m, 1, vec![a]).is_none());
        m.opened();
        assert!(!m.wants_open());
        change(&mut m, 3 * DEBOUNCE_NS, vec![]).unwrap();
        assert!(!m.admitted("sensoralpha"));
        assert!(!m.admitted("SENSORALPHA"));
    }

    #[test]
    fn hotplug_doctor_native_gone_matches_os_spelling() {
        let a = sensor("SensorAlpha", "port-a-1", true);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        let gone = m
            .native_gone("sensoralpha")
            .expect("native and OS serials identify one sensor");
        assert_eq!(gone.unplugged, [a.serial]);
        assert!(!m.admitted("SENSORALPHA"));
    }
    #[test]
    fn hotplug_doctor_only_a_matching_arrival_property_can_consume_an_actual_start() {
        for property_at_completion in [false, true] {
            for already_listed in [false, true] {
                let mut a = sensor("SENSORALPHA", "", false);
                let mut m = Monitor::new(
                    Ok(if already_listed {
                        vec![a.clone()]
                    } else {
                        vec![]
                    }),
                    &[],
                );
                m.before_open();
                // Native discovery got as far as a real stream start even though the Windows
                // arrival property was unavailable. Its failed-start record still spends one attempt.
                let mut completion = a.clone();
                if property_at_completion {
                    completion.attachment = "instance-a@arrival-1".into();
                    completion.ready = true;
                }
                m.reconcile_open(
                    1,
                    &[stream_start(&a.serial, "instance-a@arrival-1", true)],
                    Some(std::slice::from_ref(&completion)),
                );
                m.opened();
                if !already_listed {
                    change(&mut m, 1, vec![a.clone()]);
                }
                a.attachment = "instance-a@arrival-1".into();
                a.ready = true;
                change(&mut m, 3 * DEBOUNCE_NS, vec![a]);
                assert!(
                    m.wants_open() != property_at_completion,
                    "an ambiguous completion must leave the attachment eligible for its own attempt"
                );
            }
        }
    }

    #[test]
    fn hotplug_doctor_a_start_before_the_first_usb_poll_can_be_removed() {
        for failed in [false, true] {
            let mut m = Monitor::new(Ok(vec![]), &[]);
            m.before_open();
            let a = sensor("SENSORALPHA", "port-a-1", true);
            m.reconcile_open(
                1,
                &[stream_start(&a.serial, "port-a-1", failed)],
                Some(std::slice::from_ref(&a)),
            );
            m.opened();
            let removed =
                change(&mut m, 1, vec![]).expect("the native attempt witnessed this attachment");
            assert_eq!(removed.unplugged, std::slice::from_ref(&a.serial));
            assert!(!m.admitted(&a.serial));
        }
    }

    #[test]
    fn hotplug_doctor_a_start_before_the_first_usb_poll_can_report_no_device() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        m.before_open();
        let a = sensor("SENSORALPHA", "port-a-1", true);
        m.reconcile_open(
            1,
            &[stream_start(&a.serial, "port-a-1", false)],
            Some(std::slice::from_ref(&a)),
        );
        assert!(m.native_gone(&a.serial).is_some());
        assert!(!m.admitted(&a.serial));
    }

    #[test]
    fn hotplug_doctor_a_replacement_before_the_first_usb_poll_retires_the_old_handle() {
        let mut m = Monitor::new(Ok(vec![]), &[]);
        m.before_open();
        let mut a = sensor("SENSORALPHA", "port-a-1", true);
        m.reconcile_open(
            1,
            &[stream_start(&a.serial, "port-a-1", false)],
            Some(std::slice::from_ref(&a)),
        );
        a.attachment = "port-a-2".into();
        let changed = change(&mut m, 1, vec![a.clone()]).unwrap();
        assert_eq!(changed.unplugged, [a.serial.clone()]);
        assert_eq!(changed.plugged, [a.clone()]);
        assert!(!m.admitted(&a.serial));
        m.opened();
        assert!(m.wants_open());
    }

    /// The post-open listing raced an unplug and still shows the old node, and the unplug cannot
    /// settle while the engine discovers. The stream is in use until the first view settled
    /// after the roster shows its serial gone; a reattached sensor then waits for its own attempt.
    #[test]
    fn hotplug_doctor_a_late_listing_keeps_a_departed_sensor_only_until_its_removal_settles() {
        let a1 = sensor("SENSORALPHA", "port-a-1", true);
        let mut m = Monitor::new(Ok(vec![a1.clone()]), &[]);
        m.before_open();
        assert!(
            change(&mut m, 1, vec![]).is_none(),
            "nothing settles while the engine discovers"
        );
        m.reconcile_open(
            1 + DEBOUNCE_NS,
            &[stream_start(&a1.serial, &a1.attachment, false)],
            Some(std::slice::from_ref(&a1)),
        );
        m.opened();
        assert!(m.in_use(&a1.serial));
        // The first scan applied after the roster may have been read before the start.
        assert!(m.update(3 * DEBOUNCE_NS, Ok(vec![]), &[]).is_none());
        assert!(m.admitted(&a1.serial));
        let removed = m.update(4 * DEBOUNCE_NS, Ok(vec![]), &[]).unwrap();
        assert_eq!(removed.unplugged, std::slice::from_ref(&a1.serial));
        assert!(!m.admitted(&a1.serial));
        let a2 = sensor("SENSORALPHA", "port-a-2", true);
        change(&mut m, 5 * DEBOUNCE_NS, vec![a2.clone()]).unwrap();
        assert!(!m.admitted(&a2.serial));
        assert!(
            m.wants_open(),
            "a late old record spent the reattached sensor's attempt"
        );
    }
    #[test]
    fn hotplug_doctor_keeps_known_generation_when_an_arrival_query_fails() {
        let mut a = sensor("SENSORALPHA", "instance-a@arrival-1", false);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        m.before_open();
        let mut incomplete = a.clone();
        incomplete.attachment.clear();
        m.reconcile_open(
            1,
            &[stream_start(&a.serial, "instance-a@arrival-1", true)],
            Some(std::slice::from_ref(&incomplete)),
        );
        m.opened();
        a.ready = true;
        change(&mut m, 1, vec![a]);
        assert!(!m.wants_open());
    }

    #[test]
    fn hotplug_doctor_a_completion_listing_cannot_spend_a_replacements_attempt() {
        let mut a = sensor("SENSORALPHA", "port-a-1", true);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        m.before_open();
        // Only the completion scan has seen a new attachment. The watcher has not yet
        // settled its change, so an old native start cannot spend its budget: the stream is
        // in use, on whichever device it opened, and the replacement's attempt is untouched.
        a.attachment = "port-a-2".into();
        m.reconcile_open(
            1,
            &[stream_start(&a.serial, "port-a-1", false)],
            Some(std::slice::from_ref(&a)),
        );
        assert!(m.in_use(&a.serial));
        assert_ne!(m.attempted.get(&a.serial), Some(&a.attachment));
        m.opened();
        assert!(!m.wants_open());
    }
    #[test]
    fn hotplug_doctor_readiness_after_reopen_uses_only_that_engines_attempts() {
        for actually_started in [false, true] {
            let mut a = sensor("SENSORALPHA", "port-a-1", true);
            let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
            m.before_open();
            m.opened();
            a.ready = false;
            change(&mut m, 1, vec![a.clone()]);
            // Another engine reopen happened while this sensor's access was unavailable.
            m.before_open();
            let starts = if actually_started {
                vec![stream_start(&a.serial, "port-a-1", true)]
            } else {
                vec![]
            };
            m.reconcile_open(1 + DEBOUNCE_NS, &starts, Some(std::slice::from_ref(&a)));
            m.opened();
            a.ready = true;
            change(&mut m, 3 * DEBOUNCE_NS, vec![a]);
            assert_eq!(
                m.wants_open(),
                !actually_started,
                "a previous engine's attempt cannot hide a sensor omitted by the current engine"
            );
        }
    }

    fn details(m: &Monitor) -> Vec<(&'static str, String)> {
        m.pending()
            .into_iter()
            .map(|p| (p.state, p.detail))
            .collect()
    }

    /// A USB change that is still settling is being checked, whatever caused it: a replugged or
    /// unplugged sensor, a NO_DEVICE awaiting two confirming scans, or access just granted. It
    /// is not given the USB access reason, which stays for access the OS does not grant.
    #[test]
    fn hotplug_a_usb_change_still_settling_is_checked_not_blamed_on_access() {
        let checking = [("checking_usb", CHECKING_DETAIL.to_string())];
        let access = [("waiting_usb", usb_access_detail().to_string())];
        // A NO_DEVICE: two scans of a ready attachment confirm it is back.
        let a = sensor("SENSORALPHA", "port-a-1", true);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        m.native_gone(&a.serial).unwrap();
        assert_eq!(details(&m), checking);
        assert!(m.update(1, Ok(vec![a.clone()]), &[]).is_none());
        assert_eq!(details(&m), checking);
        // ... but only while the OS still grants access to it
        let mut unready = a.clone();
        unready.ready = false;
        m.update(2, Ok(vec![unready.clone()]), &[]);
        assert_eq!(details(&m), access);
        m.update(3, Ok(vec![a.clone()]), &[]);
        assert_eq!(details(&m), checking);
        m.update(3 + DEBOUNCE_NS, Ok(vec![a.clone()]), &[]).unwrap();
        assert!(m.waits_for_engine_start());
        // The same sensor unplugged, or replugged, before the next engine start.
        m.update(5 * DEBOUNCE_NS, Ok(vec![]), &[]);
        assert_eq!(details(&m), checking);
        let mut b = a.clone();
        b.attachment = "port-a-2".into();
        m.update(5 * DEBOUNCE_NS + 1, Ok(vec![b.clone()]), &[]);
        assert_eq!(details(&m), checking);
        m.update(6 * DEBOUNCE_NS + 1, Ok(vec![b.clone()]), &[])
            .unwrap();
        assert!(m.waits_for_engine_start());
        // Access the OS grants late settles for a second, then the sensor waits for the engine.
        let mut m = Monitor::new(Ok(vec![unready.clone()]), &[]);
        m.before_open();
        m.opened();
        assert_eq!(details(&m), access);
        m.update(1, Ok(vec![a.clone()]), &[]);
        assert_eq!(details(&m), checking);
        m.update(1 + DEBOUNCE_NS, Ok(vec![a.clone()]), &[]).unwrap();
        assert!(m.wants_open());
        m.update(2 + DEBOUNCE_NS, Ok(vec![unready]), &[]);
        assert_eq!(details(&m), access);
    }

    /// A replacement seen around an engine open is what that engine started. It does not settle
    /// while the engine discovers, so it can neither retire the new engine's sensor nor say the
    /// sensor waits; the listing read after the open shows where the sensor is, and the settle
    /// that follows retires nothing.
    #[test]
    fn hotplug_a_replacement_settling_during_an_engine_open_keeps_the_new_engines_sensor() {
        let a1 = sensor("SENSORALPHA", "port-a-1", true);
        let a2 = sensor("SENSORALPHA", "port-a-2", true);
        let new_start = || [stream_start(&a1.serial, &a2.attachment, false)];
        for seen_before_open in [false, true] {
            let mut m = Monitor::new(Ok(vec![a1.clone()]), &[]);
            if seen_before_open {
                assert!(m.update(1, Ok(vec![a2.clone()]), &[]).is_none());
                m.before_open();
            } else {
                m.before_open();
                assert!(m.update(1, Ok(vec![a2.clone()]), &[]).is_none());
            }
            assert!(
                m.update(1 + DEBOUNCE_NS, Ok(vec![a2.clone()]), &[])
                    .is_none(),
                "nothing settles while the engine discovers"
            );
            assert_eq!(states(&m), ["opening"]);
            let found = m.reconcile_open(
                1 + DEBOUNCE_NS,
                &new_start(),
                Some(std::slice::from_ref(&a2)),
            );
            assert_eq!(found.unplugged, std::slice::from_ref(&a1.serial));
            assert_eq!(found.plugged, std::slice::from_ref(&a2));
            m.opened();
            for tick in [2, 3] {
                assert!(m
                    .update(tick * DEBOUNCE_NS, Ok(vec![a2.clone()]), &[])
                    .is_none());
            }
            assert!(m.in_use(&a2.serial));
            assert!(m.detached().is_empty());
            assert!(m.pending().is_empty());
        }
        // The watcher saw nothing before the open; the roster replaces its stale attachment.
        let mut m = Monitor::new(Ok(vec![a1.clone()]), &[]);
        m.before_open();
        assert_eq!(
            m.reconcile_open(1, &new_start(), Some(std::slice::from_ref(&a2)))
                .plugged,
            std::slice::from_ref(&a2)
        );
        assert!(m.update(1, Ok(vec![a2.clone()]), &[]).is_none());
        assert!(m
            .update(1 + DEBOUNCE_NS, Ok(vec![a2.clone()]), &[])
            .is_none());
        assert!(m.admitted(&a2.serial));
        m.opened();
        assert!(m.pending().is_empty());
        // A completion listing without the Windows arrival cannot confirm the start, but the
        // settle onto the attachment the engine started still retires nothing.
        let mut m = Monitor::new(Ok(vec![a1.clone()]), &[]);
        m.before_open();
        let mut unknown = a2.clone();
        unknown.attachment.clear();
        m.reconcile_open(1, &new_start(), Some(std::slice::from_ref(&unknown)));
        m.opened();
        assert!(m.update(1, Ok(vec![a2.clone()]), &[]).is_none());
        assert!(m
            .update(1 + DEBOUNCE_NS, Ok(vec![a2.clone()]), &[])
            .is_none());
        assert!(m.admitted(&a2.serial));
        assert!(m.pending().is_empty());
        // An engine that started the old attachment still has it retired.
        let mut m = Monitor::new(Ok(vec![a1.clone()]), &[]);
        m.before_open();
        m.reconcile_open(
            1,
            &[stream_start(&a1.serial, &a1.attachment, false)],
            Some(std::slice::from_ref(&a1)),
        );
        m.opened();
        let replaced = change(&mut m, 1, vec![a2.clone()]).unwrap();
        assert_eq!(replaced.unplugged, std::slice::from_ref(&a1.serial));
        assert!(!m.admitted(&a1.serial));
        assert!(m.wants_open());
    }

    /// An unplug seen during an engine open settles after its roster is known, retiring the
    /// sensor whether or not the engine started it; a listing gap the watcher sees around the
    /// open leaves the engine's start in use.
    #[test]
    fn hotplug_an_unplug_during_an_engine_open_settles_on_its_own() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        for started in [false, true] {
            let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
            m.update(1, Ok(vec![]), &[]);
            m.before_open();
            let starts = if started {
                vec![stream_start(&a.serial, &a.attachment, false)]
            } else {
                vec![]
            };
            // the post-open listing still had it
            m.reconcile_open(1, &starts, Some(std::slice::from_ref(&a)));
            m.opened();
            // The first scan applied after the roster may have been read before the starts.
            assert!(m.update(1 + DEBOUNCE_NS, Ok(vec![]), &[]).is_none());
            let removed = m.update(2 * DEBOUNCE_NS, Ok(vec![]), &[]).unwrap();
            assert_eq!(removed.unplugged, std::slice::from_ref(&a.serial));
            assert!(!m.admitted(&a.serial));
            assert!(m.pending().is_empty());
        }
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        m.update(1, Ok(vec![]), &[]);
        m.before_open();
        m.reconcile_open(
            1,
            &[stream_start(&a.serial, &a.attachment, false)],
            Some(std::slice::from_ref(&a)),
        );
        m.opened();
        assert!(m.update(2, Ok(vec![a.clone()]), &[]).is_none());
        assert!(m
            .update(2 + DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
            .is_none());
        assert!(m.admitted(&a.serial));
        assert!(m.pending().is_empty());
    }

    /// A new sensor plugged in during an open is used if native discovery started it, and
    /// waits for the next engine start if discovery had already passed.
    #[test]
    fn hotplug_a_sensor_plugged_in_during_an_engine_open_waits_only_if_it_was_not_started() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        for (started, watcher_first) in [(false, false), (true, false), (true, true)] {
            let mut m = Monitor::new(Ok(vec![]), &[]);
            m.before_open();
            if watcher_first {
                assert!(change(&mut m, 1, vec![a.clone()]).is_none());
            }
            let starts = if started {
                vec![stream_start(&a.serial, &a.attachment, false)]
            } else {
                vec![]
            };
            m.reconcile_open(1 + DEBOUNCE_NS, &starts, Some(std::slice::from_ref(&a)));
            m.opened();
            change(&mut m, 3 * DEBOUNCE_NS, vec![a.clone()]);
            assert!(m.admitted(&a.serial));
            assert_eq!(m.wants_open(), !started);
        }
    }

    /// A stream that started with no identified attachment (no Windows arrival, a failed Linux
    /// pin) is in use: it is neither consumed nor reported as waiting for an engine start. Its
    /// removal or NO_DEVICE still retires it.
    #[test]
    fn hotplug_a_started_sensor_without_an_identified_attachment_is_in_use() {
        let a = sensor("SENSORALPHA", "instance-a@arrival-1", true);
        let unidentified = |failed| {
            let mut start = stream_start(&a.serial, &a.attachment, failed);
            start.attachment = None;
            start
        };
        let opened = |failed| {
            let mut m = Monitor::new(Ok(vec![]), &[]);
            m.before_open();
            m.reconcile_open(1, &[unidentified(failed)], Some(std::slice::from_ref(&a)));
            m.opened();
            m
        };
        let mut m = opened(false);
        assert!(m.in_use(&a.serial));
        assert!(
            change(&mut m, 1, vec![a.clone()]).is_none(),
            "an attachment in use is not a sensor waiting to be used"
        );
        assert!(!m.attempted.contains_key(&a.serial));
        assert!(m.pending().is_empty());
        let mut unready = a.clone();
        unready.ready = false;
        m.update(3 * DEBOUNCE_NS, Ok(vec![unready]), &[]);
        assert!(m
            .update(4 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
            .is_none());
        assert!(m
            .update(5 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
            .is_none());
        assert!(m.pending().is_empty());
        // The next engine start records its attachment like any other.
        m.before_open();
        assert!(!m.in_use(&a.serial));
        assert_eq!(m.attempted[&a.serial], a.attachment);
        // A failed start is no stream in use: its attachment still gets its own attempt.
        let mut m = opened(true);
        assert!(!m.in_use(&a.serial));
        change(&mut m, 1, vec![a.clone()]).unwrap();
        assert!(m.wants_open());
        // Unplugging it, or a NO_DEVICE from its stream, retires it.
        let mut m = opened(false);
        change(&mut m, 1, vec![a.clone()]);
        let removed = change(&mut m, 3 * DEBOUNCE_NS, vec![]).unwrap();
        assert_eq!(removed.unplugged, std::slice::from_ref(&a.serial));
        assert!(!m.admitted(&a.serial));
        assert!(!m.in_use(&a.serial));
        let mut m = opened(false);
        change(&mut m, 1, vec![a.clone()]);
        m.native_gone(&a.serial).unwrap();
        assert!(!m.in_use(&a.serial));
        m.update(3 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]);
        m.update(4 * DEBOUNCE_NS, Ok(vec![a.clone()]), &[]).unwrap();
        assert!(m.wants_open());
        // A NO_DEVICE retires it before the watcher lists it, even when no listing showed it.
        for listed in [true, false] {
            let mut m = Monitor::new(Ok(vec![]), &[]);
            m.before_open();
            let inventory = listed.then(|| vec![a.clone()]);
            m.reconcile_open(1, &[unidentified(false)], inventory.as_deref());
            m.opened();
            assert_eq!(
                m.native_gone(&a.serial).unwrap().unplugged,
                std::slice::from_ref(&a.serial)
            );
            assert!(!m.admitted(&a.serial));
            assert!(!m.in_use(&a.serial));
            change(&mut m, 1, vec![a.clone()]);
            change(&mut m, 3 * DEBOUNCE_NS, vec![a.clone()]);
            assert!(m.wants_open());
        }
    }

    /// A replug with a gap: the watcher saw the old attachment gone before the open, and native
    /// discovery started the new one before the watcher admitted it. The removal does not settle
    /// while the engine discovers, so it cannot retire the new engine's sensor or say it waits.
    #[test]
    fn hotplug_a_gap_replug_during_an_engine_open_keeps_the_new_engines_sensor() {
        let a1 = sensor("SENSORALPHA", "port-a-1", true);
        let a2 = sensor("SENSORALPHA", "port-b-2", true);
        let mut m = Monitor::new(Ok(vec![a1.clone()]), &[]);
        assert!(m.update(1, Ok(vec![]), &[]).is_none());
        m.before_open();
        assert!(
            m.update(1 + DEBOUNCE_NS, Ok(vec![]), &[]).is_none(),
            "the removal settled while the engine discovered"
        );
        assert!(m.detached().is_empty());
        let found = m.reconcile_open(
            1 + DEBOUNCE_NS,
            &[stream_start(&a2.serial, &a2.attachment, false)],
            Some(std::slice::from_ref(&a2)),
        );
        assert_eq!(found.unplugged, std::slice::from_ref(&a1.serial));
        assert_eq!(found.plugged, std::slice::from_ref(&a2));
        assert!(m.in_use(&a2.serial));
        m.opened();
        for tick in 2..5 {
            assert!(m
                .update(tick * DEBOUNCE_NS, Ok(vec![a2.clone()]), &[])
                .is_none());
        }
        assert!(m.detached().is_empty());
        assert!(m.pending().is_empty());
    }

    /// A stream the engine started is in use even when the listing read after the open failed or
    /// lacked the Windows arrival: USB access, or the arrival, that the watcher sees afterwards
    /// does not make it wait.
    #[test]
    fn hotplug_access_or_an_arrival_seen_after_an_unconfirmed_start_does_not_make_it_wait() {
        let a = sensor("SENSORALPHA", "instance-a@arrival-1", true);
        for arrival in [true, false] {
            let mut known = a.clone();
            known.ready = false;
            if !arrival {
                known.attachment.clear();
            }
            for inventory in [None, Some(vec![]), Some(vec![known.clone()])] {
                let mut m = Monitor::new(Ok(vec![known.clone()]), &[]);
                m.before_open();
                let found = m.reconcile_open(
                    1,
                    &[stream_start(&a.serial, &a.attachment, false)],
                    inventory.as_deref(),
                );
                assert_eq!(found, Change::default());
                m.opened();
                for tick in 1..4 {
                    assert!(m
                        .update(tick * DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
                        .is_none());
                }
                assert!(m.in_use(&a.serial));
                assert!(m.pending().is_empty());
            }
        }
    }

    /// A stream whose start names no attachment (a failed Linux pin) or an older one (a Windows
    /// arrival snapshot taken before a replug) is in use on whichever device it opened. A
    /// replacement settling afterwards does not retire it: no listing read after the start
    /// showed another device of the serial leaving, and the serial is still there.
    #[test]
    fn hotplug_a_missing_or_stale_start_identity_survives_the_replacements_settle() {
        let a1 = sensor("SENSORALPHA", "port-a-1", true);
        let a2 = sensor("SENSORALPHA", "port-a-2", true);
        for stamp in [None, Some(a1.attachment.clone())] {
            for inventory in [None, Some(vec![]), Some(vec![a2.clone()])] {
                let mut m = Monitor::new(Ok(vec![a1.clone()]), &[]);
                m.before_open();
                let mut start = stream_start(&a1.serial, "", false);
                start.attachment.clone_from(&stamp);
                m.reconcile_open(1, &[start], inventory.as_deref());
                m.opened();
                let changes: Vec<_> = (1..5)
                    .filter_map(|tick| m.update(tick * DEBOUNCE_NS, Ok(vec![a2.clone()]), &[]))
                    .collect();
                assert!(changes.is_empty(), "{changes:?}");
                assert!(m.admitted(&a1.serial));
                assert!(m.in_use(&a1.serial));
                assert!(m.pending().is_empty());
            }
        }
    }

    /// The watcher reads USB without the lock, so the first scan applied after the roster is
    /// known may have been read before the engine started its sensors: it settles nothing. A
    /// listing read before a replug and applied after the open cannot retire the new sensor.
    #[test]
    fn hotplug_a_listing_read_before_the_starts_cannot_retire_their_sensors() {
        let a1 = sensor("SENSORALPHA", "port-a-1", true);
        let a2 = sensor("SENSORALPHA", "port-a-2", true);
        for inventory in [Some(vec![a2.clone()]), Some(vec![]), None] {
            let mut m = Monitor::new(Ok(vec![a1.clone()]), &[]);
            assert!(m.update(DEBOUNCE_NS, Ok(vec![]), &[]).is_none());
            // The watcher reads USB again now, before the replug, and applies it after the open.
            let stale = Ok(vec![]);
            m.before_open();
            m.reconcile_open(
                DEBOUNCE_NS,
                &[stream_start(&a2.serial, &a2.attachment, false)],
                inventory.as_deref(),
            );
            m.opened();
            assert!(m.update(2 * DEBOUNCE_NS, stale, &[]).is_none());
            assert!(m.in_use(&a2.serial));
            assert!(m.admitted(&a2.serial));
            for tick in 3..6 {
                assert!(m
                    .update(tick * DEBOUNCE_NS, Ok(vec![a2.clone()]), &[])
                    .is_none());
            }
            assert!(m.in_use(&a2.serial));
            assert!(m.pending().is_empty());
            // Gone in a listing read after the start, it is retired.
            m.update(6 * DEBOUNCE_NS, Ok(vec![]), &[]);
            let removed = m.update(7 * DEBOUNCE_NS, Ok(vec![]), &[]).unwrap();
            assert_eq!(removed.unplugged, std::slice::from_ref(&a2.serial));
            assert!(!m.admitted(&a2.serial));
        }
    }

    /// A sensor `before_open` expected the engine to try, but whose attempt its roster does not
    /// account for (access went away for a moment, or a failed start named no attachment while
    /// the listing after the open shows another), waits for the next engine start, and the
    /// reconciliation says so. A failed start that names the attachment the watcher knows spends
    /// it, even without a listing after the open, and one that names none spends the expected
    /// attempt when that listing agrees.
    #[test]
    fn hotplug_a_sensor_the_open_did_not_account_for_waits_and_says_so() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        let mut unnamed = stream_start(&a.serial, &a.attachment, true);
        unnamed.attachment = None;
        let mut elsewhere = a.clone();
        elsewhere.attachment = "port-a-2".into();
        for (starts, inventory) in [
            (vec![], vec![a.clone()]),
            (vec![unnamed.clone()], vec![elsewhere]),
        ] {
            let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
            m.before_open();
            let found = m.reconcile_open(1, &starts, Some(&inventory));
            assert_eq!(found.unopened, std::slice::from_ref(&a.serial));
            m.opened();
            assert!(m.wants_open());
        }
        let named = stream_start(&a.serial, &a.attachment, true);
        for (start, inventory) in [(named, None), (unnamed, Some(vec![a.clone()]))] {
            let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
            m.before_open();
            let found = m.reconcile_open(1, &[start], inventory.as_deref());
            assert_eq!(found, Change::default());
            m.opened();
            assert!(m.pending().is_empty());
        }
    }

    /// A sensor the listing read after the open lacks has left, whether or not the engine started
    /// it: it is checked until its removal settles like any other, also once the engine closes,
    /// and is never said to wait for the next engine start. A replacement the watcher saw while
    /// the engine discovered settles only after a debounce from its return. Without that listing
    /// the roster cannot tell a sensor it did not account for from one whose access went away for
    /// a moment, and that one waits.
    #[test]
    fn hotplug_a_sensor_the_listing_after_the_open_lacks_does_not_wait_for_the_next_start() {
        let a = sensor("SENSORALPHA", "port-a-1", true);
        for started in [false, true] {
            let starts = if started {
                vec![stream_start(&a.serial, &a.attachment, false)]
            } else {
                vec![]
            };
            let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
            m.before_open();
            assert_eq!(m.reconcile_open(1, &starts, Some(&[])), Change::default());
            m.opened();
            assert_eq!(m.in_use(&a.serial), started);
            let checking = [("checking_usb", CHECKING_DETAIL.to_string())];
            assert_eq!(details(&m), if started { &[][..] } else { &checking });
            let mut closed = m.clone();
            closed.closed();
            assert_eq!(details(&closed), checking);
            // The first scan applied after the roster may have been read before it: it settles
            // nothing.
            assert!(m.update(2, Ok(vec![]), &[]).is_none());
            let removed = m.update(1 + DEBOUNCE_NS, Ok(vec![]), &[]).unwrap();
            assert_eq!(removed.unplugged, std::slice::from_ref(&a.serial));
            assert!(!m.admitted(&a.serial));
            assert!(!m.in_use(&a.serial));
            assert!(m.pending().is_empty());
        }
        let a2 = sensor("SENSORALPHA", "port-a-2", true);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        m.before_open();
        assert!(m.update(1, Ok(vec![a2.clone()]), &[]).is_none());
        assert_eq!(
            m.reconcile_open(DEBOUNCE_NS, &[], Some(&[])),
            Change::default()
        );
        m.opened();
        for at in [DEBOUNCE_NS + 1, 2 * DEBOUNCE_NS] {
            assert!(m.update(at, Ok(vec![a2.clone()]), &[]).is_none());
        }
        let replaced = m
            .update(2 * DEBOUNCE_NS + 1, Ok(vec![a2.clone()]), &[])
            .unwrap();
        assert_eq!(replaced.unplugged, std::slice::from_ref(&a.serial));
        assert_eq!(replaced.plugged, std::slice::from_ref(&a2));
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        m.before_open();
        assert_eq!(
            m.reconcile_open(1, &[], None).unopened,
            std::slice::from_ref(&a.serial)
        );
        m.opened();
        assert!(m.wants_open());
    }

    /// The ready line and the wait it announces agree: an attempt this engine spent on an older
    /// attachment does not hide that the current one waits for the next engine start.
    #[test]
    fn hotplug_access_ready_on_a_new_attachment_is_announced_despite_an_old_attempt() {
        let a1 = sensor("SENSORALPHA", "port-a-1", true);
        let mut a2 = sensor("SENSORALPHA", "port-a-2", false);
        let mut m = Monitor::new(Ok(vec![]), &[]);
        // Seen, not yet admitted, when the engine opens; its failed start names it.
        assert!(m.update(1, Ok(vec![a1.clone()]), &[]).is_none());
        m.before_open();
        m.reconcile_open(1, &[stream_start(&a1.serial, &a1.attachment, true)], None);
        m.opened();
        let plugged = change(&mut m, 2 * DEBOUNCE_NS, vec![a2.clone()]).unwrap();
        assert_eq!(plugged.plugged, [a2.clone()]);
        a2.ready = true;
        let ready = change(&mut m, 4 * DEBOUNCE_NS, vec![a2.clone()]).unwrap();
        assert_eq!(ready.ready, [a2.serial.clone()]);
        assert!(m.wants_open());
    }

    /// While the engine discovers, a sensor it may open waits for that open, whatever the watcher
    /// sees, and its access becoming ready is not announced as waiting for the next start: the
    /// roster decides that.
    #[test]
    fn hotplug_nothing_said_while_the_engine_discovers_claims_what_it_will_do() {
        let mut a = sensor("SENSORALPHA", "port-a-1", false);
        let mut m = Monitor::new(Ok(vec![a.clone()]), &[]);
        m.before_open();
        assert_eq!(details(&m), [("opening", OPENING_DETAIL.to_string())]);
        a.ready = true;
        assert!(change(&mut m, 1, vec![a.clone()]).is_none());
        assert_eq!(details(&m), [("opening", OPENING_DETAIL.to_string())]);
        m.reconcile_open(
            1 + DEBOUNCE_NS,
            &[stream_start(&a.serial, &a.attachment, false)],
            Some(std::slice::from_ref(&a)),
        );
        m.opened();
        for tick in 3..6 {
            assert!(m
                .update(tick * DEBOUNCE_NS, Ok(vec![a.clone()]), &[])
                .is_none());
        }
        assert!(m.pending().is_empty());
    }

    /// A closed engine holds no sensor: a change seen before the next open is reported like any
    /// other, not taken for a sensor in use.
    #[test]
    fn hotplug_a_closed_engine_uses_no_sensor() {
        let a1 = sensor("SENSORALPHA", "port-a-1", true);
        let a2 = sensor("SENSORALPHA", "port-a-2", true);
        let mut m = Monitor::new(Ok(vec![a1.clone()]), &[]);
        m.before_open();
        // No listing after the start: a replacement would not rule out the device it opened.
        m.reconcile_open(1, &[stream_start(&a1.serial, &a1.attachment, false)], None);
        m.opened();
        m.closed();
        assert!(!m.in_use(&a1.serial));
        let replaced = change(&mut m, 1, vec![a2.clone()]).unwrap();
        assert_eq!(replaced.unplugged, std::slice::from_ref(&a1.serial));
        assert_eq!(replaced.plugged, std::slice::from_ref(&a2));
        assert!(m.waits_for_engine_start());
    }

    /// A failed start spends the attachment the listing after the open shows for it, even when
    /// the watcher has not caught up: a replacement it saw but had not settled, one it did not
    /// see, or a gap it saw. The stream check reports the failure; the sensor is not also said to
    /// wait for the next engine start.
    #[test]
    fn hotplug_a_failed_start_spends_its_listed_attachment_whatever_the_watcher_saw() {
        let a1 = sensor("SENSORALPHA", "port-a-1", true);
        let a2 = sensor("SENSORALPHA", "port-a-2", true);
        for (seen, started) in [
            (Some(vec![a2.clone()]), &a2),
            (None, &a2),
            (Some(vec![]), &a1),
        ] {
            let mut m = Monitor::new(Ok(vec![a1.clone()]), &[]);
            if let Some(seen) = seen.clone() {
                assert!(m.update(1, Ok(seen), &[]).is_none());
            }
            m.before_open();
            m.reconcile_open(
                1,
                &[stream_start(&started.serial, &started.attachment, true)],
                Some(std::slice::from_ref(started)),
            );
            m.opened();
            for tick in 2..5 {
                m.update(tick * DEBOUNCE_NS, Ok(vec![started.clone()]), &[]);
            }
            assert!(m.pending().is_empty(), "{seen:?}: {:?}", details(&m));
        }
    }
}
