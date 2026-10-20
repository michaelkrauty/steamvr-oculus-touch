// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! An exhaustive, bounded explorer of sensor hot-plug around engine opens. It drives the real
//! `sensor_hotplug::Monitor` through the daemon's own glue: `poll_sensors` for each USB scan,
//! `begin_engine_open` and `finish_engine_open` around `Engine::open`, `admit_engine_sensors`
//! (the roster's reconciliation and engine_session's retirement sweep), `engine_updated`,
//! `retire_gone_sensors` for a native NO_DEVICE and `end_engine_sensors` when the engine closes.
//! Native retirement and log lines are recorded, not performed: nothing opens a device.
//!
//! The model is the physical world, the watcher thread and the native engine around that glue.
//! Each serial is absent or attached at a USB generation, a new one for every plug, with its
//! access ready or not and (Windows) its arrival property known or not. The watcher reads USB
//! without the watch's lock and applies what it read later, as `sensor_watch_main` does, so an
//! open can run in between: a read lists exactly the world at that moment, or fails; a tick
//! moves the clock one debounce on. An engine open begins (`before_open`), takes the Windows
//! arrival snapshot, then discovers: every attached, accessible device gets a sensor object
//! whose video starts or fails, and whose start record names its generation, no generation (a
//! failed Linux pin, a missing arrival) or the snapshot's older one (a replug between the
//! snapshot and discovery). It can also fail outright (no headset). The post-open listing is read
//! later, and fails or lacks an arrival like any read; then the engine is admitted and its first
//! update finishes the open. A running engine's object whose device left reports NO_DEVICE at
//! any later step, and a mode change or I/O failure closes it for the next open. Every one of
//! these steps interleaves with every other, within the bounds.
//!
//! After every step it checks:
//! - I1: no object whose device is still attached at the generation it opened is retired, and
//!   its serial stays admitted.
//! - I2: an object whose device left is retired by the step that delivers its native
//!   NO_DEVICE, and, if the post-open listing showed the device it opened, by the watcher once
//!   its departure settles: two scans a debounce apart, read after the open (checked at
//!   quiescence).
//! - I3: while the engine holds a live object, its serial is not pending except as `opening`,
//!   and doctor's hot-plug check passes for it, also while the engine is still opening.
//! - I4 (at quiescence): an attached, ready sensor without a live object waits for the next
//!   engine start, nothing is stuck checking or opening, and the next start opens it.
//! - I5: once an engine is running, the last plugged/unplugged/ready line logged about a serial
//!   it is using since it began opening does not say the sensor is gone or waits; nothing, logged
//!   or pending, says a sensor waits for the next engine start while it is unplugged and the last
//!   listing the watch was given lacks it; and a sensor waiting for the next engine start was
//!   last logged as that (at quiescence).
//! - I6 (at quiescence): a settled reopen leaves the watch exactly as it was, so nothing it keeps
//!   grows from one engine cycle to the next.
//!
//! Quiescence is a canonical completion of each state: the open finishes with every start
//! identified, scans settle the world, native NO_DEVICE arrives for every object whose device
//! left, and the engine is opened once more. States are deduplicated on the whole model and the
//! watch's `Debug`. The search is depth-first and keeps each state's smallest depth, so every
//! state within the bound is explored and each violation is reported with a shortest trace.
//!
//! `cargo test` runs one sensor to depth 13 and two to depth 8; the ignored
//! `hotplug_explorer_deeper` runs one sensor to depth 14 with more plugs, ticks, opens and flips,
//! and two to depth 10 (about five minutes in all).

use super::*;
use cv1touch::sensor_hotplug::{Monitor, DEBOUNCE_NS};
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::hash::{DefaultHasher, Hash, Hasher};

const SERIALS: [&str; 2] = ["SENSORALPHA", "SENSORBRAVO"];

thread_local! {
    static RETIRED: RefCell<Option<Vec<String>>> = const { RefCell::new(None) };
}

/// Records a native retirement `retire_native_sensors` asks for while the explorer runs a step.
pub(super) fn note_retired(serial: &str) {
    RETIRED.with_borrow_mut(|retired| {
        if let Some(retired) = retired {
            retired.push(serial.to_owned());
        }
    });
}

fn attachment(serial: usize, generation: u8) -> String {
    format!("{}@{generation}", SERIALS[serial])
}

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
struct Device {
    generation: u8,
    ready: bool,
    arrival: bool,
}

/// One serial in a USB listing: the generation if its arrival is known, and its readiness.
type Entry = Option<(Option<u8>, bool)>;

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
struct Object {
    /// The device native discovery opened.
    generation: u8,
    /// The generation its start record names.
    stamp: Option<u8>,
    /// Its video started.
    started: bool,
    retired: bool,
    reported_gone: bool,
}

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
enum Phase {
    /// No engine: the daemon just started, its open failed, or it closed for a reopen.
    Closed,
    /// `before_open` ran; `Engine::open` has not discovered the sensors yet.
    Opening,
    /// `Engine::open` returned with its sensor objects.
    Discovered,
    /// engine_session admitted its sensors.
    Running { updated: bool },
}

/// What native discovery did with one serial.
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
enum Start {
    Absent,
    Started(Option<u8>),
    Failed(Option<u8>),
}

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
enum Step {
    Unplug(usize),
    Plug(usize),
    Access(usize, bool),
    Arrival(usize, bool),
    /// The watcher reads USB (without the watch's lock): exactly the world, or a failure.
    Read(bool),
    /// The watcher applies what it read (`poll_sensors`).
    Apply,
    Tick,
    BeginOpen,
    OpenFails,
    Snapshot,
    Discover([Start; 2]),
    /// The post-open listing: read, or failed.
    Listing(bool),
    Admit,
    Update,
    NativeGone(usize),
    Close,
}

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
enum Claim {
    Gone,
    NextStart,
    Access,
    InUse,
    Noticed,
}

fn claim(line: &str) -> Option<Claim> {
    if line.starts_with("sensor unplugged: ") {
        Some(Claim::Gone)
    } else if line.contains("will be used the next time the tracking engine starts") {
        Some(Claim::NextStart)
    } else if line.contains("waiting for access to its USB device") {
        Some(Claim::Access)
    } else if line.contains("in use") || line.contains("is using it") {
        Some(Claim::InUse)
    } else if line.contains("noticed while the engine was opening") {
        Some(Claim::Noticed)
    } else {
        None
    }
}

#[derive(Clone, Copy, Debug)]
struct Bounds {
    serials: usize,
    depth: usize,
    plugs: u8,
    ticks: u8,
    opens: u8,
    flips: u8,
    scan_errors: u8,
}

#[derive(Clone)]
struct State {
    world: [Option<Device>; 2],
    generations: [u8; 2],
    plugs: u8,
    ticks: u8,
    flips: u8,
    scan_errors: u8,
    opens: u8,
    phase: Phase,
    snapshot: Option<[Option<u8>; 2]>,
    objects: [Option<Object>; 2],
    listing: Option<Option<[Entry; 2]>>,
    /// What the watcher read and has not applied yet (`None` inside: the read failed).
    read: Option<Option<[Entry; 2]>>,
    /// The last line about each serial since the current engine began opening, and ever.
    claims: [Option<Claim>; 2],
    ever: [Option<Claim>; 2],
    /// The last listing the watch was given (a scan applied, or the post-open listing at
    /// admission) lacks the serial. A failed one tells it nothing.
    shown_gone: [bool; 2],
    /// The last line about the serial says it waits for the next engine start, and was logged
    /// while it was unplugged and shown gone.
    false_next_start: [bool; 2],
    watch: Monitor,
}

impl State {
    fn key(&self) -> u64 {
        let mut h = DefaultHasher::new();
        (self.quiescence_key(), self.generations, self.budget()).hash(&mut h);
        h.finish()
    }

    /// What a canonical completion depends on: everything but the bounds' budgets.
    fn quiescence_key(&self) -> u64 {
        let mut h = DefaultHasher::new();
        (
            self.world,
            self.ticks,
            self.phase,
            self.snapshot,
            self.objects,
            self.listing,
            self.read,
            self.claims,
            self.ever,
            self.shown_gone,
            self.false_next_start,
        )
            .hash(&mut h);
        format!("{:?}", self.watch).hash(&mut h);
        h.finish()
    }

    fn budget(&self) -> (u8, u8, u8, u8) {
        (self.plugs, self.flips, self.scan_errors, self.opens)
    }

    fn now(&self) -> u64 {
        (u64::from(self.ticks) + 1) * DEBOUNCE_NS
    }

    fn entries(&self) -> [Entry; 2] {
        self.world
            .map(|dev| dev.map(|d| (d.arrival.then_some(d.generation), d.ready)))
    }

    fn sensors(entries: &[Entry; 2]) -> Vec<platform::usb::Sensor> {
        entries
            .iter()
            .enumerate()
            .filter_map(|(i, e)| {
                let (generation, ready) = (*e)?;
                Some(platform::usb::Sensor {
                    serial: SERIALS[i].into(),
                    attachment: generation.map(|g| attachment(i, g)).unwrap_or_default(),
                    ready,
                })
            })
            .collect()
    }

    fn initial(world: [Option<Device>; 2]) -> Self {
        let s = Self {
            world,
            generations: world.map(|d| d.map_or(0, |d| d.generation)),
            plugs: 0,
            ticks: 0,
            flips: 0,
            scan_errors: 0,
            opens: 0,
            phase: Phase::Closed,
            snapshot: None,
            objects: [None; 2],
            listing: None,
            read: None,
            claims: [None; 2],
            ever: [None; 2],
            shown_gone: world.map(|d| d.is_none()),
            false_next_start: [false; 2],
            watch: Monitor::new(Ok(Vec::new()), &[]),
        };
        Self {
            watch: Monitor::new(Ok(Self::sensors(&s.entries())), &[]),
            ..s
        }
    }

    /// The daemon just started with `world` attached, and its first engine running with every
    /// sensor it could open: the second leaves the bound to what happens around a reopen.
    fn roots(d: &Daemon, b: &Bounds, world: [Option<Device>; 2]) -> [State; 2] {
        let s = State::initial(world);
        let mut running = s.clone();
        let q = &mut Quiescence {
            d,
            b,
            trace: Vec::new(),
            before: HashSet::new(),
            found: None,
        };
        running.finish_open(q);
        if let Some((f, steps)) = &q.found {
            panic!(
                "the first engine open already violates an invariant:\n{} [{}]: {}\n  steps: {steps:?}",
                f.id, f.kind, f.message
            );
        }
        running.opens = 0;
        [s, running]
    }

    fn healthy(&self, i: usize, o: &Object) -> bool {
        o.started && self.world[i].is_some_and(|d| d.generation == o.generation)
    }

    fn live(o: &Object) -> bool {
        o.started && !o.retired
    }

    /// Runs daemon glue against this state's watch, recording native retirement and log lines.
    fn glue(&mut self, d: &Daemon, run: impl FnOnce(&Daemon)) {
        std::mem::swap(&mut *d.sensor_watch.lock().unwrap(), &mut self.watch);
        *d.sensor_retirement.lock().unwrap() =
            matches!(self.phase, Phase::Running { .. }).then(engine::SensorRetirement::default);
        RETIRED.set(Some(Vec::new()));
        let lines = daemon_log::capture(|| run(d));
        let retired = RETIRED.take().unwrap_or_default();
        std::mem::swap(&mut *d.sensor_watch.lock().unwrap(), &mut self.watch);
        for serial in retired {
            for (i, o) in self.objects.iter_mut().enumerate() {
                if let Some(o) = o
                    .as_mut()
                    .filter(|_| cv1touch::status::same_sensor(SERIALS[i], &serial))
                {
                    o.retired = true;
                }
            }
        }
        for line in lines {
            let Some(c) = claim(&line) else { continue };
            for (i, serial) in SERIALS.iter().enumerate() {
                if line.contains(serial) {
                    self.claims[i] = Some(c);
                    self.ever[i] = Some(c);
                    self.false_next_start[i] =
                        c == Claim::NextStart && self.world[i].is_none() && self.shown_gone[i];
                }
            }
        }
    }

    fn starts(&self) -> Vec<engine::SensorStreamStart> {
        self.objects
            .iter()
            .enumerate()
            .filter_map(|(i, o)| {
                let o = o.as_ref()?;
                let mut raw = ffi::ohmd_cv1touch_sensor_stream::default();
                for (out, byte) in raw.serial.iter_mut().zip(SERIALS[i].bytes()) {
                    *out = byte as std::ffi::c_char;
                }
                if o.started {
                    raw.alt_setting = 2;
                } else {
                    raw.error_kind = ffi::OHMD_CV1TOUCH_STREAM_ERROR_OTHER;
                    raw.error_code = -1;
                }
                let mut start = engine::SensorStreamStart::from(raw);
                start.attachment = o.stamp.map(|g| attachment(i, g));
                Some(start)
            })
            .collect()
    }

    fn steps(&self, b: &Bounds) -> Vec<Step> {
        let mut steps = Vec::new();
        for i in 0..b.serials {
            match self.world[i] {
                Some(dev) => {
                    steps.push(Step::Unplug(i));
                    if self.flips < b.flips {
                        steps.push(Step::Access(i, !dev.ready));
                        steps.push(Step::Arrival(i, !dev.arrival));
                    }
                }
                None if self.plugs < b.plugs => steps.push(Step::Plug(i)),
                None => {}
            }
        }
        if self.read.is_some() {
            steps.push(Step::Apply);
        } else {
            steps.push(Step::Read(true));
            if self.scan_errors < b.scan_errors {
                steps.push(Step::Read(false));
            }
        }
        if self.ticks < b.ticks {
            steps.push(Step::Tick);
        }
        match self.phase {
            Phase::Closed if self.opens < b.opens => steps.push(Step::BeginOpen),
            Phase::Closed => {}
            Phase::Opening if self.snapshot.is_none() => {
                steps.push(Step::Snapshot);
                steps.push(Step::OpenFails);
            }
            Phase::Opening => {
                steps.extend(self.discoveries(b).into_iter().map(Step::Discover));
                steps.push(Step::OpenFails);
            }
            Phase::Discovered if self.listing.is_none() => {
                steps.push(Step::Listing(true));
                if self.scan_errors < b.scan_errors {
                    steps.push(Step::Listing(false));
                }
            }
            Phase::Discovered => steps.push(Step::Admit),
            Phase::Running { updated } => {
                if !updated {
                    steps.push(Step::Update);
                }
                for (i, o) in self.objects.iter().enumerate() {
                    if o.is_some_and(|o| o.started && !o.reported_gone && !self.healthy(i, &o)) {
                        steps.push(Step::NativeGone(i));
                    }
                }
                steps.push(Step::Close);
            }
        }
        steps
    }

    /// Every combination of what native discovery can do with each attached, accessible serial.
    fn discoveries(&self, b: &Bounds) -> Vec<[Start; 2]> {
        let mut all = vec![[Start::Absent; 2]];
        for i in 0..b.serials {
            let Some(dev) = self.world[i].filter(|d| d.ready) else {
                continue;
            };
            let mut stamps = vec![Some(dev.generation), None];
            if let Some(g) = self.snapshot.and_then(|s| s[i]) {
                if g != dev.generation {
                    stamps.push(Some(g));
                }
            }
            all = all
                .into_iter()
                .flat_map(|starts| {
                    stamps.iter().flat_map(move |&stamp| {
                        [Start::Started(stamp), Start::Failed(stamp)].map(|start| {
                            let mut starts = starts;
                            starts[i] = start;
                            starts
                        })
                    })
                })
                .collect();
        }
        all
    }

    fn apply(&self, d: &Daemon, step: Step) -> State {
        let mut s = self.clone();
        match step {
            Step::Unplug(i) => s.world[i] = None,
            Step::Plug(i) => {
                s.plugs += 1;
                s.generations[i] += 1;
                s.world[i] = Some(Device {
                    generation: s.generations[i],
                    ready: true,
                    arrival: true,
                });
            }
            Step::Access(i, ready) => {
                s.flips += 1;
                s.world[i].as_mut().unwrap().ready = ready;
            }
            Step::Arrival(i, arrival) => {
                s.flips += 1;
                s.world[i].as_mut().unwrap().arrival = arrival;
            }
            Step::Read(ok) => {
                s.scan_errors += u8::from(!ok);
                s.read = Some(ok.then(|| s.entries()));
            }
            Step::Apply => {
                let now = s.now();
                let listed = match s.read.take().unwrap() {
                    Some(entries) => {
                        s.shown_gone = entries.map(|e| e.is_none());
                        Ok(Self::sensors(&entries))
                    }
                    None => Err(io::Error::other("listing interrupted")),
                };
                s.glue(d, |d| poll_sensors(d, now, listed));
            }
            Step::Tick => s.ticks += 1,
            Step::BeginOpen => {
                s.opens += 1;
                s.phase = Phase::Opening;
                s.snapshot = None;
                s.objects = [None; 2];
                s.listing = None;
                s.claims = [None; 2];
                s.false_next_start = [false; 2];
                s.glue(d, begin_engine_open);
            }
            Step::OpenFails => {
                s.phase = Phase::Closed;
                s.glue(d, |d| {
                    finish_engine_open::<()>(d, &mut String::new(), Err("no headset".into()));
                });
            }
            Step::Snapshot => {
                s.snapshot = Some(
                    s.world
                        .map(|dev| dev.filter(|d| d.arrival).map(|d| d.generation)),
                );
            }
            Step::Discover(starts) => {
                for (i, start) in starts.into_iter().enumerate() {
                    let generation = s.world[i].map_or(0, |d| d.generation);
                    s.objects[i] = match start {
                        Start::Absent => None,
                        Start::Started(stamp) | Start::Failed(stamp) => Some(Object {
                            generation,
                            stamp,
                            started: matches!(start, Start::Started(_)),
                            retired: false,
                            reported_gone: false,
                        }),
                    };
                }
                s.phase = Phase::Discovered;
                s.glue(d, |d| {
                    finish_engine_open(d, &mut String::new(), Ok(()));
                });
            }
            Step::Listing(ok) => s.listing = Some(ok.then(|| s.entries())),
            Step::Admit => {
                s.phase = Phase::Running { updated: false };
                let starts = s.starts();
                let listing = match s.listing.unwrap() {
                    Some(entries) => {
                        s.shown_gone = entries.map(|e| e.is_none());
                        Ok(Self::sensors(&entries))
                    }
                    None => Err(io::Error::other("listing interrupted")),
                };
                let now = s.now();
                s.glue(d, |d| {
                    admit_engine_sensors(
                        d,
                        now,
                        &[],
                        &starts,
                        listing,
                        engine::SensorRetirement::default(),
                    )
                });
            }
            Step::Update => {
                s.phase = Phase::Running { updated: true };
                s.glue(d, |d| engine_updated(d, true, &mut true));
            }
            Step::NativeGone(i) => {
                s.objects[i].as_mut().unwrap().reported_gone = true;
                let mut native = engine::Sensor::default();
                for (slot, byte) in native.serial.iter_mut().zip(SERIALS[i].bytes()) {
                    *slot = byte as _;
                }
                native.usb_gone = 1;
                s.glue(d, |d| retire_gone_sensors(d, &[native], &[]));
            }
            Step::Close => {
                s.phase = Phase::Closed;
                s.objects = [None; 2];
                s.glue(d, end_engine_sensors);
            }
        }
        s
    }

    /// The serial's pending state, and whether doctor's hot-plug check passes for it alone. An
    /// engine opening it passes by definition: doctor samples again.
    fn pending(&self, i: usize) -> Option<(&'static str, bool)> {
        let state = self
            .watch
            .pending()
            .into_iter()
            .find(|p| cv1touch::status::same_sensor(&p.serial, SERIALS[i]))?
            .state;
        if state == "opening" {
            return Some((state, true));
        }
        let status = hotplug_status(&self.watch);
        let entry = status["pending"]
            .as_array()
            .unwrap()
            .iter()
            .find(|p| cv1touch::status::same_sensor(p["serial"].as_str().unwrap(), SERIALS[i]))
            .unwrap()
            .clone();
        let doctor =
            serde_json::json!({"sensor_hotplug": {"pending": [entry], "scan_error": null}});
        let passes = cv1touch::status::sensor_hotplug_check(&doctor, &Default::default())
            .is_none_or(|(ok, _)| ok);
        Some((state, passes))
    }

    /// How this object's start and the post-open listing identified its device.
    fn identity(&self, i: usize, o: &Object) -> String {
        let stamp = match o.stamp {
            Some(g) if g == o.generation => "its start names its generation",
            Some(_) => "its start names an older generation",
            None => "its start names no generation",
        };
        let listing = match self.listing {
            None => "",
            Some(None) => ", the post-open listing failed",
            Some(Some(entries)) => match entries[i] {
                None => ", the post-open listing lacks it",
                Some((None, _)) => ", the post-open listing lacks its arrival",
                Some((Some(g), _)) if g == o.generation => ", the post-open listing shows it",
                Some((Some(_), _)) => ", the post-open listing shows another generation",
            },
        };
        format!("{stamp}{listing}")
    }

    /// I1, I2 (native evidence), I3 and I5 after `step`.
    fn check(&self, step: Step, b: &Bounds) -> Vec<Finding> {
        let mut out = Vec::new();
        let engine = !matches!(self.phase, Phase::Closed | Phase::Opening);
        let running = matches!(self.phase, Phase::Running { .. });
        for (i, serial) in SERIALS.iter().copied().enumerate().take(b.serials) {
            let Some(o) = self.objects[i].filter(|_| engine) else {
                continue;
            };
            let mut found = |id, kind: String, message: String| {
                out.push(Finding {
                    id,
                    serial: i,
                    kind,
                    message,
                })
            };
            if running && self.healthy(i, &o) && (o.retired || !self.watch.admitted(serial)) {
                found(
                    "I1",
                    self.identity(i, &o),
                    format!(
                        "{serial} retired while attached at generation {} (natively: {}, admitted: {})",
                        o.generation,
                        o.retired,
                        self.watch.admitted(serial)
                    ),
                );
            }
            if step == Step::NativeGone(i) && !o.retired {
                found(
                    "I2",
                    "NO_DEVICE retires nothing".into(),
                    format!(
                        "{serial}'s NO_DEVICE did not retire it; {}",
                        self.identity(i, &o)
                    ),
                );
            }
            if Self::live(&o) {
                if let Some((state, passes)) = self.pending(i).filter(|p| *p != ("opening", true)) {
                    found(
                        if running { "I3" } else { "I3-open" },
                        format!("{state}; {}", self.identity(i, &o)),
                        format!(
                            "{serial} is in use but pending as {state} (doctor passes: {passes})"
                        ),
                    );
                }
            }
            if running && self.healthy(i, &o) && !o.retired {
                if let Some(c @ (Claim::Gone | Claim::NextStart | Claim::Access)) = self.claims[i] {
                    found(
                        "I5",
                        format!("in use, last logged {c:?}"),
                        format!(
                            "{serial} is in use, but the last line logged about it since this engine began opening says {c:?}"
                        ),
                    );
                }
            }
        }
        // Whatever the engine does, a sensor that is unplugged, and that the last listing the
        // watch was given lacks, is not said to wait for the next engine start.
        for (i, serial) in SERIALS.iter().copied().enumerate().take(b.serials) {
            let mut found = |kind: &str, message: String| {
                out.push(Finding {
                    id: "I5",
                    serial: i,
                    kind: kind.into(),
                    message,
                })
            };
            if self.false_next_start[i] {
                found(
                    "logged NextStart while unplugged and listed gone",
                    format!("{serial} was logged as waiting for the next engine start after a listing showed it gone"),
                );
            }
            if self.world[i].is_none()
                && self.shown_gone[i]
                && self
                    .pending(i)
                    .is_some_and(|p| p.0 == "waiting_engine_start")
            {
                found(
                    "waits for the next start while unplugged and listed gone",
                    format!("{serial} is unplugged and the last listing lacks it, but it waits for the next engine start"),
                );
            }
        }
        out
    }

    /// The canonical completion: finish the open with every start identified.
    fn finish_open(&mut self, q: &mut Quiescence) {
        loop {
            let step = match self.phase {
                Phase::Closed => Step::BeginOpen,
                Phase::Opening if self.snapshot.is_none() => Step::Snapshot,
                Phase::Opening => {
                    let mut starts = [Start::Absent; 2];
                    for (i, start) in starts.iter_mut().enumerate().take(q.b.serials) {
                        if let Some(dev) = self.world[i].filter(|d| d.ready) {
                            *start = Start::Started(Some(dev.generation));
                        }
                    }
                    Step::Discover(starts)
                }
                Phase::Discovered if self.listing.is_none() => Step::Listing(true),
                Phase::Discovered => Step::Admit,
                Phase::Running { updated: false } => Step::Update,
                Phase::Running { updated: true } => return,
            };
            if !self.run(step, q) {
                return;
            }
        }
    }

    /// One step of a completion, which stops at its first new violation.
    fn run(&mut self, step: Step, q: &mut Quiescence) -> bool {
        if q.found.is_some() {
            return false;
        }
        *self = self.apply(q.d, step);
        q.trace.push(step);
        let out = self.check(step, q.b);
        q.note(out);
        true
    }

    fn settle(&mut self, cycles: usize, q: &mut Quiescence) {
        if self.read.is_some() {
            self.run(Step::Apply, q);
        }
        for _ in 0..cycles {
            self.run(Step::Tick, q);
            self.run(Step::Read(true), q);
            self.run(Step::Apply, q);
        }
    }

    /// I2 (watcher evidence), I4, I5 (waiting sensors) and I6 at this state's quiescence.
    fn check_quiescence(&self, d: &Daemon, b: &Bounds, before: &[Finding]) -> Option<Found> {
        let q = &mut Quiescence {
            d,
            b,
            trace: Vec::new(),
            before: before.iter().map(Finding::key).collect(),
            found: None,
        };
        let mut s = self.clone();
        s.finish_open(q);
        s.settle(2, q);
        // The watcher retires an object whose departure it can attribute: the post-open listing
        // showed the device it opened, and two scans a debounce apart list the serial without
        // it (gone, or at another known arrival; an unknown arrival rules nothing out).
        let mut out = Vec::new();
        for i in 0..b.serials {
            let Some(o) = s.objects[i] else { continue };
            let listed = s.listing.flatten().and_then(|l| l[i]).and_then(|e| e.0);
            let ruled_out = s.world[i].is_none_or(|d| d.arrival && d.generation != o.generation);
            if o.started && ruled_out && listed == Some(o.generation) && !o.retired {
                out.push(Finding {
                    id: "I2",
                    serial: i,
                    kind: "a settled departure retires nothing".into(),
                    message: format!(
                        "{}'s settled departure did not retire it; {}",
                        SERIALS[i],
                        s.identity(i, &o)
                    ),
                });
            }
        }
        q.found_new(out);
        for i in 0..b.serials {
            if s.objects[i].is_some_and(|o| o.started && !o.reported_gone && !s.healthy(i, &o)) {
                s.run(Step::NativeGone(i), q);
            }
        }
        s.settle(3, q);
        let mut out = Vec::new();
        for (i, serial) in SERIALS.iter().copied().enumerate().take(b.serials) {
            let o = s.objects[i];
            let mut found = |id, kind: String, message: String| {
                out.push(Finding {
                    id,
                    serial: i,
                    kind,
                    message,
                })
            };
            if o.is_some_and(|o| o.started && !s.healthy(i, &o) && !o.retired) {
                found(
                    "I2",
                    "NO_DEVICE and a settled departure retire nothing".into(),
                    format!("{serial} left and reported NO_DEVICE but is not retired"),
                );
            }
            let pending = s.pending(i).map(|p| p.0);
            let usable = s.world[i].is_some_and(|d| d.ready && d.arrival);
            // This engine's one attempt at this attachment failed: it is not retried until the
            // next start, and the stream check reports why.
            let failed_here = o.is_some_and(|o| {
                !o.started && s.world[i].is_some_and(|d| d.generation == o.generation)
            });
            let waits = pending == Some("waiting_engine_start") || failed_here && pending.is_none();
            if usable && !o.is_some_and(|o| Self::live(&o)) && !waits {
                found(
                    "I4",
                    format!("not waiting for the next start but {pending:?}"),
                    format!("{serial} is attached and ready without a live object, but pending as {pending:?}"),
                );
            }
            if let Some(
                state @ ("opening" | "checking_usb" | "waiting_replug" | "waiting_headset"),
            ) = pending
            {
                found(
                    "I4",
                    format!("stuck as {state}"),
                    format!("{serial} stuck as {state}"),
                );
            }
            if pending == Some("waiting_engine_start") && s.ever[i] != Some(Claim::NextStart) {
                found(
                    "I5",
                    format!("waits for the next start, last logged {:?}", s.ever[i]),
                    format!(
                        "{serial} waits for the next engine start, but the last line logged about it says {:?}",
                        s.ever[i]
                    ),
                );
            }
        }
        q.found_new(out);
        // The next engine start opens every attached, accessible sensor.
        s.run(Step::Close, q);
        s.finish_open(q);
        s.settle(2, q);
        let mut out = Vec::new();
        for (i, serial) in SERIALS.iter().enumerate().take(b.serials) {
            if s.world[i].is_some_and(|d| d.ready && d.arrival) && s.pending(i).is_some() {
                out.push(Finding {
                    id: "I4",
                    serial: i,
                    kind: "the next start does not open it".into(),
                    message: format!("the next engine start did not open {serial}"),
                });
            }
        }
        q.found_new(out);
        // Settled, another reopen leaves the watch as it was: nothing accumulates per cycle.
        let settled = format!("{:?}", s.watch);
        s.run(Step::Close, q);
        s.finish_open(q);
        s.settle(2, q);
        let again = format!("{:?}", s.watch);
        if again != settled {
            q.found_new(vec![Finding {
                id: "I6",
                serial: 0,
                kind: "a settled reopen changes the watch".into(),
                message: format!("a settled reopen changed the watch:\n    {settled}\n    {again}"),
            }]);
        }
        q.found.take()
    }
}

/// A violation, with the steps of the completion that found it.
type Found = (Finding, Vec<Step>);

#[derive(Clone)]
struct Finding {
    id: &'static str,
    serial: usize,
    /// What kind of violation it is: the shortest trace of each kind is reported.
    kind: String,
    message: String,
}

impl Finding {
    fn key(&self) -> (&'static str, usize, String) {
        (self.id, self.serial, self.kind.clone())
    }
}

/// A canonical completion in progress. It reports only violations its own steps cause: one its
/// state already had is reported where it first occurred.
struct Quiescence<'a> {
    d: &'a Daemon,
    b: &'a Bounds,
    trace: Vec<Step>,
    before: HashSet<(&'static str, usize, String)>,
    found: Option<Found>,
}

impl Quiescence<'_> {
    /// The violations after a step: what holds now replaces what held before it.
    fn note(&mut self, out: Vec<Finding>) {
        let keys = out.iter().map(Finding::key).collect();
        self.found_new(out);
        self.before = keys;
    }

    /// Violations of a check at the end of a stage, which takes no step.
    fn found_new(&mut self, out: Vec<Finding>) {
        if self.found.is_none() {
            if let Some(f) = out.into_iter().find(|f| !self.before.contains(&f.key())) {
                self.found = Some((f, self.trace.clone()));
            }
        }
    }
}

struct Report {
    states: usize,
    transitions: usize,
    completions: usize,
    depth: usize,
    violations: Vec<String>,
}

/// A depth-first search that keeps each state's smallest depth: a state reached again by a
/// shorter path is explored again, so every state within the bound is, and each violation is
/// reported with a shortest trace. Only state hashes are kept.
struct Search<'a> {
    d: &'a Daemon,
    b: Bounds,
    seen: HashMap<u64, u8>,
    /// Completions checked, by state but its bounds' budgets: the depth checked at, and what was
    /// found.
    quiet: HashMap<u64, (u8, Option<Found>)>,
    path: Vec<Step>,
    transitions: usize,
    deepest: usize,
    found: HashMap<(&'static str, String), (usize, String)>,
}

impl Search<'_> {
    fn report(&mut self, f: &Finding, completing: &[Step]) {
        let by = completing
            .last()
            .or(self.path.last())
            .map(|s| format!("{s:?}"));
        let by = by.as_deref().map_or("", |s| s.split('(').next().unwrap());
        let kind = if f.id == "I1" {
            format!("retired by {by}; {}", f.kind)
        } else {
            f.kind.clone()
        };
        let length = self.path.len() + completing.len();
        if self
            .found
            .get(&(f.id, kind.clone()))
            .is_some_and(|(shortest, _)| *shortest <= length)
        {
            return;
        }
        let mut text = format!("{} [{kind}]: {}\n", f.id, f.message);
        for (n, step) in self.path.iter().enumerate() {
            text += &format!("  {:2}. {step:?}\n", n + 1);
        }
        if !completing.is_empty() {
            text += &format!("  then, completing: {completing:?}\n");
        }
        self.found.insert((f.id, kind), (length, text));
    }

    fn visit(&mut self, s: &State) {
        let depth = self.path.len();
        self.deepest = self.deepest.max(depth);
        // A violation a state already has was reported by the step that caused it.
        let before = s.check(Step::Tick, &self.b);
        // Intermediate phases complete through states the search also reaches.
        if matches!(s.phase, Phase::Closed | Phase::Running { updated: true }) {
            let key = s.quiescence_key();
            match self.quiet.get(&key) {
                Some((at, _)) if *at as usize <= depth => {}
                Some((_, found)) => {
                    let found = found.clone();
                    if let Some((f, completing)) = &found {
                        self.report(f, completing);
                    }
                    self.quiet.insert(key, (depth as u8, found));
                }
                None => {
                    let found = s.check_quiescence(self.d, &self.b, &before);
                    if let Some((f, completing)) = &found {
                        self.report(f, completing);
                    }
                    self.quiet.insert(key, (depth as u8, found));
                }
            }
        }
        if depth == self.b.depth {
            return;
        }
        let before: HashSet<_> = before.iter().map(Finding::key).collect();
        for step in s.steps(&self.b) {
            let n = s.apply(self.d, step);
            self.transitions += 1;
            self.path.push(step);
            for f in n.check(step, &self.b) {
                if !before.contains(&f.key()) {
                    self.report(&f, &[]);
                }
            }
            let at = depth as u8 + 1;
            if self.seen.get(&n.key()).is_none_or(|seen| at < *seen) {
                self.seen.insert(n.key(), at);
                self.visit(&n);
            }
            self.path.pop();
        }
    }
}

fn explore(b: Bounds) -> Report {
    let d = super::tests::status_daemon();
    let mut search = Search {
        d: &d,
        b,
        seen: HashMap::new(),
        quiet: HashMap::new(),
        path: Vec::new(),
        transitions: 0,
        deepest: 0,
        found: HashMap::new(),
    };
    let worlds = [
        None,
        Some(Device {
            generation: 1,
            ready: true,
            arrival: true,
        }),
        Some(Device {
            generation: 1,
            ready: false,
            arrival: true,
        }),
    ];
    let mut roots = Vec::new();
    for a in worlds {
        for c in if b.serials > 1 {
            &worlds[..]
        } else {
            &worlds[..1]
        } {
            roots.extend(State::roots(&d, &b, [a, *c]));
        }
    }
    for s in roots {
        if search.seen.insert(s.key(), 0).is_none() {
            search.visit(&s);
        }
    }
    let mut violations: Vec<_> = search.found.into_iter().collect();
    violations.sort();
    Report {
        states: search.seen.len(),
        transitions: search.transitions,
        completions: search.quiet.len(),
        depth: search.deepest,
        violations: violations.into_iter().map(|(_, (_, v))| v).collect(),
    }
}

fn run(b: Bounds) {
    let started = std::time::Instant::now();
    let report = explore(b);
    eprintln!(
        "hot-plug explorer {b:?}: {} distinct states, {} transitions, {} completions checked, depth {}, {:.1} s",
        report.states,
        report.transitions,
        report.completions,
        report.depth,
        started.elapsed().as_secs_f64()
    );
    assert!(
        report.violations.is_empty(),
        "{} violated invariant(s):\n{}",
        report.violations.len(),
        report.violations.join("\n")
    );
}

/// The state after one trace, which must be possible within the bounds, and the violations
/// after each of its steps.
fn replay(
    b: Bounds,
    root: impl FnOnce(&Daemon, &Bounds) -> State,
    trace: &[Step],
) -> (State, Vec<String>) {
    let d = super::tests::status_daemon();
    let mut s = root(&d, &b);
    let mut out = Vec::new();
    for (n, &step) in trace.iter().enumerate() {
        assert!(
            s.steps(&b).contains(&step),
            "step {} ({step:?}) is not possible",
            n + 1
        );
        s = s.apply(&d, step);
        for f in s.check(step, &b) {
            out.push(format!(
                "{}. {step:?}: {} [{}]: {}",
                n + 1,
                f.id,
                f.kind,
                f.message
            ));
        }
    }
    (s, out)
}

/// A sensor unplugged while a reopen discovers, before any watcher scan lists it gone: the
/// listing read after the open lacks it, so it is checked, not said to wait for the next engine
/// start, until its removal settles like any other. Without that listing the roster cannot tell
/// it from a sensor whose access went away for a moment: it says it waits, until the watcher
/// lists it gone.
#[test]
fn hotplug_explorer_a_sensor_unplugged_during_a_reopen_does_not_wait_for_the_next_start() {
    use Start::Absent;
    use Step::*;
    let attached = Device {
        generation: 1,
        ready: true,
        arrival: true,
    };
    let b = Bounds {
        serials: 1,
        depth: 8,
        plugs: 0,
        ticks: 0,
        opens: 1,
        flips: 0,
        scan_errors: 1,
    };
    for listed in [true, false] {
        let (s, found) = replay(
            b,
            |d, b| {
                let [_, running] = State::roots(d, b, [Some(attached), None]);
                running
            },
            &[
                Close,
                BeginOpen,
                Unplug(0),
                Snapshot,
                Discover([Absent; 2]),
                Listing(listed),
                Admit,
                Update,
            ],
        );
        assert!(found.is_empty(), "{}", found.join("\n"));
        let (state, claim) = if listed {
            ("checking_usb", None)
        } else {
            ("waiting_engine_start", Some(Claim::NextStart))
        };
        assert_eq!(s.pending(0).map(|p| p.0), Some(state), "listed: {listed}");
        assert_eq!(s.claims[0], claim, "listed: {listed}");
    }
}

#[test]
fn hotplug_explorer_one_sensor() {
    run(Bounds {
        serials: 1,
        depth: 13,
        plugs: 2,
        ticks: 3,
        opens: 2,
        flips: 1,
        scan_errors: 1,
    });
}

#[test]
fn hotplug_explorer_two_sensors() {
    run(Bounds {
        serials: 2,
        depth: 8,
        plugs: 1,
        ticks: 2,
        opens: 2,
        flips: 1,
        scan_errors: 1,
    });
}

#[test]
#[ignore = "about five minutes: cargo test --bin cv1touchd hotplug_explorer -- --ignored"]
fn hotplug_explorer_deeper() {
    run(Bounds {
        serials: 1,
        depth: 14,
        plugs: 3,
        ticks: 4,
        opens: 3,
        flips: 2,
        scan_errors: 1,
    });
    run(Bounds {
        serials: 2,
        depth: 10,
        plugs: 1,
        ticks: 2,
        opens: 2,
        flips: 1,
        scan_errors: 1,
    });
}
