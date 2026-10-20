// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The radio thread only fills bounded, preallocated windows and exchanges ownership.
//! Extraction and correspondence searches run on a normal-priority worker started before
//! the daemon's radio thread raises its priority. Neither side holds a lock during analysis.

use super::{analyse, Landmark, Shift, COLLECTION_RADIUS_PX};
use crate::status::same_sensor;
use nalgebra::Vector2;
use std::collections::BTreeMap;
use std::io;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::{
    atomic::{AtomicBool, AtomicUsize, Ordering},
    Arc, Mutex,
};
use std::thread::{self, JoinHandle};

// All of a sensor's spots together yield about 625 samples/s, so a one-minute window
// holds about 38k samples and this cap is about 1.7 times that. Overflow is unknown,
// never a measurement from a truncated window. Two buffers cost 2 MiB per sensor.
const CAPACITY: usize = 65_536;

struct Work {
    generation: u64,
    sequence: u64,
    serial: Arc<str>,
    reference: Arc<[Vector2<f64>]>,
    samples: Vec<Vector2<f64>>,
    min_hits: usize,
}
struct Done {
    generation: u64,
    sequence: u64,
    samples: Vec<Vector2<f64>>,
    shift: Option<Shift>,
}
struct Ready {
    generation: u64,
    sequence: u64,
    shift: Option<Shift>,
}
#[derive(Default)]
struct Exchange {
    work: Option<Work>,
    done: Option<Done>,
}
struct Sensor {
    serial: Arc<str>,
    reference: Arc<[Vector2<f64>]>,
    exchange: Arc<Mutex<Exchange>>,
    generation: u64,
    sequence: u64,
    enabled: bool,
    start: Option<u64>,
    samples: Vec<Vector2<f64>>,
    spare: Option<Vec<Vector2<f64>>>,
    overflow: bool,
    in_flight: bool,
    ready: Option<Ready>,
    unknown: bool,
}

pub struct BackgroundMonitor {
    sensors: Vec<Sensor>,
    window_ns: u64,
    min_hits: usize,
    stop: Arc<AtomicBool>,
    /// Analyses that panicked and were contained, and whether the caller was told.
    faults: Arc<AtomicUsize>,
    fault_reported: bool,
    worker: Option<JoinHandle<()>>,
}

impl BackgroundMonitor {
    pub fn new(reference: Vec<Landmark>, window_ns: u64, min_hits: usize) -> io::Result<Self> {
        Self::start(reference, window_ns, min_hits, analyse)
    }

    /// The worker blocks inside analysis until `release` receives. The daemon's radio-exit test
    /// uses this to prove a failure is marked before `Drop` joins. Nothing else calls it.
    #[doc(hidden)]
    pub fn block_in_analysis_for_test(
        entered: std::sync::mpsc::Sender<()>,
        release: std::sync::mpsc::Receiver<()>,
    ) -> io::Result<Self> {
        let entered = Mutex::new(Some(entered));
        let release = Mutex::new(release);
        Self::start(
            vec![Landmark {
                serial: "A".into(),
                x: 0.0,
                y: 0.0,
                seen: 1.0,
            }],
            1,
            1,
            move |_, _, _, _| {
                if let Some(entered) = entered.lock().unwrap().take() {
                    let _ = entered.send(());
                }
                let _ = release.lock().unwrap().recv();
                None
            },
        )
    }

    fn start(
        reference: Vec<Landmark>,
        window_ns: u64,
        min_hits: usize,
        mut analysis: impl FnMut(&str, &[Vector2<f64>], &[Vector2<f64>], usize) -> Option<Shift>
            + Send
            + 'static,
    ) -> io::Result<Self> {
        let mut per = BTreeMap::<String, Vec<Vector2<f64>>>::new();
        for r in reference {
            let point = Vector2::new(r.x, r.y);
            if let Some((_, points)) = per.iter_mut().find(|(s, _)| same_sensor(s, &r.serial)) {
                points.push(point);
            } else {
                per.insert(r.serial, vec![point]);
            }
        }
        let sensors: Vec<_> = per
            .into_iter()
            .map(|(serial, reference)| Sensor {
                serial: serial.into(),
                reference: reference.into(),
                exchange: Arc::default(),
                generation: 0,
                sequence: 0,
                enabled: true,
                start: None,
                samples: Vec::with_capacity(CAPACITY),
                spare: Some(Vec::with_capacity(CAPACITY)),
                overflow: false,
                in_flight: false,
                ready: None,
                unknown: false,
            })
            .collect();
        let exchanges: Vec<_> = sensors.iter().map(|s| Arc::clone(&s.exchange)).collect();
        let stop = Arc::new(AtomicBool::new(false));
        let stopping = Arc::clone(&stop);
        let faults = Arc::new(AtomicUsize::new(0));
        let faulted = Arc::clone(&faults);
        let worker = thread::Builder::new()
            .name("cv1t-landmarks".into())
            .spawn(move || {
                while !stopping.load(Ordering::Acquire) {
                    let mut worked = false;
                    for exchange in &exchanges {
                        let work = exchange.lock().unwrap().work.take();
                        if let Some(work) = work {
                            worked = true;
                            // The check is advisory. A panic must leave this window unknown,
                            // not kill the one worker every sensor shares: its jobs would
                            // stay in flight and every later window would be unknown.
                            let shift = catch_unwind(AssertUnwindSafe(|| {
                                analysis(
                                    &work.serial,
                                    &work.reference,
                                    &work.samples,
                                    work.min_hits,
                                )
                            }))
                            .unwrap_or_else(|_| {
                                faulted.fetch_add(1, Ordering::Release);
                                None
                            });
                            exchange.lock().unwrap().done = Some(Done {
                                generation: work.generation,
                                sequence: work.sequence,
                                samples: work.samples,
                                shift,
                            });
                        }
                    }
                    // unpark retains a token if a producer raced this scan.
                    if !worked {
                        thread::park();
                    }
                }
            })?;
        Ok(Self {
            sensors,
            window_ns,
            min_hits,
            stop,
            faults,
            fault_reported: false,
            worker: Some(worker),
        })
    }

    /// True once, the first time an analysis panicked and was contained. The window
    /// itself is reported as unknown by `poll`; the panic text comes from the panic hook.
    pub fn take_first_fault(&mut self) -> bool {
        if self.fault_reported || self.faults.load(Ordering::Acquire) == 0 {
            return false;
        }
        self.fault_reported = true;
        true
    }

    /// No strings, collections or queue nodes are allocated on this path. Samples beyond
    /// the preallocated capacity invalidate the window, and are never passed to analysis.
    pub fn frame(
        &mut self,
        serial: &str,
        host_ns: u64,
        blobs: impl IntoIterator<Item = Vector2<f64>>,
    ) {
        let Some(s) = self
            .sensors
            .iter_mut()
            .find(|s| s.enabled && same_sensor(&s.serial, serial))
        else {
            return;
        };
        let start = *s.start.get_or_insert(host_ns);
        if host_ns < start {
            return;
        }
        for b in blobs {
            if s.reference
                .iter()
                .any(|r| (b - r).norm_squared() <= COLLECTION_RADIUS_PX.powi(2))
            {
                if s.samples.len() == CAPACITY {
                    s.overflow = true;
                } else {
                    s.samples.push(b);
                }
            }
        }
    }

    /// Close windows even when a camera sends no frames. One job at most is in flight
    /// per sensor. Busy/overflowed windows are unknown and supersede any older result.
    pub fn tick(&mut self, host_ns: u64) {
        for s in &mut self.sensors {
            if !s.enabled {
                continue;
            }
            let start = *s.start.get_or_insert(host_ns);
            if host_ns.saturating_sub(start) < self.window_ns {
                continue;
            }
            s.start = Some(host_ns);
            // A completed but unpublished window is also superseded. Its absence
            // must interrupt a pending alert streak even when the worker is idle.
            if s.ready
                .take()
                .is_some_and(|r| r.generation == s.generation && r.sequence == s.sequence)
            {
                s.unknown = true;
            }
            s.sequence += 1;
            let queued = if !s.in_flight && !s.overflow {
                if let Ok(mut exchange) = s.exchange.try_lock() {
                    let samples =
                        std::mem::replace(&mut s.samples, s.spare.take().expect("idle buffer"));
                    exchange.work = Some(Work {
                        generation: s.generation,
                        sequence: s.sequence,
                        serial: Arc::clone(&s.serial),
                        reference: Arc::clone(&s.reference),
                        samples,
                        min_hits: self.min_hits,
                    });
                    s.in_flight = true;
                    self.worker.as_ref().unwrap().thread().unpark();
                    true
                } else {
                    false
                }
            } else {
                false
            };
            if !queued {
                s.samples.clear();
                s.unknown = true;
            }
            s.overflow = false;
        }
    }

    /// Reclaim completed buffers without publishing their results. The caller can then
    /// advance overdue windows before accepting a result from an earlier sequence.
    /// Returns the number of buffers reclaimed; all exchanges remain nonblocking.
    pub fn recycle(&mut self) -> usize {
        let mut reclaimed = 0;
        for s in &mut self.sensors {
            let done = s.exchange.try_lock().ok().and_then(|mut e| e.done.take());
            if let Some(mut done) = done {
                done.samples.clear();
                s.spare = Some(done.samples);
                s.in_flight = false;
                s.ready = Some(Ready {
                    generation: done.generation,
                    sequence: done.sequence,
                    shift: done.shift,
                });
                reclaimed += 1;
            }
        }
        reclaimed
    }

    /// Nonblocking consumption, also called on radio passes without camera frames.
    pub fn poll(&mut self, mut visit: impl FnMut(&str, Option<Shift>)) {
        self.recycle();
        for s in &mut self.sensors {
            if s.unknown {
                s.unknown = false;
                visit(&s.serial, None);
            }
            if let Some(ready) = s.ready.take() {
                if s.enabled && ready.generation == s.generation && ready.sequence == s.sequence {
                    visit(&s.serial, ready.shift);
                }
            }
        }
    }

    /// Change the reference epoch. A job using the old pose may finish, but can never
    /// publish into this epoch. The daemon does not call this: after a pose change it only
    /// calls `forget`, and the sensor stays unwatched until the engine reopens.
    pub fn replace(&mut self, serial: &str, reference: Vec<Vector2<f64>>, host_ns: u64) {
        if let Some(s) = self
            .sensors
            .iter_mut()
            .find(|s| same_sensor(&s.serial, serial))
        {
            s.generation += 1;
            s.sequence += 1;
            s.reference = reference.into();
            s.enabled = true;
            s.start = Some(host_ns);
            s.samples.clear();
            s.ready = None;
            s.overflow = false;
            s.unknown = false;
        }
    }

    pub fn forget(&mut self, serial: &str) {
        if let Some(s) = self
            .sensors
            .iter_mut()
            .find(|s| same_sensor(&s.serial, serial))
        {
            s.generation += 1;
            s.enabled = false;
            s.samples.clear();
            s.ready = None;
            s.unknown = false;
        }
    }
}
impl Drop for BackgroundMonitor {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            worker.thread().unpark();
            let _ = worker.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::time::{Duration, Instant};

    fn reference() -> Vec<Landmark> {
        vec![Landmark {
            serial: "A".into(),
            x: 100.0,
            y: 300.0,
            seen: 0.1,
        }]
    }
    fn fill(m: &mut BackgroundMonitor, start: u64) {
        for k in 0..100 {
            m.frame("A", start + k * 10, [Vector2::new(101.5, 300.0)]);
        }
        m.tick(start + 1000);
    }
    fn drain(m: &mut BackgroundMonitor) -> Option<Shift> {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let mut result = None;
            m.poll(|_, s| result = Some(s));
            if let Some(s) = result {
                return s;
            }
            assert!(Instant::now() < deadline, "worker result timed out");
            thread::sleep(Duration::from_millis(1));
        }
    }
    #[test]
    fn mixed_case_background_aliases_share_one_sensor() {
        let mut refs = reference();
        let mut other = refs[0].clone();
        other.serial = "a".into();
        other.x = 500.0;
        refs.push(other);
        let m = BackgroundMonitor::new(refs, 1000, 20).unwrap();
        assert_eq!(m.sensors.len(), 1);
        assert_eq!(m.sensors[0].reference.len(), 2);
    }

    #[test]
    fn mixed_case_background_collection_replacement_and_forgetting() {
        let mut m = BackgroundMonitor::new(reference(), 1000, 20).unwrap();
        for k in 0..100 {
            m.frame("a", 1 + k * 10, [Vector2::new(101.5, 300.0)]);
        }
        m.tick(1001);
        assert!((drain(&mut m).unwrap().shift.x - 1.5).abs() < 1e-8);
        m.replace("a", vec![Vector2::new(200.0, 300.0)], 1001);
        assert_eq!(&*m.sensors[0].reference, &[Vector2::new(200.0, 300.0)]);
        m.forget("a");
        assert!(!m.sensors[0].enabled);
    }

    #[test]
    fn analysis_runs_off_the_collector_thread_and_results_need_no_new_frame() {
        let (tx, rx) = mpsc::sync_channel(1);
        let mut m = BackgroundMonitor::start(reference(), 1000, 20, move |s, r, b, n| {
            tx.send(thread::current().id()).unwrap();
            analyse(s, r, b, n)
        })
        .unwrap();
        fill(&mut m, 1);
        assert_ne!(
            rx.recv_timeout(Duration::from_secs(5)).unwrap(),
            thread::current().id()
        );
        assert!((drain(&mut m).unwrap().shift.x - 1.5).abs() < 1e-8);
    }

    // Release before monitor's Drop even on an assertion failure, so a failed test
    // never strands the worker on its deliberate gate.
    struct Release(Option<mpsc::SyncSender<()>>);
    impl Drop for Release {
        fn drop(&mut self) {
            if let Some(tx) = self.0.take() {
                let _ = tx.send(());
            }
        }
    }
    fn gated() -> (BackgroundMonitor, Release, mpsc::Receiver<()>) {
        let (release, gate) = mpsc::sync_channel(1);
        let (started, rx) = mpsc::sync_channel(1);
        let mut first = true;
        let m = BackgroundMonitor::start(reference(), 1000, 20, move |s, r, b, n| {
            if first {
                first = false;
                started.send(()).unwrap();
                gate.recv().unwrap();
            }
            analyse(s, r, b, n)
        })
        .unwrap();
        (m, Release(Some(release)), rx)
    }
    fn wait_idle(m: &mut BackgroundMonitor, mut visit: impl FnMut(&str, Option<Shift>)) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while m.sensors[0].in_flight {
            m.poll(&mut visit);
            assert!(Instant::now() < deadline, "worker stayed busy");
            thread::sleep(Duration::from_millis(1));
        }
    }
    #[test]
    fn a_busy_sensor_has_one_job_and_newer_unknown_supersedes_its_result() {
        let (mut m, release, started) = gated();
        fill(&mut m, 1);
        started.recv_timeout(Duration::from_secs(5)).unwrap();
        let ptr = m.sensors[0].samples.as_ptr();
        fill(&mut m, 1001);
        assert_eq!(m.sensors[0].samples.as_ptr(), ptr);
        assert_eq!(m.sensors[0].samples.capacity(), CAPACITY);
        assert!(m.sensors[0].exchange.try_lock().unwrap().work.is_none());
        let mut unknown = 0;
        m.poll(|_, s| {
            assert!(s.is_none());
            unknown += 1;
        });
        assert_eq!(unknown, 1);
        drop(release);
        wait_idle(&mut m, |_, s| panic!("stale result {s:?}"));
        fill(&mut m, 2001);
        assert!(drain(&mut m).is_some());
    }
    #[test]
    fn reference_changes_discard_an_in_flight_result() {
        for replace in [false, true] {
            let (mut m, release, started) = gated();
            fill(&mut m, 1);
            started.recv_timeout(Duration::from_secs(5)).unwrap();
            m.forget("A");
            if replace {
                m.replace("A", vec![Vector2::new(101.0, 300.0)], 1001);
            }
            drop(release);
            wait_idle(&mut m, |_, s| panic!("old reference result {s:?}"));
            if replace {
                fill(&mut m, 1001);
                assert!((drain(&mut m).unwrap().shift.x - 0.5).abs() < 1e-8);
            }
        }
    }
    #[test]
    fn a_panicking_analysis_leaves_its_window_unknown_and_the_worker_alive() {
        let mut two = reference();
        two.push(Landmark {
            serial: "B".into(),
            ..two[0].clone()
        });
        let mut m = BackgroundMonitor::start(two, 1000, 20, |s, r, b, n| {
            if s == "A" {
                panic!("injected analysis failure");
            }
            analyse(s, r, b, n)
        })
        .unwrap();
        let round = |m: &mut BackgroundMonitor, start: u64| {
            for k in 0..100 {
                for serial in ["A", "B"] {
                    m.frame(serial, start + k * 10, [Vector2::new(101.5, 300.0)]);
                }
            }
            m.tick(start + 1000);
            let deadline = Instant::now() + Duration::from_secs(5);
            let mut results = BTreeMap::new();
            while results.len() < 2 {
                m.poll(|serial, shift| {
                    results.insert(serial.to_owned(), shift);
                });
                assert!(Instant::now() < deadline, "worker result timed out");
                thread::sleep(Duration::from_millis(1));
            }
            results
        };
        for start in [1, 1001, 2001] {
            let results = round(&mut m, start);
            assert!(results["A"].is_none());
            assert!((results["B"].as_ref().unwrap().shift.x - 1.5).abs() < 1e-8);
            assert!(m.sensors.iter().all(|s| !s.in_flight));
        }
        assert!(m.take_first_fault());
        assert!(!m.take_first_fault());
    }
    #[test]
    fn collection_overflow_is_unknown_without_growing_or_analysing_a_prefix() {
        let mut m = BackgroundMonitor::start(reference(), 1000, 20, |_, _, _, _| {
            panic!("truncated analysis")
        })
        .unwrap();
        let ptr = m.sensors[0].samples.as_ptr();
        m.frame(
            "A",
            1,
            std::iter::repeat_n(Vector2::new(100.0, 300.0), CAPACITY + 1),
        );
        assert_eq!(m.sensors[0].samples.as_ptr(), ptr);
        assert_eq!(m.sensors[0].samples.capacity(), CAPACITY);
        m.tick(1001);
        let mut unknown = 0;
        m.poll(|_, s| {
            assert!(s.is_none());
            unknown += 1;
        });
        assert_eq!(unknown, 1);
        assert!(!m.sensors[0].in_flight);
    }
}
