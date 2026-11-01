// SPDX-License-Identifier: Apache-2.0
// Deterministic session-admission races. All images and storage are synthetic.
use super::*;
use std::cell::RefCell;
use std::sync::atomic::AtomicUsize;

thread_local! {
    static ENCODE_PAUSE: RefCell<Option<Box<dyn FnOnce()>>> = RefCell::new(None);
    static WRITE_PAUSE: RefCell<Option<Box<dyn FnOnce()>>> = RefCell::new(None);
}

pub(super) fn before_encode() {
    let pause = ENCODE_PAUSE.with(|slot| slot.borrow_mut().take());
    if let Some(pause) = pause {
        pause();
    }
}

pub(super) fn before_write_lock() {
    let pause = WRITE_PAUSE.with(|slot| slot.borrow_mut().take());
    if let Some(pause) = pause {
        pause();
    }
}

#[derive(Clone, Default)]
struct Storage {
    bytes: Arc<Mutex<Vec<u8>>>,
    syncs: Arc<AtomicUsize>,
    closes: Arc<AtomicUsize>,
}

struct Sink(Storage);
impl Write for Sink {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.0.bytes.lock().unwrap().extend_from_slice(bytes);
        Ok(bytes.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}
impl RecordingSink for Sink {
    fn sync_all(&mut self) -> io::Result<()> {
        self.0.syncs.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
    fn disk_space(&self) -> io::Result<DiskSpace> {
        Ok(DiskSpace {
            available: 100 << 30,
            capacity: 200 << 30,
        })
    }
}
impl Drop for Sink {
    fn drop(&mut self) {
        self.0.closes.fetch_add(1, Ordering::SeqCst);
    }
}

fn start(rec: &Recorder, name: &str, ts: u64, frames: bool) -> Storage {
    let storage = Storage::default();
    rec.start_with_sink(Path::new(name), ts, frames, Box::new(Sink(storage.clone())))
        .unwrap();
    assert!(rec.active());
    storage
}

fn finish(rec: &Recorder, storage: &Storage) -> Status {
    let status = rec.stop_timeout(None, Duration::from_secs(5)).unwrap();
    assert_eq!(status.state, State::Complete);
    assert!(status.finished);
    assert!(!status.pending);
    assert_eq!(status.stats.write_errors, 0);
    assert_eq!(status.stats.dropped, 0);
    assert!(!rec.active());
    assert_eq!(storage.syncs.load(Ordering::SeqCst), 1);
    assert_eq!(storage.closes.load(Ordering::SeqCst), 1);
    status
}

fn records(storage: &Storage) -> Vec<Record> {
    let bytes = storage.bytes.lock().unwrap();
    assert_eq!(&bytes[..8], MAGIC);
    let mut rest = &bytes[FILE_HEADER_LEN..];
    let mut out = Vec::new();
    while !rest.is_empty() {
        let len = u32::from_le_bytes(rest[4..8].try_into().unwrap()) as usize;
        out.push(Record {
            rtype: u16::from_le_bytes(rest[..2].try_into().unwrap()),
            host_ns: u64::from_le_bytes(rest[8..16].try_into().unwrap()),
            payload: rest[RECORD_HEADER_LEN..RECORD_HEADER_LEN + len].to_vec(),
        });
        rest = &rest[RECORD_HEADER_LEN + len..];
    }
    out
}

fn synthetic_image(rec: &Recorder, ts: u64) {
    rec.write_frame(
        &FrameInfo {
            frame_start_host_ns: ts,
            ..Default::default()
        },
        3,
        2,
        4,
        &[0, 8, 240, 255, 90, 2, 0, 255],
        2,
    );
}

fn pause() -> (
    Box<dyn FnOnce() + Send>,
    mpsc::Receiver<()>,
    mpsc::Sender<()>,
) {
    let (entered, ready) = mpsc::channel();
    let (resume, resumed) = mpsc::channel();
    (
        Box::new(move || {
            entered.send(()).unwrap();
            resumed.recv_timeout(Duration::from_secs(10)).unwrap();
        }),
        ready,
        resume,
    )
}

// Exercise the same admission, encoder and enqueue path that Daemon::image calls. The pause
// occurs after admission, before real encoding; neither stop nor replacement waits for it.
fn image_across_stop(replacement: Option<bool>) {
    let rec = Arc::new(Recorder::new(4096));
    let a = start(&rec, "synthetic-session", 100, true);
    rec.write(RAW_REPORT, 105, &[b"already-accepted-A"]);
    synthetic_image(&rec, 106);
    let (blocked, ready, resume) = pause();
    let producer_rec = Arc::clone(&rec);
    let producer = std::thread::spawn(move || {
        ENCODE_PAUSE.with(|slot| *slot.borrow_mut() = Some(blocked));
        synthetic_image(&producer_rec, 110);
    });
    ready.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(finish(&rec, &a).stats.records, 2);
    let a_before = a.bytes.lock().unwrap().clone();

    // Reuse both path and start timestamp deliberately: neither identifies a session.
    let b = replacement.map(|frames| start(&rec, "synthetic-session", 100, frames));
    if let Some(frames) = replacement {
        assert_eq!(rec.frames_enabled(), frames);
        if !frames {
            ENCODE_PAUSE.with(|slot| {
                *slot.borrow_mut() = Some(Box::new(|| panic!("frames disabled must not encode")));
            });
            synthetic_image(&rec, 120);
            ENCODE_PAUSE.with(|slot| assert!(slot.borrow_mut().take().is_some()));
            // The generic API also must enforce this session's no-frames policy.
            rec.write(RAW_FRAME, 120, &[b"synthetic-unpermitted-frame"]);
        }
    }
    resume.send(()).unwrap();
    producer.join().unwrap();
    if let Some(b) = b {
        if replacement == Some(true) {
            synthetic_image(&rec, 120);
        }
        let status = finish(&rec, &b);
        let b_records = records(&b);
        assert_eq!(status.stats.records, u64::from(replacement == Some(true)));
        if replacement == Some(true) {
            assert_eq!(b_records.len(), 1);
            assert_eq!(b_records[0].rtype, RAW_FRAME);
            assert_eq!(b_records[0].host_ns, 120);
            let (info, w, h, pixels) = decode_raw_frame(&b_records[0].payload).unwrap();
            assert_eq!((info.frame_start_host_ns, w, h), (120, 3, 2));
            assert_eq!(pixels, [0, 8, 240, 90, 0, 0]);
        } else {
            assert!(b_records.is_empty(), "stale A image entered no-frames B");
        }
    }
    assert!(!rec.active());
    assert_eq!(*a.bytes.lock().unwrap(), a_before);
    let a_records = records(&a);
    assert_eq!(a_records.len(), 2);
    assert_eq!(a_records[0].payload, b"already-accepted-A");
    assert_eq!((a_records[1].rtype, a_records[1].host_ns), (RAW_FRAME, 106));
}

#[test]
fn old_image_cannot_enter_replacement_without_frames() {
    image_across_stop(Some(false));
}

#[test]
fn old_image_cannot_enter_replacement_with_frames() {
    image_across_stop(Some(true));
}

#[test]
fn stop_without_restart_rejects_late_image() {
    image_across_stop(None);
}

#[test]
fn prelock_write_cannot_enter_replacement_or_stopped_session() {
    for replacement in [None, Some(false), Some(true)] {
        let rec = Arc::new(Recorder::new(4096));
        let a = start(&rec, "synthetic-session", 100, false);
        let (blocked, ready, resume) = pause();
        let producer_rec = Arc::clone(&rec);
        let producer = std::thread::spawn(move || {
            WRITE_PAUSE.with(|slot| *slot.borrow_mut() = Some(blocked));
            producer_rec.write(RAW_REPORT, 110, &[b"synthetic-old-report-from-A"]);
        });
        ready.recv_timeout(Duration::from_secs(5)).unwrap();
        finish(&rec, &a);
        let b = replacement.map(|frames| start(&rec, "synthetic-session", 100, frames));
        resume.send(()).unwrap();
        producer.join().unwrap();
        if let Some(b) = b {
            rec.write(RAW_REPORT, 120, &[b"fresh-B"]);
            assert_eq!(finish(&rec, &b).stats.records, 1);
            let b_records = records(&b);
            assert_eq!(b_records.len(), 1);
            assert_eq!(b_records[0].payload, b"fresh-B");
        }
        assert!(records(&a).is_empty());
        assert!(!rec.active());
    }
}

#[test]
fn request_stop_revokes_frame_permission_immediately() {
    let rec = Recorder::new(4096);
    let a = start(&rec, "synthetic-session", 100, true);
    assert!(rec.request_stop(None));
    assert!(!rec.frames_enabled());
    synthetic_image(&rec, 110);
    assert_eq!(finish(&rec, &a).stats.records, 0);
}

#[test]
fn cancelled_reservations_consume_generations_and_exhaustion_never_wraps() {
    let rec = Recorder::new(4096);
    let reservation = rec
        .reserve_with_frames(Path::new("synthetic-session"), true)
        .unwrap();
    let first = rec.shared.ring.lock().unwrap().admission;
    assert!(!rec.active());
    drop(reservation);
    let reservation = rec
        .reserve_with_frames(Path::new("synthetic-session"), false)
        .unwrap();
    let second = rec.shared.ring.lock().unwrap().admission;
    assert_ne!(first & !1, second & !1);
    drop(reservation);
    rec.shared.ring.lock().unwrap().admission = u64::MAX;
    assert!(rec
        .reserve_with_frames(Path::new("synthetic-session"), true)
        .is_err());
    assert!(!rec.has_session());
    assert!(!rec.active());
    assert_eq!(rec.shared.ring.lock().unwrap().admission, u64::MAX);
}

#[test]
fn frame_permission_tracks_off_on_off_on_sessions() {
    let rec = Recorder::new(4096);
    for frames in [false, true, false, true] {
        let storage = start(&rec, "synthetic-session", 100, frames);
        assert_eq!(rec.frames_enabled(), frames);
        synthetic_image(&rec, 110);
        assert_eq!(finish(&rec, &storage).stats.records, u64::from(frames));
        assert_eq!(records(&storage).len(), usize::from(frames));
        assert!(!rec.frames_enabled());
    }
}
