// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

//! Shared-memory transport between cv1touchd (single writer) and readers (SteamVR driver, tools).
//! Layout: common/cv1touch_shm.h via bindgen. Hand slots and the sensor block are seqlock-protected;
//! `update_futex` is bumped and woken once per publish batch.

use std::ffi::CString;
use std::fmt;
use std::fs::{DirBuilder, File, OpenOptions};
use std::io;
use std::mem::size_of;
use std::os::fd::{AsRawFd, FromRawFd};
use std::os::unix::fs::{DirBuilderExt, MetadataExt, OpenOptionsExt};
use std::path::Path;
use std::ptr::{self, addr_of_mut};
use std::sync::atomic::{fence, AtomicU32, AtomicU64, Ordering};
use std::sync::Mutex;

use crate::ffi;

pub use ffi::cv1t_alignment as Alignment;
pub use ffi::cv1t_controller_state as ControllerState;
pub use ffi::cv1t_sensor_status as SensorStatus;

pub const DEFAULT_NAME: &str = "/cv1touch";
pub const HAND_LEFT: usize = ffi::CV1T_HAND_LEFT as usize;
pub const HAND_RIGHT: usize = ffi::CV1T_HAND_RIGHT as usize;
/// Fixed legacy diagnostics capacity, not an acquisition or tracking limit.
pub const MAX_SENSORS: usize = ffi::CV1T_MAX_SENSORS as usize;
const READ_TRIES: usize = 1000;

/// CLOCK_MONOTONIC in nanoseconds: the clock every cv1touch timestamp uses.
pub fn now_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: valid out-pointer.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

pub fn set_serial(s: &mut SensorStatus, value: &str) {
    s.serial = [0; 32];
    for (dst, b) in s.serial.iter_mut().zip(value.bytes().take(31)) {
        *dst = b as libc::c_char;
    }
}

pub fn serial_str(s: &SensorStatus) -> String {
    let bytes: Vec<u8> = s
        .serial
        .iter()
        .take_while(|&&c| c != 0)
        .map(|&c| c as u8)
        .collect();
    String::from_utf8_lossy(&bytes).into_owned()
}

fn cname(name: &str) -> io::Result<CString> {
    CString::new(name)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "shm name contains NUL"))
}

/// SAFETY: `p` must point to an aligned u32 inside a mapping that outlives the returned reference.
unsafe fn atomic_u32<'a>(p: *mut u32) -> &'a AtomicU32 {
    AtomicU32::from_ptr(p)
}

/// SAFETY: as `atomic_u32`.
unsafe fn atomic_u64<'a>(p: *mut u64) -> &'a AtomicU64 {
    AtomicU64::from_ptr(p)
}

/// SAFETY: `seq` and `data` must point into a live mapping.
unsafe fn seq_write_begin(seq: *mut u32) {
    let seq = atomic_u32(seq);
    seq.store(
        seq.load(Ordering::Relaxed).wrapping_add(1),
        Ordering::Relaxed,
    );
    fence(Ordering::SeqCst);
}

/// SAFETY: as `seq_write_begin`.
unsafe fn seq_write_end(seq: *mut u32) {
    fence(Ordering::SeqCst);
    let seq = atomic_u32(seq);
    seq.store(
        seq.load(Ordering::Relaxed).wrapping_add(1),
        Ordering::Relaxed,
    );
}

/// Consistent copy of a seqlock-protected value, or None if the writer kept it busy.
/// SAFETY: as `seq_write_begin`.
unsafe fn seq_read<T: Copy>(seq: *mut u32, data: *const T) -> Option<T> {
    let seq = atomic_u32(seq);
    for _ in 0..READ_TRIES {
        let s1 = seq.load(Ordering::Acquire);
        if s1 & 1 != 0 {
            std::hint::spin_loop();
            continue;
        }
        let value = ptr::read_volatile(data);
        fence(Ordering::SeqCst);
        if seq.load(Ordering::Relaxed) == s1 {
            return Some(value);
        }
    }
    None
}

/// A mapped segment; the owner (writer or reader) keeps the mapping alive.
#[derive(Clone, Copy)]
struct Segment(*mut ffi::cv1t_shm);

// SAFETY: all shared access goes through atomics or the seqlock protocol.
unsafe impl Send for Segment {}
unsafe impl Sync for Segment {}

impl Segment {
    fn hand(&self, hand: usize) -> Option<ControllerState> {
        assert!(hand < 2, "hand index");
        // SAFETY: mapping is live; fields are naturally aligned.
        unsafe {
            let slot = addr_of_mut!((*self.0).hands[hand]);
            seq_read(addr_of_mut!((*slot).seq), addr_of_mut!((*slot).state))
        }
    }

    fn sensors(&self) -> Option<Vec<SensorStatus>> {
        // SAFETY: mapping is live.
        unsafe {
            let s = self.0;
            let seq = atomic_u32(addr_of_mut!((*s).sensor_seq));
            for _ in 0..READ_TRIES {
                let s1 = seq.load(Ordering::Acquire);
                if s1 & 1 != 0 {
                    std::hint::spin_loop();
                    continue;
                }
                let count = ptr::read_volatile(addr_of_mut!((*s).sensor_count)) as usize;
                let all = ptr::read_volatile(addr_of_mut!((*s).sensors));
                fence(Ordering::SeqCst);
                if seq.load(Ordering::Relaxed) == s1 {
                    return Some(all[..count.min(MAX_SENSORS)].to_vec());
                }
            }
            None
        }
    }

    fn alignment(&self) -> Option<Alignment> {
        // SAFETY: mapping is live; fields are naturally aligned.
        unsafe {
            seq_read(
                addr_of_mut!((*self.0).alignment_seq),
                addr_of_mut!((*self.0).alignment),
            )
        }
    }

    fn magic(&self) -> u32 {
        unsafe { atomic_u32(addr_of_mut!((*self.0).magic)).load(Ordering::Acquire) }
    }

    fn generation(&self) -> u64 {
        unsafe { ptr::read_volatile(addr_of_mut!((*self.0).generation)) }
    }

    fn heartbeat(&self) -> u64 {
        unsafe { atomic_u64(addr_of_mut!((*self.0).heartbeat_host_ns)).load(Ordering::Acquire) }
    }

    fn update_counter(&self) -> u32 {
        unsafe { atomic_u32(addr_of_mut!((*self.0).update_futex)).load(Ordering::Acquire) }
    }
}

// Use the same mount namespace as POSIX shared memory: runtime-dir and XDG overrides
// must not let two writers own one segment. Never unlink a lock file (that splits locks).
fn private_lock_directory(path: &Path) -> io::Result<File> {
    match DirBuilder::new().mode(0o700).create(path) {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {}
        Err(e) => return Err(e),
    }
    let dir = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(path)?;
    let meta = dir.metadata()?;
    // SAFETY: geteuid has no arguments and cannot fail.
    if !meta.is_dir() || meta.uid() != unsafe { libc::geteuid() } || meta.mode() & 0o777 != 0o700 {
        return Err(io::Error::new(
            io::ErrorKind::PermissionDenied,
            "shared-memory lock directory must be owned by this user with mode 0700",
        ));
    }
    Ok(dir)
}

fn lock_shm_name(name: &str) -> io::Result<File> {
    let component = name
        .strip_prefix('/')
        .filter(|s| !s.is_empty() && !s.contains('/') && *s != "." && *s != "..")
        .ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "shared-memory name must be / followed by a single name",
            )
        })?;
    let component = cname(component)?;
    // SAFETY: geteuid has no arguments and cannot fail.
    let uid = unsafe { libc::geteuid() };
    let dir = private_lock_directory(Path::new(&format!("/dev/shm/cv1touch-locks-{uid}")))?;
    // SAFETY: live directory fd and NUL-terminated single component; no symlinks followed.
    let fd = unsafe {
        libc::openat(
            dir.as_raw_fd(),
            component.as_ptr(),
            libc::O_CREAT | libc::O_RDWR | libc::O_CLOEXEC | libc::O_NOFOLLOW,
            0o600,
        )
    };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: fresh owned descriptor, closed on every error/return through File.
    let file = unsafe { File::from_raw_fd(fd) };
    let meta = file.metadata()?;
    if !meta.is_file() || meta.uid() != uid || meta.mode() & 0o777 != 0o600 || meta.nlink() != 1 {
        return Err(io::Error::new(
            io::ErrorKind::PermissionDenied,
            "invalid shared-memory ownership lock",
        ));
    }
    // SAFETY: valid fd, held for the complete ShmWriter lifetime including unlink on Drop.
    if unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(file)
}

pub struct ShmWriter {
    seg: Segment,
    name: CString,
    _ownership: File,
    // Alignment reloads and the universe watcher can publish concurrently.
    alignment_writer: Mutex<()>,
}

impl ShmWriter {
    /// Locks this SHM name across processes, replaces any stale segment, and creates a fresh one.
    pub fn create(name: &str, generation: u64) -> io::Result<Self> {
        let ownership = lock_shm_name(name)?;
        let name = cname(name)?;
        let len = size_of::<ffi::cv1t_shm>();
        // SAFETY: plain POSIX calls; the new mapping is exclusively ours until magic is published.
        unsafe {
            libc::shm_unlink(name.as_ptr());
            let fd = libc::shm_open(
                name.as_ptr(),
                libc::O_CREAT | libc::O_EXCL | libc::O_RDWR | libc::O_CLOEXEC,
                0o600,
            );
            if fd < 0 {
                return Err(io::Error::last_os_error());
            }
            if libc::ftruncate(fd, len as libc::off_t) < 0 {
                let e = io::Error::last_os_error();
                libc::close(fd);
                libc::shm_unlink(name.as_ptr());
                return Err(e);
            }
            let p = libc::mmap(
                ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            );
            libc::close(fd);
            if p == libc::MAP_FAILED {
                let e = io::Error::last_os_error();
                libc::shm_unlink(name.as_ptr());
                return Err(e);
            }
            let s = p as *mut ffi::cv1t_shm;
            (*s).version = ffi::CV1T_SHM_VERSION;
            (*s).size = len as u32;
            (*s).generation = generation;
            (*s).daemon_pid = std::process::id();
            (*s).heartbeat_host_ns = now_ns();
            // Publish the magic last: readers trust nothing else until they see it.
            atomic_u32(addr_of_mut!((*s).magic)).store(ffi::CV1T_SHM_MAGIC, Ordering::Release);
            Ok(Self {
                seg: Segment(s),
                name,
                _ownership: ownership,
                alignment_writer: Mutex::new(()),
            })
        }
    }

    pub fn publish_hand(&self, hand: usize, state: &ControllerState) {
        assert!(hand < 2, "hand index");
        // SAFETY: single writer; mapping is live.
        unsafe {
            let slot = addr_of_mut!((*self.seg.0).hands[hand]);
            seq_write_begin(addr_of_mut!((*slot).seq));
            ptr::write_volatile(addr_of_mut!((*slot).state), *state);
            seq_write_end(addr_of_mut!((*slot).seq));
        }
    }

    /// Bumps the futex word and wakes waiting readers; call once per publish batch.
    pub fn signal(&self) {
        // SAFETY: mapping is live; FUTEX_WAKE on a shared mapping.
        unsafe {
            let word = addr_of_mut!((*self.seg.0).update_futex);
            atomic_u32(word).fetch_add(1, Ordering::Release);
            libc::syscall(
                libc::SYS_futex,
                word,
                libc::FUTEX_WAKE,
                i32::MAX,
                ptr::null::<libc::timespec>(),
                ptr::null::<u32>(),
                0,
            );
        }
    }

    pub fn publish_sensors(&self, sensors: &[SensorStatus]) {
        let n = sensors.len().min(MAX_SENSORS);
        let mut all = [SensorStatus::default(); MAX_SENSORS];
        all[..n].copy_from_slice(&sensors[..n]);
        // SAFETY: single writer; mapping is live.
        unsafe {
            let s = self.seg.0;
            seq_write_begin(addr_of_mut!((*s).sensor_seq));
            ptr::write_volatile(addr_of_mut!((*s).sensors), all);
            ptr::write_volatile(addr_of_mut!((*s).sensor_count), n as u32);
            seq_write_end(addr_of_mut!((*s).sensor_seq));
        }
    }

    pub fn publish_alignment(&self, a: &Alignment) {
        let _writer = self.alignment_writer.lock().unwrap();
        // SAFETY: serialized writers; mapping is live.
        unsafe {
            let s = self.seg.0;
            seq_write_begin(addr_of_mut!((*s).alignment_seq));
            ptr::write_volatile(addr_of_mut!((*s).alignment), *a);
            seq_write_end(addr_of_mut!((*s).alignment_seq));
        }
    }

    pub fn alignment(&self) -> Option<Alignment> {
        self.seg.alignment()
    }

    pub fn heartbeat(&self, now_ns: u64) {
        unsafe {
            atomic_u64(addr_of_mut!((*self.seg.0).heartbeat_host_ns))
                .store(now_ns, Ordering::Release)
        }
    }

    pub fn generation(&self) -> u64 {
        self.seg.generation()
    }

    pub fn hand(&self, hand: usize) -> Option<ControllerState> {
        self.seg.hand(hand)
    }

    pub fn sensors(&self) -> Option<Vec<SensorStatus>> {
        self.seg.sensors()
    }
}

impl Drop for ShmWriter {
    /// Zeroes magic (readers holding the mapping see the daemon is gone), unmaps and unlinks.
    fn drop(&mut self) {
        // SAFETY: we own the mapping.
        unsafe {
            atomic_u32(addr_of_mut!((*self.seg.0).magic)).store(0, Ordering::Release);
            libc::munmap(self.seg.0 as *mut libc::c_void, size_of::<ffi::cv1t_shm>());
            libc::shm_unlink(self.name.as_ptr());
        }
    }
}

#[derive(Debug)]
pub enum ShmError {
    NotRunning,
    Abi { version: u32, size: u32 },
    Io(io::Error),
}

impl fmt::Display for ShmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ShmError::NotRunning => write!(f, "cv1touchd is not running"),
            ShmError::Abi { version, size } => write!(
                f,
                "shared-memory ABI mismatch: version {version} size {size}, expected {} {}",
                ffi::CV1T_SHM_VERSION,
                size_of::<ffi::cv1t_shm>()
            ),
            ShmError::Io(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for ShmError {}

pub struct ShmReader {
    seg: Segment,
    generation: u64,
}

impl ShmReader {
    pub fn open(name: &str) -> Result<Self, ShmError> {
        let cname = cname(name).map_err(ShmError::Io)?;
        let len = size_of::<ffi::cv1t_shm>();
        // SAFETY: read-only mapping of the daemon's segment; validated before use.
        unsafe {
            let fd = libc::shm_open(cname.as_ptr(), libc::O_RDONLY | libc::O_CLOEXEC, 0);
            if fd < 0 {
                let e = io::Error::last_os_error();
                return Err(if e.kind() == io::ErrorKind::NotFound {
                    ShmError::NotRunning
                } else {
                    ShmError::Io(e)
                });
            }
            // A segment that is not (yet) fully sized would SIGBUS on first access: the daemon
            // creates it, then sizes it, then publishes the magic.
            let mut st: libc::stat = std::mem::zeroed();
            if libc::fstat(fd, &mut st) < 0 || (st.st_size as usize) < len {
                libc::close(fd);
                return Err(ShmError::NotRunning);
            }
            let p = libc::mmap(
                ptr::null_mut(),
                len,
                libc::PROT_READ,
                libc::MAP_SHARED,
                fd,
                0,
            );
            libc::close(fd);
            if p == libc::MAP_FAILED {
                return Err(ShmError::Io(io::Error::last_os_error()));
            }
            let mut reader = Self {
                seg: Segment(p as *mut ffi::cv1t_shm),
                generation: 0,
            };
            if reader.seg.magic() != ffi::CV1T_SHM_MAGIC {
                return Err(ShmError::NotRunning);
            }
            let version = ptr::read_volatile(addr_of_mut!((*reader.seg.0).version));
            let size = ptr::read_volatile(addr_of_mut!((*reader.seg.0).size));
            if version != ffi::CV1T_SHM_VERSION || size as usize != len {
                return Err(ShmError::Abi { version, size });
            }
            reader.generation = reader.seg.generation();
            Ok(reader)
        }
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn hand(&self, hand: usize) -> Option<ControllerState> {
        self.seg.hand(hand)
    }

    pub fn sensors(&self) -> Option<Vec<SensorStatus>> {
        self.seg.sensors()
    }

    pub fn alignment(&self) -> Option<Alignment> {
        self.seg.alignment()
    }

    pub fn heartbeat_ns(&self) -> u64 {
        self.seg.heartbeat()
    }

    pub fn update_counter(&self) -> u32 {
        self.seg.update_counter()
    }

    /// True when the daemon exited (magic zeroed) or stopped heartbeating (crash or restart: a
    /// restarted daemon creates a fresh segment, so re-open by name).
    pub fn stale(&self, now_ns: u64, max_age_ns: u64) -> bool {
        self.seg.magic() != ffi::CV1T_SHM_MAGIC
            || self.seg.generation() != self.generation
            || now_ns.saturating_sub(self.seg.heartbeat()) > max_age_ns
    }

    /// Blocks until the update counter differs from `last_seen` or the timeout passes.
    pub fn wait(&self, last_seen: u32, timeout_ns: u64) -> u32 {
        let ts = libc::timespec {
            tv_sec: (timeout_ns / 1_000_000_000) as libc::time_t,
            tv_nsec: (timeout_ns % 1_000_000_000) as libc::c_long,
        };
        // SAFETY: FUTEX_WAIT on the mapped futex word.
        unsafe {
            libc::syscall(
                libc::SYS_futex,
                addr_of_mut!((*self.seg.0).update_futex),
                libc::FUTEX_WAIT,
                last_seen,
                &ts as *const libc::timespec,
                ptr::null::<u32>(),
                0,
            );
        }
        self.seg.update_counter()
    }
}

impl Drop for ShmReader {
    fn drop(&mut self) {
        unsafe { libc::munmap(self.seg.0 as *mut libc::c_void, size_of::<ffi::cv1t_shm>()) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;
    use std::time::Duration;

    fn test_name(tag: &str) -> String {
        format!("/cv1touch-test-{}-{tag}", std::process::id())
    }

    fn filled(k: u32) -> ControllerState {
        let f = k as f32;
        ControllerState {
            sample_host_ns: k as u64,
            received_host_ns: k as u64,
            publish_host_ns: k as u64,
            last_position_fix_host_ns: k as u64,
            device_time_ns: k as u64,
            orient: [f; 4],
            pos: [f; 3],
            raw_orient: [f; 4],
            raw_pos: [f; 3],
            lin_vel: [f; 3],
            ang_vel: [f; 3],
            lin_acc: [f; 3],
            gyro_raw: [f; 3],
            gyro_bias: [f; 3],
            imu_accel: [f; 3],
            imu_gyro: [f; 3],
            pos_stddev: [f; 3],
            rot_stddev: [f; 3],
            trigger: f,
            grip: f,
            stick: [f; 2],
            cap: [f; 5],
            flags: k,
            buttons: k,
            touches: k,
            battery_percent: k as i32,
        }
    }

    fn bytes<T>(v: &T) -> &[u8] {
        // SAFETY: plain-old-data test structs.
        unsafe { std::slice::from_raw_parts(v as *const T as *const u8, size_of::<T>()) }
    }

    #[test]
    fn abi_sizes() {
        assert_eq!(size_of::<ControllerState>(), 256);
        assert_eq!(size_of::<ffi::cv1t_slot>(), 264);
        assert_eq!(size_of::<SensorStatus>(), 80);
        assert_eq!(size_of::<Alignment>(), 104);
        assert_eq!(size_of::<ffi::cv1t_shm>(), 1328);
        assert_eq!(ffi::CV1T_SHM_VERSION, 3);
    }

    #[test]
    fn concurrent_alignment_publishers_never_mix_snapshots() {
        let name = test_name("alignment-writers");
        let w = Arc::new(ShmWriter::create(&name, 1).unwrap());
        let r = ShmReader::open(&name).unwrap();
        let gate = Arc::new(std::sync::Barrier::new(3));
        let writers: Vec<_> = (1..=2u32)
            .map(|k| {
                let w = Arc::clone(&w);
                let gate = Arc::clone(&gate);
                thread::spawn(move || {
                    let v = Alignment {
                        universe_id: k as u64,
                        valid: k,
                        lighthouse_orient: [k as f32; 4],
                        lighthouse_pos: [k as f32; 3],
                        render_orient: [[k as f32; 4]; 2],
                        render_pos: [[k as f32; 3]; 2],
                        ..Default::default()
                    };
                    gate.wait();
                    for _ in 0..200_000 {
                        w.publish_alignment(&v);
                    }
                })
            })
            .collect();
        gate.wait();
        let mut torn = 0;
        for _ in 0..200_000 {
            if let Some(v) = r.alignment() {
                let k = v.valid;
                if v.universe_id != k as u64
                    || v.lighthouse_orient != [k as f32; 4]
                    || v.lighthouse_pos != [k as f32; 3]
                    || v.render_orient != [[k as f32; 4]; 2]
                    || v.render_pos != [[k as f32; 3]; 2]
                {
                    torn += 1;
                }
            }
        }
        for writer in writers {
            writer.join().unwrap();
        }
        assert_eq!(torn, 0, "readers accepted mixed alignment snapshots");
        assert!(r.alignment().is_some(), "writers left the seqlock busy");
    }

    #[test]
    fn ownership_directory_rejects_symlinks_and_shared_permissions() {
        use std::os::unix::fs::{symlink, PermissionsExt};
        let dir = std::env::temp_dir().join(format!("cv1t-lockdir-{}", std::process::id()));
        DirBuilder::new().mode(0o700).create(&dir).unwrap();
        let target = dir.join("target");
        let link = dir.join("link");
        DirBuilder::new().mode(0o700).create(&target).unwrap();
        symlink(&target, &link).unwrap();
        assert!(
            private_lock_directory(&link).is_err(),
            "followed a lock-directory symlink"
        );
        std::fs::set_permissions(&target, std::fs::Permissions::from_mode(0o755)).unwrap();
        assert!(
            private_lock_directory(&target).is_err(),
            "accepted a shared lock directory"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_second_writer_cannot_replace_the_owned_segment() {
        let name = test_name("exclusive-writer");
        let first = ShmWriter::create(&name, 41).unwrap();
        first.publish_hand(HAND_LEFT, &filled(19));
        assert!(
            ShmWriter::create(&name, 42).is_err(),
            "a second writer replaced a live segment"
        );
        let reader = ShmReader::open(&name).unwrap();
        assert_eq!(reader.generation(), 41);
        assert_eq!(reader.hand(HAND_LEFT).unwrap().flags, 19);
        drop(first);
        let second = ShmWriter::create(&name, 42).unwrap();
        assert_eq!(ShmReader::open(&name).unwrap().generation(), 42);
        drop(second);
    }

    #[test]
    fn alignment_roundtrip() {
        let name = test_name("alignment");
        let w = ShmWriter::create(&name, 7).unwrap();
        let r = ShmReader::open(&name).unwrap();
        assert_eq!(r.alignment().unwrap().valid, 0);
        let a = Alignment {
            universe_id: 1_234_567_890_123,
            valid: ffi::CV1T_ALIGN_LIGHTHOUSE | ffi::CV1T_ALIGN_RENDER_RIGHT,
            lighthouse_orient: [
                0.0,
                std::f32::consts::FRAC_1_SQRT_2,
                0.0,
                std::f32::consts::FRAC_1_SQRT_2,
            ],
            lighthouse_pos: [1.0, 2.0, 3.0],
            render_orient: [[0.0, 0.0, 0.0, 1.0], [0.1, 0.0, 0.0, 0.995]],
            render_pos: [[0.0; 3], [0.01, -0.02, 0.03]],
            ..Default::default()
        };
        w.publish_alignment(&a);
        let got = r.alignment().unwrap();
        assert_eq!(got.universe_id, a.universe_id);
        assert_eq!(got.valid, a.valid);
        assert_eq!(got.lighthouse_pos, a.lighthouse_pos);
        assert_eq!(got.render_pos, a.render_pos);
        assert_eq!(w.alignment().unwrap().render_orient, a.render_orient);
    }

    #[test]
    fn roundtrip_and_header() {
        let name = test_name("roundtrip");
        let w = ShmWriter::create(&name, 0x1234).unwrap();
        let r = ShmReader::open(&name).unwrap();
        assert_eq!(r.generation(), 0x1234);
        let s = filled(77);
        w.publish_hand(HAND_RIGHT, &s);
        assert_eq!(bytes(&r.hand(HAND_RIGHT).unwrap()), bytes(&s));

        let mut sensors = [SensorStatus::default(); 2];
        set_serial(&mut sensors[0], "WMTD000000001");
        sensors[0].placed = 1;
        set_serial(&mut sensors[1], "WMTD000000004");
        w.publish_sensors(&sensors);
        let got = r.sensors().unwrap();
        assert_eq!(got.len(), 2);
        assert_eq!(serial_str(&got[1]), "WMTD000000004");
        assert_eq!(got[0].placed, 1);

        w.heartbeat(999);
        assert_eq!(r.heartbeat_ns(), 999);
        assert!(!r.stale(1000, 500_000_000));
        assert!(r.stale(999 + 600_000_000, 500_000_000));
    }

    #[test]
    fn torn_reads_impossible() {
        let name = test_name("torn");
        let w = Arc::new(ShmWriter::create(&name, 1).unwrap());
        let r = ShmReader::open(&name).unwrap();
        let writer = {
            let w = Arc::clone(&w);
            thread::spawn(move || {
                for k in 1..=200_000u32 {
                    w.publish_hand(HAND_LEFT, &filled(k));
                }
            })
        };
        let mut ok = 0;
        for _ in 0..200_000 {
            if let Some(s) = r.hand(HAND_LEFT) {
                assert_eq!(bytes(&s), bytes(&filled(s.flags)));
                ok += 1;
            }
        }
        writer.join().unwrap();
        assert!(ok > 0);
    }

    #[test]
    fn futex_wakes_reader() {
        let name = test_name("futex");
        let w = ShmWriter::create(&name, 1).unwrap();
        let r = ShmReader::open(&name).unwrap();
        let start = r.update_counter();
        let waiter = thread::spawn(move || {
            let got = r.wait(start, 2_000_000_000);
            (got, now_ns())
        });
        thread::sleep(Duration::from_millis(20));
        let signalled = now_ns();
        w.signal();
        let (got, woke) = waiter.join().unwrap();
        assert_ne!(got, start);
        // The counter only changes in signal(), so the waiter cannot return a new value earlier; it
        // must wake promptly (a missed wake would sit out the 2 s timeout). Measuring from the
        // waiter's own start made this flaky when the thread started late under load.
        assert!(
            woke >= signalled && woke - signalled < 1_000_000_000,
            "woke {} ns after signal",
            woke as i64 - signalled as i64
        );
    }

    /// A reader racing segment creation (created but not yet sized) must not die with SIGBUS:
    /// this reader is the reference for the SteamVR driver inside vrserver.
    #[test]
    fn open_of_unsized_segment_is_not_running() {
        let name = test_name("unsized");
        let cname = cname(&name).unwrap();
        // SAFETY: plain POSIX calls on a test-only segment.
        unsafe {
            libc::shm_unlink(cname.as_ptr());
            let fd = libc::shm_open(
                cname.as_ptr(),
                libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
                0o600,
            );
            assert!(fd >= 0);
            libc::close(fd);
        }
        assert!(matches!(ShmReader::open(&name), Err(ShmError::NotRunning)));
        unsafe { libc::shm_unlink(cname.as_ptr()) };
    }

    /// Review Focus 2: a reader holding the old mapping must see the daemon go away and a restart.
    #[test]
    fn close_zeroes_magic_and_new_generation_is_detected() {
        let name = test_name("restart");
        let w = ShmWriter::create(&name, 5).unwrap();
        let r = ShmReader::open(&name).unwrap();
        assert!(!r.stale(now_ns(), 500_000_000));
        drop(w);
        assert!(r.stale(now_ns(), 500_000_000));
        assert!(matches!(ShmReader::open(&name), Err(ShmError::NotRunning)));
        let w2 = ShmWriter::create(&name, 6).unwrap();
        assert!(r.stale(now_ns(), 500_000_000));
        assert_eq!(ShmReader::open(&name).unwrap().generation(), 6);
        drop(w2);
    }
}
