//! Startup settings use disposable files; FIFO checks run in a bounded child.
use cv1touch::settings::{load, save, Settings};
use std::path::PathBuf;

struct Fixture(PathBuf);
impl Fixture {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir().join(format!(
            "cv1touch-settings-load-{}-{name}",
            std::process::id()
        ));
        std::fs::create_dir(&dir).unwrap();
        Self(dir)
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[test]
fn regular_files_defaults_and_byte_limit() {
    let dir = Fixture::new("regular");
    let path = dir.0.join("settings.json");
    assert_eq!(load(&path), Ok(Settings::default()));
    let settings = Settings {
        prediction_percent: 42,
        rotation_prediction_percent: Some(13),
        latency_adjustment_ms: Some(-3.0),
        tilt_deg: Some(7.5),
        ..Settings::default()
    };
    save(&path, &settings).unwrap();
    assert_eq!(load(&path), Ok(settings.clone()));
    let json = std::fs::read(&path).unwrap();
    assert!(json.len() < 1024, "normal serializer remains small");
    for size in [1024 * 1024 - 1, 1024 * 1024, 1024 * 1024 + 1] {
        let mut bytes = vec![b' '; size - json.len()];
        bytes.extend_from_slice(&json);
        std::fs::write(&path, bytes).unwrap();
        if size <= 1024 * 1024 {
            assert_eq!(load(&path), Ok(settings.clone()));
        } else {
            assert!(load(&path).unwrap_err().contains("byte limit"));
        }
    }
    // A large sparse file must be rejected by size, without allocating its contents.
    let oversized = std::fs::OpenOptions::new().write(true).open(&path).unwrap();
    oversized.set_len(64 * 1024 * 1024).unwrap();
    drop(oversized);
    assert!(load(&path).unwrap_err().contains("byte limit"));
    for bytes in [&b"not json"[..], &b"\xff"[..]] {
        std::fs::write(&path, bytes).unwrap();
        assert!(load(&path).is_err());
    }
    // Ignored fields must not bypass the original whole-file UTF-8 requirement.
    let mut invalid_utf8 = json;
    invalid_utf8.pop(); // closing brace from the serializer
    invalid_utf8.extend_from_slice(b",\"unknown\":\"\xff\"}");
    std::fs::write(&path, invalid_utf8).unwrap();
    assert!(load(&path).is_err());
    assert!(load(&dir.0).is_err(), "directory is not a regular file");
}

#[cfg(unix)]
#[test]
fn symlinks_to_regular_files_still_load() {
    let dir = Fixture::new("symlink");
    let path = dir.0.join("settings.json");
    let link = dir.0.join("link.json");
    let settings = Settings {
        prediction_percent: 42,
        ..Settings::default()
    };
    save(&path, &settings).unwrap();
    std::os::unix::fs::symlink(&path, &link).unwrap();
    assert_eq!(load(&link), Ok(settings));
}

#[cfg(unix)]
#[test]
fn fifo_returns_error_without_waiting_for_a_writer() {
    use std::process::{Child, Command};
    use std::time::{Duration, Instant};
    const CHILD_PATH: &str = "CV1TOUCH_SETTINGS_FIFO_TEST_PATH";
    if let Some(path) = std::env::var_os(CHILD_PATH) {
        let error = load(std::path::Path::new(&path)).unwrap_err();
        assert!(error.contains("regular file"), "{error}");
        return;
    }
    // Kill/reap on assertion or I/O failure as well as on the deadline.
    struct Guard(Child);
    impl Drop for Guard {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
    let dir = Fixture::new("fifo");
    let path = dir.0.join("settings.json");
    use std::os::unix::ffi::OsStrExt;
    let cpath = std::ffi::CString::new(path.as_os_str().as_bytes()).unwrap();
    assert_eq!(unsafe { libc::mkfifo(cpath.as_ptr(), 0o600) }, 0);
    let mut child = Guard(
        Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "fifo_returns_error_without_waiting_for_a_writer"])
            .env(CHILD_PATH, &path)
            .spawn()
            .unwrap(),
    );
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if let Some(status) = child.0.try_wait().unwrap() {
            assert!(status.success(), "FIFO loader child failed: {status}");
            break;
        }
        assert!(Instant::now() < deadline, "FIFO loader child timed out");
        std::thread::sleep(Duration::from_millis(10));
    }
}
