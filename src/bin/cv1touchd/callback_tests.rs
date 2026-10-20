// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

use super::*;
use std::io::{Read, Write};
use std::os::fd::{AsRawFd, FromRawFd};
use std::os::unix::process::ExitStatusExt;
use std::process::Stdio;
use std::time::Instant;

// Compile the actual production C boundaries with the daemon's real logger and panic hook.
#[path = "../../engine/callbacks.rs"]
mod callbacks;

struct PoisonedSink(Mutex<()>);

impl EngineSink for PoisonedSink {
    fn frame(&self, _: &engine::FrameInfo, _: &[Blob]) {
        drop(self.0.lock().unwrap());
    }
    fn inspection_identity(&self, _: &engine::InspectionIdentity, _: u64) {
        drop(self.0.lock().unwrap());
    }
    fn search_started(&self, _: &engine::FrameInfo, _: u64) {
        drop(self.0.lock().unwrap());
    }
    fn report(&self, _: i32, _: u64, _: &[u8]) {
        drop(self.0.lock().unwrap());
    }
    fn image(&self, _: &engine::FrameInfo, _: usize, _: usize, _: usize, _: &[u8]) {
        drop(self.0.lock().unwrap());
    }
    fn activity(&self, _: engine::RadioReader, _: i32, _: u64, _: u64, _: bool, _: f32) {
        drop(self.0.lock().unwrap());
    }
}

fn run(kind: &str, test: &str) {
    const CHILD: &str = "CV1T_PANICKING_CALLBACK";
    if std::env::var_os(CHILD).is_some() {
        let _logging = daemon_log::init().unwrap();
        let sink = PoisonedSink(Mutex::new(()));
        let _ = std::panic::catch_unwind(|| {
            let _held = sink.0.lock().unwrap();
            panic!("poison the callback test mutex");
        });
        let sink: Arc<dyn EngineSink> = Arc::new(sink);
        let user = (&sink as *const Arc<dyn EngineSink>).cast_mut().cast();
        let info = engine::FrameInfo::default();
        expected_abort::prepare();
        // SAFETY: valid input buffers and a live Arc; none of these callbacks touches hardware.
        unsafe {
            match kind {
                "frame" => callbacks::frame_cb(user, &info, 0, std::ptr::null()),
                "search" => callbacks::search_cb(user, &info, 1),
                "identity" => {
                    callbacks::identity_cb(user, &engine::InspectionIdentity::default(), 1)
                }
                "image" => callbacks::image_cb(user, &info, 1, 1, 1, [0u8].as_ptr()),
                "report" => callbacks::report_cb(user, 0, 0, std::ptr::null(), 0),
                "activity" => callbacks::activity_cb(user, 1, 2, 0, 0, false, 0.0),
                _ => unreachable!(),
            }
        }
        panic!("panicking C callback returned");
    }
    for full in [true, false] {
        let mut fds = [-1; 2];
        // SAFETY: two descriptor slots, each owned by one File below.
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
        let mut read = unsafe { fs::File::from_raw_fd(fds[0]) };
        let mut write = unsafe { fs::File::from_raw_fd(fds[1]) };
        let fd = write.as_raw_fd();
        let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
        assert!(flags >= 0);
        if full {
            assert_eq!(
                unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) },
                0
            );
            loop {
                match write.write(&[b'x'; 4096]) {
                    Ok(_) => {}
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                    Err(e) => panic!("fill stderr: {e}"),
                }
            }
            assert_eq!(unsafe { libc::fcntl(fd, libc::F_SETFL, flags) }, 0);
        }
        let observed = write.try_clone().unwrap();
        let mut child = process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", test, "--nocapture"])
            .env(CHILD, kind)
            .stdout(Stdio::null())
            .stderr(Stdio::from(write))
            .spawn()
            .unwrap();
        let start = Instant::now();
        let status = loop {
            if let Some(status) = child.try_wait().unwrap() {
                break Some(status);
            }
            if start.elapsed() >= Duration::from_secs(2) {
                child.kill().unwrap();
                child.wait().unwrap();
                break None;
            }
            thread::sleep(Duration::from_millis(5));
        };
        let after = unsafe { libc::fcntl(observed.as_raw_fd(), libc::F_GETFL) };
        drop(observed);
        let mut bytes = Vec::new();
        read.read_to_end(&mut bytes).unwrap();
        assert!(status.is_some(), "{kind} callback panic waited for stderr");
        assert_eq!(status.unwrap().signal(), Some(libc::SIGABRT));
        assert_eq!(after, flags);
        if !full {
            let text = String::from_utf8(bytes).unwrap();
            assert!(text.contains("PoisonError"), "{text}");
            assert!(!text.contains("cannot unwind"), "{text}");
        }
    }
}

#[test]
fn frame_panic_aborts_with_a_full_pipe() {
    run(
        "frame",
        "callback_tests::frame_panic_aborts_with_a_full_pipe",
    );
}

#[test]
fn search_panic_aborts_with_a_full_pipe() {
    run(
        "search",
        "callback_tests::search_panic_aborts_with_a_full_pipe",
    );
}

#[test]
fn image_panic_aborts_with_a_full_pipe() {
    run(
        "image",
        "callback_tests::image_panic_aborts_with_a_full_pipe",
    );
}

#[test]
fn report_panic_aborts_with_a_full_pipe() {
    run(
        "report",
        "callback_tests::report_panic_aborts_with_a_full_pipe",
    );
}

#[test]
fn activity_panic_aborts_with_a_full_pipe() {
    run(
        "activity",
        "callback_tests::activity_panic_aborts_with_a_full_pipe",
    );
}

#[test]
fn identity_panic_aborts_with_a_full_pipe() {
    run(
        "identity",
        "callback_tests::identity_panic_aborts_with_a_full_pipe",
    );
}
