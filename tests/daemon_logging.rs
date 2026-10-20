// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Exercises the real fatal-exit path without opening a device or using the installed state.
#[cfg(all(unix, feature = "engine"))]
#[test]
fn a_full_stderr_pipe_cannot_hold_up_a_fatal_exit() {
    use std::io::{Read, Write};
    use std::os::fd::{AsRawFd, FromRawFd};
    use std::process::{Command, Stdio};
    use std::time::{Duration, Instant};

    let mut fds = [-1; 2];
    // SAFETY: space for the two new descriptors; each is then owned by one File.
    assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
    let mut read = unsafe { std::fs::File::from_raw_fd(fds[0]) };
    let mut write = unsafe { std::fs::File::from_raw_fd(fds[1]) };
    let fd = write.as_raw_fd();
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
    assert!(flags >= 0);
    assert_eq!(
        unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) },
        0
    );
    let fill = [b'x'; 4096];
    loop {
        match write.write(&fill) {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
            Err(e) => panic!("fill stderr: {e}"),
        }
    }
    assert_eq!(unsafe { libc::fcntl(fd, libc::F_SETFL, flags) }, 0);
    // The duplicate observes any change the child makes to this pipe's file status flags.
    let observed = write.try_clone().unwrap();
    let mut child = Command::new(env!("CARGO_BIN_EXE_cv1touchd"))
        .arg("--invalid-option-for-log-test")
        .env_remove("CV1TOUCH_CPUS")
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
        std::thread::sleep(Duration::from_millis(5));
    };
    let after = unsafe { libc::fcntl(observed.as_raw_fd(), libc::F_GETFL) };
    drop(observed);
    let mut bytes = Vec::new();
    read.read_to_end(&mut bytes).unwrap();
    assert!(status.is_some(), "fatal exit waited for a full stderr pipe");
    assert_eq!(status.unwrap().code(), Some(2));
    assert_eq!(after, flags, "the daemon changed the pipe's blocking mode");
}
