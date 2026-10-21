// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! driver_contract DRIVER_TEST: plays cv1touchd for the SteamVR driver's contract test
//! (driver/tests/test_daemon_contract.cpp) with the daemon's own shared-memory and control code, so
//! both sides must agree, on this platform, on the object names, the layout, the clock and the
//! wake-ups. Each "publish" request on the control endpoint bumps the update counter once.
#[path = "../tests/support/socket_dir.rs"]
mod socket_dir;

use std::io;
use std::path::Path;
use std::process::{Command, ExitCode};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::time::Duration;

use cv1touch::platform::ipc;
use cv1touch::shm::{now_ns, Alignment, ControllerState, ShmWriter, HAND_RIGHT};
use cv1touch::{control, ffi};

const CONTROL_TIMEOUT: Duration = Duration::from_secs(5);

fn published_hand(at: u64) -> ControllerState {
    ControllerState {
        publish_host_ns: at,
        pos: [1.5, 0.0, 0.0],
        flags: ffi::CV1T_FLAG_CONNECTED,
        ..Default::default()
    }
}

/// Answers control requests until "exit": each "publish" bumps the update counter once.
fn serve(listener: &mut ipc::Listener, shm: &ShmWriter, stopped: Receiver<()>) {
    loop {
        if !matches!(stopped.try_recv(), Err(mpsc::TryRecvError::Empty)) {
            return;
        }
        let conn = match listener.accept(false) {
            Ok(conn) => conn,
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                // A failed shutdown request can wake this wait without using the socket.
                if stopped.recv_timeout(Duration::from_millis(10)) != Err(RecvTimeoutError::Timeout)
                {
                    return;
                }
                continue;
            }
            Err(_) => return,
        };
        let mut exit = false;
        control::serve_client_with_initial_deadline(conn, CONTROL_TIMEOUT, |line| match line {
            "publish" => {
                let now = now_ns();
                shm.publish_hand(HAND_RIGHT, &published_hand(now));
                shm.heartbeat(now);
                shm.signal();
                "ok".into()
            }
            "exit" => {
                exit = true;
                "ok".into()
            }
            _ => "error unknown".into(),
        });
        if exit {
            return;
        }
    }
}

fn shutdown(
    endpoint: &Path,
    stopped: &Sender<()>,
    request: impl FnOnce(&Path, &str, Duration) -> io::Result<String>,
) -> bool {
    match request(endpoint, "exit", CONTROL_TIMEOUT) {
        Ok(reply) if reply == "ok" => true,
        result => {
            eprintln!(
                "contract server shutdown failed at {}: {result:?}",
                endpoint.display()
            );
            let _ = stopped.send(());
            false
        }
    }
}

fn main() -> ExitCode {
    let Some(driver_test) = std::env::args_os().nth(1) else {
        eprintln!("usage: driver_contract DRIVER_TEST");
        return ExitCode::from(2);
    };
    let id = std::process::id();
    let name = format!("/cv1t-contract-{id}");
    let fixture = socket_dir::SocketDir::new();
    let control = fixture.path().join("control.sock");

    let shm = ShmWriter::create(&name, 0xC0FFEE).expect("shared memory");
    shm.publish_hand(HAND_RIGHT, &published_hand(0));
    shm.publish_alignment(&Alignment {
        valid: ffi::CV1T_ALIGN_LIGHTHOUSE,
        lighthouse_pos: [0.0, 0.0, -2.0],
        ..Default::default()
    });
    shm.heartbeat(now_ns());
    let mut listener = ipc::Listener::bind(&control).expect("control endpoint");
    #[cfg(unix)]
    let shared = fixture.path().join("private/control.sock");
    #[cfg(unix)]
    let mut shared_listener =
        ipc::Listener::bind_private(&shared).expect("private control endpoint");

    let status = std::thread::scope(|scope| {
        let shm = &shm;
        let (stop, stopped) = mpsc::channel();
        scope.spawn(move || serve(&mut listener, shm, stopped));
        #[cfg(unix)]
        let (shared_stop, shared_stopped) = mpsc::channel();
        #[cfg(unix)]
        scope.spawn(move || serve(&mut shared_listener, shm, shared_stopped));
        let run = |runtime: &Path, fallback: Option<&Path>| {
            shm.heartbeat(now_ns());
            let mut command = Command::new(&driver_test);
            command.arg(&name).arg(runtime);
            if let Some(fallback) = fallback {
                command.arg(fallback);
            }
            command.status().expect("driver test").success()
        };
        // An old daemon's single endpoint remains sufficient.
        let status = run(&control, None);
        #[cfg(unix)]
        let status = status
            & run(&control, Some(&shared))
            & run(&control.with_extension("missing"), Some(&shared));
        let stopped = shutdown(&control, &stop, control::request);
        #[cfg(unix)]
        let stopped = shutdown(&shared, &shared_stop, control::request) & stopped;
        status & stopped
    });
    let _ = std::fs::remove_file(&control);
    #[cfg(unix)]
    {
        ipc::unlink_private_socket(&shared).expect("remove private control socket");
        std::fs::remove_dir(shared.parent().unwrap()).expect("remove private control directory");
    }
    if status {
        println!("driver contract ok");
        ExitCode::SUCCESS
    } else {
        eprintln!("driver test failed");
        ExitCode::FAILURE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_failed_shutdown_before_exit_is_sent_releases_the_server() {
        let fixture = socket_dir::SocketDir::new();
        let endpoint = fixture.path().join("stop.sock");
        let mut listener = ipc::Listener::bind(&endpoint).unwrap();
        let name = format!("/cv1t-contract-stop-{}", std::process::id());
        let shm = ShmWriter::create(&name, 1).unwrap();
        let (stop, stopped) = mpsc::channel();
        let (done, finished) = mpsc::channel();
        let server = std::thread::spawn(move || {
            serve(&mut listener, &shm, stopped);
            done.send(()).unwrap();
        });
        let mut budget = Duration::ZERO;
        let ok = shutdown(&endpoint, &stop, |path, line, timeout| {
            assert_eq!(path, endpoint);
            assert_eq!(line, "exit");
            budget = timeout;
            // Exercise the real request path with an already expired deadline: no
            // connection or exit bytes can be sent, even if the server runs immediately.
            control::request_until(
                path,
                line,
                std::time::Instant::now() - Duration::from_secs(1),
            )
        });
        let ended = finished.recv_timeout(Duration::from_secs(5)).is_ok();
        let _ = stop.send(()); // always release a mutated server before asserting
        if !ended {
            // A mutation back to blocking accept cannot observe the channel. Let it
            // accept an exit request for cleanup rather than hanging the test join.
            let _ = control::request(&endpoint, "exit", CONTROL_TIMEOUT);
        }
        server.join().unwrap();
        assert_eq!(budget, Duration::from_secs(5));
        assert!(!ok);
        assert!(
            ended,
            "failed shutdown left the server waiting for a connection"
        );
    }
}
