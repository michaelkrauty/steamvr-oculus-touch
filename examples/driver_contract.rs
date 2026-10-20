// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! driver_contract DRIVER_TEST: plays cv1touchd for the SteamVR driver's contract test
//! (driver/tests/test_daemon_contract.cpp) with the daemon's own shared-memory and control code, so
//! both sides must agree, on this platform, on the object names, the layout, the clock and the
//! wake-ups. Each "publish" request on the control endpoint bumps the update counter once.
use std::io::{BufRead, BufReader, Write};
use std::path::Path;
use std::process::{Command, ExitCode};
use std::time::Duration;

use cv1touch::platform::ipc;
use cv1touch::shm::{now_ns, Alignment, ControllerState, ShmWriter, HAND_RIGHT};
use cv1touch::{control, ffi};

/// Answers control requests until "exit": each "publish" bumps the update counter once.
fn serve(listener: &mut ipc::Listener, shm: &ShmWriter) {
    while let Ok(conn) = listener.accept(true) {
        let mut conn = BufReader::new(conn);
        let mut line = String::new();
        if conn.read_line(&mut line).is_err() {
            continue;
        }
        let reply: &[u8] = match line.trim() {
            "publish" => {
                shm.heartbeat(now_ns());
                shm.signal();
                b"ok\n"
            }
            "exit" => {
                let _ = conn.get_mut().write_all(b"ok\n");
                return;
            }
            _ => b"error unknown\n",
        };
        let _ = conn.get_mut().write_all(reply);
    }
}

fn main() -> ExitCode {
    let Some(driver_test) = std::env::args_os().nth(1) else {
        eprintln!("usage: driver_contract DRIVER_TEST");
        return ExitCode::from(2);
    };
    let id = std::process::id();
    let name = format!("/cv1t-contract-{id}");
    let control = std::env::temp_dir().join(format!("cv1t-contract-{id}.sock"));

    let shm = ShmWriter::create(&name, 0xC0FFEE).expect("shared memory");
    shm.publish_hand(
        HAND_RIGHT,
        &ControllerState {
            pos: [1.5, 0.0, 0.0],
            flags: ffi::CV1T_FLAG_CONNECTED,
            ..Default::default()
        },
    );
    shm.publish_alignment(&Alignment {
        valid: ffi::CV1T_ALIGN_LIGHTHOUSE,
        lighthouse_pos: [0.0, 0.0, -2.0],
        ..Default::default()
    });
    shm.heartbeat(now_ns());
    let mut listener = ipc::Listener::bind(&control).expect("control endpoint");
    #[cfg(unix)]
    let shared = std::env::temp_dir()
        .join(format!("cv1t-contract-private-{id}"))
        .join("control.sock");
    #[cfg(unix)]
    let mut shared_listener =
        ipc::Listener::bind_private(&shared).expect("private control endpoint");

    let status = std::thread::scope(|scope| {
        scope.spawn(|| serve(&mut listener, &shm));
        #[cfg(unix)]
        scope.spawn(|| serve(&mut shared_listener, &shm));
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
        // release the server thread
        let _ = control::request(&control, "exit", Duration::from_secs(2));
        #[cfg(unix)]
        let _ = control::request(&shared, "exit", Duration::from_secs(2));
        status
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
