// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! steamvr_prediction_probe: measures how SteamVR extrapolates a driver's pose from the velocities
//! it reports, which its driver documentation does not describe correctly. The driver relies on the
//! answer (driver/src/pose.cpp): SteamVR integrates `vecAngularVelocity` in the frame of the
//! reported `qRotation` itself, whatever the world-from-driver alignment or the render offset, and
//! takes `vecVelocity` in driver world space, rotated by the alignment. Measured 2026-10 on Linux;
//! run this again when SteamVR changes.
//!
//! It plays the daemon: it publishes one synthetic right controller through the shared memory
//! (pitched 90 degrees, turning at 1 rad/s about the world's up axis, moving at 1 m/s), then reads
//! the poses SteamVR predicts ahead through the OpenVR client API and compares the axis of the
//! predicted turn, and the direction of the predicted move, with what each possible convention
//! would give for the body rate the driver hands over. Run it without cv1touchd (`cv1touch stop`, or stop the service), start SteamVR once
//! it says it is waiting, and start the daemon again afterwards:
//!
//!     cargo run --locked --no-default-features --example steamvr_prediction_probe
//!
//! Exit code 0: SteamVR behaves as the driver assumes. 1: it does not (the output says how), and
//! the driver's velocity hand-off needs another look. 2: no SteamVR or no controller appeared.
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cv1touch::ffi;
use cv1touch::shm::{self, Alignment, ControllerState, ShmWriter};
use cv1touch::vr::Client;
use nalgebra::{UnitQuaternion, Vector3};

/// How far ahead SteamVR is asked to predict (s): large enough to read the axis cleanly.
const HORIZON_S: f32 = 0.1;
/// How long to wait for SteamVR, then for the controller to appear (s).
const WAIT_S: u64 = 240;
/// The published controller: where it stands, how it turns and moves.
const POSITION: [f32; 3] = [0.0, 1.2, -0.5];
const TURN_RAD_S: f64 = 1.0;
const MOVE_M_S: f64 = 1.0;

/// What the publisher thread hands the driver.
#[derive(Clone)]
struct Synthetic {
    orient: UnitQuaternion<f64>,
    /// The turn, as a world rate (rad/s) in the driver's world.
    turn_world: Vector3<f64>,
    /// The move (m/s) in the driver's world.
    move_world: Vector3<f64>,
    /// World-from-driver alignment, if any.
    alignment: Option<UnitQuaternion<f64>>,
    /// Render offset (driver-from-head rotation) for the right hand, if any.
    render: Option<UnitQuaternion<f64>>,
}

fn xyzw(q: &UnitQuaternion<f64>) -> [f32; 4] {
    [q.i as f32, q.j as f32, q.k as f32, q.w as f32]
}

fn f3(v: &Vector3<f64>) -> [f32; 3] {
    [v.x as f32, v.y as f32, v.z as f32]
}

fn show(v: &Vector3<f64>) -> String {
    format!("({:+.2}, {:+.2}, {:+.2})", v.x, v.y, v.z)
}

/// Publishes `synthetic` at 500 Hz until `stop`, as the daemon would: fresh stamps, a position
/// fix, the body rate the driver hands SteamVR (`ang_vel - gyro_bias`, bias zero here).
fn publish(writer: &ShmWriter, synthetic: &Mutex<Synthetic>, stop: &AtomicBool) {
    while !stop.load(Ordering::Relaxed) {
        let s = synthetic.lock().unwrap().clone();
        let now = shm::now_ns();
        // SAFETY: plain C structs whose every field is valid when zero.
        let (mut st, mut al): (ControllerState, Alignment) = unsafe { std::mem::zeroed() };
        st.sample_host_ns = now;
        st.received_host_ns = now;
        st.publish_host_ns = now;
        st.last_position_fix_host_ns = now;
        st.device_time_ns = now;
        st.orient = xyzw(&s.orient);
        st.raw_orient = st.orient;
        st.pos = POSITION;
        st.raw_pos = st.pos;
        st.lin_vel = f3(&s.move_world);
        st.ang_vel = f3(&(s.orient.inverse() * s.turn_world));
        st.flags = ffi::CV1T_FLAG_CALIBRATED | ffi::CV1T_FLAG_CONNECTED | ffi::CV1T_FLAG_POSITION;
        st.battery_percent = 100;
        writer.publish_hand(ffi::CV1T_HAND_RIGHT as usize, &st);
        if let Some(a) = s.alignment {
            al.valid |= ffi::CV1T_ALIGN_LIGHTHOUSE;
            al.lighthouse_orient = xyzw(&a);
        }
        if let Some(r) = s.render {
            al.valid |= ffi::CV1T_ALIGN_RENDER_RIGHT;
            al.render_orient[ffi::CV1T_HAND_RIGHT as usize] = xyzw(&r);
        }
        writer.publish_alignment(&al);
        writer.heartbeat(now);
        writer.signal();
        std::thread::sleep(Duration::from_millis(2));
    }
    writer.retire();
}

/// A convention SteamVR might follow, and the axis the predicted turn would have under it.
struct Hypothesis {
    name: &'static str,
    axis: Vector3<f64>,
}

/// One published configuration and the conventions it tells apart.
struct Scenario {
    name: &'static str,
    synthetic: Synthetic,
    turn: Vec<Hypothesis>,
    /// The direction the predicted move would have if SteamVR rotates `vecVelocity` by the
    /// alignment, and if it does not (the same without an alignment).
    moves: Option<(Vector3<f64>, Vector3<f64>)>,
}

/// The driver hands SteamVR the body rate `orient^-1 turn` (driver/src/pose.cpp). Each convention
/// reads that vector in a frame of its own and the alignment turns the result into SteamVR's
/// universe, so a convention's axis is: in the orientation's frame, `alignment * turn` (the true
/// turn); in the head frame (orientation times the render offset), `alignment * orient * render *
/// orient^-1 * turn`; in driver world space, as documented, `alignment * orient^-1 * turn`.
fn scenarios() -> Vec<Scenario> {
    let half_pi = std::f64::consts::FRAC_PI_2;
    let pitched = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), half_pi);
    let yaw = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), half_pi);
    let roll = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), half_pi);
    let up = Vector3::y();
    let fwd = Vector3::x();
    let id = UnitQuaternion::identity();
    let body = pitched.inverse() * up;
    let in_orient = |al: &UnitQuaternion<f64>| al * up;
    let in_head = |al: &UnitQuaternion<f64>, r: &UnitQuaternion<f64>| al * (pitched * (r * body));
    let documented = |al: &UnitQuaternion<f64>| al * body;
    let base = Synthetic {
        orient: pitched,
        turn_world: up * TURN_RAD_S,
        move_world: fwd * MOVE_M_S,
        alignment: None,
        render: None,
    };
    vec![
        Scenario {
            name: "no alignment, no render offset",
            synthetic: base.clone(),
            turn: vec![
                Hypothesis {
                    name: "orientation frame",
                    axis: in_orient(&id),
                },
                Hypothesis {
                    name: "driver world space (documented)",
                    axis: documented(&id),
                },
            ],
            moves: Some((fwd, fwd)),
        },
        Scenario {
            name: "world-from-driver alignment: 90 degrees roll",
            synthetic: Synthetic {
                alignment: Some(roll),
                ..base.clone()
            },
            turn: vec![
                Hypothesis {
                    name: "orientation frame",
                    axis: in_orient(&roll),
                },
                Hypothesis {
                    name: "driver world space (documented)",
                    axis: documented(&roll),
                },
                Hypothesis {
                    name: "driver world space, alignment ignored",
                    axis: documented(&id),
                },
            ],
            moves: Some((roll * fwd, fwd)),
        },
        Scenario {
            name: "render offset: 90 degrees yaw of the right hand",
            synthetic: Synthetic {
                render: Some(yaw),
                ..base.clone()
            },
            turn: vec![
                Hypothesis {
                    name: "orientation frame",
                    axis: in_orient(&id),
                },
                Hypothesis {
                    name: "head (render) frame",
                    axis: in_head(&id, &yaw),
                },
                Hypothesis {
                    name: "driver world space (documented)",
                    axis: documented(&id),
                },
            ],
            moves: None,
        },
        Scenario {
            name: "90 degrees yaw alignment and the render offset together",
            synthetic: Synthetic {
                alignment: Some(yaw),
                render: Some(yaw),
                ..base
            },
            turn: vec![
                Hypothesis {
                    name: "orientation frame",
                    axis: in_orient(&yaw),
                },
                Hypothesis {
                    name: "head (render) frame",
                    axis: in_head(&yaw, &yaw),
                },
                Hypothesis {
                    name: "driver world space (documented)",
                    axis: documented(&yaw),
                },
            ],
            moves: None,
        },
    ]
}

/// The name of the hypothesis whose axis the measured one matches, if one does.
fn matching<'a>(axis: &Vector3<f64>, hypotheses: &'a [Hypothesis]) -> Option<&'a str> {
    hypotheses
        .iter()
        .find(|h| axis.dot(&h.axis.normalize()) > 0.95)
        .map(|h| h.name)
}

fn main() -> ExitCode {
    let writer = match ShmWriter::create(shm::DEFAULT_NAME, 0x5045_5250) {
        Ok(w) => Arc::new(w),
        Err(e) => {
            eprintln!("cannot create the shared memory ({e}); is cv1touchd stopped?");
            return ExitCode::from(2);
        }
    };
    let synthetic = Arc::new(Mutex::new(scenarios()[0].synthetic.clone()));
    let stop = Arc::new(AtomicBool::new(false));
    let publisher = {
        let (w, s, stop) = (writer.clone(), synthetic.clone(), stop.clone());
        std::thread::spawn(move || publish(&w, &s, &stop))
    };
    let finish = |code: u8| {
        stop.store(true, Ordering::Relaxed);
        ExitCode::from(code)
    };
    println!("publishing a synthetic right controller; start SteamVR now (waiting {WAIT_S} s)");
    let started = Instant::now();
    let client = loop {
        match Client::connect() {
            Ok(c) => break c,
            Err(e) => {
                if started.elapsed() > Duration::from_secs(WAIT_S) {
                    eprintln!("no SteamVR: {e}");
                    return finish(2);
                }
                std::thread::sleep(Duration::from_secs(2));
            }
        }
    };
    let index = loop {
        let devices = client.devices();
        let poses = client.raw_poses();
        let ours = devices.iter().find(|d| {
            d.tracking_system == "oculus"
                && poses.get(d.index as usize).copied().flatten().is_some()
        });
        if let Some(d) = ours {
            println!(
                "SteamVR shows the controller as device {} ({})",
                d.index, d.serial
            );
            break d.index as usize;
        }
        if started.elapsed() > Duration::from_secs(WAIT_S) {
            eprintln!("the controller never appeared in SteamVR; is the driver installed?");
            return finish(2);
        }
        std::thread::sleep(Duration::from_millis(500));
    };
    let mut as_assumed = true;
    for scenario in scenarios() {
        *synthetic.lock().unwrap() = scenario.synthetic.clone();
        std::thread::sleep(Duration::from_millis(1500));
        let now = client.raw_poses();
        let ahead = client.raw_poses_ahead(HORIZON_S);
        let (Some(Some(p0)), Some(Some((p1, _, reported_turn)))) =
            (now.get(index), ahead.get(index))
        else {
            println!("{}: SteamVR gave no valid pose", scenario.name);
            as_assumed = false;
            continue;
        };
        let relative = p1.rotation * p0.rotation.inverse();
        let (axis, angle) = relative
            .axis_angle()
            .map_or((Vector3::zeros(), 0.0), |(a, an)| (a.into_inner(), an));
        let expected_angle = TURN_RAD_S * HORIZON_S as f64;
        let turn_frame = matching(&axis, &scenario.turn).unwrap_or("none of the conventions");
        println!(
            "{}: predicted turn {:.1} deg about {} (expected {:.1} deg): {}",
            scenario.name,
            angle.to_degrees(),
            show(&axis),
            expected_angle.to_degrees(),
            turn_frame
        );
        if turn_frame != "orientation frame"
            || (angle - expected_angle).abs() > 0.3 * expected_angle
        {
            as_assumed = false;
        }
        if let Some((rotated, unrotated)) = scenario.moves {
            let shift = p1.translation.vector - p0.translation.vector;
            let dir = shift.normalize();
            let verdict = if dir.dot(&rotated) > 0.95 {
                if scenario.synthetic.alignment.is_some() {
                    "rotated by the alignment"
                } else {
                    "as published"
                }
            } else if dir.dot(&unrotated) > 0.95 {
                "not rotated by the alignment"
            } else {
                as_assumed = false;
                "neither"
            };
            println!(
                "    predicted move {:.3} m along {} (expected {:.3} m): {}",
                shift.norm(),
                show(&dir),
                MOVE_M_S * HORIZON_S as f64,
                verdict
            );
        }
        println!(
            "    SteamVR reports the angular velocity as {} rad/s in its universe",
            show(&Vector3::new(
                reported_turn[0],
                reported_turn[1],
                reported_turn[2]
            ))
        );
    }
    stop.store(true, Ordering::Relaxed);
    let _ = publisher.join();
    if as_assumed {
        println!("SteamVR integrates the angular velocity in the orientation's frame and rotates the linear velocity by the alignment: as the driver assumes");
        ExitCode::SUCCESS
    } else {
        println!(
            "SteamVR does not behave as the driver assumes: see above, and driver/src/pose.cpp"
        );
        ExitCode::from(1)
    }
}
