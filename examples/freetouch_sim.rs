// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! freetouch_sim REC --link PATH [options]: a FreeTouch dongle simulated on a pseudo-terminal,
//! played in real time from a recording made with a Rift headset (`cv1touch::freetouch::sim`).
//! PATH is made a symbolic link to the port, again on each replug; give it to a daemon whose
//! system shows it as the dongle's node. Linux only.
//!
//!   --flash left|right=FILE  that controller's calibration, from a factory calibration cache
//!                            file (`rift-touch-config-<hash>.bin`): config reads are answered
//!                            from it, with its hash; without one that controller answers none
//!   --drift-ppm N            the dongle's clock against the host's (50): the recording plays
//!                            that much faster
//!   --start-us N             the dongle's clock at the start (4294000000: wraps 1 s in)
//!   --unplug AT:DOWN         unplug AT s into the recording for DOWN s (repeatable)
//!   --until S                stop S s into the recording
//!   --foreign                be another device on the dongle's USB ID instead: text, no records
//!   --exposures FILE         each recorded frame as its arrival played, one per line: host ns
//!                            then, its exposure on the dongle's clock (us), its FRAME_OBS
//!                            record's place in the recording, sensor serial
//!
//! Each event and each command a host sends is printed with its host time, then a summary.
#[cfg(target_os = "linux")]
use std::path::Path;

#[cfg(target_os = "linux")]
fn main() -> Result<(), String> {
    use cv1touch::freetouch::{sim, Identity};
    use std::fmt::Write;
    use std::time::Duration;

    let a: Vec<String> = std::env::args().skip(1).collect();
    let mut config = sim::Config {
        // fictional: a real dongle makes its own on first boot
        identity: Identity {
            address: [0x5e, 0x1a, 0xd0, 0x6e, 0x01],
            key: *b"simulated dongle",
            firmware: 0x0301,
        },
        drift_ppm: 50.0,
        start_us: 4_294_000_000,
        flash: [None, None],
    };
    let (mut rec, mut link, mut exposures) = (None, None, None);
    let mut plan = sim::Plan::default();
    let mut it = a.iter();
    while let Some(arg) = it.next() {
        let mut value = || it.next().ok_or(format!("{arg} needs a value"));
        let secs = |v: &str| {
            v.parse::<f64>()
                .map(Duration::from_secs_f64)
                .map_err(|e| format!("{v}: {e}"))
        };
        match arg.as_str() {
            "--link" => link = Some(value()?.clone()),
            "--exposures" => exposures = Some(value()?.clone()),
            "--drift-ppm" => config.drift_ppm = value()?.parse().map_err(|e| format!("{e}"))?,
            "--start-us" => config.start_us = value()?.parse().map_err(|e| format!("{e}"))?,
            "--until" => plan.until = Some(secs(value()?)?),
            "--foreign" => plan.foreign = true,
            "--unplug" => {
                let v = value()?;
                let (at, down) = v.split_once(':').ok_or("--unplug AT:DOWN")?;
                plan.unplug.push((secs(at)?, secs(down)?));
            }
            "--flash" => {
                let v = value()?;
                let (hand, file) = v.split_once('=').ok_or("--flash left|right=FILE")?;
                let hand = ["left", "right"]
                    .iter()
                    .position(|h| *h == hand)
                    .ok_or("--flash left|right=FILE")?;
                config.flash[hand] = Some(flash(Path::new(file))?);
            }
            _ if rec.is_none() && !arg.starts_with("--") => rec = Some(arg.clone()),
            _ => return Err(format!("unknown argument {arg}")),
        }
    }
    let (Some(rec), Some(link)) = (rec, link) else {
        return Err("usage: freetouch_sim REC --link PATH [options]; see the source".into());
    };
    let items = sim::items(Path::new(&rec))?;
    let mut s = sim::Sim::new(config);
    let mut exposures_out = String::new();
    let link = Path::new(&link);
    let served = sim::serve(
        &mut s,
        &items,
        &plan,
        &mut |port| {
            let _ = std::fs::remove_file(link);
            std::os::unix::fs::symlink(port, link)
        },
        &mut |e| {
            let _ = writeln!(
                exposures_out,
                "{} {} {} {}",
                e.played_ns, e.device_us, e.record, e.serial
            );
        },
        &mut |line| println!("{} {line}", cv1touch::platform::now_ns()),
    )
    .map_err(|e| e.to_string())?;
    for (at, c) in &s.commands {
        println!("{at} command {c:?}");
    }
    if let Some(path) = exposures {
        std::fs::write(&path, exposures_out).map_err(|e| format!("{path}: {e}"))?;
    }
    println!(
        "summary: opens {} records {} commands {} exposures {} foreign_bytes_received {}",
        served.opens,
        served.records,
        s.commands.len(),
        s.exposures.len(),
        served.foreign_bytes
    );
    let _ = std::fs::remove_file(link);
    Ok(())
}

/// A controller's flash from a factory calibration cache file, named by its hash.
#[cfg(target_os = "linux")]
fn flash(file: &Path) -> Result<Vec<u8>, String> {
    let name = file
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or_default();
    let hex = name
        .strip_prefix("rift-touch-config-")
        .and_then(|n| n.strip_suffix(".bin"))
        .filter(|h| h.len() == 32)
        .ok_or(format!("{name}: not a rift-touch-config-<hash>.bin"))?;
    let mut hash = [0u8; 16];
    for (i, b) in hash.iter_mut().enumerate() {
        *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).map_err(|e| format!("{name}: {e}"))?;
    }
    let calibration = std::fs::read(file).map_err(|e| format!("{}: {e}", file.display()))?;
    Ok(cv1touch::freetouch::sim::flash_image(&calibration, &hash))
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("freetouch_sim runs on Linux only");
}
