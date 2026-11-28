// SPDX-License-Identifier: Apache-2.0

use super::*;
use pairing::Device;
use serde_json::{json, Value};
use std::io::{BufRead, Write};
use std::sync::mpsc::{self, Receiver, TryRecvError};

struct Commands {
    rx: Receiver<Option<Command>>,
    first: Option<Result<Option<Command>, TryRecvError>>,
}

impl Commands {
    fn new(rx: Receiver<Option<Command>>) -> Self {
        Self { rx, first: None }
    }

    fn stopping(&mut self) -> bool {
        matches!(
            self.first.get_or_insert_with(|| self.rx.try_recv()),
            Ok(Some(Command::Stop)) | Err(TryRecvError::Disconnected)
        )
    }

    fn next(&mut self) -> Result<Option<Command>, TryRecvError> {
        self.first.take().unwrap_or_else(|| self.rx.try_recv())
    }
}

#[derive(Debug, PartialEq, Eq)]
enum Command {
    Pair(Device, u32),
    Stop,
}

fn hand(device: Device) -> &'static str {
    match device {
        Device::LeftTouch => "left",
        Device::RightTouch => "right",
    }
}

fn parse(line: &str) -> Option<Command> {
    let mut args = line.split_whitespace();
    let command = match args.next()? {
        "done" | "cancel" | "quit" => Command::Stop,
        "pair" => {
            let name = args.next()?;
            let device = Device::ALL.into_iter().find(|&d| hand(d) == name)?;
            let address = args.next()?;
            if address.len() != 8 || !address.bytes().all(|b| b.is_ascii_hexdigit()) {
                return None;
            }
            Command::Pair(device, u32::from_str_radix(address, 16).ok()?)
        }
        _ => return None,
    };
    args.next().is_none().then_some(command)
}

fn emit(event: Value) -> Result<(), String> {
    let mut out = io::stdout().lock();
    writeln!(out, "{event}")
        .and_then(|()| out.flush())
        .map_err(|e| e.to_string())
}

fn record_event(event: &str, record: &Record) -> Value {
    let devices: Vec<_> = record
        .devices
        .iter()
        .map(|slot| {
            json!({
                "device": hand(slot.device),
                "address": slot.info.as_ref().map(|info| format!("{:08x}", info.address)),
            })
        })
        .collect();
    json!({"event": event, "headset": record.headset, "devices": devices})
}

/// Armed before requesting the mode change, including when reopening fails. Drop never
/// depends on a working stdout or stdin. The normal path also checks the reboot result.
struct BackToNormal {
    dev: Option<FeatureDevice>,
    armed: bool,
}

impl BackToNormal {
    fn restore(&mut self) -> Result<FeatureDevice, String> {
        let start = Instant::now();
        loop {
            if let Some(mut dev) = self.dev.take().or_else(|| open_control().ok()) {
                let dev = if pairing::boot_mode(&mut dev).ok() == Some(Ok(BootMode::Normal)) {
                    dev
                } else {
                    reboot(dev, BootMode::Normal)?
                };
                self.armed = false;
                return Ok(dev);
            }
            if start.elapsed() >= REBOOT_TIMEOUT {
                return Err("Could not find the headset to return it to normal mode.".into());
            }
            thread::sleep(Duration::from_millis(250));
        }
    }
}

impl Drop for BackToNormal {
    fn drop(&mut self) {
        if self.armed {
            if let Err(error) = self.restore() {
                eprintln!("cv1touch: {error}");
            }
        }
    }
}

#[derive(Default)]
struct Discovered(BTreeMap<(u8, u32), Broadcast>);

impl Discovered {
    fn get(&self, device: Device, address: u32) -> Result<&Broadcast, String> {
        self.0
            .get(&(device.code(), address))
            .ok_or_else(|| format!("No matching {} has asked to pair.", device.name()))
    }
}

fn poll(
    dev: &mut FeatureDevice,
    headset: [u8; 5],
    before: &Record,
    commands: &mut Commands,
) -> Result<(), String> {
    let mut radio = open_radio()?;
    let waker = Waker::new().map_err(|e| e.to_string())?;
    emit(json!({"event":"listening"}))?;
    let mut slots: BTreeMap<_, _> = before
        .devices
        .iter()
        .filter_map(|s| s.info.as_ref().map(|i| (s.device.code(), i.address)))
        .collect();
    let mut discovered = Discovered::default();
    let mut paired = BTreeMap::new();
    let mut buf = [0; pairing::RADIO_REPORT_LEN];
    let start = Instant::now();
    while start.elapsed() < LISTEN_LIMIT {
        // Drain commands in FIFO order, without starving radio polling or signal handling.
        for _ in 0..64 {
            if interrupted() {
                return Err("Pairing was interrupted.".into());
            }
            match commands.next() {
                Ok(Some(Command::Stop)) | Err(TryRecvError::Disconnected) => return Ok(()),
                Ok(None) => emit(
                    json!({"event":"error", "fatal":false, "message":"Invalid pairing command."}),
                )?,
                Ok(Some(Command::Pair(device, address))) => {
                    let key = (device.code(), address);
                    let mut radio_error = None;
                    let result = discovered.get(device, address).and_then(|b| {
                        if paired.contains_key(&key)
                            || pairing::offer(b, headset, slots.get(&device.code()).copied())
                                == Offer::AlreadyPaired
                        {
                            return Ok(());
                        }
                        pairing::pair(dev, b, headset).map_err(|e| {
                            let error = format!("Could not pair the {}: {e}", device.name());
                            radio_error = Some(error.clone());
                            error
                        })?;
                        slots.insert(device.code(), address);
                        paired.insert(key, ());
                        Ok(())
                    });
                    let mut event = json!({"event":"paired", "device":hand(device), "address":format!("{address:08x}"), "ok":result.is_ok()});
                    if let Err(error) = result {
                        event["error"] = json!(error);
                    }
                    emit(event)?;
                    if let Some(error) = radio_error {
                        return Err(error);
                    }
                }
                Err(TryRecvError::Empty) => break,
            }
        }
        radio
            .wait(&waker, Some(Duration::from_millis(100)))
            .map_err(|e| e.to_string())?;
        // Bound each drain so a busy radio cannot starve commands or shutdown.
        for _ in 0..64 {
            let Some(n) = radio.try_read(&mut buf).map_err(|e| e.to_string())? else {
                break;
            };
            for b in pairing::broadcasts(&buf[..n]).into_iter().flatten() {
                let code = b.device.code();
                let key = (code, b.address);
                let std::collections::btree_map::Entry::Vacant(entry) = discovered.0.entry(key)
                else {
                    continue;
                };
                let offer = pairing::offer(&b, headset, slots.get(&code).copied());
                emit(json!({"event":"broadcast", "device":hand(b.device),
                    "address":format!("{:08x}", b.address), "firmware":b.firmware,
                    "already_paired":offer == Offer::AlreadyPaired,
                    "also_paired":matches!(offer, Offer::Pair { also_paired: Some(_) }),
                }))?;
                entry.insert(b);
            }
        }
    }
    Ok(())
}

fn work(commands: Receiver<Option<Command>>, recovery: &mut bool) -> Result<(), String> {
    if daemon_is_running() {
        return Err(stop_daemon_hint().into());
    }
    let mut dev = open_control()?;
    let boot = pairing::boot_mode(&mut dev).map_err(|e| e.to_string())?;
    let mut guard = BackToNormal {
        dev: Some(dev),
        armed: boot == Ok(BootMode::Pairing),
    };
    let mut commands = Commands::new(commands);
    let result = (|| {
        let dev = guard.dev.as_mut().expect("headset opened");
        let before = read_record(dev)?;
        save(&before, "before")?;
        emit(record_event("before", &before))?;
        let mode = boot.map_err(|_| {
            *recovery = true;
            "The headset reported an unexpected mode."
        })?;
        if interrupted() || commands.stopping() {
            return Ok(());
        }
        let headset = pairing::headset_address(dev).map_err(|e| e.to_string())?;
        if mode == BootMode::Normal {
            emit(json!({"event":"status", "message":"Restarting the headset in pairing mode…"}))?;
            guard.armed = true;
            guard.dev = Some(
                reboot(guard.dev.take().expect("headset opened"), BootMode::Pairing)
                    .inspect_err(|_| *recovery = true)?,
            );
        }
        poll(
            guard.dev.as_mut().expect("headset reopened"),
            headset,
            &before,
            &mut commands,
        )
    })();
    if guard.armed {
        // A closed output pipe must not prevent restoration or the saved snapshot.
        let _ = emit(json!({"event":"status", "message":"Returning the headset to normal mode…"}));
        let mut dev = guard.restore().inspect_err(|_| *recovery = true)?;
        let snapshot = (|| {
            let after = read_record(&mut dev)?;
            save(&after, "after")?;
            let _ = emit(record_event("after", &after));
            Ok(())
        })();
        return result.and(snapshot);
    }
    result
}

pub(super) fn event_mode() -> i32 {
    catch_interrupts();
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        for line in io::stdin().lock().lines().map_while(Result::ok) {
            let command = parse(&line);
            let stop = command == Some(Command::Stop);
            if tx.send(command).is_err() || stop {
                break;
            }
        }
    });
    let mut recovery = false;
    if let Err(message) = work(rx, &mut recovery) {
        let _ =
            emit(json!({"event":"error", "fatal":true, "message":message, "recovery":recovery}));
        return 1;
    }
    if emit(json!({"event":"done"})).is_ok() {
        0
    } else {
        1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn queued_pair_precedes_stop_or_disconnect() {
        for explicit_stop in [false, true] {
            let (tx, rx) = mpsc::channel();
            tx.send(parse("pair right 000000ab")).unwrap();
            if explicit_stop {
                tx.send(parse("done")).unwrap();
            }
            drop(tx);
            let mut commands = Commands::new(rx);
            assert!(!commands.stopping());
            assert_eq!(
                commands.next(),
                Ok(Some(Command::Pair(Device::RightTouch, 0xab)))
            );
            assert_eq!(
                commands.next(),
                if explicit_stop {
                    Ok(Some(Command::Stop))
                } else {
                    Err(TryRecvError::Disconnected)
                }
            );
        }
    }

    #[test]
    fn commands_accept_only_touch_and_exact_addresses() {
        assert_eq!(
            parse("pair left 000000aB"),
            Some(Command::Pair(Device::LeftTouch, 0xab))
        );
        for stop in ["done", "cancel", "quit"] {
            assert_eq!(parse(stop), Some(Command::Stop));
        }
        for invalid in [
            "pair right 12",
            "pair other 000000ab",
            "pair left +00000ab",
            "done now",
            "",
        ] {
            assert_eq!(parse(invalid), None);
        }
        assert_eq!(flags(&["--events"]), Some(Mode::Events));
        for other in ["--normal", "--check", "--listen", "--json"] {
            assert_eq!(flags(&["--events", other]), None);
        }
    }

    #[test]
    fn lookup_never_accepts_a_different_hand_or_address() {
        let mut discovered = Discovered::default();
        let device = Device::RightTouch;
        discovered.0.insert(
            (device.code(), 12),
            Broadcast {
                device,
                address: 12,
                paired_to: [0; 5],
                firmware: String::new(),
                signal: 1,
            },
        );
        assert!(discovered.get(device, 12).is_ok());
        assert!(discovered.get(device, 13).is_err());
        assert!(discovered.get(Device::LeftTouch, 12).is_err());
    }
}
