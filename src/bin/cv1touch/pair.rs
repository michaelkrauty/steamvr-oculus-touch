// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! `cv1touch pair`: pairing Touch controllers to the CV1 headset, with no Oculus
//! software. The protocol lives in the library (`cv1touch::pairing`); this is the person's side.

use super::*;
use cv1touch::pairing::{self, BootMode, Broadcast, Offer, Record};
use cv1touch::platform::hid::{self, Device as RadioDevice, FeatureDevice, Waker};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Mode {
    /// Read and save what is paired; change nothing.
    Check { json: bool },
    /// Pairing mode: list what asks to be paired; pair nothing.
    Listen,
    /// Pairing mode: offer to pair each controller that asks.
    Pair,
    /// Return a headset left in pairing mode back to normal mode.
    Normal,
    /// Pair Touch controllers using commands and JSON events.
    Events,
}

/// Parses `cv1touch pair`'s flags, or None when they do not match.
pub(super) fn flags(args: &[&str]) -> Option<Mode> {
    match args {
        [] => Some(Mode::Pair),
        ["--check"] => Some(Mode::Check { json: false }),
        ["--check", "--json"] | ["--json", "--check"] => Some(Mode::Check { json: true }),
        ["--listen"] => Some(Mode::Listen),
        ["--normal"] => Some(Mode::Normal),
        ["--events"] => Some(Mode::Events),
        _ => None,
    }
}

/// How long the headset may take to come back after rebooting into another mode.
const REBOOT_TIMEOUT: Duration = Duration::from_secs(20);
/// How long to wait for the radio input interface (interface 1) after interface 0 comes back.
const RADIO_INTERFACE_TIMEOUT: Duration = Duration::from_secs(5);
/// How long pairing mode listens when nobody ends it.
const LISTEN_LIMIT: Duration = Duration::from_secs(180);

const HOW_TO_ASK: &str = "To make a controller ask to be paired, hold these until its light blinks:
  left controller:   Menu (☰) and Y
  right controller:  Oculus button and B
Keep every other controller asleep or remove its battery: pairing writes this headset's radio
address into the controller without unpairing any previous one, and two controllers of the same
hand awake at the same time will collide on the radio.";

fn open_control() -> Result<FeatureDevice, String> {
    let path =
        hid::headset_interface(0).ok_or("no CV1 headset found on USB (is it plugged in?)")?;
    FeatureDevice::open(&path).map_err(|e| open_error(&path, &e))
}

fn open_radio() -> Result<RadioDevice, String> {
    let start = Instant::now();
    loop {
        match hid::headset_interface(1) {
            Some(path) => match RadioDevice::open(&path) {
                Ok(dev) => return Ok(dev),
                Err(e) if start.elapsed() >= RADIO_INTERFACE_TIMEOUT => {
                    return Err(open_error(&path, &e))
                }
                Err(_) => {}
            },
            None if start.elapsed() >= RADIO_INTERFACE_TIMEOUT => {
                return Err(
                    "the CV1 headset's radio interface did not appear on USB (is it plugged in?)"
                        .into(),
                );
            }
            None => {}
        }
        thread::sleep(Duration::from_millis(150));
    }
}

fn open_error(path: &Path, e: &io::Error) -> String {
    if cfg!(unix) && e.kind() == io::ErrorKind::PermissionDenied {
        format!(
            "{}: {e} (install the udev rule in docs/setup.md#device-access and reconnect the headset)",
            path.display()
        )
    } else {
        format!("{}: {e}", path.display())
    }
}

fn daemon_is_running() -> bool {
    let runtime = control::runtime_dir();
    if locked(&runtime.join("daemon.lock")) || locked(&runtime.join("supervisor.lock")) {
        return true;
    }
    daemon_status(Duration::from_secs(2), DaemonPurpose::Check)
        .ok()
        .flatten()
        .is_some()
}

fn stop_daemon_hint() -> &'static str {
    if cfg!(windows) {
        "cv1touchd is running and uses the headset's radio; stop it first with `cv1touch stop` \
         (or Stop daemon in the tray app), then start it again afterwards"
    } else {
        "cv1touchd is running and uses the headset's radio; stop it first with `cv1touch stop` \
         (or `systemctl --user stop cv1touchd`), then start it again afterwards with \
         `systemctl --user start cv1touchd`"
    }
}

fn start_daemon_hint() -> &'static str {
    if cfg!(windows) {
        "Start the daemon again from the tray app (or cv1touchd-quiet)."
    } else {
        "Start the daemon again: systemctl --user start cv1touchd"
    }
}

/// Reads one line from the terminal, returning early on Ctrl+C (`Err`) or EOF (`Ok(None)`).
fn read_prompt_line() -> Result<Option<String>, String> {
    loop {
        if interrupted() {
            return Err("interrupted".into());
        }
        match next_line(Duration::from_millis(100)) {
            Line::Typed(s) => return Ok(Some(s)),
            Line::Closed => return Ok(None),
            Line::NotYet => {}
        }
    }
}

/// Asks the headset to reboot into `mode`, then waits for it to come back in that mode.
fn reboot(mut dev: FeatureDevice, mode: BootMode) -> Result<FeatureDevice, String> {
    // The headset drops off USB at once; the request may report an error for that reason.
    let _ = pairing::FeatureReports::set_feature(&mut dev, &pairing::boot_mode_report(mode));
    drop(dev);
    let start = Instant::now();
    thread::sleep(Duration::from_millis(1000));
    while start.elapsed() < REBOOT_TIMEOUT {
        if let Ok(mut dev) = open_control() {
            if let Ok(Ok(now)) = pairing::boot_mode(&mut dev) {
                if now == mode {
                    return Ok(dev);
                }
            }
        }
        thread::sleep(Duration::from_millis(250));
    }
    Err(
        "the headset did not come back after rebooting: unplug its USB cable, plug it back in, \
         then run `cv1touch pair --check`"
            .into(),
    )
}

/// Puts the headset back in normal mode if it is still in pairing mode when dropped.
struct BackToNormal(Option<FeatureDevice>);
impl Drop for BackToNormal {
    fn drop(&mut self) {
        if let Some(dev) = self.0.take() {
            println!("Returning the headset to normal mode...");
            match reboot(dev, BootMode::Normal) {
                Ok(_) => println!("The headset is back in normal mode."),
                Err(e) => eprintln!("cv1touch: {e}"),
            }
        }
    }
}

fn save(record: &Record, label: &str) -> Result<PathBuf, String> {
    let dir = state_dir().join("pairing");
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let path = dir.join(format!("{}-{label}.json", record.created_unix_s));
    std::fs::write(
        &path,
        serde_json::to_string_pretty(record).map_err(|e| e.to_string())?,
    )
    .map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(path)
}

fn print_record(record: &Record) {
    println!(
        "Headset radio {} ({} mode)",
        record.headset, record.boot_mode
    );
    for slot in &record.devices {
        match &slot.info {
            Some(i) => println!(
                "  {:<17} {:08x}  serial {}  firmware {}",
                slot.device.name(),
                i.address,
                i.serial,
                i.firmware
            ),
            None => println!(
                "  {:<17} no answer (asleep, out of range, or none paired)",
                slot.device.name()
            ),
        }
    }
}

fn read_record(dev: &mut FeatureDevice) -> Result<Record, String> {
    pairing::record(dev, unix_now()).map_err(|e| format!("reading the headset's radio: {e}"))
}

/// Waits for Enter or `limit`, reading pairing broadcasts meanwhile and handing each newly seen
/// device to `seen`.
fn listen(
    radio: &mut RadioDevice,
    limit: Duration,
    mut seen: impl FnMut(&Broadcast) -> Result<bool, String>,
) -> Result<(), String> {
    let waker = Waker::new().map_err(|e| format!("preparing to wait for radio reports: {e}"))?;
    let start = Instant::now();
    let mut known: BTreeMap<(u8, u32), ()> = BTreeMap::new();
    let mut buf = [0u8; pairing::RADIO_REPORT_LEN];
    while start.elapsed() < limit {
        if interrupted() {
            return Err("interrupted".into());
        }
        match next_line(Duration::ZERO) {
            Line::Typed(_) | Line::Closed => return Ok(()),
            Line::NotYet => {}
        }
        let _woke = radio
            .wait(&waker, Some(Duration::from_millis(100)))
            .map_err(|e| format!("reading the headset's radio: {e}"))?;
        // Always drain after wait: on Linux a hung-up hidraw fd wakes poll with
        // POLLHUP/POLLERR rather than POLLIN and reports the disconnect on try_read.
        while let Some(n) = radio
            .try_read(&mut buf)
            .map_err(|e| format!("reading the headset's radio: {e}"))?
        {
            for item in pairing::broadcasts(&buf[..n]) {
                match item {
                    Ok(b) => {
                        if known.insert((b.device.code(), b.address), ()).is_none() && !seen(&b)? {
                            return Ok(());
                        }
                    }
                    Err(e) => eprintln!("cv1touch: ignored a pairing message: {e}"),
                }
            }
        }
    }
    println!("Stopped listening after {} s.", limit.as_secs());
    Ok(())
}

fn describe(b: &Broadcast, headset: &str) -> String {
    let to = pairing::hex(&b.paired_to);
    let whose = if to == headset {
        "this headset".to_string()
    } else {
        format!("another headset ({to})")
    };
    format!(
        "{} {:08x} (firmware {}) asks to be paired; it is paired to {whose}",
        b.device.name(),
        b.address,
        b.firmware
    )
}

fn pair_question(device: pairing::Device, also_paired: Option<u32>) -> String {
    match also_paired {
        Some(old) => format!(
            "  Pair it? Another {} ({old:08x}) is already paired to this headset; pairing does not \
             unpair it, so keep the unused one asleep or remove its battery. Enter: pair  s: skip",
            device.name()
        ),
        None => "  Pair it? Enter: pair  s: skip".into(),
    }
}

pub(super) fn run(mode: Mode) -> i32 {
    if mode == Mode::Events {
        return event_mode();
    }
    match work(mode) {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

fn work(mode: Mode) -> Result<(), String> {
    catch_interrupts();
    if daemon_is_running() {
        return Err(stop_daemon_hint().into());
    }
    let mut dev = open_control()?;
    if mode == Mode::Normal {
        match pairing::boot_mode(&mut dev)
            .map_err(|e| format!("reading the headset's boot mode: {e}"))?
        {
            Ok(BootMode::Normal) => {
                println!("The headset is already in normal mode.");
            }
            Ok(BootMode::Pairing) => {
                println!("Returning the headset to normal mode...");
                dev = reboot(dev, BootMode::Normal)?;
                println!("The headset is back in normal mode.");
            }
            Err(other) => {
                return Err(format!(
                    "the headset reports an unexpected boot mode ({other}); unplug its USB cable and plug it back in"
                ));
            }
        }
        let rec = read_record(&mut dev)?;
        print_record(&rec);
        println!("{}", start_daemon_hint());
        return Ok(());
    }

    if mode != (Mode::Check { json: true }) {
        human_action("Wake both controllers (press any button on each), then press Enter.");
        if read_prompt_line()?.is_none() {
            return Ok(());
        }
    }
    let before = read_record(&mut dev)?;
    let saved = save(
        &before,
        if matches!(mode, Mode::Check { .. }) {
            "check"
        } else {
            "before"
        },
    )?;
    if mode == (Mode::Check { json: true }) {
        println!(
            "{}",
            serde_json::to_string_pretty(&before).map_err(|e| e.to_string())?
        );
        return Ok(());
    }
    print_record(&before);
    println!("Saved to {}", saved.display());
    if matches!(mode, Mode::Check { .. }) {
        if before.boot_mode != "normal" {
            println!(
                "The headset is not in normal mode: run `cv1touch pair --normal` to return it to \
                 normal mode."
            );
        }
        return Ok(());
    }

    let headset_address = pairing::headset_address(&mut dev)
        .map_err(|e| format!("reading the headset's radio address: {e}"))?;
    let mut current_slots: BTreeMap<u8, u32> = before
        .devices
        .iter()
        .filter_map(|s| s.info.as_ref().map(|i| (s.device.code(), i.address)))
        .collect();
    println!(
        "\nThe headset now restarts in pairing mode{}. Its normal mode comes back at the end, or \
         on Ctrl+C.\n{HOW_TO_ASK}",
        if mode == Mode::Listen {
            " to list what asks to be paired, without pairing anything"
        } else {
            ""
        }
    );
    human_action("Press Enter to start (q: quit).");
    let Some(line) = read_prompt_line()? else {
        return Ok(());
    };
    if line.trim().eq_ignore_ascii_case("q") {
        return Ok(());
    }
    let dev = if pairing::boot_mode(&mut dev).ok() == Some(Ok(BootMode::Pairing)) {
        dev
    } else {
        reboot(dev, BootMode::Pairing)?
    };
    let mut guard = BackToNormal(Some(dev));
    let mut radio = open_radio()?;
    human_action("Pairing mode is on. Ask with each controller now; press Enter when done.");
    let mut paired = Vec::new();
    let result = listen(&mut radio, LISTEN_LIMIT, |b| {
        println!("{}", describe(b, &before.headset));
        if mode == Mode::Listen {
            return Ok(true);
        }
        let current = current_slots.get(&b.device.code()).copied();
        match pairing::offer(b, headset_address, current) {
            Offer::AlreadyPaired => {
                println!("  already paired to this headset in that place: nothing to do");
                Ok(true)
            }
            Offer::Pair { also_paired } => {
                let question = pair_question(b.device, also_paired);
                human_action(&question);
                let Some(answer) = read_prompt_line()? else {
                    println!("  skipped");
                    return Ok(false);
                };
                if !answer.trim().is_empty() {
                    println!("  skipped");
                    return Ok(true);
                }
                let dev = guard.0.as_mut().ok_or("the headset is gone")?;
                pairing::pair(dev, b, headset_address)
                    .map_err(|e| format!("pairing the {} failed: {e}", b.device.name()))?;
                println!("  paired (ask with another controller, or press Enter when done)");
                current_slots.insert(b.device.code(), b.address);
                paired.push(b.clone());
                Ok(true)
            }
        }
    });
    drop(radio);
    // Back to normal mode, then read what is paired now.
    let dev = guard.0.take().ok_or("the headset is gone")?;
    println!("Returning the headset to normal mode...");
    let mut dev = reboot(dev, BootMode::Normal)?;
    result?;
    if !paired.is_empty() {
        human_action(
            "Press a button on the paired controller(s) to wake them, then press Enter to verify.",
        );
        let _ = read_prompt_line();
    }
    let after = read_record(&mut dev)?;
    print_record(&after);
    let saved = save(&after, "after")?;
    println!("Saved to {}", saved.display());
    println!("{}", start_daemon_hint());
    Ok(())
}

#[path = "pair_events.rs"]
mod protocol;
use protocol::event_mode;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pair_flags_parse_all_modes_and_reject_unknown_arguments() {
        assert_eq!(flags(&[]), Some(Mode::Pair));
        assert_eq!(flags(&["--check"]), Some(Mode::Check { json: false }));
        assert_eq!(
            flags(&["--check", "--json"]),
            Some(Mode::Check { json: true })
        );
        assert_eq!(
            flags(&["--json", "--check"]),
            Some(Mode::Check { json: true })
        );
        assert_eq!(flags(&["--listen"]), Some(Mode::Listen));
        assert_eq!(flags(&["--normal"]), Some(Mode::Normal));
        assert_eq!(flags(&["--json"]), None);
        assert_eq!(flags(&["--bogus"]), None);
    }

    #[test]
    fn pair_question_warns_when_another_controller_shares_the_slot() {
        let plain = pair_question(pairing::Device::LeftTouch, None);
        assert_eq!(plain, "  Pair it? Enter: pair  s: skip");
        let shared = pair_question(pairing::Device::LeftTouch, Some(0xe976_a15d));
        assert!(shared.contains("e976a15d"));
        assert!(shared.contains("does not unpair it"));
    }
}
