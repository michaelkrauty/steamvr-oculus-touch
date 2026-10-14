// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! `cv1touch sensors`: whether each Rift sensor plugged in can be used. On Windows the engine
//! reads a sensor through WinUSB, Windows' own USB driver, which Windows does not give the
//! sensors by itself: `sensors install` gives it to each of a sensor's two interfaces (asking
//! Windows for permission once), and `sensors remove` takes off what it gave. Linux needs no
//! such step: the udev rules installed with cv1touch give access, and `sensors` checks that they
//! do.

use super::*;
use cv1touch::platform::usb::{self, Node};
use std::collections::BTreeSet;

/// The Rift sensor's USB product (of the Oculus vendor).
const SENSOR: u16 = 0x0211;

/// Windows' driver for a composite device, which lists the interfaces as nodes of their own.
const COMPOSITE: &str = "usbccgp";

/// The exit code of a run as an administrator that changed the drivers but needs Windows
/// restarted to finish; 0 and 1 as usual.
const RESTART: i32 = 3;

/// A sensor and the interfaces the engine reads it through.
struct Sensor {
    device: Node,
    interfaces: Vec<Node>,
}

impl Sensor {
    /// How a person tells the sensors apart: the serial number Windows names the device by, or
    /// on Linux where it is plugged in.
    fn name(&self) -> &str {
        self.device
            .id
            .rsplit('\\')
            .next()
            .unwrap_or(&self.device.id)
    }

    /// The device nodes a driver goes on: the interfaces, which Windows lists once the sensor
    /// runs its composite driver; the sensor itself while it lists none, as when another driver
    /// took the whole sensor.
    fn nodes(&self) -> &[Node] {
        if self.interfaces.is_empty() {
            std::slice::from_ref(&self.device)
        } else {
            &self.interfaces
        }
    }
}

/// The sensors among `nodes`: each device, with the nodes whose parent it is.
fn sensors(nodes: &[Node]) -> Vec<Sensor> {
    let ids: Vec<&str> = nodes.iter().map(|n| n.id.as_str()).collect();
    let is_interface = |n: &Node| {
        n.parent
            .as_deref()
            .is_some_and(|parent| ids.contains(&parent))
    };
    nodes
        .iter()
        .filter(|n| n.product == SENSOR && !is_interface(n))
        .map(|device| Sensor {
            device: device.clone(),
            interfaces: nodes
                .iter()
                .filter(|n| n.parent.as_deref() == Some(device.id.as_str()))
                .cloned()
                .collect(),
        })
        .collect()
}

/// What a sensor needs before the engine can read it (on Windows).
#[derive(Debug, PartialEq)]
enum Need {
    Nothing,
    /// Windows has not set the sensor up yet: it lists no interfaces, and the sensor itself has
    /// no driver, or only the composite driver that is to list them. Normally that passes in a
    /// moment.
    Waiting,
    /// Some of its nodes have no driver.
    Driver,
    /// Some of its nodes run other drivers: their names.
    OtherDriver(BTreeSet<String>),
    /// A node has its driver, but Windows reports a problem with it: the problem code.
    Problem(u32),
}

fn winusb(driver: &str) -> bool {
    driver.eq_ignore_ascii_case("WinUSB")
}

fn need(sensor: &Sensor) -> Need {
    if sensor.interfaces.is_empty() {
        match sensor.device.driver.as_deref() {
            None => return Need::Waiting,
            Some(d) if d.eq_ignore_ascii_case(COMPOSITE) => {
                return sensor.device.problem.map_or(Need::Waiting, Need::Problem);
            }
            Some(_) => {}
        }
    }
    let nodes = sensor.nodes();
    let other: BTreeSet<String> = nodes
        .iter()
        .filter_map(|n| n.driver.as_deref())
        .filter(|d| !winusb(d))
        .map(String::from)
        .collect();
    if !other.is_empty() {
        Need::OtherDriver(other)
    } else if nodes.iter().any(|n| n.driver.is_none()) {
        // a node without a driver has a problem code too (28): the driver comes first
        Need::Driver
    } else if let Some(code) = nodes.iter().find_map(|n| n.problem) {
        Need::Problem(code)
    } else {
        Need::Nothing
    }
}

/// The nodes `install` gives WinUSB: those of a sensor that needs a driver, or runs another, not
/// on it yet. A sensor Windows is still setting up is left to Windows; one with a problem is
/// not helped by installing again.
fn to_install(sensor: &Sensor) -> Vec<&Node> {
    match need(sensor) {
        Need::Driver | Need::OtherDriver(_) => sensor
            .nodes()
            .iter()
            .filter(|n| !n.driver.as_deref().is_some_and(winusb))
            .collect(),
        Need::Nothing | Need::Waiting | Need::Problem(_) => Vec::new(),
    }
}

/// Whether the nodes `ids`, just given WinUSB, run with it now.
fn on_winusb(sensors: &[Sensor], ids: &[&str]) -> bool {
    let nodes: Vec<&Node> = sensors.iter().flat_map(Sensor::nodes).collect();
    ids.iter().all(|id| {
        nodes
            .iter()
            .any(|n| n.id == *id && n.driver.as_deref().is_some_and(winusb))
    })
}

/// The nodes running the WinUSB cv1touch set up, by `set_up_by_us`: what `remove` takes off,
/// leaving a WinUSB someone else set up alone, with each node's sensor's name.
fn ours(sensors: &[Sensor], set_up_by_us: impl Fn(&str) -> bool) -> Vec<(&str, &Node)> {
    sensors
        .iter()
        .flat_map(|s| s.nodes().iter().map(move |n| (s.name(), n)))
        .filter(|(_, n)| n.driver.as_deref().is_some_and(winusb) && set_up_by_us(&n.id))
        .collect()
}

fn list() -> Result<Vec<Sensor>, String> {
    usb::devices(OCULUS)
        .map(|nodes| sensors(&nodes))
        .map_err(|e| format!("cannot list the USB devices: {e}"))
}

/// A sensor's line on Windows: whether it is ready, and what it needs if not.
fn windows_line(need: Need) -> (bool, String) {
    match need {
        Need::Nothing => (true, "ready".to_string()),
        Need::Waiting => (
            false,
            "Windows is still setting it up: wait a few seconds and run this again; if it stays \
             so, unplug it and plug it back in"
                .to_string(),
        ),
        Need::Driver => (false, "needs its driver".to_string()),
        Need::OtherDriver(names) => (
            false,
            format!(
                "uses another driver ({}), so cv1touch cannot use it",
                names.into_iter().collect::<Vec<_>>().join(", ")
            ),
        ),
        Need::Problem(code) => (
            false,
            format!(
                "Windows reports a problem with it (code {code}): unplug it and plug it back in"
            ),
        ),
    }
}

/// A sensor's line on Linux, from whether this process could open it.
fn linux_line(opened: io::Result<()>) -> (bool, String) {
    match opened {
        Ok(()) => (true, "ready".to_string()),
        Err(e) if e.kind() == io::ErrorKind::PermissionDenied => (
            false,
            "cannot be opened: the udev rules let whoever is logged in at the machine itself \
             open it, so install them (docs/setup.md), log in at the machine, then unplug it and \
             plug it back in"
                .to_string(),
        ),
        Err(e) => (false, format!("cannot be opened: {e}")),
    }
}

/// One line per sensor, and whether every sensor is ready (none plugged in: not).
fn report(sensors: &[Sensor]) -> bool {
    if sensors.is_empty() {
        println!("No Rift sensors are plugged in.");
        return false;
    }
    let mut ready = true;
    for s in sensors {
        let (ok, line) = if cfg!(windows) {
            windows_line(need(s))
        } else {
            linux_line(usb::can_open(&s.device.id))
        };
        ready &= ok;
        println!("Sensor {}: {line}", s.name());
    }
    ready
}

/// `doctor`'s verdict on the sensors plugged in: whether all run with WinUSB, and what to do.
#[cfg_attr(unix, allow(dead_code))]
fn summary(sensors: &[Sensor]) -> (bool, String) {
    if sensors.is_empty() {
        return (false, "No Rift sensors are plugged in.".to_string());
    }
    let installable = sensors.iter().filter(|s| !to_install(s).is_empty()).count();
    let other = sensors
        .iter()
        .filter(|s| need(s) != Need::Nothing && to_install(s).is_empty())
        .count();
    let mut parts = Vec::new();
    if installable > 0 {
        parts.push(format!(
            "{installable} sensor(s) need WinUSB: run cv1touch sensors install"
        ));
    }
    if other > 0 {
        parts.push(format!(
            "{other} sensor(s) are not ready: run cv1touch sensors for details"
        ));
    }
    if parts.is_empty() {
        (true, format!("{} sensor(s) run with WinUSB", sensors.len()))
    } else {
        (false, parts.join("; "))
    }
}

/// `doctor`'s check that every sensor plugged in has the driver the engine reads it through.
#[cfg(windows)]
pub(super) fn doctor_check() -> DoctorCheck {
    match list() {
        Ok(sensors) => {
            let (ok, detail) = summary(&sensors);
            ("sensor_drivers", ok, detail)
        }
        Err(e) => ("sensor_drivers", false, e),
    }
}

pub(super) fn status() -> i32 {
    match list() {
        Ok(sensors) => {
            if report(&sensors) {
                0
            } else {
                if cfg!(windows) && sensors.iter().any(|s| !to_install(s).is_empty()) {
                    println!(
                        "To set them up, run: cv1touch sensors install (Windows asks you once to \
                         allow it)."
                    );
                }
                1
            }
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

/// Runs `args` again as an administrator when this process is not one: Windows asks the person
/// first. Some(exit code) when it did, None when this process may go on itself.
fn as_administrator(args: &[&str], why: &str) -> Option<i32> {
    if platform::process::elevated() {
        return None;
    }
    println!("{why} Windows asks whether to allow it: click Yes.");
    Some(match platform::process::run_elevated(args) {
        Ok(code) => code,
        Err(e) if e.kind() == io::ErrorKind::PermissionDenied => {
            println!("You chose not to allow it, so nothing was changed.");
            1
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    })
}

/// Said when Windows could not restart a device with its new driver (a program holds it).
fn restart_to_finish() {
    println!(
        "Restart Windows to finish: a program (SteamVR or cv1touchd, probably) was using a \
         sensor, so Windows could not restart it."
    );
}

/// The exit code of a run that changed drivers: `done` says the nodes are as they should be,
/// `restart` that Windows asked for a restart first.
fn outcome(done: bool, restart: bool) -> i32 {
    if !done {
        1
    } else if restart {
        RESTART
    } else {
        0
    }
}

/// `code`, once the result can have been read: the window an administrator's run opens closes as
/// soon as the program ends, so on success it stays a moment, and otherwise until Enter is
/// pressed (a minute at most). CV1TOUCH_NO_PAUSE set skips the wait, for tests.
fn leave_readable(code: i32) -> i32 {
    if platform::console::closes_with_us() && std::env::var_os("CV1TOUCH_NO_PAUSE").is_none() {
        if code == 0 {
            std::thread::sleep(Duration::from_secs(3));
        } else {
            println!("Press Enter to close this window.");
            platform::console::next_line(Duration::from_secs(60));
        }
    }
    code
}

pub(super) fn install() -> i32 {
    if !cfg!(windows) {
        println!(
            "Linux needs no driver for the sensors: the udev rules installed with cv1touch give \
             access."
        );
        return 0;
    }
    let sensors = match list() {
        Ok(sensors) => sensors,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    };
    let todo: Vec<(&str, &Node)> = sensors
        .iter()
        .flat_map(|s| to_install(s).into_iter().map(move |n| (s.name(), n)))
        .collect();
    if todo.is_empty() {
        return status();
    }
    if let Some(code) = as_administrator(
        &["sensors", "install"],
        "Your sensors need Windows' own USB driver (WinUSB) before cv1touch can read them.",
    ) {
        // what the administrator's run did, seen from here
        if code == RESTART {
            restart_to_finish();
            return RESTART;
        }
        return status();
    }
    let (mut failed, mut restart) = (false, false);
    let mut given = Vec::new();
    for (sensor, node) in &todo {
        match usb::use_winusb(&node.id) {
            Ok(needs_restart) => {
                restart |= needs_restart;
                given.push(node.id.as_str());
            }
            Err(e) => {
                eprintln!(
                    "cv1touch: Sensor {sensor}: Windows did not install the driver.\n  {}: {e}",
                    node.id
                );
                failed = true;
            }
        }
    }
    // the nodes given WinUSB run with it now, unless Windows has to restart them first
    let done = match list() {
        Ok(sensors) => {
            report(&sensors);
            !failed && (restart || on_winusb(&sensors, &given))
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            false
        }
    };
    if restart {
        restart_to_finish();
    }
    leave_readable(outcome(done, restart))
}

pub(super) fn remove() -> i32 {
    if !cfg!(windows) {
        println!("Linux needs no driver for the sensors: there is nothing to remove.");
        return 0;
    }
    let sensors = match list() {
        Ok(sensors) => sensors,
        Err(e) => {
            eprintln!("cv1touch: {e}");
            return 1;
        }
    };
    let given = ours(&sensors, usb::set_up_by_us);
    if given.is_empty() {
        println!("No sensor runs with the driver cv1touch gave it.");
        return 0;
    }
    if let Some(code) = as_administrator(
        &["sensors", "remove"],
        "This takes the driver cv1touch gave your sensors off again.",
    ) {
        // what the administrator's run did, seen from here
        if code == RESTART {
            restart_to_finish();
            return RESTART;
        }
        return removed();
    }
    let (mut failed, mut restart) = (false, false);
    for (sensor, node) in &given {
        match usb::remove_driver(&node.id) {
            Ok(needs_restart) => restart |= needs_restart,
            Err(e) => {
                eprintln!(
                    "cv1touch: Sensor {sensor}: Windows did not finish taking the driver off.\n  \
                     {}: {e}",
                    node.id
                );
                failed = true;
            }
        }
    }
    if restart {
        restart_to_finish();
    }
    leave_readable(outcome(finished(failed, restart, removed), restart))
}

/// Whether a remove is done: no node failed, and `left` (what still runs with our driver) says
/// none. Windows takes the driver off at a restart it asked for, so what is left before it says
/// nothing and is not asked.
fn finished(failed: bool, restart: bool, left: impl FnOnce() -> i32) -> bool {
    let left = if restart { 0 } else { left() };
    !failed && left == 0
}

/// Says whether any sensor still runs with the driver cv1touch gave it: the exit code.
fn removed() -> i32 {
    match list() {
        Ok(sensors) => {
            let left = ours(&sensors, usb::set_up_by_us).len();
            if left == 0 {
                println!("The sensors are back to the driver Windows gives them.");
                0
            } else {
                println!(
                    "{left} sensor interface(s) still run with the driver cv1touch gave them: run \
                     cv1touch sensors remove again."
                );
                1
            }
        }
        Err(e) => {
            eprintln!("cv1touch: {e}");
            1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(id: &str, product: u16, driver: Option<&str>, parent: Option<&str>) -> Node {
        Node {
            id: id.into(),
            product,
            name: String::new(),
            driver: driver.map(String::from),
            // as Windows reports a node without a driver
            problem: driver.is_none().then_some(28),
            parent: parent.map(String::from),
        }
    }

    const SENSOR_A: &str = r"USB\VID_2833&PID_0211\WMTD300D2001AB";
    const SENSOR_B: &str = r"USB\VID_2833&PID_0211\WMTD300D2002CD";
    const A0: &str = r"USB\VID_2833&PID_0211&MI_00\7&a&0&0000";
    const A1: &str = r"USB\VID_2833&PID_0211&MI_01\7&a&0&0001";
    const B0: &str = r"USB\VID_2833&PID_0211&MI_00\7&b&0&0000";
    const B1: &str = r"USB\VID_2833&PID_0211&MI_01\7&b&0&0001";

    /// As Windows lists them: a sensor's two interfaces are device nodes of their own, whose
    /// parent is the sensor.
    fn windows(drivers: [Option<&str>; 4]) -> Vec<Node> {
        vec![
            node(
                SENSOR_A,
                SENSOR,
                Some(COMPOSITE),
                Some(r"USB\ROOT_HUB30\4&1"),
            ),
            node(A0, SENSOR, drivers[0], Some(SENSOR_A)),
            node(A1, SENSOR, drivers[1], Some(SENSOR_A)),
            node(
                SENSOR_B,
                SENSOR,
                Some(COMPOSITE),
                Some(r"USB\ROOT_HUB30\4&1"),
            ),
            node(B0, SENSOR, drivers[2], Some(SENSOR_B)),
            node(B1, SENSOR, drivers[3], Some(SENSOR_B)),
            // the headset's HID interface is no sensor
            node(
                r"USB\VID_2833&PID_0031&MI_00\7&c",
                0x0031,
                Some("HidUsb"),
                None,
            ),
        ]
    }

    fn ids(nodes: &[(&str, &Node)]) -> Vec<(String, String)> {
        nodes
            .iter()
            .map(|(s, n)| (s.to_string(), n.id.clone()))
            .collect()
    }

    fn install_ids(sensor: &Sensor) -> Vec<String> {
        to_install(sensor).iter().map(|n| n.id.clone()).collect()
    }

    #[test]
    fn sensors_are_grouped_with_their_interfaces() {
        let found = sensors(&windows([None; 4]));
        assert_eq!(found.len(), 2);
        assert_eq!(found[0].name(), "WMTD300D2001AB");
        assert_eq!(found[0].interfaces.len(), 2);
        assert!(found[0]
            .interfaces
            .iter()
            .all(|n| n.parent.as_deref() == Some(SENSOR_A)));
        // on Linux an interface's parent is its device's sysfs name
        let linux = vec![
            node("2-5", SENSOR, Some("usb"), None),
            node("2-5:1.0", SENSOR, None, Some("2-5")),
            node("2-5:1.1", SENSOR, Some("usbfs"), Some("2-5")),
        ];
        let found = sensors(&linux);
        assert_eq!(
            (found.len(), found[0].name(), found[0].interfaces.len()),
            (1, "2-5", 2)
        );
    }

    #[test]
    fn what_each_sensor_needs() {
        let needs = |drivers| {
            sensors(&windows(drivers))
                .iter()
                .map(need)
                .collect::<Vec<_>>()
        };
        assert_eq!(
            needs([
                Some("WinUSB"),
                Some("WINUSB"),
                Some("WinUSB"),
                Some("WinUSB")
            ]),
            [Need::Nothing, Need::Nothing]
        );
        // a node without a driver carries problem code 28 too: it needs the driver, not a replug
        assert_eq!(
            needs([Some("WinUSB"), None, None, None]),
            [Need::Driver, Need::Driver]
        );
        // another driver wins over a missing one: the person should hear about it, once per name
        assert_eq!(
            needs([
                Some("oculus_sensor"),
                Some("oculus_sensor"),
                Some("WinUSB"),
                None
            ]),
            [
                Need::OtherDriver(["oculus_sensor".to_string()].into()),
                Need::Driver
            ]
        );
        // a problem Windows reports with a node that has its driver
        let mut nodes = windows([Some("WinUSB"); 4]);
        nodes[2].problem = Some(43);
        assert_eq!(need(&sensors(&nodes)[0]), Need::Problem(43));
    }

    /// While Windows lists no interfaces, the sensor itself is the node. With no driver, or only
    /// the composite driver that is to list them, Windows is still setting it up and nothing is
    /// installed (over the composite driver least of all); another driver may have taken the
    /// whole sensor; and WinUSB on the whole sensor is ready too.
    #[test]
    fn a_sensor_without_interfaces_is_judged_by_itself() {
        let alone = |driver| sensors(&[node(SENSOR_A, SENSOR, driver, None)]);
        assert_eq!(need(&alone(None)[0]), Need::Waiting);
        assert!(to_install(&alone(None)[0]).is_empty());
        let composite = alone(Some(COMPOSITE));
        assert_eq!(need(&composite[0]), Need::Waiting);
        assert!(to_install(&composite[0]).is_empty());
        let mut stuck = alone(Some(COMPOSITE));
        stuck[0].device.problem = Some(10);
        assert_eq!(need(&stuck[0]), Need::Problem(10));
        assert!(to_install(&stuck[0]).is_empty());
        let taken = alone(Some("ocusbvid"));
        assert_eq!(
            need(&taken[0]),
            Need::OtherDriver(["ocusbvid".to_string()].into())
        );
        assert_eq!(install_ids(&taken[0]), [SENSOR_A]);
        let done = alone(Some("WinUSB"));
        assert_eq!(need(&done[0]), Need::Nothing);
        assert!(to_install(&done[0]).is_empty());
        assert_eq!(
            ids(&ours(&done, |_| true)),
            [("WMTD300D2001AB".to_string(), SENSOR_A.to_string())]
        );
    }

    /// `install` sets up every interface not on WinUSB at once, whether it has another driver or
    /// none: one run, one permission. Ready sensors, and one with a problem, are left alone.
    #[test]
    fn install_sets_up_every_interface_not_on_winusb() {
        let found = sensors(&windows([
            Some("oculus_sensor"),
            None,
            Some("WinUSB"),
            Some("WinUSB"),
        ]));
        assert_eq!(install_ids(&found[0]), [A0, A1]);
        assert!(to_install(&found[1]).is_empty());
        let mut nodes = windows([Some("WinUSB"); 4]);
        nodes[1].problem = Some(10);
        assert!(to_install(&sensors(&nodes)[0]).is_empty());
    }

    /// An install is done when the nodes it gave WinUSB run with it, whatever the other sensors
    /// do; Windows asking for a restart first is its own outcome.
    #[test]
    fn an_install_is_judged_by_the_nodes_it_gave_winusb() {
        let found = sensors(&windows([
            Some("WinUSB"),
            Some("WinUSB"),
            None,
            Some("WinUSB"),
        ]));
        assert!(on_winusb(&found, &[A0, A1]));
        assert!(!on_winusb(&found, &[A0, B0]));
        assert!(!on_winusb(&found, &["USB\\VID_2833&PID_0211&MI_00\\gone"]));
        assert_eq!(
            (
                outcome(true, false),
                outcome(true, true),
                outcome(false, true)
            ),
            (0, RESTART, 1)
        );
    }

    /// A remove is done once nothing is left on our driver, unless Windows takes it off at a
    /// restart, when what is left now says nothing; a node that failed is never done.
    #[test]
    fn a_remove_is_done_by_what_is_left_or_by_the_restart() {
        assert!(finished(false, false, || 0));
        assert!(!finished(false, false, || 1));
        assert!(finished(false, true, || unreachable!(
            "asked what is left before the restart"
        )));
        assert!(!finished(true, true, || unreachable!(
            "asked what is left before the restart"
        )));
        assert!(!finished(true, false, || 0));
    }

    /// `remove` takes off only the WinUSB cv1touch set up: another driver, and a WinUSB someone
    /// else set up (Zadig's, say), are kept.
    #[test]
    fn remove_takes_off_only_what_cv1touch_set_up() {
        let found = sensors(&windows([
            Some("WinUSB"),
            Some("WinUSB"),
            Some("WinUSB"),
            Some("libusbK"),
        ]));
        let set_up_by_us = |id: &str| id != A1;
        assert_eq!(
            ids(&ours(&found, set_up_by_us)),
            [
                ("WMTD300D2001AB".to_string(), A0.to_string()),
                ("WMTD300D2002CD".to_string(), B0.to_string())
            ]
        );
        assert!(ours(&found, |_| false).is_empty());
    }

    /// `doctor` tells what to run: install for what install fixes, `sensors` for the rest; no
    /// sensor at all fails, as `sensors` does.
    #[test]
    fn doctor_says_what_to_run() {
        assert_eq!(
            summary(&[]),
            (false, "No Rift sensors are plugged in.".to_string())
        );
        let all = sensors(&windows([Some("WinUSB"); 4]));
        assert_eq!(
            summary(&all),
            (true, "2 sensor(s) run with WinUSB".to_string())
        );
        let one = sensors(&windows([
            Some("WinUSB"),
            None,
            Some("WinUSB"),
            Some("WinUSB"),
        ]));
        assert_eq!(
            summary(&one),
            (
                false,
                "1 sensor(s) need WinUSB: run cv1touch sensors install".to_string()
            )
        );
        let mut nodes = windows([Some("WinUSB"), None, Some("WinUSB"), Some("WinUSB")]);
        nodes.push(node(
            r"USB\VID_2833&PID_0211\WMTD300D2003EF",
            SENSOR,
            None,
            None,
        ));
        assert_eq!(
            summary(&sensors(&nodes)),
            (
                false,
                "1 sensor(s) need WinUSB: run cv1touch sensors install; 1 sensor(s) are not \
                 ready: run cv1touch sensors for details"
                    .to_string()
            )
        );
    }

    /// Each state has its line, and only ready is ready.
    #[test]
    fn each_state_has_its_line() {
        assert_eq!(windows_line(Need::Nothing), (true, "ready".to_string()));
        let (ok, line) = windows_line(Need::Waiting);
        assert!(!ok && line.contains("still setting it up"), "{line}");
        assert_eq!(
            windows_line(Need::Driver),
            (false, "needs its driver".to_string())
        );
        let (ok, line) = windows_line(Need::OtherDriver(["b".to_string(), "a".to_string()].into()));
        assert!(!ok && line.contains("(a, b)"), "{line}");
        let (ok, line) = windows_line(Need::Problem(43));
        assert!(!ok && line.contains("code 43"), "{line}");
        assert_eq!(linux_line(Ok(())), (true, "ready".to_string()));
        let denied = io::Error::from(io::ErrorKind::PermissionDenied);
        let (ok, line) = linux_line(Err(denied));
        assert!(!ok && line.contains("logged in at the machine"), "{line}");
        let (ok, line) = linux_line(Err(io::Error::other("no such bus")));
        assert!(!ok && line.ends_with("no such bus"), "{line}");
    }
}
