// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
//! Pairing Touch controllers and the remote to a CV1 headset with our own code: no Oculus software.
//!
//! The headset's radio is driven through two feature reports on its first HID interface: 0x1a
//! starts a radio command (three bytes, then polled until the busy bit clears) and 0x1b carries
//! the command's 28-byte argument or result. Report 0x06 reboots the headset into a mode; in
//! pairing mode its second interface carries broadcasts (input report 0x0c) from every controller
//! asking to be paired. The commands, report layouts and pairing sequence are ported from ouvrt
//! (github.com/pH5/ouvrt, src/rift-radio.c and src/rift-hid-reports.h, Copyright 2015-2016
//! Philipp Zabel), used under the Boost Software License 1.0.

use std::io;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use crate::platform::hid::FeatureDevice;

/// Feature reports: radio command, radio command data, boot mode.
const RADIO_CONTROL: u8 = 0x1a;
const RADIO_DATA: u8 = 0x1b;
const BOOT_MODE: u8 = 0x06;
/// The input report carrying radio messages (in pairing mode: pairing broadcasts).
pub const RADIO_REPORT: u8 = 0x0c;
pub const RADIO_REPORT_LEN: usize = 64;
const PAYLOAD: usize = 28;
/// Radio command status (byte 3 of report 0x1a): still busy, and no answer from the device.
const BUSY: u8 = 0x80;
const NO_RESPONSE: u8 = 0x08;
/// How long a radio command may stay busy.
const COMMAND_TIMEOUT: Duration = Duration::from_secs(3);

/// Feature report access to the headset's first HID interface.
pub trait FeatureReports {
    fn set_feature(&mut self, report: &[u8]) -> io::Result<()>;
    /// `buf[0]` names the report; fills `buf` and returns the length read.
    fn get_feature(&mut self, buf: &mut [u8]) -> io::Result<usize>;
}

impl FeatureReports for FeatureDevice {
    fn set_feature(&mut self, report: &[u8]) -> io::Result<()> {
        FeatureDevice::set_feature(self, report)
    }

    fn get_feature(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        FeatureDevice::get_feature(self, buf)
    }
}

/// What can be paired to a CV1 headset, in the radio's own numbering.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Device {
    Remote,
    LeftTouch,
    RightTouch,
}

impl Device {
    pub const ALL: [Device; 3] = [Device::Remote, Device::LeftTouch, Device::RightTouch];

    pub fn code(self) -> u8 {
        match self {
            Device::Remote => 1,
            Device::LeftTouch => 2,
            Device::RightTouch => 3,
        }
    }

    pub fn from_code(code: u8) -> Option<Device> {
        Device::ALL.into_iter().find(|d| d.code() == code)
    }

    /// The second parameter of the pairing configuration (a channel or slot, by ouvrt's guess).
    fn channel(self) -> u16 {
        match self {
            Device::Remote => 750,
            Device::LeftTouch => 1000,
            Device::RightTouch => 1250,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Device::Remote => "remote",
            Device::LeftTouch => "left controller",
            Device::RightTouch => "right controller",
        }
    }
}

/// The modes the headset is ever asked to boot into. Mode 1, the bootloader, is deliberately not
/// representable: without Oculus tools there is no way back from it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootMode {
    Normal,
    Pairing,
}

#[derive(Debug)]
pub enum RadioError {
    Io(io::Error),
    /// The device the command was for did not answer (asleep, out of range, not paired).
    NoResponse,
    /// The radio stayed busy past `COMMAND_TIMEOUT`.
    Busy,
    /// A report came back shorter than expected.
    Short,
}

impl std::fmt::Display for RadioError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RadioError::Io(e) => write!(f, "{e}"),
            RadioError::NoResponse => write!(f, "no answer over the radio"),
            RadioError::Busy => write!(f, "the headset's radio stayed busy"),
            RadioError::Short => write!(f, "short report from the headset"),
        }
    }
}

impl From<io::Error> for RadioError {
    fn from(e: io::Error) -> Self {
        RadioError::Io(e)
    }
}

/// A controller (or the remote) as the headset's radio reports it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ControllerInfo {
    pub device: Device,
    /// Its own 32-bit radio address.
    pub address: u32,
    pub serial: String,
    pub firmware: String,
}

/// A device asking to be paired, from a pairing-mode broadcast.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Broadcast {
    pub device: Device,
    pub address: u32,
    /// The 5-byte radio address of the headset it is paired to now.
    pub paired_to: [u8; 5],
    pub firmware: String,
    pub signal: u8,
}

pub fn boot_mode_report(mode: BootMode) -> [u8; 4] {
    let code = match mode {
        BootMode::Normal => 0,
        BootMode::Pairing => 2,
    };
    [BOOT_MODE, 0, 0, code]
}

/// The headset's current boot mode: Err(raw value) for anything but normal or pairing.
pub fn boot_mode(dev: &mut impl FeatureReports) -> Result<Result<BootMode, u8>, RadioError> {
    let mut buf = [0u8; 4];
    buf[0] = BOOT_MODE;
    if dev.get_feature(&mut buf)? < 4 {
        return Err(RadioError::Short);
    }
    Ok(match buf[3] {
        0 => Ok(BootMode::Normal),
        2 => Ok(BootMode::Pairing),
        other => Err(other),
    })
}

/// Waits until any in-flight radio command clears the busy bit (ignoring NO_RESPONSE from earlier
/// commands).
fn wait_idle(dev: &mut impl FeatureReports) -> Result<(), RadioError> {
    let start = Instant::now();
    loop {
        let mut buf = [0u8; 6];
        buf[0] = RADIO_CONTROL;
        if dev.get_feature(&mut buf)? < 4 {
            return Err(RadioError::Short);
        }
        if buf[3] & BUSY == 0 {
            return Ok(());
        }
        if start.elapsed() > COMMAND_TIMEOUT {
            return Err(RadioError::Busy);
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// Runs one radio command and waits for it to finish.
fn transfer(dev: &mut impl FeatureReports, cmd: [u8; 3]) -> Result<(), RadioError> {
    wait_idle(dev)?;
    dev.set_feature(&[RADIO_CONTROL, 0, 0, cmd[0], cmd[1], cmd[2]])?;
    let start = Instant::now();
    loop {
        let mut buf = [0u8; 6];
        buf[0] = RADIO_CONTROL;
        if dev.get_feature(&mut buf)? < 4 {
            return Err(RadioError::Short);
        }
        if buf[3] & BUSY == 0 {
            return if buf[3] & NO_RESPONSE != 0 {
                Err(RadioError::NoResponse)
            } else {
                Ok(())
            };
        }
        if start.elapsed() > COMMAND_TIMEOUT {
            return Err(RadioError::Busy);
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// Runs a radio command and returns its 28-byte result.
fn read(dev: &mut impl FeatureReports, cmd: [u8; 3]) -> Result<[u8; PAYLOAD], RadioError> {
    transfer(dev, cmd)?;
    let mut buf = [0u8; 3 + PAYLOAD];
    buf[0] = RADIO_DATA;
    if dev.get_feature(&mut buf)? < buf.len() {
        return Err(RadioError::Short);
    }
    let mut payload = [0u8; PAYLOAD];
    payload.copy_from_slice(&buf[3..]);
    Ok(payload)
}

/// Sends a radio command's 28-byte argument, then the command.
fn write(
    dev: &mut impl FeatureReports,
    cmd: [u8; 3],
    payload: &[u8; PAYLOAD],
) -> Result<(), RadioError> {
    wait_idle(dev)?;
    let mut buf = [0u8; 3 + PAYLOAD];
    buf[0] = RADIO_DATA;
    buf[3..].copy_from_slice(payload);
    dev.set_feature(&buf)?;
    transfer(dev, cmd)
}

/// Printable ASCII up to the first NUL or non-printable byte.
fn text(bytes: &[u8]) -> String {
    bytes
        .iter()
        .take_while(|b| b.is_ascii_graphic() || **b == b' ')
        .map(|&b| b as char)
        .collect::<String>()
        .trim()
        .to_string()
}

/// The headset's own 5-byte radio address (the one paired devices and the sensors follow).
pub fn headset_address(dev: &mut impl FeatureReports) -> Result<[u8; 5], RadioError> {
    let payload = read(dev, [0x05, 0x03, 0x05])?;
    let mut address = [0u8; 5];
    address.copy_from_slice(&payload[..5]);
    Ok(address)
}

/// The device paired in `device`'s place, as the headset reaches it over the radio.
pub fn controller_info(
    dev: &mut impl FeatureReports,
    device: Device,
) -> Result<ControllerInfo, RadioError> {
    let serial = read(dev, [0x03, 0x88, device.code()])?;
    let firmware = read(dev, [0x03, 0x82, device.code()])?;
    Ok(ControllerInfo {
        device,
        address: u32::from_le_bytes([serial[0], serial[1], serial[2], serial[3]]),
        serial: text(&serial[9..23]),
        firmware: text(&firmware[14..24]),
    })
}

/// The pairing broadcasts in one radio report (input 0x0c); Err for a message that looks like a
/// pairing broadcast but does not parse, so nothing is paired from it.
pub fn broadcasts(report: &[u8]) -> Vec<Result<Broadcast, String>> {
    if report.len() < RADIO_REPORT_LEN || report[0] != RADIO_REPORT {
        return Vec::new();
    }
    [&report[3..31], &report[31..59]]
        .into_iter()
        // empty slots, and normal traffic (other flags), are not broadcasts
        .filter(|m| m[..2] == [0x1a, 0x00])
        .map(|m| {
            if m[2] != 0x03 || m[3] != 0x01 {
                return Err(format!("unexpected pairing message: {m:02x?}"));
            }
            let device = Device::from_code(m[6])
                .ok_or_else(|| format!("unknown device type {:#04x} asks to pair", m[6]))?;
            let mut paired_to = [0u8; 5];
            paired_to.copy_from_slice(&m[11..16]);
            Ok(Broadcast {
                device,
                address: u32::from_le_bytes([m[7], m[8], m[9], m[10]]),
                paired_to,
                firmware: text(&m[17..25]),
                signal: m[4],
            })
        })
        .collect()
}

/// The four radio writes that pair `device` at `address` to the headset at `headset`, as
/// (command, argument).
pub fn pairing_writes(
    device: Device,
    address: u32,
    headset: [u8; 5],
) -> [([u8; 3], [u8; PAYLOAD]); 4] {
    // 1: the device's address
    let mut target = [0u8; PAYLOAD];
    target[..4].copy_from_slice(&address.to_le_bytes());
    // 2: its type, the headset it belongs to, and two parameters only known by value
    let mut config = [0u8; PAYLOAD];
    config[..3].copy_from_slice(&[0x11, 0x05, device.code()]);
    config[3..7].copy_from_slice(&headset[..4]);
    config[7..12].copy_from_slice(&headset);
    config[12..14].copy_from_slice(&device.channel().to_le_bytes());
    config[16..18].copy_from_slice(&2000u16.to_le_bytes());
    // 3: leave pairing
    let mut stop = [0u8; PAYLOAD];
    stop[0] = 0x21;
    [
        ([0x04, 0x07, 0x05], target),
        ([0x04, 0x09, 0x05], config),
        ([0x04, 0x09, 0x05], stop),
        // 4: done
        ([0x04, 0x08, 0x05], [0u8; PAYLOAD]),
    ]
}

/// Pairs the device behind `b` to the headset at `headset`; stops at the first failed step.
pub fn pair(
    dev: &mut impl FeatureReports,
    b: &Broadcast,
    headset: [u8; 5],
) -> Result<(), RadioError> {
    for (cmd, arg) in pairing_writes(b.device, b.address, headset) {
        write(dev, cmd, &arg)?;
    }
    Ok(())
}

/// What to do with a device asking to be paired.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Offer {
    /// It is paired to this headset already, in its own place: nothing to write.
    AlreadyPaired,
    /// Pairing writes this headset's radio address into the device without unpairing any other
    /// controller already answering in the same slot (`also_paired`).
    Pair { also_paired: Option<u32> },
}

/// `b` against this headset (`headset`) and what answers in its place there now (`current`).
pub fn offer(b: &Broadcast, headset: [u8; 5], current: Option<u32>) -> Offer {
    if b.paired_to == headset && current == Some(b.address) {
        Offer::AlreadyPaired
    } else {
        Offer::Pair {
            also_paired: current,
        }
    }
}

/// One place the headset pairs a device in, as read.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Slot {
    pub device: Device,
    pub info: Option<ControllerInfo>,
    /// Why nothing answered (asleep, out of range, or nothing paired there).
    pub error: Option<String>,
}

/// What a headset has paired, for the person to keep: enough to pair the same devices again.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Record {
    pub format: String,
    pub created_unix_s: u64,
    /// The headset's radio address, as hex bytes.
    pub headset: String,
    pub boot_mode: String,
    pub devices: Vec<Slot>,
}

pub fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// Reads the headset's address and what answers in each of its places. Only reads.
pub fn record(dev: &mut impl FeatureReports, unix_s: u64) -> Result<Record, RadioError> {
    let boot = match boot_mode(dev)? {
        Ok(BootMode::Normal) => "normal".to_string(),
        Ok(BootMode::Pairing) => "pairing".to_string(),
        Err(other) => format!("unknown ({other})"),
    };
    let headset = headset_address(dev)?;
    let devices = Device::ALL
        .into_iter()
        .map(|device| match controller_info(dev, device) {
            Ok(info) => Slot {
                device,
                info: Some(info),
                error: None,
            },
            Err(e) => Slot {
                device,
                info: None,
                error: Some(e.to_string()),
            },
        })
        .collect();
    Ok(Record {
        format: "cv1touch-pairing".into(),
        created_unix_s: unix_s,
        headset: hex(&headset),
        boot_mode: boot,
        devices,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A headset's radio as seen through feature reports: commands answer after a few busy polls
    /// with `results` (per command), devices in `silent` never answer, and every report sent is
    /// kept.
    #[derive(Default)]
    struct FakeHeadset {
        sent: Vec<Vec<u8>>,
        boot: u8,
        busy_polls: usize,
        polls_left: usize,
        current: Option<[u8; 3]>,
        silent: Vec<[u8; 3]>,
        results: std::collections::HashMap<[u8; 3], [u8; PAYLOAD]>,
        data: [u8; PAYLOAD],
    }

    impl FeatureReports for FakeHeadset {
        fn set_feature(&mut self, report: &[u8]) -> io::Result<()> {
            self.sent.push(report.to_vec());
            match report[0] {
                RADIO_CONTROL => {
                    let cmd = [report[3], report[4], report[5]];
                    self.current = Some(cmd);
                    self.polls_left = self.busy_polls;
                    if let Some(r) = self.results.get(&cmd) {
                        self.data = *r;
                    }
                }
                BOOT_MODE => self.boot = report[3],
                _ => {}
            }
            Ok(())
        }

        fn get_feature(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            match buf[0] {
                RADIO_CONTROL => {
                    let mut status = 0;
                    if self.polls_left > 0 {
                        self.polls_left -= 1;
                        status = BUSY;
                    } else if self.current.is_some_and(|c| self.silent.contains(&c)) {
                        status = NO_RESPONSE;
                    }
                    buf[..6].copy_from_slice(&[RADIO_CONTROL, 0, 0, status, 0, 0]);
                    Ok(6)
                }
                RADIO_DATA => {
                    buf[..3].copy_from_slice(&[RADIO_DATA, 0, 0]);
                    buf[3..3 + PAYLOAD].copy_from_slice(&self.data);
                    Ok(3 + PAYLOAD)
                }
                BOOT_MODE => {
                    buf[..4].copy_from_slice(&[BOOT_MODE, 0, 0, self.boot]);
                    Ok(4)
                }
                _ => Err(io::Error::from(io::ErrorKind::Unsupported)),
            }
        }
    }

    const HEADSET: [u8; 5] = [0xa9, 0x1f, 0xaa, 0xed, 0xd1];

    /// A pairing broadcast message (28 bytes) for `device` at `address`, paired to `to`.
    fn pairing_message(device: u8, address: u32, to: [u8; 5]) -> [u8; PAYLOAD] {
        let mut m = [0u8; PAYLOAD];
        m[..3].copy_from_slice(&[0x1a, 0x00, 0x03]);
        m[3] = 0x01;
        m[4] = 42;
        m[6] = device;
        m[7..11].copy_from_slice(&address.to_le_bytes());
        m[11..16].copy_from_slice(&to);
        m[17..25].copy_from_slice(b"1.15.0\0\0");
        m
    }

    fn radio_report(first: [u8; PAYLOAD], second: [u8; PAYLOAD]) -> [u8; RADIO_REPORT_LEN] {
        let mut r = [0u8; RADIO_REPORT_LEN];
        r[0] = RADIO_REPORT;
        r[3..31].copy_from_slice(&first);
        r[31..59].copy_from_slice(&second);
        r
    }

    #[test]
    fn the_bootloader_mode_can_be_read_but_never_requested() {
        assert_eq!(boot_mode_report(BootMode::Normal), [0x06, 0, 0, 0]);
        assert_eq!(boot_mode_report(BootMode::Pairing), [0x06, 0, 0, 2]);
        let mut h = FakeHeadset::default();
        assert_eq!(boot_mode(&mut h).unwrap(), Ok(BootMode::Normal));
        h.boot = 2;
        assert_eq!(boot_mode(&mut h).unwrap(), Ok(BootMode::Pairing));
        h.boot = 1;
        assert_eq!(boot_mode(&mut h).unwrap(), Err(1));
    }

    #[test]
    fn a_radio_command_waits_out_the_busy_bit_and_reports_a_silent_device() {
        let mut h = FakeHeadset {
            busy_polls: 3,
            ..Default::default()
        };
        transfer(&mut h, [0x05, 0x03, 0x05]).unwrap();
        assert_eq!(h.sent, vec![vec![0x1a, 0, 0, 0x05, 0x03, 0x05]]);
        h.silent.push([0x03, 0x88, 0x02]);
        assert!(matches!(
            transfer(&mut h, [0x03, 0x88, 0x02]),
            Err(RadioError::NoResponse)
        ));
        // A subsequent command succeeds even though the previous command left NO_RESPONSE set.
        transfer(&mut h, [0x05, 0x03, 0x05]).unwrap();
    }

    #[test]
    fn an_in_flight_busy_bit_is_waited_out_before_sending_a_new_command() {
        let mut h = FakeHeadset {
            polls_left: 2,
            ..Default::default()
        };
        transfer(&mut h, [0x05, 0x03, 0x05]).unwrap();
        assert_eq!(h.polls_left, 0);
        assert_eq!(h.sent, vec![vec![0x1a, 0, 0, 0x05, 0x03, 0x05]]);
    }

    #[test]
    fn broadcasts_parse_both_slots_and_reject_malformed_or_unknown_devices() {
        let mut normal = [0u8; PAYLOAD];
        normal[..2].copy_from_slice(&[0x1c, 0x00]);
        let report = radio_report(pairing_message(2, 0x1234_5678, HEADSET), normal);
        let got = broadcasts(&report);
        assert_eq!(got.len(), 1);
        let b = got[0].as_ref().unwrap();
        assert_eq!(b.device, Device::LeftTouch);
        assert_eq!(b.address, 0x1234_5678);
        assert_eq!(b.paired_to, HEADSET);
        assert_eq!(b.firmware, "1.15.0");
        assert_eq!(b.signal, 42);

        let mut bad_type = pairing_message(9, 1, HEADSET);
        let mut bad_header = pairing_message(3, 2, HEADSET);
        bad_header[3] = 0x02;
        let both = radio_report(bad_type, bad_header);
        let errs = broadcasts(&both);
        assert_eq!(errs.len(), 2);
        assert!(errs[0].is_err());
        assert!(errs[1].is_err());

        bad_type[6] = 3;
        let short = &radio_report(bad_type, [0; PAYLOAD])[..10];
        assert!(broadcasts(short).is_empty());
    }

    #[test]
    fn record_reads_headset_and_slots_and_round_trips_as_json() {
        let mut h = FakeHeadset::default();
        let mut addr = [0u8; PAYLOAD];
        addr[..5].copy_from_slice(&HEADSET);
        h.results.insert([0x05, 0x03, 0x05], addr);
        let mut left_ser = [0u8; PAYLOAD];
        left_ser[..4].copy_from_slice(&0x0102_0304u32.to_le_bytes());
        left_ser[9..23].copy_from_slice(b"WMTD1234567890");
        h.results.insert([0x03, 0x88, 2], left_ser);
        let mut left_fw = [0u8; PAYLOAD];
        left_fw[14..20].copy_from_slice(b"1.15.0");
        h.results.insert([0x03, 0x82, 2], left_fw);
        h.silent.push([0x03, 0x88, 1]);
        h.silent.push([0x03, 0x88, 3]);

        let rec = record(&mut h, 1_700_000_000).unwrap();
        assert_eq!(rec.format, "cv1touch-pairing");
        assert_eq!(rec.headset, "a9 1f aa ed d1");
        assert_eq!(rec.boot_mode, "normal");
        assert!(rec.devices[0].info.is_none());
        let left = rec.devices[1].info.as_ref().unwrap();
        assert_eq!(left.address, 0x0102_0304);
        assert_eq!(left.serial, "WMTD1234567890");
        assert_eq!(left.firmware, "1.15.0");
        assert!(rec.devices[2].info.is_none());

        let json = serde_json::to_string_pretty(&rec).unwrap();
        assert_eq!(serde_json::from_str::<Record>(&json).unwrap(), rec);
    }

    #[test]
    fn a_device_already_paired_here_is_left_alone_and_a_new_one_names_any_other_in_that_slot() {
        let mut b = broadcast(Device::LeftTouch, 0x1111);
        b.paired_to = HEADSET;
        assert_eq!(offer(&b, HEADSET, Some(0x1111)), Offer::AlreadyPaired);
        // points at this headset, but another left controller answered in this slot
        assert_eq!(
            offer(&b, HEADSET, Some(0x2222)),
            Offer::Pair {
                also_paired: Some(0x2222)
            }
        );
        // paired to another headset, nothing answering in this place here
        b.paired_to = [9; 5];
        assert_eq!(offer(&b, HEADSET, None), Offer::Pair { also_paired: None });
        // the same device, reported paired to another headset, is still offered
        assert_eq!(
            offer(&b, HEADSET, Some(0x1111)),
            Offer::Pair {
                also_paired: Some(0x1111)
            }
        );
    }

    #[test]
    fn the_pairing_writes_are_ouvrts_four_steps() {
        let w = pairing_writes(Device::LeftTouch, 0x1234_5678, HEADSET);
        assert_eq!(w[0].0, [0x04, 0x07, 0x05]);
        assert_eq!(w[0].1[..4], 0x1234_5678u32.to_le_bytes());
        assert!(w[0].1[4..].iter().all(|&b| b == 0));
        assert_eq!(w[1].0, [0x04, 0x09, 0x05]);
        let mut config = [0u8; PAYLOAD];
        config[..3].copy_from_slice(&[0x11, 0x05, 0x02]);
        config[3..7].copy_from_slice(&HEADSET[..4]);
        config[7..12].copy_from_slice(&HEADSET);
        config[12..14].copy_from_slice(&1000u16.to_le_bytes());
        config[16..18].copy_from_slice(&2000u16.to_le_bytes());
        assert_eq!(w[1].1, config);
        assert_eq!(w[2].0, [0x04, 0x09, 0x05]);
        assert_eq!(w[2].1[0], 0x21);
        assert!(w[2].1[1..].iter().all(|&b| b == 0));
        assert_eq!(w[3].0, [0x04, 0x08, 0x05]);
        assert_eq!(w[3].1, [0; PAYLOAD]);
        assert_eq!(
            pairing_writes(Device::Remote, 1, HEADSET)[1].1[12..14],
            750u16.to_le_bytes()
        );
        assert_eq!(
            pairing_writes(Device::RightTouch, 1, HEADSET)[1].1[12..14],
            1250u16.to_le_bytes()
        );
    }

    fn broadcast(device: Device, address: u32) -> Broadcast {
        Broadcast {
            device,
            address,
            paired_to: [1, 2, 3, 4, 5],
            firmware: "1.15.0".into(),
            signal: 40,
        }
    }

    #[test]
    fn pairing_sends_each_argument_before_its_command() {
        let mut h = FakeHeadset {
            busy_polls: 2,
            ..Default::default()
        };
        pair(&mut h, &broadcast(Device::RightTouch, 0xabcd), HEADSET).unwrap();
        let writes = pairing_writes(Device::RightTouch, 0xabcd, HEADSET);
        let mut expected = Vec::new();
        for (cmd, arg) in writes {
            let mut data = vec![RADIO_DATA, 0, 0];
            data.extend_from_slice(&arg);
            expected.push(data);
            expected.push(vec![RADIO_CONTROL, 0, 0, cmd[0], cmd[1], cmd[2]]);
        }
        assert_eq!(h.sent, expected);
    }

    #[test]
    fn pairing_stops_at_the_first_step_the_device_does_not_answer() {
        let mut h = FakeHeadset {
            silent: vec![[0x04, 0x09, 0x05]],
            ..Default::default()
        };
        assert!(matches!(
            pair(&mut h, &broadcast(Device::LeftTouch, 9), HEADSET),
            Err(RadioError::NoResponse)
        ));
        // step 1 and the configuration of step 2 went out; nothing after the failed command
        let commands: Vec<&[u8]> = h
            .sent
            .iter()
            .filter(|r| r[0] == RADIO_CONTROL)
            .map(|r| &r[3..6])
            .collect();
        assert_eq!(
            commands,
            vec![&[0x04, 0x07, 0x05][..], &[0x04, 0x09, 0x05][..]]
        );
    }
}
