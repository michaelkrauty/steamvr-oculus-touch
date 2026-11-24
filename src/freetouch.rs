// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The host side of a FreeTouch dongle's serial protocol (firmware 0x0300 on, documented in its
//! `docs/PROTOCOL.md`): the dongle's 54-byte records, the link's AES-128-CCM, the headset's
//! 64-byte radio report rebuilt around a decrypted controller report, and the dongle's commands.
//! A dongle takes the place of the Rift headset's radio; nothing here reads or writes a port.

use aes::Aes128;
use ccm::aead::{AeadInPlace, KeyInit};
use ccm::consts::{U13, U4};
use ccm::Ccm;

/// Every record the dongle sends is this long and starts with a 4-byte magic.
pub const RECORD_SIZE: usize = 54;
/// A controller's uplink: a 5-byte header, the encrypted report and a 4-byte MIC.
pub const UPLINK_SIZE: usize = 34;
/// A controller report: the headset radio's Touch message less its flags and device type.
pub const REPORT_SIZE: usize = 25;
/// The firmware that first sends its identity (status code 9); older firmware is not supported.
pub const MIN_FIRMWARE: u32 = 0x0300;

const INPUT: &[u8; 4] = b"CV1I";
const STATUS: &[u8; 4] = b"CV1S";
const REPLY: &[u8; 4] = b"CV1R";
/// A controller answers a poll up to five 2 ms poll slots after it was sent.
const SLOT_US: u32 = 2000;
const LATE_SLOTS: u32 = 5;

type LinkCipher = Ccm<Aes128, U4, U13>;

/// Status codes (`CV1S`) a host acts on.
pub const STATUS_PAIRING: u32 = 6;
pub const STATUS_POLL_CLOCK: u32 = 7;
pub const STATUS_IDENTITY: u32 = 9;

/// A controller report as it came off the radio, still encrypted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Input {
    pub sequence: u32,
    /// The dongle's clock (us) at the poll the report answers.
    pub poll_us: u32,
    /// Signal strength, -dBm.
    pub rssi: u8,
    /// The reply slot the controller was paired to (1..4).
    pub slot: u8,
    /// 2 left, 3 right.
    pub session: u8,
    pub uplink: [u8; UPLINK_SIZE],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    pub code: u32,
    pub a: u32,
    pub b: u32,
    /// Records the dongle could not queue over USB, so far.
    pub drops: u32,
    pub raw: [u8; 34],
}

/// A controller's answer to a read command (`R`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reply {
    pub len: u8,
    pub frame: [u8; 48],
    pub slot: u8,
}

impl Reply {
    pub fn frame(&self) -> &[u8] {
        &self.frame[..usize::from(self.len.min(48))]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Record {
    Input(Input),
    Status(Status),
    Reply(Reply),
}

/// The dongle's radio address (for the sensors, in place of the headset's) and its link key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Identity {
    pub address: [u8; 5],
    pub key: [u8; 16],
    pub firmware: u32,
}

impl Status {
    pub fn identity(&self) -> Option<Identity> {
        (self.code == STATUS_IDENTITY && self.a == 21).then(|| Identity {
            address: self.raw[..5].try_into().unwrap(),
            key: self.raw[5..21].try_into().unwrap(),
            firmware: self.b,
        })
    }

    /// A poll's timestamp on the `poll_us` clock and the exposure period (us), every 10 ms.
    pub fn poll_clock(&self) -> Option<(u32, u32)> {
        (self.code == STATUS_POLL_CLOCK).then_some((self.a, self.b))
    }
}

fn u32_at(r: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(r[at..at + 4].try_into().unwrap())
}

/// One record, or None if `r` is not one.
pub fn parse(r: &[u8; RECORD_SIZE]) -> Option<Record> {
    match r[..4].try_into().unwrap() {
        INPUT if usize::from(r[16]) == UPLINK_SIZE => Some(Record::Input(Input {
            sequence: u32_at(r, 4),
            poll_us: u32_at(r, 8),
            rssi: r[15],
            slot: r[17] >> 4,
            session: r[20 + 2],
            uplink: r[20..].try_into().unwrap(),
        })),
        STATUS => Some(Record::Status(Status {
            code: u32_at(r, 4),
            a: u32_at(r, 8),
            b: u32_at(r, 12),
            drops: u32_at(r, 16),
            raw: r[20..].try_into().unwrap(),
        })),
        REPLY => Some(Record::Reply(Reply {
            len: r[4],
            frame: r[5..53].try_into().unwrap(),
            slot: r[53],
        })),
        _ => None,
    }
}

/// The link's nonce for a report that answers the poll slot at `slot_us`.
fn nonce(slot_us: u32) -> [u8; 13] {
    let mut n = [0, 0, 0, 0, 0, 0x5c, 0xcf, 0, 0, 0, 0, 0, 0];
    n[7..11].copy_from_slice(&slot_us.to_le_bytes());
    n
}

/// The decrypted report, if its MIC checks under `key` for one of the poll slots it may answer.
pub fn decrypt(key: &[u8; 16], input: &Input) -> Option<[u8; REPORT_SIZE]> {
    let cipher = LinkCipher::new(key.into());
    let mic = &input.uplink[5 + REPORT_SIZE..];
    (0..=LATE_SLOTS).find_map(|late| {
        let mut report: [u8; REPORT_SIZE] = input.uplink[5..5 + REPORT_SIZE].try_into().unwrap();
        let slot_us = input.poll_us.wrapping_add(late * SLOT_US);
        cipher
            .decrypt_in_place_detached(&nonce(slot_us).into(), &[0], &mut report, mic.into())
            .ok()
            .map(|()| report)
    })
}

/// The record a dongle would send for `report` from the controller of `session`, answering
/// `late` poll slots after the poll at `poll_us`: for tests and simulation.
pub fn encode_input(
    key: &[u8; 16],
    sequence: u32,
    poll_us: u32,
    late: u32,
    session: u8,
    slot: u8,
    report: &[u8; REPORT_SIZE],
) -> [u8; RECORD_SIZE] {
    let mut r = [0u8; RECORD_SIZE];
    r[..4].copy_from_slice(INPUT);
    r[4..8].copy_from_slice(&sequence.to_le_bytes());
    r[8..12].copy_from_slice(&poll_us.to_le_bytes());
    r[12] = 1;
    r[13] = 1;
    r[16] = UPLINK_SIZE as u8;
    r[17] = (slot << 4) | (session & 0x0f);
    r[20 + 2] = session;
    let body = &mut r[25..25 + REPORT_SIZE];
    body.copy_from_slice(report);
    let slot_us = poll_us.wrapping_add(late * SLOT_US);
    let mic = LinkCipher::new(key.into())
        .encrypt_in_place_detached(&nonce(slot_us).into(), &[0], body)
        .expect("a 25-byte report fits CCM");
    r[50..].copy_from_slice(&mic);
    r
}

/// The headset's 64-byte radio report (id 0x0c) carrying `report` from the controller of
/// `session` as its first message, for `RadioDecoder`; None for a session that is not a hand.
pub fn radio_report(session: u8, report: &[u8; REPORT_SIZE]) -> Option<[u8; 64]> {
    if session != 2 && session != 3 {
        return None;
    }
    let mut r = [0u8; 64];
    r[0] = 0x0c;
    r[3] = 0x1c; // message flags: a Touch
    r[5] = session; // device type: 2 left, 3 right, as the session
    r[6..6 + REPORT_SIZE].copy_from_slice(report);
    Some(r)
}

/// What a `Parser` saw go wrong in the stream, so far.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Losses {
    /// Bytes skipped to find the next record.
    pub skipped: u64,
    /// Controller reports missing from the sequence.
    pub missing: u64,
    /// Controller reports whose MIC did not check under the identity's key.
    pub bad_mic: u64,
    /// Controller reports that came before any identity.
    pub no_key: u64,
}

/// A record as `Parser::next_event` returns it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// A controller report that decrypted under the dongle's key.
    Report {
        input: Input,
        report: [u8; REPORT_SIZE],
    },
    Status(Status),
    Reply(Reply),
}

/// Splits the dongle's byte stream into records, finding its place again at the next magic after
/// lost bytes, and decrypts controller reports with the key from the dongle's identity.
#[derive(Debug, Default)]
pub struct Parser {
    buf: Vec<u8>,
    key: Option<[u8; 16]>,
    last_sequence: Option<u32>,
    pub losses: Losses,
}

impl Parser {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// The next record in what was pushed, or None until more bytes come.
    pub fn next_event(&mut self) -> Option<Event> {
        let mut at = 0;
        let event = loop {
            let Some(r) = self.buf.get(at..at + RECORD_SIZE) else {
                break None;
            };
            let Some(record) = parse(r.try_into().unwrap()) else {
                at += 1;
                self.losses.skipped += 1;
                continue;
            };
            at += RECORD_SIZE;
            match record {
                Record::Status(s) => {
                    if let Some(id) = s.identity() {
                        self.key = Some(id.key);
                    }
                    break Some(Event::Status(s));
                }
                Record::Reply(r) => break Some(Event::Reply(r)),
                Record::Input(input) => {
                    let Some(key) = self.key else {
                        // Without a key the record boundary is unauthenticated: a cut record's
                        // magic over the next record's bytes would otherwise swallow the identity.
                        self.losses.no_key += 1;
                        at -= RECORD_SIZE - 1;
                        continue;
                    };
                    let Some(report) = decrypt(&key, &input) else {
                        // Corrupt, or a cut record's magic over the next record's bytes: look
                        // for a record from the next byte on.
                        self.losses.bad_mic += 1;
                        at -= RECORD_SIZE - 1;
                        continue;
                    };
                    // Gaps between reports that decrypted; a sequence that goes back means the
                    // dongle restarted: count afresh.
                    if let Some(last) = self.last_sequence {
                        if input.sequence > last {
                            self.losses.missing += u64::from(input.sequence - last - 1);
                        }
                    }
                    self.last_sequence = Some(input.sequence);
                    break Some(Event::Report { input, report });
                }
            }
        };
        self.buf.drain(..at);
        event
    }
}

pub mod sim;

/// Commands to the dongle, as the bytes to write to its port.
pub mod cmd {
    /// Send the identity (status code 9) now.
    pub const IDENTITY: [u8; 1] = *b"I";
    /// Pair the next controller in pairing mode within 60 s (status code 6 reports the result).
    pub const PAIR: [u8; 1] = *b"P";
    /// Power the controllers off; a button wakes each.
    pub const SLEEP: [u8; 2] = [b'Z', 0];

    fn with_u32(c: u8, v: u32) -> [u8; 5] {
        let v = v.to_le_bytes();
        [c, v[0], v[1], v[2], v[3]]
    }

    /// Vibrate the controllers of `session` at `amplitude` (0 stops) and 320 Hz or 160 Hz.
    pub fn haptic(session: u8, amplitude: u8, frequency_hz: u16) -> [u8; 5] {
        let f = frequency_hz.to_le_bytes();
        [b'V', session, amplitude, f[0], f[1]]
    }

    /// 1: keep the controllers from dozing (the dongle's default); 0: let them power off.
    pub fn keep_awake(on: bool) -> [u8; 2] {
        [b'A', u8::from(on)]
    }

    /// The camera exposure period the polls announce, 5000-65535 us.
    pub fn exposure_period(us: u32) -> [u8; 5] {
        with_u32(b'X', us)
    }

    /// Slew the polls' exposure phase by `us`, gradually (within +-33333 us).
    pub fn exposure_phase(us: i32) -> [u8; 5] {
        with_u32(b'E', us as u32)
    }

    /// Read `length` bytes at `offset` of the configuration of the controller of `session`;
    /// the answer comes as a reply record.
    pub fn config_read(session: u8, offset: u16, length: u16) -> [u8; 11] {
        let r = with_u32(b'R', u32::from(offset) | (u32::from(length) << 16));
        [
            b'O', 0x0a, b'S', session, b'Y', 0xff, r[0], r[1], r[2], r[3], r[4],
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: [u8; 16] = *b"fictional key 01";

    fn report(seed: u8) -> [u8; REPORT_SIZE] {
        std::array::from_fn(|i| seed.wrapping_mul(31).wrapping_add(i as u8))
    }

    fn identity(key: &[u8; 16]) -> [u8; RECORD_SIZE] {
        let mut r = [0u8; RECORD_SIZE];
        r[..4].copy_from_slice(b"CV1S");
        r[4] = 9;
        r[8] = 21;
        r[12..14].copy_from_slice(&0x0301u16.to_le_bytes());
        r[20..25].copy_from_slice(&[1, 2, 3, 4, 5]);
        r[25..41].copy_from_slice(key);
        r
    }

    fn reports(p: &mut Parser) -> Vec<(u32, [u8; REPORT_SIZE])> {
        std::iter::from_fn(|| p.next_event())
            .filter_map(|e| match e {
                Event::Report { input, report } => Some((input.sequence, report)),
                _ => None,
            })
            .collect()
    }

    // RFC 3610 packet vector 1 (8-byte MIC); its 4-byte-MIC analogue is not in the RFC, so this
    // checks the crate's CCM itself and the round trips below check the 4-byte MIC.
    #[test]
    fn ccm_matches_rfc_3610_vector_1() {
        let key: [u8; 16] = std::array::from_fn(|i| 0xc0 + i as u8);
        let nonce = [0, 0, 0, 3, 2, 1, 0, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5];
        let aad: [u8; 8] = std::array::from_fn(|i| i as u8);
        let mut m: [u8; 23] = std::array::from_fn(|i| 8 + i as u8);
        let tag = Ccm::<Aes128, ccm::consts::U8, U13>::new(&key.into())
            .encrypt_in_place_detached(&nonce.into(), &aad, &mut m)
            .unwrap();
        let ct = [
            0x58, 0x8c, 0x97, 0x9a, 0x61, 0xc6, 0x63, 0xd2, 0xf0, 0x66, 0xd0, 0xc2, 0xc0, 0xf9,
            0x89, 0x80, 0x6d, 0x5f, 0x6b, 0x61, 0xda, 0xc3, 0x84,
        ];
        assert_eq!(m, ct);
        assert_eq!(tag[..], [0x17, 0xe8, 0xd1, 0x2c, 0xfd, 0xf9, 0x26, 0xe0]);
    }

    #[test]
    fn a_report_round_trips_for_every_poll_slot_it_may_answer() {
        // the documented nonce, built here by hand: prefix, then the slot 6 bytes LE, b11 = 0
        let poll_us = 0x1234_5678u32;
        let slot_us = poll_us + 2 * SLOT_US;
        let mut n = [0, 0, 0, 0, 0, 0x5c, 0xcf, 0, 0, 0, 0, 0, 0];
        n[7..11].copy_from_slice(&slot_us.to_le_bytes());
        let mut uplink = [0u8; UPLINK_SIZE];
        uplink[5..30].copy_from_slice(&report(5));
        let mic = LinkCipher::new(&KEY.into())
            .encrypt_in_place_detached(&n.into(), &[0], &mut uplink[5..30])
            .unwrap();
        uplink[30..].copy_from_slice(&mic);
        let input = Input {
            sequence: 0,
            poll_us,
            rssi: 0,
            slot: 1,
            session: 2,
            uplink,
        };
        assert_eq!(decrypt(&KEY, &input), Some(report(5)));
        for late in 0..=5 {
            let mut r = encode_input(&KEY, 7, u32::MAX - 3000, late, 3, 1, &report(1));
            r[17] = 0x10; // the session is read from the uplink's header
            let Some(Record::Input(input)) = parse(&r) else {
                panic!()
            };
            assert_eq!((input.sequence, input.session, input.slot), (7, 3, 1));
            assert_eq!(decrypt(&KEY, &input), Some(report(1)));
        }
        // and not one slot later, nor under another key
        let r = encode_input(&KEY, 7, 1000, 6, 3, 1, &report(1));
        let Some(Record::Input(input)) = parse(&r) else {
            panic!()
        };
        assert_eq!(decrypt(&KEY, &input), None);
        let r = encode_input(&KEY, 7, 1000, 0, 3, 1, &report(1));
        let Some(Record::Input(input)) = parse(&r) else {
            panic!()
        };
        assert_eq!(decrypt(b"fictional key 02", &input), None);
    }

    #[test]
    fn the_parser_decrypts_with_the_identitys_key_only() {
        let mut p = Parser::new();
        p.push(&encode_input(&KEY, 1, 0, 0, 2, 1, &report(1)));
        p.push(&identity(b"fictional key 02"));
        p.push(&encode_input(&KEY, 2, 2000, 0, 2, 1, &report(2)));
        p.push(&identity(&KEY));
        p.push(&encode_input(&KEY, 3, 4000, 0, 2, 1, &report(3)));
        assert_eq!(reports(&mut p), [(3, report(3))]);
        assert_eq!(
            p.losses,
            Losses {
                skipped: 106,
                missing: 0,
                bad_mic: 1,
                no_key: 1,
            }
        );
    }

    #[test]
    fn the_parser_finds_its_place_after_lost_bytes_and_counts_gaps() {
        let mut p = Parser::new();
        p.push(&identity(&KEY));
        p.push(&encode_input(&KEY, 9, 0, 0, 3, 1, &report(0)));
        p.push(&encode_input(&KEY, 10, 2000, 0, 3, 1, &report(1))[..20]); // cut short
        let mut corrupt = encode_input(&KEY, 11, 4000, 0, 3, 1, &report(2));
        corrupt[30] ^= 1; // one bit of the ciphertext
        p.push(&corrupt);
        p.push(&encode_input(&KEY, 14, 10000, 0, 3, 1, &report(3)));
        // out of order, as after a restart of the dongle: still decoded, nothing counted missing
        p.push(&encode_input(&KEY, 0, 100, 0, 3, 1, &report(4)));
        p.push(&encode_input(&KEY, 1, 2100, 0, 3, 1, &report(5)));
        let last = encode_input(&KEY, 2, 4100, 0, 3, 1, &report(6));
        p.push(&last[..53]); // not complete yet
        assert_eq!(
            reports(&mut p),
            [
                (9, report(0)),
                (14, report(3)),
                (0, report(4)),
                (1, report(5))
            ]
        );
        // the cut record read over the corrupt one, then the corrupt one: both refused, and
        // 19 + 53 bytes passed over to reach the next record
        assert_eq!(
            p.losses,
            Losses {
                skipped: 72,
                missing: 4,
                bad_mic: 2,
                no_key: 0
            }
        );
        p.push(&last[53..]);
        assert_eq!(reports(&mut p), [(2, report(6))]);
    }

    #[test]
    fn a_cut_input_before_the_identity_does_not_swallow_it() {
        let mut p = Parser::new();
        // A truncated CV1I whose first 20 bytes pass parse() once the identity's bytes follow.
        p.push(&encode_input(&KEY, 1, 0, 0, 2, 1, &report(1))[..20]);
        p.push(&identity(&KEY));
        p.push(&encode_input(&KEY, 2, 2000, 0, 2, 1, &report(2)));
        assert_eq!(reports(&mut p), [(2, report(2))]);
        assert_eq!(
            p.losses,
            Losses {
                skipped: 19,
                missing: 0,
                bad_mic: 0,
                no_key: 1,
            }
        );

        // Same across a read boundary: the first 54 bytes arrive before the rest of the identity.
        let mut p = Parser::new();
        let mut stream = Vec::new();
        stream.extend_from_slice(&encode_input(&KEY, 1, 0, 0, 2, 1, &report(1))[..20]);
        stream.extend_from_slice(&identity(&KEY));
        stream.extend_from_slice(&encode_input(&KEY, 2, 2000, 0, 2, 1, &report(2)));
        p.push(&stream[..RECORD_SIZE]);
        assert_eq!(p.next_event(), None);
        p.push(&stream[RECORD_SIZE..]);
        assert_eq!(reports(&mut p), [(2, report(2))]);
        assert_eq!(p.losses.no_key, 1);
        assert_eq!(p.losses.skipped, 19);
    }

    #[test]
    fn statuses_and_replies() {
        let mut r = identity(&KEY);
        r[8] = 20; // not the identity's length
        let Some(Record::Status(s)) = parse(&r) else {
            panic!()
        };
        assert_eq!(s.identity(), None);
        let Some(Record::Status(s)) = parse(&identity(&KEY)) else {
            panic!()
        };
        assert_eq!(
            s.identity(),
            Some(Identity {
                address: [1, 2, 3, 4, 5],
                key: KEY,
                firmware: 0x0301
            })
        );
        let mut r = [0u8; RECORD_SIZE];
        r[..4].copy_from_slice(b"CV1S");
        r[4] = 7;
        r[8..12].copy_from_slice(&123_456u32.to_le_bytes());
        r[12..16].copy_from_slice(&19_200u32.to_le_bytes());
        let Some(Record::Status(s)) = parse(&r) else {
            panic!()
        };
        assert_eq!(
            (s.identity(), s.poll_clock()),
            (None, Some((123_456, 19_200)))
        );
        r[..5].copy_from_slice(b"CV1R\x03");
        r[5..8].copy_from_slice(&[0xaa, 0xbb, 0xcc]);
        r[53] = 2;
        let Some(Record::Reply(reply)) = parse(&r) else {
            panic!()
        };
        assert_eq!((reply.frame(), reply.slot), (&[0xaa, 0xbb, 0xcc][..], 2));
    }

    #[test]
    fn the_radio_report_is_the_headsets() {
        let mut m = report(9);
        m[22] = 0x21; // the battery channel
        m[23..].copy_from_slice(&87u16.to_le_bytes());
        let r = radio_report(2, &m).unwrap();
        assert_eq!(
            crate::hand::battery_from_radio(&r).collect::<Vec<_>>(),
            [(crate::shm::HAND_LEFT, 87)]
        );
        assert_eq!(r[6..31], m);
        assert!(r[31..].iter().all(|&b| b == 0)); // no second message
        assert_eq!(radio_report(4, &m), None);
    }

    #[test]
    fn commands() {
        assert_eq!(cmd::haptic(3, 255, 320), [b'V', 3, 255, 0x40, 0x01]);
        assert_eq!(cmd::exposure_period(19_200), [b'X', 0x00, 0x4b, 0, 0]);
        assert_eq!(cmd::exposure_phase(-2), [b'E', 0xfe, 0xff, 0xff, 0xff]);
        assert_eq!(cmd::keep_awake(false), [b'A', 0]);
        assert_eq!(
            cmd::config_read(2, 0x10, 0x20),
            [b'O', 0x0a, b'S', 2, b'Y', 0xff, b'R', 0x10, 0, 0x20, 0]
        );
    }
}
