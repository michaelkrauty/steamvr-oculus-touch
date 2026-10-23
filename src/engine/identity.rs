// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

use std::ffi::{c_char, c_int};

/// Factory-calibration identity owned by one query, without a heap allocation.
/// Keep this value out of public logs, like its owned input-calibration counterpart.
pub struct ControllerIdentity([u8; 32]);

impl ControllerIdentity {
    /// Read the native contract once, rejecting errors, partial writes and malformed identities.
    pub(super) fn query(read: impl FnOnce(&mut [c_char; 33]) -> c_int) -> Option<Self> {
        // A non-hex, non-NUL sentinel detects missing writes, including the terminator.
        let mut buf = [1; 33];
        if read(&mut buf) != 0 || buf[32] != 0 {
            return None;
        }
        // Casting back to u8 preserves the byte on both signed- and unsigned-C-char targets.
        let bytes = std::array::from_fn(|i| buf[i] as u8);
        bytes
            .iter()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(b))
            .then_some(Self(bytes))
    }

    pub fn as_str(&self) -> &str {
        // Construction accepts only ASCII lowercase hex; no lossy conversion or pointer cast.
        std::str::from_utf8(&self.0).expect("validated controller identity")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_identity_validates_every_byte_and_termination() {
        let valid = *b"0123456789abcdef0123456789abcdef\0";
        for position in 0..33 {
            for byte in 0..=255u8 {
                let mut input = valid;
                input[position] = byte;
                let result = ControllerIdentity::query(|out| {
                    *out = input.map(|b| b as c_char);
                    0
                });
                let expected = if position == 32 {
                    byte == 0
                } else {
                    byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)
                };
                assert_eq!(
                    result.is_some(),
                    expected,
                    "position={position} byte={byte}"
                );
                if let Some(identity) = result {
                    assert_eq!(identity.as_str().as_bytes(), &input[..32]);
                }
            }
        }
    }

    #[test]
    fn native_identity_rejects_errors_and_partial_writes() {
        for status in [-1, 1] {
            assert!(ControllerIdentity::query(|out| {
                out[..32].fill(b'a' as c_char);
                out[32] = 0;
                status
            })
            .is_none());
        }
        for written in 0..=32 {
            assert!(ControllerIdentity::query(|out| {
                out[..written].fill(b'a' as c_char);
                0
            })
            .is_none());
        }
        let mut calls = 0;
        let a = ControllerIdentity::query(|out| {
            calls += 1;
            out[..32].fill(b'a' as c_char);
            out[32] = 0;
            0
        })
        .unwrap();
        let b = ControllerIdentity::query(|out| {
            calls += 1;
            out[..32].fill(b'b' as c_char);
            out[32] = 0;
            0
        })
        .unwrap();
        assert_eq!(calls, 2);
        assert_eq!(a.as_str(), "a".repeat(32));
        assert_eq!(b.as_str(), "b".repeat(32));
    }
}
