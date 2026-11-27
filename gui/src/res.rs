// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The program icon as a Win32 resource file (.res), which the MSVC linker links in.

/// The sizes Explorer, the taskbar and shortcuts pick from.
pub const SIZES: [usize; 7] = [16, 24, 32, 48, 64, 128, 256];

const RT_ICON: u16 = 3;
const RT_GROUP_ICON: u16 = 14;

/// Icon group 1: one PNG RT_ICON resource per size, numbered from 1.
pub fn icon() -> Vec<u8> {
    let mut res = Vec::new();
    // A .res file opens with an empty resource.
    resource(&mut res, 0, 0, 0, &[]);
    let mut group = [0u16, 1, SIZES.len() as u16].map(u16::to_le_bytes).concat();
    for (id, size) in (1u16..).zip(SIZES) {
        let rgba = crate::icon::rgba(crate::icon::Tint::Ready, size);
        let png = crate::png::encode(&rgba, (size, size));
        resource(&mut res, RT_ICON, id, 0x1010, &png);
        // Width and height are a byte each, with 0 meaning 256.
        let side = size as u8;
        group.extend_from_slice(&[side, side, 0, 0, 1, 0, 32, 0]);
        group.extend_from_slice(&(png.len() as u32).to_le_bytes());
        group.extend_from_slice(&id.to_le_bytes());
    }
    resource(&mut res, RT_GROUP_ICON, 1, 0x1030, &group);
    res
}

/// Appends a resource with a numeric type and name, in the neutral language, padded to a
/// DWORD.
fn resource(res: &mut Vec<u8>, kind: u16, name: u16, flags: u16, data: &[u8]) {
    res.extend_from_slice(&(data.len() as u32).to_le_bytes());
    res.extend_from_slice(&32u32.to_le_bytes());
    for word in [0xffff, kind, 0xffff, name, 0, 0, flags, 0, 0, 0, 0, 0] {
        res.extend_from_slice(&word.to_le_bytes());
    }
    res.extend_from_slice(data);
    res.resize(res.len().next_multiple_of(4), 0);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn icon_is_a_res_file_of_png_icons_and_their_group() {
        let res = icon();
        let word = |at: usize| u16::from_le_bytes([res[at], res[at + 1]]);
        let dword = |at: usize| u32::from_le_bytes(res[at..at + 4].try_into().unwrap()) as usize;
        let mut empty = [0u8; 32];
        empty[4] = 32;
        empty[8..10].copy_from_slice(&[0xff; 2]);
        empty[12..14].copy_from_slice(&[0xff; 2]);
        assert_eq!(res[..32], empty);
        let mut resources = Vec::new();
        let mut at = 32;
        while at < res.len() {
            assert_eq!(at % 4, 0);
            let len = dword(at);
            assert_eq!(dword(at + 4), 32);
            assert_eq!((word(at + 8), word(at + 12)), (0xffff, 0xffff));
            resources.push((word(at + 10), word(at + 14), &res[at + 32..at + 32 + len]));
            at += (32 + len).next_multiple_of(4);
        }
        assert_eq!(at, res.len());
        let ((kind, name, group), icons) = resources.split_last().unwrap();
        assert_eq!((*kind, *name), (RT_GROUP_ICON, 1));
        assert_eq!(group[..6], [0, 0, 1, 0, SIZES.len() as u8, 0]);
        assert_eq!(group.len(), 6 + 14 * SIZES.len());
        assert_eq!(icons.len(), SIZES.len());
        for (i, ((kind, id, png), size)) in icons.iter().zip(SIZES).enumerate() {
            assert_eq!((*kind, *id), (RT_ICON, i as u16 + 1));
            assert_eq!(png[..8], *b"\x89PNG\r\n\x1a\n");
            assert_eq!(png[12..16], *b"IHDR");
            let side = (size as u32).to_be_bytes();
            assert_eq!(png[16..24], [side, side].concat());
            let entry = &group[6 + 14 * i..][..14];
            let byte = if size == 256 { 0 } else { size as u8 };
            assert_eq!(entry[..8], [byte, byte, 0, 0, 1, 0, 32, 0]);
            assert_eq!(entry[8..12], (png.len() as u32).to_le_bytes());
            assert_eq!(entry[12..], id.to_le_bytes());
        }
    }
}
