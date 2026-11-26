// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! A minimal PNG writer: RGBA8, filter 0.

/// `pixels`, straight-alpha RGBA rows top to bottom, as a PNG file.
pub fn encode(pixels: &[u8], (w, h): (usize, usize)) -> Vec<u8> {
    fn crc(bytes: &[u8]) -> u32 {
        let mut c = !0u32;
        for &b in bytes {
            c ^= b as u32;
            for _ in 0..8 {
                c = (c >> 1) ^ (0xedb88320 & (0u32.wrapping_sub(c & 1)));
            }
        }
        !c
    }
    fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
        out.extend_from_slice(&(data.len() as u32).to_be_bytes());
        let start = out.len();
        out.extend_from_slice(kind);
        out.extend_from_slice(data);
        out.extend_from_slice(&crc(&out[start..]).to_be_bytes());
    }
    let mut raw = Vec::with_capacity((w * 4 + 1) * h);
    for row in pixels.chunks_exact(w * 4) {
        raw.push(0);
        raw.extend_from_slice(row);
    }
    let mut png = b"\x89PNG\r\n\x1a\n".to_vec();
    let mut header = Vec::new();
    header.extend_from_slice(&(w as u32).to_be_bytes());
    header.extend_from_slice(&(h as u32).to_be_bytes());
    header.extend_from_slice(&[8, 6, 0, 0, 0]);
    chunk(&mut png, b"IHDR", &header);
    chunk(
        &mut png,
        b"IDAT",
        &miniz_oxide::deflate::compress_to_vec_zlib(&raw, 6),
    );
    chunk(&mut png, b"IEND", &[]);
    png
}

pub fn write(path: &std::path::Path, pixels: &[u8], size: (usize, usize)) -> std::io::Result<()> {
    std::fs::write(path, encode(pixels, size))
}
