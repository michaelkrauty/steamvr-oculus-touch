// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Writing zip archives: deflated entries with UTF-8 names, each under 4 GiB, fewer than 65535 of
//! them (no ZIP64). It is what a report is sent as, one file that every system opens.
//!
//! Entry times are kept twice: as MS-DOS times in UTC, which have no time zone (Windows Explorer
//! shows them as local times), and in the extended-timestamp field (0x5455) as Unix times, which
//! Info-ZIP, 7-Zip and libarchive read and show in the reader's time zone.

use std::io::{self, Write};
use std::time::{SystemTime, UNIX_EPOCH};

/// CRC-32 (IEEE 802.3, as zip uses it).
pub fn crc32(data: &[u8]) -> u32 {
    const TABLE: [u32; 256] = {
        let mut table = [0u32; 256];
        let mut n = 0;
        while n < 256 {
            let mut c = n as u32;
            let mut k = 0;
            while k < 8 {
                c = if c & 1 != 0 {
                    0xedb8_8320 ^ (c >> 1)
                } else {
                    c >> 1
                };
                k += 1;
            }
            table[n] = c;
            n += 1;
        }
        table
    };
    !data.iter().fold(!0u32, |c, &b| {
        TABLE[((c ^ u32::from(b)) & 0xff) as usize] ^ (c >> 8)
    })
}

/// `at` as an MS-DOS time and date, in UTC: the first and the last a DOS date holds
/// (1980-01-01 00:00:00, 2107-12-31 23:59:58) for a time outside them.
fn dos_time(at: SystemTime) -> (u16, u16) {
    let t = crate::calendar::Utc::of(at);
    if t.year < 1980 {
        return (0, (1 << 5) | 1);
    }
    if t.year > 2107 {
        return ((23 << 11) | (59 << 5) | 29, (127 << 9) | (12 << 5) | 31);
    }
    let time = (t.hour << 11) | (t.minute << 5) | (t.second / 2);
    let date = (((t.year - 1980) as u32) << 9) | (t.month << 5) | t.day;
    (time as u16, date as u16)
}

/// The extended-timestamp field for `at`: its last change as a Unix time, which readers take as
/// signed (2038-01-19 for anything later).
fn unix_time_field(at: SystemTime) -> [u8; 9] {
    let secs = at
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
        .min(i32::MAX as u64) as u32;
    let mut field = [0u8; 9];
    field[..2].copy_from_slice(&0x5455u16.to_le_bytes());
    field[2..4].copy_from_slice(&5u16.to_le_bytes());
    field[4] = 1; // the last change only
    field[5..].copy_from_slice(&secs.to_le_bytes());
    field
}

/// A plain relative path: parts between '/' that are neither empty, "." nor "..", none ending in
/// a space or a dot, none a name Windows keeps for devices (CON, NUL, COM1, with or without an
/// extension), and no control character or one of `\:<>"|?*` anywhere: a name every system's
/// unpacker writes where it says.
fn plain_name(name: &str) -> bool {
    let device = |part: &str| {
        let stem = part.split('.').next().unwrap_or(part).to_ascii_uppercase();
        matches!(stem.as_str(), "CON" | "PRN" | "AUX" | "NUL")
            || ["COM", "LPT"].iter().any(|kind| {
                stem.strip_prefix(kind)
                    .is_some_and(|n| n.len() == 1 && n.as_bytes()[0].is_ascii_digit())
            })
    };
    name.len() <= usize::from(u16::MAX)
        && !name
            .chars()
            .any(|c| c.is_control() || "\\:<>\"|?*".contains(c))
        && name.split('/').all(|part| {
            !part.is_empty()
                && part != "."
                && part != ".."
                && !part.ends_with([' ', '.'])
                && !device(part)
        })
}

/// What ZIP64 marks its fields with: no value here may be it.
const ZIP64_MARK_16: usize = u16::MAX as usize;
const ZIP64_MARK_32: u64 = u32::MAX as u64;

fn fits_32(value: u64, what: &str) -> io::Result<u32> {
    if value >= ZIP64_MARK_32 {
        return Err(too_large(what));
    }
    Ok(value as u32)
}

#[derive(Clone)]
struct Entry {
    name: String,
    crc: u32,
    compressed: u32,
    size: u32,
    offset: u32,
    time: u16,
    date: u16,
    unix_time: [u8; 9],
}

fn too_large(what: &str) -> io::Error {
    io::Error::new(
        io::ErrorKind::InvalidInput,
        format!("{what} is too large for a zip without ZIP64"),
    )
}

/// A zip archive being written to `out`; `finish` writes its directory.
pub struct ZipWriter<W: Write> {
    out: W,
    written: u64,
    entries: Vec<Entry>,
}

impl<W: Write> ZipWriter<W> {
    pub fn new(out: W) -> ZipWriter<W> {
        ZipWriter {
            out,
            written: 0,
            entries: Vec::new(),
        }
    }

    fn put(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.out.write_all(bytes)?;
        self.written += bytes.len() as u64;
        Ok(())
    }

    /// Adds `data` as `name` (a plain relative path with '/' between its parts: see
    /// `plain_name`), last changed at `modified`.
    pub fn add(&mut self, name: &str, data: &[u8], modified: SystemTime) -> io::Result<()> {
        if !plain_name(name) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("{name:?} is not a plain relative path"),
            ));
        }
        if self.entries.len() + 1 >= ZIP64_MARK_16 {
            return Err(too_large("the number of entries"));
        }
        let size = fits_32(data.len() as u64, name)?;
        let offset = fits_32(self.written, "the archive")?;
        let compressed = miniz_oxide::deflate::compress_to_vec(data, 6);
        let compressed_size = fits_32(compressed.len() as u64, name)?;
        let (time, date) = dos_time(modified);
        let entry = Entry {
            name: name.to_string(),
            crc: crc32(data),
            compressed: compressed_size,
            size,
            offset,
            time,
            date,
            unix_time: unix_time_field(modified),
        };
        let mut header = Vec::with_capacity(30 + name.len() + 9);
        header.extend_from_slice(&0x0403_4b50u32.to_le_bytes());
        header.extend_from_slice(&20u16.to_le_bytes()); // version needed: 2.0, deflate
        header.extend_from_slice(&0x0800u16.to_le_bytes()); // the name is UTF-8
        header.extend_from_slice(&8u16.to_le_bytes()); // deflated
        header.extend_from_slice(&entry.time.to_le_bytes());
        header.extend_from_slice(&entry.date.to_le_bytes());
        header.extend_from_slice(&entry.crc.to_le_bytes());
        header.extend_from_slice(&entry.compressed.to_le_bytes());
        header.extend_from_slice(&entry.size.to_le_bytes());
        header.extend_from_slice(&(name.len() as u16).to_le_bytes());
        header.extend_from_slice(&(entry.unix_time.len() as u16).to_le_bytes());
        header.extend_from_slice(name.as_bytes());
        header.extend_from_slice(&entry.unix_time);
        self.put(&header)?;
        self.put(&compressed)?;
        self.entries.push(entry);
        Ok(())
    }

    /// Writes the central directory, and gives `out` back.
    pub fn finish(mut self) -> io::Result<W> {
        let start = fits_32(self.written, "the archive")?;
        let mut directory = Vec::new();
        for e in &self.entries {
            directory.extend_from_slice(&0x0201_4b50u32.to_le_bytes());
            directory.extend_from_slice(&20u16.to_le_bytes()); // made by: 2.0, MS-DOS attributes
            directory.extend_from_slice(&20u16.to_le_bytes()); // version needed
            directory.extend_from_slice(&0x0800u16.to_le_bytes());
            directory.extend_from_slice(&8u16.to_le_bytes());
            directory.extend_from_slice(&e.time.to_le_bytes());
            directory.extend_from_slice(&e.date.to_le_bytes());
            directory.extend_from_slice(&e.crc.to_le_bytes());
            directory.extend_from_slice(&e.compressed.to_le_bytes());
            directory.extend_from_slice(&e.size.to_le_bytes());
            directory.extend_from_slice(&(e.name.len() as u16).to_le_bytes());
            directory.extend_from_slice(&(e.unix_time.len() as u16).to_le_bytes());
            // comment, disk, internal and external attributes: none
            directory.extend_from_slice(&[0; 10]);
            directory.extend_from_slice(&e.offset.to_le_bytes());
            directory.extend_from_slice(e.name.as_bytes());
            directory.extend_from_slice(&e.unix_time);
        }
        let size = fits_32(directory.len() as u64, "the directory")?;
        let count = self.entries.len() as u16;
        directory.extend_from_slice(&0x0605_4b50u32.to_le_bytes());
        directory.extend_from_slice(&[0; 4]); // this disk, the directory's disk
        directory.extend_from_slice(&count.to_le_bytes());
        directory.extend_from_slice(&count.to_le_bytes());
        directory.extend_from_slice(&size.to_le_bytes());
        directory.extend_from_slice(&start.to_le_bytes());
        directory.extend_from_slice(&0u16.to_le_bytes()); // no comment
        self.put(&directory)?;
        self.out.flush()?;
        Ok(self.out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn u16_at(b: &[u8], at: usize) -> usize {
        usize::from(u16::from_le_bytes([b[at], b[at + 1]]))
    }

    fn u32_at(b: &[u8], at: usize) -> usize {
        u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]]) as usize
    }

    struct ReadBack {
        name: String,
        data: Vec<u8>,
        unix_time: usize,
    }

    /// Reads an archive back through its central directory, each entry checked against its local
    /// header (every field they share), its UTF-8 flag, its timestamps and its CRC.
    fn read_back(zip: &[u8]) -> Vec<ReadBack> {
        let end = zip.len() - 22;
        assert_eq!(u32_at(zip, end), 0x0605_4b50);
        let (count, size, start) = (
            u16_at(zip, end + 10),
            u32_at(zip, end + 12),
            u32_at(zip, end + 16),
        );
        assert_eq!(u16_at(zip, end + 8), count);
        assert_eq!(start + size, end);
        let mut at = start;
        let mut out = Vec::new();
        for _ in 0..count {
            assert_eq!(u32_at(zip, at), 0x0201_4b50);
            let (crc, compressed, size) = (
                u32_at(zip, at + 16),
                u32_at(zip, at + 20),
                u32_at(zip, at + 24),
            );
            let (name_len, extra_len) = (u16_at(zip, at + 28), u16_at(zip, at + 30));
            let local = u32_at(zip, at + 42);
            assert_eq!(u32_at(zip, local), 0x0403_4b50);
            // version needed, flags, method, time, date, CRC, sizes and the name's length
            assert_eq!(zip[local + 4..local + 28], zip[at + 6..at + 30]);
            assert_eq!(u16_at(zip, at + 8) & 0x0800, 0x0800, "UTF-8 names");
            let name = String::from_utf8(zip[at + 46..at + 46 + name_len].to_vec()).unwrap();
            assert_eq!(zip[local + 30..local + 30 + name_len], *name.as_bytes());
            let extra = &zip[at + 46 + name_len..at + 46 + name_len + extra_len];
            let local_extra_len = u16_at(zip, local + 28);
            let local_extra = &zip[local + 30 + name_len..local + 30 + name_len + local_extra_len];
            assert_eq!(extra, local_extra);
            assert_eq!(
                (u16_at(extra, 0), u16_at(extra, 2), extra[4]),
                (0x5455, 5, 1)
            );
            let data_at = local + 30 + name_len + local_extra_len;
            let data = miniz_oxide::inflate::decompress_to_vec(&zip[data_at..data_at + compressed])
                .unwrap();
            assert_eq!((data.len(), crc32(&data) as usize), (size, crc));
            out.push(ReadBack {
                name,
                data,
                unix_time: u32_at(extra, 5),
            });
            at += 46 + name_len + extra_len;
        }
        assert_eq!(at, end);
        out
    }

    #[test]
    fn crc32_is_the_standard_one() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn times_are_dos_times_within_their_range() {
        // 2026-10-14 17:05:03 UTC: seconds are kept in steps of two
        let at = |secs| UNIX_EPOCH + Duration::from_secs(secs);
        let fields = |(time, date): (u16, u16)| {
            (
                1980 + (date >> 9),
                (date >> 5) & 15,
                date & 31,
                time >> 11,
                (time >> 5) & 63,
                (time & 31) * 2,
            )
        };
        assert_eq!(
            fields(dos_time(at(1_791_997_503))),
            (2026, 10, 14, 17, 5, 2)
        );
        assert_eq!(fields(dos_time(UNIX_EPOCH)), (1980, 1, 1, 0, 0, 0));
        // 2128-06-11: past the last DOS date
        assert_eq!(
            fields(dos_time(at(4_999_999_999))),
            (2107, 12, 31, 23, 59, 58)
        );
    }

    #[test]
    fn entries_read_back_as_written() {
        let mut zip = ZipWriter::new(Vec::new());
        let log = "a line of the log\n".repeat(1000);
        let at = UNIX_EPOCH + Duration::from_secs(1_791_997_503);
        zip.add("report.txt", "überblick\n".as_bytes(), at).unwrap();
        zip.add("logs/cv1touchd.1.log", log.as_bytes(), at).unwrap();
        zip.add("empty", b"", at).unwrap();
        let bytes = zip.finish().unwrap();
        assert!(
            bytes.len() < log.len() / 10,
            "deflated: {} bytes",
            bytes.len()
        );
        let entries = read_back(&bytes);
        let names: Vec<&str> = entries.iter().map(|e| e.name.as_str()).collect();
        assert_eq!(names, ["report.txt", "logs/cv1touchd.1.log", "empty"]);
        assert_eq!(entries[0].data, "überblick\n".as_bytes());
        assert_eq!(entries[1].data, log.as_bytes());
        assert!(entries[2].data.is_empty());
        assert!(entries.iter().all(|e| e.unix_time == 1_791_997_503));
    }

    #[test]
    fn names_must_be_plain_relative_paths() {
        let mut zip = ZipWriter::new(Vec::new());
        let long = "x".repeat(usize::from(u16::MAX) + 1);
        for name in [
            "",
            "/etc/passwd",
            "../up",
            "a/../b",
            "./a",
            "a/./b",
            "a//b",
            "a\\b",
            "dir/",
            "C:/x",
            "a:b",
            "a/b ",
            "a.",
            "a\0b",
            "a\nb",
            "a<b",
            "a?b",
            "a*b",
            "a|b",
            "a\"b",
            "CON",
            "logs/nul.txt",
            "COM1",
            "lpt9.log",
            &long,
        ] {
            assert!(zip.add(name, b"x", SystemTime::now()).is_err(), "{name:?}");
        }
        assert!(read_back(&zip.finish().unwrap()).is_empty());
    }

    #[test]
    fn names_like_device_names_but_not_them_pass() {
        for name in ["console.txt", "logs/null", "COM10", "config/settings.json"] {
            assert!(plain_name(name), "{name:?}");
        }
    }

    #[test]
    fn unix_times_stop_where_signed_readers_do() {
        let far = UNIX_EPOCH + Duration::from_secs(u64::from(u32::MAX));
        let field = unix_time_field(far);
        assert_eq!(
            u32::from_le_bytes(field[5..].try_into().unwrap()),
            i32::MAX as u32
        );
    }

    /// No count, size or offset may take the value that marks a ZIP64 field.
    #[test]
    fn values_zip64_marks_are_refused() {
        assert!(fits_32(u64::from(u32::MAX) - 1, "x").is_ok());
        assert!(fits_32(u64::from(u32::MAX), "x").is_err());
        let mut zip = ZipWriter::new(Vec::new());
        let dummy = Entry {
            name: "x".into(),
            crc: 0,
            compressed: 0,
            size: 0,
            offset: 0,
            time: 0,
            date: 0,
            unix_time: [0; 9],
        };
        zip.entries = vec![dummy; ZIP64_MARK_16 - 2];
        zip.add("last", b"", SystemTime::now()).unwrap();
        assert_eq!(zip.entries.len(), ZIP64_MARK_16 - 1);
        assert!(zip.add("one more", b"", SystemTime::now()).is_err());
    }
}
