// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Bounded assembly of lines from a byte stream. Once a line is too long its remaining bytes
//! are discarded through the newline; the next line always starts at a real line boundary.

pub(crate) const LINE_BYTES: usize = 8192;
pub(crate) const READ_BYTES: usize = 64 * 1024;

#[derive(Default)]
pub(crate) struct LineBuffer {
    bytes: Vec<u8>,
    truncated: bool,
}

impl LineBuffer {
    pub(crate) fn push(&mut self, bytes: &[u8], mut line: impl FnMut(&[u8], bool)) {
        for part in bytes.split_inclusive(|&b| b == b'\n') {
            let ended = part.last() == Some(&b'\n');
            let text = if ended { &part[..part.len() - 1] } else { part };
            let n = text.len().min(LINE_BYTES - self.bytes.len());
            self.bytes.extend_from_slice(&text[..n]);
            self.truncated |= n < text.len();
            if ended {
                line(&self.bytes, self.truncated);
                self.bytes.clear();
                self.truncated = false;
            }
        }
    }

    pub(crate) fn finish(&mut self, mut line: impl FnMut(&[u8], bool)) {
        if !self.bytes.is_empty() || self.truncated {
            line(&self.bytes, self.truncated);
        }
        self.bytes.clear();
        self.truncated = false;
    }

    pub(crate) fn is_truncated(&self) -> bool {
        self.truncated
    }

    #[cfg(test)]
    pub(crate) fn len(&self) -> usize {
        self.bytes.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_line_stays_bounded_across_reads_and_the_next_one_is_whole() {
        let mut lines = LineBuffer::default();
        let mut got = Vec::new();
        let mut take = |text: &[u8], truncated| got.push((text.to_vec(), truncated));
        lines.push(&[b'x'; LINE_BYTES], &mut take);
        lines.push(b"overflow", &mut take);
        assert_eq!(lines.len(), LINE_BYTES);
        lines.push(b"\nnext\r\nlast", &mut take);
        lines.finish(take);
        assert_eq!(
            got,
            [
                (vec![b'x'; LINE_BYTES], true),
                (b"next\r".to_vec(), false),
                (b"last".to_vec(), false)
            ]
        );
    }

    #[test]
    fn a_split_utf8_character_is_kept_until_its_line_ends() {
        let mut lines = LineBuffer::default();
        let mut got = Vec::new();
        for byte in "é\n".as_bytes() {
            lines.push(&[*byte], |text, truncated| {
                got.push((text.to_vec(), truncated))
            });
        }
        assert_eq!(got, [("é".as_bytes().to_vec(), false)]);
    }
}
