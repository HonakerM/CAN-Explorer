//! Backend-independent CAN frame representation used throughout the app.

use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    Rx,
    Tx,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct MsgKey {
    pub id: u32,
    pub ext: bool,
}

impl MsgKey {
    pub fn fmt_id(&self) -> String {
        if self.ext {
            format!("{:08X}", self.id)
        } else {
            format!("{:03X}", self.id)
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CanMsg {
    /// Seconds since the Unix epoch.
    pub ts: f64,
    pub id: u32,
    pub ext: bool,
    pub rtr: bool,
    pub fd: bool,
    pub brs: bool,
    pub dir: Dir,
    pub len: u8,
    pub data: [u8; 64],
}

impl CanMsg {
    pub fn new(id: u32, ext: bool, data: &[u8]) -> Self {
        let mut buf = [0u8; 64];
        let len = data.len().min(64);
        buf[..len].copy_from_slice(&data[..len]);
        Self {
            ts: now_ts(),
            id,
            ext,
            rtr: false,
            fd: len > 8,
            brs: false,
            dir: Dir::Rx,
            len: len as u8,
            data: buf,
        }
    }

    pub fn key(&self) -> MsgKey {
        MsgKey {
            id: self.id,
            ext: self.ext,
        }
    }

    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    pub fn data_hex(&self) -> String {
        if self.rtr {
            return "RTR".into();
        }
        hex_bytes(self.data())
    }

    /// Approximate number of bits this frame occupies on the wire, including
    /// worst-case-ish bit stuffing and inter-frame space. For FD frames with
    /// BRS this returns the nominal-bitrate-equivalent bit count, using
    /// `data_ratio` = nominal / data bitrate to scale the data phase.
    pub fn wire_bits(&self, data_ratio: f64) -> f64 {
        let payload = if self.rtr { 0.0 } else { self.len as f64 * 8.0 };
        if !self.fd {
            // SOF..CRC fields that are subject to stuffing
            let stuffed = if self.ext { 54.0 } else { 34.0 } + payload;
            // CRC delim, ACK, EOF, IFS
            stuffed + (stuffed - 1.0) / 8.0 /* avg stuffing */ + 13.0
        } else {
            let arb = if self.ext { 32.0 } else { 13.0 } + 5.0;
            let crc = if self.len > 16 { 21.0 } else { 17.0 };
            let data_phase = 4.0 + 4.0 + payload + crc + 5.0;
            let ratio = if self.brs { data_ratio } else { 1.0 };
            arb * 1.1 + data_phase * 1.1 * ratio + 13.0
        }
    }
}

pub fn hex_bytes(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len() * 3);
    for (i, b) in data.iter().enumerate() {
        if i > 0 {
            s.push(' ');
        }
        s.push_str(&format!("{b:02X}"));
    }
    s
}

pub fn now_ts() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// Parse "11 22 33", "112233", "11,22,33" style hex byte strings.
pub fn parse_hex_bytes(s: &str) -> Result<Vec<u8>, String> {
    let cleaned: String = s
        .chars()
        .filter(|c| !c.is_whitespace() && *c != ',' && *c != ':' && *c != '.')
        .collect();
    let cleaned = cleaned.trim_start_matches("0x").trim_start_matches("0X");
    if cleaned.len() % 2 != 0 {
        // Allow space-separated single nibble bytes like "1 2 3"
        let mut out = Vec::new();
        for tok in s.split(|c: char| c.is_whitespace() || c == ',') {
            if tok.is_empty() {
                continue;
            }
            out.push(
                u8::from_str_radix(tok.trim_start_matches("0x"), 16)
                    .map_err(|e| format!("'{tok}': {e}"))?,
            );
        }
        return Ok(out);
    }
    (0..cleaned.len())
        .step_by(2)
        .map(|i| {
            u8::from_str_radix(&cleaned[i..i + 2], 16)
                .map_err(|e| format!("'{}': {e}", &cleaned[i..i + 2]))
        })
        .collect()
}

pub fn parse_hex_id(s: &str) -> Result<u32, String> {
    let t = s.trim().trim_start_matches("0x").trim_start_matches("0X");
    let t = t.trim_end_matches(['h', 'H']);
    u32::from_str_radix(t, 16).map_err(|e| format!("invalid ID '{s}': {e}"))
}

/// Valid CAN FD payload lengths.
pub fn fd_len_ok(len: usize) -> bool {
    matches!(len, 0..=8 | 12 | 16 | 20 | 24 | 32 | 48 | 64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_parse() {
        assert_eq!(parse_hex_bytes("11 22 aa").unwrap(), vec![0x11, 0x22, 0xAA]);
        assert_eq!(parse_hex_bytes("1122AA").unwrap(), vec![0x11, 0x22, 0xAA]);
        assert_eq!(parse_hex_bytes("1 2 3").unwrap(), vec![1, 2, 3]);
        assert_eq!(parse_hex_bytes("").unwrap(), Vec::<u8>::new());
        assert_eq!(parse_hex_id("0x7ff").unwrap(), 0x7FF);
        assert_eq!(parse_hex_id("123h").unwrap(), 0x123);
    }
}
