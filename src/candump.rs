//! Reading and writing Linux `candump -L` style log files.
//!
//! Log line format: `(1695658000.123456) can0 123#DEADBEEF`
//!   * extended IDs use 8 hex digits: `12345678#00`
//!   * remote frames: `123#R` or `123#R4`
//!   * CAN FD: `123##<flags nibble><data>` (flag 1 = BRS, 2 = ESI)
//!
//! Files written here can be replayed with can-utils `canplayer` and vice versa.

use std::fmt::Write as _;
use std::path::Path;

use crate::msg::{CanMsg, Dir};

pub fn format_line(msg: &CanMsg, iface: &str) -> String {
    let mut s = String::with_capacity(48 + msg.len as usize * 2);
    let _ = write!(s, "({:.6}) {} ", msg.ts, iface);
    if msg.ext {
        let _ = write!(s, "{:08X}", msg.id);
    } else {
        let _ = write!(s, "{:03X}", msg.id);
    }
    if msg.fd {
        let flags = if msg.brs { 1 } else { 0 };
        let _ = write!(s, "##{flags:X}");
    } else {
        s.push('#');
        if msg.rtr {
            s.push('R');
            if msg.len > 0 {
                let _ = write!(s, "{}", msg.len);
            }
            return s;
        }
    }
    for b in msg.data() {
        let _ = write!(s, "{b:02X}");
    }
    s
}

/// Parses a single log line. Returns `Ok(None)` for blank/comment lines.
pub fn parse_line(line: &str) -> Result<Option<CanMsg>, String> {
    let line = line.trim();
    if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
        return Ok(None);
    }
    let mut tokens = line.split_whitespace().peekable();
    let mut ts = None;
    if let Some(t) = tokens.peek()
        && t.starts_with('(')
    {
        let t = tokens.next().unwrap();
        let inner = t.trim_start_matches('(').trim_end_matches(')');
        ts = Some(
            inner
                .parse::<f64>()
                .map_err(|e| format!("bad timestamp '{inner}': {e}"))?,
        );
    }
    let rest: Vec<&str> = tokens.collect();
    // Find the frame token (contains '#'); the interface name precedes it.
    let mut msg = if let Some(frame_tok) = rest.iter().find(|t| t.contains('#')) {
        parse_frame_token(frame_tok)?
    } else {
        parse_screen_format(&rest)?
    };
    msg.ts = ts.unwrap_or(0.0);
    msg.dir = Dir::Rx;
    Ok(Some(msg))
}

fn parse_frame_token(tok: &str) -> Result<CanMsg, String> {
    let (id_str, rest) = tok.split_once('#').ok_or("missing '#'")?;
    let id = u32::from_str_radix(id_str, 16).map_err(|e| format!("bad id '{id_str}': {e}"))?;
    let ext = id_str.len() > 3;
    if ext && id & 0x2000_0000 != 0 {
        return Err("error frame".into());
    }
    let id = id & 0x1FFF_FFFF;

    if let Some(fd_rest) = rest.strip_prefix('#') {
        let mut chars = fd_rest.chars();
        let flags = chars
            .next()
            .and_then(|c| c.to_digit(16))
            .ok_or("missing FD flags")?;
        let data = decode_hex(chars.as_str())?;
        if data.len() > 64 {
            return Err("FD payload > 64 bytes".into());
        }
        let mut m = CanMsg::new(id, ext, &data);
        m.fd = true;
        m.brs = flags & 1 != 0;
        return Ok(m);
    }
    if let Some(r) = rest.strip_prefix(['R', 'r']) {
        let mut m = CanMsg::new(id, ext, &[]);
        m.rtr = true;
        m.len = r.parse::<u8>().unwrap_or(0).min(8);
        return Ok(m);
    }
    let data = decode_hex(&rest.replace('.', ""))?;
    if data.len() > 8 {
        return Err("classic payload > 8 bytes".into());
    }
    Ok(CanMsg::new(id, ext, &data))
}

/// `can0  123   [4]  11 22 33 44` (candump default output)
fn parse_screen_format(tokens: &[&str]) -> Result<CanMsg, String> {
    let lb = tokens
        .iter()
        .position(|t| t.starts_with('['))
        .ok_or("unrecognized line")?;
    if lb == 0 {
        return Err("missing id".into());
    }
    let id_str = tokens[lb - 1];
    let id = u32::from_str_radix(id_str, 16).map_err(|e| format!("bad id '{id_str}': {e}"))?;
    let ext = id_str.len() > 3;
    let n: usize = tokens[lb]
        .trim_matches(['[', ']'])
        .parse()
        .map_err(|_| "bad length")?;
    let rest = &tokens[lb + 1..];
    if rest
        .first()
        .is_some_and(|t| t.eq_ignore_ascii_case("remote"))
    {
        let mut m = CanMsg::new(id, ext, &[]);
        m.rtr = true;
        m.len = n.min(8) as u8;
        return Ok(m);
    }
    let data: Result<Vec<u8>, _> = rest
        .iter()
        .take(n)
        .map(|t| u8::from_str_radix(t, 16))
        .collect();
    let data = data.map_err(|e| format!("bad data: {e}"))?;
    Ok(CanMsg::new(id, ext, &data))
}

fn decode_hex(s: &str) -> Result<Vec<u8>, String> {
    if s.len() % 2 != 0 {
        return Err(format!("odd-length hex '{s}'"));
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| format!("bad hex '{s}': {e}")))
        .collect()
}

/// A log loaded for transmission/replay. Timestamps are relative to the first frame.
pub struct LoadedLog {
    pub frames: Vec<CanMsg>,
    pub skipped: usize,
}

impl LoadedLog {
    pub fn duration(&self) -> f64 {
        self.frames.last().map(|m| m.ts).unwrap_or(0.0)
    }
}

pub fn load_file(path: &Path) -> anyhow::Result<LoadedLog> {
    let text = std::fs::read_to_string(path)?;
    let mut frames = Vec::new();
    let mut skipped = 0;
    let mut t0 = None;
    let mut last = 0.0f64;
    for line in text.lines() {
        match parse_line(line) {
            Ok(Some(mut m)) => {
                // Lines without timestamps are spaced 1ms apart.
                let ts = if m.ts == 0.0 { last + 0.001 } else { m.ts };
                let base = *t0.get_or_insert(ts);
                m.ts = (ts - base).max(0.0);
                last = ts;
                frames.push(m);
            }
            Ok(None) => {}
            Err(_) => skipped += 1,
        }
    }
    Ok(LoadedLog { frames, skipped })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let lines = [
            "(1695658000.123456) can0 123#DEADBEEF",
            "(1695658000.123456) can0 12345678#0102",
            "(1695658000.123456) can0 123#R",
            "(1695658000.123456) can0 123#R4",
            "(1695658000.123456) can0 123##1000102030405060708090A0B",
            "(1695658000.123456) can0 7FF#",
        ];
        for l in lines {
            let m = parse_line(l).unwrap().unwrap();
            assert_eq!(format_line(&m, "can0"), l, "roundtrip of {l}");
        }
    }

    #[test]
    fn screen_format() {
        let m = parse_line("  can0  1F334455   [4]  11 22 33 44")
            .unwrap()
            .unwrap();
        assert_eq!(m.id, 0x1F334455);
        assert!(m.ext);
        assert_eq!(m.data(), &[0x11, 0x22, 0x33, 0x44]);
    }
}
