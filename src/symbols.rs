//! Symbol databases: map CAN IDs to message names and decode signals.
//!
//! Supported formats:
//!   * Vector DBC (`.dbc`) via the `can-dbc` crate
//!   * PEAK PCAN Symbol files (`.sym`, format version 5/6) via a built-in parser
//!
//! Internally all signal start bits use the DBC convention: for Intel
//! (little-endian) signals it is the LSB position, for Motorola (big-endian)
//! signals it is the MSB position in "sawtooth" bit numbering.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, anyhow, bail};

use crate::msg::{CanMsg, MsgKey};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SigKind {
    Int,
    Float32,
    Float64,
}

#[derive(Debug, Clone)]
pub struct SignalDef {
    pub name: String,
    pub start_bit: u32,
    pub size: u32,
    pub little_endian: bool,
    pub signed: bool,
    pub kind: SigKind,
    pub factor: f64,
    pub offset: f64,
    pub unit: String,
    /// Only decoded when the message's multiplexor equals this value.
    pub mux: Option<u64>,
    pub values: HashMap<i64, String>,
    /// Fixed-width display format, computed by [`SignalDef::finalize`].
    pub fmt: ValueFormat,
}

impl SignalDef {
    fn plain(name: String, start_bit: u32, size: u32, little_endian: bool) -> Self {
        Self {
            name,
            start_bit,
            size,
            little_endian,
            signed: false,
            kind: SigKind::Int,
            factor: 1.0,
            offset: 0.0,
            unit: String::new(),
            mux: None,
            values: HashMap::new(),
            fmt: ValueFormat::default(),
        }
    }

    /// Computes the display format once all fields are known.
    fn finalize(&mut self) {
        self.fmt = ValueFormat::for_signal(self);
    }

    /// Extracts the raw (un-scaled) integer bits of this signal.
    pub fn raw(&self, data: &[u8]) -> Option<u64> {
        extract_bits(data, self.start_bit, self.size, self.little_endian)
    }

    pub fn decode(&self, data: &[u8]) -> Option<DecodedSignal> {
        let raw = self.raw(data)?;
        let (value, int_val) = match self.kind {
            SigKind::Float32 if self.size == 32 => (f32::from_bits(raw as u32) as f64, None),
            SigKind::Float64 if self.size == 64 => (f64::from_bits(raw), None),
            _ => {
                let v = if self.signed {
                    sign_extend(raw, self.size)
                } else {
                    raw as i64
                };
                (if self.signed { v as f64 } else { raw as f64 }, Some(v))
            }
        };
        let phys = value * self.factor + self.offset;
        let text = int_val.and_then(|v| self.values.get(&v).cloned());
        Some(DecodedSignal {
            name: self.name.clone(),
            raw,
            value: phys,
            unit: self.unit.clone(),
            text,
            fmt: self.fmt,
        })
    }
}

#[derive(Debug, Clone)]
pub struct DecodedSignal {
    pub name: String,
    pub raw: u64,
    pub value: f64,
    pub unit: String,
    pub text: Option<String>,
    pub fmt: ValueFormat,
}

impl DecodedSignal {
    /// Fixed-width value (plus value-table text, if any), e.g. `  5354.00`.
    pub fn display_value(&self) -> String {
        self.fmt.value(self.value, self.text.as_deref())
    }
}

/// Fixed-width numeric format for a signal, derived from its definition so
/// the rendered width never changes while values change (no flicker).
///
/// * `decimals` comes from the factor/offset (0.25 -> 2, 0.1 -> 1, capped at 4)
/// * `width` fits the largest physical value the signal's bits can encode
/// * `text_width` fits the longest value-table entry (0 if there is none)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ValueFormat {
    pub width: usize,
    pub decimals: usize,
    pub text_width: usize,
}

impl ValueFormat {
    const MAX_DECIMALS: usize = 4;

    pub fn for_signal(sig: &SignalDef) -> Self {
        let text_width = sig
            .values
            .values()
            .map(|t| t.chars().count())
            .max()
            .unwrap_or(0);
        if sig.kind != SigKind::Int {
            // IEEE floats: range isn't meaningful, use a generous fixed width.
            return Self {
                width: 14,
                decimals: Self::MAX_DECIMALS,
                text_width,
            };
        }
        let decimals = decimals_for(sig.factor).max(decimals_for(sig.offset));
        let bits = sig.size.clamp(1, 64) as i32;
        let (raw_lo, raw_hi) = if sig.signed {
            (-(2f64.powi(bits - 1)), 2f64.powi(bits - 1) - 1.0)
        } else {
            (0.0, 2f64.powi(bits) - 1.0)
        };
        let a = raw_lo * sig.factor + sig.offset;
        let b = raw_hi * sig.factor + sig.offset;
        let neg = a.min(b) < 0.0;
        // Width of the widest endpoint once rounded to `decimals`.
        let widest = [a, b]
            .iter()
            .map(|v| format!("{:.*}", decimals, v.abs()).len())
            .max()
            .unwrap_or(1);
        Self {
            width: (widest + neg as usize).min(24),
            decimals,
            text_width,
        }
    }

    /// Characters used by the fraction, including the '.' (0 for integers).
    pub fn frac_width(&self) -> usize {
        if self.decimals > 0 {
            self.decimals + 1
        } else {
            0
        }
    }

    /// Characters used by the sign and integer digits.
    pub fn int_width(&self) -> usize {
        self.width.saturating_sub(self.frac_width())
    }

    /// Right-aligned number, e.g. `  89.00`.
    pub fn number(&self, v: f64) -> String {
        format!("{:>w$.d$}", v, w = self.width, d = self.decimals)
    }

    /// Number followed by left-aligned value-table text (if the signal has one).
    pub fn value(&self, v: f64, text: Option<&str>) -> String {
        let n = self.number(v);
        if self.text_width > 0 {
            format!("{n} {:<tw$}", text.unwrap_or(""), tw = self.text_width)
        } else {
            n
        }
    }
}

/// Layout shared by several signals shown in one column: integer parts are
/// right-aligned, decimal points line up, and value-table text gets its own
/// column. Every rendered value has the same length, so units that follow
/// also line up.
///
/// ```text
///  1000.00       <- RPM
///    89          <- CoolantTemp
///     3    D1    <- Gear
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ValueColumn {
    pub int_width: usize,
    pub frac_width: usize,
    pub text_width: usize,
}

impl ValueColumn {
    /// Smallest column that fits every one of `fmts`. Because formats come
    /// from the definitions, not the data, the result is stable over time.
    pub fn fit(fmts: impl IntoIterator<Item = ValueFormat>) -> Self {
        fmts.into_iter().fold(Self::default(), |c, f| Self {
            int_width: c.int_width.max(f.int_width()),
            frac_width: c.frac_width.max(f.frac_width()),
            text_width: c.text_width.max(f.text_width),
        })
    }

    /// Same layout without the value-table text column (for min/max).
    pub fn numbers_only(self) -> Self {
        Self {
            text_width: 0,
            ..self
        }
    }

    /// Renders `v` with the signal's own number of decimals, aligned on the
    /// decimal point within this column.
    pub fn render(&self, fmt: &ValueFormat, v: f64, text: Option<&str>) -> String {
        let n = format!("{:.*}", fmt.decimals, v);
        let (int, frac) = n.split_at(n.find('.').unwrap_or(n.len()));
        let mut out = format!(
            "{int:>iw$}{frac:<fw$}",
            iw = self.int_width,
            fw = self.frac_width
        );
        if self.text_width > 0 {
            out.push(' ');
            out.push_str(&format!(
                "{:<tw$}",
                text.unwrap_or(""),
                tw = self.text_width
            ));
        }
        out
    }
}

/// Smallest number of decimals (<= 4) that represents `x` exactly.
fn decimals_for(x: f64) -> usize {
    for d in 0..ValueFormat::MAX_DECIMALS {
        let scaled = x * 10f64.powi(d as i32);
        if (scaled - scaled.round()).abs() < 1e-9 * scaled.abs().max(1.0) {
            return d;
        }
    }
    ValueFormat::MAX_DECIMALS
}

/// One-line summary with fixed-width values (for dense views like the stream):
/// `RPM  5354.00 rpm | CoolantTemp  89 degC`.
pub fn format_compact(sigs: &[DecodedSignal]) -> String {
    let mut out = String::new();
    for (i, s) in sigs.iter().enumerate() {
        if i > 0 {
            out.push_str(" | ");
        }
        out.push_str(&s.name);
        out.push(' ');
        out.push_str(&s.display_value());
        if !s.unit.is_empty() {
            out.push(' ');
            out.push_str(&s.unit);
        }
    }
    out
}

#[derive(Debug, Clone)]
pub struct MessageDef {
    pub name: String,
    pub key: MsgKey,
    pub dlc: u8,
    pub mux: Option<SignalDef>,
    /// For SYM files each multiplexed variant has its own block name.
    pub mux_names: HashMap<u64, String>,
    pub signals: Vec<SignalDef>,
    pub comment: Option<String>,
    /// Most signals decoded at once (for multiplexed messages: across all
    /// multiplexor values). Views reserve this many lines so the height of a
    /// message never changes.
    pub max_lines: usize,
    /// Longest signal name, for aligning values into a column.
    pub name_width: usize,
    /// Shared value layout for all of this message's signals.
    pub value_column: ValueColumn,
}

impl MessageDef {
    pub fn decode(&self, data: &[u8]) -> Vec<DecodedSignal> {
        let mut out = Vec::with_capacity(self.signals.len() + 1);
        let mux_val = match &self.mux {
            Some(m) => {
                let v = m.raw(data);
                if let Some(d) = m.decode(data) {
                    out.push(d);
                }
                v
            }
            None => None,
        };
        for s in &self.signals {
            if let Some(mv) = s.mux
                && mux_val != Some(mv)
            {
                continue;
            }
            if let Some(d) = s.decode(data) {
                out.push(d);
            }
        }
        out
    }

    fn finalize(&mut self) {
        if let Some(m) = &mut self.mux {
            m.finalize();
        }
        for s in &mut self.signals {
            s.finalize();
        }
        let plain = self.signals.iter().filter(|s| s.mux.is_none()).count();
        let mut per_mux: HashMap<u64, usize> = HashMap::new();
        for s in &self.signals {
            if let Some(v) = s.mux {
                *per_mux.entry(v).or_default() += 1;
            }
        }
        self.max_lines =
            self.mux.is_some() as usize + plain + per_mux.values().copied().max().unwrap_or(0);
        self.name_width = self
            .mux
            .iter()
            .chain(&self.signals)
            .map(|s| s.name.chars().count())
            .max()
            .unwrap_or(0);
        self.value_column = ValueColumn::fit(self.mux.iter().chain(&self.signals).map(|s| s.fmt));
    }

    /// Decodes `data` into exactly [`MessageDef::max_lines`] lines of
    /// `name  value unit` (padded with blank lines). Names, decimal points,
    /// value-table text and units each line up in their own column.
    pub fn decode_lines(&self, data: &[u8]) -> Vec<String> {
        let col = self.value_column;
        let mut lines: Vec<String> = self
            .decode(data)
            .iter()
            .map(|d| {
                let mut l = format!(
                    "{:<nw$}  {}",
                    d.name,
                    col.render(&d.fmt, d.value, d.text.as_deref()),
                    nw = self.name_width
                );
                if !d.unit.is_empty() {
                    l.push(' ');
                    l.push_str(&d.unit);
                }
                l
            })
            .collect();
        lines.resize(self.max_lines.max(lines.len()), String::new());
        lines
    }

    pub fn name_for(&self, data: &[u8]) -> &str {
        if !self.mux_names.is_empty()
            && let Some(v) = self.mux.as_ref().and_then(|m| m.raw(data))
            && let Some(n) = self.mux_names.get(&v)
        {
            return n;
        }
        &self.name
    }
}

#[derive(Debug, Default)]
pub struct SymbolDb {
    pub source: Option<PathBuf>,
    pub messages: HashMap<MsgKey, MessageDef>,
}

impl SymbolDb {
    pub fn load(path: &Path) -> anyhow::Result<Self> {
        let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        let text = String::from_utf8(bytes.clone())
            .unwrap_or_else(|_| bytes.iter().map(|&b| b as char).collect()); // latin-1/cp1252 fallback
        let ext = path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();
        let looks_sym = text.trim_start().starts_with("FormatVersion");
        let mut db = if ext == "sym" || looks_sym {
            parse_sym(&text)?
        } else {
            parse_dbc(&text)?
        };
        db.source = Some(path.to_path_buf());
        Ok(db)
    }

    pub fn get(&self, key: MsgKey) -> Option<&MessageDef> {
        self.messages.get(&key)
    }

    pub fn name(&self, msg: &CanMsg) -> Option<&str> {
        self.messages
            .get(&msg.key())
            .map(|m| m.name_for(msg.data()))
    }
}

// ---------------------------------------------------------------------------
// Bit extraction
// ---------------------------------------------------------------------------

pub fn extract_bits(data: &[u8], start: u32, size: u32, little_endian: bool) -> Option<u64> {
    if size == 0 || size > 64 {
        return None;
    }
    let mut v: u64 = 0;
    if little_endian {
        for i in 0..size {
            let bit = start + i;
            let byte = *data.get((bit / 8) as usize)?;
            v |= (((byte >> (bit % 8)) & 1) as u64) << i;
        }
    } else {
        let mut pos = start;
        for _ in 0..size {
            let byte = *data.get((pos / 8) as usize)?;
            v = (v << 1) | ((byte >> (pos % 8)) & 1) as u64;
            if pos % 8 == 0 {
                pos += 15;
            } else {
                pos -= 1;
            }
        }
    }
    Some(v)
}

fn sign_extend(v: u64, size: u32) -> i64 {
    if size >= 64 {
        return v as i64;
    }
    let shift = 64 - size;
    ((v << shift) as i64) >> shift
}

// ---------------------------------------------------------------------------
// DBC
// ---------------------------------------------------------------------------

fn parse_dbc(text: &str) -> anyhow::Result<SymbolDb> {
    use can_dbc::{ByteOrder, MessageId, MultiplexIndicator, SignalExtendedValueType, ValueType};

    let dbc = can_dbc::Dbc::try_from(text).map_err(|e| anyhow!("DBC parse error: {e:?}"))?;
    let mut db = SymbolDb::default();
    for m in &dbc.messages {
        let key = match m.id {
            MessageId::Standard(id) => MsgKey {
                id: id as u32,
                ext: false,
            },
            MessageId::Extended(id) => MsgKey { id, ext: true },
        };
        if key.id > 0x1FFF_FFFF || (!key.ext && key.id > 0x7FF) {
            continue; // e.g. VECTOR__INDEPENDENT_SIG_MSG
        }
        let mut def = MessageDef {
            name: m.name.clone(),
            key,
            dlc: m.size.min(64) as u8,
            mux: None,
            mux_names: HashMap::new(),
            signals: Vec::new(),
            comment: dbc.message_comment(m.id).map(str::to_string),
            max_lines: 0,
            name_width: 0,
            value_column: ValueColumn::default(),
        };
        for s in &m.signals {
            let mut sd = SignalDef::plain(
                s.name.clone(),
                s.start_bit as u32,
                s.size as u32,
                s.byte_order == ByteOrder::LittleEndian,
            );
            sd.signed = s.value_type == ValueType::Signed;
            sd.factor = s.factor;
            sd.offset = s.offset;
            sd.unit = s.unit.clone();
            sd.kind = match dbc.extended_value_type_for_signal(m.id, &s.name) {
                Some(SignalExtendedValueType::IEEEfloat32Bit) => SigKind::Float32,
                Some(SignalExtendedValueType::IEEEdouble64bit) => SigKind::Float64,
                _ => SigKind::Int,
            };
            if let Some(vals) = dbc.value_descriptions_for_signal(m.id, &s.name) {
                sd.values = vals.iter().map(|v| (v.id, v.description.clone())).collect();
            }
            match s.multiplexer_indicator {
                MultiplexIndicator::Multiplexor => {
                    def.mux = Some(sd);
                    continue;
                }
                MultiplexIndicator::MultiplexedSignal(v)
                | MultiplexIndicator::MultiplexorAndMultiplexedSignal(v) => sd.mux = Some(v),
                MultiplexIndicator::Plain => {}
            }
            def.signals.push(sd);
        }
        def.finalize();
        db.messages.insert(key, def);
    }
    Ok(db)
}

// ---------------------------------------------------------------------------
// PCAN SYM
// ---------------------------------------------------------------------------

/// Splits on whitespace, keeping "quoted strings" together (quotes removed).
fn tokenize(s: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut in_q = false;
    let mut has_tok = false;
    for c in s.chars() {
        match c {
            '"' => {
                in_q = !in_q;
                has_tok = true;
            }
            c if c.is_whitespace() && !in_q => {
                if has_tok {
                    out.push(std::mem::take(&mut cur));
                    has_tok = false;
                }
            }
            c => {
                cur.push(c);
                has_tok = true;
            }
        }
    }
    if has_tok {
        out.push(cur);
    }
    out
}

fn strip_comment(line: &str) -> &str {
    let mut in_q = false;
    let b = line.as_bytes();
    for i in 0..b.len() {
        match b[i] {
            b'"' => in_q = !in_q,
            b'/' if !in_q && b.get(i + 1) == Some(&b'/') => return &line[..i],
            _ => {}
        }
    }
    line
}

fn parse_sym_num(s: &str) -> anyhow::Result<u64> {
    let s = s.trim();
    if let Some(h) = s.strip_suffix(['h', 'H']) {
        Ok(u64::from_str_radix(h, 16)?)
    } else if let Some(h) = s.strip_prefix("0x") {
        Ok(u64::from_str_radix(h, 16)?)
    } else if let Some(b) = s.strip_suffix(['b', 'B']) {
        Ok(u64::from_str_radix(b, 2)?)
    } else {
        Ok(s.parse()?)
    }
}

/// Converts a SYM Motorola start bit into the DBC sawtooth MSB convention.
fn sym_motorola_start(start: u32) -> u32 {
    8 * (start / 8) + (7 - start % 8)
}

struct SymSigTemplate {
    type_name: String,
    size: u32,
    opts: Vec<String>,
}

fn apply_sym_type(sd: &mut SignalDef, type_name: &str) {
    match type_name.to_ascii_lowercase().as_str() {
        "signed" => sd.signed = true,
        "float" => sd.kind = SigKind::Float32,
        "double" => sd.kind = SigKind::Float64,
        _ => {}
    }
}

fn apply_sym_opts(
    sd: &mut SignalDef,
    opts: &[String],
    enums: &HashMap<String, HashMap<i64, String>>,
) {
    for o in opts {
        let lower = o.to_ascii_lowercase();
        if lower == "-m" {
            if sd.little_endian {
                sd.little_endian = false;
                sd.start_bit = sym_motorola_start(sd.start_bit);
            }
        } else if let Some(v) = o.strip_prefix("/u:") {
            sd.unit = v.to_string();
        } else if let Some(v) = o.strip_prefix("/f:") {
            sd.factor = v.parse().unwrap_or(1.0);
        } else if let Some(v) = o.strip_prefix("/o:") {
            sd.offset = v.parse().unwrap_or(0.0);
        } else if let Some(v) = o.strip_prefix("/e:")
            && let Some(e) = enums.get(v)
        {
            sd.values = e.clone();
        }
    }
}

fn parse_sym_enum(line: &str) -> Option<(String, HashMap<i64, String>)> {
    // enum Name(0="Off", 1="On")
    let rest = line.trim().strip_prefix("enum")?.trim();
    let (name, body) = rest.split_once('(')?;
    let body = body.trim_end().trim_end_matches(')');
    let mut map = HashMap::new();
    let mut remaining = body;
    while let Some((k, after)) = remaining.split_once('=') {
        let k = k.trim().trim_start_matches(',').trim();
        let after = after.trim_start();
        let (val, next) = if let Some(q) = after.strip_prefix('"') {
            let end = q.find('"').unwrap_or(q.len());
            (q[..end].to_string(), &q[(end + 1).min(q.len())..])
        } else {
            let end = after.find(',').unwrap_or(after.len());
            (after[..end].trim().to_string(), &after[end..])
        };
        if let Ok(k) = parse_sym_num(k) {
            map.insert(k as i64, val);
        } else if let Ok(k) = k.parse::<i64>() {
            map.insert(k, val);
        }
        remaining = next;
    }
    Some((name.trim().to_string(), map))
}

fn parse_sym(text: &str) -> anyhow::Result<SymbolDb> {
    let mut enums: HashMap<String, HashMap<i64, String>> = HashMap::new();
    let mut sig_templates: HashMap<String, SymSigTemplate> = HashMap::new();

    // First pass: enums (may span multiple lines) and {SIGNALS} templates.
    let mut section = String::new();
    let mut pending_enum = String::new();
    for raw in text.lines() {
        let line = strip_comment(raw).trim();
        if line.starts_with('{') {
            section = line.to_ascii_uppercase();
            continue;
        }
        if section == "{ENUMS}" {
            if line.starts_with("enum") || !pending_enum.is_empty() {
                pending_enum.push_str(line);
                pending_enum.push(' ');
                if pending_enum.trim_end().ends_with(')') {
                    if let Some((n, m)) = parse_sym_enum(&pending_enum) {
                        enums.insert(n, m);
                    }
                    pending_enum.clear();
                }
            }
        } else if section == "{SIGNALS}"
            && let Some(rest) = line.strip_prefix("Sig=")
        {
            let toks = tokenize(rest);
            if toks.len() >= 3 {
                sig_templates.insert(
                    toks[0].clone(),
                    SymSigTemplate {
                        type_name: toks[1].clone(),
                        size: toks[2].parse().unwrap_or(8),
                        opts: toks[3..].to_vec(),
                    },
                );
            }
        }
    }

    // Second pass: message blocks.
    #[derive(Default)]
    struct Block {
        name: String,
        id: Option<u32>,
        ext: bool,
        dlc: u8,
        mux: Option<(SignalDef, u64)>,
        signals: Vec<SignalDef>,
    }
    let mut blocks: Vec<Block> = Vec::new();
    let mut in_msgs = false;
    for (lineno, raw) in text.lines().enumerate() {
        let line = strip_comment(raw).trim();
        if line.is_empty() {
            continue;
        }
        if line.starts_with('{') {
            let s = line.to_ascii_uppercase();
            in_msgs = matches!(s.as_str(), "{SEND}" | "{RECEIVE}" | "{SENDRECEIVE}");
            continue;
        }
        if !in_msgs {
            continue;
        }
        if line.starts_with('[') && line.ends_with(']') {
            blocks.push(Block {
                name: line[1..line.len() - 1].to_string(),
                dlc: 8,
                ..Default::default()
            });
            continue;
        }
        let Some(b) = blocks.last_mut() else { continue };
        let Some((k, v)) = line.split_once('=') else {
            continue;
        };
        let v = v.trim();
        let ctx = || format!("line {}: {}", lineno + 1, raw.trim());
        match k.trim() {
            "ID" => {
                let first = v.split('-').next().unwrap_or(v);
                b.id = Some(parse_sym_num(first).with_context(ctx)? as u32);
            }
            "Type" => b.ext = v.eq_ignore_ascii_case("extended"),
            "DLC" | "Len" => b.dlc = v.parse().unwrap_or(8),
            "Var" | "Mux" | "Sig" => {
                let toks = tokenize(v);
                let is_mux = k.trim() == "Mux";
                let is_sig = k.trim() == "Sig";
                if toks.len() < 2 {
                    bail!("{}: too few fields", ctx());
                }
                let name = toks[0].clone();
                let (sd, opts_start, mux_val) = if is_sig {
                    let t = sig_templates
                        .get(&name)
                        .ok_or_else(|| anyhow!("{}: unknown signal '{name}'", ctx()))?;
                    let start: u32 = toks[1].parse().with_context(ctx)?;
                    let mut sd = SignalDef::plain(name, start, t.size, true);
                    apply_sym_type(&mut sd, &t.type_name);
                    apply_sym_opts(&mut sd, &t.opts, &enums);
                    (sd, 2, None)
                } else if is_mux {
                    // Mux=Name start,len value [-m] [-t]
                    let (start, len) = toks[1]
                        .split_once(',')
                        .ok_or_else(|| anyhow!("{}: bad bit range", ctx()))?;
                    let sd = SignalDef::plain(
                        name,
                        start.parse().with_context(ctx)?,
                        len.parse().with_context(ctx)?,
                        true,
                    );
                    let val = toks
                        .get(2)
                        .map(|s| parse_sym_num(s))
                        .transpose()
                        .with_context(ctx)?
                        .unwrap_or(0);
                    (sd, 3, Some(val))
                } else {
                    // Var=Name type start,len [opts]
                    if toks.len() < 3 {
                        bail!("{}: too few fields", ctx());
                    }
                    let (start, len) = toks[2]
                        .split_once(',')
                        .ok_or_else(|| anyhow!("{}: bad bit range", ctx()))?;
                    let mut sd = SignalDef::plain(
                        name,
                        start.parse().with_context(ctx)?,
                        len.parse().with_context(ctx)?,
                        true,
                    );
                    apply_sym_type(&mut sd, &toks[1]);
                    (sd, 3, None)
                };
                let mut sd = sd;
                apply_sym_opts(&mut sd, toks.get(opts_start..).unwrap_or(&[]), &enums);
                if let Some(v) = mux_val {
                    b.mux = Some((sd, v));
                } else {
                    b.signals.push(sd);
                }
            }
            _ => {}
        }
    }

    let mut db = SymbolDb::default();
    for b in blocks {
        let Some(id) = b.id else { continue };
        let ext = b.ext || id > 0x7FF;
        let key = MsgKey { id, ext };
        let def = db.messages.entry(key).or_insert_with(|| MessageDef {
            name: b.name.clone(),
            key,
            dlc: b.dlc,
            mux: None,
            mux_names: HashMap::new(),
            signals: Vec::new(),
            comment: None,
            max_lines: 0,
            name_width: 0,
            value_column: ValueColumn::default(),
        });
        match b.mux {
            Some((mux_sig, val)) => {
                if def.mux.is_none() {
                    def.mux = Some(mux_sig);
                }
                def.mux_names.insert(val, b.name.clone());
                for mut s in b.signals {
                    s.mux = Some(val);
                    def.signals.push(s);
                }
            }
            None => def.signals.extend(b.signals),
        }
    }
    for def in db.messages.values_mut() {
        def.finalize();
    }
    Ok(db)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intel_and_motorola_extraction() {
        let data = [0x34, 0x12, 0, 0, 0, 0, 0, 0];
        assert_eq!(extract_bits(&data, 0, 16, true), Some(0x1234));
        // Motorola 16-bit signal with MSB at bit 7 of byte 0 (DBC start 7)
        assert_eq!(extract_bits(&data, 7, 16, false), Some(0x3412));
        assert_eq!(extract_bits(&data, 4, 4, true), Some(0x3));
        assert_eq!(sign_extend(0xFF, 8), -1);
    }

    #[test]
    fn dbc_decode() {
        let dbc = r#"VERSION ""

NS_ :

BS_:

BU_: ECU

BO_ 256 Engine: 8 ECU
 SG_ RPM : 0|16@1+ (0.25,0) [0|16383] "rpm" Vector__XXX
 SG_ Temp : 16|8@1- (1,-40) [-40|215] "C" Vector__XXX
 SG_ Gear : 31|4@0+ (1,0) [0|8] "" Vector__XXX

VAL_ 256 Gear 0 "P" 1 "R" 2 "N" ;
"#;
        let db = parse_dbc(dbc).unwrap();
        let m = db
            .get(MsgKey {
                id: 256,
                ext: false,
            })
            .unwrap();
        assert_eq!(m.name, "Engine");
        let d = m.decode(&[0x10, 0x27, 0x50, 0x20, 0, 0, 0, 0]);
        assert_eq!(d[0].value, 2500.0);
        assert_eq!(d[1].value, 40.0);
        assert_eq!(d[2].text.as_deref(), Some("N"));
    }

    #[test]
    fn sym_decode() {
        let sym = r#"FormatVersion=6.0 // Do not edit this line!
Title="Test"

{ENUMS}
enum OnOff(0="Off", 1="On")

{SENDRECEIVE}

[Engine]
ID=100h
DLC=8
Var=RPM unsigned 0,16 /u:rpm /f:0.25
Var=Fan bit 16,1 /e:OnOff

[Page0]
ID=1ABCDEF0h
Type=Extended
DLC=8
Mux=Page 0,8 0
Var=A unsigned 8,8

[Page1]
ID=1ABCDEF0h
Type=Extended
DLC=8
Mux=Page 0,8 1
Var=B unsigned 8,8 /u:"km/h"
"#;
        let db = parse_sym(sym).unwrap();
        let m = db
            .get(MsgKey {
                id: 0x100,
                ext: false,
            })
            .unwrap();
        let d = m.decode(&[0x10, 0x27, 0x01]);
        assert_eq!(d[0].value, 2500.0);
        assert_eq!(d[1].text.as_deref(), Some("On"));

        let m = db
            .get(MsgKey {
                id: 0x1ABCDEF0,
                ext: true,
            })
            .unwrap();
        assert_eq!(m.name_for(&[1, 5]), "Page1");
        let d = m.decode(&[1, 5]);
        assert_eq!(d.len(), 2);
        assert_eq!(d[1].name, "B");
        assert_eq!(d[1].unit, "km/h");
    }
}

#[cfg(test)]
mod demo_file_tests {
    use super::*;

    #[test]
    fn demo_dbc_loads() {
        let db = SymbolDb::load(Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/examples/demo.dbc"
        )))
        .unwrap();
        assert_eq!(db.messages.len(), 7);
        let eec1 = db
            .get(MsgKey {
                id: 0x0CF00400,
                ext: true,
            })
            .unwrap();
        assert_eq!(eec1.name, "EEC1");
        let mux = db
            .get(MsgKey {
                id: 0x201,
                ext: false,
            })
            .unwrap();
        let d = mux.decode(&[2, 7, 0xAA, 0x55]);
        assert_eq!(
            d.iter().map(|s| s.name.as_str()).collect::<Vec<_>>(),
            ["Page", "CounterC", "Magic"]
        );
        assert!(
            db.get(MsgKey {
                id: 0x100,
                ext: false
            })
            .unwrap()
            .comment
            .is_some()
        );
    }
}

#[cfg(test)]
mod format_tests {
    use super::*;

    fn demo() -> SymbolDb {
        SymbolDb::load(Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/examples/demo.dbc"
        )))
        .unwrap()
    }

    fn sig<'a>(db: &'a SymbolDb, id: u32, name: &str) -> &'a SignalDef {
        db.get(MsgKey { id, ext: false })
            .unwrap()
            .signals
            .iter()
            .find(|s| s.name == name)
            .unwrap()
    }

    #[test]
    fn width_and_decimals_come_from_definition() {
        let db = demo();
        // RPM: 16-bit unsigned, factor 0.25 -> 0..16383.75
        let rpm = sig(&db, 0x100, "RPM").fmt;
        assert_eq!((rpm.width, rpm.decimals), (8, 2));
        assert_eq!(rpm.number(100.0), "  100.00");
        assert_eq!(rpm.number(1000.0), " 1000.00");
        assert_eq!(rpm.number(16383.75), "16383.75");
        // CoolantTemp: 8-bit, offset -40 -> -40..215 (needs a sign column)
        let temp = sig(&db, 0x100, "CoolantTemp").fmt;
        assert_eq!((temp.width, temp.decimals), (4, 0));
        assert_eq!(temp.number(-40.0), " -40");
        assert_eq!(temp.number(9.0), "   9");
        // Gear: value table -> number plus text padded to the longest entry
        let gear = sig(&db, 0x200, "Gear").fmt;
        assert_eq!(gear.value(3.0, Some("D1")), "  3 D1");
        assert_eq!(gear.value(0.0, Some("P")), "  0 P ");
        // Speed factor 0.01 -> 2 decimals
        assert_eq!(sig(&db, 0x200, "Speed").fmt.decimals, 2);
    }

    #[test]
    fn column_aligns_decimal_points_and_text() {
        let db = demo();
        let rpm = sig(&db, 0x100, "RPM").fmt; // 5 int digits, 2 decimals
        let temp = sig(&db, 0x100, "CoolantTemp").fmt; // sign + 3 digits, 0 decimals
        let gear = sig(&db, 0x200, "Gear").fmt; // 3 digits + value table
        let col = ValueColumn::fit([rpm, temp, gear]);
        let lines = [
            col.render(&rpm, 1000.0, None),
            col.render(&temp, -40.0, None),
            col.render(&gear, 3.0, Some("D1")),
        ];
        assert_eq!(lines[0], " 1000.00   ");
        assert_eq!(lines[1], "  -40      ");
        assert_eq!(lines[2], "    3    D1");
        // same total length -> anything after (units) lines up
        assert!(lines.iter().all(|l| l.len() == lines[2].len()), "{lines:?}");
    }

    #[test]
    fn decimals_for_factors() {
        assert_eq!(decimals_for(1.0), 0);
        assert_eq!(decimals_for(0.1), 1);
        assert_eq!(decimals_for(0.25), 2);
        assert_eq!(decimals_for(0.125), 3);
        assert_eq!(decimals_for(0.00390625), 4); // capped
    }

    #[test]
    fn decoded_lines_are_stable() {
        let db = demo();
        let engine = db
            .get(MsgKey {
                id: 0x100,
                ext: false,
            })
            .unwrap();
        let a = engine.decode_lines(&[0x90, 0x01, 0x30, 0, 0, 0, 0, 0x01]); // 100 rpm
        let b = engine.decode_lines(&[0xA0, 0x0F, 0xFF, 0, 0, 0, 0, 0x0F]); // 1000 rpm
        assert_eq!(a.len(), 3);
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(x.len(), y.len(), "{x:?} vs {y:?}");
        }
        // name padded to the longest name ("RollingCounter"), then the 8-wide value
        assert_eq!(a[0], format!("{:<14}  {} rpm", "RPM", "  100.00"));
        // Units start in the same column on every line.
        let unit_col = |l: &str, u: &str| l.rfind(u).unwrap();
        assert_eq!(unit_col(&a[0], "rpm"), unit_col(&a[1], "degC"));

        // Multiplexed message: same number of lines whatever the mux value.
        let mux = db
            .get(MsgKey {
                id: 0x201,
                ext: false,
            })
            .unwrap();
        assert_eq!(mux.max_lines, 3);
        for page in 0..6u8 {
            let lines = mux.decode_lines(&[page, 7, 0xAA, 0x55]);
            assert_eq!(lines.len(), 3, "page {page}");
        }
    }
}
