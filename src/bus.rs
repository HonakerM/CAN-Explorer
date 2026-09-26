//! The bus thread: owns the CAN device and is the single source of truth.
//!
//! Every received / transmitted frame is, *in this thread*:
//!   1. applied to the per-ID [`Summary`] (never dropped),
//!   2. written to the recording file if one is active (never dropped),
//!   3. decoded into per-signal values (min/max/history) when a symbol file
//!      is loaded (never dropped),
//!   4. offered to the UI's live stream channel (may be dropped if the UI
//!      can't keep up — drops are counted and shown).

use std::collections::{HashMap, VecDeque};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, TrySendError};
use parking_lot::{Mutex, RwLock};

use crate::candump;
use crate::device::{self, BusEvent, CanDevice, ConnectConfig, CtrlState, DevEvent};
use crate::msg::{CanMsg, Dir, MsgKey, now_ts};
use crate::symbols::{SymbolDb, ValueFormat};

pub const STREAM_CHANNEL_CAP: usize = 200_000;
const MAX_EVENTS: usize = 1000;
pub const HISTORY_SECS: usize = 300;
/// Samples of history kept per decoded signal.
pub const SIGNAL_HISTORY: usize = 20_000;

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct IdStats {
    pub key: MsgKey,
    pub count: u64,
    pub rx_count: u64,
    pub tx_count: u64,
    pub last: CanMsg,
    /// Wall-clock time (unix secs) the last frame was processed.
    pub last_wall: f64,
    /// Exponential moving average of the inter-frame period, seconds.
    pub period: f64,
    pub period_min: f64,
    pub period_max: f64,
    /// Wall-clock time each data byte last changed (for highlighting).
    pub byte_changed: [f64; 64],
    pub dlc_changes: u64,
}

impl IdStats {
    fn new(msg: &CanMsg, wall: f64) -> Self {
        Self {
            key: msg.key(),
            count: 0,
            rx_count: 0,
            tx_count: 0,
            last: *msg,
            last_wall: wall,
            period: 0.0,
            period_min: f64::MAX,
            period_max: 0.0,
            byte_changed: [0.0; 64],
            dlc_changes: 0,
        }
    }

    fn update(&mut self, msg: &CanMsg, wall: f64) {
        if self.count > 0 {
            let dt = msg.ts - self.last.ts;
            if dt >= 0.0 {
                self.period = if self.count == 1 {
                    dt
                } else {
                    self.period * 0.9 + dt * 0.1
                };
                self.period_min = self.period_min.min(dt);
                self.period_max = self.period_max.max(dt);
            }
            if msg.len != self.last.len {
                self.dlc_changes += 1;
            }
            for i in 0..msg.len as usize {
                if i >= self.last.len as usize || msg.data[i] != self.last.data[i] {
                    self.byte_changed[i] = wall;
                }
            }
        }
        self.count += 1;
        match msg.dir {
            Dir::Rx => self.rx_count += 1,
            Dir::Tx => self.tx_count += 1,
        }
        self.last = *msg;
        self.last_wall = wall;
    }

    pub fn rate_hz(&self) -> f64 {
        if self.period > 0.0 {
            1.0 / self.period
        } else {
            0.0
        }
    }
}

#[derive(Default)]
pub struct Summary {
    pub entries: HashMap<MsgKey, IdStats>,
    pub total: u64,
    /// Incremented on every change so the UI can skip redundant work.
    pub generation: u64,
}

#[derive(Clone)]
pub struct EventEntry {
    pub ts: f64,
    pub text: String,
    pub severe: bool,
}

#[derive(Clone, Copy, Default)]
pub struct HistoryPoint {
    pub load: f64,
    pub fps: f64,
}

#[derive(Clone, Default)]
pub struct ReplayProgress {
    pub index: usize,
    pub total: usize,
    pub looped: bool,
    pub loops: u64,
}

#[derive(Clone, Default)]
pub struct BusStats {
    pub connected: bool,
    pub iface: String,
    pub bitrate: u32,
    pub state: CtrlState,
    pub tec: Option<u8>,
    pub rec: Option<u8>,
    pub rx_total: u64,
    pub tx_total: u64,
    pub tx_errors: u64,
    pub error_frames: u64,
    pub rx_overflows: u64,
    pub fps: f64,
    pub load_pct: f64,
    pub peak_load_pct: f64,
    pub history: VecDeque<HistoryPoint>,
    pub events: VecDeque<EventEntry>,
    pub recording: Option<(PathBuf, u64)>,
    pub replay: Option<ReplayProgress>,
    pub finished: bool,
}

impl BusStats {
    fn push_event(&mut self, text: impl Into<String>, severe: bool) {
        if self.events.len() >= MAX_EVENTS {
            self.events.pop_front();
        }
        self.events.push_back(EventEntry {
            ts: now_ts(),
            text: text.into(),
            severe,
        });
    }
}

/// Latest decoded value and statistics for one signal of one message.
#[derive(Clone)]
pub struct SignalStats {
    pub msg_key: MsgKey,
    pub msg_name: String,
    pub name: String,
    pub unit: String,
    pub value: f64,
    pub raw: u64,
    pub text: Option<String>,
    /// Fixed-width display format from the symbol file.
    pub fmt: ValueFormat,
    pub min: f64,
    pub max: f64,
    pub count: u64,
    /// Frame timestamp of the last update.
    pub last_ts: f64,
    /// Wall-clock time of the last update.
    pub last_wall: f64,
    /// (frame timestamp, physical value), oldest first.
    pub history: VecDeque<(f64, f64)>,
}

impl SignalStats {
    pub fn display_value(&self) -> String {
        self.fmt.value(self.value, self.text.as_deref())
    }

    /// Cheap copy for UI tables (history can be tens of thousands of samples).
    pub fn without_history(&self) -> SignalStats {
        SignalStats {
            msg_key: self.msg_key,
            msg_name: self.msg_name.clone(),
            name: self.name.clone(),
            unit: self.unit.clone(),
            value: self.value,
            raw: self.raw,
            text: self.text.clone(),
            fmt: self.fmt,
            min: self.min,
            max: self.max,
            count: self.count,
            last_ts: self.last_ts,
            last_wall: self.last_wall,
            history: VecDeque::new(),
        }
    }
}

#[derive(Default)]
pub struct SignalTable {
    pub entries: HashMap<(MsgKey, String), SignalStats>,
    pub generation: u64,
}

impl SignalTable {
    pub fn clear(&mut self) {
        self.entries.clear();
        self.generation += 1;
    }
}

#[derive(Default)]
pub struct Shared {
    pub summary: Mutex<Summary>,
    /// Symbol database used by the bus thread to decode signals.
    pub symbols: RwLock<Option<Arc<SymbolDb>>>,
    pub signals: Mutex<SignalTable>,
    pub stats: Mutex<BusStats>,
    /// Frames the live stream view did not receive because it fell behind.
    pub stream_dropped: AtomicU64,
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

pub enum Cmd {
    Send(CanMsg),
    SetPeriodic {
        id: u64,
        msg: CanMsg,
        period: Duration,
        enabled: bool,
    },
    RemovePeriodic(u64),
    StartReplay {
        frames: Arc<Vec<CanMsg>>,
        speed: f64,
        looped: bool,
    },
    StopReplay,
    StartRecording(PathBuf),
    StopRecording,
    Stop,
}

pub struct BusHandle {
    pub cmd: Sender<Cmd>,
    pub stream: Receiver<CanMsg>,
    pub connect_result: Receiver<Result<String, String>>,
    thread: Option<JoinHandle<()>>,
}

impl BusHandle {
    pub fn send(&self, cmd: Cmd) {
        let _ = self.cmd.send(cmd);
    }
}

impl Drop for BusHandle {
    fn drop(&mut self) {
        let _ = self.cmd.send(Cmd::Stop);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

pub fn spawn(cfg: ConnectConfig, shared: Arc<Shared>) -> BusHandle {
    let (cmd_tx, cmd_rx) = crossbeam_channel::unbounded();
    let (stream_tx, stream_rx) = crossbeam_channel::bounded(STREAM_CHANNEL_CAP);
    let (res_tx, res_rx) = crossbeam_channel::bounded(1);
    let thread = std::thread::Builder::new()
        .name("can-bus".into())
        .spawn(move || {
            let dev = match device::open(&cfg) {
                Ok(d) => d,
                Err(e) => {
                    let _ = res_tx.send(Err(format!("{e:#}")));
                    return;
                }
            };
            let name = dev.log_name();
            {
                let mut st = shared.stats.lock();
                let history = std::mem::take(&mut st.history);
                let events = std::mem::take(&mut st.events);
                *st = BusStats {
                    connected: true,
                    iface: name.clone(),
                    bitrate: cfg.bitrate,
                    state: CtrlState::Unknown,
                    history,
                    events,
                    ..Default::default()
                };
                st.push_event(
                    format!("Connected to {} ({})", name, cfg.kind.label()),
                    false,
                );
            }
            let _ = res_tx.send(Ok(name));
            let mut worker = Worker::new(dev, cfg, shared.clone(), stream_tx);
            worker.run(cmd_rx);
            let mut st = shared.stats.lock();
            st.connected = false;
            st.recording = None;
            st.replay = None;
            st.push_event("Disconnected", false);
        })
        .expect("spawn bus thread");
    BusHandle {
        cmd: cmd_tx,
        stream: stream_rx,
        connect_result: res_rx,
        thread: Some(thread),
    }
}

// ---------------------------------------------------------------------------
// Worker
// ---------------------------------------------------------------------------

struct Periodic {
    msg: CanMsg,
    period: Duration,
    next: Instant,
    enabled: bool,
}

struct Replay {
    frames: Arc<Vec<CanMsg>>,
    speed: f64,
    looped: bool,
    loops: u64,
    idx: usize,
    start: Instant,
}

struct Recorder {
    path: PathBuf,
    out: BufWriter<File>,
    count: u64,
}

struct Worker {
    dev: Box<dyn CanDevice>,
    cfg: ConnectConfig,
    shared: Arc<Shared>,
    stream: Sender<CanMsg>,
    iface: String,
    periodic: HashMap<u64, Periodic>,
    replay: Option<Replay>,
    recorder: Option<Recorder>,
    // rolling window for fps / bus load
    win_start: Instant,
    win_frames: u64,
    win_bits: f64,
    last_tick: Instant,
    consecutive_errors: u32,
    // batch buffers (reused)
    batch: Vec<CanMsg>,
    events: Vec<BusEvent>,
    stop: bool,
}

impl Worker {
    fn new(
        dev: Box<dyn CanDevice>,
        cfg: ConnectConfig,
        shared: Arc<Shared>,
        stream: Sender<CanMsg>,
    ) -> Self {
        let iface = dev.log_name();
        Self {
            dev,
            cfg,
            shared,
            stream,
            iface,
            periodic: HashMap::new(),
            replay: None,
            recorder: None,
            win_start: Instant::now(),
            win_frames: 0,
            win_bits: 0.0,
            last_tick: Instant::now(),
            consecutive_errors: 0,
            batch: Vec::with_capacity(4096),
            events: Vec::new(),
            stop: false,
        }
    }

    fn run(&mut self, cmd_rx: Receiver<Cmd>) {
        let mut announced_finish = false;
        while !self.stop {
            while let Ok(cmd) = cmd_rx.try_recv() {
                self.handle_cmd(cmd);
            }
            if self.stop {
                break;
            }

            self.run_scheduled_tx();

            // Wait for the first frame no longer than the next scheduled TX
            // (and never more than 5 ms so commands stay responsive).
            let timeout = self.next_deadline().map_or(Duration::from_millis(5), |d| {
                d.saturating_duration_since(Instant::now())
                    .min(Duration::from_millis(5))
            });
            self.receive_burst(timeout);
            self.flush_batch();

            if self.last_tick.elapsed() >= Duration::from_millis(250) {
                self.tick();
            }
            if !announced_finish && self.dev.finished() {
                announced_finish = true;
                let mut st = self.shared.stats.lock();
                st.finished = true;
                st.push_event("End of log file reached", false);
            }
        }
        if let Some(mut r) = self.recorder.take() {
            let _ = r.out.flush();
        }
    }

    fn receive_burst(&mut self, first_timeout: Duration) {
        let mut timeout = first_timeout;
        // Drain everything the device has queued, up to a cap so that TX
        // scheduling and commands still get serviced under heavy load.
        for _ in 0..4096 {
            match self.dev.recv(timeout) {
                Ok(Some(DevEvent::Frame(m))) => {
                    self.consecutive_errors = 0;
                    self.batch.push(m);
                }
                Ok(Some(DevEvent::Bus(ev))) => self.events.push(ev),
                Ok(None) => break,
                Err(e) => {
                    self.consecutive_errors += 1;
                    let text = format!("{e:#}");
                    let mut ev = BusEvent::new(format!("receive error: {text}"));
                    let lower = text.to_lowercase();
                    if lower.contains("bus-off")
                        || lower.contains("busoff")
                        || lower.contains("bus off")
                    {
                        ev.state = Some(CtrlState::BusOff);
                    }
                    if lower.contains("overrun") || lower.contains("overflow") {
                        ev.rx_overflow = true;
                    }
                    // Avoid flooding the event log with an identical repeating error.
                    if self.consecutive_errors <= 3 || self.consecutive_errors % 1000 == 0 {
                        self.events.push(ev);
                    }
                    std::thread::sleep(Duration::from_millis(if self.consecutive_errors > 10 {
                        20
                    } else {
                        1
                    }));
                    break;
                }
            }
            timeout = Duration::ZERO;
        }
    }

    /// Apply the current batch of frames/events to shared state.
    fn flush_batch(&mut self) {
        if self.batch.is_empty() && self.events.is_empty() {
            return;
        }
        let wall = now_ts();
        let data_ratio = if self.cfg.fd && self.cfg.data_bitrate > 0 {
            self.cfg.bitrate as f64 / self.cfg.data_bitrate as f64
        } else {
            1.0
        };

        if !self.batch.is_empty() {
            let mut summary = self.shared.summary.lock();
            for m in &self.batch {
                summary
                    .entries
                    .entry(m.key())
                    .or_insert_with(|| IdStats::new(m, wall))
                    .update(m, wall);
                summary.total += 1;
            }
            summary.generation += 1;
        }

        let db = self.shared.symbols.read().clone();
        if let Some(db) = db
            && !self.batch.is_empty()
        {
            let mut table = self.shared.signals.lock();
            let mut touched = false;
            for m in &self.batch {
                let Some(def) = db.get(m.key()) else { continue };
                let msg_name = def.name_for(m.data());
                for d in def.decode(m.data()) {
                    touched = true;
                    let e = table
                        .entries
                        .entry((m.key(), d.name.clone()))
                        .or_insert_with(|| SignalStats {
                            msg_key: m.key(),
                            msg_name: msg_name.to_string(),
                            name: d.name.clone(),
                            unit: d.unit.clone(),
                            value: d.value,
                            raw: d.raw,
                            text: None,
                            fmt: d.fmt,
                            min: d.value,
                            max: d.value,
                            count: 0,
                            last_ts: m.ts,
                            last_wall: wall,
                            history: VecDeque::new(),
                        });
                    e.value = d.value;
                    e.raw = d.raw;
                    e.text = d.text;
                    e.min = e.min.min(d.value);
                    e.max = e.max.max(d.value);
                    e.count += 1;
                    e.last_ts = m.ts;
                    e.last_wall = wall;
                    if e.history.len() >= SIGNAL_HISTORY {
                        e.history.pop_front();
                    }
                    e.history.push_back((m.ts, d.value));
                }
            }
            if touched {
                table.generation += 1;
            }
        }

        let mut rx = 0u64;
        let mut tx = 0u64;
        for m in &self.batch {
            self.win_frames += 1;
            self.win_bits += m.wire_bits(data_ratio);
            match m.dir {
                Dir::Rx => rx += 1,
                Dir::Tx => tx += 1,
            }
            if let Some(r) = &mut self.recorder {
                let _ = writeln!(r.out, "{}", candump::format_line(m, &self.iface));
                r.count += 1;
            }
            match self.stream.try_send(*m) {
                Ok(()) => {}
                Err(TrySendError::Full(_)) => {
                    self.shared.stream_dropped.fetch_add(1, Ordering::Relaxed);
                }
                Err(TrySendError::Disconnected(_)) => {}
            }
        }
        self.batch.clear();

        let mut st = self.shared.stats.lock();
        st.rx_total += rx;
        st.tx_total += tx;
        for ev in self.events.drain(..) {
            st.error_frames += 1;
            if ev.rx_overflow {
                st.rx_overflows += 1;
            }
            if let Some(s) = ev.state {
                st.state = s;
            }
            if ev.tec.is_some() {
                st.tec = ev.tec;
                st.rec = ev.rec;
            }
            let severe = matches!(ev.state, Some(CtrlState::BusOff | CtrlState::ErrorPassive))
                || ev.rx_overflow;
            st.push_event(ev.desc, severe);
        }
    }

    fn tick(&mut self) {
        self.last_tick = Instant::now();
        let status = self.dev.status();
        let win = self.win_start.elapsed().as_secs_f64();
        let mut st = self.shared.stats.lock();
        if let Some(s) = status {
            if s.state != st.state && st.state != CtrlState::Unknown {
                let severe = matches!(s.state, CtrlState::BusOff | CtrlState::ErrorPassive);
                st.push_event(format!("Controller state: {}", s.state.label()), severe);
            }
            st.state = s.state;
            if s.tec.is_some() {
                st.tec = s.tec;
            }
            if s.rec.is_some() {
                st.rec = s.rec;
            }
        } else if st.state == CtrlState::Unknown && st.rx_total > 0 {
            st.state = CtrlState::ErrorActive;
        }
        if let Some(r) = &mut self.recorder {
            let _ = r.out.flush();
            st.recording = Some((r.path.clone(), r.count));
        }
        st.replay = self.replay.as_ref().map(|r| ReplayProgress {
            index: r.idx,
            total: r.frames.len(),
            looped: r.looped,
            loops: r.loops,
        });
        if win >= 1.0 {
            st.fps = self.win_frames as f64 / win;
            st.load_pct = if self.cfg.bitrate > 0 {
                (self.win_bits / win / self.cfg.bitrate as f64 * 100.0).min(999.0)
            } else {
                0.0
            };
            st.peak_load_pct = st.peak_load_pct.max(st.load_pct);
            let point = HistoryPoint {
                load: st.load_pct,
                fps: st.fps,
            };
            if st.history.len() >= HISTORY_SECS {
                st.history.pop_front();
            }
            st.history.push_back(point);
            self.win_start = Instant::now();
            self.win_frames = 0;
            self.win_bits = 0.0;
        }
    }

    fn handle_cmd(&mut self, cmd: Cmd) {
        match cmd {
            Cmd::Send(m) => self.transmit(m),
            Cmd::SetPeriodic {
                id,
                msg,
                period,
                enabled,
            } => {
                let period = period.max(Duration::from_micros(100));
                self.periodic.insert(
                    id,
                    Periodic {
                        msg,
                        period,
                        next: Instant::now(),
                        enabled,
                    },
                );
            }
            Cmd::RemovePeriodic(id) => {
                self.periodic.remove(&id);
            }
            Cmd::StartReplay {
                frames,
                speed,
                looped,
            } => {
                let n = frames.len();
                self.replay = Some(Replay {
                    frames,
                    speed: speed.max(0.001),
                    looped,
                    loops: 0,
                    idx: 0,
                    start: Instant::now(),
                });
                self.shared
                    .stats
                    .lock()
                    .push_event(format!("Playback started ({n} frames, {speed}x)"), false);
            }
            Cmd::StopReplay => {
                if self.replay.take().is_some() {
                    let mut st = self.shared.stats.lock();
                    st.replay = None;
                    st.push_event("Playback stopped", false);
                }
            }
            Cmd::StartRecording(path) => {
                let mut st = self.shared.stats.lock();
                match File::create(&path) {
                    Ok(f) => {
                        st.push_event(format!("Recording to {}", path.display()), false);
                        st.recording = Some((path.clone(), 0));
                        self.recorder = Some(Recorder {
                            path,
                            out: BufWriter::with_capacity(1 << 16, f),
                            count: 0,
                        });
                    }
                    Err(e) => {
                        st.push_event(format!("Cannot record to {}: {e}", path.display()), true)
                    }
                }
            }
            Cmd::StopRecording => {
                if let Some(mut r) = self.recorder.take() {
                    let _ = r.out.flush();
                    let mut st = self.shared.stats.lock();
                    st.recording = None;
                    st.push_event(
                        format!(
                            "Recording stopped: {} frames written to {}",
                            r.count,
                            r.path.display()
                        ),
                        false,
                    );
                }
            }
            Cmd::Stop => self.stop = true,
        }
    }

    fn transmit(&mut self, mut m: CanMsg) {
        match self.dev.send(&m) {
            Ok(()) => {
                m.ts = now_ts();
                m.dir = Dir::Tx;
                self.batch.push(m);
            }
            Err(e) => {
                let mut st = self.shared.stats.lock();
                st.tx_errors += 1;
                // Don't spam the log when a periodic message keeps failing.
                if st.tx_errors <= 5 || st.tx_errors % 500 == 0 {
                    st.push_event(format!("Transmit {} failed: {e:#}", m.key().fmt_id()), true);
                }
            }
        }
    }

    fn next_deadline(&self) -> Option<Instant> {
        let p = self
            .periodic
            .values()
            .filter(|p| p.enabled)
            .map(|p| p.next)
            .min();
        let r = self.replay.as_ref().and_then(|r| {
            r.frames
                .get(r.idx)
                .map(|m| r.start + Duration::from_secs_f64(m.ts / r.speed))
        });
        match (p, r) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    fn run_scheduled_tx(&mut self) {
        let now = Instant::now();
        let mut due: Vec<CanMsg> = Vec::new();
        for p in self.periodic.values_mut().filter(|p| p.enabled) {
            if p.next <= now {
                due.push(p.msg);
                p.next += p.period;
                if p.next < now {
                    p.next = now + p.period; // don't burst after a stall
                }
            }
        }
        let mut finished_replay = false;
        if let Some(r) = &mut self.replay {
            let mut sent = 0;
            while let Some(m) = r.frames.get(r.idx) {
                let t = r.start + Duration::from_secs_f64(m.ts / r.speed);
                if t > now || sent >= 1000 {
                    break;
                }
                due.push(*m);
                r.idx += 1;
                sent += 1;
            }
            if r.idx >= r.frames.len() {
                if r.looped && !r.frames.is_empty() {
                    r.idx = 0;
                    r.loops += 1;
                    r.start = Instant::now();
                } else {
                    finished_replay = true;
                }
            }
        }
        if finished_replay {
            self.replay = None;
            let mut st = self.shared.stats.lock();
            st.replay = None;
            st.push_event("Playback finished", false);
        }
        for m in due {
            self.transmit(m);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Runs the simulated ~10k fps bus and checks that the summary accounts
    /// for every frame, that the recording contains every frame, and that
    /// frames the stream view received plus those it skipped add up exactly.
    #[test]
    fn summary_and_recording_are_lossless() {
        let shared = Arc::new(Shared::default());
        let cfg = ConnectConfig {
            channel: "stress".into(),
            ..Default::default()
        };
        let h = spawn(cfg, shared.clone());
        assert!(h.connect_result.recv().unwrap().is_ok());

        let log =
            std::env::temp_dir().join(format!("can_explorer_test_{}.log", std::process::id()));
        h.send(Cmd::StartRecording(log.clone()));
        h.send(Cmd::Send(CanMsg::new(0x555, false, &[1, 2, 3])));

        // Deliberately don't read the stream for a while, then drain it.
        std::thread::sleep(Duration::from_millis(1500));
        h.send(Cmd::StopRecording);
        std::thread::sleep(Duration::from_millis(100));
        let mut streamed = 0u64;
        drop_after_stop(h, &mut streamed);

        let summary = shared.summary.lock();
        let stats = shared.stats.lock();
        let per_id: u64 = summary.entries.values().map(|e| e.count).sum();
        assert!(
            summary.total > 5000,
            "expected stress traffic, got {}",
            summary.total
        );
        assert_eq!(per_id, summary.total);
        assert_eq!(summary.total, stats.rx_total + stats.tx_total);
        assert_eq!(stats.tx_total, 1);
        assert_eq!(
            summary.entries[&MsgKey {
                id: 0x555,
                ext: false
            }]
                .tx_count,
            1
        );
        let dropped = shared.stream_dropped.load(Ordering::Relaxed);
        assert_eq!(streamed + dropped, summary.total);

        let text = std::fs::read_to_string(&log).unwrap();
        let recorded = text.lines().count() as u64;
        let _ = std::fs::remove_file(&log);
        assert!(
            recorded > 5000 && recorded <= summary.total,
            "recorded {recorded} of {}",
            summary.total
        );
        for line in text.lines() {
            assert!(candump::parse_line(line).unwrap().is_some());
        }
    }

    fn drop_after_stop(h: BusHandle, streamed: &mut u64) {
        let rx = h.stream.clone();
        drop(h); // stops and joins the bus thread
        while rx.try_recv().is_ok() {
            *streamed += 1;
        }
    }
}

#[cfg(test)]
mod signal_tests {
    use super::*;

    /// Every frame of a message with a symbol definition updates its signals.
    #[test]
    fn signals_decoded_for_every_frame() {
        let shared = Arc::new(Shared::default());
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/demo.dbc");
        *shared.symbols.write() = Some(Arc::new(
            SymbolDb::load(std::path::Path::new(path)).unwrap(),
        ));

        let h = spawn(ConnectConfig::default(), shared.clone());
        assert!(h.connect_result.recv().unwrap().is_ok());
        std::thread::sleep(Duration::from_millis(1000));
        drop(h);

        let summary = shared.summary.lock();
        let signals = shared.signals.lock();
        let engine = MsgKey {
            id: 0x100,
            ext: false,
        };
        let frames = summary.entries[&engine].count;
        assert!(frames > 50);
        for name in ["RPM", "CoolantTemp", "RollingCounter"] {
            let s = &signals.entries[&(engine, name.to_string())];
            assert_eq!(s.count, frames, "{name} updates");
            assert_eq!(s.history.len() as u64, frames);
            assert!(s.min <= s.value && s.value <= s.max);
            assert_eq!(s.msg_name, "EngineData");
        }
        // RPM = raw * 0.25; the simulator sweeps 500..5500 rpm
        let rpm = &signals.entries[&(engine, "RPM".to_string())];
        assert!(
            rpm.value >= 400.0 && rpm.value <= 5600.0,
            "rpm {}",
            rpm.value
        );
        // IDs without a definition (0x300) produce no signals
        assert!(signals.entries.keys().all(|(k, _)| k.id != 0x300));
    }
}
