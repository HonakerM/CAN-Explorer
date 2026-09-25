//! egui front-end. The UI never talks to hardware; it only reads shared state
//! published by the bus thread and sends it commands.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use egui::{Color32, RichText, Sense};
use egui_extras::{Column, TableBuilder};

use crate::bus::{self, BusHandle, Cmd, IdStats, Shared};
use crate::candump;
use crate::device::{self, BackendKind, ConnectConfig, CtrlState, PcanBus};
use crate::msg::{self, CanMsg, Dir, MsgKey, now_ts};
use crate::symbols::SymbolDb;

const BITRATES: &[u32] = &[
    10_000, 20_000, 50_000, 100_000, 125_000, 250_000, 500_000, 800_000, 1_000_000,
];
const DATA_BITRATES: &[u32] = &[1_000_000, 2_000_000, 4_000_000, 5_000_000, 8_000_000];
const STREAM_CAPS: &[usize] = &[10_000, 100_000, 500_000, 1_000_000];
const ROW_H: f32 = 18.0;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    Stream,
    Summary,
    Transmit,
    Status,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum TimeMode {
    Relative,
    Absolute,
    Delta,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SortBy {
    Id,
    Name,
    Count,
    Rate,
    Recent,
}

struct PeriodicUi {
    id: u64,
    msg: CanMsg,
    period_ms: f64,
    enabled: bool,
}

struct LoadedLog {
    path: PathBuf,
    frames: Arc<Vec<CanMsg>>,
    duration: f64,
    skipped: usize,
}

/// Parsed stream filter: `100 1A0 -7E8 engine` → include IDs 0x100/0x1A0 or
/// names containing "engine", excluding 0x7E8.
#[derive(Default, Clone, PartialEq)]
struct Filter {
    include: Vec<String>,
    exclude: Vec<String>,
}

impl Filter {
    fn parse(s: &str) -> Self {
        let mut f = Filter::default();
        for tok in s.split([' ', ',']).filter(|t| !t.is_empty()) {
            if let Some(t) = tok.strip_prefix('-') {
                if !t.is_empty() {
                    f.exclude.push(t.to_lowercase());
                }
            } else {
                f.include.push(tok.to_lowercase());
            }
        }
        f
    }

    fn is_empty(&self) -> bool {
        self.include.is_empty() && self.exclude.is_empty()
    }

    fn tok_matches(tok: &str, key: MsgKey, name: Option<&str>) -> bool {
        if let Ok(id) = msg::parse_hex_id(tok)
            && id == key.id {
                return true;
            }
        name.is_some_and(|n| n.to_lowercase().contains(tok))
    }

    fn matches(&self, key: MsgKey, name: Option<&str>) -> bool {
        if self.exclude.iter().any(|t| Self::tok_matches(t, key, name)) {
            return false;
        }
        self.include.is_empty() || self.include.iter().any(|t| Self::tok_matches(t, key, name))
    }
}

#[derive(Default)]
struct FilterCache {
    filter: Filter,
    sym_gen: u64,
    last_seq: u64,
    seqs: VecDeque<u64>,
}

pub struct App {
    shared: Arc<Shared>,
    bus: Option<BusHandle>,
    connecting: bool,
    connected_name: Option<String>,
    cfg: ConnectConfig,
    serial_ports: Vec<String>,
    #[cfg(target_os = "linux")]
    socketcan_ifaces: Vec<String>,

    symbols: Option<SymbolDb>,
    sym_gen: u64,

    tab: Tab,

    // live stream
    stream: VecDeque<CanMsg>,
    stream_front_seq: u64,
    stream_cap: usize,
    paused: bool,
    paused_skipped: u64,
    autoscroll: bool,
    stream_filter_text: String,
    filter_cache: FilterCache,
    time_mode: TimeMode,
    t_ref: Option<f64>,

    // summary
    summary_rows: Vec<IdStats>,
    summary_gen: u64,
    sort_by: SortBy,
    summary_filter_text: String,
    selected: Option<MsgKey>,

    // transmit
    tx_id: String,
    tx_ext: bool,
    tx_fd: bool,
    tx_brs: bool,
    tx_rtr: bool,
    tx_data: String,
    tx_period_ms: f64,
    periodic: Vec<PeriodicUi>,
    next_periodic_id: u64,
    loaded_log: Option<LoadedLog>,
    replay_speed: f64,
    replay_loop: bool,

    toast: Option<(String, Instant, bool)>,
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Table rows are clickable; selectable labels would swallow the clicks.
        cc.egui_ctx.style_mut(|s| s.interaction.selectable_labels = false);
        Self {
            shared: Arc::new(Shared::default()),
            bus: None,
            connecting: false,
            connected_name: None,
            cfg: ConnectConfig::default(),
            serial_ports: device::serial_ports(),
            #[cfg(target_os = "linux")]
            socketcan_ifaces: device::socketcan_interfaces(),
            symbols: None,
            sym_gen: 0,
            tab: Tab::Summary,
            stream: VecDeque::new(),
            stream_front_seq: 0,
            stream_cap: 100_000,
            paused: false,
            paused_skipped: 0,
            autoscroll: true,
            stream_filter_text: String::new(),
            filter_cache: FilterCache::default(),
            time_mode: TimeMode::Relative,
            t_ref: None,
            summary_rows: Vec::new(),
            summary_gen: u64::MAX,
            sort_by: SortBy::Id,
            summary_filter_text: String::new(),
            selected: None,
            tx_id: "123".into(),
            tx_ext: false,
            tx_fd: false,
            tx_brs: false,
            tx_rtr: false,
            tx_data: "00 00 00 00 00 00 00 00".into(),
            tx_period_ms: 100.0,
            periodic: Vec::new(),
            next_periodic_id: 1,
            loaded_log: None,
            replay_speed: 1.0,
            replay_loop: false,
            toast: None,
        }
    }

    fn info(&mut self, s: impl Into<String>) {
        self.toast = Some((s.into(), Instant::now(), false));
    }

    fn error(&mut self, s: impl Into<String>) {
        self.toast = Some((s.into(), Instant::now(), true));
    }

    fn is_connected(&self) -> bool {
        self.bus.is_some() && !self.connecting
    }

    fn send_cmd(&self, cmd: Cmd) {
        if let Some(b) = &self.bus {
            b.send(cmd);
        }
    }

    fn msg_name(&self, m: &CanMsg) -> Option<&str> {
        self.symbols.as_ref().and_then(|s| s.name(m))
    }

    // -----------------------------------------------------------------------
    // Connection handling
    // -----------------------------------------------------------------------

    fn connect(&mut self) {
        self.bus = Some(bus::spawn(self.cfg.clone(), self.shared.clone()));
        self.connecting = true;
    }

    fn disconnect(&mut self) {
        self.bus = None; // Drop joins the thread
        self.connecting = false;
        self.connected_name = None;
    }

    fn poll_bus(&mut self) {
        let Some(b) = &self.bus else { return };
        if self.connecting
            && let Ok(res) = b.connect_result.try_recv() {
                self.connecting = false;
                match res {
                    Ok(name) => {
                        self.info(format!("Connected: {name}"));
                        self.connected_name = Some(name);
                        // Re-arm periodic messages configured while offline.
                        for p in &self.periodic {
                            self.send_cmd(Cmd::SetPeriodic {
                                id: p.id,
                                msg: p.msg,
                                period: Duration::from_secs_f64(p.period_ms / 1000.0),
                                enabled: p.enabled,
                            });
                        }
                    }
                    Err(e) => {
                        self.bus = None;
                        self.error(format!("Connect failed: {e}"));
                        return;
                    }
                }
            }
        // Drain the live stream channel.
        let Some(b) = &self.bus else { return };
        while let Ok(m) = b.stream.try_recv() {
            if self.paused {
                self.paused_skipped += 1;
                continue;
            }
            if self.t_ref.is_none() {
                self.t_ref = Some(m.ts);
            }
            self.stream.push_back(m);
        }
        while self.stream.len() > self.stream_cap {
            self.stream.pop_front();
            self.stream_front_seq += 1;
        }
    }

    fn clear_all(&mut self) {
        self.stream.clear();
        self.stream_front_seq = 0;
        self.filter_cache = FilterCache::default();
        self.t_ref = None;
        self.paused_skipped = 0;
        {
            let mut s = self.shared.summary.lock();
            s.entries.clear();
            s.total = 0;
            s.generation += 1;
        }
        self.shared.stream_dropped.store(0, Ordering::Relaxed);
        let mut st = self.shared.stats.lock();
        st.rx_total = 0;
        st.tx_total = 0;
        st.error_frames = 0;
        st.rx_overflows = 0;
        st.tx_errors = 0;
        st.peak_load_pct = 0.0;
        st.history.clear();
        st.events.clear();
    }

    fn load_symbols(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("Symbol files", &["dbc", "sym"])
            .add_filter("All files", &["*"])
            .pick_file()
        else {
            return;
        };
        match SymbolDb::load(&path) {
            Ok(db) => {
                let n = db.messages.len();
                self.symbols = Some(db);
                self.sym_gen += 1;
                self.info(format!(
                    "Loaded {n} message definitions from {}",
                    path.display()
                ));
            }
            Err(e) => self.error(format!("Failed to load symbols: {e:#}")),
        }
    }

    fn start_recording(&mut self) {
        let default = format!("can_{}.log", chrono::Local::now().format("%Y%m%d_%H%M%S"));
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("candump log", &["log"])
            .set_file_name(default)
            .save_file()
        {
            self.send_cmd(Cmd::StartRecording(path));
        }
    }

    // -----------------------------------------------------------------------
    // Top bar
    // -----------------------------------------------------------------------

    fn top_bar(&mut self, ui: &mut egui::Ui) {
        let connected = self.bus.is_some();
        ui.horizontal_wrapped(|ui| {
            ui.add_enabled_ui(!connected, |ui| {
                egui::ComboBox::from_id_salt("backend")
                    .selected_text(self.cfg.kind.label())
                    .width(170.0)
                    .show_ui(ui, |ui| {
                        for k in BackendKind::all() {
                            if ui
                                .selectable_value(&mut self.cfg.kind, k, k.label())
                                .clicked()
                            {
                                self.on_backend_changed();
                            }
                        }
                    });
                self.channel_ui(ui);
            });

            let (label, enabled) = if self.connecting {
                ("Connecting...", false)
            } else if connected {
                ("⏹ Disconnect", true)
            } else {
                ("▶ Connect", true)
            };
            if ui
                .add_enabled(enabled, egui::Button::new(RichText::new(label).strong()))
                .clicked()
            {
                if connected {
                    self.disconnect();
                } else {
                    self.connect();
                }
            }

            ui.separator();
            let recording = self.shared.stats.lock().recording.clone();
            if let Some((path, n)) = recording {
                if ui
                    .button(RichText::new("⏺ Stop recording").color(Color32::from_rgb(230, 60, 60)))
                    .on_hover_text(format!("{} ({} frames)", path.display(), n))
                    .clicked()
                {
                    self.send_cmd(Cmd::StopRecording);
                }
            } else if ui
                .add_enabled(self.is_connected(), egui::Button::new("⏺ Record…"))
                .on_hover_text("Save every frame on the bus to a candump log file")
                .on_disabled_hover_text("Connect first")
                .clicked()
            {
                self.start_recording();
            }

            ui.separator();
            if ui
                .button("📂 Load symbols…")
                .on_hover_text("Load a DBC or PCAN .sym file")
                .clicked()
            {
                self.load_symbols();
            }
            if self.symbols.is_some()
                && ui
                    .small_button("✖")
                    .on_hover_text("Unload symbols")
                    .clicked()
            {
                self.symbols = None;
                self.sym_gen += 1;
            }
            ui.separator();
            if ui
                .button("🗑 Clear")
                .on_hover_text("Clear stream, summary and counters")
                .clicked()
            {
                self.clear_all();
            }
        });
    }

    fn on_backend_changed(&mut self) {
        self.cfg.channel = match self.cfg.kind {
            BackendKind::Virtual => "demo".into(),
            #[cfg(target_os = "linux")]
            BackendKind::SocketCan => {
                self.socketcan_ifaces = device::socketcan_interfaces();
                self.socketcan_ifaces
                    .first()
                    .cloned()
                    .unwrap_or_else(|| "can0".into())
            }
            BackendKind::Slcan => {
                self.serial_ports = device::serial_ports();
                self.serial_ports.first().cloned().unwrap_or_default()
            }
            _ => String::new(),
        };
        if !self.cfg.kind.supports_fd() {
            self.cfg.fd = false;
        }
    }

    fn channel_ui(&mut self, ui: &mut egui::Ui) {
        match self.cfg.kind {
            BackendKind::Virtual => {
                egui::ComboBox::from_id_salt("vmode")
                    .selected_text(self.cfg.channel.clone())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.cfg.channel, "demo".to_string(), "demo")
                            .on_hover_text("A handful of vehicle-like periodic messages");
                        ui.selectable_value(&mut self.cfg.channel, "stress".to_string(), "stress")
                            .on_hover_text("~10,000 frames/s to exercise the pipeline");
                        ui.selectable_value(&mut self.cfg.channel, "quiet".to_string(), "quiet");
                    });
            }
            BackendKind::Pcan => {
                egui::ComboBox::from_id_salt("pcanbus")
                    .selected_text(match self.cfg.pcan_bus {
                        PcanBus::Usb => "USB",
                        PcanBus::Pci => "PCI",
                        PcanBus::Lan => "LAN",
                    })
                    .width(60.0)
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.cfg.pcan_bus, PcanBus::Usb, "USB");
                        ui.selectable_value(&mut self.cfg.pcan_bus, PcanBus::Pci, "PCI");
                        ui.selectable_value(&mut self.cfg.pcan_bus, PcanBus::Lan, "LAN");
                    });
                egui::ComboBox::from_id_salt("pcanidx")
                    .selected_text(format!("Channel {}", self.cfg.index + 1))
                    .show_ui(ui, |ui| {
                        for i in 0..16 {
                            ui.selectable_value(
                                &mut self.cfg.index,
                                i,
                                format!("Channel {}", i + 1),
                            );
                        }
                    });
            }
            BackendKind::Kvaser => {
                ui.label("Channel");
                ui.add(egui::DragValue::new(&mut self.cfg.index).range(0..=63));
            }
            #[cfg(target_os = "linux")]
            BackendKind::SocketCan => {
                egui::ComboBox::from_id_salt("scif")
                    .selected_text(self.cfg.channel.clone())
                    .show_ui(ui, |ui| {
                        for i in self.socketcan_ifaces.clone() {
                            ui.selectable_value(&mut self.cfg.channel, i.clone(), i);
                        }
                    });
                ui.add(egui::TextEdit::singleline(&mut self.cfg.channel).desired_width(70.0));
                if ui
                    .small_button("⟳")
                    .on_hover_text("Refresh interfaces")
                    .clicked()
                {
                    self.socketcan_ifaces = device::socketcan_interfaces();
                }
            }
            BackendKind::Slcan => {
                egui::ComboBox::from_id_salt("port")
                    .selected_text(self.cfg.channel.clone())
                    .show_ui(ui, |ui| {
                        for p in self.serial_ports.clone() {
                            ui.selectable_value(&mut self.cfg.channel, p.clone(), p);
                        }
                    });
                ui.add(
                    egui::TextEdit::singleline(&mut self.cfg.channel)
                        .desired_width(110.0)
                        .hint_text("port"),
                );
                if ui
                    .small_button("⟳")
                    .on_hover_text("Refresh serial ports")
                    .clicked()
                {
                    self.serial_ports = device::serial_ports();
                }
            }
            BackendKind::LogFile => {
                let name = self
                    .cfg
                    .log_path
                    .as_ref()
                    .and_then(|p| p.file_name())
                    .map(|n| n.to_string_lossy().into_owned())
                    .unwrap_or_else(|| "(no file)".into());
                if ui.button(format!("📂 {name}")).clicked()
                    && let Some(p) = rfd::FileDialog::new()
                        .add_filter("candump log", &["log", "txt", "candump"])
                        .add_filter("All files", &["*"])
                        .pick_file()
                    {
                        self.cfg.log_path = Some(p);
                    }
                ui.label("Speed");
                egui::ComboBox::from_id_salt("logspeed")
                    .selected_text(speed_label(self.cfg.log_speed))
                    .width(70.0)
                    .show_ui(ui, |ui| {
                        for s in [0.0, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 100.0] {
                            ui.selectable_value(&mut self.cfg.log_speed, s, speed_label(s));
                        }
                    });
            }
        }

        // Bitrate. SocketCAN bitrate is configured with `ip link`, but we still
        // need it for the bus-load estimate.
        let label = if self.cfg.kind.has_bitrate() {
            "Bitrate"
        } else {
            "Bitrate (load calc)"
        };
        ui.label(label);
        egui::ComboBox::from_id_salt("bitrate")
            .selected_text(fmt_bitrate(self.cfg.bitrate))
            .width(80.0)
            .show_ui(ui, |ui| {
                for &b in BITRATES {
                    ui.selectable_value(&mut self.cfg.bitrate, b, fmt_bitrate(b));
                }
            });
        if self.cfg.kind.supports_fd() {
            ui.checkbox(&mut self.cfg.fd, "CAN FD");
            if self.cfg.fd {
                ui.label("Data");
                egui::ComboBox::from_id_salt("dbitrate")
                    .selected_text(fmt_bitrate(self.cfg.data_bitrate))
                    .width(80.0)
                    .show_ui(ui, |ui| {
                        for &b in DATA_BITRATES {
                            ui.selectable_value(&mut self.cfg.data_bitrate, b, fmt_bitrate(b));
                        }
                    });
            }
        }
    }

    // -----------------------------------------------------------------------
    // Status bar
    // -----------------------------------------------------------------------

    fn status_bar(&mut self, ui: &mut egui::Ui) {
        let st = self.shared.stats.lock().clone_light();
        let dropped = self.shared.stream_dropped.load(Ordering::Relaxed);
        ui.horizontal(|ui| {
            if self.bus.is_some() && !self.connecting {
                ui.label(RichText::new(format!("● {}", st.iface)).color(Color32::from_rgb(60, 180, 90)));
                ui.label(RichText::new(st.state.label()).color(state_color(st.state)).strong());
            } else if self.connecting {
                ui.label("Connecting…");
            } else {
                ui.label(RichText::new("○ Disconnected").weak());
            }
            ui.separator();
            ui.label(RichText::new(format!("Load {:.1}%", st.load_pct)).color(load_color(st.load_pct, ui)));
            ui.label(format!("{:.0} fps", st.fps));
            ui.separator();
            ui.label(format!("RX {}  TX {}", st.rx_total, st.tx_total));
            if st.error_frames > 0 {
                ui.label(RichText::new(format!("Errors {}", st.error_frames)).color(ui.visuals().warn_fg_color));
            }
            if st.tx_errors > 0 {
                ui.label(RichText::new(format!("TX fail {}", st.tx_errors)).color(ui.visuals().error_fg_color));
            }
            if dropped > 0 {
                ui.label(RichText::new(format!("View skipped {dropped}")).color(ui.visuals().warn_fg_color))
                    .on_hover_text("The live stream view fell behind and skipped frames.\nThe summary, counters and recording still include every frame.");
            }
            if let Some((p, n)) = &st.recording {
                ui.separator();
                ui.label(RichText::new(format!("⏺ REC {n}")).color(Color32::from_rgb(230, 60, 60)))
                    .on_hover_text(p.display().to_string());
            }
            if let Some(db) = &self.symbols {
                ui.separator();
                let name = db
                    .source
                    .as_ref()
                    .and_then(|p| p.file_name())
                    .map(|n| n.to_string_lossy().into_owned())
                    .unwrap_or_default();
                ui.label(format!("Symbols: {name} ({})", db.messages.len()));
            }
            if let Some((t, at, err)) = &self.toast
                && at.elapsed() < Duration::from_secs(8) {
                    ui.separator();
                    let c = if *err { ui.visuals().error_fg_color } else { ui.visuals().text_color() };
                    ui.label(RichText::new(t).color(c));
                }
        });
    }

    // -----------------------------------------------------------------------
    // Stream tab
    // -----------------------------------------------------------------------

    fn update_filter_cache(&mut self) {
        let filter = Filter::parse(&self.stream_filter_text);
        let back_seq = self.stream_front_seq + self.stream.len() as u64;
        let fc = &mut self.filter_cache;
        if fc.filter != filter || fc.sym_gen != self.sym_gen || fc.last_seq > back_seq {
            fc.filter = filter;
            fc.sym_gen = self.sym_gen;
            fc.seqs.clear();
            fc.last_seq = self.stream_front_seq;
        }
        if fc.filter.is_empty() {
            fc.last_seq = back_seq;
            return;
        }
        let start = fc.last_seq.max(self.stream_front_seq);
        for seq in start..back_seq {
            let m = &self.stream[(seq - self.stream_front_seq) as usize];
            let name = self.symbols.as_ref().and_then(|s| s.name(m));
            if fc.filter.matches(m.key(), name) {
                fc.seqs.push_back(seq);
            }
        }
        fc.last_seq = back_seq;
        while fc.seqs.front().is_some_and(|&s| s < self.stream_front_seq) {
            fc.seqs.pop_front();
        }
    }

    fn stream_tab(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let pause_label = if self.paused {
                "▶ Resume"
            } else {
                "⏸ Pause"
            };
            if ui.button(pause_label).clicked() {
                self.paused = !self.paused;
            }
            if self.paused && self.paused_skipped > 0 {
                ui.label(
                    RichText::new(format!(
                        "{} frames not shown while paused",
                        self.paused_skipped
                    ))
                    .weak(),
                );
            }
            ui.checkbox(&mut self.autoscroll, "Auto-scroll");
            ui.separator();
            ui.label("Filter");
            ui.add(
                egui::TextEdit::singleline(&mut self.stream_filter_text)
                    .desired_width(220.0)
                    .hint_text("IDs / names, -ID to exclude"),
            );
            ui.separator();
            ui.label("Time");
            ui.selectable_value(&mut self.time_mode, TimeMode::Relative, "Relative");
            ui.selectable_value(&mut self.time_mode, TimeMode::Absolute, "Absolute");
            ui.selectable_value(&mut self.time_mode, TimeMode::Delta, "Δ");
            ui.separator();
            ui.label("Buffer");
            egui::ComboBox::from_id_salt("cap")
                .selected_text(format!("{}", self.stream_cap))
                .show_ui(ui, |ui| {
                    for &c in STREAM_CAPS {
                        ui.selectable_value(&mut self.stream_cap, c, format!("{c}"));
                    }
                });
            ui.label(RichText::new(format!("{} frames", self.stream.len())).weak());
        });
        ui.separator();

        self.update_filter_cache();
        let filtered = !self.filter_cache.filter.is_empty();
        let n = if filtered {
            self.filter_cache.seqs.len()
        } else {
            self.stream.len()
        };
        let stream = &self.stream;
        let front = self.stream_front_seq;
        let seqs = &self.filter_cache.seqs;
        let symbols = self.symbols.as_ref();
        let t_ref = self.t_ref.unwrap_or(0.0);
        let time_mode = self.time_mode;
        let get = |row: usize| -> Option<&CanMsg> {
            if filtered {
                seqs.get(row)
                    .and_then(|&s| stream.get((s - front) as usize))
            } else {
                stream.get(row)
            }
        };

        TableBuilder::new(ui)
            .striped(true)
            .resizable(true)
            .stick_to_bottom(self.autoscroll)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(Column::initial(120.0).at_least(60.0))
            .column(Column::initial(30.0))
            .column(Column::initial(80.0))
            .column(Column::initial(180.0).at_least(60.0).clip(true))
            .column(Column::initial(30.0))
            .column(Column::remainder().at_least(150.0))
            .header(20.0, |mut h| {
                for t in ["Time", "Dir", "ID", "Name", "Len", "Data"] {
                    h.col(|ui| {
                        ui.strong(t);
                    });
                }
            })
            .body(|body| {
                body.rows(ROW_H, n, |mut row| {
                    let i = row.index();
                    let Some(m) = get(i) else { return };
                    let time = match time_mode {
                        TimeMode::Relative => format!("{:.6}", m.ts - t_ref),
                        TimeMode::Absolute => fmt_abs_time(m.ts),
                        TimeMode::Delta => {
                            let prev = if i > 0 {
                                get(i - 1).map(|p| p.ts)
                            } else {
                                None
                            };
                            format!("{:.6}", prev.map_or(0.0, |p| m.ts - p))
                        }
                    };
                    row.col(|ui| {
                        ui.monospace(time);
                    });
                    row.col(|ui| {
                        dir_label(ui, m.dir);
                    });
                    row.col(|ui| {
                        ui.monospace(fmt_id(m));
                    });
                    row.col(|ui| {
                        if let Some(n) = symbols.and_then(|s| s.name(m)) {
                            ui.label(n);
                        }
                    });
                    row.col(|ui| {
                        ui.monospace(m.len.to_string());
                    });
                    row.col(|ui| {
                        let r = ui.monospace(m.data_hex());
                        if let Some(def) = symbols.and_then(|s| s.get(m.key())) {
                            r.on_hover_ui(|ui| {
                                signals_grid(ui, &def.decode(m.data()), "stream_hover")
                            });
                        }
                    });
                });
            });
    }

    // -----------------------------------------------------------------------
    // Summary tab
    // -----------------------------------------------------------------------

    fn refresh_summary(&mut self) {
        let s = self.shared.summary.lock();
        if s.generation == self.summary_gen {
            return;
        }
        self.summary_gen = s.generation;
        self.summary_rows.clear();
        self.summary_rows.extend(s.entries.values().cloned());
    }

    fn summary_tab(&mut self, ui: &mut egui::Ui) {
        self.refresh_summary();
        let now = now_ts();
        let total: u64 = self.summary_rows.iter().map(|r| r.count).sum();

        // Sort
        let symbols = self.symbols.as_ref();
        let name_of = |r: &IdStats| {
            symbols
                .and_then(|s| s.name(&r.last))
                .unwrap_or("")
                .to_string()
        };
        match self.sort_by {
            SortBy::Id => self.summary_rows.sort_by_key(|r| (r.key.ext, r.key.id)),
            SortBy::Name => self
                .summary_rows
                .sort_by_cached_key(|r| (name_of(r).is_empty(), name_of(r), r.key)),
            SortBy::Count => self.summary_rows.sort_by(|a, b| b.count.cmp(&a.count)),
            SortBy::Rate => self
                .summary_rows
                .sort_by(|a, b| b.rate_hz().total_cmp(&a.rate_hz())),
            SortBy::Recent => self
                .summary_rows
                .sort_by(|a, b| b.last_wall.total_cmp(&a.last_wall)),
        }

        ui.horizontal(|ui| {
            ui.label(format!("{} IDs, {} frames", self.summary_rows.len(), total));
            ui.separator();
            ui.label("Sort");
            ui.selectable_value(&mut self.sort_by, SortBy::Id, "ID");
            ui.selectable_value(&mut self.sort_by, SortBy::Name, "Name");
            ui.selectable_value(&mut self.sort_by, SortBy::Count, "Count");
            ui.selectable_value(&mut self.sort_by, SortBy::Rate, "Rate");
            ui.selectable_value(&mut self.sort_by, SortBy::Recent, "Recent");
            ui.separator();
            ui.label("Filter");
            ui.add(
                egui::TextEdit::singleline(&mut self.summary_filter_text)
                    .desired_width(200.0)
                    .hint_text("IDs / names, -ID to exclude"),
            );
        });
        ui.separator();

        // Selected message detail panel
        let selected_row = self
            .selected
            .and_then(|k| self.summary_rows.iter().find(|r| r.key == k))
            .cloned();
        if let Some(r) = &selected_row {
            egui::SidePanel::right("signals_panel")
                .resizable(true)
                .default_width(340.0)
                .show_inside(ui, |ui| {
                    self.detail_panel(ui, r, now);
                });
        }

        let filter = Filter::parse(&self.summary_filter_text);
        let symbols = self.symbols.as_ref();
        let rows: Vec<&IdStats> = self
            .summary_rows
            .iter()
            .filter(|r| {
                filter.is_empty() || filter.matches(r.key, symbols.and_then(|s| s.name(&r.last)))
            })
            .collect();
        let hl = ui.visuals().warn_fg_color;
        let normal = ui.visuals().text_color();
        let mut clicked: Option<MsgKey> = None;
        let selected = self.selected;

        TableBuilder::new(ui)
            .striped(true)
            .resizable(true)
            .sense(Sense::click())
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(Column::initial(80.0))
            .column(Column::initial(170.0).clip(true))
            .column(Column::initial(30.0))
            .column(Column::initial(200.0).at_least(80.0))
            .column(Column::initial(80.0))
            .column(Column::initial(70.0))
            .column(Column::initial(150.0))
            .column(Column::initial(60.0))
            .column(Column::remainder().at_least(40.0))
            .header(20.0, |mut h| {
                for (t, tip) in [
                    ("ID", ""),
                    ("Name", ""),
                    ("Len", ""),
                    ("Last data", "Bytes that changed recently are highlighted"),
                    ("Count", "Exact number of frames seen with this ID"),
                    ("Rate", "Frames per second (from average period)"),
                    ("Period ms (avg/min/max)", ""),
                    ("Age s", "Seconds since last frame"),
                    ("Dir", ""),
                ] {
                    h.col(|ui| {
                        let r = ui.strong(t);
                        if !tip.is_empty() {
                            r.on_hover_text(tip);
                        }
                    });
                }
            })
            .body(|body| {
                body.rows(ROW_H, rows.len(), |mut row| {
                    let r = rows[row.index()];
                    row.set_selected(selected == Some(r.key));
                    let m = &r.last;
                    row.col(|ui| {
                        ui.monospace(r.key.fmt_id());
                    });
                    row.col(|ui| {
                        if let Some(n) = symbols.and_then(|s| s.name(m)) {
                            ui.label(n);
                        }
                    });
                    row.col(|ui| {
                        ui.monospace(m.len.to_string());
                    });
                    row.col(|ui| {
                        if m.rtr {
                            ui.monospace("RTR");
                            return;
                        }
                        ui.spacing_mut().item_spacing.x = 4.0;
                        for (i, b) in m.data().iter().enumerate() {
                            let age = (now - r.byte_changed[i]).max(0.0);
                            let c = if r.count > 1 && age < 1.5 {
                                mix(hl, normal, (age / 1.5) as f32)
                            } else {
                                normal
                            };
                            ui.label(RichText::new(format!("{b:02X}")).monospace().color(c));
                        }
                    });
                    row.col(|ui| {
                        ui.monospace(r.count.to_string());
                    });
                    row.col(|ui| {
                        ui.monospace(format!("{:.1}", r.rate_hz()));
                    });
                    row.col(|ui| {
                        if r.count > 1 {
                            ui.monospace(format!(
                                "{:.1} / {:.1} / {:.1}",
                                r.period * 1e3,
                                r.period_min * 1e3,
                                r.period_max * 1e3
                            ));
                        }
                    });
                    row.col(|ui| {
                        let age = (now - r.last_wall).max(0.0);
                        let t = RichText::new(format!("{age:.1}")).monospace();
                        // Grey out IDs that stopped (> 3 periods and > 1 s)
                        if age > 1.0 && r.period > 0.0 && age > r.period * 3.0 {
                            ui.label(t.weak());
                        } else {
                            ui.label(t);
                        }
                    });
                    row.col(|ui| {
                        let s = match (r.rx_count > 0, r.tx_count > 0) {
                            (true, true) => "Rx/Tx",
                            (false, true) => "Tx",
                            _ => "Rx",
                        };
                        ui.label(s);
                    });
                    if row.response().clicked() {
                        clicked = Some(r.key);
                    }
                });
            });

        if let Some(k) = clicked {
            self.selected = if self.selected == Some(k) {
                None
            } else {
                Some(k)
            };
        }
    }

    fn detail_panel(&mut self, ui: &mut egui::Ui, r: &IdStats, now: f64) {
        let m = r.last;
        ui.horizontal(|ui| {
            ui.heading(r.key.fmt_id());
            if let Some(n) = self.msg_name(&m) {
                ui.heading(n.to_string());
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.small_button("✖").clicked() {
                    self.selected = None;
                }
            });
        });
        ui.label(format!(
            "{} frames · {:.1} Hz · last {:.2}s ago · {}",
            r.count,
            r.rate_hz(),
            (now - r.last_wall).max(0.0),
            if r.key.ext { "extended" } else { "standard" }
        ));
        if r.dlc_changes > 0 {
            ui.label(RichText::new(format!("Length changed {} times", r.dlc_changes)).weak());
        }
        ui.monospace(m.data_hex());
        ui.horizontal(|ui| {
            if ui.button("Copy to Transmit").clicked() {
                self.tx_id = format!("{:X}", m.id);
                self.tx_ext = m.ext;
                self.tx_fd = m.fd;
                self.tx_brs = m.brs;
                self.tx_rtr = m.rtr;
                self.tx_data = m.data_hex();
                self.tab = Tab::Transmit;
            }
            if ui.button("Filter stream").clicked() {
                self.stream_filter_text = r.key.fmt_id();
                self.tab = Tab::Stream;
            }
        });
        ui.separator();
        match self.symbols.as_ref().and_then(|s| s.get(r.key)) {
            Some(def) => {
                if let Some(c) = &def.comment {
                    ui.label(RichText::new(c).italics().weak());
                }
                let decoded = def.decode(m.data());
                if decoded.is_empty() {
                    ui.label("No signals defined");
                } else {
                    egui::ScrollArea::vertical()
                        .show(ui, |ui| signals_grid(ui, &decoded, "detail_grid"));
                }
            }
            None => {
                ui.label(RichText::new("No symbol definition for this ID.").weak());
                ui.label(RichText::new("Load a DBC or SYM file to decode signals.").weak());
                ui.separator();
                // Show a bit-level view as a fallback.
                egui::Grid::new("bits").striped(true).show(ui, |ui| {
                    ui.strong("Byte");
                    ui.strong("Hex");
                    ui.strong("Binary");
                    ui.strong("Dec");
                    ui.end_row();
                    for (i, b) in m.data().iter().enumerate() {
                        ui.monospace(i.to_string());
                        ui.monospace(format!("{b:02X}"));
                        ui.monospace(format!("{b:08b}"));
                        ui.monospace(b.to_string());
                        ui.end_row();
                    }
                });
            }
        }
    }

    // -----------------------------------------------------------------------
    // Transmit tab
    // -----------------------------------------------------------------------

    fn build_tx_msg(&self) -> Result<CanMsg, String> {
        let id = msg::parse_hex_id(&self.tx_id)?;
        let ext = self.tx_ext || id > 0x7FF;
        if id > 0x1FFF_FFFF {
            return Err("ID exceeds 29 bits".into());
        }
        let data = if self.tx_rtr {
            Vec::new()
        } else {
            msg::parse_hex_bytes(&self.tx_data)?
        };
        if self.tx_fd {
            if !msg::fd_len_ok(data.len()) {
                return Err(format!(
                    "{} bytes is not a valid CAN FD length (0-8,12,16,20,24,32,48,64)",
                    data.len()
                ));
            }
        } else if data.len() > 8 {
            return Err("classic CAN frames carry at most 8 bytes (enable FD)".into());
        }
        let mut m = CanMsg::new(id, ext, &data);
        m.fd = self.tx_fd;
        m.brs = self.tx_fd && self.tx_brs;
        m.rtr = self.tx_rtr && !self.tx_fd;
        if m.rtr {
            m.len = msg::parse_hex_bytes(&self.tx_data)
                .map(|d| d.len().min(8) as u8)
                .unwrap_or(0);
        }
        m.dir = Dir::Tx;
        Ok(m)
    }

    fn transmit_tab(&mut self, ui: &mut egui::Ui) {
        let connected = self.is_connected();
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Single frame");
            egui::Grid::new("txgrid").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                if let Some(db) = &self.symbols {
                    ui.label("From symbol");
                    let mut names: Vec<(&String, MsgKey, u8)> =
                        db.messages.values().map(|m| (&m.name, m.key, m.dlc)).collect();
                    names.sort();
                    let mut pick: Option<(MsgKey, u8)> = None;
                    egui::ComboBox::from_id_salt("symtx").selected_text("choose…").height(400.0).show_ui(ui, |ui| {
                        for (n, k, dlc) in names {
                            if ui.selectable_label(false, format!("{n}  ({})", k.fmt_id())).clicked() {
                                pick = Some((k, dlc));
                            }
                        }
                    });
                    if let Some((k, dlc)) = pick {
                        self.tx_id = format!("{:X}", k.id);
                        self.tx_ext = k.ext;
                        self.tx_fd = dlc > 8;
                        self.tx_data = msg::hex_bytes(&vec![0u8; dlc as usize]);
                    }
                    ui.end_row();
                }
                ui.label("ID (hex)");
                ui.horizontal(|ui| {
                    ui.add(egui::TextEdit::singleline(&mut self.tx_id).desired_width(100.0).font(egui::TextStyle::Monospace));
                    ui.checkbox(&mut self.tx_ext, "Extended (29-bit)");
                    ui.checkbox(&mut self.tx_rtr, "RTR");
                    ui.add_enabled(self.cfg.fd, egui::Checkbox::new(&mut self.tx_fd, "FD"))
                        .on_disabled_hover_text("Connect with CAN FD enabled to send FD frames");
                    ui.add_enabled(self.tx_fd, egui::Checkbox::new(&mut self.tx_brs, "BRS"));
                });
                ui.end_row();
                ui.label("Data (hex)");
                ui.add(
                    egui::TextEdit::singleline(&mut self.tx_data)
                        .desired_width(420.0)
                        .font(egui::TextStyle::Monospace)
                        .hint_text("11 22 33 44"),
                );
                ui.end_row();
                ui.label("Period (ms)");
                ui.add(egui::DragValue::new(&mut self.tx_period_ms).range(0.1..=60_000.0).speed(1.0));
                ui.end_row();
            });

            let built = self.build_tx_msg();
            match &built {
                Err(e) => {
                    ui.label(RichText::new(e).color(ui.visuals().error_fg_color));
                }
                Ok(m) => {
                    if let Some(def) = self.symbols.as_ref().and_then(|s| s.get(m.key())) {
                        ui.collapsing(format!("Decoded as {}", def.name_for(m.data())), |ui| {
                            signals_grid(ui, &def.decode(m.data()), "tx_preview");
                        });
                    }
                }
            }
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(connected && built.is_ok(), egui::Button::new("📤 Send once"))
                    .on_disabled_hover_text("Connect and enter a valid frame")
                    .clicked()
                    && let Ok(m) = &built {
                        self.send_cmd(Cmd::Send(*m));
                    }
                if ui.add_enabled(built.is_ok(), egui::Button::new("➕ Add periodic")).clicked()
                    && let Ok(m) = built.clone() {
                        let id = self.next_periodic_id;
                        self.next_periodic_id += 1;
                        let p = PeriodicUi { id, msg: m, period_ms: self.tx_period_ms, enabled: false };
                        self.send_cmd(Cmd::SetPeriodic {
                            id,
                            msg: p.msg,
                            period: Duration::from_secs_f64(p.period_ms / 1000.0),
                            enabled: false,
                        });
                        self.periodic.push(p);
                    }
            });

            ui.add_space(12.0);
            ui.separator();
            ui.heading("Periodic messages");
            if self.periodic.is_empty() {
                ui.label(RichText::new("None. Use \"Add periodic\" above.").weak());
            }
            let mut remove = None;
            let mut changed = Vec::new();
            egui::Grid::new("periodic").striped(true).num_columns(6).show(ui, |ui| {
                if !self.periodic.is_empty() {
                    for h in ["Active", "ID", "Name", "Data", "Period ms", ""] {
                        ui.strong(h);
                    }
                    ui.end_row();
                }
                for p in &mut self.periodic {
                    if ui.checkbox(&mut p.enabled, "").changed() {
                        changed.push(p.id);
                    }
                    ui.monospace(p.msg.key().fmt_id());
                    ui.label(self.symbols.as_ref().and_then(|s| s.name(&p.msg)).unwrap_or(""));
                    ui.monospace(p.msg.data_hex());
                    if ui.add(egui::DragValue::new(&mut p.period_ms).range(0.1..=60_000.0)).changed() {
                        changed.push(p.id);
                    }
                    if ui.small_button("🗑").clicked() {
                        remove = Some(p.id);
                    }
                    ui.end_row();
                }
            });
            for id in changed {
                if let Some(p) = self.periodic.iter().find(|p| p.id == id) {
                    self.send_cmd(Cmd::SetPeriodic {
                        id,
                        msg: p.msg,
                        period: Duration::from_secs_f64(p.period_ms / 1000.0),
                        enabled: p.enabled,
                    });
                }
            }
            if let Some(id) = remove {
                self.periodic.retain(|p| p.id != id);
                self.send_cmd(Cmd::RemovePeriodic(id));
            }

            ui.add_space(12.0);
            ui.separator();
            ui.heading("Log playback (candump)");
            ui.label(
                RichText::new("Transmit a recorded candump log (e.g. from this app or `candump -l`) onto the bus with original timing.")
                    .weak(),
            );
            ui.horizontal(|ui| {
                if ui.button("📂 Load log…").clicked()
                    && let Some(path) = rfd::FileDialog::new()
                        .add_filter("candump log", &["log", "txt", "candump"])
                        .add_filter("All files", &["*"])
                        .pick_file()
                    {
                        match candump::load_file(&path) {
                            Ok(l) if l.frames.is_empty() => self.error("No frames found in file"),
                            Ok(l) => {
                                self.loaded_log = Some(LoadedLog {
                                    duration: l.duration(),
                                    skipped: l.skipped,
                                    frames: Arc::new(l.frames),
                                    path,
                                });
                            }
                            Err(e) => self.error(format!("Failed to load log: {e:#}")),
                        }
                    }
                if let Some(l) = &self.loaded_log {
                    ui.label(format!(
                        "{} — {} frames, {:.2}s{}",
                        l.path.file_name().unwrap_or_default().to_string_lossy(),
                        l.frames.len(),
                        l.duration,
                        if l.skipped > 0 { format!(", {} unparsable lines skipped", l.skipped) } else { String::new() }
                    ));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Speed");
                ui.add(egui::DragValue::new(&mut self.replay_speed).range(0.01..=100.0).speed(0.05).suffix("x"));
                ui.checkbox(&mut self.replay_loop, "Loop");
                let replay = self.shared.stats.lock().replay.clone();
                if let Some(p) = replay {
                    if ui.button("⏹ Stop").clicked() {
                        self.send_cmd(Cmd::StopReplay);
                    }
                    let frac = if p.total > 0 { p.index as f32 / p.total as f32 } else { 0.0 };
                    let text = if p.looped {
                        format!("{}/{} (loop {})", p.index, p.total, p.loops + 1)
                    } else {
                        format!("{}/{}", p.index, p.total)
                    };
                    ui.add(egui::ProgressBar::new(frac).text(text).desired_width(300.0));
                } else if ui
                    .add_enabled(connected && self.loaded_log.is_some(), egui::Button::new("▶ Play"))
                    .clicked()
                    && let Some(l) = &self.loaded_log {
                        self.send_cmd(Cmd::StartReplay {
                            frames: l.frames.clone(),
                            speed: self.replay_speed,
                            looped: self.replay_loop,
                        });
                    }
            });
            if self.loaded_log.as_ref().is_some_and(|l| l.frames.iter().any(|m| m.fd)) && !self.cfg.fd {
                ui.label(
                    RichText::new("This log contains CAN FD frames; connect with CAN FD enabled to send them.")
                        .color(ui.visuals().warn_fg_color),
                );
            }
        });
    }

    // -----------------------------------------------------------------------
    // Bus status tab
    // -----------------------------------------------------------------------

    fn status_tab(&mut self, ui: &mut egui::Ui) {
        let st = self.shared.stats.lock().clone();
        let dropped = self.shared.stream_dropped.load(Ordering::Relaxed);

        ui.horizontal(|ui| {
            ui.label(
                RichText::new(st.state.label())
                    .size(26.0)
                    .strong()
                    .color(state_color(st.state)),
            );
            if st.load_pct >= 80.0 {
                ui.label(
                    RichText::new("  HEAVY BUS LOAD")
                        .size(20.0)
                        .strong()
                        .color(ui.visuals().warn_fg_color),
                );
            }
            if st.finished {
                ui.label(RichText::new("  (end of log)").size(18.0).weak());
            }
        });
        ui.add_space(4.0);

        ui.columns(2, |cols| {
            egui::Grid::new("statgrid")
                .num_columns(2)
                .spacing([24.0, 4.0])
                .striped(true)
                .show(&mut cols[0], |ui| {
                    let row = |ui: &mut egui::Ui, k: &str, v: String| {
                        ui.label(k);
                        ui.monospace(v);
                        ui.end_row();
                    };
                    row(
                        ui,
                        "Interface",
                        if st.connected {
                            st.iface.clone()
                        } else {
                            "(disconnected)".into()
                        },
                    );
                    row(ui, "Bitrate", fmt_bitrate(st.bitrate));
                    row(
                        ui,
                        "Bus load",
                        format!("{:.1} %  (peak {:.1} %)", st.load_pct, st.peak_load_pct),
                    );
                    row(ui, "Frame rate", format!("{:.0} frames/s", st.fps));
                    row(ui, "RX frames", st.rx_total.to_string());
                    row(ui, "TX frames", st.tx_total.to_string());
                    row(ui, "TX errors", st.tx_errors.to_string());
                    row(ui, "Error / status events", st.error_frames.to_string());
                    row(ui, "Controller overruns", st.rx_overflows.to_string());
                    row(
                        ui,
                        "TEC / REC",
                        format!(
                            "{} / {}",
                            st.tec.map_or("-".into(), |v| v.to_string()),
                            st.rec.map_or("-".into(), |v| v.to_string())
                        ),
                    );
                    row(ui, "Frames skipped by stream view", dropped.to_string());
                });
            let ui = &mut cols[1];
            ui.label("Bus load (%)");
            ui.add(
                egui::ProgressBar::new((st.load_pct / 100.0).clamp(0.0, 1.0) as f32)
                    .text(format!("{:.1}%", st.load_pct)),
            );
            ui.add_space(4.0);
            let load: Vec<f64> = st.history.iter().map(|p| p.load).collect();
            sparkline(
                ui,
                &load,
                100.0_f64.max(load.iter().cloned().fold(0.0, f64::max)),
                Color32::from_rgb(80, 150, 230),
                "load % — last 5 min",
            );
            let fps: Vec<f64> = st.history.iter().map(|p| p.fps).collect();
            sparkline(
                ui,
                &fps,
                fps.iter().cloned().fold(1.0, f64::max) * 1.1,
                Color32::from_rgb(90, 190, 120),
                "frames/s — last 5 min",
            );
        });

        ui.separator();
        ui.horizontal(|ui| {
            ui.heading("Events");
            if ui.small_button("Clear").clicked() {
                self.shared.stats.lock().events.clear();
            }
        });
        let events: Vec<_> = st.events.iter().rev().cloned().collect();
        let err = ui.visuals().error_fg_color;
        TableBuilder::new(ui)
            .striped(true)
            .column(Column::initial(130.0))
            .column(Column::remainder())
            .body(|body| {
                body.rows(ROW_H, events.len(), |mut row| {
                    let e = &events[row.index()];
                    row.col(|ui| {
                        let t = fmt_abs_time(e.ts);
                        ui.monospace(t.get(..12).unwrap_or(&t));
                    });
                    row.col(|ui| {
                        if e.severe {
                            ui.label(RichText::new(&e.text).color(err));
                        } else {
                            ui.label(&e.text);
                        }
                    });
                });
            });
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_bus();

        // Detect the bus thread dying on its own (e.g. device unplugged).
        if self.bus.is_some() && !self.connecting && !self.shared.stats.lock().connected {
            self.disconnect();
            self.error("Bus connection closed");
        }

        egui::TopBottomPanel::top("top").show(ctx, |ui| {
            ui.add_space(4.0);
            self.top_bar(ui);
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.tab, Tab::Summary, "📋 Summary");
                ui.selectable_value(&mut self.tab, Tab::Stream, "📜 Stream");
                ui.selectable_value(&mut self.tab, Tab::Transmit, "📤 Transmit");
                ui.selectable_value(&mut self.tab, Tab::Status, "🩺 Bus status");
            });
            ui.add_space(2.0);
        });
        egui::TopBottomPanel::bottom("status").show(ctx, |ui| {
            ui.add_space(2.0);
            self.status_bar(ui);
            ui.add_space(2.0);
        });
        egui::CentralPanel::default().show(ctx, |ui| match self.tab {
            Tab::Stream => self.stream_tab(ui),
            Tab::Summary => self.summary_tab(ui),
            Tab::Transmit => self.transmit_tab(ui),
            Tab::Status => self.status_tab(ui),
        });

        ctx.request_repaint_after(Duration::from_millis(if self.bus.is_some() {
            33
        } else {
            250
        }));
    }
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

trait CloneLight {
    fn clone_light(&self) -> bus::BusStats;
}

impl CloneLight for bus::BusStats {
    /// Clone without the (potentially large) history/event buffers.
    fn clone_light(&self) -> bus::BusStats {
        bus::BusStats {
            connected: self.connected,
            iface: self.iface.clone(),
            bitrate: self.bitrate,
            state: self.state,
            tec: self.tec,
            rec: self.rec,
            rx_total: self.rx_total,
            tx_total: self.tx_total,
            tx_errors: self.tx_errors,
            error_frames: self.error_frames,
            rx_overflows: self.rx_overflows,
            fps: self.fps,
            load_pct: self.load_pct,
            peak_load_pct: self.peak_load_pct,
            recording: self.recording.clone(),
            replay: self.replay.clone(),
            finished: self.finished,
            ..Default::default()
        }
    }
}

fn signals_grid(ui: &mut egui::Ui, sigs: &[crate::symbols::DecodedSignal], id: &str) {
    egui::Grid::new(id)
        .striped(true)
        .num_columns(4)
        .show(ui, |ui| {
            ui.strong("Signal");
            ui.strong("Value");
            ui.strong("Unit");
            ui.strong("Raw");
            ui.end_row();
            for s in sigs {
                ui.label(&s.name);
                ui.monospace(s.display_value());
                ui.label(&s.unit);
                ui.monospace(format!("0x{:X}", s.raw));
                ui.end_row();
            }
        });
}

fn sparkline(ui: &mut egui::Ui, values: &[f64], max: f64, color: Color32, label: &str) {
    let width = ui.available_width();
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, 70.0), Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 3.0, ui.visuals().extreme_bg_color);
    painter.text(
        rect.left_top() + egui::vec2(4.0, 2.0),
        egui::Align2::LEFT_TOP,
        format!("{label} (max {max:.0})"),
        egui::FontId::proportional(11.0),
        ui.visuals().weak_text_color(),
    );
    if values.len() >= 2 && max > 0.0 {
        let n = bus::HISTORY_SECS;
        let dx = rect.width() / (n - 1) as f32;
        let offset = n - values.len();
        let pts: Vec<egui::Pos2> = values
            .iter()
            .enumerate()
            .map(|(i, v)| {
                let x = rect.left() + (offset + i) as f32 * dx;
                let y =
                    rect.bottom() - (v / max).clamp(0.0, 1.0) as f32 * (rect.height() - 16.0) - 2.0;
                egui::pos2(x, y)
            })
            .collect();
        painter.add(egui::Shape::line(pts, egui::Stroke::new(1.5, color)));
    }
    ui.add_space(4.0);
}

fn dir_label(ui: &mut egui::Ui, d: Dir) {
    match d {
        Dir::Rx => ui.label(RichText::new("Rx").weak()),
        Dir::Tx => ui.label(RichText::new("Tx").color(Color32::from_rgb(80, 150, 230))),
    };
}

fn fmt_id(m: &CanMsg) -> String {
    let mut s = m.key().fmt_id();
    if m.fd {
        s.push_str(if m.brs { " FD+" } else { " FD" });
    }
    s
}

fn fmt_abs_time(ts: f64) -> String {
    use chrono::TimeZone;
    let secs = ts.floor() as i64;
    let nanos = ((ts - ts.floor()) * 1e9) as u32;
    match chrono::Local.timestamp_opt(secs, nanos) {
        chrono::LocalResult::Single(t) => t.format("%H:%M:%S%.6f").to_string(),
        _ => format!("{ts:.6}"),
    }
}

fn fmt_bitrate(b: u32) -> String {
    if b == 0 {
        "-".into()
    } else if b % 1_000_000 == 0 {
        format!("{} Mbit/s", b / 1_000_000)
    } else {
        format!("{} kbit/s", b / 1000)
    }
}

fn speed_label(s: f64) -> String {
    if s == 0.0 {
        "max".into()
    } else {
        format!("{s}x")
    }
}

fn state_color(s: CtrlState) -> Color32 {
    match s {
        CtrlState::Unknown => Color32::GRAY,
        CtrlState::ErrorActive => Color32::from_rgb(60, 180, 90),
        CtrlState::Warning => Color32::from_rgb(220, 190, 40),
        CtrlState::ErrorPassive => Color32::from_rgb(240, 130, 30),
        CtrlState::BusOff => Color32::from_rgb(230, 50, 50),
    }
}

fn load_color(pct: f64, ui: &egui::Ui) -> Color32 {
    if pct >= 80.0 {
        ui.visuals().error_fg_color
    } else if pct >= 50.0 {
        ui.visuals().warn_fg_color
    } else {
        ui.visuals().text_color()
    }
}

fn mix(a: Color32, b: Color32, t: f32) -> Color32 {
    let t = t.clamp(0.0, 1.0);
    let l = |x: u8, y: u8| (x as f32 + (y as f32 - x as f32) * t) as u8;
    Color32::from_rgb(l(a.r(), b.r()), l(a.g(), b.g()), l(a.b(), b.b()))
}
