//! Hardware / virtual CAN interfaces behind a single trait.
//!
//! Every device is created *inside* the bus thread, so devices do not need to
//! be `Send` and the UI thread never touches hardware directly.

use std::collections::VecDeque;
use std::io::{Read as _, Write as _};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context, anyhow, bail};

use crate::msg::{CanMsg, Dir, now_ts};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CtrlState {
    #[default]
    Unknown,
    ErrorActive,
    Warning,
    ErrorPassive,
    BusOff,
}

impl CtrlState {
    pub fn label(self) -> &'static str {
        match self {
            CtrlState::Unknown => "Unknown",
            CtrlState::ErrorActive => "Error Active (OK)",
            CtrlState::Warning => "Error Warning",
            CtrlState::ErrorPassive => "Error Passive",
            CtrlState::BusOff => "BUS OFF",
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct DeviceStatus {
    pub state: CtrlState,
    pub tec: Option<u8>,
    pub rec: Option<u8>,
}

/// Something noteworthy reported by the controller (error frame, overrun, ...).
#[derive(Debug, Clone)]
pub struct BusEvent {
    pub desc: String,
    pub state: Option<CtrlState>,
    pub rx_overflow: bool,
    pub tec: Option<u8>,
    pub rec: Option<u8>,
}

impl BusEvent {
    pub fn new(desc: impl Into<String>) -> Self {
        Self {
            desc: desc.into(),
            state: None,
            rx_overflow: false,
            tec: None,
            rec: None,
        }
    }
}

pub enum DevEvent {
    Frame(CanMsg),
    Bus(BusEvent),
}

pub trait CanDevice {
    /// Waits up to `timeout` for the next frame/event. A zero timeout must
    /// return immediately.
    fn recv(&mut self, timeout: Duration) -> anyhow::Result<Option<DevEvent>>;
    fn send(&mut self, msg: &CanMsg) -> anyhow::Result<()>;
    fn status(&mut self) -> Option<DeviceStatus> {
        None
    }
    /// Short interface name used in candump log files (e.g. `can0`, `pcan1`).
    fn log_name(&self) -> String;
    /// Log replay devices finish; hardware never does.
    fn finished(&self) -> bool {
        false
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackendKind {
    Virtual,
    Pcan,
    Kvaser,
    #[cfg(target_os = "linux")]
    SocketCan,
    Slcan,
    LogFile,
}

impl BackendKind {
    pub fn all() -> Vec<BackendKind> {
        let mut v = vec![BackendKind::Virtual, BackendKind::Pcan];
        if cfg!(any(target_os = "windows", target_os = "linux")) {
            v.push(BackendKind::Kvaser);
        }
        #[cfg(target_os = "linux")]
        v.push(BackendKind::SocketCan);
        v.push(BackendKind::Slcan);
        v.push(BackendKind::LogFile);
        v
    }

    pub fn label(self) -> &'static str {
        match self {
            BackendKind::Virtual => "Virtual (simulated)",
            BackendKind::Pcan => "PEAK PCAN",
            BackendKind::Kvaser => "Kvaser CANlib",
            #[cfg(target_os = "linux")]
            BackendKind::SocketCan => "SocketCAN",
            BackendKind::Slcan => "SLCAN (serial)",
            BackendKind::LogFile => "Log file (offline review)",
        }
    }

    pub fn has_bitrate(self) -> bool {
        matches!(
            self,
            BackendKind::Pcan | BackendKind::Kvaser | BackendKind::Slcan
        )
    }

    pub fn supports_fd(self) -> bool {
        match self {
            BackendKind::Pcan | BackendKind::Kvaser => true,
            #[cfg(target_os = "linux")]
            BackendKind::SocketCan => true,
            _ => false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PcanBus {
    Usb,
    Pci,
    Lan,
}

#[derive(Debug, Clone)]
pub struct ConnectConfig {
    pub kind: BackendKind,
    /// Interface / port name / virtual mode, depending on the backend.
    pub channel: String,
    /// 0-based channel index for PCAN / Kvaser.
    pub index: u32,
    pub pcan_bus: PcanBus,
    pub bitrate: u32,
    pub fd: bool,
    pub data_bitrate: u32,
    pub log_path: Option<PathBuf>,
    /// Log replay speed multiplier; 0 = as fast as possible.
    pub log_speed: f64,
}

impl Default for ConnectConfig {
    fn default() -> Self {
        Self {
            kind: BackendKind::Virtual,
            channel: "demo".into(),
            index: 0,
            pcan_bus: PcanBus::Usb,
            bitrate: 500_000,
            fd: false,
            data_bitrate: 2_000_000,
            log_path: None,
            log_speed: 1.0,
        }
    }
}

pub fn open(cfg: &ConnectConfig) -> anyhow::Result<Box<dyn CanDevice>> {
    match cfg.kind {
        BackendKind::Virtual => Ok(Box::new(VirtualDevice::new(&cfg.channel))),
        BackendKind::Pcan => hal::open_pcan(cfg),
        BackendKind::Kvaser => hal::open_kvaser(cfg),
        #[cfg(target_os = "linux")]
        BackendKind::SocketCan => Ok(Box::new(socketcan_dev::SocketCanDevice::open(
            &cfg.channel,
        )?)),
        BackendKind::Slcan => Ok(Box::new(SlcanDevice::open(&cfg.channel, cfg.bitrate)?)),
        BackendKind::LogFile => {
            let p = cfg
                .log_path
                .as_ref()
                .ok_or_else(|| anyhow!("no log file selected"))?;
            Ok(Box::new(LogFileDevice::open(p, cfg.log_speed)?))
        }
    }
}

// ---------------------------------------------------------------------------
// can-hal backends (PCAN, Kvaser)
// ---------------------------------------------------------------------------

mod hal {
    use super::*;
    use can_hal::{
        BusState, BusStatus, CanFdFrame, CanFrame, CanId, Frame, Receive, ReceiveFd, Transmit,
        TransmitFd,
    };

    fn hal_id(msg: &CanMsg) -> anyhow::Result<CanId> {
        let id = if msg.ext {
            CanId::new_extended(msg.id)
        } else {
            u16::try_from(msg.id).ok().and_then(CanId::new_standard)
        };
        id.ok_or_else(|| anyhow!("invalid CAN id {:X}", msg.id))
    }

    fn to_classic(msg: &CanMsg) -> anyhow::Result<CanFrame> {
        if msg.rtr {
            bail!("remote frames are not supported by this backend");
        }
        if msg.fd {
            bail!("CAN FD frame on a classic channel (enable FD when connecting)");
        }
        CanFrame::new(hal_id(msg)?, msg.data()).ok_or_else(|| anyhow!("invalid classic frame"))
    }

    fn from_parts(id: CanId, data: &[u8], fd: bool, brs: bool) -> CanMsg {
        let mut m = CanMsg::new(id.raw(), id.is_extended(), data);
        m.fd = fd;
        m.brs = brs;
        m.ts = now_ts();
        m.dir = Dir::Rx;
        m
    }

    fn map_status<S: BusStatus>(ch: &S) -> Option<DeviceStatus> {
        let state = match ch.bus_state().ok()? {
            BusState::ErrorActive => CtrlState::ErrorActive,
            BusState::ErrorPassive => CtrlState::ErrorPassive,
            BusState::BusOff => CtrlState::BusOff,
        };
        let (tec, rec) = match ch.error_counters() {
            Ok(c) => (Some(c.transmit), Some(c.receive)),
            Err(_) => (None, None),
        };
        // Refine "active" into "warning" when counters say so.
        let state = match (state, tec, rec) {
            (CtrlState::ErrorActive, Some(t), Some(r)) if t >= 96 || r >= 96 => CtrlState::Warning,
            (s, _, _) => s,
        };
        Some(DeviceStatus { state, tec, rec })
    }

    pub struct Classic<C> {
        pub ch: C,
        pub name: String,
    }

    impl<C: Receive + Transmit + BusStatus> CanDevice for Classic<C> {
        fn recv(&mut self, timeout: Duration) -> anyhow::Result<Option<DevEvent>> {
            match self.ch.receive_timeout(timeout) {
                Ok(Some(ts)) => {
                    let f = ts.frame();
                    Ok(Some(DevEvent::Frame(from_parts(
                        f.id(),
                        f.data(),
                        false,
                        false,
                    ))))
                }
                Ok(None) => Ok(None),
                Err(e) => Err(anyhow::Error::new(e)),
            }
        }
        fn send(&mut self, msg: &CanMsg) -> anyhow::Result<()> {
            let f = to_classic(msg)?;
            self.ch.transmit(&f).map_err(anyhow::Error::new)
        }
        fn status(&mut self) -> Option<DeviceStatus> {
            map_status(&self.ch)
        }
        fn log_name(&self) -> String {
            self.name.clone()
        }
    }

    pub struct Fd<C> {
        pub ch: C,
        pub name: String,
    }

    impl<C: ReceiveFd + TransmitFd + BusStatus> CanDevice for Fd<C> {
        fn recv(&mut self, timeout: Duration) -> anyhow::Result<Option<DevEvent>> {
            match self.ch.receive_fd_timeout(timeout) {
                Ok(Some(ts)) => {
                    let m = match ts.frame() {
                        Frame::Can(f) => from_parts(f.id(), f.data(), false, false),
                        Frame::Fd(f) => from_parts(f.id(), f.data(), true, f.brs()),
                    };
                    Ok(Some(DevEvent::Frame(m)))
                }
                Ok(None) => Ok(None),
                Err(e) => Err(anyhow::Error::new(e)),
            }
        }
        fn send(&mut self, msg: &CanMsg) -> anyhow::Result<()> {
            if msg.rtr {
                bail!("remote frames are not supported by this backend");
            }
            // can-hal FD channels transmit FD frames only; classic payloads
            // (<= 8 bytes) are still sent as FD frames without BRS.
            let f = CanFdFrame::new(hal_id(msg)?, msg.data(), msg.brs, false)
                .ok_or_else(|| anyhow!("invalid FD length {}", msg.len))?;
            self.ch.transmit_fd(&f).map_err(anyhow::Error::new)
        }
        fn status(&mut self) -> Option<DeviceStatus> {
            map_status(&self.ch)
        }
        fn log_name(&self) -> String {
            self.name.clone()
        }
    }

    pub fn open_pcan(cfg: &ConnectConfig) -> anyhow::Result<Box<dyn CanDevice>> {
        use can_hal_pcan::{ClassicBitrate, PcanBusType, PcanDriver};

        // macOS uses the MacCAN PCBUSB library which exposes the PCAN-Basic API.
        let driver = if cfg!(target_os = "macos") {
            PcanDriver::with_library_path("libPCBUSB.dylib")
        } else {
            PcanDriver::new()
        }
        .context("loading PCAN-Basic library (is the PEAK driver installed?)")?;

        let bus = match cfg.pcan_bus {
            PcanBus::Usb => PcanBusType::Usb,
            PcanBus::Pci => PcanBusType::Pci,
            PcanBus::Lan => PcanBusType::Lan,
        };
        let builder = driver.channel_on_bus(bus, cfg.index)?;
        let name = format!("pcan{}", cfg.index + 1);
        if cfg.fd {
            let ch = builder.fd(cfg.bitrate, cfg.data_bitrate)?.connect()?;
            Ok(Box::new(Fd { ch, name }))
        } else {
            let br = match cfg.bitrate {
                1_000_000 => ClassicBitrate::Br1M,
                800_000 => ClassicBitrate::Br800K,
                500_000 => ClassicBitrate::Br500K,
                250_000 => ClassicBitrate::Br250K,
                125_000 => ClassicBitrate::Br125K,
                100_000 => ClassicBitrate::Br100K,
                50_000 => ClassicBitrate::Br50K,
                20_000 => ClassicBitrate::Br20K,
                10_000 => ClassicBitrate::Br10K,
                5_000 => ClassicBitrate::Br5K,
                other => bail!("PCAN does not support classic bitrate {other}"),
            };
            let ch = builder.classic(br).connect()?;
            Ok(Box::new(Classic { ch, name }))
        }
    }

    pub fn open_kvaser(cfg: &ConnectConfig) -> anyhow::Result<Box<dyn CanDevice>> {
        use can_hal_kvaser::KvaserDriver;
        let driver = KvaserDriver::new()
            .context("loading Kvaser CANlib (is the Kvaser driver installed?)")?;
        let name = format!("kvaser{}", cfg.index);
        if cfg.fd {
            let ch = driver
                .channel(cfg.index)
                .fd(cfg.bitrate, cfg.data_bitrate)?
                .connect()?;
            Ok(Box::new(Fd { ch, name }))
        } else {
            let ch = driver.channel(cfg.index).classic(cfg.bitrate)?.connect()?;
            Ok(Box::new(Classic { ch, name }))
        }
    }
}

// ---------------------------------------------------------------------------
// SocketCAN (Linux)
// ---------------------------------------------------------------------------

#[cfg(target_os = "linux")]
pub fn socketcan_interfaces() -> Vec<String> {
    // ARPHRD_CAN = 280
    let mut out = Vec::new();
    if let Ok(rd) = std::fs::read_dir("/sys/class/net") {
        for e in rd.flatten() {
            let ty = std::fs::read_to_string(e.path().join("type")).unwrap_or_default();
            if ty.trim() == "280" {
                out.push(e.file_name().to_string_lossy().into_owned());
            }
        }
    }
    out.sort();
    out
}

#[cfg(target_os = "linux")]
mod socketcan_dev {
    use super::*;
    use socketcan::{
        CanAnyFrame, CanDataFrame, CanError, CanFdFrame, CanFdSocket, CanRemoteFrame,
        EmbeddedFrame, ExtendedId, Frame as _, Id, Socket, SocketOptions, StandardId,
        errors::ControllerProblem,
    };

    pub struct SocketCanDevice {
        sock: CanFdSocket,
        name: String,
        state: CtrlState,
        tec: Option<u8>,
        rec: Option<u8>,
    }

    impl SocketCanDevice {
        pub fn open(name: &str) -> anyhow::Result<Self> {
            let sock = CanFdSocket::open(name)
                .with_context(|| format!("opening SocketCAN interface '{name}'"))?;
            sock.set_error_filter_accept_all()?;
            Ok(Self {
                sock,
                name: name.to_string(),
                state: CtrlState::ErrorActive,
                tec: None,
                rec: None,
            })
        }
    }

    fn id_of(id: Id) -> (u32, bool) {
        match id {
            Id::Standard(s) => (s.as_raw() as u32, false),
            Id::Extended(e) => (e.as_raw(), true),
        }
    }

    fn sc_id(msg: &CanMsg) -> anyhow::Result<Id> {
        Ok(if msg.ext {
            Id::Extended(ExtendedId::new(msg.id).ok_or_else(|| anyhow!("bad id"))?)
        } else {
            Id::Standard(StandardId::new(msg.id as u16).ok_or_else(|| anyhow!("bad id"))?)
        })
    }

    impl CanDevice for SocketCanDevice {
        fn recv(&mut self, timeout: Duration) -> anyhow::Result<Option<DevEvent>> {
            let frame = match self.sock.read_frame_timeout(timeout) {
                Ok(f) => f,
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                    ) =>
                {
                    return Ok(None);
                }
                Err(e) => return Err(e.into()),
            };
            let msg = match frame {
                CanAnyFrame::Normal(f) => {
                    let (id, ext) = id_of(f.id());
                    CanMsg::new(id, ext, f.data())
                }
                CanAnyFrame::Remote(f) => {
                    let (id, ext) = id_of(f.id());
                    let mut m = CanMsg::new(id, ext, &[]);
                    m.rtr = true;
                    m.len = f.dlc() as u8;
                    m
                }
                CanAnyFrame::Fd(f) => {
                    let (id, ext) = id_of(f.id());
                    let mut m = CanMsg::new(id, ext, f.data());
                    m.fd = true;
                    m.brs = f.is_brs();
                    m
                }
                CanAnyFrame::Error(ef) => {
                    let data = ef.data().to_vec();
                    let has_cnt = ef.raw_id() & 0x200 != 0;
                    let err: CanError = ef.into_error();
                    let mut ev = BusEvent::new(err.to_string());
                    match err {
                        CanError::BusOff => ev.state = Some(CtrlState::BusOff),
                        CanError::Restarted => ev.state = Some(CtrlState::ErrorActive),
                        CanError::ControllerProblem(p) => match p {
                            ControllerProblem::ReceiveBufferOverflow
                            | ControllerProblem::TransmitBufferOverflow => ev.rx_overflow = true,
                            ControllerProblem::ReceiveErrorWarning
                            | ControllerProblem::TransmitErrorWarning => {
                                ev.state = Some(CtrlState::Warning)
                            }
                            ControllerProblem::ReceiveErrorPassive
                            | ControllerProblem::TransmitErrorPassive => {
                                ev.state = Some(CtrlState::ErrorPassive)
                            }
                            ControllerProblem::Active => ev.state = Some(CtrlState::ErrorActive),
                            _ => {}
                        },
                        _ => {}
                    }
                    if has_cnt && data.len() >= 8 {
                        ev.tec = Some(data[6]);
                        ev.rec = Some(data[7]);
                        self.tec = ev.tec;
                        self.rec = ev.rec;
                    }
                    if let Some(s) = ev.state {
                        self.state = s;
                    }
                    return Ok(Some(DevEvent::Bus(ev)));
                }
            };
            let mut msg = msg;
            msg.ts = now_ts();
            Ok(Some(DevEvent::Frame(msg)))
        }

        fn send(&mut self, msg: &CanMsg) -> anyhow::Result<()> {
            let id = sc_id(msg)?;
            if msg.fd {
                let mut f =
                    CanFdFrame::new(id, msg.data()).ok_or_else(|| anyhow!("invalid FD frame"))?;
                f.set_brs(msg.brs);
                self.sock.write_frame(&f)?;
            } else if msg.rtr {
                let f = CanRemoteFrame::new_remote(id, msg.len as usize)
                    .ok_or_else(|| anyhow!("invalid RTR"))?;
                self.sock.write_frame(&f)?;
            } else {
                let f =
                    CanDataFrame::new(id, msg.data()).ok_or_else(|| anyhow!("invalid frame"))?;
                self.sock.write_frame(&f)?;
            }
            Ok(())
        }

        fn status(&mut self) -> Option<DeviceStatus> {
            Some(DeviceStatus {
                state: self.state,
                tec: self.tec,
                rec: self.rec,
            })
        }

        fn log_name(&self) -> String {
            self.name.clone()
        }
    }
}

// ---------------------------------------------------------------------------
// SLCAN (Lawicel ASCII over serial) - CANable, USBtin, etc.
// ---------------------------------------------------------------------------

pub fn serial_ports() -> Vec<String> {
    serialport::available_ports()
        .map(|v| v.into_iter().map(|p| p.port_name).collect())
        .unwrap_or_default()
}

pub struct SlcanDevice {
    port: Box<dyn serialport::SerialPort>,
    name: String,
    buf: Vec<u8>,
    status: DeviceStatus,
    last_status_poll: Instant,
    pending_events: VecDeque<DevEvent>,
}

impl SlcanDevice {
    pub fn open(path: &str, bitrate: u32) -> anyhow::Result<Self> {
        let code = match bitrate {
            10_000 => '0',
            20_000 => '1',
            50_000 => '2',
            100_000 => '3',
            125_000 => '4',
            250_000 => '5',
            500_000 => '6',
            800_000 => '7',
            1_000_000 => '8',
            other => bail!("SLCAN does not support bitrate {other}"),
        };
        let mut port = serialport::new(path, 115_200)
            .timeout(Duration::from_millis(50))
            .open()
            .with_context(|| format!("opening serial port {path}"))?;
        port.write_all(b"\r\r\rC\r")?;
        std::thread::sleep(Duration::from_millis(50));
        let _ = port.clear(serialport::ClearBuffer::Input);
        port.write_all(format!("S{code}\r").as_bytes())?;
        port.write_all(b"O\r")?;
        port.flush()?;
        let name = format!(
            "slcan{}",
            path.chars()
                .filter(|c| c.is_ascii_alphanumeric())
                .collect::<String>()
                .to_lowercase()
        );
        Ok(Self {
            port,
            name,
            buf: Vec::with_capacity(4096),
            status: DeviceStatus {
                state: CtrlState::ErrorActive,
                tec: None,
                rec: None,
            },
            last_status_poll: Instant::now(),
            pending_events: VecDeque::new(),
        })
    }

    fn parse_line(&mut self, line: &[u8]) -> Option<DevEvent> {
        let s = std::str::from_utf8(line).ok()?;
        let kind = s.chars().next()?;
        let hex = |s: &str| u32::from_str_radix(s, 16).ok();
        let (ext, rtr, fd, brs, id_len) = match kind {
            't' => (false, false, false, false, 3),
            'T' => (true, false, false, false, 8),
            'r' => (false, true, false, false, 3),
            'R' => (true, true, false, false, 8),
            'd' => (false, false, true, false, 3),
            'D' => (true, false, true, false, 8),
            'b' => (false, false, true, true, 3),
            'B' => (true, false, true, true, 8),
            'F' => {
                let flags = hex(s.get(1..3)?)?;
                return self.status_flags(flags as u8);
            }
            _ => return None,
        };
        let id = hex(s.get(1..1 + id_len)?)?;
        let dlc_c = s.get(1 + id_len..2 + id_len)?;
        let dlc = hex(dlc_c)? as usize;
        let len = if fd {
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64][dlc.min(15)]
        } else {
            dlc.min(8)
        };
        let mut m = CanMsg::new(id, ext, &[]);
        if rtr {
            m.rtr = true;
            m.len = len as u8;
        } else {
            let d = s.get(2 + id_len..2 + id_len + len * 2)?;
            for i in 0..len {
                m.data[i] = u8::from_str_radix(&d[i * 2..i * 2 + 2], 16).ok()?;
            }
            m.len = len as u8;
        }
        m.fd = fd;
        m.brs = brs;
        m.ts = now_ts();
        Some(DevEvent::Frame(m))
    }

    fn status_flags(&mut self, flags: u8) -> Option<DevEvent> {
        let prev = self.status.state;
        self.status.state = if flags & 0x20 != 0 {
            CtrlState::ErrorPassive
        } else if flags & 0x04 != 0 {
            CtrlState::Warning
        } else {
            CtrlState::ErrorActive
        };
        if flags == 0 && prev == self.status.state {
            return None;
        }
        let mut parts = Vec::new();
        for (bit, name) in [
            (0x01, "RX FIFO full"),
            (0x02, "TX FIFO full"),
            (0x04, "error warning"),
            (0x08, "data overrun"),
            (0x20, "error passive"),
            (0x40, "arbitration lost"),
            (0x80, "bus error"),
        ] {
            if flags & bit != 0 {
                parts.push(name);
            }
        }
        let mut ev = BusEvent::new(if parts.is_empty() {
            "status OK".to_string()
        } else {
            parts.join(", ")
        });
        ev.state = Some(self.status.state);
        ev.rx_overflow = flags & 0x09 != 0;
        Some(DevEvent::Bus(ev))
    }

    fn take_line(&mut self) -> Option<Vec<u8>> {
        loop {
            let pos = self.buf.iter().position(|&b| b == b'\r' || b == 0x07)?;
            let line: Vec<u8> = self.buf.drain(..=pos).collect();
            if line.last() == Some(&0x07) {
                self.pending_events.push_back(DevEvent::Bus(BusEvent::new(
                    "SLCAN adapter reported an error (BELL)",
                )));
            }
            let line = &line[..line.len() - 1];
            let line: Vec<u8> = line.iter().copied().filter(|b| *b != b'\n').collect();
            if !line.is_empty() {
                return Some(line);
            }
        }
    }
}

impl CanDevice for SlcanDevice {
    fn recv(&mut self, timeout: Duration) -> anyhow::Result<Option<DevEvent>> {
        if self.last_status_poll.elapsed() > Duration::from_millis(500) {
            self.last_status_poll = Instant::now();
            let _ = self.port.write_all(b"F\r");
        }
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(ev) = self.pending_events.pop_front() {
                return Ok(Some(ev));
            }
            while let Some(line) = self.take_line() {
                if let Some(ev) = self.parse_line(&line) {
                    return Ok(Some(ev));
                }
            }
            let avail = self.port.bytes_to_read().unwrap_or(0) as usize;
            let remaining = deadline.saturating_duration_since(Instant::now());
            if avail == 0 && remaining.is_zero() {
                return Ok(None);
            }
            let mut tmp = [0u8; 2048];
            if avail == 0 {
                self.port
                    .set_timeout(remaining.max(Duration::from_millis(1)))?;
            }
            match self.port.read(&mut tmp) {
                Ok(0) => {}
                Ok(n) => self.buf.extend_from_slice(&tmp[..n]),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(e.into()),
            }
        }
    }

    fn send(&mut self, msg: &CanMsg) -> anyhow::Result<()> {
        use std::fmt::Write as _;
        let mut s = String::new();
        let c = match (msg.fd, msg.brs, msg.rtr, msg.ext) {
            (true, true, _, false) => 'b',
            (true, true, _, true) => 'B',
            (true, false, _, false) => 'd',
            (true, false, _, true) => 'D',
            (false, _, true, false) => 'r',
            (false, _, true, true) => 'R',
            (false, _, false, false) => 't',
            (false, _, false, true) => 'T',
        };
        s.push(c);
        if msg.ext {
            let _ = write!(s, "{:08X}", msg.id);
        } else {
            let _ = write!(s, "{:03X}", msg.id);
        }
        let dlc = match msg.len {
            0..=8 => msg.len,
            12 => 9,
            16 => 10,
            20 => 11,
            24 => 12,
            32 => 13,
            48 => 14,
            _ => 15,
        };
        let _ = write!(s, "{dlc:X}");
        if !msg.rtr {
            for b in msg.data() {
                let _ = write!(s, "{b:02X}");
            }
        }
        s.push('\r');
        self.port.write_all(s.as_bytes())?;
        Ok(())
    }

    fn status(&mut self) -> Option<DeviceStatus> {
        Some(self.status.clone())
    }

    fn log_name(&self) -> String {
        self.name.clone()
    }
}

impl Drop for SlcanDevice {
    fn drop(&mut self) {
        let _ = self.port.write_all(b"C\r");
    }
}

// ---------------------------------------------------------------------------
// Virtual bus: synthetic traffic for demos and testing without hardware.
// ---------------------------------------------------------------------------

struct SimMsg {
    id: u32,
    ext: bool,
    period: Duration,
    next: Instant,
    counter: u32,
}

pub struct VirtualDevice {
    msgs: Vec<SimMsg>,
    start: Instant,
    mode: String,
}

impl VirtualDevice {
    pub fn new(mode: &str) -> Self {
        let now = Instant::now();
        let ms = Duration::from_millis;
        let us = Duration::from_micros;
        let spec: Vec<(u32, bool, Duration)> = match mode {
            // ~10k frames/s: useful to verify that the summary never misses frames.
            "stress" => (0..20).map(|i| (0x100 + i, i % 5 == 0, us(2000))).collect(),
            "quiet" => vec![(0x100, false, ms(1000))],
            _ => vec![
                (0x0C0, false, ms(10)),
                (0x100, false, ms(10)),
                (0x200, false, ms(20)),
                (0x201, false, ms(50)),
                (0x300, false, ms(100)),
                (0x7E8, false, ms(1000)),
                (0x18FEF100, true, ms(100)),
                (0x0CF00400, true, ms(20)),
            ],
        };
        let msgs = spec
            .into_iter()
            .enumerate()
            .map(|(i, (id, ext, period))| SimMsg {
                id,
                ext,
                period,
                next: now + us(i as u64 * 137),
                counter: 0,
            })
            .collect();
        Self {
            msgs,
            start: now,
            mode: mode.to_string(),
        }
    }

    fn payload(&self, m: &SimMsg) -> Vec<u8> {
        let t = self.start.elapsed().as_secs_f64();
        let c = m.counter;
        match m.id {
            0x0C0 => vec![(c & 0xFF) as u8, (c >> 8) as u8],
            0x100 => {
                let rpm = (3000.0 + 2500.0 * (t * 0.5).sin()) / 0.25;
                let rpm = rpm as u16;
                let temp = (90.0 + 5.0 * (t * 0.05).sin() + 40.0) as u8;
                vec![
                    rpm as u8,
                    (rpm >> 8) as u8,
                    temp,
                    0,
                    0,
                    0,
                    0,
                    (c & 0xF) as u8,
                ]
            }
            0x200 => {
                let speed = ((60.0 + 40.0 * (t * 0.2).sin()) * 100.0) as u16;
                vec![
                    (speed >> 8) as u8,
                    speed as u8,
                    ((t as u32 / 5) % 7) as u8,
                    0,
                    0,
                    0,
                ]
            }
            0x201 => vec![(c % 4) as u8, (c % 256) as u8, 0xAA, 0x55],
            0x7E8 => vec![0x03, 0x41, 0x0C, (c % 256) as u8, 0, 0, 0, 0],
            _ => (0..8).map(|i| ((c + i) & 0xFF) as u8).collect(),
        }
    }
}

impl CanDevice for VirtualDevice {
    fn recv(&mut self, timeout: Duration) -> anyhow::Result<Option<DevEvent>> {
        let now = Instant::now();
        let (idx, next) = self
            .msgs
            .iter()
            .enumerate()
            .map(|(i, m)| (i, m.next))
            .min_by_key(|(_, n)| *n)
            .expect("virtual device has messages");
        if next > now {
            let wait = (next - now).min(timeout);
            if !wait.is_zero() {
                std::thread::sleep(wait);
            }
            if Instant::now() < next {
                return Ok(None);
            }
        }
        let data = self.payload(&self.msgs[idx]);
        let m = &mut self.msgs[idx];
        m.counter = m.counter.wrapping_add(1);
        m.next += m.period;
        // If we fell far behind (e.g. system sleep), resync instead of bursting.
        if Instant::now().saturating_duration_since(m.next) > Duration::from_secs(1) {
            m.next = Instant::now() + m.period;
        }
        let mut msg = CanMsg::new(m.id, m.ext, &data);
        msg.ts = now_ts();
        Ok(Some(DevEvent::Frame(msg)))
    }

    fn send(&mut self, _msg: &CanMsg) -> anyhow::Result<()> {
        Ok(())
    }

    fn status(&mut self) -> Option<DeviceStatus> {
        Some(DeviceStatus {
            state: CtrlState::ErrorActive,
            tec: Some(0),
            rec: Some(0),
        })
    }

    fn log_name(&self) -> String {
        format!("vcan_{}", self.mode)
    }
}

// ---------------------------------------------------------------------------
// Log file "device": plays back a recorded candump log as if it were a bus,
// preserving original timestamps so the summary shows real periods.
// ---------------------------------------------------------------------------

pub struct LogFileDevice {
    frames: Vec<CanMsg>,
    t0: f64,
    idx: usize,
    start: Instant,
    speed: f64,
    name: String,
}

impl LogFileDevice {
    pub fn open(path: &std::path::Path, speed: f64) -> anyhow::Result<Self> {
        let text =
            std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        let mut frames = Vec::new();
        let mut last = 0.0;
        for line in text.lines() {
            if let Ok(Some(mut m)) = crate::candump::parse_line(line) {
                if m.ts == 0.0 {
                    m.ts = last + 0.001;
                }
                last = m.ts;
                frames.push(m);
            }
        }
        if frames.is_empty() {
            bail!("no CAN frames found in {}", path.display());
        }
        let t0 = frames[0].ts;
        let name = path
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| "log".into());
        Ok(Self {
            frames,
            t0,
            idx: 0,
            start: Instant::now(),
            speed,
            name,
        })
    }
}

impl CanDevice for LogFileDevice {
    fn recv(&mut self, timeout: Duration) -> anyhow::Result<Option<DevEvent>> {
        let Some(m) = self.frames.get(self.idx) else {
            std::thread::sleep(timeout);
            return Ok(None);
        };
        if self.speed > 0.0 {
            let due =
                self.start + Duration::from_secs_f64(((m.ts - self.t0) / self.speed).max(0.0));
            let now = Instant::now();
            if due > now {
                std::thread::sleep((due - now).min(timeout));
                if Instant::now() < due {
                    return Ok(None);
                }
            }
        }
        self.idx += 1;
        Ok(Some(DevEvent::Frame(*m)))
    }

    fn send(&mut self, _msg: &CanMsg) -> anyhow::Result<()> {
        bail!("cannot transmit while reviewing a log file")
    }

    fn log_name(&self) -> String {
        self.name.clone()
    }

    fn finished(&self) -> bool {
        self.idx >= self.frames.len()
    }
}
