//! The radio side of the audio rings: a TCI connection, rig control plus a
//! sound card (`rig::thread`), or a loopback simulator that feeds our own
//! transmissions, noise and simulated stations back to the receiver.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender};
use ion_dsp::TX_LEVEL;
use ion_dsp::dsp::{AUDIO_RATE, NoiseGen, noise_sigma};
use rtrb::{Consumer, Producer};
use tci::{Command, Event, FrameHeader, RadioState};

use crate::engine::{Shared, UiEvent, discard, pop_slice, push_slice};
use crate::resample::{Downsampler, Upsampler};

#[derive(Clone, Debug, Default)]
pub struct RadioInfo {
    pub connected: bool,
    pub device: Option<String>,
    pub dial_hz: Option<i64>,
    pub ptt: bool,
    pub drive: Option<u32>,
    pub message: String,
    /// A radio setting that will stop Ion working (wrong mode, narrow filter).
    pub problem: Option<String>,
    /// The radio's power cannot be set from here: the drive slider does
    /// nothing.
    pub drive_unsupported: bool,
}

#[derive(Clone, Debug)]
pub enum LoopCmd {
    Ptt(bool),
    Drive(u32),
    SnrDb(f32),
    /// A simulated Ion station "LOOP" that answers calls and echoes text.
    IonPeer(bool),
    /// A simulated Ion net: `Some(true)` members joining ours,
    /// `Some(false)` a net with its own net control to join, `None` off.
    IonNet(Option<bool>),
    Shutdown,
}

/// Commands to whichever radio is attached.
#[derive(Clone)]
pub enum RadioCtl {
    Tci(mpsc::Sender<Command>),
    Loopback(Sender<LoopCmd>),
    Rig(Sender<crate::rig::thread::RigCmd>),
}

impl RadioCtl {
    pub fn ptt(&self, on: bool) {
        match self {
            RadioCtl::Tci(tx) => {
                let _ = tx.send(Command::Trx {
                    trx: 0,
                    on,
                    tci_audio: true,
                });
            }
            RadioCtl::Loopback(tx) => {
                let _ = tx.send(LoopCmd::Ptt(on));
            }
            RadioCtl::Rig(tx) => {
                let _ = tx.send(crate::rig::thread::RigCmd::Ptt(on));
            }
        }
    }

    pub fn drive(&self, pct: u32) {
        match self {
            RadioCtl::Tci(tx) => {
                let _ = tx.send(Command::Drive { trx: 0, pct });
            }
            RadioCtl::Loopback(tx) => {
                let _ = tx.send(LoopCmd::Drive(pct));
            }
            RadioCtl::Rig(tx) => {
                let _ = tx.send(crate::rig::thread::RigCmd::Drive(pct));
            }
        }
    }

    pub fn loopback(&self) -> Option<&Sender<LoopCmd>> {
        match self {
            RadioCtl::Loopback(tx) => Some(tx),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// TCI
// ---------------------------------------------------------------------------

struct TciBridge {
    rx: Producer<f32>,
    tx: Consumer<f32>,
    shared: Arc<Shared>,
    ui: Sender<UiEvent>,
    repaint: Box<dyn Fn() + Send>,
    mono: Vec<f32>,
    commands: mpsc::Sender<Command>,
    warned_rate: bool,
    /// We sent `modulation:0,digu;` and have not seen it echoed yet.
    digu_pending: bool,
    /// TRX 0 RX filter as last reported.
    rx_filter: Option<(i32, i32)>,
    /// TCI audio rate on the wire.
    wire_rate: u32,
    up: Upsampler,
    down: Downsampler,
    wide: Vec<f32>,
}

impl TciBridge {
    fn problem(&self, state: &RadioState) -> Option<String> {
        let mode = state.modulation.first().map(String::as_str).unwrap_or("");
        if state.ready && !self.digu_pending && mode != "digu" {
            return Some(format!("radio in {}; Ion expects DIGU", mode.to_ascii_uppercase()));
        }
        match self.rx_filter {
            // The filter must pass both the waterfall's 300-2700 Hz and
            // Ion's OFDM band.
            Some((lo, hi))
                if lo as f32 > ion::link::audio::ofdm_band_hz().0.min(300.0)
                    || (hi as f32) < ion::link::audio::ofdm_band_hz().1.max(2700.0) =>
            {
                Some(format!("RX filter {lo}..{hi} Hz; set 0-3000 in ESDR3"))
            }
            _ => None,
        }
    }
}

impl tci::Handler for TciBridge {
    fn on_event(&mut self, event: &Event, state: &RadioState) {
        let message = match event {
            Event::Connected => {
                self.digu_pending = false;
                self.rx_filter = None;
                "connected, waiting for ready".to_string()
            }
            Event::Ready => {
                // Switch to DIGU; the RX filter is left to ExpertSDR3.
                let mode = state.modulation.first().cloned().unwrap_or_default();
                if mode != "digu" {
                    let _ = self.commands.send(Command::Modulation {
                        trx: 0,
                        mode: "digu".into(),
                    });
                    self.digu_pending = true;
                }
                "ready".to_string()
            }
            Event::Modulation { trx: 0, mode } => {
                if mode == "digu" {
                    self.digu_pending = false;
                }
                String::new()
            }
            Event::Disconnected(why) => format!("disconnected: {why}"),
            Event::Other(c) if c.name == "rx_filter_band" => {
                if c.args.first().map(String::as_str) != Some("0") {
                    return;
                }
                let lo = c.args.get(1).and_then(|a| a.parse().ok());
                let hi = c.args.get(2).and_then(|a| a.parse().ok());
                if let (Some(lo), Some(hi)) = (lo, hi) {
                    self.rx_filter = Some((lo, hi));
                }
                String::new()
            }
            Event::Other(c) => {
                log::debug!("tci: unhandled {c}");
                return;
            }
            _ => String::new(),
        };
        let info = RadioInfo {
            connected: state.connected,
            device: state.device.clone(),
            dial_hz: state.dial_hz(0),
            ptt: state.trx.first().copied().unwrap_or(false),
            drive: state.drive.first().copied().flatten(),
            message,
            problem: if state.connected { self.problem(state) } else { None },
            drive_unsupported: false,
        };
        let _ = self.ui.try_send(UiEvent::Radio(info));
        (self.repaint)();
    }

    fn on_rx_audio(&mut self, header: &FrameHeader, samples: &[f32], channels: usize) {
        if header.sample_rate != self.wire_rate && !self.warned_rate {
            self.warned_rate = true;
            let _ = self.ui.try_send(UiEvent::Warning(format!(
                "radio sends {} Hz audio; Ion asked for {}",
                header.sample_rate, self.wire_rate
            )));
        }
        self.mono.clear();
        self.mono.extend(samples.iter().step_by(channels.max(1)));
        self.wide.clear();
        self.up.process(&self.mono, &mut self.wide);
        push_slice(&mut self.rx, &self.wide);
    }

    fn on_tx_request(&mut self, _header: &FrameHeader, out: &mut [f32]) {
        if !self.shared.keyed.load(Ordering::Acquire) {
            discard(&mut self.tx);
            out.fill(0.0);
            return;
        }
        // `out` is at the wire rate: take the matching 48 kHz audio.
        let k = (48_000 / self.wire_rate) as usize;
        self.wide.clear();
        self.wide.resize(out.len() * k, 0.0);
        let n = pop_slice(&mut self.tx, &mut self.wide);
        if n < self.wide.len() && !self.shared.tx_done.load(Ordering::Acquire) {
            self.shared.underruns.fetch_add(1, Ordering::Relaxed);
        }
        let wrote = self.down.process(&self.wide, out);
        out[wrote..].fill(0.0);
    }
}

pub struct RadioThread {
    pub ctl: RadioCtl,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
    sound: Option<crate::soundcard::SoundCard>,
}

impl RadioThread {
    pub fn new(ctl: RadioCtl, stop: Arc<AtomicBool>, handle: JoinHandle<()>) -> Self {
        Self {
            ctl,
            stop,
            handle: Some(handle),
            sound: None,
        }
    }

    pub fn with_sound(mut self, sound: crate::soundcard::SoundCard) -> Self {
        self.sound = Some(sound);
        self
    }

    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(tx) = self.ctl.loopback() {
            let _ = tx.send(LoopCmd::Shutdown);
        }
        // The rig thread unkeys before it exits; then the audio stops.
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
        if let Some(mut s) = self.sound.take() {
            s.stop();
        }
    }
}

impl Drop for RadioThread {
    fn drop(&mut self) {
        self.stop();
    }
}

pub fn spawn_tci(
    url: String,
    wire_rate: u32,
    rx: Producer<f32>,
    tx: Consumer<f32>,
    shared: Arc<Shared>,
    ui: Sender<UiEvent>,
    repaint: Box<dyn Fn() + Send>,
) -> RadioThread {
    let (cmd_tx, cmd_rx) = mpsc::channel();
    let stop = Arc::new(AtomicBool::new(false));
    let stop2 = stop.clone();
    let commands = cmd_tx.clone();
    let handle = std::thread::Builder::new()
        .name("tci".into())
        .spawn(move || {
            let cfg = tci::Config {
                url,
                sample_rate: wire_rate,
                ..tci::Config::default()
            };
            let k = (48_000 / wire_rate) as usize;
            log::info!("TCI audio at {wire_rate} Hz (x{k} to 48 kHz inside)");
            let mut bridge = TciBridge {
                rx,
                tx,
                shared,
                ui,
                repaint,
                mono: Vec::with_capacity(8192),
                commands,
                warned_rate: false,
                digu_pending: false,
                rx_filter: None,
                wire_rate,
                up: Upsampler::new(k),
                down: Downsampler::new(k),
                wide: Vec::with_capacity(8192 * k),
            };
            tci::run(&cfg, &mut bridge, &cmd_rx, &stop2);
        })
        .expect("spawn tci thread");
    RadioThread::new(RadioCtl::Tci(cmd_tx), stop, handle)
}

// ---------------------------------------------------------------------------
// Loopback
// ---------------------------------------------------------------------------

/// 10 ms.
const BLOCK: usize = 480;
/// The bandwidth the loopback SNR is measured in, Hz.
const NOISE_BW_HZ: f32 = 2400.0;

pub fn spawn_loopback(
    snr_db: f32,
    mut rx: Producer<f32>,
    mut tx: Consumer<f32>,
    shared: Arc<Shared>,
    ui: Sender<UiEvent>,
    repaint: Box<dyn Fn() + Send>,
) -> RadioThread {
    let (cmd_tx, cmd_rx): (Sender<LoopCmd>, Receiver<LoopCmd>) = crossbeam_channel::unbounded();
    let stop = Arc::new(AtomicBool::new(false));
    let stop2 = stop.clone();
    let handle = std::thread::Builder::new()
        .name("loopback".into())
        .spawn(move || {
            let mut info = RadioInfo {
                connected: true,
                device: Some("Loopback".into()),
                dial_hz: Some(14_080_000),
                ptt: false,
                drive: Some(50),
                message: "loopback: no radio".into(),
                problem: None,
                drive_unsupported: false,
            };
            let publish = |info: &RadioInfo| {
                let _ = ui.try_send(UiEvent::Radio(info.clone()));
                repaint();
            };
            publish(&info);

            // Noise is set against a full-level steady tone.
            let ref_power = TX_LEVEL * TX_LEVEL / 2.0;
            let mut sigma = noise_sigma(ref_power, snr_db, NOISE_BW_HZ, AUDIO_RATE);
            let mut noise = NoiseGen::new(0x5eed);
            // The test peer's bursts, and how far each has played.
            let mut replies: Vec<(Vec<f32>, usize)> = Vec::new();
            let mut peer: Option<crate::ion_engine::PeerLink> = None;
            let mut net: Option<crate::ion_engine::NetPeers> = None;
            let mut net_fifo: std::collections::VecDeque<f32> = std::collections::VecDeque::new();
            let mut block = [0.0f32; BLOCK];
            let mut own = [0.0f32; BLOCK];
            let mut next = Instant::now();

            while !stop2.load(Ordering::Relaxed) {
                while let Ok(cmd) = cmd_rx.try_recv() {
                    match cmd {
                        LoopCmd::Ptt(on) => {
                            info.ptt = on;
                            publish(&info);
                        }
                        LoopCmd::Drive(d) => info.drive = Some(d),
                        LoopCmd::SnrDb(s) => sigma = noise_sigma(ref_power, s, NOISE_BW_HZ, AUDIO_RATE),
                        LoopCmd::IonPeer(on) => {
                            peer = on.then(crate::ion_engine::spawn_peer);
                        }
                        LoopCmd::IonNet(kind) => {
                            net = kind.map(crate::ion_engine::spawn_net_peers);
                            net_fifo.clear();
                        }
                        LoopCmd::Shutdown => return,
                    }
                }

                for x in block.iter_mut() {
                    *x = sigma * noise.gaussian();
                }
                if shared.keyed.load(Ordering::Acquire) {
                    let n = pop_slice(&mut tx, &mut own);
                    if n < BLOCK {
                        own[n..].fill(0.0);
                        if !shared.tx_done.load(Ordering::Acquire) {
                            shared.underruns.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    for (b, o) in block.iter_mut().zip(&own) {
                        *b += o;
                    }
                } else {
                    discard(&mut tx);
                }
                // The Ion peer hears what we hear minus itself: noise
                // plus our transmission. Its replies play from now on.
                if let Some(p) = &peer {
                    let _ = p.to_peer.send(block.to_vec());
                    while let Ok(a) = p.from_peer.try_recv() {
                        replies.push((a, 0));
                    }
                }
                // The simulated net hears this block; what it sends back plays
                // as a continuous stream (one block behind).
                if let Some(nt) = &net {
                    let _ = nt.to_net.send(block.to_vec());
                    while let Ok(b) = nt.from_net.try_recv() {
                        net_fifo.extend(b);
                    }
                    for b in block.iter_mut() {
                        match net_fifo.pop_front() {
                            Some(x) => *b += x,
                            None => break,
                        }
                    }
                }
                for (audio, pos) in replies.iter_mut() {
                    let n = BLOCK.min(audio.len() - *pos);
                    for (b, a) in block.iter_mut().zip(&audio[*pos..*pos + n]) {
                        *b += a;
                    }
                    *pos += n;
                }
                replies.retain(|(audio, pos)| *pos < audio.len());
                push_slice(&mut rx, &block);

                next += Duration::from_millis(10);
                let now = Instant::now();
                if next > now {
                    std::thread::sleep(next - now);
                } else if now - next > Duration::from_millis(200) {
                    next = now; // fell far behind (debugger, sleep); resync
                }
            }
        })
        .expect("spawn loopback thread");
    RadioThread::new(RadioCtl::Loopback(cmd_tx), stop, handle)
}
