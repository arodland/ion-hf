//! Sound-card audio for radios without TCI: capture into the engine's RX
//! ring, and play the engine's TX ring while keyed.
//!
//! The cpal callbacks only copy to and from small rings at the device rate.
//! A thread of our own owns the streams, resamples, and reopens the streams
//! when a device goes away (USB unplugged, radio switched off).

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use crossbeam_channel::Sender;
use rtrb::{Consumer, Producer, RingBuffer};

use crate::engine::{Shared, UiEvent, discard, pop_slice, push_slice};
use crate::resample::FracResampler;

const RATE: u32 = 48_000;
/// TX audio kept queued at the device: short, so PTT-off follows the
/// engine's end of over closely.
const PLAY_AHEAD_MS: u32 = 30;
const TICK: Duration = Duration::from_millis(5);
const RETRY: Duration = Duration::from_secs(2);
/// An input that delivers nothing for this long is treated as lost.
const STALL: Duration = Duration::from_secs(3);

/// Shared by the sound-card thread and the radio thread.
pub struct AudioLevels {
    /// TX audio gain, 0 to 1.
    pub tx_gain: f32,
    /// TX audio queued at the device, in microseconds.
    pub backlog_us: AtomicU64,
    /// The radio thread has sent PTT-on (or there is nothing to send, VOX):
    /// TX audio may flow. Until then the engine's TX audio waits in its
    /// ring, so a slow CAT round trip cannot cost the start of an over.
    pub ptt_ready: AtomicBool,
}

impl AudioLevels {
    pub fn new(tx_gain: f32) -> Self {
        Self {
            tx_gain: tx_gain.clamp(0.0, 1.0),
            backlog_us: AtomicU64::new(0),
            ptt_ready: AtomicBool::new(false),
        }
    }

    pub fn backlog(&self) -> Duration {
        Duration::from_micros(self.backlog_us.load(Ordering::Relaxed))
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct AudioConfig {
    /// Device names; empty means the system default.
    pub input: String,
    pub output: String,
    /// Silence played after PTT-on before the TX audio, so a relay or a
    /// CAT round trip does not eat the start of the over.
    pub ptt_lead_ms: u32,
}

/// Moves audio between the device-rate rings and the engine's 48 kHz rings.
/// No cpal in here, so it is tested directly.
pub struct Pump {
    pub rx: Producer<f32>,
    pub tx: Consumer<f32>,
    shared: Arc<Shared>,
    levels: Arc<AudioLevels>,
    up: FracResampler,
    down: FracResampler,
    out_rate: u32,
    lead: usize,
    lead_left: usize,
    was_keyed: bool,
    narrow: Vec<f32>,
    wide: Vec<f32>,
}

impl Pump {
    pub fn new(
        rx: Producer<f32>,
        tx: Consumer<f32>,
        shared: Arc<Shared>,
        levels: Arc<AudioLevels>,
        ptt_lead_ms: u32,
    ) -> Self {
        let mut p = Self {
            rx,
            tx,
            shared,
            levels,
            up: FracResampler::new(RATE, RATE),
            down: FracResampler::new(RATE, RATE),
            out_rate: RATE,
            lead: 0,
            lead_left: 0,
            was_keyed: false,
            narrow: Vec::with_capacity(8192),
            wide: Vec::with_capacity(8192),
        };
        p.set_rates(RATE, RATE, ptt_lead_ms);
        p
    }

    pub fn set_rates(&mut self, in_rate: u32, out_rate: u32, ptt_lead_ms: u32) {
        self.up = FracResampler::new(in_rate, RATE);
        self.down = FracResampler::new(RATE, out_rate);
        self.out_rate = out_rate;
        self.lead = (out_rate as u64 * ptt_lead_ms as u64 / 1000) as usize;
    }

    /// One pass: everything captured goes to the engine; the device output
    /// is topped up to [`PLAY_AHEAD_MS`] while keyed.
    pub fn run(&mut self, cap: &mut Consumer<f32>, play: &mut Producer<f32>) {
        self.narrow.clear();
        self.narrow.resize(cap.slots(), 0.0);
        let n = pop_slice(cap, &mut self.narrow);
        self.narrow.truncate(n);
        self.wide.clear();
        self.up.process(&self.narrow, &mut self.wide);
        push_slice(&mut self.rx, &self.wide);

        // TX: the engine is keyed and the radio has been told.
        let engine_keyed = self.shared.keyed.load(Ordering::Acquire);
        let keyed = engine_keyed && self.levels.ptt_ready.load(Ordering::Acquire);
        if keyed && !self.was_keyed {
            self.lead_left = self.lead;
        }
        self.was_keyed = keyed;
        let queued = play.buffer().capacity() - play.slots();
        if keyed {
            let target = (self.out_rate * PLAY_AHEAD_MS / 1000) as usize;
            let mut need = target.saturating_sub(queued);
            let silence = need.min(self.lead_left);
            if silence > 0 {
                self.narrow.clear();
                self.narrow.resize(silence, 0.0);
                push_slice(play, &self.narrow);
                self.lead_left -= silence;
                need -= silence;
            }
            if need > 0 {
                let want = (need as u64 * RATE as u64).div_ceil(self.out_rate as u64) as usize;
                self.wide.clear();
                self.wide.resize(want, 0.0);
                let got = pop_slice(&mut self.tx, &mut self.wide);
                if got < want && !self.shared.tx_done.load(Ordering::Acquire) {
                    self.shared.underruns.fetch_add(1, Ordering::Relaxed);
                }
                self.narrow.clear();
                self.down.process(&self.wide[..got], &mut self.narrow);
                let g = self.levels.tx_gain;
                for x in self.narrow.iter_mut() {
                    *x *= g;
                }
                push_slice(play, &self.narrow);
            }
        } else if !engine_keyed {
            // What is already at the device plays out (the end of the
            // over); nothing new goes to it.
            discard(&mut self.tx);
        }
        // Keyed but PTT not sent yet: the engine's audio waits.
        let queued = play.buffer().capacity() - play.slots();
        self.levels
            .backlog_us
            .store(queued as u64 * 1_000_000 / self.out_rate as u64, Ordering::Relaxed);
    }
}

/// Input and output device names, for the Settings menus.
pub fn list_devices() -> (Vec<String>, Vec<String>) {
    fn names<I: Iterator<Item = cpal::Device>>(it: Result<I, cpal::DevicesError>) -> Vec<String> {
        it.map(|ds| ds.filter_map(|d| d.name().ok()).collect())
            .unwrap_or_default()
    }
    let host = cpal::default_host();
    (names(host.input_devices()), names(host.output_devices()))
}

/// Running sound-card audio. Dropping it stops the streams.
pub struct SoundCard {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl SoundCard {
    pub fn start(
        cfg: AudioConfig,
        levels: Arc<AudioLevels>,
        rx: Producer<f32>,
        tx: Consumer<f32>,
        shared: Arc<Shared>,
        ui: Sender<UiEvent>,
    ) -> SoundCard {
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
        let thread = std::thread::Builder::new()
            .name("soundcard".into())
            .spawn(move || run(cfg, levels, rx, tx, shared, ui, stop2))
            .expect("spawn soundcard thread");
        SoundCard {
            stop,
            thread: Some(thread),
        }
    }

    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for SoundCard {
    fn drop(&mut self) {
        self.stop();
    }
}

fn run(
    cfg: AudioConfig,
    levels: Arc<AudioLevels>,
    rx: Producer<f32>,
    tx: Consumer<f32>,
    shared: Arc<Shared>,
    ui: Sender<UiEvent>,
    stop: Arc<AtomicBool>,
) {
    let mut pump = Pump::new(rx, tx, shared, levels.clone(), cfg.ptt_lead_ms);
    let mut streams: Option<Streams> = None;
    let mut retry_at = Instant::now();
    let mut last_error = String::new();
    while !stop.load(Ordering::Relaxed) {
        if streams.is_none() && Instant::now() >= retry_at {
            match Streams::open(&cfg) {
                Ok(s) => {
                    pump.set_rates(s.in_rate, s.out_rate, cfg.ptt_lead_ms);
                    let d = format!(
                        "in: {} @ {:.1} kHz; out: {} @ {:.1} kHz",
                        s.in_name,
                        s.in_rate as f32 / 1e3,
                        s.out_name,
                        s.out_rate as f32 / 1e3
                    );
                    log::info!("sound card {d}");
                    if !last_error.is_empty() {
                        let _ = ui.try_send(UiEvent::Warning(format!("audio back: {d}")));
                    }
                    last_error.clear();
                    streams = Some(s);
                }
                Err(e) => {
                    if e != last_error {
                        log::warn!("sound card: {e}");
                        let _ = ui.try_send(UiEvent::Warning(format!("audio: {e}")));
                        last_error = e;
                    }
                    retry_at = Instant::now() + RETRY;
                }
            }
        }
        match streams.as_mut() {
            Some(s) => {
                let before = s.cap.slots();
                pump.run(&mut s.cap, &mut s.play);
                if before > 0 {
                    s.last_input = Instant::now();
                }
                let why = if s.lost.load(Ordering::Relaxed) {
                    Some("audio device lost")
                } else if s.last_input.elapsed() > STALL {
                    Some("audio input stopped")
                } else {
                    None
                };
                if let Some(why) = why {
                    log::warn!("sound card: {why}");
                    let _ = ui.try_send(UiEvent::Warning(format!("{why}; reopening")));
                    last_error = why.to_string();
                    streams = None;
                    retry_at = Instant::now() + RETRY;
                }
            }
            None => {
                // No device: TX audio has nowhere to go.
                discard(&mut pump.tx);
                levels.backlog_us.store(0, Ordering::Relaxed);
            }
        }
        std::thread::sleep(TICK);
    }
}

/// Both streams, and the rings their callbacks use.
struct Streams {
    _input: cpal::Stream,
    _output: cpal::Stream,
    cap: Consumer<f32>,
    play: Producer<f32>,
    in_rate: u32,
    out_rate: u32,
    in_name: String,
    out_name: String,
    lost: Arc<AtomicBool>,
    last_input: Instant,
}

fn find<I: Iterator<Item = cpal::Device>>(devices: Result<I, cpal::DevicesError>, name: &str) -> Option<cpal::Device> {
    devices.ok()?.find(|d| d.name().map(|n| n == name).unwrap_or(false))
}

/// A config at 48 kHz if the device offers one (no resampling), else its
/// default.
fn pick_config(
    ranges: Vec<cpal::SupportedStreamConfigRange>,
    default: Result<cpal::SupportedStreamConfig, cpal::DefaultStreamConfigError>,
) -> Result<cpal::SupportedStreamConfig, String> {
    let usable = |f: cpal::SampleFormat| {
        matches!(
            f,
            cpal::SampleFormat::F32 | cpal::SampleFormat::I16 | cpal::SampleFormat::I32 | cpal::SampleFormat::U16
        )
    };
    let want = cpal::SampleRate(RATE);
    let mut best: Option<cpal::SupportedStreamConfigRange> = None;
    for r in ranges {
        if usable(r.sample_format()) && r.min_sample_rate() <= want && r.max_sample_rate() >= want {
            let better = match &best {
                None => true,
                Some(b) => {
                    // Prefer f32, then fewer channels.
                    (r.sample_format() == cpal::SampleFormat::F32) as u8 * 2 + (r.channels() < b.channels()) as u8
                        > (b.sample_format() == cpal::SampleFormat::F32) as u8 * 2
                }
            };
            if better {
                best = Some(r);
            }
        }
    }
    match best {
        Some(r) => Ok(r.with_sample_rate(want)),
        None => default.map_err(|e| e.to_string()),
    }
}

impl Streams {
    fn open(cfg: &AudioConfig) -> Result<Streams, String> {
        let host = cpal::default_host();
        let input = if cfg.input.is_empty() {
            host.default_input_device().ok_or("no audio input device")?
        } else {
            find(host.input_devices(), &cfg.input).ok_or_else(|| format!("input device \"{}\" not found", cfg.input))?
        };
        let output = if cfg.output.is_empty() {
            host.default_output_device().ok_or("no audio output device")?
        } else {
            find(host.output_devices(), &cfg.output)
                .ok_or_else(|| format!("output device \"{}\" not found", cfg.output))?
        };
        let in_name = input.name().unwrap_or_else(|_| "input".into());
        let out_name = output.name().unwrap_or_else(|_| "output".into());
        let in_cfg = pick_config(
            input.supported_input_configs().map(|c| c.collect()).unwrap_or_default(),
            input.default_input_config(),
        )
        .map_err(|e| format!("{in_name}: {e}"))?;
        let out_cfg = pick_config(
            output
                .supported_output_configs()
                .map(|c| c.collect())
                .unwrap_or_default(),
            output.default_output_config(),
        )
        .map_err(|e| format!("{out_name}: {e}"))?;
        let in_rate = in_cfg.sample_rate().0;
        let out_rate = out_cfg.sample_rate().0;
        let (cap_tx, cap) = RingBuffer::<f32>::new(in_rate as usize / 2);
        let (play, play_rx) = RingBuffer::<f32>::new(out_rate as usize / 5);
        let lost = Arc::new(AtomicBool::new(false));

        let in_format = in_cfg.sample_format();
        let in_stream = match in_format {
            cpal::SampleFormat::F32 => build_input::<f32>(&input, in_cfg.into(), cap_tx, lost.clone()),
            cpal::SampleFormat::I16 => build_input::<i16>(&input, in_cfg.into(), cap_tx, lost.clone()),
            cpal::SampleFormat::I32 => build_input::<i32>(&input, in_cfg.into(), cap_tx, lost.clone()),
            cpal::SampleFormat::U16 => build_input::<u16>(&input, in_cfg.into(), cap_tx, lost.clone()),
            f => Err(format!("unsupported input format {f:?}")),
        }
        .map_err(|e| format!("{in_name}: {e}"))?;
        let out_format = out_cfg.sample_format();
        let out_stream = match out_format {
            cpal::SampleFormat::F32 => build_output::<f32>(&output, out_cfg.into(), play_rx, lost.clone()),
            cpal::SampleFormat::I16 => build_output::<i16>(&output, out_cfg.into(), play_rx, lost.clone()),
            cpal::SampleFormat::I32 => build_output::<i32>(&output, out_cfg.into(), play_rx, lost.clone()),
            cpal::SampleFormat::U16 => build_output::<u16>(&output, out_cfg.into(), play_rx, lost.clone()),
            f => Err(format!("unsupported output format {f:?}")),
        }
        .map_err(|e| format!("{out_name}: {e}"))?;
        in_stream.play().map_err(|e| format!("{in_name}: {e}"))?;
        out_stream.play().map_err(|e| format!("{out_name}: {e}"))?;
        Ok(Streams {
            _input: in_stream,
            _output: out_stream,
            cap,
            play,
            in_rate,
            out_rate,
            in_name,
            out_name,
            lost,
            last_input: Instant::now(),
        })
    }
}

/// Captures the first channel (the radio's main receiver on a stereo codec).
fn build_input<T>(
    device: &cpal::Device,
    config: cpal::StreamConfig,
    mut cap: Producer<f32>,
    lost: Arc<AtomicBool>,
) -> Result<cpal::Stream, String>
where
    T: cpal::SizedSample,
    f32: cpal::FromSample<T>,
{
    let channels = config.channels.max(1) as usize;
    let lost2 = lost.clone();
    device
        .build_input_stream(
            &config,
            move |data: &[T], _| {
                for frame in data.chunks(channels) {
                    // Full ring: the thread fell behind; drop.
                    let _ = cap.push(<f32 as cpal::FromSample<T>>::from_sample_(frame[0]));
                }
            },
            move |e| {
                log::warn!("audio input: {e}");
                lost2.store(true, Ordering::Relaxed);
            },
            None,
        )
        .map_err(|e| e.to_string())
}

/// Plays mono TX audio on every channel; silence when there is none.
fn build_output<T>(
    device: &cpal::Device,
    config: cpal::StreamConfig,
    mut play: Consumer<f32>,
    lost: Arc<AtomicBool>,
) -> Result<cpal::Stream, String>
where
    T: cpal::SizedSample + cpal::FromSample<f32>,
{
    let channels = config.channels.max(1) as usize;
    device
        .build_output_stream(
            &config,
            move |data: &mut [T], _| {
                for frame in data.chunks_mut(channels) {
                    let x = play.pop().unwrap_or(0.0).clamp(-1.0, 1.0);
                    frame.fill(<T as cpal::FromSample<f32>>::from_sample_(x));
                }
            },
            move |e| {
                log::warn!("audio output: {e}");
                lost.store(true, Ordering::Relaxed);
            },
            None,
        )
        .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Bench {
        pump: Pump,
        shared: Arc<Shared>,
        levels: Arc<AudioLevels>,
        cap_in: Producer<f32>,
        cap: Consumer<f32>,
        play: Producer<f32>,
        play_out: Consumer<f32>,
        eng_rx: Consumer<f32>,
        eng_tx: Producer<f32>,
    }

    fn bench(in_rate: u32, out_rate: u32, lead_ms: u32) -> Bench {
        let shared = Arc::new(Shared::default());
        let levels = Arc::new(AudioLevels::new(0.5));
        let (rx, eng_rx) = RingBuffer::new(96_000);
        let (eng_tx, tx) = RingBuffer::new(48_000);
        let (cap_in, cap) = RingBuffer::new(in_rate as usize);
        let (play, play_out) = RingBuffer::new(out_rate as usize / 5);
        levels.ptt_ready.store(true, Ordering::Release);
        let mut pump = Pump::new(rx, tx, shared.clone(), levels.clone(), lead_ms);
        pump.set_rates(in_rate, out_rate, lead_ms);
        Bench {
            pump,
            shared,
            levels,
            cap_in,
            cap,
            play,
            play_out,
            eng_rx,
            eng_tx,
        }
    }

    impl Bench {
        fn run(&mut self) {
            self.pump.run(&mut self.cap, &mut self.play);
        }
        /// The device plays `n` samples.
        fn device_plays(&mut self, n: usize) -> Vec<f32> {
            let mut v = vec![0.0; n];
            let got = pop_slice(&mut self.play_out, &mut v);
            v.truncate(got);
            v
        }
    }

    #[test]
    fn rx_at_44k1_reaches_the_engine_at_48k() {
        let mut b = bench(44_100, 48_000, 0);
        let tone: Vec<f32> = (0..44_100)
            .map(|i| (std::f32::consts::TAU * 1000.0 * i as f32 / 44_100.0).sin() * 0.25)
            .collect();
        for chunk in tone.chunks(441) {
            push_slice(&mut b.cap_in, chunk);
            b.run();
        }
        let n = b.eng_rx.slots();
        assert!((n as i64 - 48_000).abs() <= 2, "{n}");
    }

    #[test]
    fn tx_flows_only_while_keyed_with_gain_and_lead_silence() {
        let mut b = bench(48_000, 44_100, 20);
        push_slice(&mut b.eng_tx, &vec![0.8; 4800]);
        b.run();
        // Not keyed: the engine's TX audio is discarded, nothing plays.
        assert_eq!(b.eng_tx.slots(), 48_000);
        assert!(b.device_plays(10_000).is_empty());

        b.shared.keyed.store(true, Ordering::Release);
        push_slice(&mut b.eng_tx, &vec![0.8; 40_000]);
        let mut played = Vec::new();
        for _ in 0..100 {
            b.run();
            played.extend(b.device_plays(220)); // 5 ms at 44.1 kHz
        }
        // 20 ms of lead silence at 44.1 kHz, then the audio at half gain.
        let first = played.iter().position(|&x| x.abs() > 1e-3).unwrap();
        assert!((first as i64 - 882).abs() <= 4, "lead {first}");
        let tail = &played[first + 100..first + 2000];
        assert!(tail.iter().all(|&x| (x - 0.4).abs() < 0.01), "{:?}", &tail[..4]);
        // The device queue stays short: PTT-off follows the engine.
        assert!(b.levels.backlog() <= Duration::from_millis(PLAY_AHEAD_MS as u64 + 1));
        assert_eq!(b.shared.underruns.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn running_dry_mid_over_counts_an_underrun_but_not_at_the_end() {
        let mut b = bench(48_000, 48_000, 0);
        b.shared.keyed.store(true, Ordering::Release);
        b.run();
        assert_eq!(b.shared.underruns.load(Ordering::Relaxed), 1);
        b.device_plays(10_000);
        b.shared.tx_done.store(true, Ordering::Release);
        b.run();
        assert_eq!(b.shared.underruns.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn tx_waits_for_the_ptt_command() {
        let mut b = bench(48_000, 48_000, 10);
        b.levels.ptt_ready.store(false, Ordering::Release);
        b.shared.keyed.store(true, Ordering::Release);
        push_slice(&mut b.eng_tx, &vec![0.5; 9600]);
        for _ in 0..5 {
            b.run();
        }
        // Nothing played, nothing taken from the engine, no underrun.
        assert!(b.device_plays(10_000).is_empty());
        assert_eq!(b.eng_tx.slots(), 48_000 - 9600);
        assert_eq!(b.shared.underruns.load(Ordering::Relaxed), 0);
        // PTT sent: the lead silence, then the audio.
        b.levels.ptt_ready.store(true, Ordering::Release);
        b.run();
        let played = b.device_plays(10_000);
        let first = played.iter().position(|&x| x.abs() > 1e-3).unwrap();
        assert_eq!(first, 480);
        assert!((played[first + 10] - 0.25).abs() < 1e-3);
    }

    /// Needs a sound card; plays silence.
    #[test]
    #[ignore]
    fn opens_the_default_devices() {
        let (i, o) = list_devices();
        println!("inputs {i:?}\noutputs {o:?}");
        let s = Streams::open(&AudioConfig::default()).expect("open");
        println!("{} @ {} / {} @ {}", s.in_name, s.in_rate, s.out_name, s.out_rate);
    }
}
