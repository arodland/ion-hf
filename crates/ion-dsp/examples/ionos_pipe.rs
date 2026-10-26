//! IONOS SIM as a two-way pipe, for running another modem's stations
//! through the same channels as Ion (`tools/mercury-bench`).
//!
//! cargo run -p ion-dsp --release --example ionos_pipe -- --mode MPP [--paths 2]
//!   [--snr 10 --peak-a 0.5 --peak-b 0.5] [--seed 1] [--block 160]
//!
//! Lockstep on stdin/stdout, 8 kHz mono i16 little-endian: each frame in is
//! `pttA u8, pttB u8, block × A's TX, block × B's TX`; each frame out is
//! `block × what A hears, block × what B hears` (B → A and A → B through
//! their own boxes), a fixed 40 ms behind and 6 dB down. Noise is on all the
//! time, at `--snr` dB in 3 kHz against a sine at the far station's peak
//! (`--peak-a` / `--peak-b`, full scale 1); without `--snr`, none. On EOF,
//! stderr gets each station's bursts (start, length, peak and mean power at
//! the box's input) and the noise heard.

use std::collections::VecDeque;
use std::io::{self, Read, Write};

use ion_dsp::ionos::{Ionos, Mode, SIM_RATE, StreamResampler};

const RATE: u32 = 8_000;
/// Output delay: covers both resamplers' half filters with room to spare.
const DELAY: usize = 320;
/// What a station hears goes out 6 dB down: a fade's peaks would clip i16
/// at the input level, and signal and noise drop alike.
const OUT_GAIN: f32 = 0.5;

struct Dir {
    up: StreamResampler,
    box_: Ionos,
    down: StreamResampler,
    out: VecDeque<f32>,
    noise: Option<(f32, f32)>,
}

impl Dir {
    fn new(mode: Mode, paths: usize, seed: u64, noise: Option<(f32, f32)>) -> Self {
        Self {
            up: StreamResampler::new(RATE, SIM_RATE as u32),
            box_: Ionos::new(mode, paths, 3000, seed),
            down: StreamResampler::new(SIM_RATE as u32, RATE),
            out: std::iter::repeat_n(0.0, DELAY).collect(),
            noise,
        }
    }

    /// One block through; returns its input at 44.1 kHz too (for levels).
    fn run(&mut self, x: &[f32], y: &mut [f32]) -> Vec<f32> {
        let sim_in = self.up.push(x);
        let sim_out = self.box_.process_sim(&sim_in, self.noise);
        self.out.extend(self.down.push(&sim_out));
        for v in y.iter_mut() {
            *v = OUT_GAIN * self.out.pop_front().expect("pipe ran dry");
        }
        sim_in
    }
}

#[derive(Default)]
struct Burst {
    start: u64,
    blocks: u64,
    peak: f32,
    energy: f64,
    samples: u64,
}

fn arg<T: std::str::FromStr>(args: &[String], k: &str) -> Option<T> {
    args.iter()
        .position(|a| a == k)
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mode_name: String = arg(&args, "--mode").unwrap_or_else(|| "WGN".into());
    let mode = *Mode::ALL
        .iter()
        .find(|m| m.name().eq_ignore_ascii_case(&mode_name))
        .expect("--mode");
    let paths: usize = arg(&args, "--paths").unwrap_or(2);
    let seed: u64 = arg(&args, "--seed").unwrap_or(1);
    let block: usize = arg(&args, "--block").unwrap_or(160);
    let snr: Option<f32> = arg(&args, "--snr");
    let peak_a: f32 = arg(&args, "--peak-a").unwrap_or(0.5);
    let peak_b: f32 = arg(&args, "--peak-b").unwrap_or(0.5);
    // Sine at the transmitting station's peak: P²/2.
    let noise = |peak: f32| snr.map(|s| (peak * peak / 2.0, s));
    // [0]: A → B, [1]: B → A.
    let mut dirs = [
        Dir::new(mode, paths, seed, noise(peak_a)),
        Dir::new(mode, paths, seed ^ 0x5555, noise(peak_b)),
    ];

    let mut stdin = io::stdin().lock();
    let mut stdout = io::BufWriter::new(io::stdout().lock());
    let mut frame = vec![0u8; 2 + 4 * block];
    let mut out = vec![0u8; 4 * block];
    let mut x = vec![0.0f32; block];
    let mut y = vec![0.0f32; block];
    let mut bursts: [Vec<Burst>; 2] = [Vec::new(), Vec::new()];
    let mut keyed = [false; 2];
    // Noise heard while neither station had been keyed for a while.
    let mut quiet = [(0.0f64, 0u64); 2];
    let mut since_key = [0u64; 2];
    let mut clipped = 0u64;
    let mut n_block = 0u64;
    while stdin.read_exact(&mut frame).is_ok() {
        for s in 0..2 {
            let ptt = frame[s] == 1;
            let raw = &frame[2 + 2 * block * s..2 + 2 * block * (s + 1)];
            for (v, b) in x.iter_mut().zip(raw.as_chunks::<2>().0) {
                *v = i16::from_le_bytes(*b) as f32 / 32_768.0;
            }
            let sim_in = dirs[s].run(&x, &mut y);
            if ptt {
                if !keyed[s] {
                    bursts[s].push(Burst {
                        start: n_block,
                        ..Default::default()
                    });
                }
                let b = bursts[s].last_mut().unwrap();
                b.blocks += 1;
                for &v in &sim_in {
                    b.peak = b.peak.max(v.abs());
                    b.energy += (v * v) as f64;
                }
                b.samples += sim_in.len() as u64;
                since_key[s] = 0;
            } else {
                since_key[s] += 1;
            }
            keyed[s] = ptt;
            // Station s's signal lands at the other one.
            let rx = 1 - s;
            if since_key[s] > 20 && since_key[rx] > 20 {
                quiet[rx].0 += y.iter().map(|v| (v * v) as f64).sum::<f64>();
                quiet[rx].1 += block as u64;
            }
            let o = &mut out[2 * block * rx..2 * block * (rx + 1)];
            for (b, &v) in o.as_chunks_mut::<2>().0.iter_mut().zip(&y) {
                let q = (v * 32_768.0).round();
                if q.abs() > 32_767.0 {
                    clipped += 1;
                }
                *b = (q.clamp(-32_768.0, 32_767.0) as i16).to_le_bytes();
            }
        }
        if stdout.write_all(&out).and_then(|_| stdout.flush()).is_err() {
            break;
        }
        n_block += 1;
    }
    let ms = |b: u64| b as f64 * block as f64 * 1000.0 / RATE as f64;
    for (s, name) in ["A", "B"].iter().enumerate() {
        for b in &bursts[s] {
            let mean = b.energy / b.samples.max(1) as f64;
            eprintln!(
                "BURST {name} at {:.2} s, {:.0} ms, peak {:.2} dBFS, mean {:.2} dBFS",
                ms(b.start) / 1000.0,
                ms(b.blocks),
                20.0 * (b.peak.max(1e-9) as f64).log10(),
                10.0 * mean.max(1e-18).log10()
            );
        }
    }
    for (s, name) in ["A", "B"].iter().enumerate() {
        let (e, n) = quiet[s];
        eprintln!(
            "NOISE {name} hears {:.2} dBFS",
            10.0 * (e / n.max(1) as f64).max(1e-18).log10()
        );
    }
    eprintln!("BLOCKS {n_block} ({:.1} s), clipped {clipped}", ms(n_block) / 1000.0);
}
