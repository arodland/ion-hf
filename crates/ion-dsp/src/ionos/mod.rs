//! IONOS SIM, the Teensy HF channel simulator ARSFI used to compare Winlink
//! modems (QEX March/April 2022), ported from its firmware
//! (`ARSFI/HFSimulator`, `HFSim_BFD_2_05_Proto.ino`, rev 2.05).
//!
//! The firmware's 44.1 kHz rate, filter tables, path model, delays, gain
//! generator and update timing are kept. Defined here instead is the S:N:
//! the power of a sine at the input's peak (or the input's mean power) over
//! the noise in the filter's bandwidth, with the output scaled so WGN gives
//! back the input level. Not ported: flat fading, frequency offset, FM and
//! the busy detector.
//!
//! Licence of the ported code and tables:
//!
//! > IONOS SIM HF/VHF Channel Simulator, Rick Muething, KN6KB.
//! > Copyright (C) 2020 Amateur Radio Safety Foundation Inc. (ARSFI).
//! > Licensed under the MIT Open Source Initiative.
//! >
//! > Permission is hereby granted, free of charge, to any person obtaining a
//! > copy of this software and associated documentation files (the
//! > "Software"), to deal in the Software without restriction, including
//! > without limitation the rights to use, copy, modify, merge, publish,
//! > distribute, sublicense, and/or sell copies of the Software, and to
//! > permit persons to whom the Software is furnished to do so, subject to
//! > the following conditions: The above copyright notice and this
//! > permission notice shall be included in all copies or substantial
//! > portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT
//! > WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
//! > THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//! > NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//! > LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//! > OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//! > WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#[allow(clippy::excessive_precision)] // the firmware's tables, kept verbatim
pub mod coeffs;

use std::f64::consts::PI;

use crate::dsp::AUDIO_RATE;

/// The firmware's audio rate (the filters are designed for it).
pub const SIM_RATE: f64 = 44_100.0;
/// The Teensy 4's exact audio rate, which its delay object rounds with.
const TEENSY_RATE: f64 = 44_117.647_06;
/// Teensy audio block: mixer gains change only between blocks.
const BLOCK: usize = 128;
/// I-path delay that lines it up with the Hilbert filter's group delay.
const HILBERT_COMP_MS: f64 = 1.337_868_480_7;
/// WGN gain on the I path (`ApplyChannelMode`).
const WGN_GAIN: f32 = 0.8498;
/// `mixPathSel` gain on the direct path.
const PATH_SELECT: f32 = 0.5;
/// The Hilbert taps are loaded as Q15 after scaling by this.
const HILBERT_SCALE: f64 = 33_702.0;

/// The firmware's channel types.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Mode {
    /// White Gaussian noise only.
    Wgn,
    /// Multipath good: 0.1 Hz spread, 0.5 ms delay.
    Mpg,
    /// Multipath moderate: 0.5 Hz, 1 ms.
    Mpm,
    /// Multipath poor: 1 Hz, 2 ms.
    Mpp,
    /// Multipath disturbed: 2 Hz, 4 ms.
    Mpd,
}

impl Mode {
    pub const ALL: [Mode; 5] = [Mode::Wgn, Mode::Mpg, Mode::Mpm, Mode::Mpp, Mode::Mpd];

    pub fn name(self) -> &'static str {
        match self {
            Mode::Wgn => "WGN",
            Mode::Mpg => "MPG",
            Mode::Mpm => "MPM",
            Mode::Mpp => "MPP",
            Mode::Mpd => "MPD",
        }
    }

    /// Gain update period, µs (the main loop's constants).
    fn update_us(self) -> Option<f64> {
        match self {
            Mode::Wgn => None,
            Mode::Mpg => Some(156_250.0),
            Mode::Mpm => Some(31_250.0),
            Mode::Mpp => Some(15_625.0),
            Mode::Mpd => Some(7_812.0),
        }
    }

    /// Extra delay of each path, ms (`SetIQTapDelays`).
    fn delays_ms(self, paths: usize) -> Vec<f64> {
        let d = match self {
            Mode::Wgn => return vec![0.0],
            Mode::Mpg => 0.5,
            Mode::Mpm => 1.0,
            Mode::Mpp => 2.0,
            Mode::Mpd => 4.0,
        };
        if paths == 4 {
            // Equal steps of half the delay, MPG in quarter steps.
            let step = if self == Mode::Mpg { 0.25 } else { d / 2.0 };
            (0..4).map(|k| k as f64 * step).collect()
        } else {
            vec![0.0, d]
        }
    }
}

/// S:N reference: what "signal" means.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reference {
    /// A sine with the input's peak: how the box is levelled and calibrated.
    Peak,
    /// The input's mean power.
    Mean,
}

/// Teensy `AudioEffectDelay::delay(ms)` in samples.
fn delay_samples(ms: f64) -> usize {
    (ms * TEENSY_RATE / 1000.0 + 0.5) as usize
}

/// A FIR filter with its history (newest sample first, as `fir_float`),
/// kept twice over so each output is one contiguous dot product.
#[derive(Clone, Debug)]
struct Fir {
    h: Vec<f32>,
    x: Vec<f32>,
    pos: usize,
}

impl Fir {
    fn new(h: Vec<f32>) -> Self {
        let n = h.len();
        Self {
            h,
            x: vec![0.0; 2 * n],
            pos: 0,
        }
    }

    fn q15(h: &[i16]) -> Self {
        Self::new(h.iter().map(|&c| c as f32 / 32_768.0).collect())
    }

    fn push(&mut self, v: f32) -> f32 {
        let n = self.h.len();
        self.pos = (self.pos + n - 1) % n;
        self.x[self.pos] = v;
        self.x[self.pos + n] = v;
        self.h
            .iter()
            .zip(&self.x[self.pos..self.pos + n])
            .map(|(c, x)| c * x)
            .sum()
    }
}

/// Arduino-style `random(lo, hi)` stand-in: xorshift64*.
#[derive(Clone, Debug)]
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Integer in `lo..hi`.
    fn range(&mut self, lo: i64, hi: i64) -> i64 {
        lo + (self.next() % (hi - lo) as u64) as i64
    }

    /// Uniform in [-1, 1).
    fn uniform(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 23) as f32 - 1.0
    }
}

/// One path's gain generator (`QuadGauss12FIR128` for one path).
#[derive(Clone, Debug)]
struct PathGain {
    fir_i: Fir,
    fir_q: Fir,
}

impl PathGain {
    fn new() -> Self {
        let h: Vec<f32> = coeffs::GAUSS.to_vec();
        Self {
            fir_i: Fir::new(h.clone()),
            fir_q: Fir::new(h),
        }
    }

    /// A new (gI, gQ): one polar Box–Muller pair through the Gaussian FIR.
    fn step(&mut self, rng: &mut Rng, std: f32) -> (f32, f32) {
        let (mut i, mut q, mut s);
        loop {
            i = 0.000_01 * rng.range(-100_000, 100_001) as f32;
            q = 0.000_01 * rng.range(-100_000, 100_001) as f32;
            s = i * i + q * q;
            if (0.1..1.0).contains(&s) {
                break;
            }
        }
        let mag = s.sqrt();
        let root = (-2.0 * s.ln()).sqrt();
        let (i, q) = (i * root * std / mag, q * root * std / mag);
        (self.fir_i.push(i), self.fir_q.push(q))
    }
}

/// One direction through IONOS SIM: 48 kHz audio in, faded and noised
/// 48 kHz audio out.
#[derive(Clone, Debug)]
pub struct Ionos {
    mode: Mode,
    bandwidth: f32,
    lp_in: Fir,
    lp_noise: Fir,
    hilbert: Fir,
    /// Delay lines of the I (direct) and Q (Hilbert) signals.
    line_i: Vec<f32>,
    line_q: Vec<f32>,
    pos: usize,
    taps_i: Vec<usize>,
    taps_q: Vec<usize>,
    gens: Vec<PathGain>,
    /// Gains in use, and the newest generated (applied at a block start).
    gains: Vec<(f32, f32)>,
    pending: Vec<(f32, f32)>,
    std: f32,
    rng: Rng,
    noise: Rng,
    /// Samples processed, and when the next gain update is due (µs).
    n: u64,
    next_us: f64,
    down: Resampler,
    up: Resampler,
}

impl Ionos {
    /// `paths`: 2 or 4 (the firmware's MULTIPATHS setting); `bandwidth`:
    /// 3000 or 6000 Hz.
    pub fn new(mode: Mode, paths: usize, bandwidth: u32, seed: u64) -> Self {
        assert!(paths == 2 || paths == 4, "IONOS: 2 or 4 paths");
        assert!(bandwidth == 3000 || bandwidth == 6000, "IONOS: 3000 or 6000 Hz");
        let lp = if bandwidth == 3000 {
            &coeffs::LP_3K
        } else {
            &coeffs::LP_6K
        };
        let hil: Vec<f32> = coeffs::HILBERT
            .iter()
            .map(|&c| ((c * HILBERT_SCALE).round().clamp(-32_768.0, 32_767.0) / 32_768.0) as f32)
            .collect();
        let delays = mode.delays_ms(paths);
        let taps_i = delays.iter().map(|d| delay_samples(HILBERT_COMP_MS + d)).collect();
        let taps_q = delays.iter().map(|&d| delay_samples(d)).collect();
        let n_paths = delays.len();
        let mut s = Self {
            mode,
            bandwidth: bandwidth as f32,
            lp_in: Fir::q15(lp),
            lp_noise: Fir::q15(lp),
            hilbert: Fir::new(hil),
            line_i: vec![0.0; 512],
            line_q: vec![0.0; 512],
            pos: 0,
            taps_i,
            taps_q,
            gens: (0..n_paths).map(|_| PathGain::new()).collect(),
            gains: vec![(0.0, 0.0); n_paths],
            pending: vec![(0.0, 0.0); n_paths],
            // The firmware's literal, not SQRT_2: the port stays bit-exact.
            #[allow(clippy::approx_constant)]
            std: if paths == 4 { 4.0 / 1.41421 } else { 4.0 },
            rng: Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1),
            noise: Rng(seed.wrapping_add(0xA5A5_5A5A) | 1),
            n: 0,
            next_us: 0.0,
            down: Resampler::new(AUDIO_RATE as u32, SIM_RATE as u32),
            up: Resampler::new(SIM_RATE as u32, AUDIO_RATE as u32),
        };
        if mode == Mode::Wgn {
            s.gains[0] = (WGN_GAIN, 0.0);
        } else {
            // The box has been running: fill the Doppler filters first.
            for _ in 0..coeffs::GAUSS.len() {
                s.update();
            }
            s.gains.clone_from(&s.pending);
        }
        s
    }

    /// One gain update for every path, in the firmware's order.
    fn update(&mut self) {
        for (g, p) in self.gens.iter_mut().zip(self.pending.iter_mut()) {
            *p = g.step(&mut self.rng, self.std);
        }
    }

    /// Delay through the box at 48 kHz, samples (filters, Hilbert
    /// compensation, longest path), rounded up.
    pub fn latency(&self) -> usize {
        let sim = 59.5 + self.taps_i.iter().max().copied().unwrap_or(0) as f64;
        (sim * AUDIO_RATE as f64 / SIM_RATE).ceil() as usize + 8
    }

    /// One 44.1 kHz sample through the paths (no noise).
    fn sample(&mut self, x: f32) -> f32 {
        if let Some(period) = self.mode.update_us()
            && self.n.is_multiple_of(BLOCK as u64)
        {
            let now_us = self.n as f64 * 1e6 / SIM_RATE;
            // Updates due since the last block, the newest applied now.
            while now_us >= self.next_us {
                self.update();
                self.next_us += period;
            }
            self.gains.clone_from(&self.pending);
        }
        self.n += 1;
        let s = PATH_SELECT * self.lp_in.push(x);
        let q = self.hilbert.push(s);
        let len = self.line_i.len();
        self.pos = (self.pos + 1) % len;
        self.line_i[self.pos] = s;
        self.line_q[self.pos] = q;
        let mut y = 0.0;
        for (k, &(gi, gq)) in self.gains.iter().enumerate() {
            let i = self.line_i[(self.pos + len - self.taps_i[k]) % len];
            let q = self.line_q[(self.pos + len - self.taps_q[k]) % len];
            y += gi * i - gq * q;
        }
        // WGN gives back the input level.
        y / (PATH_SELECT * WGN_GAIN)
    }

    /// `input` (48 kHz) through the channel into `out` (same length), with
    /// noise at `snr_db` against `reference_power` (see [`Reference`] and
    /// [`reference_power`]) in the filter's bandwidth. `None`: no noise.
    pub fn process(&mut self, input: &[f32], noise: Option<(f32, f32)>, out: &mut [f32]) {
        assert_eq!(input.len(), out.len());
        let sim_in = self.down.run(input, None);
        let sim_out = self.process_sim(&sim_in, noise);
        let back = self.up.run(&sim_out, Some(input.len()));
        out.copy_from_slice(&back);
    }

    /// The box itself, at its own 44.1 kHz: `input` through the paths, with
    /// noise as [`process`](Self::process). The state carries over from
    /// call to call, so a stream can go through in blocks (with
    /// [`StreamResampler`] either side).
    pub fn process_sim(&mut self, input: &[f32], noise: Option<(f32, f32)>) -> Vec<f32> {
        // White noise of variance σ² at 44.1 kHz puts σ²·B/(fs/2) into B Hz
        // after the (unity-gain) low-pass.
        let amp = noise.map(|(power, snr_db)| {
            let n_power = power / 10f32.powf(snr_db / 10.0);
            let var = n_power * (SIM_RATE as f32 / 2.0) / self.bandwidth;
            (3.0 * var).sqrt() // uniform in [-a, a]: variance a²/3
        });
        input
            .iter()
            .map(|&x| {
                let mut y = self.sample(x);
                if let Some(a) = amp {
                    y += self.lp_noise.push(a * self.noise.uniform());
                }
                y
            })
            .collect()
    }
}

/// The S:N reference power of `signal` (48 kHz audio).
pub fn reference_power(signal: &[f32], reference: Reference) -> f32 {
    match reference {
        Reference::Peak => {
            let peak = signal.iter().fold(0.0f32, |m, x| m.max(x.abs()));
            peak * peak / 2.0
        }
        Reference::Mean => signal.iter().map(|x| x * x).sum::<f32>() / signal.len().max(1) as f32,
    }
}

/// Windowed-sinc resampler between two integer rates (Blackman, 16 zero
/// crossings each side, cut off at 0.45 × the lower rate), as a polyphase
/// table: output `j` sits at input position `j·from/to`, whose fractional
/// part repeats every `to/gcd` outputs.
#[derive(Clone, Debug)]
pub struct Resampler {
    /// The rates over their gcd: output j sits at input j·from/to.
    from: u64,
    to: u64,
    /// Per phase: first input index offset and taps.
    phases: Vec<(i64, Vec<f32>)>,
}

impl Resampler {
    pub fn new(from: u32, to: u32) -> Self {
        let g = gcd(from as u64, to as u64);
        let (f, t) = (from as u64 / g, to as u64 / g);
        let fc = 0.45 * from.min(to) as f64;
        let half_s = 16.0 / (2.0 * fc);
        let gain = 2.0 * fc / from as f64;
        let phases = (0..t)
            .map(|j| {
                // Output j at time j/to; input k at k/from.
                let pos = j as f64 * f as f64 / t as f64; // input samples
                let k0 = (pos - half_s * from as f64).ceil() as i64;
                let k1 = (pos + half_s * from as f64).floor() as i64;
                let taps = (k0..=k1)
                    .map(|k| {
                        let d = (pos - k as f64) / from as f64;
                        let arg = 2.0 * fc * d;
                        let sinc = if arg.abs() < 1e-12 {
                            1.0
                        } else {
                            (PI * arg).sin() / (PI * arg)
                        };
                        let u = d / half_s;
                        let w = 0.42 + 0.5 * (PI * u).cos() + 0.08 * (2.0 * PI * u).cos();
                        (gain * sinc * w) as f32
                    })
                    .collect();
                (k0, taps)
            })
            .collect();
        Self { from: f, to: t, phases }
    }

    /// `x` resampled; `len` output samples (default: the same duration).
    pub fn run(&self, x: &[f32], len: Option<usize>) -> Vec<f32> {
        let n_out = len.unwrap_or_else(|| (x.len() as u64 * self.to / self.from) as usize);
        (0..n_out)
            .map(|j| {
                let cycle = (j as u64 / self.to) as i64 * self.from as i64;
                let (k0, taps) = &self.phases[j % self.to as usize];
                let start = cycle + k0;
                let mut acc = 0.0f32;
                for (i, &h) in taps.iter().enumerate() {
                    let k = start + i as i64;
                    if k >= 0
                        && let Some(&v) = x.get(k as usize)
                    {
                        acc += h * v;
                    }
                }
                acc
            })
            .collect()
    }
}

/// [`Resampler`] over a stream fed in blocks: it keeps the input it still
/// needs, and gives each output once all its taps have arrived (so it runs
/// half a filter behind, about 2 ms).
#[derive(Clone, Debug)]
pub struct StreamResampler {
    r: Resampler,
    /// Input not yet behind every output, and the stream index of its first
    /// sample.
    buf: Vec<f32>,
    base: i64,
    /// The next output's index.
    j: u64,
}

impl StreamResampler {
    pub fn new(from: u32, to: u32) -> Self {
        Self {
            r: Resampler::new(from, to),
            buf: Vec::new(),
            base: 0,
            j: 0,
        }
    }

    /// `x` in; every output now complete out.
    pub fn push(&mut self, x: &[f32]) -> Vec<f32> {
        self.buf.extend_from_slice(x);
        let end = self.base + self.buf.len() as i64;
        let mut out = Vec::new();
        loop {
            let cycle = (self.j / self.r.to) as i64 * self.r.from as i64;
            let (k0, taps) = &self.r.phases[(self.j % self.r.to) as usize];
            let start = cycle + k0;
            if start + taps.len() as i64 > end {
                break;
            }
            let mut acc = 0.0f32;
            for (i, &h) in taps.iter().enumerate() {
                let k = start + i as i64;
                if k >= self.base {
                    acc += h * self.buf[(k - self.base) as usize];
                }
            }
            out.push(acc);
            self.j += 1;
        }
        // Drop what no later output reaches.
        let cycle = (self.j / self.r.to) as i64 * self.r.from as i64;
        let first = cycle + self.r.phases[(self.j % self.r.to) as usize].0;
        let drop = (first - self.base).clamp(0, self.buf.len() as i64) as usize;
        self.buf.drain(..drop);
        self.base += drop as i64;
        out
    }
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { gcd(b, a % b) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(hz: f32, secs: f32, amp: f32) -> Vec<f32> {
        let n = (secs * AUDIO_RATE) as usize;
        (0..n)
            .map(|i| amp * (std::f32::consts::TAU * hz * i as f32 / AUDIO_RATE).sin())
            .collect()
    }

    fn power(x: &[f32]) -> f32 {
        x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32
    }

    #[test]
    fn stream_resampler_matches_batch() {
        // Fed in odd blocks, the stream gives the batch output (where the
        // batch saw all its taps), only later.
        for (from, to) in [(8_000, 44_100), (44_100, 8_000), (48_000, 44_100)] {
            let x: Vec<f32> = (0..9_000)
                .map(|i| ((i as f32 * 0.37).sin() + (i as f32 * 0.011).cos()) * 0.3)
                .collect();
            let batch = Resampler::new(from, to).run(&x, None);
            let mut s = StreamResampler::new(from, to);
            let mut got = Vec::new();
            for c in x.chunks(161) {
                got.extend(s.push(c));
            }
            let tail = batch.len() / 10;
            assert!(
                got.len() + tail >= batch.len(),
                "{from}->{to}: {} of {}",
                got.len(),
                batch.len()
            );
            for (j, (a, b)) in got.iter().zip(&batch).enumerate().skip(tail) {
                assert!((a - b).abs() < 1e-5, "{from}->{to} at {j}: {a} vs {b}");
            }
        }
    }

    #[test]
    fn teensy_delays() {
        // Hilbert compensation and the path delays, in samples.
        assert_eq!(delay_samples(HILBERT_COMP_MS), 59);
        assert_eq!(Mode::Mpp.delays_ms(2), vec![0.0, 2.0]);
        assert_eq!(delay_samples(2.0), 88);
        assert_eq!(Mode::Mpd.delays_ms(4), vec![0.0, 2.0, 4.0, 6.0]);
        assert_eq!(Mode::Mpg.delays_ms(4), vec![0.0, 0.25, 0.5, 0.75]);
    }

    #[test]
    fn gauss_filter_is_the_documented_one() {
        // "Freq Response: 1Hz: -9.1dB, 2Hz: -37.1 dB" at 64 Hz.
        let resp = |f: f64| {
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (k, &h) in coeffs::GAUSS.iter().enumerate() {
                let ph = std::f64::consts::TAU * f * k as f64 / 64.0;
                re += h as f64 * ph.cos();
                im -= h as f64 * ph.sin();
            }
            20.0 * (re * re + im * im).sqrt().log10()
        };
        assert!(resp(0.0).abs() < 0.01, "{}", resp(0.0));
        assert!((resp(1.0) + 9.1).abs() < 0.3, "{}", resp(1.0));
        assert!((resp(2.0) + 37.1).abs() < 1.0, "{}", resp(2.0));
    }

    #[test]
    fn wgn_passes_a_tone_at_its_level() {
        let mut c = Ionos::new(Mode::Wgn, 2, 3000, 1);
        let x = tone(1500.0, 1.0, 0.3);
        let mut y = vec![0.0; x.len()];
        c.process(&x, None, &mut y);
        let skip = 4800;
        let ratio = power(&y[skip..]) / power(&x[skip..]);
        assert!((10.0 * ratio.log10()).abs() < 0.3, "{ratio}");
    }

    #[test]
    fn lowpass_stops_above_the_band() {
        let mut c = Ionos::new(Mode::Wgn, 2, 3000, 1);
        let x = tone(5000.0, 1.0, 0.3);
        let mut y = vec![0.0; x.len()];
        c.process(&x, None, &mut y);
        let ratio = power(&y[4800..]) / power(&x[4800..]);
        assert!(10.0 * ratio.log10() < -40.0, "{}", 10.0 * ratio.log10());
    }

    #[test]
    fn noise_reads_the_set_snr_in_3k() {
        // No signal; noise for S:N 10 dB against a full-scale sine reference.
        let mut c = Ionos::new(Mode::Wgn, 2, 3000, 3);
        let x = vec![0.0f32; 48_000 * 20];
        let mut y = vec![0.0; x.len()];
        let reference = 0.5f32; // sine of peak 1
        c.process(&x, Some((reference, 10.0)), &mut y);
        // White noise of variance v before the low-pass leaves v·Σh² after
        // it, and v·3000/22050 within 3000 Hz of its (unity) passband.
        let sumsq: f32 = coeffs::LP_3K.iter().map(|&c| (c as f32 / 32_768.0).powi(2)).sum();
        let total = power(&y[4800..]);
        let in_3k = total / sumsq * 3000.0 / 22_050.0;
        let snr = 10.0 * (reference / in_3k).log10();
        assert!((snr - 10.0).abs() < 0.5, "{snr}");
    }

    /// The firmware's gain scaling (4 for two paths) against WGN's 0.8498.
    #[test]
    fn multipath_mean_level() {
        for (paths, mode) in [(2, Mode::Mpd), (4, Mode::Mpd)] {
            let mut c = Ionos::new(mode, paths, 3000, 7);
            let x = tone(1500.0, 120.0, 0.3);
            let mut y = vec![0.0; x.len()];
            c.process(&x, None, &mut y);
            let db = 10.0 * (power(&y[4800..]) / power(&x[4800..])).log10();
            assert!(db.abs() < 2.5, "{db}");
        }
    }
}
