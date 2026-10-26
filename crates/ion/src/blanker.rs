//! Impulse noise blanker for the receive front ends (static crashes,
//! ignition clicks, power-line buzz pulses).
//!
//! It sits on the 48 kHz audio before a receiver's mix-down. A detector
//! watches the complex envelope of 100–2900 Hz around the receiver's
//! centre (8 kHz, a short filter so a click stays short) against a running
//! mean of the clean samples. A sample more than [`BLANK_DB`] above that
//! mean starts a *run*; the run goes on while samples stay
//! [`CONTINUE_DB`] above it and ends after [`HOLD_MS`] below. A run that
//! ended within [`MAX_RUN_MS`] was an impulse: its audio (plus a fraction
//! of a millisecond each side) is zeroed. A run that lasts longer is a new
//! level (a burst starting, a fade ending): nothing is zeroed and the mean
//! moves to it. Deciding that needs the run's end, so the audio comes out
//! [`Blanker::delay`] samples late; the receivers take that delay out of
//! the times they report.
//!
//! The receivers also learn *where* audio was zeroed
//! ([`Blanker::blanked_count`]) and treat what a blanked stretch fed as
//! less reliable.

use ion_dsp::dsp::{AUDIO_RATE, Decimator, Nco, lowpass_taps};

/// Start of a run: this far above the running mean, dB. Well above both
/// Gaussian noise and the clipped OFDM bursts' peaks.
pub const BLANK_DB: f32 = 12.0;
/// A run continues while samples are this far above the mean, dB.
pub const CONTINUE_DB: f32 = 6.0;
/// A run ends after this long below [`CONTINUE_DB`].
pub const HOLD_MS: f32 = 2.0;
/// Runs longer than this are a new level, not an impulse.
pub const MAX_RUN_MS: f32 = 20.0;

/// Detector decimation (48 kHz → 8 kHz).
const DEC: usize = 6;
const DET_RATE: f32 = AUDIO_RATE / DEC as f32;
/// Detector filter: ±1.4 kHz around the centre, 2.5 ms long.
const DET_TAPS: usize = 121;
const DET_CUTOFF_HZ: f32 = 1400.0;
/// Audio samples between the newest input and the centre of the detector
/// filter.
const DET_DELAY: usize = (DET_TAPS - 1) / 2;
/// Running mean: 50 ms (detector samples), and the first 20 ms that seed it.
const ENV_SAMPLES: f32 = 0.050 * DET_RATE;
const WARM: u64 = (0.020 * DET_RATE) as u64;
/// Audio zeroed before a run's first sample and after its last (the
/// detector filter's main lobe, about ±0.3 ms).
const PAD: usize = 2 * DEC + 3;

const fn det_samples(ms: f32) -> usize {
    (ms * 1e-3 * DET_RATE) as usize
}
const HOLD: usize = det_samples(HOLD_MS);
const MAX_RUN: usize = det_samples(MAX_RUN_MS);
/// Output delay, audio samples: every decision about an audio sample is
/// final by the time it leaves. A whole number of 8 kHz samples, so the
/// receivers' decimators keep their phase.
const DELAY: usize = (DET_DELAY + DEC * (MAX_RUN + HOLD + 4) + PAD).div_ceil(DEC) * DEC;
/// Audio history kept: a power of two above `DELAY` plus the longest
/// receiver filter's half length.
const RING: usize = 4096;
const MASK: u64 = RING as u64 - 1;
const _: () = assert!(DELAY + 1024 < RING);

#[derive(Clone, Copy)]
struct Run {
    /// Audio indices (filter centre) of the first and latest sample above
    /// the thresholds, detector index of the latest.
    a_start: u64,
    a_last: u64,
    k_start: u64,
    k_last: u64,
    sum: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BlankerStats {
    /// Runs blanked as impulses.
    pub impulses: u64,
    /// Runs taken as a new level.
    pub levels: u64,
    /// Audio samples zeroed.
    pub blanked: u64,
}

pub struct Blanker {
    on: bool,
    nco: Nco,
    dec: Decimator,
    start_thr: f32,
    cont_thr: f32,
    /// Raw audio, and whether each sample is to be zeroed.
    buf: Vec<f32>,
    zero: Vec<bool>,
    /// Audio samples in.
    n_in: u64,
    /// Detector samples.
    k: u64,
    /// Running mean of the clean detector power.
    env: f32,
    run: Option<Run>,
    /// `cum[i & MASK]`: samples zeroed among outputs before `i`.
    cum: Vec<u64>,
    stats: BlankerStats,
}

impl Blanker {
    /// A blanker for a receiver centred on `center_hz` (the detector looks
    /// at ±1.4 kHz around it, kept above 0 Hz). On until [`Blanker::set_on`].
    pub fn new(center_hz: f32) -> Self {
        Self {
            on: true,
            nco: Nco::new(center_hz.max(1500.0), AUDIO_RATE),
            dec: Decimator::new(DEC, lowpass_taps(DET_TAPS, DET_CUTOFF_HZ, AUDIO_RATE)),
            start_thr: 10f32.powf(BLANK_DB / 10.0),
            cont_thr: 10f32.powf(CONTINUE_DB / 10.0),
            buf: vec![0.0; RING],
            zero: vec![false; RING],
            n_in: 0,
            k: 0,
            env: 0.0,
            run: None,
            cum: vec![0; RING],
            stats: BlankerStats::default(),
        }
    }

    /// # Panics
    /// After the first sample (the output delay would jump).
    pub fn set_on(&mut self, on: bool) {
        assert!(self.n_in == 0, "Blanker::set_on after audio has gone through");
        self.on = on;
    }

    pub fn is_on(&self) -> bool {
        self.on
    }

    /// Audio samples between a sample going in and coming out (0 when off).
    pub fn delay(&self) -> usize {
        if self.on { DELAY } else { 0 }
    }

    pub fn stats(&self) -> BlankerStats {
        self.stats
    }

    /// Takes one audio sample, returns the sample [`delay`] samples
    /// earlier, zeroed if it was part of an impulse (zeros before the
    /// first).
    ///
    /// [`delay`]: Blanker::delay
    #[inline]
    pub fn push(&mut self, x: f32) -> f32 {
        if !self.on {
            return x;
        }
        let i = (self.n_in & MASK) as usize;
        self.buf[i] = x;
        self.zero[i] = false;
        self.n_in += 1;
        if let Some(z) = self.dec.push(self.nco.mix_down(x)) {
            let centre = (self.n_in - 1).saturating_sub(DET_DELAY as u64);
            self.detect(z.norm_sqr(), centre);
        }
        // Output index n_in − 1 is input index n_in − 1 − DELAY.
        let out = self.n_in - 1;
        let (v, z) = match out.checked_sub(DELAY as u64) {
            Some(m) => {
                let j = (m & MASK) as usize;
                if self.zero[j] {
                    (0.0, true)
                } else {
                    (self.buf[j], false)
                }
            }
            None => (0.0, false),
        };
        let before = self.cum[(out & MASK) as usize];
        self.cum[((out + 1) & MASK) as usize] = before + z as u64;
        v
    }

    /// Of the outputs from `back + half` to `back − half` samples before the
    /// newest, how many were zeroed (0 when off). `back + half` must stay
    /// below 3000.
    #[inline]
    pub fn blanked_count(&self, back: usize, half: usize) -> u32 {
        if !self.on {
            return 0;
        }
        let end = self.n_in as i64 - back as i64 + half as i64; // exclusive
        let start = end - 2 * half as i64 - 1;
        if end <= 0 {
            return 0;
        }
        let at = |i: i64| self.cum[(i.max(0) as u64 & MASK) as usize];
        (at(end) - at(start)) as u32
    }

    fn detect(&mut self, p: f32, centre: u64) {
        let k = self.k;
        self.k += 1;
        if k < WARM {
            self.env += (p - self.env) / (k + 1) as f32;
            return;
        }
        match self.run.as_mut() {
            None => {
                if p > self.start_thr * self.env {
                    self.run = Some(Run {
                        a_start: centre,
                        a_last: centre,
                        k_start: k,
                        k_last: k,
                        sum: p as f64,
                    });
                } else {
                    self.env += (p - self.env) / ENV_SAMPLES;
                }
            }
            Some(r) => {
                r.sum += p as f64;
                if p > self.cont_thr * self.env {
                    r.a_last = centre;
                    r.k_last = k;
                }
                let r = *r;
                if k - r.k_last > HOLD as u64 {
                    self.stats.impulses += 1;
                    self.mark(r.a_start.saturating_sub(PAD as u64), r.a_last + PAD as u64);
                    self.run = None;
                } else if k - r.k_start > MAX_RUN as u64 {
                    self.stats.levels += 1;
                    self.env = (r.sum / (k - r.k_start + 1) as f64) as f32;
                    self.run = None;
                }
            }
        }
    }

    /// Zeroes audio `lo..=hi` (input indices not yet out).
    fn mark(&mut self, lo: u64, hi: u64) {
        // `push` sends input n_in − 1 − DELAY right after this.
        let first_unsent = (self.n_in - 1).saturating_sub(DELAY as u64);
        let hi = hi.min(self.n_in - 1);
        for a in lo.max(first_unsent)..=hi {
            let j = (a & MASK) as usize;
            if !self.zero[j] {
                self.zero[j] = true;
                self.stats.blanked += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ion_dsp::dsp::NoiseGen;

    fn run(b: &mut Blanker, x: &[f32]) -> Vec<f32> {
        let mut y: Vec<f32> = x.iter().map(|&v| b.push(v)).collect();
        y.drain(..b.delay());
        y
    }

    #[test]
    fn off_is_transparent() {
        let mut b = Blanker::new(1500.0);
        b.set_on(false);
        let x: Vec<f32> = (0..1000).map(|i| (i as f32 * 0.1).sin()).collect();
        let y: Vec<f32> = x.iter().map(|&v| b.push(v)).collect();
        assert_eq!(x, y);
        assert_eq!(b.delay(), 0);
    }

    #[test]
    fn white_noise_is_left_alone() {
        let mut g = NoiseGen::new(5);
        let x: Vec<f32> = (0..48_000 * 10).map(|_| g.gaussian()).collect();
        let mut b = Blanker::new(1500.0);
        b.set_on(true);
        let y = run(&mut b, &x);
        let changed = x.iter().zip(&y).filter(|(a, b)| a != b).count();
        assert!(changed < 48, "{changed} samples changed, {:?}", b.stats());
    }

    #[test]
    fn clicks_are_zeroed_and_bursts_are_not() {
        let mut g = NoiseGen::new(9);
        let n = 48_000 * 2;
        let mut x: Vec<f32> = (0..n).map(|_| 0.1 * g.gaussian()).collect();
        // A 1 ms click 30 dB up at 0.5 s.
        for v in &mut x[24_000..24_048] {
            *v += 3.0 * g.gaussian();
        }
        // A tone 30 dB up from 1.0 s on: a new level, not an impulse.
        for (i, v) in x.iter_mut().enumerate().skip(48_000) {
            *v += 3.0 * (i as f32 * 0.2).sin();
        }
        let mut b = Blanker::new(1500.0);
        b.set_on(true);
        let y = run(&mut b, &x);
        assert!(y[24_000..24_048].iter().all(|&v| v == 0.0), "{:?}", b.stats());
        assert!(
            y[48_000..].iter().zip(&x[48_000..]).all(|(a, b)| a == b),
            "{:?}",
            b.stats()
        );
        assert_eq!(b.stats().impulses, 1);
        assert_eq!(b.stats().levels, 1);
    }

    #[test]
    fn counts_line_up_with_the_output() {
        let mut g = NoiseGen::new(3);
        let mut b = Blanker::new(1500.0);
        b.set_on(true);
        let mut y = Vec::new();
        let mut counts = Vec::new();
        for i in 0..48_000 {
            let mut v = 0.1 * g.gaussian();
            if (30_000..30_048).contains(&i) {
                v += 3.0 * g.gaussian();
            }
            y.push(b.push(v));
            counts.push(b.blanked_count(0, 0));
        }
        for (i, (&v, &c)) in y.iter().zip(&counts).enumerate().skip(b.delay()) {
            assert_eq!(c == 1, v == 0.0, "sample {i}");
        }
        assert!(counts.iter().sum::<u32>() >= 48);
    }
}
