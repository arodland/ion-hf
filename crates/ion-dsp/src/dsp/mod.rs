//! Complex arithmetic, oscillators, FIR filters and a deterministic noise
//! source. Nothing here allocates after `new()`.

use core::f32::consts::PI;
use core::ops::{Add, AddAssign, Mul, Neg, Sub};

/// Audio rate on the TCI side and at the modulator output.
pub const AUDIO_RATE: f32 = 48_000.0;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Complex {
    pub re: f32,
    pub im: f32,
}

impl Complex {
    pub const ZERO: Complex = Complex { re: 0.0, im: 0.0 };

    #[inline]
    pub const fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }

    #[inline]
    pub fn from_polar(mag: f32, angle: f32) -> Self {
        let (s, c) = angle.sin_cos();
        Self::new(mag * c, mag * s)
    }

    #[inline]
    pub fn conj(self) -> Self {
        Self::new(self.re, -self.im)
    }

    #[inline]
    pub fn norm_sqr(self) -> f32 {
        self.re * self.re + self.im * self.im
    }

    #[inline]
    pub fn norm(self) -> f32 {
        self.norm_sqr().sqrt()
    }

    #[inline]
    pub fn arg(self) -> f32 {
        self.im.atan2(self.re)
    }

    #[inline]
    pub fn scale(self, k: f32) -> Self {
        Self::new(self.re * k, self.im * k)
    }
}

impl Add for Complex {
    type Output = Complex;
    #[inline]
    fn add(self, o: Complex) -> Complex {
        Complex::new(self.re + o.re, self.im + o.im)
    }
}

impl Sub for Complex {
    type Output = Complex;
    #[inline]
    fn sub(self, o: Complex) -> Complex {
        Complex::new(self.re - o.re, self.im - o.im)
    }
}

impl Mul for Complex {
    type Output = Complex;
    #[inline]
    fn mul(self, o: Complex) -> Complex {
        Complex::new(self.re * o.re - self.im * o.im, self.re * o.im + self.im * o.re)
    }
}

impl Mul<f32> for Complex {
    type Output = Complex;
    #[inline]
    fn mul(self, k: f32) -> Complex {
        self.scale(k)
    }
}

impl Neg for Complex {
    type Output = Complex;
    #[inline]
    fn neg(self) -> Complex {
        Complex::new(-self.re, -self.im)
    }
}

impl AddAssign for Complex {
    #[inline]
    fn add_assign(&mut self, o: Complex) {
        self.re += o.re;
        self.im += o.im;
    }
}

/// Numerically controlled oscillator. Phase is in cycles as `f64` so it stays
/// exact over hours; frequency changes are phase-continuous.
#[derive(Clone, Debug)]
pub struct Nco {
    phase: f64,
    inc: f64,
    rate: f32,
}

impl Nco {
    pub fn new(freq_hz: f32, rate: f32) -> Self {
        Self {
            phase: 0.0,
            inc: freq_hz as f64 / rate as f64,
            rate,
        }
    }

    pub fn set_freq(&mut self, freq_hz: f32) {
        self.inc = freq_hz as f64 / self.rate as f64;
    }

    /// Returns `e^{j·2π·phase}` and advances one sample.
    #[inline]
    pub fn next_phasor(&mut self) -> Complex {
        let (s, c) = (self.phase * core::f64::consts::TAU).sin_cos();
        self.phase += self.inc;
        self.phase -= self.phase.floor();
        Complex::new(c as f32, s as f32)
    }

    /// Mixes a real sample down by the oscillator frequency.
    #[inline]
    pub fn mix_down(&mut self, x: f32) -> Complex {
        self.next_phasor().conj().scale(x)
    }
}

/// Complex-data FIR with real taps. The delay line is stored twice so every
/// dot product is over one contiguous slice.
#[derive(Clone, Debug)]
pub struct Fir {
    taps: Vec<f32>,
    buf: Vec<Complex>,
    pos: usize,
}

impl Fir {
    pub fn new(taps: Vec<f32>) -> Self {
        assert!(!taps.is_empty());
        let n = taps.len();
        Self {
            taps,
            buf: vec![Complex::ZERO; 2 * n],
            pos: 0,
        }
    }

    pub fn taps(&self) -> &[f32] {
        &self.taps
    }

    #[inline]
    pub fn push(&mut self, x: Complex) {
        let n = self.taps.len();
        self.pos = if self.pos == 0 { n - 1 } else { self.pos - 1 };
        self.buf[self.pos] = x;
        self.buf[self.pos + n] = x;
    }

    /// `y[n] = Σ h[k]·x[n-k]` for the samples pushed so far.
    #[inline]
    pub fn output(&self) -> Complex {
        let n = self.taps.len();
        let window = &self.buf[self.pos..self.pos + n];
        let mut re = 0.0f32;
        let mut im = 0.0f32;
        for (h, x) in self.taps.iter().zip(window) {
            re += h * x.re;
            im += h * x.im;
        }
        Complex::new(re, im)
    }

    #[inline]
    pub fn filter(&mut self, x: Complex) -> Complex {
        self.push(x);
        self.output()
    }
}

/// Windowed-sinc low-pass (Blackman window), normalised to unity DC gain.
pub fn lowpass_taps(n: usize, cutoff_hz: f32, rate: f32) -> Vec<f32> {
    assert!(n >= 3);
    let fc = cutoff_hz as f64 / rate as f64;
    let m = (n - 1) as f64;
    let mut taps: Vec<f32> = (0..n)
        .map(|i| {
            let x = i as f64 - m / 2.0;
            let sinc = if x == 0.0 {
                2.0 * fc
            } else {
                (core::f64::consts::TAU * fc * x).sin() / (core::f64::consts::PI * x)
            };
            let w = 0.42 - 0.5 * (core::f64::consts::TAU * i as f64 / m).cos()
                + 0.08 * (2.0 * core::f64::consts::TAU * i as f64 / m).cos();
            (sinc * w) as f32
        })
        .collect();
    let sum: f32 = taps.iter().sum();
    for t in &mut taps {
        *t /= sum;
    }
    taps
}

/// Magnitude of a real-tap FIR at `freq_hz` (DTFT).
pub fn fir_response(taps: &[f32], freq_hz: f32, rate: f32) -> f32 {
    let w = core::f64::consts::TAU * freq_hz as f64 / rate as f64;
    let (mut re, mut im) = (0.0f64, 0.0f64);
    for (k, &h) in taps.iter().enumerate() {
        re += h as f64 * (w * k as f64).cos();
        im -= h as f64 * (w * k as f64).sin();
    }
    (re * re + im * im).sqrt() as f32
}

/// Decimating FIR: only every `factor`-th output is computed.
#[derive(Clone, Debug)]
pub struct Decimator {
    fir: Fir,
    factor: usize,
    phase: usize,
}

impl Decimator {
    pub fn new(factor: usize, taps: Vec<f32>) -> Self {
        Self {
            fir: Fir::new(taps),
            factor,
            phase: 0,
        }
    }

    pub fn taps(&self) -> &[f32] {
        self.fir.taps()
    }

    #[inline]
    pub fn push(&mut self, x: Complex) -> Option<Complex> {
        self.fir.push(x);
        self.phase += 1;
        if self.phase == self.factor {
            self.phase = 0;
            Some(self.fir.output())
        } else {
            None
        }
    }
}

/// Deterministic Gaussian noise (xorshift64* + Box–Muller).
#[derive(Clone, Debug)]
pub struct NoiseGen {
    state: u64,
    spare: Option<f32>,
}

impl NoiseGen {
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed.max(1),
            spare: None,
        }
    }

    #[inline]
    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in (0, 1].
    #[inline]
    pub fn uniform(&mut self) -> f32 {
        ((self.next_u64() >> 40) as f32 + 1.0) / (1u64 << 24) as f32
    }

    /// Standard normal sample.
    #[inline]
    pub fn gaussian(&mut self) -> f32 {
        if let Some(s) = self.spare.take() {
            return s;
        }
        let u1 = self.uniform();
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let (s, c) = (2.0 * PI * u2).sin_cos();
        self.spare = Some(r * s);
        r * c
    }
}

/// Standard deviation of white noise at `rate` that gives `snr_db` against a
/// signal of power `signal_power`, measured in `bandwidth_hz`.
///
/// White noise of variance σ² spreads over `rate/2` Hz, so the power that
/// lands in the measurement bandwidth is σ²·bw/(rate/2).
pub fn noise_sigma(signal_power: f32, snr_db: f32, bandwidth_hz: f32, rate: f32) -> f32 {
    let var = signal_power * 10f32.powf(-snr_db / 10.0) * (rate / 2.0) / bandwidth_hz;
    var.sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nco_tone_and_mix_down() {
        let mut nco = Nco::new(1000.0, AUDIO_RATE);
        let mut mix = Nco::new(1000.0, AUDIO_RATE);
        // Mixing a 1 kHz cosine down by 1 kHz yields DC of 0.5.
        let mut acc = Complex::ZERO;
        for _ in 0..4800 {
            let x = nco.next_phasor().re;
            acc += mix.mix_down(x);
        }
        let avg = acc.scale(1.0 / 4800.0);
        assert!((avg.re - 0.5).abs() < 1e-3, "{avg:?}");
        assert!(avg.im.abs() < 1e-3, "{avg:?}");
    }

    #[test]
    fn decimator_impulse_is_taps() {
        let mut d = Decimator::new(1, vec![0.25, 0.5, 0.25]);
        let mut out = Vec::new();
        for i in 0..5 {
            let x = if i == 0 { Complex::new(1.0, 0.0) } else { Complex::ZERO };
            out.push(d.push(x).unwrap().re);
        }
        assert_eq!(out, vec![0.25, 0.5, 0.25, 0.0, 0.0]);
    }

    #[test]
    fn noise_sigma_gives_requested_snr() {
        let mut g = NoiseGen::new(7);
        let n = 200_000;
        let var: f32 = (0..n).map(|_| g.gaussian().powi(2)).sum::<f32>() / n as f32;
        assert!((var - 1.0).abs() < 0.02, "{var}");
        // 0 dB in 2.4 kHz at 48 kHz: total noise power is 10x the signal.
        let s = noise_sigma(0.5, 0.0, 2400.0, AUDIO_RATE);
        assert!((s * s - 5.0).abs() < 1e-4);
    }
}
