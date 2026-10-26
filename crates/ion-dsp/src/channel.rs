//! HF channel simulator: the Watterson model of ITU-R F.1487 and CCIR Rec. 520.
//!
//! Each path is a delayed copy faded by an independent complex Gaussian tap
//! with a Gaussian Doppler spectrum; "spread" is two standard deviations of
//! that spectrum. Path powers are equal and sum to 1. Input and output are
//! real 48 kHz audio (made analytic with a Hilbert FIR inside); noise is added
//! separately ([`add_awgn`], [`ImpulseNoise`]) so the SNR can be set against
//! the clean signal.

use core::f64::consts::TAU;

use crate::dsp::{AUDIO_RATE, Complex, NoiseGen};

/// One propagation path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathSpec {
    pub delay_ms: f32,
    /// Two-sigma Doppler spread in Hz; 0 is a static (non-fading) path.
    pub spread_hz: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ChannelSpec {
    pub name: &'static str,
    pub paths: Vec<PathSpec>,
}

impl ChannelSpec {
    /// Two equal-power fading paths `delay_ms` apart, both with `spread_hz`.
    pub fn two_path(name: &'static str, delay_ms: f32, spread_hz: f32) -> Self {
        Self {
            name,
            paths: vec![
                PathSpec {
                    delay_ms: 0.0,
                    spread_hz,
                },
                PathSpec { delay_ms, spread_hz },
            ],
        }
    }
}

/// Standard test channels: CCIR Rec. 520 and the ITU-R F.1487 table.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Profile {
    Awgn,
    CcirGood,
    CcirModerate,
    CcirPoor,
    CcirFlutter,
    LowQuiet,
    LowModerate,
    LowDisturbed,
    MidQuiet,
    MidModerate,
    MidDisturbed,
    MidDisturbedNvis,
    HighQuiet,
    HighModerate,
    HighDisturbed,
}

impl Profile {
    pub const ALL: [Profile; 15] = [
        Profile::Awgn,
        Profile::CcirGood,
        Profile::CcirModerate,
        Profile::CcirPoor,
        Profile::CcirFlutter,
        Profile::LowQuiet,
        Profile::LowModerate,
        Profile::LowDisturbed,
        Profile::MidQuiet,
        Profile::MidModerate,
        Profile::MidDisturbed,
        Profile::MidDisturbedNvis,
        Profile::HighQuiet,
        Profile::HighModerate,
        Profile::HighDisturbed,
    ];

    pub fn spec(self) -> ChannelSpec {
        use ChannelSpec as C;
        match self {
            Profile::Awgn => C {
                name: "AWGN",
                paths: vec![PathSpec {
                    delay_ms: 0.0,
                    spread_hz: 0.0,
                }],
            },
            Profile::CcirGood => C::two_path("CCIR good", 0.5, 0.1),
            Profile::CcirModerate => C::two_path("CCIR moderate", 1.0, 0.5),
            Profile::CcirPoor => C::two_path("CCIR poor", 2.0, 1.0),
            Profile::CcirFlutter => C::two_path("CCIR flutter", 0.5, 10.0),
            Profile::LowQuiet => C::two_path("ITU low-lat quiet", 0.5, 0.5),
            Profile::LowModerate => C::two_path("ITU low-lat moderate", 2.0, 1.5),
            Profile::LowDisturbed => C::two_path("ITU low-lat disturbed", 6.0, 10.0),
            Profile::MidQuiet => C::two_path("ITU mid-lat quiet", 0.5, 0.1),
            Profile::MidModerate => C::two_path("ITU mid-lat moderate", 1.0, 0.5),
            Profile::MidDisturbed => C::two_path("ITU mid-lat disturbed", 2.0, 1.0),
            Profile::MidDisturbedNvis => C::two_path("ITU mid-lat disturbed NVIS", 7.0, 1.0),
            Profile::HighQuiet => C::two_path("ITU high-lat quiet", 1.0, 0.5),
            Profile::HighModerate => C::two_path("ITU high-lat moderate", 3.0, 10.0),
            Profile::HighDisturbed => C::two_path("ITU high-lat disturbed", 7.0, 30.0),
        }
    }
}

/// Unit-power complex Gaussian process with a Gaussian Doppler spectrum,
/// generated at a low rate and linearly interpolated to the audio rate.
#[derive(Clone, Debug)]
struct FadingTap {
    taps: Vec<f32>,
    hist: Vec<Complex>,
    pos: usize,
    noise: NoiseGen,
    prev: Complex,
    next: Complex,
    frac: f64,
    step: f64,
    fixed: Option<Complex>,
}

impl FadingTap {
    fn new(spread_hz: f32, seed: u64) -> Self {
        if spread_hz <= 0.0 {
            return Self {
                taps: Vec::new(),
                hist: Vec::new(),
                pos: 0,
                noise: NoiseGen::new(seed),
                prev: Complex::ZERO,
                next: Complex::ZERO,
                frac: 0.0,
                step: 0.0,
                fixed: Some(Complex::new(1.0, 0.0)),
            };
        }
        let sigma = spread_hz as f64 / 2.0;
        // Sample the tap well above its bandwidth so linear interpolation
        // is clean; the filter length then comes out the same at any spread.
        let rate = 32.0 * spread_hz as f64;
        // |H(f)|² = exp(-f²/2σ²) ⇔ h(t) ∝ exp(-(2πσt)²); keep ±4 std devs.
        let t_std = 1.0 / (2.0 * core::f64::consts::SQRT_2 * core::f64::consts::PI * sigma);
        let half = (4.0 * t_std * rate).ceil() as i64;
        let mut taps: Vec<f32> = (-half..=half)
            .map(|n| {
                let t = n as f64 / rate;
                (-(TAU / 2.0 * 2.0 * sigma * t).powi(2)).exp() as f32
            })
            .collect();
        let energy: f32 = taps.iter().map(|h| h * h).sum();
        for h in &mut taps {
            *h /= energy.sqrt();
        }
        let mut s = Self {
            hist: vec![Complex::ZERO; taps.len()],
            taps,
            pos: 0,
            noise: NoiseGen::new(seed),
            prev: Complex::ZERO,
            next: Complex::ZERO,
            frac: 0.0,
            step: rate / AUDIO_RATE as f64,
            fixed: None,
        };
        // Fill the filter so there is no start-up transient.
        for _ in 0..s.taps.len() {
            s.low_rate();
        }
        s.prev = s.low_rate();
        s.next = s.low_rate();
        s
    }

    /// One output of the low-rate Gaussian filter.
    fn low_rate(&mut self) -> Complex {
        // Unit total power: real and imaginary parts each have variance 1/2.
        let k = core::f32::consts::FRAC_1_SQRT_2;
        let w = Complex::new(self.noise.gaussian() * k, self.noise.gaussian() * k);
        self.hist[self.pos] = w;
        let n = self.hist.len();
        let mut acc = Complex::ZERO;
        for (i, h) in self.taps.iter().enumerate() {
            acc += self.hist[(self.pos + n - i) % n].scale(*h);
        }
        self.pos = (self.pos + 1) % n;
        acc
    }

    #[inline]
    fn next_gain(&mut self) -> Complex {
        if let Some(g) = self.fixed {
            return g;
        }
        let f = self.frac as f32;
        let g = self.prev.scale(1.0 - f) + self.next.scale(f);
        self.frac += self.step;
        while self.frac >= 1.0 {
            self.frac -= 1.0;
            self.prev = self.next;
            self.next = self.low_rate();
        }
        g
    }
}

/// Hilbert transformer length (odd). Accurate (≲0.1 dB) from ~250 Hz up.
const HILBERT_LEN: usize = 401;

#[derive(Clone, Debug)]
struct PathState {
    delay: usize,
    amp: f32,
    tap: FadingTap,
}

pub struct Channel {
    hilbert: Vec<f32>,
    x_hist: Vec<f32>,
    x_pos: usize,
    z_hist: Vec<Complex>,
    z_pos: usize,
    paths: Vec<PathState>,
    phase: f64,
    freq_hz: f64,
    drift_hz_per_s: f64,
}

impl Channel {
    pub fn new(spec: &ChannelSpec, seed: u64) -> Self {
        let center = (HILBERT_LEN / 2) as i64;
        let hilbert = (0..HILBERT_LEN as i64)
            .map(|i| {
                let n = i - center;
                if n % 2 == 0 {
                    return 0.0;
                }
                let m = (HILBERT_LEN - 1) as f64;
                let w = 0.42 - 0.5 * (TAU * i as f64 / m).cos() + 0.08 * (2.0 * TAU * i as f64 / m).cos();
                (2.0 / (core::f64::consts::PI * n as f64) * w) as f32
            })
            .collect();
        let amp = 1.0 / (spec.paths.len() as f32).sqrt();
        let paths: Vec<PathState> = spec
            .paths
            .iter()
            .enumerate()
            .map(|(i, p)| PathState {
                delay: (p.delay_ms * AUDIO_RATE / 1000.0).round() as usize,
                amp,
                tap: FadingTap::new(p.spread_hz, seed.wrapping_mul(0x9E37_79B9).wrapping_add(i as u64 + 1)),
            })
            .collect();
        let max_delay = paths.iter().map(|p| p.delay).max().unwrap_or(0);
        Self {
            hilbert,
            x_hist: vec![0.0; 2 * HILBERT_LEN],
            x_pos: 0,
            z_hist: vec![Complex::ZERO; max_delay + 1],
            z_pos: 0,
            paths,
            phase: 0.0,
            freq_hz: 0.0,
            drift_hz_per_s: 0.0,
        }
    }

    pub fn from_profile(profile: Profile, seed: u64) -> Self {
        Self::new(&profile.spec(), seed)
    }

    /// Adds a frequency offset and a linear drift (Hz per second).
    pub fn with_offset(mut self, offset_hz: f32, drift_hz_per_s: f32) -> Self {
        self.freq_hz = offset_hz as f64;
        self.drift_hz_per_s = drift_hz_per_s as f64;
        self
    }

    /// Samples of delay added by the Hilbert transformer (before any path delay).
    pub fn latency(&self) -> usize {
        HILBERT_LEN / 2
    }

    /// Runs `input` through the channel into `out` (same length).
    pub fn process(&mut self, input: &[f32], out: &mut [f32]) {
        assert_eq!(input.len(), out.len());
        let n = HILBERT_LEN;
        let half = n / 2;
        let dt = 1.0 / AUDIO_RATE as f64;
        for (x, o) in input.iter().zip(out.iter_mut()) {
            // Analytic signal: delayed real part + j·Hilbert.
            self.x_pos = if self.x_pos == 0 { n - 1 } else { self.x_pos - 1 };
            self.x_hist[self.x_pos] = *x;
            self.x_hist[self.x_pos + n] = *x;
            let w = &self.x_hist[self.x_pos..self.x_pos + n];
            let mut im = 0.0f32;
            for k in (1..n).step_by(2) {
                im += self.hilbert[k] * w[k];
            }
            let z = Complex::new(w[half], im);

            let len = self.z_hist.len();
            self.z_hist[self.z_pos] = z;
            let mut acc = Complex::ZERO;
            for p in &mut self.paths {
                let past = self.z_hist[(self.z_pos + len - p.delay) % len];
                acc += past * p.tap.next_gain().scale(p.amp);
            }
            self.z_pos = (self.z_pos + 1) % len;

            if self.freq_hz != 0.0 || self.drift_hz_per_s != 0.0 {
                acc = acc * Complex::from_polar(1.0, (self.phase * TAU) as f32);
                self.phase += self.freq_hz * dt;
                self.phase -= self.phase.floor();
                self.freq_hz += self.drift_hz_per_s * dt;
            }
            *o = acc.re;
        }
    }
}

/// Adds white Gaussian noise of standard deviation `sigma` (see
/// [`crate::dsp::noise_sigma`] to set it from an SNR).
pub fn add_awgn(samples: &mut [f32], sigma: f32, noise: &mut NoiseGen) {
    for s in samples {
        *s += sigma * noise.gaussian();
    }
}

/// Random clicks (static crashes, ignition noise): Poisson arrivals, each a
/// short exponentially decaying burst of noise.
pub struct ImpulseNoise {
    rate_hz: f32,
    amplitude: f32,
    decay: f32,
    env: f32,
    noise: NoiseGen,
}

impl ImpulseNoise {
    /// `rate_hz` clicks per second, peak `amplitude`, each lasting ~`ms`.
    pub fn new(rate_hz: f32, amplitude: f32, ms: f32, seed: u64) -> Self {
        Self {
            rate_hz,
            amplitude,
            decay: (-1.0 / (ms * 1e-3 * AUDIO_RATE)).exp(),
            env: 0.0,
            noise: NoiseGen::new(seed),
        }
    }

    pub fn add(&mut self, samples: &mut [f32]) {
        let p = self.rate_hz / AUDIO_RATE;
        for s in samples {
            if self.noise.uniform() < p {
                self.env = self.amplitude;
            }
            if self.env > 1e-6 {
                *s += self.env * self.noise.gaussian();
                self.env *= self.decay;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profiles_are_two_path_except_awgn() {
        for p in Profile::ALL {
            let s = p.spec();
            assert_eq!(s.paths.len(), if p == Profile::Awgn { 1 } else { 2 }, "{}", s.name);
        }
        assert_eq!(
            Profile::LowModerate.spec().paths[1],
            PathSpec {
                delay_ms: 2.0,
                spread_hz: 1.5
            }
        );
    }

    #[test]
    fn fading_tap_has_unit_power() {
        let mut t = FadingTap::new(10.0, 3);
        let n = 48_000 * 60;
        let p: f64 = (0..n).map(|_| t.next_gain().norm_sqr() as f64).sum::<f64>() / n as f64;
        assert!((p - 1.0).abs() < 0.1, "{p}");
    }
}
