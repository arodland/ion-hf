//! Integer-ratio resampling between the TCI wire rate (12, 24 or 48 kHz)
//! and the internal 48 kHz, with one low-pass for both directions (Ion
//! lives below 3 kHz). [`FracResampler`] handles sound cards at other
//! rates (44.1 kHz).

use ion_dsp::dsp::lowpass_taps;

const RATE: f32 = 48_000.0;
const TAPS: usize = 193;
const CUTOFF_HZ: f32 = 4_500.0;

fn taps() -> Vec<f32> {
    lowpass_taps(TAPS, CUTOFF_HZ, RATE)
}

/// Wire rate → 48 kHz: zero-stuff by `k` and low-pass (polyphase).
pub struct Upsampler {
    k: usize,
    /// `phases[j][i]`: tap `i·k + j`, times `k` for unity gain.
    phases: Vec<Vec<f32>>,
    hist: Vec<f32>,
    pos: usize,
}

impl Upsampler {
    pub fn new(k: usize) -> Self {
        let h = taps();
        let per = h.len().div_ceil(k);
        let phases = (0..k)
            .map(|j| {
                (0..per)
                    .map(|i| h.get(i * k + j).copied().unwrap_or(0.0) * k as f32)
                    .collect()
            })
            .collect();
        Self {
            k,
            phases,
            hist: vec![0.0; per],
            pos: 0,
        }
    }

    /// Appends `input.len() · k` samples to `out`.
    pub fn process(&mut self, input: &[f32], out: &mut Vec<f32>) {
        if self.k == 1 {
            out.extend_from_slice(input);
            return;
        }
        let n = self.hist.len();
        for &x in input {
            self.pos = (self.pos + n - 1) % n;
            self.hist[self.pos] = x;
            for ph in &self.phases {
                let mut acc = 0.0;
                for (i, &t) in ph.iter().enumerate() {
                    acc += t * self.hist[(self.pos + i) % n];
                }
                out.push(acc);
            }
        }
    }
}

/// 48 kHz → wire rate: low-pass and keep every `k`-th sample.
pub struct Downsampler {
    k: usize,
    taps: Vec<f32>,
    hist: Vec<f32>,
    pos: usize,
    phase: usize,
}

impl Downsampler {
    pub fn new(k: usize) -> Self {
        let taps = taps();
        let n = taps.len();
        Self {
            k,
            taps,
            hist: vec![0.0; n],
            pos: 0,
            phase: 0,
        }
    }

    /// Consumes 48 kHz `input`; writes one output per `k` inputs into `out`
    /// and returns how many.
    pub fn process(&mut self, input: &[f32], out: &mut [f32]) -> usize {
        if self.k == 1 {
            let n = input.len().min(out.len());
            out[..n].copy_from_slice(&input[..n]);
            return n;
        }
        let n = self.hist.len();
        let mut w = 0;
        for &x in input {
            self.pos = (self.pos + n - 1) % n;
            self.hist[self.pos] = x;
            self.phase += 1;
            if self.phase == self.k {
                self.phase = 0;
                if w < out.len() {
                    let mut acc = 0.0;
                    for (i, &t) in self.taps.iter().enumerate() {
                        acc += t * self.hist[(self.pos + i) % n];
                    }
                    out[w] = acc;
                    w += 1;
                }
            }
        }
        w
    }
}

/// Any rate to any rate by 4-point cubic Hermite interpolation. Good for
/// audio below 3 kHz at 44.1 or 48 kHz, oversampled ~15 times; not for
/// large down-conversions of wideband audio.
pub struct FracResampler {
    /// Input samples per output sample.
    step: f64,
    pos: f64,
    hist: [f32; 4],
    passthrough: bool,
}

impl FracResampler {
    pub fn new(in_rate: u32, out_rate: u32) -> Self {
        Self {
            step: in_rate as f64 / out_rate as f64,
            pos: 0.0,
            hist: [0.0; 4],
            passthrough: in_rate == out_rate,
        }
    }

    /// Appends about `input.len() · out/in` samples to `out`.
    pub fn process(&mut self, input: &[f32], out: &mut Vec<f32>) {
        if self.passthrough {
            out.extend_from_slice(input);
            return;
        }
        for &x in input {
            self.hist.rotate_left(1);
            self.hist[3] = x;
            // Interpolate between hist[1] and hist[2].
            while self.pos < 1.0 {
                let t = self.pos as f32;
                let [y0, y1, y2, y3] = self.hist;
                let c1 = 0.5 * (y2 - y0);
                let c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
                let c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
                out.push(((c3 * t + c2) * t + c1) * t + y1);
                self.pos += self.step;
            }
            self.pos -= 1.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(hz: f32, rate: f32, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| (std::f32::consts::TAU * hz * i as f32 / rate).sin() * 0.5)
            .collect()
    }

    fn rms(x: &[f32]) -> f32 {
        (x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32).sqrt()
    }

    #[test]
    fn passband_tones_keep_their_level_both_ways() {
        for k in [2, 4] {
            let wire = RATE / k as f32;
            for hz in [300.0, 1500.0, 2900.0] {
                // Up: wire-rate tone → 48 kHz, compare with an ideal 48 kHz tone.
                let mut up = Upsampler::new(k);
                let mut out = Vec::new();
                up.process(&tone(hz, wire, 6000), &mut out);
                let ideal = tone(hz, RATE, out.len());
                let got = rms(&out[2000..]);
                assert!((got / rms(&ideal) - 1.0).abs() < 0.02, "up k={k} {hz} Hz: {got}");

                // Down: 48 kHz tone → wire rate.
                let mut down = Downsampler::new(k);
                let src = tone(hz, RATE, 48_000);
                let mut out = vec![0.0; 48_000 / k];
                let n = down.process(&src, &mut out);
                assert_eq!(n, 48_000 / k);
                let got = rms(&out[1000..]);
                assert!((got / 0.3536 - 1.0).abs() < 0.02, "down k={k} {hz} Hz: {got}");
            }
        }
    }

    #[test]
    fn down_then_up_is_clean() {
        // A 1 kHz tone through 12 kHz and back is still a clean 1 kHz tone.
        let src = tone(1000.0, RATE, 48_000);
        let mut down = Downsampler::new(4);
        let mut wire = vec![0.0; 12_000];
        down.process(&src, &mut wire);
        let mut up = Upsampler::new(4);
        let mut back = Vec::new();
        up.process(&wire, &mut back);
        // Both filters delay the signal; find by how much.
        let residual = |d: usize| {
            let e: f32 = back[4000..44_000]
                .iter()
                .zip(&src[4000 - d..44_000 - d])
                .map(|(a, b)| (a - b) * (a - b))
                .sum::<f32>()
                / 40_000.0;
            e.sqrt()
        };
        let best = (100..300).map(residual).fold(f32::MAX, f32::min);
        assert!(best < 0.01, "residual {best}");
    }

    /// Residual against an ideal tone at the output rate, best over delays
    /// (in output samples: a coarse search, then a fine one).
    fn residual_vs_ideal(out: &[f32], hz: f32, rate: f32) -> f32 {
        let at = |d: f64| {
            let e: f64 = out[500..out.len() - 500]
                .iter()
                .enumerate()
                .map(|(i, &y)| {
                    let t = (i + 500) as f64 - d;
                    let ideal = (std::f64::consts::TAU * hz as f64 * t / rate as f64).sin() * 0.5;
                    (y as f64 - ideal).powi(2)
                })
                .sum::<f64>()
                / (out.len() - 1000) as f64;
            e.sqrt()
        };
        let coarse = (0..100)
            .map(|i| i as f64 / 10.0)
            .min_by(|a, b| at(*a).total_cmp(&at(*b)))
            .unwrap();
        (-100..=100)
            .map(|i| at(coarse + i as f64 / 1000.0))
            .fold(f64::MAX, f64::min) as f32
    }

    #[test]
    fn fractional_44k1_both_ways_keeps_tones_clean() {
        for (from, to) in [(44_100u32, 48_000u32), (48_000, 44_100)] {
            for hz in [300.0, 1500.0, 2900.0] {
                let src = tone(hz, from as f32, from as usize);
                let mut r = FracResampler::new(from, to);
                let mut out = Vec::new();
                r.process(&src[..20_000], &mut out);
                r.process(&src[20_000..], &mut out);
                // One second in, one second (± a sample) out.
                assert!((out.len() as i64 - to as i64).abs() <= 2, "{from}->{to}: {}", out.len());
                let level = rms(&out[1000..out.len() - 1000]);
                assert!(
                    (level / 0.3536 - 1.0).abs() < 0.01,
                    "{from}->{to} {hz} Hz level {level}"
                );
                // Better than -50 dB against an ideal tone (0.5 peak).
                let res = residual_vs_ideal(&out, hz, to as f32);
                assert!(
                    res < 0.5 * std::f32::consts::FRAC_1_SQRT_2 * 0.0032,
                    "{from}->{to} {hz} Hz residual {res}"
                );
            }
        }
    }

    #[test]
    fn fractional_same_rate_is_a_copy() {
        let src = tone(1000.0, 48_000.0, 4800);
        let mut out = Vec::new();
        FracResampler::new(48_000, 48_000).process(&src, &mut out);
        assert_eq!(out, src);
    }

    #[test]
    fn images_and_aliases_are_suppressed() {
        // Nothing near 7.5 kHz survives the trip down to 12 kHz (it would
        // alias to 4.5 kHz).
        let mut down = Downsampler::new(4);
        let mut out = vec![0.0; 12_000];
        down.process(&tone(7_500.0, RATE, 48_000), &mut out);
        assert!(
            rms(&out[1000..]) < 0.5 * std::f32::consts::FRAC_1_SQRT_2 * 0.01,
            "{}",
            rms(&out[1000..])
        );
    }
}
