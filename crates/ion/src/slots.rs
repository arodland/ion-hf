//! Slice placement: the receiving side watches the three thirds of the
//! passband a 650 Hz slice burst can sit in and names the cleanest.
//!
//! [`SlotMeter`] compares each third against the whole passband in the same
//! FFT frame, so the radio's AGC, which moves all three together, cancels.
//! A third's score is its share of frames with a hit ([`HIT_DB`]), in per
//! cent, plus [`EXCESS_PCT_PER_DB`] for each dB its median sits above the
//! passband's, averaged over [`MEMORY_S`]. Frames under the link's own
//! bursts are left out ([`SlotMeter::mask`]).

use std::collections::VecDeque;

use ion_dsp::dsp::Complex;
use ion_dsp::fft::Fft;

/// The thirds, as offsets from the link's centre: 0 the centre (the
/// default), 1 below, 2 above.
pub const OFFSETS_HZ: [f32; 3] = [0.0, -800.0, 800.0];
pub const SLOTS: usize = 3;
/// Half the width a third is watched over: the slice's 650 Hz and a
/// little.
pub const HALF_HZ: f32 = 350.0;
/// A bin this far over the passband's median in the same frame is a
/// signal: white noise alone crosses it in about one frame in 30 000.
pub const HIT_DB: f32 = 13.0;
/// Score: per cent of hit frames that each dB of excess counts as.
pub const EXCESS_PCT_PER_DB: f32 = 1.0;
/// Time constant of the meter's averages, seconds: signals in the passband
/// come and go within tens of seconds.
pub const MEMORY_S: f32 = 8.0;
/// Score points a third must beat the current one by to take over.
pub const HYSTERESIS: f32 = 5.0;
/// Frames the meter needs before it names anything but the centre.
const MIN_FRAMES: f32 = 100.0;

const N: usize = 2048;
const HOP: usize = N / 2;

/// One frame's measurements, kept until it is old enough to be sure no
/// burst of ours covered it.
#[derive(Clone, Debug)]
struct Frame {
    /// Audio samples (from the first given to the meter) it spans.
    start: u64,
    end: u64,
    hit: [bool; SLOTS],
    excess_db: [f32; SLOTS],
    /// Thirds a burst of the link's covered.
    masked: [bool; SLOTS],
}

/// The passband watcher (48 kHz audio).
pub struct SlotMeter {
    rate: f32,
    center_hz: f32,
    fft: Fft,
    win: Vec<f32>,
    pending: Vec<f32>,
    /// Audio samples taken so far.
    taken: u64,
    /// Frames not yet averaged in, newest last.
    recent: VecDeque<Frame>,
    /// How long a frame waits before it counts, samples.
    lag: u64,
    /// Averages per third: hit share, excess (dB), and frames counted.
    hit: [f32; SLOTS],
    excess: [f32; SLOTS],
    count: [f32; SLOTS],
    current: usize,
    buf: Vec<Complex>,
    spec: Vec<Complex>,
}

impl SlotMeter {
    /// A meter for a link centred on `center_hz`; frames count once they
    /// are `lag_s` old (longer than the longest burst, so a burst decoded
    /// at its end can still mask the frames it covered).
    pub fn new(center_hz: f32, rate: f32, lag_s: f32) -> Self {
        let win = (0..N)
            .map(|i| 0.5 - 0.5 * (core::f32::consts::TAU * i as f32 / N as f32).cos())
            .collect();
        Self {
            rate,
            center_hz,
            fft: Fft::new(N),
            win,
            pending: Vec::new(),
            taken: 0,
            recent: VecDeque::new(),
            lag: (lag_s * rate) as u64,
            hit: [0.0; SLOTS],
            excess: [0.0; SLOTS],
            count: [0.0; SLOTS],
            current: 0,
            buf: vec![Complex::ZERO; N],
            spec: vec![Complex::ZERO; N],
        }
    }

    /// Audio samples taken so far (the clock [`Self::mask`] counts in).
    pub fn samples(&self) -> u64 {
        self.taken
    }

    /// The next stretch of the receiver's audio. Its frames count from
    /// the next [`Self::fold`] (or [`Self::best`]) on, once [`Self::mask`]
    /// has had the chance to leave out the link's bursts in it.
    pub fn push(&mut self, x: &[f32]) {
        self.pending.extend_from_slice(x);
        while self.pending.len() >= N {
            let start = self.taken;
            let f = self.frame(start);
            self.recent.push_back(f);
            self.pending.drain(..HOP);
            self.taken += HOP as u64;
        }
    }

    /// Leaves samples `start..end` out of third `slot` (`None`: all of
    /// them): one of the link's own bursts, there.
    pub fn mask(&mut self, start: u64, end: u64, slot: Option<usize>) {
        for f in self.recent.iter_mut().filter(|f| f.end > start && f.start < end) {
            match slot {
                Some(k) => f.masked[k] = true,
                None => f.masked = [true; SLOTS],
            }
        }
    }

    /// Each third's score: lower is cleaner.
    pub fn scores(&self) -> [f32; SLOTS] {
        std::array::from_fn(|k| 100.0 * self.hit[k] + EXCESS_PCT_PER_DB * self.excess[k].max(0.0))
    }

    /// The third to use: the current one unless another beats it by
    /// [`HYSTERESIS`]; the centre until there is enough to go on.
    pub fn best(&mut self) -> usize {
        self.fold();
        if self.count.iter().any(|&c| c < MIN_FRAMES) {
            return self.current;
        }
        let s = self.scores();
        let k = (0..SLOTS).min_by(|&a, &b| s[a].total_cmp(&s[b])).unwrap_or(0);
        if s[k] + HYSTERESIS < s[self.current] {
            self.current = k;
        }
        self.current
    }

    fn frame(&mut self, start: u64) -> Frame {
        for i in 0..N {
            self.buf[i] = Complex::new(self.pending[i] * self.win[i], 0.0);
        }
        self.fft.forward(&self.buf, &mut self.spec);
        let hz = self.rate / N as f32;
        let bin = |f: f32| (f / hz).round() as usize;
        let db = |c: Complex| 10.0 * (c.norm_sqr() + 1e-20).log10();
        let lo = bin(self.center_hz + OFFSETS_HZ[1] - HALF_HZ);
        let hi = bin(self.center_hz + OFFSETS_HZ[2] + HALF_HZ);
        let mut all: Vec<f32> = self.spec[lo..=hi].iter().map(|&c| db(c)).collect();
        all.sort_by(f32::total_cmp);
        let floor = all[all.len() / 2];
        let mut hit = [false; SLOTS];
        let mut excess_db = [0f32; SLOTS];
        for k in 0..SLOTS {
            let c = self.center_hz + OFFSETS_HZ[k];
            let mut v: Vec<f32> = self.spec[bin(c - HALF_HZ)..=bin(c + HALF_HZ)]
                .iter()
                .map(|&c| db(c))
                .collect();
            hit[k] = v.iter().any(|&d| d > floor + HIT_DB);
            v.sort_by(f32::total_cmp);
            excess_db[k] = v[v.len() / 2] - floor;
        }
        Frame {
            start,
            end: start + N as u64,
            hit,
            excess_db,
            masked: [false; SLOTS],
        }
    }

    /// Averages in the frames old enough to be sure of.
    pub fn fold(&mut self) {
        let a = HOP as f32 / (MEMORY_S * self.rate);
        while let Some(f) = self.recent.front() {
            if f.end + self.lag > self.taken {
                break;
            }
            let f = self.recent.pop_front().unwrap();
            for k in 0..SLOTS {
                if f.masked[k] {
                    continue;
                }
                // A running mean until the memory is full, then exponential.
                self.count[k] += 1.0;
                let w = (1.0 / self.count[k]).max(a);
                self.hit[k] += w * (f.hit[k] as u8 as f32 - self.hit[k]);
                self.excess[k] += w * (f.excess_db[k] - self.excess[k]);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ion_dsp::dsp::NoiseGen;

    fn noise(n: usize, seed: u64) -> Vec<f32> {
        let mut g = NoiseGen::new(seed);
        (0..n).map(|_| 0.05 * g.gaussian()).collect()
    }

    #[test]
    fn white_noise_stays_at_the_centre() {
        let mut m = SlotMeter::new(1500.0, 48_000.0, 0.0);
        m.push(&noise(48_000 * 20, 1));
        assert_eq!(m.best(), 0, "{:?}", m.scores());
        assert!(m.scores().iter().all(|&s| s < 1.0), "{:?}", m.scores());
    }

    #[test]
    fn a_keyed_carrier_moves_it() {
        // A carrier at 1750 Hz (in the centre third), on half the time.
        let mut x = noise(48_000 * 20, 2);
        for (i, v) in x.iter_mut().enumerate() {
            if (i / 24_000) % 2 == 0 {
                *v += 0.05 * (core::f32::consts::TAU * 1750.0 * i as f32 / 48_000.0).sin();
            }
        }
        let mut m = SlotMeter::new(1500.0, 48_000.0, 0.0);
        m.push(&x);
        m.fold();
        let s = m.scores();
        assert!(s[0] > 30.0 && s[1] < 1.0 && s[2] < 1.0, "{s:?}");
        assert_ne!(m.best(), 0);
    }

    #[test]
    fn our_own_bursts_are_left_out() {
        // A strong signal over the centre third, but it was one of ours.
        let mut x = noise(48_000 * 20, 3);
        for (i, v) in x.iter_mut().enumerate().skip(48_000).take(48_000 * 8) {
            *v += 0.2 * (core::f32::consts::TAU * 1500.0 * i as f32 / 48_000.0).sin();
        }
        // Masked when it is decoded, at its end; then 20 s more to count.
        let mut m = SlotMeter::new(1500.0, 48_000.0, 10.0);
        m.push(&x[..48_000 * 9]);
        m.mask(48_000, 48_000 * 9, Some(0));
        m.push(&x[48_000 * 9..]);
        m.push(&noise(48_000 * 20, 4));
        assert_eq!(m.best(), 0, "{:?}", m.scores());
        assert!(m.scores()[0] < 1.0, "{:?}", m.scores());
    }
}
