//! Re-timing during a burst: where the paths are, from the cyclic prefix.
//!
//! The window is placed once, from preamble B. A path that is faded there
//! (MPD: two paths 4 ms apart, either can be 20 dB down for a few hundred
//! ms) is not seen, and when it comes up it can fall before the window or
//! past the CP. Each path repeats its CP `n` samples later, so
//! `y[t]·y*[t+n]`, averaged over symbols, is the sum over the paths of
//! their power times a box `[a − cp, a)` at their arrival `a`: its first
//! rise and last fall bound the earliest and latest path. The window is
//! stepped (like the drift steps, the grid kept as if it had not moved)
//! until every path arrives within `[w, w + cp]`.

use ion_dsp::dsp::Complex;

use super::PhyRx;

/// Forgetting factor per symbol of the CP correlation (~14 symbols).
const FORGET: f32 = 0.93;
/// Symbols averaged before the first decision.
const MIN_SYMS: usize = 10;
/// Largest step per symbol, samples.
const MAX_STEP: i32 = 4;

impl PhyRx {
    /// Adds the CP correlation of symbol `g` (its window as used) over
    /// offsets `−2cp..2cp` from the window start.
    pub(super) fn cp_track(&mut self, g: usize) {
        if !self.retiming {
            return;
        }
        let (n, cp) = (self.l.n as u64, self.l.cp as u64);
        let w = self.window_start(g);
        if w < 2 * cp || w - 2 * cp < self.oldest() {
            return;
        }
        let t0 = w - 2 * cp;
        for i in 0..self.cpm.len() {
            let t = t0 + i as u64;
            self.cpm[i] = self.cpm[i].scale(FORGET) + self.at(t) * self.at(t + n).conj();
        }
        self.cpm_n += 1;
    }

    /// Window step for the next symbol (0 for most), and the correlation
    /// moved along with it.
    pub(super) fn retime_step(&mut self) -> i32 {
        if !self.retiming || self.cpm_n < MIN_SYMS {
            return 0;
        }
        let cp = self.l.cp as i64;
        let len = self.cpm.len();
        // Magnitude, smoothed over 9 samples.
        let mut mag = vec![0f32; len];
        for (i, v) in mag.iter_mut().enumerate() {
            let (a, b) = (i.saturating_sub(4), (i + 5).min(len));
            *v = self.cpm[a..b].iter().fold(Complex::ZERO, |s, &x| s + x).norm() / (b - a) as f32;
        }
        let mut sorted = mag.clone();
        sorted.sort_by(|a, b| a.total_cmp(b));
        let base = sorted[len / 5];
        let top = sorted[len - 1];
        // Too little above the noise to act on.
        if top < 3.0 * base {
            return 0;
        }
        let thr = base + 0.5 * (top - base);
        // The run over the threshold around the peak: the paths' boxes
        // join up (an echo past the CP still overlaps its box with the
        // first path's), while noise elsewhere crosses it in short spikes.
        let peak = mag
            .iter()
            .enumerate()
            .fold(0, |b, (i, &v)| if v > mag[b] { i } else { b });
        let rise = (0..peak).rev().find(|&i| mag[i] < thr).map_or(0, |i| i + 1) as i64;
        let fall = (peak..len).find(|&i| mag[i] < thr).map_or(len - 1, |i| i - 1) as i64;
        // Arrivals relative to the window: earliest and latest path.
        let a1 = rise - 2 * cp + cp;
        let al = fall - 2 * cp + 1;
        let (lo, hi) = (al - cp + 2, a1 - 2);
        let target = if lo <= hi { 0.clamp(lo, hi) } else { (lo + hi) / 2 };
        if target.abs() < 4 {
            return 0;
        }
        let step = target.clamp(-MAX_STEP as i64, MAX_STEP as i64) as i32;
        self.stats.retime_steps += 1;
        // The correlation is kept relative to the window.
        let s = step as i64;
        let old = self.cpm.clone();
        for (i, m) in self.cpm.iter_mut().enumerate() {
            let j = i as i64 + s;
            *m = if (0..len as i64).contains(&j) {
                old[j as usize]
            } else {
                Complex::ZERO
            };
        }
        step
    }

    /// The grid is kept as if the window had stayed at the sync position,
    /// so the channel model (centred on that window) would see a path the
    /// re-timing went after outside its delay span. Moves the reference to
    /// the mean window of the burst.
    pub(super) fn rereference(&mut self) {
        let (ng, nc) = (self.ng, self.l.nc);
        let mean = self.rshift[..ng].iter().map(|&r| r as f64).sum::<f64>() / ng.max(1) as f64;
        let r = mean.round();
        if r == 0.0 {
            return;
        }
        for c in 0..nc {
            let a = core::f64::consts::TAU * self.l.bin(c) as f64 * r / self.l.n as f64;
            let rot = Complex::from_polar(1.0, a as f32);
            for g in 0..ng {
                self.grid[g * nc + c] = self.grid[g * nc + c] * rot;
            }
        }
    }
}
