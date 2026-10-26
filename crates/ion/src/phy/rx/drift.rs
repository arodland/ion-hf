//! Sample-clock drift between the two sound cards.

use ion_dsp::dsp::Complex;

use crate::phy::frame::{FS, NC_MAX};

use super::{PhyRx, known};

/// Sample-clock drift: the FFT window walks by `ε` samples per sample when
/// the two sound cards' clocks differ; over a long burst that runs past
/// the cyclic prefix's margin.
///
/// The grid is stored as if the FFT window had never moved, so a delay
/// `d(g) = ε·g·sym` turns carrier `k` by `e^{−j2πk·d/n}`. Pilot pairs on the
/// same carrier `L` symbols apart measure the phase slope across the band
/// `−2π f·ε·L·Ts`; a pooled least-squares fit over all pairs so far gives
/// `ε`. The window is stepped by whole samples as `ε·g·sym` grows (so no
/// path leaves the CP), and the remaining fraction is taken out as a phase
/// slope when the burst is finished.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Drift {
    sxx: f64,
    sxy: f64,
    syy: f64,
    n: f64,
}

/// Longest pilot-pair lag used for the drift estimate, s: fading
/// decorrelates longer lags.
const DRIFT_MAX_LAG_S: f64 = 0.45;
/// Clock offsets beyond this are not believed.
const DRIFT_MAX: f64 = 300e-6;
/// Spread of the clock offsets expected (the shrinkage prior).
const DRIFT_PRIOR: f64 = 100e-6;

impl Drift {
    fn add(&mut self, x: f64, y: f64) {
        self.sxx += x * x;
        self.sxy += x * y;
        self.syy += y * y;
        self.n += 1.0;
    }

    /// Least-squares `ε` and its variance.
    fn raw(&self) -> (f64, f64) {
        if self.n < 8.0 || self.sxx <= 0.0 {
            return (0.0, f64::INFINITY);
        }
        let e = self.sxy / self.sxx;
        let s2 = ((self.syy - e * self.sxy) / (self.n - 1.0)).max(0.0);
        (e, s2 / self.sxx)
    }

    /// `ε` to apply: the estimate shrunk towards 0 by its uncertainty (the
    /// pairs overlap, so the variance is inflated 4x) and by a prior of
    /// ±100 ppm, so a short or noisy burst is not pushed around by noise.
    fn eps(&self) -> f64 {
        let (e, var) = self.raw();
        if !var.is_finite() {
            return 0.0;
        }
        let v = 4.0 * var;
        let prior = DRIFT_PRIOR * DRIFT_PRIOR;
        let e = e * prior / (prior + v);
        (e * e * e / (e * e + 4.0 * v)).clamp(-DRIFT_MAX, DRIFT_MAX)
    }
}

impl PhyRx {
    fn drift_eps(&self) -> f64 {
        if self.track_drift { self.drift.eps() } else { 0.0 }
    }

    /// Window shift for symbol `g`: follows the drift estimate one sample
    /// at a time, with hysteresis.
    pub(super) fn next_shift(&self, g: usize) -> i32 {
        let prev = self.wshift[g - 1];
        let d = self.drift_eps() * (g * self.l.sym()) as f64;
        if d > prev as f64 + 0.7 {
            prev + 1
        } else if d < prev as f64 - 0.7 {
            prev - 1
        } else {
            prev
        }
    }

    /// Adds the pilot pairs ending in symbol `g` to the drift estimate.
    pub(super) fn drift_update(&mut self, g: usize) {
        let l = self.l;
        let nc = l.nc;
        let map = self.map;
        if g == 0 {
            return;
        }
        let ts = l.sym() as f64 / FS;
        let per = map.pattern(g).period;
        let eps = self.drift.raw().0;
        let sp = l.spacing_hz();
        let mut lag = per;
        let mut q = [Complex::ZERO; NC_MAX];
        let mut fq = [0f64; NC_MAX];
        while lag <= g && lag as f64 * ts <= DRIFT_MAX_LAG_S.max(per as f64 * ts) {
            let g0 = g - lag;
            let lt = lag as f64 * ts;
            let mut np = 0;
            for c in 0..nc {
                let (Some(x1), Some(x0)) = (known(&map, c, g), known(&map, c, g0)) else {
                    continue;
                };
                let h1 = self.grid[g * nc + c] * x1.conj();
                let h0 = self.grid[g0 * nc + c] * x0.conj();
                let f = l.bin(c) as f64 * sp;
                // Take out the slope already estimated, so what is left is
                // small.
                let a = (core::f64::consts::TAU * f * eps * lt) as f32;
                q[np] = h1 * h0.conj() * Complex::from_polar(1.0, a);
                fq[np] = f;
                np += 1;
            }
            if np >= 2 {
                let mut s = Complex::ZERO;
                let (mut sw, mut swf) = (0f64, 0f64);
                for k in 0..np {
                    s += q[k];
                    let w = q[k].norm() as f64;
                    sw += w;
                    swf += w * fq[k];
                }
                if sw > 0.0 && s.norm() > 0.0 {
                    let fbar = swf / sw;
                    let rot = Complex::from_polar(1.0, -s.arg());
                    for k in 0..np {
                        let v = q[k] * rot;
                        let x = -(v.norm() as f64) * core::f64::consts::TAU * (fq[k] - fbar) * lt;
                        let y = v.im as f64 + x * eps;
                        self.drift.add(x, y);
                    }
                }
            }
            lag *= 2;
        }
    }

    /// Takes the clock drift left after the window steps out of the grid
    /// (a phase slope across the carriers growing along the burst), with
    /// the estimate from the whole burst.
    pub(super) fn apply_drift(&mut self) -> f64 {
        let l = self.l;
        let nc = l.nc;
        let eps = self.drift_eps();
        if eps == 0.0 {
            return 0.0;
        }
        for g in 1..self.ng {
            let d = eps * (g * l.sym()) as f64;
            for c in 0..nc {
                let a = core::f64::consts::TAU * l.bin(c) as f64 * d / l.n as f64;
                self.grid[g * nc + c] = self.grid[g * nc + c] * Complex::from_polar(1.0, a as f32);
            }
        }
        eps
    }
}
