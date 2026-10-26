//! Per-burst statistical model of the tone energies (Rician signal,
//! noise with a fraction of interference) and the tone metrics from it.

use super::{EXT, Energies, Grid, Known, MfskRx, NE};
use crate::phy_mfsk::frame::Fmt;
use crate::phy_mfsk::util::{ln_i0, lse};

/// Per-burst model of the tone energies for one analysis window.
#[derive(Clone, Copy, Debug)]
pub(super) struct Params {
    /// Bins of the grid's energy arrays in use.
    ne: usize,
    /// Per-bin level of the clean noise (energy units of the window).
    pub(super) noise: Energies,
    /// Rician model of the energy at offsets −2..+2 from the tone bin, in
    /// noise units: `a` = 1 + diffuse power (≥ 1), `s` = specular power.
    /// Offset 0 is the tone itself; the others are leakage (window
    /// truncation, Doppler spread, residual frequency error).
    pub(super) a: [f32; 5],
    pub(super) s: [f32; 5],
    /// Noise model: a fraction `eps` of the empty bins carries interference
    /// (echoes of other symbols, clicks) with `r` times the noise level; the
    /// rest is clean noise. Keeps one hot bin from producing confident
    /// wrong LLRs.
    pub(super) eps: f32,
    pub(super) r: f32,
    /// Signal energy per tone over all five bins, noise units.
    pub(super) gtot: f32,
}

/// Per-bin log-likelihood terms of one symbol: `[j][e]` is the term for
/// bin `e` sitting at offset `j − 2` from a hypothesised tone.
type Terms = [[f32; NE]; 5];

impl Params {
    /// ln p(y | noise), clean-noise units.
    #[inline]
    fn ln_noise(&self, y: f32) -> f32 {
        if self.eps <= 0.0 {
            return -y;
        }
        lse((1.0 - self.eps).ln() - y, (self.eps / self.r).ln() - y / self.r)
    }

    /// ln p(y | signal model j) / p(y | noise).
    #[inline]
    fn term(&self, j: usize, y: f32) -> f32 {
        let (a, s) = (self.a[j], self.s[j]);
        if a <= 1.0 && s <= 0.0 {
            return 0.0;
        }
        let y = y.max(0.0);
        let sig = -a.ln() - (y + s) / a + ln_i0(2.0 * (y * s).sqrt() / a);
        sig - self.ln_noise(y)
    }

    pub(super) fn terms(&self, x: &Energies) -> Terms {
        let mut t = [[0f32; NE]; 5];
        for e in 0..self.ne {
            let y = x[e] / self.noise[e];
            for (j, tj) in t.iter_mut().enumerate() {
                tj[e] = self.term(j, y);
            }
        }
        t
    }

    /// [`Params::terms`] with the noise of each bin `k[e]` times the
    /// burst's (local noise, see [`local_noise`]): the bin is measured in
    /// the raised noise, and the signal is that much weaker against it.
    pub(super) fn terms_local(&self, x: &Energies, k: &Energies) -> Terms {
        let mut t = [[0f32; NE]; 5];
        for e in 0..self.ne {
            let ke = k[e];
            if ke == 1.0 {
                let y = x[e] / self.noise[e];
                for (j, tj) in t.iter_mut().enumerate() {
                    tj[e] = self.term(j, y);
                }
                continue;
            }
            let y = x[e] / (self.noise[e] * ke);
            for (j, tj) in t.iter_mut().enumerate() {
                let (a, s) = (1.0 + (self.a[j] - 1.0) / ke, self.s[j] / ke);
                tj[e] = if a <= 1.0 && s <= 0.0 {
                    0.0
                } else {
                    let yy = y.max(0.0);
                    -a.ln() - (yy + s) / a + ln_i0(2.0 * (yy * s).sqrt() / a) - self.ln_noise(yy)
                };
            }
        }
        t
    }

    /// Scales every signal power by `k` (R1 tones before their own
    /// statistics are known).
    pub(super) fn scaled(mut self, k: f32) -> Params {
        for j in 0..5 {
            self.a[j] = 1.0 + (self.a[j] - 1.0) * k;
            self.s[j] *= k;
        }
        self.gtot *= k;
        self
    }
}

/// Log-likelihood ratio (against noise only) of a tone at ext bin `e`.
#[inline]
pub(super) fn metric(t: &Terms, e: usize) -> f32 {
    let mut l = 0.0;
    for (j, tj) in t.iter().enumerate() {
        let i = e as i32 + j as i32 - 2;
        if (0..NE as i32).contains(&i) {
            l += tj[i as usize];
        }
    }
    l
}

/// The model for R0 header decoding, from the preamble alone.
pub(super) fn header_prior(pre: &[Energies]) -> Params {
    estimate(pre, &MfskRx::preamble_known(), NE)
}

/// The model for R1 header decoding: the preamble's signal model (window
/// `w0` on R0's grid) scaled to R1's window, and a flat noise level from
/// the median energy of the R1 header symbols (whose tones are not known
/// yet; one bin in 36 holds a tone, which the median barely notices).
pub(super) fn header_prior_r1(pre: &[Energies], hdr: &[Energies], w0: usize, g: &Grid) -> Params {
    let w1 = g
        .wins
        .iter()
        .position(|&w| w * 2 == w0)
        .map_or(g.wins[2], |i| g.wins[i]);
    let mut p = estimate(pre, &MfskRx::preamble_known(), NE).scaled(w1 as f32 / w0 as f32);
    let mut all: Vec<f32> = hdr.iter().flat_map(|x| x[..g.ne].iter().copied()).collect();
    let n0 = if all.is_empty() {
        1.0
    } else {
        let mid = all.len() / 2;
        *all.select_nth_unstable_by(mid, |a, b| a.total_cmp(b)).1 / core::f32::consts::LN_2
    };
    p.ne = g.ne;
    p.noise = [n0.max(1e-30); NE];
    p
}

/// Data symbols each side in a bin's local noise level (R0: ±0.34 s).
const LOCAL_SYMS: usize = 12;

/// Noise as it changes through an MFSK burst, the floor's counterpart of
/// the OFDM receiver's local noise. The burst's model takes its noise per
/// bin from the preamble and header; band noise moves (a signal keying on
/// in a few bins, a crash over a symbol). For every data symbol and bin,
/// the growth over that level, at least 1: the larger of the symbol's own level (median over
/// its bins) and the bin's level over ±[`LOCAL_SYMS`] symbols, both in
/// burst-noise units and both leaving out the bins around each symbol's
/// likeliest tone. Each estimate is lowered by about two of its standard
/// errors, so that white noise leaves it at 1.
pub(super) fn local_noise(e: &[Energies], p: &Params, fmt: Fmt, hsyms: usize) -> Vec<Energies> {
    let ne = p.ne;
    let n = e.len();
    // Mean of an empty bin under the burst's model (clean noise plus its
    // share of interference).
    let m0 = 1.0 + p.eps * (p.r - 1.0);
    // Energies in burst-noise units, NaN near the decided tones.
    let z: Vec<Energies> = e
        .iter()
        .enumerate()
        .map(|(i, x)| {
            let mut zi = [f32::NAN; NE];
            for (j, v) in zi.iter_mut().enumerate().take(ne) {
                *v = x[j] / p.noise[j];
            }
            let h = fmt.hop(hsyms + i);
            for t in 0..fmt.tones {
                let mut best = (f32::MIN, 0usize);
                for v in 0..fmt.vals() {
                    let b = fmt.tone_bin(h, t, v) + EXT;
                    if x[b] / p.noise[b] > best.0 {
                        best = (x[b] / p.noise[b], b);
                    }
                }
                zi[best.1.saturating_sub(2)..(best.1 + 3).min(ne)].fill(f32::NAN);
            }
            zi
        })
        .collect();
    // The symbol's level: median of its clean bins (an exponential's median
    // is ln 2 of its mean; standard error about 1.5/√m of the mean).
    let sym: Vec<f32> = z
        .iter()
        .map(|zi| {
            let mut v: Vec<f32> = zi[..ne].iter().copied().filter(|v| !v.is_nan()).collect();
            if v.len() < 8 {
                return 1.0;
            }
            let m = v.len();
            let med = *v.select_nth_unstable_by(m / 2, |a, b| a.total_cmp(b)).1;
            (med / core::f32::consts::LN_2 - 3.0 / (m as f32).sqrt()).max(1.0)
        })
        .collect();
    (0..n)
        .map(|i| {
            let lo = i.saturating_sub(LOCAL_SYMS);
            let hi = (i + LOCAL_SYMS + 1).min(n);
            let mut k = [1f32; NE];
            for (b, kb) in k.iter_mut().enumerate().take(ne) {
                let (mut s, mut c) = (0f32, 0usize);
                for zi in &z[lo..hi] {
                    if !zi[b].is_nan() {
                        s += zi[b];
                        c += 1;
                    }
                }
                let bin = if c >= 6 {
                    s / c as f32 / m0 - 2.0 / (c as f32).sqrt()
                } else {
                    1.0
                };
                *kb = bin.max(sym[i]).max(1.0);
            }
            k
        })
        .collect()
}

/// Model estimate from known symbols: per-bin noise from the bins away from
/// the known tones, side-bin energies and Rician moments of the tone bin.
/// `ne`: bins of the grid's energy arrays in use.
pub(super) fn estimate(e: &[Energies], known: &[Known], ne: usize) -> Params {
    let mut nsum = [0f64; NE];
    let mut ncnt = [0usize; NE];
    for (x, k) in e.iter().zip(known) {
        for (j, v) in x[..ne].iter().enumerate() {
            if (0..k.n).all(|t| j.abs_diff(k.e[t]) > 2) {
                nsum[j] += *v as f64;
                ncnt[j] += 1;
            }
        }
    }
    let gsum: f64 = nsum.iter().sum();
    let gcnt: usize = ncnt.iter().sum();
    let global = (gsum / gcnt.max(1) as f64).max(1e-30);
    let mut noise = [1f32; NE];
    for (j, n) in noise.iter_mut().enumerate().take(ne) {
        let lo = j.saturating_sub(4);
        let hi = (j + 4).min(ne - 1);
        let (s, c) = (lo..=hi).fold((0f64, 0usize), |a, i| (a.0 + nsum[i], a.1 + ncnt[i]));
        *n = if c >= 8 { (s / c as f64) as f32 } else { global as f32 };
    }
    // Cap the window SNR of a tone at 30 dB: on a near-clean channel the
    // "noise" is numerical leakage, and a model fitted to it would be
    // absurdly sharp.
    let (mut msum, mut mcnt) = (0f64, 0usize);
    for (x, k) in e.iter().zip(known) {
        for t in 0..k.n {
            msum += x[k.e[t]] as f64;
            mcnt += 1;
        }
    }
    let floor = (1e-3 * msum / mcnt.max(1) as f64) as f32;
    for n in &mut noise[..ne] {
        *n = n.max(floor);
    }
    // Split the empty-bin energies into clean noise and interference: the
    // median fixes the clean level, the first two moments the rest.
    let mut z: Vec<f32> = Vec::with_capacity(gcnt);
    for (x, k) in e.iter().zip(known) {
        for (j, v) in x[..ne].iter().enumerate() {
            if (0..k.n).all(|t| j.abs_diff(k.e[t]) > 2) {
                z.push(v / noise[j]);
            }
        }
    }
    let (mut eps, mut r) = (0f32, 1f32);
    if z.len() >= 32 {
        let mut zs = z.clone();
        let mid = zs.len() / 2;
        let med = *zs.select_nth_unstable_by(mid, |a, b| a.total_cmp(b)).1;
        let mu0 = (med / core::f32::consts::LN_2).clamp(0.5, 1.0);
        let n = z.len() as f32;
        let m = z.iter().map(|v| v / mu0).sum::<f32>() / n - 1.0;
        let q = z.iter().map(|v| (v / mu0) * (v / mu0)).sum::<f32>() / n / 2.0 - 1.0;
        if m > 0.01 {
            let rr = q / m - 1.0;
            if rr > 1.5 {
                r = rr.min(1000.0);
                eps = (m / (r - 1.0)).min(0.5);
            }
        }
        for v in &mut noise[..ne] {
            *v *= mu0;
        }
    }
    // Energies at offsets −2..+2 of every known tone of the mode.
    let mut ys: [Vec<f32>; 5] = Default::default();
    for (x, k) in e.iter().zip(known) {
        for t in 0..k.n {
            let c = k.e[t] as i32;

            for (j, y) in ys.iter_mut().enumerate() {
                let i = c + j as i32 - 2;
                // Skip bins outside the grid and the other tones' own bins.
                if (0..ne as i32).contains(&i) && (j == 2 || (0..k.n).all(|u| k.e[u] as i32 != i)) {
                    y.push(x[i as usize] / noise[i as usize]);
                }
            }
        }
    }
    let mut a = [1f32; 5];
    let mut s = [0f32; 5];
    let mut gtot = 0.0;
    for j in 0..5 {
        let y = &ys[j];
        let n = y.len().max(1) as f32;
        let m1 = y.iter().sum::<f32>() / n;
        let var = y.iter().map(|v| (v - m1) * (v - m1)).sum::<f32>() / n;
        // Moments of a Rician energy: mean A + S, variance A² + 2AS.
        let g = m1 - 1.0;
        // Leakage bins need clear evidence (two standard errors of noise).
        let floor = if j == 2 { 0.05 } else { 2.0 / n.sqrt() };
        if g < floor {
            if j == 2 {
                a[j] = 1.05;
            }
            continue;
        }
        gtot += g;
        // The sample variance of a few dozen faded (exponential) energies
        // is often well below the truth, which would fit a sharp Rician
        // model to a Rayleigh channel and make deep fades look impossible.
        // Use an upper confidence value instead (exponential kurtosis 9:
        // relative standard error √(8/n)).
        let var = var * (1.0 + 1.5 * (8.0 / n).sqrt());
        let aj = (m1 - (m1 * m1 - var).max(0.0).sqrt()).clamp(1.0, m1);
        a[j] = aj;
        s[j] = (m1 - aj).max(0.0);
    }
    Params {
        ne,
        noise,
        a,
        s,
        eps,
        r,
        gtot,
    }
}

/// Mean mutual information (bits per tone) the model gives the known
/// header tones.
pub(super) fn mutual_info(e: &[Energies], known: &[Known], p: &Params) -> f32 {
    let mut sum = 0.0;
    let mut n = 0;
    for (x, k) in e.iter().zip(known) {
        let tm = p.terms(x);
        for t in 0..k.n {
            let mut norm = f32::NEG_INFINITY;
            let mut truth = 0.0;
            for v in 0..k.vals {
                let l = metric(&tm, k.fmt.tone_bin(k.hop, t, v) + EXT);
                norm = lse(norm, l);
                if v == k.v[t] as usize {
                    truth = l;
                }
            }
            sum += (k.vals as f32).log2() + (truth - norm) / core::f32::consts::LN_2;
            n += 1;
        }
    }
    sum / n.max(1) as f32
}
