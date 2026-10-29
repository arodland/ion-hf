//! Iterative (turbo) data stage: given the decoder's opinion of the data
//! bits, every data cell becomes a pilot too, weighted by how sure that
//! opinion is (Valenti & Woerner 2001; Sandell et al. 1998), and for
//! DFT-spread bursts the equaliser cancels the interference between the
//! despread symbols (Tüchler & Singer 2011, frequency domain).

use std::fmt;

use ion_dsp::dsp::Complex;

use crate::phy::Constellation;
use crate::phy::est::{self, Estimator};
use crate::phy::frame::*;

use super::known;

/// Soft data cells whose expected value carries less than this share of
/// the constellation's mean energy are left out of the estimate: they say
/// little and would take a place in each estimate's pilot set.
const SOFT_MIN: f32 = 0.1;

/// Pilots per estimate in the refined passes: twice the first pass's, as
/// there are many more of them near each cell.
pub const REFINE_K: usize = 40;

/// What the data stage of one burst saw, kept with the burst so that it
/// can be decoded again with what the decoder learnt
/// ([`BurstGrid::refine`]) while the receiver goes on to the next.
pub struct BurstGrid {
    l: &'static Layout,
    hdr: Header,
    map: Map,
    ng: usize,
    /// The grid as the data stage used it (drift and phase corrected).
    grid: Vec<Complex>,
    /// Noise per carrier from the preamble, and per data cell as the
    /// data stage demapped it.
    sigma2: [f32; NC_MAX],
    cell_s2: Vec<f32>,
    /// Mean channel power from the pilots.
    p_h: f32,
}

impl fmt::Debug for BurstGrid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BurstGrid({} symbols)", self.ng)
    }
}

impl BurstGrid {
    pub(super) fn new(
        l: &'static Layout,
        hdr: Header,
        map: Map,
        grid: &[Complex],
        sigma2: [f32; NC_MAX],
        cell_s2: Vec<f32>,
        p_h: f32,
    ) -> Self {
        let ng = grid.len() / l.nc;
        Self {
            l,
            hdr,
            map,
            ng,
            grid: grid.to_vec(),
            sigma2,
            cell_s2,
            p_h,
        }
    }

    /// Data symbols of the burst.
    pub fn data_symbols(&self) -> usize {
        self.ng - self.l.pre_syms()
    }

    /// The burst's LLRs again, in [`RxBurst::llrs`](super::RxBurst::llrs)
    /// order, given `prior`, an LLR per data bit in the same order
    /// (decoded frames: their re-encoded bits, sure; the rest: the
    /// decoder's posteriors; missing entries count as 0). Each data cell's
    /// expected value and variance under the prior make it a pilot of
    /// noise variance `σ² + p_h·Var[x]`, and the estimate of a cell leaves
    /// its own sample out; DFT-spread bursts are equalised with the
    /// expected values of the other despread symbols taken out and an MMSE
    /// filter for what remains uncertain. `k` pilots per estimate (at most
    /// [`est::KMAX`]).
    pub fn refine(&self, prior: &[f32], k: usize) -> Vec<f32> {
        let l = self.l;
        let (nc, pre, ng) = (l.nc, l.pre_syms(), self.ng);
        let nsym = ng - pre;
        let cons = Constellation::from_code(self.hdr.constellation);
        let mode = self.hdr.mode;
        let gain = mode.gain();
        let g2 = gain * gain;
        let pts = cons.points();
        let mut pts_s = cons.points();
        for v in pts_s.pts.iter_mut() {
            *v = v.scale(gain);
        }
        let bps = pts.bps;
        let es = g2 * pts.pts[..pts.count()].iter().map(|p| p.norm_sqr()).sum::<f32>() / pts.count() as f32;
        let map = self.map;
        let (pat, fd) = match map.pilots {
            Pilots::Robust => (&l.robust, &l.fd),
            Pilots::Sparse => (&l.sparse, &l.fd_sparse),
        };
        let dtab = data_carrier_table(l, pat);
        let dc = l.data_carriers(map.pilots);
        let per = dtab.len();
        let ncell = nsym * dc;
        let nbits = ncell * bps;
        let a = interleave_step(nbits) as u64;

        // The prior per slot (scrambled and interleaved as sent), then the
        // expected value and variance of every data cell as sent (spread
        // symbols spread again, their variance shared evenly).
        let mut sl = vec![0f32; nbits];
        for i in 0..nbits {
            let v = prior.get(i).copied().unwrap_or(0.0);
            sl[((a * i as u64) % nbits as u64) as usize] = if scramble(SEED_DATA, i) == 1 { -v } else { v };
        }
        let mut xm = vec![Complex::ZERO; ncell];
        let mut xv = vec![0f32; ncell];
        let unit = if mode.spread { &pts } else { &pts_s };
        for (cell, (m, v)) in xm.iter_mut().zip(xv.iter_mut()).enumerate() {
            (*m, *v) = soft_point(&sl[cell * bps..(cell + 1) * bps], unit);
        }
        // The equaliser's prior: each despread symbol's expected value and
        // the mean variance over its OFDM symbol.
        let (em, ev) = if mode.spread {
            let em = xm.clone();
            let ev: Vec<f32> = xv.chunks_exact(dc).map(|v| v.iter().sum::<f32>() / dc as f32).collect();
            let mut sp = Spreader::new(dc);
            for (row, var) in xm.chunks_exact_mut(dc).zip(xv.chunks_exact_mut(dc)) {
                sp.forward(row);
                for v in row.iter_mut() {
                    *v = v.scale(gain);
                }
                let mv = g2 * var.iter().sum::<f32>() / dc as f32;
                var.fill(mv);
            }
            (em, ev)
        } else {
            (Vec::new(), Vec::new())
        };

        let p_h = self.p_h;
        let mut soft: Vec<Option<(Complex, f32)>> = vec![None; ng * nc];
        for g in pre..ng {
            for (j, &cc) in dtab[g % per].iter().enumerate() {
                let cell = (g - pre) * dc + j;
                let x = xm[cell];
                if x.norm_sqr() >= SOFT_MIN * es {
                    soft[g * nc + cc as usize] = Some((x, self.cell_s2[cell] + p_h * xv[cell]));
                }
            }
        }
        let sigma2 = self.sigma2;
        let mut est = Estimator::new(l, ng);
        est.load_with(&self.grid, ng, |c, g| {
            known(&map, c, g).map(|x| (x, sigma2[c])).or(soft[g * nc + c])
        });
        est.k = k.clamp(1, est::KMAX);
        est.select_model(ng, p_h, fd);
        let mut hbuf = vec![Complex::ZERO; ncell];
        let mut mbuf = vec![0f32; ncell];
        for g in pre..ng {
            for (j, &cc) in dtab[g % per].iter().enumerate() {
                let c = cc as usize;
                let cell = (g - pre) * dc + j;
                let own = if soft[g * nc + c].is_some() {
                    est.pilot_index(c, g).unwrap_or(usize::MAX)
                } else {
                    usize::MAX
                };
                (hbuf[cell], mbuf[cell]) = est.estimate_without(c, g, ng, p_h, own);
            }
        }

        let mut slot_llr = vec![0f32; nbits];
        let mut sp = Spreader::new(dc);
        let mut ya = [Complex::ZERO; NC_MAX];
        let mut va = [0f32; NC_MAX];
        let mut za = [Complex::ZERO; NC_MAX];
        for g in pre..ng {
            let base = (g - pre) * dc;
            for (j, &cc) in dtab[g % per].iter().enumerate() {
                let cell = base + j;
                let y = self.grid[g * nc + cc as usize];
                if mode.spread {
                    ya[j] = y;
                    va[j] = self.cell_s2[cell] + mbuf[cell] * g2;
                } else {
                    est::demap(
                        y,
                        hbuf[cell],
                        mbuf[cell],
                        self.cell_s2[cell],
                        &pts_s,
                        &mut slot_llr[cell * bps..cell * bps + bps],
                    );
                }
            }
            if mode.spread {
                let z = &mut za[..dc];
                let (mu, var) = sic(
                    &ya[..dc],
                    &hbuf[base..base + dc],
                    &va[..dc],
                    gain,
                    &em[base..base + dc],
                    ev[g - pre],
                    z,
                    &mut sp,
                );
                for (m, &zm) in z.iter().enumerate() {
                    let cell = base + m;
                    est::demap(
                        zm,
                        Complex::new(mu, 0.0),
                        0.0,
                        var,
                        &pts,
                        &mut slot_llr[cell * bps..cell * bps + bps],
                    );
                }
            }
        }
        (0..nbits)
            .map(|i| {
                let v = slot_llr[((a * i as u64) % nbits as u64) as usize];
                if scramble(SEED_DATA, i) == 1 { -v } else { v }
            })
            .collect()
    }
}

/// Soft interference cancellation and MMSE equalisation of one DFT-spread
/// symbol: cells `y`, channel `h` (times `gain`), noise plus estimation
/// variances `v`, expected despread symbols `xm` of mean variance `vx`.
/// Like [`est::fde`], returns `z = mu·x + e` per despread symbol with the
/// variance of `e`; `vx = 1` (nothing known) is the linear equaliser.
#[allow(clippy::too_many_arguments)]
fn sic(
    y: &[Complex],
    h: &[Complex],
    v: &[f32],
    gain: f32,
    xm: &[Complex],
    vx: f32,
    z: &mut [Complex],
    sp: &mut Spreader,
) -> (f32, f32) {
    let m = y.len() as f32;
    let mut xs = [Complex::ZERO; NC_MAX];
    let xs = &mut xs[..y.len()];
    xs.copy_from_slice(xm);
    sp.forward(xs);
    let (mut mu, mut p, mut q) = (0f32, 0f32, 0f32);
    for j in 0..y.len() {
        let g = h[j].scale(gain);
        let e = g.norm_sqr();
        let w = g.conj().scale(1.0 / (vx * e + v[j]).max(1e-30));
        z[j] = w * (y[j] - g * xs[j]);
        let wg = (w * g).re;
        mu += wg;
        p += wg * wg;
        q += w.norm_sqr() * v[j];
    }
    sp.inverse(z);
    let mu = (mu / m).max(1e-6);
    // Each symbol's own expected value goes back in: the filter removed
    // it with the others.
    for (zj, &x) in z.iter_mut().zip(xm) {
        *zj += x.scale(mu);
    }
    let var = (vx * (p / m - mu * mu) + q / m).max(1e-3 * mu * mu);
    (mu, var)
}

/// `ln(1 + e^x)` without overflow.
fn softplus(x: f32) -> f32 {
    if x > 0.0 {
        x + (-x).exp().ln_1p()
    } else {
        x.exp().ln_1p()
    }
}

/// Mean and variance of a point whose label bits (MSB first, as the
/// transmitter maps them) have LLRs `llr`.
fn soft_point(llr: &[f32], pts: &Points) -> (Complex, f32) {
    let bps = pts.bps;
    let (mut mu, mut e2) = (Complex::ZERO, 0f32);
    for (k, &x) in pts.pts[..pts.count()].iter().enumerate() {
        let mut lp = 0f32;
        for (b, &lam) in llr.iter().enumerate().take(bps) {
            let lam = lam.clamp(-30.0, 30.0);
            // ln P(bit) with P(0) = 1 / (1 + e^{−λ}).
            lp -= softplus(if (k >> (bps - 1 - b)) & 1 == 0 { -lam } else { lam });
        }
        let p = lp.exp();
        mu += x.scale(p);
        e2 += p * x.norm_sqr();
    }
    (mu, (e2 - mu.norm_sqr()).max(0.0))
}
