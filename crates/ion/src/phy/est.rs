//! Pilot-aided channel estimation and exact soft demapping.
//!
//! The channel at a cell is a linear MMSE (Wiener) interpolation of the
//! K nearest pilots' least-squares samples (Hoeher/Kaiser/Robertson 1997).
//! The correlation model is the "robust" one of Li/Cimini/Sollenberger
//! 1998: a uniform delay profile of width `tm` (centred on 0 by a fixed
//! per-carrier rotation in the receiver) and a uniform Doppler spectrum of
//! ±`fd`; both give real sinc correlations, so the weights are real.
//! Instead of one worst-case design, each burst picks `(fd, tm)` from a
//! small grid by leave-one-out cross-validation on its own pilots, and the
//! predicted error variance is scaled up if the cross-validation residual
//! says the model is optimistic.

use ion_dsp::dsp::Complex;

use super::frame::{FS, Layout, Points, Spreader};

/// Most pilots one estimate may combine ([`Layout::est_k`] picks how many).
pub const KMAX: usize = 40;
/// Largest LLR magnitude handed out.
pub const LLR_MAX: f32 = 40.0;

/// The model used until a burst has been cross-validated: the largest
/// Doppler and delay span of the numerology's grid ([`Layout::fd`] ×
/// [`Layout::tm`]).
fn default_model(l: &Layout) -> (f64, f64) {
    (l.fd[2], l.tm[2])
}

fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-9 {
        1.0
    } else {
        let p = core::f64::consts::PI * x;
        p.sin() / p
    }
}

#[derive(Clone, Copy, Default)]
pub struct Pilot {
    pub c: u8,
    pub g: u16,
    /// Least-squares channel sample `Y/X`.
    pub h: Complex,
    /// Its noise variance `σ²_c/|X|²`.
    pub var: f32,
}

pub struct Estimator {
    l: &'static Layout,
    rf: Vec<f64>,
    rt: Vec<f64>,
    lc: f32,
    lg: f32,
    /// Error-variance scale from cross-validation (>= 1).
    pub mse_scale: f32,
    pub model: (f64, f64),
    pub pilots: Vec<Pilot>,
    start: Vec<usize>,
    cand: Vec<(f32, u32)>,
    /// Workspace of one estimate: the model covariance of the chosen
    /// pilots and its Cholesky factor (only the leading `k × k` is used).
    a: Box<[[f64; KMAX]; KMAX]>,
    chol: Box<[[f64; KMAX]; KMAX]>,
}

impl Estimator {
    pub fn new(l: &'static Layout, max_g: usize) -> Self {
        let mut e = Self {
            l,
            rf: vec![0.0; 2 * l.nc + 1],
            rt: vec![0.0; 4 * l.gw + 1],
            lc: 5.0,
            lg: 8.0,
            mse_scale: 1.0,
            model: default_model(l),
            pilots: Vec::with_capacity(max_g * (l.robust.pilots(l.nc).max(l.sparse.pilots(l.nc)) + 1) + l.nc),
            start: Vec::with_capacity(max_g + 1),
            cand: Vec::with_capacity((2 * l.gw + 1) * (l.nc + 2)),
            a: Box::new([[0.0; KMAX]; KMAX]),
            chol: Box::new([[0.0; KMAX]; KMAX]),
        };
        assert!(l.est_k <= KMAX);
        let (fd, tm) = default_model(l);
        e.set_model(fd, tm);
        e
    }

    /// Doppler `fd` (Hz, one-sided) and delay span `tm` (samples).
    pub fn set_model(&mut self, fd: f64, tm: f64) {
        self.model = (fd, tm);
        let (n, nc, gw) = (self.l.n, self.l.nc, self.l.gw);
        for (i, r) in self.rf.iter_mut().enumerate() {
            *r = sinc(tm / n as f64 * (i as f64 - nc as f64));
        }
        let ts = self.l.sym() as f64 / FS;
        for (i, r) in self.rt.iter_mut().enumerate() {
            *r = sinc(2.0 * fd * ts * (i as f64 - 2.0 * gw as f64));
        }
        // Neighbourhood scales: about half the first sinc zero. `est_lg`
        // shrinks the time scale so a narrow numerology's pilots come from
        // every pilot carrier rather than the two nearest over a long
        // stretch of time.
        self.lc = (0.5 * n as f64 / tm) as f32;
        self.lg = (0.5 / (2.0 * fd * ts)).min(4.0 * gw as f64) as f32 * self.l.est_lg;
    }

    /// Collects the pilots of grid symbols `0..ng`. `value(c, g)` gives the
    /// known cell value or `None` for data cells.
    pub fn load(
        &mut self,
        grid: &[Complex],
        ng: usize,
        sigma2: &[f32],
        value: impl Fn(usize, usize) -> Option<Complex>,
    ) {
        let nc = self.l.nc;
        self.pilots.clear();
        self.start.clear();
        for g in 0..ng {
            self.start.push(self.pilots.len());
            for c in 0..nc {
                if let Some(x) = value(c, g) {
                    let e = x.norm_sqr();
                    let y = grid[g * nc + c];
                    self.pilots.push(Pilot {
                        c: c as u8,
                        g: g as u16,
                        h: (y * x.conj()).scale(1.0 / e),
                        var: sigma2[c] / e,
                    });
                }
            }
        }
        self.start.push(self.pilots.len());
    }

    /// Mean channel power from the pilots, noise removed.
    pub fn channel_power(&self) -> f32 {
        let n = self.pilots.len().max(1) as f32;
        let raw: f32 = self.pilots.iter().map(|p| p.h.norm_sqr()).sum::<f32>() / n;
        let var: f32 = self.pilots.iter().map(|p| p.var).sum::<f32>() / n;
        (raw - var).max(0.05 * raw).max(1e-30)
    }

    /// Picks the model with the smallest leave-one-out prediction error on
    /// the pilots of symbols `0..ng`, and sets `mse_scale`. `fd` is the
    /// Doppler grid (it depends on the pilot pattern's time spacing).
    pub fn select_model(&mut self, ng: usize, p_h: f32, fd_grid: &[f64; 3]) {
        let np = self.pilots.len();
        let stride = (np / 160).max(1);
        let mut best = (f64::INFINITY, default_model(self.l), 1.0f32);
        let l = self.l;
        for &fd in fd_grid {
            for &tm in &l.tm {
                self.set_model(fd, tm);
                let (mut r, mut v, mut m) = (0f64, 0f64, 0f64);
                let mut i = 0;
                while i < np {
                    let p = self.pilots[i];
                    let (h, mse) = self.estimate_excl(p.c as usize, p.g as usize, ng, p_h, i);
                    r += (h - p.h).norm_sqr() as f64;
                    v += p.var as f64;
                    m += mse as f64;
                    i += stride;
                }
                if r < best.0 {
                    let scale = ((r - v) / m.max(1e-30)).clamp(1.0, 10.0) as f32;
                    best = (r, (fd, tm), scale);
                }
            }
        }
        self.set_model(best.1.0, best.1.1);
        self.mse_scale = best.2;
    }

    /// MMSE estimate of the channel at (`c`, `g`) from pilots of symbols
    /// `0..ng`, and its error variance.
    pub fn estimate(&mut self, c: usize, g: usize, ng: usize, p_h: f32) -> (Complex, f32) {
        let (h, mse) = self.estimate_excl(c, g, ng, p_h, usize::MAX);
        (h, mse * self.mse_scale)
    }

    // Small dense linear algebra reads best with explicit indices.
    #[allow(clippy::needless_range_loop)]
    fn estimate_excl(&mut self, c: usize, g: usize, ng: usize, p_h: f32, exclude: usize) -> (Complex, f32) {
        self.cand.clear();
        let (gw, cw) = (self.l.gw, self.l.cw);
        let g_lo = g.saturating_sub(gw);
        let g_hi = (g + gw + 1).min(ng);
        for gg in g_lo..g_hi {
            let dg = (gg as f32 - g as f32) / self.lg;
            for i in self.start[gg]..self.start[gg + 1] {
                let p = &self.pilots[i];
                let dc = p.c as i32 - c as i32;
                if dc.unsigned_abs() as usize > cw || i == exclude {
                    continue;
                }
                let d = (dc as f32 / self.lc).powi(2) + dg * dg;
                self.cand.push((d, i as u32));
            }
        }
        let k = self.cand.len().min(self.l.est_k);
        if k == 0 {
            return (Complex::ZERO, p_h);
        }
        if self.cand.len() > k {
            self.cand.select_nth_unstable_by(k - 1, |a, b| a.0.total_cmp(&b.0));
        }
        // Disjoint borrows: the workspace matrices are written while the
        // model and the pilots are read.
        let (rf_v, rt_v) = (&self.rf, &self.rt);
        let (nc, gw2) = (self.l.nc as i32, 2 * self.l.gw as i32);
        let rf = |d: i32| rf_v[(d + nc) as usize];
        let rt = |d: i32| rt_v[(d + gw2) as usize];
        let (pilots, cand) = (&self.pilots, &self.cand);
        let a = &mut *self.a;
        let chol = &mut *self.chol;
        let mut rhs = [0f64; KMAX];
        let ph = p_h as f64;
        for i in 0..k {
            let pi = pilots[cand[i].1 as usize];
            rhs[i] = ph * rf(c as i32 - pi.c as i32) * rt(g as i32 - pi.g as i32);
            for j in 0..=i {
                let pj = pilots[cand[j].1 as usize];
                let v = ph * rf(pi.c as i32 - pj.c as i32) * rt(pi.g as i32 - pj.g as i32);
                a[i][j] = v;
                a[j][i] = v;
            }
            a[i][i] += pi.var as f64;
        }
        for i in 0..k {
            chol[i][..k].copy_from_slice(&a[i][..k]);
        }
        let mut w = solve_spd(chol, &rhs, k);
        // Unit-gain constraint: the plain Wiener weights shrink the
        // estimate towards zero (they sum to less than one), which is right
        // on average over Rayleigh fading but biases a steady channel and
        // mis-scales 16QAM. Rescale to unit sum and use the exact error
        // variance of those weights under the model:
        // MSE = p_h - 2 wᵀr + wᵀ(R + diag(var)) w.
        let s: f64 = w[..k].iter().sum();
        if s > 0.2 {
            for v in &mut w[..k] {
                *v /= s;
            }
        }
        let mut h = Complex::ZERO;
        let mut mse = ph;
        for i in 0..k {
            let p = pilots[cand[i].1 as usize];
            h += p.h.scale(w[i] as f32);
            mse -= 2.0 * w[i] * rhs[i];
            for j in 0..k {
                mse += w[i] * a[i][j] * w[j];
            }
        }
        (h, (mse.max(0.0)) as f32)
    }
}

/// Solves `A w = b` for symmetric positive definite `A` (Cholesky).
#[allow(clippy::needless_range_loop)]
fn solve_spd(a: &mut [[f64; KMAX]; KMAX], b: &[f64; KMAX], k: usize) -> [f64; KMAX] {
    for j in 0..k {
        let mut d = a[j][j];
        for p in 0..j {
            d -= a[j][p] * a[j][p];
        }
        let d = d.max(1e-12 * a[j][j].abs().max(1e-300)).sqrt();
        a[j][j] = d;
        for i in j + 1..k {
            let mut s = a[i][j];
            for p in 0..j {
                s -= a[i][p] * a[j][p];
            }
            a[i][j] = s / d;
        }
    }
    let mut y = [0f64; KMAX];
    for i in 0..k {
        let mut s = b[i];
        for p in 0..i {
            s -= a[i][p] * y[p];
        }
        y[i] = s / a[i][i];
    }
    let mut w = [0f64; KMAX];
    for i in (0..k).rev() {
        let mut s = y[i];
        for p in i + 1..k {
            s -= a[p][i] * w[p];
        }
        w[i] = s / a[i][i];
    }
    w
}

/// Exact bit LLRs (`ln P(0)/P(1)`) of one cell `y = h·x + n`, where the
/// channel estimate `h` has error variance `mse` and the noise has
/// variance `sigma2`: each point gets variance `sigma2 + mse·|x|²`.
pub fn demap(y: Complex, h: Complex, mse: f32, sigma2: f32, pts: &Points, out: &mut [f32]) {
    let m = pts.count();
    let mut d = [0f32; 64];
    for (k, dk) in d.iter_mut().enumerate().take(m) {
        let x = pts.pts[k];
        let v = (sigma2 + mse * x.norm_sqr()).max(1e-30);
        *dk = -(y - h * x).norm_sqr() / v - v.ln();
    }
    for (b, o) in out.iter_mut().enumerate().take(pts.bps) {
        let shift = pts.bps - 1 - b;
        let (mut m0, mut m1) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
        for (k, &dk) in d.iter().enumerate().take(m) {
            if (k >> shift) & 1 == 0 {
                m0 = m0.max(dk);
            } else {
                m1 = m1.max(dk);
            }
        }
        let (mut s0, mut s1) = (0f32, 0f32);
        for (k, &dk) in d.iter().enumerate().take(m) {
            if (k >> shift) & 1 == 0 {
                s0 += (dk - m0).exp();
            } else {
                s1 += (dk - m1).exp();
            }
        }
        let l = (m0 + s0.ln()) - (m1 + s1.ln());
        *o = l.clamp(-LLR_MAX, LLR_MAX);
    }
}

/// Nearest constellation point to `y/h` (for decision-directed noise).
pub fn decide(y: Complex, h: Complex, pts: &Points) -> Complex {
    let mut best = f32::INFINITY;
    let mut bx = Complex::ZERO;
    for &x in &pts.pts[..pts.count()] {
        let e = (y - h * x).norm_sqr();
        if e < best {
            best = e;
            bx = x;
        }
    }
    bx
}

/// Frequency-domain MMSE equalisation of one DFT-spread symbol: cells `y`
/// with channel estimates `h` (data gain `gain` on top) and noise plus
/// estimation-error variances `v`; the despread symbols go to `z`.
/// Returns the mean gain `mu` of the despread symbols and the variance of
/// what is left (noise and residual interference), for unit-energy
/// points: `z = mu·x + e`.
pub fn fde(y: &[Complex], h: &[Complex], v: &[f32], gain: f32, z: &mut [Complex], sp: &mut Spreader) -> (f32, f32) {
    let m = y.len() as f32;
    let (mut mu, mut p) = (0f32, 0f32);
    for j in 0..y.len() {
        let g = h[j].scale(gain);
        let e = g.norm_sqr();
        let w = g.conj().scale(1.0 / (e + v[j]).max(1e-30));
        z[j] = w * y[j];
        let wg = (w * g).re;
        mu += wg;
        p += wg * wg + w.norm_sqr() * v[j];
    }
    sp.inverse(z);
    let mu = (mu / m).max(1e-6);
    let var = (p / m - mu * mu).max(1e-3 * mu * mu);
    (mu, var)
}
