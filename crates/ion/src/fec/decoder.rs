//! Layered belief-propagation decoder for the lifted Ion codes.

use super::code::{MAX_DEG, QcCode};

/// Input LLRs are clipped to this magnitude (NaN becomes an erasure):
/// below the largest message a check can send (`phi(PHI_MIN)` ≈ 28.3), so
/// that no single channel bit outweighs a check.
const LLR_CLIP: f32 = 24.0;

/// Check-node rule.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Algorithm {
    /// Exact sum-product (tanh rule, evaluated as `phi(x) = -ln tanh(x/2)`).
    SumProduct,
    /// Normalized min-sum with the given factor.
    MinSum(f32),
}

/// Reusable layered LDPC decoder: base rows in order, each a layer of `Z`
/// independent checks, stopping as soon as the hard decisions satisfy
/// every active check. Holds the message buffers, so decoding allocates
/// nothing per iteration.
pub struct Decoder {
    post: Vec<f32>,
    msg: Vec<f32>,
    /// Channel LLRs of a soft-in soft-out decode.
    ch: Vec<f32>,
    /// A priori LLRs of the first information bits (see
    /// [`Decoder::set_prior`]).
    prior: Vec<f32>,
    active: Vec<usize>,
    max_iter: usize,
    algo: Algorithm,
    iters: usize,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `phi(x) = -ln tanh(x / 2) = ln(1 + 2 / (e^x - 1))`, its own inverse.
/// The lower clamp sets the largest message a check can send:
/// `phi(PHI_MIN)` ≈ 28.3, above [`LLR_CLIP`], so that the checks can
/// always outvote one confident wrong channel bit.
const PHI_MIN: f32 = 1e-12;

/// Clips an input LLR to [`LLR_CLIP`]; NaN becomes an erasure.
fn clip(l: f32) -> f32 {
    if l.is_nan() { 0.0 } else { l.clamp(-LLR_CLIP, LLR_CLIP) }
}

#[inline]
fn phi(x: f32) -> f32 {
    let x = x.clamp(PHI_MIN, 25.0);
    (2.0 / x.exp_m1()).ln_1p()
}

impl Decoder {
    /// Default iteration cap.
    pub const MAX_ITER: usize = 50;
    /// Default check-node rule.
    pub const ALGORITHM: Algorithm = Algorithm::SumProduct;

    pub fn new() -> Self {
        Self {
            post: Vec::new(),
            msg: Vec::new(),
            ch: Vec::new(),
            prior: Vec::new(),
            active: Vec::new(),
            max_iter: Self::MAX_ITER,
            algo: Self::ALGORITHM,
            iters: 0,
        }
    }

    /// Changes the iteration cap (at least 1).
    pub fn set_max_iterations(&mut self, n: usize) {
        self.max_iter = n.max(1);
    }

    /// Changes the check-node rule.
    pub fn set_algorithm(&mut self, algo: Algorithm) {
        self.algo = algo;
    }

    /// A priori LLRs (ln P(0)/P(1); 0 = nothing known) of the first
    /// `prior.len()` information bits, added to the channel's opinion of
    /// each in every later decode until changed. Empty clears them. Unlike
    /// channel LLRs, priors also reach the punctured information bits.
    pub fn set_prior(&mut self, prior: &[f32]) {
        self.prior.clear();
        self.prior.extend(prior.iter().map(|&l| clip(l)));
    }

    /// Iterations used by the last decode (the cap if it failed).
    pub fn iterations(&self) -> usize {
        self.iters
    }

    /// Decodes the first `llrs.len()` bits of the transmission order of
    /// `code`. Bits not received (later in the order, or punctured) are
    /// erasures. Returns the information bits once every active check is
    /// satisfied.
    pub(crate) fn run(&mut self, code: &QcCode, llrs: &[f32]) -> Option<Vec<u8>> {
        let n_tx = llrs.len().min(code.n_tx_max());
        self.post.clear();
        self.post.resize(code.n_full(), 0.0);
        let t0 = code.tx_start();
        for (o, &l) in self.post[t0..t0 + n_tx].iter_mut().zip(llrs) {
            *o = clip(l);
        }
        self.add_prior(code);
        self.clear_checks(code, n_tx);
        for it in 1..=self.max_iter {
            self.iters = it;
            if let Some(info) = self.iterate_and_check(code, n_tx) {
                return Some(info);
            }
        }
        None
    }

    fn add_prior(&mut self, code: &QcCode) {
        assert!(self.prior.len() <= code.k(), "prior: more LLRs than information bits");
        for (o, &p) in self.post.iter_mut().zip(&self.prior) {
            *o += p;
        }
    }

    fn clear_checks(&mut self, code: &QcCode, n_tx: usize) {
        self.msg.clear();
        self.msg.resize(code.adj.len(), 0.0);
        self.active.clear();
        self.active.extend((0..code.mb).map(|r| code.active(r, n_tx)));
        self.iters = 0;
    }

    /// One iteration; the information bits if every active check then holds.
    fn iterate_and_check(&mut self, code: &QcCode, n_tx: usize) -> Option<Vec<u8>> {
        match self.algo {
            Algorithm::SumProduct => self.iterate::<true>(code, 0.0),
            Algorithm::MinSum(a) => self.iterate::<false>(code, a),
        }
        let post = &self.post;
        // A posterior of exactly zero is an undecided bit (an all-erasure
        // input would otherwise "decode" to all zeros).
        let decided = !post[..code.tx_start() + n_tx].contains(&0.0) && code.checks_ok(n_tx, |v| (post[v] < 0.0) as u8);
        decided.then(|| post[..code.k()].iter().map(|&l| (l < 0.0) as u8).collect())
    }

    /// Starts soft-in soft-out decoding of the first `n_tx` bits of the
    /// transmission order of `code`: clears every check message. Follow
    /// with [`Self::soft_run`] as often as new channel LLRs arrive.
    pub(crate) fn soft_start(&mut self, code: &QcCode, n_tx: usize) {
        let n_tx = n_tx.min(code.n_tx_max());
        self.clear_checks(code, n_tx);
        self.ch.clear();
        self.ch.resize(n_tx, 0.0);
        self.post.clear();
        self.post.resize(code.n_full(), 0.0);
    }

    /// Replaces the channel LLRs (`n_tx` of them, as given to
    /// [`Self::soft_start`]) and runs up to `iters` more layered
    /// iterations, keeping the check messages of the earlier runs. Returns
    /// the information bits once every active check is satisfied.
    pub(crate) fn soft_run(&mut self, code: &QcCode, llrs: &[f32], iters: usize) -> Option<Vec<u8>> {
        let n_tx = self.ch.len();
        assert_eq!(llrs.len(), n_tx, "soft_run: expected the soft_start length");
        let t0 = code.tx_start();
        for (c, &l) in self.ch.iter_mut().zip(llrs) {
            *c = clip(l);
        }
        // Posterior = channel + every check message into the bit.
        self.post.iter_mut().for_each(|p| *p = 0.0);
        self.post[t0..t0 + n_tx].copy_from_slice(&self.ch);
        self.add_prior(code);
        for (&v, &m) in code.adj.iter().zip(&self.msg) {
            self.post[v as usize] += m;
        }
        for _ in 0..iters.max(1) {
            self.iters += 1;
            if let Some(info) = self.iterate_and_check(code, n_tx) {
                return Some(info);
            }
        }
        None
    }

    /// Extrinsic LLRs of the transmitted bits after the last
    /// [`Self::soft_run`]: what the code (and the prior) says about each
    /// bit beyond its own channel LLR (posterior minus channel).
    pub(crate) fn soft_extrinsic(&self, code: &QcCode) -> Vec<f32> {
        let t0 = code.tx_start();
        self.post[t0..t0 + self.ch.len()]
            .iter()
            .zip(&self.ch)
            .map(|(p, c)| p - c)
            .collect()
    }

    /// One layered iteration over every active check.
    fn iterate<const SPA: bool>(&mut self, code: &QcCode, alpha: f32) {
        let mut t = [0f32; MAX_DEG];
        let mut f = [0f32; MAX_DEG];
        for r in 0..code.mb {
            let d = code.deg[r];
            let off = code.row_off[r];
            for i in 0..self.active[r] {
                let base = off + i * d;
                let nb = &code.adj[base..base + d];
                let msg = &mut self.msg[base..base + d];
                let mut neg = false;
                for k in 0..d {
                    let tk = self.post[nb[k] as usize] - msg[k];
                    t[k] = tk;
                    neg ^= tk < 0.0;
                }
                if SPA {
                    // Exact zeros (erasures) are counted, not clamped: a
                    // check with two erased inputs must send exactly 0.
                    let (mut s, mut nz) = (0.0, 0);
                    for k in 0..d {
                        if t[k] == 0.0 {
                            f[k] = 0.0;
                            nz += 1;
                        } else {
                            f[k] = phi(t[k].abs());
                            s += f[k];
                        }
                    }
                    for k in 0..d {
                        let m = match nz {
                            0 => phi(s - f[k]),
                            1 if t[k] == 0.0 => phi(s),
                            _ => 0.0,
                        };
                        let new = if neg ^ (t[k] < 0.0) { -m } else { m };
                        msg[k] = new;
                        self.post[nb[k] as usize] = t[k] + new;
                    }
                } else {
                    let (mut m1, mut m2, mut pos) = (f32::MAX, f32::MAX, 0);
                    for (k, &tk) in t[..d].iter().enumerate() {
                        let a = tk.abs();
                        if a < m1 {
                            m2 = m1;
                            m1 = a;
                            pos = k;
                        } else if a < m2 {
                            m2 = a;
                        }
                    }
                    let (m1, m2) = (alpha * m1, alpha * m2);
                    for k in 0..d {
                        let m = if k == pos { m2 } else { m1 };
                        let new = if neg ^ (t[k] < 0.0) { -m } else { m };
                        msg[k] = new;
                        self.post[nb[k] as usize] = t[k] + new;
                    }
                }
            }
        }
    }
}
