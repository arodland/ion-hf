//! Non-binary LDPC over GF(32) for the MFSK floor: one code symbol per
//! 32-ary tone, decoded on the tones' own likelihoods (no BICM loss).
//!
//! GF(2⁵) with primitive polynomial x⁵ + x² + 1 (addition is XOR). The
//! parity-check matrix is built by progressive edge growth (Hu,
//! Eleftheriou, Arnold, IEEE Trans. Inf. Theory 51(1), 2005) with
//! pseudo-random coefficients and encoded through its reduced row-echelon
//! form. Decoding is flooding belief propagation with the check-node
//! convolution in the Walsh–Hadamard domain (Davey, MacKay, IEEE Commun.
//! Lett. 2(6), 1998; Barnault, Declercq, ITW 2003).

use std::sync::OnceLock;

use super::Rate;

/// Field size.
pub const Q: usize = 32;
/// Code symbols (tones) per codeword.
pub const N: usize = 205;
/// Information symbols per codeword: 256 bits + 4 zero bits.
pub const K: usize = 52;
/// Information bits per codeword.
pub const K_BITS: usize = 256;
/// Iteration cap.
pub const MAX_ITER: usize = 100;

/// GF(32) log/antilog tables.
struct Gf {
    exp: [u8; 62],
    log: [u8; Q],
}

impl Gf {
    const fn new() -> Self {
        let mut exp = [0u8; 62];
        let mut log = [0u8; Q];
        let mut x = 1u8;
        let mut i = 0;
        while i < 31 {
            exp[i] = x;
            exp[i + 31] = x;
            log[x as usize] = i as u8;
            x <<= 1;
            if x & 0x20 != 0 {
                x ^= 0x25;
            }
            i += 1;
        }
        Self { exp, log }
    }
    #[inline]
    fn mul(&self, a: u8, b: u8) -> u8 {
        if a == 0 || b == 0 {
            0
        } else {
            self.exp[self.log[a as usize] as usize + self.log[b as usize] as usize]
        }
    }
    #[inline]
    fn inv(&self, a: u8) -> u8 {
        debug_assert!(a != 0);
        self.exp[(31 - self.log[a as usize] as usize) % 31]
    }
}

static GF: Gf = Gf::new();

/// SplitMix64.
fn mix(s: &mut u64) -> u64 {
    *s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *s;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// A GF(32) LDPC code and its encoder.
pub struct NbCode {
    n: usize,
    m: usize,
    /// Edges grouped by check: (variable, coefficient).
    edges: Vec<(u16, u8)>,
    /// Start of each check's edges in `edges` (m + 1 entries).
    row: Vec<usize>,
    /// Edge indices of each variable.
    col: Vec<Vec<usize>>,
    /// Information positions (all non-pivot columns; the ones past the
    /// first `k_used` are known zeros) and parity positions.
    info: Vec<usize>,
    parity: Vec<usize>,
    /// parity[i] = Σ_j a[i][j] · info[j], m_rank × info.len().
    a: Vec<Vec<u8>>,
}

impl NbCode {
    /// Builds a code of `n` symbols and `m` checks whose variables have the
    /// degrees `var_deg` (one per variable), by progressive edge growth from
    /// `seed`.
    pub fn new(n: usize, m: usize, var_deg: &[usize], seed: u64) -> Self {
        assert_eq!(var_deg.len(), n);
        let mut s = seed;
        let mut chk: Vec<Vec<usize>> = vec![Vec::new(); m];
        let mut var: Vec<Vec<usize>> = vec![Vec::new(); n];
        // Lowest degrees first, as PEG prescribes.
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by_key(|&v| (var_deg[v], v));
        for &v in &order {
            for k in 0..var_deg[v] {
                let c = if k == 0 {
                    pick_min(
                        &chk,
                        &(0..m).filter(|c| !var[v].contains(c)).collect::<Vec<_>>(),
                        &mut s,
                    )
                } else {
                    // Breadth-first from v: the checks at the greatest depth
                    // (or not reached at all), least loaded first.
                    let mut seen_c = vec![false; m];
                    let mut seen_v = vec![false; n];
                    let mut front: Vec<usize> = vec![v];
                    seen_v[v] = true;
                    let mut last: Vec<usize> = Vec::new();
                    loop {
                        let mut nc = Vec::new();
                        for &u in &front {
                            for &c in &var[u] {
                                if !seen_c[c] {
                                    seen_c[c] = true;
                                    nc.push(c);
                                }
                            }
                        }
                        let unreached: Vec<usize> = (0..m).filter(|&c| !seen_c[c]).collect();
                        if !unreached.is_empty() && nc.is_empty() {
                            break pick_min(&chk, &unreached, &mut s);
                        }
                        if unreached.is_empty() {
                            // Everything reachable: the newest layer.
                            let cand: Vec<usize> = if nc.is_empty() { last.clone() } else { nc.clone() };
                            let cand: Vec<usize> = cand.into_iter().filter(|c| !var[v].contains(c)).collect();
                            let cand = if cand.is_empty() {
                                (0..m).filter(|c| !var[v].contains(c)).collect()
                            } else {
                                cand
                            };
                            break pick_min(&chk, &cand, &mut s);
                        }
                        last = nc.clone();
                        let mut nv = Vec::new();
                        for &c in &nc {
                            for &u in &chk[c] {
                                if !seen_v[u] {
                                    seen_v[u] = true;
                                    nv.push(u);
                                }
                            }
                        }
                        if nv.is_empty() {
                            break pick_min(&chk, &unreached, &mut s);
                        }
                        front = nv;
                    }
                };
                chk[c].push(v);
                var[v].push(c);
            }
        }
        let mut edges = Vec::new();
        let mut row = vec![0];
        let mut col = vec![Vec::new(); n];
        for vs in &chk {
            for &v in vs {
                let h = 1 + (mix(&mut s) % 31) as u8;
                col[v].push(edges.len());
                edges.push((v as u16, h));
            }
            row.push(edges.len());
        }
        let mut code = Self {
            n,
            m,
            edges,
            row,
            col,
            info: Vec::new(),
            parity: Vec::new(),
            a: Vec::new(),
        };
        // Search pivots from the last column back, so the information
        // symbols tend to come first.
        let order: Vec<usize> = (0..n).rev().collect();
        code.build_encoder(&order);
        code
    }

    /// Gauss–Jordan elimination of H, pivots searched in the columns'
    /// `order`: pivot columns become the parity symbols, the rest
    /// information (in position order).
    fn build_encoder(&mut self, order: &[usize]) {
        let (n, m) = (self.n, self.m);
        let mut h = vec![vec![0u8; n]; m];
        for (c, hc) in h.iter_mut().enumerate() {
            for &(v, x) in &self.edges[self.row[c]..self.row[c + 1]] {
                hc[v as usize] ^= x;
            }
        }
        let mut pivots = Vec::new();
        let mut r = 0;
        for &col in order {
            if r == m {
                break;
            }
            let Some(p) = (r..m).find(|&i| h[i][col] != 0) else {
                continue;
            };
            h.swap(r, p);
            let iv = GF.inv(h[r][col]);
            for x in h[r].iter_mut() {
                *x = GF.mul(*x, iv);
            }
            let pivot = h[r].clone();
            for (i, hi) in h.iter_mut().enumerate() {
                if i != r && hi[col] != 0 {
                    let f = hi[col];
                    for (x, &p) in hi.iter_mut().zip(&pivot) {
                        *x ^= GF.mul(f, p);
                    }
                }
            }
            pivots.push(col);
            r += 1;
        }
        let info: Vec<usize> = (0..n).filter(|c| !pivots.contains(c)).collect();
        // Row i of the reduced matrix: x[pivot_i] + Σ_j h[i][info_j] x[info_j] = 0.
        self.a = (0..r).map(|i| info.iter().map(|&c| h[i][c]).collect()).collect();
        self.parity = pivots;
        self.info = info;
    }

    /// Information symbols the code can carry (at least [`K`] for the
    /// link's code; any beyond the caller's are sent as zeros).
    pub fn k(&self) -> usize {
        self.info.len()
    }

    pub fn n(&self) -> usize {
        self.n
    }

    /// Code position of each information symbol, in information order.
    pub fn info_positions(&self) -> &[usize] {
        &self.info
    }

    /// Number of checks variable `v` takes part in.
    pub fn degree(&self, v: usize) -> usize {
        self.col[v].len()
    }

    /// The check of edge `e`.
    fn check_of(&self, e: usize) -> usize {
        self.row.partition_point(|&r| r <= e) - 1
    }

    /// Greedy peeling over the Tanner graph with nothing known: while some
    /// check has exactly one unknown symbol, that symbol follows from it;
    /// when none has, the unknown symbol that would leave the most checks
    /// with one unknown (ties: `prefer` highest) is picked as given. The
    /// picks, in order. Exactly `n − m` picks (with every check used) would
    /// make an information set from which belief propagation recovers
    /// every other symbol one check at a time.
    pub fn peel_picks(&self, prefer: impl Fn(usize) -> i64) -> Vec<usize> {
        let mut known = vec![false; self.n];
        let mut open: Vec<usize> = (0..self.m).map(|c| self.row[c + 1] - self.row[c]).collect();
        let mut used = vec![false; self.m];
        let mut picks = Vec::new();
        let mut left = self.n;
        let learn = |v: usize, known: &mut Vec<bool>, open: &mut Vec<usize>| {
            known[v] = true;
            for &e in &self.col[v] {
                open[self.check_of(e)] -= 1;
            }
        };
        while left > 0 {
            if let Some(c) = (0..self.m).find(|&c| !used[c] && open[c] == 1) {
                used[c] = true;
                let v = self.edges[self.row[c]..self.row[c + 1]]
                    .iter()
                    .map(|&(v, _)| v as usize)
                    .find(|&v| !known[v])
                    .unwrap();
                learn(v, &mut known, &mut open);
                left -= 1;
                continue;
            }
            let v = (0..self.n)
                .filter(|&v| !known[v])
                .max_by_key(|&v| {
                    let freed = self.col[v].iter().filter(|&&e| {
                        let c = self.check_of(e);
                        !used[c] && open[c] == 2
                    });
                    (freed.count(), prefer(v))
                })
                .unwrap();
            picks.push(v);
            learn(v, &mut known, &mut open);
            left -= 1;
        }
        picks
    }

    /// The same code with its information positions chosen by peeling
    /// ([`NbCode::peel_picks`]): every symbol not picked is a parity symbol,
    /// and of the picks the latest ones become parity too until the rank is
    /// full. The codewords do not change, only which positions carry the
    /// information.
    pub fn with_peeled_info(mut self, prefer: impl Fn(usize) -> i64) -> Self {
        let picks = self.peel_picks(prefer);
        let mut order: Vec<usize> = (0..self.n).filter(|v| !picks.contains(v)).collect();
        order.extend(picks.iter().rev());
        self.build_encoder(&order);
        self
    }

    /// Encodes `u` (at most `k()` symbols, the rest zero) into `n` symbols.
    pub fn encode(&self, u: &[u8]) -> Vec<u8> {
        assert!(u.len() <= self.info.len());
        let mut x = vec![0u8; self.n];
        for (j, &p) in self.info.iter().enumerate() {
            x[p] = u.get(j).copied().unwrap_or(0) & 31;
        }
        for (i, &p) in self.parity.iter().enumerate() {
            let mut acc = 0u8;
            for (j, &q) in self.info.iter().enumerate() {
                acc ^= GF.mul(self.a[i][j], x[q]);
            }
            x[p] = acc;
        }
        x
    }

    /// The information symbols of codeword `x`.
    pub fn info_of(&self, x: &[u8]) -> Vec<u8> {
        self.info.iter().map(|&p| x[p]).collect()
    }

    /// True when `x` satisfies every check.
    pub fn checks_ok(&self, x: &[u8]) -> bool {
        (0..self.m).all(|c| {
            self.edges[self.row[c]..self.row[c + 1]]
                .iter()
                .fold(0u8, |a, &(v, h)| a ^ GF.mul(h, x[v as usize]))
                == 0
        })
    }

    /// Belief-propagation decoding. `metrics` holds `n × 32` channel
    /// log-likelihoods (symbol-major, any per-symbol offset); `known` the
    /// number of leading information symbols the caller sends (the rest of
    /// the information positions are known zeros). Returns the codeword
    /// once every check is satisfied, within `max_iter` iterations.
    pub fn decode(&self, metrics: &[f32], known: usize, max_iter: usize) -> Option<Vec<u8>> {
        self.decode_soft(metrics, known, max_iter).ok()
    }

    /// [`NbCode::decode`], or, when it fails, the last posteriors (`n × 32`
    /// probabilities, symbol-major).
    pub fn decode_soft(&self, metrics: &[f32], known: usize, max_iter: usize) -> Result<Vec<u8>, Vec<f32>> {
        assert_eq!(metrics.len(), self.n * Q);
        let e = self.edges.len();
        // Channel probabilities.
        let mut ch = vec![0f32; self.n * Q];
        for v in 0..self.n {
            let m = &metrics[v * Q..(v + 1) * Q];
            let mx = m.iter().fold(f32::MIN, |a, &b| a.max(b));
            let p = &mut ch[v * Q..(v + 1) * Q];
            for (o, &l) in p.iter_mut().zip(m) {
                *o = (l - mx).max(-60.0).exp();
            }
            normalize(p);
        }
        for &p in &self.info[known.min(self.info.len())..] {
            let c = &mut ch[p * Q..(p + 1) * Q];
            c.fill(0.0);
            c[0] = 1.0;
        }
        let mut v2c = vec![0f32; e * Q];
        let mut c2v = vec![1.0 / Q as f32; e * Q];
        for v in 0..self.n {
            for &ei in &self.col[v] {
                v2c[ei * Q..(ei + 1) * Q].copy_from_slice(&ch[v * Q..(v + 1) * Q]);
            }
        }
        let mut hard = vec![0u8; self.n];
        let mut posts = vec![0f32; self.n * Q];
        let mut f = vec![[0f32; Q]; 32];
        for _ in 0..max_iter {
            // Check nodes.
            for c in 0..self.m {
                let (lo, hi) = (self.row[c], self.row[c + 1]);
                let d = hi - lo;
                for (i, &(_, h)) in self.edges[lo..hi].iter().enumerate() {
                    let src = &v2c[(lo + i) * Q..(lo + i + 1) * Q];
                    let t = &mut f[i];
                    for (x, &pv) in src.iter().enumerate() {
                        t[GF.mul(h, x as u8) as usize] = pv;
                    }
                    wht(t);
                }
                // Products of all the others (prefix/suffix).
                let mut pre = [1f32; Q];
                let mut outs = vec![[0f32; Q]; d];
                for i in 0..d {
                    outs[i] = pre;
                    for y in 0..Q {
                        pre[y] *= f[i][y];
                    }
                }
                let mut suf = [1f32; Q];
                for i in (0..d).rev() {
                    for y in 0..Q {
                        outs[i][y] *= suf[y];
                        suf[y] *= f[i][y];
                    }
                }
                for (i, &(_, h)) in self.edges[lo..hi].iter().enumerate() {
                    let mut r = outs[i];
                    wht(&mut r);
                    let dst = &mut c2v[(lo + i) * Q..(lo + i + 1) * Q];
                    for (x, o) in dst.iter_mut().enumerate() {
                        *o = r[GF.mul(h, x as u8) as usize].max(1e-30);
                    }
                    normalize(dst);
                }
            }
            // Variable nodes and decisions.
            for v in 0..self.n {
                let es = &self.col[v];
                let mut post = [0f32; Q];
                post.copy_from_slice(&ch[v * Q..(v + 1) * Q]);
                for &ei in es {
                    for (p, &m) in post.iter_mut().zip(&c2v[ei * Q..(ei + 1) * Q]) {
                        *p *= m;
                    }
                }
                normalize(&mut post);
                hard[v] = (0..Q).fold(0, |b, x| if post[x] > post[b] { x } else { b }) as u8;
                posts[v * Q..(v + 1) * Q].copy_from_slice(&post);
                for &ei in es {
                    let dst = &mut v2c[ei * Q..(ei + 1) * Q];
                    dst.copy_from_slice(&ch[v * Q..(v + 1) * Q]);
                    for &ej in es {
                        if ej != ei {
                            for (p, &m) in dst.iter_mut().zip(&c2v[ej * Q..(ej + 1) * Q]) {
                                *p *= m;
                            }
                        }
                    }
                    normalize(dst);
                }
            }
            if self.checks_ok(&hard) {
                return Ok(hard);
            }
        }
        Err(posts)
    }

    /// The codeword of information `u` with symbol `slot` set to 1 and the
    /// rest 0 (a generator row).
    fn unit_codeword(&self, slot: usize) -> Vec<u8> {
        let mut u = vec![0u8; self.info.len()];
        u[slot] = 1;
        self.encode(&u)
    }

    /// List decoding around belief propagation's last posteriors `post`:
    /// the information symbols at their likeliest values, the `l` least
    /// sure of them also at their second likeliest, every combination
    /// re-encoded and scored by the log-likelihoods `metrics`; the best
    /// codeword. For a priori decoding, where belief propagation cannot
    /// carry the known symbols' certainty around the parity cycles (see
    /// [`InfoSet::Echelon`]) but most information symbols are right.
    pub fn list_decode(&self, metrics: &[f32], post: &[f32], l: usize) -> Vec<u8> {
        let k = self.info.len();
        let mut u = vec![0u8; k];
        let mut alt: Vec<(f32, usize, u8)> = Vec::with_capacity(k);
        for (s, &p) in self.info.iter().enumerate() {
            let pv = &post[p * Q..(p + 1) * Q];
            let mut b = [0usize, 1];
            if pv[1] > pv[0] {
                b = [1, 0];
            }
            for x in 2..Q {
                if pv[x] > pv[b[0]] {
                    b = [x, b[0]];
                } else if pv[x] > pv[b[1]] {
                    b[1] = x;
                }
            }
            u[s] = b[0] as u8;
            let margin = (pv[b[0]].max(1e-30) / pv[b[1]].max(1e-30)).ln();
            alt.push((margin, s, (b[0] ^ b[1]) as u8));
        }
        alt.sort_by(|a, b| a.0.total_cmp(&b.0));
        alt.truncate(l);
        let mut x = self.encode(&u);
        let score = |x: &[u8]| -> f32 { x.iter().enumerate().map(|(v, &c)| metrics[v * Q + c as usize]).sum() };
        let gens: Vec<Vec<u8>> = alt
            .iter()
            .map(|&(_, s, d)| self.unit_codeword(s).iter().map(|&g| GF.mul(g, d)).collect())
            .collect();
        let mut cur = score(&x);
        let (mut best, mut best_x) = (cur, x.clone());
        // Gray code: step i flips alternative `trailing_zeros(i)`.
        for i in 1u32..(1 << alt.len()) {
            let g = &gens[i.trailing_zeros() as usize];
            for (v, &d) in g.iter().enumerate() {
                if d != 0 {
                    let old = x[v];
                    x[v] ^= d;
                    cur += metrics[v * Q + x[v] as usize] - metrics[v * Q + old as usize];
                }
            }
            if cur > best {
                best = cur;
                best_x.copy_from_slice(&x);
            }
        }
        best_x
    }
}

/// Normalises a probability vector (a uniform one if it vanished).
#[inline]
fn normalize(p: &mut [f32]) {
    let s: f32 = p.iter().sum();
    if s > 1e-30 && s.is_finite() {
        let k = 1.0 / s;
        for x in p.iter_mut() {
            *x *= k;
        }
    } else {
        p.fill(1.0 / p.len() as f32);
    }
}

/// In-place Walsh–Hadamard transform of size 32 (unnormalised; applying
/// it twice multiplies by 32, which `normalize` absorbs).
#[inline]
fn wht(a: &mut [f32; Q]) {
    let mut h = 1;
    while h < Q {
        for i in (0..Q).step_by(2 * h) {
            for j in i..i + h {
                let (x, y) = (a[j], a[j + h]);
                a[j] = x + y;
                a[j + h] = x - y;
            }
        }
        h *= 2;
    }
}

fn pick_min(chk: &[Vec<usize>], cand: &[usize], s: &mut u64) -> usize {
    let lo = cand.iter().map(|&c| chk[c].len()).min().unwrap();
    let best: Vec<usize> = cand.iter().copied().filter(|&c| chk[c].len() == lo).collect();
    best[(mix(s) % best.len() as u64) as usize]
}

/// Variable degrees of the link's code: `n` symbols, `e` edges spread as
/// evenly as possible (degrees `e / n` and one more).
pub fn even_degrees(n: usize, e: usize) -> Vec<usize> {
    (0..n).map(|v| e / n + usize::from(v < e % n)).collect()
}

/// Variable degrees of the link's code: 2 everywhere but 20 columns of
/// degree 6.
pub fn link_degrees() -> Vec<usize> {
    (0..N).map(|v| if v < 20 { 6 } else { 2 }).collect()
}

/// Code rates with a GF(32) code: 1/4 (the link's short code, every
/// control frame and the MFSK floor) and the faster MFSK levels' 1/3, 1/2
/// and 2/3. All carry the same [`K`] information symbols.
pub const RATES: [Rate; 4] = [Rate::R1_4, Rate::R1_3, Rate::R1_2, Rate::R2_3];

/// Code symbols (tones) per codeword at `rate`: [`N`] = 205 at 1/4, then
/// 156, 104 and 78.
pub fn n_for(rate: Rate) -> usize {
    match rate {
        Rate::R1_4 => N,
        Rate::R1_3 => 3 * K,
        Rate::R1_2 => 2 * K,
        Rate::R2_3 => 3 * K / 2,
        r => panic!("nb: no GF(32) code at rate {r:?}"),
    }
}

/// Variable degrees of the code at `rate` (1/4: [`link_degrees`]): degree
/// 2 everywhere but a few heavy columns.
pub fn degrees_for(rate: Rate) -> Vec<usize> {
    let n = n_for(rate);
    let (heavy, d) = match rate {
        Rate::R1_4 => return link_degrees(),
        Rate::R1_3 => (16, 5),
        Rate::R1_2 => (12, 4),
        _ => (12, 3),
    };
    (0..n).map(|v| if v < heavy { d } else { 2 }).collect()
}

/// The link's code (see `docs/fec.md`, GF(32) short code), with
/// the link's information set.
pub fn code() -> &'static NbCode {
    code_with(link_layout().info)
}

/// The link's code with information set `info` (the same codewords).
pub fn code_with(info: InfoSet) -> &'static NbCode {
    code_at(Rate::R1_4, info)
}

/// The code at `rate` with information set `info`.
pub fn code_at(rate: Rate, info: InfoSet) -> &'static NbCode {
    static C: [[OnceLock<NbCode>; 2]; 4] = [const { [const { OnceLock::new() }; 2] }; 4];
    let r = RATES
        .iter()
        .position(|&x| x == rate)
        .unwrap_or_else(|| panic!("nb: no GF(32) code at rate {rate:?}"));
    C[r][info as usize].get_or_init(|| {
        let d = degrees_for(rate);
        let n = n_for(rate);
        let c = NbCode::new(n, n - K, &d, 0x4E42_4C44_5043);
        match info {
            InfoSet::Echelon => c,
            InfoSet::Peeled => c.with_peeled_info(|v| d[v] as i64),
        }
    })
}

/// Which code positions carry the information symbols (the codewords are
/// the same either way). Part of the wire format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InfoSet {
    /// The non-pivot columns of H's row-echelon form, pivots searched from
    /// the last column back: positions 0..55 but three, the 20 degree-6
    /// columns among them. The parity columns (all degree 2) then form
    /// closed cycles through the checks, so belief propagation cannot pass
    /// certainty about the information on to them.
    Echelon,
    /// Chosen by peeling ([`NbCode::peel_picks`]): from the information
    /// symbols, all but one check in turn has a single unknown symbol, so
    /// certain information makes almost every parity symbol certain.
    Peeled,
}

/// Information bits with the four padding bits of the last symbol.
const PADDED_BITS: usize = 5 * K;

/// The check-in frame's bits that a priori decoding cannot know
/// (`link::apriori`: the SNR, capabilities and software-version bytes 12,
/// 13 and 15 and the CRC-32, bytes 28..32), then the caller's packed call
/// (bytes 7..12), known only from a roster. Every other bit of a check-in
/// is fixed by its format. The layouts that place bits by this list put
/// these where the code protects them best.
pub const CHECKIN_VARIABLE_BITS: [std::ops::Range<usize>; 5] = [96..104, 104..112, 120..128, 224..256, 56..96];

/// How a frame's 256 bits become the code's 52 information symbols: a
/// permutation of the bits (with the 4 padding bits) before they are
/// packed five to a symbol, MSB first, and the information slot each
/// packed symbol takes. Information slots differ in strength: the link's
/// code gives 20 of them degree 6 in the Tanner graph, the rest degree 2.
/// Blind decoding does not care which; a priori decoding, which pins the
/// known symbols and leaves the rest to the code, does (`docs/fec.md`,
/// "A priori bits and the layout"). Part of the wire format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitMap {
    /// Bits in order, symbol j in slot j: with the echelon information set
    /// the check-in's fixed header and call land on the degree-6 slots, its
    /// variable fields whole on degree-2 ones.
    Plain,
    /// The check-in's variable symbols ([`CHECKIN_VARIABLE_BITS`], in that
    /// order) on the degree-6 slots, bits in order.
    Heavy,
    /// Bits spread over all symbols (`i → 53·i mod 260`), slots in order:
    /// a run of variable bits leaves most symbols partly known (the link's,
    /// [`LINK_LAYOUT`]).
    Spread,
    /// The check-in's variable bits dealt round-robin over the 20 degree-6
    /// slots (two or three to each), the fixed bits fill the rest.
    SpreadHeavy,
}

impl BitMap {
    pub const ALL: [BitMap; 4] = [BitMap::Plain, BitMap::Heavy, BitMap::Spread, BitMap::SpreadHeavy];

    pub fn name(self) -> &'static str {
        match self {
            BitMap::Plain => "plain",
            BitMap::Heavy => "heavy",
            BitMap::Spread => "spread",
            BitMap::SpreadHeavy => "spreadheavy",
        }
    }
}

/// A frame's way into the GF(32) code: the information set and the bit
/// map. Names: `echelon-plain`, `peeled-heavy`, ...
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layout {
    pub info: InfoSet,
    pub map: BitMap,
}

impl Layout {
    pub const fn new(info: InfoSet, map: BitMap) -> Self {
        Self { info, map }
    }

    /// Every combination.
    pub fn all() -> Vec<Layout> {
        [InfoSet::Echelon, InfoSet::Peeled]
            .into_iter()
            .flat_map(|i| BitMap::ALL.map(|m| Layout::new(i, m)))
            .collect()
    }

    pub fn name(self) -> String {
        let i = match self.info {
            InfoSet::Echelon => "echelon",
            InfoSet::Peeled => "peeled",
        };
        format!("{i}-{}", self.map.name())
    }

    pub fn from_name(s: &str) -> Option<Layout> {
        Layout::all().into_iter().find(|l| l.name() == s)
    }

    fn code(self, rate: Rate) -> &'static NbCode {
        code_at(rate, self.info)
    }

    fn tables(self, rate: Rate) -> &'static Tables {
        static T: OnceLock<Vec<Vec<Tables>>> = OnceLock::new();
        let t = T.get_or_init(|| {
            RATES
                .iter()
                .map(|&r| {
                    Layout::all()
                        .into_iter()
                        .map(|l| Tables::new(l.map, l.code(r)))
                        .collect()
                })
                .collect()
        });
        let r = RATES
            .iter()
            .position(|&x| x == rate)
            .unwrap_or_else(|| panic!("nb: no GF(32) code at rate {rate:?}"));
        &t[r][self.info as usize * BitMap::ALL.len() + self.map as usize]
    }
}

/// The layout the link sends and receives with (`ION_NB_LAYOUT`, a
/// [`Layout::name`], overrides it for A/B tables only: both ends must
/// agree).
pub fn link_layout() -> Layout {
    static L: OnceLock<Layout> = OnceLock::new();
    *L.get_or_init(|| {
        std::env::var("ION_NB_LAYOUT")
            .ok()
            .and_then(|v| Layout::from_name(&v))
            .unwrap_or(LINK_LAYOUT)
    })
}

/// The default layout: the echelon information set with the bits spread
/// (`docs/fec.md`, "A priori bits and the layout").
pub const LINK_LAYOUT: Layout = Layout::new(InfoSet::Echelon, BitMap::Spread);

struct Tables {
    /// Packed position of each padded information bit.
    pos: [u16; PADDED_BITS],
    /// Padded bit at each packed position.
    bit: [u16; PADDED_BITS],
    /// Information slot of each packed symbol.
    slot: [u8; K],
}

impl Tables {
    fn new(map: BitMap, code: &NbCode) -> Self {
        // Slots, strongest (highest degree) first, ties in slot order.
        let mut strong: Vec<usize> = (0..K).collect();
        strong.sort_by_key(|&j| std::cmp::Reverse(code.degree(code.info[j])));
        let heavy: Vec<usize> = strong
            .iter()
            .copied()
            .filter(|&j| code.degree(code.info[j]) > 2)
            .collect();
        let variable: Vec<usize> = CHECKIN_VARIABLE_BITS.iter().flat_map(|r| r.clone()).collect();
        let mut pos = [0u16; PADDED_BITS];
        let mut slot = [0u8; K];
        for (j, s) in slot.iter_mut().enumerate() {
            *s = j as u8;
        }
        match map {
            BitMap::Plain => {
                for (i, p) in pos.iter_mut().enumerate() {
                    *p = i as u16;
                }
            }
            BitMap::Heavy => {
                for (i, p) in pos.iter_mut().enumerate() {
                    *p = i as u16;
                }
                // Packed symbols holding variable bits, in the list's order.
                let mut order: Vec<usize> = Vec::new();
                for &b in &variable {
                    if !order.contains(&(b / 5)) {
                        order.push(b / 5);
                    }
                }
                for j in 0..K {
                    if !order.contains(&j) {
                        order.push(j);
                    }
                }
                // order[r] takes the r-th strongest slot.
                for (r, &j) in order.iter().enumerate() {
                    slot[j] = strong[r] as u8;
                }
            }
            BitMap::Spread => {
                for (i, p) in pos.iter_mut().enumerate() {
                    *p = ((53 * i) % PADDED_BITS) as u16;
                }
            }
            BitMap::SpreadHeavy => {
                // Packed positions of the heavy slots' symbols (symbol j in
                // slot j here), dealt round-robin: bit 0 of every heavy
                // symbol, then bit 1, ...
                let mut deal: Vec<usize> = Vec::new();
                for k in 0..5 {
                    for &j in &heavy {
                        deal.push(5 * j + k);
                    }
                }
                let mut used = [false; PADDED_BITS];
                let mut taken = [false; PADDED_BITS];
                for (&b, &p) in variable.iter().zip(&deal) {
                    pos[b] = p as u16;
                    used[b] = true;
                    taken[p] = true;
                }
                let mut free = (0..PADDED_BITS).filter(|&p| !taken[p]);
                for b in 0..PADDED_BITS {
                    if !used[b] {
                        pos[b] = free.next().unwrap() as u16;
                    }
                }
            }
        }
        let mut bit = [0u16; PADDED_BITS];
        for (i, &p) in pos.iter().enumerate() {
            bit[p as usize] = i as u16;
        }
        debug_assert!({
            let mut seen = [false; PADDED_BITS];
            pos.iter().all(|&p| !std::mem::replace(&mut seen[p as usize], true))
        });
        Self { pos, bit, slot }
    }
}

/// 256 information bits → [`N`] code symbols (0..32), in the link's
/// layout.
pub fn encode_bits(info: &[u8]) -> Vec<u8> {
    encode_bits_with(link_layout(), info)
}

/// [`encode_bits`] in `layout`.
pub fn encode_bits_with(layout: Layout, info: &[u8]) -> Vec<u8> {
    encode_bits_at(Rate::R1_4, layout, info)
}

/// 256 information bits → [`n_for`]`(rate)` code symbols in `layout`.
pub fn encode_bits_at(rate: Rate, layout: Layout, info: &[u8]) -> Vec<u8> {
    assert_eq!(info.len(), K_BITS);
    let t = layout.tables(rate);
    let mut packed = [0u8; PADDED_BITS];
    for (i, &b) in info.iter().enumerate() {
        packed[t.pos[i] as usize] = b & 1;
    }
    let syms = bits_to_syms(&packed);
    let mut u = vec![0u8; K];
    for (j, &x) in syms.iter().enumerate() {
        u[t.slot[j] as usize] = x;
    }
    layout.code(rate).encode(&u)
}

/// Decodes [`N`] × 32 tone log-likelihoods; returns the 256 information
/// bits once every check holds.
pub fn decode_bits(metrics: &[f32]) -> Option<Vec<u8>> {
    decode_bits_prior(metrics, &[])
}

/// A priori log-likelihood that rules a value out (far below anything the
/// channel gives; the decoder clips at −60 relative to the best value).
const RULED_OUT: f32 = -100.0;

/// [`decode_bits`] with a priori LLRs (ln P(0)/P(1), 0 = unknown) of the
/// first `prior.len()` information bits, as `fec::decode_short_prior` takes
/// them for the binary short code: each information symbol's five bits
/// become a prior over its 32 values, added to that symbol's channel
/// metrics. The four padding bits are always known zeros.
pub fn decode_bits_prior(metrics: &[f32], prior: &[f32]) -> Option<Vec<u8>> {
    decode_bits_prior_with(link_layout(), metrics, prior)
}

/// [`decode_bits_prior`] in `layout`.
pub fn decode_bits_prior_with(layout: Layout, metrics: &[f32], prior: &[f32]) -> Option<Vec<u8>> {
    decode_bits_prior_fb(layout, metrics, prior, link_fallback())
}

/// What a priori decoding does when belief propagation fails. Blind
/// decoding (no prior) never falls back: its answer is a codeword whose
/// every check holds, or nothing. With a prior, the fallback returns its
/// likeliest candidate, which the caller's frame CRC-32 accepts or rejects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fallback {
    /// None: belief propagation alone.
    Bp,
    /// [`NbCode::list_decode`] around belief propagation's posteriors, 2^10
    /// candidates.
    List,
}

impl Fallback {
    pub const ALL: [Fallback; 2] = [Fallback::Bp, Fallback::List];

    pub fn name(self) -> &'static str {
        match self {
            Fallback::Bp => "bp",
            Fallback::List => "list",
        }
    }
}

/// The link's fallback.
pub const A_PRIORI_FALLBACK: Fallback = Fallback::Bp;
/// The link's fallback (`ION_NB_FALLBACK` = a [`Fallback::name`]
/// overrides it, for A/B tables).
pub fn link_fallback() -> Fallback {
    static F: OnceLock<Fallback> = OnceLock::new();
    *F.get_or_init(|| {
        std::env::var("ION_NB_FALLBACK")
            .ok()
            .and_then(|v| Fallback::ALL.into_iter().find(|f| f.name() == v))
            .unwrap_or(A_PRIORI_FALLBACK)
    })
}

/// [`decode_bits_prior_with`] with fallback `fb`.
pub fn decode_bits_prior_fb(layout: Layout, metrics: &[f32], prior: &[f32], fb: Fallback) -> Option<Vec<u8>> {
    decode_bits_prior_at(Rate::R1_4, layout, metrics, prior, fb)
}

/// [`decode_bits_prior_fb`] for the code at `rate` ([`n_for`]`(rate)` ×
/// 32 metrics).
pub fn decode_bits_prior_at(
    rate: Rate,
    layout: Layout,
    metrics: &[f32],
    prior: &[f32],
    fb: Fallback,
) -> Option<Vec<u8>> {
    assert_eq!(metrics.len(), n_for(rate) * Q);
    let c = layout.code(rate);
    let t = layout.tables(rate);
    let mut m = metrics.to_vec();
    for j in 0..K {
        let p = c.info[t.slot[j] as usize];
        for (v, l) in m[p * Q..(p + 1) * Q].iter_mut().enumerate() {
            for k in 0..5 {
                let b = t.bit[5 * j + k] as usize;
                let one = (v >> (4 - k)) & 1 == 1;
                if b >= K_BITS {
                    if one {
                        *l += RULED_OUT;
                    }
                } else if let Some(&a) = prior.get(b) {
                    *l += if one { -0.5 * a } else { 0.5 * a };
                }
            }
        }
    }
    let with_prior = prior.iter().any(|&a| a != 0.0);
    let x = match c.decode_soft(&m, K, MAX_ITER) {
        Ok(x) => x,
        Err(_) if !with_prior => return None,
        Err(post) => match fb {
            Fallback::Bp => return None,
            Fallback::List => c.list_decode(&m, &post, 10),
        },
    };
    let u = c.info_of(&x);
    let syms: Vec<u8> = (0..K).map(|j| u[t.slot[j] as usize]).collect();
    let packed: Vec<u8> = syms
        .iter()
        .flat_map(|&x| (0..5).rev().map(move |i| (x >> i) & 1))
        .collect();
    if (K_BITS..PADDED_BITS).any(|i| packed[t.pos[i] as usize] != 0) {
        return None;
    }
    Some((0..K_BITS).map(|i| packed[t.pos[i] as usize]).collect())
}

/// Bits (MSB first, five per symbol, zero-padded) → symbols.
pub fn bits_to_syms(b: &[u8]) -> Vec<u8> {
    b.chunks(5)
        .map(|c| (0..5).fold(0u8, |a, i| (a << 1) | c.get(i).copied().unwrap_or(0) & 1))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn field_axioms() {
        for a in 1..32u8 {
            assert_eq!(GF.mul(a, GF.inv(a)), 1);
            for b in 1..32u8 {
                assert_eq!(GF.mul(a, b), GF.mul(b, a));
                assert_ne!(GF.mul(a, b), 0);
            }
        }
    }

    #[test]
    fn encode_satisfies_checks_and_decodes_clean() {
        let c = code();
        assert!(c.k() >= K, "rank too high: k = {}", c.k());
        let info: Vec<u8> = (0..K_BITS).map(|i| ((i * 7 + 3) % 5 % 2) as u8).collect();
        let x = encode_bits(&info);
        assert_eq!(x.len(), N);
        assert!(c.checks_ok(&x));
        let mut m = vec![0f32; N * Q];
        for (v, &s) in x.iter().enumerate() {
            m[v * Q + s as usize] = 4.0;
        }
        assert_eq!(decode_bits(&m).as_deref(), Some(&info[..]));
    }
}
