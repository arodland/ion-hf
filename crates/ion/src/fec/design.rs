//! Offline design tools for the Ion LDPC family: the BI-AWGN information
//! functions, protograph EXIT (PEXIT) thresholds, the greedy row-by-row
//! protograph search and the girth-aware circulant lifting.
//!
//! None of this runs in the modem. The tables in `tables.rs` were produced
//! by these functions (see the `#[ignore]` generator tests in
//! `tests/fec.rs`), and `docs/fec.md` describes the procedure.
//!
//! References for the methods (no code or tables were copied from them):
//! - G. Liva, M. Chiani, "Protograph LDPC codes design based on EXIT
//!   analysis", IEEE GLOBECOM 2007 (PEXIT).
//! - T.-Y. Chen, K. Vakilinia, D. Divsalar, R. D. Wesel, "Protograph-based
//!   raptor-like LDPC codes", IEEE Trans. Commun. 63(5), 2015 (PBRL
//!   structure and row-by-row extension design).
//! - X.-Y. Hu, E. Eleftheriou, D. M. Arnold, "Regular and irregular
//!   progressive edge-growth Tanner graphs", IEEE Trans. Inf. Theory 51(1),
//!   2005 (greedy cycle-avoiding edge placement).
//! - Y. Polyanskiy, H. V. Poor, S. Verdú, "Channel coding rate in the finite
//!   blocklength regime", IEEE Trans. Inf. Theory 56(5), 2010 (normal
//!   approximation).

use std::sync::OnceLock;

use super::tables::{CORE_ROWS, KB, PUNCT};

/// Shift of the weight-3 core parity column in core rows 0 and 3.
pub const CORE_A: u16 = 1;

// ---------------------------------------------------------------------
// BI-AWGN information functions
// ---------------------------------------------------------------------

/// `E[f(L)]` for a consistent Gaussian LLR `L ~ N(s^2/2, s^2)`, Simpson rule.
fn gauss_expect(s: f64, f: impl Fn(f64) -> f64) -> f64 {
    let mu = s * s / 2.0;
    let (a, b) = (mu - 12.0 * s, mu + 12.0 * s);
    let m = 2000; // even
    let h = (b - a) / m as f64;
    let norm = 1.0 / (s * (2.0 * std::f64::consts::PI).sqrt());
    let mut acc = 0.0;
    for i in 0..=m {
        let l = a + i as f64 * h;
        let w = if i == 0 || i == m {
            1.0
        } else if i % 2 == 1 {
            4.0
        } else {
            2.0
        };
        let d = (l - mu) / s;
        acc += w * norm * (-0.5 * d * d).exp() * f(l);
    }
    acc * h / 3.0
}

/// `log2(1 + e^-l)`, numerically stable.
fn log2_1p_exp_neg(l: f64) -> f64 {
    let n = if l > 0.0 {
        (-l).exp().ln_1p()
    } else {
        -l + l.exp().ln_1p()
    };
    n / std::f64::consts::LN_2
}

const J_STEP: f64 = 0.005;
const J_MAX: f64 = 40.0;

fn j_table() -> &'static [f64] {
    static T: OnceLock<Vec<f64>> = OnceLock::new();
    T.get_or_init(|| {
        let n = (J_MAX / J_STEP) as usize + 1;
        (0..n)
            .map(|i| {
                let s = i as f64 * J_STEP;
                if s < 1e-3 {
                    0.0
                } else {
                    (1.0 - gauss_expect(s, log2_1p_exp_neg)).clamp(0.0, 1.0)
                }
            })
            .collect()
    })
}

/// Mutual information between a bit and a consistent Gaussian LLR of
/// standard deviation `s`.
pub fn j(s: f64) -> f64 {
    let t = j_table();
    let x = (s / J_STEP).max(0.0);
    let i = x.floor() as usize;
    if i + 1 >= t.len() {
        return t[t.len() - 1];
    }
    let f = x - i as f64;
    t[i] * (1.0 - f) + t[i + 1] * f
}

/// Inverse of [`j`].
pub fn j_inv(mi: f64) -> f64 {
    let t = j_table();
    if mi <= 0.0 {
        return 0.0;
    }
    if mi >= t[t.len() - 1] {
        return J_MAX;
    }
    let i = t.partition_point(|&v| v < mi); // t[i-1] < mi <= t[i]
    let (lo, hi) = (t[i - 1], t[i]);
    let f = if hi > lo { (mi - lo) / (hi - lo) } else { 0.0 };
    (i as f64 - 1.0 + f) * J_STEP
}

/// LLR standard deviation of BPSK on AWGN at `es_n0_db` per coded bit.
pub fn sigma_ch(es_n0_db: f64) -> f64 {
    (8.0 * 10f64.powf(es_n0_db / 10.0)).sqrt()
}

/// BI-AWGN capacity (bits per coded bit) at `es_n0_db`.
pub fn bi_awgn_capacity(es_n0_db: f64) -> f64 {
    let s = sigma_ch(es_n0_db);
    1.0 - gauss_expect(s, log2_1p_exp_neg)
}

/// Shannon limit of BPSK: the Es/N0 (dB, per coded bit) at which the
/// BI-AWGN capacity equals `rate`.
pub fn bi_awgn_limit_db(rate: f64) -> f64 {
    let (mut lo, mut hi) = (-20.0, 20.0);
    for _ in 0..60 {
        let m = 0.5 * (lo + hi);
        if bi_awgn_capacity(m) < rate {
            lo = m;
        } else {
            hi = m;
        }
    }
    0.5 * (lo + hi)
}

/// BI-AWGN capacity and dispersion (bits, bits^2) at `es_n0_db`.
pub fn bi_awgn_cap_disp(es_n0_db: f64) -> (f64, f64) {
    let s = sigma_ch(es_n0_db);
    let c = gauss_expect(s, |l| 1.0 - log2_1p_exp_neg(l));
    let m2 = gauss_expect(s, |l| (1.0 - log2_1p_exp_neg(l)).powi(2));
    (c, (m2 - c * c).max(0.0))
}

/// Gaussian tail probability `Q(x)`, by Simpson integration of the density.
fn q_func(x: f64) -> f64 {
    if x < 0.0 {
        return 1.0 - q_func(-x);
    }
    let (a, b) = (x, x + 40.0);
    let m = 8000;
    let h = (b - a) / m as f64;
    let mut acc = 0.0;
    for i in 0..=m {
        let t = a + i as f64 * h;
        let w = if i == 0 || i == m {
            1.0
        } else if i % 2 == 1 {
            4.0
        } else {
            2.0
        };
        acc += w * (-0.5 * t * t).exp();
    }
    acc * h / 3.0 / (2.0 * std::f64::consts::PI).sqrt()
}

/// Normal approximation (Polyanskiy-Poor-Verdu, with the `log2(n)/2`
/// term) to the best block error rate of an `(n, k)` code on BI-AWGN.
pub fn normal_approx_fer(n: usize, k: usize, es_n0_db: f64) -> f64 {
    let (c, v) = bi_awgn_cap_disp(es_n0_db);
    let nf = n as f64;
    let arg = (nf * c - k as f64 + 0.5 * nf.log2()) / (nf * v).sqrt().max(1e-12);
    q_func(arg)
}

/// Es/N0 (dB, per coded bit) at which [`normal_approx_fer`] equals `fer`.
pub fn normal_approx_db(n: usize, k: usize, fer: f64) -> f64 {
    let (mut lo, mut hi) = (-20.0, 20.0);
    for _ in 0..60 {
        let m = 0.5 * (lo + hi);
        if normal_approx_fer(n, k, m) > fer {
            lo = m;
        } else {
            hi = m;
        }
    }
    0.5 * (lo + hi)
}

// ---------------------------------------------------------------------
// Protograph structure
// ---------------------------------------------------------------------

/// Expands the free entries of a base matrix (per row, the columns not
/// fixed by the structure) into full rows: core rows get the dual-diagonal
/// parity part with the weight-3 first column, extension row `j` gets its
/// own degree-1 parity column `KB + CORE_ROWS + j`.
///
/// Returns `(col, fixed_shift)` per entry; `fixed_shift` is `Some` for the
/// structural entries.
pub fn full_rows(free: &[Vec<usize>]) -> Vec<Vec<(usize, Option<u16>)>> {
    let p = |i: usize| KB + i;
    free.iter()
        .enumerate()
        .map(|(r, cols)| {
            let mut row: Vec<(usize, Option<u16>)> = cols.iter().map(|&c| (c, None)).collect();
            match r {
                0 => row.extend([(p(0), Some(CORE_A)), (p(1), Some(0))]),
                1 => row.extend([(p(0), Some(0)), (p(1), Some(0)), (p(2), Some(0))]),
                2 => row.extend([(p(2), Some(0)), (p(3), Some(0))]),
                3 => row.extend([(p(0), Some(CORE_A)), (p(3), Some(0))]),
                _ => row.push((KB + r, Some(0))),
            }
            row
        })
        .collect()
}

/// PEXIT decoding test of the sub-protograph made of the core and the
/// first `ext` extension rows, at `es_n0_db` per transmitted coded bit.
fn pexit_converges(rows: &[Vec<usize>], ext: usize, es_n0_db: f64, max_iter: usize) -> bool {
    let nrows = CORE_ROWS + ext;
    let ncols = KB + CORE_ROWS + ext;
    let sch2 = sigma_ch(es_n0_db).powi(2);
    // Edge list.
    let mut erow = Vec::new();
    let mut ecol = Vec::new();
    for (r, row) in rows.iter().take(nrows).enumerate() {
        for &c in row {
            erow.push(r);
            ecol.push(c);
        }
    }
    let ne = erow.len();
    let ch: Vec<f64> = (0..ncols).map(|c| if c < PUNCT { 0.0 } else { sch2 }).collect();
    let mut i_c2v = vec![0.0f64; ne];
    let mut i_v2c = vec![0.0f64; ne];
    let mut sq = vec![0.0f64; ne];
    let mut acc_c = vec![0.0f64; ncols];
    let mut acc_r = vec![0.0f64; nrows];
    let mut prev_min = -1.0;
    let mut stall = 0;
    for _ in 0..max_iter {
        // Variable nodes.
        acc_c.copy_from_slice(&ch);
        for e in 0..ne {
            let s = j_inv(i_c2v[e]);
            sq[e] = s * s;
            acc_c[ecol[e]] += sq[e];
        }
        for e in 0..ne {
            i_v2c[e] = j((acc_c[ecol[e]] - sq[e]).max(0.0).sqrt());
        }
        // Check nodes.
        acc_r.iter_mut().for_each(|a| *a = 0.0);
        for e in 0..ne {
            let s = j_inv(1.0 - i_v2c[e]);
            sq[e] = s * s;
            acc_r[erow[e]] += sq[e];
        }
        for e in 0..ne {
            i_c2v[e] = 1.0 - j((acc_r[erow[e]] - sq[e]).max(0.0).sqrt());
        }
        // A-posteriori.
        acc_c.copy_from_slice(&ch);
        for e in 0..ne {
            let s = j_inv(i_c2v[e]);
            acc_c[ecol[e]] += s * s;
        }
        let min_app = acc_c.iter().map(|&a| j(a.sqrt())).fold(1.0f64, f64::min);
        if min_app > 1.0 - 1e-6 {
            return true;
        }
        if min_app - prev_min < 1e-9 {
            stall += 1;
            if stall > 20 {
                return false;
            }
        } else {
            stall = 0;
        }
        prev_min = min_app;
    }
    false
}

/// Rate of the sub-protograph with `ext` extension rows (punctured info
/// columns not sent).
pub fn proto_rate(ext: usize) -> f64 {
    KB as f64 / (KB - PUNCT + CORE_ROWS + ext) as f64
}

/// PEXIT threshold (Es/N0 dB per coded bit, BPSK/AWGN) of the core plus
/// `ext` extension rows, searched in `[limit, limit + span]`. Returns
/// `f64::INFINITY` if it does not converge at the top of the range.
pub fn pexit_threshold(free: &[Vec<usize>], ext: usize, span: f64, max_iter: usize) -> f64 {
    let rows: Vec<Vec<usize>> = full_rows(free)
        .into_iter()
        .map(|r| r.into_iter().map(|(c, _)| c).collect())
        .collect();
    let lim = bi_awgn_limit_db(proto_rate(ext));
    let (mut lo, mut hi) = (lim, lim + span);
    if !pexit_converges(&rows, ext, hi, max_iter) {
        return f64::INFINITY;
    }
    while hi - lo > 0.005 {
        let m = 0.5 * (lo + hi);
        if pexit_converges(&rows, ext, m, max_iter) {
            hi = m;
        } else {
            lo = m;
        }
    }
    hi
}

// ---------------------------------------------------------------------
// Deterministic RNG
// ---------------------------------------------------------------------

/// SplitMix64.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    pub fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
    pub fn shuffle<T>(&mut self, v: &mut [T]) {
        for i in (1..v.len()).rev() {
            let j = self.below(i + 1);
            v.swap(i, j);
        }
    }
}

// ---------------------------------------------------------------------
// Protograph search
// ---------------------------------------------------------------------

/// Evaluates `f` over `items` on all cores; returns results in order.
fn par_map<T: Sync, R: Send>(items: &[T], f: impl Fn(&T) -> R + Sync) -> Vec<R> {
    let nt = std::thread::available_parallelism().map_or(4, |n| n.get());
    let chunk = items.len().div_ceil(nt).max(1);
    std::thread::scope(|s| {
        let hs: Vec<_> = items
            .chunks(chunk)
            .map(|c| s.spawn(|| c.iter().map(&f).collect::<Vec<R>>()))
            .collect();
        hs.into_iter().flat_map(|h| h.join().unwrap()).collect()
    })
}

/// Number of rows of `rows` containing both `a` and `b`.
fn pair_count(rows: &[Vec<usize>], a: usize, b: usize) -> usize {
    rows.iter().filter(|r| r.contains(&a) && r.contains(&b)).count()
}

/// Least number of core rows per information column: one for the
/// punctured columns, three for the sent ones. PEXIT alone favours many
/// sent columns of core degree 2 sharing the same two rows; such pairs
/// lift to low-weight codewords (weight 4 when the shift differences
/// differ by `Z / 2`), seen as undetected errors at the high rates.
fn core_min_deg(c: usize) -> usize {
    if c < PUNCT { 1 } else { 3 }
}

/// Greedy design of the core (hill climbing on the PEXIT gap at the core
/// rate) followed by `ext` extension rows added one by one, each chosen to
/// minimise the PEXIT gap of the code it completes (PBRL procedure).
/// `max_pair` caps the number of rows sharing any column pair, so that the
/// smallest lifting (`Z = 16`) can still be free of 4-cycles.
/// `max_punct_pair` caps the rows that contain two punctured columns:
/// PEXIT keeps choosing them, but beyond a few they buy little at finite
/// length.
pub fn design_base(ext: usize, seed: u64, max_pair: usize, max_punct_pair: usize, log: bool) -> Vec<Vec<usize>> {
    let mut rng = Rng::new(seed);
    let iters = 400;
    let span = 6.0;
    let to_rows =
        |m: &Vec<Vec<bool>>| -> Vec<Vec<usize>> { m.iter().map(|r| (0..KB).filter(|&c| r[c]).collect()).collect() };
    let flips: Vec<(usize, usize)> = (0..CORE_ROWS).flat_map(|r| (0..KB).map(move |c| (r, c))).collect();
    // Core: hill climbing over single flips from a few random starts (a
    // start must already converge somewhere within `span` dB).
    let mut core: Vec<Vec<bool>> = Vec::new();
    let mut best = f64::INFINITY;
    for restart in 0..4 {
        let (mut m, mut thr) = loop {
            let mut m: Vec<Vec<bool>> = (0..CORE_ROWS)
                .map(|_| (0..KB).map(|_| rng.below(4) != 0).collect())
                .collect();
            for c in 0..KB {
                while m.iter().filter(|row| row[c]).count() < core_min_deg(c) {
                    let r = rng.below(CORE_ROWS);
                    m[r][c] = true;
                }
            }
            let t = pexit_threshold(&to_rows(&m), 0, span, iters);
            if t.is_finite() {
                break (m, t);
            }
        };
        loop {
            let res = par_map(&flips, |&(r, c)| {
                let mut t = m.clone();
                t[r][c] = !t[r][c];
                if (0..CORE_ROWS).filter(|&rr| t[rr][c]).count() < core_min_deg(c) {
                    return f64::INFINITY;
                }
                pexit_threshold(&to_rows(&t), 0, span, iters)
            });
            let (bi, &bv) = res.iter().enumerate().min_by(|a, b| a.1.total_cmp(b.1)).unwrap();
            if bv < thr - 0.002 {
                let (r, c) = flips[bi];
                m[r][c] = !m[r][c];
                thr = bv;
            } else {
                break;
            }
        }
        if log {
            eprintln!("core restart {restart}: thr {thr:.3} {:?}", to_rows(&m));
        }
        if thr < best {
            best = thr;
            core = m;
        }
    }
    let mut rows = to_rows(&core);
    if log {
        eprintln!(
            "core rate {:.3} thr {best:.3} limit {:.3}",
            proto_rate(0),
            bi_awgn_limit_db(proto_rate(0))
        );
    }
    // Extension rows. Candidates: an optional subset of the punctured
    // columns plus one to three other columns (info or core parity).
    let others: Vec<usize> = (PUNCT..KB + CORE_ROWS).collect();
    let mut cands: Vec<Vec<usize>> = Vec::new();
    for pmask in 0..(1usize << PUNCT) {
        let ps: Vec<usize> = (0..PUNCT).filter(|&p| pmask >> p & 1 == 1).collect();
        for (i, &a) in others.iter().enumerate() {
            let mut v = ps.clone();
            v.push(a);
            cands.push(v.clone());
            for (j, &b) in others.iter().enumerate().skip(i + 1) {
                let mut w = v.clone();
                w.push(b);
                cands.push(w.clone());
                for &c in &others[j + 1..] {
                    let mut x = w.clone();
                    x.push(c);
                    cands.push(x);
                }
            }
        }
    }
    for e in 1..=ext {
        let ok: Vec<&Vec<usize>> = cands
            .iter()
            .filter(|cand| {
                cand.iter().enumerate().all(|(i, &a)| {
                    cand[i + 1..].iter().all(|&b| {
                        let cap = if b < PUNCT { max_punct_pair } else { max_pair };
                        pair_count(&rows, a, b) < cap
                    })
                })
            })
            .collect();
        let res = par_map(&ok, |cand| {
            let mut t = rows.clone();
            t.push((*cand).clone());
            pexit_threshold(&t, e, 3.0, iters)
        });
        // Smallest threshold; ties (within 0.01 dB) go to the lighter row,
        // then to the less used columns.
        let deg = |c: usize| rows.iter().filter(|r| r.contains(&c)).count();
        let (bi, _) = res
            .iter()
            .enumerate()
            .min_by(|a, b| {
                let ka = (
                    (a.1 * 100.0).round(),
                    ok[a.0].len(),
                    ok[a.0].iter().map(|&c| deg(c)).sum::<usize>(),
                );
                let kb = (
                    (b.1 * 100.0).round(),
                    ok[b.0].len(),
                    ok[b.0].iter().map(|&c| deg(c)).sum::<usize>(),
                );
                ka.partial_cmp(&kb).unwrap()
            })
            .unwrap();
        rows.push(ok[bi].clone());
        if log {
            let r = proto_rate(e);
            eprintln!(
                "ext {e:2} rate {r:.3} {:?} thr {:.3} gap {:.3}",
                ok[bi],
                res[bi],
                res[bi] - bi_awgn_limit_db(r)
            );
        }
    }
    rows
}

// ---------------------------------------------------------------------
// Lifting
// ---------------------------------------------------------------------

/// Closed base-graph cycles through each edge, as "forbidden shift"
/// tallies: `t4` 4-cycles, `t6` 6-cycles, `t8` base 4-cycles that close
/// after two turns (sum `Z/2`, lifted 8-cycles). `shift[r][c]` is `None`
/// for unassigned or absent entries; `rows_of` / `cols_of` give the
/// adjacency.
#[allow(clippy::too_many_arguments)]
fn tally(
    r0: usize,
    c0: usize,
    z: usize,
    shift: &[Vec<Option<u16>>],
    rows_of: &[Vec<usize>],
    cols_of: &[Vec<usize>],
    t4: &mut [u32],
    t6: &mut [u32],
    t8: &mut [u32],
) {
    let zi = z as i64;
    let s = |r: usize, c: usize| shift[r][c].map(|v| v as i64);
    t4.iter_mut().for_each(|v| *v = 0);
    t6.iter_mut().for_each(|v| *v = 0);
    t8.iter_mut().for_each(|v| *v = 0);
    for &r1 in &rows_of[c0] {
        if r1 == r0 {
            continue;
        }
        let Some(s10) = s(r1, c0) else { continue };
        for &c1 in &cols_of[r1] {
            if c1 == c0 {
                continue;
            }
            let Some(s11) = s(r1, c1) else { continue };
            // 4-cycle r0-c0-r1-c1-r0.
            if let Some(s01) = s(r0, c1) {
                let v = (s10 - s11 + s01).rem_euclid(zi) as usize;
                t4[v] += 1;
                // Closing at Z/2 instead lifts to 8-cycles; between two
                // degree-2 columns that is a weight-4 codeword.
                if z.is_multiple_of(2) {
                    t8[(v + z / 2) % z] += 1;
                }
            }
            // 6-cycle r0-c0-r1-c1-r2-c2-r0.
            for &r2 in &rows_of[c1] {
                if r2 == r0 || r2 == r1 {
                    continue;
                }
                let Some(s21) = s(r2, c1) else { continue };
                for &c2 in &cols_of[r2] {
                    if c2 == c0 || c2 == c1 {
                        continue;
                    }
                    let (Some(s22), Some(s02)) = (s(r2, c2), s(r0, c2)) else {
                        continue;
                    };
                    let v = (s10 - s11 + s21 - s22 + s02).rem_euclid(zi) as usize;
                    t6[v] += 1;
                }
            }
        }
    }
}

/// Chooses circulant shifts for every free entry of `free` at lifting `z`:
/// edge by edge (row order), the shift that closes the fewest 4-cycles,
/// then the fewest 6-cycles plus doubled 4-cycles (`t8`), ties broken by
/// a seeded shuffle. Returns the shifts of the free entries, aligned with
/// `free`.
pub fn lift(free: &[Vec<usize>], z: usize, seed: u64) -> Vec<Vec<u16>> {
    let full = full_rows(free);
    let nb = KB + full.len();
    let mut shift: Vec<Vec<Option<u16>>> = vec![vec![None; nb]; full.len()];
    let mut rows_of: Vec<Vec<usize>> = vec![Vec::new(); nb];
    let cols_of: Vec<Vec<usize>> = full.iter().map(|r| r.iter().map(|e| e.0).collect()).collect();
    for (r, row) in full.iter().enumerate() {
        for &(c, fixed) in row {
            rows_of[c].push(r);
            shift[r][c] = fixed;
        }
    }
    let mut rng = Rng::new(seed);
    let mut t4 = vec![0u32; z];
    let mut t6 = vec![0u32; z];
    let mut t8 = vec![0u32; z];
    let mut order: Vec<u16> = (0..z as u16).collect();
    for (r, row) in free.iter().enumerate() {
        for &c in row {
            tally(r, c, z, &shift, &rows_of, &cols_of, &mut t4, &mut t6, &mut t8);
            rng.shuffle(&mut order);
            let &best = order
                .iter()
                .min_by_key(|&&v| (t4[v as usize], t6[v as usize] + t8[v as usize]))
                .unwrap();
            shift[r][c] = Some(best);
        }
    }
    free.iter()
        .enumerate()
        .map(|(r, row)| row.iter().map(|&c| shift[r][c].unwrap()).collect())
        .collect()
}

/// Numbers of base-graph cycles that close in the lifting: 4-cycles,
/// 6-cycles (each lifts to `z` Tanner-graph cycles of that length) and
/// 4-cycles closing at `Z/2` (lifted 8-cycles), counting only rows
/// `< nrows`.
pub fn cycle_counts(free: &[Vec<usize>], shifts: &[Vec<u16>], z: usize, nrows: usize) -> (usize, usize, usize) {
    let full = full_rows(&free[..nrows]);
    let nb = KB + full.len();
    let mut shift: Vec<Vec<Option<u16>>> = vec![vec![None; nb]; full.len()];
    let mut rows_of: Vec<Vec<usize>> = vec![Vec::new(); nb];
    let cols_of: Vec<Vec<usize>> = full.iter().map(|r| r.iter().map(|e| e.0).collect()).collect();
    for (r, row) in full.iter().enumerate() {
        let mut k = 0;
        for &(c, fixed) in row {
            rows_of[c].push(r);
            shift[r][c] = Some(fixed.unwrap_or_else(|| {
                k += 1;
                shifts[r][k - 1]
            }));
        }
    }
    let (mut n4, mut n6, mut n8) = (0, 0, 0);
    let mut t4 = vec![0u32; z];
    let mut t6 = vec![0u32; z];
    let mut t8 = vec![0u32; z];
    for (r, row) in full.iter().enumerate() {
        for &(c, _) in row {
            tally(r, c, z, &shift, &rows_of, &cols_of, &mut t4, &mut t6, &mut t8);
            let own = shift[r][c].unwrap() as usize;
            n4 += t4[own] as usize;
            n6 += t6[own] as usize;
            n8 += t8[own] as usize;
        }
    }
    // `tally` walks each cycle once from each of its edges (leaving the
    // row through that edge's column fixes the direction).
    (n4 / 4, n6 / 6, n8 / 4)
}

/// Tries `tries` lifting seeds starting at `seed0` and returns the one
/// whose lifting has the fewest closed base 4-cycles, then 4-cycles closing
/// at `Z/2`, then 6-cycles.
pub fn best_lift_seed(free: &[Vec<usize>], z: usize, seed0: u64, tries: u64) -> u64 {
    let seeds: Vec<u64> = (0..tries).map(|i| seed0 + i).collect();
    let scores = par_map(&seeds, |&sd| {
        let (c4, c6, c8) = cycle_counts(free, &lift(free, z, sd), z, free.len());
        (c4, c8, c6)
    });
    let (i, _) = scores.iter().enumerate().min_by_key(|e| *e.1).unwrap();
    seeds[i]
}

// ---------------------------------------------------------------------
// The tables in use
// ---------------------------------------------------------------------

/// The free entries of the base matrix in `tables.rs`.
pub fn table_base() -> Vec<Vec<usize>> {
    super::tables::BASE
        .iter()
        .map(|r| r.iter().map(|&c| c as usize).collect())
        .collect()
}

/// The shift table in `tables.rs` for lifting `z` (90 or 16).
pub fn table_shifts(z: usize) -> Vec<Vec<u16>> {
    let t = match z {
        90 => &super::tables::SHIFT_90,
        16 => &super::tables::SHIFT_16,
        _ => panic!("no table for Z = {z}"),
    };
    t.iter().map(|r| r.to_vec()).collect()
}
