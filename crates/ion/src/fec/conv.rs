//! Feedforward convolutional codes for short blocks (headers, tiny ACKs):
//! one input bit per trellis step, up to 16 output bits per step taken as
//! one *label* (a whole MFSK tone, or a group of BPSK bits), zero-tailed or
//! tail-biting, decoded by Viterbi over per-step label metrics. Tail-biting
//! codes use the wrap-around Viterbi algorithm (WAVA; R. Y. Shao, S. Lin,
//! M. P. C. Fossorier, "Two decoding algorithms for tailbiting codes", IEEE
//! Trans. Commun. 51(10), 2003).
//!
//! Register convention: bit 0 is the newest input, bit K − 1 the oldest;
//! output bit i is the parity of `gens[i] & register`; the trellis state is
//! the register without its oldest bit.

/// A rate-1/n feedforward convolutional code, constraint length `k`.
pub struct ConvCode {
    k: usize,
    nout: usize,
    /// Label of every register value.
    labels: Vec<u16>,
}

impl ConvCode {
    /// `gens` are `k`-bit generators, one per output bit (at most 16).
    pub fn new(k: usize, gens: &[u32]) -> Self {
        assert!((2..=16).contains(&k) && !gens.is_empty() && gens.len() <= 16);
        let labels = (0..1usize << k)
            .map(|r| {
                gens.iter().enumerate().fold(0u16, |o, (i, &g)| {
                    o | ((((g as usize & r).count_ones() & 1) as u16) << i)
                })
            })
            .collect();
        Self {
            k,
            nout: gens.len(),
            labels,
        }
    }

    pub fn k(&self) -> usize {
        self.k
    }

    /// Output bits per step.
    pub fn nout(&self) -> usize {
        self.nout
    }

    fn states(&self) -> usize {
        1 << (self.k - 1)
    }

    /// Zero-tailed encoding: `bits`, then zeros up to `steps` steps (at
    /// least K − 1 of them, so the encoder ends in state 0). One label per
    /// step.
    pub fn encode_zt(&self, bits: &[u8], steps: usize) -> Vec<u16> {
        assert!(steps >= bits.len() + self.k - 1);
        let mask = (1usize << self.k) - 1;
        let mut reg = 0usize;
        (0..steps)
            .map(|j| {
                let u = bits.get(j).copied().unwrap_or(0) as usize & 1;
                reg = ((reg << 1) | u) & mask;
                self.labels[reg]
            })
            .collect()
    }

    /// Tail-biting encoding: the register starts holding the last K − 1
    /// bits, so it ends where it started. One label per bit.
    pub fn encode_tb(&self, bits: &[u8]) -> Vec<u16> {
        let n = bits.len();
        assert!(n >= self.k - 1);
        let mask = (1usize << self.k) - 1;
        let mut reg = (0..self.k - 1).fold(0usize, |s, i| s | ((bits[n - 1 - i] as usize & 1) << i));
        bits.iter()
            .map(|&b| {
                reg = ((reg << 1) | (b as usize & 1)) & mask;
                self.labels[reg]
            })
            .collect()
    }

    /// One Viterbi pass over `steps` steps from the path metrics `pm`
    /// (replaced by the final ones). `metric(j, m)` fills `m` with the
    /// log-likelihood of every label at step `j`. `zt`: steps `≥ zt` only
    /// take input 0 (the zero tail). Returns the decisions, one bit per
    /// state and step.
    fn pass(
        &self,
        steps: usize,
        zt: Option<usize>,
        pm: &mut Vec<f32>,
        metric: &mut dyn FnMut(usize, &mut [f32]),
    ) -> Vec<u64> {
        let ns = self.states();
        let words = ns.div_ceil(64);
        let mut dec = vec![0u64; steps * words];
        let mut next = vec![f32::NEG_INFINITY; ns];
        let mut m = vec![0f32; 1 << self.nout];
        let top = 1usize << (self.k - 1);
        for j in 0..steps {
            metric(j, &mut m);
            let d = &mut dec[j * words..(j + 1) * words];
            let tail = zt.is_some_and(|t| j >= t);
            for (s, nx) in next.iter_mut().enumerate() {
                if tail && s & 1 == 1 {
                    *nx = f32::NEG_INFINITY;
                    continue;
                }
                // Predecessors: register = s | (oldest << (K − 1)), state =
                // register >> 1.
                let (r0, r1) = (s, s | top);
                let a = pm[r0 >> 1] + m[self.labels[r0] as usize];
                let b = pm[r1 >> 1] + m[self.labels[r1] as usize];
                if b > a {
                    *nx = b;
                    d[s / 64] |= 1 << (s % 64);
                } else {
                    *nx = a;
                }
            }
            std::mem::swap(pm, &mut next);
        }
        dec
    }

    /// Traceback from `end` over all steps: (inputs, start state).
    fn traceback(&self, dec: &[u64], steps: usize, end: usize) -> (Vec<u8>, usize) {
        let words = self.states().div_ceil(64);
        let mut s = end;
        let mut out = vec![0u8; steps];
        for j in (0..steps).rev() {
            out[j] = (s & 1) as u8;
            let old = (dec[j * words + s / 64] >> (s % 64)) & 1;
            s = (s | ((old as usize) << (self.k - 1))) >> 1;
        }
        (out, s)
    }

    /// Zero-tail Viterbi: starts and ends in state 0. Returns the first
    /// `nbits` inputs and the path metric.
    pub fn decode_zt(&self, steps: usize, nbits: usize, metric: &mut dyn FnMut(usize, &mut [f32])) -> (Vec<u8>, f32) {
        let mut pm = vec![f32::NEG_INFINITY; self.states()];
        pm[0] = 0.0;
        let dec = self.pass(steps, Some(steps - (self.k - 1)), &mut pm, metric);
        let best = pm[0];
        let (mut out, _) = self.traceback(&dec, steps, 0);
        out.truncate(nbits);
        (out, best)
    }

    /// Tail-biting decode by WAVA: up to `passes` passes round the
    /// circular trellis, each starting from the previous pass's final path
    /// metrics; stops at the first pass whose best path starts and ends in
    /// the same state, else returns the best tail-biting path seen (or the
    /// last pass's best path). `metric` is called `passes × steps` times at
    /// most. Returns one input per step and the path metric gained in the
    /// last pass.
    pub fn decode_tb(&self, steps: usize, passes: usize, metric: &mut dyn FnMut(usize, &mut [f32])) -> (Vec<u8>, f32) {
        let ns = self.states();
        let mut pm = vec![0f32; ns];
        let mut best_tb: Option<(f32, Vec<u8>)> = None;
        let mut last = (Vec::new(), 0.0);
        for _ in 0..passes.max(1) {
            let start = pm.clone();
            let dec = self.pass(steps, None, &mut pm, metric);
            let end = (0..ns).fold(0, |b, s| if pm[s] > pm[b] { s } else { b });
            let (bits, s0) = self.traceback(&dec, steps, end);
            let gain = pm[end] - start[s0];
            if s0 == end {
                return (bits, gain);
            }
            // The best path that does bite its tail: the state whose
            // traceback returns to itself, by metric gained this pass.
            for s in 0..ns {
                if pm[s] == f32::NEG_INFINITY {
                    continue;
                }
                let g = pm[s] - start[s];
                if best_tb.as_ref().is_some_and(|b| b.0 >= g) {
                    continue;
                }
                let (b2, t0) = self.traceback(&dec, steps, s);
                if t0 == s {
                    best_tb = Some((g, b2));
                }
            }
            // Keep the metrics bounded.
            let mx = pm[end];
            for p in &mut pm {
                *p -= mx;
            }
            last = (bits, gain);
        }
        match best_tb {
            Some((g, b)) => (b, g),
            None => last,
        }
    }

    /// Free distance and its multiplicity: the fewest `weight(label)`
    /// summed over a path that leaves state 0 and returns to it, and how
    /// many such paths there are (counted up to `cap`; `usize::MAX` for a
    /// catastrophic code, which has a zero-weight cycle). `weight` is the
    /// Hamming weight for binary outputs, or `label != 0` for a tone code.
    pub fn free_distance(&self, weight: &dyn Fn(u16) -> usize, cap: usize) -> (usize, usize) {
        use std::cmp::Reverse;
        use std::collections::BinaryHeap;
        let ns = self.states();
        let mask = (1usize << self.k) - 1;
        // Dijkstra for the distance first.
        let mut dist = vec![usize::MAX; ns];
        let mut heap = BinaryHeap::new();
        heap.push(Reverse((weight(self.labels[1]), 1usize & (ns - 1))));
        let mut dfree = usize::MAX;
        while let Some(Reverse((w, s))) = heap.pop() {
            if s == 0 {
                dfree = w;
                break;
            }
            if dist[s] <= w {
                continue;
            }
            dist[s] = w;
            for u in 0..2 {
                let reg = ((s << 1) | u) & mask;
                heap.push(Reverse((w + weight(self.labels[reg]), reg & (ns - 1))));
            }
        }
        // Count paths of weight dfree: dynamic programming over (weight,
        // state), paths that have not yet returned to 0.
        let mut cnt = vec![vec![0usize; ns]; dfree + 1];
        let w1 = weight(self.labels[1]);
        let mut total = 0usize;
        if w1 <= dfree {
            if 1 & (ns - 1) == 0 {
                total += 1;
            } else {
                cnt[w1][1 & (ns - 1)] = 1;
            }
        }
        for w in 0..=dfree {
            // Zero-weight branches stay in this bucket: propagate them with
            // a work list (a non-catastrophic code has no zero-weight cycle
            // away from state 0, so it empties).
            let mut work: Vec<usize> = (1..ns).filter(|&s| cnt[w][s] > 0).collect();
            let mut steps = 0usize;
            while let Some(s) = work.pop() {
                let c = std::mem::take(&mut cnt[w][s]);
                steps += 1;
                if steps > 64 * ns {
                    // A zero-weight cycle: catastrophic.
                    return (dfree, usize::MAX);
                }
                if c == 0 {
                    continue;
                }
                for u in 0..2 {
                    let reg = ((s << 1) | u) & mask;
                    let nw = w + weight(self.labels[reg]);
                    if nw > dfree {
                        continue;
                    }
                    let t = reg & (ns - 1);
                    if t == 0 {
                        if nw == dfree {
                            total = (total + c).min(cap);
                        }
                    } else {
                        if nw == w && cnt[w][t] == 0 {
                            work.push(t);
                        }
                        cnt[nw][t] = (cnt[nw][t] + c).min(cap);
                    }
                }
            }
        }
        (dfree, total)
    }
}

/// Random search for `nout` generators of constraint length `k` with the
/// largest free distance under `weight`, then the fewest paths at it
/// (`trials` candidates, seeded by `seed`). Every generator taps the
/// newest and the oldest register bit. Returns (generators, distance,
/// multiplicity).
pub fn search(
    k: usize,
    nout: usize,
    weight: &dyn Fn(u16) -> usize,
    trials: usize,
    seed: u64,
) -> (Vec<u32>, usize, usize) {
    let mut rng = super::design::Rng::new(seed);
    let ends = 1u32 | (1u32 << (k - 1));
    let mut best: (Vec<u32>, usize, usize) = (Vec::new(), 0, usize::MAX);
    for _ in 0..trials {
        let gens: Vec<u32> = (0..nout)
            .map(|_| (rng.next_u64() as u32 & ((1u32 << k) - 1)) | ends)
            .collect();
        let c = ConvCode::new(k, &gens);
        let (d, m) = c.free_distance(weight, 1 << 20);
        if m == usize::MAX || m == 0 {
            continue;
        }
        if d > best.1 || (d == best.1 && m < best.2) {
            best = (gens, d, m);
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bits(n: usize, seed: u64) -> Vec<u8> {
        let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
        (0..n)
            .map(|_| {
                s ^= s << 13;
                s ^= s >> 7;
                s ^= s << 17;
                (s >> 33) as u8 & 1
            })
            .collect()
    }

    #[test]
    fn clean_round_trips() {
        let c = ConvCode::new(9, &[0o522, 0o363, 0o570, 0o505, 0o765]);
        for seed in 0..5 {
            let b = bits(48, seed);
            for zt in [true, false] {
                let labels = if zt { c.encode_zt(&b, 56) } else { c.encode_tb(&b) };
                let mut m = |j: usize, out: &mut [f32]| {
                    for (l, o) in out.iter_mut().enumerate() {
                        *o = if l == labels[j] as usize { 1.0 } else { 0.0 };
                    }
                };
                let got = if zt {
                    c.decode_zt(56, 48, &mut m).0
                } else {
                    c.decode_tb(48, 3, &mut m).0
                };
                assert_eq!(got, b, "zt {zt} seed {seed}");
            }
        }
    }

    #[test]
    fn free_distance_of_known_codes() {
        // A K = 9 tone code: symbol distance 9.
        let c = ConvCode::new(9, &[0o522, 0o363, 0o570, 0o505, 0o765]);
        assert_eq!(c.free_distance(&|l| usize::from(l != 0), 100), (9, 1));
        // 3GPP rate-1/3 K = 7 (133, 171, 165): d_free 15.
        let c = ConvCode::new(7, &[0o133, 0o171, 0o165]);
        assert_eq!(c.free_distance(&|l| l.count_ones() as usize, 100).0, 15);
        // Rate-1/2 K = 7 (171, 133): d_free 10, 11 paths.
        let c = ConvCode::new(7, &[0o171, 0o133]);
        assert_eq!(c.free_distance(&|l| l.count_ones() as usize, 100), (10, 11));
    }
}
