//! Erasure coding for net bulletins: a systematic Cauchy code over GF(256).
//!
//! A block of up to [`MAX_K`] source pieces of [`PIECE`] bytes is sent as
//! pieces numbered 0..256. Pieces `0..k` are the sources themselves; piece
//! `i >= k` is the sum of every source `s` times `1 / (i ^ s)`. Every square
//! part of a Cauchy matrix is invertible, so any `k` distinct pieces give
//! the block back. A station short of `n` pieces needs any `n` it has not
//! got: one repair piece fills a different hole at every station.

/// Bytes in one piece.
pub const PIECE: usize = 20;
/// Most source pieces in one block (the rest of the 256 numbers are repairs).
pub const MAX_K: usize = 128;
/// Bytes in a full block.
pub const BLOCK_BYTES: usize = PIECE * MAX_K;

const POLY: u16 = 0x11D;

const fn tables() -> ([u8; 512], [u8; 256]) {
    let mut exp = [0u8; 512];
    let mut log = [0u8; 256];
    let mut x: u16 = 1;
    let mut i = 0;
    while i < 255 {
        exp[i] = x as u8;
        log[x as usize] = i as u8;
        x <<= 1;
        if x & 0x100 != 0 {
            x ^= POLY;
        }
        i += 1;
    }
    let mut j = 255;
    while j < 512 {
        exp[j] = exp[j - 255];
        j += 1;
    }
    (exp, log)
}

const TABLES: ([u8; 512], [u8; 256]) = tables();

fn mul(a: u8, b: u8) -> u8 {
    if a == 0 || b == 0 {
        return 0;
    }
    let (exp, log) = &TABLES;
    exp[log[a as usize] as usize + log[b as usize] as usize]
}

fn inv(a: u8) -> u8 {
    assert!(a != 0, "no inverse of 0");
    let (exp, log) = &TABLES;
    exp[255 - log[a as usize] as usize]
}

/// `dst += c * src`.
fn axpy(dst: &mut [u8], c: u8, src: &[u8]) {
    if c == 0 {
        return;
    }
    for (d, &s) in dst.iter_mut().zip(src) {
        *d ^= mul(c, s);
    }
}

/// What piece `index` is made of, for a block of `k` sources.
fn coeffs(k: usize, index: u8) -> Vec<u8> {
    let i = index as usize;
    if i < k {
        let mut c = vec![0; k];
        c[i] = 1;
        return c;
    }
    (0..k).map(|s| inv(index ^ s as u8)).collect()
}

/// Cuts a block's bytes into source pieces (the last zero-padded).
pub fn sources(block: &[u8]) -> Vec<[u8; PIECE]> {
    block
        .chunks(PIECE)
        .map(|c| {
            let mut p = [0u8; PIECE];
            p[..c.len()].copy_from_slice(c);
            p
        })
        .collect()
}

/// Source pieces of a block of `len` bytes.
pub fn k_of(len: usize) -> usize {
    len.div_ceil(PIECE)
}

/// Piece `index` of a block.
pub fn encode(src: &[[u8; PIECE]], index: u8) -> [u8; PIECE] {
    let k = src.len();
    if (index as usize) < k {
        return src[index as usize];
    }
    let mut out = [0u8; PIECE];
    for (c, s) in coeffs(k, index).into_iter().zip(src) {
        axpy(&mut out, c, s);
    }
    out
}

/// Collects pieces of one block until it can be solved (Gauss–Jordan, one
/// piece at a time, so a station always knows how many it is short).
#[derive(Clone, Debug)]
pub struct Decoder {
    k: usize,
    /// Rows by pivot column: (coefficients, data), reduced.
    rows: Vec<Option<(Vec<u8>, [u8; PIECE])>>,
    rank: usize,
    seen: [bool; 256],
}

impl Decoder {
    pub fn new(k: usize) -> Self {
        assert!((1..=MAX_K).contains(&k));
        Self {
            k,
            rows: vec![None; k],
            rank: 0,
            seen: [false; 256],
        }
    }

    /// Pieces still needed.
    pub fn short(&self) -> usize {
        self.k - self.rank
    }

    pub fn done(&self) -> bool {
        self.rank == self.k
    }

    /// Takes piece `index`. Returns whether it was new information.
    pub fn add(&mut self, index: u8, data: &[u8; PIECE]) -> bool {
        if self.done() || self.seen[index as usize] {
            return false;
        }
        self.seen[index as usize] = true;
        let mut c = coeffs(self.k, index);
        let mut d = *data;
        for col in 0..self.k {
            if c[col] != 0
                && let Some((rc, rd)) = &self.rows[col]
            {
                let f = c[col];
                axpy(&mut c, f, rc);
                axpy(&mut d, f, rd);
            }
        }
        let Some(p) = c.iter().position(|&x| x != 0) else {
            return false;
        };
        let s = inv(c[p]);
        for x in c.iter_mut() {
            *x = mul(*x, s);
        }
        for x in d.iter_mut() {
            *x = mul(*x, s);
        }
        // Keep every other row clear of the new pivot.
        for row in self.rows.iter_mut().flatten() {
            let f = row.0[p];
            if f != 0 {
                axpy(&mut row.0, f, &c);
                axpy(&mut row.1, f, &d);
            }
        }
        self.rows[p] = Some((c, d));
        self.rank += 1;
        true
    }

    /// The block's first `len` bytes, once solved.
    pub fn data(&self, len: usize) -> Option<Vec<u8>> {
        if !self.done() {
            return None;
        }
        let mut out: Vec<u8> = self
            .rows
            .iter()
            .flat_map(|r| r.as_ref().map_or([0; PIECE], |r| r.1))
            .collect();
        out.truncate(len);
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ion_dsp::dsp::NoiseGen;

    #[test]
    fn field_arithmetic() {
        for a in 1..=255u8 {
            assert_eq!(mul(a, inv(a)), 1);
            assert_eq!(mul(a, 1), a);
            assert_eq!(mul(a, 0), 0);
        }
        assert_eq!(mul(2, 0x80), (0x100u16 ^ POLY) as u8);
    }

    fn block(len: usize, seed: u64) -> Vec<u8> {
        let mut g = NoiseGen::new(seed);
        (0..len).map(|_| (g.uniform() * 256.0) as u8).collect()
    }

    /// Any `k` distinct pieces give the block back.
    #[test]
    fn any_k_pieces_decode() {
        let mut g = NoiseGen::new(7);
        for (len, trials) in [(1, 20), (45, 50), (PIECE * 9, 50), (BLOCK_BYTES, 6)] {
            let data = block(len, len as u64);
            let src = sources(&data);
            let k = src.len();
            for _ in 0..trials {
                // A random k of all 256 numbers.
                let mut idx: Vec<u8> = (0..=255u8).collect();
                for i in (1..idx.len()).rev() {
                    let j = (g.uniform() * (i + 1) as f32) as usize % (i + 1);
                    idx.swap(i, j);
                }
                let mut d = Decoder::new(k);
                for &i in &idx[..k] {
                    assert!(d.add(i, &encode(&src, i)), "piece {i} not new (k {k})");
                }
                assert_eq!(d.data(len).unwrap(), data);
                assert!(!d.add(idx[k], &encode(&src, idx[k])));
            }
        }
    }

    #[test]
    fn repeats_are_not_new() {
        let src = sources(&block(100, 3));
        let mut d = Decoder::new(src.len());
        assert!(d.add(200, &encode(&src, 200)));
        assert!(!d.add(200, &encode(&src, 200)));
        assert_eq!(d.short(), src.len() - 1);
    }
}
