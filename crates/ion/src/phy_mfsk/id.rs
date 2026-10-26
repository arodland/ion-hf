//! Decoding the short codewords that fill an MFSK burst: GF(32) codewords
//! on the tone metrics, binary ones by iterative demapping and decoding
//! (BICM-ID; Li, Ritcey, IEEE Commun. Lett. 1(6), 1997; Valenti, Cheng,
//! IEEE JSAC 23(9), 2005). The LDPC decoder's extrinsic LLRs become the
//! demapper's priors on the other bits of each tone label. Codewords share
//! tones, so they are decoded together and a decoded one gives the others
//! certain priors, as do the padding bits.

use super::MfskBurst;
use super::frame::{self, Fmt};
use super::rx::LLR_MAX;
use super::util::{interleave_mult, lse};
use crate::fec::{self, Rate, ShortSoft, nb};

/// Outer (demapper) rounds, and LDPC iterations in the first round and in
/// each later one. The first round is plain BICM decoding.
const OUTER: usize = 12;
const FIRST: usize = 20;
const INNER: usize = 6;
/// Prior LLR of a bit known for certain (padding, decoded codewords).
const KNOWN: f32 = 30.0;

/// Extrinsic LLRs of the data bits (`burst.llrs.len()` of them, ln P(0)/P(1))
/// from the tone metrics, given a priori LLRs of every data bit. With all
/// priors 0 this is `burst.llrs` (plus what the known padding bits add).
pub fn demap(burst: &MfskBurst, prior: &[f32]) -> Vec<f32> {
    let fmt = Fmt::of(burst.mode);
    let nbits = burst.llrs.len();
    assert_eq!(prior.len(), nbits, "demap: one prior per data bit");
    let nsym = frame::data_symbols(burst.mode, nbits);
    let n = nsym * fmt.bps();
    let vals = fmt.vals();
    assert_eq!(burst.metrics.len(), nsym * fmt.tones * vals, "demap: no tone metrics");
    let a = interleave_mult(n);
    // Priors in slot order (the transmitter's interleaver and scrambler).
    let mut sp = vec![0f32; n];
    for p in 0..n {
        let l = if p < nbits { prior[p] } else { KNOWN };
        sp[(a * p) % n] = if frame::scramble(p) == 1 { -l } else { l };
    }
    let labels: Vec<usize> = (0..vals).map(frame::gray).collect();
    let mut out = vec![0f32; n];
    let mut sc = vec![0f32; vals];
    for (s, m) in burst.metrics.chunks_exact(vals).enumerate() {
        let base = s * fmt.bpt;
        let ap = &sp[base..base + fmt.bpt];
        for (v, c) in sc.iter_mut().enumerate() {
            let mut x = m[v];
            for (k, &l) in ap.iter().enumerate() {
                x += if (labels[v] >> k) & 1 == 0 { 0.5 * l } else { -0.5 * l };
            }
            *c = x;
        }
        for (k, &l) in ap.iter().enumerate() {
            let h = 0.5 * l;
            let (mut l0, mut l1) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
            for (v, &c) in sc.iter().enumerate() {
                if (labels[v] >> k) & 1 == 0 {
                    l0 = lse(l0, c - h);
                } else {
                    l1 = lse(l1, c + h);
                }
            }
            out[base + k] = (l0 - l1).clamp(-LLR_MAX, LLR_MAX);
        }
    }
    frame::deinterleave(&out, nbits)
}

/// Decodes the short frames that fill `burst`'s data. One entry per
/// codeword: its [`fec::K_SHORT`] information bits, or `None`.
///
/// - Symbol-mapped bursts (`modulate_symbols`): GF(32) codewords of
///   [`nb::N`] symbols, decoded on the tone metrics (`fec::nb`).
/// - Bit-mapped bursts: consecutive [`fec::n_short`]-bit binary short
///   codewords (any bits after the last whole one are ignored), decoded by
///   iterative demapping.
pub fn decode_short_frames(burst: &MfskBurst) -> Vec<Option<Vec<u8>>> {
    decode_short_frames_prior(burst, &[])
}

/// [`decode_short_frames_prior`] for a symbol-mapped burst of GF(32)
/// codewords at `rate` ([`nb::n_for`] symbols each; 1/4 is the link's
/// short code, and a bit-mapped burst only has that).
pub fn decode_short_frames_at(burst: &MfskBurst, rate: Rate, info_prior: &[f32]) -> Vec<Option<Vec<u8>>> {
    if rate == Rate::R1_4 {
        return decode_short_frames_prior(burst, info_prior);
    }
    if !burst.symbols {
        return Vec::new();
    }
    burst
        .metrics
        .chunks_exact(nb::n_for(rate) * nb::Q)
        .map(|m| nb::decode_bits_prior_at(rate, nb::link_layout(), m, info_prior, nb::link_fallback()))
        .collect()
}

/// [`decode_short_frames`] with a priori LLRs of the first `prior.len()`
/// information bits of every codeword (ln P(0)/P(1), 0 = unknown): bits
/// the receiver knows or expects, such as a frame's fixed fields (see
/// `fec::Decoder::set_prior`).
pub fn decode_short_frames_prior(burst: &MfskBurst, info_prior: &[f32]) -> Vec<Option<Vec<u8>>> {
    if burst.symbols {
        // GF(32) codewords, one code symbol per tone (`fec::nb`).
        return burst
            .metrics
            .as_chunks::<{ nb::N * nb::Q }>()
            .0
            .iter()
            .map(|m| nb::decode_bits_prior(m, info_prior))
            .collect();
    }
    let n = fec::n_short();
    let nf = burst.llrs.len() / n;
    if burst.metrics.is_empty() {
        return burst
            .llrs
            .chunks_exact(n)
            .map(|c| fec::decode_short_prior(c, info_prior))
            .collect();
    }
    let mut dec: Vec<ShortSoft> = (0..nf).map(|_| ShortSoft::with_prior(info_prior)).collect();
    let mut out: Vec<Option<Vec<u8>>> = vec![None; nf];
    let mut prior = vec![0f32; burst.llrs.len()];
    let mut ch = demap(burst, &prior);
    for round in 0..OUTER {
        let iters = if round == 0 { FIRST } else { INNER };
        for (j, (d, o)) in dec.iter_mut().zip(out.iter_mut()).enumerate() {
            if o.is_some() {
                continue;
            }
            let w = j * n..(j + 1) * n;
            match d.run(&ch[w.clone()], iters) {
                Some(info) => {
                    let cw = fec::encode_short(&info);
                    for (p, &b) in prior[w].iter_mut().zip(&cw) {
                        *p = if b == 0 { KNOWN } else { -KNOWN };
                    }
                    *o = Some(info);
                }
                None => prior[w].copy_from_slice(&d.extrinsic()),
            }
        }
        if out.iter().all(Option::is_some) {
            break;
        }
        ch = demap(burst, &prior);
    }
    out
}
