//! Header code: a K = 15 tail-biting convolutional code whose every trellis
//! step emits one whole 32-ary tone (5 output bits), decoded by the
//! wrap-around Viterbi algorithm over tone metrics (no bit LLRs, so no BICM
//! loss). One tone per information bit: 48 or 64 tones, no tail.
//!
//! Generators from `fec::conv::search`: free symbol distance 15 = K, the
//! most a K = 15 code can have.

use std::sync::OnceLock;

use crate::fec::conv::ConvCode;

/// Constraint length.
pub const K: usize = 15;
/// Generators (register bit 0 = newest input; output bit i = parity of
/// `GEN[i] & register`).
pub const GEN: [u32; 5] = [0o42101, 0o77127, 0o60225, 0o42073, 0o64667];
/// WAVA passes round the circular trellis.
const PASSES: usize = 3;

fn code() -> &'static ConvCode {
    static C: OnceLock<ConvCode> = OnceLock::new();
    C.get_or_init(|| ConvCode::new(K, &GEN))
}

/// Tail-biting encoding: one tone label per bit.
pub fn encode(bits: &[u8]) -> Vec<u16> {
    code().encode_tb(bits)
}

/// Decodes `steps` tone slots (= information bits). `metric(j, label)` is
/// the log-likelihood (up to a per-step constant) that step `j` sent tone
/// `label`.
pub fn decode(steps: usize, metric: impl Fn(usize, usize) -> f32) -> Vec<u8> {
    let mut m = |j: usize, out: &mut [f32]| {
        for (l, o) in out.iter_mut().enumerate() {
            *o = metric(j, l);
        }
    };
    code().decode_tb(steps, PASSES, &mut m).0
}

/// Free symbol distance (trellis steps whose tone differs) of the code.
#[doc(hidden)]
pub fn free_symbol_distance() -> usize {
    code().free_distance(&|l| usize::from(l != 0), 1 << 20).0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn symbol_distance_is_k() {
        assert_eq!(free_symbol_distance(), K);
    }

    #[test]
    fn round_trip_clean() {
        for n in [48usize, 64] {
            let bits: Vec<u8> = (0..n).map(|i| ((i * 37 + 5) % 7 % 2) as u8).collect();
            let tones = encode(&bits);
            assert_eq!(tones.len(), n);
            let dec = decode(n, |j, l| if l == tones[j] as usize { 1.0 } else { 0.0 });
            assert_eq!(dec, bits);
        }
    }
}
