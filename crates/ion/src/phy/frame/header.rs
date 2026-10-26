//! Burst header: 32 information bits and a CRC-16 under a K=13 rate-1/8
//! tail-biting convolutional code.

use std::sync::OnceLock;

use crate::fec::conv::ConvCode;

use super::{DataMode, Layout, Pilots};

pub const HDR_INFO: usize = 32;
pub const HDR_BITS: usize = HDR_INFO + 16;
/// Constraint length and generators of the header code (register bit 0 =
/// newest input), from `fec::conv::search`: d_free 64, one path at it.
const HDR_K: usize = 13;
const GEN: [u32; 8] = [0o14623, 0o10771, 0o17567, 0o10433, 0o13351, 0o17265, 0o15237, 0o17221];
/// Coded bits of one header: 8 per information bit, tail-biting. The slots
/// of a header copy (352–408) take them in order and repeat from the
/// start; a header-only burst's second copy continues the sequence.
pub const HDR_CODED: usize = HDR_BITS * GEN.len();
/// Most QPSK slots of one header copy (any numerology).
pub const HDR_SLOTS_MAX: usize = 408;

/// CRC-16-CCITT (poly 0x1021, init 0xFFFF) over a bit slice, MSB first.
pub fn crc16(bits: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in bits {
        let fb = ((crc >> 15) as u8 ^ b) & 1;
        crc <<= 1;
        if fb == 1 {
            crc ^= 0x1021;
        }
    }
    crc
}

fn push_bits(out: &mut [u8], at: &mut usize, v: u32, n: usize) {
    for i in (0..n).rev() {
        out[*at] = ((v >> i) & 1) as u8;
        *at += 1;
    }
}

fn read_bits(bits: &[u8], at: &mut usize, n: usize) -> u32 {
    let mut v = 0;
    for _ in 0..n {
        v = (v << 1) | bits[*at] as u32;
        *at += 1;
    }
    v
}

/// Header contents.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    /// 0 = QPSK, 1 = 8PSK, 2 = 16QAM, 3 = 64QAM (0 in a header-only burst).
    pub constellation: u8,
    /// Data symbols, 1..=1023 (0 in a header-only burst).
    pub nsym: u16,
    pub user: u16,
    /// The 16 extra bits of a header-only burst (0 otherwise).
    pub extra: u16,
    /// The data symbols use the sparse pilot pattern (data bursts only).
    pub sparse: bool,
    /// How the data symbols are sent (data bursts only).
    pub mode: DataMode,
}

impl Header {
    pub fn header_only(&self) -> bool {
        self.nsym == 0
    }

    pub fn pilots(&self) -> Pilots {
        if self.sparse { Pilots::Sparse } else { Pilots::Robust }
    }
}

/// 48 header bits: constellation (2), data symbols (10), user (16),
/// data mode (3, [`DataMode::bits`]) and the sparse-pilot flag (1), CRC-16
/// XOR `l.crc_data` (16). A header-only burst puts `extra` into the
/// constellation, length, reserved and flag fields (bits 15..14, 13..4,
/// 3..0) and masks the CRC with `l.crc_ho` instead.
pub fn header_bits(h: &Header, l: &Layout) -> [u8; HDR_BITS] {
    let mut b = [0u8; HDR_BITS];
    let mut at = 0;
    let mask = if h.header_only() {
        let e = h.extra as u32;
        push_bits(&mut b, &mut at, e >> 14, 2);
        push_bits(&mut b, &mut at, (e >> 4) & 0x3FF, 10);
        push_bits(&mut b, &mut at, h.user as u32, 16);
        push_bits(&mut b, &mut at, e & 0xF, 4);
        l.crc_ho
    } else {
        push_bits(&mut b, &mut at, h.constellation as u32, 2);
        push_bits(&mut b, &mut at, h.nsym as u32, 10);
        push_bits(&mut b, &mut at, h.user as u32, 16);
        push_bits(&mut b, &mut at, h.mode.bits() | h.sparse as u32, 4);
        l.crc_data
    };
    let crc = crc16(&b[..HDR_INFO]) ^ mask;
    push_bits(&mut b, &mut at, crc as u32, 16);
    b
}

/// Checks the CRC (data or header-only mask of `l`) and, for data
/// headers, the field ranges.
pub fn parse_header(b: &[u8; HDR_BITS], l: &Layout) -> Option<Header> {
    let mut at = 0;
    let c = read_bits(b, &mut at, 2) as u8;
    let nsym = read_bits(b, &mut at, 10) as u16;
    let user = read_bits(b, &mut at, 16) as u16;
    let reserved = read_bits(b, &mut at, 4);
    let crc = read_bits(b, &mut at, 16) as u16;
    let calc = crc16(&b[..HDR_INFO]);
    if crc == calc ^ l.crc_data {
        if nsym == 0 {
            return None;
        }
        return Some(Header {
            constellation: c,
            nsym,
            user,
            extra: 0,
            sparse: reserved & 1 == 1,
            mode: DataMode::from_bits(reserved),
        });
    }
    if crc == calc ^ l.crc_ho {
        let extra = ((c as u32) << 14 | (nsym as u32) << 4 | reserved) as u16;
        return Some(Header {
            constellation: 0,
            nsym: 0,
            user,
            extra,
            sparse: false,
            mode: DataMode::PLAIN,
        });
    }
    None
}

fn hdr_code() -> &'static ConvCode {
    static C: OnceLock<ConvCode> = OnceLock::new();
    C.get_or_init(|| ConvCode::new(HDR_K, &GEN))
}

/// Tail-biting convolutional encoding of the 48 header bits. The output is
/// stream-major (all 48 bits of generator 0, then of generator 1, ...), so
/// a header copy shorter than the codeword drops part of the last stream
/// and a longer one repeats the first.
pub fn conv_encode(bits: &[u8; HDR_BITS]) -> [u8; HDR_CODED] {
    let labels = hdr_code().encode_tb(bits);
    let mut out = [0u8; HDR_CODED];
    for (i, o) in out.as_chunks_mut::<HDR_BITS>().0.iter_mut().enumerate() {
        for (t, b) in o.iter_mut().enumerate() {
            *b = ((labels[t] >> i) & 1) as u8;
        }
    }
    out
}

/// Soft-decision decoder (LLR > 0 means 0) of [`conv_encode`]: wrap-around
/// Viterbi, at most 3 passes.
pub fn viterbi(llr: &[f32; HDR_CODED]) -> [u8; HDR_BITS] {
    let n = GEN.len();
    let mut m = |t: usize, out: &mut [f32]| {
        for (l, o) in out.iter_mut().enumerate() {
            let mut s = 0.0;
            for i in 0..n {
                let v = llr[i * HDR_BITS + t];
                s += if (l >> i) & 1 == 0 { v } else { -v };
            }
            *o = 0.5 * s;
        }
    };
    let bits = hdr_code().decode_tb(HDR_BITS, 3, &mut m).0;
    let mut out = [0u8; HDR_BITS];
    out.copy_from_slice(&bits);
    out
}

#[cfg(test)]
mod tests {
    use super::super::{LAYOUTS, NORMAL};
    use super::*;

    #[test]
    fn header_masks_round_trip() {
        for l in LAYOUTS {
            for h in [
                Header {
                    constellation: 1,
                    nsym: 5,
                    user: 0xABCD,
                    extra: 0,
                    sparse: false,
                    mode: DataMode::PLAIN,
                },
                Header {
                    constellation: 3,
                    nsym: 1023,
                    user: 0x0F0F,
                    extra: 0,
                    sparse: true,
                    mode: DataMode::PLAIN,
                },
                Header {
                    constellation: 0,
                    nsym: 0,
                    user: 0x1234,
                    extra: 0xFFFF,
                    sparse: false,
                    mode: DataMode::PLAIN,
                },
                Header {
                    constellation: 0,
                    nsym: 0,
                    user: 0,
                    extra: 0,
                    sparse: false,
                    mode: DataMode::PLAIN,
                },
            ] {
                assert_eq!(parse_header(&header_bits(&h, l), l), Some(h));
            }
        }
        // A header of one numerology never parses in another.
        let h = Header {
            constellation: 2,
            nsym: 9,
            user: 7,
            extra: 0,
            sparse: false,
            mode: DataMode::PLAIN,
        };
        for a in LAYOUTS {
            for b in LAYOUTS {
                if !std::ptr::eq(a, b) {
                    assert_eq!(parse_header(&header_bits(&h, a), b), None);
                }
            }
        }
    }

    #[test]
    fn viterbi_inverts_encoder() {
        let h = Header {
            constellation: 2,
            nsym: 777,
            user: 0xBEEF,
            extra: 0,
            sparse: false,
            mode: DataMode::PLAIN,
        };
        let b = header_bits(&h, &NORMAL);
        let c = conv_encode(&b);
        let mut llr = [0f32; HDR_CODED];
        for (l, &x) in llr.iter_mut().zip(c.iter()) {
            *l = if x == 0 { 2.0 } else { -2.0 };
        }
        // A few errors are corrected.
        llr[5] = -llr[5];
        llr[40] = -llr[40];
        llr[100] = -llr[100];
        let d = viterbi(&llr);
        assert_eq!(parse_header(&d, &NORMAL), Some(h));
    }
}
