//! Rate-compatible protograph LDPC (PBRL style): one base matrix
//! (`fec/tables.rs`) lifted at `Z = 90` for the main code (every [`Rate`]
//! sends a prefix of one transmission order, so HARQ can send the next
//! bits) and at `Z = 16` for the rate-1/4 short code. See `docs/fec.md`.
//!
//! Positions from [`transmission_order`] and [`transmitted_positions`]
//! index the full mother codeword of [`encode_mother`], whose first two
//! information blocks are never sent.
//!
//! LLRs are `ln P(0)/P(1)` (positive means 0); bits are `u8` 0/1.

#[doc(hidden)]
pub mod design;

pub mod conv;
pub mod nb;

mod code;
mod decoder;
mod tables;

use std::cell::RefCell;
use std::sync::OnceLock;

use code::QcCode;
pub use decoder::{Algorithm, Decoder};

/// Circulant size of the main code.
pub const Z: usize = 90;
/// Circulant size of the short code.
pub const Z_SHORT: usize = 16;

/// Information bits per codeword (the same for every rate).
pub const K: usize = tables::KB * Z;
/// Length of the full main mother codeword, never-sent blocks included.
pub const N_FULL: usize = (tables::KB + tables::MB) * Z;
/// Most bits the main code can send: the rate-1/4 mother code. Positions
/// beyond [`Rate::R1_3`]`.n()` are extra parity for incremental redundancy.
pub const N_MOTHER: usize = N_FULL - tables::PUNCT * Z;

/// Information bits of the short (link-control) code: a 32-byte frame.
pub const K_SHORT: usize = 256;
/// Bits sent by the short code (rate 1/4).
pub const N_SHORT: usize = (tables::KB + tables::MB - tables::PUNCT) * Z_SHORT;

const _: () = assert!(tables::KB * Z_SHORT == K_SHORT);
const _: () = assert!(4 * K == N_MOTHER);

/// Code rates of the ladder. Every rate encodes the same number of
/// information bits ([`K`]); lower rates send more parity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Rate {
    /// The whole mother code: the most robust rate.
    R1_4,
    R1_3,
    R1_2,
    R2_3,
    R3_4,
    R5_6,
}

impl Rate {
    pub const ALL: [Rate; 6] = [Rate::R1_4, Rate::R1_3, Rate::R1_2, Rate::R2_3, Rate::R3_4, Rate::R5_6];

    /// Codeword length in bits at this rate (exactly `K / rate`; all are
    /// multiples of 12, so they fill whole QPSK, 8PSK and 16QAM symbols).
    pub fn n(self) -> usize {
        match self {
            Rate::R1_4 => 4 * K,
            Rate::R1_3 => 3 * K,
            Rate::R1_2 => 2 * K,
            Rate::R2_3 => 3 * K / 2,
            Rate::R3_4 => 4 * K / 3,
            Rate::R5_6 => 6 * K / 5,
        }
    }

    /// The rate as a number.
    pub fn value(self) -> f64 {
        K as f64 / self.n() as f64
    }
}

fn main_code() -> &'static QcCode {
    static C: OnceLock<QcCode> = OnceLock::new();
    C.get_or_init(|| QcCode::new(&tables::SHIFT_90, Z))
}

fn short_code() -> &'static QcCode {
    static C: OnceLock<QcCode> = OnceLock::new();
    C.get_or_init(|| QcCode::new(&tables::SHIFT_16, Z_SHORT))
}

/// The fixed transmission order of the main code: [`N_MOTHER`] positions
/// into the full mother codeword of [`encode_mother`]. Every rate sends a
/// prefix of it.
pub fn transmission_order() -> &'static [usize] {
    static O: OnceLock<Vec<usize>> = OnceLock::new();
    O.get_or_init(|| (tables::PUNCT * Z..N_FULL).collect())
}

/// Positions (into the full mother codeword) sent at `rate`, in the order
/// [`encode`] emits them: the first `rate.n()` entries of
/// [`transmission_order`].
pub fn transmitted_positions(rate: Rate) -> &'static [usize] {
    &transmission_order()[..rate.n()]
}

/// Encodes `K` information bits into the full mother codeword
/// ([`N_FULL`] bits, including the two never-sent information blocks).
pub fn encode_mother(info: &[u8]) -> Vec<u8> {
    main_code().encode(info)
}

/// True when the full mother codeword `x` ([`N_FULL`] bits) satisfies
/// every parity check of the main code.
pub fn checks_satisfied(x: &[u8]) -> bool {
    let c = main_code();
    assert_eq!(x.len(), c.n_full());
    c.checks_ok(c.n_tx_max(), |v| x[v] & 1)
}

/// Encodes `K` information bits into `rate.n()` coded bits.
pub fn encode(rate: Rate, info: &[u8]) -> Vec<u8> {
    let x = encode_mother(info);
    let t0 = tables::PUNCT * Z;
    x[t0..t0 + rate.n()].to_vec()
}

/// Encodes `K` information bits and returns `bits` bits of the
/// transmission order starting at position `from`, wrapping round after
/// [`N_MOTHER`] (an incremental-redundancy or chase transmission).
/// `encode_span(info, 0, rate.n())` is [`encode`]`(rate, info)`.
pub fn encode_span(info: &[u8], from: usize, bits: usize) -> Vec<u8> {
    let x = encode_mother(info);
    let t0 = tables::PUNCT * Z;
    (0..bits).map(|i| x[t0 + (from + i) % N_MOTHER]).collect()
}

thread_local! {
    static DECODER: RefCell<Decoder> = RefCell::new(Decoder::new());
}

/// Soft-decision decoding of `rate.n()` LLRs. Returns the `K` information
/// bits once every parity check is satisfied, `None` if it gives up.
pub fn decode(rate: Rate, llrs: &[f32]) -> Option<Vec<u8>> {
    DECODER.with_borrow_mut(|d| d.decode(rate, llrs))
}

/// [`decode_short`] with a priori LLRs of the first `prior.len()`
/// information bits (see [`Decoder::set_prior`]).
pub fn decode_short_prior(llrs: &[f32], prior: &[f32]) -> Option<Vec<u8>> {
    DECODER.with_borrow_mut(|d| {
        d.set_prior(prior);
        let out = d.decode_short(llrs);
        d.set_prior(&[]);
        out
    })
}

/// Encodes [`K_SHORT`] information bits into [`n_short`] coded bits.
pub fn encode_short(info: &[u8]) -> Vec<u8> {
    let c = short_code();
    let x = c.encode(info);
    x[c.tx_start()..c.tx_start() + N_SHORT].to_vec()
}

/// Codeword length of the short code.
pub fn n_short() -> usize {
    N_SHORT
}

/// Soft-decision decoding of [`n_short`] LLRs of the short code.
pub fn decode_short(llrs: &[f32]) -> Option<Vec<u8>> {
    DECODER.with_borrow_mut(|d| d.decode_short(llrs))
}

/// Soft-in soft-out decoding of one short codeword, for iterative
/// demapping (BICM-ID): the check messages persist between calls to
/// [`ShortSoft::run`], each of which takes fresh channel LLRs, and
/// [`ShortSoft::extrinsic`] gives the code's opinion of every sent bit to
/// feed back to the demapper.
pub struct ShortSoft {
    d: Decoder,
}

impl Default for ShortSoft {
    fn default() -> Self {
        Self::new()
    }
}

impl ShortSoft {
    pub fn new() -> Self {
        Self::with_prior(&[])
    }

    /// Like [`ShortSoft::new`], with a priori LLRs of the first
    /// `prior.len()` information bits (see [`Decoder::set_prior`]).
    pub fn with_prior(prior: &[f32]) -> Self {
        let mut d = Decoder::new();
        d.set_prior(prior);
        d.soft_start(short_code(), N_SHORT);
        Self { d }
    }

    /// Up to `iters` more iterations on new channel LLRs ([`n_short`] of
    /// them). Returns the [`K_SHORT`] information bits once every check is
    /// satisfied.
    pub fn run(&mut self, llrs: &[f32], iters: usize) -> Option<Vec<u8>> {
        self.d.soft_run(short_code(), llrs, iters)
    }

    /// Extrinsic LLR of every sent bit after the last [`ShortSoft::run`].
    pub fn extrinsic(&self) -> Vec<f32> {
        self.d.soft_extrinsic(short_code())
    }

    /// Iterations run so far.
    pub fn iterations(&self) -> usize {
        self.d.iterations()
    }
}

/// Encodes [`K_SHORT`] bits into the full short mother codeword (for
/// tests; layout as [`encode_mother`] with `Z_SHORT`).
pub fn encode_short_mother(info: &[u8]) -> Vec<u8> {
    short_code().encode(info)
}

/// [`checks_satisfied`] for the short code.
pub fn checks_satisfied_short(x: &[u8]) -> bool {
    let c = short_code();
    assert_eq!(x.len(), c.n_full());
    c.checks_ok(c.n_tx_max(), |v| x[v] & 1)
}

impl Decoder {
    /// Decodes `rate.n()` LLRs of the main code (see [`decode`]).
    pub fn decode(&mut self, rate: Rate, llrs: &[f32]) -> Option<Vec<u8>> {
        assert_eq!(llrs.len(), rate.n(), "decode: expected rate.n() LLRs");
        self.run(main_code(), llrs)
    }

    /// Decodes LLRs for any prefix of [`transmission_order`] (length up to
    /// [`N_MOTHER`]). This is the HARQ entry point: combine the LLRs of
    /// every transmission of a codeword into one buffer in transmission
    /// order and decode the longest prefix received.
    pub fn decode_prefix(&mut self, llrs: &[f32]) -> Option<Vec<u8>> {
        assert!(llrs.len() <= N_MOTHER, "decode_prefix: too many LLRs");
        self.run(main_code(), llrs)
    }

    /// Posterior LLRs of the first `n` positions of the transmission order
    /// after the last [`Self::decode_prefix`], decoded or not.
    pub(crate) fn prefix_posterior(&self, n: usize) -> Vec<f32> {
        self.posterior(main_code(), n)
    }

    /// Decodes [`n_short`] LLRs of the short code.
    pub fn decode_short(&mut self, llrs: &[f32]) -> Option<Vec<u8>> {
        assert_eq!(llrs.len(), N_SHORT, "decode_short: expected n_short() LLRs");
        self.run(short_code(), llrs)
    }
}
