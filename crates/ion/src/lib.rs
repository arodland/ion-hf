//! Ion: an adaptive HF data modem and ARQ link.
//!
//! - [`phy`]: OFDM burst modem, 48 kHz audio ↔ soft bits.
//! - [`phy_mfsk`]: noncoherent MFSK bursts, the robust floor.
//! - [`phy_chorus`]: many stations checking in at once in narrow
//!   sub-bands.
//! - [`fec`]: rate-compatible LDPC, bits ↔ codewords.
//! - [`level`]: the speed ladder (constellation × code rate).
//! - [`link`]: frames, ARQ and gear shifting.
//! - [`blanker`]: the receivers' impulse noise blanker.
//!
//! Bits are `u8` values 0 or 1. LLRs are `ln P(bit = 0) / P(bit = 1)`:
//! positive means 0.

pub mod blanker;
pub mod chan;
pub mod fec;
pub mod level;
pub mod link;
pub mod phy;
pub mod phy_chorus;
pub mod phy_mfsk;
pub mod slots;
pub mod sounding;
