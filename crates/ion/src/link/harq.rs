//! Hybrid ARQ with incremental redundancy for long frames: the receiving
//! side's LLR store and the rules both sides compute from it.
//!
//! A long data burst is `[P parity slots][F fresh frames]`. The parity
//! slots continue, in order, the codewords that failed in the previous
//! burst (the *stored set*); slot `i` carries [`increment`]`(h_i, n)` bits
//! of the transmission order from [`start`]`(h_i)`, where `h_i` is how many
//! bits of that codeword the receiver already holds and `n` is the
//! level's frame length. Nothing in the burst names them: both sides
//! derive the layout from the stored set, and the burst's check bits
//! ([`burst_check_bits`]) catch a disagreement. A burst behind an MFSK cue
//! may continue only the first `P` stored codewords and name `P` in the
//! cue ([`partial_field`]); the rest stay stored, ahead of its failures.

use super::audio::bits_to_bytes;
use crate::fec::{self, Decoder, Rate};
use crate::level::{FrameSize, LADDER};

/// A codeword that has had this many bits sent without decoding is dropped
/// (by both sides, without signalling) and sent fresh: two full passes of
/// the mother code.
pub const H_MAX: usize = 2 * fec::N_MOTHER;

/// Most slots in one burst (the ACK bitmap).
pub const MAX_SLOTS: usize = 64;

/// Length of the next transmission of a codeword the receiver holds `h`
/// bits of, in a burst whose frames are `slot` bits: up to the next rate
/// boundary of [`Rate::ALL`], or, once the whole mother code has been
/// sent, a chase repeat of `slot` bits.
pub fn increment(h: usize, slot: usize) -> usize {
    Rate::ALL
        .iter()
        .map(|r| r.n())
        .filter(|&n| n > h)
        .min()
        .map_or(slot, |n| n - h)
}

/// Transmission-order position where the next transmission starts.
pub fn start(h: usize) -> usize {
    h % fec::N_MOTHER
}

/// Most parity slots a burst names explicitly
/// ([`crate::link::Config::harq_partial`]). The field is 6 bits: 0 means
/// "the whole stored set", `p + 1` the first `p` stored codewords.
pub const PARTIAL_MAX: usize = 62;

/// The explicit parity field of a burst carrying increments for the first
/// `parity` of `stored` codewords (0: all of them).
pub fn partial_field(parity: usize, stored: usize) -> u16 {
    if parity >= stored {
        0
    } else {
        parity.min(PARTIAL_MAX) as u16 + 1
    }
}

/// Check bits (`user` bits 14–15) of a data burst with `parity` parity
/// slots: 0 for none, else `1 + parity mod 3`.
pub fn check_bits(parity: usize) -> u8 {
    if parity == 0 { 0 } else { 1 + (parity % 3) as u8 }
}

/// Check bits of a burst sent against `stored` codewords with parity
/// field `field`. A partial burst counts one more: a receiver that missed
/// the cue, and so the field, finds them wrong and keeps nothing rather
/// than misreading the layout.
pub fn burst_check_bits(stored: usize, field: u16) -> u8 {
    check_bits(stored + usize::from(field > 0))
}

/// Soft content of one long-frame data burst.
#[derive(Clone, Debug)]
pub enum Soft {
    /// From the PHY: the burst's LLRs in order (trailing pad included).
    Llrs(Vec<f32>),
    /// From the statistical model channel: one entry per slot as sent.
    Model(Vec<ModelSlot>),
}

/// One slot as the model channel delivers it.
#[derive(Clone, Debug, PartialEq)]
pub struct ModelSlot {
    /// Information bytes of the frame the slot belongs to.
    pub info: Vec<u8>,
    /// The transmission-order span sent: start and length.
    pub from: usize,
    pub bits: usize,
    /// Equivalent SNR (linear) of every coded bit of this transmission.
    /// Repeats of a bit add (maximum-ratio combining); a bit carries
    /// `min(1, log2(1 + gamma))` bits of information, and a codeword
    /// decodes once its bits carry [`fec::K`].
    pub gamma: f32,
}

struct Entry {
    /// Combined LLRs (or model SNRs) per transmission-order position.
    acc: Vec<f32>,
    /// Bits received so far (may exceed `N_MOTHER`: chase repeats).
    h: usize,
    /// Model only: the frame this entry is.
    truth: Option<Vec<u8>>,
}

/// What the receiver made of one long data burst.
#[derive(Clone, Debug, Default)]
pub struct Received {
    /// One per slot, parity slots first: the frame's bytes, or `None`.
    pub frames: Vec<Option<Vec<u8>>>,
    /// Coded bits of each slot.
    pub bits: Vec<usize>,
    /// Parity slots, at the front.
    pub parity: usize,
    /// Codewords decoded thanks to combining.
    pub recovered: usize,
    /// The failed slots are kept (except those past [`H_MAX`]). False when
    /// the layout could not be worked out: then nothing decodes or is kept.
    pub stored: bool,
}

/// The receiving side's store: the codewords that failed in the last data
/// burst, with their LLRs.
pub struct HarqRx {
    entries: Vec<Entry>,
    spare: Vec<Vec<f32>>,
    decoder: Decoder,
    scratch: Vec<f32>,
}

impl Default for HarqRx {
    fn default() -> Self {
        Self::new()
    }
}

impl HarqRx {
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
            spare: Vec::new(),
            decoder: Decoder::new(),
            scratch: Vec::new(),
        }
    }

    /// Codewords stored.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Bits held of each stored codeword, in order.
    pub fn held(&self) -> Vec<usize> {
        self.entries.iter().map(|e| e.h).collect()
    }

    /// Drops everything stored.
    pub fn clear(&mut self) {
        for e in std::mem::take(&mut self.entries) {
            self.recycle(e.acc);
        }
    }

    /// Keeps a freed buffer for reuse (at most one burst's worth).
    fn recycle(&mut self, acc: Vec<f32>) {
        if self.spare.len() < MAX_SLOTS {
            self.spare.push(acc);
        }
    }

    fn entry(&mut self, truth: Option<Vec<u8>>) -> Entry {
        let mut acc = self.spare.pop().unwrap_or_default();
        acc.clear();
        acc.resize(fec::N_MOTHER, 0.0);
        Entry { acc, h: 0, truth }
    }

    /// One long-frame data burst at ladder `level` with check bits `hq`.
    /// Replaces the stored set with this burst's failures.
    pub fn receive(&mut self, level: usize, hq: u8, soft: &Soft) -> Received {
        self.receive_partial(level, hq, 0, soft)
    }

    /// [`HarqRx::receive`] with an explicit parity field ([`partial_field`]):
    /// when it names `p` slots, only the first `p` stored codewords are
    /// continued; the others stay stored, ahead of this burst's failures
    /// (so the longest waiting go first next time).
    pub fn receive_partial(&mut self, level: usize, hq: u8, field: u16, soft: &Soft) -> Received {
        let lv = LADDER[level];
        assert_eq!(lv.frame, FrameSize::Long, "HARQ is for long frames");
        let slot = lv.coded_bits();
        let old = std::mem::take(&mut self.entries);
        let mut out = Received {
            stored: true,
            ..Received::default()
        };
        let old = if hq == 0 {
            for e in old {
                self.recycle(e.acc);
            }
            Vec::new()
        } else if burst_check_bits(old.len(), field) != hq {
            // We do not know which slots are parity: keep nothing.
            for e in old {
                self.recycle(e.acc);
            }
            out.stored = false;
            return out;
        } else {
            old
        };
        let total = old.len();
        let mut old = old;
        if field > 0 && !old.is_empty() {
            let p = field as usize - 1;
            if p == 0 || p > old.len() {
                for e in old {
                    self.recycle(e.acc);
                }
                out.stored = false;
                return out;
            }
            self.entries.extend(old.split_off(p));
        }
        let spans: Vec<(usize, usize)> = old.iter().map(|e| (start(e.h), increment(e.h, slot))).collect();
        let parity_bits: usize = spans.iter().map(|s| s.1).sum();
        out.parity = old.len();
        match soft {
            Soft::Llrs(llrs) => {
                if llrs.len() < parity_bits {
                    for e in old.into_iter().chain(std::mem::take(&mut self.entries)) {
                        self.recycle(e.acc);
                    }
                    out.stored = false;
                    out.parity = 0;
                    return out;
                }
                let fresh = ((llrs.len() - parity_bits) / slot).min(MAX_SLOTS - total);
                let mut pos = 0;
                for (mut e, (from, bits)) in old.into_iter().zip(spans) {
                    add(&mut e.acc, from, &llrs[pos..pos + bits]);
                    pos += bits;
                    e.h += bits;
                    self.settle(e, bits, true, &mut out);
                }
                for _ in 0..fresh {
                    let mut e = self.entry(None);
                    add(&mut e.acc, 0, &llrs[pos..pos + slot]);
                    pos += slot;
                    e.h = slot;
                    self.settle(e, slot, false, &mut out);
                }
            }
            Soft::Model(slots) => {
                if slots.len() < old.len() {
                    for e in old.into_iter().chain(std::mem::take(&mut self.entries)) {
                        self.recycle(e.acc);
                    }
                    out.stored = false;
                    out.parity = 0;
                    return out;
                }
                let p = old.len();
                for ((mut e, (from, bits)), s) in old.into_iter().zip(spans).zip(slots) {
                    // A slot that is not what we expected adds nothing.
                    if (s.from, s.bits) == (from, bits) && e.truth.as_ref() == Some(&s.info) {
                        add_const(&mut e.acc, from, bits, s.gamma);
                    }
                    e.h += bits;
                    self.settle(e, bits, true, &mut out);
                }
                for s in slots[p..].iter().take(MAX_SLOTS - total) {
                    let mut e = self.entry(Some(s.info.clone()));
                    if (s.from, s.bits) == (0, slot) {
                        add_const(&mut e.acc, 0, slot, s.gamma);
                    }
                    e.h = slot;
                    self.settle(e, slot, false, &mut out);
                }
            }
        }
        out
    }

    /// Tries to decode `e`; keeps it if it fails and has bits left to go.
    fn settle(&mut self, e: Entry, bits: usize, combined: bool, out: &mut Received) {
        let got = self.decode(&e);
        out.bits.push(bits);
        if got.is_some() {
            out.recovered += combined as usize;
            self.recycle(e.acc);
        } else if e.h < H_MAX {
            self.entries.push(e);
        } else {
            self.recycle(e.acc);
        }
        out.frames.push(got);
    }

    fn decode(&mut self, e: &Entry) -> Option<Vec<u8>> {
        let n = e.h.min(fec::N_MOTHER);
        if let Some(truth) = &e.truth {
            let info: f32 = e.acc[..n].iter().map(|&g| (1.0 + g).log2().min(1.0)).sum();
            return (info >= fec::K as f32).then(|| truth.clone());
        }
        // The decoder never accepts an exactly-zero posterior; a combined
        // LLR can cancel to zero.
        self.scratch.clear();
        self.scratch
            .extend(e.acc[..n].iter().map(|&l| if l == 0.0 { 1e-6 } else { l }));
        self.decoder.decode_prefix(&self.scratch).map(|b| bits_to_bytes(&b))
    }
}

/// `acc[from + i] += x[i]`, wrapping round the transmission order.
fn add(acc: &mut [f32], from: usize, x: &[f32]) {
    for (i, &v) in x.iter().enumerate() {
        acc[(from + i) % fec::N_MOTHER] += v;
    }
}

fn add_const(acc: &mut [f32], from: usize, bits: usize, v: f32) {
    for i in 0..bits {
        acc[(from + i) % fec::N_MOTHER] += v;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn increments_stop_at_every_rate_boundary_then_chase() {
        assert_eq!(increment(Rate::R5_6.n(), 1728), 192);
        assert_eq!(increment(Rate::R2_3.n(), 2160), 720);
        assert_eq!(increment(Rate::R1_2.n(), 2880), 1440);
        assert_eq!(increment(Rate::R1_3.n(), 4320), 1440);
        assert_eq!(increment(fec::N_MOTHER, 2160), 2160);
        assert_eq!(start(fec::N_MOTHER + 100), 100);
        // Odd h (after a chase repeat) still lands on the boundary.
        assert_eq!(increment(2000, 1728), 160);
        assert_eq!(check_bits(0), 0);
        assert_eq!(check_bits(3), 1);
        assert_eq!(check_bits(5), 3);
    }

    #[test]
    fn a_partial_burst_continues_the_first_stored_and_keeps_the_rest() {
        let level = LADDER.iter().position(|l| l.frame == FrameSize::Long).unwrap();
        let slot = LADDER[level].coded_bits();
        let lost = |k: u8, from: usize, bits: usize| ModelSlot {
            info: vec![k; 4],
            from,
            bits,
            gamma: 0.0,
        };
        let mut rx = HarqRx::new();
        rx.receive(
            level,
            0,
            &Soft::Model(vec![lost(1, 0, slot), lost(2, 0, slot), lost(3, 0, slot)]),
        );
        assert_eq!(rx.held(), vec![slot; 3]);
        let inc = increment(slot, slot);
        // Missing the field (no cue), the receiver keeps nothing.
        let mut missed = HarqRx::new();
        missed.receive(
            level,
            0,
            &Soft::Model(vec![lost(1, 0, slot), lost(2, 0, slot), lost(3, 0, slot)]),
        );
        let r = missed.receive(
            level,
            burst_check_bits(3, partial_field(1, 3)),
            &Soft::Model(vec![lost(1, start(slot), inc), lost(4, 0, slot)]),
        );
        assert!(!r.stored && missed.is_empty());
        let r = rx.receive_partial(
            level,
            burst_check_bits(3, partial_field(1, 3)),
            partial_field(1, 3),
            &Soft::Model(vec![lost(1, start(slot), inc), lost(4, 0, slot)]),
        );
        assert!(r.stored);
        assert_eq!(r.parity, 1);
        // The two left out first, then the burst's failures in order.
        assert_eq!(rx.held(), vec![slot, slot, slot + inc, slot]);
        // A field naming more than is stored: nothing is kept.
        let r = rx.receive_partial(
            level,
            burst_check_bits(4, partial_field(5, 9)),
            partial_field(5, 9),
            &Soft::Model(vec![lost(5, 0, slot)]),
        );
        assert!(!r.stored && rx.is_empty());
        assert_eq!(partial_field(4, 4), 0);
    }
}
