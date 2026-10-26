//! Chorus: many stations check in at once in one shared slot, each in its
//! own narrow sub-band (OFDMA with noncoherent MFSK tones); NCS receives
//! them all from one recording.
//!
//! **Numerology** is MFSK R0's: 28 ms symbols (a 20 ms analysis window
//! and an 8 ms guard against echoes), tones on a 50 Hz grid of 48 bins,
//! `center_hz` ± 1175 Hz. The grid is cut into sub-bands of `tones` bins
//! plus `guard` empty bins. Each station sends one tone at a time,
//! constant envelope, with 1 ms phase-continuous frequency sweeps, so its
//! whole transmitter power sits in its own sub-band.
//!
//! **Message:** a compact check-in of 80 bits (packed call, the SNR it
//! hears NCS at, capabilities, software version, CRC-16). It is coded with
//! the short LDPC code *shortened*: the other information bits are known
//! zeros, never sent and given to the decoder as a priori LLRs. 240 bits
//! are sent: the 80 message bits and the first 160 parity bits of the
//! transmission order (rate 1/3), scrambled, interleaved and Gray mapped.
//!
//! **Burst:** 8 known preamble tones, then the data. The receiver knows
//! the slot's start to within a tolerance and searches each sub-band's
//! timing and frequency on the preamble on its own, since every station
//! has its own clock error and frequency offset.

use core::f64::consts::TAU;

use ion_dsp::TX_LEVEL;
use ion_dsp::dsp::{AUDIO_RATE, Complex, Decimator, Nco, lowpass_taps};

use crate::fec;
use crate::link::frame::crc32;
use crate::link::net::{pack_call, unpack_call};

const NBINS: usize = 48;
/// Grid spacing, Hz (1 / the 20 ms window).
const SPACING_HZ: f32 = 50.0;
/// Baseband decimation and rate.
const DEC: usize = 6;
const BB_RATE: f32 = 8000.0;
/// Symbol and analysis window, baseband samples (28 and 20 ms).
const SYM: usize = 224;
const WIN: usize = 160;
/// Symbol, audio samples.
const SYM_AUDIO: usize = SYM * DEC;
/// Frequency sweep and start/end ramps, audio samples (1 ms).
const TRANSITION: usize = 48;
/// Preamble symbols.
pub const PRE_LEN: usize = 8;
/// Message: 8 bytes of payload and a CRC-16.
pub const MSG_BITS: usize = 80;
/// Coded bits sent: the message and 160 parity bits (rate 1/3).
pub const SENT_BITS: usize = 240;
const PARITY_BITS: usize = SENT_BITS - MSG_BITS;
/// Information bits of the short code that hold the message: the first
/// transmitted ones (bits 0..32 are punctured).
const MSG_AT: usize = 32;
/// Transmission-order position of the first parity bit of the short code.
const PARITY_AT: usize = fec::K_SHORT - MSG_AT;
/// Prior LLR of the shortened (known zero) information bits.
const SURE: f32 = 20.0;
/// Front-end filter, as the MFSK receiver's: flat to ±1.3 kHz.
const FE_TAPS: usize = 511;
const FE_CUTOFF_HZ: f32 = 1560.0;
/// Audio index of baseband sample k is DEC·k − FE_DELAY.
const FE_DELAY: i64 = (FE_TAPS as i64 - 1) / 2 - (DEC as i64 - 1);
/// Frequency search on the preamble: ±FMAX in FSTEP steps, then a
/// parabolic refinement.
const FMAX: f32 = 24.0;
const FSTEP: f32 = 4.0;
/// Timing search step, baseband samples (1 ms).
const TSTEP: usize = 8;

/// How the grid is cut into sub-bands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChorusFmt {
    /// Tones per sub-band (a power of two: 4 or 8).
    pub tones: usize,
    /// Empty bins between sub-bands.
    pub guard: usize,
}

impl ChorusFmt {
    /// 5 sub-bands of 8 tones (3 bits per symbol).
    pub const T8: ChorusFmt = ChorusFmt { tones: 8, guard: 1 };
    /// 9 sub-bands of 4 tones (2 bits per symbol).
    pub const T4: ChorusFmt = ChorusFmt { tones: 4, guard: 1 };

    /// Stations that fit in one slot.
    pub fn subbands(self) -> usize {
        (NBINS + self.guard) / (self.tones + self.guard)
    }

    pub fn bits_per_symbol(self) -> usize {
        self.tones.trailing_zeros() as usize
    }

    pub fn data_symbols(self) -> usize {
        SENT_BITS.div_ceil(self.bits_per_symbol())
    }

    /// Audio samples of a burst.
    pub fn burst_samples(self) -> usize {
        (PRE_LEN + self.data_symbols()) * SYM_AUDIO + TRANSITION
    }

    /// Seconds on air.
    pub fn burst_s(self) -> f32 {
        self.burst_samples() as f32 / AUDIO_RATE
    }

    /// Width of one sub-band's tones, Hz.
    pub fn subband_hz(self) -> f32 {
        self.tones as f32 * SPACING_HZ
    }

    /// Grid bin (0..48) of tone `v` of sub-band `sub`. The used bins are
    /// centred in the grid.
    fn bin(self, sub: usize, v: usize) -> usize {
        let used = self.subbands() * (self.tones + self.guard) - self.guard;
        (NBINS - used) / 2 + sub * (self.tones + self.guard) + v
    }

    /// Baseband frequency (Hz from `center_hz`) of tone `v` of `sub`.
    fn hz(self, sub: usize, v: usize) -> f32 {
        (self.bin(sub, v) as f32 - 23.5) * SPACING_HZ
    }
}

/// A chorus check-in.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChorusCheckin {
    pub call: String,
    /// How well the station hears NCS, dB (−24..+39, 1 dB steps).
    pub snr_ncs_db: i8,
    pub caps: u8,
    pub version: u8,
}

impl ChorusCheckin {
    /// 10 bytes: call (5), SNR, caps, version, CRC-16 (low half of the
    /// frame CRC-32).
    pub fn to_bytes(&self) -> [u8; MSG_BITS / 8] {
        self.to_bytes_in(0)
    }

    pub fn from_bytes(b: &[u8]) -> Option<Self> {
        Self::from_bytes_in(b, 0)
    }

    /// As [`ChorusCheckin::to_bytes`], for net `net` (its 8-bit id): the
    /// id is folded into the CRC, so another net's chorus check-in fails
    /// ours. Net 0 is the plain format.
    pub fn to_bytes_in(&self, net: u8) -> [u8; MSG_BITS / 8] {
        let mut b = [0u8; MSG_BITS / 8];
        b[..5].copy_from_slice(&pack_call(&self.call));
        b[5] = (self.snr_ncs_db.clamp(-24, 39) as i16 + 24) as u8;
        b[6] = self.caps;
        b[7] = self.version;
        let crc = crc32(&b[..8]) as u16 ^ net_mask(net);
        b[8..].copy_from_slice(&crc.to_le_bytes());
        b
    }

    pub fn from_bytes_in(b: &[u8], net: u8) -> Option<Self> {
        if b.len() != MSG_BITS / 8 || crc32(&b[..8]) as u16 ^ net_mask(net) != u16::from_le_bytes([b[8], b[9]]) {
            return None;
        }
        Some(Self {
            call: unpack_call(&b[..5]),
            snr_ncs_db: (b[5] as i16 - 24) as i8,
            caps: b[6],
            version: b[7],
        })
    }
}

/// What net `net` XORs into the CRC-16 (0 for net 0).
fn net_mask(net: u8) -> u16 {
    if net == 0 {
        0
    } else {
        mix64(0xC0_0100 ^ net as u64) as u16 | 1
    }
}

fn mix64(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn scramble(i: usize) -> u8 {
    (mix64(0xC0_0005 ^ i as u64) & 1) as u8
}

/// Interleaver step: about n/φ, coprime with n.
// The literal, not the std constant: it fixes the interleaver on the air.
#[allow(clippy::approx_constant)]
fn interleave_mult(n: usize) -> usize {
    let gcd = |mut a: usize, mut b: usize| {
        while b != 0 {
            (a, b) = (b, a % b);
        }
        a
    };
    let mut a = (n as f64 / 1.618_034).round() as usize;
    while gcd(a, n) != 1 {
        a += 1;
    }
    a
}

/// Per-symbol rotation of the tone values (spreads a narrow interferer
/// over every value).
fn hop(fmt: ChorusFmt, s: usize) -> usize {
    (mix64(0xC0_0001 ^ s as u64) % fmt.tones as u64) as usize
}

/// Tone value of preamble symbol `i`.
fn pre_tone(fmt: ChorusFmt, i: usize) -> usize {
    // A fixed pattern that visits different tones (8 tones: a Costas-like
    // permutation; 4: each tone twice).
    const P8: [usize; PRE_LEN] = [0, 5, 2, 7, 3, 6, 1, 4];
    P8[i] % fmt.tones
}

fn gray(v: usize) -> usize {
    v ^ (v >> 1)
}

/// The 240 coded bits of a check-in, in transmission order.
pub fn encode(msg: &ChorusCheckin) -> Vec<u8> {
    encode_in(msg, 0)
}

/// [`encode`] for net `net` (see [`ChorusCheckin::to_bytes_in`]).
pub fn encode_in(msg: &ChorusCheckin, net: u8) -> Vec<u8> {
    let mut info = vec![0u8; fec::K_SHORT];
    for (i, byte) in msg.to_bytes_in(net).iter().enumerate() {
        for k in 0..8 {
            info[MSG_AT + 8 * i + k] = byte >> (7 - k) & 1;
        }
    }
    let cw = fec::encode_short(&info);
    let mut out = cw[..MSG_BITS].to_vec();
    out.extend_from_slice(&cw[PARITY_AT..PARITY_AT + PARITY_BITS]);
    out
}

/// Tone values (0..tones) of the data symbols.
fn data_values(fmt: ChorusFmt, bits: &[u8]) -> Vec<usize> {
    let bps = fmt.bits_per_symbol();
    let n = fmt.data_symbols() * bps;
    let a = interleave_mult(n);
    let mut slots = vec![0u8; n];
    for p in 0..n {
        let b = bits.get(p).copied().unwrap_or(0);
        slots[(a * p) % n] = b ^ scramble(p);
    }
    slots
        .chunks(bps)
        .enumerate()
        .map(|(s, c)| {
            let label = c.iter().fold(0usize, |acc, &b| acc << 1 | b as usize);
            (gray(label) + hop(fmt, s)) % fmt.tones
        })
        .collect()
}

/// One station's burst in sub-band `sub`: 48 kHz audio, peak `TX_LEVEL`.
pub fn modulate(fmt: ChorusFmt, sub: usize, msg: &ChorusCheckin, center_hz: f32) -> Vec<f32> {
    modulate_in(fmt, sub, msg, 0, center_hz)
}

/// [`modulate`] for net `net` (see [`ChorusCheckin::to_bytes_in`]).
pub fn modulate_in(fmt: ChorusFmt, sub: usize, msg: &ChorusCheckin, net: u8, center_hz: f32) -> Vec<f32> {
    assert!(sub < fmt.subbands(), "sub-band {sub} of {}", fmt.subbands());
    let mut tones: Vec<f32> = (0..PRE_LEN).map(|i| fmt.hz(sub, pre_tone(fmt, i))).collect();
    tones.extend(data_values(fmt, &encode_in(msg, net)).iter().map(|&v| fmt.hz(sub, v)));
    let fs = AUDIO_RATE as f64;
    let ramp: Vec<f64> = (0..TRANSITION)
        .map(|s| 0.5 - 0.5 * (core::f64::consts::PI * (s as f64 + 0.5) / TRANSITION as f64).cos())
        .collect();
    let mut out = Vec::with_capacity(fmt.burst_samples());
    let mut phase = 0f64;
    let mut f_old = (center_hz + tones[0]) as f64;
    let a = TX_LEVEL as f64;
    for (k, &t) in tones.iter().enumerate() {
        let f_new = (center_hz + t) as f64;
        for smp in 0..SYM_AUDIO {
            let (f, amp) = match ramp.get(smp) {
                Some(&w) => (f_old + (f_new - f_old) * w, if k == 0 { a * w } else { a }),
                None => (f_new, a),
            };
            phase += f / fs;
            phase -= phase.floor();
            out.push((amp * (TAU * phase).cos()) as f32);
        }
        f_old = f_new;
    }
    for &w in &ramp {
        phase += f_old / fs;
        phase -= phase.floor();
        out.push((a * (1.0 - w) * (TAU * phase).cos()) as f32);
    }
    out
}

/// A check-in heard in a chorus slot.
#[derive(Clone, Debug)]
pub struct Heard {
    pub sub: usize,
    pub checkin: ChorusCheckin,
    /// Estimated SNR in 2.4 kHz, dB.
    pub snr_db: f32,
    pub freq_offset_hz: f32,
    /// Audio sample where its preamble started.
    pub start_sample: u64,
}

/// ln I0(x), the noncoherent tone likelihood.
fn ln_i0(x: f32) -> f32 {
    if x < 2.0 {
        let y = x * x / 4.0;
        (1.0 + y * (1.0 + y / 4.0 * (1.0 + y / 9.0 * (1.0 + y / 16.0)))).ln()
    } else {
        x - 0.5 * (TAU as f32 * x).ln() + (1.0 + 1.0 / (8.0 * x)).ln()
    }
}

fn lse(a: f32, b: f32) -> f32 {
    let m = a.max(b);
    if m == f32::NEG_INFINITY {
        m
    } else {
        m + ((a - m).exp() + (b - m).exp()).ln()
    }
}

/// The NCS side: a recording of the slot as complex baseband.
struct Baseband {
    bb: Vec<Complex>,
}

impl Baseband {
    fn new(audio: &[f32], center_hz: f32) -> Self {
        let mut nco = Nco::new(center_hz, AUDIO_RATE);
        let mut dec = Decimator::new(DEC, lowpass_taps(FE_TAPS, FE_CUTOFF_HZ, AUDIO_RATE));
        let bb = audio.iter().filter_map(|&x| dec.push(nco.mix_down(x))).collect();
        Self { bb }
    }

    /// Energy at `hz` (baseband) in the analysis window of the symbol
    /// starting at baseband sample `start` (its last 20 ms).
    fn energy(&self, start: i64, hz: f32) -> f32 {
        let s = start + (SYM - WIN) as i64;
        if s < 0 || s as usize + WIN > self.bb.len() {
            return 0.0;
        }
        let step = Complex::from_polar(1.0, -(TAU as f32) * hz / BB_RATE);
        let mut ph = Complex::new(1.0, 0.0);
        let mut acc = Complex::ZERO;
        for &x in &self.bb[s as usize..s as usize + WIN] {
            acc += x * ph;
            ph = ph * step;
        }
        acc.norm_sqr()
    }

    /// Preamble statistic of `sub` starting at baseband `t0`, offset `f`:
    /// energy at the known tones over the energy in all of the sub-band's
    /// tones (so a loud neighbour's leakage does not pull it).
    fn pre_stat(&self, fmt: ChorusFmt, sub: usize, t0: i64, f: f32) -> f32 {
        let (mut sig, mut all) = (0.0, 0.0);
        for i in 0..PRE_LEN {
            let st = t0 + (i * SYM) as i64;
            for v in 0..fmt.tones {
                let e = self.energy(st, f + fmt.hz(sub, v));
                all += e;
                if v == pre_tone(fmt, i) {
                    sig += e;
                }
            }
        }
        sig / all.max(1e-30)
    }
}

/// Receives one chorus slot: `audio` holds the recording, the slot starts
/// at audio sample `slot_start` give or take `tol` samples. Returns the
/// check-ins that decoded, by sub-band.
pub fn receive(fmt: ChorusFmt, audio: &[f32], slot_start: u64, tol: u64, center_hz: f32) -> Vec<Heard> {
    receive_in(fmt, audio, slot_start, tol, 0, center_hz)
}

/// Mean power of `audio` in the chorus band (the grid around
/// `center_hz`, as the receiver's front end passes it): tells a slot that
/// held something from an empty one, decoded or not.
pub fn band_power(audio: &[f32], center_hz: f32) -> f32 {
    let bb = Baseband::new(audio, center_hz).bb;
    // Skip the front end's start-up.
    let skip = (FE_TAPS / DEC).min(bb.len());
    let x = &bb[skip..];
    x.iter().map(|c| c.norm_sqr()).sum::<f32>() / x.len().max(1) as f32
}

/// [`receive`] for net `net`: only its check-ins pass the CRC.
pub fn receive_in(fmt: ChorusFmt, audio: &[f32], slot_start: u64, tol: u64, net: u8, center_hz: f32) -> Vec<Heard> {
    let bb = Baseband::new(audio, center_hz);
    let to_bb = |a: i64| (a + FE_DELAY).div_euclid(DEC as i64);
    let (lo, hi) = (to_bb(slot_start as i64 - tol as i64), to_bb((slot_start + tol) as i64));
    (0..fmt.subbands())
        .filter_map(|sub| receive_sub(fmt, &bb, sub, lo, hi, net))
        .collect()
}

/// [`receive`] for one sub-band only.
pub fn receive_subband(
    fmt: ChorusFmt,
    audio: &[f32],
    sub: usize,
    slot_start: u64,
    tol: u64,
    center_hz: f32,
) -> Option<Heard> {
    let bb = Baseband::new(audio, center_hz);
    let to_bb = |a: i64| (a + FE_DELAY).div_euclid(DEC as i64);
    receive_sub(
        fmt,
        &bb,
        sub,
        to_bb(slot_start as i64 - tol as i64),
        to_bb((slot_start + tol) as i64),
        0,
    )
}

fn receive_sub(fmt: ChorusFmt, bb: &Baseband, sub: usize, lo: i64, hi: i64, net: u8) -> Option<Heard> {
    // Coarse timing and frequency on the preamble.
    let nf = (FMAX / FSTEP) as i32;
    let mut best = (f32::MIN, lo, 0.0f32);
    let mut t = lo;
    while t <= hi {
        for k in -nf..=nf {
            let f = k as f32 * FSTEP;
            let z = bb.pre_stat(fmt, sub, t, f);
            if z > best.0 {
                best = (z, t, f);
            }
        }
        t += TSTEP as i64;
    }
    let (_, t0, f0) = best;
    // Parabolic refinement of the frequency.
    let (a, b, c) = (
        bb.pre_stat(fmt, sub, t0, f0 - FSTEP),
        bb.pre_stat(fmt, sub, t0, f0),
        bb.pre_stat(fmt, sub, t0, f0 + FSTEP),
    );
    let den = a - 2.0 * b + c;
    let f = if den < 0.0 {
        f0 + 0.5 * (a - c) / den * FSTEP
    } else {
        f0
    };
    // Tone energies of every data symbol.
    let ns = fmt.data_symbols();
    let m = fmt.tones;
    let mut e = vec![0f32; ns * m];
    for s in 0..ns {
        let st = t0 + ((PRE_LEN + s) * SYM) as i64;
        let h = hop(fmt, s);
        for v in 0..m {
            // Value v was sent on tone (v + hop) mod m.
            e[s * m + v] = bb.energy(st, f + fmt.hz(sub, (v + h) % m));
        }
    }
    // Noise from the losing tones, signal from the winners (preamble
    // included: its tones are known).
    let (mut nsum, mut ncnt, mut smax) = (0f32, 0usize, 0f32);
    for row in e.chunks_exact(m) {
        let (imax, &mx) = row
            .iter()
            .enumerate()
            .fold((0, &f32::MIN), |b, (i, v)| if *v > *b.1 { (i, v) } else { b });
        smax += mx;
        for (i, &v) in row.iter().enumerate() {
            if i != imax {
                nsum += v;
                ncnt += 1;
            }
        }
    }
    let noise = (nsum / ncnt as f32).max(1e-30);
    let sig = (smax / ns as f32 - noise).max(0.05 * noise);
    let snr_db = 10.0 * (sig / noise).log10() - 10.0 * (0.3 * WIN as f32).log10();
    // Bit LLRs: log-sum-exp over the tone values with each label bit.
    let bps = fmt.bits_per_symbol();
    let n = ns * bps;
    let mut slot_llrs = vec![0f32; n];
    let g = 2.0 * sig.sqrt() / noise;
    for (s, row) in e.chunks_exact(m).enumerate() {
        let l: Vec<f32> = row.iter().map(|&x| ln_i0(g * x.max(0.0).sqrt())).collect();
        for k in 0..bps {
            let (mut l0, mut l1) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
            for (v, &lv) in l.iter().enumerate() {
                let label = (0..m).find(|&x| gray(x) == v).unwrap();
                if label >> (bps - 1 - k) & 1 == 0 {
                    l0 = lse(l0, lv);
                } else {
                    l1 = lse(l1, lv);
                }
            }
            slot_llrs[s * bps + k] = (l0 - l1).clamp(-12.0, 12.0);
        }
    }
    // Deinterleave and descramble into the short code's transmission
    // order; the shortened bits get the prior.
    let a = interleave_mult(n);
    let mut llrs = vec![0f32; fec::n_short()];
    for p in 0..SENT_BITS {
        let l = slot_llrs[(a * p) % n];
        let l = if scramble(p) == 1 { -l } else { l };
        let at = if p < MSG_BITS { p } else { PARITY_AT + p - MSG_BITS };
        llrs[at] = l;
    }
    let mut prior = vec![SURE; fec::K_SHORT];
    prior[MSG_AT..MSG_AT + MSG_BITS].fill(0.0);
    let info = fec::decode_short_prior(&llrs, &prior)?;
    let bytes: Vec<u8> = info[MSG_AT..MSG_AT + MSG_BITS]
        .chunks(8)
        .map(|c| c.iter().fold(0u8, |acc, &b| acc << 1 | b))
        .collect();
    let checkin = ChorusCheckin::from_bytes_in(&bytes, net)?;
    Some(Heard {
        sub,
        checkin,
        snr_db,
        freq_offset_hz: f,
        start_sample: (DEC as i64 * t0 - FE_DELAY).max(0) as u64,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numerology() {
        assert_eq!(ChorusFmt::T8.subbands(), 5);
        assert_eq!(ChorusFmt::T4.subbands(), 9);
        for fmt in [ChorusFmt::T8, ChorusFmt::T4] {
            let last = fmt.bin(fmt.subbands() - 1, fmt.tones - 1);
            assert!(last < NBINS, "{fmt:?}");
            // Sub-bands never overlap and keep their guard.
            for s in 1..fmt.subbands() {
                assert_eq!(fmt.bin(s, 0) - fmt.bin(s - 1, fmt.tones - 1), fmt.guard + 1);
            }
        }
        assert_eq!(ChorusFmt::T8.data_symbols(), 80);
        assert_eq!(ChorusFmt::T4.data_symbols(), 120);
    }

    #[test]
    fn message_round_trip() {
        let m = ChorusCheckin {
            call: "KD9QRS".into(),
            snr_ncs_db: -7,
            caps: 5,
            version: 0x01,
        };
        let b = m.to_bytes();
        assert_eq!(ChorusCheckin::from_bytes(&b), Some(m.clone()));
        let mut bad = b;
        bad[2] ^= 4;
        assert_eq!(ChorusCheckin::from_bytes(&bad), None);
        // Another net's check-in fails ours.
        let ours = m.to_bytes_in(0x5A);
        assert_eq!(ChorusCheckin::from_bytes_in(&ours, 0x5A), Some(m.clone()));
        assert_eq!(ChorusCheckin::from_bytes_in(&ours, 0x33), None);
        assert_eq!(ChorusCheckin::from_bytes(&ours), None);
    }
}
