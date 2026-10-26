//! What a receiver measures of the channel over one burst: the echo
//! (delay) spread, the Doppler spread and an effective SNR. The link
//! carries it back in its ACKs (`link::report`) so the sender can pick its
//! waveform and level.
//!
//! - **Delay** (OFDM): the power delay profile from the decision-directed
//!   channel of every data symbol, Hann-windowed across the carriers; the
//!   shortest delay span holding 90% of its power above the noise floor,
//!   less the span a single path shows through the same window. MFSK: from
//!   the energy the previous symbol's tone leaves in the current symbol's
//!   window.
//! - **Doppler**: the channel's correlation over a time lag, turned into
//!   the spread (2σ) of a Gaussian Doppler spectrum: OFDM from the pilots,
//!   MFSK from the phase coherence of the two halves of each tone.
//! - **Effective SNR**: the AWGN SNR (dB in 2.4 kHz) that would decode as
//!   well, by EESM (exponential effective SINR mapping): OFDM over the
//!   demapper's own per-cell SINR, MFSK from the spread of its tone
//!   energies (a Nakagami fit) mapped onto full-band OFDM cells. On a flat
//!   steady channel it equals the mean SNR.

/// Delay span (ms) beyond which the echoes count as long: past the 4 ms
/// guard of the slice and OFDM-normal levels.
pub const ECHO_MS: f32 = 4.0;
/// Doppler spread (Hz) from which the OFDM levels count as unusable.
pub const DOPPLER_HZ: f32 = 6.0;

/// One burst's channel measurement. NaN: not measured (a header-only
/// burst, or too little signal to tell).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChannelMeas {
    /// Echo span, ms: the delay holding 90% of the received power beyond
    /// what a single path shows.
    pub delay_ms: f32,
    /// Doppler spread, Hz (2σ of a Gaussian spectrum, as the Watterson
    /// profiles quote it).
    pub doppler_hz: f32,
    /// Effective SNR, dB in 2.4 kHz (EESM).
    pub eff_snr_db: f32,
    /// Raw statistics behind the numbers above, for calibration tests:
    /// OFDM [ρ over a pilot period, ρ over one symbol, PDP span (ms, with
    /// the single path), fading amount]; MFSK [ρ over half a window, ISI
    /// ratio of the whole-symbol window, ISI ratio of the window starting
    /// 4 ms in, fading amount].
    #[doc(hidden)]
    pub raw: [f32; 4],
}

impl ChannelMeas {
    /// Nothing measured but the mean SNR.
    pub fn of_snr(snr_db: f32) -> Self {
        Self {
            delay_ms: f32::NAN,
            doppler_hz: f32::NAN,
            eff_snr_db: snr_db,
            raw: [f32::NAN; 4],
        }
    }
}

impl Default for ChannelMeas {
    fn default() -> Self {
        Self::of_snr(f32::NAN)
    }
}

/// EESM β per constellation (QPSK, 8PSK, 16QAM, 64QAM; per-cell SINR in
/// linear units), from the usual link-abstraction values for turbo/LDPC
/// codes, and the effective SNR (dB in 2.4 kHz) from which that
/// constellation's β is the one reported: an effective SNR is quoted with
/// the β of the densest constellation it would support (the AWGN entry
/// points of `level::LADDER`).
pub const EESM_BETA: [(f32, f32); 4] = [(1.6, f32::NEG_INFINITY), (3.0, 9.0), (5.5, 11.0), (20.0, 17.0)];

/// Running sums of an EESM over cells.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Eesm {
    exp: [f64; 4],
    sig: f64,
    noise: f64,
    n: f64,
}

impl Eesm {
    /// One cell of channel power `h2` against noise (and estimation error)
    /// `v`.
    pub fn add(&mut self, h2: f32, v: f32) {
        let v = v.max(1e-30);
        let g = (h2 / v) as f64;
        for (e, b) in self.exp.iter_mut().zip(EESM_BETA) {
            *e += (-g / b.0 as f64).exp();
        }
        self.sig += h2 as f64;
        self.noise += v as f64;
        self.n += 1.0;
    }

    /// Effective SNR in the units of `mean_snr_db`, the mean SNR of the
    /// same cells (signal over noise, ratio of the sums): on a flat
    /// channel the two agree. NaN if no cells.
    pub fn eff_db(&self, mean_snr_db: f32) -> f32 {
        if self.n < 1.0 || self.sig <= 0.0 {
            return f32::NAN;
        }
        let rm = self.sig / self.noise;
        pick_beta(|j| {
            let m = (self.exp[j] / self.n).max(1e-300);
            let ge = -(EESM_BETA[j].0 as f64) * m.ln();
            mean_snr_db + (10.0 * (ge.max(1e-30) / rm).log10()) as f32
        })
    }
}

/// The effective SNR quoted with the right β: the densest constellation
/// whose entry point the effective SNR (with its own β) reaches.
fn pick_beta(eff: impl Fn(usize) -> f32) -> f32 {
    let mut best = eff(0);
    for (j, b) in EESM_BETA.iter().enumerate().skip(1) {
        let e = eff(j);
        if e >= b.1 {
            best = e;
        }
    }
    best
}

/// Effective SNR of full-band OFDM cells whose power, relative to its
/// mean, has fading amount `af` (variance over mean², Nakagami m = 1/af),
/// at mean SNR `snr_db` (dB in 2.4 kHz). `cell_scale`: per-cell SNR over
/// the SNR in 2.4 kHz.
pub(crate) fn nakagami_eff_db(snr_db: f32, af: f32, cell_scale: f32) -> f32 {
    if !snr_db.is_finite() || af < 1e-3 {
        return snr_db;
    }
    let g = 10f64.powf(snr_db as f64 / 10.0) * cell_scale as f64;
    let m = 1.0 / af as f64;
    pick_beta(|j| {
        let b = EESM_BETA[j].0 as f64;
        let ge = m * b * (1.0 + g / (m * b)).ln();
        snr_db + (10.0 * (ge / g).log10()) as f32
    })
}

/// Doppler spread (2σ, Hz) of a Gaussian spectrum whose correlation over
/// `lag_s` seconds is `rho` (`exp(−2π²σ²τ²)`).
pub(crate) fn doppler_from_rho(rho: f32, lag_s: f32) -> f32 {
    let r = rho.clamp(1e-6, 1.0);
    let sigma = (-r.ln() / 2.0).sqrt() / (core::f32::consts::PI * lag_s);
    2.0 * sigma
}

/// Shortest circular span (bins) of `p` holding `frac` of its sum.
pub(crate) fn span_bins(p: &[f64], frac: f64) -> usize {
    let n = p.len();
    let total: f64 = p.iter().sum();
    if n == 0 || total <= 0.0 {
        return 0;
    }
    let need = frac * total;
    // Prefix sums over two laps.
    let mut pre = vec![0f64; 2 * n + 1];
    for i in 0..2 * n {
        pre[i + 1] = pre[i] + p[i % n];
    }
    for len in 1..=n {
        if (0..n).any(|s| pre[s + len] - pre[s] >= need) {
            return len;
        }
    }
    n
}

/// `q`-quantile of `v` (copied and sorted).
pub(crate) fn quantile(v: &[f64], q: f64) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    let mut s = v.to_vec();
    s.sort_by(f64::total_cmp);
    s[((s.len() - 1) as f64 * q).round() as usize]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eesm_of_a_flat_channel_is_the_mean() {
        let mut e = Eesm::default();
        for _ in 0..100 {
            e.add(2.0, 1.0);
        }
        assert!((e.eff_db(3.0) - 3.0).abs() < 0.01);
        // Half the cells faded away: well below the mean.
        let mut f = Eesm::default();
        for i in 0..100 {
            f.add(if i % 2 == 0 { 4.0 } else { 0.0 }, 1.0);
        }
        assert!(f.eff_db(3.0) < 0.0, "{}", f.eff_db(3.0));
    }

    #[test]
    fn doppler_and_spans() {
        let d = doppler_from_rho((-2.0f32 * (core::f32::consts::PI * 0.5 * 0.1).powi(2)).exp(), 0.1);
        assert!((d - 1.0).abs() < 1e-3, "{d}");
        let mut p = vec![0.0; 100];
        p[98] = 1.0;
        p[3] = 1.0;
        assert_eq!(span_bins(&p, 0.9), 6);
        assert!(nakagami_eff_db(10.0, 1.0, 1.0) < 7.0);
        assert_eq!(nakagami_eff_db(10.0, 0.0, 1.0), 10.0);
    }
}
