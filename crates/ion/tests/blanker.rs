//! The impulse blanker in the receivers (`ion::blanker`): it must leave
//! clean bursts and their timing alone, and it must rescue bursts hit by
//! clicks.

use ion::fec::{self, Rate};
use ion::phy::{self, Constellation, Numerology, PhyRx, Pilots, RxBurst, TxOptions};
use ion::phy_mfsk::{self, MfskBurst, MfskMode, MfskRx};
use ion_dsp::channel::{ImpulseNoise, add_awgn};
use ion_dsp::dsp::{AUDIO_RATE, NoiseGen, noise_sigma};

fn bits(n: usize, seed: u64) -> Vec<u8> {
    let mut g = NoiseGen::new(seed);
    (0..n).map(|_| (g.uniform() < 0.5) as u8).collect()
}

fn pad(burst: &[f32], lead: usize, tail: usize) -> Vec<f32> {
    let mut v = vec![0.0; lead];
    v.extend_from_slice(burst);
    v.extend(std::iter::repeat_n(0.0, tail));
    v
}

fn power(x: &[f32]) -> f32 {
    x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32
}

fn ofdm_rx(num: Numerology, center: f32, blank: bool, audio: &[f32]) -> Vec<RxBurst> {
    let mut rx = PhyRx::with_numerology(num, center);
    rx.set_blanker(blank);
    let mut out = Vec::new();
    for c in audio.chunks(4800) {
        rx.process(c, &mut out);
    }
    out
}

fn mfsk_rx(blank: bool, audio: &[f32]) -> Vec<MfskBurst> {
    let mut rx = MfskRx::new(1600.0);
    rx.set_blanker(blank);
    let mut out = Vec::new();
    for c in audio.chunks(4800) {
        rx.process(c, &mut out);
    }
    out
}

/// Clicks (`rate` per second, `level_db` above the noise σ, ~`ms` long).
fn clicks(x: &mut [f32], sigma: f32, rate: f32, level_db: f32, ms: f32, seed: u64) {
    ImpulseNoise::new(rate, sigma * 10f32.powf(level_db / 20.0), ms, seed).add(x);
}

#[test]
fn clean_bursts_keep_their_bits_and_timing() {
    // 64QAM at a high SNR: its peaks must not be taken for clicks.
    let c = Constellation::Qam64;
    let num = Numerology::NormalWide;
    let coded = bits(phy::capacity_bits_with(num, Pilots::Sparse, c, 20), 3);
    let burst = phy::modulate_full(
        num,
        c,
        0x123,
        0,
        &coded,
        1600.0,
        &TxOptions::for_burst(c, Pilots::Sparse),
    );
    let mut audio = pad(&burst, 14_400, 14_400);
    let sigma = noise_sigma(power(&burst), 30.0, 2400.0, AUDIO_RATE);
    add_awgn(&mut audio, sigma, &mut NoiseGen::new(4));
    let off = ofdm_rx(num, 1600.0, false, &audio);
    let on = ofdm_rx(num, 1600.0, true, &audio);
    assert_eq!(off.len(), 1);
    assert_eq!(on.len(), 1);
    assert!(
        on[0].start_sample.abs_diff(off[0].start_sample) <= 6,
        "{} {}",
        on[0].start_sample,
        off[0].start_sample
    );
    let errs = |b: &RxBurst| {
        b.llrs
            .iter()
            .zip(&coded)
            .filter(|(l, c)| (**l < 0.0) != (**c == 1))
            .count()
    };
    let (e_on, e_off) = (errs(&on[0]), errs(&off[0]));
    assert!(
        e_on <= e_off + coded.len() / 2000,
        "bit errors {e_on} with the blanker, {e_off} without"
    );

    // MFSK R0 on silence: same burst, same start.
    let info = bits(fec::K_SHORT, 5);
    let cw = fec::encode_short(&info);
    let audio = pad(&phy_mfsk::modulate(MfskMode::R0, 7, 9, &cw, 1600.0), 24_000, 28_800);
    let off = mfsk_rx(false, &audio);
    let on = mfsk_rx(true, &audio);
    assert_eq!(on.len(), 1);
    assert_eq!((on[0].user, on[0].extra), (7, 9));
    assert!(
        on[0].start_sample.abs_diff(off[0].start_sample) <= 6,
        "{} {}",
        on[0].start_sample,
        off[0].start_sample
    );
    assert_eq!(
        phy_mfsk::decode_short_frames(&on[0]).into_iter().next().flatten(),
        Some(info)
    );
}

#[test]
fn clicks_are_survived_with_the_blanker() {
    // Slice QPSK 1/3 at −4 dB (which white noise alone does not stop)
    // under 10 clicks a second, 30 dB up: lost without the blanker, kept
    // with it.
    let c = Constellation::Qpsk;
    let num = Numerology::Slice;
    let (mut ok_on, mut ok_off) = (0, 0);
    for seed in 0..3u64 {
        let info = bits(fec::K, 10 + seed);
        let coded = fec::encode(Rate::R1_3, &info);
        let burst = phy::modulate_full(
            num,
            c,
            0x55,
            0,
            &coded,
            1600.0,
            &TxOptions::for_burst(c, Pilots::Robust),
        );
        let mut audio = pad(&burst, 14_400, 14_400);
        let sigma = noise_sigma(power(&burst), -4.0, 2400.0, AUDIO_RATE);
        add_awgn(&mut audio, sigma, &mut NoiseGen::new(20 + seed));
        clicks(&mut audio, sigma, 10.0, 30.0, 2.0, 30 + seed);
        let decode = |out: &[RxBurst]| {
            out.iter().any(|b| {
                b.llrs.len() >= Rate::R1_3.n()
                    && fec::decode(Rate::R1_3, &b.llrs[..Rate::R1_3.n()]) == Some(info.clone())
            })
        };
        ok_off += decode(&ofdm_rx(num, 1600.0, false, &audio)) as usize;
        ok_on += decode(&ofdm_rx(num, 1600.0, true, &audio)) as usize;
    }
    assert_eq!(ok_on, 3, "with the blanker");
    assert!(ok_off < 3, "without: {ok_off}");
}
