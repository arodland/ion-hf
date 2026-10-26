// Slotted reception (`MfskRx::expect`): a net check-in arrives in a slot
// whose start the receiver knows to within ±50 ms (the burst is actually up
// to ±30 ms off), and in a net its header is known too. Compared: blind
// reception, the slot alone, the slot and the expected header, and the
// whole chain with the roster priors (`link::apriori`). The table comes
// from the ignored generator:
//
// cargo test -p ion --release --test slotted -- --ignored --nocapture

use ion::fec;
use ion::link::apriori::{Assume, decode_checkin, report_prior, try_checkin};
use ion::link::audio::{bits_to_bytes, bytes_to_bits};
use ion::link::frame::Frame;
use ion::link::net::{Caps, checkin_frame_bytes, checkin_user, floor_reply_headers, idle_header, report_frame_bytes};
use ion::link::station::user_bits;
use ion::phy_mfsk::{
    EXPECT_MIN_MARGIN, EXPECT_MIN_SCORE, Expect, ExpectedHeader, MfskBurst, MfskMode, MfskRx,
    decode_short_frames_prior, modulate, modulate_symbols,
};
use ion_dsp::channel::{Channel, Profile, add_awgn};
use ion_dsp::dsp::{AUDIO_RATE, NoiseGen, noise_sigma};

const CENTER: f32 = 1500.0;
const NET: u8 = 0x5A;
const OTHER_NET: u8 = 0x33;
/// The channel simulator's Hilbert delay, audio samples.
const CH_DELAY: u64 = 200;
const LEAD_S: f32 = 0.5;
/// Slot tolerance the receiver is told, and the largest real error.
const TOL_S: f32 = 0.050;
const ERR_S: f32 = 0.030;

const ROSTER: [&str; 20] = [
    "N0JLF", "AG5B", "W1AW", "K4XYZ", "N5ABC", "KD9QRS", "W7DEF", "KB0GHI", "AE4JKL", "N8MNO", "K0PQR", "W5STU",
    "KC3VWX", "N2YZA", "AB1BCD", "K6EFG", "W9HIJ", "KE7KLM", "N4NOP", "AD0QRS",
];

/// `MFSK_CODE=nb`: the table generators send check-ins in the GF(32) short
/// code (`fec::nb`, symbol-mapped bursts), as the link does; by default in
/// the binary short code.
fn nb_env() -> bool {
    std::env::var("MFSK_CODE").is_ok_and(|v| v == "nb")
}

/// Data LLRs of a one-check-in burst: one binary short codeword, or one
/// GF(32) codeword (five bits per symbol).
fn checkin_len(len: usize) -> bool {
    len == fec::n_short() || len == 5 * fec::nb::N
}

fn checkin_audio(net: u8, call: &str, seed: u64) -> Vec<f32> {
    checkin_audio_code(net, call, seed, nb_env())
}

/// A check-in burst in the binary short code or (`nb`) the GF(32) one.
fn checkin_audio_code(net: u8, call: &str, seed: u64, nb: bool) -> Vec<f32> {
    let snr = (seed % 30) as f32 - 15.0;
    let info = bytes_to_bits(&checkin_frame_bytes(call, snr, Caps::default(), 0x01));
    if nb {
        let syms = fec::nb::encode_bits(&info);
        modulate_symbols(MfskMode::R0, checkin_user(net), 0, &syms, CENTER)
    } else {
        modulate(MfskMode::R0, checkin_user(net), 0, &fec::encode_short(&info), CENTER)
    }
}

fn power(x: &[f32]) -> f32 {
    x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32
}

fn rng_unit(seed: u64) -> f32 {
    let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    ((z ^ (z >> 31)) >> 40) as f32 / (1u64 << 24) as f32
}

/// `burst` (or silence, if empty) starting `err_s` off the slot, through
/// `profile` at `snr` (against `ref_power`), noise everywhere.
fn air(burst: &[f32], ref_power: f32, err_s: f32, profile: Profile, snr: f32, seed: u64) -> Vec<f32> {
    let start = ((LEAD_S + err_s) * AUDIO_RATE) as usize;
    let len = start + burst.len().max((2.0 * AUDIO_RATE) as usize) + (0.5 * AUDIO_RATE) as usize;
    let mut audio = vec![0.0; len];
    audio[start..start + burst.len()].copy_from_slice(burst);
    let mut ch = Channel::from_profile(profile, seed);
    let mut out = vec![0.0; len];
    ch.process(&audio, &mut out);
    let sigma = noise_sigma(ref_power, snr, 2400.0, AUDIO_RATE);
    add_awgn(&mut out, sigma, &mut NoiseGen::new(seed ^ 0x5151));
    out
}

#[derive(Clone, Copy, PartialEq)]
enum Rx {
    Blind,
    Slot,
    SlotHeader,
}

fn expect_for(rx: Rx) -> Option<Expect> {
    expect_for_code(rx, nb_env())
}

/// The slot, and for [`Rx::SlotHeader`] the header of a check-in in the
/// binary or (`nb`) the GF(32) short code.
fn expect_for_code(rx: Rx, nb: bool) -> Option<Expect> {
    let header = ExpectedHeader {
        mode: MfskMode::R0,
        user: checkin_user(NET),
        extra: 0,
        nbits: if nb { 5 * fec::nb::N } else { fec::n_short() },
        symbols: nb,
        user_mask: 0,
        extra_mask: 0,
    };
    match rx {
        Rx::Blind => None,
        Rx::Slot | Rx::SlotHeader => Some(Expect {
            start_sample: (LEAD_S * AUDIO_RATE) as u64 + CH_DELAY,
            tol_samples: (TOL_S * AUDIO_RATE) as u64,
            header: (rx == Rx::SlotHeader).then_some(header),
            alt: None,
            min_score: EXPECT_MIN_SCORE,
            min_margin: EXPECT_MIN_MARGIN,
        }),
    }
}

fn receive(audio: &[f32], rx: Rx) -> Vec<MfskBurst> {
    receive_with(audio, expect_for(rx))
}

fn receive_with(audio: &[f32], e: Option<Expect>) -> Vec<MfskBurst> {
    let mut r = MfskRx::new(CENTER);
    r.expect(e);
    let mut got = Vec::new();
    for c in audio.chunks(4800) {
        r.process(c, &mut got);
    }
    got
}

fn our_checkin(got: Vec<MfskBurst>) -> Option<MfskBurst> {
    got.into_iter()
        .find(|b| b.user == checkin_user(NET) && checkin_len(b.llrs.len()))
}

#[test]
fn clean_checkin_in_its_slot() {
    let a = checkin_audio(NET, "AG5B", 1);
    let audio = air(&a, power(&a), 0.02, Profile::Awgn, 10.0, 1);
    for rx in [Rx::Blind, Rx::Slot, Rx::SlotHeader] {
        let b = our_checkin(receive(&audio, rx)).expect("burst");
        assert_eq!(try_checkin(&b, Assume::Nothing).unwrap().1, "AG5B");
    }
}

/// Noise in the slot: the expected-header test must (almost) never pass.
#[test]
fn empty_slots_stay_empty() {
    let mut false_bursts = 0;
    for seed in 0..30u64 {
        let audio = air(&[], 1.0, 0.0, Profile::Awgn, 0.0, 500 + seed);
        false_bursts += receive(&audio, Rx::SlotHeader).len();
    }
    assert_eq!(false_bursts, 0);
}

/// Another net's check-in in our slot must not pass as ours: its header
/// differs from the expected one only in the net id, so most of its tones
/// match, and the check-in frame itself carries no net id. The margin over
/// every other `user` value catches it. The score's pass mark is dropped
/// to 0 here so that every burst reaches the margin test.
#[test]
fn another_nets_checkin_is_not_ours() {
    let mut tested = 0;
    for seed in 0..12u64 {
        let snr = -17.0 + (seed % 4) as f32;
        let a = checkin_audio(OTHER_NET, ROSTER[seed as usize], seed);
        let audio = air(&a, power(&a), 0.0, Profile::Awgn, snr, 200 + seed);
        let mut r = MfskRx::new(CENTER);
        let mut e = expect_for(Rx::SlotHeader).unwrap();
        e.min_score = 0.0;
        r.expect(Some(e));
        let mut got = Vec::new();
        for c in audio.chunks(4800) {
            r.process(c, &mut got);
        }
        tested += (r.stats().last_known_score >= 0.0) as usize;
        assert!(
            our_checkin(got).is_none(),
            "seed {seed}: another net's check-in taken for ours"
        );
    }
    assert!(tested >= 4, "only {tested} reached the margin test");
}

/// Below the blind floor the expected header gets check-ins through, and
/// the whole chain decodes more of them than blind reception.
#[test]
fn slot_and_header_beat_blind_at_minus_15_db() {
    let (mut blind, mut chain) = (0, 0);
    for seed in 0..8u64 {
        let call = ROSTER[seed as usize];
        let a = checkin_audio(NET, call, seed);
        let err = (2.0 * rng_unit(seed) - 1.0) * ERR_S;
        let audio = air(&a, power(&a), err, Profile::Awgn, -15.0, 70 + seed);
        if let Some(b) = our_checkin(receive(&audio, Rx::Blind)) {
            blind += try_checkin(&b, Assume::Nothing).is_some() as usize;
        }
        if let Some(b) = our_checkin(receive(&audio, Rx::SlotHeader))
            && let Some(c) = decode_checkin(&b, &ROSTER)
        {
            assert_eq!(c.call, call);
            chain += 1;
        }
    }
    assert!(chain > blind, "chain {chain} vs blind {blind}");
}

/// The same chain with the GF(32) short code the link sends: the expected
/// header carries the symbol-mapped length field, and the bursts it lets
/// through come out symbol-mapped.
#[test]
fn gf32_slot_and_header_beat_blind_at_minus_16_db() {
    let (mut blind, mut chain, mut known) = (0, 0, 0);
    for seed in 0..8u64 {
        let call = ROSTER[seed as usize];
        let a = checkin_audio_code(NET, call, seed, true);
        let err = (2.0 * rng_unit(seed) - 1.0) * ERR_S;
        let audio = air(&a, power(&a), err, Profile::Awgn, -16.0, 70 + seed);
        if let Some(b) = our_checkin(receive_with(&audio, None)) {
            blind += try_checkin(&b, Assume::Nothing).is_some() as usize;
        }
        let mut r = MfskRx::new(CENTER);
        r.expect(expect_for_code(Rx::SlotHeader, true));
        let mut got = Vec::new();
        for c in audio.chunks(4800) {
            r.process(c, &mut got);
        }
        known += r.stats().known_headers as usize;
        if let Some(b) = our_checkin(got) {
            assert!(b.symbols && b.llrs.len() == 5 * fec::nb::N);
            if let Some(c) = decode_checkin(&b, &ROSTER) {
                assert_eq!(c.call, call);
                chain += 1;
            }
        }
    }
    assert!(known > 0, "the expected-header test never passed");
    assert!(chain > blind, "chain {chain} vs blind {blind}");
}

// ---------------------------------------------------------------------
// Table generator (ignored)
// ---------------------------------------------------------------------

#[derive(Default, Clone, Copy)]
struct Cell {
    n: usize,
    hdr: [usize; 3],
    /// Check-ins decoded: blind reception and decoding; slot and header
    /// reception with the roster priors.
    blind: usize,
    chain: usize,
    wrong: usize,
}

fn add(a: Cell, c: Cell) -> Cell {
    Cell {
        n: a.n + c.n,
        hdr: [a.hdr[0] + c.hdr[0], a.hdr[1] + c.hdr[1], a.hdr[2] + c.hdr[2]],
        blind: a.blind + c.blind,
        chain: a.chain + c.chain,
        wrong: a.wrong + c.wrong,
    }
}

fn par<F: Fn(usize) -> Cell + Sync>(n: usize, f: F) -> Cell {
    // ION_THREADS: worker threads (default 2; results do not depend
    // on it, every check-in has its own seed).
    let threads = std::env::var("ION_THREADS")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(2)
        .max(1);
    std::thread::scope(|s| {
        let f = &f;
        let hs: Vec<_> = (0..threads)
            .map(|t| s.spawn(move || (t..n).step_by(threads).map(f).fold(Cell::default(), add)))
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).fold(Cell::default(), add)
    })
}

fn run_cell(profile: Profile, snr: f32, n: usize) -> Cell {
    par(n, |i| {
        let seed = 7000 * (snr as i64 + 100) as u64 + i as u64;
        let call = ROSTER[i % ROSTER.len()];
        let a = checkin_audio(NET, call, seed);
        let err = (2.0 * rng_unit(seed) - 1.0) * ERR_S;
        let audio = air(&a, power(&a), err, profile, snr, seed);
        let mut c = Cell {
            n: 1,
            ..Cell::default()
        };
        for (k, rx) in [Rx::Blind, Rx::Slot, Rx::SlotHeader].into_iter().enumerate() {
            let Some(b) = our_checkin(receive(&audio, rx)) else {
                continue;
            };
            c.hdr[k] = 1;
            match rx {
                Rx::Blind => c.blind = try_checkin(&b, Assume::Nothing).is_some() as usize,
                Rx::SlotHeader => match decode_checkin(&b, &ROSTER) {
                    Some(k) if k.call == call => c.chain = 1,
                    Some(_) => c.wrong = 1,
                    None => {}
                },
                Rx::Slot => {}
            }
        }
        c
    })
}

#[test]
#[ignore]
fn slotted_table() {
    let n: usize = std::env::var("SLOTTED_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(40);
    let grid = [
        (Profile::Awgn, "AWGN", -18..=-11),
        (Profile::CcirPoor, "CCIR poor", -16..=-9),
        (Profile::MidDisturbedNvis, "NVIS disturbed", -14..=-7),
    ];
    for (profile, name, snrs) in grid {
        if !table_profile(name) {
            continue;
        }
        println!(
            "\n{name}, {n} check-ins per SNR, slot ±{:.0} ms, real error ±{:.0} ms\n",
            TOL_S * 1e3,
            ERR_S * 1e3
        );
        println!(
            "| SNR dB | header: blind | slot | slot + known header | check-in: blind | slot + header + roster | wrong |"
        );
        println!("|---:|---:|---:|---:|---:|---:|---:|");
        for snr in snrs.filter(|&s| table_snr(s)) {
            let c = run_cell(profile, snr as f32, n);
            let f = |x: usize| x as f32 / c.n as f32;
            println!(
                "| {snr} | {:.2} | {:.2} | {:.2} | {:.2} | {:.2} | {} |",
                f(c.hdr[0]),
                f(c.hdr[1]),
                f(c.hdr[2]),
                f(c.blind),
                f(c.chain),
                c.wrong
            );
        }
    }
}

/// False alarms: empty slots (noise only) and slots holding another net's
/// check-in, received with the slot and the expected header. Also the
/// score distribution of the expected-header test on empty slots.
#[test]
#[ignore]
fn slotted_false_alarms() {
    let n: usize = std::env::var("SLOTTED_FA_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(400);
    let empty = par(n, |i| {
        let audio = air(&[], 1.0, 0.0, Profile::Awgn, 0.0, 90_000 + i as u64);
        let got = receive(&audio, Rx::SlotHeader);
        Cell {
            n: 1,
            wrong: got.len(),
            ..Cell::default()
        }
    });
    println!("empty slots: {} bursts out of {} slots", empty.wrong, empty.n);
    let mut scores = Vec::new();
    for i in 0..60u64 {
        let audio = air(&[], 1.0, 0.0, Profile::Awgn, 0.0, 95_000 + i);
        let mut r = MfskRx::new(CENTER);
        r.expect(expect_for(Rx::SlotHeader));
        let mut got = Vec::new();
        for c in audio.chunks(4800) {
            r.process(c, &mut got);
        }
        if r.stats().last_known_score != 0.0 {
            scores.push(r.stats().last_known_score);
        }
    }
    scores.sort_by(f32::total_cmp);
    if !scores.is_empty() {
        println!(
            "expected-header score on empty slots ({} tested): min {:.1}, median {:.1}, max {:.1} (pass mark {EXPECT_MIN_SCORE})",
            scores.len(),
            scores[0],
            scores[scores.len() / 2],
            scores[scores.len() - 1]
        );
    }
    for snr in [-17.0f32, -16.0, -15.0, -14.0, -12.0] {
        let other = par(n / 4, |i| {
            let a = checkin_audio(OTHER_NET, ROSTER[i % 20], i as u64);
            let audio = air(&a, power(&a), 0.0, Profile::Awgn, snr, 97_000 + i as u64);
            let wrong = our_checkin(receive(&audio, Rx::SlotHeader)).is_some() as usize;
            Cell {
                n: 1,
                wrong,
                ..Cell::default()
            }
        });
        println!(
            "another net's check-in at {snr} dB: taken for ours {} of {}",
            other.wrong, other.n
        );
    }
}

/// `TABLE_PROFILES` (channel names as printed, e.g. `AWGN,CCIR poor`) and
/// `TABLE_SNR_MIN` / `TABLE_SNR_MAX` run part of a table; every check-in
/// keeps its seed.
fn table_profile(name: &str) -> bool {
    std::env::var("TABLE_PROFILES").map_or(true, |v| v.split(',').any(|p| p.trim() == name))
}

fn table_snr(s: i32) -> bool {
    let env = |k: &str, d: i32| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
    (env("TABLE_SNR_MIN", i32::MIN)..=env("TABLE_SNR_MAX", i32::MAX)).contains(&s)
}

// ---------------------------------------------------------------------
// Polled replies (wired into the net as `NetConfig::prime`): NCS polls
// member SLOT at the floor and knows when its answer starts. The answer is
// an Idle (header-only; its SNR byte unknown) or one short frame (its
// "more" flag unknown); a report frame's layout is known too.
// ---------------------------------------------------------------------

const SLOT: u8 = 5;
const BURST: u8 = 2;

/// Member `slot`'s answer: an Idle, or a report listing up to 4 stations
/// (in a one-frame burst, "more" set or not).
fn reply_audio(slot: u8, idle: bool, seed: u64) -> (Vec<f32>, Option<Vec<u8>>) {
    let snr = (seed % 30) as f32 - 15.0;
    if idle {
        let (user, extra) = idle_header(NET, slot, snr);
        return (modulate(MfskMode::R0, user, extra, &[], CENTER), None);
    }
    let hears = (seed % 5) as usize;
    let mut f = Frame::decode(&report_frame_bytes(slot, hears)).unwrap();
    for (i, b) in f.payload.iter_mut().enumerate().skip(2) {
        *b = (seed.wrapping_mul(0x9E37_79B9).wrapping_add(i as u64 * 77) >> 7) as u8;
    }
    let bytes = f.encode(32);
    let more = (seed / 5 % 2) as u8;
    let user = user_bits(0, NET, BURST | more << 2);
    let bits = fec::encode_short(&bytes_to_bits(&bytes));
    (modulate(MfskMode::R0, user, 0, &bits, CENTER), Some(bytes))
}

fn reply_expect(score: f32) -> Expect {
    let (idle, data) = floor_reply_headers(NET, SLOT, Some(BURST));
    Expect {
        start_sample: (LEAD_S * AUDIO_RATE) as u64 + CH_DELAY,
        tol_samples: (TOL_S * AUDIO_RATE) as u64,
        header: Some(idle),
        alt: Some(data),
        min_score: score,
        min_margin: EXPECT_MIN_MARGIN,
    }
}

fn receive_reply(audio: &[f32], e: Option<Expect>) -> (Vec<MfskBurst>, MfskRx) {
    let mut r = MfskRx::new(CENTER);
    r.expect(e);
    let mut got = Vec::new();
    for c in audio.chunks(4800) {
        r.process(c, &mut got);
    }
    (got, r)
}

/// The answer as NCS takes it: (header right, data decoded blind, data
/// decoded blind or with the report priors, a wrong slot taken).
fn take_reply(got: &[MfskBurst], idle: bool, bytes: &Option<Vec<u8>>) -> (bool, bool, bool, bool) {
    let (iu, _) = idle_header(NET, SLOT, 0.0);
    let mut out = (false, false, false, false);
    for b in got {
        if idle && b.user == iu {
            if b.extra & 63 == SLOT as u16 {
                out.0 = true;
            } else {
                out.3 = true;
            }
        }
        if !idle && b.user & 0x0FFF == user_bits(0, NET, 0) && b.llrs.len() == fec::n_short() {
            out.0 = true;
            let want = bytes.as_ref().unwrap();
            let dec = |prior: &[f32]| {
                decode_short_frames_prior(b, prior)
                    .into_iter()
                    .next()
                    .flatten()
                    .map(|x| bits_to_bytes(&x))
            };
            out.1 = dec(&[]).as_ref() == Some(want);
            out.2 = out.1
                || (0..=4)
                    .any(|h| dec(&report_prior(SLOT, h)).is_some_and(|x| Frame::decode(&x).is_some() && x == *want));
        }
    }
    out
}

#[test]
fn a_polled_reply_in_its_slot() {
    for (i, idle) in [true, false].into_iter().enumerate() {
        let (a, bytes) = reply_audio(SLOT, idle, 3 + i as u64);
        let audio = air(&a, power(&a), 0.01, Profile::Awgn, 0.0, 11 + i as u64);
        for e in [None, Some(reply_expect(EXPECT_MIN_SCORE))] {
            let (got, _) = receive_reply(&audio, e);
            let r = take_reply(&got, idle, &bytes);
            assert!(r.0 && (idle || r.1) && !r.3, "idle {idle}: {r:?}");
        }
    }
}

/// Another member's Idle (it differs from the expected one only in the
/// slot bits of `extra`) is not taken for the polled member's, even with
/// the score's pass mark dropped to 0: the margin over every other
/// `extra` stops it.
#[test]
fn another_members_idle_is_not_ours() {
    for seed in 0..10u64 {
        let snr = -17.0 + (seed % 4) as f32;
        let (a, _) = reply_audio(SLOT + 1, true, seed);
        let audio = air(&a, power(&a), 0.0, Profile::Awgn, snr, 300 + seed);
        let (got, _) = receive_reply(&audio, Some(reply_expect(0.0)));
        let (iu, _) = idle_header(NET, SLOT, 0.0);
        assert!(
            !got.iter().any(|b| b.user == iu && b.extra & 63 == SLOT as u16),
            "seed {seed}: taken for ours"
        );
    }
}

/// CPU time of the expected-header test's margin pass (every other
/// header encoded and scored), against plain reception of the same audio.
#[test]
#[ignore]
fn margin_test_cost() {
    let a = checkin_audio(OTHER_NET, "AG5B", 1);
    let audio = air(&a, power(&a), 0.0, Profile::Awgn, -16.0, 201);
    let time = |e: Option<Expect>| {
        let t = std::time::Instant::now();
        let (_, r) = receive_reply(&audio, e);
        (t.elapsed().as_secs_f64(), r.stats().last_known_margin)
    };
    let mut e = expect_for(Rx::SlotHeader).unwrap();
    e.min_score = f32::MIN;
    let blind = time(None);
    let checkin = time(Some(e));
    let (ia, _) = reply_audio(SLOT + 1, true, 1);
    let iaudio = air(&ia, power(&ia), 0.0, Profile::Awgn, -16.0, 202);
    let t = std::time::Instant::now();
    let (_, r) = receive_reply(&iaudio, Some(reply_expect(f32::MIN)));
    let idle = (t.elapsed().as_secs_f64(), r.stats().last_known_margin);
    println!("blind {blind:?} s; check-in margin pass {checkin:?}; Idle (users and extras) {idle:?}");
}

#[test]
fn empty_reply_slots_stay_empty() {
    let mut false_bursts = 0;
    for seed in 0..20u64 {
        let audio = air(&[], 1.0, 0.0, Profile::Awgn, 0.0, 700 + seed);
        false_bursts += receive_reply(&audio, Some(reply_expect(EXPECT_MIN_SCORE))).0.len();
    }
    assert_eq!(false_bursts, 0);
}

#[derive(Default, Clone, Copy)]
struct ReplyCell {
    /// Header right, data decoded: [blind, primed]. Wrong slot taken.
    hdr: [usize; 2],
    data: [usize; 2],
    wrong: usize,
}

fn reply_trial(profile: Profile, snr: f32, idle: bool, i: usize) -> ReplyCell {
    let seed = 9100 * (snr as i64 + 100) as u64 + i as u64 + if idle { 0 } else { 50_000 };
    let (a, bytes) = reply_audio(SLOT, idle, seed);
    let err = (2.0 * rng_unit(seed) - 1.0) * ERR_S;
    let audio = air(&a, power(&a), err, profile, snr, seed);
    let mut c = ReplyCell::default();
    for (k, e) in [None, Some(reply_expect(EXPECT_MIN_SCORE))].into_iter().enumerate() {
        let r = take_reply(&receive_reply(&audio, e).0, idle, &bytes);
        c.hdr[k] = r.0 as usize;
        c.data[k] = if k == 0 { r.1 } else { r.2 } as usize;
        c.wrong += r.3 as usize;
    }
    c
}

fn reply_cell(profile: Profile, snr: f32, idle: bool, n: usize) -> ReplyCell {
    let threads = std::env::var("ION_THREADS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(2usize)
        .max(1);
    let cells: Vec<ReplyCell> = std::thread::scope(|s| {
        let hs: Vec<_> = (0..threads)
            .map(|t| {
                s.spawn(move || {
                    (t..n)
                        .step_by(threads)
                        .map(|i| reply_trial(profile, snr, idle, i))
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        hs.into_iter().flat_map(|h| h.join().unwrap()).collect()
    });
    cells.into_iter().fold(ReplyCell::default(), |a, c| ReplyCell {
        hdr: [a.hdr[0] + c.hdr[0], a.hdr[1] + c.hdr[1]],
        data: [a.data[0] + c.data[0], a.data[1] + c.data[1]],
        wrong: a.wrong + c.wrong,
    })
}

#[test]
#[ignore]
fn slotted_reply_table() {
    let n: usize = std::env::var("SLOTTED_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(40);
    let grid = [
        (Profile::Awgn, "AWGN", -18..=-11),
        (Profile::CcirPoor, "CCIR poor", -16..=-9),
        (Profile::MidDisturbedNvis, "NVIS disturbed", -14..=-8),
    ];
    for (profile, name, snrs) in grid {
        println!(
            "\n{name}, {n} replies per SNR and kind, slot ±{:.0} ms, real error ±{:.0} ms\n",
            TOL_S * 1e3,
            ERR_S * 1e3
        );
        println!("| SNR dB | Idle: blind | Idle: primed | report: blind | report: primed | wrong |");
        println!("|---:|---:|---:|---:|---:|---:|");
        for snr in snrs {
            let i = reply_cell(profile, snr as f32, true, n);
            let d = reply_cell(profile, snr as f32, false, n);
            let f = |x: usize| x as f32 / n as f32;
            println!(
                "| {snr} | {:.2} | {:.2} | {:.2} | {:.2} | {} |",
                f(i.hdr[0]),
                f(i.hdr[1]),
                f(d.data[0]),
                f(d.data[1]),
                i.wrong + d.wrong
            );
        }
    }
}
