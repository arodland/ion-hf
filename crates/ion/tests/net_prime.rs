//! Net mode with NCS's receivers primed (`NetConfig::prime`), over real
//! modulated audio. Every station hears the sum of the others through its
//! own path (gain, optionally a Watterson channel) plus its own noise.
//!
//! The tables come from the ignored generators:
//!
//! cargo test -p ion --release --test net_prime -- --ignored --nocapture --test-threads 1

use ion::fec;
use ion::link::audio::bytes_to_bits;
use ion::link::net::{Caps, NetConfig, NetEvent, NetNode, checkin_frame_bytes, checkin_user, net_id};
use ion::phy_mfsk::{MfskMode, modulate};
use ion_dsp::channel::{Channel, Profile, add_awgn};
use ion_dsp::dsp::{NoiseGen, noise_sigma};

const CHUNK: usize = 480;
const FS: f64 = 48_000.0;
const NET: &str = "TNARES";
const CENTER: f32 = 1600.0;
const SIGMA: f32 = 0.02;

const ROSTER: [&str; 20] = [
    "N0JLF", "AG5B", "W1AW", "K4XYZ", "N5ABC", "KD9QRS", "W7DEF", "KB0GHI", "AE4JKL", "N8MNO", "K0PQR", "W5STU",
    "KC3VWX", "N2YZA", "AB1BCD", "K6EFG", "W9HIJ", "KE7KLM", "N4NOP", "AD0QRS",
];

/// Mean power of an R0 burst (constant envelope, so any one will do).
fn r0_power() -> f32 {
    static P: std::sync::OnceLock<f32> = std::sync::OnceLock::new();
    *P.get_or_init(|| {
        let a = modulate(MfskMode::R0, 0, 0, &[], CENTER);
        a.iter().map(|x| x * x).sum::<f32>() / a.len() as f32
    })
}

/// Path gain that puts an R0 burst at `snr_db` against noise `SIGMA`.
fn gain_for(snr_db: f32) -> f32 {
    SIGMA / noise_sigma(r0_power(), snr_db, 2400.0, FS as f32)
}

struct Air {
    nodes: Vec<NetNode>,
    out: Vec<Vec<f32>>,
    events: Vec<Vec<NetEvent>>,
    /// Amplitude from `[from][to]`, and a fading channel on some paths.
    gain: Vec<Vec<f32>>,
    chan: Vec<Vec<Option<Channel>>>,
    noise: NoiseGen,
    t: f64,
    /// Audio played into one station's ears from time `.0`: (to, start, audio).
    inject: Vec<(usize, f64, Vec<f32>)>,
}

impl Air {
    fn new(cfgs: Vec<NetConfig>, seed: u64) -> Air {
        let n = cfgs.len();
        Air {
            nodes: cfgs.into_iter().map(|c| NetNode::new(c, 0.2)).collect(),
            out: vec![Vec::new(); n],
            events: vec![Vec::new(); n],
            gain: vec![vec![1.0; n]; n],
            chan: (0..n).map(|_| (0..n).map(|_| None).collect()).collect(),
            noise: NoiseGen::new(seed),
            t: 0.0,
            inject: Vec::new(),
        }
    }

    fn step(&mut self) {
        self.t += CHUNK as f64 / FS;
        let t = self.t;
        let n = self.nodes.len();
        let mut played = vec![vec![0.0f32; CHUNK]; n];
        for (i, p) in played.iter_mut().enumerate() {
            if self.out[i].is_empty()
                && let Some(a) = self.nodes[i].poll(t)
            {
                self.out[i].extend(a);
            }
            let k = self.out[i].len().min(CHUNK);
            p[..k].copy_from_slice(&self.out[i][..k]);
            self.out[i].drain(..k);
            if k > 0 && self.out[i].is_empty() {
                self.nodes[i].tx_done(t);
            }
        }
        for i in 0..n {
            let mut heard = vec![0.0f32; CHUNK];
            for (j, p) in played.iter().enumerate() {
                let g = self.gain[j][i];
                if j == i || g <= 0.0 {
                    continue;
                }
                match self.chan[j][i].as_mut() {
                    Some(ch) => {
                        let mut faded = vec![0.0f32; CHUNK];
                        ch.process(p, &mut faded);
                        for (h, x) in heard.iter_mut().zip(&faded) {
                            *h += g * x;
                        }
                    }
                    None => {
                        for (h, x) in heard.iter_mut().zip(p) {
                            *h += g * x;
                        }
                    }
                }
            }
            for (to, start, a) in &self.inject {
                if *to != i {
                    continue;
                }
                let first = ((t - CHUNK as f64 / FS - start) * FS).round() as i64;
                for (k, h) in heard.iter_mut().enumerate() {
                    let s = first + k as i64;
                    if s >= 0 && (s as usize) < a.len() {
                        *h += a[s as usize];
                    }
                }
            }
            add_awgn(&mut heard, SIGMA, &mut self.noise);
            self.nodes[i].push_audio(&heard, t);
            let ev = self.nodes[i].station_mut().take_events();
            self.events[i].extend(ev);
        }
    }

    fn run_until(&mut self, secs: f64, mut done: impl FnMut(&Air) -> bool) -> bool {
        while self.t < secs {
            if done(self) {
                return true;
            }
            self.step();
        }
        done(self)
    }
}

fn checked_in(ev: &[NetEvent]) -> Vec<String> {
    ev.iter()
        .filter_map(|e| match e {
            NetEvent::CheckedIn { call, via: None, .. } => Some(call.clone()),
            _ => None,
        })
        .collect()
}

fn ncs_cfg(prime: bool) -> NetConfig {
    let mut c = NetConfig::new("N0NCS", NET, true);
    c.prime = prime;
    c.window_slots = 1;
    c.regulars = ROSTER.iter().map(|s| s.to_string()).collect();
    c
}

#[derive(Default, Clone, Copy, Debug)]
struct Outcome {
    n: usize,
    ok: usize,
    wrong: usize,
    /// Check-ins that decoded only with a prior; bursts NCS's receiver
    /// took on the expected-header test.
    prior: usize,
    known: usize,
}

fn add(a: Outcome, b: Outcome) -> Outcome {
    Outcome {
        n: a.n + b.n,
        ok: a.ok + b.ok,
        wrong: a.wrong + b.wrong,
        prior: a.prior + b.prior,
        known: a.known + b.known,
    }
}

/// One beacon, one check-in slot, one member: does NCS take the check-in?
/// NCS hears the member at `snr` (R0 burst power) through `profile`; the
/// member hears NCS at +10 dB (it always hears the beacon).
fn checkin_trial(prime: bool, profile: Profile, snr: f32, seed: u64) -> Outcome {
    let call = ROSTER[seed as usize % ROSTER.len()];
    let mut m = NetConfig::new(call, NET, false);
    m.seed = seed.wrapping_mul(7919) | 1;
    let mut air = Air::new(vec![ncs_cfg(prime), m], seed ^ 0xC0FFEE);
    air.gain[0][1] = gain_for(10.0);
    air.gain[1][0] = gain_for(snr);
    if profile != Profile::Awgn {
        air.chan[1][0] = Some(Channel::from_profile(profile, seed ^ 0x77));
    }
    air.run_until(22.0, |a| !checked_in(&a.events[0]).is_empty());
    let got = checked_in(&air.events[0]);
    let st = air.nodes[0].station().prime_stats.clone();
    Outcome {
        n: 1,
        ok: got.iter().any(|c| c == call) as usize,
        wrong: got.iter().filter(|c| *c != call).count(),
        prior: st.prior_checkins as usize,
        known: air.nodes[0].mfsk_stats().known_headers as usize,
    }
}

/// A check-in burst of net `net` from `call`, as audio.
fn checkin_audio(net: &str, call: &str) -> Vec<f32> {
    let info = bytes_to_bits(&checkin_frame_bytes(call, 3.0, Caps::default(), 0x01));
    modulate(
        MfskMode::R0,
        checkin_user(net_id(net)),
        0,
        &fec::encode_short(&info),
        CENTER,
    )
}

/// NCS alone, its check-in slot holding noise (`foreign` None) or another
/// net's check-in at `snr` placed where NCS expects ours. Returns the
/// check-ins NCS took and the bursts its receiver took on the
/// expected-header test.
fn false_trial(foreign: Option<f32>, seed: u64) -> Outcome {
    let mut air = Air::new(vec![ncs_cfg(true)], seed ^ 0xFA15E);
    // Find the slot: run until the beacon is out and the slot is known.
    air.run_until(30.0, |a| {
        a.nodes[0]
            .station()
            .expectation(a.t)
            .is_some_and(|e| e.header.is_some())
    });
    let slot = air.nodes[0].station().expectation(air.t).map(|e| e.start_s);
    if let (Some(snr), Some(start)) = (foreign, slot) {
        let call = ROSTER[seed as usize % ROSTER.len()];
        let a: Vec<f32> = checkin_audio("OTHERNET", call)
            .iter()
            .map(|x| x * gain_for(snr))
            .collect();
        let err = ((seed % 7) as f64 - 3.0) * 0.01;
        air.inject.push((0, start + err, a));
    }
    air.run_until(22.0, |_| false);
    let got = checked_in(&air.events[0]);
    Outcome {
        n: 1,
        ok: 0,
        wrong: got.len(),
        prior: air.nodes[0].station().prime_stats.prior_checkins as usize,
        known: air.nodes[0].mfsk_stats().known_headers as usize,
    }
}

fn par<F: Fn(usize) -> Outcome + Sync>(n: usize, f: F) -> Outcome {
    let threads: usize = std::env::var("ION_THREADS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(2)
        .max(1);
    std::thread::scope(|s| {
        let f = &f;
        let hs: Vec<_> = (0..threads)
            .map(|t| s.spawn(move || (t..n).step_by(threads).map(f).fold(Outcome::default(), add)))
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).fold(Outcome::default(), add)
    })
}

/// A primed NCS learns where check-ins and replies really start, and they
/// fall well inside its tolerance; members check in and report as before.
#[test]
fn primed_bursts_land_in_their_slots() {
    let mut air = Air::new(
        vec![NetConfig::new("N0NCS", NET, true), NetConfig::new("AG5B", NET, false)],
        5,
    );
    air.gain[0][1] = gain_for(10.0);
    air.gain[1][0] = gain_for(-8.0);
    air.nodes[1].station_mut().send(b"AG5B: all well");
    air.nodes[1].station_mut().send(b"AG5B: generator test complete");
    let ok = air.run_until(400.0, |a| {
        a.events[0]
            .iter()
            .filter(|e| matches!(e, NetEvent::Message { overheard: false, .. }))
            .count()
            >= 2
            && a.nodes[0].station().prime_stats.offsets.iter().filter(|o| !o.0).count() >= 4
    });
    let st = air.nodes[0].station().prime_stats.clone();
    assert!(ok, "after {:.0} s: {:#?}\n{st:?}", air.t, air.events[0]);
    let tol = NetConfig::new("X", NET, true).slot_tol_s as f32;
    assert!(st.offsets.iter().any(|o| o.0), "no check-in timed");
    // The first reply teaches NCS the offset; every later one lands well
    // inside the tolerance.
    let replies: Vec<f32> = st.offsets.iter().filter(|o| !o.0).map(|o| o.2).collect();
    assert!(replies[1..].iter().all(|r| r.abs() < tol / 2.0), "{replies:?}");
    assert!(
        st.offsets.iter().filter(|o| o.0).all(|o| o.2.abs() < tol),
        "{:?}",
        st.offsets
    );
}

/// At −15 dB on AWGN an unprimed NCS takes almost no check-ins; a primed
/// one takes most.
#[test]
fn a_primed_ncs_takes_weak_check_ins() {
    let (mut plain, mut primed) = (Outcome::default(), Outcome::default());
    for seed in 0..4u64 {
        plain = add(plain, checkin_trial(false, Profile::Awgn, -15.0, 100 + seed));
        primed = add(primed, checkin_trial(true, Profile::Awgn, -15.0, 100 + seed));
    }
    assert_eq!(primed.wrong + plain.wrong, 0);
    assert!(
        primed.ok > plain.ok && primed.ok >= 2,
        "unprimed {plain:?}, primed {primed:?}"
    );
}

/// Another net's check-in in our slot is never taken for ours.
#[test]
fn another_nets_checkin_in_our_slot_is_not_taken() {
    for seed in 0..3u64 {
        let o = false_trial(Some(-15.0), seed);
        assert_eq!(o.wrong, 0, "seed {seed}: {o:?}");
    }
}

/// `n` members (each heard by NCS at `snr`, R0 burst power, AWGN; hearing
/// NCS at +10 dB) all want in. Returns the seconds until every one has
/// joined (`None`: not within `t_max`), and how many came in by chorus.
fn join_trial(chorus: bool, n: usize, snr: f32, seed: u64, t_max: f64) -> (Option<f64>, u32) {
    let mut ncs = NetConfig::new("N0NCS", NET, true);
    if chorus {
        ncs.window_slots = 1;
        ncs.chorus_slots = 2;
    }
    let mut cfgs = vec![ncs];
    for i in 0..n {
        // Each its own random choices (seeded from its call, as on the air).
        let mut m = NetConfig::new(ROSTER[(seed as usize + i) % ROSTER.len()], NET, false);
        m.caps.chorus = chorus;
        cfgs.push(m);
    }
    let mut air = Air::new(cfgs, seed ^ 0x1D);
    for i in 1..=n {
        air.gain[0][i] = gain_for(10.0);
        air.gain[i][0] = gain_for(snr);
    }
    let joined = |a: &Air| {
        a.events[1..]
            .iter()
            .all(|e| e.iter().any(|x| matches!(x, NetEvent::Joined { .. })))
    };
    let ok = air.run_until(t_max, joined);
    (ok.then_some(air.t), air.nodes[0].station().prime_stats.chorus_checkins)
}

/// Members that can, check in by chorus: all of them in fewer, shorter
/// windows.
#[test]
fn members_check_in_by_chorus() {
    let (t, by_chorus) = join_trial(true, 3, 0.0, 3, 300.0);
    assert!(t.is_some() && by_chorus >= 3, "{t:?} {by_chorus}");
}

/// A member with nothing to say, at the floor, polled round after round:
/// NCS hears it at −8 dB until it is in, then at `snr`. Returns (polls
/// after that answered, polls it missed).
fn poll_trial(prime: bool, snr: f32, seed: u64, secs: f64) -> (u32, u32) {
    let mut ncs = NetConfig::new("N0NCS", NET, true);
    ncs.prime = prime;
    ncs.idle_round_gap_s = 5.0;
    let m = NetConfig::new(ROSTER[seed as usize % ROSTER.len()], NET, false);
    let mut air = Air::new(vec![ncs, m], seed ^ 0x9011);
    air.gain[0][1] = gain_for(10.0);
    air.gain[1][0] = gain_for(-8.0);
    // In, and answered one poll at −8 dB (NCS has learned its timing).
    let ready = air.run_until(200.0, |a| {
        a.nodes[0].station().stats.replies >= 1 && a.events[0].iter().any(|e| matches!(e, NetEvent::CheckedIn { .. }))
    });
    if !ready {
        return (0, 0);
    }
    air.gain[1][0] = gain_for(snr);
    let replies0 = air.nodes[0].station().stats.replies;
    let missed0 = air.events[0]
        .iter()
        .filter(|e| matches!(e, NetEvent::Missed { .. }))
        .count();
    let end = air.t + secs;
    air.run_until(end, |_| false);
    let replies = air.nodes[0].station().stats.replies - replies0;
    let missed = air.events[0]
        .iter()
        .filter(|e| matches!(e, NetEvent::Missed { .. }))
        .count()
        - missed0;
    (replies, missed as u32)
}

#[test]
#[ignore]
fn net_poll_table() {
    let trials: usize = std::env::var("NET_PRIME_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(6);
    println!("| SNR dB | turns answered: unprimed | primed |");
    println!("|---:|---:|---:|");
    for snr in [-17.0f32, -16.0, -15.0, -14.0, -13.0] {
        let mut row = Vec::new();
        for prime in [false, true] {
            let threads: usize = std::env::var("ION_THREADS")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(2);
            let runs: Vec<(u32, u32)> = std::thread::scope(|s| {
                let hs: Vec<_> = (0..threads)
                    .map(|t| {
                        s.spawn(move || {
                            (t..trials)
                                .step_by(threads)
                                .map(|i| poll_trial(prime, snr, 700 + i as u64, 150.0))
                                .collect::<Vec<_>>()
                        })
                    })
                    .collect();
                hs.into_iter().flat_map(|h| h.join().unwrap()).collect()
            });
            let ans: u32 = runs.iter().map(|r| r.0).sum();
            let missed: u32 = runs.iter().map(|r| r.1).sum();
            row.push(format!(
                "{:.2} ({ans}/{})",
                ans as f32 / (ans + missed).max(1) as f32,
                ans + missed
            ));
        }
        println!("| {snr} | {} | {} |", row[0], row[1]);
    }
}

#[test]
#[ignore]
fn net_join_table() {
    let trials: usize = std::env::var("NET_PRIME_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(6);
    println!("| members | SNR dB | window | all joined | median s | mean s | slowest s | chorus check-ins |");
    println!("|---:|---:|---|---:|---:|---:|---:|---:|");
    for (n, snr) in [(5usize, 0.0f32), (5, -6.0)] {
        for chorus in [false, true] {
            let runs: Vec<(Option<f64>, u32)> = {
                let threads: usize = std::env::var("ION_THREADS")
                    .ok()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(2);
                std::thread::scope(|s| {
                    let hs: Vec<_> = (0..threads)
                        .map(|t| {
                            s.spawn(move || {
                                (t..trials)
                                    .step_by(threads)
                                    .map(|i| join_trial(chorus, n, snr, 900 + i as u64, 600.0))
                                    .collect::<Vec<_>>()
                            })
                        })
                        .collect();
                    hs.into_iter().flat_map(|h| h.join().unwrap()).collect()
                })
            };
            let mut ts: Vec<f64> = runs.iter().filter_map(|r| r.0).collect();
            ts.sort_by(f64::total_cmp);
            let med = ts.get(ts.len() / 2).copied().unwrap_or(f64::NAN);
            let mean = ts.iter().sum::<f64>() / ts.len().max(1) as f64;
            println!(
                "| {n} | {snr} | {} | {}/{} | {med:.0} | {mean:.0} | {:.0} | {} |",
                if chorus { "1 slot + 2 chorus slots" } else { "4 slots" },
                ts.len(),
                trials,
                ts.last().copied().unwrap_or(f64::NAN),
                runs.iter().map(|r| r.1).sum::<u32>()
            );
        }
    }
}

#[test]
#[ignore]
fn net_checkin_table() {
    let n: usize = std::env::var("NET_PRIME_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(40);
    let only: Option<String> = std::env::var("NET_PRIME_PROFILE").ok();
    let grid = [
        (Profile::Awgn, "AWGN", -18..=-11),
        (Profile::CcirPoor, "CCIR poor", -16..=-9),
        (Profile::MidDisturbedNvis, "NVIS disturbed", -14..=-8),
    ];
    for (profile, name, snrs) in grid {
        if only.as_deref().is_some_and(|o| !name.eq_ignore_ascii_case(o)) {
            continue;
        }
        println!("\n{name}: one member checking in, {n} trials per SNR (same seeds unprimed and primed)\n");
        println!("| SNR dB | unprimed | primed | of which only with a prior | expected-header bursts | wrong |");
        println!("|---:|---:|---:|---:|---:|---:|");
        for snr in snrs {
            let seed0 = 10_000 * (snr as i64 + 100) as u64;
            let b = par(n, |i| checkin_trial(false, profile, snr as f32, seed0 + i as u64));
            let a = par(n, |i| checkin_trial(true, profile, snr as f32, seed0 + i as u64));
            let f = |x: usize| x as f32 / n as f32;
            println!(
                "| {snr} | {:.2} | {:.2} | {:.2} | {:.2} | {} |",
                f(b.ok),
                f(a.ok),
                f(a.prior),
                f(a.known),
                a.wrong + b.wrong
            );
        }
    }
}

#[test]
#[ignore]
fn net_false_checkin_table() {
    let n: usize = std::env::var("NET_PRIME_FA_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(100);
    let e = par(n, |i| false_trial(None, 50_000 + i as u64));
    println!(
        "empty slots: {} check-ins taken, {} expected-header bursts, in {} slots",
        e.wrong, e.known, e.n
    );
    for snr in [-17.0f32, -15.0, -12.0, -8.0] {
        let o = par(n / 2, |i| false_trial(Some(snr), 60_000 + i as u64));
        println!(
            "another net's check-in in our slot at {snr} dB: taken for ours {} of {} ({} expected-header bursts)",
            o.wrong, o.n, o.known
        );
    }
}
