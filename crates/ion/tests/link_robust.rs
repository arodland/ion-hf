//! The link under lost bursts and lopsided paths (one station far louder
//! than the other): it must never stall with both ends waiting, must run
//! each direction at its own speed, and must close promptly on request.

use std::sync::Arc;

use ion::level::{Guard, LADDER, Wave};
use ion::link::sim::{Medium, ModelAir, ModelMedium, Sim};
use ion::link::{Air, Config, Incoming, Outgoing, State, Station};

/// Whether to drop a burst: (from, burst, nth burst from that station).
type Rule = Box<dyn FnMut(usize, &Outgoing, usize) -> bool>;

/// The model channel, with bursts dropped by a [`Rule`].
struct Dropping {
    inner: ModelMedium,
    sent: [usize; 2],
    rule: Rule,
}

impl Medium for Dropping {
    fn carry(&mut self, from: usize, burst: &Outgoing, start: f64, end: f64) -> Option<Incoming> {
        let n = self.sent[from];
        self.sent[from] += 1;
        let inc = self.inner.carry(from, burst, start, end);
        if (self.rule)(from, burst, n) { None } else { inc }
    }
}

fn pair(snr_db: f32, dir_db: [f32; 2], seed: u64, rule: impl FnMut(usize, &Outgoing, usize) -> bool + 'static) -> Sim {
    let air: Arc<dyn Air> = Arc::new(ModelAir::default());
    let a = Station::new(Config::new("N0ABC"), air.clone());
    let mut b = Station::new(Config::new("W1AW"), air.clone());
    b.listen();
    let mut m = ModelMedium::new(snr_db, seed);
    m.dir_db = dir_db;
    Sim::new(
        a,
        b,
        Box::new(Dropping {
            inner: m,
            sent: [0; 2],
            rule: Box::new(rule),
        }),
        air,
    )
}

fn payload(n: usize, seed: u32) -> Vec<u8> {
    let mut x = seed.wrapping_mul(2_654_435_761).max(1);
    (0..n)
        .map(|_| {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x as u8
        })
        .collect()
}

fn is_data(b: &Outgoing) -> bool {
    b.code < LADDER.len()
}

/// A sends `ab` bytes to B, B `ba` back (queued from the start, so the turn
/// changes hands); runs until both arrive or `t_max`.
fn exchange(sim: &mut Sim, ab: usize, ba: usize, t_max: f64) -> bool {
    sim.stations[1].send(&payload(ba, 2));
    sim.stations[0].connect("W1AW", 0.0);
    sim.stations[0].send(&payload(ab, 1));
    sim.run_until(t_max, |s| {
        s.stations[1].stats().bytes_received >= ab as u64 && s.stations[0].stats().bytes_received >= ba as u64
    })
}

fn states(sim: &Sim) -> String {
    let tail: Vec<String> = sim.log[sim.log.len().saturating_sub(24)..]
        .iter()
        .map(|e| {
            format!(
                "{:.0} {} {:?} c{} {}",
                e.start,
                "AB".as_bytes()[e.from] as char,
                e.wave,
                e.code,
                if e.delivered { "ok" } else { "LOST" }
            )
        })
        .collect();
    format!(
        "{}\nA level {} target {}\nt {:.0} s: A {:?} {:?} got {} B, B {:?} {:?} got {} B",
        tail.join("\n"),
        sim.stations[0].level(),
        sim.stations[0].target_level(),
        sim.t,
        sim.stations[0].state(),
        sim.stations[0].role(),
        sim.stations[0].stats().bytes_received,
        sim.stations[1].state(),
        sim.stations[1].role(),
        sim.stations[1].stats().bytes_received,
    )
}

#[test]
fn the_turn_survives_the_new_senders_first_burst_lost() {
    // B takes the turn and its first data burst never reaches A: A must
    // not keep repeating the Turn while B polls.
    let mut first = true;
    let mut sim = pair(15.0, [0.0; 2], 1, move |from, b, _| {
        let drop = from == 1 && is_data(b) && first;
        if drop {
            first = false;
        }
        drop
    });
    assert!(exchange(&mut sim, 4_000, 2_000, 900.0), "{}", states(&sim));
}

#[test]
fn every_burst_of_the_handover_lost_once_still_hands_over() {
    // Lose, once each, B's first data burst, A's first Turn and B's first
    // poll after taking over.
    let mut lost = [false; 3];
    let mut sim = pair(15.0, [0.0; 2], 2, move |from, b, _| {
        let kind = b.user >> 12;
        let which = match (from, b.code) {
            (1, c) if c < LADDER.len() => 0,
            (0, ion::link::station::TINY) if kind == 4 => 1,
            (1, ion::link::station::TINY) if kind == 3 => 2,
            _ => return false,
        };
        let drop = !lost[which];
        lost[which] = true;
        drop
    });
    assert!(exchange(&mut sim, 4_000, 2_000, 1_200.0), "{}", states(&sim));
}

#[test]
fn a_lopsided_link_runs_each_direction_at_its_own_speed() {
    // A loud (20 dB at B), B weak (−2 dB at A): A's data climbs into
    // full-band OFDM while B's stays at the slice or below.
    let mut sim = pair(9.0, [11.0, -11.0], 3, |_, _, _| false);
    assert!(exchange(&mut sim, 20_000, 1_500, 3_600.0), "{}", states(&sim));
    let top = |s: &Station| s.stats().levels.iter().copied().max().unwrap_or(0);
    let full_band = LADDER.iter().position(|l| l.wave == Wave::Ofdm(Guard::Normal)).unwrap();
    assert!(
        top(&sim.stations[0]) >= full_band,
        "A never reached full band: {:?}",
        sim.stations[0].stats().levels
    );
    assert!(
        top(&sim.stations[1]) < full_band,
        "B went too fast: {:?}",
        sim.stations[1].stats().levels
    );
}

#[test]
fn the_turn_report_starts_a_loud_new_sender_fast() {
    // A weak (−2 dB at B) hands over to loud B (20 dB at A): B's first
    // data burst goes at its own direction's speed, not A's.
    let mut sim = pair(9.0, [-11.0, 11.0], 4, |_, _, _| false);
    assert!(exchange(&mut sim, 300, 20_000, 3_600.0), "{}", states(&sim));
    let full_band = LADDER.iter().position(|l| l.wave == Wave::Ofdm(Guard::Normal)).unwrap();
    let b_first = sim.stations[1].stats().levels[0];
    assert!(b_first >= full_band, "B started at level {b_first}");
}

#[test]
fn random_losses_and_lopsided_paths_never_stall() {
    // Many links, each with its own SNR, lopsidedness and share of bursts
    // lost at random. Each must either deliver everything both ways or
    // end (failed or disconnected); none may sit with both ends connected
    // and no data moving (the deadlock: control bursts back and forth,
    // never a data burst).
    let runs = std::env::var("ION_ROBUST_RUNS").map_or(40, |v| v.parse().unwrap());
    let mut done = 0;
    let only: Option<usize> = std::env::var("ION_ROBUST_SEED").ok().map(|v| v.parse().unwrap());
    for seed in 0..runs {
        if only.is_some_and(|o| o != seed) {
            continue;
        }
        let mut x = (seed as u32).wrapping_mul(0x9E37_79B9) | 1;
        let mut rnd = move || {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x as f32 / u32::MAX as f32
        };
        let snr = 2.0 + 18.0 * rnd();
        let lop = -12.0 + 24.0 * rnd();
        let p = 0.4 * rnd();
        let mut y = (seed as u32).wrapping_mul(0x85EB_CA6B) | 1;
        let mut sim = pair(snr, [lop / 2.0, -lop / 2.0], seed as u64 + 100, move |_, _, _| {
            y ^= y << 13;
            y ^= y >> 17;
            y ^= y << 5;
            (y as f32 / u32::MAX as f32) < p
        });
        sim.stations[1].send(&payload(1_500, 2));
        sim.stations[0].connect("W1AW", 0.0);
        sim.stations[0].send(&payload(4_000, 1));
        let mut last = (0u64, State::Idle, State::Idle);
        let mut since = 0.0;
        let all =
            |s: &Sim| s.stations[1].stats().bytes_received >= 4_000 && s.stations[0].stats().bytes_received >= 1_500;
        let ended = |s: &Sim| {
            s.stations
                .iter()
                .any(|st| matches!(st.state(), State::Failed | State::Disconnected))
        };
        while sim.t < 7_200.0 && !all(&sim) && !ended(&sim) {
            let t0 = sim.t;
            sim.run_until(t0 + 1.0, |_| false);
            let now = (
                sim.stations
                    .iter()
                    .map(|s| s.stats().bytes_received + s.stats().data_bursts as u64)
                    .sum::<u64>(),
                sim.stations[0].state(),
                sim.stations[1].state(),
            );
            if now != last {
                if only.is_some() {
                    let st = |i: usize| {
                        let x = &sim.stations[i];
                        format!(
                            "{:?}/{:?} pend {} got {}",
                            x.state(),
                            x.role(),
                            x.pending(),
                            x.stats().bytes_received
                        )
                    };
                    eprintln!("{:.0}: A {} | B {}", sim.t, st(0), st(1));
                }
                last = now;
                since = sim.t;
            }
            assert!(
                sim.t - since < 400.0,
                "run {seed} (SNR {snr:.1} dB, lopsided {lop:.1} dB, loss {:.0}%) stalled: {}",
                p * 100.0,
                states(&sim)
            );
        }
        assert!(
            all(&sim) || ended(&sim),
            "run {seed} (SNR {snr:.1}, lopsided {lop:.1}, loss {:.0}%) unfinished: {}",
            p * 100.0,
            states(&sim)
        );
        if all(&sim) {
            assert_eq!(
                sim.stations[1].take_received(),
                payload(4_000, 1),
                "run {seed}: A to B corrupted"
            );
            assert_eq!(
                sim.stations[0].take_received(),
                payload(1_500, 2),
                "run {seed}: B to A corrupted"
            );
            done += 1;
        }
    }
    assert!(
        done * 10 >= runs * 8,
        "only {done} of {runs} links delivered everything"
    );
}

#[test]
fn disconnect_mid_transfer_is_prompt() {
    let mut sim = pair(15.0, [0.0; 2], 5, |_, _, _| false);
    sim.stations[0].connect("W1AW", 0.0);
    sim.stations[0].send(&payload(60_000, 1));
    assert!(sim.run_until(600.0, |s| s.stations[1].stats().bytes_received >= 5_000));
    let t0 = sim.t;
    sim.stations[0].disconnect();
    let ok = sim.run_until(t0 + 30.0, |s| {
        s.stations.iter().all(|st| st.state() == State::Disconnected)
    });
    assert!(ok, "{}", states(&sim));
}

#[test]
fn the_receiver_can_end_it_mid_transfer() {
    let mut sim = pair(15.0, [0.0; 2], 6, |_, _, _| false);
    sim.stations[0].connect("W1AW", 0.0);
    sim.stations[0].send(&payload(60_000, 1));
    assert!(sim.run_until(600.0, |s| s.stations[1].stats().bytes_received >= 5_000));
    let t0 = sim.t;
    sim.stations[1].disconnect();
    let ok = sim.run_until(t0 + 60.0, |s| {
        s.stations.iter().all(|st| st.state() == State::Disconnected)
    });
    assert!(ok, "{}", states(&sim));
}

#[test]
fn disconnect_closes_even_when_the_peer_cannot_answer() {
    // B hears A, A hears nothing of B: the goodbye goes unanswered, and A
    // closes after its tries anyway.
    let mut deaf = false;
    let mut sim = pair(15.0, [0.0; 2], 7, move |from, _, _| from == 1 && deaf);
    sim.stations[0].connect("W1AW", 0.0);
    sim.stations[0].send(&payload(60_000, 1));
    assert!(sim.run_until(600.0, |s| s.stations[1].stats().bytes_received >= 3_000));
    deaf = true;
    let _ = deaf;
    let t0 = sim.t;
    sim.medium = Box::new(Dropping {
        inner: ModelMedium::new(15.0, 7),
        sent: [0; 2],
        rule: Box::new(|from, _, _| from == 1),
    });
    sim.stations[0].disconnect();
    let ok = sim.run_until(t0 + 90.0, |s| s.stations[0].state() == State::Disconnected);
    assert!(ok, "{}", states(&sim));
}

/// A sends `n` bytes and the link goes idle (A the sender, nothing left).
fn settle(sim: &mut Sim, n: usize) {
    sim.stations[0].connect("W1AW", 0.0);
    sim.stations[0].send(&payload(n, 1));
    assert!(
        sim.run_until(1_800.0, |s| s.stations[1].stats().bytes_received >= n as u64),
        "{}",
        states(sim)
    );
    let t = sim.t;
    sim.run_until(t + 15.0, |_| false);
}

fn reply_time(sim: &mut Sim, t_max: f64) -> f64 {
    let t0 = sim.t;
    sim.stations[1].send(b"QSL, 73\n");
    assert!(
        sim.run_until(t0 + t_max, |s| s.stations[0].stats().bytes_received >= 8),
        "{}",
        states(sim)
    );
    sim.t - t0
}

#[test]
fn a_message_typed_on_the_receiving_side_goes_within_seconds() {
    let mut sim = pair(15.0, [0.0; 2], 11, |_, _, _| false);
    settle(&mut sim, 2_000);
    let dt = reply_time(&mut sim, 30.0);
    assert!(dt < 15.0, "{dt:.1} s");
}

#[test]
fn the_weak_side_of_a_lopsided_link_gets_its_message_through() {
    // B weak (−8 dB at A: its fast ACKs are lost, floor ones get
    // through), A loud: B's reply still goes promptly.
    let mut sim = pair(6.0, [14.0, -14.0], 12, |_, _, _| false);
    settle(&mut sim, 2_000);
    let dt = reply_time(&mut sim, 300.0);
    assert!(dt < 20.0, "{dt:.1} s");
}

#[test]
fn break_in_lets_a_reply_through_mid_file() {
    let mut sim = pair(15.0, [0.0; 2], 13, |_, _, _| false);
    sim.stations[0].connect("W1AW", 0.0);
    sim.stations[0].send(&payload(150_000, 1));
    assert!(sim.run_until(900.0, |s| s.stations[1].stats().bytes_received >= 10_000));
    let t0 = sim.t;
    sim.stations[1].send(b"QSL, 73\n");
    assert!(
        sim.run_until(t0 + 60.0, |s| s.stations[0].stats().bytes_received >= 8),
        "{}",
        states(&sim)
    );
    assert!(
        sim.stations[1].stats().bytes_received < 150_000,
        "the file finished first"
    );
    assert!(
        sim.run_until(3_600.0, |s| s.stations[1].stats().bytes_received >= 150_000),
        "{}",
        states(&sim)
    );
    assert_eq!(sim.stations[1].take_received(), payload(150_000, 1));
}

#[test]
fn text_progress_follows_each_message_to_delivery() {
    let mut sim = pair(15.0, [0.0; 2], 14, |_, _, _| false);
    sim.stations[0].connect("W1AW", 0.0);
    for i in 0..5 {
        sim.stations[0].send(&payload(400, i));
    }
    assert_eq!(sim.stations[0].text_progress(), (0, 0));
    assert!(
        sim.run_until(900.0, |s| s.stations[1].stats().bytes_received >= 2_000),
        "{}",
        states(&sim)
    );
    let t = sim.t;
    sim.run_until(t + 10.0, |_| false);
    assert_eq!(sim.stations[0].text_progress(), (2_000, 2_000));
}

#[test]
fn a_transfer_whose_acks_fail_fast_waveforms_still_flows() {
    // A loud, B's way back at −8 dB: ACKs must drop to what gets through
    // rather than be polled for burst after burst.
    let mut sim = pair(6.0, [14.0, -14.0], 15, |_, _, _| false);
    sim.stations[0].connect("W1AW", 0.0);
    sim.stations[0].send(&payload(30_000, 1));
    assert!(
        sim.run_until(3_600.0, |s| s.stations[1].stats().bytes_received >= 30_000),
        "{}",
        states(&sim)
    );
    assert!(sim.t < 65.0, "{:.0} s, {} polls", sim.t, sim.stations[0].stats().polls);
}
