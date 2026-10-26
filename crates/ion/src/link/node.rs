//! One complete Ion station driven by audio: the link [`Station`], every
//! streaming receiver and the codecs. The host feeds it received 48 kHz
//! audio and plays the bursts it hands out.

use std::sync::Arc;

use super::audio::{self, PhyAir, Receivers};
use super::frame::{AckInfo, Frame, Kind, Tiny};
use super::station::{Air, CONTROL, Config, Incoming, State, Station, TINY, user_hi};
use crate::level::{LADDER, Wave};

/// One burst sent or heard, for the operator's log.
#[derive(Clone, Debug)]
pub struct BurstNote {
    /// We sent it.
    pub tx: bool,
    /// Ladder level, for a data burst.
    pub level: Option<usize>,
    /// "DATA L6 MFSK R0 ×7", "ACK 2 111111 +6 dB", "CONREQ A>B", ...
    pub what: String,
    /// Heard: its SNR, and its codewords that failed.
    pub snr_db: Option<f32>,
    pub lost: u32,
    /// Sent: seconds on the air, lead-in included.
    pub secs: f64,
    /// A connect request: (from, to).
    pub calls: Option<(String, String)>,
    /// An ACK: the SNR it reports (how its sender hears the other side).
    pub reported_db: Option<f32>,
}

/// Hears a CW ID on the Ion centre: per 20 ms, the strongest of three
/// bins about [`audio::CENTER_HZ`] against six well clear of it (a burst
/// fills them all alike, a keyed tone only the first).
struct CwSense {
    buf: Vec<f32>,
    /// One bit per window, newest lowest: the tone was there.
    hits: u16,
}

const CW_WIN: usize = 960;
/// Tone over the mean reference bin; noise alone passes ≈ 1.4e-4 a window.
const CW_RATIO: f32 = 10.0;

impl CwSense {
    fn new() -> Self {
        Self {
            buf: Vec::with_capacity(CW_WIN),
            hits: 0,
        }
    }

    fn power(x: &[f32], hz: f32) -> f32 {
        let w = std::f32::consts::TAU * hz / 48_000.0;
        let k = 2.0 * w.cos();
        let (mut s1, mut s2) = (0.0f32, 0.0f32);
        for &v in x {
            let s = v + k * s1 - s2;
            s2 = s1;
            s1 = s;
        }
        s1 * s1 + s2 * s2 - k * s1 * s2
    }

    /// Audio in; whether the peer's CW is on the air (two windows of the
    /// last fifteen, 0.3 s, so the gaps between characters count too).
    fn push(&mut self, samples: &[f32]) -> bool {
        for &v in samples {
            self.buf.push(v);
            if self.buf.len() == CW_WIN {
                let c = audio::CENTER_HZ;
                let tone = [-25.0, 0.0, 25.0]
                    .iter()
                    .map(|d| Self::power(&self.buf, c + d))
                    .fold(0.0, f32::max);
                let refs: f32 = [-700.0, -500.0, -300.0, 300.0, 500.0, 700.0]
                    .iter()
                    .map(|d| Self::power(&self.buf, c + d))
                    .sum();
                let hit = tone > CW_RATIO * refs / 6.0 && tone > 0.0;
                self.hits = self.hits << 1 | hit as u16;
                self.buf.clear();
            }
        }
        (self.hits & 0x7FFF).count_ones() >= 2
    }
}

/// A burst's log line, the calls of a connect request and the SNR an ACK
/// reports.
type Described = (String, Option<(String, String)>, Option<f32>);

/// What a burst is, from its header and (control bursts) first frame.
fn describe(code: usize, user: u16, extra: u16, first: Option<&[u8]>, frames: usize) -> Described {
    let mut reported = None;
    let what = match code {
        TINY => match Tiny::decode(user_hi(user), extra) {
            Some(Tiny::Ack {
                burst,
                bitmap,
                snr_db,
                want_turn,
                want_disc,
                stored,
                ..
            }) => {
                reported = Some(snr_db);
                format!(
                    "ACK {burst} {bitmap:06b} {snr_db:+.0} dB{}{}{}",
                    if stored { " HARQ" } else { "" },
                    if want_turn { " TURN?" } else { "" },
                    if want_disc { " DISC?" } else { "" },
                )
            }
            Some(Tiny::ConnAck { snr_db, .. }) => {
                reported = Some(snr_db);
                format!("CONACK {snr_db:+.0} dB")
            }
            Some(Tiny::Poll) => "POLL".into(),
            Some(Tiny::Turn) => "TURN".into(),
            Some(Tiny::Idle) => "IDLE".into(),
            Some(Tiny::DiscReq) => "DISC".into(),
            Some(Tiny::DiscAck) => "DISCACK".into(),
            None => "TINY".into(),
        },
        CONTROL => match first.and_then(Frame::decode) {
            Some(f) if f.kind == Kind::ConnReq => {
                let text: String = f
                    .payload
                    .iter()
                    .map(|&c| if c.is_ascii_graphic() { c as char } else { ' ' })
                    .collect();
                let mut it = text.split_whitespace();
                let (from, to) = (
                    it.next().unwrap_or("?").to_string(),
                    it.next().unwrap_or("?").to_string(),
                );
                return (format!("CONREQ {from}>{to}"), Some((from, to)), None);
            }
            Some(f) if f.kind == Kind::Ack => match AckInfo::from_bytes(&f.payload) {
                Some(a) => {
                    reported = Some(a.snr_db);
                    format!("ACK {} {} ok {:+.0} dB", a.burst, a.bitmap.count_ones(), a.snr_db)
                }
                None => "ACK".into(),
            },
            _ => "CTRL".into(),
        },
        // Received long-frame bursts arrive as soft bits: no count yet.
        l if frames == 0 => format!("DATA {}", crate::sounding::level_name(l, LADDER[l].wave)),
        l => format!("DATA {} ×{frames}", crate::sounding::level_name(l, LADDER[l].wave)),
    };
    (what, None, reported)
}

/// Default silence before each burst: covers the radio's PTT-to-RF delay so
/// the preamble is not clipped.
pub const DEFAULT_LEAD_S: f64 = 0.2;

/// Silence between a CW ID and the burst after it.
const ID_GAP_S: f64 = 0.3;

/// [`PhyAir`] plus the lead-in silence.
pub struct NodeAir {
    pub lead_s: f64,
}

impl Air for NodeAir {
    fn duration(&self, wave: Wave, code: usize, bits: usize) -> f64 {
        self.lead_s + PhyAir.duration(wave, code, bits)
    }

    fn max_bits(&self, wave: Wave, code: usize) -> usize {
        PhyAir.max_bits(wave, code)
    }
}

pub struct Node {
    ofdm_gain: f32,
    station: Station,
    rx: Receivers,
    bursts: Vec<Incoming>,
    lead: usize,
    /// Our burst is out (handed to the host and not yet finished).
    on_air: bool,
    /// SNR of the last burst we decoded.
    pub last_snr_db: Option<f32>,
    /// Bursts decoded, and their codewords that failed.
    pub bursts_heard: u32,
    pub codewords_failed: u32,
    journal: Vec<BurstNote>,
    cw: CwSense,
    /// Mid-link CW ID this often (None: never, as in simulation), and when
    /// we last sent one (or first keyed in this link).
    id_every_s: Option<f64>,
    id_at: Option<f64>,
}

impl Node {
    pub fn new(cfg: Config, lead_s: f64) -> Self {
        let air: Arc<dyn Air> = Arc::new(NodeAir { lead_s });
        Self {
            ofdm_gain: 1.0,
            station: Station::new(cfg, air),
            rx: Receivers::new(),
            bursts: Vec::new(),
            lead: (lead_s * 48_000.0) as usize,
            on_air: false,
            last_snr_db: None,
            bursts_heard: 0,
            codewords_failed: 0,
            journal: Vec::new(),
            cw: CwSense::new(),
            id_every_s: None,
            id_at: None,
        }
    }

    /// On the air: a CW ID ahead of our first burst after `every_s` since
    /// the last (or since we first keyed in this link). The peer's own
    /// waits stretch while they hear it ([`Station::peer_busy`]).
    pub fn set_id_every(&mut self, every_s: Option<f64>) {
        self.id_every_s = every_s;
    }

    /// Bursts sent and heard since the last call (at most the last 256).
    pub fn take_journal(&mut self) -> Vec<BurstNote> {
        std::mem::take(&mut self.journal)
    }

    fn note(&mut self, n: BurstNote) {
        if self.journal.len() >= 256 {
            self.journal.remove(0);
        }
        self.journal.push(n);
    }

    /// A station config sized for the real codes, for `call`.
    pub fn config(call: &str) -> Config {
        audio::config(call)
    }

    pub fn station(&self) -> &Station {
        &self.station
    }

    pub fn station_mut(&mut self) -> &mut Station {
        &mut self.station
    }

    /// Received audio, 48 kHz, ending at `now`. Ignored while our own burst
    /// is out (so we never decode ourselves).
    pub fn push_audio(&mut self, samples: &[f32], now: f64) {
        if self.on_air {
            return;
        }
        if self.cw.push(samples) {
            self.station.peer_busy(now);
        }
        self.bursts.clear();
        self.rx.process(samples, &mut self.bursts);
        for inc in std::mem::take(&mut self.bursts) {
            self.bursts_heard += 1;
            // Long-frame codewords are decoded (and counted) by the station.
            let before = self.station.stats().codewords_failed;
            self.last_snr_db = Some(inc.snr_db);
            self.station.on_rx(&inc, now);
            let long_failed = self.station.stats().codewords_failed - before;
            let short_failed = if inc.soft.is_some() {
                0
            } else {
                inc.frames.iter().filter(|f| f.is_none()).count() as u32
            };
            let lost = long_failed.max(short_failed);
            self.codewords_failed += lost;
            let first = inc.frames.first().and_then(|f| f.as_deref());
            let (what, calls, reported_db) = describe(inc.code, inc.user, inc.extra, first, inc.frames.len());
            self.note(BurstNote {
                tx: false,
                level: (inc.code < LADDER.len()).then_some(inc.code),
                what,
                snr_db: Some(inc.snr_db),
                lost,
                secs: 0.0,
                calls,
                reported_db,
            });
        }
    }

    /// Receiving a burst, or transmitting one: not the moment to key.
    pub fn busy(&self) -> bool {
        self.on_air || self.rx.busy()
    }

    /// The slot meter: the third of the passband our receivers would have
    /// slice bursts in (0 the centre, 1 below, 2 above) and each third's
    /// score (lower is cleaner; `crate::slots`).
    pub fn band(&self) -> (u8, [f32; 3]) {
        self.rx.slot()
    }

    /// Audio to transmit now, if the station has something to send. The
    /// host plays it and calls [`Node::tx_done`] when the last sample has
    /// gone out.
    pub fn poll(&mut self, now: f64) -> Option<Vec<f32>> {
        if self.busy() {
            return None;
        }
        // The ID clock is per link (the end-of-link ID is the host's).
        if !matches!(
            self.station.state(),
            State::Connecting | State::Connected | State::Disconnecting
        ) {
            self.id_at = None;
        }
        let b = self.station.poll_tx(now)?;
        let mut out = vec![0.0; self.lead];
        if let Some(every) = self.id_every_s {
            let since = *self.id_at.get_or_insert(now);
            if now - since >= every {
                self.id_at = Some(now);
                out.extend(crate::sounding::cw(&format!("DE {}", self.station.call())));
                out.extend(std::iter::repeat_n(0.0, (ID_GAP_S * 48_000.0) as usize));
                self.note(BurstNote {
                    tx: true,
                    level: None,
                    what: "CW ID".into(),
                    snr_db: None,
                    lost: 0,
                    secs: out.len() as f64 / 48_000.0,
                    calls: None,
                    reported_db: None,
                });
            }
        }
        out.extend(audio::transmit_with(&b, self.ofdm_gain));
        self.on_air = true;
        let first = b.frames.first().map(Vec::as_slice);
        let (what, calls, reported_db) = describe(b.code, b.user, b.extra, first, b.frames.len());
        self.note(BurstNote {
            tx: true,
            level: (b.code < LADDER.len()).then_some(b.code),
            what,
            snr_db: None,
            lost: 0,
            secs: out.len() as f64 / 48_000.0,
            calls,
            reported_db,
        });
        Some(out)
    }

    /// Extra level for OFDM bursts, dB ([`audio::ofdm_gain`]): on a real
    /// transmitter, so OFDM reads the same power as the steady carriers;
    /// 0 (the default) in simulation.
    pub fn set_ofdm_gain_db(&mut self, db: f32) {
        self.ofdm_gain = audio::ofdm_gain(db);
    }

    pub fn tx_done(&mut self, now: f64) {
        if self.on_air {
            self.on_air = false;
            self.station.on_tx_done(now);
        }
    }

    pub fn on_air(&self) -> bool {
        self.on_air
    }
}
