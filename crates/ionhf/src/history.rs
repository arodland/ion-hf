//! What the screens plot and list: one link's levels, SNRs and data over
//! time, every burst sent and heard, and the stations heard calling.

use std::collections::VecDeque;
use std::time::Instant;

use crate::ion_engine::IonStatus;

/// Log rows kept.
const ROWS: usize = 400;
/// Stations kept in the Heard list.
const HEARD: usize = 40;

/// One burst, for the Log.
pub struct Row {
    /// Seconds into the link.
    pub t: f64,
    pub tx: bool,
    pub what: String,
    pub snr: Option<f32>,
    pub lost: u32,
    pub secs: f64,
}

/// A station heard calling (anyone's connect request), or a link peer.
pub struct Heard {
    pub call: String,
    /// Whom it was calling.
    pub to: String,
    pub snr: Option<f32>,
    pub at: Instant,
    pub count: u32,
}

pub struct History {
    clock: Instant,
    /// When this link began (clock seconds).
    link_t0: f64,
    /// Data bursts we sent and heard: (t, level).
    pub us_levels: Vec<(f64, f32)>,
    pub peer_levels: Vec<(f64, f32)>,
    /// SNR of each burst we heard, and as the peer reports hearing us.
    pub snr_here: Vec<(f64, f32)>,
    pub snr_there: Vec<(f64, f32)>,
    /// kB moved (acknowledged to us plus received), over time.
    pub data: Vec<(f64, f32)>,
    pub rows: VecDeque<Row>,
    pub heard: Vec<Heard>,
    pub last_here: Option<f32>,
    pub last_there: Option<f32>,
    /// Level of the peer's last data burst.
    pub peer_level: Option<usize>,
    pub bursts_heard: u32,
    last_bytes: u64,
}

impl History {
    pub fn new() -> Self {
        Self {
            clock: Instant::now(),
            link_t0: 0.0,
            us_levels: Vec::new(),
            peer_levels: Vec::new(),
            snr_here: Vec::new(),
            snr_there: Vec::new(),
            data: Vec::new(),
            rows: VecDeque::new(),
            heard: Vec::new(),
            last_here: None,
            last_there: None,
            peer_level: None,
            bursts_heard: 0,
            last_bytes: 0,
        }
    }

    /// Seconds into the current link.
    pub fn t(&self) -> f64 {
        self.clock.elapsed().as_secs_f64() - self.link_t0
    }

    /// A new link: the charts and counters start again (the Heard list
    /// stays).
    pub fn new_link(&mut self) {
        self.link_t0 = self.clock.elapsed().as_secs_f64();
        self.us_levels.clear();
        self.peer_levels.clear();
        self.snr_here.clear();
        self.snr_there.clear();
        self.data.clear();
        self.rows.clear();
        self.last_here = None;
        self.last_there = None;
        self.peer_level = None;
        self.bursts_heard = 0;
        self.last_bytes = 0;
    }

    pub fn on_status(&mut self, s: &IonStatus) {
        let t = self.t();
        for b in &s.bursts {
            if let Some(l) = b.level {
                if b.tx {
                    self.us_levels.push((t, l as f32));
                } else {
                    self.peer_levels.push((t, l as f32));
                    self.peer_level = Some(l);
                }
            }
            if !b.tx {
                self.bursts_heard += 1;
                if let Some(snr) = b.snr_db {
                    self.snr_here.push((t, snr));
                    self.last_here = Some(snr);
                }
                if let Some(r) = b.reported_db {
                    self.snr_there.push((t, r));
                    self.last_there = Some(r);
                }
                if let Some((from, to)) = &b.calls {
                    self.heard_call(from, to, b.snr_db);
                }
            }
            if self.rows.len() >= ROWS {
                self.rows.pop_front();
            }
            self.rows.push_back(Row {
                t,
                tx: b.tx,
                what: b.what.clone(),
                snr: b.snr_db,
                lost: b.lost,
                secs: b.secs,
            });
        }
        let bytes = s.bytes_acked + s.bytes_received;
        if bytes != self.last_bytes {
            if self.data.is_empty() {
                self.data.push((t, self.last_bytes as f32 / 1000.0));
            }
            self.last_bytes = bytes;
            self.data.push((t, bytes as f32 / 1000.0));
        }
    }

    /// The link peer heard (keeps it at the top of Heard).
    pub fn heard_peer(&mut self, peer: &str, snr: Option<f32>) {
        if !peer.is_empty() {
            self.heard_call(peer, "", snr);
        }
    }

    fn heard_call(&mut self, call: &str, to: &str, snr: Option<f32>) {
        match self.heard.iter_mut().position(|h| h.call == call) {
            Some(i) => {
                let mut h = self.heard.remove(i);
                h.at = Instant::now();
                h.count += 1;
                if snr.is_some() {
                    h.snr = snr;
                }
                if !to.is_empty() {
                    h.to = to.to_string();
                }
                self.heard.insert(0, h);
            }
            None => {
                self.heard.insert(
                    0,
                    Heard {
                        call: call.to_string(),
                        to: to.to_string(),
                        snr,
                        at: Instant::now(),
                        count: 1,
                    },
                );
                self.heard.truncate(HEARD);
            }
        }
    }
}
