//! Messages cut into pieces and put back together, each one on its own.

use super::*;

/// Most bytes in one message: its length prefix and every offset in it fit
/// in 16 bits.
pub const MAX_MSG: usize = 0xFFFF - 2;

/// The field of an uplink frame: message number and offset in the message.
pub(super) fn msg_field(seq: u32, off: usize) -> u32 {
    (seq & 0xFF) << 16 | off as u32 & 0xFFFF
}

/// One message going out, cut into pieces as room allows.
#[derive(Debug)]
struct OutMsg {
    seq: u32,
    /// Length-prefixed.
    data: Vec<u8>,
    /// Bytes cut into pieces so far.
    cut: usize,
    /// Pieces sent and not acknowledged: offset → (length, stale). A stale
    /// piece got no answer at all: it goes after new ones, so a station NCS
    /// cannot hear still gets all of a message out to those who relay it.
    inflight: BTreeMap<usize, (usize, bool)>,
}

/// Our messages, numbered from 0, until NCS (or our relay) holds them.
#[derive(Default, Debug)]
pub(super) struct Outbox {
    msgs: VecDeque<OutMsg>,
    pub(super) next_seq: u32,
}

impl Outbox {
    /// Queues one message (length-prefixed, so the far end knows its end).
    pub(super) fn push_msg(&mut self, data: &[u8]) {
        let n = data.len().min(MAX_MSG);
        let mut d = (n as u16).to_le_bytes().to_vec();
        d.extend_from_slice(&data[..n]);
        self.msgs.push_back(OutMsg {
            seq: self.next_seq,
            data: d,
            cut: 0,
            inflight: BTreeMap::new(),
        });
        self.next_seq += 1;
    }

    pub(super) fn pending(&self) -> usize {
        self.msgs
            .iter()
            .map(|m| m.data.len() - m.cut + m.inflight.values().map(|v| v.0).sum::<usize>())
            .sum()
    }

    /// Bytes never sent yet.
    pub(super) fn has_new(&self) -> bool {
        self.msgs.iter().any(|m| m.cut < m.data.len())
    }

    /// Up to `room` pieces (message, offset, bytes) of at most `max` bytes:
    /// lost ones first (re-cut to `max`), then new, then stale.
    pub(super) fn take(&mut self, max: usize, room: usize) -> Vec<(u32, usize, Vec<u8>)> {
        let mut out = Vec::new();
        for m in self.msgs.iter_mut() {
            let big: Vec<(usize, (usize, bool))> = m
                .inflight
                .iter()
                .filter(|(_, v)| v.0 > max)
                .map(|(&o, &v)| (o, v))
                .collect();
            for (o, (l, stale)) in big {
                m.inflight.remove(&o);
                let mut x = o;
                while x < o + l {
                    let n = (o + l - x).min(max);
                    m.inflight.insert(x, (n, stale));
                    x += n;
                }
            }
        }
        for pass in 0..3 {
            for m in self.msgs.iter_mut() {
                if pass == 1 {
                    while out.len() < room && m.cut < m.data.len() {
                        let n = (m.data.len() - m.cut).min(max);
                        m.inflight.insert(m.cut, (n, false));
                        out.push((m.seq, m.cut, m.data[m.cut..m.cut + n].to_vec()));
                        m.cut += n;
                    }
                    continue;
                }
                for (&o, v) in m.inflight.iter_mut() {
                    if out.len() < room && v.1 == (pass == 2) {
                        v.1 = false;
                        out.push((m.seq, o, m.data[o..o + v.0].to_vec()));
                    }
                }
            }
        }
        out
    }

    /// Our last burst got no answer at all.
    pub(super) fn stale(&mut self) {
        for m in self.msgs.iter_mut() {
            for v in m.inflight.values_mut() {
                v.1 = true;
            }
        }
    }

    pub(super) fn ack(&mut self, seq: u32, off: usize) {
        if let Some(m) = self.msgs.iter_mut().find(|m| m.seq == seq) {
            m.inflight.remove(&off);
        }
        self.msgs.retain(|m| m.cut < m.data.len() || !m.inflight.is_empty());
    }

    /// The far end holds every message below `next`, and those in `also`.
    pub(super) fn held(&mut self, next: u32, also: &[u32]) {
        self.msgs.retain(|m| m.seq >= next && !also.contains(&m.seq));
    }
}

/// A message's bytes once all of them are in.
fn whole(ib: &Inbox) -> Option<&[u8]> {
    let h = &ib.hist;
    if h.len() < 2 {
        return None;
    }
    let n = u16::from_le_bytes([h[0], h[1]]) as usize;
    (h.len() >= 2 + n).then(|| &h[2..2 + n])
}

/// One station's messages coming in, each put together on its own, so a
/// lost piece holds up only its own message. Everything is kept, for
/// relaying.
#[derive(Default, Debug, Clone)]
pub(super) struct MsgIn {
    /// Every message below this is whole.
    pub(super) next: u32,
    pub(super) msgs: BTreeMap<u32, Inbox>,
}

impl MsgIn {
    /// Takes one piece. Returns the message (number, bytes) if this
    /// completed it.
    pub(super) fn accept(&mut self, field: u32, data: &[u8]) -> Option<(u32, Vec<u8>)> {
        let seq = widen8((field >> 16) as u8, self.next)?;
        if seq < self.next && !self.msgs.contains_key(&seq) {
            return None;
        }
        let ib = self.msgs.entry(seq).or_default();
        if whole(ib).is_some() {
            return None;
        }
        ib.accept(field & 0xFFFF, data);
        let msg = whole(ib)?.to_vec();
        while self.msgs.get(&self.next).is_some_and(|ib| whole(ib).is_some()) {
            self.next += 1;
        }
        Some((seq, msg))
    }

    pub(super) fn has(&self, seq: u32) -> bool {
        seq < self.next || self.msgs.get(&seq).is_some_and(|ib| whole(ib).is_some())
    }

    /// Which of the 8 messages after `next` are whole.
    pub(super) fn bitmap(&self) -> u8 {
        (0..8).fold(0, |a, i| a | (self.has(self.next + 1 + i) as u8) << i)
    }

    /// Whole messages past one that is missing.
    pub(super) fn holes(&self) -> bool {
        self.msgs.range(self.next + 1..).any(|(_, ib)| whole(ib).is_some())
    }
}

/// What NCS holds of one station's messages, as a member sees it (from
/// received lists and the ACKs it overhears).
#[derive(Default, Debug)]
pub(super) struct Held {
    pub(super) next: u32,
    pub(super) also: BTreeSet<u32>,
    /// Pieces of other messages NCS acknowledged: message → (offset, length).
    pub(super) acked: BTreeMap<u32, Vec<(usize, usize)>>,
}

impl Held {
    pub(super) fn has(&self, seq: u32) -> bool {
        seq < self.next || self.also.contains(&seq)
    }

    pub(super) fn covered(&self, seq: u32, off: usize, len: usize) -> bool {
        let Some(v) = self.acked.get(&seq) else { return false };
        let mut v = v.clone();
        v.sort();
        let mut at = off;
        for (o, l) in v {
            if o <= at && o + l > at {
                at = o + l;
            }
        }
        at >= off + len
    }
}

/// A stream coming in: reassembled in order, kept whole (for relaying),
/// split into messages.
#[derive(Default, Debug, Clone)]
pub(super) struct Inbox {
    pub(super) delivered: u32,
    pub(super) pending: BTreeMap<u32, Vec<u8>>,
    /// Everything delivered, from offset 0.
    pub(super) hist: Vec<u8>,
    /// Where the next undelivered message starts in `hist`.
    pub(super) msg_at: usize,
}

impl Inbox {
    /// Returns whether anything new was delivered.
    pub(super) fn accept(&mut self, field: u32, data: &[u8]) -> bool {
        let off = widen(field, self.delivered);
        let end = off.wrapping_add(data.len() as u32);
        if end.wrapping_sub(self.delivered) as i32 <= 0 {
            return false;
        }
        let skip = self.delivered.wrapping_sub(off) as i32;
        let (off, data) = if skip > 0 {
            (self.delivered, &data[skip as usize..])
        } else {
            (off, data)
        };
        if self.pending.get(&off).is_none_or(|v| v.len() < data.len()) {
            self.pending.insert(off, data.to_vec());
        }
        let before = self.delivered;
        loop {
            let next = self
                .pending
                .range(..=self.delivered)
                .next_back()
                .map(|(&k, v)| (k, v.len()));
            let Some((k, len)) = next else { break };
            let end = k + len as u32;
            if end > self.delivered {
                let v = &self.pending[&k];
                let from = (self.delivered - k) as usize;
                self.hist.extend_from_slice(&v[from..]);
                self.delivered = end;
            }
            let done: Vec<u32> = self
                .pending
                .range(..=self.delivered)
                .filter(|(k, v)| **k + v.len() as u32 <= self.delivered)
                .map(|(&k, _)| k)
                .collect();
            if done.is_empty() {
                break;
            }
            for k in done {
                self.pending.remove(&k);
            }
        }
        self.delivered != before
    }

    /// Whole messages delivered since the last call.
    pub(super) fn messages(&mut self) -> Vec<Vec<u8>> {
        self.split(false)
    }

    /// Whole messages delivered since the last call; `packed`: lengths
    /// marked [`LEN_PACKED`] are compressed messages, unpacked here.
    pub(super) fn split(&mut self, packed: bool) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        loop {
            let rest = &self.hist[self.msg_at..];
            if rest.len() < 2 {
                break;
            }
            let mut n = u16::from_le_bytes([rest[0], rest[1]]);
            let marked = packed && n & LEN_PACKED != 0;
            if marked {
                n &= !LEN_PACKED;
            }
            let n = n as usize;
            if rest.len() < 2 + n {
                break;
            }
            let body = &rest[2..2 + n];
            out.push(if marked {
                crate::link::compress::unpack(body).unwrap_or_else(|| body.to_vec())
            } else {
                body.to_vec()
            });
            self.msg_at += 2 + n;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn messages_reassemble_out_of_order() {
        let mut o = Outbox::default();
        o.push_msg(b"first report");
        o.push_msg(b"second");
        let pieces = o.take(5, 10);
        let mut mi = MsgIn::default();
        let mut got = Vec::new();
        for (seq, off, data) in pieces.iter().rev() {
            got.extend(mi.accept(msg_field(*seq, *off), data));
        }
        got.sort();
        assert_eq!(got, vec![(0, b"first report".to_vec()), (1, b"second".to_vec())]);
        assert_eq!(mi.next, 2);
        // A repeat is not delivered twice.
        assert!(mi.accept(msg_field(pieces[0].0, pieces[0].1), &pieces[0].2).is_none());
        for (seq, off, _) in &pieces {
            o.ack(*seq, *off);
        }
        assert_eq!(o.pending(), 0);
    }

    #[test]
    fn a_lost_message_leaves_a_hole_the_list_describes() {
        let mut o = Outbox::default();
        for m in [&b"zero"[..], b"one", b"two"] {
            o.push_msg(m);
        }
        let pieces = o.take(64, 10);
        let mut mi = MsgIn::default();
        for (seq, off, data) in &pieces {
            if *seq != 1 {
                mi.accept(msg_field(*seq, *off), data);
            }
        }
        assert_eq!((mi.next, mi.bitmap()), (1, 0b1));
        assert!(mi.holes());
        // The sender hears the list and keeps only what is missing.
        o.held(1, &[2]);
        assert_eq!(o.msgs.iter().map(|m| m.seq).collect::<Vec<_>>(), vec![1]);
        assert_eq!(widen8(1, 300), Some(257));
        assert_eq!(widen8(250, 3), None);
    }
}
