//! Text compression for chat and net traffic (`ion::link::compress`) on
//! held-out text: `crates/ion/data/chat_heldout.txt`, kept apart from the
//! priming text. One message per line; a blank line ends a session (a net,
//! a QSO). The ignored tables: `-- --ignored --nocapture`.

use ion::link::compress::{self, Model, PRIME, Params, StreamEncoder, V1, pack, unpack};
use ion::link::frame::Frame;

const HELDOUT: &str = include_str!("../data/chat_heldout.txt");

fn sessions(text: &str) -> Vec<Vec<String>> {
    let mut out = vec![Vec::new()];
    for line in text.lines() {
        let line = line.trim_end_matches('\r');
        if line.is_empty() {
            if !out.last().unwrap().is_empty() {
                out.push(Vec::new());
            }
        } else {
            out.last_mut().unwrap().push(line.to_string());
        }
    }
    out.retain(|s| !s.is_empty());
    out
}

fn frames(bytes: usize, payload: usize) -> usize {
    bytes.div_ceil(payload).max(1)
}

/// Bytes on the air for each message, by one method.
struct Row {
    name: &'static str,
    bytes: Vec<usize>,
}

fn rows(ss: &[Vec<String>]) -> Vec<Row> {
    let msgs: Vec<&String> = ss.iter().flatten().collect();
    let mut rows = Vec::new();
    rows.push(Row {
        name: "As text",
        bytes: msgs.iter().map(|m| m.len()).collect(),
    });
    // Compressor with no priming, learning through each session.
    let mut b = Vec::new();
    for s in ss {
        let mut m = Model::new(V1);
        for t in s {
            let mut tmp = m.clone();
            let body = tmp.encode(t.as_bytes());
            if body.len() < t.len() {
                m = tmp;
                b.push(1 + body.len());
            } else {
                b.push(1 + t.len());
            }
        }
    }
    rows.push(Row {
        name: "Compressor, not primed (session)",
        bytes: b,
    });
    // Net mode: each message on its own.
    rows.push(Row {
        name: "Compressor (per message: net mode)",
        bytes: msgs.iter().map(|t| pack(t.as_bytes()).len()).collect(),
    });
    // Link: one stream per session, the model learning as it goes.
    let mut b = Vec::new();
    for s in ss {
        let mut enc = StreamEncoder::new();
        for t in s {
            let mut w = Vec::new();
            enc.push(t.as_bytes(), &mut w);
            b.push(w.len());
        }
    }
    rows.push(Row {
        name: "Compressor (session: link)",
        bytes: b,
    });
    rows
}

fn print_table(ss: &[Vec<String>], rows: &[Row]) {
    let msgs: Vec<&String> = ss.iter().flatten().collect();
    let floor = Frame::max_payload(32);
    let long = Frame::max_payload(180);
    let raw: usize = msgs.iter().map(|m| m.len()).sum();
    let raw_floor: usize = msgs.iter().map(|m| frames(m.len(), floor)).sum();
    println!(
        "{} messages in {} sessions, {raw} bytes, {:.0} bytes per message",
        msgs.len(),
        ss.len(),
        raw as f64 / msgs.len() as f64
    );
    println!("| Method | Bytes | of text | MFSK floor frames (22 B) | Long frames (170 B) |");
    println!("|---|---:|---:|---:|---:|");
    for r in rows {
        let total: usize = r.bytes.iter().sum();
        let fl: usize = r.bytes.iter().map(|&b| frames(b, floor)).sum();
        let lo: usize = r.bytes.iter().map(|&b| frames(b, long)).sum();
        println!(
            "| {} | {total} | {:.1}% | {fl} ({:.0}%) | {lo} |",
            r.name,
            100.0 * total as f64 / raw as f64,
            100.0 * fl as f64 / raw_floor as f64,
        );
    }
}

#[test]
fn heldout_table() {
    let ss = sessions(HELDOUT);
    // Round trips first: every held-out message, both formats.
    for s in &ss {
        let mut enc = StreamEncoder::new();
        let mut dec = compress::StreamDecoder::new();
        for t in s {
            assert_eq!(unpack(&pack(t.as_bytes())).as_deref(), Some(t.as_bytes()));
            let mut w = Vec::new();
            enc.push(t.as_bytes(), &mut w);
            let mut got = Vec::new();
            dec.feed(&w, &mut got);
            assert_eq!(got, t.as_bytes());
        }
    }
    let rows = rows(&ss);
    print_table(&ss, &rows);
    let total = |name: &str| {
        rows.iter()
            .find(|r| r.name == name)
            .unwrap()
            .bytes
            .iter()
            .sum::<usize>()
    };
    let raw = total("As text");
    let net = total("Compressor (per message: net mode)");
    let link = total("Compressor (session: link)");
    assert!(link <= net, "per message {net}, session {link}");
    assert!(net * 2 < raw, "compressor saved less than half: {net} of {raw}");
}

/// Same table on the training text itself (what the priming text does to
/// text it contains: an upper bound, not a result).
#[test]
#[ignore]
fn train_table() {
    let ss = sessions(PRIME);
    print_table(&ss, &rows(&ss));
}

/// Parameter choice, on the training text alone: sessions 0, 1, 2 of
/// every 4 prime the model, the 4th is coded (per message). The held-out
/// file is not read.
#[test]
#[ignore]
fn tuning_table() {
    let ss = sessions(PRIME);
    let prime: String = ss
        .iter()
        .enumerate()
        .filter(|(i, _)| i % 4 != 3)
        .flat_map(|(_, s)| s.iter().cloned())
        .collect::<Vec<_>>()
        .join("\n");
    let eval: Vec<&String> = ss
        .iter()
        .enumerate()
        .filter(|(i, _)| i % 4 == 3)
        .flat_map(|(_, s)| s.iter())
        .collect();
    let raw: usize = eval.iter().map(|m| m.len()).sum();
    println!(
        "tuning: {} messages, {raw} bytes coded; primed on {} bytes",
        eval.len(),
        prime.len()
    );
    let variants: Vec<(&str, Params)> = vec![
        ("V1", V1),
        (
            "orders 1-3",
            Params {
                orders: &[1, 2, 3],
                ..V1
            },
        ),
        (
            "orders 1-4",
            Params {
                orders: &[1, 2, 3, 4],
                ..V1
            },
        ),
        (
            "orders 1-6",
            Params {
                orders: &[1, 2, 3, 4, 5, 6],
                ..V1
            },
        ),
        ("no word", Params { word: false, ..V1 }),
        ("2^16 slots", Params { table_bits: 16, ..V1 }),
        ("2^20 slots", Params { table_bits: 20, ..V1 }),
        ("limit 30", Params { limit: 30, ..V1 }),
        ("limit 255", Params { limit: 255, ..V1 }),
        ("mix 8", Params { mix_rate: 8, ..V1 }),
        ("mix 48", Params { mix_rate: 48, ..V1 }),
        ("mix 82", Params { mix_rate: 82, ..V1 }),
        (
            "orders 1-3, 2^19",
            Params {
                orders: &[1, 2, 3],
                table_bits: 19,
                ..V1
            },
        ),
        (
            "orders 1-3, 2^20",
            Params {
                orders: &[1, 2, 3],
                table_bits: 20,
                ..V1
            },
        ),
        (
            "orders 1-3, 2^17",
            Params {
                orders: &[1, 2, 3],
                table_bits: 17,
                ..V1
            },
        ),
        ("orders 1-2", Params { orders: &[1, 2], ..V1 }),
    ];
    println!("| Variant | Bytes | of text | Model memory |");
    println!("|---|---:|---:|---:|");
    for (name, p) in variants {
        let base = Model::primed_with(p, &prime);
        let total: usize = eval
            .iter()
            .map(|t| 1 + base.clone().encode(t.as_bytes()).len().min(t.len()))
            .sum();
        let tables = p.orders.len() + p.word as usize;
        let mem = tables * (4 << p.table_bits);
        println!(
            "| {name} | {total} | {:.1}% | {:.1} MB |",
            100.0 * total as f64 / raw as f64,
            mem as f64 / 1e6
        );
    }
}
