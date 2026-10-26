//! In-process radio emulators for the tests: an Icom CI-V radio, a
//! Kenwood/Elecraft/Yaesu CAT radio behind a fake serial [`Port`], and a
//! fake `rigctld` on a loopback TCP port. No real COM port or radio is
//! touched. They implement the protocols as the published references
//! describe them, which is all the backends are verified against.

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::cat::Brand;
use super::civ::{CONTROLLER, bcd_be, bcd_le, frame, from_bcd_be, take_frames};
use super::{Line, Port};

/// A radio on the other end of a serial cable.
pub trait Emulator: Send + 'static {
    /// Bytes from the computer; returns what the radio sends back.
    fn input(&mut self, bytes: &[u8]) -> Vec<u8>;
    fn line(&mut self, _line: Line, _on: bool) {}
}

struct Inner<E> {
    emu: E,
    out: VecDeque<u8>,
}

/// A fake serial port wired to an emulator. Clones share the emulator, so
/// a test keeps one to inspect the radio's state.
pub struct EmuPort<E: Emulator> {
    inner: Arc<Mutex<Inner<E>>>,
}

impl<E: Emulator> Clone for EmuPort<E> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}

impl<E: Emulator> EmuPort<E> {
    pub fn new(emu: E) -> Self {
        Self {
            inner: Arc::new(Mutex::new(Inner {
                emu,
                out: VecDeque::new(),
            })),
        }
    }

    pub fn with<R>(&self, f: impl FnOnce(&mut E) -> R) -> R {
        f(&mut self.inner.lock().unwrap().emu)
    }
}

impl<E: Emulator> Read for EmuPort<E> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let mut g = self.inner.lock().unwrap();
        if g.out.is_empty() {
            drop(g);
            // A real port blocks for its timeout.
            std::thread::sleep(Duration::from_millis(2));
            return Err(std::io::ErrorKind::TimedOut.into());
        }
        let n = buf.len().min(g.out.len());
        for b in buf.iter_mut().take(n) {
            *b = g.out.pop_front().unwrap();
        }
        Ok(n)
    }
}

impl<E: Emulator> Write for EmuPort<E> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let mut g = self.inner.lock().unwrap();
        let reply = g.emu.input(buf);
        g.out.extend(reply);
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl<E: Emulator> Port for EmuPort<E> {
    fn set_line(&mut self, line: Line, on: bool) -> std::io::Result<()> {
        self.inner.lock().unwrap().emu.line(line, on);
        Ok(())
    }
    fn clear_input(&mut self) -> std::io::Result<()> {
        self.inner.lock().unwrap().out.clear();
        Ok(())
    }
}

/// A port with nothing but control lines, for RTS/DTR PTT.
#[derive(Default)]
pub struct LineEmu {
    pub rts: bool,
    pub dtr: bool,
}

impl Emulator for LineEmu {
    fn input(&mut self, _bytes: &[u8]) -> Vec<u8> {
        Vec::new()
    }
    fn line(&mut self, line: Line, on: bool) {
        match line {
            Line::Rts => self.rts = on,
            Line::Dtr => self.dtr = on,
        }
    }
}

// ---------------------------------------------------------------------------
// Icom CI-V
// ---------------------------------------------------------------------------

pub struct CivEmu {
    pub addr: u8,
    pub freq: i64,
    pub mode: u8,
    pub filter: u8,
    pub data: u8,
    /// `1A 03` filter width index (34 = 3.0 kHz).
    pub width_index: u32,
    /// RF power, 0-255.
    pub power: u32,
    pub ptt: bool,
    /// Echo every frame back, as the one-wire bus does.
    pub echo: bool,
    /// Send a transceive broadcast before every reply.
    pub broadcast_noise: bool,
    /// Knows `1A 03` and `1A 06` (IC-7300 class); older rigs answer NG.
    pub supports_1a: bool,
    pub rts: bool,
    pub dtr: bool,
    /// Frames addressed to this radio.
    pub frames_seen: usize,
    /// Their bodies.
    pub log: Vec<Vec<u8>>,
    inbuf: Vec<u8>,
}

impl CivEmu {
    pub fn new(addr: u8) -> Self {
        Self {
            addr,
            freq: 14_074_000,
            mode: 0x01,
            filter: 1,
            data: 1,
            width_index: 34,
            power: 128,
            ptt: false,
            echo: false,
            broadcast_noise: false,
            supports_1a: true,
            rts: false,
            dtr: false,
            frames_seen: 0,
            log: Vec::new(),
            inbuf: Vec::new(),
        }
    }

    fn handle(&mut self, b: &[u8]) -> Vec<u8> {
        const OK: u8 = 0xFB;
        const NG: u8 = 0xFA;
        match b {
            [0x03] => {
                let mut r = vec![0x03];
                r.extend(bcd_le(self.freq as u64, 5));
                r
            }
            [0x04] => vec![0x04, self.mode, self.filter],
            [0x1A, 0x06] if self.supports_1a => vec![0x1A, 0x06, self.data, self.filter],
            [0x1A, 0x03] if self.supports_1a => {
                let mut r = vec![0x1A, 0x03];
                r.extend(bcd_be(self.width_index, 1));
                r
            }
            [0x1C, 0x00, x @ (0 | 1)] => {
                self.ptt = *x == 1;
                vec![OK]
            }
            [0x1C, 0x00] => vec![0x1C, 0x00, u8::from(self.ptt)],
            [0x14, 0x0A, a, c] => match from_bcd_be(&[*a, *c]) {
                Some(v) if v <= 255 => {
                    self.power = v;
                    vec![OK]
                }
                _ => vec![NG],
            },
            [0x14, 0x0A] => {
                let mut r = vec![0x14, 0x0A];
                r.extend(bcd_be(self.power, 2));
                r
            }
            _ => vec![NG],
        }
    }
}

impl Emulator for CivEmu {
    fn input(&mut self, bytes: &[u8]) -> Vec<u8> {
        self.inbuf.extend_from_slice(bytes);
        let mut out = Vec::new();
        for f in take_frames(&mut self.inbuf) {
            if f.to != self.addr {
                continue;
            }
            if self.echo {
                out.extend(frame(f.to, f.from, &f.body));
            }
            self.frames_seen += 1;
            self.log.push(f.body.clone());
            if self.broadcast_noise {
                let mut b = vec![0x00];
                b.extend(bcd_le(self.freq as u64, 5));
                out.extend(frame(0x00, self.addr, &b));
            }
            let reply = self.handle(&f.body);
            out.extend(frame(CONTROLLER, self.addr, &reply));
        }
        out
    }

    fn line(&mut self, line: Line, on: bool) {
        match line {
            Line::Rts => self.rts = on,
            Line::Dtr => self.dtr = on,
        }
    }
}

// ---------------------------------------------------------------------------
// Kenwood / Elecraft / Yaesu CAT
// ---------------------------------------------------------------------------

pub struct CatEmu {
    pub brand: Brand,
    pub freq: i64,
    /// `MD` code ('2' USB; Yaesu 'C' DATA-U).
    pub mode: char,
    /// `DA` (Kenwood) or `DT` (Elecraft) value.
    pub data: u32,
    /// Elecraft `BW` in 10 Hz units.
    pub bw_10hz: u32,
    pub watts: u32,
    pub ptt: bool,
    /// Kenwood: keyed with `TX1;` (DATA SEND).
    pub data_send: bool,
    /// Send unsolicited Auto Information before replies.
    pub auto_info: bool,
    /// Answer `?;` to `PC`.
    pub no_power: bool,
    /// Wrong baud rate: never answers.
    pub silent: bool,
    /// Every command received, with its `;`.
    pub log: Vec<String>,
    inbuf: String,
}

impl CatEmu {
    pub fn new(brand: Brand) -> Self {
        let (mode, data) = match brand {
            Brand::Kenwood => ('2', 1),
            Brand::Elecraft => ('6', 0),
            Brand::Yaesu => ('C', 0),
        };
        Self {
            brand,
            freq: 14_074_000,
            mode,
            data,
            bw_10hz: 300,
            watts: 50,
            ptt: false,
            data_send: false,
            auto_info: false,
            no_power: false,
            silent: false,
            log: Vec::new(),
            inbuf: String::new(),
        }
    }

    fn digits(&self) -> usize {
        if self.brand == Brand::Yaesu { 9 } else { 11 }
    }

    fn handle(&mut self, cmd: &str) -> Option<String> {
        let (k, y) = (self.brand == Brand::Kenwood, self.brand == Brand::Yaesu);
        let e = self.brand == Brand::Elecraft;
        let err = Some("?;".to_string());
        if cmd == "FA" {
            return Some(format!("FA{:0w$};", self.freq, w = self.digits()));
        }
        match cmd {
            "MD" if !y => return Some(format!("MD{};", self.mode)),
            "MD0" if y => return Some(format!("MD0{};", self.mode)),
            "DA" if k => return Some(format!("DA{};", self.data)),
            "DT" if e => return Some(format!("DT{};", self.data)),
            "BW" if e => return Some(format!("BW{:04};", self.bw_10hz)),
            "PC" if self.no_power => return err,
            "PC" => return Some(format!("PC{:03};", self.watts)),
            "RX" if !y => {
                self.ptt = false;
                return None;
            }
            "TX" if y => return Some(format!("TX{};", u8::from(self.ptt))),
            "TX" => {
                self.ptt = true;
                self.data_send = false;
                return None;
            }
            "TX0" if y => {
                self.ptt = false;
                return None;
            }
            "TX0" | "TX1" if k || y => {
                self.ptt = true;
                self.data_send = cmd == "TX1";
                return None;
            }
            _ => {}
        }
        if let Some(v) = cmd.strip_prefix("PC")
            && v.len() == 3
            && let Ok(w) = v.parse()
        {
            if self.no_power {
                return err;
            }
            self.watts = w;
            return None;
        }
        err
    }
}

impl Emulator for CatEmu {
    fn input(&mut self, bytes: &[u8]) -> Vec<u8> {
        self.inbuf.push_str(&String::from_utf8_lossy(bytes));
        let mut out = String::new();
        while let Some(i) = self.inbuf.find(';') {
            let cmd: String = self.inbuf.drain(..=i).collect();
            self.log.push(cmd.clone());
            if self.silent {
                continue;
            }
            if self.auto_info {
                // VFO B and an IF-style status: not what was asked for.
                out.push_str("FB00007000000;IF00014074000     +00000000002000000;");
            }
            if let Some(r) = self.handle(cmd.trim_end_matches(';')) {
                out.push_str(&r);
            }
        }
        out.into_bytes()
    }
}

// ---------------------------------------------------------------------------
// Fake rigctld
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct RigctldState {
    pub freq: i64,
    pub ptt: bool,
    pub mode: String,
    pub passband: u32,
    pub power: f32,
    pub no_power: bool,
}

/// A `rigctld` stand-in speaking the extended response protocol on
/// 127.0.0.1 at a port the OS picks.
pub struct FakeRigctld {
    addr: String,
    state: Arc<Mutex<RigctldState>>,
    stop: Arc<AtomicBool>,
    conns: Arc<Mutex<Vec<TcpStream>>>,
}

impl FakeRigctld {
    pub fn start() -> FakeRigctld {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        listener.set_nonblocking(true).unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let state = Arc::new(Mutex::new(RigctldState {
            freq: 14_074_000,
            ptt: false,
            mode: "PKTUSB".into(),
            passband: 3000,
            power: 0.5,
            no_power: false,
        }));
        let stop = Arc::new(AtomicBool::new(false));
        let conns = Arc::new(Mutex::new(Vec::new()));
        let (st, sp, cs) = (state.clone(), stop.clone(), conns.clone());
        std::thread::spawn(move || {
            while !sp.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((s, _)) => {
                        s.set_nonblocking(false).unwrap();
                        cs.lock().unwrap().push(s.try_clone().unwrap());
                        let st = st.clone();
                        std::thread::spawn(move || serve(s, st));
                    }
                    Err(_) => std::thread::sleep(Duration::from_millis(5)),
                }
            }
        });
        FakeRigctld {
            addr,
            state,
            stop,
            conns,
        }
    }

    pub fn addr(&self) -> String {
        self.addr.clone()
    }

    pub fn state(&self) -> RigctldState {
        self.state.lock().unwrap().clone()
    }

    pub fn state_mut(&self, f: impl FnOnce(&mut RigctldState)) {
        f(&mut self.state.lock().unwrap())
    }

    /// The daemon exits: every connection closes.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
        for c in self.conns.lock().unwrap().drain(..) {
            let _ = c.shutdown(std::net::Shutdown::Both);
        }
    }
}

impl Drop for FakeRigctld {
    fn drop(&mut self) {
        self.stop();
    }
}

fn serve(s: TcpStream, state: Arc<Mutex<RigctldState>>) {
    let mut out = s.try_clone().unwrap();
    let mut lines = BufReader::new(s);
    let mut line = String::new();
    loop {
        line.clear();
        match lines.read_line(&mut line) {
            Ok(0) | Err(_) => return,
            Ok(_) => {}
        }
        let cmd = line.trim().trim_start_matches('+');
        let mut st = state.lock().unwrap();
        let words: Vec<&str> = cmd.split_whitespace().collect();
        let reply = match words.as_slice() {
            ["f"] => format!("get_freq:\nFrequency: {}\nRPRT 0\n", st.freq),
            ["F", hz] => match hz.parse::<i64>() {
                Ok(h) if h > 0 => {
                    st.freq = h;
                    format!("set_freq: {hz}\nRPRT 0\n")
                }
                _ => format!("set_freq: {hz}\nRPRT -1\n"),
            },
            ["t"] => format!("get_ptt:\nPTT: {}\nRPRT 0\n", u8::from(st.ptt)),
            ["T", v] => {
                st.ptt = *v == "1";
                format!("set_ptt: {v}\nRPRT 0\n")
            }
            ["m"] => format!("get_mode:\nMode: {}\nPassband: {}\nRPRT 0\n", st.mode, st.passband),
            ["l", "RFPOWER"] if st.no_power => "get_level: RFPOWER\nRPRT -11\n".to_string(),
            ["l", "RFPOWER"] => format!("get_level: RFPOWER\nRFPOWER: {:.6}\nRPRT 0\n", st.power),
            ["L", "RFPOWER", _] if st.no_power => "set_level: RFPOWER\nRPRT -11\n".to_string(),
            ["L", "RFPOWER", v] => match v.parse::<f32>() {
                Ok(p) if (0.0..=1.0).contains(&p) => {
                    st.power = p;
                    format!("set_level: RFPOWER {v}\nRPRT 0\n")
                }
                _ => "set_level: RFPOWER\nRPRT -1\n".to_string(),
            },
            _ => "RPRT -4\n".to_string(),
        };
        drop(st);
        if out.write_all(reply.as_bytes()).is_err() {
            return;
        }
    }
}
