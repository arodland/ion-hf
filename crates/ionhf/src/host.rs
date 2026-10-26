//! Winlink host interface: Ion looks like a VARA HF modem on TCP, so Pat
//! or Winlink Express can drive it. Command port (default 8300) and data
//! port (one above), on 127.0.0.1 only, per "VARA Protocol Native TNC
//! Commands" (2022).
//!
//! Settings that mean nothing to Ion (`BW…`, `COMPRESSION …`, `CWID …`,
//! ...) are answered `OK`. `BUFFER n` counts bytes not yet acknowledged by
//! the far end (Pat waits for 0 before closing); `PTT ON/OFF` is for
//! information only, as Ion keys the radio itself.

use std::io::{ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender};
use ion::link::State;

use crate::engine::UiEvent;
use crate::ion_engine::{IonCmd, IonStatus};

/// From the Ion worker to the host interface.
#[derive(Debug)]
pub enum HostIn {
    Status(IonStatus),
    Received(Vec<u8>),
}

pub struct HostHandle {
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Drop for HostHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// Opens the two ports and serves one host at a time.
pub fn spawn(port: u16, ion: Sender<IonCmd>, ui: Sender<UiEvent>) -> std::io::Result<HostHandle> {
    let cmd = TcpListener::bind(("127.0.0.1", port))?;
    let data = TcpListener::bind(("127.0.0.1", port + 1))?;
    cmd.set_nonblocking(true)?;
    data.set_nonblocking(true)?;
    let (tx, rx) = crossbeam_channel::unbounded();
    let _ = ion.send(IonCmd::AttachHost(Some(tx)));
    let stop = Arc::new(AtomicBool::new(false));
    let stop2 = stop.clone();
    let thread = std::thread::Builder::new().name("winlink-host".into()).spawn(move || {
        let mut h = Host::new(ion.clone(), ui);
        while !stop2.load(Ordering::Relaxed) {
            h.service(&cmd, &data, &rx);
            std::thread::sleep(Duration::from_millis(20));
        }
        let _ = ion.send(IonCmd::AttachHost(None));
    })?;
    Ok(HostHandle {
        stop,
        thread: Some(thread),
    })
}

struct Host {
    ion: Sender<IonCmd>,
    ui: Sender<UiEvent>,
    cmd: Option<TcpStream>,
    data: Option<TcpStream>,
    line: Vec<u8>,
    mycall: String,
    listen: bool,
    /// We sent CONNECT (so we are the caller).
    calling: Option<String>,
    /// The link is up: (caller, callee).
    connected: Option<(String, String)>,
    last_buffer: Option<usize>,
    last_ptt: bool,
    last_busy: bool,
    last_state: Option<State>,
    alive_at: Instant,
    /// Bytes from the host waiting for the data socket to accept them
    /// (and received bytes waiting for the host).
    to_host: Vec<u8>,
}

impl Host {
    fn new(ion: Sender<IonCmd>, ui: Sender<UiEvent>) -> Self {
        Self {
            ion,
            ui,
            cmd: None,
            data: None,
            line: Vec::new(),
            mycall: String::new(),
            listen: false,
            calling: None,
            connected: None,
            last_buffer: None,
            last_ptt: false,
            last_busy: false,
            last_state: None,
            alive_at: Instant::now(),
            to_host: Vec::new(),
        }
    }

    fn note(&self, s: String) {
        let _ = self.ui.try_send(UiEvent::Host(s));
    }

    fn say(&mut self, line: &str) {
        if let Some(c) = self.cmd.as_mut() {
            let mut b = line.as_bytes().to_vec();
            b.push(b'\r');
            if c.write_all(&b).is_err() {
                self.cmd = None;
            }
        }
    }

    fn service(&mut self, cmd_l: &TcpListener, data_l: &TcpListener, rx: &Receiver<HostIn>) {
        // New hosts replace old ones.
        if let Ok((s, addr)) = cmd_l.accept() {
            let _ = s.set_nonblocking(true);
            let _ = s.set_nodelay(true);
            self.cmd = Some(s);
            self.line.clear();
            self.note(format!("Winlink host connected from {addr}"));
        }
        if let Ok((s, _)) = data_l.accept() {
            let _ = s.set_nonblocking(true);
            let _ = s.set_nodelay(true);
            self.data = Some(s);
        }
        let mut buf = [0u8; 1024];
        if let Some(c) = self.cmd.as_mut() {
            match c.read(&mut buf) {
                Ok(0) => {
                    self.cmd = None;
                    self.note("Winlink host disconnected".into());
                }
                Ok(n) => self.line.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(_) => self.cmd = None,
            }
        }
        while let Some(i) = self.line.iter().position(|&b| b == b'\r' || b == b'\n') {
            let l: Vec<u8> = self.line.drain(..=i).collect();
            let text = String::from_utf8_lossy(&l[..l.len() - 1]).trim().to_string();
            if !text.is_empty() {
                self.command(&text);
            }
        }
        // Data from the host goes onto the link.
        if let Some(d) = self.data.as_mut() {
            match d.read(&mut buf) {
                Ok(0) => self.data = None,
                Ok(n) => {
                    let _ = self.ion.send(IonCmd::Send(buf[..n].to_vec()));
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(_) => self.data = None,
            }
        }
        for m in rx.try_iter() {
            match m {
                HostIn::Received(b) => self.to_host.extend(b),
                HostIn::Status(s) => self.status(&s),
            }
        }
        if !self.to_host.is_empty()
            && let Some(d) = self.data.as_mut()
        {
            match d.write(&self.to_host) {
                Ok(n) => {
                    self.to_host.drain(..n);
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(_) => self.data = None,
            }
        }
        if self.alive_at.elapsed() >= Duration::from_secs(60) {
            self.alive_at = Instant::now();
            self.say("IAMALIVE");
        }
    }

    fn command(&mut self, text: &str) {
        let up = text.to_ascii_uppercase();
        let words: Vec<&str> = up.split_whitespace().collect();
        match words.as_slice() {
            ["MYCALL", call, ..] => {
                self.mycall = call.split('-').next().unwrap_or(call).to_string();
                let _ = self.ion.send(IonCmd::SetCall(self.mycall.clone()));
                self.say("OK");
            }
            ["LISTEN", "ON"] => {
                self.listen = true;
                if self.connected.is_none() && self.calling.is_none() {
                    let _ = self.ion.send(IonCmd::Listen);
                }
                self.say("OK");
            }
            ["LISTEN", "OFF"] => {
                self.listen = false;
                if self.connected.is_none() && self.calling.is_none() {
                    let _ = self.ion.send(IonCmd::Stop);
                }
                self.say("OK");
            }
            ["CONNECT", src, dst, ..] => {
                if self.mycall.is_empty() {
                    self.mycall = src.to_string();
                }
                let dst = dst.split('-').next().unwrap_or(dst).to_string();
                self.calling = Some(dst.clone());
                let _ = self.ion.send(IonCmd::Connect(dst.clone()));
                self.note(format!("Winlink host: calling {dst}"));
                self.say("OK");
            }
            ["DISCONNECT"] => {
                self.say("OK");
                if self.connected.is_some() {
                    let _ = self.ion.send(IonCmd::Disconnect);
                } else {
                    // Nothing up (or still calling): give up now.
                    let _ = self.ion.send(IonCmd::Stop);
                    self.calling = None;
                    self.say("DISCONNECTED");
                    self.relisten();
                }
            }
            ["ABORT"] => {
                self.say("OK");
                let _ = self.ion.send(IonCmd::Stop);
                self.calling = None;
                if self.connected.take().is_some() {
                    self.say("DISCONNECTED");
                }
                self.relisten();
            }
            ["VERSION"] => self.say(&format!("VERSION IonHF {}", env!("CARGO_PKG_VERSION"))),
            [w, ..]
                if w.starts_with("BW")
                    || matches!(
                        *w,
                        "COMPRESSION" | "CHAT" | "PUBLIC" | "CWID" | "WINLINK" | "P2P" | "CQFRAME"
                    ) =>
            {
                self.say("OK")
            }
            _ => {
                log::info!("Winlink host: unknown command {text:?}");
                self.say("WRONG");
            }
        }
    }

    fn relisten(&mut self) {
        if self.listen {
            let _ = self.ion.send(IonCmd::Listen);
        }
    }

    fn status(&mut self, s: &IonStatus) {
        let state = s.state;
        if self.last_state != Some(state) {
            self.last_state = Some(state);
            match state {
                State::Connected if self.connected.is_none() => {
                    let pair = match self.calling.take() {
                        Some(dst) => (self.mycall.clone(), dst),
                        None => (s.peer.clone(), self.mycall.clone()),
                    };
                    self.say(&format!("CONNECTED {} {} 2300", pair.0, pair.1));
                    self.note(format!("Winlink host: connected {} → {}", pair.0, pair.1));
                    self.connected = Some(pair);
                    self.last_buffer = None;
                }
                State::Disconnected | State::Failed | State::Idle | State::Listening => {
                    let was_up = self.connected.take().is_some();
                    let was_calling = self.calling.take().is_some();
                    if was_up || was_calling {
                        self.say("DISCONNECTED");
                        if state != State::Listening {
                            self.relisten();
                        }
                    }
                }
                _ => {}
            }
        }
        if self.connected.is_some() && self.last_buffer != Some(s.pending) {
            self.last_buffer = Some(s.pending);
            self.say(&format!("BUFFER {}", s.pending));
        }
        if s.on_air != self.last_ptt {
            self.last_ptt = s.on_air;
            self.say(if s.on_air { "PTT ON" } else { "PTT OFF" });
        }
        if s.busy != self.last_busy {
            self.last_busy = s.busy;
            self.say(if s.busy { "BUSY ON" } else { "BUSY OFF" });
        }
    }
}
