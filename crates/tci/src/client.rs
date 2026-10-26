//! Blocking TCI client: one WebSocket, reconnect with backoff, audio frames
//! handed to a [`Handler`] on the calling thread.

use std::io;
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::time::Duration;

use tungstenite::{Message, WebSocket};

use crate::protocol::{
    Command, Event, FrameHeader, HEADER_LEN, Parser, RadioState, TextCommand, decode_samples, frame_type, sample_format,
};

#[derive(Clone, Debug)]
pub struct Config {
    /// ExpertSDR3 listens on `ws://127.0.0.1:50001` by default (ExpertSDR2:
    /// port 40001).
    pub url: String,
    /// Receiver whose audio is streamed.
    pub receiver: u32,
    pub sample_rate: u32,
    /// Interleaved channels in audio frames (TCI default: stereo).
    pub channels: usize,
    /// Send `audio_samplerate` + `audio_start` once the server is ready.
    pub start_audio: bool,
    /// Whether a TX chrono frame's `length` counts samples on every channel
    /// rather than per channel.
    pub length_counts_all_channels: bool,
    pub connect_timeout: Duration,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            url: "ws://127.0.0.1:50001".into(),
            receiver: 0,
            sample_rate: 48_000,
            channels: 2,
            start_audio: true,
            length_counts_all_channels: true,
            connect_timeout: Duration::from_secs(2),
        }
    }
}

pub trait Handler {
    /// Every parsed server event, after `state` has been updated.
    fn on_event(&mut self, event: &Event, state: &RadioState);

    /// Every binary frame, before it is dispatched (for capture/logging).
    fn on_frame(&mut self, _header: &FrameHeader, _payload_len: usize) {}

    /// RX audio, interleaved as received.
    fn on_rx_audio(&mut self, _header: &FrameHeader, _samples: &[f32], _channels: usize) {}

    /// The radio wants `out.len()` mono samples of TX audio; they are
    /// duplicated to every channel. Must not block.
    fn on_tx_request(&mut self, _header: &FrameHeader, out: &mut [f32]) {
        out.fill(0.0);
    }
}

/// Runs until `stop` is set, reconnecting with backoff on every failure.
pub fn run<H: Handler>(cfg: &Config, handler: &mut H, commands: &Receiver<Command>, stop: &AtomicBool) {
    let mut backoff = Duration::from_millis(500);
    let mut state = RadioState::default();
    while !stop.load(Ordering::Relaxed) {
        match connect(cfg) {
            Ok(ws) => {
                backoff = Duration::from_millis(500);
                let ev = Event::Connected;
                state.apply(&ev);
                handler.on_event(&ev, &state);
                let reason = match Session::new(cfg, ws).run(handler, commands, stop, &mut state) {
                    Ok(()) => "closed".to_string(),
                    Err(e) => e.to_string(),
                };
                let ev = Event::Disconnected(reason);
                state.apply(&ev);
                handler.on_event(&ev, &state);
            }
            Err(e) => {
                let ev = Event::Disconnected(e.to_string());
                state.apply(&ev);
                handler.on_event(&ev, &state);
            }
        }
        // Drop commands queued while disconnected; they refer to old state.
        while commands.try_recv().is_ok() {}
        let mut waited = Duration::ZERO;
        while waited < backoff && !stop.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_millis(50));
            waited += Duration::from_millis(50);
        }
        backoff = (backoff * 2).min(Duration::from_secs(5));
    }
}

fn connect(cfg: &Config) -> io::Result<WebSocket<TcpStream>> {
    let url = url::Url::parse(&cfg.url).map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
    let host = url
        .host_str()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "URL has no host"))?;
    let port = url.port_or_known_default().unwrap_or(50001);
    let addr = (host, port)
        .to_socket_addrs()?
        .next()
        .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "host did not resolve"))?;
    let stream = TcpStream::connect_timeout(&addr, cfg.connect_timeout)?;
    stream.set_nodelay(true)?;
    let (ws, _) = tungstenite::client(cfg.url.as_str(), stream)
        .map_err(|e| io::Error::new(io::ErrorKind::ConnectionRefused, e.to_string()))?;
    // Short read timeout so outgoing commands are never held up by reads.
    ws.get_ref().set_read_timeout(Some(Duration::from_millis(2)))?;
    Ok(ws)
}

struct Session<'a> {
    cfg: &'a Config,
    ws: WebSocket<TcpStream>,
    parser: Parser,
    cmds: Vec<TextCommand>,
    samples: Vec<f32>,
    mono: Vec<f32>,
    tx_frame: Vec<u8>,
}

impl<'a> Session<'a> {
    fn new(cfg: &'a Config, ws: WebSocket<TcpStream>) -> Self {
        Self {
            cfg,
            ws,
            parser: Parser::new(),
            cmds: Vec::with_capacity(64),
            samples: Vec::with_capacity(8192),
            mono: Vec::with_capacity(8192),
            tx_frame: Vec::with_capacity(HEADER_LEN + 8192 * 8),
        }
    }

    fn send(&mut self, cmd: &Command) -> tungstenite::Result<()> {
        let wire = cmd.to_wire();
        log::debug!("tci > {wire}");
        self.ws.send(Message::text(wire))
    }

    fn run<H: Handler>(
        &mut self,
        handler: &mut H,
        commands: &Receiver<Command>,
        stop: &AtomicBool,
        state: &mut RadioState,
    ) -> tungstenite::Result<()> {
        loop {
            if stop.load(Ordering::Relaxed) {
                let _ = self.ws.close(None);
                let _ = self.ws.flush();
                return Ok(());
            }
            while let Ok(cmd) = commands.try_recv() {
                self.send(&cmd)?;
            }
            match self.ws.read() {
                Ok(Message::Text(t)) => {
                    log::debug!("tci < {}", t.as_str());
                    self.cmds.clear();
                    self.parser.feed(t.as_str(), &mut self.cmds);
                    for i in 0..self.cmds.len() {
                        let ev = Event::parse(&self.cmds[i]);
                        state.apply(&ev);
                        if ev == Event::Ready && self.cfg.start_audio {
                            self.send(&Command::AudioSampleRate(self.cfg.sample_rate))?;
                            self.send(&Command::AudioStart(self.cfg.receiver))?;
                        }
                        if let Event::Other(c) = &ev {
                            log::trace!("tci: ignoring {c}");
                        }
                        handler.on_event(&ev, state);
                    }
                }
                Ok(Message::Binary(b)) => self.binary(&b, handler)?,
                Ok(Message::Close(_)) => return Ok(()),
                Ok(_) => {}
                Err(tungstenite::Error::Io(e))
                    if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) =>
                {
                    // Lets tungstenite finish pending writes (e.g. pongs).
                    match self.ws.flush() {
                        Ok(()) => {}
                        Err(tungstenite::Error::Io(e))
                            if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {}
                        Err(e) => return Err(e),
                    }
                }
                Err(e) => return Err(e),
            }
        }
    }

    fn binary<H: Handler>(&mut self, b: &[u8], handler: &mut H) -> tungstenite::Result<()> {
        let Some(hdr) = FrameHeader::parse(b) else {
            log::warn!("tci: short binary frame ({} bytes)", b.len());
            return Ok(());
        };
        let payload = &b[HEADER_LEN..];
        handler.on_frame(&hdr, payload.len());
        match hdr.frame_type {
            // Another client may have started audio for a different
            // receiver; splicing its frames in would wreck ours.
            frame_type::RX_AUDIO if hdr.receiver != self.cfg.receiver => {
                log::trace!("tci: skipping RX audio of receiver {}", hdr.receiver);
            }
            frame_type::RX_AUDIO => {
                if decode_samples(hdr.format, payload, &mut self.samples) {
                    handler.on_rx_audio(&hdr, &self.samples, self.cfg.channels.max(1));
                } else {
                    log::warn!("tci: unsupported sample format {}", hdr.format);
                }
            }
            frame_type::TX_CHRONO => {
                let ch = self.cfg.channels.max(1);
                let frames = if self.cfg.length_counts_all_channels {
                    hdr.length as usize / ch
                } else {
                    hdr.length as usize
                };
                self.mono.clear();
                self.mono.resize(frames, 0.0);
                handler.on_tx_request(&hdr, &mut self.mono);

                let reply = FrameHeader {
                    receiver: hdr.receiver,
                    sample_rate: hdr.sample_rate,
                    format: sample_format::FLOAT32,
                    codec: 0,
                    crc: 0,
                    length: hdr.length,
                    frame_type: frame_type::TX_AUDIO,
                    reserved: [0; 9],
                };
                self.tx_frame.clear();
                reply.write(&mut self.tx_frame);
                for &s in &self.mono {
                    for _ in 0..ch {
                        self.tx_frame.extend_from_slice(&s.to_le_bytes());
                    }
                }
                self.ws.send(Message::binary(self.tx_frame.clone()))?;
            }
            other => log::trace!("tci: ignoring frame type {other}"),
        }
        Ok(())
    }
}
