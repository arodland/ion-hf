//! The window: header (views, call, state, lamps), the view, the
//! waterfall; events from the engine and the Ion worker; settings saved
//! as they change.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender};
use eframe::egui::{self, Margin, RichText, Stroke};
use ion::link::State;
use ion::link::net::NetEvent;

use crate::config::Config;
use crate::engine::{Msg, Status, UiEvent};
use crate::history::History;
use crate::ion_engine::{IonCmd, IonEvent, IonHandle, IonStatus, NetStatus};
use crate::ion_files::{self, Demux, Item, Kind};
use crate::radio::{LoopCmd, RadioInfo, RadioThread};
use crate::waterfall::{Band, Cursor, Waterfall};
use crate::{theme, views};

pub const FULL_SIZE: [f32; 2] = [1320.0, 880.0];
pub const MINI_SIZE: [f32; 2] = [520.0, 360.0];
/// Lines kept in the notes and net logs.
const LOG_LINES: usize = 300;

pub struct Threads {
    pub engine: Option<std::thread::JoinHandle<()>>,
    pub radio: RadioThread,
    pub ion: IonHandle,
    pub ui_tx: Sender<UiEvent>,
}

/// What the Link view's centre shows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Center {
    Chat,
    Charts,
    Log,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tab {
    Link,
    Listen,
    Net,
    Setup,
}

/// A line of the net's traffic.
pub struct NetLine {
    pub time: String,
    pub from: String,
    pub to: String,
    pub text: String,
    /// A note (check-in, missed, relay) rather than a message.
    pub note: bool,
}

/// One entry of the chat: a line of text, or a file (an index into
/// [`App::files`]).
pub enum Chat {
    /// `ends_at`: for a line we sent, our bytes sent through its end.
    Text {
        sent: bool,
        text: String,
        ends_at: u64,
    },
    File(usize),
}

/// A file or picture sent or received on the link (`crate::ion_files`).
pub struct Xfer {
    pub sent: bool,
    pub name: String,
    pub kind: Kind,
    /// Bytes: as framed on the stream (sent) or the file's (received).
    pub size: usize,
    pub have: usize,
    pub done: bool,
    /// Pictures: what has arrived (sent: what the other side will see),
    /// and (strips in, strips); `image_new` until the window has it.
    pub image: Option<egui::ColorImage>,
    pub strips: (usize, usize),
    pub image_new: bool,
    pub texture: Option<egui::TextureHandle>,
    /// Received: where it was saved, or why it was not.
    pub saved: Option<std::path::PathBuf>,
    pub error: Option<String>,
    /// Sent: the link's bytes acknowledged when it has all gone.
    pub ends_at: u64,
}

impl Xfer {
    fn new(sent: bool, name: String, kind: Kind, size: usize) -> Self {
        Self {
            sent,
            name,
            kind,
            size,
            have: 0,
            done: false,
            image: None,
            strips: (0, 0),
            image_new: false,
            texture: None,
            saved: None,
            error: None,
            ends_at: 0,
        }
    }

    /// The picture from `payload` so far; returns it once all its strips
    /// are in.
    fn picture(&mut self, payload: &[u8]) -> Option<image::RgbImage> {
        let (img, strips) = ion_files::image_so_far(payload)?;
        if strips != self.strips || self.image.is_none() {
            self.strips = strips;
            self.image = Some(egui::ColorImage::from_rgb(
                [img.width() as usize, img.height() as usize],
                img.as_raw(),
            ));
            self.image_new = true;
        }
        (strips.0 == strips.1).then_some(img)
    }
}

pub struct App {
    pub cfg: Config,
    pub loopback: bool,
    engine: Sender<Msg>,
    events: Receiver<UiEvent>,
    spectrum_return: Sender<Vec<f32>>,
    pub threads: Threads,
    pub status: Status,
    pub radio: RadioInfo,
    pub link: Option<IonStatus>,
    pub net: Option<NetStatus>,
    pub hist: History,
    /// (UTC time, text) of what happened.
    pub notes: VecDeque<(String, String)>,
    pub net_log: VecDeque<NetLine>,
    /// The chat: text both ways, and files.
    pub chat: Vec<Chat>,
    pub files: Vec<Xfer>,
    demux: Demux,
    /// A file to send (dropped on the window, or typed).
    pub file_path: String,
    /// Bytes we have put on this link's stream (text and files): where the
    /// next one ends, in the link's acknowledged bytes.
    sent_bytes: u64,
    pub compose: String,
    /// The Call box while it is being edited.
    pub call_draft: Option<String>,
    /// Sound cards for Setup: read in the background (slow on some systems), once and on Rescan.
    audio_devices: Option<(Vec<String>, Vec<String>)>,
    audio_scan: Option<std::sync::mpsc::Receiver<(Vec<String>, Vec<String>)>>,
    pub net_text: String,
    pub net_to: String,
    /// What we announce when we start or join a net.
    pub net_caps: ion::link::net::Caps,
    pub waterfall: Waterfall,
    pub tab: Tab,
    pub center: Center,
    host: Option<crate::host::HostHandle>,
    last_state: State,
    dirty: bool,
    last_saved: Instant,
    /// Radio settings changed: they apply on the next start.
    pub restart: bool,
    /// Screenshots to this folder every 15 s; quit after this many seconds.
    pub shots: Option<std::path::PathBuf>,
    pub quit_after: Option<f64>,
    started: Instant,
    next_shot: f64,
    /// The window has been set to the saved size (eframe restores the last
    /// one, which may be the other mode's).
    sized: bool,
}

fn utc() -> String {
    chrono::Utc::now().format("%H:%M:%S").to_string()
}

impl App {
    pub fn new(
        cfg: Config,
        loopback: bool,
        engine: Sender<Msg>,
        events: Receiver<UiEvent>,
        spectrum_return: Sender<Vec<f32>>,
        threads: Threads,
    ) -> Self {
        let _ = engine.send(Msg::Ion(Some(threads.ion.audio.clone())));
        let cmds = &threads.ion.cmds;
        let _ = cmds.send(IonCmd::SetCall(cfg.call.clone()));
        let _ = cmds.send(IonCmd::SetOfdmDb(cfg.ofdm_db));
        if loopback && let Some(lb) = threads.radio.ctl.loopback() {
            // The simulated station "LOOP" answers calls and echoes text.
            let _ = lb.send(LoopCmd::IonPeer(true));
        }
        let mut app = Self {
            loopback,
            engine,
            events,
            spectrum_return,
            threads,
            status: Status::default(),
            radio: RadioInfo::default(),
            link: None,
            net: None,
            hist: History::new(),
            notes: VecDeque::new(),
            net_log: VecDeque::new(),
            chat: Vec::new(),
            files: Vec::new(),
            demux: Demux::new(),
            file_path: String::new(),
            sent_bytes: 0,
            compose: String::new(),
            call_draft: None,
            audio_devices: None,
            audio_scan: None,
            net_text: String::new(),
            net_to: String::new(),
            net_caps: Default::default(),
            waterfall: Waterfall::new(),
            tab: Tab::Listen,
            center: Center::Chat,
            host: None,
            last_state: State::Idle,
            dirty: false,
            last_saved: Instant::now(),
            restart: false,
            shots: None,
            quit_after: None,
            started: Instant::now(),
            next_shot: 10.0,
            sized: false,
            cfg,
        };
        app.note(format!(
            "Ion {} · {}",
            env!("CARGO_PKG_VERSION"),
            if loopback {
                "loopback (LOOP answers)"
            } else {
                app.cfg.radio.backend.short()
            }
        ));
        if app.cfg.call.trim().is_empty() {
            app.tab = Tab::Setup;
            app.note("! set your call");
        } else if app.cfg.answer {
            app.cmd(IonCmd::Listen);
        }
        if app.cfg.host_enabled {
            app.set_host(true);
        }
        app
    }

    pub fn cmd(&self, c: IonCmd) {
        let _ = self.threads.ion.cmds.send(c);
    }

    pub fn note(&mut self, text: impl Into<String>) {
        if self.notes.len() >= LOG_LINES {
            self.notes.pop_front();
        }
        self.notes.push_back((utc(), text.into()));
    }

    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    pub fn state(&self) -> State {
        self.link.as_ref().map_or(State::Idle, |s| s.state)
    }

    /// A link is up or being set up.
    pub fn linked(&self) -> bool {
        matches!(
            self.state(),
            State::Connecting | State::Connected | State::Disconnecting
        )
    }

    pub fn connect(&mut self, call: &str) {
        let call = call.trim().to_ascii_uppercase();
        if call.is_empty() || call.contains(char::is_whitespace) {
            return;
        }
        self.cfg.target = call.clone();
        self.cfg.remember(&call);
        self.mark_dirty();
        self.hist.new_link();
        self.new_chat();
        self.cmd(IonCmd::Connect(call));
        self.tab = Tab::Link;
    }

    pub fn disconnect(&mut self) {
        self.cmd(IonCmd::Disconnect);
    }

    /// Drops the link at once, without transmitting.
    pub fn abort(&mut self) {
        self.cmd(IonCmd::Stop);
        if self.cfg.answer {
            self.cmd(IonCmd::Listen);
        }
    }

    pub fn set_answer(&mut self, on: bool) {
        self.cfg.answer = on;
        self.mark_dirty();
        if !self.linked() && self.net.is_none() {
            self.cmd(if on { IonCmd::Listen } else { IonCmd::Stop });
        }
    }

    pub fn send_text(&mut self) {
        let text = std::mem::take(&mut self.compose);
        if text.trim().is_empty() {
            return;
        }
        let mut line = text.trim_end().to_string();
        line.push('\n');
        self.sent_bytes += line.len() as u64;
        self.chat.push(Chat::Text {
            sent: true,
            text: line.clone(),
            ends_at: self.sent_bytes,
        });
        self.cmd(IonCmd::Send(line.into_bytes()));
    }

    /// A new chat (a new link).
    fn new_chat(&mut self) {
        self.chat.clear();
        self.files.clear();
        self.demux = Demux::new();
        self.sent_bytes = 0;
    }

    /// Sends the file at `path` on the link: a picture re-coded to arrive
    /// in strips, anything else as it is (`crate::ion_files`).
    pub fn send_file(&mut self, path: &std::path::Path) {
        match ion_files::prepare(path) {
            Err(e) => self.note(format!("! {e}")),
            Ok((bytes, kind, name)) => {
                let mut x = Xfer::new(true, name.clone(), kind, bytes.len());
                self.sent_bytes += bytes.len() as u64;
                x.ends_at = self.sent_bytes;
                if kind == Kind::Image
                    && let Some(i) = bytes.iter().position(|&b| b == b'\n')
                {
                    x.picture(&bytes[i + 1..]);
                }
                self.note(format!("Sending {name} ({})", theme::bytes(bytes.len() as u64)));
                self.files.push(x);
                self.chat.push(Chat::File(self.files.len() - 1));
                self.cmd(IonCmd::Send(bytes));
            }
        }
    }

    /// One piece of what the link brought: text, or part of a file.
    fn received_item(&mut self, item: Item) {
        match item {
            Item::Text(text) => {
                // A chat entry per line: the rest of an unfinished one
                // first, then each new one.
                for line in text.split_inclusive('\n') {
                    match self.chat.last_mut() {
                        Some(Chat::Text {
                            sent: false,
                            text: last,
                            ..
                        }) if !last.ends_with('\n') => last.push_str(line),
                        _ => self.chat.push(Chat::Text {
                            sent: false,
                            text: line.to_string(),
                            ends_at: 0,
                        }),
                    }
                }
            }
            Item::Start(h) => {
                self.note(format!("Receiving {} ({})", h.name, theme::bytes(h.size as u64)));
                self.files.push(Xfer::new(false, h.name, h.kind, h.size));
                self.chat.push(Chat::File(self.files.len() - 1));
            }
            Item::Progress(n) => {
                let Some(x) = self.files.iter_mut().rev().find(|x| !x.sent && !x.done) else {
                    return;
                };
                x.have = n;
                if x.kind == Kind::Image
                    && let Some((_, b)) = self.demux.current()
                {
                    x.picture(b);
                }
            }
            Item::Done(h, data) => {
                let Some(x) = self.files.iter_mut().rev().find(|x| !x.sent && !x.done) else {
                    return;
                };
                x.have = h.size;
                x.done = true;
                let img = if x.kind == Kind::Image { x.picture(&data) } else { None };
                match ion_files::save_received(&h.name, &data, img.as_ref()) {
                    Ok(path) => {
                        let msg = format!("Saved {}", path.display());
                        x.saved = Some(path);
                        self.note(msg);
                    }
                    Err(e) => {
                        x.error = Some(e.clone());
                        self.note(format!("! {}: {e}", h.name));
                    }
                }
            }
        }
    }

    /// Files we sent: how far the link has acknowledged them.
    fn sent_progress(&mut self, acked: u64) {
        for x in self.files.iter_mut().filter(|x| x.sent && !x.done) {
            let start = x.ends_at - x.size as u64;
            x.have = acked.saturating_sub(start).min(x.size as u64) as usize;
            x.done = acked >= x.ends_at;
        }
    }

    /// Starts or stops the Winlink host interface.
    pub fn set_host(&mut self, on: bool) {
        if !on {
            if self.host.take().is_some() {
                self.cmd(IonCmd::AttachHost(None));
                self.note("Host off");
            }
            return;
        }
        if self.host.is_some() {
            return;
        }
        let port = self.cfg.host_port;
        match crate::host::spawn(port, self.threads.ion.cmds.clone(), self.threads.ui_tx.clone()) {
            Ok(h) => {
                self.note(format!("Host 127.0.0.1:{port} / {}", port + 1));
                self.host = Some(h);
            }
            Err(e) => {
                self.note(format!("! host port {port}: {e}"));
                self.cfg.host_enabled = false;
            }
        }
    }

    /// Loopback only: the simulated net (`Some(true)` members joining ours,
    /// `Some(false)` a net of its own to join, `None` off).
    pub fn loop_net(&self, kind: Option<bool>) {
        if let Some(lb) = self.threads.radio.ctl.loopback() {
            let _ = lb.send(LoopCmd::IonNet(kind));
        }
    }

    pub fn host_on(&self) -> bool {
        self.host.is_some()
    }

    pub fn set_drive(&mut self, pct: u32) {
        self.cfg.drive = pct;
        self.threads.radio.ctl.drive(pct);
        self.mark_dirty();
    }

    pub fn set_ofdm_db(&mut self, db: f32) {
        self.cfg.ofdm_db = db;
        self.cmd(IonCmd::SetOfdmDb(db));
        self.mark_dirty();
    }

    /// The sound card lists, starting a read the first time.
    pub fn audio_devices(&mut self) -> Option<&(Vec<String>, Vec<String>)> {
        if let Some(rx) = &self.audio_scan
            && let Ok(lists) = rx.try_recv()
        {
            self.audio_devices = Some(lists);
            self.audio_scan = None;
        }
        if self.audio_devices.is_none() && self.audio_scan.is_none() {
            self.rescan_audio_devices();
        }
        self.audio_devices.as_ref()
    }

    pub fn rescan_audio_devices(&mut self) {
        if self.audio_scan.is_some() {
            return;
        }
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(crate::soundcard::list_devices());
        });
        self.audio_scan = Some(rx);
    }

    pub fn set_call(&mut self, call: String) {
        self.cfg.call = call.trim().to_ascii_uppercase();
        self.cmd(IonCmd::SetCall(self.cfg.call.clone()));
        if self.cfg.answer && !self.cfg.call.is_empty() {
            self.cmd(IonCmd::Listen);
        }
        self.mark_dirty();
    }

    // -- events ------------------------------------------------------------

    fn pump(&mut self) {
        while let Ok(ev) = self.events.try_recv() {
            match ev {
                UiEvent::Spectrum(row) => {
                    self.waterfall.push_row(&row);
                    let _ = self.spectrum_return.try_send(row);
                }
                UiEvent::Status(s) => self.status = s,
                UiEvent::Warning(w) => self.note(format!("! {w}")),
                UiEvent::Radio(info) => {
                    if let Some(d) = info.drive {
                        self.cfg.drive = d;
                    }
                    self.radio = info;
                }
                UiEvent::Ion(ev) => self.ion_event(ev),
                UiEvent::Host(note) => self.note(note),
            }
        }
    }

    fn ion_event(&mut self, ev: IonEvent) {
        match ev {
            IonEvent::Status(s) => {
                if s.state != self.last_state {
                    self.state_changed(self.last_state, &s);
                    self.last_state = s.state;
                }
                self.hist.on_status(&s);
                self.sent_progress(s.text_acked);
                if s.state == State::Connected {
                    self.hist.heard_peer(&s.peer.clone(), self.hist.last_here);
                }
                self.link = Some(s);
            }
            IonEvent::Received(bytes) => {
                for item in self.demux.feed(&bytes) {
                    self.received_item(item);
                }
            }
            IonEvent::Note(text) => {
                let text = text.strip_prefix("Ion: ").unwrap_or(&text).to_string();
                // The state pill already says so.
                if text != "Listening" {
                    self.note(text);
                }
            }
            IonEvent::NetStatus(s) => {
                if s.closed && self.net.as_ref().is_some_and(|n| !n.closed) {
                    self.net_line("", "", "Net closed", true);
                }
                self.net = Some(s);
            }
            IonEvent::Net(e) => self.net_event(e),
        }
    }

    fn state_changed(&mut self, from: State, s: &IonStatus) {
        match s.state {
            // Someone called us while we listened.
            State::Connecting | State::Connected if from == State::Listening => {
                self.hist.new_link();
                self.new_chat();
                if self.tab == Tab::Listen {
                    self.tab = Tab::Link;
                }
                if s.state == State::Connected && !s.peer.is_empty() {
                    self.cfg.remember(&s.peer);
                    self.mark_dirty();
                }
            }
            State::Connected if !s.peer.is_empty() => {
                self.cfg.remember(&s.peer);
                self.mark_dirty();
            }
            State::Disconnected | State::Failed if self.cfg.answer && self.net.is_none() => {
                self.cmd(IonCmd::Listen);
            }
            _ => {}
        }
    }

    fn net_line(&mut self, from: &str, to: &str, text: &str, note: bool) {
        if self.net_log.len() >= LOG_LINES {
            self.net_log.pop_front();
        }
        self.net_log.push_back(NetLine {
            time: utc(),
            from: from.to_string(),
            to: to.to_string(),
            text: text.to_string(),
            note,
        });
    }

    fn net_event(&mut self, e: NetEvent) {
        match e {
            NetEvent::CheckedIn { call, slot, via } => {
                let how = via.map_or(String::new(), |v| format!(" via {v}"));
                self.net_line(&call, "", &format!("in, slot {slot}{how}"), true);
            }
            NetEvent::Joined { ncs, net, slot } => {
                self.net_line(&ncs, "", &format!("joined {net}, slot {slot}"), true);
            }
            NetEvent::Message {
                from,
                to,
                data,
                overheard,
            } => {
                let to = to.unwrap_or_else(|| "ALL".into());
                let mut text = String::from_utf8_lossy(&data).trim_end().to_string();
                if overheard {
                    text.insert_str(0, "(overheard) ");
                }
                self.net_line(&from, &to, &text, false);
            }
            NetEvent::Missed { call } => self.net_line(&call, "", "missed", true),
            NetEvent::Relayed { source, via } => self.net_line(&source, "", &format!("relayed by {via}"), true),
            NetEvent::Routed { call, via } => {
                let how = via.map_or("direct".to_string(), |v| format!("via {v}"));
                self.net_line(&call, "", &how, true);
            }
            NetEvent::Closed => self.net_line("", "", "Net closed", true),
        }
    }

    // -- drawing -----------------------------------------------------------

    fn header(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 14.0;
            ui.label(RichText::new("ION HF").strong().size(15.0).color(theme::TEXT));
            egui::Frame::new()
                .fill(theme::BG)
                .stroke(Stroke::new(1.0, theme::LINE))
                .corner_radius(6)
                .inner_margin(Margin::same(2))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 2.0;
                        for (t, name) in [
                            (Tab::Link, "Link"),
                            (Tab::Listen, "Listen"),
                            (Tab::Net, "Net"),
                            (Tab::Setup, "Setup"),
                        ] {
                            if ui.selectable_label(self.tab == t, name).clicked() {
                                self.tab = t;
                            }
                        }
                    });
                });
            ui.label(RichText::new("Call").color(theme::MUTED));
            let call = ui.add(
                egui::TextEdit::singleline(&mut self.cfg.target)
                    .font(egui::TextStyle::Monospace)
                    .desired_width(110.0)
                    .char_limit(10),
            );
            if call.changed() {
                self.cfg.target = self.cfg.target.to_ascii_uppercase();
                self.mark_dirty();
            }
            let enter = call.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if self.linked() {
                if ui.button(RichText::new("Disconnect").color(theme::BAD)).clicked() {
                    self.disconnect();
                }
                if ui.button("Abort").clicked() {
                    self.abort();
                }
            } else {
                let can = !self.cfg.target.trim().is_empty() && !self.cfg.call.trim().is_empty();
                let go = ui.add_enabled(can, theme::primary("Connect")).clicked();
                if (go || enter) && can {
                    let t = self.cfg.target.clone();
                    self.connect(&t);
                }
            }

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                if ui.small_button("Mini").clicked() {
                    self.set_mini(ui.ctx(), true);
                }
                let busy = self.link.as_ref().is_some_and(|s| s.busy);
                theme::lamp(ui, "Host", self.host_on(), theme::NET);
                theme::lamp(ui, "RX", busy && !self.status.keyed, theme::OK);
                theme::lamp(ui, "TX", self.status.keyed, theme::TX);
                ui.add_space(8.0);
                let (color, text, detail) = self.state_pill();
                theme::pill(ui, color, &text, &detail);
            });
        });
    }

    /// Colour, word and detail for the state pill.
    pub fn state_pill(&self) -> (egui::Color32, String, String) {
        if let Some(n) = &self.net
            && !n.closed
        {
            let role = if n.ncs { "NCS" } else { "Member" };
            return (theme::NET, format!("Net · {role}"), n.net.clone());
        }
        let s = self.link.as_ref();
        let peer = s.map_or(String::new(), |s| s.peer.clone());
        match self.state() {
            State::Idle => (theme::MUTED, "Idle".into(), String::new()),
            State::Listening => (theme::US, "Listening".into(), self.cfg.call.clone()),
            State::Connecting => (theme::WARN, "Calling".into(), peer),
            State::Connected => (
                theme::OK,
                "Connected".into(),
                format!("{peer} {}", theme::clock(s.map_or(0.0, |s| s.connected_s))),
            ),
            State::Disconnecting => (theme::WARN, "Closing".into(), peer),
            State::Disconnected => (theme::MUTED, "Closed".into(), peer),
            State::Failed => (theme::BAD, "Failed".into(), peer),
        }
    }

    pub fn set_mini(&mut self, ctx: &egui::Context, on: bool) {
        self.cfg.mini = on;
        self.mark_dirty();
        let size = if on { MINI_SIZE } else { FULL_SIZE };
        ctx.send_viewport_cmd(egui::ViewportCommand::InnerSize(egui::vec2(size[0], size[1])));
    }

    /// `--shots` and `--quit`: a screenshot when one is due (saved as it
    /// arrives), and close when the time is up.
    fn scripted(&mut self, ctx: &egui::Context) {
        let t = self.started.elapsed().as_secs_f64();
        if let Some(dir) = &self.shots
            && t >= self.next_shot
        {
            self.next_shot += 15.0;
            let path = dir.join(format!("ion_{:03}s.png", t as u64));
            ctx.request_screenshot(move |img| {
                let [w, h] = img.size;
                let rgba: Vec<u8> = img.pixels.iter().flat_map(|c| c.to_array()).collect();
                if let Some(buf) = image::RgbaImage::from_raw(w as u32, h as u32, rgba)
                    && let Err(e) = buf.save(&path)
                {
                    log::warn!("{}: {e}", path.display());
                }
            });
        }
        if self.quit_after.is_some_and(|q| t >= q) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }

    /// The third of the passband the slice is in now, if the link is on a
    /// slice level: ours as sender (the peer's pick), the peer's as
    /// receiver (ours, which its next burst follows).
    pub fn slice_band(&self) -> Option<(f32, f32, bool)> {
        use ion::level::{Guard, LADDER, Wave};
        let s = self.link.as_ref().filter(|s| s.state == State::Connected)?;
        let sending = s.role == ion::link::Role::Sender;
        let level = if sending { Some(s.level) } else { self.hist.peer_level };
        if LADDER.get(level?)?.wave != Wave::Ofdm(Guard::Slice) {
            return None;
        }
        let slot = if sending { s.slot_tx } else { s.slot_rx } as usize;
        let c = ion::link::audio::CENTER_HZ + ion::slots::OFFSETS_HZ[slot.min(2)];
        Some((c - 325.0, c + 325.0, sending))
    }

    fn waterfall_panel(&mut self, ui: &mut egui::Ui, height: f32) {
        let (lo, hi) = ion::link::audio::ofdm_band_hz();
        let c = ion::link::audio::CENTER_HZ;
        let cursor = Cursor {
            center_hz: c,
            marks: [lo - c, hi - c],
            keyed: self.status.keyed,
        };
        let bands: Vec<Band> = self
            .slice_band()
            .map(|(lo, hi, us)| Band {
                lo_hz: lo,
                hi_hz: hi,
                color: if us { theme::US } else { theme::PEER },
                label: "Slice".into(),
            })
            .into_iter()
            .collect();
        self.waterfall.show(ui, height, &cursor, &bands);
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.pump();
        let ctx = ui.ctx().clone();
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.abort();
        }
        // A file dropped on the window: send it on a link, else hold it.
        let dropped: Vec<std::path::PathBuf> = ctx.input(|i| {
            i.raw
                .dropped_files
                .iter()
                .map(|f| f.path().to_path_buf())
                .filter(|p| !p.as_os_str().is_empty())
                .collect()
        });
        for p in dropped {
            if self.state() == State::Connected {
                self.send_file(&p);
            } else {
                self.file_path = p.display().to_string();
            }
        }
        if !self.sized {
            self.sized = true;
            // Mini always at its size; full only if it was left at Mini's.
            let width = ctx.input(|i| i.viewport().inner_rect.map_or(FULL_SIZE[0], |r| r.width()));
            if self.cfg.mini || width < 800.0 {
                let size = if self.cfg.mini { MINI_SIZE } else { FULL_SIZE };
                ctx.send_viewport_cmd(egui::ViewportCommand::InnerSize(egui::vec2(size[0], size[1])));
            }
        }
        self.scripted(&ctx);
        if self.cfg.mini {
            egui::CentralPanel::default()
                .frame(egui::Frame::new().fill(theme::INSET).inner_margin(Margin::same(12)))
                .show(ui, |ui| views::mini::show(self, ui));
        } else {
            egui::Panel::top("header")
                .frame(
                    egui::Frame::new()
                        .fill(theme::PANEL)
                        .inner_margin(Margin::symmetric(14, 8)),
                )
                .show(ui, |ui| self.header(ui));
            if self.tab != Tab::Setup {
                egui::Panel::bottom("air")
                    .resizable(false)
                    .frame(
                        egui::Frame::new()
                            .fill(theme::PANEL)
                            .inner_margin(Margin::symmetric(14, 8)),
                    )
                    .show(ui, |ui| {
                        let h = if self.tab == Tab::Link { 130.0 } else { 100.0 };
                        self.waterfall_panel(ui, h);
                    });
            }
            match self.tab {
                Tab::Link => views::link::show(self, ui),
                Tab::Listen => views::listen::show(self, ui),
                Tab::Net => views::net::show(self, ui),
                Tab::Setup => views::setup::show(self, ui),
            }
        }

        if self.dirty && self.last_saved.elapsed() > Duration::from_secs(2) {
            self.cfg.save();
            self.dirty = false;
            self.last_saved = Instant::now();
        }
        ctx.request_repaint_after(Duration::from_millis(50));
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        self.cmd(IonCmd::Stop);
        let _ = self.engine.send(Msg::Shutdown);
        if let Some(h) = self.threads.engine.take() {
            let _ = h.join();
        }
        self.threads.radio.stop();
        self.cfg.save();
    }
}
