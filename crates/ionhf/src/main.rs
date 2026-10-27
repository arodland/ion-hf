//! Ion HF: an ARQ data modem for Winlink (over VARA-style host ports),
//! keyboard links and nets.
//!
//! Usage: ionhf [--loopback] [--tci ws://host:port] [--connect CALL]
//! [--send TEXT] [--file PATH] [--net NAME] [--tab link|listen|net|setup]
//! [--view chat|charts|log] [--mini] [--shots DIR] [--quit S] [--loop-snr DB]

// Release builds are GUI-subsystem exes: no console window on launch.
#![cfg_attr(all(not(debug_assertions), not(test)), windows_subsystem = "windows")]

mod app;
mod charts;
mod config;
mod engine;
mod history;
mod host;
mod ion_engine;
mod ion_files;
#[cfg(test)]
mod pipeline_tests;
mod radio;
mod resample;
mod rig;
mod soundcard;
mod theme;
mod views;
mod waterfall;

use std::sync::Arc;

use eframe::egui;

use crate::config::{Backend, Config};
use crate::engine::{Engine, SAMPLE_RATE, Shared};

struct Args {
    loopback: bool,
    tci: Option<String>,
    /// Call this station at start; `send` and `file` go once linked.
    connect: Option<String>,
    send: Option<String>,
    file: Option<String>,
    tab: Option<String>,
    /// The Link view's centre: chat, charts or log.
    view: Option<String>,
    /// Run this net as NCS at start.
    net: Option<String>,
    mini: bool,
    /// Save a screenshot to this folder every 15 s; quit after `quit` s.
    shots: Option<String>,
    quit: Option<f64>,
    /// Loopback only: the simulated channel's SNR, dB.
    loop_snr: Option<f32>,
}

fn parse_args() -> Args {
    let mut args = Args {
        loopback: false,
        tci: None,
        connect: None,
        send: None,
        file: None,
        tab: None,
        view: None,
        net: None,
        mini: false,
        shots: None,
        quit: None,
        loop_snr: None,
    };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--loopback" => args.loopback = true,
            "--tci" => args.tci = it.next(),
            "--connect" => args.connect = it.next(),
            "--send" => args.send = it.next(),
            "--file" => args.file = it.next(),
            "--tab" => args.tab = it.next(),
            "--view" => args.view = it.next(),
            "--net" => args.net = it.next(),
            "--mini" => args.mini = true,
            "--shots" => args.shots = it.next(),
            "--quit" => args.quit = it.next().and_then(|v| v.parse().ok()),
            "--loop-snr" => args.loop_snr = it.next().and_then(|v| v.parse().ok()),
            "-h" | "--help" => {
                println!("ionhf [--loopback] [--tci ws://127.0.0.1:50001] [--connect CALL] [--send TEXT]");
                std::process::exit(0);
            }
            other => eprintln!("ignoring unknown argument {other}"),
        }
    }
    args
}

/// To stderr, or to `ion.log` in [`Config::dir`] for a release build
/// started without a terminal (from Explorer, Finder or a desktop menu).
fn init_logging() {
    let mut builder = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"));
    #[cfg(not(debug_assertions))]
    {
        use std::io::IsTerminal;
        if !std::io::stderr().is_terminal() {
            let dir = Config::dir();
            let file = std::fs::create_dir_all(&dir).and_then(|()| std::fs::File::create(dir.join("ion.log")));
            if let Ok(file) = file {
                builder.target(env_logger::Target::Pipe(Box::new(file)));
                let default_hook = std::panic::take_hook();
                std::panic::set_hook(Box::new(move |info| {
                    log::error!("{info}");
                    default_hook(info);
                }));
            }
        }
    }
    builder.init();
}

fn main() -> eframe::Result {
    init_logging();
    log::info!("ion {} starting", env!("CARGO_PKG_VERSION"));
    let args = parse_args();
    let mut cfg = Config::load();
    let _ = std::fs::create_dir_all(Config::dir());
    if args.mini {
        cfg.mini = true;
    }
    if let Some(url) = args.tci {
        cfg.tci_url = url;
        cfg.radio.backend = Backend::Tci;
    }

    let mut viewport = egui::ViewportBuilder::default()
        .with_title(if args.loopback { "Ion HF (loopback)" } else { "Ion HF" })
        .with_inner_size(if cfg.mini { app::MINI_SIZE } else { app::FULL_SIZE })
        .with_min_inner_size([480.0, 320.0]);
    match eframe::icon_data::from_png_bytes(include_bytes!("../assets/icon.png")) {
        Ok(icon) => viewport = viewport.with_icon(icon),
        Err(e) => log::warn!("window icon: {e}"),
    }
    let options = eframe::NativeOptions {
        viewport,
        // Beside the settings file, so a portable install keeps it too.
        persistence_path: Some(Config::dir().join("window.ron")),
        ..Default::default()
    };

    eframe::run_native(
        "Ion HF",
        options,
        Box::new(move |cc| {
            theme::apply(&cc.egui_ctx);
            let ctx = cc.egui_ctx.clone();
            let repaint = move || ctx.request_repaint();
            let s = &cfg;

            let (rx_prod, rx_cons) = rtrb::RingBuffer::<f32>::new(2 * SAMPLE_RATE);
            let (tx_prod, tx_cons) = rtrb::RingBuffer::<f32>::new(SAMPLE_RATE);
            let shared = Arc::new(Shared::default());
            let (ui_tx, ui_rx) = crossbeam_channel::bounded(16_384);
            let (msg_tx, msg_rx) = crossbeam_channel::unbounded();
            let (spec_tx, spec_rx) = crossbeam_channel::bounded(32);

            let radio = if args.loopback {
                radio::spawn_loopback(
                    s.loopback_snr_db,
                    rx_prod,
                    tx_cons,
                    shared.clone(),
                    ui_tx.clone(),
                    Box::new(repaint.clone()),
                )
            } else if s.radio.backend == Backend::Tci {
                radio::spawn_tci(
                    s.tci_url.clone(),
                    s.tci_rate(),
                    rx_prod,
                    tx_cons,
                    shared.clone(),
                    ui_tx.clone(),
                    Box::new(repaint.clone()),
                )
            } else {
                rig::thread::spawn(
                    s.radio.clone(),
                    rx_prod,
                    tx_cons,
                    shared.clone(),
                    ui_tx.clone(),
                    Box::new(repaint.clone()),
                )
            };

            let ion = ion_engine::spawn(
                s.call.to_ascii_uppercase(),
                msg_tx.clone(),
                ui_tx.clone(),
                Box::new(repaint.clone()),
            );

            let engine = Engine::new(
                rx_cons,
                tx_prod,
                shared,
                radio.ctl.clone(),
                ui_tx.clone(),
                msg_rx,
                spec_rx,
                Box::new(repaint),
            );
            let engine = std::thread::Builder::new()
                .name("dsp".into())
                .spawn(move || engine.run())
                .expect("spawn dsp thread");

            let mut app = app::App::new(
                cfg,
                args.loopback,
                msg_tx,
                ui_rx,
                spec_tx,
                app::Threads {
                    engine: Some(engine),
                    radio,
                    ion,
                    ui_tx,
                },
            );
            if let Some(db) = args.loop_snr
                && let Some(lb) = app.threads.radio.ctl.loopback()
            {
                let _ = lb.send(crate::radio::LoopCmd::SnrDb(db));
            }
            if let Some(call) = &args.connect {
                app.connect(call);
                if let Some(text) = &args.send {
                    app.compose = text.clone();
                    app.send_text();
                }
                if let Some(f) = &args.file {
                    app.send_file(std::path::Path::new(f));
                }
            }
            if let Some(v) = &args.view {
                app.center = match v.as_str() {
                    "charts" => app::Center::Charts,
                    "log" => app::Center::Log,
                    _ => app::Center::Chat,
                };
            }
            if let Some(t) = &args.tab {
                app.tab = match t.as_str() {
                    "link" => app::Tab::Link,
                    "net" => app::Tab::Net,
                    "setup" => app::Tab::Setup,
                    _ => app::Tab::Listen,
                };
            }
            if let Some(net) = &args.net {
                app.cfg.net = net.to_ascii_uppercase();
                app.loop_net(Some(true));
                app.cmd(crate::ion_engine::IonCmd::NetStart(app.cfg.net.clone()));
                app.tab = app::Tab::Net;
            }
            app.shots = args.shots.as_ref().map(std::path::PathBuf::from);
            app.quit_after = args.quit;
            Ok(Box::new(app))
        }),
    )
}
