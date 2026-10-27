//! The modem: what Ion is doing, at a glance. Figures, the link's
//! counts, the groups and who is heard in them, the passband (slice thirds
//! and waterfall), the latest bursts and notes.

use eframe::egui::{self, RichText};

use crate::app::App;
use crate::theme;

/// Bursts and notes shown under the waterfall.
const NOTES: usize = 4;

fn figure(ui: &mut egui::Ui, caption: &str, value: String) {
    ui.vertical_centered(|ui| {
        theme::caption(ui, caption);
        ui.label(RichText::new(value).monospace().size(16.0).strong());
    });
}

/// Caption / value pairs as one centred line.
fn centered(ui: &mut egui::Ui, pairs: &[(&str, RichText)]) {
    let style = ui.style().clone();
    let mut job = egui::text::LayoutJob::default();
    for (i, (k, v)) in pairs.iter().enumerate() {
        let gap = if i == 0 { "" } else { "    " };
        for part in [
            RichText::new(format!("{gap}{k} ")).size(11.0).color(theme::MUTED),
            v.clone(),
        ] {
            part.append_to(&mut job, &style, egui::FontSelection::Default, egui::Align::Center);
        }
    }
    job.halign = egui::Align::Center;
    ui.vertical_centered(|ui| ui.label(job));
}

pub fn show(app: &mut App, ui: &mut egui::Ui) {
    let s = app.link.clone();
    ui.columns(4, |c| {
        figure(&mut c[0], "S/N", format!("{} dB", theme::db(app.hist.last_here)));
        let level = s.as_ref().filter(|_| app.linked()).and_then(|s| {
            if s.role == ion::link::Role::Sender {
                Some(s.level)
            } else {
                app.hist.peer_level
            }
        });
        let caption = match level.and_then(theme::level_guard) {
            Some(g) => format!("Level · {g}"),
            None => "Level".into(),
        };
        figure(&mut c[1], &caption, level.map_or("—".into(), theme::level_short));
        let rate = s.as_ref().filter(|s| s.connected_s > 5.0).map_or("—".into(), |s| {
            format!(
                "{:.0} bit/s",
                (s.bytes_acked + s.bytes_received) as f64 * 8.0 / s.connected_s
            )
        });
        figure(&mut c[2], "Rate", rate);
        let data = s
            .as_ref()
            .map_or("—".into(), |s| theme::bytes(s.bytes_acked + s.bytes_received));
        figure(&mut c[3], "Data", data);
    });
    ui.add_space(6.0);

    if let Some(s) = s.as_ref().filter(|_| app.linked()) {
        let total = s.bytes_acked + s.pending as u64;
        let frac = if total == 0 {
            0.0
        } else {
            s.bytes_acked as f32 / total as f32
        };
        theme::bar(
            ui,
            frac,
            theme::US,
            &format!("TX {} / {}", theme::bytes(s.bytes_acked), theme::bytes(total)),
        );
        ui.add_space(4.0);
    }

    // The link's counts.
    if let Some(s) = &s {
        let counts = [
            ("Bursts", s.data_bursts.to_string()),
            ("Frames", s.frames_sent.to_string()),
            ("Resent", s.frames_resent.to_string()),
            ("HARQ", format!("{}/{}", s.increments_sent, s.harq_recovered)),
            ("Lost", s.codewords_failed.to_string()),
            ("Polls", s.polls.to_string()),
            ("Ctrl", s.control_bursts.to_string()),
            ("Air", theme::clock(s.airtime_s)),
            ("Allowed", format!("L{}", s.target_level)),
        ]
        .map(|(k, v)| (k, RichText::new(v).monospace()));
        centered(ui, &counts);
        ui.add_space(4.0);
    }

    // Broadcast groups a host program has open, and who is heard in them.
    if let Some(s) = &s
        && !s.groups.is_empty()
    {
        let heard = if s.heard.is_empty() {
            RichText::new("—").monospace().color(theme::MUTED)
        } else {
            let calls: Vec<String> = s.heard.iter().map(|(c, snr)| format!("{c} {snr:+.0}")).collect();
            RichText::new(calls.join("  ")).monospace()
        };
        centered(
            ui,
            &[
                (
                    "Groups",
                    RichText::new(s.groups.join("  ")).monospace().color(theme::NET),
                ),
                ("Heard", heard),
            ],
        );
        ui.add_space(4.0);
    }

    if let Some(s) = &s {
        ui.vertical_centered(|ui| theme::caption(ui, "Band"));
        theme::band_meter(ui, s.band, app.band_use());
    }

    let notes_h = NOTES as f32 * 17.0 + 8.0;
    let buttons_h = if app.linked() { 34.0 } else { 0.0 };
    let fall_h = (ui.available_height() - notes_h - buttons_h - 8.0).max(60.0);
    app.waterfall_panel(ui, fall_h);
    ui.add_space(4.0);

    // The latest bursts beside the latest notes.
    ui.columns(2, |c| {
        for r in app.hist.rows.iter().rev().take(NOTES).rev() {
            let snr = r.snr.map_or(String::new(), |v| format!(" {v:+.0} dB"));
            let lost = if r.lost > 0 {
                format!(" lost {}", r.lost)
            } else {
                String::new()
            };
            let dir = if r.tx { "TX" } else { "RX" };
            let color = if r.tx { theme::US } else { theme::PEER };
            let text = format!("{} {dir} {}{snr}{lost} {:.1} s", theme::clock(r.t), r.what, r.secs);
            c[0].add(egui::Label::new(RichText::new(text).monospace().size(11.0).color(color)).truncate());
        }
        for (t, n) in app.notes.iter().rev().take(NOTES).rev() {
            let color = if n.starts_with('!') { theme::BAD } else { theme::TEXT };
            let style = c[1].style().clone();
            let mut job = egui::text::LayoutJob::default();
            for part in [
                RichText::new(format!("{t} "))
                    .monospace()
                    .size(11.0)
                    .color(theme::MUTED),
                RichText::new(n).size(12.0).color(color),
            ] {
                part.append_to(&mut job, &style, egui::FontSelection::Default, egui::Align::Center);
            }
            c[1].add(egui::Label::new(job).truncate());
        }
    });
    if app.linked() {
        // Either side of the centre line.
        let (mut disconnect, mut abort) = (false, false);
        ui.columns(2, |c| {
            c[0].with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                disconnect = ui.button("Disconnect").clicked();
            });
            c[1].with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                abort = ui.button(RichText::new("Abort").color(theme::BAD)).clicked();
            });
        });
        if disconnect {
            app.cmd(crate::ion_engine::IonCmd::Disconnect);
        }
        if abort {
            app.abort();
        }
    }
}
