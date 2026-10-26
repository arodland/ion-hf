//! Listen: who to call, who has been heard calling, and what happened.

use eframe::egui::{self, Margin, RichText};

use crate::app::App;
use crate::theme;

pub fn show(app: &mut App, ui: &mut egui::Ui) {
    egui::Panel::left("connect")
        .resizable(false)
        .exact_size(290.0)
        .frame(egui::Frame::new().fill(theme::PANEL).inner_margin(Margin::same(16)))
        .show(ui, |ui| connect(app, ui));
    egui::CentralPanel::default()
        .frame(egui::Frame::new().fill(theme::INSET).inner_margin(Margin::same(16)))
        .show(ui, |ui| {
            ui.horizontal_top(|ui| {
                let w = ui.available_width();
                let down = egui::Layout::top_down(egui::Align::Min);
                ui.allocate_ui_with_layout(egui::vec2(w * 0.62, 340.0), down, |ui| heard(app, ui));
                ui.add_space(16.0);
                ui.allocate_ui_with_layout(egui::vec2(ui.available_width(), 340.0), down, |ui| band(app, ui));
            });
            ui.add_space(14.0);
            notes(app, ui);
        });
}

/// The band here: the slot meter's three thirds, and the bursts heard
/// lately.
fn band(app: &App, ui: &mut egui::Ui) {
    ui.label(RichText::new("Band").strong());
    ui.add_space(4.0);
    match app.link.as_ref() {
        Some(s) => theme::band_meter(ui, s.band, s.slot_rx),
        None => {
            ui.label(RichText::new("—").color(theme::MUTED));
        }
    }
    ui.add_space(12.0);
    theme::caption(ui, "Heard lately");
    let rows: Vec<_> = app.hist.rows.iter().rev().filter(|r| !r.tx).take(8).collect();
    if rows.is_empty() {
        ui.label(RichText::new("—").color(theme::MUTED));
    }
    egui::Grid::new("air_rows")
        .num_columns(3)
        .spacing([12.0, 4.0])
        .show(ui, |ui| {
            for r in rows {
                ui.label(RichText::new(format!("{:.0}", r.t)).monospace().color(theme::MUTED));
                let snr = if r.lost > 0 {
                    RichText::new(format!("{} lost {}", theme::db(r.snr), r.lost))
                        .monospace()
                        .color(theme::BAD)
                } else {
                    RichText::new(format!("{} dB", theme::db(r.snr)))
                        .monospace()
                        .color(theme::OK)
                };
                ui.label(snr);
                ui.label(RichText::new(&r.what).monospace());
                ui.end_row();
            }
        });
}

fn connect(app: &mut App, ui: &mut egui::Ui) {
    theme::caption(ui, "Call");
    let edit = ui.add(
        egui::TextEdit::singleline(&mut app.cfg.target)
            .font(egui::FontId::monospace(22.0))
            .desired_width(f32::INFINITY)
            .char_limit(10)
            .margin(Margin::symmetric(10, 8)),
    );
    if edit.changed() {
        app.cfg.target = app.cfg.target.to_ascii_uppercase();
        app.mark_dirty();
    }
    let enter = edit.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
    ui.add_space(4.0);
    let can = !app.cfg.target.trim().is_empty() && !app.cfg.call.trim().is_empty() && !app.linked();
    let size = egui::vec2(ui.available_width(), 40.0);
    let go = ui.add_enabled(can, theme::primary("Connect").min_size(size)).clicked();
    if (go || enter) && can {
        let t = app.cfg.target.clone();
        app.connect(&t);
    }
    if app.linked()
        && ui
            .add(egui::Button::new("Abort").min_size(egui::vec2(size.x, 30.0)))
            .clicked()
    {
        app.abort();
    }

    ui.add_space(16.0);
    theme::caption(ui, "Recent");
    let recent = app.cfg.recent.clone();
    if recent.is_empty() {
        ui.label(RichText::new("—").color(theme::MUTED));
    }
    for call in recent {
        let b = egui::Button::new(RichText::new(&call).monospace().strong())
            .min_size(egui::vec2(ui.available_width(), 30.0));
        if ui.add_enabled(!app.linked(), b).clicked() {
            app.cfg.target = call.clone();
            app.connect(&call);
        }
    }

    ui.add_space(16.0);
    theme::caption(ui, "Answer");
    ui.horizontal(|ui| {
        let on = app.cfg.answer;
        if ui.selectable_label(on, "Auto").clicked() && !on {
            app.set_answer(true);
        }
        if ui.selectable_label(!on, "Off").clicked() && on {
            app.set_answer(false);
        }
    });
}

fn age(at: std::time::Instant) -> String {
    let s = at.elapsed().as_secs();
    match s {
        0..60 => "now".into(),
        60..3600 => format!("{} m", s / 60),
        _ => format!("{} h", s / 3600),
    }
}

fn heard(app: &mut App, ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.label(RichText::new("Heard").strong());
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(
                RichText::new(format!("{}", app.hist.heard.len()))
                    .monospace()
                    .color(theme::MUTED),
            );
        });
    });
    ui.add_space(4.0);
    if app.hist.heard.is_empty() {
        ui.label(RichText::new("—").color(theme::MUTED));
        return;
    }
    let mut call_to: Option<String> = None;
    let linked = app.linked();
    egui::ScrollArea::vertical()
        .id_salt("heard")
        .max_height(300.0)
        .show(ui, |ui| {
            egui::Grid::new("heard_rows")
                .num_columns(6)
                .spacing([22.0, 8.0])
                .striped(true)
                .show(ui, |ui| {
                    for head in ["Call", "To", "SNR", "Last", "Seen", ""] {
                        theme::caption(ui, head);
                    }
                    ui.end_row();
                    for h in &app.hist.heard {
                        ui.label(RichText::new(&h.call).monospace().strong().color(theme::US));
                        ui.label(
                            RichText::new(if h.to.is_empty() { "—" } else { &h.to })
                                .monospace()
                                .color(theme::MUTED),
                        );
                        ui.label(RichText::new(theme::db(h.snr)).monospace());
                        ui.label(RichText::new(age(h.at)).monospace().color(theme::MUTED));
                        ui.label(RichText::new(h.count.to_string()).monospace().color(theme::MUTED));
                        if ui.add_enabled(!linked, egui::Button::new("Connect")).clicked() {
                            call_to = Some(h.call.clone());
                        }
                        ui.end_row();
                    }
                });
        });
    if let Some(c) = call_to {
        app.connect(&c);
    }
}

fn notes(app: &App, ui: &mut egui::Ui) {
    theme::caption(ui, "Log");
    egui::ScrollArea::vertical()
        .id_salt("notes")
        .stick_to_bottom(true)
        .auto_shrink([false, false])
        .show(ui, |ui| {
            for (t, text) in &app.notes {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(t).monospace().color(theme::MUTED));
                    let c = if text.starts_with('!') {
                        theme::WARN
                    } else {
                        theme::TEXT
                    };
                    ui.label(RichText::new(text).monospace().color(c));
                });
            }
        });
}
