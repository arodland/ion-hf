//! Net: run one as net control or join one; the roster; the traffic.

use crate::ion_engine::IonCmd;
use eframe::egui::{self, Margin, RichText};
use ion::link::net::Caps;

use crate::app::App;
use crate::theme;

pub fn show(app: &mut App, ui: &mut egui::Ui) {
    egui::Panel::left("net_ctl")
        .resizable(false)
        .exact_size(270.0)
        .frame(egui::Frame::new().fill(theme::PANEL).inner_margin(Margin::same(16)))
        .show(ui, |ui| {
            egui::ScrollArea::vertical()
                .id_salt("net_ctl_scroll")
                .show(ui, |ui| controls(app, ui))
        });
    egui::Panel::right("traffic")
        .resizable(false)
        .exact_size(380.0)
        .frame(egui::Frame::new().fill(theme::PANEL).inner_margin(Margin::same(16)))
        .show(ui, |ui| traffic(app, ui));
    egui::CentralPanel::default()
        .frame(egui::Frame::new().fill(theme::INSET).inner_margin(Margin::same(16)))
        .show(ui, |ui| roster(app, ui));
}

fn active(app: &App) -> bool {
    app.net.as_ref().is_some_and(|n| !n.closed)
}

fn controls(app: &mut App, ui: &mut egui::Ui) {
    theme::caption(ui, "Net");
    let edit = ui.add_enabled(
        !active(app),
        egui::TextEdit::singleline(&mut app.cfg.net)
            .font(egui::TextStyle::Monospace)
            .desired_width(f32::INFINITY)
            .hint_text("any"),
    );
    if edit.changed() {
        app.cfg.net = app.cfg.net.to_ascii_uppercase();
        app.mark_dirty();
    }
    ui.add_space(6.0);
    let w = ui.available_width();
    if !active(app) {
        let has_call = !app.cfg.call.trim().is_empty();
        ui.horizontal(|ui| {
            let b = egui::vec2((w - 8.0) / 2.0, 34.0);
            let start = ui.add_enabled(
                has_call && !app.cfg.net.trim().is_empty() && !app.linked(),
                egui::Button::new(RichText::new("Start").strong()).min_size(b),
            );
            if start.clicked() {
                app.loop_net(Some(true));
                app.cmd(IonCmd::NetStart(app.cfg.net.clone()));
            }
            if ui
                .add_enabled(has_call && !app.linked(), egui::Button::new("Join").min_size(b))
                .clicked()
            {
                app.loop_net(Some(false));
                app.cmd(IonCmd::NetJoin(app.cfg.net.clone()));
            }
        });
    } else {
        let ncs = app.net.as_ref().is_some_and(|n| n.ncs);
        if ncs {
            let mut open = app.net.as_ref().is_some_and(|n| n.open);
            if ui.checkbox(&mut open, "Open").changed() {
                app.cmd(IonCmd::NetOpen(open));
            }
            if ui
                .add(egui::Button::new(RichText::new("Close net").color(theme::BAD)).min_size(egui::vec2(w, 32.0)))
                .clicked()
            {
                app.cmd(IonCmd::NetClose);
            }
        }
        if ui
            .add(egui::Button::new("Leave").min_size(egui::vec2(w, 30.0)))
            .clicked()
        {
            app.cmd(IonCmd::Stop);
            app.loop_net(None);
            app.net = None;
            if app.cfg.answer {
                app.cmd(IonCmd::Listen);
            }
        }
    }

    ui.add_space(14.0);
    if let Some(n) = &app.net {
        let rows = [
            ("Role", if n.ncs { "NCS".to_string() } else { "Member".to_string() }),
            ("NCS", n.ncs_call.clone().unwrap_or_else(|| "—".into())),
            ("Slot", n.slot.map_or("—".into(), |s| s.to_string())),
            ("Round", n.rounds.to_string()),
            ("Via", n.relay.clone().unwrap_or_else(|| "—".into())),
            ("Queue", n.pending.to_string()),
        ];
        theme::kv(ui, "net_kv", &rows);
        ui.add_space(14.0);
    }

    theme::caption(ui, "Caps");
    let mut c: Caps = match &app.net {
        Some(n) if active(app) => n.caps,
        _ => app.net_caps,
    };
    let mut changed = false;
    ui.add_enabled_ui(!active(app), |ui| {
        changed |= ui.checkbox(&mut c.relay, "Relay").changed();
        changed |= ui.checkbox(&mut c.wide, "Wide").changed();
        changed |= ui.checkbox(&mut c.qsy, "QSY").changed();
        changed |= ui.checkbox(&mut c.attended, "Attended").changed();
    });
    if changed {
        app.net_caps = c;
        app.cmd(IonCmd::NetCaps(c));
    }
}

fn roster(app: &mut App, ui: &mut egui::Ui) {
    let Some(n) = app.net.clone() else {
        ui.label(RichText::new("No net").color(theme::MUTED));
        return;
    };
    ui.horizontal(|ui| {
        ui.label(RichText::new("Roster").strong());
        ui.label(RichText::new(&n.net).monospace().color(theme::NET));
    });
    ui.add_space(6.0);
    if n.ncs {
        let mut to: Option<String> = None;
        egui::ScrollArea::vertical().id_salt("roster").show(ui, |ui| {
            egui::Grid::new("roster_rows")
                .num_columns(8)
                .spacing([20.0, 8.0])
                .striped(true)
                .show(ui, |ui| {
                    for h in ["Slot", "Call", "Up", "Down", "Via", "Cap", "Missed", ""] {
                        theme::caption(ui, h);
                    }
                    ui.end_row();
                    for r in &n.roster {
                        ui.label(RichText::new(r.slot.to_string()).monospace().color(theme::MUTED));
                        ui.label(RichText::new(&r.call).monospace().strong());
                        ui.label(RichText::new(theme::db(r.snr_up)).monospace());
                        ui.label(RichText::new(theme::db(r.snr_down)).monospace());
                        ui.label(
                            RichText::new(r.via.as_deref().unwrap_or("—"))
                                .monospace()
                                .color(theme::MUTED),
                        );
                        ui.label(RichText::new(format!("L{}", r.cap)).monospace().color(theme::MUTED));
                        let mc = if r.missed > 0 { theme::BAD } else { theme::MUTED };
                        ui.label(RichText::new(r.missed.to_string()).monospace().color(mc));
                        if ui.small_button("To").clicked() {
                            to = Some(r.call.clone());
                        }
                        ui.end_row();
                    }
                });
        });
        if let Some(c) = to {
            app.net_to = c;
        }
    } else {
        egui::Grid::new("members")
            .num_columns(2)
            .spacing([20.0, 8.0])
            .striped(true)
            .show(ui, |ui| {
                theme::caption(ui, "Slot");
                theme::caption(ui, "Call");
                ui.end_row();
                for (slot, call) in &n.members {
                    ui.label(RichText::new(slot.to_string()).monospace().color(theme::MUTED));
                    ui.label(RichText::new(call).monospace().strong());
                    ui.end_row();
                }
            });
    }
}

fn traffic(app: &mut App, ui: &mut egui::Ui) {
    ui.label(RichText::new("Traffic").strong());
    ui.add_space(6.0);
    let in_net = active(app);
    let ncs = app.net.as_ref().is_some_and(|n| n.ncs);
    let input_h = if ncs { 96.0 } else { 64.0 };
    egui::ScrollArea::vertical()
        .id_salt("net_log")
        .stick_to_bottom(true)
        .auto_shrink([false, false])
        .max_height((ui.available_height() - input_h).max(60.0))
        .show(ui, |ui| {
            for l in &app.net_log {
                ui.horizontal_wrapped(|ui| {
                    ui.label(RichText::new(&l.time).monospace().color(theme::MUTED));
                    if !l.from.is_empty() {
                        ui.label(RichText::new(&l.from).monospace().strong().color(theme::PEER));
                    }
                    if !l.to.is_empty() {
                        ui.label(RichText::new(format!("› {}", l.to)).monospace().color(theme::MUTED));
                    }
                    let c = if l.note { theme::MUTED } else { theme::TEXT };
                    ui.label(RichText::new(&l.text).color(c));
                });
            }
        });
    ui.add_space(6.0);
    if ncs {
        ui.horizontal(|ui| {
            theme::caption(ui, "To");
            ui.add(
                egui::TextEdit::singleline(&mut app.net_to)
                    .font(egui::TextStyle::Monospace)
                    .desired_width(120.0)
                    .hint_text("ALL"),
            );
        });
    }
    ui.horizontal(|ui| {
        let edit = ui.add_enabled(
            in_net,
            egui::TextEdit::singleline(&mut app.net_text).desired_width(ui.available_width() - 64.0),
        );
        let enter = edit.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
        if (ui.add_enabled(in_net, egui::Button::new("Send")).clicked() || enter) && in_net {
            let text = std::mem::take(&mut app.net_text);
            if !text.trim().is_empty() {
                let to = app.net_to.trim().to_ascii_uppercase();
                if ncs && !to.is_empty() && to != "ALL" {
                    app.cmd(IonCmd::NetSendTo(to, text.into_bytes()));
                } else {
                    app.cmd(IonCmd::NetSend(text.into_bytes()));
                }
            }
        }
    });
}
