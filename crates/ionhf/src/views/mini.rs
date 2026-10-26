//! Mini: the compact window, for running behind Winlink Express or Pat.

use eframe::egui::{self, RichText};

use crate::app::App;
use crate::theme;

pub fn show(app: &mut App, ui: &mut egui::Ui) {
    let s = app.link.clone();
    ui.horizontal(|ui| {
        ui.label(RichText::new("ION HF").strong().color(theme::TEXT));
        ui.label(RichText::new(&app.cfg.call).monospace().color(theme::MUTED));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            let busy = s.as_ref().is_some_and(|s| s.busy);
            theme::lamp(ui, "Host", app.host_on(), theme::NET);
            theme::lamp(ui, "RX", busy && !app.status.keyed, theme::OK);
            theme::lamp(ui, "TX", app.status.keyed, theme::TX);
        });
    });
    ui.add_space(8.0);
    let (color, text, detail) = app.state_pill();
    theme::pill(ui, color, &text, &detail);
    ui.add_space(8.0);

    // SNR heard here on a −10…+20 dB bar.
    theme::caption(ui, "SNR");
    let snr = app.hist.last_here;
    let frac = snr.map_or(0.0, |v| ((v + 10.0) / 30.0).clamp(0.0, 1.0));
    theme::bar(ui, frac, theme::OK, &format!("{} dB", theme::db(snr)));
    ui.add_space(6.0);

    ui.columns(3, |c| {
        let level = s.as_ref().map_or("—".into(), |s| {
            if s.role == ion::link::Role::Sender {
                theme::level(s.level)
            } else {
                app.hist.peer_level.map_or("—".into(), theme::level)
            }
        });
        c[0].vertical(|ui| {
            theme::caption(ui, "Level");
            ui.label(RichText::new(level).monospace().strong());
        });
        let rate = s.as_ref().filter(|s| s.connected_s > 5.0).map_or("—".into(), |s| {
            format!(
                "{:.0} bit/s",
                (s.bytes_acked + s.bytes_received) as f64 * 8.0 / s.connected_s
            )
        });
        c[1].vertical(|ui| {
            theme::caption(ui, "Rate");
            ui.label(RichText::new(rate).monospace().strong());
        });
        let data = s
            .as_ref()
            .map_or("—".into(), |s| theme::bytes(s.bytes_acked + s.bytes_received));
        c[2].vertical(|ui| {
            theme::caption(ui, "Data");
            ui.label(RichText::new(data).monospace().strong());
        });
    });
    ui.add_space(6.0);
    if let Some(s) = &s {
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
    }
    ui.with_layout(egui::Layout::bottom_up(egui::Align::Min), |ui| {
        ui.horizontal(|ui| {
            if app.linked() && ui.button(RichText::new("Abort").color(theme::BAD)).clicked() {
                app.abort();
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("Expand").clicked() {
                    app.set_mini(ui.ctx(), false);
                }
            });
        });
    });
}
