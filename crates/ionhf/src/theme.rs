//! Colours and the small widgets every screen uses: dark, instrument-like,
//! our station blue and the peer orange.

use eframe::egui::{self, Color32, CornerRadius, Margin, RichText, Stroke, Vec2};

pub const BG: Color32 = Color32::from_rgb(0x0E, 0x11, 0x15);
pub const PANEL: Color32 = Color32::from_rgb(0x15, 0x1A, 0x20);
pub const INSET: Color32 = Color32::from_rgb(0x12, 0x16, 0x1B);
pub const RAISED: Color32 = Color32::from_rgb(0x1B, 0x21, 0x29);
pub const LINE: Color32 = Color32::from_rgb(0x2A, 0x32, 0x3C);
pub const TEXT: Color32 = Color32::from_rgb(0xDC, 0xE3, 0xEA);
pub const MUTED: Color32 = Color32::from_rgb(0xA9, 0xB4, 0xC0);
pub const US: Color32 = Color32::from_rgb(0x5A, 0xA9, 0xFF);
pub const PEER: Color32 = Color32::from_rgb(0xFF, 0xA9, 0x4D);
pub const OK: Color32 = Color32::from_rgb(0x4C, 0xC3, 0x8A);
pub const BAD: Color32 = Color32::from_rgb(0xFF, 0x8F, 0x8F);
pub const WARN: Color32 = Color32::from_rgb(0xF5, 0xC5, 0x42);
pub const NET: Color32 = Color32::from_rgb(0xB4, 0x9B, 0xFF);
pub const TX: Color32 = Color32::from_rgb(0xFF, 0x5D, 0x5D);

pub fn apply(ctx: &egui::Context) {
    ctx.set_theme(egui::Theme::Dark);
    let mut v = egui::Visuals::dark();
    v.panel_fill = PANEL;
    v.window_fill = PANEL;
    v.extreme_bg_color = BG;
    v.faint_bg_color = INSET;
    v.code_bg_color = INSET;
    v.window_stroke = Stroke::new(1.0, LINE);
    v.override_text_color = None;
    v.widgets.noninteractive.bg_fill = PANEL;
    v.widgets.noninteractive.weak_bg_fill = PANEL;
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, LINE);
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, TEXT);
    v.widgets.inactive.bg_fill = RAISED;
    v.widgets.inactive.weak_bg_fill = RAISED;
    v.widgets.inactive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x3A, 0x45, 0x52));
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, TEXT);
    v.widgets.hovered.weak_bg_fill = Color32::from_rgb(0x24, 0x30, 0x3D);
    v.widgets.hovered.bg_fill = Color32::from_rgb(0x24, 0x30, 0x3D);
    v.widgets.active.weak_bg_fill = Color32::from_rgb(0x2B, 0x3A, 0x4A);
    v.selection.bg_fill = Color32::from_rgb(0x21, 0x67, 0xB5);
    v.selection.stroke = Stroke::new(1.0, TEXT);
    ctx.set_visuals_of(egui::Theme::Dark, v);
    ctx.global_style_mut(|s| {
        s.spacing.item_spacing = Vec2::new(8.0, 6.0);
        s.spacing.button_padding = Vec2::new(10.0, 5.0);
        s.spacing.interact_size.y = 26.0;
    });
}

/// A panel's frame: fill, border, padding.
pub fn card() -> egui::Frame {
    egui::Frame::new()
        .fill(PANEL)
        .stroke(Stroke::new(1.0, LINE))
        .corner_radius(CornerRadius::same(6))
        .inner_margin(Margin::same(12))
}

/// A small caption above a value.
pub fn caption(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).size(11.0).color(MUTED));
}

/// A keying lamp: TX, RX, Busy, Host.
pub fn lamp(ui: &mut egui::Ui, label: &str, on: bool, color: Color32) {
    let (fill, stroke, text) = if on {
        (color.gamma_multiply(0.25), color, color)
    } else {
        (Color32::TRANSPARENT, Color32::from_rgb(0x3A, 0x45, 0x52), MUTED)
    };
    egui::Frame::new()
        .fill(fill)
        .stroke(Stroke::new(1.0, stroke))
        .corner_radius(CornerRadius::same(4))
        .inner_margin(Margin::symmetric(9, 4))
        .show(ui, |ui| {
            ui.label(RichText::new(label).monospace().size(12.0).color(text).strong());
        });
}

/// A pill showing the link state. Painted at its own size, so it reads the
/// same in any layout (the header lays out right to left).
pub fn pill(ui: &mut egui::Ui, color: Color32, text: &str, detail: &str) {
    let p = ui.painter();
    let g1 = p.layout_no_wrap(text.to_string(), egui::FontId::proportional(13.0), TEXT);
    let g2 = p.layout_no_wrap(detail.to_string(), egui::FontId::monospace(12.0), MUTED);
    let gap = if detail.is_empty() { 0.0 } else { 8.0 + g2.size().x };
    let size = Vec2::new(12.0 + 8.0 + 7.0 + g1.size().x + gap + 12.0, 26.0);
    let (r, _) = ui.allocate_exact_size(size, egui::Sense::hover());
    let p = ui.painter();
    p.rect(
        r,
        CornerRadius::same(13),
        color.gamma_multiply(0.14),
        Stroke::new(1.0, color.gamma_multiply(0.55)),
        egui::StrokeKind::Inside,
    );
    p.circle_filled(egui::pos2(r.min.x + 16.0, r.center().y), 4.0, color);
    let x = r.min.x + 27.0;
    p.galley(egui::pos2(x, r.center().y - g1.size().y / 2.0), g1.clone(), TEXT);
    if !detail.is_empty() {
        let x2 = x + g1.size().x + 8.0;
        p.galley(egui::pos2(x2, r.center().y - g2.size().y / 2.0), g2, MUTED);
    }
}

/// A progress bar with mono text on it.
pub fn bar(ui: &mut egui::Ui, frac: f32, color: Color32, text: &str) {
    ui.add(
        egui::ProgressBar::new(frac.clamp(0.0, 1.0))
            .fill(color.gamma_multiply(0.45))
            .text(RichText::new(text).monospace().color(TEXT)),
    );
}

/// The slot meter (`ion::slots`): how clean each third of the passband
/// is here (100 less its score), by frequency, the one our receivers pick
/// in the accent colour.
pub fn band_meter(ui: &mut egui::Ui, scores: [f32; 3], pick: u8) {
    let w = ui.available_width();
    let (rect, _) = ui.allocate_exact_size(egui::vec2(w, 52.0), egui::Sense::hover());
    let p = ui.painter_at(rect);
    let col = w / 3.0;
    for (i, (k, name)) in [(1usize, "Low"), (0, "Mid"), (2, "High")].into_iter().enumerate() {
        let clean = ((100.0 - scores[k]) / 100.0).clamp(0.0, 1.0);
        let x0 = rect.min.x + i as f32 * col + 3.0;
        let x1 = x0 + col - 6.0;
        let (top, bot) = (rect.min.y, rect.min.y + 32.0);
        p.rect_filled(
            egui::Rect::from_min_max(egui::pos2(x0, top), egui::pos2(x1, bot)),
            3.0,
            INSET,
        );
        let on = k as u8 == pick;
        let y = bot - clean * (bot - top);
        p.rect_filled(
            egui::Rect::from_min_max(egui::pos2(x0, y), egui::pos2(x1, bot)),
            3.0,
            if on { US } else { MUTED.gamma_multiply(0.5) },
        );
        p.text(
            egui::pos2((x0 + x1) / 2.0, bot + 4.0),
            egui::Align2::CENTER_TOP,
            format!("{name} {:.0}", clean * 100.0),
            egui::FontId::proportional(12.0),
            if on { TEXT } else { MUTED },
        );
    }
}

/// "+6.2" or "—".
pub fn db(v: Option<f32>) -> String {
    v.map_or("—".into(), |v| format!("{v:+.1}"))
}

/// "L6 QPSK 1/3".
pub fn level(l: usize) -> String {
    match ion::level::LADDER.get(l) {
        Some(x) => ion::sounding::level_name(l, x.wave),
        None => "—".into(),
    }
}

/// A level without "OFDM", which its modulation already implies.
pub fn level_short(l: usize) -> String {
    level(l).replacen(" OFDM", "", 1)
}

/// Bytes as "812 B" or "5.6 kB".
pub fn bytes(n: u64) -> String {
    if n < 1000 {
        format!("{n} B")
    } else {
        format!("{:.1} kB", n as f64 / 1000.0)
    }
}

/// Seconds as m:ss or h:mm:ss.
pub fn clock(s: f64) -> String {
    let s = s.max(0.0) as u64;
    if s >= 3600 {
        format!("{}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
    } else {
        format!("{}:{:02}", s / 60, s % 60)
    }
}
