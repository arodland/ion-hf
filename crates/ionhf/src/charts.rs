//! Time charts for the Link screen: levels as steps, SNRs as dots, data as
//! a line. The last five minutes of the link.

use eframe::egui::{self, Color32, Pos2, Rect, Sense, Stroke, Vec2};

use crate::theme;

/// Seconds shown.
const WINDOW: f64 = 300.0;

pub enum Style {
    Steps,
    Dots,
    Line,
}

pub struct Series<'a> {
    pub color: Color32,
    pub points: &'a [(f64, f32)],
    pub style: Style,
    /// The latest value as text, drawn at the right end in the series'
    /// colour ("L6", "+4.2", "5.6").
    pub last: Option<String>,
}

/// One chart: `title` top left, `legend` top right, y range and ticks,
/// optional horizontal reference line.
#[allow(clippy::too_many_arguments)]
pub fn chart(
    ui: &mut egui::Ui,
    title: &str,
    legend: &[(Color32, &str)],
    height: f32,
    t_now: f64,
    y: (f32, f32),
    y_ticks: &[(f32, String)],
    series: &[Series],
    hline: Option<f32>,
) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), height), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 3.0, theme::INSET);
    // Room for tick labels left, the title above, time labels below and the
    // latest values right.
    let plot = Rect::from_min_max(rect.min + Vec2::new(46.0, 24.0), rect.max - Vec2::new(64.0, 20.0));
    let t0 = (t_now - WINDOW).max(0.0);
    let span = (t_now - t0).max(60.0);
    let xf = |t: f64| plot.min.x + ((t - t0) / span) as f32 * plot.width();
    let yf = |v: f32| plot.max.y - (v - y.0) / (y.1 - y.0) * plot.height();
    let small = egui::FontId::monospace(12.0);
    for (v, l) in y_ticks {
        let yy = yf(*v);
        p.line_segment(
            [Pos2::new(plot.min.x, yy), Pos2::new(plot.max.x, yy)],
            Stroke::new(1.0, theme::LINE),
        );
        p.text(
            Pos2::new(plot.min.x - 8.0, yy),
            egui::Align2::RIGHT_CENTER,
            l,
            small.clone(),
            theme::MUTED,
        );
    }
    if let Some(v) = hline {
        let yy = yf(v);
        p.line_segment(
            [Pos2::new(plot.min.x, yy), Pos2::new(plot.max.x, yy)],
            Stroke::new(1.5, Color32::from_rgb(0x6F, 0x7B, 0x88)),
        );
    }
    let step = if span > 200.0 {
        60.0
    } else if span > 60.0 {
        30.0
    } else {
        15.0
    };
    let mut tt = (t0 / step).ceil() * step;
    while tt <= t0 + span {
        p.text(
            Pos2::new(xf(tt), plot.max.y + 2.0),
            egui::Align2::CENTER_TOP,
            theme::clock(tt),
            small.clone(),
            theme::MUTED,
        );
        tt += step;
    }
    for s in series {
        let vis: Vec<Pos2> = s
            .points
            .iter()
            .filter(|(t, _)| *t >= t0)
            .map(|&(t, v)| Pos2::new(xf(t), yf(v.clamp(y.0, y.1))))
            .collect();
        match s.style {
            Style::Steps => {
                let mut line: Vec<Pos2> = Vec::new();
                for q in &vis {
                    if let Some(last) = line.last().copied() {
                        line.push(Pos2::new(q.x, last.y));
                    }
                    line.push(*q);
                }
                if let Some(last) = vis.last() {
                    line.push(Pos2::new(xf(t_now), last.y));
                }
                if line.len() >= 2 {
                    p.add(egui::Shape::line(line, Stroke::new(3.0, s.color)));
                }
            }
            Style::Line => {
                let mut line = vis.clone();
                if let Some(last) = vis.last() {
                    line.push(Pos2::new(xf(t_now), last.y));
                }
                if line.len() >= 2 {
                    p.add(egui::Shape::line(line, Stroke::new(3.0, s.color)));
                }
            }
            Style::Dots => {
                for q in vis {
                    p.circle_filled(q, 4.0, s.color);
                }
            }
        }
    }
    // Latest values at the right edge, nudged apart when they collide.
    let mut used: Vec<f32> = Vec::new();
    for s in series {
        let (Some(text), Some(&(_, v))) = (&s.last, s.points.last()) else {
            continue;
        };
        let mut y = yf(v.clamp(y.0, y.1));
        while used.iter().any(|u| (u - y).abs() < 15.0) {
            y += 15.0;
        }
        used.push(y);
        p.text(
            Pos2::new(plot.max.x + 8.0, y),
            egui::Align2::LEFT_CENTER,
            text,
            egui::FontId::monospace(13.0),
            s.color,
        );
    }
    let cap = egui::FontId::proportional(13.0);
    p.text(
        rect.min + Vec2::new(10.0, 4.0),
        egui::Align2::LEFT_TOP,
        title,
        cap.clone(),
        theme::TEXT,
    );
    let mut x = rect.max.x - 10.0;
    for (c, name) in legend.iter().rev() {
        let r = p.text(
            Pos2::new(x, rect.min.y + 4.0),
            egui::Align2::RIGHT_TOP,
            *name,
            cap.clone(),
            *c,
        );
        x = r.min.x - 12.0;
    }
}
