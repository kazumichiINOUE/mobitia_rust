use eframe::egui;
use rand::Rng;

struct Ripple {
    center: egui::Pos2,
    spawn_time: f64,
    max_radius: f32,
    duration: f32,
}

pub struct ExpandingRippleDemo {
    ripples: Vec<Ripple>,
    last_ripple_spawn_time: f64,
}

impl ExpandingRippleDemo {
    pub fn new() -> Self {
        Self {
            ripples: Vec::new(),
            last_ripple_spawn_time: 0.0,
        }
    }

    pub fn update_and_draw(&mut self, ui: &mut egui::Ui) {
        let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
        let rect = response.rect;

        let time = ui.input(|i| i.time);
        let mut rng = rand::thread_rng();

        if time - self.last_ripple_spawn_time > 0.4 {
            self.last_ripple_spawn_time = time;
            let center_x = rng.gen_range(rect.left()..=rect.right());
            let center_y = rng.gen_range(rect.top()..=rect.bottom());
            let center = egui::pos2(center_x, center_y);

            self.ripples.push(Ripple {
                center,
                spawn_time: time,
                max_radius: rng.gen_range(50.0..=300.0),
                duration: rng.gen_range(2.0..=5.0),
            });
        }

        let current_ripples = std::mem::take(&mut self.ripples);
        self.ripples = current_ripples
            .into_iter()
            .filter_map(|ripple| {
                let elapsed_time = (time - ripple.spawn_time) as f32;
                if elapsed_time > ripple.duration {
                    return None;
                }

                let progress = elapsed_time / ripple.duration;
                let current_radius = ripple.max_radius * progress;

                let alpha = (1.0 - progress).powf(2.0) * 1.0;
                let stroke_alpha = (alpha * 255.0) as u8;

                let color = egui::Color32::from_rgba_unmultiplied(0, 200, 255, stroke_alpha);

                let stroke_width = (1.0 - progress) * 3.5 + 0.5;

                painter.circle_stroke(
                    ripple.center,
                    current_radius,
                    egui::Stroke::new(stroke_width, color),
                );
                ui.ctx().request_repaint();

                Some(ripple)
            })
            .collect();

        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "Expanding Ripple Demo",
            egui::FontId::proportional(40.0),
            egui::Color32::WHITE,
        );
    }
}
