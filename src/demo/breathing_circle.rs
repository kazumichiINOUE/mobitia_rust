use eframe::egui;

pub struct BreathingCircleDemo;

impl BreathingCircleDemo {
    pub fn new() -> Self {
        Self
    }

    pub fn update_and_draw(&mut self, ui: &mut egui::Ui) {
        let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
        let rect = response.rect;

        let time = ui.input(|i| i.time) as f32;
        let center = rect.center();

        let cycle_duration = 8.0;
        let progress = (time / cycle_duration * std::f32::consts::TAU).cos() * -0.5 + 0.5;

        let min_radius_ratio = 0.10;
        let max_radius_ratio = 0.40;
        let radius_ratio = min_radius_ratio + (max_radius_ratio - min_radius_ratio) * progress;
        let radius = rect.height().min(rect.width()) * radius_ratio;

        let min_alpha = 0.3;
        let max_alpha = 0.8;
        let alpha = min_alpha + (max_alpha - min_alpha) * progress;

        let color = egui::Color32::from_rgba_unmultiplied(128, 128, 128, (alpha * 255.0) as u8);

        painter.circle_filled(center, radius, color);
        ui.ctx().request_repaint();

        painter.text(
            center,
            egui::Align2::CENTER_CENTER,
            "Breathing Circle Demo",
            egui::FontId::proportional(40.0),
            egui::Color32::WHITE,
        );
    }
}
