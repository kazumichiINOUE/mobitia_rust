use eframe::egui;
use eframe::egui::ecolor::Hsva;

pub struct RotatingScanDemo;

impl RotatingScanDemo {
    pub fn new() -> Self {
        Self
    }

    pub fn update_and_draw(&mut self, ui: &mut egui::Ui) {
        let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
        let rect = response.rect;

        let time = ui.input(|i| i.time);
        let center = rect.center();
        let radius = rect.height().min(rect.width()) * 0.4;
        let angle = time as f32 * std::f32::consts::TAU / 4.0;

        let scan_angle_width = std::f32::consts::FRAC_PI_3;
        let num_segments = 60;
        let inner_radius_for_fan = radius * 0.25;

        let start_scan_angle = angle - scan_angle_width;
        let mut current_segment_angle = start_scan_angle;

        for i in 0..num_segments {
            let next_segment_angle =
                start_scan_angle + scan_angle_width * ((i + 1) as f32 / num_segments as f32);

            let p1 = center
                + radius * egui::vec2(current_segment_angle.cos(), -current_segment_angle.sin());
            let p2 =
                center + radius * egui::vec2(next_segment_angle.cos(), -next_segment_angle.sin());

            let inner_p1 = center
                + inner_radius_for_fan
                    * egui::vec2(current_segment_angle.cos(), -current_segment_angle.sin());
            let inner_p2 = center
                + inner_radius_for_fan
                    * egui::vec2(next_segment_angle.cos(), -next_segment_angle.sin());

            let hue =
                current_segment_angle.rem_euclid(std::f32::consts::TAU) / std::f32::consts::TAU;

            let color: egui::Color32 = Hsva {
                h: hue,
                s: 1.0,
                v: 1.0,
                a: 0.1,
            }
            .into();

            painter.add(egui::Shape::convex_polygon(
                vec![inner_p1, p1, p2],
                color,
                egui::Stroke::NONE,
            ));
            painter.add(egui::Shape::convex_polygon(
                vec![inner_p1, p2, inner_p2],
                color,
                egui::Stroke::NONE,
            ));
            current_segment_angle = next_segment_angle;
        }

        painter.text(
            center,
            egui::Align2::CENTER_CENTER,
            "Signal Lost",
            egui::FontId::proportional(40.0),
            egui::Color32::WHITE,
        );
    }
}
