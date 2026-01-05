use eframe::egui;

use crate::app::CameraState;

#[derive(Default)]
pub struct CameraScreen {}

impl CameraScreen {
    pub fn new() -> Self {
        Self {}
    }

    pub fn draw(&mut self, ui: &mut egui::Ui, cameras: &[CameraState]) {
        ui.heading("Camera Mode");
        if let Some(camera_state) = cameras.get(0) {
            if let Some(texture) = &camera_state.texture {
                // 利用可能な描画領域のサイズを取得
                let available_size = ui.available_size();
                let image_size = texture.size_vec2();

                if image_size.y > 0.0 {
                    let image_aspect = image_size.x / image_size.y;
                    let available_aspect = available_size.x / available_size.y;

                    let new_size = if image_aspect > available_aspect {
                        // 画像が描画領域より横長 -> 幅に合わせる
                        let new_height = available_size.x / image_aspect;
                        egui::vec2(available_size.x, new_height)
                    } else {
                        // 画像が描画領域より縦長 -> 高さに合わせる
                        let new_width = available_size.y * image_aspect;
                        egui::vec2(new_width, available_size.y)
                    };

                    // 画像を中央揃えで描画
                    ui.centered_and_justified(|ui| {
                        ui.add(egui::Image::new(texture).fit_to_exact_size(new_size));
                    });
                }
            } else {
                ui.centered_and_justified(|ui| {
                    ui.label(&camera_state.connection_status);
                });
            }
        } else {
            ui.centered_and_justified(|ui| {
                ui.label("No camera available.");
            });
        }
    }
}
