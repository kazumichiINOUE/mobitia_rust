use eframe::egui;

// LidarState と Config を main または app モジュールからインポートする必要がある
use crate::app::LidarState;
use crate::config::Config;

#[derive(Default)]
pub struct LidarScreen {}

impl LidarScreen {
    pub fn new() -> Self {
        Self {}
    }

    pub fn draw(
        &mut self,
        ui: &mut egui::Ui,
        config: &Config,
        lidars: &[LidarState],
        lidar_draw_rect: &mut Option<egui::Rect>,
    ) {
        ui.heading("LiDAR Data Visualization");
        let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
        let rect = response.rect;
        *lidar_draw_rect = Some(rect);

        // --- 全Lidar共通の描画設定 ---
        let side = rect.height();
        let square_rect = egui::Rect::from_center_size(rect.center(), egui::vec2(side, side));

        let half_view_size = config.slam.online_slam_view_size / 2.0;
        // ワールド座標からスクリーン座標への変換を定義
        // Y軸を反転させるため、fromに渡すRectのYのmin/maxを入れ替える
        let world_to_screen_rect = egui::Rect::from_min_max(
            egui::pos2(-half_view_size, half_view_size), // ワールドの左上 (min_x, max_y)
            egui::pos2(half_view_size, -half_view_size), // ワールドの右下 (max_x, min_y)
        );
        let to_screen = egui::emath::RectTransform::from_to(world_to_screen_rect, square_rect);

        // 背景とグリッドを描画
        painter.rect_filled(square_rect, 0.0, egui::Color32::from_rgb(20, 20, 20));
        painter.hline(
            square_rect.x_range(),
            to_screen.transform_pos(egui::pos2(0.0, 0.0)).y,
            egui::Stroke::new(0.5, egui::Color32::DARK_GRAY),
        );
        painter.vline(
            to_screen.transform_pos(egui::pos2(0.0, 0.0)).x,
            square_rect.y_range(),
            egui::Stroke::new(0.5, egui::Color32::DARK_GRAY),
        );

        let mut any_lidar_connected = false;

        // --- 各Lidarのデータを描画 ---
        for (i, lidar_state) in lidars.iter().enumerate() {
            if lidar_state.connection_status.starts_with("Connected") {
                any_lidar_connected = true;

                let lidar_origin_world = lidar_state.origin;
                let lidar_color = match i {
                    0 => egui::Color32::GREEN,
                    1 => egui::Color32::YELLOW,
                    2 => egui::Color32::BLUE,
                    _ => egui::Color32::WHITE,
                };

                // Lidarの原点を描画
                let lidar_origin_screen = to_screen.transform_pos(lidar_origin_world.to_pos2());
                painter.circle_filled(lidar_origin_screen, 5.0, lidar_color);
                painter.text(
                    lidar_origin_screen + egui::vec2(10.0, -10.0),
                    egui::Align2::LEFT_BOTTOM,
                    format!("LIDAR {}", lidar_state.id),
                    egui::FontId::default(),
                    egui::Color32::WHITE,
                );

                // Lidarの点群を描画
                for point in &lidar_state.points {
                    let px_raw = point.0;
                    let py_raw = point.1;

                    // Lidarの回転を適用
                    let rotation = lidar_state.rotation;
                    let px_rotated = px_raw * rotation.cos() - py_raw * rotation.sin();
                    let py_rotated = px_raw * rotation.sin() + py_raw * rotation.cos();

                    // ワールド座標に変換 (Lidarの原点オフセットを加える)
                    let world_pos = egui::pos2(px_rotated, py_rotated) + lidar_origin_world;

                    // スクリーン座標に変換
                    let screen_pos = to_screen.transform_pos(world_pos);

                    if square_rect.contains(screen_pos) {
                        painter.circle_filled(screen_pos, 2.0, lidar_color);
                    }
                }
            }
        }

        // どのLidarも接続されていない場合にメッセージを表示
        if !any_lidar_connected {
            ui.allocate_ui_at_rect(rect, |ui| {
                ui.centered_and_justified(|ui| {
                    // 最初のLidarのステータスを代表として表示（暫定）
                    let status = lidars
                        .get(0)
                        .map_or("No Lidars configured", |l| &l.connection_status);
                    let text = egui::RichText::new(status)
                        .color(egui::Color32::WHITE)
                        .font(egui::FontId::proportional(24.0));
                    ui.add(egui::Label::new(text).wrap(true));
                });
            });
        }
    }
}
