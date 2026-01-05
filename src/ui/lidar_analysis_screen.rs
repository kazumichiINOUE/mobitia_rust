use eframe::egui;

use crate::app::LidarState;
use crate::config::Config;

#[derive(Default)]
pub struct LidarAnalysisScreen {}

impl LidarAnalysisScreen {
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
        ui.heading("LiDAR Feature Analysis");
        let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
        let rect = response.rect;
        *lidar_draw_rect = Some(rect);

        // --- 描画設定 ---
        let side = rect.height();
        let square_rect = egui::Rect::from_center_size(rect.center(), egui::vec2(side, side));

        let half_view_size = config.slam.online_slam_view_size / 2.0;
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

        // ロボットの原点 (0,0) を描画
        let robot_origin_screen = to_screen.transform_pos(egui::pos2(0.0, 0.0));
        painter.circle_filled(robot_origin_screen, 5.0, egui::Color32::from_rgb(255, 0, 0));

        // 各LiDARの点群を描画 (フェーズ1: 通常の点と法線)
        for lidar_state in lidars {
            let rotation = lidar_state.rotation;
            let origin = lidar_state.origin;

            for point in &lidar_state.points {
                let px_raw = point.0;
                let py_raw = point.1;
                let edge_ness = point.4;
                let nx_raw = point.5;
                let ny_raw = point.6;
                let corner_ness = point.7;

                // Lidarの回転を適用
                let px_rotated = px_raw * rotation.cos() - py_raw * rotation.sin();
                let py_rotated = px_raw * rotation.sin() + py_raw * rotation.cos();

                let nx_rotated = nx_raw * rotation.cos() - ny_raw * rotation.sin();
                let ny_rotated = nx_raw * rotation.sin() + ny_raw * rotation.cos();

                // ワールド座標に変換 (Lidarの原点オフセットを加える)
                let world_x = px_rotated + origin.x;
                let world_y = py_rotated + origin.y;

                let screen_pos = to_screen.transform_pos(egui::pos2(world_x, world_y));

                let corner_threshold = 0.5; // この閾値は調整可能
                if corner_ness <= corner_threshold {
                    // コーナーでない点
                    // edge_ness に基づいて色を決定
                    let color = egui::Color32::from_rgb(
                        (edge_ness * 255.0) as u8,         // エッジらしさが高いほど赤が強く
                        ((1.0 - edge_ness) * 255.0) as u8, // 低いほど緑が強く
                        0,                                 // 青は常に0
                    );
                    painter.circle_filled(screen_pos, 2.0, color);

                    // 法線ベクトルを描画 (edge_ness が閾値以上の場合のみ)
                    if edge_ness > 0.1 {
                        // ある程度エッジらしい点のみ法線を描画
                        let normal_len = 0.1; // 法線ベクトルの長さ (ワールド座標)
                        let normal_end_x = world_x + nx_rotated * normal_len;
                        let normal_end_y = world_y + ny_rotated * normal_len;
                        let normal_end_screen =
                            to_screen.transform_pos(egui::pos2(normal_end_x, normal_end_y));
                        painter.line_segment(
                            [screen_pos, normal_end_screen],
                            egui::Stroke::new(1.0, egui::Color32::YELLOW),
                        );
                    }
                }
            }
        }

        // 各LiDARの点群を描画 (フェーズ2: コーナー点のみを最前面に描画)
        for lidar_state in lidars {
            let rotation = lidar_state.rotation;
            let origin = lidar_state.origin;

            for point in &lidar_state.points {
                let px_raw = point.0;
                let py_raw = point.1;
                let corner_ness = point.7;

                // Lidarの回転を適用
                let px_rotated = px_raw * rotation.cos() - py_raw * rotation.sin();
                let py_rotated = px_raw * rotation.sin() + py_raw * rotation.cos();

                // ワールド座標に変換 (Lidarの原点オフセットを加える)
                let world_x = px_rotated + origin.x;
                let world_y = py_rotated + origin.y;

                let screen_pos = to_screen.transform_pos(egui::pos2(world_x, world_y));

                let corner_threshold = 0.5; // この閾値は調整可能
                if corner_ness > corner_threshold {
                    // コーナー点のみを描画
                    painter.circle_filled(screen_pos, 5.0, egui::Color32::from_rgb(255, 0, 255));
                    // マゼンタ色で大きく描画
                }
            }
        }
    }
}
