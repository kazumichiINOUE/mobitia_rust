use eframe::egui;
use nalgebra::{Isometry2, Point2};

use crate::config::Config;

#[derive(Default)]
pub struct SlamScreen {}

impl SlamScreen {
    pub fn new() -> Self {
        Self {}
    }

    #[allow(clippy::too_many_arguments)]
    pub fn draw(
        &mut self,
        ui: &mut egui::Ui,
        config: &Config,
        lidar_draw_rect: &mut Option<egui::Rect>,
        current_robot_pose: &Isometry2<f32>,
        robot_trajectory: &[(egui::Pos2, f32)],
        current_map_points: &[(Point2<f32>, f64)],
        latest_scan_for_draw: &[(f32, f32, f32, f32, f32, f32, f32, f32)],
    ) {
        ui.heading("SLAM Mode");

        let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
        let rect = response.rect;
        *lidar_draw_rect = Some(rect);
        painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(20, 20, 20));

        // Draw robot pose
        let robot_pose = current_robot_pose;

        // In online SLAM, always center the view on the robot
        let robot_center_world = egui::pos2(robot_pose.translation.x, robot_pose.translation.y);
        let map_view_size = config.slam.online_slam_view_size;
        let map_view_rect = egui::Rect::from_center_size(
            robot_center_world,
            egui::vec2(map_view_size, map_view_size),
        );

        // Y-axis inversion for screen coordinates
        let mut inverted_map_view_rect = map_view_rect;
        inverted_map_view_rect.min.y = map_view_rect.max.y;
        inverted_map_view_rect.max.y = map_view_rect.min.y;
        let to_screen = egui::emath::RectTransform::from_to(
            inverted_map_view_rect,
            rect, // The actual drawing area
        );

        // Draw trajectory lines and orientation triangles
        if robot_trajectory.len() > 1 {
            // Trajectory line
            let trajectory_line_points: Vec<egui::Pos2> = robot_trajectory
                .iter()
                .map(|(world_pos, _angle)| to_screen.transform_pos(*world_pos))
                .collect();
            let line_stroke = egui::Stroke::new(1.0, egui::Color32::WHITE);
            painter.add(egui::Shape::line(trajectory_line_points, line_stroke));

            // 向きを示すくの字マーカー
            let chevron_color = egui::Color32::from_rgb(200, 200, 100); // 彩度を落とした黄色
            let chevron_stroke = egui::Stroke::new(1.0, chevron_color); // 細めの線
            let chevron_size = 8.0; // 小さめのサイズ

            for (i, (world_pos, angle)) in robot_trajectory.iter().enumerate() {
                if i > 0 {
                    let center_screen = to_screen.transform_pos(*world_pos);

                    // くの字の頂点を定義
                    // 先端
                    let p1 = center_screen + egui::vec2(angle.cos(), -angle.sin()) * chevron_size;
                    // 左後ろの点 (先端から約135度)
                    let angle_left = *angle + (135.0f32).to_radians();
                    let p2 = center_screen
                        + egui::vec2(angle_left.cos(), -angle_left.sin()) * chevron_size * 0.7;
                    // 右後ろの点 (先端から約-135度)
                    let angle_right = *angle - (135.0f32).to_radians();
                    let p3 = center_screen
                        + egui::vec2(angle_right.cos(), -angle_right.sin()) * chevron_size * 0.7;

                    // 2本の線分でくの字を描画
                    painter.add(egui::Shape::line_segment([p2, p1], chevron_stroke));
                    painter.add(egui::Shape::line_segment([p3, p1], chevron_stroke));
                }
            }
        }

        // Draw the map points
        for (point, probability) in current_map_points {
            let screen_pos = to_screen.transform_pos(egui::pos2(point.x, point.y));
            if rect.contains(screen_pos) {
                let prob_f32 = *probability as f32;

                if (prob_f32 - 0.5).abs() < 1e-6 {
                    continue; // Don't draw unknown cells
                }

                if prob_f32 > 0.5 {
                    // Occupied
                    let intensity = (prob_f32 - 0.5) / 0.5;
                    let color = egui::Color32::from_rgb(
                        (intensity * 100.0) as u8,
                        (intensity * 100.0) as u8,
                        (intensity * 255.0) as u8,
                    );
                    painter.circle_filled(screen_pos, 2.0, color);
                } else {
                    // Free
                    let color = egui::Color32::from_gray(35);
                    painter.circle_filled(screen_pos, 1.0, color);
                }
            }
        }

        // Draw the latest scan data in a different color
        for point in latest_scan_for_draw {
            let local_point = nalgebra::Point2::new(point.0, point.1);
            let world_point = current_robot_pose * local_point;
            let screen_pos = to_screen.transform_pos(egui::pos2(world_point.x, world_point.y));
            if rect.contains(screen_pos) {
                painter.circle_filled(screen_pos, 2.5, egui::Color32::YELLOW);
            }
        }

        // Draw the robot's current pose
        let robot_pos_on_screen = to_screen.transform_pos(egui::pos2(
            robot_pose.translation.x,
            robot_pose.translation.y,
        ));
        let angle = robot_pose.rotation.angle();
        painter.circle_filled(robot_pos_on_screen, 5.0, egui::Color32::GREEN);
        painter.line_segment(
            [
                robot_pos_on_screen,
                robot_pos_on_screen + egui::vec2(angle.cos(), -angle.sin()) * 20.0,
            ],
            egui::Stroke::new(2.0, egui::Color32::GREEN),
        );
    }
}
