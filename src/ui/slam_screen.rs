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
        slam_map_bounding_box: &Option<egui::Rect>,
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
        // 描画範囲を決定
        let map_view_rect = if let Some(bounds) = slam_map_bounding_box {
            // ロードした地図がある場合、そのバウンディングボックスを表示範囲とする
            let new_bounds = bounds.expand(bounds.width() * 0.1); // 先に少しマージンを追加

            let screen_aspect = rect.width() / rect.height();
            if !screen_aspect.is_nan() {
                let bounds_aspect = new_bounds.width() / new_bounds.height();
                let center = new_bounds.center();
                let (width, height) = if bounds_aspect > screen_aspect {
                    // バウンディングボックスの方が横長 -> 高さを増やす
                    (new_bounds.width(), new_bounds.width() / screen_aspect)
                } else {
                    // バウンディングボックスの方が縦長 -> 幅を増やす
                    (new_bounds.height() * screen_aspect, new_bounds.height())
                };
                egui::Rect::from_center_size(center, egui::vec2(width, height))
            } else {
                new_bounds
            }
        } else {
            // オンラインSLAM中の場合、これまで通りロボットを中心に表示
            let robot_center_world = egui::pos2(robot_pose.translation.x, robot_pose.translation.y);
            let map_view_size = config.slam.online_slam_view_size; // 追従時の表示サイズ
            egui::Rect::from_center_size(
                robot_center_world,
                egui::vec2(map_view_size, map_view_size),
            )
        };

        // Y軸を反転させるため、map_view_rectのYのmin/maxを入れ替える
        let mut inverted_map_view_rect = map_view_rect;
        inverted_map_view_rect.min.y = map_view_rect.max.y;
        inverted_map_view_rect.max.y = map_view_rect.min.y;
        let to_screen = egui::emath::RectTransform::from_to(
            inverted_map_view_rect,
            rect, // 実際の描画エリア
        );

        // 軌跡の線と向き（三角形）を描画
        if robot_trajectory.len() > 1 {
            // 軌跡の線
            let trajectory_line_points: Vec<egui::Pos2> = robot_trajectory
                .iter()
                .map(|(world_pos, _angle)| to_screen.transform_pos(*world_pos))
                .collect();
            let line_stroke = egui::Stroke::new(1.0, egui::Color32::DARK_GRAY);
            painter.add(egui::Shape::line(trajectory_line_points, line_stroke));

            // 向きを示す三角形
            let triangle_color = egui::Color32::GRAY;
            for (i, (world_pos, angle)) in robot_trajectory.iter().enumerate() {
                if i > 0 {
                    // 始点を除くすべての点で描画
                    let center_screen = to_screen.transform_pos(*world_pos);

                    // 三角形の頂点を定義
                    let triangle_size = 20.0; // 三角形の大きさ
                    let p1 = center_screen + egui::vec2(angle.cos(), -angle.sin()) * triangle_size; // 先端
                    let angle_left = *angle + (150.0f32).to_radians();
                    let p2 = center_screen
                        + egui::vec2(angle_left.cos(), -angle_left.sin()) * triangle_size * 0.7;
                    let angle_right = *angle - (150.0f32).to_radians();
                    let p3 = center_screen
                        + egui::vec2(angle_right.cos(), -angle_right.sin()) * triangle_size * 0.7;

                    painter.add(egui::Shape::convex_polygon(
                        vec![p1, p2, p3],
                        triangle_color,
                        egui::Stroke::NONE,
                    ));
                }
            }
        }

        // Draw the map points
        for (point, probability) in current_map_points {
            // Right = +X, Up = +Y
            let screen_pos = to_screen.transform_pos(egui::pos2(point.x, point.y));
            if rect.contains(screen_pos) {
                // 占有確率に基づいて色を調整
                let intensity = (*probability as f32 * 255.0).min(255.0).max(0.0);
                let color = egui::Color32::from_rgb(
                    (intensity * 0.4).min(255.0) as u8, // 青みを強くするためR,Gを低めに
                    (intensity * 0.4).min(255.0) as u8,
                    intensity as u8,
                );
                painter.circle_filled(screen_pos, 2.0, color);
            }
        }

        // 最新のスキャンデータを別の色で描画
        for point in latest_scan_for_draw {
            let local_point = nalgebra::Point2::new(point.0, point.1);
            let world_point = current_robot_pose * local_point;
            let screen_pos = to_screen.transform_pos(egui::pos2(world_point.x, world_point.y));
            if rect.contains(screen_pos) {
                painter.circle_filled(
                    screen_pos,
                    2.5, // 少し大きくして目立たせる
                    egui::Color32::YELLOW,
                );
            }
        }

        // Draw robot pose (この部分は変更しない)
        let robot_pos_on_screen = to_screen.transform_pos(egui::pos2(
            robot_pose.translation.x,
            robot_pose.translation.y,
        ));
        let angle = robot_pose.rotation.angle();
        painter.circle_filled(robot_pos_on_screen, 5.0, egui::Color32::RED);
        painter.line_segment(
            [
                robot_pos_on_screen,
                robot_pos_on_screen + egui::vec2(angle.cos(), -angle.sin()) * 20.0,
            ],
            egui::Stroke::new(2.0, egui::Color32::RED),
        );
    }
}
