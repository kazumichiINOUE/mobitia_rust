use eframe::egui;
use nalgebra::Isometry2;

#[derive(Default)]
pub struct NavScreen {}

impl NavScreen {
    pub fn new() -> Self {
        Self {}
    }

    #[allow(clippy::too_many_arguments)]
    pub fn draw(
        &mut self,
        ui: &mut egui::Ui,
        lidar_draw_rect: &mut Option<egui::Rect>,
        // Static data
        nav_map_texture: &Option<egui::TextureHandle>,
        nav_map_bounds: &Option<egui::Rect>,
        nav_trajectory_points: &[egui::Pos2],
        // Real-time data
        current_robot_pose: &Isometry2<f32>,
        latest_scan_for_draw: &[(f32, f32, f32, f32, f32, f32, f32, f32)],
        current_nav_target: &Option<egui::Pos2>,
    ) {
        ui.heading("Navigation Mode");
        let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
        let rect = response.rect;
        *lidar_draw_rect = Some(rect);
        painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(20, 20, 20));

        // --- Coordinate System Setup ---
        // Scale the view to fit the entire map, similar to map_screen.rs
        let map_view_rect = if let Some(bounds) = nav_map_bounds {
            let screen_aspect = rect.width() / rect.height();
            let bounds_aspect = bounds.width() / bounds.height();
            let center = bounds.center();
            let (width, height) = if bounds_aspect > screen_aspect {
                (bounds.width(), bounds.width() / screen_aspect)
            } else {
                (bounds.height() * screen_aspect, bounds.height())
            };
            egui::Rect::from_center_size(center, egui::vec2(width, height))
        } else {
            // Default view if no map is loaded, centered on the robot
             let robot_center_world = egui::pos2(current_robot_pose.translation.x, current_robot_pose.translation.y);
            egui::Rect::from_center_size(robot_center_world, egui::vec2(20.0, 20.0))
        };

        let mut inverted_map_view_rect = map_view_rect;
        inverted_map_view_rect.min.y = map_view_rect.max.y;
        inverted_map_view_rect.max.y = map_view_rect.min.y;
        let to_screen = egui::emath::RectTransform::from_to(inverted_map_view_rect, rect);

        // --- Static Drawing ---

        // 1. Draw the map texture
        if let (Some(texture), Some(bounds)) = (nav_map_texture, nav_map_bounds) {
            let screen_rect = to_screen.transform_rect(*bounds);
            painter.image(
                texture.id(),
                screen_rect,
                egui::Rect::from_min_max(egui::pos2(0.0, 1.0), egui::pos2(1.0, 0.0)), // Flipped UV Y-axis
                egui::Color32::WHITE,
            );
        }

        // 2. Draw Origin and Axes
        let origin_world = egui::Pos2::ZERO;
        let origin_screen = to_screen.transform_pos(origin_world);
        let axis_length_world = 1.0; // 1 meter
        painter.line_segment(
            [origin_screen, to_screen.transform_pos(egui::pos2(axis_length_world, 0.0))],
            egui::Stroke::new(1.0, egui::Color32::from_rgb(100, 50, 50)), // Dim Red
        );
        painter.line_segment(
            [origin_screen, to_screen.transform_pos(egui::pos2(0.0, axis_length_world))],
            egui::Stroke::new(1.0, egui::Color32::from_rgb(50, 100, 50)), // Dim Green
        );

        // 3. Draw the full trajectory path
        if nav_trajectory_points.len() > 1 {
             let path_points: Vec<egui::Pos2> = nav_trajectory_points
                .iter()
                .map(|p| to_screen.transform_pos(*p))
                .collect();
            painter.add(egui::Shape::line(path_points, egui::Stroke::new(1.0, egui::Color32::from_rgb(100, 100, 200))));
        } else {
             for point in nav_trajectory_points {
                let screen_pos = to_screen.transform_pos(*point);
                if rect.contains(screen_pos) {
                    painter.circle_filled(screen_pos, 3.0, egui::Color32::from_rgb(100, 100, 200));
                }
            }
        }
        
        // --- Real-time Drawing ---

        // 4. Draw the latest LiDAR scan
        for point in latest_scan_for_draw {
            let local_point = nalgebra::Point2::new(point.0, point.1);
            let world_point = current_robot_pose * local_point;
            let screen_pos = to_screen.transform_pos(egui::pos2(world_point.x, world_point.y));
            if rect.contains(screen_pos) {
                painter.circle_filled(screen_pos, 1.5, egui::Color32::YELLOW);
            }
        }

        // 5. Draw the current navigation target
        if let Some(target) = current_nav_target {
            let screen_pos = to_screen.transform_pos(*target);
            if rect.contains(screen_pos) {
                let stroke = egui::Stroke::new(2.0, egui::Color32::from_rgb(0, 255, 255));
                painter.circle_stroke(screen_pos, 10.0, stroke);
                painter.circle_filled(screen_pos, 8.0, egui::Color32::from_rgba_unmultiplied(0, 255, 255, 30));
            }
        }

        // 6. Draw the robot's current pose
        let robot_pos_on_screen = to_screen.transform_pos(egui::pos2(
            current_robot_pose.translation.x,
            current_robot_pose.translation.y,
        ));
        let angle = current_robot_pose.rotation.angle();
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
