use crate::navigation::NavigationManager;
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
        navigation_manager: &NavigationManager,
        latest_scan_for_draw: &[(f32, f32, f32, f32, f32, f32, f32, f32)],
        motor_odometry: &(f32, f32, f32),
    ) {
        ui.heading("Navigation Mode");

        // --- Info Overlay ---
        let current_robot_pose = &navigation_manager.current_robot_pose;
        egui::Area::new("nav_info_overlay")
            .fixed_pos(ui.min_rect().min)
            .show(ui.ctx(), |ui| {
                let background_color = egui::Color32::from_rgba_unmultiplied(0, 0, 0, 128);
                let pose_x = current_robot_pose.translation.x;
                let pose_y = current_robot_pose.translation.y;
                let pose_angle_deg = current_robot_pose.rotation.angle().to_degrees();
                ui.label(
                    egui::RichText::new(format!(
                        "Est Pose: x: {:>8.3}, y: {:>8.3}, a: {:>6.1}°",
                        pose_x, pose_y, pose_angle_deg
                    ))
                    .monospace()
                    .background_color(background_color),
                );
                let (odom_x, odom_y, odom_angle) = motor_odometry;
                ui.label(
                    egui::RichText::new(format!(
                        "Motor Odom: x: {:>8.3}, y: {:>8.3}, a: {:>6.1}°",
                        odom_x,
                        odom_y,
                        odom_angle.to_degrees()
                    ))
                    .monospace()
                    .background_color(background_color),
                );
                ui.label(
                    egui::RichText::new(format!("Lidar Points: {}", latest_scan_for_draw.len()))
                        .monospace()
                        .background_color(background_color),
                );
            });

        let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
        let rect = response.rect;
        *lidar_draw_rect = Some(rect);
        painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(20, 20, 20));

        let nav_map_bounds = &navigation_manager.nav_map_bounds;
        let nav_map_texture = &navigation_manager.nav_map_texture;
        let nav_trajectory_points = &navigation_manager.nav_trajectory_points;
        let current_nav_target = &navigation_manager.current_nav_target;

        // --- Coordinate System Setup ---
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
            let robot_center_world = egui::pos2(
                current_robot_pose.translation.x,
                current_robot_pose.translation.y,
            );
            egui::Rect::from_center_size(robot_center_world, egui::vec2(20.0, 20.0))
        };

        let mut inverted_map_view_rect = map_view_rect;
        inverted_map_view_rect.min.y = map_view_rect.max.y;
        inverted_map_view_rect.max.y = map_view_rect.min.y;
        let to_screen = egui::emath::RectTransform::from_to(inverted_map_view_rect, rect);

        // --- Static Drawing ---
        if let (Some(texture), Some(bounds)) = (nav_map_texture, nav_map_bounds) {
            let screen_rect = to_screen.transform_rect(*bounds);
            painter.image(
                texture.id(),
                screen_rect,
                egui::Rect::from_min_max(egui::pos2(0.0, 1.0), egui::pos2(1.0, 0.0)),
                egui::Color32::WHITE,
            );
        }

        // --- Debug: Show Corner Cells ---
        if navigation_manager.config.debug_show_corner_cells {
            if let (Some(grid), Some(info)) = (
                &navigation_manager.occupancy_grid,
                &navigation_manager.map_info,
            ) {
                let resolution = info.resolution;
                let origin_x = info.origin[0];
                let origin_y = info.origin[1];
                let width = grid.width;
                let height = grid.height;

                for y in 0..height {
                    for x in 0..width {
                        let idx = y * width + x;
                        if grid.data[idx].edge_ness > 0.5 {
                            // Grid to World (Top-Left Origin)
                            let wx = origin_x + (x as f32 + 0.5) * resolution;
                            let wy = origin_y - (y as f32 + 0.5) * resolution;

                            let screen_pos = to_screen.transform_pos(egui::pos2(wx, wy));
                            if rect.contains(screen_pos) {
                                painter.rect_filled(
                                    egui::Rect::from_center_size(screen_pos, egui::vec2(5.0, 5.0)),
                                    0.0,
                                    egui::Color32::BLUE,
                                );
                            }
                        }
                    }
                }
            }
        }

        // Axes
        let origin_world = egui::Pos2::ZERO;
        let origin_screen = to_screen.transform_pos(origin_world);
        let axis_length_world = 1.0;
        painter.line_segment(
            [
                origin_screen,
                to_screen.transform_pos(egui::pos2(axis_length_world, 0.0)),
            ],
            egui::Stroke::new(1.0, egui::Color32::from_rgb(100, 50, 50)),
        );
        painter.line_segment(
            [
                origin_screen,
                to_screen.transform_pos(egui::pos2(0.0, axis_length_world)),
            ],
            egui::Stroke::new(1.0, egui::Color32::from_rgb(50, 100, 50)),
        );

        // Trajectory
        if nav_trajectory_points.len() > 1 {
            let path_points: Vec<egui::Pos2> = nav_trajectory_points
                .iter()
                .map(|p| to_screen.transform_pos(*p))
                .collect();
            painter.add(egui::Shape::line(
                path_points,
                egui::Stroke::new(1.0, egui::Color32::from_rgb(100, 100, 200)),
            ));
        } else {
            for point in nav_trajectory_points {
                let screen_pos = to_screen.transform_pos(*point);
                if rect.contains(screen_pos) {
                    painter.circle_filled(screen_pos, 3.0, egui::Color32::from_rgb(100, 100, 200));
                }
            }
        }

        // --- Draw DE Particles ---
        if navigation_manager.is_localizing {
            let particles = &navigation_manager.de_solver.population;
            for particle in particles {
                // particle is Vector3 (x, y, theta)
                // World coordinates
                let screen_pos = to_screen.transform_pos(egui::pos2(particle.x, particle.y));
                if rect.contains(screen_pos) {
                    painter.circle_filled(
                        screen_pos,
                        2.0,
                        egui::Color32::from_rgba_unmultiplied(255, 0, 0, 150),
                    );

                    // Optional: Draw orientation
                    let arrow_len = 5.0;
                    let end_pos =
                        screen_pos + egui::vec2(particle.z.cos(), -particle.z.sin()) * arrow_len;
                    painter.line_segment(
                        [screen_pos, end_pos],
                        egui::Stroke::new(
                            1.0,
                            egui::Color32::from_rgba_unmultiplied(255, 0, 0, 100),
                        ),
                    );
                }
            }
        }

        // --- Real-time Drawing ---
        let scan_to_draw = if !navigation_manager.viz_scan.is_empty() {
            &navigation_manager.viz_scan
        } else {
            latest_scan_for_draw
        };

        // Pass 1: Draw non-feature points in YELLOW
        for point in scan_to_draw {
            if point.4 <= 0.5 {
                let local_point = nalgebra::Point2::new(point.0, point.1);
                let world_point = current_robot_pose * local_point;
                let screen_pos = to_screen.transform_pos(egui::pos2(world_point.x, world_point.y));
                if rect.contains(screen_pos) {
                    painter.circle_filled(screen_pos, 1.5, egui::Color32::YELLOW);
                }
            }
        }

        // Pass 2: Draw feature points in MAGENTA on top
        for point in scan_to_draw {
            if point.4 > 0.5 {
                let local_point = nalgebra::Point2::new(point.0, point.1);
                let world_point = current_robot_pose * local_point;
                let screen_pos = to_screen.transform_pos(egui::pos2(world_point.x, world_point.y));
                if rect.contains(screen_pos) {
                    painter.circle_filled(screen_pos, 2.0, egui::Color32::from_rgb(255, 0, 255));
                }
            }
        }

        if let Some(target) = current_nav_target {
            let screen_pos = to_screen.transform_pos(*target);
            if rect.contains(screen_pos) {
                let stroke = egui::Stroke::new(2.0, egui::Color32::from_rgb(0, 255, 255));
                painter.circle_stroke(screen_pos, 10.0, stroke);
                painter.circle_filled(
                    screen_pos,
                    8.0,
                    egui::Color32::from_rgba_unmultiplied(0, 255, 255, 30),
                );
            }
        }

        // Robot
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

        // --- Converged Message ---
        if navigation_manager.converged_message_timer > 0 {
            let center = rect.center();
            let text = "Localization Converged";
            let font_id = egui::FontId::proportional(30.0);

            // Draw text with a simple shadow/outline for visibility
            painter.text(
                center + egui::vec2(2.0, 2.0),
                egui::Align2::CENTER_CENTER,
                text,
                font_id.clone(),
                egui::Color32::BLACK,
            );
            painter.text(
                center,
                egui::Align2::CENTER_CENTER,
                text,
                font_id,
                egui::Color32::GREEN,
            );
        }

        // --- Debug Info on Hover ---
        if let Some(mouse_pos) = ui.input(|i| i.pointer.hover_pos()) {
            if rect.contains(mouse_pos) {
                let world_pos = to_screen.inverse().transform_pos(mouse_pos);
                let mut debug_text = format!("World: ({:.3}, {:.3})", world_pos.x, world_pos.y);

                if let (Some(grid), Some(info)) = (
                    &navigation_manager.occupancy_grid,
                    &navigation_manager.map_info,
                ) {
                    let resolution = info.resolution;
                    let origin_x = info.origin[0];
                    let origin_y = info.origin[1];
                    let width = grid.width;
                    let height = grid.height;

                    // Assuming Origin = Top-Left (confirmed by user)
                    let gx = ((world_pos.x - origin_x) / resolution).floor() as i32;
                    let gy = ((origin_y - world_pos.y) / resolution).floor() as i32;

                    debug_text += &format!("\nGrid(TL): ({}, {})", gx, gy);

                    if gx >= 0 && gx < width as i32 && gy >= 0 && gy < height as i32 {
                        // For display, gy is the index from top (0) to bottom (height-1)
                        let idx = (gy as usize) * width + (gx as usize);
                        if let Some(cell) = grid.data.get(idx) {
                            debug_text += &format!("\nLogOdds: {:.2}", cell.log_odds);
                            if cell.log_odds > 0.0 {
                                debug_text += "\n[OCCUPIED]";
                            } else if cell.log_odds < 0.0 {
                                debug_text += "\n[FREE]";
                            } else {
                                debug_text += "\n[UNKNOWN]";
                            }
                        }
                    } else {
                        debug_text += "\n[OUT OF BOUNDS]";
                    }
                }

                egui::show_tooltip(ui.ctx(), egui::Id::new("nav_debug_tooltip"), |ui| {
                    ui.label(debug_text);
                });
            }
        }
    }
}
