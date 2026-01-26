use crate::navigation::NavigationManager;
use eframe::egui;

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
        ui_config: &crate::config::UiConfig,
        navigation_manager: &NavigationManager,
        latest_scan_for_draw: &[(f32, f32, f32, f32, f32, f32, f32, f32)],
        motor_odometry: &(f32, f32, f32),
        robot_trajectory: &[(egui::Pos2, f32)],
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
                    egui::RichText::new(format!("Travel Dist: {:>8.2} m", navigation_manager.total_travel_distance))
                        .monospace()
                        .color(egui::Color32::GOLD)
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
        
        // Fill background
        painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(20, 20, 20));

        // --- 1. Main View Setup ---
        let nav_map_bounds = &navigation_manager.nav_map_bounds;
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
        let to_screen_main = egui::emath::RectTransform::from_to(inverted_map_view_rect, rect);

        // --- 2. Draw Main View ---
        Self::draw_nav_content(
            &painter,
            rect,
            &to_screen_main,
            navigation_manager,
            latest_scan_for_draw,
            robot_trajectory,
            true, // Draw background map
        );

        // --- 3. Sub View (PIP) Setup ---
        // Fixed size 300x300 at bottom right
        let sub_size = egui::vec2(300.0, 300.0);
        let sub_rect = egui::Rect::from_min_size(
            rect.max - sub_size - egui::vec2(20.0, 20.0),
            sub_size
        );

        // Only draw PIP if there is space
        if rect.contains(sub_rect.min) {
            let sub_painter = painter.with_clip_rect(sub_rect);
            
            // Background and Border
            sub_painter.rect_filled(sub_rect, 0.0, egui::Color32::from_rgb(10, 10, 10));
            sub_painter.rect_stroke(sub_rect, 0.0, egui::Stroke::new(2.0, egui::Color32::WHITE));

            // Viewport: Robot center +/- zoom_view_size/2
            let zoom_size = ui_config.nav_zoom_view_size;
            let robot_pos = egui::pos2(current_robot_pose.translation.x, current_robot_pose.translation.y);
            let sub_view_rect = egui::Rect::from_center_size(robot_pos, egui::vec2(zoom_size, zoom_size));
            
            let mut inverted_sub_view_rect = sub_view_rect;
            inverted_sub_view_rect.min.y = sub_view_rect.max.y;
            inverted_sub_view_rect.max.y = sub_view_rect.min.y;
            let to_screen_sub = egui::emath::RectTransform::from_to(inverted_sub_view_rect, sub_rect);

            // --- 4. Draw Sub View ---
            Self::draw_nav_content(
                &sub_painter,
                sub_rect,
                &to_screen_sub,
                navigation_manager,
                latest_scan_for_draw,
                robot_trajectory,
                true,
            );
            
            // Label for PIP
            sub_painter.text(
                sub_rect.min + egui::vec2(5.0, 5.0),
                egui::Align2::LEFT_TOP,
                format!("Zoom ({:.1}m x {:.1}m)", zoom_size, zoom_size),
                egui::FontId::monospace(14.0),
                egui::Color32::WHITE,
            );
        }

        // --- Converged & Finished Messages (Overlay on whole screen) ---
        if navigation_manager.converged_message_timer > 0 {
            let center = rect.center();
            let text = "Localization Converged";
            let font_id = egui::FontId::proportional(30.0);
            painter.text(center + egui::vec2(2.0, 2.0), egui::Align2::CENTER_CENTER, text, font_id.clone(), egui::Color32::BLACK);
            painter.text(center, egui::Align2::CENTER_CENTER, text, font_id, egui::Color32::GREEN);
        }
        if navigation_manager.navigation_finished_timer > 0 {
            let center = rect.center();
            let text = "Goal Reached / Navigation Finished";
            let font_id = egui::FontId::proportional(30.0);
            painter.text(center + egui::vec2(2.0, 2.0), egui::Align2::CENTER_CENTER, text, font_id.clone(), egui::Color32::BLACK);
            painter.text(center, egui::Align2::CENTER_CENTER, text, font_id, egui::Color32::GOLD);
        }

        // --- Debug Info on Hover (Main View Only) ---
        if let Some(mouse_pos) = ui.input(|i| i.pointer.hover_pos()) {
            if rect.contains(mouse_pos) {
                // If hovering over PIP, use sub transform?
                // For simplicity, let's just support tooltips on the main map for now, 
                // or check if mouse is inside sub_rect.
                let transform = if rect.contains(sub_rect.min) && sub_rect.contains(mouse_pos) {
                    // Hovering PIP
                    let robot_pos = egui::pos2(current_robot_pose.translation.x, current_robot_pose.translation.y);
                    let sub_view_rect = egui::Rect::from_center_size(robot_pos, egui::vec2(4.0, 4.0));
                    let mut inverted_sub_view_rect = sub_view_rect;
                    inverted_sub_view_rect.min.y = sub_view_rect.max.y;
                    inverted_sub_view_rect.max.y = sub_view_rect.min.y;
                    egui::emath::RectTransform::from_to(inverted_sub_view_rect, sub_rect)
                } else {
                    // Hovering Main
                    to_screen_main
                };

                let world_pos = transform.inverse().transform_pos(mouse_pos);
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

                    let gx = ((world_pos.x - origin_x) / resolution).floor() as i32;
                    let gy = ((origin_y - world_pos.y) / resolution).floor() as i32;

                    debug_text += &format!("\nGrid(TL): ({}, {})", gx, gy);

                    if gx >= 0 && gx < width as i32 && gy >= 0 && gy < height as i32 {
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

    #[allow(clippy::too_many_arguments)]
    fn draw_nav_content(
        painter: &egui::Painter,
        rect: egui::Rect,
        to_screen: &egui::emath::RectTransform,
        navigation_manager: &NavigationManager,
        latest_scan_for_draw: &[(f32, f32, f32, f32, f32, f32, f32, f32)],
        robot_trajectory: &[(egui::Pos2, f32)],
        draw_background: bool,
    ) {
        let current_robot_pose = &navigation_manager.current_robot_pose;
        let nav_map_bounds = &navigation_manager.nav_map_bounds;
        let nav_map_texture = &navigation_manager.nav_map_texture;
        let nav_trajectory_points = &navigation_manager.nav_trajectory_points;
        let local_path = &navigation_manager.local_path;
        let current_nav_target = &navigation_manager.current_nav_target;

        // --- Static Drawing (Map Image) ---
        if draw_background {
            if let (Some(texture), Some(bounds)) = (nav_map_texture, nav_map_bounds) {
                // Optimize: Only draw if bounds intersect view
                let screen_rect = to_screen.transform_rect(*bounds);
                if rect.intersects(screen_rect) {
                    painter.image(
                        texture.id(),
                        screen_rect,
                        egui::Rect::from_min_max(egui::pos2(0.0, 1.0), egui::pos2(1.0, 0.0)),
                        egui::Color32::WHITE,
                    );
                }
            }
        }

        // --- Visualization of Config Parameters (Safety Zones) ---
        {
            let robot_width = navigation_manager.robot_config.width;
            let nav_config = &navigation_manager.config;
            let robot_screen_pos = to_screen.transform_pos(egui::pos2(
                current_robot_pose.translation.x,
                current_robot_pose.translation.y,
            ));

            // 1. Elastic Band Safety Distance (Cyan, weak stroke)
            // This is where the path starts to bend away from obstacles.
            {
                let radius = nav_config.elastic_band.obstacle_safety_dist;
                let p1 = to_screen.transform_pos(egui::pos2(0.0, 0.0));
                let p2 = to_screen.transform_pos(egui::pos2(radius, 0.0));
                let radius_screen = p1.distance(p2);

                if rect.contains(robot_screen_pos) {
                    painter.circle_stroke(
                        robot_screen_pos,
                        radius_screen,
                        egui::Stroke::new(
                            1.0,
                            egui::Color32::from_rgba_unmultiplied(0, 255, 255, 60),
                        ), // Transparent Cyan
                    );
                }
            }

            // 2. Recovery Trigger Corridor (Purple, filled rect)
            // Rect: x=[0, trigger_dist], y=[-width/2, +width/2] (Local Frame)
            {
                let trigger_dist = nav_config.recovery_trigger_dist;
                let safety_half_width = robot_width / 2.0 + 0.01;

                // Define 4 corners in Local Frame
                let corners_local = [
                    nalgebra::Point2::new(0.0, safety_half_width),
                    nalgebra::Point2::new(trigger_dist, safety_half_width),
                    nalgebra::Point2::new(trigger_dist, -safety_half_width),
                    nalgebra::Point2::new(0.0, -safety_half_width),
                ];

                let corners_screen: Vec<egui::Pos2> = corners_local
                    .iter()
                    .map(|p| {
                        let world_p = current_robot_pose * p;
                        to_screen.transform_pos(egui::pos2(world_p.x, world_p.y))
                    })
                    .collect();

                painter.add(egui::Shape::convex_polygon(
                    corners_screen,
                    egui::Color32::from_rgba_unmultiplied(150, 0, 255, 40), // Transparent Purple fill
                    egui::Stroke::new(
                        1.0,
                        egui::Color32::from_rgba_unmultiplied(150, 0, 255, 100),
                    ), // Purple outline
                ));
            }

            // 3. Lidar Avoidance/Stop Boundary (Red, rectangular stroke)
            // Rectangle = Footprint + lidar_avoid_dist margin
            {
                let margin = nav_config.lidar_avoid_dist;
                let df = navigation_manager.robot_config.dimension_front;
                let dr = navigation_manager.robot_config.dimension_rear;
                let half_w = navigation_manager.robot_config.width / 2.0;

                // Define 4 corners in Local Frame expanded by margin
                let corners_local = [
                    nalgebra::Point2::new(df + margin, half_w + margin),
                    nalgebra::Point2::new(df + margin, -half_w - margin),
                    nalgebra::Point2::new(-dr - margin, -half_w - margin),
                    nalgebra::Point2::new(-dr - margin, half_w + margin),
                ];

                let corners_screen: Vec<egui::Pos2> = corners_local
                    .iter()
                    .map(|p| {
                        let world_p = current_robot_pose * p;
                        to_screen.transform_pos(egui::pos2(world_p.x, world_p.y))
                    })
                    .collect();

                painter.add(egui::Shape::convex_polygon(
                    corners_screen,
                    egui::Color32::TRANSPARENT, 
                    egui::Stroke::new(
                        1.5,
                        egui::Color32::from_rgba_unmultiplied(255, 50, 50, 150),
                    ), // Red stroke
                ));
            }
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

                // Optimization: Iterate only visible cells?
                // For now, simple iteration but check rect containment early
                for y in (0..height).step_by(5) { // Skip some for perf
                    for x in (0..width).step_by(5) {
                        let idx = y * width + x;
                        if grid.data[idx].edge_ness > 0.5 {
                            let wx = origin_x + (x as f32 + 0.5) * resolution;
                            let wy = origin_y - (y as f32 + 0.5) * resolution;
                            let screen_pos = to_screen.transform_pos(egui::pos2(wx, wy));
                            
                            if rect.contains(screen_pos) {
                                painter.rect_filled(
                                    egui::Rect::from_center_size(screen_pos, egui::vec2(3.0, 3.0)),
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
            [origin_screen, to_screen.transform_pos(egui::pos2(axis_length_world, 0.0))],
            egui::Stroke::new(1.0, egui::Color32::from_rgb(100, 50, 50)),
        );
        painter.line_segment(
            [origin_screen, to_screen.transform_pos(egui::pos2(0.0, axis_length_world))],
            egui::Stroke::new(1.0, egui::Color32::from_rgb(50, 100, 50)),
        );

        // Global Trajectory (Faded Blue)
        if nav_trajectory_points.len() > 1 {
            let path_points: Vec<egui::Pos2> = nav_trajectory_points
                .iter()
                .map(|p| to_screen.transform_pos(*p))
                .collect();
            // Basic optimization: don't draw lines far outside
            painter.add(egui::Shape::line(
                path_points,
                egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(100, 100, 255, 60)),
            ));
        }

        // Robot History Trajectory (Gray)
        if robot_trajectory.len() > 1 {
            let path_points: Vec<egui::Pos2> = robot_trajectory
                .iter()
                .map(|(pos, _)| to_screen.transform_pos(*pos))
                .collect();
            painter.add(egui::Shape::line(
                path_points,
                egui::Stroke::new(1.0, egui::Color32::GRAY),
            ));
        }

        // Local Path / Elastic Band (Cyan/Green)
        if local_path.len() > 1 {
            let path_points: Vec<egui::Pos2> = local_path
                .iter()
                .map(|p| to_screen.transform_pos(*p))
                .collect();
            painter.add(egui::Shape::line(
                path_points,
                egui::Stroke::new(2.0, egui::Color32::from_rgb(0, 255, 128)),
            ));
        }

        // --- Draw DE Particles ---
        if navigation_manager.is_localizing {
            for particle in &navigation_manager.de_solver.population {
                let screen_pos = to_screen.transform_pos(egui::pos2(particle.x, particle.y));
                if rect.contains(screen_pos) {
                    painter.circle_filled(
                        screen_pos,
                        2.0,
                        egui::Color32::from_rgba_unmultiplied(255, 0, 0, 150),
                    );
                }
            }
        }

        // --- Real-time Drawing (Scan) ---
        let scan_to_draw = if !navigation_manager.viz_scan.is_empty() {
            &navigation_manager.viz_scan
        } else {
            latest_scan_for_draw
        };

        for point in scan_to_draw {
            let local_point = nalgebra::Point2::new(point.0, point.1);
            let world_point = current_robot_pose * local_point;
            let screen_pos = to_screen.transform_pos(egui::pos2(world_point.x, world_point.y));
            if rect.contains(screen_pos) {
                let color = if point.4 > 0.5 {
                    egui::Color32::from_rgb(255, 0, 255)
                } else {
                    egui::Color32::YELLOW
                };
                let radius = if point.4 > 0.5 { 2.0 } else { 1.5 };
                painter.circle_filled(screen_pos, radius, color);
            }
        }

        if let Some(target) = current_nav_target {
            let screen_pos = to_screen.transform_pos(*target);
            if rect.contains(screen_pos) {
                let stroke = egui::Stroke::new(2.0, egui::Color32::from_rgb(0, 255, 255));
                painter.circle_stroke(screen_pos, 10.0, stroke);
            }
        }

        // Footprint
        let w = navigation_manager.robot_config.width;
        let df = navigation_manager.robot_config.dimension_front;
        let dr = navigation_manager.robot_config.dimension_rear;
        let half_w = w / 2.0;
        let corners_local = [
            nalgebra::Point2::new(df, half_w),
            nalgebra::Point2::new(df, -half_w),
            nalgebra::Point2::new(-dr, -half_w),
            nalgebra::Point2::new(-dr, half_w),
        ];
        let corners_screen: Vec<egui::Pos2> = corners_local
            .iter()
            .map(|p| {
                let world_p = current_robot_pose * p;
                to_screen.transform_pos(egui::pos2(world_p.x, world_p.y))
            })
            .collect();

        painter.add(egui::Shape::convex_polygon(
            corners_screen,
            egui::Color32::from_rgba_unmultiplied(0, 255, 0, 30),
            egui::Stroke::new(1.0, egui::Color32::GREEN),
        ));

        // Predicted Footprint
        if let Some(pred_pose) = navigation_manager.predicted_footprint_pose {
            let corners_screen_pred: Vec<egui::Pos2> = corners_local
                .iter()
                .map(|p| {
                    let world_p = pred_pose * p;
                    to_screen.transform_pos(egui::pos2(world_p.x, world_p.y))
                })
                .collect();

            painter.add(egui::Shape::convex_polygon(
                corners_screen_pred,
                egui::Color32::from_rgba_unmultiplied(0, 255, 255, 30), // Cyan, transparent
                egui::Stroke::new(1.0, egui::Color32::from_rgb(0, 255, 255)), // Cyan outline
            ));
        }

        // Robot Center
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