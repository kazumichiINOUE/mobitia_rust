use crate::app::MyApp;
use crate::slam::{log_odds_to_probability, world_to_map_coords, ScanData, Submap};
use anyhow::Result;
use bresenham::Bresenham;
use eframe::egui;
use nalgebra::{Isometry2, Point2};
use std::fs;

// Struct to hold the state of the current submap loading process
pub struct SubmapLoadProgress {
    pub submap_id: usize,
    pub submap_global_pose: Isometry2<f32>,
    pub scans: Vec<ScanData>,
    pub next_scan_index: usize,
    pub log_odds_occ: f64,
    pub log_odds_free: f64,
}

impl MyApp {
    /// Reads submap files and prepares a new session for scan-by-scan processing.
    pub fn start_submap_loading_session(&mut self, submap_path_str: &str) -> Result<()> {
        let path = std::path::PathBuf::from(submap_path_str);
        let info_path = path.join("info.yaml");
        let scans_path = path.join("scans.json");

        if !info_path.exists() || !scans_path.exists() {
            return Err(anyhow::anyhow!(
                "Submap info.yaml or scans.json not found in: {}",
                path.display()
            ));
        }

        if self.offline_map.is_none() {
            return Err(anyhow::anyhow!(
                "Offline map grid was not initialized before starting submap session."
            ));
        }

        let info_file = fs::File::open(info_path)?;
        let submap_info: Submap = serde_yaml::from_reader(info_file)?;

        let scans_file_content = fs::read_to_string(scans_path)?;
        let scans_data: Vec<ScanData> = serde_json::from_str(&scans_file_content)?;

        let submap_global_pose = Isometry2::new(
            nalgebra::Vector2::new(submap_info.pose_x, submap_info.pose_y),
            submap_info.pose_theta,
        );

        let log_odds_occ =
            (self.config.slam.prob_occupied / (1.0 - self.config.slam.prob_occupied)).ln();
        let log_odds_free = (self.config.slam.prob_free / (1.0 - self.config.slam.prob_free)).ln();

        self.current_submap_load_progress = Some(SubmapLoadProgress {
            submap_id: submap_info.id,
            submap_global_pose,
            scans: scans_data,
            next_scan_index: 0,
            log_odds_occ,
            log_odds_free,
        });

        self.submaps.insert(submap_info.id, submap_info.clone());

        Ok(())
    }

    /// Processes the next scan from the currently active submap loading session.
    pub fn process_next_scan_in_submap(&mut self, _ctx: &egui::Context) -> Result<bool> {
        let progress = match &mut self.current_submap_load_progress {
            Some(p) => p,
            None => return Ok(false), // No submap is being processed
        };

        if progress.next_scan_index >= progress.scans.len() {
            self.current_submap_load_progress = None;
            return Ok(false); // Finished with this submap
        }

        let grid = self.offline_map.as_mut().ok_or_else(|| {
            anyhow::anyhow!("Offline map grid was not initialized before processing scan.")
        })?;

        let scan_data = &progress.scans[progress.next_scan_index];

        let scan_pose_relative = Isometry2::new(
            nalgebra::Vector2::new(scan_data.relative_pose.x, scan_data.relative_pose.y),
            scan_data.relative_pose.theta,
        );
        let scan_pose_world = progress.submap_global_pose * scan_pose_relative;

        self.robot_trajectory.push((
            egui::pos2(scan_pose_world.translation.x, scan_pose_world.translation.y),
            scan_pose_world.rotation.angle(),
        ));
        self.current_robot_pose = scan_pose_world;

        let robot_pos_map = world_to_map_coords(
            scan_pose_world.translation.x,
            scan_pose_world.translation.y,
            &self.config.slam,
        );

        // Force free space around the robot within min_mapping_dist
        let min_dist = self.config.robot.min_mapping_dist;
        if min_dist > 0.0 {
            let radius_cells = (min_dist / self.config.slam.csize).ceil() as isize;
            let min_dist_cells_sq = (min_dist / self.config.slam.csize).powi(2);

            for dy in -radius_cells..=radius_cells {
                for dx in -radius_cells..=radius_cells {
                    // Check if cell is within the circle
                    if (dx as f32).powi(2) + (dy as f32).powi(2) <= min_dist_cells_sq {
                        let mx = robot_pos_map.0 as isize + dx;
                        let my = robot_pos_map.1 as isize + dy;

                        if mx >= 0
                            && mx < grid.width as isize
                            && my >= 0
                            && my < grid.height as isize
                        {
                            let idx = (my as usize) * grid.width + (mx as usize);
                            grid.data[idx].log_odds =
                                (grid.data[idx].log_odds + progress.log_odds_free).clamp(
                                    self.config.slam.log_odds_clamp_min,
                                    self.config.slam.log_odds_clamp_max,
                                );
                        }
                    }
                }
            }
        }

        for scan_point in &scan_data.scan_points {
            let endpoint_local = Point2::new(scan_point.x, scan_point.y);
            let endpoint_world = scan_pose_world * endpoint_local;
            let endpoint_map =
                world_to_map_coords(endpoint_world.x, endpoint_world.y, &self.config.slam);

            for (px, py) in Bresenham::new(robot_pos_map, endpoint_map) {
                if px < 0 || (px as usize) >= grid.width || py < 0 || (py as usize) >= grid.height {
                    continue;
                }
                let manhattan_dist = (px - endpoint_map.0).abs() + (py - endpoint_map.1).abs();
                if manhattan_dist <= 1 {
                    break;
                }
                let index = py as usize * grid.width + px as usize;
                grid.data[index].log_odds = (grid.data[index].log_odds + progress.log_odds_free)
                    .clamp(
                        self.config.slam.log_odds_clamp_min,
                        self.config.slam.log_odds_clamp_max,
                    );
            }

            if endpoint_map.0 >= 0
                && (endpoint_map.0 as usize) < grid.width
                && endpoint_map.1 >= 0
                && (endpoint_map.1 as usize) < grid.height
            {
                let index = endpoint_map.1 as usize * grid.width + endpoint_map.0 as usize;
                grid.data[index].log_odds = (grid.data[index].log_odds + progress.log_odds_occ)
                    .clamp(
                        self.config.slam.log_odds_clamp_min,
                        self.config.slam.log_odds_clamp_max,
                    );
            }
        }

        progress.next_scan_index += 1;
        Ok(true) // A scan was processed
    }

    /// Converts the current offline grid to points for visualization.
    pub fn update_map_points_from_grid(&mut self) {
        if let Some(grid) = &self.offline_map {
            self.current_map_points.clear();
            for y in 0..grid.height {
                for x in 0..grid.width {
                    let index = y * grid.width + x;
                    let cell_log_odds = grid.data[index].log_odds;

                    if (cell_log_odds - 0.0).abs() > 1e-6 {
                        let world_x = ((x as isize - (self.config.slam.map_width / 2) as isize)
                            as f32)
                            * self.config.slam.csize;
                        let world_y = (-(y as isize - (self.config.slam.map_height / 2) as isize)
                            as f32)
                            * self.config.slam.csize;

                        let probability = log_odds_to_probability(cell_log_odds);
                        self.current_map_points
                            .push((Point2::new(world_x, world_y), probability));
                    }
                }
            }
        }
    }

    /// Updates the bounding box for the loaded map.
    pub fn update_slam_map_bounding_box(&mut self) {
        if !self.current_map_points.is_empty() {
            let egui_points: Vec<egui::Pos2> = self
                .current_map_points
                .iter()
                .map(|(p, _prob)| egui::pos2(p.x, p.y))
                .collect();
            self.slam_map_bounding_box = Some(egui::Rect::from_points(&egui_points));
        }
    }
}
