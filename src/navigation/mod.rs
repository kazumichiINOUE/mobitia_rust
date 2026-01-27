pub mod de_tiny;
pub mod dwa;
pub mod elastic_band;
pub mod localization;
pub mod persistence_grid;
pub mod pure_pursuit;
pub mod recovery;

use crate::config::{NavConfig, RobotConfig, SlamConfig};
use crate::lidar::features::compute_features;
use crate::navigation::de_tiny::{DeTinySolver, NavScanPoint};
use crate::slam::{world_to_map_coords, CellData, OccupancyGrid, ScanData, Submap};
use eframe::egui;
use image::imageops;
use nalgebra::{Isometry2, Point2, Rotation2, Translation2, Vector2};
use serde::Deserialize;
use std::fs;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use tracing::{info, instrument};

#[derive(Deserialize, Debug, Clone)]
pub struct MapInfo {
    pub image: String,
    pub resolution: f32,
    pub origin: [f32; 3],
    pub free_thresh: f64,
    pub occupied_thresh: f64,
    pub negate: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavState {
    Standby,
    AdjustingPose,
    Running,
}

pub struct NavigationManager {
    // 状態データ
    pub nav_state: NavState,
    pub pose_adjustment_start: Option<egui::Pos2>, // For UI interaction

    pub nav_map_texture: Option<egui::TextureHandle>,
    pub nav_map_bounds: Option<egui::Rect>,
    pub nav_trajectory_points: Vec<egui::Pos2>, // Global Path (Initial)
    pub local_path: Vec<egui::Pos2>,            // Local Path (Deformed by EB)
    pub current_nav_target: Option<egui::Pos2>,
    pub current_robot_pose: Isometry2<f32>,

    // 地図データとメタデータ
    pub occupancy_grid: Option<OccupancyGrid>,
    pub map_info: Option<MapInfo>,

    // オドメトリ履歴 (前回フレームの値: x, y, theta)
    last_odom: Option<(f32, f32, f32)>,

    // Localization (DE)
    pub de_solver: DeTinySolver,
    pub initial_scan: Option<Vec<NavScanPoint>>,
    pub is_localizing: bool,
    pub de_frame_counter: usize,

    pub viz_scan: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
    pub converged_message_timer: usize,
    pub navigation_finished_timer: usize,
    pub total_travel_distance: f32,

    // Autonomous Navigation
    pub is_autonomous: bool,
    pub autonomy_intent: bool, // Remember if this was started with 'nav start'
    pub predicted_footprint_pose: Option<Isometry2<f32>>,

    // Planners
    pub dwa_planner: dwa::DwaPlanner,
    pub elastic_band: elastic_band::ElasticBand,
    pub recovery_manager: recovery::RecoveryManager,
    pub persistence_grid: persistence_grid::LocalPersistenceGrid,

    pub config: NavConfig,
    pub robot_config: RobotConfig,
}

impl NavigationManager {
    pub fn new(config: NavConfig, slam_config: SlamConfig, robot_config: RobotConfig) -> Self {
        let pose_config = config.initial_pose;
        let initial_pose = Isometry2::new(
            nalgebra::Vector2::new(pose_config[0], pose_config[1]),
            pose_config[2].to_radians(),
        );

        let dwa_planner = dwa::DwaPlanner::new(
            config.dwa.clone(),
            robot_config.clone(),
            slam_config.clone(),
        );
        let elastic_band = elastic_band::ElasticBand::new(config.elastic_band.clone());
        let recovery_manager = recovery::RecoveryManager::new();
        let persistence_grid = persistence_grid::LocalPersistenceGrid::new(0.1); // 10cm grid

        Self {
            nav_state: NavState::Standby,
            pose_adjustment_start: None,
            nav_map_texture: None,
            nav_map_bounds: None,
            nav_trajectory_points: Vec::new(),
            local_path: Vec::new(),
            current_nav_target: None,
            current_robot_pose: initial_pose,
            occupancy_grid: None,
            map_info: None,
            last_odom: None,
            de_solver: DeTinySolver::new(slam_config),
            initial_scan: None,
            is_localizing: true,
            de_frame_counter: 0,
            viz_scan: Vec::new(),
            converged_message_timer: 0,
            navigation_finished_timer: 0,
            total_travel_distance: 0.0,
            is_autonomous: false,
            autonomy_intent: false,
            predicted_footprint_pose: None,
            dwa_planner,
            elastic_band,
            recovery_manager,
            persistence_grid,
            config,
            robot_config,
        }
    }

    pub fn load_data(
        &mut self,
        path: &PathBuf,
        ctx: &egui::Context,
        is_autonomous: bool,
    ) -> Result<String, String> {
        self.reset();
        self.is_autonomous = is_autonomous;
        self.autonomy_intent = is_autonomous; // Store the intent
        self.nav_state = NavState::Standby;

        // 初期姿勢のリセット
        let pose_config = self.config.initial_pose;
        self.current_robot_pose = Isometry2::new(
            nalgebra::Vector2::new(pose_config[0], pose_config[1]),
            pose_config[2].to_radians(),
        );

        // 1. Load map_info.toml
        let map_info_path = path.join("map_info.toml");
        let map_info: MapInfo = fs::read_to_string(&map_info_path)
            .map_err(|e| format!("ERROR: Failed to read '{}': {}", map_info_path.display(), e))
            .and_then(|content| {
                toml::from_str(&content).map_err(|e| {
                    format!(
                        "ERROR: Failed to parse '{}': {}",
                        map_info_path.display(),
                        e
                    )
                })
            })?;

        self.map_info = Some(map_info.clone());

        // 2. Load occMap.png to determine the required grid dimensions
        let map_image_path = path.join(&map_info.image);
        let img = image::open(&map_image_path).map_err(|e| {
            format!(
                "ERROR: Failed to load map image '{}': {}",
                map_image_path.display(),
                e
            )
        })?;

        let width = img.width() as usize;
        let height = img.height() as usize;
        let mut grid = OccupancyGrid::new(width, height);

        // Fill initial occupancy from image
        let luma_img = img.to_luma8();
        for (x, y, pixel) in luma_img.enumerate_pixels() {
            let value = pixel.0[0];
            let prob = if map_info.negate == 0 {
                (255.0 - value as f64) / 255.0
            } else {
                value as f64 / 255.0
            };

            let log_odds = if prob > map_info.occupied_thresh {
                3.5 // Occupied
            } else if prob < map_info.free_thresh {
                -3.5 // Free
            } else {
                0.0 // Unknown
            };

            let index = (y as usize) * width + (x as usize);
            grid.data[index].log_odds = log_odds;
        }

        // --- 3. Submap Loading and Grid Reconstruction (Feature Enrichment) ---
        let submaps_dir = path.join("submaps");
        if submaps_dir.exists() && submaps_dir.is_dir() {
            println!(
                "Loading submaps for feature enrichment from: {}",
                submaps_dir.display()
            );

            let mut submap_paths = Vec::new();
            if let Ok(entries) = std::fs::read_dir(&submaps_dir) {
                for entry in entries.flatten() {
                    if entry.path().is_dir() {
                        submap_paths.push(entry.path());
                    }
                }
            }
            submap_paths.sort();

            for submap_path in submap_paths {
                let info_path = submap_path.join("info.yaml");
                let submap_meta: Submap = match fs::read_to_string(&info_path) {
                    Ok(content) => match serde_yaml::from_str(&content) {
                        Ok(info) => info,
                        Err(_) => continue,
                    },
                    Err(_) => continue,
                };

                let scans_path = submap_path.join(&submap_meta.scans_file);
                let scans_data: Vec<ScanData> = match fs::read_to_string(&scans_path) {
                    Ok(content) => match serde_json::from_str(&content) {
                        Ok(data) => data,
                        Err(_) => continue,
                    },
                    Err(_) => continue,
                };

                let submap_rotation = Rotation2::new(submap_meta.pose_theta);
                let submap_translation = Translation2::new(submap_meta.pose_x, submap_meta.pose_y);
                let submap_global_pose =
                    Isometry2::from_parts(submap_translation, submap_rotation.into());

                for scan in scans_data {
                    let rel_rotation = Rotation2::new(scan.relative_pose.theta);
                    let rel_translation =
                        Translation2::new(scan.relative_pose.x, scan.relative_pose.y);
                    let relative_pose = Isometry2::from_parts(rel_translation, rel_rotation.into());

                    let robot_global_pose = submap_global_pose * relative_pose;

                    for p in scan.scan_points {
                        // Filter out points too close to the robot center (likely self-reflections)
                        if (p.x * p.x + p.y * p.y).sqrt() < self.robot_config.min_mapping_dist {
                            continue;
                        }
                        let p_local = Point2::new(p.x, p.y);
                        let p_global = robot_global_pose * p_local;

                        let gx = ((p_global.x - map_info.origin[0]) / map_info.resolution).floor()
                            as i32;
                        let gy = ((map_info.origin[1] - p_global.y) / map_info.resolution).floor()
                            as i32;

                        if gx >= 0 && gx < width as i32 && gy >= 0 && gy < height as i32 {
                            let idx = (gy as usize) * width + (gx as usize);
                            let cell = &mut grid.data[idx];

                            let normal_local = Vector2::new(p.nx, p.ny);
                            let normal_global = robot_global_pose.rotation * normal_local;

                            cell.edge_ness = (cell.edge_ness * 0.7) + (p.feature as f64 * 0.3);
                            cell.corner_ness = (cell.corner_ness * 0.7) + (p.corner as f64 * 0.3);

                            cell.normal_x = (cell.normal_x * 0.7) + (normal_global.x as f64 * 0.3);
                            cell.normal_y = (cell.normal_y * 0.7) + (normal_global.y as f64 * 0.3);

                            let len = (cell.normal_x.powi(2) + cell.normal_y.powi(2)).sqrt();
                            if len > 1e-9 {
                                cell.normal_x /= len;
                                cell.normal_y /= len;
                            }

                            if cell.log_odds < 2.0 {
                                cell.log_odds += 0.5;
                            }
                        }
                    }
                }
            }
        }

        self.occupancy_grid = Some(grid.clone());

        // 4. Texture creation
        let mut color_image = egui::ColorImage::new([width, height], egui::Color32::TRANSPARENT);
        for y in 0..height {
            for x in 0..width {
                let index = y * width + x;
                let cell = &grid.data[index];
                let prob = crate::slam::log_odds_to_probability(cell.log_odds);

                let color = if prob > map_info.occupied_thresh {
                    egui::Color32::BLACK
                } else if prob < map_info.free_thresh {
                    egui::Color32::WHITE
                } else {
                    egui::Color32::from_gray(128)
                };
                color_image[(x, y)] = color;
            }
        }

        self.nav_map_texture = Some(ctx.load_texture(
            "nav-map-texture",
            color_image,
            egui::TextureOptions::NEAREST,
        ));

        // 5. Calculate map bounds
        let resolution = map_info.resolution;
        let origin_x = map_info.origin[0];
        let origin_y = map_info.origin[1];
        let width_m = width as f32 * resolution;
        let height_m = height as f32 * resolution;

        self.nav_map_bounds = Some(egui::Rect::from_min_size(
            egui::pos2(origin_x, origin_y - height_m),
            egui::vec2(width_m, height_m),
        ));

        // 6. Load trajectory.txt
        let trajectory_path = path.join("trajectory.txt");
        if let Ok(file) = fs::File::open(&trajectory_path) {
            let reader = BufReader::new(file);
            let mut points = Vec::new();
            for line in reader.lines() {
                if let Ok(line_str) = line {
                    let parts: Vec<f32> = line_str
                        .split_whitespace()
                        .filter_map(|s| s.parse().ok())
                        .collect();
                    if parts.len() >= 2 {
                        points.push(egui::pos2(parts[0], parts[1]));
                    }
                }
            }
            self.nav_trajectory_points = points.clone();
            // Initialize local_path with the global path
            self.local_path = points;
        } else {
            return Err(format!(
                "ERROR: Failed to load trajectory file '{}'",
                trajectory_path.display()
            ));
        }

        Ok(format!(
            "Successfully loaded navigation data from '{}' (with high-precision reconstruction)",
            path.display()
        ))
    }

    pub fn reset(&mut self) {
        self.nav_state = NavState::Standby;
        self.pose_adjustment_start = None;
        self.nav_map_texture = None;
        self.nav_map_bounds = None;
        self.occupancy_grid = None;
        self.map_info = None;
        self.nav_trajectory_points.clear();
        self.local_path.clear();
        self.current_nav_target = None;
        self.last_odom = None;
        self.initial_scan = None;
        self.is_localizing = true;
        self.de_frame_counter = 0;
        self.viz_scan.clear();
        self.total_travel_distance = 0.0;
        self.is_autonomous = false;
        self.predicted_footprint_pose = None;
        self.recovery_manager.reset();
        self.persistence_grid.reset();
    }

    #[instrument(skip(self, latest_scan), fields(autonomous = self.is_autonomous, localizing = self.is_localizing))]
    pub fn update(
        &mut self,
        current_odom: (f32, f32, f32),
        latest_scan: &[(f32, f32, f32, f32, f32, f32, f32, f32)],
    ) -> Option<(f32, f32)> {
        // Standby or AdjustingPose means we are PAUSED.
        // We do not update localization or send motor commands.
        // We only update last_odom to prevent large jumps when we finally start.
        if self.nav_state != NavState::Running {
            self.last_odom = Some(current_odom);
            // Also update viz_scan for UI
            self.viz_scan = latest_scan.to_vec();
            return None;
        }

        if self.converged_message_timer > 0 {
            self.converged_message_timer -= 1;
        }
        if self.navigation_finished_timer > 0 {
            self.navigation_finished_timer -= 1;
        }

        // --- Temporary Dummy Scan Injection ---
        let mut dummy_scan_storage; // To keep the vec alive
        let effective_scan = if self.config.debug_use_dummy_scan && latest_scan.is_empty() {
            // Generate a U-shape (corridor) dummy scan
            let mut dummy = Vec::new();
            for i in 0..100 {
                let y = (i as f32 / 100.0) * 1.6 - 0.8;
                let x = 5.0;
                let r = (x * x + y * y).sqrt();
                let theta = y.atan2(x);
                dummy.push((x, y, r, theta, 0.0, 0.0, 0.0, 0.0));
            }
            for i in 0..100 {
                let x = (i as f32 / 100.0) * 5.0;
                let y = 0.8;
                let r = (x * x + y * y).sqrt();
                let theta = y.atan2(x);
                dummy.push((x, y, r, theta, 0.0, 0.0, 0.0, 0.0));
            }
            // Right wall at y = -0.8m
            for i in 0..100 {
                let x = (i as f32 / 100.0) * 5.0; // 0.0 to 5.0
                let y = -0.8;
                let r = (x * x + y * y).sqrt();
                let theta = y.atan2(x);
                dummy.push((x, y, r, theta, 0.0, 0.0, 0.0, 0.0));
            }

            // Sort by theta to simulate scan order for feature extraction
            dummy.sort_by(|a, b| a.3.partial_cmp(&b.3).unwrap_or(std::cmp::Ordering::Equal));

            dummy_scan_storage = compute_features(&dummy);

            // Force artificial features at corners for visualization check
            if !dummy_scan_storage.is_empty() {
                let len = dummy_scan_storage.len();
                // Indices around 100 and 200 are corners (since we pushed 100 points per wall)
                if len > 100 {
                    dummy_scan_storage[99].4 = 1.0;
                    dummy_scan_storage[100].4 = 1.0;
                }
                if len > 200 {
                    dummy_scan_storage[199].4 = 1.0;
                    dummy_scan_storage[200].4 = 1.0;
                }
            }

            &dummy_scan_storage[..]
        } else {
            latest_scan
        };
        // --------------------------------------

        // Update visualization scan
        self.viz_scan = effective_scan.to_vec();

        // If we are localizing and haven't captured the initial scan yet
        if self.is_localizing && self.initial_scan.is_none() && !effective_scan.is_empty() {
            let points: Vec<NavScanPoint> = effective_scan
                .iter()
                .map(|p| NavScanPoint {
                    pos: Point2::new(p.0, p.1),
                    normal: Vector2::new(p.5, p.6),
                    feature: p.4,
                })
                .collect();

            self.de_solver.init(self.current_robot_pose, None);
            self.initial_scan = Some(points);
        }

        if self.is_localizing {
            // --- Localization Mode ---
            if let (Some(scan), Some(grid), Some(info)) =
                (&self.initial_scan, &self.occupancy_grid, &self.map_info)
            {
                // Throttle DE updates to observe convergence
                if self.de_frame_counter % self.config.initial_localization_interval_frames == 0 {
                    self.de_solver
                        .step(grid, scan, info.resolution, info.origin);
                    //println!("DEBUG: Step executed. Gen: {}", self.de_solver.generation);
                    self.current_robot_pose = self.de_solver.get_best_pose();

                    if self.de_solver.is_converged {
                        info!("Localization Converged");
                        self.is_localizing = false;
                        // Restore autonomous intent (if any) so the robot starts moving immediately
                        self.is_autonomous = self.autonomy_intent;
                        self.de_frame_counter = 0;
                        self.converged_message_timer = 180;

                        // CRITICAL: Clear obstacles and recovery state one more time
                        // now that we have a stable pose. This prevents "phantom" obstacles
                        // created during the localization search from blocking the start.
                        self.persistence_grid.reset();
                        self.recovery_manager.reset();
                    }
                }
                self.de_frame_counter += 1;
            }
            self.last_odom = Some(current_odom);
            None
        } else {
            // --- Tracking Mode ---
            if let Some((last_x, last_y, last_theta)) = self.last_odom {
                let (curr_x, curr_y, curr_theta) = current_odom;
                let delta_x_global = curr_x - last_x;
                let delta_y_global = curr_y - last_y;
                let delta_theta = curr_theta - last_theta;
                let cos_theta = last_theta.cos();
                let sin_theta = last_theta.sin();
                let delta_x_local = delta_x_global * cos_theta + delta_y_global * sin_theta;
                let delta_y_local = -delta_x_global * sin_theta + delta_y_global * cos_theta;

                // Add to total travel distance
                let step_dist = (delta_x_local.powi(2) + delta_y_local.powi(2)).sqrt();
                self.total_travel_distance += step_dist;

                let movement = Isometry2::new(
                    nalgebra::Vector2::new(delta_x_local, delta_y_local),
                    delta_theta,
                );
                self.current_robot_pose *= movement;
            }
            self.last_odom = Some(current_odom);

            if self.de_frame_counter % self.config.tracking_update_interval_frames == 0
                && !effective_scan.is_empty()
            {
                if let (Some(grid), Some(info)) = (&self.occupancy_grid, &self.map_info) {
                    let points: Vec<NavScanPoint> = effective_scan
                        .iter()
                        .map(|p| NavScanPoint {
                            pos: Point2::new(p.0, p.1),
                            normal: Vector2::new(p.5, p.6),
                            feature: p.4,
                        })
                        .collect();

                    self.de_solver.init(
                        self.current_robot_pose,
                        Some((
                            self.config.tracking_wxy,
                            self.config.tracking_wa_degrees,
                            self.config.tracking_population_size,
                            self.config.tracking_generations,
                        )),
                    );

                    while !self.de_solver.is_converged {
                        self.de_solver
                            .step(grid, &points, info.resolution, info.origin);
                    }
                    self.current_robot_pose = self.de_solver.get_best_pose();
                }
            }
            self.de_frame_counter += 1;

            // --- Path Planning & Navigation (Runs when localized) ---
            if !self.is_localizing {
                use crate::navigation::pure_pursuit;

                // --- 1. Filter Self-Reflection & Update Persistence Grid ---
                // We do this first so ALL subsystems use clean, time-filtered data.
                let mut filtered_raw_scan = Vec::new(); // For Persistence Grid input
                let df = self.robot_config.dimension_front;
                let dr = self.robot_config.dimension_rear;
                let half_w = self.robot_config.width / 2.0;

                for point in effective_scan {
                    // Filter out points that are inside the robot's footprint (self-reflection)
                    if point.0 > -dr - 0.05 && point.0 < df + 0.05 && point.1.abs() < half_w + 0.05
                    {
                        continue;
                    }
                    if point.2 < self.robot_config.min_mapping_dist {
                        continue;
                    }
                    filtered_raw_scan.push(*point);
                }

                // Update Persistence Grid with filtered raw points
                let stable_local_obstacles = self
                    .persistence_grid
                    .update(&filtered_raw_scan, &self.current_robot_pose);

                // Prepare obstacles for Elastic Band & other checks (convert to Vec<(f32,f32,f32,f32)>)
                // stable_local_obstacles is already Vec<(f32, f32, f32, f32)> (x, y, nx, ny)

                // --- 2. Recovery Manager Update (Reverse Path Tracking) ---
                if self.is_autonomous {
                    if let Some(recovery_cmd) = self.recovery_manager.update(
                        &self.current_robot_pose,
                        &stable_local_obstacles,
                        &self.nav_trajectory_points,
                        &self.config,
                        self.robot_config.width,
                    ) {
                        // Recovery active: Override normal navigation
                        return Some(recovery_cmd);
                    }
                }

                // --- 3. Update Elastic Band (Local Path) ---
                let robot_pos_egui = egui::pos2(
                    self.current_robot_pose.translation.x,
                    self.current_robot_pose.translation.y,
                );

                let robot_tf = self.current_robot_pose;
                // Convert obstacles to global frame for Elastic Band optimization
                let global_obstacles: Vec<(f32, f32, f32, f32)> = stable_local_obstacles
                    .iter()
                    .map(|(lx, ly, nx, ny)| {
                        let p_local = Point2::new(*lx, *ly);
                        let n_local = Vector2::new(*nx, *ny);

                        let p_global = robot_tf * p_local;
                        let n_global = robot_tf.rotation * n_local;

                        (p_global.x, p_global.y, n_global.x, n_global.y)
                    })
                    .collect();

                // Prune passed points from local_path
                if !self.local_path.is_empty() {
                    let mut closest_idx = 0;
                    let mut min_dist_sq = f32::MAX;

                    let robot_yaw = self.current_robot_pose.rotation.angle();
                    let forward_x = robot_yaw.cos();
                    let forward_y = robot_yaw.sin();

                    for (i, p) in self.local_path.iter().enumerate() {
                        let dx = p.x - robot_pos_egui.x;
                        let dy = p.y - robot_pos_egui.y;
                        let d2 = dx * dx + dy * dy;
                        let d = d2.sqrt();

                        if d < 1e-6 {
                            continue;
                        }

                        // Normalized dot product to check if the point is within 45 degrees in front
                        // cos(45 deg) approx 0.707
                        let dot_norm = (dx * forward_x + dy * forward_y) / d;

                        if dot_norm > 0.707 {
                            // Point is within 45 deg cone in front.
                            if d2 < min_dist_sq {
                                min_dist_sq = d2;
                                closest_idx = i;
                            }
                        }
                    }

                    // If no points are in front, we fallback to the absolute nearest point
                    // (to prevent sticking if the robot is slightly off the path)
                    if min_dist_sq == f32::MAX {
                        for (i, p) in self.local_path.iter().enumerate() {
                            let dx = p.x - robot_pos_egui.x;
                            let dy = p.y - robot_pos_egui.y;
                            let d2 = dx * dx + dy * dy;
                            if d2 < min_dist_sq {
                                min_dist_sq = d2;
                                closest_idx = i;
                            }
                        }
                    }

                    // Keep a margin of points behind the robot (e.g., 5 points)
                    let margin = 5;
                    if closest_idx > margin {
                        let remove_count = closest_idx - margin;
                        self.local_path.drain(0..remove_count);
                    }
                }

                // Run EB optimization
                self.elastic_band.optimize(
                    &mut self.local_path,
                    robot_pos_egui,
                    &global_obstacles,
                    &self.nav_trajectory_points,
                );

                // --- 4. Generate Command (Only if Autonomous) ---
                if self.is_autonomous {
                    // Compute command using the deformed local_path
                    let command = pure_pursuit::compute_command(
                        &self.current_robot_pose,
                        &self.local_path,
                        &mut self.current_nav_target,
                        self.config.lookahead_distance,
                        self.config.target_velocity,
                        self.config.goal_tolerance,
                    );

                    // Check for goal reach explicitly to trigger message
                    if let Some(last_p) = self.local_path.last() {
                        let robot_pos = self.current_robot_pose.translation.vector;
                        let goal_pos = nalgebra::Vector2::new(last_p.x, last_p.y);
                        let dist_to_goal = (robot_pos - goal_pos).norm();

                        if dist_to_goal < self.config.goal_tolerance {
                            info!(
                                "GOAL REACHED! Total Travel Distance: {:.2}m, Accuracy: {:.3}m",
                                self.total_travel_distance, dist_to_goal
                            );
                            self.is_autonomous = false;
                            self.navigation_finished_timer = 180; // Show message for ~3 seconds
                            return Some((0.0, 0.0));
                        }
                    }

                    // --- 5. Collision Check (Safety Stop) ---
                    let mut final_command = command;

                    let margin = self.config.lidar_avoid_dist;
                    let df = self.robot_config.dimension_front;
                    let dr = self.robot_config.dimension_rear;
                    let half_w = self.robot_config.width / 2.0;

                    let mut detected = false;

                    // Use stable_local_obstacles (Time-filtered)
                    for (x, y, _nx, _ny) in &stable_local_obstacles {
                        // Rectangular check: Robot Footprint + Margin
                        if *x >= (-dr - margin)
                            && *x <= (df + margin)
                            && y.abs() <= (half_w + margin)
                        {
                            detected = true;
                            info!("COLLISION AVOIDANCE: Obstacle at ({:.2}, {:.2}) local", x, y);
                            break;
                        }
                    }

                    if detected {
                        final_command = Some((0.0, 0.0));
                    }

                    if let Some((v, w)) = final_command {
                        if v.abs() > 0.001 || w.abs() > 0.001 {
                            // Log movement commands occasionally or if needed
                            // info!("Nav Command: v={:.2}, w={:.2}", v, w);
                        }

                        // Update predicted pose for visualization
                        let dt = 1.0;
                        let current_yaw = self.current_robot_pose.rotation.angle();
                        let pred_yaw = current_yaw + w * dt;
                        let avg_yaw = (current_yaw + pred_yaw) / 2.0;
                        let pred_x = self.current_robot_pose.translation.x + v * dt * avg_yaw.cos();
                        let pred_y = self.current_robot_pose.translation.y + v * dt * avg_yaw.sin();
                        self.predicted_footprint_pose = Some(Isometry2::new(
                            nalgebra::Vector2::new(pred_x, pred_y),
                            pred_yaw,
                        ));
                    } else {
                        self.predicted_footprint_pose = None;
                    }

                    final_command
                } else {
                    None
                }
            } else {
                None
            }
        }
    }

    fn is_point_inside_current_footprint(&self, p: &Point2<f32>) -> bool {
        let local_p = self.current_robot_pose.inverse() * p;
        let half_w = self.robot_config.width / 2.0;
        let df = self.robot_config.dimension_front;
        let dr = self.robot_config.dimension_rear;
        // Use a small margin to handle resolution artifacts and blobs
        local_p.x >= (-dr - 0.05) && local_p.x <= (df + 0.05) && local_p.y.abs() <= (half_w + 0.05)
    }

    fn check_footprint_collision(&self, pose: &Isometry2<f32>) -> bool {
        if let (Some(grid), Some(map_info)) = (&self.occupancy_grid, &self.map_info) {
            let w = self.robot_config.width;
            let df = self.robot_config.dimension_front;
            let dr = self.robot_config.dimension_rear;
            let half_w = w / 2.0;

            // Robot local corners: FL, FR, BR, BL
            let corners_local = [
                Point2::new(df, half_w),
                Point2::new(df, -half_w),
                Point2::new(-dr, -half_w),
                Point2::new(-dr, half_w),
            ];

            // Transform to world and check edges
            let corners_world: Vec<Point2<f32>> = corners_local.iter().map(|p| pose * p).collect();

            // Check edges by sampling
            let num_corners = corners_world.len();
            for i in 0..num_corners {
                let p1 = corners_world[i];
                let p2 = corners_world[(i + 1) % num_corners];

                let dist = (p1 - p2).norm();
                let steps = (dist / map_info.resolution).ceil() as usize;

                for s in 0..=steps {
                    let t = s as f32 / steps as f32;
                    let p = p1 + (p2 - p1) * t;

                    let (gx, gy) = world_to_map_coords(
                        p.x,
                        p.y,
                        &crate::config::SlamConfig {
                            csize: map_info.resolution,
                            map_width: grid.width,
                            map_height: grid.height,
                            ..Default::default()
                        },
                    );

                    if gx >= 0 && gx < grid.width as isize && gy >= 0 && gy < grid.height as isize {
                        let idx = (gy as usize) * grid.width + (gx as usize);
                        let log_odds = grid.data[idx].log_odds;
                        if log_odds > 2.0 {
                            // SELF-FILTER: Ignore points that are already inside our current footprint.
                            // This prevents colliding with "phantom" points created by the robot itself.
                            if self.is_point_inside_current_footprint(&p) {
                                continue;
                            }

                            println!(
                                "COLLISION DETECTED at Grid({}, {}), World({:.3}, {:.3}), LogOdds: {:.2}",
                                gx, gy, p.x, p.y, log_odds
                            );
                            return true; // Collision detected
                        }
                    }
                }
            }
        }
        false
    }
}
