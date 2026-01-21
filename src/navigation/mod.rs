pub mod de_tiny;
pub mod localization;
pub mod pure_pursuit;

use crate::config::{NavConfig, SlamConfig};
use crate::lidar::features::compute_features;
use crate::navigation::de_tiny::{DeTinySolver, NavScanPoint};
use crate::slam::{CellData, OccupancyGrid, ScanData, Submap};
use eframe::egui;
use image::imageops;
use nalgebra::{Isometry2, Point2, Rotation2, Translation2, Vector2};
use serde::Deserialize;
use std::fs;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;

#[derive(Deserialize, Debug, Clone)]
pub struct MapInfo {
    pub image: String,
    pub resolution: f32,
    pub origin: [f32; 3],
    pub free_thresh: f64,
    pub occupied_thresh: f64,
    pub negate: i32,
}

pub struct NavigationManager {
    // 状態データ
    pub nav_map_texture: Option<egui::TextureHandle>,
    pub nav_map_bounds: Option<egui::Rect>,
    pub nav_trajectory_points: Vec<egui::Pos2>,
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

    // Visualization
    pub viz_scan: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
    pub converged_message_timer: usize,

    // Autonomous Navigation
    pub is_autonomous: bool,

    pub config: NavConfig,
}

impl NavigationManager {
    pub fn new(config: NavConfig, slam_config: SlamConfig) -> Self {
        let pose_config = config.initial_pose;
        let initial_pose = Isometry2::new(
            nalgebra::Vector2::new(pose_config[0], pose_config[1]),
            pose_config[2].to_radians(),
        );

        Self {
            nav_map_texture: None,
            nav_map_bounds: None,
            nav_trajectory_points: Vec::new(),
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
            is_autonomous: false,
            config,
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
            println!("Loading submaps for feature enrichment from: {}", submaps_dir.display());
            
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
                let submap_global_pose = Isometry2::from_parts(submap_translation, submap_rotation.into());

                for scan in scans_data {
                    let rel_rotation = Rotation2::new(scan.relative_pose.theta);
                    let rel_translation = Translation2::new(scan.relative_pose.x, scan.relative_pose.y);
                    let relative_pose = Isometry2::from_parts(rel_translation, rel_rotation.into());
                    
                    let robot_global_pose = submap_global_pose * relative_pose;

                    for p in scan.scan_points {
                        let p_local = Point2::new(p.x, p.y);
                        let p_global = robot_global_pose * p_local;
                        
                        let gx = ((p_global.x - map_info.origin[0]) / map_info.resolution).floor() as i32;
                        let gy = ((map_info.origin[1] - p_global.y) / map_info.resolution).floor() as i32;

                        if gx >= 0 && gx < width as i32 && gy >= 0 && gy < height as i32 {
                            let idx = (gy as usize) * width + (gx as usize);
                            let cell = &mut grid.data[idx];

                            let normal_local = Vector2::new(p.nx, p.ny); 
                            let normal_global = robot_global_pose.rotation * normal_local;
                            
                            cell.edge_ness = (cell.edge_ness * 0.7) + (p.feature as f64 * 0.3);
                            cell.corner_ness = (cell.corner_ness * 0.7) + (p.corner as f64 * 0.3);
                            
                            // Debug print
                            if p.feature > 0.5 {
                                println!("DEBUG: Loaded high edge feature ({:.3}) at Grid({}, {}). Current cell edge_ness: {:.3}", p.feature, gx, gy, cell.edge_ness);
                            }

                            cell.normal_x = (cell.normal_x * 0.7) + (normal_global.x as f64 * 0.3);
                            cell.normal_y = (cell.normal_y * 0.7) + (normal_global.y as f64 * 0.3);
                            
                            let len = (cell.normal_x.powi(2) + cell.normal_y.powi(2)).sqrt();
                            if len > 1e-9 {
                                cell.normal_x /= len;
                                cell.normal_y /= len;
                            }
                            
                            if cell.log_odds < 2.0 { cell.log_odds += 0.5; }
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
            self.nav_trajectory_points = points;
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
        self.nav_map_texture = None;
        self.nav_map_bounds = None;
        self.occupancy_grid = None;
        self.map_info = None;
        self.nav_trajectory_points.clear();
        self.current_nav_target = None;
        self.last_odom = None;
        self.initial_scan = None;
        self.is_localizing = true;
        self.de_frame_counter = 0;
        self.viz_scan.clear();
        self.is_autonomous = false;
    }

    pub fn update(
        &mut self,
        current_odom: (f32, f32, f32),
        latest_scan: &[(f32, f32, f32, f32, f32, f32, f32, f32)],
    ) -> Option<(f32, f32)> {
        if self.converged_message_timer > 0 {
            self.converged_message_timer -= 1;
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
                if len > 100 { dummy_scan_storage[99].4 = 1.0; dummy_scan_storage[100].4 = 1.0; }
                if len > 200 { dummy_scan_storage[199].4 = 1.0; dummy_scan_storage[200].4 = 1.0; }
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
                                        self.is_localizing = false;
                        self.de_frame_counter = 0; 
                        self.converged_message_timer = 180;
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
                let movement = Isometry2::new(nalgebra::Vector2::new(delta_x_local, delta_y_local), delta_theta);
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
            
            if self.is_autonomous && !self.is_localizing {
                use crate::navigation::pure_pursuit;
                pure_pursuit::compute_command(
                    &self.current_robot_pose,
                    &self.nav_trajectory_points,
                    &mut self.current_nav_target,
                    self.config.lookahead_distance,
                    self.config.target_velocity,
                    self.config.goal_tolerance,
                )
            } else {
                None
            }
        }
    }
}
