pub mod localization;
pub mod pure_pursuit;

use crate::config::NavConfig;
use crate::slam::{CellData, OccupancyGrid};
use eframe::egui;
use image::imageops;
use nalgebra::{Isometry2, Point2};
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
    
    config: NavConfig,
}

impl NavigationManager {
    pub fn new(config: NavConfig) -> Self {
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
            config,
        }
    }

    pub fn load_data(
        &mut self,
        path: &PathBuf,
        ctx: &egui::Context,
    ) -> Result<String, String> {
        self.reset();

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
                    format!("ERROR: Failed to parse '{}': {}", map_info_path.display(), e)
                })
            })?;
        
        self.map_info = Some(map_info.clone());

        // 2. Load occMap.png and create OccupancyGrid
        let map_image_path = path.join(&map_info.image);
        let img = image::open(&map_image_path)
            .map_err(|e| format!("ERROR: Failed to load map image '{}': {}", map_image_path.display(), e))?;

        let width = img.width() as usize;
        let height = img.height() as usize;
        let mut grid = OccupancyGrid::new(width, height);

        // log_odds calculation (simplified from app.rs logic)
        let _free_log_odds = (1.0 - map_info.occupied_thresh).ln() - (map_info.occupied_thresh).ln();
        let _occupied_log_odds = (map_info.free_thresh).ln() - (1.0 - map_info.free_thresh).ln();
        
        let rgba_image = img.to_rgba8();
        let pixels = rgba_image.as_flat_samples();
        let color_image = egui::ColorImage::from_rgba_unmultiplied([width, height], pixels.as_slice());

        // Assuming image is grayscale, 0=black=occupied, 255=white=free
        let luma_img = img.to_luma8();
        for (x, y, pixel) in luma_img.enumerate_pixels() {
            let value = pixel.0[0];
            let prob = if map_info.negate == 0 {
                (255.0 - value as f64) / 255.0 // 0(black) -> 1.0(occupied), 255(white) -> 0.0(free)
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
            grid.data[index] = CellData {
                log_odds,
                ..Default::default()
            };
        }
        
        self.occupancy_grid = Some(grid);

        // 3. Texture creation
        self.nav_map_texture = Some(ctx.load_texture(
            "nav-map-texture",
            color_image,
            egui::TextureOptions::NEAREST,
        ));

        // 4. Calculate map bounds
        let resolution = map_info.resolution;
        let origin_x = map_info.origin[0];
        let origin_y = map_info.origin[1];
        let width_m = width as f32 * resolution;
        let height_m = height as f32 * resolution;

        self.nav_map_bounds = Some(egui::Rect::from_min_size(
            egui::pos2(origin_x, origin_y - height_m), // egui's y is down, map's y is up
            egui::vec2(width_m, height_m),
        ));

        // 5. Load trajectory.txt
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
            return Err(format!("ERROR: Failed to load trajectory file '{}'", trajectory_path.display()));
        }

        Ok(format!("Successfully loaded navigation data from '{}'", path.display()))
    }

    pub fn reset(&mut self) {
        self.nav_map_texture = None;
        self.nav_map_bounds = None;
        self.occupancy_grid = None;
        self.map_info = None;
        self.nav_trajectory_points.clear();
        self.current_nav_target = None;
        self.last_odom = None;
    }

    pub fn update(
        &mut self,
        current_odom: (f32, f32, f32),
        // latest_scan: &[(f32, f32, f32, f32, f32, f32, f32, f32)], // Unused for now
    ) {
        // 1. Odometry Update
        if let Some((last_x, last_y, last_theta)) = self.last_odom {
            let (curr_x, curr_y, curr_theta) = current_odom;

            let delta_x_global = curr_x - last_x;
            let delta_y_global = curr_y - last_y;
            let delta_theta = curr_theta - last_theta;

            let cos_theta = last_theta.cos();
            let sin_theta = last_theta.sin();
            let delta_x_local = delta_x_global * cos_theta + delta_y_global * sin_theta;
            let delta_y_local = -delta_x_global * sin_theta + delta_y_global * cos_theta;

            let movement = Isometry2::new(
                nalgebra::Vector2::new(delta_x_local, delta_y_local),
                delta_theta,
            );
            self.current_robot_pose *= movement;
        }
        self.last_odom = Some(current_odom);
    }
}
