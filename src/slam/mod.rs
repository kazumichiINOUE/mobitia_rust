pub mod differential_evolution;

use lazy_static::lazy_static;
use nalgebra::{Isometry2, Point2};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::io::Write;
use bresenham::Bresenham;

// --- Map Configuration ---
pub const CSIZE: f32 = 0.025;
pub const MAP_WIDTH: usize = 3200;
pub const MAP_HEIGHT: usize = 3200;
pub const MAP_ORIGIN_X: usize = MAP_WIDTH / 2;
pub const MAP_ORIGIN_Y: usize = MAP_HEIGHT / 2;
const DECAY_RATE: f64 = 0.999; // For map decay

// --- Map Update Method Selection ---
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MapUpdateMethod {
    Binary,
    Probabilistic,
}

// --- Constants for Probabilistic Update ---
const PROB_OCCUPIED: f64 = 0.6; // Probability of a cell being occupied upon a "hit"
const PROB_FREE: f64 = 0.4;     // Probability of a cell being free upon a "miss"

lazy_static! {
    static ref LOG_ODDS_OCC: f64 = (PROB_OCCUPIED / (1.0 - PROB_OCCUPIED)).ln();
    static ref LOG_ODDS_FREE: f64 = (PROB_FREE / (1.0 - PROB_FREE)).ln();
}

// Clamp limits for log-odds to prevent infinite values and allow for unlearning
const LOG_ODDS_CLAMP_MAX: f64 = 5.0;
const LOG_ODDS_CLAMP_MIN: f64 = -5.0;



/// Represents a 2D occupancy grid map where each cell holds a log-odds value.
pub struct OccupancyGrid {
    pub width: usize,
    pub height: usize,
    pub data: Vec<f64>,
}

impl OccupancyGrid {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            data: vec![0.0; width * height], // Initialize with 0.0 log-odds (0.5 probability)
        }
    }
}

/// サブマップのメタデータ構造体
#[derive(Debug, Serialize, Deserialize)]
pub struct Submap {
    pub id: usize,
    #[serde(skip)]
    pub global_pose: Isometry2<f32>,
    pub pose_x: f32,
    pub pose_y: f32,
    pub pose_theta: f32,
    pub timestamp_ms: u128,
    pub points_file: String,
    pub info_file: String,
}

/// The main SLAM state manager.
pub struct SlamManager {
    map_update_method: MapUpdateMethod,
    is_initial_scan: bool,
    map_gmap: OccupancyGrid,
    robot_pose: Isometry2<f32>,
    de_solver: differential_evolution::DifferentialEvolutionSolver,

    // サブマップ関連のフィールド
    submap_counter: usize,
    num_scans_per_submap: usize,
    current_submap_scan_buffer: Vec<Vec<Point2<f32>>>,
    current_submap_robot_poses: Vec<Isometry2<f32>>,
    current_submap_timestamps_buffer: Vec<u128>,
    submaps: HashMap<usize, Submap>,
    output_base_dir: std::path::PathBuf,
    
    // キャッシュされた点群
    cached_map_points: Vec<Point2<f32>>,
    is_map_dirty: bool, // 地図が更新されたかを示すフラグ
}

impl SlamManager {
    pub fn new(output_base_dir: std::path::PathBuf, map_update_method: MapUpdateMethod) -> Self {
        Self {
            map_update_method,
            is_initial_scan: true,
            map_gmap: OccupancyGrid::new(MAP_WIDTH, MAP_HEIGHT),
            robot_pose: Isometry2::identity(),
            de_solver: differential_evolution::DifferentialEvolutionSolver::new(),
            submap_counter: 0,
            num_scans_per_submap: 20,
            current_submap_scan_buffer: Vec::new(),
            current_submap_robot_poses: Vec::new(),
            current_submap_timestamps_buffer: Vec::new(),
            submaps: HashMap::new(),
            output_base_dir,
            cached_map_points: Vec::new(),
            is_map_dirty: true,
        }
    }

    /// Processes a new LiDAR scan and updates the map and pose.
    pub fn update(&mut self, lidar_points: &[(f32, f32)], timestamp: u128) {
        // --- Map Decay ---
        // Decay the entire map slightly to forget old, unobserved areas.
        // This helps prevent matching against stale parts of the map.
        if self.map_update_method == MapUpdateMethod::Probabilistic {
            for cell in self.map_gmap.data.iter_mut() {
                *cell *= DECAY_RATE;
            }
        }

        let current_scan: Vec<Point2<f32>> =
            lidar_points.iter().map(|p| Point2::new(p.0, p.1)).collect();

        if self.is_initial_scan {
            self.robot_pose = Isometry2::identity();
            self.is_initial_scan = false;
        } else {
            // NOTE: DE評価関数の修正が次のステップで必要
            let (best_pose, _score) =
                self.de_solver
                    .optimize_de(&self.map_gmap, &current_scan, self.robot_pose, self.map_update_method);
            self.robot_pose = best_pose;
        }

        let pose = self.robot_pose; // Copy pose to avoid borrow checker issues

        // Update the occupancy grid based on the selected method
        match self.map_update_method {
            MapUpdateMethod::Probabilistic => {
                self.update_grid_probabilistic(&current_scan, &pose);
            }
            MapUpdateMethod::Binary => {
                self.update_grid_binary(&current_scan, &pose);
            }
        }
        self.is_map_dirty = true;

        // --- Submap generation logic (unchanged for now) ---
        // TODO: This part might need refactoring as it relies on storing scans, 
        // which is not ideal for long-term probabilistic mapping.
        self.current_submap_scan_buffer.push(current_scan.clone());
        self.current_submap_robot_poses.push(self.robot_pose);
        self.current_submap_timestamps_buffer.push(timestamp);

        if self.current_submap_scan_buffer.len() >= self.num_scans_per_submap {
            self.generate_and_save_submap();
        }
    }

    /// Updates the grid using a probabilistic (log-odds) model.
    fn update_grid_probabilistic(&mut self, scan: &[Point2<f32>], pose: &Isometry2<f32>) {
        // --- Free Space Update ---
        // This part applies the inverse sensor model to update free space probabilities.
        // The logic is encapsulated here and can be replaced with other sensor models in the future.
        self.update_free_space(scan, pose);

        // --- Occupied Space Update ---
        // This part updates the cells where the laser beams are considered to have hit an obstacle.
        self.update_occupied_space(scan, pose);
    }
    
    /// Updates the cells that are considered free space based on the laser scan.
    fn update_free_space(&mut self, scan: &[Point2<f32>], pose: &Isometry2<f32>) {
        let robot_pos_map = world_to_map_coords(pose.translation.x, pose.translation.y);

        for endpoint_local in scan.iter() {
            let endpoint_world = pose * endpoint_local;
            let endpoint_map = world_to_map_coords(endpoint_world.x, endpoint_world.y);

            // Use Bresenham's algorithm to trace the laser beam
            for (px, py) in Bresenham::new(robot_pos_map, endpoint_map) {
                if px < 0 || px as usize >= MAP_WIDTH || py < 0 || py as usize >= MAP_HEIGHT { continue; }
                let index = py as usize * MAP_WIDTH + px as usize;
                
                // Don't update the endpoint itself as free space
                if (px, py) == endpoint_map { break; }

                // Update cell as free
                self.map_gmap.data[index] = (self.map_gmap.data[index] + *LOG_ODDS_FREE).clamp(LOG_ODDS_CLAMP_MIN, LOG_ODDS_CLAMP_MAX);
            }
        }
    }
    
    /// Updates the cells that are considered occupied space based on the laser scan.
    fn update_occupied_space(&mut self, scan: &[Point2<f32>], pose: &Isometry2<f32>) {
        for endpoint_local in scan.iter() {
            let endpoint_world = pose * endpoint_local;
            let (ix, iy) = world_to_map_coords(endpoint_world.x, endpoint_world.y);

            if ix >= 0 && (ix as usize) < MAP_WIDTH && iy >= 0 && (iy as usize) < MAP_HEIGHT {
                let index = (iy as usize) * MAP_WIDTH + (ix as usize);
                // Update cell as occupied
                self.map_gmap.data[index] = (self.map_gmap.data[index] + *LOG_ODDS_OCC).clamp(LOG_ODDS_CLAMP_MIN, LOG_ODDS_CLAMP_MAX);
            }
        }
    }

    /// Updates the grid using a binary (0 or 1) model.
    fn update_grid_binary(&mut self, scan: &[Point2<f32>], pose: &Isometry2<f32>) {
        for point_local in scan {
            let point_world = pose * point_local;
            let (ix, iy) = world_to_map_coords(point_world.x, point_world.y);

            if ix >= 0 && (ix as usize) < MAP_WIDTH && iy >= 0 && (iy as usize) < MAP_HEIGHT {
                let index = (iy as usize) * MAP_WIDTH + (ix as usize);
                self.map_gmap.data[index] = 1.0; // Occupied
            }
        }
    }

    /// Returns the map points for visualization.
    /// This now generates the points from the occupancy grid.
    pub fn get_map_points(&mut self) -> &Vec<Point2<f32>> {
        if !self.is_map_dirty {
            return &self.cached_map_points;
        }

        self.cached_map_points.clear();
        // Determine the threshold based on the map update method
        let threshold = match self.map_update_method {
            MapUpdateMethod::Probabilistic => 0.0, // log-odds > 0 means P > 0.5
            MapUpdateMethod::Binary => 0.5,         // binary map uses 1.0 for occupied
        };

        for y in 0..self.map_gmap.height {
            for x in 0..self.map_gmap.width {
                let index = y * self.map_gmap.width + x;
                if self.map_gmap.data[index] > threshold {
                    // Convert map index back to world coordinates
                    let world_x = ((x as isize - MAP_ORIGIN_X as isize) as f32) * CSIZE;
                    let world_y = (-(y as isize - MAP_ORIGIN_Y as isize) as f32) * CSIZE;
                    self.cached_map_points.push(Point2::new(world_x, world_y));
                }
            }
        }
        self.is_map_dirty = false;
        &self.cached_map_points
    }
    
    fn generate_and_save_submap(&mut self) {
        let submap_global_pose = self.current_submap_robot_poses[0];

        let mut submap_points_local: Vec<Point2<f32>> = Vec::new();
        for (scan_local, pose_local_to_global) in self
            .current_submap_scan_buffer
            .iter()
            .zip(self.current_submap_robot_poses.iter())
        {
            let relative_pose = submap_global_pose.inverse() * pose_local_to_global;
            let transformed_scan = scan_local.iter().map(|p| relative_pose * p);
            submap_points_local.extend(transformed_scan);
        }

        let submap_id = self.submap_counter;
        let submap_dir_name = format!("submap_{:03}", submap_id);
        let submap_path = self.output_base_dir.join("submaps").join(&submap_dir_name);

        fs::create_dir_all(&submap_path).expect("Failed to create submap directory");
        
        let trajectory_file_name = "trajectory.txt";
        let trajectory_file_path = submap_path.join(trajectory_file_name);

        let mut traj_file = fs::File::create(&trajectory_file_path).expect("Failed to create trajectory.txt");
        for (pose, timestamp) in self.current_submap_robot_poses.iter().zip(self.current_submap_timestamps_buffer.iter()) {
            writeln!(traj_file, "{} {} {} {}", timestamp, pose.translation.x, pose.translation.y, pose.rotation.angle()).expect("Failed to write trajectory data");
        }

        let points_file_name = "points.txt";
        let points_file_path = submap_path.join(points_file_name);
        let info_file_name = "info.yaml";
        let info_file_path = submap_path.join(info_file_name);

        let mut file = fs::File::create(&points_file_path).expect("Failed to create points.txt");
        for p in &submap_points_local {
            writeln!(file, "{} {}", p.x, p.y).expect("Failed to write point");
        }

        let submap_creation_timestamp = self.current_submap_timestamps_buffer.last().cloned().unwrap_or(0);
        let submap_info = Submap {
            id: submap_id,
            global_pose: submap_global_pose,
            pose_x: submap_global_pose.translation.x,
            pose_y: submap_global_pose.translation.y,
            pose_theta: submap_global_pose.rotation.angle(),
            timestamp_ms: submap_creation_timestamp,
            points_file: points_file_name.to_string(),
            info_file: info_file_name.to_string(),
        };

        let yaml_string = serde_yaml::to_string(&submap_info).expect("Failed to serialize submap info to YAML");
        fs::write(&info_file_path, yaml_string).expect("Failed to write info.yaml");

        self.submaps.insert(submap_id, submap_info);
        self.current_submap_scan_buffer.clear();
        self.current_submap_robot_poses.clear();
        self.current_submap_timestamps_buffer.clear();
        self.submap_counter += 1;
    }

    pub fn get_robot_pose(&self) -> &Isometry2<f32> {
        &self.robot_pose
    }
}

/// Converts world coordinates to map grid coordinates.
fn world_to_map_coords(x: f32, y: f32) -> (isize, isize) {
    let ix = (x / CSIZE).round() as isize + MAP_ORIGIN_X as isize;
    let iy = (-y / CSIZE).round() as isize + MAP_ORIGIN_Y as isize;
    (ix, iy)
}

// NOTE: This function is now deprecated in favor of the new update methods.
// It is kept here for reference or for a full-binary-regeneration mode if needed.
#[allow(dead_code)]
pub fn create_occupancy_grid(points: &[Point2<f32>]) -> OccupancyGrid {
    let mut gmap = OccupancyGrid::new(MAP_WIDTH, MAP_HEIGHT);
    for p in points {
        let (ix, iy) = world_to_map_coords(p.x, p.y);
        if ix >= 0 && (ix as usize) < gmap.width && iy >= 0 && (iy as usize) < gmap.height {
            let index = (iy as usize) * gmap.width + (ix as usize);
            gmap.data[index] = 1.0; // Occupied
        }
    }
    gmap
}
