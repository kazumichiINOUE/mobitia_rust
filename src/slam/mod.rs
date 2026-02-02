pub mod differential_evolution;

use bresenham::Bresenham;
use nalgebra::{Isometry2, Point2};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::sync::mpsc;
use std::thread;

use crate::config::{MapUpdateMethod, PointRepresentationMethod, SlamConfig};

type SubmapSaveData = (PathBuf, Vec<ScanData>, Submap);

// Manages a background thread for writing submaps to disk
struct SubmapWriter {
    sender: Option<mpsc::Sender<SubmapSaveData>>,
    thread_handle: Option<thread::JoinHandle<()>>,
}

impl SubmapWriter {
    fn new() -> Self {
        let (sender, receiver) = mpsc::channel::<SubmapSaveData>();

        let thread_handle = thread::spawn(move || {
            for (submap_path, scans_data, submap_info) in receiver {
                // This block runs in a separate thread.
                if let Err(e) = fs::create_dir_all(&submap_path) {
                    eprintln!(
                        "ERROR: Failed to create submap directory {:?}: {}",
                        submap_path, e
                    );
                    continue;
                }

                // Save scans.json
                let scans_file_path = submap_path.join(&submap_info.scans_file);
                match fs::File::create(&scans_file_path) {
                    Ok(scans_file) => {
                        if let Err(e) = serde_json::to_writer_pretty(scans_file, &scans_data) {
                            eprintln!("ERROR: Failed to write to {:?}: {}", scans_file_path, e);
                        }
                    }
                    Err(e) => {
                        eprintln!("ERROR: Failed to create {:?}: {}", scans_file_path, e);
                    }
                }

                // Save info.yaml
                let info_file_path = submap_path.join(&submap_info.info_file);
                match serde_yaml::to_string(&submap_info) {
                    Ok(yaml_string) => {
                        if let Err(e) = fs::write(&info_file_path, yaml_string) {
                            eprintln!("ERROR: Failed to write to {:?}: {}", info_file_path, e);
                        }
                    }
                    Err(e) => {
                        eprintln!("ERROR: Failed to serialize submap info to YAML: {}", e);
                    }
                }
            }
        });

        Self {
            sender: Some(sender),
            thread_handle: Some(thread_handle),
        }
    }
}

impl Drop for SubmapWriter {
    fn drop(&mut self) {
        if let Some(sender) = self.sender.take() {
            // Drop the sender to close the channel.
            drop(sender);
        }

        if let Some(handle) = self.thread_handle.take() {
            handle.join().expect("Failed to join submap writer thread");
        }
    }
}

/// Represents a single cell in the occupancy grid.
#[derive(Clone, Copy, Debug)]
pub struct CellData {
    pub log_odds: f64,
    pub edge_ness: f64, // 0.0: 直線, 1.0: エッジ
    pub normal_x: f64,
    pub normal_y: f64,
    pub centroid_x: f64,
    pub centroid_y: f64,
    pub point_count: u32,
    pub corner_ness: f64, // 追加
}

impl Default for CellData {
    fn default() -> Self {
        Self {
            log_odds: 0.0,  // 未知
            edge_ness: 0.5, // 不明
            normal_x: 0.0,
            normal_y: 0.0,
            centroid_x: 0.0,
            centroid_y: 0.0,
            point_count: 0,
            corner_ness: 0.0, // 追加
        }
    }
}

/// Represents a 2D occupancy grid map where each cell holds a log-odds value.
#[derive(Clone)]
pub struct OccupancyGrid {
    pub width: usize,
    pub height: usize,
    pub data: Vec<CellData>,
}

impl OccupancyGrid {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            data: vec![CellData::default(); width * height], // Initialize with default CellData
        }
    }
}

/// サブマップのメタデータ構造体
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Submap {
    pub id: usize,
    #[serde(skip)]
    pub global_pose: Isometry2<f32>,
    pub pose_x: f32,
    pub pose_y: f32,
    pub pose_theta: f32,
    pub timestamp_ms: u128,
    #[serde(default = "default_scans_file")]
    pub scans_file: String,
    pub info_file: String,
}

fn default_scans_file() -> String {
    "scans.json".to_string()
}

#[derive(Serialize, Deserialize)]
pub struct ScanData {
    pub timestamp: u128,
    pub relative_pose: Pose,
    pub scan_points: Vec<ScanPoint>,
    pub valid_point_count: usize,
    pub estimation_method: String,
}

#[derive(Serialize, Deserialize)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

#[derive(Serialize, Deserialize)]
pub struct ScanPoint {
    pub x: f32,
    pub y: f32,
    pub r: f32,
    pub theta: f32,
    pub feature: f32,
    pub nx: f32,
    pub ny: f32,
    pub corner: f32,
}

/// The main SLAM state manager.
pub struct SlamManager {
    is_initial_scan: bool,
    map_gmap: OccupancyGrid,
    robot_pose: Isometry2<f32>,
    de_solver: differential_evolution::DifferentialEvolutionSolver,
    pub(crate) config: SlamConfig,
    log_odds_occ: f64,
    log_odds_free: f64,

    // サブマップ関連のフィールド
    submap_counter: usize,
    num_scans_per_submap: usize,
    current_submap_scan_buffer: Vec<Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>>,
    current_submap_robot_poses: Vec<Isometry2<f32>>,
    current_submap_timestamps_buffer: Vec<u128>,
    current_submap_valid_point_counts: Vec<usize>,
    current_submap_estimation_methods: Vec<String>,
    submaps: HashMap<usize, Submap>,
    output_base_dir: std::path::PathBuf,
    submap_writer: SubmapWriter,

    // キャッシュされた点群
    cached_map_points: Vec<(Point2<f32>, f64)>,
    is_map_dirty: bool, // 地図が更新されたかを示すフラグ

    // UI表示用のフィールド
    pub last_valid_point_count: usize,
}

impl SlamManager {
    pub fn new(output_base_dir: std::path::PathBuf, config: SlamConfig) -> Self {
        let log_odds_occ = (config.prob_occupied / (1.0 - config.prob_occupied)).ln();
        let log_odds_free = (config.prob_free / (1.0 - config.prob_free)).ln();

        Self {
            is_initial_scan: true,
            map_gmap: OccupancyGrid::new(config.map_width, config.map_height),
            robot_pose: Isometry2::identity(),
            de_solver: differential_evolution::DifferentialEvolutionSolver::new(config.clone()),
            submap_counter: 0,
            num_scans_per_submap: config.num_scans_per_submap,
            current_submap_scan_buffer: Vec::new(),
            current_submap_robot_poses: Vec::new(),
            current_submap_timestamps_buffer: Vec::new(),
            current_submap_valid_point_counts: Vec::new(),
            current_submap_estimation_methods: Vec::new(),
            submaps: HashMap::new(),
            output_base_dir,
            submap_writer: SubmapWriter::new(),
            cached_map_points: Vec::new(),
            is_map_dirty: true,
            config,
            log_odds_occ,
            log_odds_free,
            last_valid_point_count: 0,
        }
    }

    pub fn get_current_pose(&self) -> Isometry2<f32> {
        self.robot_pose
    }

    /// Returns a reference to the current occupancy grid.
    pub fn get_grid(&self) -> &OccupancyGrid {
        &self.map_gmap
    }

    /// Returns a reference to the DE solver (which contains the scoring logic).
    pub fn get_solver(&self) -> &differential_evolution::DifferentialEvolutionSolver {
        &self.de_solver
    }

    /// Processes a new LiDAR scan using a provided pose, updating the map and submaps.
    /// This is useful for brute-force search or when ground truth pose is available.
    pub fn update_with_pose(
        &mut self,
        raw_scan_data: &[(f32, f32, f32, f32, f32, f32, f32, f32)],
        pose: Isometry2<f32>,
        timestamp: u128,
    ) {
        // Update current pose
        self.robot_pose = pose;
        self.is_initial_scan = false;

        // --- Map Decay ---
        if self.config.map_update_method == MapUpdateMethod::Probabilistic
            || self.config.map_update_method == MapUpdateMethod::Hybrid
        {
            for cell in self.map_gmap.data.iter_mut() {
                cell.log_odds *= self.config.decay_rate;
                cell.edge_ness = (cell.edge_ness - 0.5) * self.config.decay_rate + 0.5;
                cell.normal_x *= self.config.decay_rate;
                cell.normal_y *= self.config.decay_rate;
            }
        }

        // 地図更新用のスキャンデータ (生データ)
        let mapping_scan_with_features: Vec<(Point2<f32>, f32, f32, f32, f32)> = raw_scan_data
            .iter()
            .map(|p| (Point2::new(p.0, p.1), p.4, p.5, p.6, p.7)) // (point, edge_ness, nx, ny, corner_ness)
            .collect();
        let mapping_scan: Vec<Point2<f32>> = mapping_scan_with_features
            .iter()
            .map(|(p, _, _, _, _)| *p)
            .collect();

        // 地図更新
        match self.config.map_update_method {
            MapUpdateMethod::Probabilistic | MapUpdateMethod::Hybrid => {
                self.update_grid_probabilistic(&mapping_scan, &mapping_scan_with_features, &pose);
            }
            MapUpdateMethod::Binary => {
                self.update_grid_binary(&mapping_scan, &pose);
            }
        }
        self.is_map_dirty = true;

        // --- Submap generation logic ---
        self.current_submap_scan_buffer.push(raw_scan_data.to_vec());
        self.current_submap_robot_poses.push(self.robot_pose);
        self.current_submap_timestamps_buffer.push(timestamp);
        self.current_submap_valid_point_counts.push(raw_scan_data.len());
        self.current_submap_estimation_methods.push("External".to_string());

        if self.current_submap_scan_buffer.len() >= self.config.num_scans_per_submap {
            self.generate_and_save_submap();
        }
    }

    /// Processes a new LiDAR scan and updates the map and pose.
    pub fn update(
        &mut self,
        raw_scan_data: &[(f32, f32, f32, f32, f32, f32, f32, f32)],
        interpolated_scan_data: &[(f32, f32, f32, f32, f32, f32, f32, f32)],
        timestamp: u128,
        odom_guess: Option<(f32, f32, f32)>,
    ) {
        // --- Map Decay ---
        if self.config.map_update_method == MapUpdateMethod::Probabilistic
            || self.config.map_update_method == MapUpdateMethod::Hybrid
        {
            for cell in self.map_gmap.data.iter_mut() {
                cell.log_odds *= self.config.decay_rate;
                // 0.5(不明)に近づける
                cell.edge_ness = (cell.edge_ness - 0.5) * self.config.decay_rate + 0.5;
                // 法線も減衰（ゼロベクトルに近づける）
                cell.normal_x *= self.config.decay_rate;
                cell.normal_y *= self.config.decay_rate;
            }
        }

        // マッチング用のスキャンデータ (補間済み・特徴量付き)
        let matching_scan: Vec<(Point2<f32>, f32, f32, f32)> = interpolated_scan_data
            .iter()
            .map(|p| (Point2::new(p.0, p.1), p.4, p.5, p.6)) // (point, edge_ness, nx, ny)
            .collect();

        // 地図更新用のスキャンデータ (生データ)
        let mapping_scan_with_features: Vec<(Point2<f32>, f32, f32, f32, f32)> = raw_scan_data
            .iter()
            .map(|p| (Point2::new(p.0, p.1), p.4, p.5, p.6, p.7)) // (point, edge_ness, nx, ny, corner_ness)
            .collect();
        let mapping_scan: Vec<Point2<f32>> = mapping_scan_with_features
            .iter()
            .map(|(p, _, _, _, _)| *p)
            .collect();

        let valid_point_count = raw_scan_data.len();
        self.last_valid_point_count = valid_point_count;

        let estimation_method: String;

        if self.is_initial_scan {
            self.robot_pose = Isometry2::identity();
            self.is_initial_scan = false;
            estimation_method = "Initial".to_string();
        } else if valid_point_count < self.config.min_valid_points_for_de {
            // Point count is too low, update pose with odometry guess
            if let Some((odom_dx, odom_dy, odom_dtheta)) = odom_guess {
                let odom_update =
                    Isometry2::new(nalgebra::Vector2::new(odom_dx, odom_dy), odom_dtheta);
                self.robot_pose = self.robot_pose * odom_update;
            }
            // If odom_guess is None, pose is not updated.
            estimation_method = "Odometry".to_string();
        } else {
            // Sufficient points, perform DE optimization
            // --- 生のコーナー点群の作成 ---
            let raw_corner_points: Vec<(Point2<f32>, f32)> = raw_scan_data
                .iter()
                .filter(|p| p.7 > 0.5) // corner_nessが0.5より大きい点をフィルタリング (閾値は要調整)
                .map(|p| (Point2::new(p.0, p.1), p.7)) // (point, corner_ness)
                .collect();

            let (best_pose, _score) = self.de_solver.optimize_de(
                &self.map_gmap,
                &matching_scan,
                &raw_corner_points,
                self.robot_pose,
                odom_guess,
            );
            self.robot_pose = best_pose;
            estimation_method = "DifferentialEvolution".to_string();
        }

        let pose = self.robot_pose; // Copy pose to avoid borrow checker issues

        // 地図更新には生の（補間されていない）スキャンデータを使用
        match self.config.map_update_method {
            MapUpdateMethod::Probabilistic | MapUpdateMethod::Hybrid => {
                self.update_grid_probabilistic(&mapping_scan, &mapping_scan_with_features, &pose);
            }
            MapUpdateMethod::Binary => {
                self.update_grid_binary(&mapping_scan, &pose);
            }
        }
        self.is_map_dirty = true;

        // --- Submap generation logic ---
        self.current_submap_scan_buffer.push(raw_scan_data.to_vec());
        self.current_submap_robot_poses.push(self.robot_pose);
        self.current_submap_timestamps_buffer.push(timestamp);
        self.current_submap_valid_point_counts
            .push(valid_point_count);
        self.current_submap_estimation_methods
            .push(estimation_method);

        if self.current_submap_scan_buffer.len() >= self.config.num_scans_per_submap {
            self.generate_and_save_submap();
        }
    }

    /// Updates the grid using a probabilistic (log-odds) model.
    fn update_grid_probabilistic(
        &mut self,
        scan_for_free: &[Point2<f32>],
        scan_for_occupied: &[(Point2<f32>, f32, f32, f32, f32)], // ここを修正
        pose: &Isometry2<f32>,
    ) {
        // --- Free Space Update ---
        // This part applies the inverse sensor model to update free space probabilities.
        // The logic is encapsulated here and can be replaced with other sensor models in the future.
        self.update_free_space(scan_for_free, pose);

        // --- Occupied Space Update ---
        // This part updates the cells where the laser beams are considered to have hit an obstacle.
        self.update_occupied_space(scan_for_occupied, pose);
    }

    /// Updates the cells that are considered free space based on the laser scan.
    fn update_free_space(&mut self, scan: &[Point2<f32>], pose: &Isometry2<f32>) {
        let robot_pos_map =
            world_to_map_coords(pose.translation.x, pose.translation.y, &self.config);

        for endpoint_local in scan.iter() {
            let endpoint_world = pose * endpoint_local;
            let endpoint_map =
                world_to_map_coords(endpoint_world.x, endpoint_world.y, &self.config);

            // Use Bresenham's algorithm to trace the laser beam
            for (px, py) in Bresenham::new(robot_pos_map, endpoint_map) {
                if px < 0
                    || px as usize >= self.config.map_width
                    || py < 0
                    || py as usize >= self.config.map_height
                {
                    continue;
                }
                let index = py as usize * self.config.map_width + px as usize;

                // Don't update the endpoint itself as free space
                if (px, py) == endpoint_map {
                    break;
                }

                // Update cell as free
                self.map_gmap.data[index].log_odds =
                    (self.map_gmap.data[index].log_odds + self.log_odds_free).clamp(
                        self.config.log_odds_clamp_min,
                        self.config.log_odds_clamp_max,
                    );
            }
        }
    }

    /// Updates the cells that are considered occupied space based on the laser scan.
    fn update_occupied_space(
        &mut self,
        scan: &[(Point2<f32>, f32, f32, f32, f32)], // 5要素に修正 (point, edge_ness, nx, ny, corner_ness)
        pose: &Isometry2<f32>,
    ) {
        for (endpoint_local, feature, nx, ny, corner_feature) in scan.iter() {
            // ここも修正
            let endpoint_world = pose * endpoint_local;
            let (ix, iy) = world_to_map_coords(endpoint_world.x, endpoint_world.y, &self.config);

            if ix >= 0
                && (ix as usize) < self.config.map_width
                && iy >= 0
                && (iy as usize) < self.config.map_height
            {
                let index = (iy as usize) * self.config.map_width + (ix as usize);
                let cell = &mut self.map_gmap.data[index];

                // log_odds を更新
                cell.log_odds = (cell.log_odds + self.log_odds_occ).clamp(
                    self.config.log_odds_clamp_min,
                    self.config.log_odds_clamp_max,
                );

                // edge_ness を更新 (加重平均)
                cell.edge_ness = (cell.edge_ness * 0.7) + (*feature as f64 * 0.3);

                // corner_ness を更新 (加重平均)
                cell.corner_ness = (cell.corner_ness * 0.7) + (*corner_feature as f64 * 0.3);

                // 法線ベクトルを更新 (加重平均)
                cell.normal_x = (cell.normal_x * 0.7) + (*nx as f64 * 0.3);
                cell.normal_y = (cell.normal_y * 0.7) + (*ny as f64 * 0.3);
                // 正規化
                let len = (cell.normal_x.powi(2) + cell.normal_y.powi(2)).sqrt();
                if len > 1e-9 {
                    cell.normal_x /= len;
                    cell.normal_y /= len;
                }

                // 重心座標を逐次計算で更新
                let old_count = cell.point_count as f64;
                let new_count = old_count + 1.0;
                if cell.point_count == 0 {
                    cell.centroid_x = endpoint_world.x as f64;
                    cell.centroid_y = endpoint_world.y as f64;
                } else {
                    cell.centroid_x =
                        (cell.centroid_x * old_count + endpoint_world.x as f64) / new_count;
                    cell.centroid_y =
                        (cell.centroid_y * old_count + endpoint_world.y as f64) / new_count;
                }
                cell.point_count += 1;
            }
        }
    }

    /// Updates the grid using a binary (0 or 1) model.
    fn update_grid_binary(&mut self, scan: &[Point2<f32>], pose: &Isometry2<f32>) {
        for point_local in scan {
            let point_world = pose * point_local;
            let (ix, iy) = world_to_map_coords(point_world.x, point_world.y, &self.config);

            if ix >= 0
                && (ix as usize) < self.config.map_width
                && iy >= 0
                && (iy as usize) < self.config.map_height
            {
                let index = (iy as usize) * self.config.map_width + (ix as usize);
                self.map_gmap.data[index].log_odds = 1.0; // Occupied
            }
        }
    }

    /// Returns the map points for visualization.
    /// This now generates the points from the occupancy grid.
    pub fn get_map_points(&mut self) -> &Vec<(Point2<f32>, f64)> {
        if !self.is_map_dirty {
            return &self.cached_map_points;
        }

        self.cached_map_points.clear();
        // Determine the threshold based on the map update method
        let threshold = match self.config.map_update_method {
            MapUpdateMethod::Probabilistic | MapUpdateMethod::Hybrid => 0.0, // log-odds > 0 means P > 0.5
            MapUpdateMethod::Binary => 0.5, // binary map uses 1.0 for occupied
        };

        for y in 0..self.map_gmap.height {
            for x in 0..self.map_gmap.width {
                let index = y * self.map_gmap.width + x;
                let cell_data = self.map_gmap.data[index];
                let cell_log_odds = cell_data.log_odds;

                if cell_log_odds > threshold {
                    let (world_x, world_y) = match self.config.point_representation {
                        PointRepresentationMethod::CellCenter => {
                            // Convert map index back to world coordinates (cell center)
                            (
                                ((x as isize - (self.config.map_width / 2) as isize) as f32)
                                    * self.config.csize,
                                (-(y as isize - (self.config.map_height / 2) as isize) as f32)
                                    * self.config.csize,
                            )
                        }
                        PointRepresentationMethod::Centroid => {
                            if cell_data.point_count > 0 {
                                (cell_data.centroid_x as f32, cell_data.centroid_y as f32)
                            } else {
                                // Fallback to cell center if no points have hit the cell
                                (
                                    ((x as isize - (self.config.map_width / 2) as isize) as f32)
                                        * self.config.csize,
                                    (-(y as isize - (self.config.map_height / 2) as isize) as f32)
                                        * self.config.csize,
                                )
                            }
                        }
                    };
                    let probability = log_odds_to_probability(cell_log_odds);
                    self.cached_map_points
                        .push((Point2::new(world_x, world_y), probability));
                }
            }
        }
        self.is_map_dirty = false;
        &self.cached_map_points
    }

    fn generate_and_save_submap(&mut self) {
        if self.current_submap_robot_poses.is_empty() {
            return;
        }

        let submap_global_pose = self.current_submap_robot_poses[0];
        let submap_id = self.submap_counter;

        let mut scans_data: Vec<ScanData> = Vec::new();

        for ((((scan, pose), timestamp), valid_count), estimation_method) in self
            .current_submap_scan_buffer
            .iter()
            .zip(self.current_submap_robot_poses.iter())
            .zip(self.current_submap_timestamps_buffer.iter())
            .zip(self.current_submap_valid_point_counts.iter())
            .zip(self.current_submap_estimation_methods.iter())
        {
            let relative_pose_isometry = submap_global_pose.inverse() * pose;
            let relative_pose = Pose {
                x: relative_pose_isometry.translation.x,
                y: relative_pose_isometry.translation.y,
                theta: relative_pose_isometry.rotation.angle(),
            };

            let scan_points = scan
                .iter()
                .map(|p| ScanPoint {
                    x: p.0,
                    y: p.1,
                    r: p.2,
                    theta: p.3,
                    feature: p.4,
                    nx: p.5,
                    ny: p.6,
                    corner: p.7,
                })
                .collect();

            scans_data.push(ScanData {
                timestamp: *timestamp,
                relative_pose,
                scan_points,
                valid_point_count: *valid_count,
                estimation_method: estimation_method.clone(),
            });
        }

        // --- Prepare data for saving ---
        let submap_dir_name = format!("submap_{:03}", submap_id);
        let submap_path = self.output_base_dir.join("submaps").join(&submap_dir_name);

        let scans_file_name = "scans.json";
        let info_file_name = "info.yaml";
        let submap_creation_timestamp = self
            .current_submap_timestamps_buffer
            .last()
            .cloned()
            .unwrap_or(0);

        let submap_info = Submap {
            id: submap_id,
            global_pose: submap_global_pose,
            pose_x: submap_global_pose.translation.x,
            pose_y: submap_global_pose.translation.y,
            pose_theta: submap_global_pose.rotation.angle(),
            timestamp_ms: submap_creation_timestamp,
            scans_file: scans_file_name.to_string(),
            info_file: info_file_name.to_string(),
        };

        // --- Send data to writer thread ---
        if let Some(sender) = &self.submap_writer.sender {
            if let Err(e) = sender.send((submap_path, scans_data, submap_info.clone())) {
                eprintln!("ERROR: Failed to send submap to writer thread: {}", e);
            }
        }

        // --- Clear buffers ---
        self.submaps.insert(submap_id, submap_info);
        self.current_submap_scan_buffer.clear();
        self.current_submap_robot_poses.clear();
        self.current_submap_timestamps_buffer.clear();
        self.current_submap_valid_point_counts.clear();
        self.current_submap_estimation_methods.clear();
        self.submap_counter += 1;
    }

    pub fn get_robot_pose(&self) -> &Isometry2<f32> {
        &self.robot_pose
    }

    pub fn get_last_valid_point_count(&self) -> usize {
        self.last_valid_point_count
    }
}

impl Drop for SlamManager {
    fn drop(&mut self) {
        if !self.current_submap_scan_buffer.is_empty() {
            self.generate_and_save_submap();
        }
        // The submap_writer field will be dropped automatically here,
        // which will wait for the I/O thread to finish.
    }
}

/// Converts world coordinates to map grid coordinates.
pub fn world_to_map_coords(x: f32, y: f32, config: &SlamConfig) -> (isize, isize) {
    let map_origin_x = config.map_width / 2;
    let map_origin_y = config.map_height / 2;
    let ix = (x / config.csize).round() as isize + map_origin_x as isize;
    let iy = (-y / config.csize).round() as isize + map_origin_y as isize;
    (ix, iy)
}

// NOTE: This function is now deprecated in favor of the new update methods.
// It is kept here for reference or for a full-binary-regeneration mode if needed.
#[allow(dead_code)]
pub fn create_occupancy_grid(points: &[Point2<f32>], config: &SlamConfig) -> OccupancyGrid {
    let mut gmap = OccupancyGrid::new(config.map_width, config.map_height);
    for p in points {
        let (ix, iy) = world_to_map_coords(p.x, p.y, config);
        if ix >= 0 && (ix as usize) < gmap.width && iy >= 0 && (iy as usize) < gmap.height {
            let index = (iy as usize) * gmap.width + (ix as usize);
            gmap.data[index].log_odds = 1.0; // Occupied
        }
    }
    gmap
}

/// Converts a log-odds value to a probability (0.0 to 1.0).
pub fn log_odds_to_probability(log_odds: f64) -> f64 {
    1.0 / (1.0 + (-log_odds).exp())
}
