pub mod differential_evolution;

use nalgebra::{Isometry2, Point2};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs; // 追加
use std::io::Write; // 追加
use std::time::SystemTime; // 追加

const CSIZE: f32 = 0.025;
const MAP_WIDTH: usize = 3200;
const MAP_HEIGHT: usize = 3200;
const MAP_ORIGIN_X: usize = MAP_WIDTH / 2;
const MAP_ORIGIN_Y: usize = MAP_HEIGHT / 2;

/// Represents a 2D occupancy grid map.
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
            data: vec![0.0; width * height],
        }
    }
}

/// サブマップのメタデータ構造体
#[derive(Debug, Serialize, Deserialize)]
pub struct Submap {
    pub id: usize,
    #[serde(skip)] // nalgebraのIsometry2はSerialize/Deserializeを実装していないのでスキップ
    pub global_pose: Isometry2<f32>, // サブマップ中心のグローバルポーズ
    pub pose_x: f32,         // シリアライズ用
    pub pose_y: f32,         // シリアライズ用
    pub pose_theta: f32,     // シリアライズ用
    pub timestamp_ms: u128,  // 作成時のUnixエポックタイム (ミリ秒)
    pub points_file: String, // 点群データが保存されているファイルパス (submap_XXX/points.txt)
    pub info_file: String,   // このSubmap情報YAMLファイル自身のパス (submap_XXX/info.yaml)
                             // その他、特徴記述子などを追加予定
}

/// The main SLAM state manager.
pub struct SlamManager {
    is_initial_scan: bool,
    map_gmap: OccupancyGrid,
    robot_pose: Isometry2<f32>,
    map_scans_in_world: Vec<Point2<f32>>,
    de_solver: differential_evolution::DifferentialEvolutionSolver,

    // サブマップ関連のフィールド
    submap_counter: usize,       // 次に作成するサブマップのID
    num_scans_per_submap: usize, // 1つのサブマップを構成するスキャン数
    current_submap_scan_buffer: Vec<Vec<Point2<f32>>>, // 現在構築中のサブマップのスキャンデータ
    current_submap_robot_poses: Vec<Isometry2<f32>>, // 現在構築中のサブマップ中のロボットポーズ
    current_submap_timestamps_buffer: Vec<u128>, // 追加: 各スキャンのタイムスタンプ
    submaps: HashMap<usize, Submap>, // 完成したサブマップのリスト (IDでアクセスできるようにHashMapに)
    output_base_dir: std::path::PathBuf, // 追加: 結果を保存するルートディレクトリ
}

impl SlamManager {
    pub fn new(output_base_dir: std::path::PathBuf) -> Self {
        Self {
            is_initial_scan: true,
            map_gmap: OccupancyGrid::new(MAP_WIDTH, MAP_HEIGHT),
            robot_pose: Isometry2::identity(),
            map_scans_in_world: Vec::new(),
            de_solver: differential_evolution::DifferentialEvolutionSolver::new(),
            submap_counter: 0,
            num_scans_per_submap: 20, // 暫定的に20スキャンで1つのサブマップを生成
            current_submap_scan_buffer: Vec::new(),
            current_submap_robot_poses: Vec::new(),
            current_submap_timestamps_buffer: Vec::new(),
            submaps: HashMap::new(),
            output_base_dir,
        }
    }
    /// Processes a new LiDAR scan and updates the map and pose.
    pub fn update(&mut self, lidar_points: &[(f32, f32)], timestamp: u128) {
        let current_scan: Vec<Point2<f32>> =
            lidar_points.iter().map(|p| Point2::new(p.0, p.1)).collect();

        if self.is_initial_scan {
            // First scan: use it as the initial map
            self.robot_pose = Isometry2::identity();
            self.map_scans_in_world = current_scan.clone();
            self.map_gmap = create_occupancy_grid(&self.map_scans_in_world);
            self.is_initial_scan = false;
            
            // 最初のスキャンもバッファに追加
            self.current_submap_scan_buffer.push(current_scan);
            self.current_submap_robot_poses.push(self.robot_pose);
            self.current_submap_timestamps_buffer.push(timestamp);

        } else {
            // Subsequent scans: perform scan matching
            let (best_pose, _score) =
                self.de_solver
                    .optimize_de(&self.map_gmap, &current_scan, self.robot_pose);
            self.robot_pose = best_pose;

            // Add the new scan to the map, transformed by the new pose
            let transformed_scan = current_scan.clone().into_iter().map(|p| best_pose * p);
            self.map_scans_in_world.extend(transformed_scan);
            self.map_gmap = create_occupancy_grid(&self.map_scans_in_world);

            // 現在のスキャンとポーズをサブマップバッファに追加
            self.current_submap_scan_buffer.push(current_scan.clone()); // current_scanの所有権が移動するのでcloneする
            self.current_submap_robot_poses.push(self.robot_pose);
            self.current_submap_timestamps_buffer.push(timestamp);

            // サブマップバッファが一定数に達したらサブマップを生成・保存
            if self.current_submap_scan_buffer.len() >= self.num_scans_per_submap {
                // サブマップのグローバルポーズを推定 (バッファ内のポーズの中央値など)
                // 簡単のため、バッファの最初のポーズを使う
                let submap_global_pose = self.current_submap_robot_poses[0];

                // サブマップ用の点群を統合 (ローカル座標系)
                let mut submap_points_local: Vec<Point2<f32>> = Vec::new();
                for (scan_local, pose_local_to_global) in self
                    .current_submap_scan_buffer
                    .iter()
                    .zip(self.current_submap_robot_poses.iter())
                {
                    // 各スキャンをサブマップの原点（最初のポーズ）からの相対位置に変換
                    let relative_pose = submap_global_pose.inverse() * pose_local_to_global;
                    let transformed_scan = scan_local.iter().map(|p| relative_pose * p);
                    submap_points_local.extend(transformed_scan);
                }

                let submap_id = self.submap_counter;
                let submap_dir_name = format!("submap_{:03}", submap_id); // submap_000, submap_001...
                let submap_path = self.output_base_dir.join("submaps").join(&submap_dir_name);

                // ディレクトリ作成
                fs::create_dir_all(&submap_path).expect("Failed to create submap directory");

                // 軌跡ファイルパス
                let trajectory_file_name = "trajectory.txt";
                let trajectory_file_path = submap_path.join(trajectory_file_name);

                // 軌跡データを保存 (trajectory.txt)
                let mut traj_file =
                    fs::File::create(&trajectory_file_path).expect("Failed to create trajectory.txt");
                for (pose, timestamp) in self
                    .current_submap_robot_poses
                    .iter()
                    .zip(self.current_submap_timestamps_buffer.iter())
                {
                    let x = pose.translation.x;
                    let y = pose.translation.y;
                    let angle = pose.rotation.angle();
                    writeln!(traj_file, "{} {} {} {}", timestamp, x, y, angle)
                        .expect("Failed to write trajectory data");
                }

                // 点群ファイルパス
                let points_file_name = "points.txt";
                let points_file_path = submap_path.join(points_file_name);

                // メタデータファイルパス
                let info_file_name = "info.yaml";
                let info_file_path = submap_path.join(info_file_name);

                // 点群データを保存 (points.txt)
                let mut file =
                    fs::File::create(&points_file_path).expect("Failed to create points.txt");
                for p in &submap_points_local {
                    writeln!(file, "{} {}", p.x, p.y).expect("Failed to write point");
                }

                // サブマップメタデータを構築
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

                // メタデータをYAML形式で保存 (info.yaml)
                let yaml_string = serde_yaml::to_string(&submap_info)
                    .expect("Failed to serialize submap info to YAML");
                fs::write(&info_file_path, yaml_string).expect("Failed to write info.yaml");

                // サブマップリストに追加
                self.submaps.insert(submap_id, submap_info);

                // バッファをクリア
                self.current_submap_scan_buffer.clear();
                self.current_submap_robot_poses.clear();
                self.current_submap_timestamps_buffer.clear();
                self.submap_counter += 1;
            }
        }
    }

    pub fn get_map_points(&self) -> &Vec<Point2<f32>> {
        &self.map_scans_in_world
    }

    pub fn get_robot_pose(&self) -> &Isometry2<f32> {
        &self.robot_pose
    }
}

/// Creates an occupancy grid from a point cloud.
pub fn create_occupancy_grid(points: &[Point2<f32>]) -> OccupancyGrid {
    let mut gmap = OccupancyGrid::new(MAP_WIDTH, MAP_HEIGHT);
    for p in points {
        let ix = (p.x / CSIZE) as i32 + MAP_ORIGIN_X as i32;
        let iy = (-p.y / CSIZE) as i32 + MAP_ORIGIN_Y as i32;
        if ix >= 0 && (ix as usize) < gmap.width && iy >= 0 && (iy as usize) < gmap.height {
            let index = (iy as usize) * gmap.width + (ix as usize);
            gmap.data[index] = 1.0; // Occupied
        }
    }
    gmap
}
