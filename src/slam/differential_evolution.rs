use super::{OccupancyGrid, SlamConfig};
use crate::config::PointRepresentationMethod;
use nalgebra::{Isometry2, Point2, Rotation2, Translation2, Vector2, Vector3};
use rand::seq::SliceRandom;
use rand::Rng;

// --- GaussianKernel moved here ---
struct GaussianKernel {
    radius: i32,
    kernel: Vec<f64>,
}

impl GaussianKernel {
    fn new(sigma: f64, kernel_radius: i32) -> Self {
        let size = (2 * kernel_radius + 1) as usize;
        let mut kernel = vec![0.0; size * size];
        let mut sum_val = 0.0;
        let sigma2 = sigma * sigma;

        for y in -kernel_radius..=kernel_radius {
            for x in -kernel_radius..=kernel_radius {
                let distance_sq = (x * x + y * y) as f64;
                let weight = (-distance_sq / (2.0 * sigma2)).exp();
                let index = ((y + kernel_radius) as usize * size) + (x + kernel_radius) as usize;
                kernel[index] = weight;
                sum_val += weight;
            }
        }

        // Normalize the kernel
        for val in kernel.iter_mut() {
            *val /= sum_val;
        }

        Self {
            radius: kernel_radius,
            kernel,
        }
    }
}
// --- End GaussianKernel ---

/// The main component for solving SLAM using Differential Evolution.
pub struct DifferentialEvolutionSolver {
    gaussian_kernel: GaussianKernel,
    pub(crate) config: SlamConfig,
}

impl DifferentialEvolutionSolver {
    pub fn new(config: SlamConfig) -> Self {
        Self {
            gaussian_kernel: GaussianKernel::new(
                config.gaussian_kernel_sigma,
                config.gaussian_kernel_radius,
            ),
            config,
        }
    }

    /// Helper function to convert map grid coordinates to world center coordinates.
    fn map_grid_to_world_center(map_x: i32, map_y: i32, config: &SlamConfig) -> Point2<f32> {
        let map_origin_x = config.map_width / 2;
        let map_origin_y = config.map_height / 2;
        let world_x = ((map_x as isize - map_origin_x as isize) as f32) * config.csize;
        let world_y = (-(map_y as isize - map_origin_y as isize) as f32) * config.csize;
        Point2::new(world_x, world_y)
    }

    /// Calculates the matching score of a scan against the map at a given pose.
    fn gaussian_match_count(
        &self,
        gmap: &OccupancyGrid,
        points: &[(Point2<f32>, f32, f32, f32)], // (point, edge_ness, nx, ny)
        raw_corner_points: &[(Point2<f32>, f32)], // (point, corner_ness)
        pose: &Isometry2<f32>,
    ) -> f64 {
        let mut total_score = 0.0;

        let height = gmap.height as i32;
        let width = gmap.width as i32;

        let kernel_radius = self.config.gaussian_kernel_radius;
        let kernel_size = (2 * kernel_radius + 1) as usize;

        for (p_coord, scan_feature, scan_nx, scan_ny) in points {
            let transformed_p = pose * p_coord;
            let transformed_normal = pose.rotation * Vector2::new(*scan_nx, *scan_ny);

            let gx =
                (transformed_p.x / self.config.csize) as i32 + (self.config.map_width / 2) as i32;
            let gy =
                (-transformed_p.y / self.config.csize) as i32 + (self.config.map_height / 2) as i32;

            let mut point_score = 0.0;
            for dy in -kernel_radius..=kernel_radius {
                for dx in -kernel_radius..=kernel_radius {
                    let map_x = gx + dx;
                    let map_y = gy + dy;

                    if map_x >= 0 && map_x < width && map_y >= 0 && map_y < height {
                        let map_idx = (map_y as usize) * gmap.width + (map_x as usize);
                        let cell_data = gmap.data[map_idx];
                        let log_odds = cell_data.log_odds;
                        let map_edgeness = cell_data.edge_ness;
                        let map_normal_x = cell_data.normal_x;
                        let map_normal_y = cell_data.normal_y;

                        let kernel_idx = ((dy + kernel_radius) as usize) * kernel_size
                            + ((dx + kernel_radius) as usize);
                        let kernel_weight = self.gaussian_kernel.kernel[kernel_idx];

                        // --- 位置スコアの計算 ---
                        let mut position_score = 0.0;
                        if log_odds > 0.0 {
                            let map_point_for_comparison = match self.config.point_representation {
                                PointRepresentationMethod::CellCenter => {
                                    Self::map_grid_to_world_center(map_x, map_y, &self.config)
                                }
                                PointRepresentationMethod::Centroid => {
                                    if cell_data.point_count > 0 {
                                        Point2::new(
                                            cell_data.centroid_x as f32,
                                            cell_data.centroid_y as f32,
                                        )
                                    } else {
                                        // Fallback to cell center if no points have hit the cell
                                        Self::map_grid_to_world_center(map_x, map_y, &self.config)
                                    }
                                }
                            };

                            let dist = (transformed_p - map_point_for_comparison).norm();

                            if dist < self.config.max_matching_dist {
                                let occupancy_prob = 1.0 - 1.0 / (1.0 + log_odds.exp());
                                // 距離ペナルティ: 距離が遠いほど0に近づくガウス関数
                                let distance_penalty =
                                    (-dist.powi(2) / (2.0 * self.config.match_sigma.powi(2))).exp();
                                position_score = occupancy_prob * distance_penalty as f64;
                            }
                        }

                        // --- 特徴類似度スコアの計算 ---
                        let scan_feature_f64 = *scan_feature as f64;
                        let feature_similarity = scan_feature_f64 * map_edgeness
                            + (1.0 - scan_feature_f64) * (1.0 - map_edgeness);

                        // --- 法線類似度スコアの計算 ---
                        let normal_similarity = (transformed_normal.x as f64 * map_normal_x
                            + transformed_normal.y as f64 * map_normal_y)
                            .max(0.0); // 内積が負の場合は0にクランプ

                        // --- 最終スコアの計算 ---
                        let combined_score = (position_score * self.config.position_score_weight)
                            + (feature_similarity * self.config.feature_score_weight)
                            + (normal_similarity * self.config.normal_alignment_score_weight);

                        point_score += combined_score * kernel_weight;
                    }
                }
            }
            total_score += point_score;
        }

        // --- コーナー点スコアの計算 ---
        let mut corner_match_score = 0.0;
        let corner_match_threshold = 0.1; // スキャンコーナー点と地図コーナー点の距離閾値 (要調整)

        for (p_coord_corner, scan_corner_ness) in raw_corner_points {
            let transformed_p_corner = pose * p_coord_corner;

            let gx_corner =
                (transformed_p_corner.x / self.config.csize) as i32 + (self.config.map_width / 2) as i32;
            let gy_corner =
                (-transformed_p_corner.y / self.config.csize) as i32 + (self.config.map_height / 2) as i32;
            
            if gx_corner >= 0 && gx_corner < width && gy_corner >= 0 && gy_corner < height {
                let map_idx_corner = (gy_corner as usize) * gmap.width + (gx_corner as usize);
                let cell_data_corner = gmap.data[map_idx_corner];
                
                // 地図上のコーナーらしさも考慮
                if cell_data_corner.corner_ness > 0.5 { // 地図のコーナー閾値 (要調整)
                    let map_point_for_comparison = match self.config.point_representation {
                        PointRepresentationMethod::CellCenter => {
                            Self::map_grid_to_world_center(gx_corner, gy_corner, &self.config)
                        }
                        PointRepresentationMethod::Centroid => {
                            if cell_data_corner.point_count > 0 {
                                Point2::new(
                                    cell_data_corner.centroid_x as f32,
                                    cell_data_corner.centroid_y as f32,
                                )
                            } else {
                                Self::map_grid_to_world_center(gx_corner, gy_corner, &self.config)
                            }
                        }
                    };

                    let dist_to_map_corner = (transformed_p_corner - map_point_for_comparison).norm();
                    if dist_to_map_corner < corner_match_threshold {
                        // スキャンと地図のコーナーネスが高いほどスコアも高く
                        corner_match_score += (cell_data_corner.corner_ness * *scan_corner_ness as f64);
                    }
                }
            }
        }
        total_score += corner_match_score * self.config.corner_score_weight;
        
        total_score
    }

    /// Optimizes the robot's pose using Differential Evolution.
    pub fn optimize_de(
        &self,
        gmap: &OccupancyGrid,
        points: &[(Point2<f32>, f32, f32, f32)], // (point, edge_ness, nx, ny)
        raw_corner_points: &[(Point2<f32>, f32)], // (point, corner_ness)
        initial_pose: Isometry2<f32>,
    ) -> (Isometry2<f32>, f64) {
        // DE Parameters (from config)
        let wxy = self.config.wxy;
        let wa = self.config.wa_degrees.to_radians();
        let population_size = self.config.population_size;
        let generations = self.config.generations;
        let f_de = self.config.f_de;
        let cr = self.config.cr;

        let mut rng = rand::thread_rng();

        let initial_x = initial_pose.translation.x;
        let initial_y = initial_pose.translation.y;
        let initial_a = initial_pose.rotation.angle();

        let mut population: Vec<Vector3<f32>> = Vec::with_capacity(population_size);
        for _ in 0..population_size {
            let x = rng.gen_range(-wxy..=wxy) + initial_x;
            let y = rng.gen_range(-wxy..=wxy) + initial_y;
            let a = rng.gen_range(-wa..=wa) + initial_a;
            population.push(Vector3::new(x, y, a));
        }

        let mut scores: Vec<f64> = Vec::with_capacity(population_size);
        for i in 0..population_size {
            let current_x = population[i].x;
            let current_y = population[i].y;
            let current_a = population[i].z;

            let rotation = Rotation2::new(current_a);
            let translation = Translation2::new(current_x, current_y);
            let pose = Isometry2::from_parts(translation, rotation.into());

            scores.push(self.gaussian_match_count(gmap, points, raw_corner_points, &pose));
        }

        let mut best_idx = 0;
        let mut best_eval = scores[0];
        for i in 1..population_size {
            if scores[i] > best_eval {
                best_eval = scores[i];
                best_idx = i;
            }
        }
        let mut best_pose_params = population[best_idx];

        for _gen in 0..generations {
            for i in 0..population_size {
                let mut candidates: Vec<usize> =
                    (0..population_size).filter(|&idx| idx != i).collect();
                candidates.shuffle(&mut rng);
                let r1 = candidates[0];
                let r2 = candidates[1];
                let r3 = candidates[2];

                let p_r1 = &population[r1];
                let p_r2 = &population[r2];
                let p_r3 = &population[r3];

                let vx = p_r1.x + f_de * (p_r2.x - p_r3.x);
                let vy = p_r1.y + f_de * (p_r2.y - p_r3.y);

                let ax1 = p_r1.z.cos();
                let ay1 = p_r1.z.sin();
                let ax2 = p_r2.z.cos();
                let ay2 = p_r2.z.sin();
                let ax3 = p_r3.z.cos();
                let ay3 = p_r3.z.sin();

                let vax = ax1 + f_de * (ax2 - ax3);
                let vay = ay1 + f_de * (ay2 - ay3);
                let va = vay.atan2(vax);

                let mut trial_pose = population[i];
                let j_rand = rng.gen_range(0..3);

                if rng.gen::<f32>() < cr || j_rand == 0 {
                    trial_pose.x = vx;
                }
                if rng.gen::<f32>() < cr || j_rand == 1 {
                    trial_pose.y = vy;
                }
                if rng.gen::<f32>() < cr || j_rand == 2 {
                    trial_pose.z = va;
                }

                let rotation_trial = Rotation2::new(trial_pose.z);
                let translation_trial = Translation2::new(trial_pose.x, trial_pose.y);
                let pose_trial = Isometry2::from_parts(translation_trial, rotation_trial.into());
                let eval_trial = self.gaussian_match_count(gmap, points, raw_corner_points, &pose_trial);

                if eval_trial > scores[i] {
                    population[i] = trial_pose;
                    scores[i] = eval_trial;
                    if eval_trial > best_eval {
                        best_eval = eval_trial;
                        best_pose_params = trial_pose;
                    }
                }
            }
        }

        let x = best_pose_params.x;
        let y = best_pose_params.y;
        let a = best_pose_params.z;

        let final_rotation = Rotation2::new(a);
        let final_translation = Translation2::new(x, y);
        let best_transform = Isometry2::from_parts(final_translation, final_rotation.into());

        (best_transform, best_eval)
    }
}
