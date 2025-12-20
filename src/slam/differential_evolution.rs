use super::{MapUpdateMethod, OccupancyGrid, CSIZE, MAP_ORIGIN_X, MAP_ORIGIN_Y};
use nalgebra::{Isometry2, Point2, Rotation2, Translation2, Vector3};
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
}

impl DifferentialEvolutionSolver {
    pub fn new() -> Self {
        Self {
            gaussian_kernel: GaussianKernel::new(0.8, 2),
        }
    }

    /// Calculates the matching score of a scan against the map at a given pose.
    fn gaussian_match_count(
        gmap: &OccupancyGrid,
        points: &[Point2<f32>],
        pose: &Isometry2<f32>,
        kernel_obj: &GaussianKernel,
        map_update_method: MapUpdateMethod, // 引数を追加
    ) -> f64 {
        let mut total_score = 0.0;

        let height = gmap.height as i32;
        let width = gmap.width as i32;

        let kernel_radius = kernel_obj.radius;
        let kernel_size = (2 * kernel_radius + 1) as usize;

        for p in points {
            let transformed_p = pose * p;
            let gx = (transformed_p.x / CSIZE) as i32 + MAP_ORIGIN_X as i32;
            let gy = (-transformed_p.y / CSIZE) as i32 + MAP_ORIGIN_Y as i32;

            let mut point_score = 0.0;
            for dy in -kernel_radius..=kernel_radius {
                for dx in -kernel_radius..=kernel_radius {
                    let map_x = gx + dx;
                    let map_y = gy + dy;

                    if map_x >= 0 && map_x < width && map_y >= 0 && map_y < height {
                        let map_idx = (map_y as usize) * gmap.width + (map_x as usize);
                        let log_odds = gmap.data[map_idx];

                        // Calculate cell score based on map update method
                        let cell_score = match map_update_method {
                            MapUpdateMethod::Probabilistic => {
                                if log_odds > 0.0 {
                                    1.0 - 1.0 / (1.0 + log_odds.exp())
                                } else {
                                    0.0
                                }
                            }
                            MapUpdateMethod::Binary | MapUpdateMethod::Hybrid => {
                                if log_odds > 0.0 {
                                    1.0
                                } else {
                                    0.0
                                }
                            }
                        };

                        if cell_score > 0.0 {
                            let kernel_idx = ((dy + kernel_radius) as usize) * kernel_size
                                + ((dx + kernel_radius) as usize);
                            point_score += cell_score * kernel_obj.kernel[kernel_idx];
                        }
                    }
                }
            }
            total_score += point_score;
        }
        total_score
    }

    /// Optimizes the robot's pose using Differential Evolution.
    pub fn optimize_de(
        &self,
        gmap: &OccupancyGrid,
        points: &[Point2<f32>],
        initial_pose: Isometry2<f32>,
        map_update_method: MapUpdateMethod, // 引数を追加
    ) -> (Isometry2<f32>, f64) {
        // DE Parameters
        const WXY: f32 = 0.8;
        const WA: f32 = std::f32::consts::PI * 35.0 / 180.0;
        const POPULATION_SIZE: usize = 200;
        const GENERATIONS: usize = 100; // Reduced for faster computation
        const F: f32 = 0.1; // Mutation factor
        const CR: f32 = 0.6;

        let mut rng = rand::thread_rng();

        let initial_x = initial_pose.translation.x;
        let initial_y = initial_pose.translation.y;
        let initial_a = initial_pose.rotation.angle();

        let mut population: Vec<Vector3<f32>> = Vec::with_capacity(POPULATION_SIZE);
        for _ in 0..POPULATION_SIZE {
            let x = rng.gen_range(-WXY..=WXY) + initial_x;
            let y = rng.gen_range(-WXY..=WXY) + initial_y;
            let a = rng.gen_range(-WA..=WA) + initial_a;
            population.push(Vector3::new(x, y, a));
        }

        let mut scores: Vec<f64> = Vec::with_capacity(POPULATION_SIZE);
        for i in 0..POPULATION_SIZE {
            let current_x = population[i].x;
            let current_y = population[i].y;
            let current_a = population[i].z;

            let rotation = Rotation2::new(current_a);
            let translation = Translation2::new(current_x, current_y);
            let pose = Isometry2::from_parts(translation, rotation.into());

            scores.push(Self::gaussian_match_count(
                gmap,
                points,
                &pose,
                &self.gaussian_kernel,
                map_update_method,
            ));
        }

        let mut best_idx = 0;
        let mut best_eval = scores[0];
        for i in 1..POPULATION_SIZE {
            if scores[i] > best_eval {
                best_eval = scores[i];
                best_idx = i;
            }
        }
        let mut best_pose_params = population[best_idx];

        for _gen in 0..GENERATIONS {
            for i in 0..POPULATION_SIZE {
                let mut candidates: Vec<usize> =
                    (0..POPULATION_SIZE).filter(|&idx| idx != i).collect();
                candidates.shuffle(&mut rng);
                let r1 = candidates[0];
                let r2 = candidates[1];
                let r3 = candidates[2];

                let p_r1 = &population[r1];
                let p_r2 = &population[r2];
                let p_r3 = &population[r3];

                let vx = p_r1.x + F * (p_r2.x - p_r3.x);
                let vy = p_r1.y + F * (p_r2.y - p_r3.y);

                let ax1 = p_r1.z.cos();
                let ay1 = p_r1.z.sin();
                let ax2 = p_r2.z.cos();
                let ay2 = p_r2.z.sin();
                let ax3 = p_r3.z.cos();
                let ay3 = p_r3.z.sin();

                let vax = ax1 + F * (ax2 - ax3);
                let vay = ay1 + F * (ay2 - ay3);
                let va = vay.atan2(vax);

                let mut trial_pose = population[i];
                let j_rand = rng.gen_range(0..3);

                if rng.gen::<f32>() < CR || j_rand == 0 {
                    trial_pose.x = vx;
                }
                if rng.gen::<f32>() < CR || j_rand == 1 {
                    trial_pose.y = vy;
                }
                if rng.gen::<f32>() < CR || j_rand == 2 {
                    trial_pose.z = va;
                }

                let rotation_trial = Rotation2::new(trial_pose.z);
                let translation_trial = Translation2::new(trial_pose.x, trial_pose.y);
                let pose_trial = Isometry2::from_parts(translation_trial, rotation_trial.into());
                let eval_trial = Self::gaussian_match_count(
                    gmap,
                    points,
                    &pose_trial,
                    &self.gaussian_kernel,
                    map_update_method,
                );

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
