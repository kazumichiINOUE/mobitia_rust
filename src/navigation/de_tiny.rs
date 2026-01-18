use crate::config::SlamConfig;
use crate::slam::OccupancyGrid;
use nalgebra::{Isometry2, Point2, Rotation2, Translation2, Vector3};
use rand::seq::SliceRandom;
use rand::Rng;

struct GaussianKernel {
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

        for val in kernel.iter_mut() {
            *val /= sum_val;
        }

        Self { kernel }
    }
}

pub struct DeTinySolver {
    gaussian_kernel: GaussianKernel,
    pub(crate) config: SlamConfig,
}

impl DeTinySolver {
    pub fn new(config: SlamConfig) -> Self {
        Self {
            gaussian_kernel: GaussianKernel::new(
                config.gaussian_kernel_sigma,
                config.gaussian_kernel_radius,
            ),
            config,
        }
    }

    fn gaussian_match_count(
        &self,
        gmap: &OccupancyGrid,
        points: &[Point2<f32>],
        pose: &Isometry2<f32>,
        resolution: f32,
        origin: [f32; 3],
    ) -> f64 {
        let mut total_score = 0.0;
        let width = gmap.width as i32;
        let height = gmap.height as i32;
        let kernel_radius = self.config.gaussian_kernel_radius;
        let kernel_size = (2 * kernel_radius + 1) as usize;

        let origin_x = origin[0];
        let origin_y = origin[1];

        for p_coord in points {
            let transformed_p = pose * p_coord;

            // World to Grid coordinates
            // Origin is Top-Left (World), so Y decreases as grid Y (image row) increases
            let gx = ((transformed_p.x - origin_x) / resolution).floor() as i32;
            let gy = ((origin_y - transformed_p.y) / resolution).floor() as i32;

            let mut point_score = 0.0;
            for dy in -kernel_radius..=kernel_radius {
                for dx in -kernel_radius..=kernel_radius {
                    let map_x = gx + dx;
                    let map_y = gy + dy;

                    if map_x >= 0 && map_x < width && map_y >= 0 && map_y < height {
                        // map_y is directly the image row index (0 is top)
                        let map_idx = (map_y as usize) * gmap.width + (map_x as usize);
                        let cell_data = gmap.data[map_idx];
                        
                        if cell_data.log_odds > 0.0 {
                            // Cell center in world coordinates
                            // X increases with map_x
                            // Y decreases with map_y
                            let map_point_x = origin_x + (map_x as f32 + 0.5) * resolution;
                            let map_point_y = origin_y - (map_y as f32 + 0.5) * resolution;
                            let map_point = Point2::new(map_point_x, map_point_y);

                            let dist = (transformed_p - map_point).norm();

                            if dist < self.config.max_matching_dist {
                                let occupancy_prob = 1.0 - 1.0 / (1.0 + cell_data.log_odds.exp());
                                let distance_penalty = (-dist.powi(2) / (2.0 * self.config.match_sigma.powi(2))).exp();
                                
                                let kernel_idx = ((dy + kernel_radius) as usize) * kernel_size
                                    + ((dx + kernel_radius) as usize);
                                let kernel_weight = self.gaussian_kernel.kernel[kernel_idx];

                                point_score += occupancy_prob * distance_penalty as f64 * kernel_weight;
                            }
                        }
                    }
                }
            }
            total_score += point_score;
        }
        total_score
    }

    pub fn optimize_de(
        &self,
        gmap: &OccupancyGrid,
        points: &[Point2<f32>],
        initial_pose: Isometry2<f32>,
        resolution: f32,
        origin: [f32; 3],
    ) -> (Isometry2<f32>, f64) {
        // Use config parameters directly
        let wxy = self.config.wxy;
        let wa = self.config.wa_degrees.to_radians();
        let population_size = self.config.population_size;
        let generations = self.config.generations;
        let f_de = self.config.f_de;
        let cr = self.config.cr;

        let mut rng = rand::thread_rng();

        let center_x = initial_pose.translation.x;
        let center_y = initial_pose.translation.y;
        let center_a = initial_pose.rotation.angle();

        let mut population: Vec<Vector3<f32>> = Vec::with_capacity(population_size);
        // Include initial guess in population
        population.push(Vector3::new(center_x, center_y, center_a));
        for _ in 1..population_size {
            let x = rng.gen_range(-wxy..=wxy) + center_x;
            let y = rng.gen_range(-wxy..=wxy) + center_y;
            let a = rng.gen_range(-wa..=wa) + center_a;
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

            scores.push(self.gaussian_match_count(gmap, points, &pose, resolution, origin));
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
                let eval_trial =
                    self.gaussian_match_count(gmap, points, &pose_trial, resolution, origin);

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