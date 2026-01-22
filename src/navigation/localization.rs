use crate::config::{PointRepresentationMethod, SlamConfig};
use crate::slam::OccupancyGrid;
use nalgebra::{Isometry2, Point2, Rotation2, Translation2, Vector3};
use rand::seq::SliceRandom;
use rand::Rng;

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

        for val in kernel.iter_mut() {
            *val /= sum_val;
        }

        Self {
            radius: kernel_radius,
            kernel,
        }
    }
}

pub struct LocalizationSolver {
    gaussian_kernel: GaussianKernel,
    pub(crate) config: SlamConfig,
}

impl LocalizationSolver {
    pub fn new(config: SlamConfig) -> Self {
        Self {
            gaussian_kernel: GaussianKernel::new(
                config.gaussian_kernel_sigma,
                config.gaussian_kernel_radius,
            ),
            config,
        }
    }

    fn map_grid_to_world_center(map_x: i32, map_y: i32, config: &SlamConfig) -> Point2<f32> {
        let map_origin_x = config.map_width / 2;
        let map_origin_y = config.map_height / 2;
        let world_x = ((map_x as isize - map_origin_x as isize) as f32) * config.csize;
        let world_y = (-(map_y as isize - map_origin_y as isize) as f32) * config.csize;
        Point2::new(world_x, world_y)
    }

    fn simplified_gaussian_match_count(
        &self,
        gmap: &OccupancyGrid,
        points: &[Point2<f32>],
        pose: &Isometry2<f32>,
    ) -> f64 {
        let mut total_score = 0.0;
        let height = gmap.height as i32;
        let width = gmap.width as i32;
        let kernel_radius = self.config.gaussian_kernel_radius;
        let kernel_size = (2 * kernel_radius + 1) as usize;

        for p_coord in points {
            let transformed_p = pose * p_coord;
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

                        if log_odds > 0.0 {
                            let kernel_idx = ((dy + kernel_radius) as usize) * kernel_size
                                + ((dx + kernel_radius) as usize);
                            let kernel_weight = self.gaussian_kernel.kernel[kernel_idx];
                            let occupancy_prob = 1.0 - 1.0 / (1.0 + log_odds.exp());
                            point_score += occupancy_prob * kernel_weight;
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
        odom_guess: Option<(f32, f32, f32)>,
    ) -> (Isometry2<f32>, f64) {
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

        let (center_x, center_y, center_a) = if let Some((dx, dy, dtheta)) = odom_guess {
            (initial_x + dx, initial_y + dy, initial_a + dtheta)
        } else {
            (initial_x, initial_y, initial_a)
        };

        let mut population: Vec<Vector3<f32>> = Vec::with_capacity(population_size);
        for _ in 0..population_size {
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

            scores.push(self.simplified_gaussian_match_count(gmap, points, &pose));
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
                let va = p_r1.z + f_de * (p_r2.z - p_r3.z); // Simplified angle mutation

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
                let eval_trial = self.simplified_gaussian_match_count(gmap, points, &pose_trial);

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
