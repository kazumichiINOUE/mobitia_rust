use crate::config::SlamConfig;
use crate::slam::OccupancyGrid;
use nalgebra::{Isometry2, Point2, Rotation2, Translation2, Vector2, Vector3};
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
    pub config: SlamConfig,
    pub population: Vec<Vector3<f32>>,
    pub scores: Vec<f64>,
    pub best_pose_params: Vector3<f32>,
    pub best_score: f64,
    pub generation: usize,
    pub is_initialized: bool,
    pub is_converged: bool,

    current_population_size: usize,
    current_generations: usize,

    gaussian_kernel: GaussianKernel,
    rng: rand::rngs::ThreadRng,
}

/// A lightweight scan point structure for navigation localization
pub struct NavScanPoint {
    pub pos: Point2<f32>,
    pub normal: Vector2<f32>,
    pub feature: f32, // edge_ness
}

impl DeTinySolver {
    pub fn new(config: SlamConfig) -> Self {
        Self {
            gaussian_kernel: GaussianKernel::new(
                config.gaussian_kernel_sigma,
                config.gaussian_kernel_radius,
            ),
            current_population_size: config.population_size,
            current_generations: config.generations,
            config: config.clone(),
            population: Vec::new(),
            scores: Vec::new(),
            best_pose_params: Vector3::zeros(),
            best_score: 0.0,
            generation: 0,
            is_initialized: false,
            is_converged: false,
            rng: rand::thread_rng(),
        }
    }

    pub fn init(
        &mut self,
        initial_pose: Isometry2<f32>,
        override_params: Option<(f32, f32, usize, usize)>,
    ) {
        let (wxy, wa, population_size, generations) = if let Some((w, a, p, g)) = override_params {
            (w, a.to_radians(), p, g)
        } else {
            (
                self.config.wxy,
                self.config.wa_degrees.to_radians(),
                self.config.population_size,
                self.config.generations,
            )
        };

        self.current_population_size = population_size;
        self.current_generations = generations;

        // println!("DEBUG: DE Init. wxy={:.3}, wa={:.3}, pop={}, gen={}", wxy, wa, population_size, generations);

        let center_x = initial_pose.translation.x;
        let center_y = initial_pose.translation.y;
        let center_a = initial_pose.rotation.angle();

        self.population.clear();
        self.population.reserve(population_size);

        // 1. Always include the initial guess itself
        self.population
            .push(Vector3::new(center_x, center_y, center_a));

        // 2. Fill the rest with random samples around the center
        for _ in 1..population_size {
            let x = self.rng.gen_range(-wxy..=wxy) + center_x;
            let y = self.rng.gen_range(-wxy..=wxy) + center_y;
            let a = self.rng.gen_range(-wa..=wa) + center_a;
            self.population.push(Vector3::new(x, y, a));
        }

        /*
        let min_x = self.population.iter().map(|v| v.x).fold(f32::INFINITY, f32::min);
        let max_x = self.population.iter().map(|v| v.x).fold(f32::NEG_INFINITY, f32::max);
        let min_y = self.population.iter().map(|v| v.y).fold(f32::INFINITY, f32::min);
        let max_y = self.population.iter().map(|v| v.y).fold(f32::NEG_INFINITY, f32::max);
        println!("DEBUG: Population Bounds X:[{:.3}, {:.3}] (W:{:.3}), Y:[{:.3}, {:.3}] (H:{:.3})", min_x, max_x, max_x - min_x, min_y, max_y, max_y - min_y);
        */

        self.scores = vec![0.0; population_size];
        self.best_score = f64::NEG_INFINITY;
        self.generation = 0;
        self.is_initialized = true;
        self.is_converged = false;
    }

    pub fn step(
        &mut self,
        gmap: &OccupancyGrid,
        points: &[NavScanPoint],
        resolution: f32,
        origin: [f32; 3],
    ) {
        if !self.is_initialized || self.is_converged {
            return;
        }

        let population_size = self.current_population_size;

        if self.generation == 0 {
            for i in 0..population_size {
                let pose_vec = self.population[i];
                let pose = self.vec_to_pose(pose_vec);
                self.scores[i] = self.evaluate(gmap, points, &pose, resolution, origin);

                if self.scores[i] > self.best_score {
                    self.best_score = self.scores[i];
                    self.best_pose_params = pose_vec;
                }
            }
            self.generation += 1;
            return;
        }

        let f_de = self.config.f_de;
        let cr = self.config.cr;

        for i in 0..population_size {
            let mut candidates: Vec<usize> = (0..population_size).filter(|&idx| idx != i).collect();
            if candidates.len() < 3 {
                continue;
            }

            candidates.shuffle(&mut self.rng);
            let r1 = candidates[0];
            let r2 = candidates[1];
            let r3 = candidates[2];

            let p_r1 = &self.population[r1];
            let p_r2 = &self.population[r2];
            let p_r3 = &self.population[r3];

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

            let mut trial_vec = self.population[i];
            let j_rand = self.rng.gen_range(0..3);

            if self.rng.gen::<f32>() < cr || j_rand == 0 {
                trial_vec.x = vx;
            }
            if self.rng.gen::<f32>() < cr || j_rand == 1 {
                trial_vec.y = vy;
            }
            if self.rng.gen::<f32>() < cr || j_rand == 2 {
                trial_vec.z = va;
            }

            let pose_trial = self.vec_to_pose(trial_vec);
            let score_trial = self.evaluate(gmap, points, &pose_trial, resolution, origin);

            if score_trial > self.scores[i] {
                self.population[i] = trial_vec;
                self.scores[i] = score_trial;
                if score_trial > self.best_score {
                    self.best_score = score_trial;
                    self.best_pose_params = trial_vec;
                }
            }
        }

        self.generation += 1;
        if self.generation >= self.current_generations {
            self.is_converged = true;
        }
    }

    pub fn get_best_pose(&self) -> Isometry2<f32> {
        self.vec_to_pose(self.best_pose_params)
    }

    fn vec_to_pose(&self, v: Vector3<f32>) -> Isometry2<f32> {
        let rotation = Rotation2::new(v.z);
        let translation = Translation2::new(v.x, v.y);
        Isometry2::from_parts(translation, rotation.into())
    }

    /// Advanced evaluation using occupancy, normals, and features.
    fn evaluate(
        &self,
        gmap: &OccupancyGrid,
        points: &[NavScanPoint],
        pose: &Isometry2<f32>,
        resolution: f32,
        origin: [f32; 3],
    ) -> f64 {
        let mut total_score = 0.0;
        let width = gmap.width as i32;
        let height = gmap.height as i32;
        let origin_x = origin[0];
        let origin_y = origin[1];

        let kernel_radius = self.config.gaussian_kernel_radius;
        let kernel_size = (2 * kernel_radius + 1) as usize;

        // Weights from config
        let pos_w = self.config.position_score_weight;
        let feat_w = self.config.feature_score_weight;
        let norm_w = self.config.normal_alignment_score_weight;

        for p_nav in points {
            let p_world = pose * p_nav.pos;
            let normal_world = pose.rotation * p_nav.normal;

            // Transform World Coordinates to Grid Index
            let gx = ((p_world.x - origin_x) / resolution).floor() as i32;
            let gy = ((origin_y - p_world.y) / resolution).floor() as i32;

            let mut point_score = 0.0;

            for dy in -kernel_radius..=kernel_radius {
                for dx in -kernel_radius..=kernel_radius {
                    let map_x = gx + dx;
                    let map_y = gy + dy;

                    if map_x >= 0 && map_x < width && map_y >= 0 && map_y < height {
                        let map_idx = (map_y as usize) * gmap.width + (map_x as usize);
                        let cell = gmap.data[map_idx];

                        if cell.log_odds > 0.0 {
                            let cell_world_x = origin_x + (map_x as f32 + 0.5) * resolution;
                            let cell_world_y = origin_y - (map_y as f32 + 0.5) * resolution;
                            let cell_center = Point2::new(cell_world_x, cell_world_y);

                            let dist_sq = (p_world - cell_center).norm_squared();

                            if dist_sq < self.config.max_matching_dist.powi(2) {
                                // 1. Position score (Gaussian distance penalty)
                                let distance_penalty =
                                    (-dist_sq / (2.0 * self.config.match_sigma.powi(2))).exp();

                                // 2. Normal alignment score
                                let cell_normal =
                                    Vector2::new(cell.normal_x as f32, cell.normal_y as f32);
                                let normal_alignment = (normal_world.dot(&cell_normal)).abs(); // Use abs for bi-directional walls

                                // 3. Feature score (edge_ness similarity)
                                let feature_similarity =
                                    1.0 - (p_nav.feature as f64 - cell.edge_ness).abs();

                                // Pre-computed kernel weight
                                let kernel_idx = ((dy + kernel_radius) as usize) * kernel_size
                                    + ((dx + kernel_radius) as usize);
                                let kernel_weight = self.gaussian_kernel.kernel[kernel_idx];

                                let combined_score = pos_w * distance_penalty as f64
                                    + norm_w * normal_alignment as f64
                                    + feat_w * feature_similarity;

                                let prob = 1.0 - 1.0 / (1.0 + cell.log_odds.exp());
                                point_score += prob * combined_score * kernel_weight;
                            }
                        }
                    }
                }
            }
            total_score += point_score;
        }
        total_score
    }
}
