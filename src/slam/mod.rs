use nalgebra::{Isometry2, Point2, Vector3, Rotation2, Translation2};
use rand::seq::SliceRandom;
use rand::Rng;

const CSIZE: f32 = 0.025;
const MAP_WIDTH: usize = 1500;
const MAP_HEIGHT: usize = 1000;
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

/// Represents a pre-calculated Gaussian kernel for scoring.
pub struct GaussianKernel {
    pub radius: i32,
    pub kernel: Vec<f64>,
}

impl GaussianKernel {
    pub fn new(sigma: f64, kernel_radius: i32) -> Self {
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

/// The main SLAM state manager.
pub struct SlamManager {
    is_initial_scan: bool,
    map_gmap: OccupancyGrid,
    robot_pose: Isometry2<f32>,
    map_scans_in_world: Vec<Point2<f32>>,
    gaussian_kernel: GaussianKernel,
}

impl SlamManager {
    pub fn new() -> Self {
        Self {
            is_initial_scan: true,
            map_gmap: OccupancyGrid::new(MAP_WIDTH, MAP_HEIGHT),
            robot_pose: Isometry2::identity(),
            map_scans_in_world: Vec::new(),
            gaussian_kernel: GaussianKernel::new(0.8, 2),
        }
    }

    /// Processes a new LiDAR scan and updates the map and pose.
    pub fn update(&mut self, lidar_points: &[(f32, f32)]) {
        let current_scan: Vec<Point2<f32>> =
            lidar_points.iter().map(|p| Point2::new(p.0, p.1)).collect();

        if self.is_initial_scan {
            // First scan: use it as the initial map
            self.robot_pose = Isometry2::identity();
            self.map_scans_in_world = current_scan;
            self.map_gmap = create_occupancy_grid(&self.map_scans_in_world);
            self.is_initial_scan = false;
        } else {
            // Subsequent scans: perform scan matching
            let (best_pose, _score) = optimize_de(
                &self.map_gmap,
                &current_scan,
                &self.gaussian_kernel,
                self.robot_pose,
            );
            self.robot_pose = best_pose;

            // Add the new scan to the map, transformed by the new pose
            let transformed_scan = current_scan.into_iter().map(|p| best_pose * p);
            self.map_scans_in_world.extend(transformed_scan);
            self.map_gmap = create_occupancy_grid(&self.map_scans_in_world);
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
fn create_occupancy_grid(points: &[Point2<f32>]) -> OccupancyGrid {
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

/// Calculates the matching score of a scan against the map at a given pose.
fn gaussian_match_count(
    gmap: &OccupancyGrid,
    points: &[Point2<f32>],
    pose: &Isometry2<f32>,
    kernel_obj: &GaussianKernel,
) -> f64 {
    let mut total_score = 0.0;
    
    let height = gmap.height as i32;
    let width = gmap.width as i32;

    let kernel_radius = kernel_obj.radius;
    let kernel_size = (2 * kernel_radius + 1) as usize; // kernel_obj.kernel は一次元配列なのでサイズが必要

    for p in points {
        // Transform the point using the pose
        let transformed_p = pose * p; // nalgebraのIsometry2はPoint2を直接変換できる

        let gx = (transformed_p.x / CSIZE) as i32 + MAP_ORIGIN_X as i32;
        let gy = (-transformed_p.y / CSIZE) as i32 + MAP_ORIGIN_Y as i32;

        let mut point_score = 0.0;
        for dy in -kernel_radius..=kernel_radius {
            for dx in -kernel_radius..=kernel_radius {
                let map_x = gx + dx;
                let map_y = gy + dy;

                if map_x >= 0 && map_x < width && map_y >= 0 && map_y < height {
                    let map_idx = (map_y as usize) * gmap.width + (map_x as usize);
                    if gmap.data[map_idx] > 0.0 {
                        let kernel_idx = ((dy + kernel_radius) as usize) * kernel_size + ((dx + kernel_radius) as usize);
                        point_score += kernel_obj.kernel[kernel_idx];
                    }
                }
            }
        }
        total_score += point_score;
    }
    total_score
}

/// Optimizes the robot's pose using Differential Evolution.
fn optimize_de(
    gmap: &OccupancyGrid,
    points: &[Point2<f32>],
    kernel_obj: &GaussianKernel,
    initial_pose: Isometry2<f32>,
) -> (Isometry2<f32>, f64) {
    // DE Parameters
    const WXY: f32 = 0.8;                     // Search range for x, y [m]
    const WA: f32 = std::f32::consts::PI / 8.0; // Search range for angle [rad], 22.5 deg = PI / 8
    const POPULATION_SIZE: usize = 200;       // Increased for better search
    const GENERATIONS: usize = 100;           // Increased for better search
    const F: f32 = 0.5;                       // Mutation factor
    const CR: f32 = 0.2;                      // Crossover rate

    let mut rng = rand::thread_rng();

    // Extract initial pose parameters
    let initial_x = initial_pose.translation.x;
    let initial_y = initial_pose.translation.y;
    let initial_a = initial_pose.rotation.angle();

    // 1. Initialize population around initial_pose_params
    let mut population: Vec<Vector3<f32>> = Vec::with_capacity(POPULATION_SIZE);
    for _ in 0..POPULATION_SIZE {
        let x = rng.gen_range(-WXY..=WXY) + initial_x;
        let y = rng.gen_range(-WXY..=WXY) + initial_y;
        let a = rng.gen_range(-WA..=WA) + initial_a;
        population.push(Vector3::new(x, y, a));
    }

    // Evaluate initial population
    let mut scores: Vec<f64> = Vec::with_capacity(POPULATION_SIZE);
    for i in 0..POPULATION_SIZE {
        let current_x = population[i].x;
        let current_y = population[i].y;
        let current_a = population[i].z; // angle is in z component

        let rotation = Rotation2::new(current_a);
        let translation = Translation2::new(current_x, current_y);
        let pose = Isometry2::from_parts(translation, rotation.into());
        
        scores.push(gaussian_match_count(gmap, points, &pose, kernel_obj));
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

    // 2. Generation loop
    for _gen in 0..GENERATIONS {
        for i in 0..POPULATION_SIZE {
            // Mutation
            let mut candidates: Vec<usize> = (0..POPULATION_SIZE).filter(|&idx| idx != i).collect();
            candidates.shuffle(&mut rng);
            let r1 = candidates[0];
            let r2 = candidates[1];
            let r3 = candidates[2];
            
            let p_r1 = &population[r1];
            let p_r2 = &population[r2];
            let p_r3 = &population[r3];

            // v = p_r1 + F * (p_r2 - p_r3)
            let vx = p_r1.x + F * (p_r2.x - p_r3.x);
            let vy = p_r1.y + F * (p_r2.y - p_r3.y);
            
            // Angle mutation with wrap-around handling
            let ax1 = p_r1.z.cos();
            let ay1 = p_r1.z.sin();
            let ax2 = p_r2.z.cos();
            let ay2 = p_r2.z.sin();
            let ax3 = p_r3.z.cos();
            let ay3 = p_r3.z.sin();

            let vax = ax1 + F * (ax2 - ax3);
            let vay = ay1 + F * (ay2 - ay3);
            let va = vay.atan2(vax);

            // Crossover
            let mut trial_pose = population[i];
            let j_rand = rng.gen_range(0..3);
            
            if rng.gen::<f32>() < CR || j_rand == 0 { trial_pose.x = vx; }
            if rng.gen::<f32>() < CR || j_rand == 1 { trial_pose.y = vy; }
            if rng.gen::<f32>() < CR || j_rand == 2 { trial_pose.z = va; }

            // Selection
            let rotation_trial = Rotation2::new(trial_pose.z);
            let translation_trial = Translation2::new(trial_pose.x, trial_pose.y);
            let pose_trial = Isometry2::from_parts(translation_trial, rotation_trial.into());
            let eval_trial = gaussian_match_count(gmap, points, &pose_trial, kernel_obj);

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

    // Create final transformation matrix
    let x = best_pose_params.x;
    let y = best_pose_params.y;
    let a = best_pose_params.z;

    let final_rotation = Rotation2::new(a);
    let final_translation = Translation2::new(x, y);
    let best_transform = Isometry2::from_parts(final_translation, final_rotation.into());
    
    (best_transform, best_eval)
}
