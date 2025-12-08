use nalgebra::{Isometry2, Point2, Vector2};
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
    kernel: &GaussianKernel,
) -> f64 {
    // TODO: Implement the scoring logic.
    0.0
}

/// Optimizes the robot's pose using Differential Evolution.
fn optimize_de(
    gmap: &OccupancyGrid,
    points: &[Point2<f32>],
    kernel: &GaussianKernel,
    initial_pose: Isometry2<f32>,
) -> (Isometry2<f32>, f64) {
    // TODO: Implement the full Differential Evolution algorithm.
    // For now, just return the initial pose.
    (initial_pose, 0.0)
}
