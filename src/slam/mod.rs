pub mod differential_evolution;

use nalgebra::{Isometry2, Point2};

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

/// The main SLAM state manager.
pub struct SlamManager {
    is_initial_scan: bool,
    map_gmap: OccupancyGrid,
    robot_pose: Isometry2<f32>,
    map_scans_in_world: Vec<Point2<f32>>,
    de_solver: differential_evolution::DifferentialEvolutionSolver,
}

impl SlamManager {
    pub fn new() -> Self {
        Self {
            is_initial_scan: true,
            map_gmap: OccupancyGrid::new(MAP_WIDTH, MAP_HEIGHT),
            robot_pose: Isometry2::identity(),
            map_scans_in_world: Vec::new(),
            de_solver: differential_evolution::DifferentialEvolutionSolver::new(),
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
            let (best_pose, _score) =
                self.de_solver
                    .optimize_de(&self.map_gmap, &current_scan, self.robot_pose);
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
