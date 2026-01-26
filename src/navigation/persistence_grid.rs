use std::collections::HashMap;
use nalgebra::{Isometry2, Point2};

/// A sparse grid that accumulates obstacle hits over time to filter out transient noise.
/// Uses global coordinates to maintain consistency as the robot moves.
pub struct LocalPersistenceGrid {
    resolution: f32,
    cells: HashMap<(i32, i32), i32>, // (grid_x, grid_y) -> counter
    
    // Parameters
    decay_amount: i32,
    hit_increment: i32,
    max_value: i32,
    threshold: i32,
    
    // Cache for visualization or debug
    pub active_cell_count: usize,
}

impl LocalPersistenceGrid {
    pub fn new(resolution: f32) -> Self {
        Self {
            resolution, // Recommended: 0.1 (10cm)
            cells: HashMap::new(),
            decay_amount: 1,
            hit_increment: 3,
            max_value: 10,
            threshold: 5,
            active_cell_count: 0,
        }
    }

    pub fn reset(&mut self) {
        self.cells.clear();
        self.active_cell_count = 0;
    }

    /// Updates the grid with new scan points and returns the filtered stable points.
    /// 
    /// - `scan_points`: List of (x, y) tuples in Robot Local Frame.
    /// - `robot_pose`: Current global pose of the robot (to map local points to global grid).
    /// 
    /// Returns: A vector of (x, y) points in Robot Local Frame representing stable obstacles.
    pub fn update(
        &mut self, 
        scan_points: &[(f32, f32, f32, f32, f32, f32, f32, f32)], // Full scan tuple
        robot_pose: &Isometry2<f32>
    ) -> Vec<(f32, f32, f32, f32)> { // (x, y, nx, ny) for EB
        
        // 1. Decay all existing cells
        // Remove cells that drop to 0 or below
        self.cells.retain(|_, val| {
            *val -= self.decay_amount;
            *val > 0
        });

        // 2. Add new points (in Global Frame)
        for p in scan_points {
            let local_x = p.0;
            let local_y = p.1;
            
            // Skip points too close (self-reflection) - handled by caller? 
            // Better to handle it here too or rely on filtered input.
            
            let local_p = Point2::new(local_x, local_y);
            let global_p = robot_pose * local_p;
            
            let gx = (global_p.x / self.resolution).floor() as i32;
            let gy = (global_p.y / self.resolution).floor() as i32;
            
            let counter = self.cells.entry((gx, gy)).or_insert(0);
            *counter = (*counter + self.hit_increment).min(self.max_value);
        }

        self.active_cell_count = self.cells.len();

        // 3. Extract filtered points (convert back to Local Frame for EB)
        let mut filtered_obstacles = Vec::with_capacity(self.cells.len());
        let inv_pose = robot_pose.inverse();

        // Iterate over cells and output those above threshold
        for ((gx, gy), val) in &self.cells {
            if *val >= self.threshold {
                let center_x = (*gx as f32 + 0.5) * self.resolution;
                let center_y = (*gy as f32 + 0.5) * self.resolution;
                let global_p = Point2::new(center_x, center_y);
                
                // Convert back to Local Frame for Elastic Band
                let local_p = inv_pose * global_p;
                
                // Only output points within relevant range (e.g., 5m)
                // EB doesn't need obstacles 20m away.
                if local_p.coords.norm_squared() < 25.0 {
                    // We don't have normals for grid cells easily.
                    // For EB, we can estimate normal pointing towards robot?
                    // Or just return 0.0. EB implementation might use normals.
                    // Checking elastic_band.rs... it expects (x, y, nx, ny).
                    // Let's use a dummy normal or vector to robot.
                    
                    let nx = -local_p.x;
                    let ny = -local_p.y;
                    let len = (nx*nx + ny*ny).sqrt();
                    let (nx, ny) = if len > 1e-3 { (nx/len, ny/len) } else { (0.0, 0.0) };

                    filtered_obstacles.push((local_p.x, local_p.y, nx, ny));
                }
            }
        }
        
        filtered_obstacles
    }
}
