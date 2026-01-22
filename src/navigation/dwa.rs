use crate::config::{DwaConfig, RobotConfig, SlamConfig};
use crate::slam::{OccupancyGrid};
use nalgebra::{Isometry2, Point2, Vector2};

pub struct DwaTrajectory {
    pub points: Vec<Point2<f32>>,
    pub is_safe: bool,
}

struct Candidate {
    v: f32,
    w: f32,
    heading_metric: f32,
    velocity_metric: f32,
}

pub struct DwaPlanner {
    config: DwaConfig,
    robot_config: RobotConfig,
    _slam_config: SlamConfig,
}

impl DwaPlanner {
    pub fn new(config: DwaConfig, robot_config: RobotConfig, slam_config: SlamConfig) -> Self {
        Self {
            config,
            robot_config,
            _slam_config: slam_config,
        }
    }

    /// Computes the optimal velocity command (v, w) using DWA.
    pub fn compute_command(
        &self,
        current_pose: &Isometry2<f32>,
        current_vel: (f32, f32),
        target_point: &Point2<f32>,
        map: Option<&OccupancyGrid>,
        map_origin: [f32; 3],
        map_info_res: f32,
        lidar_points: &[(f32, f32)],
    ) -> (f32, f32, Vec<DwaTrajectory>) {
        let (vc, wc) = current_vel;
        let dt = self.config.dt;

        let min_v = (vc - self.config.max_accel_v * dt).max(0.0);
        let max_v = (vc + self.config.max_accel_v * dt).min(self.config.max_speed_v);
        
        let min_w = (wc - self.config.max_accel_w * dt).max(-self.config.max_speed_w);
        let max_w = (wc + self.config.max_accel_w * dt).min(self.config.max_speed_w);

        let v_step = if self.config.v_samples > 0 { (max_v - min_v) / self.config.v_samples as f32 } else { 0.0 };
        let w_step = if self.config.w_samples > 0 { (max_w - min_w) / self.config.w_samples as f32 } else { 0.0 };

        let mut candidates = Vec::new();
        let mut all_trajectories = Vec::new();

        for i in 0..=self.config.v_samples {
            let v = min_v + i as f32 * v_step;
            for j in 0..=self.config.w_samples {
                let w = min_w + j as f32 * w_step;

                let (points, is_safe) = self.predict_trajectory(current_pose, v, w, map, map_origin, map_info_res, lidar_points);
                let traj = DwaTrajectory { points: points.clone(), is_safe };
                
                if is_safe {
                    // Calculate raw metrics
                    let dt_pred = self.config.predict_time;
                    let yaw = current_pose.rotation.angle();
                    let next_yaw = yaw + w * dt_pred;
                    let avg_yaw = (yaw + next_yaw) / 2.0;
                    let next_x = current_pose.translation.x + v * dt_pred * avg_yaw.cos();
                    let next_y = current_pose.translation.y + v * dt_pred * avg_yaw.sin();
                    
                    let dx = target_point.x - next_x;
                    let dy = target_point.y - next_y;
                    let target_heading = dy.atan2(dx);
                    let heading_diff = target_heading - next_yaw;
                    let heading_metric = heading_diff.cos(); // Range [-1, 1], higher is better

                    candidates.push(Candidate {
                        v,
                        w,
                        heading_metric,
                        velocity_metric: v,
                    });
                }
                all_trajectories.push(traj);
            }
        }

        if candidates.is_empty() {
             // If no safe trajectory, try to stop or slow down
            return (0.0, 0.0, all_trajectories);
        }

        // 1. Find Min/Max for Normalization
        let mut min_heading = f32::MAX;
        let mut max_heading = -f32::MAX;
        let mut min_vel = f32::MAX;
        let mut max_vel = -f32::MAX;

        for c in &candidates {
            if c.heading_metric < min_heading { min_heading = c.heading_metric; }
            if c.heading_metric > max_heading { max_heading = c.heading_metric; }
            if c.velocity_metric < min_vel { min_vel = c.velocity_metric; }
            if c.velocity_metric > max_vel { max_vel = c.velocity_metric; }
        }

        // 2. Evaluate and find best
        let w_heading = 2.0;
        let w_vel = 1.0;
        let mut best_score = -f32::MAX;
        let mut best_cmd = (0.0, 0.0);

        for c in &candidates {
            let norm_heading = if (max_heading - min_heading).abs() > 1e-5 {
                (c.heading_metric - min_heading) / (max_heading - min_heading)
            } else {
                1.0 // If all are same, give full score
            };

            let norm_vel = if (max_vel - min_vel).abs() > 1e-5 {
                (c.velocity_metric - min_vel) / (max_vel - min_vel)
            } else {
                1.0
            };

            let score = w_heading * norm_heading + w_vel * norm_vel;

            if score > best_score {
                best_score = score;
                best_cmd = (c.v, c.w);
            }
        }

        (best_cmd.0, best_cmd.1, all_trajectories)
    }

    fn predict_trajectory(
        &self,
        current_pose: &Isometry2<f32>,
        v: f32,
        w: f32,
        map: Option<&OccupancyGrid>,
        map_origin: [f32; 3],
        res: f32,
        lidar_points: &[(f32, f32)],
    ) -> (Vec<Point2<f32>>, bool) {
        let mut points = Vec::new();
        let mut pose = *current_pose;
        let sim_time = self.config.predict_time;
        let dt = self.config.dt;
        let steps = (sim_time / dt) as usize;
        let mut is_safe = true;

        for _ in 0..steps {
            let yaw = pose.rotation.angle();
            let next_yaw = yaw + w * dt;
            let avg_yaw = (yaw + next_yaw) / 2.0;
            let next_x = pose.translation.x + v * dt * avg_yaw.cos();
            let next_y = pose.translation.y + v * dt * avg_yaw.sin();
            pose = Isometry2::new(Vector2::new(next_x, next_y), next_yaw);

            points.push(Point2::new(next_x, next_y));

            if is_safe && self.check_footprint_collision(&pose, map, map_origin, res, lidar_points) {
                is_safe = false;
            }
        }
        (points, is_safe)
    }

    fn check_footprint_collision(
        &self,
        pose: &Isometry2<f32>,
        map: Option<&OccupancyGrid>,
        map_origin: [f32; 3],
        res: f32,
        lidar_points: &[(f32, f32)],
    ) -> bool {
        let half_w = self.robot_config.width / 2.0;
        let half_l = self.robot_config.length / 2.0;
        let margin = 0.05; 
        let hw = half_w + margin;
        let hl = half_l + margin;

        let corners_local = [
            Point2::new(hl, hw),
            Point2::new(hl, -hw),
            Point2::new(-hl, -hw),
            Point2::new(-hl, hw),
        ];

        let corners_world: Vec<Point2<f32>> = corners_local.iter().map(|p| pose * p).collect();

        if let Some(grid) = map {
            let num_corners = corners_world.len();
            for i in 0..num_corners {
                let p1 = corners_world[i];
                let p2 = corners_world[(i + 1) % num_corners];
                let dist = (p1 - p2).norm();
                let steps = (dist / res).ceil() as usize;
                
                for s in 0..=steps {
                    let t = s as f32 / steps as f32;
                    let p = p1 + (p2 - p1) * t;
                    
                    // Correct conversion using map origin (Top-Left)
                    let gx = ((p.x - map_origin[0]) / res).floor() as isize;
                    let gy = ((map_origin[1] - p.y) / res).floor() as isize;
                    
                    if gx >= 0 && gx < grid.width as isize && gy >= 0 && gy < grid.height as isize {
                        let idx = (gy as usize) * grid.width + (gx as usize);
                        if grid.data[idx].log_odds > 2.0 {
                            return true;
                        }
                    }
                }
            }
        }

        let inv_pose = pose.inverse();
        for lp_world in lidar_points {
            let lp_world_pt = Point2::new(lp_world.0, lp_world.1);
            let dist_sq = (pose.translation.vector - lp_world_pt.coords).norm_squared();
            let radius = (hl*hl + hw*hw).sqrt();
            if dist_sq > radius * radius {
                continue;
            }

            let lp_local = inv_pose * lp_world_pt;
            if lp_local.x.abs() <= hl && lp_local.y.abs() <= hw {
                return true;
            }
        }

        false
    }
}