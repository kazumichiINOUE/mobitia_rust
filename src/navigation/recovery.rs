use crate::config::NavConfig;
use nalgebra::{Isometry2, Point2, Vector2};
use tracing::info;

#[derive(Debug, Clone, PartialEq)]
pub enum RecoveryState {
    Idle,
    Backing { target_idx: usize },
}

pub struct RecoveryManager {
    state: RecoveryState,
    pub is_active: bool,
    log_counter: usize,
}

impl RecoveryManager {
    pub fn new() -> Self {
        Self {
            state: RecoveryState::Idle,
            is_active: false,
            log_counter: 0,
        }
    }

    pub fn reset(&mut self) {
        self.state = RecoveryState::Idle;
        self.is_active = false;
        self.log_counter = 0;
    }

    pub fn update(
        &mut self,
        current_pose: &Isometry2<f32>,
        scan: &[(f32, f32, f32, f32, f32, f32, f32, f32)], // x, y, r, theta...
        global_path: &[eframe::egui::Pos2],
        config: &NavConfig,
        robot_width: f32,
    ) -> Option<(f32, f32)> {
        if !config.recovery_enabled {
            self.reset();
            return None;
        }

        match self.state {
            RecoveryState::Idle => {
                // Check trigger condition: Obstacle too close in front of the robot width
                let mut min_dist = f32::MAX;
                let mut trigger_obs = (0.0, 0.0);
                let mut danger_point_count = 0;
                
                // Trigger: Check obstacles in the "corridor" defined by robot width
                // This protects the front corners which are outside a simple 45-degree cone.
                let safety_half_width = robot_width / 2.0 + 0.01; // 1cm margin (tighter for narrow spaces)

                for (x, y, ..) in scan {
                    if *x > 0.0 && y.abs() < safety_half_width {
                         let d = (x * x + y * y).sqrt();
                         if d < config.recovery_trigger_dist {
                             if d < min_dist {
                                 min_dist = d;
                                 trigger_obs = (*x, *y);
                             }
                             danger_point_count += 1;
                         }
                    }
                }

                if danger_point_count >= 3 {
                    info!(
                        "RECOVERY TRIGGERED: {} points < {:.3}m (Min: {:.3}m at x={:.3}, y={:.3}). Switching to Reverse Path Tracking.",
                        danger_point_count, config.recovery_trigger_dist, min_dist, trigger_obs.0, trigger_obs.1
                    );

                    // 1. Find closest point on global path
                    let current_pos_vec = Vector2::new(current_pose.translation.x, current_pose.translation.y);
                    let mut closest_idx = 0;
                    let mut min_path_dist_sq = f32::MAX;

                    for (i, p) in global_path.iter().enumerate() {
                        let d_sq = (p.x - current_pos_vec.x).powi(2) + (p.y - current_pos_vec.y).powi(2);
                        if d_sq < min_path_dist_sq {
                            min_path_dist_sq = d_sq;
                            closest_idx = i;
                        }
                    }

                    // 2. Look backwards along the path to find target
                    // We want a point 'recovery_target_dist' meters BEHIND current position
                    let mut target_idx = closest_idx;
                    let mut dist_accum = 0.0;

                    while target_idx > 0 {
                        let curr = global_path[target_idx];
                        let prev = global_path[target_idx - 1];
                        dist_accum += curr.distance(prev);

                        target_idx -= 1;
                        if dist_accum >= config.recovery_target_dist {
                            break;
                        }
                    }

                    info!(
                        "Recovery Target Index: {} (Current Closest: {}), Back Dist: {:.2}m",
                        target_idx, closest_idx, dist_accum
                    );
                    
                    self.state = RecoveryState::Backing { target_idx };
                    self.is_active = true;
                    
                    // Stop momentarily
                    return Some((0.0, 0.0));
                }
                None
            }
            RecoveryState::Backing { target_idx } => {
                // Check exit condition 2: Reached target waypoint
                if target_idx < global_path.len() {
                    let target_pos = global_path[target_idx];
                    let curr_x = current_pose.translation.x;
                    let curr_y = current_pose.translation.y;
                    let dx = target_pos.x - curr_x;
                    let dy = target_pos.y - curr_y;
                    let dist_to_target = (dx * dx + dy * dy).sqrt();

                    if dist_to_target < 0.1 {
                        info!("Recovery Finished: Reached target waypoint. Returning to Tracking.");
                        self.state = RecoveryState::Idle;
                        self.is_active = false;
                        return Some((0.0, 0.0));
                    }
                } else {
                     // Invalid index fallback
                     self.state = RecoveryState::Idle;
                     self.is_active = false;
                     return None;
                }

                // --- Control Logic: Reverse Path Tracking ---
                // We want to move BACKWARDS towards target.
                let target_pos = global_path[target_idx];
                let target_global = Point2::new(target_pos.x, target_pos.y);
                let target_local = current_pose.inverse() * target_global;

                // Control Rule:
                // To move BACKWARDS towards a point (x<0), we use y (lateral error) to steer.
                // If target is Left-Back (y>0), we need to Turn Right (Omega < 0) to swing tail to left.
                // If target is Right-Back (y<0), we need to Turn Left (Omega > 0) to swing tail to right.
                
                let kp = 2.0; 
                let omega_cmd = -kp * target_local.y;
                let max_w = 0.5;
                let final_omega = omega_cmd.clamp(-max_w, max_w);

                let v_cmd = -config.recovery_speed;

                // Safety Check: Side and Rear Collision Avoidance
                let side_margin = 0.05; // 5cm margin
                let safety_half_width = robot_width / 2.0 + side_margin;
                
                let mut danger_detected = false;
                let mut danger_min_dist = f32::MAX;
                let mut danger_obs = (0.0, 0.0);

                for (x, y, ..) in scan {
                    // Check area behind the front bumper (x < 0.2)
                    // We care about side collisions (y) and rear collisions (x)
                    if *x < 0.2 {
                        if y.abs() < safety_half_width {
                             // Check distance to be sure it's close enough to constitute a hazard
                             // We are moving backwards, so anything close in X or Y is dangerous
                             let dist = (x * x + y * y).sqrt();
                             // Stop if anything enters the 'safety box' and is within 30cm
                             if dist < 0.3 {
                                 danger_detected = true;
                                 danger_min_dist = dist;
                                 danger_obs = (*x, *y);
                                 break;
                             }
                        }
                    }
                }
                
                if danger_detected {
                    info!("Recovery Blocked: Obstacle detected at (x={:.3}, y={:.3}, dist={:.3}m). Stopping.", danger_obs.0, danger_obs.1, danger_min_dist);
                    // Stuck! Stop to prevent collision.
                    return Some((0.0, 0.0));
                }

                if self.log_counter % 60 == 0 {
                    info!("Recovery Backing: Target Dist {:.3}m, LatError {:.3}m, Cmd(v={:.2}, w={:.2})", 
                        target_local.x.abs(), target_local.y, v_cmd, final_omega);
                }
                self.log_counter += 1;

                Some((v_cmd, final_omega))
            }
        }
    }
}
