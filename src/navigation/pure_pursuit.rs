use eframe::egui;
use nalgebra::{Isometry2, Point2, Vector2};

pub fn compute_command(
    current_pose: &Isometry2<f32>,
    trajectory: &[egui::Pos2],
    current_target: &mut Option<egui::Pos2>,
    lookahead_distance: f32,
    target_velocity: f32,
    goal_tolerance: f32,
) -> Option<(f32, f32)> {
    if trajectory.is_empty() {
        return None;
    }

    let robot_pos = Point2::new(current_pose.translation.x, current_pose.translation.y);
    let robot_yaw = current_pose.rotation.angle();

    // 1. Find the closest point on the trajectory
    let mut closest_idx = 0;
    let mut min_dist_sq = f32::MAX;

    for (i, p) in trajectory.iter().enumerate() {
        let tp = Point2::new(p.x, p.y);
        let dist_sq = (robot_pos - tp).norm_squared();
        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }

    // Check if we reached the goal (last point)
    let last_point = trajectory.last().unwrap();
    let dist_to_goal = (robot_pos - Point2::new(last_point.x, last_point.y)).norm();
    if dist_to_goal < goal_tolerance {
        // Stop
        return Some((0.0, 0.0));
    }

    // 2. Find the lookahead point
    // Search forward from the closest point
    let mut target_idx = closest_idx;
    for i in closest_idx..trajectory.len() {
        let p = trajectory[i];
        let tp = Point2::new(p.x, p.y);
        let dist = (robot_pos - tp).norm();
        if dist >= lookahead_distance {
            target_idx = i;
            break;
        }
        // If we reach the end and still haven't found a point far enough, use the last point
        target_idx = i;
    }

    let target_p = trajectory[target_idx];
    *current_target = Some(target_p);

    // 3. Compute control command (Pure Pursuit)
    // Transform target to robot local frame
    let target_world = Point2::new(target_p.x, target_p.y);

    // Vector to target
    let dx = target_world.x - robot_pos.x;
    let dy = target_world.y - robot_pos.y;

    // Angle to target in world frame
    let target_angle = dy.atan2(dx);

    // Angle difference (alpha) in robot frame
    let mut alpha = target_angle - robot_yaw;

    // Normalize alpha to [-pi, pi]
    while alpha > std::f32::consts::PI {
        alpha -= 2.0 * std::f32::consts::PI;
    }
    while alpha < -std::f32::consts::PI {
        alpha += 2.0 * std::f32::consts::PI;
    }

    // Curvature gamma = 2 * sin(alpha) / L
    // Angular velocity w = v * gamma

    // Lookahead distance L is actually the distance to the target point
    let l = (dx * dx + dy * dy).sqrt();

    // Prevent division by zero
    if l < 0.01 {
        return Some((0.0, 0.0));
    }

    let w = target_velocity * (2.0 * alpha.sin() / l);

    // Limit angular velocity? (handled by motor controller limits usually, but good to clamp here too)
    let w_clamped = w.clamp(-1.0, 1.0); // +/- 1.0 rad/s

    Some((target_velocity, w_clamped))
}
