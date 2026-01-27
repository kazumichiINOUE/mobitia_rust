use crate::config::ElasticBandConfig;
use eframe::egui;
use nalgebra::Point2;
use tracing::instrument;

pub struct ElasticBand {
    config: ElasticBandConfig,
}

impl ElasticBand {
    pub fn new(config: ElasticBandConfig) -> Self {
        Self { config }
    }

    /// Optimizes the trajectory using a stable spring-mass-damper like approach.
    /// Includes displacement limits to prevent divergence.
    #[instrument(skip(self, trajectory, obstacles, global_path))]
    pub fn optimize(
        &self,
        trajectory: &mut Vec<egui::Pos2>,
        robot_pos: egui::Pos2,
        obstacles: &[(f32, f32, f32, f32)], // x, y, nx, ny
        global_path: &[egui::Pos2],
    ) {
        if trajectory.len() < 3 {
            return;
        }

        let iterations = self.config.num_iterations.max(5);
        let sigma = self.config.obstacle_safety_dist * 0.5; // Gaussian sigma
        let _sigma_sq_2 = 2.0 * sigma * sigma;

        for _ in 0..iterations {
            if trajectory.len() < 3 {
                break;
            }

            // Recalculate active window every iteration because refine_trajectory changes length
            let mut closest_idx = 0;
            let mut min_dist_sq = f32::MAX;
            for (i, p) in trajectory.iter().enumerate() {
                let dx = p.x - robot_pos.x;
                let dy = p.y - robot_pos.y;
                let d2 = dx * dx + dy * dy;
                if d2 < min_dist_sq {
                    min_dist_sq = d2;
                    closest_idx = i;
                }
            }

            let start_idx = closest_idx.max(1);
            let end_idx = (closest_idx + 40).min(trajectory.len() - 1);

            if start_idx >= end_idx {
                break;
            }

            let mut total_len = 0.0;
            for i in start_idx..end_idx {
                let dist = trajectory[i].distance(trajectory[i - 1]);
                total_len += dist;
            }
            let count = (end_idx - start_idx) as f32;
            let rest_length = if count > 0.0 { total_len / count } else { 0.1 };

            let mut displacements = vec![egui::Vec2::ZERO; trajectory.len()];

            for i in start_idx..end_idx {
                let p = trajectory[i];
                let prev = trajectory[i - 1];
                let next = trajectory[i + 1];

                // --- 1. Internal Force ---
                let vec_prev = prev - p;
                let dist_prev = vec_prev.length();
                let force_prev = if dist_prev > 0.001 {
                    vec_prev.normalized()
                        * (dist_prev - rest_length)
                        * self.config.internal_force_gain
                } else {
                    egui::Vec2::ZERO
                };

                let vec_next = next - p;
                let dist_next = vec_next.length();
                let force_next = if dist_next > 0.001 {
                    vec_next.normalized()
                        * (dist_next - rest_length)
                        * self.config.internal_force_gain
                } else {
                    egui::Vec2::ZERO
                };

                let midpoint = prev + (next - prev) * 0.5;
                let force_smooth = (midpoint - p) * self.config.internal_force_gain;

                let internal_f = force_prev + force_next + force_smooth;

                // --- 2. External Force (Repulsion) ---
                let mut external_f = egui::Vec2::ZERO;
                let safety_sq = self.config.obstacle_safety_dist * self.config.obstacle_safety_dist;

                for obs in obstacles {
                    let obs_pos = egui::pos2(obs.0, obs.1);
                    let vec_diff = p - obs_pos;
                    let dist_sq = vec_diff.length_sq();

                    if dist_sq < safety_sq && dist_sq > 0.0001 {
                        let dist = dist_sq.sqrt();

                        // Force direction: Always push AWAY from the obstacle point.
                        // Using vec_diff directly is more stable than relying on normals
                        // which might have inconsistent orientations.
                        let force_dir = vec_diff / dist;

                        // Quadratic or Exponential decay for the force
                        // We use a combination that is strong near the obstacle but fades smoothly.
                        let weight = (1.0 - dist / self.config.obstacle_safety_dist).powi(2);

                        external_f += force_dir * weight * self.config.external_force_gain;
                    }
                }

                // --- 3. Global Path Attraction (Tether) ---
                let mut attraction_f = egui::Vec2::ZERO;
                if self.config.global_path_gain > 0.0 && !global_path.is_empty() {
                    let mut min_gp_dist_sq = f32::MAX;
                    let mut closest_gp = global_path[0];

                    // Find closest point on global path
                    // Optimization: We could search only a window, but for safety we search all for now.
                    // Assuming global_path is the reference trajectory.
                    for gp in global_path {
                        let dx = p.x - gp.x;
                        let dy = p.y - gp.y;
                        let d_sq = dx * dx + dy * dy;
                        if d_sq < min_gp_dist_sq {
                            min_gp_dist_sq = d_sq;
                            closest_gp = *gp;
                        }
                    }

                    // Only apply attraction if we are somewhat close to the track (e.g., within 2m)
                    // This prevents snapping to a completely different section of a looping track.
                    if min_gp_dist_sq < 4.0 {
                        attraction_f = (closest_gp - p) * self.config.global_path_gain;
                    }
                }

                displacements[i] = internal_f + external_f + attraction_f;
            }

            // --- 3. Apply with Limiter & Damping ---
            for i in start_idx..end_idx {
                let disp = displacements[i];
                let disp_len = disp.length();
                // Limit displacement per iteration to prevent jitter
                let max_disp = 0.02;
                let mut final_disp = if disp_len > max_disp {
                    disp * (max_disp / disp_len)
                } else {
                    disp
                };

                // Damping factor: Apply only a fraction of the force per iteration
                // to prevent overshooting and oscillations.
                final_disp *= 0.3;

                trajectory[i] += final_disp;
            }

            // --- 4. Refine (Add/Remove points) ---
            self.refine_trajectory(trajectory);
            // --- 5. Simplify (Remove points on straight lines) ---
            self.simplify_trajectory(trajectory);
        }
    }

    fn simplify_trajectory(&self, trajectory: &mut Vec<egui::Pos2>) {
        if trajectory.len() < 3 {
            return;
        }

        let mut i = 1;
        // Check up to the second to last point
        while i < trajectory.len() - 1 {
            let prev = trajectory[i - 1];
            let curr = trajectory[i];
            let next = trajectory[i + 1];

            let v1 = curr - prev;
            let v2 = next - curr;

            if v1.length_sq() < 1e-4 || v2.length_sq() < 1e-4 {
                i += 1;
                continue;
            }

            let v1_n = v1.normalized();
            let v2_n = v2.normalized();

            // Dot product > 0.999 means the angle is very small (straight line)
            // Also ensure we don't create gaps larger than max_spacing (0.5)
            let dist_new = prev.distance(next);
            let max_spacing = 0.5;

            if v1_n.dot(v2_n) > 0.999 && dist_new < max_spacing {
                trajectory.remove(i);
                // Do not increment i, so we check the new current point against the same prev
            } else {
                i += 1;
            }
        }
    }

    fn refine_trajectory(&self, trajectory: &mut Vec<egui::Pos2>) {
        let min_spacing = 0.1;
        let max_spacing = 0.5;

        let mut new_traj = Vec::with_capacity(trajectory.len());
        if trajectory.is_empty() {
            return;
        }
        new_traj.push(trajectory[0]);

        let mut last_p = trajectory[0];

        for i in 1..trajectory.len() {
            let p = trajectory[i];
            let dist = last_p.distance(p);

            if dist < min_spacing {
                // Skip (remove), UNLESS it is the very last point (goal)
                if i == trajectory.len() - 1 {
                    new_traj.push(p);
                }
                continue;
            } else if dist > max_spacing {
                // Interpolate
                let num_segments = (dist / max_spacing).ceil() as i32;
                for k in 1..num_segments {
                    let t = k as f32 / num_segments as f32;
                    let mid = last_p + (p - last_p) * t;
                    new_traj.push(mid);
                }
                new_traj.push(p);
                last_p = p;
            } else {
                new_traj.push(p);
                last_p = p;
            }
        }

        *trajectory = new_traj;
    }
}
