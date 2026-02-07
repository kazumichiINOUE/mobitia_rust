use crate::config::Config;
use crate::slam::{ScanData, SlamManager, OccupancyGrid};
use crate::slam::differential_evolution::DifferentialEvolutionSolver;
use crate::lidar::features::{compute_features, interpolate_lidar_scan};
use anyhow::Result;
use nalgebra::{Matrix3, SymmetricEigen};
use rayon::prelude::*;
use std::collections::HashMap;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

/// CLI引数から受け取る実験設定
pub struct ExperimentArgs {
    pub mode: String,
    pub input_dir: PathBuf,
    pub output_dir: PathBuf,
    pub anchors_path: Option<PathBuf>,
    pub step: usize,
}

fn load_anchors(path: &Path) -> Result<Vec<u128>> {
    let content = std::fs::read_to_string(path)?;
    let mut timestamps = Vec::new();
    for line in content.lines().skip(1) {
        let parts: Vec<&str> = line.split(',').collect();
        if let Some(ts_str) = parts.first() {
            if let Ok(ts) = ts_str.trim().parse::<u128>() {
                timestamps.push(ts);
            }
        }
    }
    timestamps.sort();
    Ok(timestamps)
}

pub fn run_experiment(args: ExperimentArgs) -> Result<()> {
    println!("Starting Offline SLAM Experiment...");
    println!("Mode: {}", args.mode);
    println!("Input Directory: {:?}", args.input_dir);
    println!("Output Directory: {:?}", args.output_dir);

    let mut target_timestamps = Vec::new();
    if let Some(ref anchors_path) = args.anchors_path {
        match load_anchors(anchors_path) {
            Ok(ts_list) => {
                println!("Loaded {} anchors from {:?}", ts_list.len(), anchors_path);
                target_timestamps = ts_list;
            }
            Err(e) => eprintln!("Warning: Failed to load anchors: {}", e),
        }
    }

    let mut config = Config::default();

    match args.mode.as_str() {
        "baseline" => {
            config.slam.position_score_weight = 1.0;
            config.slam.feature_score_weight = 0.0;
            config.slam.normal_alignment_score_weight = 0.0;
            config.slam.corner_score_weight = 0.0;
            config.slam.use_odometry_as_initial_guess = false;
        }
        "proposed" => {
            config.slam.position_score_weight = 0.1;
            config.slam.feature_score_weight = 0.4;
            config.slam.normal_alignment_score_weight = 0.5;
            config.slam.corner_score_weight = 0.3;
            config.slam.use_odometry_as_initial_guess = false;
        }
        "brute_force" => {
            config.slam.position_score_weight = 0.1;
            config.slam.feature_score_weight = 0.4;
            config.slam.normal_alignment_score_weight = 0.5;
            config.slam.corner_score_weight = 0.3;
            config.slam.use_odometry_as_initial_guess = false;
        }
        "brute_force_mapping_only" => {
            config.slam.position_score_weight = 0.1;
            config.slam.feature_score_weight = 0.4;
            config.slam.normal_alignment_score_weight = 0.5;
            config.slam.corner_score_weight = 0.3;
            config.slam.use_odometry_as_initial_guess = false;
        }
        "ground_truth" => {
            config.slam.position_score_weight = 0.1;
            config.slam.feature_score_weight = 0.4;
            config.slam.normal_alignment_score_weight = 0.5;
            config.slam.corner_score_weight = 0.3;
            config.slam.use_odometry_as_initial_guess = false;
            config.slam.wxy = 1.0;
            config.slam.wa_degrees = 45.0;
            config.slam.population_size = 400;
            config.slam.generations = 150;
        }
        _ => anyhow::bail!("Unknown mode: {}", args.mode),
    }

    config.slam.num_scans_per_submap = 10000;
    let slam_output_dir = args.output_dir.join("slam_output_temp");
    let mut slam_manager = SlamManager::new(slam_output_dir, config.slam.clone());

    let submaps_dir = args.input_dir.join("submaps");
    if !submaps_dir.exists() { anyhow::bail!("'submaps' not found."); }
    let mut submap_dirs: Vec<PathBuf> = std::fs::read_dir(&submaps_dir)?
        .filter_map(|e| e.ok()).map(|e| e.path())
        .filter(|p| p.is_dir() && p.file_name().unwrap().to_string_lossy().starts_with("submap_"))
        .collect();
    submap_dirs.sort();

    std::fs::create_dir_all(&args.output_dir)?;
    let output_csv_path = args.output_dir.join(format!("{}_trajectory.csv", args.mode));
    let mut csv_file = File::create(&output_csv_path)?;
    writeln!(csv_file, "timestamp,x,y,theta")?;

    let degeneracy_log_path = args.output_dir.join("degeneracy_log.csv");
    let mut degeneracy_log_file = if args.mode == "brute_force" {
        let mut f = File::create(&degeneracy_log_path)?;
        writeln!(f, "timestamp,x,y,theta,min_eigenvalue,max_eigenvalue,condition_number")?;
        Some(f)
    } else { None };

    // Load Trajectory for Mapping Only Mode
    let mut trajectory_map: HashMap<u128, nalgebra::Isometry2<f32>> = HashMap::new();
    if args.mode == "brute_force_mapping_only" {
        let mut traj_path = args.input_dir.join("brute_force_trajectory.csv");
        if !traj_path.exists() {
            traj_path = args.input_dir.join("results").join("brute_force").join("brute_force_trajectory.csv");
        }
        if traj_path.exists() {
            let content = std::fs::read_to_string(traj_path)?;
            for line in content.lines().skip(1) {
                let parts: Vec<&str> = line.split(',').collect();
                if parts.len() >= 4 {
                    if let (Ok(ts), Ok(x), Ok(y), Ok(th)) = (
                        parts[0].trim().parse::<u128>(),
                        parts[1].trim().parse::<f32>(),
                        parts[2].trim().parse::<f32>(),
                        parts[3].trim().parse::<f32>(),
                    ) {
                        trajectory_map.insert(ts, nalgebra::Isometry2::new(nalgebra::Vector2::new(x, y), th));
                    }
                }
            }
            println!("Loaded {} poses for mapping.", trajectory_map.len());
        }
    }

    let start_time = Instant::now();
    let mut total_scans = 0;
    let mut processed_anchors = std::collections::HashSet::new();

    for submap_dir in submap_dirs {
        let scans_json_path = submap_dir.join("scans.json");
        if !scans_json_path.exists() { continue; }
        let scan_data_list: Vec<ScanData> = serde_json::from_str(&std::fs::read_to_string(&scans_json_path)?)?;
        
        for scan_data in scan_data_list {
            if total_scans % args.step != 0 { total_scans += 1; continue; }

            let mut raw_scan: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)> = scan_data.scan_points.iter()
                .map(|p| (p.x, p.y, p.r, p.theta, 0.0, 0.0, 0.0, 0.0)).collect();
            let scan_with_features = compute_features(&raw_scan);
            let interpolated_scan = interpolate_lidar_scan(&scan_with_features, 0.05, 0.5, 0.02);

            if args.mode == "brute_force" {
                let current_pose = slam_manager.get_current_pose();
                let mut best_pose = current_pose;
                let mut max_score = -f64::INFINITY;

                if total_scans > 0 {
                    let center_x = current_pose.translation.x;
                    let center_y = current_pose.translation.y;
                    let center_a = current_pose.rotation.angle();
                    let (range_xy, step_xy, range_a, step_a) = (1.0, 0.01, 30.0f32.to_radians(), 0.5f32.to_radians());

                    let matching_scan: Vec<(nalgebra::Point2<f32>, f32, f32, f32)> = interpolated_scan.iter()
                        .map(|p| (nalgebra::Point2::new(p.0, p.1), p.4, p.5, p.6)).collect();
                    let raw_corner_points: Vec<(nalgebra::Point2<f32>, f32)> = scan_with_features.iter()
                        .filter(|p| p.7 > 0.5).map(|p| (nalgebra::Point2::new(p.0, p.1), p.7)).collect();

                    let mut capture_landscape = false;
                    for &ts in &target_timestamps {
                        if (ts as i128 - scan_data.timestamp as i128).abs() < 10000 && !processed_anchors.contains(&ts) {
                            capture_landscape = true; processed_anchors.insert(ts); break;
                        }
                    }
                    
                    let steps_a = ((range_a * 2.0f32) / step_a).round() as i32;
                    let steps_xy = ((range_xy * 2.0f32) / step_xy).round() as i32;
                    let ia_range: Vec<i32> = (-(steps_a/2)..=(steps_a/2)).collect();

                    let results: Vec<(f64, nalgebra::Isometry2<f32>, Vec<(f32, f32, f32, f64)>)> = ia_range.par_iter().map(|&ia| {
                        let a = center_a + (ia as f32) * step_a;
                        let rot = nalgebra::Rotation2::new(a);
                        let mut l_max = -f64::INFINITY;
                        let mut l_best = current_pose;
                        let mut l_data = Vec::new();
                        for iy in -(steps_xy/2)..=(steps_xy/2) {
                            let y = center_y + (iy as f32) * step_xy;
                            for ix in -(steps_xy/2)..=(steps_xy/2) {
                                let x = center_x + (ix as f32) * step_xy;
                                let p = nalgebra::Isometry2::from_parts(nalgebra::Translation2::new(x, y), rot.into());
                                let s = slam_manager.get_solver().calculate_score(slam_manager.get_grid(), &matching_scan, &raw_corner_points, &p);
                                if s > l_max { l_max = s; l_best = p; }
                                if capture_landscape { l_data.push((x, y, a, s)); }
                            }
                        }
                        (l_max, l_best, l_data)
                    }).collect();

                    let mut landscape_data = Vec::new();
                    for (s, p, l) in results {
                        if s > max_score { max_score = s; best_pose = p; }
                        if capture_landscape { landscape_data.extend(l); }
                    }
                    
                    if capture_landscape {
                        let best_a = best_pose.rotation.angle();
                        let l_path = args.output_dir.join(format!("landscape_{}_xy.csv", scan_data.timestamp));
                        if let Ok(mut f) = File::create(&l_path) {
                            writeln!(f, "x,y,score")?;
                            for (x, y, a, s) in landscape_data {
                                if (a - best_a).abs() < (step_a / 2.0) { writeln!(f, "{:.6},{:.6},{:.6}", x, y, s)?; }
                            }
                        }
                    }
                    
                    if let Some(ref mut log_file) = degeneracy_log_file {
                        let (min_ev, max_ev, cond_num) = compute_degeneracy_metrics(slam_manager.get_solver(), slam_manager.get_grid(), &matching_scan, &raw_corner_points, &best_pose);
                        writeln!(log_file, "{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}", scan_data.timestamp, best_pose.translation.x, best_pose.translation.y, best_pose.rotation.angle(), min_ev, max_ev, cond_num)?;
                    }
                } else {
                    best_pose = nalgebra::Isometry2::identity();
                }

                slam_manager.update_with_pose(&scan_with_features, best_pose, scan_data.timestamp);
                writeln!(csv_file, "{},{:.6},{:.6},{:.6}", scan_data.timestamp, best_pose.translation.x, best_pose.translation.y, best_pose.rotation.angle())?;
            } else if args.mode == "brute_force_mapping_only" {
                if let Some(pose) = trajectory_map.get(&scan_data.timestamp) {
                    slam_manager.update_with_pose(&scan_with_features, *pose, scan_data.timestamp);
                }
            } else {
                slam_manager.update(&scan_with_features, &interpolated_scan, scan_data.timestamp, None);
                let pose = slam_manager.get_current_pose();
                writeln!(csv_file, "{},{:.6},{:.6},{:.6}", scan_data.timestamp, pose.translation.x, pose.translation.y, pose.rotation.angle())?;
            }
            total_scans += 1;
        }
    }
    println!("Experiment Completed Successfully. Trajectory saved to: {:?}", output_csv_path);
    
    // Save Final Map
    save_occupancy_map(slam_manager.get_grid(), &config.slam, &args.output_dir)?;
    
    Ok(())
}

fn save_occupancy_map(grid: &OccupancyGrid, config: &crate::config::SlamConfig, output_dir: &Path) -> Result<()> {
    let width = grid.width as u32;
    let height = grid.height as u32;
    let mut img_buffer = image::ImageBuffer::new(width, height);

    // Find bounding box for cropping (skip gray/unknown areas)
    let mut min_x = width;
    let mut min_y = height;
    let mut max_x = 0;
    let mut max_y = 0;
    let mut has_content = false;

    for y in 0..height {
        for x in 0..width {
            let index = (y as usize) * grid.width + (x as usize);
            let log_odds = grid.data[index].log_odds;
            
            // Log-odds 0.0 is unknown (0.5 probability)
            if (log_odds - 0.0).abs() > 1e-6 {
                if x < min_x { min_x = x; }
                if y < min_y { min_y = y; }
                if x > max_x { max_x = x; }
                if y > max_y { max_y = y; }
                has_content = true;
            }

            let prob = 1.0 / (1.0 + (-log_odds).exp());
            let color_val = if prob > config.prob_occupied {
                0u8 // Occupied = Black
            } else if prob < config.prob_free {
                255u8 // Free = White
            } else {
                128u8 // Unknown = Gray
            };
            
            // Do not flip Y. Assuming grid data aligns with image coordinates or user prefers this orientation.
            img_buffer.put_pixel(x, y, image::Luma([color_val]));
        }
    }

    if !has_content {
        println!("Warning: Map is empty, saving blank image.");
        min_x = 0; min_y = 0; max_x = width - 1; max_y = height - 1;
    }

    // Crop image
    let crop_width = max_x - min_x + 1;
    let crop_height = max_y - min_y + 1;
    let cropped_img = image::imageops::crop_imm(&img_buffer, min_x, min_y, crop_width, crop_height).to_image();

    let map_path = output_dir.join("occMap.png");
    cropped_img.save(&map_path)?;
    println!("Saved map image to {:?}", map_path);

    // Save map info
    let initial_origin_x = -(width as f32 * config.csize) / 2.0;
    // initial_origin_y corresponds to the top of the grid (y=0) in world coords
    let initial_origin_y = (height as f32 * config.csize) / 2.0;
    
    // Calculate new origin (Top-Left of the cropped image)
    let final_origin_x = initial_origin_x + (min_x as f32 * config.csize);
    let final_origin_y = initial_origin_y - (min_y as f32 * config.csize);

    let info_content = format!(
        r#"image = "occMap.png"
resolution = {}
origin = [{:.6}, {:.6}, 0.0]
negate = 0
occupied_thresh = 0.65
free_thresh = 0.196
"#,
        config.csize, final_origin_x, final_origin_y
    );

    let info_path = output_dir.join("map_info.toml");
    std::fs::write(&info_path, info_content)?;
    println!("Saved map info to {:?}", info_path);

    Ok(())
}

fn compute_degeneracy_metrics(solver: &DifferentialEvolutionSolver, grid: &OccupancyGrid, scan: &Vec<(nalgebra::Point2<f32>, f32, f32, f32)>, corner_points: &Vec<(nalgebra::Point2<f32>, f32)>, center_pose: &nalgebra::Isometry2<f32>) -> (f64, f64, f64) {
    let (delta_xy, delta_th) = (0.005, 0.25f32.to_radians());
    let (cx, cy, cth) = (center_pose.translation.x, center_pose.translation.y, center_pose.rotation.angle());
    let mut hessian = Matrix3::zeros();
    let eval = |dx: f32, dy: f32, dth: f32| -> f64 {
        let p = nalgebra::Isometry2::from_parts(nalgebra::Translation2::new(cx + dx, cy + dy), nalgebra::Rotation2::new(cth + dth).into());
        solver.calculate_score(grid, scan, corner_points, &p)
    };
    let f_center = eval(0.0, 0.0, 0.0);
    let deltas = [delta_xy, delta_xy, delta_th];
    for i in 0..3 {
        let mut d = [0.0, 0.0, 0.0]; d[i] = deltas[i];
        hessian[(i, i)] = (eval(d[0], d[1], d[2]) - 2.0 * f_center + eval(-d[0], -d[1], -d[2])) / (deltas[i] as f64).powi(2);
    }
    for i in 0..3 {
        for j in (i + 1)..3 {
            let (mut di, mut dj) = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
            di[i] = deltas[i]; dj[j] = deltas[j];
            let val = (eval(di[0]+dj[0], di[1]+dj[1], di[2]+dj[2]) - eval(di[0]-dj[0], di[1]-dj[1], di[2]-dj[2]) - eval(-di[0]+dj[0], -di[1]+dj[1], -di[2]+dj[2]) + eval(-di[0]-dj[0], -di[1]-dj[1], -di[2]-dj[2])) / (4.0 * deltas[i] as f64 * deltas[j] as f64);
            hessian[(i, j)] = val; hessian[(j, i)] = val;
        }
    }
    let eigen = SymmetricEigen::new(hessian);
    let mut evs: Vec<f64> = eigen.eigenvalues.iter().map(|&v| v.abs()).collect();
    evs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let (min_ev, max_ev) = (evs[0], evs[2]);
    (min_ev, max_ev, if min_ev > 1e-9 { max_ev / min_ev } else { 1e9 })
}