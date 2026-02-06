use crate::config::Config;
use crate::slam::{ScanData, SlamManager, OccupancyGrid};
use crate::slam::differential_evolution::DifferentialEvolutionSolver;
use crate::lidar::features::{compute_features, interpolate_lidar_scan};
use anyhow::Result;
use nalgebra::{Matrix3, SymmetricEigen};
use rayon::prelude::*;
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
    pub step: usize, // New: Processing step
}

fn load_anchors(path: &Path) -> Result<Vec<u128>> {
    let content = std::fs::read_to_string(path)?;
    let mut timestamps = Vec::new();

    for line in content.lines().skip(1) { // Skip header
        // anchor_log.csv format can vary, but timestamp is usually the first column
        // e.g., 1770031867023,2026-02-02 20:31:07.026,...
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

    // Load anchors if provided
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

    // 1. Load default config
    // 実験ではデフォルト設定をベースに、モードに応じてパラメータを上書きします
    // 実際の運用では config.toml を読み込む形にしても良いですが、
    // ここでは実験条件をコードで固定して再現性を高めます。
    let mut config = Config::default();

    // 2. Configure based on mode
    match args.mode.as_str() {
        "baseline" => {
            // Baseline: 幾何学的特徴を使用しない (オドメトリレス、純粋な点群マッチングに近い設定)
            config.slam.position_score_weight = 1.0;
            config.slam.feature_score_weight = 0.0;
            config.slam.normal_alignment_score_weight = 0.0;
            config.slam.corner_score_weight = 0.0;
            config.slam.use_odometry_as_initial_guess = false;
            println!("Configured for BASELINE (Geometry features disabled)");
        }
        "proposed" => {
            // Proposed: 幾何学的特徴を使用する (現在のデフォルトに近い推奨設定)
            config.slam.position_score_weight = 0.1;
            config.slam.feature_score_weight = 0.4;
            config.slam.normal_alignment_score_weight = 0.5;
            config.slam.corner_score_weight = 0.3;
            config.slam.use_odometry_as_initial_guess = false;
            println!("Configured for PROPOSED (Geometry features enabled)");
        }
        "proposed_fast" => {
            // Proposed (Fast): 計算リソースを削減した設定
            config.slam.position_score_weight = 0.1;
            config.slam.feature_score_weight = 0.4;
            config.slam.normal_alignment_score_weight = 0.5;
            config.slam.corner_score_weight = 0.3;
            config.slam.use_odometry_as_initial_guess = false;
            
            // 計算量を削減 (200*100=20000 -> 100*50=5000) 約1/4
            config.slam.population_size = 100;
            config.slam.generations = 50;
            
            println!("Configured for PROPOSED FAST (Reduced computational cost, v2)");
        }
        "proposed_eco" => {
            // Proposed (Eco): FastとFast(old)の中間設定
            config.slam.position_score_weight = 0.1;
            config.slam.feature_score_weight = 0.4;
            config.slam.normal_alignment_score_weight = 0.5;
            config.slam.corner_score_weight = 0.3;
            config.slam.use_odometry_as_initial_guess = false;
            
            // 計算量を削減 (200*100=20000 -> 60*40=2400) 約1/8
            config.slam.population_size = 60;
            config.slam.generations = 40;
            
            println!("Configured for PROPOSED ECO (Even more reduced computational cost)");
        }
        "proposed_minimal" => {
            // Proposed (Minimal): 極限まで計算リソースを削減した設定 (v2)
            config.slam.position_score_weight = 0.1;
            config.slam.feature_score_weight = 0.4;
            config.slam.normal_alignment_score_weight = 0.5;
            config.slam.corner_score_weight = 0.3;
            config.slam.use_odometry_as_initial_guess = false;
            
            // 計算量を削減 (200*100=20000 -> 55*35=1925) 約1/10
            config.slam.population_size = 55;
            config.slam.generations = 35;
            
            println!("Configured for PROPOSED MINIMAL (Extreme reduced computational cost, v2)");
        }
        "brute_force" => {
            // Brute-force: グリッドサーチによる全探索設定
            // パラメータは proposed と同じ重みを使用
            config.slam.position_score_weight = 0.1;
            config.slam.feature_score_weight = 0.4;
            config.slam.normal_alignment_score_weight = 0.5;
            config.slam.corner_score_weight = 0.3;
            config.slam.use_odometry_as_initial_guess = false;
            println!("Configured for BRUTE FORCE (Deterministic grid search)");
        }
        "ground_truth" => {
             // Ground Truth generation: 高負荷だが高精度な設定 (広範囲探索)
            config.slam.position_score_weight = 0.1;
            config.slam.feature_score_weight = 0.4;
            config.slam.normal_alignment_score_weight = 0.5;
            config.slam.corner_score_weight = 0.3;
            config.slam.use_odometry_as_initial_guess = false;
            
            // 探索範囲を拡大 (計算時間は増えるが、真値に近い結果を得るため)
            config.slam.wxy = 1.0; // 1.0m
            config.slam.wa_degrees = 45.0; // 45度
            config.slam.population_size = 400; // 個体数倍増
            config.slam.generations = 150; // 世代数増加
            
            println!("Configured for GROUND TRUTH (High precision settings)");
        }
        _ => {
            anyhow::bail!("Unknown mode: {}. Use 'baseline', 'proposed', 'proposed_fast', 'proposed_eco', 'proposed_minimal', or 'ground_truth'.", args.mode);
        }
    }

    // ディスクへのサブマップ保存を無効化 (実験速度向上のため)
    // SlamManagerの実装によっては完全には無効化できないかもしれませんが、
    // 生成頻度を極端に下げることで実質的に無効化します。
    config.slam.num_scans_per_submap = 10000;

    // 3. Initialize SlamManager
    // 出力先は実験出力ディレクトリの下の一時フォルダなどを指定
    let slam_output_dir = args.output_dir.join("slam_output_temp");
    let mut slam_manager = SlamManager::new(slam_output_dir, config.slam.clone());

    // 4. Collect Scan Files
    // 入力ディレクトリ (例: slam_results/slam_result_20251110.../submaps) から
    // submap_XXX ディレクトリを探し、その中の scans.json を読み込みます。
    let submaps_dir = args.input_dir.join("submaps");
    if !submaps_dir.exists() {
        anyhow::bail!("'submaps' directory not found in input path: {:?}", args.input_dir);
    }

    let mut submap_dirs: Vec<PathBuf> = std::fs::read_dir(&submaps_dir)?
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| path.is_dir() && path.file_name().unwrap().to_string_lossy().starts_with("submap_"))
        .collect();

    // submap_000, submap_001... の順にソート
    submap_dirs.sort();

    if submap_dirs.is_empty() {
        anyhow::bail!("No submap directories found in {:?}", submaps_dir);
    }
    println!("Found {} submaps.", submap_dirs.len());

    // 5. Prepare Output CSV
    std::fs::create_dir_all(&args.output_dir)?;
    let output_csv_path = args.output_dir.join(format!("{}_trajectory.csv", args.mode));
    let mut csv_file = File::create(&output_csv_path)?;
    writeln!(csv_file, "timestamp,x,y,theta")?;

    // Degeneracy Log (only for brute_force)
    let degeneracy_log_path = args.output_dir.join("degeneracy_log.csv");
    let mut degeneracy_log_file = if args.mode == "brute_force" {
        let mut f = File::create(&degeneracy_log_path)?;
        writeln!(f, "timestamp,x,y,theta,min_eigenvalue,max_eigenvalue,condition_number")?;
        Some(f)
    } else {
        None
    };

    // 6. Processing Loop
    let start_time = Instant::now();
    let mut total_scans = 0;
    
    // Processed anchors to avoid duplicates
    let mut processed_anchors = std::collections::HashSet::new();

    for submap_dir in submap_dirs {
        let scans_json_path = submap_dir.join("scans.json");
        if !scans_json_path.exists() {
            eprintln!("Warning: scans.json not found in {:?}, skipping.", submap_dir);
            continue;
        }

        let file_content = std::fs::read_to_string(&scans_json_path)?;
        let scan_data_list: Vec<ScanData> = serde_json::from_str(&file_content)?;
        
        println!("Processing {:?} ({} scans)...", submap_dir.file_name().unwrap(), scan_data_list.len());

        for scan_data in scan_data_list {
            // Processing step logic
            if total_scans % args.step != 0 {
                total_scans += 1;
                continue;
            }

            // Reconstruct scan data for SlamManager
            // ScanPoint (x, y, r, theta) -> Raw Scan Vector
            let mut raw_scan: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)> = Vec::with_capacity(scan_data.scan_points.len());

            for p in scan_data.scan_points {
                 // ScanPoint has x, y, r, theta. 
                 // SlamManager expects (x, y, r, theta, edge_ness, nx, ny, corner_ness)
                 // We reset features to 0.0 because compute_features will recalculate them
                 // based on the raw (x,y) configuration.
                 raw_scan.push((p.x, p.y, p.r, p.theta, 0.0, 0.0, 0.0, 0.0));
            }

            // Compute Features (Recalculate features for the current configuration!)
            // 特にBaselineの場合は特徴量を使わない設定になっていますが、
            // データ構造としては必要なので計算自体は行い、SlamManager側で重み0で無視されます。
            let scan_with_features = compute_features(&raw_scan);

            // Interpolate
            // 補間パラメータは固定値を使用 (Configから取るようにしても良い)
            let interpolated_scan = interpolate_lidar_scan(
                &scan_with_features,
                0.05, // min_dist
                0.5,  // max_dist
                0.02, // interval
            );

            // Update SLAM
            if args.mode == "brute_force" {
                // Brute-force search: 決定論的グリッドサーチ
                let current_pose = slam_manager.get_current_pose();
                let mut best_pose = current_pose;
                let mut max_score = -f64::INFINITY;

                // 初回スキャンは探索せず、原点(0,0,0)に固定して地図構築を開始する
                if total_scans > 0 {
                    let center_x = current_pose.translation.x;
                    let center_y = current_pose.translation.y;
                    let center_a = current_pose.rotation.angle();

                    // 探索パラメータ (依頼通り)
                    let range_xy = 1.0; // ±1.0m
                    let step_xy = 0.02; // 2cm刻み
                    let range_a = 30.0f32.to_radians(); // ±30度
                    let step_a = 1.0f32.to_radians(); // 1度刻み

                    // マッチング用のスキャンデータ準備
                    let matching_scan: Vec<(nalgebra::Point2<f32>, f32, f32, f32)> = interpolated_scan
                        .iter()
                        .map(|p| (nalgebra::Point2::new(p.0, p.1), p.4, p.5, p.6))
                        .collect();
                    
                    let raw_corner_points: Vec<(nalgebra::Point2<f32>, f32)> = scan_with_features
                        .iter()
                        .filter(|p| p.7 > 0.5)
                        .map(|p| (nalgebra::Point2::new(p.0, p.1), p.7))
                        .collect();

                    let solver = slam_manager.get_solver();
                    let grid = slam_manager.get_grid();

                    // 3重ループ (角度 -> Y -> X)
                    let steps_a = (range_a * 2.0 / step_a as f32).round() as i32;
                    let steps_xy = (range_xy * 2.0 / step_xy as f32).round() as i32;

                    // --- Check if we need to capture landscape ---
                    let mut capture_landscape = false;
                    if !target_timestamps.is_empty() {
                        for &ts in &target_timestamps {
                            let diff = (ts as i128 - scan_data.timestamp as i128).abs();
                            if diff < 10000 && !processed_anchors.contains(&ts) { // 10000ms (10s) window to catch all anchors
                                capture_landscape = true;
                                processed_anchors.insert(ts);
                                println!("  [Anchor Hit] TS: {} matches Anchor: {}. Capturing Landscape...", scan_data.timestamp, ts);
                                break;
                            }
                        }
                    }
                    
                    // Range of indices for angle search
                    let ia_range: Vec<i32> = (-(steps_a/2)..=(steps_a/2)).collect();

                    // Parallel Search over angles
                    let results: Vec<(f64, nalgebra::Isometry2<f32>, Vec<(f32, f32, f32, f64)>)> = ia_range.par_iter().map(|&ia| {
                        let a = center_a + (ia as f32) * step_a;
                        let rot = nalgebra::Rotation2::new(a);
                        
                        let mut local_max_score = -f64::INFINITY;
                        let mut local_best_pose = current_pose;
                        let mut local_landscape_data = Vec::new();

                        for iy in -(steps_xy/2)..=(steps_xy/2) {
                            let y = center_y + (iy as f32) * step_xy;
                            
                            for ix in -(steps_xy/2)..=(steps_xy/2) {
                                let x = center_x + (ix as f32) * step_xy;
                                
                                let test_pose = nalgebra::Isometry2::from_parts(
                                    nalgebra::Translation2::new(x, y),
                                    rot.into()
                                );

                                let score = solver.calculate_score(grid, &matching_scan, &raw_corner_points, &test_pose);
                                
                                if score > local_max_score {
                                    local_max_score = score;
                                    local_best_pose = test_pose;
                                }
                                
                                if capture_landscape {
                                    local_landscape_data.push((x, y, a, score));
                                }
                            }
                        }
                        (local_max_score, local_best_pose, local_landscape_data)
                    }).collect();

                    // Reduce results from all threads
                    let mut landscape_data = Vec::new();
                    for (score, pose, l_data) in results {
                        if score > max_score {
                            max_score = score;
                            best_pose = pose;
                        }
                        if capture_landscape {
                            landscape_data.extend(l_data);
                        }
                    }
                    
                    // --- Save Landscape Data ---
                    if capture_landscape {
                        let best_a = best_pose.rotation.angle();
                        let landscape_path = args.output_dir.join(format!("landscape_{}_xy.csv", scan_data.timestamp));
                        if let Ok(mut f) = File::create(&landscape_path) {
                            writeln!(f, "x,y,score").unwrap();
                            // Filter data for the best angle (approx)
                            for (x, y, a, score) in landscape_data {
                                if (a - best_a).abs() < (step_a / 2.0) {
                                    writeln!(f, "{:.6},{:.6},{:.6}", x, y, score).unwrap();
                                }
                            }
                            println!("  Saved landscape to {:?}", landscape_path);
                        }
                    }
                    
                    // --- Degeneracy Analysis (Hessian Eigenvalues) ---
                    if let Some(ref mut log_file) = degeneracy_log_file {
                        let (min_ev, max_ev, cond_num) = compute_degeneracy_metrics(
                            slam_manager.get_solver(),
                            slam_manager.get_grid(),
                            &matching_scan,
                            &raw_corner_points,
                            &best_pose
                        );
                        
                        writeln!(
                            log_file,
                            "{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                            scan_data.timestamp,
                            best_pose.translation.x,
                            best_pose.translation.y,
                            best_pose.rotation.angle(),
                            min_ev,
                            max_ev,
                            cond_num
                        )?;
                    }
                } else {
                    // 初回スキャンの場合
                    println!("  [Scan 1] TS: {} | Initializing map at origin.", scan_data.timestamp);
                    best_pose = nalgebra::Isometry2::identity();
                    max_score = 0.0; // ダミー値
                }

                // 全探索で見つかった最適位置をセットし、地図更新を行う
                slam_manager.update_with_pose(
                    &scan_with_features,
                    best_pose,
                    scan_data.timestamp,
                );

                // 推定軌跡として記録
                let pose = best_pose;
                writeln!(
                    csv_file,
                    "{},{:.6},{:.6},{:.6}",
                    scan_data.timestamp,
                    pose.translation.x,
                    pose.translation.y,
                    pose.rotation.angle()
                )?;

                // Brute-force用の詳細進捗表示
                println!(
                    "  [Scan {}] TS: {} | Best Score: {:.4} | Pose: ({:.3}, {:.2}, {:.1} deg)",
                    total_scans + 1,
                    scan_data.timestamp,
                    max_score,
                    pose.translation.x,
                    pose.translation.y,
                    pose.rotation.angle().to_degrees()
                );
            } else {
                // 通常のSLAM更新 (Proposed, Baselineなど)
                slam_manager.update(
                    &scan_with_features,
                    &interpolated_scan,
                    scan_data.timestamp,
                    None, // No odometry used in this experiment (Odom-less)
                );

                // Record Pose
                let pose = slam_manager.get_current_pose();
                writeln!(
                    csv_file,
                    "{},{:.6},{:.6},{:.6}",
                    scan_data.timestamp,
                    pose.translation.x,
                    pose.translation.y,
                    pose.rotation.angle()
                )?;
            }

            total_scans += 1;
            if total_scans % 100 == 0 {
                print!("\rTotal scans processed: {}", total_scans);
                std::io::stdout().flush()?;
            }
        }
    }

    let duration = start_time.elapsed();
    println!("\n\nExperiment Completed Successfully.");
    println!("Mode: {}", args.mode);
    println!("Total Time: {:.2?}", duration);
    println!("Average Speed: {:.2} FPS", total_scans as f64 / duration.as_secs_f64());
    println!("Trajectory saved to: {:?}", output_csv_path);

    Ok(())
}

fn compute_degeneracy_metrics(
    solver: &DifferentialEvolutionSolver,
    grid: &OccupancyGrid,
    scan: &Vec<(nalgebra::Point2<f32>, f32, f32, f32)>,
    corner_points: &Vec<(nalgebra::Point2<f32>, f32)>,
    center_pose: &nalgebra::Isometry2<f32>,
) -> (f64, f64, f64) {
    // Parameters for numerical differentiation
    let delta_xy = 0.01; // 1cm
    let delta_th = 0.5f32.to_radians(); // 0.5 degrees

    let center_x = center_pose.translation.x;
    let center_y = center_pose.translation.y;
    let center_th = center_pose.rotation.angle();

    let mut hessian = Matrix3::zeros();

    // Helper to evaluate score at (dx, dy, dth) relative to center
    let eval = |dx: f32, dy: f32, dth: f32| -> f64 {
        let test_pose = nalgebra::Isometry2::from_parts(
            nalgebra::Translation2::new(center_x + dx, center_y + dy),
            nalgebra::Rotation2::new(center_th + dth).into()
        );
        solver.calculate_score(grid, scan, corner_points, &test_pose)
    };

    let f_center = eval(0.0, 0.0, 0.0);

    // Diagonal elements (2nd derivative)
    // H_ii = (f(x+h) - 2f(x) + f(x-h)) / h^2
    let deltas = [delta_xy, delta_xy, delta_th];
    
    for i in 0..3 {
        let mut d = [0.0, 0.0, 0.0];
        d[i] = deltas[i];
        
        let f_plus = eval(d[0], d[1], d[2]);
        let f_minus = eval(-d[0], -d[1], -d[2]);
        
        hessian[(i, i)] = (f_plus - 2.0 * f_center + f_minus) / (deltas[i] as f64).powi(2);
    }

    // Off-diagonal elements (Mixed partial derivative)
    // H_ij = (f(++ ) - f(+-) - f(-+) + f(--)) / 4h_i h_j
    for i in 0..3 {
        for j in (i + 1)..3 {
            let mut d_i = [0.0, 0.0, 0.0];
            d_i[i] = deltas[i];
            
            let mut d_j = [0.0, 0.0, 0.0];
            d_j[j] = deltas[j];

            let f_pp = eval(d_i[0] + d_j[0], d_i[1] + d_j[1], d_i[2] + d_j[2]);
            let f_pm = eval(d_i[0] - d_j[0], d_i[1] - d_j[1], d_i[2] - d_j[2]);
            let f_mp = eval(-d_i[0] + d_j[0], -d_i[1] + d_j[1], -d_i[2] + d_j[2]);
            let f_mm = eval(-d_i[0] - d_j[0], -d_i[1] - d_j[1], -d_i[2] - d_j[2]);

            let val = (f_pp - f_pm - f_mp + f_mm) / (4.0 * deltas[i] as f64 * deltas[j] as f64);
            hessian[(i, j)] = val;
            hessian[(j, i)] = val;
        }
    }

    // Eigenvalue decomposition
    // Since Hessian is symmetric, use SymmetricEigen
    let eigen = SymmetricEigen::new(hessian);
    let eigenvalues = eigen.eigenvalues;

    // Filter out very small values to avoid numerical instability
    let mut evs: Vec<f64> = eigenvalues.iter().map(|&v| v.abs()).collect();
    evs.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let min_ev = evs[0];
    let max_ev = evs[2];
    
    // Condition number
    let cond_num = if min_ev > 1e-9 { max_ev / min_ev } else { 1e9 }; // Cap at 1e9

    (min_ev, max_ev, cond_num)
}
