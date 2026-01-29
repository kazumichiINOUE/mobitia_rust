use crate::config::Config;
use crate::slam::{ScanData, SlamManager};
use crate::lidar::features::{compute_features, interpolate_lidar_scan};
use anyhow::Result;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

/// CLI引数から受け取る実験設定
pub struct ExperimentArgs {
    pub mode: String,
    pub input_dir: PathBuf,
    pub output_dir: PathBuf,
}

pub fn run_experiment(args: ExperimentArgs) -> Result<()> {
    println!("Starting Offline SLAM Experiment...");
    println!("Mode: {}", args.mode);
    println!("Input Directory: {:?}", args.input_dir);
    println!("Output Directory: {:?}", args.output_dir);

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

    // 6. Processing Loop
    let start_time = Instant::now();
    let mut total_scans = 0;

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
            slam_manager.update(
                &scan_with_features,
                &interpolated_scan,
                scan_data.timestamp,
                None, // No odometry used in this experiment (Odom-less)
            );

            // Record Pose
            let pose = slam_manager.get_current_pose(); // getterが必要かも？直接アクセスできない場合
            writeln!(
                csv_file,
                "{},{:.6},{:.6},{:.6}",
                scan_data.timestamp,
                pose.translation.x,
                pose.translation.y,
                pose.rotation.angle()
            )?;

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
