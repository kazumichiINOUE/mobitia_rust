pub fn compute_features(
    scan: &Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
) -> Vec<(f32, f32, f32, f32, f32, f32, f32, f32)> {
    let mut scan_with_features = scan.clone();
    let neighborhood_size = 5; // 片側5点、合計11点を近傍とする
    if scan.len() < (neighborhood_size * 2 + 1) {
        return scan_with_features;
    }

    let mut step_edge_candidates: Vec<(usize, f32)> = Vec::new();

    // First, compute edge_ness and normals for all points
    for i in neighborhood_size..(scan.len() - neighborhood_size) {
        let neighborhood: Vec<_> = (i - neighborhood_size..=i + neighborhood_size)
            .map(|j| nalgebra::Point2::new(scan[j].0, scan[j].1))
            .collect();
        let sum_vec: nalgebra::Vector2<f32> = neighborhood.iter().map(|p| p.coords).sum();
        let mean = nalgebra::Point2::from(sum_vec / (neighborhood.len() as f32));
        let mut covariance_matrix = nalgebra::Matrix2::<f32>::zeros();
        for point in &neighborhood {
            let centered_point = point - mean;
            covariance_matrix += centered_point * centered_point.transpose();
        }
        covariance_matrix /= neighborhood.len() as f32;
        let eigen = nalgebra::SymmetricEigen::new(covariance_matrix);
        let eigenvalues = eigen.eigenvalues;
        let eigenvectors = eigen.eigenvectors;
        let (lambda_1, lambda_2, normal_vector) = if eigenvalues[0] > eigenvalues[1] {
            (eigenvalues[0], eigenvalues[1], eigenvectors.column(1))
        } else {
            (eigenvalues[1], eigenvalues[0], eigenvectors.column(0))
        };
        let px = scan[i].0;
        let py = scan[i].1;
        let nx = normal_vector[0];
        let ny = normal_vector[1];
        let dot_product_for_direction = px * nx + py * ny;
        let (corrected_nx, corrected_ny) = if dot_product_for_direction < 0.0 {
            (-nx, -ny)
        } else {
            (nx, ny)
        };
        let linearity = if lambda_1 > 1e-9 {
            (lambda_1 - lambda_2) / lambda_1
        } else {
            0.0
        };
        let sharpness = 10.0;
        let sensitivity = 0.7;
        let edge_ness = 1.0 - (1.0 / (1.0 + (-sharpness * (linearity - sensitivity)).exp()));

        scan_with_features[i].4 = edge_ness;
        scan_with_features[i].5 = corrected_nx;
        scan_with_features[i].6 = corrected_ny;
    }

    // Second, compute corner_ness using the calculated normals
    for i in neighborhood_size..(scan.len() - neighborhood_size) {
        let mut corner_ness = 0.0;
        let normals_before: Vec<_> = (i - neighborhood_size..i)
            .map(|j| nalgebra::Vector2::new(scan_with_features[j].5, scan_with_features[j].6))
            .collect();
        let normals_after: Vec<_> = (i + 1..=i + neighborhood_size)
            .map(|j| nalgebra::Vector2::new(scan_with_features[j].5, scan_with_features[j].6))
            .collect();

        if !normals_before.is_empty() && !normals_after.is_empty() {
            let normal_before_sum: nalgebra::Vector2<f32> = normals_before.iter().sum();
            let normal_after_sum: nalgebra::Vector2<f32> = normals_after.iter().sum();
            let normal_before = normal_before_sum.normalize();
            let normal_after = normal_after_sum.normalize();
            let dot_product_normals = normal_before.dot(&normal_after);

            let point_current = nalgebra::Point2::new(scan[i].0, scan[i].1);
            let point_before =
                nalgebra::Point2::new(scan[i - neighborhood_size].0, scan[i - neighborhood_size].1);
            let point_after =
                nalgebra::Point2::new(scan[i + neighborhood_size].0, scan[i + neighborhood_size].1);

            let dist_before = (point_current - point_before).norm();
            let dist_after = (point_current - point_after).norm();
            
            // --- Debug Prints ---
            println!("[Debug] i: {}, dot_product: {:.3}, dist_before: {:.3}, dist_after: {:.3}", i, dot_product_normals, dist_before, dist_after);

            corner_ness = 1.0 - dot_product_normals.abs();
            
            let min_dist_threshold = 0.5;
            if dist_before < min_dist_threshold || dist_after < min_dist_threshold {
                corner_ness = 0.0;
            }
        }
        // --- ステップエッジ検出ロジック ---
        // 次の点が存在する場合のみ評価
        // scan_with_features には既に最初のループで法線などが格納されているため、scanではなくscan_with_featuresを使用
        if i + 1 < scan_with_features.len() {
            let point_current_coords = nalgebra::Point2::new(scan_with_features[i].0, scan_with_features[i].1);
            let point_next_coords = nalgebra::Point2::new(scan_with_features[i + 1].0, scan_with_features[i + 1].1);
            let distance_to_next = (point_next_coords - point_current_coords).norm();

            // ステップエッジと判断する距離の閾値 (要調整)
            // 現在のLiDARスキャンが疎な場合にステップエッジとして検出されないよう、
            // 補間処理で使用されている max_dist_threshold を参考に閾値を設定
            // interpolate_lidar_scanのmax_dist_threshold: 2.0 (m)
            let step_edge_distance_threshold = 1.0; // 例えば1.0メートル以上離れていたら

            if distance_to_next > step_edge_distance_threshold {
                // この点がステップエッジの「手前側」の端点である可能性を評価
                let linearity_at_step_edge = calculate_one_sided_linearity(
                    &scan_with_features, // scan_with_features を使用
                    i,
                    neighborhood_size, // compute_features全体で使われているneighborhood_sizeを流用
                    true, // 手前側 (i-neighborhood_size から i まで) の点を評価
                );
                
                // 直線性が高い場合に候補として記録
                // この閾値はLiDARデータのノイズ特性や求めるコーナーのシャープさによって調整が必要
                let linearity_threshold_for_corner = 0.8; // 例えば0.8以上なら直線的と判断

                if linearity_at_step_edge > linearity_threshold_for_corner {
                    step_edge_candidates.push((i, linearity_at_step_edge));
                    // デバッグ用に println! を追加
                    // println!("[Debug] Step edge corner candidate detected at index {}, linearity: {:.3}", i, linearity_at_step_edge);
                }
            }
        }
        
        if corner_ness > 0.0 {
            println!("[Debug] Final corner_ness for point {}: {}", i, corner_ness);
        }
        scan_with_features[i].7 = corner_ness;
    }

    // ステップ2: ステップエッジ候補に基づいてcorner_nessを更新
    for (idx, linearity_val) in step_edge_candidates {
        // linearity_valは既にlinearity_threshold_for_cornerを超えているので、
        // そのまま高めのcorner_nessとして設定
        // ただし、既に高いcorner_nessが設定されている場合は、より高い方を採用する
        scan_with_features[idx].7 = scan_with_features[idx].7.max(linearity_val);
        // デバッグ用に println! を追加
        println!("[Debug] Updated corner_ness for step edge candidate at index {} to {:.3}", idx, scan_with_features[idx].7);
    }

    // Temporary debug print
    for (i, point) in scan_with_features.iter().enumerate() {
        if point.7 > 0.1 {
            println!("[features] corner_ness for point {}: {}", i, point.7);
        }
    }

    scan_with_features
}

/// 指定された点から片側のみを考慮した近傍点の直線性を計算する
fn calculate_one_sided_linearity(
    scan: &Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
    center_idx: usize,
    neighborhood_size: usize,
    use_before: bool, // trueなら手前側、falseなら後側
) -> f32 {
    let start_idx = if use_before {
        if center_idx < neighborhood_size { return 0.0; } // 範囲外
        center_idx - neighborhood_size
    } else {
        if center_idx + neighborhood_size >= scan.len() { return 0.0; } // 範囲外
        center_idx
    };
    let end_idx = if use_before {
        center_idx
    } else {
        center_idx + neighborhood_size
    };

    // 範囲チェックを強化
    if start_idx >= scan.len() || end_idx >= scan.len() || start_idx > end_idx {
        return 0.0;
    }

    let neighborhood: Vec<_> = (start_idx..=end_idx)
        .map(|j| nalgebra::Point2::new(scan[j].0, scan[j].1))
        .collect();

    if neighborhood.len() < 2 { // 2点未満では直線性を計算できない
        return 0.0;
    }

    let sum_vec: nalgebra::Vector2<f32> = neighborhood.iter().map(|p| p.coords).sum();
    let mean = nalgebra::Point2::from(sum_vec / (neighborhood.len() as f32));

    let mut covariance_matrix = nalgebra::Matrix2::<f32>::zeros();
    for point in &neighborhood {
        let centered_point = point - mean;
        covariance_matrix += centered_point * centered_point.transpose();
    }
    covariance_matrix /= neighborhood.len() as f32;

    let eigen = nalgebra::SymmetricEigen::new(covariance_matrix);
    let eigenvalues = eigen.eigenvalues;

    let lambda_1 = eigenvalues.max();
    let lambda_2 = eigenvalues.min();

    if lambda_1 > 1e-9 {
        (lambda_1 - lambda_2) / lambda_1
    } else {
        0.0
    }
}

/// 隣接点間距離が一定以上離れている場合に、線形補間して点を追加する
/// scan: 各点の (x, y, r, theta, edge, nx, ny, corner)
/// ...
pub fn interpolate_lidar_scan(
    scan: &Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
    min_dist_threshold: f32,
    max_dist_threshold: f32,
    interpolation_interval: f32,
) -> Vec<(f32, f32, f32, f32, f32, f32, f32, f32)> {
    if scan.is_empty() {
        return Vec::new();
    }

    let min_dist_threshold_sq = min_dist_threshold * min_dist_threshold;
    let max_dist_threshold_sq = max_dist_threshold * max_dist_threshold;

    let mut thinned_scan = Vec::new();
    thinned_scan.push(scan[0]);
    let mut last_point_thinned = scan[0];

    for i in 1..scan.len() {
        let current_point = scan[i];
        let dx = current_point.0 - last_point_thinned.0;
        let dy = current_point.1 - last_point_thinned.1;
        let distance_xy_sq = dx * dx + dy * dy;

        if distance_xy_sq >= min_dist_threshold_sq {
            thinned_scan.push(current_point);
            last_point_thinned = current_point;
        }
    }

    let mut final_scan = Vec::new();
    if thinned_scan.is_empty() {
        return final_scan;
    }

    final_scan.push(thinned_scan[0]);

    for window in thinned_scan.windows(2) {
        let p1 = window[0];
        let p2 = window[1];

        let dx = p2.0 - p1.0;
        let dy = p2.1 - p1.1;
        let distance_xy_sq = dx * dx + dy * dy;

        if distance_xy_sq < max_dist_threshold_sq {
            let distance_xy = distance_xy_sq.sqrt();
            let num_steps = (distance_xy / interpolation_interval).floor() as usize;
            if num_steps > 0 {
                for step in 1..=num_steps {
                    let fraction = step as f32 * interpolation_interval / distance_xy;
                    let interpolated_x = p1.0 + dx * fraction;
                    let interpolated_y = p1.1 + dy * fraction;
                    let interpolated_r = (interpolated_x.powi(2) + interpolated_y.powi(2)).sqrt();
                    let interpolated_theta = interpolated_y.atan2(interpolated_x);

                    let interpolated_edge_ness = p1.4 * (1.0 - fraction) + p2.4 * fraction;
                    let mut interpolated_nx = p1.5 * (1.0 - fraction) + p2.5 * fraction;
                    let mut interpolated_ny = p1.6 * (1.0 - fraction) + p2.6 * fraction;
                    let interpolated_corner_ness = p1.7 * (1.0 - fraction) + p2.7 * fraction;

                    let len = (interpolated_nx.powi(2) + interpolated_ny.powi(2)).sqrt();
                    if len > 1e-9 {
                        interpolated_nx /= len;
                        interpolated_ny /= len;
                    }

                    final_scan.push((
                        interpolated_x,
                        interpolated_y,
                        interpolated_r,
                        interpolated_theta,
                        interpolated_edge_ness,
                        interpolated_nx,
                        interpolated_ny,
                        interpolated_corner_ness,
                    ));
                }
            }
        }
        final_scan.push(p2);
    }

    final_scan
}
