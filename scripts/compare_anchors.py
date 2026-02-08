import pandas as pd
import numpy as np
import argparse
import os
import matplotlib.pyplot as plt
import tomllib

def load_csv(path):
    if not os.path.exists(path):
        print(f"Error: File not found: {path}")
        return None
    
    # Read raw content to handle inconsistent headers
    df_raw = pd.read_csv(path, skipinitialspace=True, header=None, skiprows=1)
    
    # Check column count
    cols = df_raw.shape[1]
    if 'anchor_log' in path:
        if cols == 6:
            # Old format: timestamp, readable_time, x, y, theta, label
            df = df_raw.copy()
            df.columns = ['timestamp', 'datetime', 'x', 'y', 'theta', 'label']
        elif cols == 9:
            # New format: timestamp, readable_time, x, y, theta, label, x_true, y_true, theta_true
            df = df_raw.copy()
            df.columns = ['timestamp', 'datetime', 'x', 'y', 'theta', 'label', 'x_true', 'y_true', 'theta_true']
        else:
             # Fallback or standard CSV
             df = pd.read_csv(path, skipinitialspace=True)
    else:
        # Re-read with header for standard trajectory files
        df = pd.read_csv(path, skipinitialspace=True)
    
    # Ensure timestamp is numeric
    if 'timestamp' in df.columns:
        df['timestamp'] = pd.to_numeric(df['timestamp'], errors='coerce')
        df = df.dropna(subset=['timestamp'])
    return df

def find_nearest_pose(trajectory_df, target_timestamp, tolerance_ms=15000):
    """
    軌跡データの中から、指定されたタイムスタンプに最も近いポーズを見つける
    """
    # タイムスタンプの差分の絶対値を計算
    trajectory_df['diff'] = (trajectory_df['timestamp'] - target_timestamp).abs()
    
    # 最小差分の行を取得
    nearest_row = trajectory_df.loc[trajectory_df['diff'].idxmin()]
    
    if nearest_row['diff'] > tolerance_ms:
        return None 
        
    return nearest_row

def compute_rigid_transform(P, Q):
    """
    Computes the optimal rigid transformation (rotation R, translation t)
    that maps P to Q (minimize ||Q - (RP + t)||^2).
    P, Q: (N, 2) numpy arrays
    Returns: R (2x2), t (2,)
    """
    mu_p = P.mean(axis=0)
    mu_q = Q.mean(axis=0)
    P_centered = P - mu_p
    Q_centered = Q - mu_q
    
    H = P_centered.T @ Q_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
        
    t = mu_q - R @ mu_p
    return R, t

def apply_transform(points, R, t):
    """
    Applies rigid transform to points (N, 2).
    Returns (N, 2) aligned points.
    """
    return (R @ points.T).T + t

def plot_anchors_on_map(map_dir, anchors_df, output_dir, traj_df=None, aligned_anchors_df=None, aligned_traj_df=None):
    """
    アンカーポイントを地図上にプロットして保存する
    """
    toml_path = os.path.join(map_dir, "map_info.toml")
    png_path = os.path.join(map_dir, "occMap.png")
    
    if not os.path.exists(toml_path) or not os.path.exists(png_path):
        print(f"Warning: Map files not found in {map_dir}. Skipping plot.")
        return

    print(f"Plotting anchors on map: {png_path}")

    # Load map info
    with open(toml_path, "rb") as f:
        map_info = tomllib.load(f)
    
    resolution = map_info.get("resolution", 0.05)
    origin = map_info.get("origin", [0.0, 0.0, 0.0]) # [x, y, theta]
    origin_x, origin_y = origin[0], origin[1]
    
    # Load image
    img = plt.imread(png_path)
    height, width = img.shape[:2]
    
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Display image with origin='upper' (standard image coordinates)
    ax.imshow(img, cmap='gray', origin='upper') 
    
    def world_to_pixel(wx, wy):
        # Assuming origin is Top-Left (Min X, Max Y)
        px = (wx - origin_x) / resolution
        py = (origin_y - wy) / resolution
        return px, py

    # Plot Original Trajectory (Gray/Faint)
    if traj_df is not None:
        traj_px = []
        traj_py = []
        for _, row in traj_df.iterrows():
            px, py = world_to_pixel(row['x'], row['y'])
            traj_px.append(px)
            traj_py.append(py)
        style = 'k--' if aligned_traj_df is not None else 'b-'
        alpha = 0.3 if aligned_traj_df is not None else 0.5
        label = 'Original Trajectory' if aligned_traj_df is not None else 'Trajectory'
        ax.plot(traj_px, traj_py, style, linewidth=1, alpha=alpha, label=label)

    # Plot Aligned Trajectory (Blue/Solid)
    if aligned_traj_df is not None:
        traj_px = []
        traj_py = []
        for _, row in aligned_traj_df.iterrows():
            px, py = world_to_pixel(row['x'], row['y'])
            traj_px.append(px)
            traj_py.append(py)
        ax.plot(traj_px, traj_py, 'b-', linewidth=1.5, alpha=0.8, label='Aligned Trajectory')

    # Plot Anchors
    # If aligned, we assume anchors_df contains the TRUE positions (x_true, y_true) for validation,
    # or we plot the estimated positions.
    # Actually, if aligned, we want to show:
    # 1. True Anchor Positions (Green Circles) - The Target
    # 2. Aligned Estimated Positions (Red Crosses) - The Result
    
    # Check if we have true positions
    has_true = 'x_true' in anchors_df.columns and aligned_anchors_df is not None
    
    if has_true:
        # Plot True Anchors
        for i, (_, anchor) in enumerate(anchors_df.iterrows()):
            if anchor['x_true'] == 0 and anchor['y_true'] == 0: continue # Skip dummy
            px, py = world_to_pixel(anchor['x_true'], anchor['y_true'])
            ax.scatter(px, py, c='lime', marker='o', s=80, zorder=11, label='True Anchor' if i==0 else "")
            
        # Plot Aligned Estimates
        for i, (_, anchor) in enumerate(aligned_anchors_df.iterrows()):
            px, py = world_to_pixel(anchor['Traj X'], anchor['Traj Y']) # Already aligned in dataframe
            ax.scatter(px, py, c='red', marker='x', s=100, zorder=12, label='Aligned Est.' if i==0 else "")
            ax.text(px + 5, py - 5, f"@{i+1}", color='red', fontsize=10, fontweight='bold', zorder=12)
            
    else:
        # Standard Plot (No Alignment or No True Data)
        for i, (_, anchor) in enumerate(anchors_df.iterrows()):
            # Use estimated anchor position from log (not trajectory match yet, usually)
            # Or use matched trajectory position? 
            # In standard plot, we plot the 'x' 'y' from anchor_log (which is System Estimate at that time)
            px, py = world_to_pixel(anchor['x'], anchor['y'])
            simple_label = f"@{i+1}"
            ax.scatter(px, py, c='red', marker='x', s=100, zorder=10)
            ax.text(px + 5, py - 5, simple_label, color='red', fontsize=10, fontweight='bold', zorder=10)

    ax.legend()
    plt.axis('off') 
    
    output_path = os.path.join(output_dir, "anchor_map_plot.pdf")
    plt.savefig(output_path, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"Saved map plot to {output_path}")

def main():
    parser = argparse.ArgumentParser(description='Compare Anchor Log with Trajectory')
    parser.add_argument('--anchors', type=str, required=True, help='Path to anchor_log.csv')
    parser.add_argument('--trajectory', type=str, required=True, help='Path to trajectory CSV (e.g. brute_force)')
    parser.add_argument('--output', type=str, required=True, help='Output directory for the report')
    parser.add_argument('--map_dir', type=str, help='(Optional) Directory containing map_info.toml and occMap.png for plotting')
    parser.add_argument('--align', action='store_true', help='Perform trajectory alignment (Umeyama) to ground truth anchors')
    
    args = parser.parse_args()
    
    print(f"Loading Anchors: {args.anchors}")
    anchors_df = load_csv(args.anchors)
    
    print(f"Loading Trajectory: {args.trajectory}")
    traj_df = load_csv(args.trajectory)
    
    if anchors_df is None or traj_df is None:
        return

    # Ensure trajectory timestamps are numeric
    traj_df['timestamp'] = pd.to_numeric(traj_df['timestamp'], errors='coerce')
    traj_df = traj_df.dropna(subset=['timestamp'])

    results = []
    
    # Data for Alignment
    est_points_for_align = []
    true_points_for_align = []
    
    print(f"Matching timestamps (Tolerance: 15000ms)...") 
    for i, anchor in anchors_df.iterrows():
        ts = anchor['timestamp']
        label = anchor['label']
        
        # アンカー記録時の推定値 (Real-time System Estimate)
        anchor_x = anchor['x']
        anchor_y = anchor['y']
        
        # 軌跡データからの検索
        nearest = find_nearest_pose(traj_df, ts, tolerance_ms=15000)
        
        if nearest is not None:
            traj_x = nearest['x']
            traj_y = nearest['y']
            traj_ts = nearest['timestamp']
            diff_ms = nearest['diff']
            
            # True values if available
            true_x = anchor.get('x_true', 0.0)
            true_y = anchor.get('y_true', 0.0)
            has_true = (true_x != 0.0 or true_y != 0.0)

            # Collect for alignment if valid true data exists
            if args.align and has_true:
                est_points_for_align.append([traj_x, traj_y])
                true_points_for_align.append([true_x, true_y])

            # Temporary storage (will update if aligned)
            results.append({
                'Label': label,
                'Timestamp': ts,
                'TimeDiff[ms]': int(diff_ms),
                'Anchor X': anchor_x,
                'Anchor Y': anchor_y,
                'Traj X': traj_x, # This will be overwritten if aligned
                'Traj Y': traj_y, # This will be overwritten if aligned
                'True X': true_x if has_true else None,
                'True Y': true_y if has_true else None,
                'Has True': has_true
            })
    
    # --- Alignment Process ---
    aligned_traj_df = None
    aligned_results_df = None
    
    if args.align and len(est_points_for_align) >= 3:
        print(f"Performing alignment using {len(est_points_for_align)} points...")
        P = np.array(est_points_for_align)
        Q = np.array(true_points_for_align)
        
        R, t = compute_rigid_transform(P, Q)
        print(f"  Rotation:\n{R}")
        print(f"  Translation: {t}")
        
        # Apply transform to full trajectory
        traj_points = traj_df[['x', 'y']].values
        aligned_traj_points = apply_transform(traj_points, R, t)
        
        aligned_traj_df = traj_df.copy()
        aligned_traj_df['x'] = aligned_traj_points[:, 0]
        aligned_traj_df['y'] = aligned_traj_points[:, 1]
        
        # Update results with aligned coordinates and recalculate errors
        for res in results:
            if res['Has True']: # Calculate error against True
                # Transform the Traj point
                p_est = np.array([res['Traj X'], res['Traj Y']])
                p_aligned = (R @ p_est) + t
                
                res['Traj X'] = p_aligned[0]
                res['Traj Y'] = p_aligned[1]
                
                diff_x = res['Traj X'] - res['True X']
                diff_y = res['Traj Y'] - res['True Y']
                res['Dist Error'] = np.sqrt(diff_x**2 + diff_y**2)
                res['Diff X'] = diff_x
                res['Diff Y'] = diff_y
            else:
                # No true data, cannot evaluate error meaningfully after alignment
                res['Dist Error'] = np.nan 
    elif args.align:
        print("Warning: Not enough valid ground truth points for alignment (need >= 3). Skipping alignment.")
        # Fallback to standard error (Traj vs Anchor Log Estimate)
        for res in results:
             diff_x = res['Traj X'] - res['Anchor X']
             diff_y = res['Traj Y'] - res['Anchor Y']
             res['Dist Error'] = np.sqrt(diff_x**2 + diff_y**2)
             res['Diff X'] = diff_x
             res['Diff Y'] = diff_y
    else:
        # No alignment requested, standard comparison
        for res in results:
             diff_x = res['Traj X'] - res['Anchor X']
             diff_y = res['Traj Y'] - res['Anchor Y']
             res['Dist Error'] = np.sqrt(diff_x**2 + diff_y**2)
             res['Diff X'] = diff_x
             res['Diff Y'] = diff_y

    if not results:
        print("No matching data found.")
        return

    # 結果のDataFrame化
    res_df = pd.DataFrame(results)
    
    # Plotting
    if args.map_dir:
        plot_anchors_on_map(args.map_dir, anchors_df, args.output, traj_df, res_df if args.align else None, aligned_traj_df)

    # CSV出力
    csv_path = os.path.join(args.output, 'anchor_comparison.csv')
    res_df.to_csv(csv_path, index=False)
    print(f"Saved comparison CSV to {csv_path}")
    
    # Markdownレポート出力
    md_path = os.path.join(args.output, 'anchor_comparison.md')
    with open(md_path, 'w') as f:
        f.write("# Anchor vs Trajectory Comparison\n\n")
        f.write(f"- **Anchors**: `{args.anchors}`\n")
        f.write(f"- **Trajectory**: `{args.trajectory}`\n")
        if args.align and aligned_traj_df is not None:
             f.write("- **Alignment**: Enabled (Rigid Transform to Ground Truth)\n\n")
        else:
             f.write("- **Alignment**: Disabled (Raw Coordinates)\n\n")
        
        f.write("## Summary Statistics\n")
        f.write(f"- **Mean Distance Difference**: {res_df['Dist Error'].mean():.4f} m\n")
        f.write(f"- **Max Distance Difference**: {res_df['Dist Error'].max():.4f} m\n")
        if args.align:
             f.write(f"- **RMSE**: {np.sqrt((res_df['Dist Error']**2).mean()):.4f} m\n\n")
        else:
             f.write("\n")
        
        f.write("## Details\n")
        # 表形式で出力
        cols = ['Label', 'TimeDiff[ms]', 'Traj X', 'Traj Y', 'Dist Error']
        if args.align:
            cols = ['Label', 'True X', 'True Y', 'Traj X', 'Traj Y', 'Dist Error']
        
        f.write(res_df[cols].to_markdown(index=False, floatfmt=".4f"))
        f.write("\n")
        
    print(f"Saved comparison report to {md_path}")
    print("\n--- Summary ---")
    print(f"Mean Difference: {res_df['Dist Error'].mean():.4f} m")

if __name__ == "__main__":
    main()
