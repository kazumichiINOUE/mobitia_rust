import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from pathlib import Path
import math
import tomllib

def load_trajectory(csv_path):
    """
    CSVファイルから軌跡データを読み込む
    Expected columns: timestamp, x, y, theta
    """
    if not os.path.exists(csv_path):
        return None
    return pd.read_csv(csv_path)

def align_trajectories(gt_df, est_df):
    """
    タイムスタンプに基づいて、Ground Truthと 推定軌跡を同期させる。
    """
    aligned_data = []
    
    # タイムスタンプでソートされていることを 前提とする
    gt_df = gt_df.sort_values('timestamp')
    est_df = est_df.sort_values('timestamp')
    
    gt_timestamps = gt_df['timestamp'].values
    
    for _, est_row in est_df.iterrows():
        ts = est_row['timestamp']
        
        # 最も近いタイムスタンプを持つGTのイ ンデックスを探す
        idx = np.searchsorted(gt_timestamps, ts)
        
        # 境界チェックと最近傍の選択
        if idx == 0:
            closest_idx = 0
        elif idx == len(gt_timestamps):
            closest_idx = len(gt_timestamps) - 1
        else:
            prev_ts = gt_timestamps[idx-1]
            next_ts = gt_timestamps[idx]
            if abs(ts - prev_ts) < abs(ts - next_ts):
                closest_idx = idx - 1
            else:
                closest_idx = idx
        
        gt_row = gt_df.iloc[closest_idx]
        
        # タイムスタンプのズレが大きすぎる場 合は除外 (例: 100ms以上)
        if abs(gt_row['timestamp'] - ts) > 100: 
            continue
            
        aligned_data.append({
            'timestamp': ts,
            'gt_x': gt_row['x'],
            'gt_y': gt_row['y'],
            'gt_theta': gt_row['theta'],
            'est_x': est_row['x'],
            'est_y': est_row['y'],
            'est_theta': est_row['theta']
        })
        
    return pd.DataFrame(aligned_data)

def calculate_errors(df):
    """
    同期されたデータフレームから誤差を計算す る
    """
    if df.empty:
        return df
    
    # 並進誤差 (Translational Error)
    df['trans_error'] = np.sqrt((df['est_x'] - df['gt_x'])**2 + (df['est_y'] - df['gt_y'])**2)
    
    # 回転誤差 (Rotational Error) [rad] -> [-pi, pi]  に正規化
    df['rot_error'] = df['est_theta'] - df['gt_theta']
    df['rot_error'] = np.arctan2(np.sin(df['rot_error']), np.cos(df['rot_error']))
    df['rot_error_deg'] = np.degrees(np.abs(df['rot_error']))
    
    # 進行方向(Longitudinal)と横方向(Lateral)への分解
    dx = df['est_x'] - df['gt_x']
    dy = df['est_y'] - df['gt_y']
    cos_theta = np.cos(df['gt_theta'])
    sin_theta = np.sin(df['gt_theta'])
    
    df['longitudinal_error'] =  dx * cos_theta + dy * sin_theta
    df['lateral_error'] = -dx * sin_theta + dy * cos_theta
    
    return df

def plot_trajectories(gt_df, baseline_df, proposed_df, proposed_fast_df, proposed_eco_df, proposed_minimal_df, output_dir, map_dir=None):
    plt.figure(figsize=(10, 10))
    
    # Collect all data points to determine limits
    all_x = gt_df['x'].tolist()
    all_y = gt_df['y'].tolist()

    # --- Overlay Map Background ---
    if map_dir:
        toml_path = os.path.join(map_dir, "map_info.toml")
        png_path = os.path.join(map_dir, "occMap.png")
        
        if os.path.exists(toml_path) and os.path.exists(png_path):
            try:
                with open(toml_path, "rb") as f:
                    map_info = tomllib.load(f)
                
                resolution = map_info.get("resolution", 0.05)
                origin = map_info.get("origin", [0.0, 0.0, 0.0]) # [x, y, theta]
                origin_x, origin_y = origin[0], origin[1]
                
                img = plt.imread(png_path)
                height, width = img.shape[:2]
                
                # Calculate extent for imshow (origin is Top-Left based on previous findings)
                map_extent = [
                    origin_x,
                    origin_x + width * resolution,
                    origin_y - height * resolution,
                    origin_y
                ]
                
                plt.imshow(img, cmap='gray', extent=map_extent, alpha=0.5, origin='upper', zorder=0)
                print("Overlaying occupancy map on trajectory plot.")
                
            except Exception as e:
                print(f"Warning: Failed to load map for trajectory plot: {e}")
    
    # Ground Truth: Thick gray line at the bottom
    plt.plot(gt_df['x'], gt_df['y'], color='gray', linestyle='-', label='Ground Truth (Brute-force)', linewidth=4, alpha=0.5, zorder=1)
    
    if baseline_df is not None:
        plt.plot(baseline_df['x'], baseline_df['y'], 'r--', label='Baseline', linewidth=1.5, alpha=0.9, zorder=2)
        all_x.extend(baseline_df['x'].tolist())
        all_y.extend(baseline_df['y'].tolist())
        
    if proposed_df is not None:
        plt.plot(proposed_df['x'], proposed_df['y'], 'b-', label='Proposed', linewidth=1.5, alpha=0.8, zorder=3)
        all_x.extend(proposed_df['x'].tolist())
        all_y.extend(proposed_df['y'].tolist())

    if proposed_fast_df is not None:
        plt.plot(proposed_fast_df['x'], proposed_fast_df['y'], color='cyan', linestyle='-', label='Proposed (Fast)', linewidth=1.2, alpha=0.9, zorder=4)
        all_x.extend(proposed_fast_df['x'].tolist())
        all_y.extend(proposed_fast_df['y'].tolist())

    if proposed_eco_df is not None:
        plt.plot(proposed_eco_df['x'], proposed_eco_df['y'], color='green', linestyle='-.', label='Proposed (Eco)', linewidth=1.2, alpha=0.9, zorder=5)
        all_x.extend(proposed_eco_df['x'].tolist())
        all_y.extend(proposed_eco_df['y'].tolist())

    if proposed_minimal_df is not None:
        plt.plot(proposed_minimal_df['x'], proposed_minimal_df['y'], color='magenta', linestyle=':', label='Proposed (Minimal)', linewidth=1.2, alpha=0.9, zorder=6)
        all_x.extend(proposed_minimal_df['x'].tolist())
        all_y.extend(proposed_minimal_df['y'].tolist())
        
    plt.title('Trajectory Comparison')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')

    # Set limits based on data with small margin
    margin = 1.0 # 1 meter margin
    if all_x and all_y:
        plt.xlim(min(all_x) - margin, max(all_x) + margin)
        # plt.ylim(min(all_y) - margin, max(all_y) + margin)
        plt.ylim(-10, 5) # Manual limit for better visualization

    plt.legend(loc='lower right')
    plt.grid(True)
    # Use set_aspect('equal', adjustable='box') to respect set_ylim while keeping 1:1 ratio
    plt.gca().set_aspect('equal', adjustable='box')
    
    output_path = os.path.join(output_dir, 'trajectory_comparison.pdf')
    plt.savefig(output_path, bbox_inches='tight', pad_inches=0)
    plt.close()

def plot_errors(baseline_errors, proposed_errors, proposed_fast_errors, proposed_eco_errors, proposed_minimal_errors, output_dir):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    
    if baseline_errors is not None:
        ax1.plot(baseline_errors['timestamp'], baseline_errors['trans_error'], 'r-', label='Baseline', alpha=0.7)
    if proposed_errors is not None:
        ax1.plot(proposed_errors['timestamp'], proposed_errors['trans_error'], 'b-', label='Proposed', alpha=0.7)
    if proposed_fast_errors is not None:
        ax1.plot(proposed_fast_errors['timestamp'], proposed_fast_errors['trans_error'], color='cyan', linestyle='-', label='Proposed (Fast)', alpha=0.7)
    if proposed_eco_errors is not None:
        ax1.plot(proposed_eco_errors['timestamp'], proposed_eco_errors['trans_error'], color='green', linestyle='-.', label='Proposed (Eco)', alpha=0.7)
    if proposed_minimal_errors is not None:
        ax1.plot(proposed_minimal_errors['timestamp'], proposed_minimal_errors['trans_error'], color='magenta', linestyle=':', label='Proposed (Minimal)', alpha=0.7)
        
    ax1.set_title('Translational Error over Time')
    ax1.set_ylabel('Error [m]')
    ax1.legend()
    ax1.grid(True)
    
    if baseline_errors is not None:
        ax2.plot(baseline_errors['timestamp'], baseline_errors['rot_error_deg'], 'r-', label='Baseline', alpha=0.7)
    if proposed_errors is not None:
        ax2.plot(proposed_errors['timestamp'], proposed_errors['rot_error_deg'], 'b-', label='Proposed', alpha=0.7)
    if proposed_fast_errors is not None:
        ax2.plot(proposed_fast_errors['timestamp'], proposed_fast_errors['rot_error_deg'], color='cyan', linestyle='-', label='Proposed (Fast)', alpha=0.7)
    if proposed_eco_errors is not None:
        ax2.plot(proposed_eco_errors['timestamp'], proposed_eco_errors['rot_error_deg'], color='green', linestyle='-.', label='Proposed (Eco)', alpha=0.7)
    if proposed_minimal_errors is not None:
        ax2.plot(proposed_minimal_errors['timestamp'], proposed_minimal_errors['rot_error_deg'], color='magenta', linestyle=':', label='Proposed (Minimal)', alpha=0.7)
        
    ax2.set_title('Rotational Error over Time')
    ax2.set_ylabel('Error [deg]')
    ax2.set_xlabel('Timestamp')
    ax2.legend()
    ax2.grid(True)
    
    output_path = os.path.join(output_dir, 'error_comparison.pdf')
    plt.savefig(output_path, bbox_inches='tight', pad_inches=0)
    plt.close()

def calculate_statistics(df, label):
    if df is None or df.empty:
        return f"## {label}\nNo data available.\n"
    
    stats = {
        "Mean Trans Error [m]": df['trans_error'].mean(),
        "Max Trans Error [m]": df['trans_error'].max(),
        "RMSE Trans Error [m]": np.sqrt((df['trans_error']**2).mean()),
        "Mean Rot Error [deg]": df['rot_error_deg'].mean(),
        "Max Rot Error [deg]": df['rot_error_deg'].max(),
        "Mean Lat Error [m]": df['lateral_error'].abs().mean(),
        "Mean Lon Error [m]": df['longitudinal_error'].abs().mean(),
    }
    
    report = f"## {label}\n"
    report += "| Metric | Value |\n"
    report += "|---|---|\n"
    for k, v in stats.items():
        report += f"| {k} | {v:.4f} |\n"
    report += "\n"
    return report

def main():
    parser = argparse.ArgumentParser(description='Analyze SLAM Experiment Results')
    parser.add_argument('--dir', type=str, required=True, help='Directory containing the result CSV files')
    parser.add_argument('--map_dir', type=str, help='Directory containing map_info.toml and occMap.png for plotting')
    args = parser.parse_args()
    
    output_dir = args.dir
    
    def find_file(mode):
        filename = f"{mode}_trajectory.csv"
        path = os.path.join(output_dir, filename)
        if os.path.exists(path): return path
        path = os.path.join(output_dir, mode, filename)
        if os.path.exists(path): return path
        return None

    # Priority: brute_force > ground_truth (DE)
    gt_path = find_file('brute_force')
    if gt_path is None:
        gt_path = find_file('ground_truth')
        print(f"Using DE-based Ground Truth: {gt_path}")
    else:
        print(f"Using Rigorous Brute-force Ground Truth: {gt_path}")

    if gt_path is None:
        print("Error: No Ground Truth (brute_force or ground_truth) found.")
        return

    gt_df = load_trajectory(gt_path)
    baseline_df = load_trajectory(find_file('baseline'))
    proposed_df = load_trajectory(find_file('proposed'))
    proposed_fast_df = load_trajectory(find_file('proposed_fast'))
    proposed_eco_df = load_trajectory(find_file('proposed_eco'))
    proposed_minimal_df = load_trajectory(find_file('proposed_minimal'))
    
    plot_trajectories(gt_df, baseline_df, proposed_df, proposed_fast_df, proposed_eco_df, proposed_minimal_df, output_dir, args.map_dir)
    
    baseline_errors = calculate_errors(align_trajectories(gt_df, baseline_df)) if baseline_df is not None else None
    proposed_errors = calculate_errors(align_trajectories(gt_df, proposed_df)) if proposed_df is not None else None
    proposed_fast_errors = calculate_errors(align_trajectories(gt_df, proposed_fast_df)) if proposed_fast_df is not None else None
    proposed_eco_errors = calculate_errors(align_trajectories(gt_df, proposed_eco_df)) if proposed_eco_df is not None else None
    proposed_minimal_errors = calculate_errors(align_trajectories(gt_df, proposed_minimal_df)) if proposed_minimal_df is not None else None
        
    plot_errors(baseline_errors, proposed_errors, proposed_fast_errors, proposed_eco_errors, proposed_minimal_errors, output_dir)
    
    report = "# SLAM Experiment Summary\n\n"
    report += calculate_statistics(baseline_errors, "Baseline")
    report += calculate_statistics(proposed_errors, "Proposed")
    report += calculate_statistics(proposed_fast_errors, "Proposed (Fast)")
    report += calculate_statistics(proposed_eco_errors, "Proposed (Eco)")
    report += calculate_statistics(proposed_minimal_errors, "Proposed (Minimal)")
    
    with open(os.path.join(output_dir, 'experiment_summary.md'), 'w') as f:
        f.write(report)
    print(f"Analysis complete. Results saved in {output_dir}")

if __name__ == "__main__":
    main()
