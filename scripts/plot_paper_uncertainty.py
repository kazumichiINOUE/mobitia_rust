import os
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import tomllib
import glob
from matplotlib.patches import Ellipse
from PIL import Image
import matplotlib.ticker as ticker

def load_map(map_dir):
    map_img_path = os.path.join(map_dir, "occMap.png")
    map_info_path = os.path.join(map_dir, "map_info.toml")
    
    if not os.path.exists(map_img_path) or not os.path.exists(map_info_path):
        return None, None
        
    img = Image.open(map_img_path).convert('L')
    data = np.array(img)
    
    rgba = np.zeros((data.shape[0], data.shape[1], 4), dtype=np.uint8)
    rgba[:, :, 0] = rgba[:, :, 1] = rgba[:, :, 2] = data
    rgba[:, :, 3] = np.where(data < 200, 255, 0)
    
    with open(map_info_path, "rb") as f:
        info = tomllib.load(f)
    
    res = info['resolution']
    origin_x = info['origin'][0]
    origin_y = info['origin'][1]
    width, height = img.size
    extent = [origin_x, origin_x + width * res, origin_y - height * res, origin_y]
    
    return rgba, extent

def get_best_direction_data(x, y, aligned_dir):
    best_row = None
    min_dist = float('inf')
    for log_path in glob.glob(os.path.join(aligned_dir, "*/degeneracy_log.csv")):
        df_log = pd.read_csv(log_path)
        dist = np.sqrt((df_log['x'] - x)**2 + (df_log['y'] - y)**2)
        idx = dist.idxmin()
        if dist[idx] < min_dist:
            min_dist = dist[idx]
            best_row = df_log.iloc[idx]
    if min_dist < 0.5:
        return best_row
    return None

def main():
    parser = argparse.ArgumentParser(description="Generate a clean paper-ready uncertainty ellipse map.")
    parser.add_argument("--points", required=True, help="Path to final_verification_points.csv")
    parser.add_argument("--aligned_dir", required=True, help="Directory containing aligned runs")
    parser.add_argument("--output", default="paper_uncertainty_ellipses.pdf", help="Output PDF path")
    parser.add_argument("--scale", type=float, default=500.0, help="Scale factor for ellipses")
    args = parser.parse_args()

    df_points = pd.read_csv(args.points)
    
    # CRITICAL: Calculate range based on ALL trajectories to match the other map
    all_x, all_y = [], []
    map_dirs = glob.glob(os.path.join(args.aligned_dir, "slam_result_*"))
    for d in map_dirs:
        traj_path = os.path.join(d, "brute_force_trajectory.csv")
        if os.path.exists(traj_path):
            t = pd.read_csv(traj_path)
            all_x.extend(t['x'].tolist())
            all_y.extend(t['y'].tolist())
    
    best_map_dir = max(map_dirs, key=lambda d: len(pd.read_csv(os.path.join(d, "brute_force_trajectory.csv"))))
    img, extent = load_map(best_map_dir)
    
    fig, ax = plt.subplots(figsize=(12, 8))
    
    if img is not None:
        ax.imshow(img, extent=extent, alpha=0.8, origin='upper', zorder=1)
    
    for _, p in df_points.iterrows():
        color = 'red' if p['type'] == 'degenerate' else 'blue'
        row = get_best_direction_data(p['x'], p['y'], args.aligned_dir)
        if row is not None:
            min_ev = row['min_eigenvalue']
            max_ev = row['max_eigenvalue']
            angle_deg = np.degrees(row['min_ev_angle']) if 'min_ev_angle' in row else 0.0
            major = args.scale / np.sqrt(max(1.0, min_ev))
            minor = args.scale / np.sqrt(max(1.0, max_ev))
            ax.add_patch(Ellipse((p['x'], p['y']), width=major, height=minor, angle=angle_deg,
                          edgecolor=color, facecolor='none', linewidth=2.5, alpha=0.9, zorder=10))
            ax.scatter(p['x'], p['y'], c=color, marker='o', s=40, zorder=11)
            ax.text(p['x']+0.2, p['y']+0.2, f"{p['id']}", color=color, 
                    fontsize=14, fontweight='bold', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

    ax.set_xlabel("X [m]", fontsize=14)
    ax.set_ylabel("Y [m]", fontsize=14)
    ax.set_title("SLAM Estimation Uncertainty (Paper Plot)", fontsize=16)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', color='gray', alpha=0.3, which='both', zorder=0)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1.0))
    
    # MATCH THE RANGE EXACTLY (Including map extent for future-proofing)
    if all_x and all_y:
        margin = 2.0
        x_min, x_max = np.min(all_x) - margin, np.max(all_x) + margin
        y_min, y_max = np.min(all_y) - margin, np.max(all_y) + margin
        
        # If map extent is larger, include it to avoid clipping
        if img is not None:
            x_min = min(x_min, extent[0])
            x_max = max(x_max, extent[1])
            y_min = min(y_min, extent[2])
            y_max = max(y_max, extent[3])
            
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)

    plt.tight_layout()
    plt.savefig(args.output, dpi=300)
    plt.savefig(args.output.replace(".pdf", ".png"), dpi=300)
    print(f"Saved correctly scaled uncertainty map to: {args.output}")

if __name__ == "__main__":
    main()
