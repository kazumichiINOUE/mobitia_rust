import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import tomllib

def plot_degeneracy_on_map(csv_path, output_dir, map_dir=None):
    print(f"Processing {csv_path}...")
    df = pd.read_csv(csv_path)
    
    # Extract data
    x = df['x'].values
    y = df['y'].values
    min_ev = df['min_eigenvalue'].values
    cond_num = df['condition_number'].values
    
    # Determine plot bounds
    margin = 2.0
    xlim = [x.min() - margin, x.max() + margin]
    ylim = [y.min() - margin, y.max() + margin]

    # --- Extract Top 5 Degenerate & Stable Points (with Spatial Filtering) ---
    def get_top_n_filtered(indices, x_arr, y_arr, n=5, min_dist=2.0):
        selected_indices = []
        for idx in indices:
            if len(selected_indices) >= n:
                break
            
            # Check distance to already selected
            is_far_enough = True
            for sel_idx in selected_indices:
                dist = np.sqrt((x_arr[idx] - x_arr[sel_idx])**2 + (y_arr[idx] - y_arr[sel_idx])**2)
                if dist < min_dist:
                    is_far_enough = False
                    break
            
            if is_far_enough:
                selected_indices.append(idx)
        return selected_indices

    # Sort by min_eigenvalue
    sorted_idx_asc = np.argsort(min_ev) # Smallest first (Degenerate)
    sorted_idx_desc = np.argsort(min_ev)[::-1] # Largest first (Stable)
    
    # 1. Select Degenerate Points (Distance > 2.0m from each other)
    top_degenerate_idx = get_top_n_filtered(sorted_idx_asc, x, y, n=5, min_dist=2.0)
    
    # Sort by X coordinate for easier physical identification
    top_degenerate_idx = sorted(top_degenerate_idx, key=lambda idx: x[idx])
    
    # 2. Select Stable Points
    # Constraint A: Distance > 2.0m from other Stable points
    # Constraint B: Distance > 2.0m from ALL Degenerate points
    top_stable_idx = []
    for idx in sorted_idx_desc:
        if len(top_stable_idx) >= 5:
            break
        
        is_valid = True
        
        # Check dist to other Stable points
        for sel_idx in top_stable_idx:
            dist = np.sqrt((x[idx] - x[sel_idx])**2 + (y[idx] - y[sel_idx])**2)
            if dist < 2.0: # Constraint A (Increased to 2.0m)
                is_valid = False
                break
        
        if not is_valid: continue

        # Check dist to Degenerate points
        for deg_idx in top_degenerate_idx:
            dist = np.sqrt((x[idx] - x[deg_idx])**2 + (y[idx] - y[deg_idx])**2)
            if dist < 2.0: # Constraint B
                is_valid = False
                break
        
        if is_valid:
            top_stable_idx.append(idx)

    # Sort by X coordinate for easier physical identification
    top_stable_idx = sorted(top_stable_idx, key=lambda idx: x[idx])

    # Most Degenerate (Smallest EV)
    print("\n--- Top 5 Degenerate Locations (Smallest Min Eigenvalue, >2m apart) ---")
    for i, idx in enumerate(top_degenerate_idx):
        print(f"Rank {i+1}: TS={df['timestamp'][idx]}, Pos=({x[idx]:.2f}, {y[idx]:.2f}), MinEV={min_ev[idx]:.4e}, Cond={cond_num[idx]:.2f}")

    # Most Stable (Largest EV)
    print("\n--- Top 5 Stable Locations (Largest Min Eigenvalue, >2m apart) ---")
    for i, idx in enumerate(top_stable_idx):
        print(f"Rank {i+1}: TS={df['timestamp'][idx]}, Pos=({x[idx]:.2f}, {y[idx]:.2f}), MinEV={min_ev[idx]:.4e}, Cond={cond_num[idx]:.2f}")

    # --- Plot Selected Points Only ---
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Draw Map Background
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
                
                # Calculate extent for imshow
                # Standard Mobitia TOML: origin is world coordinates of the TOP-LEFT pixel
                map_extent = [
                    origin_x,
                    origin_x + width * resolution,
                    origin_y - height * resolution,
                    origin_y
                ]
                
                ax.imshow(img, cmap='gray', extent=map_extent, alpha=0.5, origin='upper')
                print(f"  Overlaying occupancy map from {toml_path}. Extent: {map_extent}")
            except Exception as e:
                print(f"  Warning: Failed to load map: {e}")
        else:
            print(f"  Warning: map_info.toml or occMap.png not found in {map_dir}")

    # Plot Degenerate (Red)
    deg_x = x[top_degenerate_idx]
    deg_y = y[top_degenerate_idx]
    ax.scatter(deg_x, deg_y, c='red', marker='x', s=150, linewidths=3, label='High Degeneracy (Small MinEV)')
    
    for i, idx in enumerate(top_degenerate_idx):
        ax.text(x[idx]+0.5, y[idx], f"D{i+1}", color='red', fontsize=12, fontweight='bold')

    # Plot Stable (Blue)
    stab_x = x[top_stable_idx]
    stab_y = y[top_stable_idx]
    ax.scatter(stab_x, stab_y, c='blue', marker='o', s=100, linewidths=2, facecolors='none', label='Stable (Large MinEV)')

    for i, idx in enumerate(top_stable_idx):
        ax.text(x[idx]+0.5, y[idx], f"S{i+1}", color='blue', fontsize=12, fontweight='bold')

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Degeneracy Analysis: Selected Verification Points')
    ax.legend(loc='upper right')
    
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_aspect('equal', adjustable='box')
    
    output_path = os.path.join(output_dir, "degeneracy_map_plot.pdf")
    plt.savefig(output_path, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"Saved Selected Points Plot to {output_path}")

    # --- 5. Line Plot (Metrics vs Position/Time) ---
    # Assuming the robot moves primarily along X or we use path distance
    # For simplicity, let's use X-coordinate as the metric for progress along the corridor
    # Alternatively, calculate cumulative distance
    dist = np.cumsum(np.sqrt(np.diff(x, prepend=x[0])**2 + np.diff(y, prepend=y[0])**2))
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # Min Eigenvalue
    ax1.plot(dist, min_ev, 'k-', linewidth=1, alpha=0.7, label='Min Eigenvalue')
    ax1.set_ylabel('Min Eigenvalue')
    ax1.set_title('Degeneracy Metrics along Trajectory')
    ax1.grid(True, alpha=0.3)
    
    # Highlight points
    ax1.scatter(dist[top_degenerate_idx], min_ev[top_degenerate_idx], c='red', marker='x', s=80, zorder=5, label='High Degeneracy')
    ax1.scatter(dist[top_stable_idx], min_ev[top_stable_idx], c='blue', marker='o', s=60, zorder=5, facecolors='none', label='Stable')
    ax1.legend(loc='upper right')

    # Condition Number
    ax2.plot(dist, cond_num, 'k-', linewidth=1, alpha=0.7, label='Condition Number')
    ax2.set_ylabel('Condition Number')
    ax2.set_xlabel('Cumulative Distance [m]')
    ax2.set_yscale('log')
    ax2.grid(True, alpha=0.3)
    
    # Highlight points
    ax2.scatter(dist[top_degenerate_idx], cond_num[top_degenerate_idx], c='red', marker='x', s=80, zorder=5)
    ax2.scatter(dist[top_stable_idx], cond_num[top_stable_idx], c='blue', marker='o', s=60, zorder=5, facecolors='none')
    
    output_path_line = os.path.join(output_dir, "degeneracy_line_plot.pdf")
    plt.savefig(output_path_line, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"Saved Line Plot to {output_path_line}")

    # --- 6. Generate Statistical Report (Markdown) ---
    report_path = os.path.join(output_dir, "degeneracy_summary.md")
    with open(report_path, "w") as f:
        f.write("# Degeneracy Analysis Report\n\n")
        
        f.write("## Overall Statistics\n")
        f.write(f"- **Total Points**: {len(df)}\n")
        f.write(f"- **Min Eigenvalue**: Mean={min_ev.mean():.4e}, Median={np.median(min_ev):.4e}, Min={min_ev.min():.4e}\n")
        f.write(f"- **Condition Number**: Mean={cond_num.mean():.2f}, Median={np.median(cond_num):.2f}, Max={cond_num.max():.2f}\n\n")
        
        f.write("## Selected Verification Points\n\n")
        
        f.write("### High Degeneracy Points (Risk Areas)\n")
        f.write("| ID | Timestamp | Position (x, y) | Min Eigenvalue | Max Eigenvalue | Condition Number |\n")
        f.write("|---|---|---|---|---|---|\n")
        for i, idx in enumerate(top_degenerate_idx):
            f.write(f"| D{i+1} | {df['timestamp'][idx]} | ({x[idx]:.2f}, {y[idx]:.2f}) | {min_ev[idx]:.4e} | {df['max_eigenvalue'][idx]:.4e} | {cond_num[idx]:.2f} |\n")
        f.write("\n")

        f.write("### Stable Points (Reference Areas)\n")
        f.write("| ID | Timestamp | Position (x, y) | Min Eigenvalue | Max Eigenvalue | Condition Number |\n")
        f.write("|---|---|---|---|---|---|\n")
        for i, idx in enumerate(top_stable_idx):
            f.write(f"| S{i+1} | {df['timestamp'][idx]} | ({x[idx]:.2f}, {y[idx]:.2f}) | {min_ev[idx]:.4e} | {df['max_eigenvalue'][idx]:.4e} | {cond_num[idx]:.2f} |\n")
            
    print(f"Saved Statistical Report to {report_path}")

    # --- 7. Save Verification Points CSV (for Landscape Plotting) ---
    csv_path = os.path.join(output_dir, "verification_points.csv")
    with open(csv_path, "w") as f:
        f.write("id,timestamp,x,y,min_eigenvalue,condition_number,type\n")
        
        # Write Degenerate Points
        for i, idx in enumerate(top_degenerate_idx):
            f.write(f"D{i+1},{df['timestamp'][idx]},{x[idx]},{y[idx]},{min_ev[idx]},{cond_num[idx]},degenerate\n")
            
        # Write Stable Points
        for i, idx in enumerate(top_stable_idx):
            f.write(f"S{i+1},{df['timestamp'][idx]},{x[idx]},{y[idx]},{min_ev[idx]},{cond_num[idx]},stable\n")
            
    print(f"Saved Verification Points CSV to {csv_path}")

def main():
    parser = argparse.ArgumentParser(description='Plot Degeneracy Metrics')
    parser.add_argument('input', help='Path to degeneracy_log.csv')
    parser.add_argument('--output', default='results', help='Output directory')
    parser.add_argument('--map_dir', help='Directory containing map_info.toml and occMap.png')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.output):
        os.makedirs(args.output)
        
    if os.path.exists(args.input):
        plot_degeneracy_on_map(args.input, args.output, args.map_dir)
    else:
        print(f"File not found: {args.input}")

if __name__ == "__main__":
    main()