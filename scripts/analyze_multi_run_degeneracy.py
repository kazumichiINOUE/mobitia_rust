"""
1. ディレクトリ作成: プロジェクトルートに experiment_degeneracy_stats を作成
2. データ配置: 過去の実験結果ディレクトリ（slam_result_...）をその中に移動またはコピー
- ※複数回のデータがあればあるほど信頼性が向上します。
3. 実行: 
'''
python3 scripts/analyze_multi_run_degeneracy.py experiment_degeneracy_stats --output multi_run_results
'''
"""
import os
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import tomllib
import subprocess
from scipy.spatial.transform import Rotation as R
from matplotlib.ticker import FuncFormatter
from matplotlib.patches import Ellipse
from scipy.ndimage import generic_filter

def run_brute_force_analysis(run_dir):
    """
    Runs the brute_force experiment for the given directory to generate degeneracy_log.csv.
    """
    print(f"  Running brute_force analysis for: {run_dir} ...")
    
    # Define output directory inside the run directory
    output_dir = os.path.join(run_dir, "results", "brute_force")
    
    # Construct cargo command
    cmd = [
        "cargo", "run", "--release", "--bin", "mobitia", "--",
        "--experiment",
        "--mode", "brute_force",
        "--input", run_dir,
        "--output", output_dir
    ]
    
    try:
        # Run command and wait for completion
        subprocess.run(cmd, check=True)
        print("  Analysis complete.")
        return True
    except subprocess.CalledProcessError as e:
        print(f"  Error running analysis: {e}")
        return False

def umeyama_alignment(source_points, target_points):
    """
    Computes the rigid transformation (R, t) that aligns source_points to target_points
    using the Umeyama algorithm.
    Args:
        source_points: (N, 2) or (N, 3) array
        target_points: (N, 2) or (N, 3) array
    Returns:
        R: (2, 2) rotation matrix
        t: (2,) translation vector
        transformed_source: (N, 2) aligned points
    """
    # Check dimensions
    n, m = source_points.shape
    
    # Calculate centroids
    centroid_A = np.mean(source_points, axis=0)
    centroid_B = np.mean(target_points, axis=0)
    
    # Center the points
    AA = source_points - centroid_A
    BB = target_points - centroid_B
    
    # Covariance matrix
    H = np.dot(AA.T, BB)
    
    # SVD
    U, S, Vt = np.linalg.svd(H)
    
    # Rotation matrix
    Rot = np.dot(Vt.T, U.T)
    
    # Reflection case handling
    if np.linalg.det(Rot) < 0:
        Vt[m-1, :] *= -1
        Rot = np.dot(Vt.T, U.T)
        
    # Translation
    t = centroid_B - np.dot(Rot, centroid_A)
    
    # Transform source points
    transformed_source = np.dot(source_points, Rot.T) + t
    
    return Rot, t, transformed_source

def get_nearest_neighbors(src, dst):
    """
    Finds the nearest neighbor in dst for each point in src.
    Returns:
        distances: (N,) array
        indices: (N,) array of indices in dst
    """
    n = src.shape[0]
    indices = np.zeros(n, dtype=int)
    distances = np.zeros(n)
    
    # Simple brute-force NN (adequate for trajectory sizes < 10k)
    for i in range(n):
        diff = dst - src[i]
        d2 = np.sum(diff**2, axis=1)
        idx = np.argmin(d2)
        indices[i] = idx
        distances[i] = d2[idx]
        
    return np.sqrt(distances), indices

def icp_alignment(source_points, target_points, max_iterations=30, tolerance=1e-4):
    """
    Iterative Closest Point alignment using Umeyama algorithm at each step.
    """
    src = np.copy(source_points)
    dst = target_points
    
    # Initial alignment: Centroids
    centroid_src = np.mean(src, axis=0)
    centroid_dst = np.mean(dst, axis=0)
    t_init = centroid_dst - centroid_src
    src += t_init
    
    total_t = t_init
    
    prev_error = float('inf')
    
    for i in range(max_iterations):
        # 1. Find correspondence
        _, indices = get_nearest_neighbors(src, dst)
        matched_dst = dst[indices]
        
        # 2. Compute Transform
        R_step, t_step, _ = umeyama_alignment(src, matched_dst)
        
        # 3. Apply Transform
        src = np.dot(src, R_step.T) + t_step
        total_t += t_step # Approx accumulation for reporting
        
        # 4. Check convergence
        mean_error = np.mean(np.linalg.norm(src - matched_dst, axis=1))
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
        
    return None, total_t, src

def load_run_data(run_dir):
    """
    Loads degeneracy_log.csv from a run directory.
    Checks typical locations: direct, inside results/brute_force, etc.
    """
    possible_paths = [
        os.path.join(run_dir, "results", "brute_force", "degeneracy_log.csv"),
        os.path.join(run_dir, "brute_force", "degeneracy_log.csv"),
        os.path.join(run_dir, "degeneracy_log.csv"),
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            print(f"  Loaded: {path}")
            return pd.read_csv(path), path
            
    # If not found, try running analysis
    print(f"  Degeneracy log not found in {run_dir}. Attempting to run analysis...")
    if run_brute_force_analysis(run_dir):
        # Check specifically the output path we used
        expected_output = os.path.join(run_dir, "results", "brute_force", "degeneracy_log.csv")
        if os.path.exists(expected_output):
             print(f"  Loaded generated log: {expected_output}")
             return pd.read_csv(expected_output), expected_output
    
    print(f"  Warning: Failed to load or generate degeneracy_log.csv for {run_dir}")
    return None, None

def main():
    parser = argparse.ArgumentParser(description="Analyze degeneracy across multiple SLAM runs with alignment.")
    parser.add_argument("root_dir", help="Root directory containing run subdirectories (e.g. experiment_degeneracy_stats)")
    parser.add_argument("--grid_size", type=float, default=0.2, help="Grid size for statistical mapping [m]")
    parser.add_argument("--output", default="multi_run_results", help="Output directory for aggregated results")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.root_dir):
        print(f"Error: Directory {args.root_dir} not found.")
        return

    # 1. Find run directories
    run_dirs = sorted([d for d in glob.glob(os.path.join(args.root_dir, "*")) if os.path.isdir(d)])
    if not run_dirs:
        print("No subdirectories found.")
        return
    
    print(f"Found {len(run_dirs)} run directories.")
    
    # 2. Load data
    runs_data = [] # List of (run_name, dataframe)
    
    for d in run_dirs:
        run_name = os.path.basename(d)
        df, path = load_run_data(d)
        if df is not None:
            runs_data.append({'name': run_name, 'df': df, 'dir': d})
            
    if not runs_data:
        print("No valid data loaded.")
        return

    # 3. Select Reference Run (Longest trajectory)
    # Sort runs by length descending, so runs_data[0] is the longest
    runs_data.sort(key=lambda x: len(x['df']), reverse=True)
    
    ref_run = runs_data[0]
    ref_df = ref_run['df']
    ref_points = ref_df[['x', 'y']].values
    
    print(f"\nReference Run (Longest): {ref_run['name']} ({len(ref_points)} points)")
    
    aligned_runs = []
    # Add reference run first
    aligned_runs.append({
        'name': ref_run['name'],
        'x': ref_points[:, 0],
        'y': ref_points[:, 1],
        'min_ev': ref_df['min_eigenvalue'].values,
        'cond_num': ref_df['condition_number'].values,
        'timestamp': ref_df['timestamp'].values
    })
    
    for i in range(1, len(runs_data)):
        target_run = runs_data[i]
        tgt_points = target_run['df'][['x', 'y']].values
        
        # Use ICP for robust alignment regardless of point count or speed
        _, t_vec, aligned_tgt_points = icp_alignment(tgt_points, ref_points)
        
        print(f"Aligned {target_run['name']} to Reference using ICP. (Approx Offset: {t_vec})")
        
        aligned_runs.append({
            'name': target_run['name'],
            'x': aligned_tgt_points[:, 0],
            'y': aligned_tgt_points[:, 1],
            'min_ev': target_run['df']['min_eigenvalue'].values,
            'cond_num': target_run['df']['condition_number'].values,
            'timestamp': target_run['df']['timestamp'].values
        })

    # 4. Generate Statistical Map
    print("\nGenerating Statistical Map...")
    
    # Determine bounds
    all_x = np.concatenate([r['x'] for r in aligned_runs])
    all_y = np.concatenate([r['y'] for r in aligned_runs])
    
    min_x, max_x = np.min(all_x), np.max(all_x)
    min_y, max_y = np.min(all_y), np.max(all_y)
    
    margin = 2.0
    grid_res = args.grid_size
    
    x_edges = np.arange(min_x - margin, max_x + margin, grid_res)
    y_edges = np.arange(min_y - margin, max_y + margin, grid_res)
    
    # Accumulate stats
    # We want: Average Min Eigenvalue per cell
    
    # Prepare accumulation grids
    sum_ev_grid = np.zeros((len(y_edges)-1, len(x_edges)-1))
    count_grid = np.zeros((len(y_edges)-1, len(x_edges)-1))
    
    for r in aligned_runs:
        # Digitize coordinates
        x_idxs = np.digitize(r['x'], x_edges) - 1
        y_idxs = np.digitize(r['y'], y_edges) - 1
        
        # Filter out-of-bounds
        valid_mask = (x_idxs >= 0) & (x_idxs < len(x_edges)-1) & \
                     (y_idxs >= 0) & (y_idxs < len(y_edges)-1)
        
        valid_x = x_idxs[valid_mask]
        valid_y = y_idxs[valid_mask]
        valid_ev = r['min_ev'][valid_mask]
        
        # Add to grid
        # Note: This loop is slow in Python, but fine for typical trajectory sizes
        for i in range(len(valid_x)):
            sum_ev_grid[valid_y[i], valid_x[i]] += valid_ev[i]
            count_grid[valid_y[i], valid_x[i]] += 1
            
    # Calculate Average
    # Avoid division by zero
    with np.errstate(divide='ignore', invalid='ignore'):
        avg_ev_grid = sum_ev_grid / count_grid
        avg_ev_grid[count_grid == 0] = np.nan

    # Fill NaNs (Dilation/Smearing) to connect sparse trajectory points
    # This ensures BFS can travel along the trajectory even if there are small gaps
    for _ in range(3): # Dilate 3 times (~15cm with 0.05 grid)
        filled_grid = avg_ev_grid.copy()
        for y in range(avg_ev_grid.shape[0]):
            for x in range(avg_ev_grid.shape[1]):
                if np.isnan(avg_ev_grid[y, x]):
                    # Check 3x3 neighbors
                    y_min, y_max = max(0, y-1), min(avg_ev_grid.shape[0], y+2)
                    x_min, x_max = max(0, x-1), min(avg_ev_grid.shape[1], x+2)
                    
                    neighbors = avg_ev_grid[y_min:y_max, x_min:x_max]
                    # If there are valid neighbors, take the mean
                    if np.any(~np.isnan(neighbors)):
                        filled_grid[y, x] = np.nanmean(neighbors)
        avg_ev_grid = filled_grid

    # 5. Identify D/S Areas (Simple Thresholding)
    # Threshold could be median of the log values or fixed
    # Here we define High Degeneracy as < 10000 (example)
    degenerate_threshold = 10000.0
    stable_threshold = 100000.0
    
    # 6. Visualization
    if not os.path.exists(args.output):
        os.makedirs(args.output)
        
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Load Map from Reference Run if available
    map_dir = ref_run['dir']
    # Check typical locations for map info
    possible_map_dirs = [
        os.path.join(map_dir, "results", "brute_force"), # Usually no map here?
        map_dir # Often map is in root of run dir
    ]
    
    # Try to find a map source. 
    # Since brute_force might not produce a map, we might need to rely on the user 
    # providing a map or finding one in the run dir if it was copied.
    # We look for occMap.png + map_info.toml
    
    map_found = False
    # Recursive search for map in ref dir
    for root, _, files in os.walk(map_dir):
        if "occMap.png" in files and "map_info.toml" in files:
            try:
                toml_path = os.path.join(root, "map_info.toml")
                png_path = os.path.join(root, "occMap.png")
                
                with open(toml_path, "rb") as f:
                    map_info = tomllib.load(f)
                
                resolution = map_info.get("resolution", 0.05)
                origin = map_info.get("origin", [0.0, 0.0, 0.0])
                origin_x, origin_y = origin[0], origin[1]
                
                img = plt.imread(png_path)
                height, width = img.shape[:2]
                
                map_extent = [
                    origin_x,
                    origin_x + width * resolution,
                    origin_y - height * resolution,
                    origin_y
                ]
                
                ax.imshow(img, cmap='gray', extent=map_extent, alpha=0.5, origin='upper', zorder=0)
                print(f"Overlaying map from {root}")
                map_found = True
                break
            except Exception as e:
                print(f"Failed to load map: {e}")
                
    # Plot Heatmap of Average Eigenvalue
    # Use meshgrid for pcolormesh
    X, Y = np.meshgrid(x_edges, y_edges)
    
    # Plot using Log scale for better visibility of orders of magnitude
    # Handle NaNs and zeros for log
    plot_grid = avg_ev_grid.copy()
    plot_grid[plot_grid <= 0] = 1e-1 # Avoid log(0)
    
    def si_format(x, pos):
        if x >= 1e6:
            return f'{x*1e-6:.1f}M'
        elif x >= 1e3:
            return f'{x*1e-3:.0f}k'
        else:
            return f'{x:.0f}'

    cmap = plt.cm.jet_r # Red=Low EV (Degenerate), Blue=High EV (Stable)
    mesh = ax.pcolormesh(X, Y, plot_grid, cmap=cmap, alpha=0.6, shading='flat', zorder=1) #, norm=plt.LogNorm())
    cbar = plt.colorbar(mesh, label='Average Min Eigenvalue', shrink=0.6, format=FuncFormatter(si_format))
    
    # Plot Trajectories (Thin lines)
    for r in aligned_runs:
        ax.plot(r['x'], r['y'], 'k-', linewidth=0.5, alpha=0.3, zorder=2)
        
    ax.set_title(f'Statistical Degeneracy Map (Aggregated from {len(runs_data)} Runs)')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_aspect('equal')
    
    # 7. Output Suggested Verification Points
    # Find local minima (Degenerate) and local maxima (Stable) in the grid
    # For simplicity, just list grid cells with extreme values
    
    # Flatten and sort
    # Create a DataFrame of valid cells
    cells = []
    for y in range(avg_ev_grid.shape[0]):
        for x in range(avg_ev_grid.shape[1]):
            val = avg_ev_grid[y, x]
            if not np.isnan(val):
                cx = (x_edges[x] + x_edges[x+1]) / 2.0
                cy = (y_edges[y] + y_edges[y+1]) / 2.0
                cells.append({'x': cx, 'y': cy, 'avg_ev': val, 'count': count_grid[y, x]})
                
    df_cells = pd.DataFrame(cells)
    
    if not df_cells.empty:
        # Define Spatial NMS function
        def select_spatially_distinct_points(df, n=5, min_dist=2.0, maximize=False, forbidden_points=None):
            """
            Selects top n points from df based on 'avg_ev' that are at least min_dist apart.
            Also ensures points are at least min_dist away from forbidden_points.
            maximize=True for Stable (High EV), False for Degenerate (Low EV).
            """
            # Sort dataframe
            sorted_df = df.sort_values('avg_ev', ascending=not maximize).reset_index(drop=True)
            
            selected = []
            for _, row in sorted_df.iterrows():
                if len(selected) >= n:
                    break
                
                # Check distance to already selected
                is_far = True
                for s in selected:
                    dist = np.sqrt((row['x'] - s['x'])**2 + (row['y'] - s['y'])**2)
                    if dist < min_dist:
                        is_far = False
                        break
                
                # Check distance to forbidden points (e.g. Degenerate points when selecting Stable)
                if is_far and forbidden_points is not None:
                    for _, f_row in forbidden_points.iterrows():
                        dist = np.sqrt((row['x'] - f_row['x'])**2 + (row['y'] - f_row['y'])**2)
                        # Use a potentially different threshold for cross-category distance if needed
                        # For now, using the same min_dist (or we can hardcode 1.0m as requested)
                        if dist < 2.0: # Hardcoded 2.0m separation between D and S
                            is_far = False
                            break
                
                if is_far:
                    selected.append(row)
            
            return pd.DataFrame(selected)

        # Select Degenerate (Low EV)
        # Filter for somewhat degenerate first
        deg_candidates = df_cells[df_cells['avg_ev'] < stable_threshold]
        if deg_candidates.empty: deg_candidates = df_cells # Fallback
        
        top_degenerate = select_spatially_distinct_points(deg_candidates, n=5, min_dist=2.0, maximize=False)
        
        # Select Stable (High EV)
        # Filter for somewhat stable first
        stab_candidates = df_cells[df_cells['avg_ev'] > degenerate_threshold]
        if stab_candidates.empty: stab_candidates = df_cells # Fallback

        # Pass top_degenerate as forbidden points to ensure S is far from D
        top_stable = select_spatially_distinct_points(
            stab_candidates, 
            n=5, 
            min_dist=2.0, 
            maximize=True, 
            forbidden_points=top_degenerate
        )
        
        # Sort spatially (by X coordinate) for easier identification
        if not top_degenerate.empty:
            top_degenerate = top_degenerate.sort_values('x').reset_index(drop=True)
        if not top_stable.empty:
            top_stable = top_stable.sort_values('x').reset_index(drop=True)

        # Sort spatially (by X coordinate) for easier identification
        if not top_degenerate.empty:
            top_degenerate = top_degenerate.sort_values('x').reset_index(drop=True)
        if not top_stable.empty:
            top_stable = top_stable.sort_values('x').reset_index(drop=True)

        # Sort spatially (by X coordinate) for easier identification
        if not top_degenerate.empty:
            top_degenerate = top_degenerate.sort_values('x').reset_index(drop=True)
        if not top_stable.empty:
            top_stable = top_stable.sort_values('x').reset_index(drop=True)

        def calculate_validity_ellipse(cx, cy, grid, x_edges, y_edges, threshold, is_stable, max_dist=2.0):
            """
            Calculates the elliptical approximation of the validity region using BFS and covariance.
            Returns: (width, height, angle_deg)
            """
            # Find grid indices for center
            xi_start = np.digitize(cx, x_edges) - 1
            yi_start = np.digitize(cy, y_edges) - 1
            
            if xi_start < 0 or xi_start >= grid.shape[1] or yi_start < 0 or yi_start >= grid.shape[0] or np.isnan(grid[yi_start, xi_start]):
                return 0.2, 0.2, 0.0
            
            center_val = grid[yi_start, xi_start]
            
            # BFS initialization
            queue = [(xi_start, yi_start)]
            visited = set([(xi_start, yi_start)])
            valid_points = [] # List of (x, y) coordinates
            
            grid_res = x_edges[1] - x_edges[0]
            max_steps = int(max_dist / grid_res)
            
            while queue:
                xi, yi = queue.pop(0)
                
                # Convert grid index to physical coordinate
                px = (x_edges[xi] + x_edges[xi+1]) / 2.0
                py = (y_edges[yi] + y_edges[yi+1]) / 2.0
                valid_points.append([px, py])
                
                # Check neighbors (7x7 neighborhood to jump over larger gaps)
                for dx in range(-3, 4):
                    for dy in range(-3, 4):
                        if dx == 0 and dy == 0: continue
                        
                        nxi, nyi = xi + dx, yi + dy
                        
                        # Check distance limit (manhattan distance in grid steps for speed)
                        if abs(nxi - xi_start) > max_steps or abs(nyi - yi_start) > max_steps:
                            continue
                            
                        if (nxi, nyi) in visited:
                            continue
                        
                        if 0 <= nxi < grid.shape[1] and 0 <= nyi < grid.shape[0]:
                            val = grid[nyi, nxi]
                            if not np.isnan(val):
                                is_valid = False
                                if is_stable:
                                    if val > (center_val * 0.33): is_valid = True
                                else:
                                    # Degenerate: Relative (10x) OR Absolute threshold (20000)
                                    # This handles cases where center is extremely low (e.g. 100)
                                    if val < (center_val * 10.0) or val < 20000.0: is_valid = True
                                
                                if is_valid:
                                    visited.add((nxi, nyi))
                                    queue.append((nxi, nyi))

            # Fit Ellipse to valid_points
            points = np.array(valid_points)
            if len(points) < 5: # Too few points for covariance
                return 0.2, 0.2, 0.0
            
            # Compute Covariance Matrix
            cov = np.cov(points, rowvar=False)
            
            # Eigenvalues and Eigenvectors
            vals, vecs = np.linalg.eigh(cov)
            
            # Sort eigenvalues
            order = vals.argsort()[::-1]
            vals, vecs = vals[order], vecs[:, order]
            
            # Calculate angle
            theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
            
            # Width and Height (2 sigma = 95% confidence interval roughly covers the region)
            # Standard deviation is sqrt(eigenvalue)
            # Width (major axis) = 4 * std_dev (2 sigma radius * 2 for diameter)
            width = 4 * np.sqrt(vals[0])
            height = 4 * np.sqrt(vals[1])
            
            return width, height, theta

        # Calculate ellipses and save to CSV
        final_csv_path = os.path.join(args.output, "final_verification_points.csv")
        with open(final_csv_path, "w") as f:
            f.write("id,x,y,avg_min_eigenvalue,type,major_axis,minor_axis,angle_deg\n")
            
            # Plot Degenerate with Ellipse
            if not top_degenerate.empty:
                # Add dummy entry for legend
                ax.scatter([], [], c='red', marker='x', s=200, linewidths=3, label='Degenerate (D)')
                
                print("Degenerate Points Ellipses:")
                for i, row in top_degenerate.iterrows():
                    w, h, angle = calculate_validity_ellipse(row['x'], row['y'], avg_ev_grid, x_edges, y_edges, degenerate_threshold, False)
                    print(f"  D{i+1}: {w:.2f}m x {h:.2f}m @ {angle:.1f}deg")
                    f.write(f"D{i+1},{row['x']:.4f},{row['y']:.4f},{row['avg_ev']:.4f},degenerate,{w:.4f},{h:.4f},{angle:.2f}\n")
                    
                    # Draw Ellipse
                    ellipse = Ellipse((row['x'], row['y']), width=w, height=h, angle=angle, color='red', alpha=0.3, zorder=9)
                    ax.add_patch(ellipse)
                    
                    ax.scatter(row['x'], row['y'], c='red', marker='x', s=100, linewidths=2, zorder=10)
                    ax.text(row['x'], row['y']+0.5, f"D{i+1}", color='red', fontsize=12, fontweight='bold', zorder=11)

            # Plot Stable with Ellipse
            if not top_stable.empty:
                # Add dummy entry for legend
                ax.scatter([], [], c='blue', marker='o', s=150, linewidths=2, facecolors='none', label='Stable (S)')
                
                print("Stable Points Ellipses:")
                for i, row in top_stable.iterrows():
                    w, h, angle = calculate_validity_ellipse(row['x'], row['y'], avg_ev_grid, x_edges, y_edges, stable_threshold, True)
                    print(f"  S{i+1}: {w:.2f}m x {h:.2f}m @ {angle:.1f}deg")
                    f.write(f"S{i+1},{row['x']:.4f},{row['y']:.4f},{row['avg_ev']:.4f},stable,{w:.4f},{h:.4f},{angle:.2f}\n")
                    
                    # Draw Ellipse
                    ellipse = Ellipse((row['x'], row['y']), width=w, height=h, angle=angle, color='blue', alpha=0.2, zorder=9)
                    ax.add_patch(ellipse)
                    
                    ax.scatter(row['x'], row['y'], c='blue', marker='o', s=100, linewidths=2, facecolors='none', zorder=10)
                    ax.text(row['x']+0.5, row['y']-0.8, f"S{i+1}", color='blue', fontsize=12, fontweight='bold', zorder=11)

        print(f"\nSaved Final Verification Points with Ellipses to {final_csv_path}")
        ax.legend(loc='lower right')

    # Save the final figure with everything
    output_plot = os.path.join(args.output, "multi_run_degeneracy_map.pdf")
    plt.savefig(output_plot, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"Saved aggregated map with verification points to {output_plot}")

if __name__ == "__main__":
    main()
