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
from matplotlib.colors import LogNorm

def run_brute_force_analysis(run_dir):
    print(f"  Running brute_force analysis for: {run_dir} ...")
    output_dir = os.path.join(run_dir, "results", "brute_force")
    cmd = [
        "cargo", "run", "--release", "--bin", "mobitia", "--",
        "--experiment", "--mode", "brute_force", "--input", run_dir, "--output", output_dir
    ]
    try:
        subprocess.run(cmd, check=True)
        print("  Analysis complete.")
        return True
    except subprocess.CalledProcessError as e:
        print(f"  Error running analysis: {e}")
        return False

def umeyama_alignment(source_points, target_points):
    n, m = source_points.shape
    centroid_A = np.mean(source_points, axis=0)
    centroid_B = np.mean(target_points, axis=0)
    AA = source_points - centroid_A
    BB = target_points - centroid_B
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    Rot = np.dot(Vt.T, U.T)
    if np.linalg.det(Rot) < 0:
        Vt[m-1, :] *= -1
        Rot = np.dot(Vt.T, U.T)
    t = centroid_B - np.dot(Rot, centroid_A)
    return Rot, t, np.dot(source_points, Rot.T) + t

def get_nearest_neighbors(src, dst):
    n = src.shape[0]
    indices = np.zeros(n, dtype=int)
    distances = np.zeros(n)
    for i in range(n):
        diff = dst - src[i]
        d2 = np.sum(diff**2, axis=1)
        idx = np.argmin(d2)
        indices[i] = idx
        distances[i] = d2[idx]
    return np.sqrt(distances), indices

def icp_alignment(source_points, target_points, max_iterations=30, tolerance=1e-4):
    src = np.copy(source_points)
    dst = target_points
    centroid_src = np.mean(src, axis=0)
    centroid_dst = np.mean(dst, axis=0)
    t_init = centroid_dst - centroid_src
    src += t_init
    total_t = t_init
    prev_error = float('inf')
    for i in range(max_iterations):
        _, indices = get_nearest_neighbors(src, dst)
        matched_dst = dst[indices]
        R_step, t_step, _ = umeyama_alignment(src, matched_dst)
        src = np.dot(src, R_step.T) + t_step
        total_t += t_step
        mean_error = np.mean(np.linalg.norm(src - matched_dst, axis=1))
        if abs(prev_error - mean_error) < tolerance: break
        prev_error = mean_error
    return None, total_t, src

def load_run_data(run_dir):
    possible_paths = [
        os.path.join(run_dir, "results", "brute_force", "degeneracy_log.csv"),
        os.path.join(run_dir, "brute_force", "degeneracy_log.csv"),
        os.path.join(run_dir, "degeneracy_log.csv"),
    ]
    for path in possible_paths:
        if os.path.exists(path):
            print(f"  Loaded: {path}")
            return pd.read_csv(path), path
    print(f"  Degeneracy log not found in {run_dir}. Attempting to run analysis...")
    if run_brute_force_analysis(run_dir):
        expected_output = os.path.join(run_dir, "results", "brute_force", "degeneracy_log.csv")
        if os.path.exists(expected_output):
             print(f"  Loaded generated log: {expected_output}")
             return pd.read_csv(expected_output), expected_output
    print(f"  Warning: Failed to load or generate degeneracy_log.csv for {run_dir}")
    return None, None

def main():
    parser = argparse.ArgumentParser(description="Analyze degeneracy across multiple SLAM runs with alignment.")
    parser.add_argument("root_dir", help="Root directory containing run subdirectories")
    parser.add_argument("--grid_size", type=float, default=0.2, help="Grid size for statistical mapping [m]")
    parser.add_argument("--output", default="multi_run_results", help="Output directory")
    args = parser.parse_args()
    
    if not os.path.exists(args.root_dir):
        print(f"Error: Directory {args.root_dir} not found.")
        return

    run_dirs = sorted([d for d in glob.glob(os.path.join(args.root_dir, "*")) if os.path.isdir(d)])
    if not run_dirs:
        print("No subdirectories found.")
        return
    
    print(f"Found {len(run_dirs)} run directories.")
    runs_data = []
    for d in run_dirs:
        run_name = os.path.basename(d)
        df, path = load_run_data(d)
        if df is not None:
            runs_data.append({'name': run_name, 'df': df, 'dir': d})
            
    if not runs_data:
        print("No valid data loaded.")
        return

    runs_data.sort(key=lambda x: len(x['df']), reverse=True)
    ref_run = runs_data[0]
    ref_df = ref_run['df']
    ref_points = ref_df[['x', 'y']].values
    print(f"\nReference Run (Longest): {ref_run['name']} ({len(ref_points)} points)")
    
    aligned_runs = []
    aligned_runs.append({
        'name': ref_run['name'], 'x': ref_points[:, 0], 'y': ref_points[:, 1],
        'min_ev': ref_df['min_eigenvalue'].values, 'timestamp': ref_df['timestamp'].values
    })
    
    for i in range(1, len(runs_data)):
        target_run = runs_data[i]
        tgt_points = target_run['df'][['x', 'y']].values
        _, t_vec, aligned_tgt_points = icp_alignment(tgt_points, ref_points)
        print(f"Aligned {target_run['name']} to Reference using ICP. (Offset: {t_vec})")
        aligned_runs.append({
            'name': target_run['name'], 'x': aligned_tgt_points[:, 0], 'y': aligned_tgt_points[:, 1],
            'min_ev': target_run['df']['min_eigenvalue'].values, 'timestamp': target_run['df']['timestamp'].values
        })

    print("\nGenerating Statistical Map...")
    all_x = np.concatenate([r['x'] for r in aligned_runs])
    all_y = np.concatenate([r['y'] for r in aligned_runs])
    margin = 2.0
    grid_res = args.grid_size
    x_edges = np.arange(np.min(all_x) - margin, np.max(all_x) + margin, grid_res)
    y_edges = np.arange(np.min(all_y) - margin, np.max(all_y) + margin, grid_res)
    
    sum_ev_grid = np.zeros((len(y_edges)-1, len(x_edges)-1))
    count_grid = np.zeros((len(y_edges)-1, len(x_edges)-1))
    for r in aligned_runs:
        x_idxs = np.digitize(r['x'], x_edges) - 1
        y_idxs = np.digitize(r['y'], y_edges) - 1
        valid_mask = (x_idxs >= 0) & (x_idxs < len(x_edges)-1) & (y_idxs >= 0) & (y_idxs < len(y_edges)-1)
        vx, vy, vev = x_idxs[valid_mask], y_idxs[valid_mask], r['min_ev'][valid_mask]
        for j in range(len(vx)):
            sum_ev_grid[vy[j], vx[j]] += vev[j]
            count_grid[vy[j], vx[j]] += 1
            
    with np.errstate(divide='ignore', invalid='ignore'):
        avg_ev_grid = sum_ev_grid / count_grid
        avg_ev_grid[count_grid == 0] = np.nan

    for _ in range(3):
        filled_grid = avg_ev_grid.copy()
        for y in range(avg_ev_grid.shape[0]):
            for x in range(avg_ev_grid.shape[1]):
                if np.isnan(avg_ev_grid[y, x]):
                    y_min, y_max = max(0, y-1), min(avg_ev_grid.shape[0], y+2)
                    x_min, x_max = max(0, x-1), min(avg_ev_grid.shape[1], x+2)
                    neighbors = avg_ev_grid[y_min:y_max, x_min:x_max]
                    if np.any(~np.isnan(neighbors)):
                        filled_grid[y, x] = np.nanmean(neighbors)
        avg_ev_grid = filled_grid

    degenerate_threshold = 10000.0
    stable_threshold = 100000.0
    
    if not os.path.exists(args.output): os.makedirs(args.output)
    fig, ax = plt.subplots(figsize=(12, 10))
    
    for root, _, files in os.walk(ref_run['dir']):
        if "occMap.png" in files and "map_info.toml" in files:
            try:
                with open(os.path.join(root, "map_info.toml"), "rb") as f:
                    m_info = tomllib.load(f)
                res = m_info.get("resolution", 0.05)
                org = m_info.get("origin", [0.0, 0.0, 0.0])
                img = plt.imread(os.path.join(root, "occMap.png"))
                h, w = img.shape[:2]
                extent = [org[0], org[0] + w * res, org[1] - h * res, org[1]]
                ax.imshow(img, cmap='gray', extent=extent, alpha=0.5, origin='upper', zorder=0)
                print(f"Overlaying map from {root}")
                break
            except: pass

    X, Y = np.meshgrid(x_edges, y_edges)
    plot_grid = avg_ev_grid.copy()
    plot_grid[plot_grid <= 0] = 1.0
    
    def si_format(x, pos):
        if x >= 1e6: return f'{x*1e-6:.1f}M'
        elif x >= 1e3: return f'{x*1e-3:.0f}k'
        else: return f'{x:.0f}'

    cmap = plt.cm.jet_r
    mesh = ax.pcolormesh(X, Y, plot_grid, cmap=cmap, norm=LogNorm(), alpha=0.6, shading='flat', zorder=1)
    plt.colorbar(mesh, label='Average Min Eigenvalue', shrink=0.6, format=FuncFormatter(si_format))
    
    for r in aligned_runs: ax.plot(r['x'], r['y'], 'k-', linewidth=0.5, alpha=0.3, zorder=2)
    ax.set_title(f'Statistical Degeneracy Map (Aggregated from {len(runs_data)} Runs)')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_aspect('equal')
    
    cells = []
    for y in range(avg_ev_grid.shape[0]):
        for x in range(avg_ev_grid.shape[1]):
            val = avg_ev_grid[y, x]
            if not np.isnan(val):
                cells.append({'x': (x_edges[x] + x_edges[x+1])/2.0, 'y': (y_edges[y] + y_edges[y+1])/2.0, 'avg_ev': val})
    df_cells = pd.DataFrame(cells)
    
    if not df_cells.empty:
        def select_spatially_distinct_points(df, n=5, min_dist=2.0, maximize=False, forbidden_points=None):
            sorted_df = df.sort_values('avg_ev', ascending=not maximize).reset_index(drop=True)
            selected = []
            for _, row in sorted_df.iterrows():
                if len(selected) >= n: break
                is_far = True
                for s in selected:
                    if np.sqrt((row['x'] - s['x'])**2 + (row['y'] - s['y'])**2) < min_dist:
                        is_far = False; break
                if is_far and forbidden_points is not None:
                    for _, f_row in forbidden_points.iterrows():
                        if np.sqrt((row['x'] - f_row['x'])**2 + (row['y'] - f_row['y'])**2) < 2.0:
                            is_far = False; break
                if is_far: selected.append(row)
            return pd.DataFrame(selected)

        top_d = select_spatially_distinct_points(df_cells[df_cells['avg_ev'] < stable_threshold], n=5, min_dist=2.0, maximize=False)
        top_s = select_spatially_distinct_points(df_cells[df_cells['avg_ev'] > degenerate_threshold], n=5, min_dist=2.0, maximize=True, forbidden_points=top_d)
        top_d = top_d.sort_values('x').reset_index(drop=True)
        top_s = top_s.sort_values('x').reset_index(drop=True)

        def calculate_validity_ellipse(cx, cy, grid, x_edges, y_edges, is_stable, max_dist=1.0):
            xi_s = np.digitize(cx, x_edges) - 1
            yi_s = np.digitize(cy, y_edges) - 1
            if xi_s < 0 or xi_s >= grid.shape[1] or yi_s < 0 or yi_s >= grid.shape[0] or np.isnan(grid[yi_s, xi_s]): return 0.1, 0.1, 0.0
            center_val = grid[yi_s, xi_s]
            queue, visited, v_pts = [(xi_s, yi_s)], set([(xi_s, yi_s)]), []
            res = x_edges[1] - x_edges[0]
            max_steps = int(max_dist / res)
            while queue:
                xi, yi = queue.pop(0)
                v_pts.append([(x_edges[xi]+x_edges[xi+1])/2.0, (y_edges[yi]+y_edges[yi+1])/2.0])
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        if dx == 0 and dy == 0: continue
                        nx, ny = xi + dx, yi + dy
                        if abs(nx - xi_s) > max_steps or abs(ny - yi_s) > max_steps: continue
                        if (nx, ny) in visited: continue
                        if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                            val = grid[ny, nx]
                            if not np.isnan(val):
                                is_v = (val > center_val*0.33) if is_stable else (val < center_val*10.0 or val < 20000.0)
                                if is_v: visited.add((nx, ny)); queue.append((nx, ny))
            pts = np.array(v_pts)
            if len(pts) < 3: return 0.1, 0.1, 0.0
            cov = np.cov(pts, rowvar=False)
            try:
                vals, vecs = np.linalg.eigh(cov)
                order = vals.argsort()[::-1]
                vals, vecs = vals[order], vecs[:, order]
                theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
                return max(0.1, 4*np.sqrt(max(0, vals[0]))), max(0.1, 4*np.sqrt(max(0, vals[1]))), theta
            except: return 0.1, 0.1, 0.0

        final_csv = os.path.join(args.output, "final_verification_points.csv")
        with open(final_csv, "w") as f:
            f.write("id,x,y,avg_min_eigenvalue,type,major_axis,minor_axis,angle_deg\n")
            if not top_d.empty:
                ax.scatter([], [], c='red', marker='x', s=200, linewidths=3, label='Degenerate (D)')
                print("Degenerate Ellipses:")
                for i, row in top_d.iterrows():
                    w, h, ang = calculate_validity_ellipse(row['x'], row['y'], avg_ev_grid, x_edges, y_edges, False)
                    print(f"  D{i+1}: {w:.2f}m x {h:.2f}m @ {ang:.1f}deg")
                    f.write(f"D{i+1},{row['x']:.4f},{row['y']:.4f},{row['avg_ev']:.4f},degenerate,{w:.4f},{h:.4f},{ang:.2f}\n")
                    ax.add_patch(Ellipse((row['x'], row['y']), w, h, angle=ang, color='red', alpha=0.3, zorder=9))
                    ax.scatter(row['x'], row['y'], c='red', marker='x', s=100, linewidths=2, zorder=10)
                    ax.text(row['x'], row['y']+0.5, f"D{i+1}", color='red', fontsize=12, fontweight='bold', zorder=11)
            if not top_s.empty:
                ax.scatter([], [], c='blue', marker='o', s=150, linewidths=2, facecolors='none', label='Stable (S)')
                print("Stable Ellipses:")
                for i, row in top_s.iterrows():
                    w, h, ang = calculate_validity_ellipse(row['x'], row['y'], avg_ev_grid, x_edges, y_edges, True)
                    print(f"  S{i+1}: {w:.2f}m x {h:.2f}m @ {ang:.1f}deg")
                    f.write(f"S{i+1},{row['x']:.4f},{row['y']:.4f},{row['avg_ev']:.4f},stable,{w:.4f},{h:.4f},{ang:.2f}\n")
                    ax.add_patch(Ellipse((row['x'], row['y']), w, h, angle=ang, color='blue', alpha=0.2, zorder=9))
                    ax.scatter(row['x'], row['y'], c='blue', marker='o', s=100, linewidths=2, facecolors='none', zorder=10)
                    ax.text(row['x'], row['y']+0.5, f"S{i+1}", color='blue', fontsize=12, fontweight='bold', zorder=11)

        ax.legend(loc='upper right')
    
    plt.tight_layout()
    pdf_path = os.path.join(args.output, "multi_run_degeneracy_map.pdf")
    plt.savefig(pdf_path)
    plt.savefig(pdf_path.replace(".pdf", ".png"))
    print(f"\nSaved statistical map to {pdf_path}")
    plt.close()

if __name__ == "__main__":
    main()