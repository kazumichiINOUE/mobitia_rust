import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import tomllib
from mpl_toolkits.mplot3d import Axes3D

def plot_landscape(csv_path, output_dir, map_dir=None, verification_info=None):
    print(f"Processing {csv_path}...")
    df = pd.read_csv(csv_path)
    
    # Pivot data for heatmap
    pivot_df = df.pivot(index='y', columns='x', values='score')
    X_grid = pivot_df.columns.values
    Y_grid = pivot_df.index.values
    Z_grid = pivot_df.values

    # Determine data bounds for plotting limits
    data_min_x, data_max_x = X_grid.min(), X_grid.max()
    data_min_y, data_max_y = Y_grid.min(), Y_grid.max()
    
    # Add some margin
    margin = 1.0
    plot_xlim = [data_min_x - margin, data_max_x + margin]
    plot_ylim = [data_min_y - margin, data_max_y + margin]

    # File basename for titles/output
    basename = os.path.basename(csv_path).replace('.csv', '')
    
    # Extract timestamp from filename (assuming landscape_TIMESTAMP.csv)
    timestamp_str = basename.replace('landscape_', '').split('_')[0]
    
    # Determine marker style based on verification info
    marker_style = {'color': 'r', 'marker': 'x', 'markersize': 10, 'markeredgewidth': 2, 'label': 'Max Score'}
    title_suffix = ""
    
    if verification_info and timestamp_str in verification_info:
        info = verification_info[timestamp_str]
        pt_type = info['type']
        pt_id = info['id']
        
        if pt_type == 'degenerate':
            marker_style = {
                'c': 'red', 'marker': 'x', 's': 200, 'linewidths': 3, 
                'label': f'Max Score ({pt_id})'
            }
            title_suffix = f" - {pt_id} (Degenerate)"
        elif pt_type == 'stable':
            marker_style = {
                'c': 'blue', 'marker': 'o', 's': 200, 'linewidths': 3, 'facecolors': 'none',
                'label': f'Max Score ({pt_id})'
            }
            title_suffix = f" - {pt_id} (Stable)"

    # --- 2D Heatmap ---
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Draw Map Background if available
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
                # origin is Top-Left (Min X, Max Y) as determined in compare_anchors.py
                map_extent = [
                    origin_x,
                    origin_x + width * resolution,
                    origin_y - height * resolution,
                    origin_y
                ]
                
                # Plot map
                ax.imshow(img, cmap='gray', extent=map_extent, alpha=0.5, origin='upper')
                print("  Overlaying occupancy map.")
                
            except Exception as e:
                print(f"  Warning: Failed to load map: {e}")
    
    # Plot Heatmap
    contour = ax.contourf(X_grid, Y_grid, Z_grid, levels=50, cmap='viridis', alpha=0.7)
    plt.colorbar(contour, label='Score')
    
    # Mark the maximum
    max_idx = np.unravel_index(np.argmax(Z_grid, axis=None), Z_grid.shape)
    max_y = Y_grid[max_idx[0]]
    max_x = X_grid[max_idx[1]]
    
    if 's' in marker_style: # Scatter style (for verification points)
        ax.scatter(max_x, max_y, **marker_style, zorder=10)
    else: # Standard plot style
        ax.plot(max_x, max_y, **marker_style)
    
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(f'Score Landscape (XY) - {basename}{title_suffix}')
    
    # Set plot limits to focus on the heatmap area
    ax.set_xlim(plot_xlim)
    ax.set_ylim(plot_ylim)
    ax.set_aspect('equal')
    
    ax.legend(loc='lower right') # Legend moved to avoid obscuring map often

    output_path_2d = os.path.join(output_dir, f"{basename}_heatmap.pdf")
    plt.savefig(output_path_2d, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"Saved 2D heatmap to {output_path_2d}")

    # --- 3D Surface Plot (No map overlay for now, hard to align visibility) ---
    # fig = plt.figure(figsize=(12, 10))
    # ax = fig.add_subplot(111, projection='3d')
    # X_mesh, Y_mesh = np.meshgrid(X_grid, Y_grid)
    # 
    # surf = ax.plot_surface(X_mesh, Y_mesh, Z_grid, cmap='viridis', edgecolor='none', alpha=0.9)
    # fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5, label='Score')
    # 
    # ax.set_xlabel('X [m]')
    # ax.set_ylabel('Y [m]')
    # ax.set_zlabel('Score')
    # ax.set_title(f'Score Landscape (3D) - {basename}')
    # 
    # output_path_3d = os.path.join(output_dir, f"{basename}_3d.pdf")
    # plt.savefig(output_path_3d, bbox_inches='tight', pad_inches=0)
    # plt.close()
    # print(f"Saved 3D plot to {output_path_3d}")

def main():
    parser = argparse.ArgumentParser(description='Plot SLAM Score Landscape')
    parser.add_argument('input', nargs='+', help='Input CSV files (landscape_*.csv)')
    parser.add_argument('--output', default='results', help='Output directory')
    parser.add_argument('--map_dir', help='Directory containing map_info.toml and occMap.png')
    parser.add_argument('--verification_points', help='Path to verification_points.csv')
    
    args = parser.parse_args()
    
    verification_info = None
    if args.verification_points and os.path.exists(args.verification_points):
        try:
            v_df = pd.read_csv(args.verification_points, dtype={'timestamp': str})
            verification_info = {}
            for _, row in v_df.iterrows():
                verification_info[row['timestamp']] = {
                    'id': row['id'],
                    'type': row['type']
                }
            print(f"Loaded {len(verification_info)} verification points.")
        except Exception as e:
            print(f"Warning: Failed to load verification points: {e}")
    
    if not os.path.exists(args.output):
        os.makedirs(args.output)
        
    for f in args.input:
        if os.path.exists(f):
            plot_landscape(f, args.output, args.map_dir, verification_info)
        else:
            print(f"File not found: {f}")

if __name__ == "__main__":
    main()
