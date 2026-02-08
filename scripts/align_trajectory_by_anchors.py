import os
import argparse
import pandas as pd
import numpy as np

def align_2d(p_start_est, p_end_est, p_start_true, p_end_true):
    """
    Calculate 2D rigid transformation (rotation + translation) 
    that maps p_start_est to p_start_true and aligns the vector 
    (p_end_est - p_start_est) with (p_end_true - p_start_true).
    """
    v_est = p_end_est - p_start_est
    v_true = p_end_true - p_start_true
    
    # Angle calculation
    angle_est = np.arctan2(v_est[1], v_est[0])
    angle_true = np.arctan2(v_true[1], v_true[0])
    angle_diff = angle_true - angle_est
    
    # Rotation matrix
    cos_a = np.cos(angle_diff)
    sin_a = np.sin(angle_diff)
    rot_matrix = np.array([
        [cos_a, -sin_a],
        [sin_a,  cos_a]
    ])
    
    return rot_matrix, angle_diff, p_start_est, p_start_true

def apply_transform(points, rot_matrix, p_start_est, p_start_true):
    # p_new = Rot * (p - p_start_est) + p_start_true
    shifted = points - p_start_est
    rotated = np.dot(shifted, rot_matrix.T)
    return rotated + p_start_true

def main():
    parser = argparse.ArgumentParser(description="Align trajectory using two anchor points (start and end).")
    parser.add_argument("--anchors", required=True, help="Path to anchor_log.csv")
    parser.add_argument("--trajectory", required=True, help="Path to trajectory.csv")
    parser.add_argument("--degeneracy_log", help="Path to degeneracy_log.csv (Optional)")
    parser.add_argument("--output_dir", required=True, help="Output directory")
    args = parser.parse_args()

    # Load anchors
    anchors = pd.read_csv(args.anchors)
    if len(anchors) < 2:
        print(f"Error: Need at least 2 anchors in {args.anchors}")
        return

    # Use first and last anchors
    start_anchor = anchors.iloc[0]
    end_anchor = anchors.iloc[-1]

    p_start_true = np.array([start_anchor['x_true'], start_anchor['y_true']])
    p_end_true = np.array([end_anchor['x_true'], end_anchor['y_true']])
    
    # Load trajectory to find estimated points
    traj = pd.read_csv(args.trajectory)
    idx_start = (traj['timestamp'] - start_anchor['timestamp']).abs().idxmin()
    idx_end = (traj['timestamp'] - end_anchor['timestamp']).abs().idxmin()
    
    p_start_est = traj.loc[idx_start, ['x', 'y']].values.astype(float)
    p_end_est = traj.loc[idx_end, ['x', 'y']].values.astype(float)
    
    print(f"Aligning based on anchors ...")
    rot_matrix, angle_diff, t_est, t_true = align_2d(p_start_est, p_end_est, p_start_true, p_end_true)
    print(f"  Rotation correction: {np.degrees(angle_diff):.4f} deg")

    os.makedirs(args.output_dir, exist_ok=True)

    # 1. Process Trajectory
    coords = traj[['x', 'y']].values.astype(float)
    aligned_coords = apply_transform(coords, rot_matrix, p_start_est, p_start_true)
    traj['x'] = aligned_coords[:, 0]
    traj['y'] = aligned_coords[:, 1]
    traj['theta'] = traj['theta'] + angle_diff
    traj_out = os.path.join(args.output_dir, os.path.basename(args.trajectory))
    traj.to_csv(traj_out, index=False)
    print(f"  Saved aligned trajectory to: {traj_out}")

    # 2. Process Degeneracy Log if exists
    if args.degeneracy_log and os.path.exists(args.degeneracy_log):
        df_deg = pd.read_csv(args.degeneracy_log)
        # Identify coordinate columns (they are at index 4,5,6 usually: x, y, theta)
        # But let's use names if available, or assume structure
        if 'x' in df_deg.columns and 'y' in df_deg.columns:
            deg_coords = df_deg[['x', 'y']].values.astype(float)
            aligned_deg_coords = apply_transform(deg_coords, rot_matrix, p_start_est, p_start_true)
            df_deg['x'] = aligned_deg_coords[:, 0]
            df_deg['y'] = aligned_deg_coords[:, 1]
            if 'theta' in df_deg.columns:
                df_deg['theta'] = df_deg['theta'] + angle_diff
            
            deg_out = os.path.join(args.output_dir, os.path.basename(args.degeneracy_log))
            df_deg.to_csv(deg_out, index=False)
            print(f"  Saved aligned degeneracy log to: {deg_out}")

if __name__ == "__main__":
    main()
