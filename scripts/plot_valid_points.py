#!/usr/bin/env python3
import json
import argparse
import matplotlib.pyplot as plt
import os
import glob

def find_scan_files(path):
    """
    If the path is a directory, find all submap_*/scans.json files within it.
    Otherwise, assume the path is a file path.

    Args:
        path (str): A path to a directory or a scans.json file.

    Returns:
        list: A list of file paths to process.
    """
    if os.path.isdir(path):
        search_pattern = os.path.join(path, "submaps", "*", "scans.json")
        found_files = glob.glob(search_pattern)
        if not found_files:
            print(f"Warning: No 'submaps/*/scans.json' files found in directory: {path}")
        return sorted(found_files)
    # If it's not a directory, assume it's a file path
    return [path]

def plot_valid_points(file_paths, x_axis_mode):
    """
    Loads data and plots valid_point_count against time or distance.

    Args:
        file_paths (list): A list of paths to scans.json files.
        x_axis_mode (str): 'time' or 'distance'.
    """
    all_data_points = []

    for file_path in file_paths:
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
                all_data_points.extend(data)
        except FileNotFoundError:
            print(f"Error: File not found at {file_path}")
            continue
        except json.JSONDecodeError:
            print(f"Error: Could not decode JSON from {file_path}")
            continue
        except Exception as e:
            print(f"An error occurred while processing {file_path}: {e}")
            continue
    
    if not all_data_points:
        print("No data points to plot.")
        return

    # Sort all collected data points globally by timestamp
    all_data_points.sort(key=lambda item: item.get('timestamp', 0))

    valid_counts = [item.get('valid_point_count', 0) for item in all_data_points]
    
    x_axis_data = []
    x_label = ""
    plot_title = ""

    if x_axis_mode == 'time':
        timestamps = [item.get('timestamp', 0) for item in all_data_points]
        if timestamps:
            first_timestamp = timestamps[0]
            x_axis_data = [(ts - first_timestamp) / 1000.0 for ts in timestamps]
        x_label = "Elapsed Time (s)"
        plot_title = "Valid LiDAR Point Count Over Time"
    
    elif x_axis_mode == 'distance':
        cumulative_distance = 0.0
        x_axis_data.append(cumulative_distance)
        
        poses = [item.get('relative_pose') for item in all_data_points]
        
        for i in range(1, len(poses)):
            prev_pose = poses[i-1]
            curr_pose = poses[i]
            
            if prev_pose and curr_pose:
                dx = curr_pose['x'] - prev_pose['x']
                dy = curr_pose['y'] - prev_pose['y']
                step_distance = (dx**2 + dy**2)**0.5
                
                # Threshold to ignore jumps between submaps
                if step_distance < 1.0: # Ignore jumps > 1 meter
                    cumulative_distance += step_distance
            
            x_axis_data.append(cumulative_distance)
            
        x_label = "Cumulative Distance (m)"
        plot_title = "Valid LiDAR Point Count by Distance"

    # Plot the continuous data
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, ax = plt.subplots()

    ax.plot(x_axis_data, valid_counts, marker='.', linestyle='-')

    ax.set_xlabel(x_label)
    ax.set_ylabel("Number of Valid LiDAR Points")
    ax.set_title(plot_title)
    ax.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot the number of valid LiDAR points from a slam_result directory or specific scans.json files."
    )
    parser.add_argument(
        "paths",
        metavar="PATH",
        nargs='+',
        help="Path to a slam_result directory or one or more scans.json files."
    )
    parser.add_argument(
        "--x-axis",
        type=str,
        choices=['time', 'distance'],
        default='time',
        help="The value for the x-axis ('time' or 'distance'). Default is 'time'."
    )
    args = parser.parse_args()
    
    all_files_to_plot = []
    for path in args.paths:
        all_files_to_plot.extend(find_scan_files(path))
    
    if all_files_to_plot:
        plot_valid_points(all_files_to_plot, args.x_axis)
    else:
        print("No valid files found to plot.")
