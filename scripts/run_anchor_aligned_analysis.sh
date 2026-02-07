#!/bin/bash

# scripts/run_anchor_aligned_analysis.sh
# Optimized workflow for anchor-based alignment and statistical analysis.
# Ensures brute_force search is only run once per raw log.

ROOT_DIR="experiment_anchor_260207"
ANALYSIS_DIR="$ROOT_DIR/analysis"
ALIGNED_DIR="$ROOT_DIR/aligned"
FINAL_DIR="$ROOT_DIR/final_results"

# Standardize filenames
TRAJ_FILE="brute_force_trajectory.csv"
DEG_FILE="degeneracy_log.csv"

mkdir -p "$ANALYSIS_DIR" "$ALIGNED_DIR" "$FINAL_DIR"

# Stage 1: Initial SLAM Analysis (Slow Full Search)
# This is the ONLY place where slow computation happens.
for dir in "$ROOT_DIR"/slam_result_*; do
    [ -d "$dir" ] || continue
    run_name=$(basename "$dir")
    [ "$run_name" == "analysis" ] || [ "$run_name" == "aligned" ] || [ "$run_name" == "final_results" ] && continue
    
    out_dir="$ANALYSIS_DIR/$run_name"
    if [ ! -f "$out_dir/$DEG_FILE" ]; then
        echo ">>> [Stage 1] Calculating base SLAM results for $run_name..."
        mkdir -p "$out_dir"
        cargo run --release --bin mobitia -- --experiment --mode brute_force --input "$dir" --output "$out_dir" --step 1
    else
        echo ">>> [Stage 1] Base results for $run_name already exist. Skipping search."
    fi
done

# Stage 2: Geometric Alignment (Instant Python Transformation)
echo ">>> [Stage 2] Aligning trajectories and logs using physical anchors..."
for dir_raw in "$ROOT_DIR"/slam_result_*; do
    [ -d "$dir_raw" ] || continue
    run_name=$(basename "$dir_raw")
    [ "$run_name" == "analysis" ] || [ "$run_name" == "aligned" ] || [ "$run_name" == "final_results" ] && continue
    
    dir_aligned="$ALIGNED_DIR/$run_name"
    mkdir -p "$dir_aligned"
    
    # This transforms the coordinates in both traj and deg_log without re-running SLAM.
    python3 scripts/align_trajectory_by_anchors.py \
        --anchors "$dir_raw/anchor_log.csv" \
        --trajectory "$ANALYSIS_DIR/$run_name/$TRAJ_FILE" \
        --degeneracy_log "$ANALYSIS_DIR/$run_name/$DEG_FILE" \
        --output_dir "$dir_aligned"
done

# Stage 3: Map Reconstruction (Fast Mapping Only)
# Re-draws the map using aligned coordinates.
echo ">>> [Stage 3] Generating aligned occupancy maps..."
for dir_aligned in "$ALIGNED_DIR"/slam_result_*; do
    [ -d "$dir_aligned" ] || continue
    run_name=$(basename "$dir_aligned")
    
    # Symlink to raw scans is required for mapping
    ln -sfn "../../$run_name/submaps" "$dir_aligned/submaps"
    
    # We must ensure mobitia uses the ALIGNED trajectory we just created.
    # brute_force_mapping_only reads 'brute_force_trajectory.csv' from the input dir.
    echo "    Mapping $run_name..."
    cargo run --release --bin mobitia -- --experiment --mode brute_force_mapping_only --input "$dir_aligned" --output "$dir_aligned" --step 1
    
    # Mobitia might output 'brute_force_mapping_only_trajectory.csv', but we want to keep our aligned one.
    # Stage 4 expects 'degeneracy_log.csv' which we already have.
done

# Stage 4: Statistical Consolidation
echo ">>> [Stage 4] Producing final statistical map and verification points..."
python3 scripts/analyze_multi_run_degeneracy.py "$ALIGNED_DIR" --output "$FINAL_DIR" --grid_size 0.05

echo "========================================================="
echo " Optimized Analysis Complete."
echo " Results: $FINAL_DIR"
echo "========================================================="
