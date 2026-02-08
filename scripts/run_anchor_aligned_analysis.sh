#!/bin/bash

# scripts/run_anchor_aligned_analysis.sh
# Optimized workflow for anchor-based alignment and statistical analysis.
# Generates both integrated statistical maps and clean paper-ready uncertainty plots.

# Targeted directory (defaults to current experiment if not provided)
ROOT_DIR=${1:-"experiment_anchor_260207"}

ANALYSIS_DIR="$ROOT_DIR/analysis"
ALIGNED_DIR="$ROOT_DIR/aligned"
FINAL_DIR="$ROOT_DIR/final_results"

# Standardize filenames
TRAJ_FILE="brute_force_trajectory.csv"
DEG_FILE="degeneracy_log.csv"

echo "========================================================="
echo " Starting Anchor-Aligned Analysis Pipeline"
echo " Target: $ROOT_DIR"
echo "========================================================="

mkdir -p "$ANALYSIS_DIR" "$ALIGNED_DIR" "$FINAL_DIR"

# Stage 1: Initial SLAM Analysis (Slow Full Search)
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
    
    python3 scripts/align_trajectory_by_anchors.py \
        --anchors "$dir_raw/anchor_log.csv" \
        --trajectory "$ANALYSIS_DIR/$run_name/$TRAJ_FILE" \
        --degeneracy_log "$ANALYSIS_DIR/$run_name/$DEG_FILE" \
        --output_dir "$dir_aligned"
done

# Stage 3: Map Reconstruction (Fast Mapping Only)
echo ">>> [Stage 3] Generating aligned occupancy maps..."
for dir_aligned in "$ALIGNED_DIR"/slam_result_*; do
    [ -d "$dir_aligned" ] || continue
    run_name=$(basename "$dir_aligned")
    
    ln -sfn "../../$run_name/submaps" "$dir_aligned/submaps"
    
    if [ ! -f "$dir_aligned/occMap.png" ]; then
        echo "    Mapping $run_name..."
        cargo run --release --bin mobitia -- --experiment --mode brute_force_mapping_only --input "$dir_aligned" --output "$dir_aligned" --step 1
    fi
done

# Stage 4: Statistical Consolidation
echo ">>> [Stage 4] Producing final statistical map and verification points..."
python3 scripts/analyze_multi_run_degeneracy.py "$ALIGNED_DIR" --output "$FINAL_DIR" --grid_size 0.05

# Stage 5: Clean Paper Plot
echo ">>> [Stage 5] Generating clean paper-ready uncertainty map..."
python3 scripts/plot_paper_uncertainty.py \
    --points "$FINAL_DIR/final_verification_points.csv" \
    --aligned_dir "$ALIGNED_DIR" \
    --output "$FINAL_DIR/paper_uncertainty_ellipses.pdf"

echo "========================================================="
echo " Analysis Complete."
echo " Integrated Map: $FINAL_DIR/multi_run_degeneracy_map.pdf"
echo " Paper Plot:     $FINAL_DIR/paper_uncertainty_ellipses.pdf"
echo " Points Data:    $FINAL_DIR/final_verification_points.csv"
echo "========================================================="