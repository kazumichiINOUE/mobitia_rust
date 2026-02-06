#!/bin/bash

# Mobitia SLAM Experiment Runner
# Usage: ./scripts/run_all_experiments.sh <path_to_log_directory>

if [ -z "$1" ]; then
    echo "Usage: $0 <path_to_log_directory>"
    echo "Example: $0 slam_results/slam_result_20251110-194724"
    exit 1
fi

LOG_DIR=$1
# Default anchor log is expected to be INSIDE the log directory
ANCHOR_LOG=${2:-"$LOG_DIR/anchor_log.csv"} 
OUTPUT_ROOT="results"
mkdir -p $OUTPUT_ROOT

echo "=============================================="
echo " Starting Mobitia SLAM Experiments"
echo " Input Log: $LOG_DIR"
echo " Output Root: $OUTPUT_ROOT"
echo " Anchor Log: $ANCHOR_LOG"
echo "=============================================="

# 1. Ground Truth (Pseudo)
echo ""
echo "[1/7] Generating Ground Truth (High Precision)..."
cargo run --release --bin mobitia -- --experiment --mode ground_truth --input "$LOG_DIR" --output "$OUTPUT_ROOT/ground_truth"

# 2. Baseline
echo ""
echo "[2/7] Running Baseline (No Features)..."
cargo run --release --bin mobitia -- --experiment --mode baseline --input "$LOG_DIR" --output "$OUTPUT_ROOT/baseline"

# 3. Proposed (Standard)
echo ""
echo "[3/7] Running Proposed (Standard)..."
cargo run --release --bin mobitia -- --experiment --mode proposed --input "$LOG_DIR" --output "$OUTPUT_ROOT/proposed"

# 4. Proposed (Fast)
echo ""
echo "[4/7] Running Proposed Fast (100 pop, 50 gen)..."
cargo run --release --bin mobitia -- --experiment --mode proposed_fast --input "$LOG_DIR" --output "$OUTPUT_ROOT/proposed_fast"

# 5. Proposed (Eco)
echo ""
echo "[5/7] Running Proposed Eco (60 pop, 40 gen)..."
cargo run --release --bin mobitia -- --experiment --mode proposed_eco --input "$LOG_DIR" --output "$OUTPUT_ROOT/proposed_eco"

# 6. Proposed (Minimal)
echo ""
echo "[6/7] Running Proposed Minimal (55 pop, 35 gen)..."
cargo run --release --bin mobitia -- --experiment --mode proposed_minimal --input "$LOG_DIR" --output "$OUTPUT_ROOT/proposed_minimal"

# 7. Brute-force (Rigorous Ground Truth)
echo ""
echo "[7/7] Running Brute-force Search (Deterministic, VERY SLOW)..."
# Pass anchor log to enable landscape generation
cargo run --release --bin mobitia -- --experiment --mode brute_force --input "$LOG_DIR" --anchors "$ANCHOR_LOG" --output "$OUTPUT_ROOT/brute_force"

# Analysis
echo ""
echo "=============================================="
echo " Analyzing Results..."
echo "=============================================="
python3 scripts/analyze_results.py --dir "$OUTPUT_ROOT" --map_dir "$LOG_DIR"

# Anchor Comparison
BRUTE_TRAJ="$OUTPUT_ROOT/brute_force/brute_force_trajectory.csv"
if [ -f "$ANCHOR_LOG" ] && [ -f "$BRUTE_TRAJ" ]; then
    echo ""
    echo "=============================================="
    echo " Comparing Anchors vs Brute-force..."
    echo "=============================================="
    # Assuming map_dir matches the input log structure for plotting
    python3 scripts/compare_anchors.py --anchors "$ANCHOR_LOG" --trajectory "$BRUTE_TRAJ" --output "$OUTPUT_ROOT" --map_dir "$LOG_DIR"
else
    echo "Skipping anchor comparison (Anchor log or Brute-force result not found)."
fi

# Degeneracy Plotting (Run FIRST to generate verification_points.csv)
DEGENERACY_LOG="$OUTPUT_ROOT/brute_force/degeneracy_log.csv"
if [ -f "$DEGENERACY_LOG" ]; then
    echo ""
    echo "=============================================="
    echo " Analyzing Degeneracy..."
    echo "=============================================="
    python3 scripts/plot_degeneracy.py "$DEGENERACY_LOG" --output "$OUTPUT_ROOT/brute_force" --map_dir "$LOG_DIR"
fi

# Landscape Plotting (Run SECOND, using verification points)
LANDSCAPE_FILES=$(find "$OUTPUT_ROOT/brute_force" -name "landscape_*.csv")
if [ ! -z "$LANDSCAPE_FILES" ]; then
    echo ""
    echo "=============================================="
    echo " Plotting Landscapes..."
    echo "=============================================="
    python3 scripts/plot_landscape.py $LANDSCAPE_FILES --output "$OUTPUT_ROOT/landscapes" --map_dir "$LOG_DIR" --verification_points "$OUTPUT_ROOT/brute_force/verification_points.csv"
fi

echo ""
echo "Done! Check '$OUTPUT_ROOT/experiment_summary.md' and plots."