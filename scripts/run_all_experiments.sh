#!/bin/bash

# Mobitia SLAM Experiment Runner
# Usage: ./scripts/run_all_experiments.sh <path_to_log_directory>

if [ -z "$1" ]; then
    echo "Usage: $0 <path_to_log_directory>"
    echo "Example: $0 slam_results/slam_result_20251110-194724"
    exit 1
fi

LOG_DIR=$1
OUTPUT_ROOT="results"
mkdir -p $OUTPUT_ROOT

echo "=============================================="
echo " Starting Mobitia SLAM Experiments"
echo " Input Log: $LOG_DIR"
echo " Output Root: $OUTPUT_ROOT"
echo "=============================================="

# 1. Ground Truth (Pseudo)
echo ""
echo "[1/6] Generating Ground Truth (High Precision)..."
cargo run --release -- --experiment --mode ground_truth --input "$LOG_DIR" --output "$OUTPUT_ROOT/ground_truth"

# 2. Baseline
echo ""
echo "[2/6] Running Baseline (No Features)..."
cargo run --release -- --experiment --mode baseline --input "$LOG_DIR" --output "$OUTPUT_ROOT/baseline"

# 3. Proposed (Standard)
echo ""
echo "[3/6] Running Proposed (Standard)..."
cargo run --release -- --experiment --mode proposed --input "$LOG_DIR" --output "$OUTPUT_ROOT/proposed"

# 4. Proposed (Fast)
echo ""
echo "[4/6] Running Proposed Fast (100 pop, 50 gen)..."
cargo run --release -- --experiment --mode proposed_fast --input "$LOG_DIR" --output "$OUTPUT_ROOT/proposed_fast"

# 5. Proposed (Eco)
echo ""
echo "[5/6] Running Proposed Eco (60 pop, 40 gen)..."
cargo run --release -- --experiment --mode proposed_eco --input "$LOG_DIR" --output "$OUTPUT_ROOT/proposed_eco"

# 6. Proposed (Minimal)
echo ""
echo "[6/6] Running Proposed Minimal (55 pop, 35 gen)..."
cargo run --release -- --experiment --mode proposed_minimal --input "$LOG_DIR" --output "$OUTPUT_ROOT/proposed_minimal"

# Analysis
echo ""
echo "=============================================="
echo " Analyzing Results..."
echo "=============================================="
python3 scripts/analyze_results.py --dir "$OUTPUT_ROOT"

echo ""
echo "Done! Check '$OUTPUT_ROOT/experiment_summary.md' and plots."
