#!/bin/bash

# Mobitia Degeneracy Mapping Batch Runner
# This script runs brute_force_mapping_only for all directories in experiment_degeneracy_stats

BASE_DIR="experiment_degeneracy_stats"

if [ ! -d "$BASE_DIR" ]; then
    echo "Error: $BASE_DIR not found."
    exit 1
fi

echo "=============================================="
echo " Starting Degeneracy Mapping Batch"
echo " Target Directory: $BASE_DIR"
echo "=============================================="

# Pre-build to avoid repeated builds in the loop
cargo build --release --bin mobitia

# Get all directories starting with slam_result_
dirs=($(find "$BASE_DIR" -maxdepth 1 -type d -name "slam_result_*" | sort))

total=${#dirs[@]}
current=0

for dir in "${dirs[@]}"; do
    current=$((current + 1))
    echo ""
    echo "[$current/$total] Processing $dir..."
    
    # Run experiment
    # Results are saved to $dir by default (as per our previous setup)
    cargo run --release --bin mobitia -- --experiment --mode brute_force_mapping_only --input "$dir" --output "$dir" --step 1
    
    if [ $? -ne 0 ]; then
        echo "Warning: Experiment failed for $dir"
    fi
done

echo ""
echo "=============================================="
echo " All experiments completed."
echo "=============================================="