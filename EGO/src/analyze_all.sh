#!/usr/bin/env bash
set -euo pipefail

RESULT_DIR=""
if [ -z "${1-}" ]; then
    echo "Usage: $0 <path_to_results_dir>"
    echo "Example: $0 ../results/headless/final_2000_fuzz/20260415_120000"
    exit 1
fi
RESULT_DIR=$(realpath "$1")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/planner/plan_manage/scripts" && pwd)"

echo "========================================="
echo "Step 1: Clustering Primary Vulnerabilities"
echo "========================================="
python3 "$SCRIPT_DIR/cluster_vulnerabilities.py" "$RESULT_DIR"

echo "========================================="
echo "Step 2: Sub-classifying Ripple Effects & Plotting"
echo "========================================="
python3 "$SCRIPT_DIR/plot_vulnerabilities.py" "$RESULT_DIR"

echo "========================================="
echo "Analysis complete. See detailed outputs in:"
echo " -> $RESULT_DIR/vulnerabilities_report.csv"
echo " -> $RESULT_DIR/vulnerability_distribution_detailed.png"
echo "========================================="
