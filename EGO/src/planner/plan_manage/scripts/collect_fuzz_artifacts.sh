#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <abs_target_fuzz_report_dir> <fuzz_run_id>"
  echo "Example: $0 /path/to/run/fuzz_report RUN_ID"
  echo "Optional: set RIPPLE_ROOT to the repository root to search another checkout."
  exit 2
fi

TARGET_FUZZ_REPORT_DIR="$1"
FUZZ_RUN_ID="$2"
USER_REPORTS_DIR="$HOME/.ego_fuzz_reports"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="${RIPPLE_ROOT:-$(cd "$SCRIPT_DIR/../../../../.." && pwd)}"
EGO_WORKSPACE_DIR="$REPOSITORY_ROOT/EGO"

SRC_DIRS=()
# primary candidate
if [ -d "$USER_REPORTS_DIR/$FUZZ_RUN_ID" ]; then
  SRC_DIRS+=("$USER_REPORTS_DIR/$FUZZ_RUN_ID")
fi
# also search workspace results locations (best-effort)
while IFS= read -r d; do
  SRC_DIRS+=("$d")
done < <(find "$EGO_WORKSPACE_DIR" -type d -name "$FUZZ_RUN_ID" 2>/dev/null || true)

if [ ${#SRC_DIRS[@]} -eq 0 ]; then
  echo "No source fuzz run dir found for $FUZZ_RUN_ID (looked under $USER_REPORTS_DIR and $EGO_WORKSPACE_DIR)."
  exit 1
fi

DEST_DIR="$TARGET_FUZZ_REPORT_DIR/$FUZZ_RUN_ID"
mkdir -p "$DEST_DIR"

copied=0
for s in "${SRC_DIRS[@]}"; do
  echo "Scanning source: $s"
  for pat in "comm_fuzz_meta.json" "comm_fuzz_trace.jsonl" "planner_rx_trace_drone_*.jsonl" "planner_safety_trace_drone_*.jsonl"; do
    shopt -s nullglob
    for f in "$s"/$pat; do
      echo "Copying $f -> $DEST_DIR/"
      cp -a "$f" "$DEST_DIR/" || true
      copied=$((copied+1))
    done
    shopt -u nullglob
  done
done

if [ "$copied" -eq 0 ]; then
  echo "No artifact files found for $FUZZ_RUN_ID in candidate dirs."
  exit 2
fi

echo "Copied $copied files into $DEST_DIR"
exit 0
