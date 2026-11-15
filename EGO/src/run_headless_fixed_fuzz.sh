#!/usr/bin/env bash
# Wrapper around run_headless.sh: fixed fuzz inputs without editing run_headless.sh.
#
# Repeat-same-seed mode (recommended for identical comm_fuzz_seed every iteration):
#   run_headless.sh uses seed = COMM_FUZZ_SEED_BASE + round_index. With RUNS=1 the only
#   round index is 1, so set COMM_FUZZ_SEED_BASE = (desired_seed - 1) to get comm_fuzz_seed = desired_seed.
#   This script loops RUNS times, each time OUT_DIR=<parent>/repeat_XXXX/, then merges */summary.csv
#   into <parent>/summary_merged.csv.
#
# Single-batch mode (one summary.csv under OUT_DIR/<BATCH_TS>/, seeds vary as BASE+1..BASE+N):
#   Omit --comm-fuzz-seed and pass --seed-base; one invocation with RUNS=N.
#
# Noise strength: run_headless samples with unseeded Python random unless MIN==MAX; use --noise-sigma
# for a fixed value (sets COMM_FUZZ_NOISE_SIGMA_POS_MIN/MAX equal).
#
# If the same repeat_* dir is reused across runs, multiple BATCH_TS subdirs exist; always pick the
# newest summary.csv/summary.txt by mtime for merge and for CSV correction (not glob order).
#
# CSV correction: when a round has no collision/emergency, run_headless.sh deletes fuzz_report and the
# round row's comm_fuzz_* fields may be filled by sample_mixed_comm_plan (wrong mode/factor). For rows
# with fuzz_report_dir=NA we rewrite comm_fuzz_seed/mode/target/window/factor from this script's CLI
# and exported COMM_FUZZ_* env. Modes random|pure_random|mixed do not override factor_name/value.
#
# Paths: run_headless.sh cds to this directory; OUT_DIR is relative to swarm-playground/main_ws/src.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_HEADLESS="$SCRIPT_DIR/run_headless.sh"

RUNS=1
RUN_SEC=60
OUT_DIR="headless_fixed_fuzz"
COLLISION_MONITOR="true"
COMM_FUZZ="true"
MODE="noise"
TARGET_DRONE=3
START_SEC="5.0"
DURATION_SEC="20.0"
SEED_BASE=""
COMM_FUZZ_SEED=""
SEED_GEN_MODE="pure_random"
KEY_TIME_SEC="12.0"
NOISE_SIGMA=""
DELAY_MIN_MS=""
DELAY_MAX_MS=""
JITTER_MIN_MS=""
JITTER_MAX_MS=""
NOISE_MIN=""
NOISE_MAX=""

usage() {
  cat <<'EOF'
Usage: run_headless_fixed_fuzz.sh [options]

Calls run_headless.sh with fixed fuzz window (start/duration min=max) and optional strength env vars.

Repeat-same-seed mode (merges CSV):
  --comm-fuzz-seed S   Loop --runs times with RUNS=1 each; each iteration uses seed S
                       (via COMM_FUZZ_SEED_BASE=S-1). Writes repeat_XXXX/ under --out-dir,
                       then summary_merged.csv and summary_merged.txt under --out-dir.
                       Each repeat's summary.csv is post-corrected when fuzz_report_dir is NA so
                       comm_fuzz columns match the fixed CLI (see header comment).

Single-batch mode:
  (default when --comm-fuzz-seed is omitted)
  --seed-base B        Passed as run_headless arg 8; round i seed = B + i. One batch: OUT_DIR/<BATCH_TS>/
                       After the run, summary.csv rows with fuzz_report_dir=NA get comm_fuzz_seed
                       set to B + round and other comm_fuzz fields from CLI (factor only for fixed
                       delay|jitter|noise when matching env vars are set). random|pure_random|mixed:
                       factor columns are not overwritten.

Common options:
  --runs N             Repeat mode: number of iterations. Single-batch: N rounds in one batch (default 1)
  --run-sec SEC        Per roslaunch wall time (default 60)
  --out-dir DIR        Parent output directory under main_ws/src (default headless_fixed_fuzz)
  --collision-monitor true|false
  --comm-fuzz true|false
  --mode MODE          delay|jitter|noise|random|pure_random|mixed (default noise)
  --target-drone ID    default 3
  --start-sec S        Fuzz window start; passed as min=max to run_headless
  --duration-sec D     Fuzz window duration; passed as min=max
  --seed-gen-mode M    pure_random | ripple_biased (default pure_random)
  --key-time SEC       Optional 14th positional arg to run_headless (default 12)

Strength (exported as COMM_FUZZ_*; for fixed noise use single value):
  --noise-sigma V      Sets COMM_FUZZ_NOISE_SIGMA_POS_MIN and MAX to V (recommended for mode=noise)
  --noise-range LO HI  Or set min/max separately
  --delay-range LO HI  COMM_FUZZ_DELAY_MIN_MS / MAX_MS
  --jitter-range LO HI COMM_FUZZ_JITTER_MIN_MS / MAX_MS

  Any env vars you export before this script (e.g. ENABLE_COMM_FULL_DUMP_SIDECAR=true) are kept.

Examples:
  # 100 identical seeds (26008568), noise 0.389, window 6.24s / 37.86s, merged CSV under ./headless_r57/
  ./run_headless_fixed_fuzz.sh --runs 100 --run-sec 60 --out-dir headless_r57 \\
    --collision-monitor true --comm-fuzz true --mode noise --target-drone 3 \\
    --start-sec 6.24 --duration-sec 37.86 --noise-sigma 0.389 \\
    --comm-fuzz-seed 26008568 --seed-gen-mode pure_random

  # Single batch: one OUT_DIR/<BATCH_TS>/, seeds = 26008511 + round_index
  ./run_headless_fixed_fuzz.sh --runs 100 --seed-base 26008511 \\
    --start-sec 6.24 --duration-sec 37.86 --noise-sigma 0.389

EOF
}

to_lower() {
  echo "${1:-}" | tr '[:upper:]' '[:lower:]'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runs)
      RUNS="${2:?}"
      shift 2
      ;;
    --run-sec)
      RUN_SEC="${2:?}"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="${2:?}"
      shift 2
      ;;
    --collision-monitor)
      COLLISION_MONITOR="${2:?}"
      shift 2
      ;;
    --comm-fuzz)
      COMM_FUZZ="${2:?}"
      shift 2
      ;;
    --mode)
      MODE="${2:?}"
      shift 2
      ;;
    --target-drone)
      TARGET_DRONE="${2:?}"
      shift 2
      ;;
    --start-sec)
      START_SEC="${2:?}"
      shift 2
      ;;
    --duration-sec)
      DURATION_SEC="${2:?}"
      shift 2
      ;;
    --seed-base)
      SEED_BASE="${2:?}"
      shift 2
      ;;
    --comm-fuzz-seed)
      COMM_FUZZ_SEED="${2:?}"
      shift 2
      ;;
    --seed-gen-mode)
      SEED_GEN_MODE="${2:?}"
      shift 2
      ;;
    --key-time)
      KEY_TIME_SEC="${2:?}"
      shift 2
      ;;
    --noise-sigma)
      NOISE_SIGMA="${2:?}"
      shift 2
      ;;
    --noise-range)
      NOISE_MIN="${2:?}"
      NOISE_MAX="${3:?}"
      shift 3
      ;;
    --delay-range)
      DELAY_MIN_MS="${2:?}"
      DELAY_MAX_MS="${3:?}"
      shift 3
      ;;
    --jitter-range)
      JITTER_MIN_MS="${2:?}"
      JITTER_MAX_MS="${3:?}"
      shift 3
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "$RUN_HEADLESS" ]]; then
  echo "ERROR: run_headless.sh not found at $RUN_HEADLESS" >&2
  exit 1
fi

if ! [[ "$RUNS" =~ ^[0-9]+$ ]] || [[ "$RUNS" -lt 1 ]]; then
  echo "ERROR: --runs must be a positive integer" >&2
  exit 1
fi

# Strength env exports
if [[ -n "$NOISE_SIGMA" ]]; then
  export COMM_FUZZ_NOISE_SIGMA_POS_MIN="$NOISE_SIGMA"
  export COMM_FUZZ_NOISE_SIGMA_POS_MAX="$NOISE_SIGMA"
fi
if [[ -n "$NOISE_MIN" && -n "$NOISE_MAX" ]]; then
  export COMM_FUZZ_NOISE_SIGMA_POS_MIN="$NOISE_MIN"
  export COMM_FUZZ_NOISE_SIGMA_POS_MAX="$NOISE_MAX"
fi
if [[ -n "$DELAY_MIN_MS" && -n "$DELAY_MAX_MS" ]]; then
  export COMM_FUZZ_DELAY_MIN_MS="$DELAY_MIN_MS"
  export COMM_FUZZ_DELAY_MAX_MS="$DELAY_MAX_MS"
fi
if [[ -n "$JITTER_MIN_MS" && -n "$JITTER_MAX_MS" ]]; then
  export COMM_FUZZ_JITTER_MIN_MS="$JITTER_MIN_MS"
  export COMM_FUZZ_JITTER_MAX_MS="$JITTER_MAX_MS"
fi

# Window: min=max
S_MIN="$START_SEC"
S_MAX="$START_SEC"
D_MIN="$DURATION_SEC"
D_MAX="$DURATION_SEC"

# Pick newest "$parent"/*/<basename> by mtime (same repeat dir may hold multiple BATCH_TS runs).
latest_file_under_batch_dirs() {
  local parent="$1"
  local basename="$2"
  shopt -s nullglob
  local -a files=( "$parent"/*/"$basename" )
  if [[ ${#files[@]} -eq 0 ]]; then
    return 1
  fi
  local best="" best_t=0 mt f
  for f in "${files[@]}"; do
    mt=$(stat -c %Y "$f" 2>/dev/null) || mt=0
    if [[ "$mt" -gt "$best_t" ]]; then
      best_t=$mt
      best=$f
    fi
  done
  [[ -n "$best" ]] || return 1
  printf '%s\n' "$best"
}

invoke_run_headless() {
  local runs="$1"
  local out="$2"
  local seed_base_arg="$3"
  bash "$RUN_HEADLESS" \
    "$runs" \
    "$RUN_SEC" \
    "$out" \
    "$(to_lower "$COLLISION_MONITOR")" \
    "$(to_lower "$COMM_FUZZ")" \
    "$MODE" \
    "$TARGET_DRONE" \
    "$seed_base_arg" \
    "$S_MIN" \
    "$S_MAX" \
    "$D_MIN" \
    "$D_MAX" \
    "$SEED_GEN_MODE" \
    "$KEY_TIME_SEC"
}

# Rewrite round rows where run_headless lost comm_fuzz_meta (fuzz_report_dir=NA).
# Env: FIX_SUMMARY_CSV, FIX_MODE, FIX_TARGET_DRONE, FIX_START_SEC, FIX_DURATION_SEC,
#      FIX_COMM_FUZZ_REPEAT_SEED (optional), FIX_SEED_BASE (optional, per-row seed = base + round).
fix_summary_comm_fuzz_csv() {
  local csv_path="$1"
  [[ -f "$csv_path" ]] || return 0
  if [[ "$(to_lower "$COMM_FUZZ")" != "true" ]]; then
    return 0
  fi
  FIX_SUMMARY_CSV="$csv_path" \
  FIX_MODE="$MODE" \
  FIX_TARGET_DRONE="$TARGET_DRONE" \
  FIX_START_SEC="$START_SEC" \
  FIX_DURATION_SEC="$DURATION_SEC" \
  FIX_COMM_FUZZ_REPEAT_SEED="${FIX_COMM_FUZZ_REPEAT_SEED:-}" \
  FIX_SEED_BASE="${FIX_SEED_BASE:-}" \
  python3 <<'PY'
import csv
import os

path = os.environ["FIX_SUMMARY_CSV"]
mode = os.environ.get("FIX_MODE", "noise").strip().lower()
target = os.environ["FIX_TARGET_DRONE"]
start_s = os.environ["FIX_START_SEC"]
dur_s = os.environ["FIX_DURATION_SEC"]
repeat_seed = os.environ.get("FIX_COMM_FUZZ_REPEAT_SEED", "").strip()
seed_base_s = os.environ.get("FIX_SEED_BASE", "").strip()
mixed_modes = {"random", "pure_random", "mixed"}

rows = []
with open(path, newline="") as f:
    reader = csv.DictReader(f)
    fieldnames = reader.fieldnames
    if not fieldnames:
        raise SystemExit(0)
    for row in reader:
        rtype = (row.get("row_type") or "").strip().lower()
        fuzz_on = (row.get("comm_fuzz_enabled") or "").strip().lower()
        rep_dir = (row.get("fuzz_report_dir") or "").strip()
        if rtype != "round" or fuzz_on != "yes":
            rows.append(row)
            continue
        if rep_dir and rep_dir.upper() != "NA":
            rows.append(row)
            continue
        row["comm_fuzz_mode"] = mode
        row["comm_fuzz_target_drone_id"] = str(target)
        row["comm_fuzz_start_s"] = start_s
        row["comm_fuzz_duration_s"] = dur_s
        if repeat_seed:
            row["comm_fuzz_seed"] = repeat_seed
        elif seed_base_s != "":
            try:
                ri = int(str(row.get("round", "0")).strip().strip('"'))
                row["comm_fuzz_seed"] = str(int(seed_base_s) + ri)
            except ValueError:
                pass
        if mode not in mixed_modes:
            if mode == "noise":
                v = os.environ.get("COMM_FUZZ_NOISE_SIGMA_POS_MIN")
                if v:
                    row["comm_fuzz_factor_name"] = "noise_sigma_pos"
                    row["comm_fuzz_factor_value"] = v
            elif mode == "delay":
                v = os.environ.get("COMM_FUZZ_DELAY_MIN_MS")
                if v:
                    row["comm_fuzz_factor_name"] = "delay_ms"
                    row["comm_fuzz_factor_value"] = v
            elif mode == "jitter":
                v = os.environ.get("COMM_FUZZ_JITTER_MIN_MS")
                if v:
                    row["comm_fuzz_factor_name"] = "jitter_ms"
                    row["comm_fuzz_factor_value"] = v
        rows.append(row)

with open(path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    for row in rows:
        w.writerow(row)
PY
}

merge_summaries() {
  local parent="$1"
  shift
  local -a subdirs=("$@")
  local merged_csv="$parent/summary_merged.csv"
  local merged_txt="$parent/summary_merged.txt"
  local first=1

  : >"$merged_txt"
  : >"${merged_csv}.tmp"

  local idx=0
  for sub in "${subdirs[@]}"; do
    idx=$((idx + 1))
    {
      echo "--- repeat ${idx} (${sub}) ---"
      local sum_txt
      if sum_txt=$(latest_file_under_batch_dirs "$sub" summary.txt); then
        head -n 40 "$sum_txt"
      else
        echo "(no summary.txt)"
      fi
      echo ""
    } >>"$merged_txt"

    local cf
    if ! cf=$(latest_file_under_batch_dirs "$sub" summary.csv); then
      echo "WARN: no summary.csv under $sub" >&2
      continue
    fi
    if [[ "$first" -eq 1 ]]; then
      cat "$cf" >>"${merged_csv}.tmp"
      first=0
    else
      tail -n +2 "$cf" >>"${merged_csv}.tmp"
    fi
  done

  mv "${merged_csv}.tmp" "$merged_csv"
  echo "Wrote $merged_csv and $merged_txt"
}

cd "$SCRIPT_DIR"

if [[ -n "$COMM_FUZZ_SEED" ]]; then
  # Repeat mode: same comm_fuzz_seed every iteration
  if ! [[ "$COMM_FUZZ_SEED" =~ ^[0-9]+$ ]]; then
    echo "ERROR: --comm-fuzz-seed must be a non-negative integer" >&2
    exit 1
  fi
  if [[ "$COMM_FUZZ_SEED" -lt 1 ]]; then
    echo "ERROR: --comm-fuzz-seed must be >= 1 (need BASE = seed - 1 >= 0)" >&2
    exit 1
  fi
  PARENT_OUT="$OUT_DIR"
  mkdir -p "$PARENT_OUT"
  BATCH_SUBDIRS=()
  SEED_BASE_USED=$((COMM_FUZZ_SEED - 1))
  echo "[run_headless_fixed_fuzz] repeat mode: iterations=$RUNS comm_fuzz_seed=$COMM_FUZZ_SEED (BASE=$SEED_BASE_USED) parent_out=$PARENT_OUT"
  export FIX_COMM_FUZZ_REPEAT_SEED="$COMM_FUZZ_SEED"
  unset FIX_SEED_BASE
  for k in $(seq -f "%04g" 1 "$RUNS"); do
    SUB_OUT="${PARENT_OUT}/repeat_${k}"
    mkdir -p "$SUB_OUT"
    echo "[run_headless_fixed_fuzz] === repeat $k / $RUNS -> $SUB_OUT ==="
    invoke_run_headless 1 "$SUB_OUT" "$SEED_BASE_USED"
    latest_csv=""
    if latest_csv=$(latest_file_under_batch_dirs "$SUB_OUT" summary.csv); then
      fix_summary_comm_fuzz_csv "$latest_csv"
    fi
    BATCH_SUBDIRS+=( "$SUB_OUT" )
  done
  unset FIX_COMM_FUZZ_REPEAT_SEED
  merge_summaries "$PARENT_OUT" "${BATCH_SUBDIRS[@]}"
else
  # Single batch
  if [[ -z "$SEED_BASE" ]]; then
    echo "ERROR: single-batch mode requires --seed-base (or use --comm-fuzz-seed for repeat mode)" >&2
    exit 1
  fi
  if ! [[ "$SEED_BASE" =~ ^[0-9]+$ ]]; then
    echo "ERROR: --seed-base must be a non-negative integer" >&2
    exit 1
  fi
  echo "[run_headless_fixed_fuzz] single batch: runs=$RUNS seed_base=$SEED_BASE out=$OUT_DIR"
  unset FIX_COMM_FUZZ_REPEAT_SEED
  export FIX_SEED_BASE="$SEED_BASE"
  invoke_run_headless "$RUNS" "$OUT_DIR" "$SEED_BASE"
  shopt -s nullglob
  for batch_csv in "$OUT_DIR"/*/summary.csv; do
    fix_summary_comm_fuzz_csv "$batch_csv"
  done
  unset FIX_SEED_BASE
fi
