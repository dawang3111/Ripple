#!/usr/bin/env bash
# Two reproducible comm-fuzz profiles (same mode=random: each round picks delay|jitter|noise).
# Defaults below = 第一组随机 fuzz 基线。第二组强化：改第 9–12 位时间窗 + 环境变量覆盖三类强度。
#
# 配置 A（基线示例，依赖默认 env）:
#   bash run_headless.sh RUNS RUN_SEC OUT_DIR true true random 3 20260407 5.0 25.0 5.0 20.0 random
#
# 配置 B（强化示例：开始 3–8s，持续 25–40s；delay/jitter/noise 加强）:
#   COMM_FUZZ_DELAY_MIN_MS=150 COMM_FUZZ_DELAY_MAX_MS=400 \
#   COMM_FUZZ_JITTER_MIN_MS=60 COMM_FUZZ_JITTER_MAX_MS=200 \
#   COMM_FUZZ_NOISE_SIGMA_POS_MIN=0.15 COMM_FUZZ_NOISE_SIGMA_POS_MAX=0.40 \
#   bash run_headless.sh RUNS RUN_SEC OUT_DIR true true random 3 20260407 3.0 8.0 25.0 40.0 random
#
# RUN_SEC 建议 >= start_max + duration_max；baseline 默认 60s。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

RUNS=${1:-5}
RUN_SEC=${2:-60}
OUT_DIR=${3:-"$WORKSPACE_DIR/results/headless"}
ENABLE_COLLISION_MONITOR=${4:-false}
ENABLE_COMM_FUZZ=${5:-false}
COMM_FUZZ_MODE=${6:-random}
COMM_FUZZ_TARGET_DRONE_ID=${7:-3}
COMM_FUZZ_SEED_BASE=${8:-20260407}
# Optional comm-fuzz range overrides for stronger A/B experiments.
# Priority: CLI args > env vars > defaults.
COMM_FUZZ_START_MIN_SEC=${9:-${COMM_FUZZ_START_MIN_SEC:-5.0}}
COMM_FUZZ_START_MAX_SEC=${10:-${COMM_FUZZ_START_MAX_SEC:-25.0}}
COMM_FUZZ_DURATION_MIN_SEC=${11:-${COMM_FUZZ_DURATION_MIN_SEC:-5.0}}
COMM_FUZZ_DURATION_MAX_SEC=${12:-${COMM_FUZZ_DURATION_MAX_SEC:-20.0}}
COMM_FUZZ_SEED_GEN_MODE=${13:-${COMM_FUZZ_SEED_GEN_MODE:-pure_random}}
COMM_FUZZ_KEY_TIME_SEC=${14:-${COMM_FUZZ_KEY_TIME_SEC:-12.0}}
COMM_FUZZ_DELAY_MIN_MS=${COMM_FUZZ_DELAY_MIN_MS:-80.0}
COMM_FUZZ_DELAY_MAX_MS=${COMM_FUZZ_DELAY_MAX_MS:-250.0}
COMM_FUZZ_JITTER_MIN_MS=${COMM_FUZZ_JITTER_MIN_MS:-30.0}
COMM_FUZZ_JITTER_MAX_MS=${COMM_FUZZ_JITTER_MAX_MS:-120.0}
COMM_FUZZ_NOISE_SIGMA_POS_MIN=${COMM_FUZZ_NOISE_SIGMA_POS_MIN:-0.05}
COMM_FUZZ_NOISE_SIGMA_POS_MAX=${COMM_FUZZ_NOISE_SIGMA_POS_MAX:-0.20}
COMM_FUZZ_RIPPLE_WARMUP_ROUNDS=${COMM_FUZZ_RIPPLE_WARMUP_ROUNDS:-0}
COMM_FUZZ_RIPPLE_WARMUP_RATIO=${COMM_FUZZ_RIPPLE_WARMUP_RATIO:-0.20}
COMM_FUZZ_RIPPLE_WARMUP_MIN_ROUNDS=${COMM_FUZZ_RIPPLE_WARMUP_MIN_ROUNDS:-6}
COMM_FUZZ_RIPPLE_CANDIDATES=${COMM_FUZZ_RIPPLE_CANDIDATES:-24}
COMM_FUZZ_RIPPLE_ELITE_RATIO=${COMM_FUZZ_RIPPLE_ELITE_RATIO:-0.20}
COMM_FUZZ_POOL_KEEP_RATIO=${COMM_FUZZ_POOL_KEEP_RATIO:-$COMM_FUZZ_RIPPLE_ELITE_RATIO}
COMM_FUZZ_POOL_MUTATION_PER_ROUND=${COMM_FUZZ_POOL_MUTATION_PER_ROUND:-24}
ENABLE_COMM_FULL_DUMP_SIDECAR=${ENABLE_COMM_FULL_DUMP_SIDECAR:-false}
COMM_FULL_DUMP_BUFFER_MAX=${COMM_FULL_DUMP_BUFFER_MAX:-4000}
COMM_FULL_DUMP_FILE_NAME=${COMM_FULL_DUMP_FILE_NAME:-comm_full_dump_on_incident.jsonl}
ENABLE_STL_METRICS_MONITOR=${ENABLE_STL_METRICS_MONITOR:-true}
STL_WINDOW_SEC=${STL_WINDOW_SEC:-5.0}
STL_MOVE_THRESHOLD_X=${STL_MOVE_THRESHOLD_X:-0.2}
STL_DELAY_TIMEOUT_THRESHOLD_S=${STL_DELAY_TIMEOUT_THRESHOLD_S:-120.0}
STL_ACTIVE_SPEED_EPS=${STL_ACTIVE_SPEED_EPS:-0.05}
STL_START_GRACE_SEC=${STL_START_GRACE_SEC:-2.0}
STL_DRIFT_EPS=${STL_DRIFT_EPS:-0.25}
STL_FUTURE_HORIZON_SEC=${STL_FUTURE_HORIZON_SEC:-3.0}
STL_CHECK_HZ=${STL_CHECK_HZ:-2.0}
ENABLE_ATTACK_PLAN_LATENCY_PROXY=${ENABLE_ATTACK_PLAN_LATENCY_PROXY:-false}
ATTACK_PLAN_FILE=${ATTACK_PLAN_FILE:-}
ENABLE_ADAPTIVE_REPLAN_LATENCY_PROXY=${ENABLE_ADAPTIVE_REPLAN_LATENCY_PROXY:-false}
ADAPTIVE_REPLAN_ATTACK_CONFIG=${ADAPTIVE_REPLAN_ATTACK_CONFIG:-}
ADAPTIVE_MAX_INJECTIONS_PER_ROUND=${ADAPTIVE_MAX_INJECTIONS_PER_ROUND:-5}
ADAPTIVE_COOLDOWN_S=${ADAPTIVE_COOLDOWN_S:-2.0}
ADAPTIVE_TRIGGER_WINDOW_S=${ADAPTIVE_TRIGGER_WINDOW_S:-1.0}
ADAPTIVE_FUTURE_HORIZON_S=${ADAPTIVE_FUTURE_HORIZON_S:-3.0}
ADAPTIVE_FUTURE_SAMPLE_DT=${ADAPTIVE_FUTURE_SAMPLE_DT:-0.1}
ADAPTIVE_INTERACTION_MARGIN_THRESHOLD=${ADAPTIVE_INTERACTION_MARGIN_THRESHOLD:-0.25}
ADAPTIVE_INTERACTION_SCORE_THRESHOLD=${ADAPTIVE_INTERACTION_SCORE_THRESHOLD:-0.6}
ADAPTIVE_LATENCY_FACTOR=${ADAPTIVE_LATENCY_FACTOR:-1.2}
ADAPTIVE_LATENCY_MIN_MS=${ADAPTIVE_LATENCY_MIN_MS:-80.0}
ADAPTIVE_LATENCY_MAX_MS=${ADAPTIVE_LATENCY_MAX_MS:-250.0}
ADAPTIVE_PERIOD_FALLBACK_MS=${ADAPTIVE_PERIOD_FALLBACK_MS:-120.0}
ADAPTIVE_SEED=${ADAPTIVE_SEED:-20260917}
ENABLE_RULE_POOL_LATENCY_PROXY=${ENABLE_RULE_POOL_LATENCY_PROXY:-false}
RULE_POOL_ACTIVE_RULE_ID=${RULE_POOL_ACTIVE_RULE_ID:-R1}
RULE_POOL_SCREENING_MODE=${RULE_POOL_SCREENING_MODE:-false}
RULE_POOL_MONITOR_ONLY=${RULE_POOL_MONITOR_ONLY:-false}
RULE_POOL_MAX_INJECTIONS_PER_ROUND=${RULE_POOL_MAX_INJECTIONS_PER_ROUND:-5}
RULE_POOL_COOLDOWN_S=${RULE_POOL_COOLDOWN_S:-1.0}
RULE_POOL_HISTORY_WINDOW_S=${RULE_POOL_HISTORY_WINDOW_S:-5.0}
RULE_POOL_TRIGGER_WINDOW_S=${RULE_POOL_TRIGGER_WINDOW_S:-1.0}
RULE_POOL_FUTURE_HORIZON_S=${RULE_POOL_FUTURE_HORIZON_S:-3.0}
RULE_POOL_FUTURE_SAMPLE_DT=${RULE_POOL_FUTURE_SAMPLE_DT:-0.1}
RULE_POOL_HEADING_ALIGNED_DEG=${RULE_POOL_HEADING_ALIGNED_DEG:-20.0}
RULE_POOL_FUTURE_COLLISION_MARGIN=${RULE_POOL_FUTURE_COLLISION_MARGIN:-0.0}
RULE_POOL_LATENCY_EPSILON_MS=${RULE_POOL_LATENCY_EPSILON_MS:-10.0}
RULE_POOL_LATENCY_MIN_MS=${RULE_POOL_LATENCY_MIN_MS:-40.0}
RULE_POOL_LATENCY_MAX_MS=${RULE_POOL_LATENCY_MAX_MS:-0.0}
RULE_POOL_PROTOCOL_TIMEOUT_MS=${RULE_POOL_PROTOCOL_TIMEOUT_MS:-1000.0}
RULE_POOL_MAX_LATENCY_RATIO=${RULE_POOL_MAX_LATENCY_RATIO:-0.8}
RULE_POOL_PERIOD_FALLBACK_MS=${RULE_POOL_PERIOD_FALLBACK_MS:-120.0}
BATCH_TS=$(date +%Y%m%d_%H%M%S)
BATCH_DIR="$OUT_DIR/$BATCH_TS"
BATCH_STL_ATTACK_SCENARIOS_FILE="$BATCH_DIR/stl_attack_scenarios.csv"

cd "$SCRIPT_DIR"

if [ ! -f ../devel/setup.bash ]; then
  echo "ERROR: ../devel/setup.bash not found"
  exit 1
fi
source ../devel/setup.bash

mkdir -p "$BATCH_DIR"
SUMMARY="$BATCH_DIR/summary.txt"
SUMMARY_CSV="$BATCH_DIR/summary.csv"

to_bool() {
  local v
  v=$(echo "${1:-false}" | tr '[:upper:]' '[:lower:]')
  case "$v" in
    1|true|yes|on) echo "true" ;;
    *) echo "false" ;;
  esac
}

COLLISION_MONITOR_FLAG=$(to_bool "$ENABLE_COLLISION_MONITOR")
if [ "$COLLISION_MONITOR_FLAG" = "true" ]; then
  COLLISION_MONITOR_ENABLED="yes"
else
  COLLISION_MONITOR_ENABLED="no"
fi

COMM_FUZZ_FLAG=$(to_bool "$ENABLE_COMM_FUZZ")
if [ "$COMM_FUZZ_FLAG" = "true" ]; then
  COMM_FUZZ_ENABLED="yes"
else
  COMM_FUZZ_ENABLED="no"
fi

COMM_FULL_DUMP_SIDECAR_FLAG=$(to_bool "$ENABLE_COMM_FULL_DUMP_SIDECAR")
if [ "$COMM_FULL_DUMP_SIDECAR_FLAG" = "true" ]; then
  COMM_FULL_DUMP_SIDECAR_ENABLED="yes"
else
  COMM_FULL_DUMP_SIDECAR_ENABLED="no"
fi

STL_METRICS_MONITOR_FLAG=$(to_bool "$ENABLE_STL_METRICS_MONITOR")
if [ "$STL_METRICS_MONITOR_FLAG" = "true" ]; then
  STL_METRICS_MONITOR_ENABLED="yes"
else
  STL_METRICS_MONITOR_ENABLED="no"
fi

ATTACK_PLAN_LATENCY_PROXY_FLAG=$(to_bool "$ENABLE_ATTACK_PLAN_LATENCY_PROXY")
if [ "$ATTACK_PLAN_LATENCY_PROXY_FLAG" = "true" ]; then
  ATTACK_PLAN_LATENCY_PROXY_ENABLED="yes"
else
  ATTACK_PLAN_LATENCY_PROXY_ENABLED="no"
fi

ADAPTIVE_REPLAN_LATENCY_PROXY_FLAG=$(to_bool "$ENABLE_ADAPTIVE_REPLAN_LATENCY_PROXY")
if [ "$ADAPTIVE_REPLAN_LATENCY_PROXY_FLAG" = "true" ]; then
  ADAPTIVE_REPLAN_LATENCY_PROXY_ENABLED="yes"
else
  ADAPTIVE_REPLAN_LATENCY_PROXY_ENABLED="no"
fi

RULE_POOL_LATENCY_PROXY_FLAG=$(to_bool "$ENABLE_RULE_POOL_LATENCY_PROXY")
if [ "$RULE_POOL_LATENCY_PROXY_FLAG" = "true" ]; then
  RULE_POOL_LATENCY_PROXY_ENABLED="yes"
else
  RULE_POOL_LATENCY_PROXY_ENABLED="no"
fi

if [ "$ATTACK_PLAN_LATENCY_PROXY_FLAG" = "true" ] || [ "$ADAPTIVE_REPLAN_LATENCY_PROXY_FLAG" = "true" ] || [ "$RULE_POOL_LATENCY_PROXY_FLAG" = "true" ]; then
  ATTACK_PLAN_TX_BRIDGE_INPUT_TOPIC="/broadcast_traj_from_planner_attacked"
else
  ATTACK_PLAN_TX_BRIDGE_INPUT_TOPIC="/broadcast_traj_from_planner"
fi

# Any match here marks a round as FAIL.
FATAL_PATTERN='Eigen::.*Assertion|Assertion .+ failed|boost::wrapexcept<boost::lock_error>|terminate called after throwing|Segmentation fault|core dumped|process monitor has been shut down|RLException: cannot add process'
SHUTDOWN_PATTERN='killing on exit|shutting down processing monitor|Shutdown errors:|escalating to SIGTERM|escalating to SIGKILL'
SHUTDOWN_MARK='__RUN_HEADLESS_SHUTDOWN_BEGIN__'

strip_ansi() {
  sed -r 's/\x1B\[[0-9;]*[mK]//g'
}

extract_runtime_log() {
  local in_file="$1"
  local out_file="$2"
  local mark_line
  local cut_line

  mark_line=$(grep -Enim1 "$SHUTDOWN_MARK" "$in_file" | cut -d: -f1 || true)
  if [ -n "${mark_line:-}" ] && [ "$mark_line" -gt 1 ]; then
    head -n "$((mark_line - 1))" "$in_file" > "$out_file"
    return
  fi

  cut_line=$(grep -Enim1 "$SHUTDOWN_PATTERN" "$in_file" | cut -d: -f1 || true)
  if [ -n "${cut_line:-}" ] && [ "$cut_line" -gt 1 ]; then
    head -n "$((cut_line - 1))" "$in_file" > "$out_file"
  else
    cp "$in_file" "$out_file"
  fi
}

extract_shutdown_log() {
  local in_file="$1"
  local out_file="$2"
  local mark_line
  local cut_line

  mark_line=$(grep -Enim1 "$SHUTDOWN_MARK" "$in_file" | cut -d: -f1 || true)
  if [ -n "${mark_line:-}" ]; then
    tail -n "+$mark_line" "$in_file" > "$out_file"
    return
  fi

  cut_line=$(grep -Enim1 "$SHUTDOWN_PATTERN" "$in_file" | cut -d: -f1 || true)
  if [ -n "${cut_line:-}" ]; then
    tail -n "+$cut_line" "$in_file" > "$out_file"
  else
    : > "$out_file"
  fi
}

csv_escape() {
  local s
  s=${1:-}
  s=${s//\"/\"\"}
  printf '"%s"' "$s"
}

is_number() {
  [[ "${1:-}" =~ ^[0-9]+([.][0-9]+)?$ ]]
}

safe_avg() {
  local sum="${1:-0}"
  local cnt="${2:-0}"
  if [ "${cnt}" -le 0 ]; then
    echo "NA"
    return
  fi
  awk -v s="$sum" -v c="$cnt" 'BEGIN { printf "%.3f", s / c }'
}

write_missing_stl_attack_scenarios() {
  local out_file="$1"
  mkdir -p "$(dirname "$out_file")"
  {
    echo "scenario_id,scenario_name,stl_value_avg,attacker_drone_id,victim_drone_id,bystander_drone_ids,evidence_time_offset_avg_s,victim_fsm_state,round_count"
    echo "1,drone_drone_collision,NA,NA,NA,NA,NA,NA,0"
    echo "2,drone_obstacle_collision,NA,NA,NA,NA,NA,NA,0"
    echo "3,deadlock,NA,NA,NA,NA,NA,NA,0"
    echo "4,delay_timeout,NA,NA,NA,NA,NA,NA,0"
    echo "5,planning_failure,NA,NA,NA,NA,NA,NA,0"
    echo "6,drift,NA,NA,NA,NA,NA,NA,0"
  } > "$out_file"
}

normalize_seed_mode() {
  local v
  v=$(echo "${1:-pure_random}" | tr '[:upper:]' '[:lower:]')
  case "$v" in
  pure_random|random|uniform) echo "pure_random" ;;
  ripple|ripple_biased|biased) echo "ripple_biased" ;;
  *) echo "pure_random" ;;
  esac
}

choose_seed_pure_random() {
  local round_idx="${1:-1}"
  echo $((COMM_FUZZ_SEED_BASE + round_idx))
}

sample_mixed_comm_plan() {
  local round_idx="${1:-1}"
  local seed_hint="$((COMM_FUZZ_SEED_BASE + round_idx * 7919))"

  python3 - "$seed_hint" "$COMM_FUZZ_START_MIN_SEC" "$COMM_FUZZ_START_MAX_SEC" "$COMM_FUZZ_DURATION_MIN_SEC" "$COMM_FUZZ_DURATION_MAX_SEC" "$COMM_FUZZ_DELAY_MIN_MS" "$COMM_FUZZ_DELAY_MAX_MS" "$COMM_FUZZ_JITTER_MIN_MS" "$COMM_FUZZ_JITTER_MAX_MS" "$COMM_FUZZ_NOISE_SIGMA_POS_MIN" "$COMM_FUZZ_NOISE_SIGMA_POS_MAX" <<'PY'
import random
import sys

seed_hint = int(float(sys.argv[1]))
start_min = float(sys.argv[2])
start_max = float(sys.argv[3])
dur_min = float(sys.argv[4])
dur_max = float(sys.argv[5])
delay_min = float(sys.argv[6])
delay_max = float(sys.argv[7])
jitter_min = float(sys.argv[8])
jitter_max = float(sys.argv[9])
noise_min = float(sys.argv[10])
noise_max = float(sys.argv[11])

def clamp(v, lo, hi):
  return max(lo, min(hi, v))

if start_min > start_max:
  start_min, start_max = start_max, start_min
if dur_min > dur_max:
  dur_min, dur_max = dur_max, dur_min
if delay_min > delay_max:
  delay_min, delay_max = delay_max, delay_min
if jitter_min > jitter_max:
  jitter_min, jitter_max = jitter_max, jitter_min
if noise_min > noise_max:
  noise_min, noise_max = noise_max, noise_min

rng = random.Random(seed_hint)
mode = rng.choice(["delay", "jitter", "noise"])
start_s = rng.uniform(start_min, start_max)
duration_s = rng.uniform(dur_min, dur_max)

if mode == "delay":
  factor_name = "delay_ms"
  factor_value = rng.uniform(delay_min, delay_max)
elif mode == "jitter":
  factor_name = "jitter_ms"
  factor_value = rng.uniform(jitter_min, jitter_max)
else:
  factor_name = "noise_sigma_pos"
  factor_value = rng.uniform(noise_min, noise_max)

print("\t".join([
  mode,
  f"{clamp(start_s, start_min, start_max):.6f}",
  f"{clamp(duration_s, dur_min, dur_max):.6f}",
  factor_name,
  f"{factor_value:.6f}",
]))
PY
}

choose_seed_pool_param_plan() {
  local round_idx="${1:-1}"
  local pool_file="${2:-}"
  local used_seed_file="${3:-}"
  local warmup_rounds="${4:-1}"

  python3 - "$round_idx" "$COMM_FUZZ_SEED_BASE" "$COMM_FUZZ_MODE" "$COMM_FUZZ_START_MIN_SEC" "$COMM_FUZZ_START_MAX_SEC" "$COMM_FUZZ_DURATION_MIN_SEC" "$COMM_FUZZ_DURATION_MAX_SEC" "$COMM_FUZZ_DELAY_MIN_MS" "$COMM_FUZZ_DELAY_MAX_MS" "$COMM_FUZZ_JITTER_MIN_MS" "$COMM_FUZZ_JITTER_MAX_MS" "$COMM_FUZZ_NOISE_SIGMA_POS_MIN" "$COMM_FUZZ_NOISE_SIGMA_POS_MAX" "$pool_file" "$used_seed_file" "$warmup_rounds" <<'PY'
import random
import sys

round_idx = int(sys.argv[1])
seed_base = int(float(sys.argv[2]))
mode_cfg = str(sys.argv[3]).strip().lower()
start_min = float(sys.argv[4])
start_max = float(sys.argv[5])
dur_min = float(sys.argv[6])
dur_max = float(sys.argv[7])
delay_min = float(sys.argv[8])
delay_max = float(sys.argv[9])
jitter_min = float(sys.argv[10])
jitter_max = float(sys.argv[11])
noise_min = float(sys.argv[12])
noise_max = float(sys.argv[13])
pool_file = str(sys.argv[14])
used_seed_file = str(sys.argv[15])
warmup_rounds = max(1, int(float(sys.argv[16])))

def clamp(v, lo, hi):
  return max(lo, min(hi, v))

if start_min > start_max:
  start_min, start_max = start_max, start_min
if dur_min > dur_max:
  dur_min, dur_max = dur_max, dur_min
if delay_min > delay_max:
  delay_min, delay_max = delay_max, delay_min
if jitter_min > jitter_max:
  jitter_min, jitter_max = jitter_max, jitter_min
if noise_min > noise_max:
  noise_min, noise_max = noise_max, noise_min

if round_idx <= warmup_rounds:
  print("")
  sys.exit(0)

if mode_cfg == "delay":
  allowed_modes = ["delay"]
elif mode_cfg == "jitter":
  allowed_modes = ["jitter"]
elif mode_cfg == "noise":
  allowed_modes = ["noise"]
else:
  allowed_modes = ["delay", "jitter", "noise"]

def param_key(mode, start_s, duration_s, factor_name, factor_value):
  return f"{mode}|{start_s:.3f}|{duration_s:.3f}|{factor_name}|{factor_value:.4f}"

used_seeds = set()
if used_seed_file and used_seed_file not in ("", "NA"):
  try:
    with open(used_seed_file, "r", encoding="ascii", errors="ignore") as f:
      for ln in f:
        ln = ln.strip()
        if not ln:
          continue
        try:
          used_seeds.add(int(ln))
        except Exception:
          pass
  except FileNotFoundError:
    pass

pool = []
if pool_file and pool_file not in ("", "NA"):
  try:
    with open(pool_file, "r", encoding="ascii", errors="ignore") as f:
      for ln in f:
        parts = ln.strip().split("\t")
        if not parts or parts[0] == "param_key":
          continue
        if len(parts) < 11:
          continue
        try:
          mode = parts[1].strip().lower()
          start_s = float(parts[2])
          duration_s = float(parts[3])
          factor_name = parts[4].strip().lower()
          factor_value = float(parts[5])
          anomaly_level = parts[6].strip().lower()
        except Exception:
          continue
        if mode not in allowed_modes:
          continue
        if anomaly_level not in ("high", "mid", "low"):
          continue
        pool.append((mode, start_s, duration_s, factor_name, factor_value, anomaly_level))
  except FileNotFoundError:
    pass

if not pool:
  print("")
  sys.exit(0)

def choose_bucket(rng, buckets):
  probs = {"high": 0.50, "mid": 0.30, "low": 0.20}
  non_empty = [k for k in ["high", "mid", "low"] if buckets[k]]
  if not non_empty:
    return None
  missing_total = sum(probs[k] for k in ["high", "mid", "low"] if not buckets[k])
  adjusted = {k: probs[k] for k in ["high", "mid", "low"]}
  if missing_total > 0.0:
    share = missing_total / float(len(non_empty))
    for k in ["high", "mid", "low"]:
      if buckets[k]:
        adjusted[k] += share
      else:
        adjusted[k] = 0.0
  r = rng.random()
  acc = 0.0
  for k in ["high", "mid", "low"]:
    acc += adjusted[k]
    if r <= acc and buckets[k]:
      return k
  return non_empty[-1]

def allocate_unique_seed(seed_hint):
  seed = int(abs(seed_hint)) & 0x7FFFFFFF
  if seed <= 0:
    seed = 1
  while seed in used_seeds:
    seed += 104729
    if seed > 0x7FFFFFFF:
      seed = (seed % 0x7FFFFFFF) + 1
  return seed

buckets = {"high": [], "mid": [], "low": []}
for row in pool:
  buckets[row[5]].append(row)

rng = random.Random(seed_base + round_idx * 7919)
source_bucket = choose_bucket(rng, buckets)
if source_bucket is None:
  print("")
  sys.exit(0)

mode, base_start, base_duration, base_factor_name, base_factor_value, base_level = rng.choice(buckets[source_bucket])
start_s = clamp(base_start + rng.uniform(-1.0, 1.0), start_min, start_max)
duration_s = clamp(base_duration + rng.uniform(-0.8, 0.8), dur_min, dur_max)

if mode == "delay":
  factor_name = "delay_ms"
  center = base_factor_value if base_factor_name == "delay_ms" else 0.5 * (delay_min + delay_max)
  factor_value = clamp(center + rng.uniform(-15.0, 15.0), delay_min, delay_max)
elif mode == "jitter":
  factor_name = "jitter_ms"
  center = base_factor_value if base_factor_name == "jitter_ms" else 0.5 * (jitter_min + jitter_max)
  factor_value = clamp(center + rng.uniform(-10.0, 10.0), jitter_min, jitter_max)
else:
  mode = "noise"
  factor_name = "noise_sigma_pos"
  center = base_factor_value if base_factor_name == "noise_sigma_pos" else 0.5 * (noise_min + noise_max)
  factor_value = clamp(center + rng.uniform(-0.01, 0.01), noise_min, noise_max)

seed_hint = seed_base + round_idx * 100003 + int(start_s * 1000) + int(duration_s * 1000) + int(factor_value * 10000)
seed = allocate_unique_seed(seed_hint)

print("\t".join([
  str(seed),
  mode,
  f"{start_s:.6f}",
  f"{duration_s:.6f}",
  factor_name,
  f"{factor_value:.6f}",
  "evolution_pool_mutation",
  source_bucket,
  base_level,
]))
PY
}

compute_round_ripple_score() {
  local emergency_count="${1:-0}"
  local collision_count="${2:-0}"
  local replan_count="${3:-0}"
  local mode_used="${4:-NA}"
  local start_s="${5:-NA}"
  local duration_s="${6:-NA}"
  local factor_name="${7:-NA}"
  local factor_value="${8:-NA}"
  local key_time="${9:-12.0}"

  python3 - "$emergency_count" "$collision_count" "$replan_count" "$mode_used" "$start_s" "$duration_s" "$factor_name" "$factor_value" "$key_time" <<'PY'
import math
import sys

def safe_float(v, default=0.0):
  try:
    return float(v)
  except Exception:
    return default

E = safe_float(sys.argv[1], 0.0)
C = safe_float(sys.argv[2], 0.0)
R = safe_float(sys.argv[3], 0.0)
mode_used = str(sys.argv[4]).strip().lower()
start_s = safe_float(sys.argv[5], float("nan"))
duration_s = safe_float(sys.argv[6], float("nan"))
factor_name = str(sys.argv[7]).strip().lower()
factor_value = safe_float(sys.argv[8], float("nan"))
key_time = safe_float(sys.argv[9], 12.0)

e_norm = min(1.0, max(0.0, E / 200.0))
c_norm = min(1.0, max(0.0, C / 2.0))
r_norm = min(1.0, max(0.0, R / 400.0))

score = 100.0 * (0.35 * e_norm + 0.35 * c_norm + 0.20 * r_norm)

if mode_used == "noise":
  score += 8.0
elif mode_used == "jitter":
  score += 5.0
elif mode_used == "delay":
  score -= 6.0

if not math.isnan(start_s):
  score += max(0.0, 6.0 - 3.0 * abs(start_s - key_time))
if not math.isnan(duration_s):
  if 5.0 <= duration_s <= 9.0:
    score += 4.0

if factor_name == "noise_sigma_pos" and not math.isnan(factor_value):
  if 0.12 <= factor_value <= 0.20:
    score += 3.0
if factor_name == "jitter_ms" and not math.isnan(factor_value):
  if 70.0 <= factor_value <= 120.0:
    score += 2.0

print(f"{score:.6f}")
PY
}

update_ripple_param_elite_file() {
  local rounds_file="${1:-}"
  local elite_file="${2:-}"
  local ratio="${3:-0.20}"

  python3 - "$rounds_file" "$elite_file" "$ratio" <<'PY'
import math
import sys

rounds_file = sys.argv[1]
elite_file = sys.argv[2]
ratio = float(sys.argv[3])

rows = []
try:
  with open(rounds_file, "r", encoding="ascii", errors="ignore") as f:
    for ln in f:
      parts = ln.strip().split("\t")
      if not parts or parts[0] == "param_key":
        continue
      if len(parts) < 9:
        continue
      key = parts[0]
      mode = parts[2]
      start_s = parts[3]
      duration_s = parts[4]
      factor_name = parts[5]
      factor_value = parts[6]
      try:
        round_score = float(parts[8])
      except Exception:
        continue
      rows.append((key, mode, start_s, duration_s, factor_name, factor_value, round_score))
except FileNotFoundError:
  rows = []

if not rows:
  open(elite_file, "w", encoding="ascii").close()
  sys.exit(0)

best = {}
for key, mode, start_s, duration_s, factor_name, factor_value, score in rows:
  prev = best.get(key)
  if prev is None or score > prev[6]:
    best[key] = (key, mode, start_s, duration_s, factor_name, factor_value, score)

items = sorted(best.values(), key=lambda x: x[6], reverse=True)
keep_n = max(3, int(math.ceil(len(items) * max(0.05, min(0.8, ratio)))))
keep_n = min(20, keep_n)
items = items[:keep_n]

with open(elite_file, "w", encoding="ascii") as f:
  for key, mode, start_s, duration_s, factor_name, factor_value, score in items:
    f.write(f"{key}\t{mode}\t{start_s}\t{duration_s}\t{factor_name}\t{factor_value}\t{score:.6f}\n")
PY
}

compute_round_tunnel_score() {
  local emergency_count="${1:-0}"
  local collision_count="${2:-0}"
  local replan_count="${3:-0}"
  local collision_type="${4:-NA}"
  local collision_pair="${5:-NA}"
  local target_drone_id="${6:-NA}"
  local first_collision_time="${7:-NA}"
  local first_emergency_time="${8:-NA}"

  python3 - "$emergency_count" "$collision_count" "$replan_count" "$collision_type" "$collision_pair" "$target_drone_id" "$first_collision_time" "$first_emergency_time" <<'PY'
import sys

def safe_float(v, default=0.0):
  try:
    return float(v)
  except Exception:
    return default

E = safe_float(sys.argv[1], 0.0)
C = safe_float(sys.argv[2], 0.0)
R = safe_float(sys.argv[3], 0.0)
collision_type = str(sys.argv[4]).strip().lower()
collision_pair = str(sys.argv[5]).strip()
target_drone_id = str(sys.argv[6]).strip()
first_collision_time = safe_float(sys.argv[7], float("nan"))
first_emergency_time = safe_float(sys.argv[8], float("nan"))

tdt = 0.0
if not (first_collision_time != first_collision_time or first_emergency_time != first_emergency_time):
  tdt = max(0.0, first_collision_time - first_emergency_time)
  tdt = min(1.0, tdt / 10.0)

target_in_pair = 0.0
if collision_type == "drone_drone" and collision_pair and target_drone_id and target_drone_id != "NA":
  pair_tokens = [tok for tok in collision_pair.replace("_", "-").split("-") if tok]
  if target_drone_id in pair_tokens:
    target_in_pair = 1.0

e_norm = min(1.0, max(0.0, E / 120.0))
gate = 1.0 if collision_type == "drone_drone" and C > 0.0 else 0.0

score = 100.0 * gate * (0.55 * tdt + 0.25 * target_in_pair + 0.20 * e_norm)

print(f"{score:.6f}")
PY
}

compute_round_deadlock_score() {
  local emergency_count="${1:-0}"
  local collision_count="${2:-0}"
  local replan_count="${3:-0}"
  local fail_count="${4:-0}"
  local on_evidence="${5:-0}"

  python3 - "$emergency_count" "$collision_count" "$replan_count" "$fail_count" "$on_evidence" <<'PY'
import sys

def safe_float(v, default=0.0):
  try:
    return float(v)
  except Exception:
    return default

E = safe_float(sys.argv[1], 0.0)
C = safe_float(sys.argv[2], 0.0)
R = safe_float(sys.argv[3], 0.0)
F = safe_float(sys.argv[4], 0.0)
On = 1.0 if safe_float(sys.argv[5], 0.0) > 0.0 else 0.0

e_norm = min(1.0, max(0.0, E / 120.0))
r_norm = min(1.0, max(0.0, R / 450.0))
f_norm = min(1.0, max(0.0, F / 80.0))

score = 100.0 * (1.0 if C == 0.0 else 0.0) * (0.70 * On + 0.15 * e_norm + 0.10 * r_norm + 0.05 * f_norm)

print(f"{score:.6f}")
PY
}

update_seed_pool_file() {
  local rounds_file="${1:-}"
  local pool_file="${2:-}"
  local max_size="${3:-50}"
  local summary_csv="${4:-}"

  python3 - "$rounds_file" "$pool_file" "$max_size" "$summary_csv" <<'PY'
import sys
import csv
import os

rounds_file = sys.argv[1]
pool_file = sys.argv[2]
max_size = max(1, int(float(sys.argv[3])))
summary_csv = sys.argv[4] if len(sys.argv) > 4 else ""
level_rank = {"none": 0, "low": 1, "mid": 2, "high": 3}

def parse_rounds(path):
  rows = []
  try:
    with open(path, "r", encoding="ascii", errors="ignore") as f:
      for ln in f:
        parts = ln.strip().split("\t")
        if not parts or parts[0] == "round_idx":
          continue
        if len(parts) < 15:
          continue
        try:
          round_idx = int(float(parts[0]))
        except Exception:
          continue
        level = str(parts[8]).strip().lower()
        if level not in ("high", "mid", "low"):
          continue
        try:
          evidence_round = int(float(parts[10]))
        except Exception:
          evidence_round = round_idx
        rows.append({
          "round_idx": round_idx,
          "key": parts[1],
          "seed": parts[2],
          "mode": parts[3],
          "start_s": parts[4],
          "duration_s": parts[5],
          "factor_name": parts[6],
          "factor_value": parts[7],
          "anomaly_level": level,
          "severe": "true" if str(parts[9]).strip().lower() == "true" else "false",
          "evidence_round": evidence_round,
          "source_bucket": parts[13] if len(parts) > 13 else "NA",
        })
  except FileNotFoundError:
    rows = []
  return rows

def load_summary_map(path):
  if not path or not os.path.exists(path):
    return {}
  m = {}
  try:
    with open(path, newline='', encoding='utf-8', errors='ignore') as fh:
      rdr = csv.reader(fh)
      hdr = next(rdr, None)
      if hdr is None:
        return {}
      # find column indexes
      try:
        idx_round = hdr.index('round')
      except ValueError:
        idx_round = 1
      try:
        idx_collision = hdr.index('collision_event_count')
      except ValueError:
        idx_collision = None
      try:
        idx_emergency = hdr.index('emergency_count')
      except ValueError:
        idx_emergency = None
      try:
        idx_replan = hdr.index('replan_count')
      except ValueError:
        idx_replan = None
      for r in rdr:
        if len(r) <= idx_round:
          continue
        rnd = r[idx_round].strip()
        if not rnd:
          continue
        try:
          rn = int(rnd)
        except Exception:
          continue
        collision = int(r[idx_collision]) if idx_collision is not None and idx_collision < len(r) and r[idx_collision].isdigit() else 0
        emergency = int(r[idx_emergency]) if idx_emergency is not None and idx_emergency < len(r) and r[idx_emergency].isdigit() else 0
        replan = int(r[idx_replan]) if idx_replan is not None and idx_replan < len(r) and r[idx_replan].isdigit() else 0
        m[rn] = (collision, emergency, replan)
  except Exception:
    return {}
  return m

rows = parse_rounds(rounds_file)
if not rows:
  with open(pool_file, "w", encoding="ascii") as f:
    f.write("param_key\tmode\tstart_s\tduration_s\tfactor_name\tfactor_value\tanomaly_level\tsevere_resource_exhaustion\tcreated_round\tevidence_round\tcount\tsource_bucket\n")
  sys.exit(0)

rows.sort(key=lambda x: x["round_idx"])
pool = []
idx_by_key = {}

def oldest_index(candidates):
  return min(candidates, key=lambda idx: (pool[idx]["created_round"], idx))

for row in rows:
  key = row["key"]
  if key in idx_by_key:
    idx = idx_by_key[key]
    cur = pool[idx]
    cur["mode"] = row["mode"]
    cur["start_s"] = row["start_s"]
    cur["duration_s"] = row["duration_s"]
    cur["factor_name"] = row["factor_name"]
    cur["factor_value"] = row["factor_value"]
    if level_rank[row["anomaly_level"]] > level_rank[cur["anomaly_level"]]:
      cur["anomaly_level"] = row["anomaly_level"]
    if row["severe"] == "true":
      cur["severe"] = "true"
    cur["count"] += 1
    cur["source_bucket"] = row["source_bucket"]
    # keep earliest created_round
    cur["created_round"] = min(cur.get("created_round", row["round_idx"]), row["round_idx"])
    continue

  candidate = {
    "key": row["key"],
    "mode": row["mode"],
    "start_s": row["start_s"],
    "duration_s": row["duration_s"],
    "factor_name": row["factor_name"],
    "factor_value": row["factor_value"],
    "anomaly_level": row["anomaly_level"],
    "severe": row["severe"],
    "created_round": row["round_idx"],
    "evidence_round": row.get("evidence_round", row["round_idx"]),
    "count": 1,
    "source_bucket": row["source_bucket"],
  }

  if len(pool) < max_size:
    pool.append(candidate)
    idx_by_key[candidate["key"]] = len(pool) - 1
    continue

  min_rank = min(level_rank[item["anomaly_level"]] for item in pool)
  cand_rank = level_rank[candidate["anomaly_level"]]
  if cand_rank < min_rank:
    continue
  if cand_rank > min_rank:
    candidates = [idx for idx, item in enumerate(pool) if level_rank[item["anomaly_level"]] == min_rank]
    rm_idx = oldest_index(candidates)
  else:
    candidates = [idx for idx, item in enumerate(pool) if level_rank[item["anomaly_level"]] == cand_rank]
    rm_idx = oldest_index(candidates)

  old_key = pool[rm_idx]["key"]
  del idx_by_key[old_key]
  pool[rm_idx] = candidate
  idx_by_key[candidate["key"]] = rm_idx

# If summary CSV provided, run a self-check to ensure evidence supports anomaly level
removed = []
summary_map = load_summary_map(summary_csv) if summary_csv else {}
if summary_map:
  keep = []
  for item in pool:
    ev = int(item.get("evidence_round", item.get("created_round", 0)))
    expected = "none"
    if ev in summary_map:
      collision, emergency, replan = summary_map[ev]
      if collision > 0:
        expected = "high"
      elif emergency > 0:
        expected = "mid"
      elif replan > 1000:
        expected = "low"
      else:
        expected = "none"
    if expected != item["anomaly_level"]:
      removed.append((item["key"], item["anomaly_level"], expected, ev))
    else:
      keep.append(item)
  pool = keep

audit_path = pool_file + ".audit"
try:
  with open(audit_path, "w", encoding="ascii") as af:
    for k, lvl, exp, ev in removed:
      af.write(f"removed\t{k}\t{lvl}\t->\t{exp}\tevidence_round={ev}\n")
except Exception:
  pass

with open(pool_file, "w", encoding="ascii") as f:
  f.write("param_key\tmode\tstart_s\tduration_s\tfactor_name\tfactor_value\tanomaly_level\tsevere_resource_exhaustion\tcreated_round\tevidence_round\tcount\tsource_bucket\n")
  for item in sorted(pool, key=lambda x: (-level_rank[x["anomaly_level"]], x["created_round"], x["key"])):
    f.write(
      f"{item['key']}\t{item['mode']}\t{item['start_s']}\t{item['duration_s']}\t{item['factor_name']}\t{item['factor_value']}\t{item['anomaly_level']}\t{item['severe']}\t{item['created_round']}\t{item.get('evidence_round','NA')}\t{item['count']}\t{item['source_bucket']}\n"
    )
PY
}

best_effort_ros_cleanup() {
  rosnode kill -a >/dev/null 2>&1 || true
}

shutdown_run() {
  local launch_pid="${1:-}"
  if [ -n "$launch_pid" ]; then
    # Prefer graceful shutdown first to reduce lock-related teardown noise.
    kill -INT "$launch_pid" >/dev/null 2>&1 || true
    local waited=0
    while kill -0 "$launch_pid" >/dev/null 2>&1; do
      if [ "$waited" -ge 50 ]; then
        break
      fi
      sleep 0.1
      waited=$((waited + 1))
    done
    if kill -0 "$launch_pid" >/dev/null 2>&1; then
      kill "$launch_pid" >/dev/null 2>&1 || true
    fi
    wait "$launch_pid" >/dev/null 2>&1 || true
  fi
  best_effort_ros_cleanup
  sleep 1
}

trap 'best_effort_ros_cleanup' EXIT

COMM_FUZZ_SEED_GEN_MODE_NORMALIZED=$(normalize_seed_mode "$COMM_FUZZ_SEED_GEN_MODE")
WARMUP_ROUNDS_EFFECTIVE=$(python3 - "$RUNS" <<'PY'
import math
import sys
runs = max(1, int(float(sys.argv[1])))
print(max(1, int(math.ceil(0.2 * runs))))
PY
)
SEED_POOL_ROUNDS_FILE="$BATCH_DIR/seed_pool_rounds.tsv"
SEED_POOL_FILE="$BATCH_DIR/seed_pool.tsv"
SEED_POOL_USED_SEEDS_FILE="$BATCH_DIR/seed_pool_used_seeds.txt"
touch "$SEED_POOL_ROUNDS_FILE" "$SEED_POOL_FILE" "$SEED_POOL_USED_SEEDS_FILE"
SEED_POOL_ROUNDS_HEADER='round_idx	param_key	seed	mode	start_s	duration_s	factor_name	factor_value	anomaly_level	severe_resource_exhaustion	evidence_round	plan_phase	plan_source	source_bucket	selected_seed_level'
SEED_POOL_HEADER='param_key	mode	start_s	duration_s	factor_name	factor_value	anomaly_level	severe_resource_exhaustion	created_round	evidence_round	count	source_bucket'
if [ ! -s "$SEED_POOL_ROUNDS_FILE" ]; then
  printf '%s\n' "$SEED_POOL_ROUNDS_HEADER" > "$SEED_POOL_ROUNDS_FILE"
fi
if [ ! -s "$SEED_POOL_FILE" ]; then
  printf '%s\n' "$SEED_POOL_HEADER" > "$SEED_POOL_FILE"
fi

{
  echo "Headless benchmark start"
  echo "runs=$RUNS"
  echo "run_sec=$RUN_SEC"
  echo "out_dir=$OUT_DIR"
  echo "batch_dir=$BATCH_DIR"
  echo "collision_monitor_enabled=$COLLISION_MONITOR_ENABLED"
  echo "comm_fuzz_enabled=$COMM_FUZZ_ENABLED"
  echo "comm_fuzz_mode=$COMM_FUZZ_MODE"
  echo "comm_fuzz_target_drone_id=$COMM_FUZZ_TARGET_DRONE_ID"
  echo "comm_fuzz_seed_base=$COMM_FUZZ_SEED_BASE"
  echo "comm_fuzz_seed_gen_mode=$COMM_FUZZ_SEED_GEN_MODE_NORMALIZED"
  echo "seed_pool_warmup_rounds_effective=$WARMUP_ROUNDS_EFFECTIVE"
  echo "comm_fuzz_key_time_sec=$COMM_FUZZ_KEY_TIME_SEC"
  echo "comm_fuzz_ripple_warmup_ratio=$COMM_FUZZ_RIPPLE_WARMUP_RATIO"
  echo "comm_fuzz_ripple_warmup_min_rounds=$COMM_FUZZ_RIPPLE_WARMUP_MIN_ROUNDS"
  echo "comm_fuzz_ripple_warmup_rounds_override=$COMM_FUZZ_RIPPLE_WARMUP_ROUNDS"
  echo "comm_fuzz_start_range_sec=$COMM_FUZZ_START_MIN_SEC,$COMM_FUZZ_START_MAX_SEC"
  echo "comm_fuzz_duration_range_sec=$COMM_FUZZ_DURATION_MIN_SEC,$COMM_FUZZ_DURATION_MAX_SEC"
  echo "comm_fuzz_delay_range_ms=$COMM_FUZZ_DELAY_MIN_MS,$COMM_FUZZ_DELAY_MAX_MS"
  echo "comm_fuzz_jitter_range_ms=$COMM_FUZZ_JITTER_MIN_MS,$COMM_FUZZ_JITTER_MAX_MS"
  echo "comm_fuzz_noise_sigma_pos_range=$COMM_FUZZ_NOISE_SIGMA_POS_MIN,$COMM_FUZZ_NOISE_SIGMA_POS_MAX"
  echo "comm_full_dump_sidecar_enabled=$COMM_FULL_DUMP_SIDECAR_ENABLED"
  echo "comm_full_dump_buffer_max=$COMM_FULL_DUMP_BUFFER_MAX"
  echo "comm_full_dump_file_name=$COMM_FULL_DUMP_FILE_NAME"
  echo "stl_metrics_monitor_enabled=$STL_METRICS_MONITOR_ENABLED"
  echo "stl_window_sec=$STL_WINDOW_SEC"
  echo "stl_move_threshold_x=$STL_MOVE_THRESHOLD_X"
  echo "stl_delay_timeout_threshold_s=$STL_DELAY_TIMEOUT_THRESHOLD_S"
  echo "stl_active_speed_eps=$STL_ACTIVE_SPEED_EPS"
  echo "stl_start_grace_sec=$STL_START_GRACE_SEC"
  echo "stl_drift_eps=$STL_DRIFT_EPS"
  echo "stl_future_horizon_sec=$STL_FUTURE_HORIZON_SEC"
  echo "stl_check_hz=$STL_CHECK_HZ"
  echo "stl_attack_scenarios_file=$BATCH_STL_ATTACK_SCENARIOS_FILE"
  echo "attack_plan_latency_proxy_enabled=$ATTACK_PLAN_LATENCY_PROXY_ENABLED"
  echo "attack_plan_file=$ATTACK_PLAN_FILE"
  echo "adaptive_replan_latency_proxy_enabled=$ADAPTIVE_REPLAN_LATENCY_PROXY_ENABLED"
  echo "adaptive_replan_attack_config=$ADAPTIVE_REPLAN_ATTACK_CONFIG"
  echo "adaptive_max_injections_per_round=$ADAPTIVE_MAX_INJECTIONS_PER_ROUND"
  echo "adaptive_cooldown_s=$ADAPTIVE_COOLDOWN_S"
  echo "adaptive_trigger_window_s=$ADAPTIVE_TRIGGER_WINDOW_S"
  echo "adaptive_future_horizon_s=$ADAPTIVE_FUTURE_HORIZON_S"
  echo "adaptive_future_sample_dt=$ADAPTIVE_FUTURE_SAMPLE_DT"
  echo "adaptive_interaction_margin_threshold=$ADAPTIVE_INTERACTION_MARGIN_THRESHOLD"
  echo "adaptive_interaction_score_threshold=$ADAPTIVE_INTERACTION_SCORE_THRESHOLD"
  echo "adaptive_latency_factor=$ADAPTIVE_LATENCY_FACTOR"
  echo "adaptive_latency_range_ms=$ADAPTIVE_LATENCY_MIN_MS,$ADAPTIVE_LATENCY_MAX_MS"
  echo "adaptive_period_fallback_ms=$ADAPTIVE_PERIOD_FALLBACK_MS"
  echo "adaptive_seed=$ADAPTIVE_SEED"
  echo "rule_pool_latency_proxy_enabled=$RULE_POOL_LATENCY_PROXY_ENABLED"
  echo "rule_pool_active_rule_id=$RULE_POOL_ACTIVE_RULE_ID"
  echo "rule_pool_screening_mode=$RULE_POOL_SCREENING_MODE"
  echo "rule_pool_monitor_only=$RULE_POOL_MONITOR_ONLY"
  echo "rule_pool_max_injections_per_round=$RULE_POOL_MAX_INJECTIONS_PER_ROUND"
  echo "rule_pool_cooldown_s=$RULE_POOL_COOLDOWN_S"
  echo "rule_pool_history_window_s=$RULE_POOL_HISTORY_WINDOW_S"
  echo "rule_pool_trigger_window_s=$RULE_POOL_TRIGGER_WINDOW_S"
  echo "rule_pool_future_horizon_s=$RULE_POOL_FUTURE_HORIZON_S"
  echo "rule_pool_future_sample_dt=$RULE_POOL_FUTURE_SAMPLE_DT"
  echo "rule_pool_heading_aligned_deg=$RULE_POOL_HEADING_ALIGNED_DEG"
  echo "rule_pool_future_collision_margin=$RULE_POOL_FUTURE_COLLISION_MARGIN"
  echo "rule_pool_latency_epsilon_ms=$RULE_POOL_LATENCY_EPSILON_MS"
  echo "rule_pool_latency_min_ms=$RULE_POOL_LATENCY_MIN_MS"
  echo "rule_pool_latency_optional_cap_ms=$RULE_POOL_LATENCY_MAX_MS"
  echo "rule_pool_protocol_timeout_ms=$RULE_POOL_PROTOCOL_TIMEOUT_MS"
  echo "rule_pool_max_latency_ratio=$RULE_POOL_MAX_LATENCY_RATIO"
  echo "rule_pool_period_fallback_ms=$RULE_POOL_PERIOD_FALLBACK_MS"
  echo "summary_csv=$SUMMARY_CSV"
  echo
} > "$SUMMARY"

echo "row_type,round,timestamp,result,bench_result,strict_result,success_count,fail_count,collision_monitor_enabled,comm_fuzz_enabled,comm_fuzz_seed,comm_fuzz_mode,comm_fuzz_target_drone_id,comm_fuzz_start_s,comm_fuzz_duration_s,comm_fuzz_factor_name,comm_fuzz_factor_value,comm_fuzz_topic,collision_event_count,drone_drone_collision_count,drone_obstacle_collision_count,collision_type,collision_pair,collision_distance,mandatory_stop_count,fuzz_report_dir,stl_attack_scenarios_file,has_replan,replan_count,last_time_ms,avg_time_ms,emergency_count,fatal_count,root_cause_primary,root_cause_types,root_cause_confidence,root_cause_summary,root_cause_report,first_collision_time,first_emergency_time,detail_txt,round_dir" > "$SUMMARY_CSV"

BENCH_SUCCESS_ROUNDS=0
STRICT_SUCCESS_ROUNDS=0
ROUNDS_WITH_REPLAN=0
TOTAL_REPLAN_COUNT=0
TOTAL_SUCCESS_COUNT=0
TOTAL_FAIL_COUNT=0
TOTAL_EMERGENCY_COUNT=0
TOTAL_FATAL_COUNT=0
TOTAL_FATAL_RUNTIME_COUNT=0
TOTAL_FATAL_SHUTDOWN_COUNT=0
TOTAL_COLLISION_EVENT_COUNT=0
TOTAL_DRONE_DRONE_COLLISION_COUNT=0
TOTAL_DRONE_OBSTACLE_COLLISION_COUNT=0
TOTAL_MANDATORY_STOP_COUNT=0
ROUNDS_WITH_EMERGENCY=0
ROUNDS_WITH_COLLISION=0
ROUNDS_WITH_LOCKERR_RUNTIME=0
ROUNDS_WITH_LOCKERR_SHUTDOWN=0
ROUNDS_WITH_PROCESS_CRASH=0
TOTAL_LAST_TIME_MS=0
TOTAL_AVG_TIME_MS=0
TOTAL_LAST_TIME_COUNT=0
TOTAL_AVG_TIME_COUNT=0

for i in $(seq 1 "$RUNS"); do
  TS=$(date +%Y%m%d_%H%M%S)
  ROUND_DIR="$BATCH_DIR/run${i}"
  mkdir -p "$ROUND_DIR"
  # Avoid carrying a prior round's sampled plan into CSV/meta fallbacks (esp. when comm fuzz is off).
  ROUND_PLAN_LINE=""

  RAW="$ROUND_DIR/run.log"
  : > "$RAW"
  CLEAN="$ROUND_DIR/run_clean.log"
  RUNTIME_CLEAN="$ROUND_DIR/runtime_clean.log"
  SHUTDOWN_CLEAN="$ROUND_DIR/shutdown_clean.log"
  TXT="$ROUND_DIR/result.txt"
  FUZZ_RUN_ID="${BATCH_TS}_run${i}"

  ROUND_COMM_FUZZ_MODE="$COMM_FUZZ_MODE"
  ROUND_COMM_FUZZ_START_MIN_SEC="$COMM_FUZZ_START_MIN_SEC"
  ROUND_COMM_FUZZ_START_MAX_SEC="$COMM_FUZZ_START_MAX_SEC"
  ROUND_COMM_FUZZ_DURATION_MIN_SEC="$COMM_FUZZ_DURATION_MIN_SEC"
  ROUND_COMM_FUZZ_DURATION_MAX_SEC="$COMM_FUZZ_DURATION_MAX_SEC"
  ROUND_COMM_FUZZ_DELAY_MIN_MS="$COMM_FUZZ_DELAY_MIN_MS"
  ROUND_COMM_FUZZ_DELAY_MAX_MS="$COMM_FUZZ_DELAY_MAX_MS"
  ROUND_COMM_FUZZ_JITTER_MIN_MS="$COMM_FUZZ_JITTER_MIN_MS"
  ROUND_COMM_FUZZ_JITTER_MAX_MS="$COMM_FUZZ_JITTER_MAX_MS"
  ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MIN="$COMM_FUZZ_NOISE_SIGMA_POS_MIN"
  ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MAX="$COMM_FUZZ_NOISE_SIGMA_POS_MAX"
  ROUND_PLAN_PHASE="warmup"
  ROUND_PLAN_SOURCE="warmup_random"
  ROUND_SOURCE_BUCKET="NA"
  ROUND_SELECTED_SEED_LEVEL="NA"

  COMM_FUZZ_SEED=$(choose_seed_pure_random "$i")
  if [ "$COMM_FUZZ_SEED_GEN_MODE_NORMALIZED" = "ripple_biased" ] && [ "$COMM_FUZZ_ENABLED" = "yes" ]; then
    if [ "$i" -gt "$WARMUP_ROUNDS_EFFECTIVE" ]; then
      ROUND_PLAN_PHASE="evolution"
      ROUND_PLAN_SOURCE="evolution_fallback_random"
      ROUND_PLAN_LINE=$(choose_seed_pool_param_plan "$i" "$SEED_POOL_FILE" "$SEED_POOL_USED_SEEDS_FILE" "$WARMUP_ROUNDS_EFFECTIVE")
      if [ -n "$ROUND_PLAN_LINE" ]; then
        IFS=$'\t' read -r COMM_FUZZ_SEED ROUND_COMM_FUZZ_MODE ROUND_PLAN_START_S ROUND_PLAN_DURATION_S ROUND_PLAN_FACTOR_NAME ROUND_PLAN_FACTOR_VALUE ROUND_PLAN_SOURCE ROUND_SOURCE_BUCKET ROUND_SELECTED_SEED_LEVEL <<< "$ROUND_PLAN_LINE"
      else
        ROUND_PLAN_SOURCE="evolution_fallback_random"
        ROUND_PLAN_LINE=$(sample_mixed_comm_plan "$i")
        IFS=$'\t' read -r ROUND_COMM_FUZZ_MODE ROUND_PLAN_START_S ROUND_PLAN_DURATION_S ROUND_PLAN_FACTOR_NAME ROUND_PLAN_FACTOR_VALUE <<< "$ROUND_PLAN_LINE"
        ROUND_SOURCE_BUCKET="NA"
        ROUND_SELECTED_SEED_LEVEL="NA"
      fi

      if [ -n "$ROUND_PLAN_START_S" ] && [ -n "$ROUND_PLAN_DURATION_S" ]; then
        ROUND_COMM_FUZZ_START_MIN_SEC="$ROUND_PLAN_START_S"
        ROUND_COMM_FUZZ_START_MAX_SEC="$ROUND_PLAN_START_S"
        ROUND_COMM_FUZZ_DURATION_MIN_SEC="$ROUND_PLAN_DURATION_S"
        ROUND_COMM_FUZZ_DURATION_MAX_SEC="$ROUND_PLAN_DURATION_S"

        if [ "$ROUND_PLAN_FACTOR_NAME" = "delay_ms" ]; then
          ROUND_COMM_FUZZ_DELAY_MIN_MS="$ROUND_PLAN_FACTOR_VALUE"
          ROUND_COMM_FUZZ_DELAY_MAX_MS="$ROUND_PLAN_FACTOR_VALUE"
        elif [ "$ROUND_PLAN_FACTOR_NAME" = "jitter_ms" ]; then
          ROUND_COMM_FUZZ_JITTER_MIN_MS="$ROUND_PLAN_FACTOR_VALUE"
          ROUND_COMM_FUZZ_JITTER_MAX_MS="$ROUND_PLAN_FACTOR_VALUE"
        elif [ "$ROUND_PLAN_FACTOR_NAME" = "noise_sigma_pos" ]; then
          ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MIN="$ROUND_PLAN_FACTOR_VALUE"
          ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MAX="$ROUND_PLAN_FACTOR_VALUE"
        fi
      fi
    else
      ROUND_PLAN_PHASE="warmup"
      ROUND_PLAN_SOURCE="warmup_random"
      ROUND_PLAN_LINE=$(sample_mixed_comm_plan "$i")
      IFS=$'\t' read -r ROUND_COMM_FUZZ_MODE ROUND_PLAN_START_S ROUND_PLAN_DURATION_S ROUND_PLAN_FACTOR_NAME ROUND_PLAN_FACTOR_VALUE <<< "$ROUND_PLAN_LINE"
      COMM_FUZZ_SEED="${COMM_FUZZ_SEED:-$((COMM_FUZZ_SEED_BASE + i))}"
      ROUND_COMM_FUZZ_START_MIN_SEC="$ROUND_PLAN_START_S"
      ROUND_COMM_FUZZ_START_MAX_SEC="$ROUND_PLAN_START_S"
      ROUND_COMM_FUZZ_DURATION_MIN_SEC="$ROUND_PLAN_DURATION_S"
      ROUND_COMM_FUZZ_DURATION_MAX_SEC="$ROUND_PLAN_DURATION_S"
      if [ "$ROUND_PLAN_FACTOR_NAME" = "delay_ms" ]; then
        ROUND_COMM_FUZZ_DELAY_MIN_MS="$ROUND_PLAN_FACTOR_VALUE"
        ROUND_COMM_FUZZ_DELAY_MAX_MS="$ROUND_PLAN_FACTOR_VALUE"
      elif [ "$ROUND_PLAN_FACTOR_NAME" = "jitter_ms" ]; then
        ROUND_COMM_FUZZ_JITTER_MIN_MS="$ROUND_PLAN_FACTOR_VALUE"
        ROUND_COMM_FUZZ_JITTER_MAX_MS="$ROUND_PLAN_FACTOR_VALUE"
      elif [ "$ROUND_PLAN_FACTOR_NAME" = "noise_sigma_pos" ]; then
        ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MIN="$ROUND_PLAN_FACTOR_VALUE"
        ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MAX="$ROUND_PLAN_FACTOR_VALUE"
      fi
    fi
    echo "$COMM_FUZZ_SEED" >> "$SEED_POOL_USED_SEEDS_FILE"
  fi
  rm -rf "$ROUND_DIR/fuzz_report" "$ROUND_DIR/fuzz_report_staging"
  mkdir -p "$ROUND_DIR/fuzz_report_staging"
  mkdir -p "$ROUND_DIR"
  mkdir -p "$(dirname "$RAW")"
  ABS_ROUND_FUZZ_REPORT_DIR="$(cd "$ROUND_DIR/fuzz_report_staging" && pwd)"
  FUZZ_REPORT_DIR_VALUE="NA"
  # Paths for bridge/planner artifacts during the run (staging; promoted after round if incident)
  COMM_FUZZ_META_FILE="$ABS_ROUND_FUZZ_REPORT_DIR/$FUZZ_RUN_ID/comm_fuzz_meta.json"
  COMM_FUZZ_TRACE_FILE="$ABS_ROUND_FUZZ_REPORT_DIR/$FUZZ_RUN_ID/comm_fuzz_trace.jsonl"
  PLANNER_RX_TRACE_GLOB="$ABS_ROUND_FUZZ_REPORT_DIR/$FUZZ_RUN_ID/planner_rx_trace_drone_*.jsonl"
  PLANNER_SAFETY_TRACE_GLOB="$ABS_ROUND_FUZZ_REPORT_DIR/$FUZZ_RUN_ID/planner_safety_trace_drone_*.jsonl"
  STL_METRICS_SUMMARY_FILE="$ROUND_DIR/stl_metrics_summary.json"
  STL_METRICS_TRACE_FILE="$ROUND_DIR/stl_metrics_trace.jsonl"
  ADAPTIVE_REPLAN_META_FILE="$ROUND_DIR/adaptive_replan_latency_meta.json"
  ADAPTIVE_REPLAN_EVENTS_FILE="$ROUND_DIR/adaptive_replan_latency_events.csv"
  ADAPTIVE_REPLAN_TRACE_FILE="$ROUND_DIR/adaptive_replan_latency_trace.jsonl"
  RULE_POOL_META_FILE="$ROUND_DIR/rule_pool_latency_meta.json"
  RULE_POOL_EVENTS_FILE="$ROUND_DIR/rule_pool_latency_events.csv"
  RULE_POOL_TRACE_FILE="$ROUND_DIR/rule_pool_latency_trace.jsonl"

  echo "[Round $i/$RUNS] launch swarm..."
  # Ensure concrete per-round comm-fuzz parameters are sampled before launch when fuzz enabled.
  if [ "$COMM_FUZZ_FLAG" = "true" ]; then
    # If using ripple_biased (seed-pool), the ROUND_COMM_FUZZ_* were set earlier when selecting from pool.
    # For non-ripple modes (pure_random / random), actively sample a concrete plan now so we pass
    # exact mode/start/duration/factor into roslaunch and record them in CSV.
    if [ "${COMM_FUZZ_SEED_GEN_MODE_NORMALIZED:-pure_random}" != "ripple_biased" ]; then
      # If mode is generic/random, sample mixed plan; otherwise sample factor for the fixed mode.
      case "${ROUND_COMM_FUZZ_MODE:-${COMM_FUZZ_MODE}}" in
        random|pure_random|mixed)
          PLAN_LINE=$(sample_mixed_comm_plan "$i") || PLAN_LINE=""
          if [ -n "$PLAN_LINE" ]; then
            IFS=$'\t' read -r SAMP_MODE SAMP_START_S SAMP_DURATION_S SAMP_FACTOR_NAME SAMP_FACTOR_VALUE <<< "$PLAN_LINE"
            ROUND_COMM_FUZZ_MODE="$SAMP_MODE"
            ROUND_COMM_FUZZ_START_MIN_SEC="$SAMP_START_S"
            ROUND_COMM_FUZZ_START_MAX_SEC="$SAMP_START_S"
            ROUND_COMM_FUZZ_DURATION_MIN_SEC="$SAMP_DURATION_S"
            ROUND_COMM_FUZZ_DURATION_MAX_SEC="$SAMP_DURATION_S"
            ROUND_COMM_FUZZ_DELAY_MIN_MS="$SAMP_FACTOR_VALUE"
            ROUND_COMM_FUZZ_DELAY_MAX_MS="$SAMP_FACTOR_VALUE"
            ROUND_COMM_FUZZ_JITTER_MIN_MS="$SAMP_FACTOR_VALUE"
            ROUND_COMM_FUZZ_JITTER_MAX_MS="$SAMP_FACTOR_VALUE"
            ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MIN="$SAMP_FACTOR_VALUE"
            ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MAX="$SAMP_FACTOR_VALUE"
            ROUND_PLAN_FACTOR_NAME="$SAMP_FACTOR_NAME"
            ROUND_PLAN_FACTOR_VALUE="$SAMP_FACTOR_VALUE"
          fi
          ;; 
        delay)
          # sample delay value within configured range
          VAL=$(python3 -c "import random; print(random.uniform($ROUND_COMM_FUZZ_DELAY_MIN_MS, $ROUND_COMM_FUZZ_DELAY_MAX_MS))") || VAL=""
          ROUND_PLAN_FACTOR_NAME="delay_ms"
          ROUND_PLAN_FACTOR_VALUE="$VAL"
          ;; 
        jitter)
          VAL=$(python3 -c "import random; print(random.uniform($ROUND_COMM_FUZZ_JITTER_MIN_MS, $ROUND_COMM_FUZZ_JITTER_MAX_MS))") || VAL=""
          ROUND_PLAN_FACTOR_NAME="jitter_ms"
          ROUND_PLAN_FACTOR_VALUE="$VAL"
          ;; 
        noise)
          VAL=$(python3 -c "import random; print(random.uniform($ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MIN, $ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MAX))") || VAL=""
          ROUND_PLAN_FACTOR_NAME="noise_sigma_pos"
          ROUND_PLAN_FACTOR_VALUE="$VAL"
          ;; 
      esac

      # ensure seed is concrete for this round
      COMM_FUZZ_SEED=$(choose_seed_pure_random "$i")
    fi
  fi

  SIDECAR_PID=""
  SIDECAR_LOG="$ROUND_DIR/comm_full_dump_sidecar.log"
  if [ "$COMM_FULL_DUMP_SIDECAR_FLAG" = "true" ]; then
    python3 planner/plan_manage/scripts/comm_full_dump_sidecar.py \
      _enable:=true \
      _report_dir:="$ABS_ROUND_FUZZ_REPORT_DIR" \
      _run_id:="$FUZZ_RUN_ID" \
      _buffer_max:="$COMM_FULL_DUMP_BUFFER_MAX" \
      _dump_file_name:="$COMM_FULL_DUMP_FILE_NAME" \
      > "$SIDECAR_LOG" 2>&1 &
    SIDECAR_PID=$!
  fi

  # O_APPEND keeps the shutdown marker intact when roslaunch flushes buffered output.
  PYTHONUNBUFFERED=1 roslaunch ego_planner swarm.launch \
    enable_hard_collision_monitor:="$COLLISION_MONITOR_FLAG" \
    enable_stl_metrics_monitor:="$STL_METRICS_MONITOR_FLAG" \
    stl_window_sec:="$STL_WINDOW_SEC" \
    stl_move_threshold_x:="$STL_MOVE_THRESHOLD_X" \
    stl_delay_timeout_threshold_s:="$STL_DELAY_TIMEOUT_THRESHOLD_S" \
    stl_active_speed_eps:="$STL_ACTIVE_SPEED_EPS" \
    stl_start_grace_sec:="$STL_START_GRACE_SEC" \
    stl_drift_eps:="$STL_DRIFT_EPS" \
    stl_future_horizon_sec:="$STL_FUTURE_HORIZON_SEC" \
    stl_check_hz:="$STL_CHECK_HZ" \
    enable_attack_plan_latency_proxy:="$ATTACK_PLAN_LATENCY_PROXY_FLAG" \
    attack_plan_file:="$ATTACK_PLAN_FILE" \
    attack_plan_tx_bridge_input_topic:="$ATTACK_PLAN_TX_BRIDGE_INPUT_TOPIC" \
    enable_adaptive_replan_latency_proxy:="$ADAPTIVE_REPLAN_LATENCY_PROXY_FLAG" \
    adaptive_replan_attack_config:="$ADAPTIVE_REPLAN_ATTACK_CONFIG" \
    adaptive_max_injections_per_round:="$ADAPTIVE_MAX_INJECTIONS_PER_ROUND" \
    adaptive_cooldown_s:="$ADAPTIVE_COOLDOWN_S" \
    adaptive_trigger_window_s:="$ADAPTIVE_TRIGGER_WINDOW_S" \
    adaptive_future_horizon_s:="$ADAPTIVE_FUTURE_HORIZON_S" \
    adaptive_future_sample_dt:="$ADAPTIVE_FUTURE_SAMPLE_DT" \
    adaptive_interaction_margin_threshold:="$ADAPTIVE_INTERACTION_MARGIN_THRESHOLD" \
    adaptive_interaction_score_threshold:="$ADAPTIVE_INTERACTION_SCORE_THRESHOLD" \
    adaptive_latency_factor:="$ADAPTIVE_LATENCY_FACTOR" \
    adaptive_latency_min_ms:="$ADAPTIVE_LATENCY_MIN_MS" \
    adaptive_latency_max_ms:="$ADAPTIVE_LATENCY_MAX_MS" \
    adaptive_period_fallback_ms:="$ADAPTIVE_PERIOD_FALLBACK_MS" \
    adaptive_seed:="$ADAPTIVE_SEED" \
    enable_rule_pool_latency_proxy:="$RULE_POOL_LATENCY_PROXY_FLAG" \
    rule_pool_active_rule_id:="$RULE_POOL_ACTIVE_RULE_ID" \
    rule_pool_screening_mode:="$RULE_POOL_SCREENING_MODE" \
    rule_pool_monitor_only:="$RULE_POOL_MONITOR_ONLY" \
    rule_pool_max_injections_per_round:="$RULE_POOL_MAX_INJECTIONS_PER_ROUND" \
    rule_pool_cooldown_s:="$RULE_POOL_COOLDOWN_S" \
    rule_pool_history_window_s:="$RULE_POOL_HISTORY_WINDOW_S" \
    rule_pool_trigger_window_s:="$RULE_POOL_TRIGGER_WINDOW_S" \
    rule_pool_future_horizon_s:="$RULE_POOL_FUTURE_HORIZON_S" \
    rule_pool_future_sample_dt:="$RULE_POOL_FUTURE_SAMPLE_DT" \
    rule_pool_heading_aligned_deg:="$RULE_POOL_HEADING_ALIGNED_DEG" \
    rule_pool_future_collision_margin:="$RULE_POOL_FUTURE_COLLISION_MARGIN" \
    rule_pool_latency_epsilon_ms:="$RULE_POOL_LATENCY_EPSILON_MS" \
    rule_pool_latency_min_ms:="$RULE_POOL_LATENCY_MIN_MS" \
    rule_pool_latency_max_ms:="$RULE_POOL_LATENCY_MAX_MS" \
    rule_pool_protocol_timeout_ms:="$RULE_POOL_PROTOCOL_TIMEOUT_MS" \
    rule_pool_max_latency_ratio:="$RULE_POOL_MAX_LATENCY_RATIO" \
    rule_pool_period_fallback_ms:="$RULE_POOL_PERIOD_FALLBACK_MS" \
    kill_all_nodes_on_collision:=false \
    fuzz_run_id:="$FUZZ_RUN_ID" \
    fuzz_report_dir:="$ABS_ROUND_FUZZ_REPORT_DIR" \
    enable_comm_fuzz:="$COMM_FUZZ_FLAG" \
    comm_fuzz_mode:="$ROUND_COMM_FUZZ_MODE" \
    comm_fuzz_target_drone_id:="$COMM_FUZZ_TARGET_DRONE_ID" \
    comm_fuzz_seed:="$COMM_FUZZ_SEED" \
    comm_fuzz_start_min_sec:="$ROUND_COMM_FUZZ_START_MIN_SEC" \
    comm_fuzz_start_max_sec:="$ROUND_COMM_FUZZ_START_MAX_SEC" \
    comm_fuzz_duration_min_sec:="$ROUND_COMM_FUZZ_DURATION_MIN_SEC" \
    comm_fuzz_duration_max_sec:="$ROUND_COMM_FUZZ_DURATION_MAX_SEC" \
    comm_fuzz_delay_min_ms:="$ROUND_COMM_FUZZ_DELAY_MIN_MS" \
    comm_fuzz_delay_max_ms:="$ROUND_COMM_FUZZ_DELAY_MAX_MS" \
    comm_fuzz_jitter_min_ms:="$ROUND_COMM_FUZZ_JITTER_MIN_MS" \
    comm_fuzz_jitter_max_ms:="$ROUND_COMM_FUZZ_JITTER_MAX_MS" \
    comm_fuzz_noise_sigma_pos_min:="$ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MIN" \
    comm_fuzz_noise_sigma_pos_max:="$ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MAX" \
    comm_fuzz_report_dir:="$ABS_ROUND_FUZZ_REPORT_DIR" \
    comm_fuzz_run_id:="$FUZZ_RUN_ID" >> "$RAW" 2>&1 &
  LAUNCH_PID=$!
  # Give bridge/node a short window to write comm_fuzz_meta.json back into the fuzz_report dir.
  # Prefer bridge-written metadata when available; otherwise fall back to planned/sample values.
  if [ "$COMM_FUZZ_FLAG" = "true" ]; then
    # wait up to WAIT_META_SEC seconds for the meta file to appear
    WAIT_META_SEC=3
    FOUND_META=0
    for s in $(seq 0 1 $WAIT_META_SEC); do
      if [ -n "${COMM_FUZZ_META_FILE:-}" ] && [ -f "$COMM_FUZZ_META_FILE" ]; then
        FOUND_META=1
        break
      fi
      sleep 1
    done
    if [ "$FOUND_META" -eq 1 ]; then
      # re-read meta fields to ensure CSV reflects actual applied fuzz
      COMM_FUZZ_SEED_USED=$(sed -nE 's/.*"seed"[[:space:]]*:[[:space:]]*([0-9-]+).*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
      COMM_FUZZ_MODE_USED=$(sed -nE 's/.*"mode"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
      COMM_FUZZ_TARGET_USED=$(sed -nE 's/.*"target_drone_id"[[:space:]]*:[[:space:]]*([0-9-]+).*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
      COMM_FUZZ_START_S=$(sed -nE 's/.*"window_start_s"[[:space:]]*:[[:space:]]*([0-9]+(\.[0-9]+)?).*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
      COMM_FUZZ_DURATION_S=$(sed -nE 's/.*"window_duration_s"[[:space:]]*:[[:space:]]*([0-9]+(\.[0-9]+)?).*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
      COMM_FUZZ_FACTOR_NAME=$(sed -nE 's/.*"factor_name"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
      COMM_FUZZ_FACTOR_VALUE=$(sed -nE 's/.*"factor_value"[[:space:]]*:[[:space:]]*([0-9]+(\.[0-9]+)?).*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
      COMM_FUZZ_TOPIC=$(sed -nE 's/.*"in_topic"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
    else
      # no meta file found in the wait window; proceed with planned/sample values (already set earlier)
      :
    fi
  fi
  sleep "$RUN_SEC"

  echo "$SHUTDOWN_MARK ts=$(date +%s)" >> "$RAW"

  shutdown_run "$LAUNCH_PID"
  if [ -n "$SIDECAR_PID" ]; then
    kill -INT "$SIDECAR_PID" >/dev/null 2>&1 || true
    wait "$SIDECAR_PID" >/dev/null 2>&1 || true
  fi

  STL_RUN_DIR="$ABS_ROUND_FUZZ_REPORT_DIR/$FUZZ_RUN_ID"
  if [ -f "$STL_RUN_DIR/stl_metrics_summary.json" ]; then
    cp "$STL_RUN_DIR/stl_metrics_summary.json" "$STL_METRICS_SUMMARY_FILE"
  fi
  if [ -f "$STL_RUN_DIR/stl_metrics_trace.jsonl" ]; then
    cp "$STL_RUN_DIR/stl_metrics_trace.jsonl" "$STL_METRICS_TRACE_FILE"
  fi
  if [ -f "$STL_RUN_DIR/adaptive_replan_latency_meta.json" ]; then
    cp "$STL_RUN_DIR/adaptive_replan_latency_meta.json" "$ADAPTIVE_REPLAN_META_FILE"
  fi
  if [ -f "$STL_RUN_DIR/adaptive_replan_latency_events.csv" ]; then
    cp "$STL_RUN_DIR/adaptive_replan_latency_events.csv" "$ADAPTIVE_REPLAN_EVENTS_FILE"
  fi
  if [ -f "$STL_RUN_DIR/adaptive_replan_latency_trace.jsonl" ]; then
    cp "$STL_RUN_DIR/adaptive_replan_latency_trace.jsonl" "$ADAPTIVE_REPLAN_TRACE_FILE"
  fi
  if [ -f "$STL_RUN_DIR/rule_pool_latency_meta.json" ]; then
    cp "$STL_RUN_DIR/rule_pool_latency_meta.json" "$RULE_POOL_META_FILE"
  fi
  if [ -f "$STL_RUN_DIR/rule_pool_latency_events.csv" ]; then
    cp "$STL_RUN_DIR/rule_pool_latency_events.csv" "$RULE_POOL_EVENTS_FILE"
  fi
  if [ -f "$STL_RUN_DIR/rule_pool_latency_trace.jsonl" ]; then
    cp "$STL_RUN_DIR/rule_pool_latency_trace.jsonl" "$RULE_POOL_TRACE_FILE"
  fi
  if [ -f "$STL_RUN_DIR/ddc10_rule_observations.csv" ]; then
    cp "$STL_RUN_DIR/ddc10_rule_observations.csv" "$ROUND_DIR/ddc10_rule_observations.csv"
  fi

  strip_ansi < "$RAW" > "$CLEAN"
  extract_runtime_log "$CLEAN" "$RUNTIME_CLEAN"
  extract_shutdown_log "$CLEAN" "$SHUTDOWN_CLEAN"

  SUCCESS_COUNT=$(grep -c "Success=yes" "$RUNTIME_CLEAN" || true)
  FAIL_COUNT=$(grep -c "Success=no" "$RUNTIME_CLEAN" || true)
  REPLAN_COUNT=$(grep -Eic 'Replan[[:space:]]+[0-9]+' "$RUNTIME_CLEAN" || true)
  HAS_REPLAN="no"
  if [ "${REPLAN_COUNT:-0}" -gt 0 ]; then
    HAS_REPLAN="yes"
  fi
  EMERGENCY_COUNT=$(grep -Eic "Emergency stop|EMERGENCY_STOP|Mandatory Stop|Depth Lost" "$RUNTIME_CLEAN" || true)
  FATAL_RUNTIME_COUNT=$(grep -Eic "$FATAL_PATTERN" "$RUNTIME_CLEAN" || true)
  FATAL_SHUTDOWN_COUNT=$(grep -Eic "$FATAL_PATTERN" "$SHUTDOWN_CLEAN" || true)
  if [ "$FATAL_SHUTDOWN_COUNT" -lt 0 ]; then
    FATAL_SHUTDOWN_COUNT=0
  fi
  FATAL_COUNT=$FATAL_RUNTIME_COUNT
  LOCKERR_RUNTIME_COUNT=$(grep -Eic "boost::wrapexcept<boost::lock_error>|boost: mutex lock failed" "$RUNTIME_CLEAN" || true)
  LOCKERR_SHUTDOWN_COUNT=$(grep -Eic "boost::wrapexcept<boost::lock_error>|boost: mutex lock failed" "$SHUTDOWN_CLEAN" || true)
  if [ "$LOCKERR_SHUTDOWN_COUNT" -lt 0 ]; then
    LOCKERR_SHUTDOWN_COUNT=0
  fi
  PROCESS_CRASH_COUNT=$(grep -Eic "process\[[^]]+\].*(has died|exit code)" "$RUNTIME_CLEAN" || true)
  CRASH_NODE_TOP="NA"
  CRASH_NODE_TOP_COUNT=0
  if [ "${PROCESS_CRASH_COUNT:-0}" -gt 0 ]; then
    CRASH_NODE_STAT=$(grep -E "process\[[^]]+\].*(has died|exit code)" "$RUNTIME_CLEAN" | sed -nE 's/.*process\[([^]-]+)-[0-9]+\].*/\1/p' | sort | uniq -c | sort -nr | head -n 1 || true)
    if [ -n "${CRASH_NODE_STAT:-}" ]; then
      CRASH_NODE_TOP_COUNT=$(echo "$CRASH_NODE_STAT" | awk '{print $1}')
      CRASH_NODE_TOP=$(echo "$CRASH_NODE_STAT" | awk '{print $2}')
    fi
  fi
  FATAL_HIT=$(grep -Eim1 "$FATAL_PATTERN" "$RUNTIME_CLEAN" || true)
  COLLISION_EVENT_COUNT=$(grep -Eic "HARD_COLLISION|fuzz_collision_monitor\] HARD_COLLISION" "$RUNTIME_CLEAN" || true)
  DRONE_DRONE_COLLISION_COUNT=$(grep -Eic '"type"[[:space:]]*:[[:space:]]*"drone_drone"' "$RUNTIME_CLEAN" || true)
  DRONE_OBSTACLE_COLLISION_COUNT=$(grep -Eic '"type"[[:space:]]*:[[:space:]]*"drone_obstacle"' "$RUNTIME_CLEAN" || true)
  MANDATORY_STOP_COUNT=$(grep -Eic "Mandatory Stop|Received a mandatory stop command" "$RUNTIME_CLEAN" || true)
  ROUND_HAS_EMERGENCY="no"
  if [ "${EMERGENCY_COUNT:-0}" -gt 0 ]; then
    ROUND_HAS_EMERGENCY="yes"
  fi
  ROUND_HAS_COLLISION="no"
  if [ "${COLLISION_EVENT_COUNT:-0}" -gt 0 ]; then
    ROUND_HAS_COLLISION="yes"
  fi
  COLLISION_TYPE="NA"
  COLLISION_PAIR="NA"
  COLLISION_DISTANCE="NA"
  FIRST_COLLISION_LINE=$(grep -Em1 "HARD_COLLISION detected:" "$RUNTIME_CLEAN" || true)
  if [ -n "${FIRST_COLLISION_LINE:-}" ]; then
    COLLISION_TYPE=$(echo "$FIRST_COLLISION_LINE" | sed -nE 's/.*"type"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/p')
    COLLISION_DISTANCE=$(echo "$FIRST_COLLISION_LINE" | sed -nE 's/.*"distance"[[:space:]]*:[[:space:]]*([0-9]+(\.[0-9]+)?).*/\1/p')
    if [ "$COLLISION_TYPE" = "drone_drone" ]; then
      COLLISION_PAIR=$(echo "$FIRST_COLLISION_LINE" | sed -nE 's/.*"drone_a"[[:space:]]*:[[:space:]]*([0-9]+),[[:space:]]*"drone_b"[[:space:]]*:[[:space:]]*([0-9]+).*/\1-\2/p')
    elif [ "$COLLISION_TYPE" = "drone_obstacle" ]; then
      COLLISION_PAIR=$(echo "$FIRST_COLLISION_LINE" | sed -nE 's/.*"drone_id"[[:space:]]*:[[:space:]]*([0-9]+).*/drone-\1/p')
    fi
  fi
  [ -n "${COLLISION_TYPE:-}" ] || COLLISION_TYPE="NA"
  [ -n "${COLLISION_PAIR:-}" ] || COLLISION_PAIR="NA"
  [ -n "${COLLISION_DISTANCE:-}" ] || COLLISION_DISTANCE="NA"

  KEEP_FUZZ_REPORT=0
  if [ "${COLLISION_EVENT_COUNT:-0}" -gt 0 ] || [ "${EMERGENCY_COUNT:-0}" -gt 0 ]; then
    KEEP_FUZZ_REPORT=1
  fi
  if [ "$KEEP_FUZZ_REPORT" -eq 1 ]; then
    if [ -d "$ROUND_DIR/fuzz_report_staging" ]; then
      rm -rf "$ROUND_DIR/fuzz_report"
      mv "$ROUND_DIR/fuzz_report_staging" "$ROUND_DIR/fuzz_report"
    fi
    ABS_ROUND_FUZZ_REPORT_DIR="$(cd "$ROUND_DIR/fuzz_report" && pwd)"
    FUZZ_REPORT_DIR_VALUE="$ABS_ROUND_FUZZ_REPORT_DIR"
  else
    rm -rf "$ROUND_DIR/fuzz_report_staging" "$ROUND_DIR/fuzz_report"
    ABS_ROUND_FUZZ_REPORT_DIR=""
    FUZZ_REPORT_DIR_VALUE="NA"
  fi

  COMM_FUZZ_SEED_USED="NA"
  COMM_FUZZ_MODE_USED="NA"
  COMM_FUZZ_TARGET_USED="NA"
  COMM_FUZZ_START_S="NA"
  COMM_FUZZ_DURATION_S="NA"
  COMM_FUZZ_FACTOR_NAME="NA"
  COMM_FUZZ_FACTOR_VALUE="NA"
  COMM_FUZZ_TOPIC="NA"
  if [ "$FUZZ_REPORT_DIR_VALUE" != "NA" ]; then
    COMM_FUZZ_META_FILE="$ROUND_DIR/fuzz_report/$FUZZ_RUN_ID/comm_fuzz_meta.json"
    COMM_FUZZ_TRACE_FILE="$ROUND_DIR/fuzz_report/$FUZZ_RUN_ID/comm_fuzz_trace.jsonl"
    PLANNER_RX_TRACE_GLOB="$ROUND_DIR/fuzz_report/$FUZZ_RUN_ID/planner_rx_trace_drone_*.jsonl"
    PLANNER_SAFETY_TRACE_GLOB="$ROUND_DIR/fuzz_report/$FUZZ_RUN_ID/planner_safety_trace_drone_*.jsonl"
  else
    COMM_FUZZ_META_FILE=""
    COMM_FUZZ_TRACE_FILE=""
    PLANNER_RX_TRACE_GLOB=""
    PLANNER_SAFETY_TRACE_GLOB=""
  fi
  ROOT_CAUSE_REPORT_FILE="$ROUND_DIR/root_cause_report.json"
  ROOT_CAUSE_KV_FILE="$ROUND_DIR/root_cause_kv.txt"
  ROOT_CAUSE_PRIMARY="unknown"
  ROOT_CAUSE_TYPES="unknown"
  ROOT_CAUSE_CONFIDENCE="0"
  ROOT_CAUSE_SUMMARY="analysis_not_available"
  FIRST_COLLISION_TIME="NA"
  FIRST_EMERGENCY_TIME="NA"

  if [ "$FUZZ_REPORT_DIR_VALUE" != "NA" ]; then
    COMM_FULL_DUMP_PATH="$ROUND_DIR/fuzz_report/$FUZZ_RUN_ID/$COMM_FULL_DUMP_FILE_NAME"
    COMM_FULL_DUMP_META_PATH="$ROUND_DIR/fuzz_report/$FUZZ_RUN_ID/comm_full_dump_meta.json"
  else
    COMM_FULL_DUMP_PATH="NA"
    COMM_FULL_DUMP_META_PATH="NA"
  fi

  if [ -n "${COMM_FUZZ_META_FILE:-}" ] && [ -f "$COMM_FUZZ_META_FILE" ]; then
    COMM_FUZZ_SEED_USED=$(sed -nE 's/.*"seed"[[:space:]]*:[[:space:]]*([0-9-]+).*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
    COMM_FUZZ_MODE_USED=$(sed -nE 's/.*"mode"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
    COMM_FUZZ_TARGET_USED=$(sed -nE 's/.*"target_drone_id"[[:space:]]*:[[:space:]]*([0-9-]+).*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
    COMM_FUZZ_START_S=$(sed -nE 's/.*"window_start_s"[[:space:]]*:[[:space:]]*([0-9]+(\.[0-9]+)?).*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
    COMM_FUZZ_DURATION_S=$(sed -nE 's/.*"window_duration_s"[[:space:]]*:[[:space:]]*([0-9]+(\.[0-9]+)?).*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
    COMM_FUZZ_FACTOR_NAME=$(sed -nE 's/.*"factor_name"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
    COMM_FUZZ_FACTOR_VALUE=$(sed -nE 's/.*"factor_value"[[:space:]]*:[[:space:]]*([0-9]+(\.[0-9]+)?).*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
    COMM_FUZZ_TOPIC=$(sed -nE 's/.*"in_topic"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/p' "$COMM_FUZZ_META_FILE" | head -n 1 || true)
  fi

  [ -n "${COMM_FUZZ_SEED_USED:-}" ] || COMM_FUZZ_SEED_USED="NA"
  [ -n "${COMM_FUZZ_MODE_USED:-}" ] || COMM_FUZZ_MODE_USED="NA"
  [ -n "${COMM_FUZZ_TARGET_USED:-}" ] || COMM_FUZZ_TARGET_USED="NA"
  [ -n "${COMM_FUZZ_START_S:-}" ] || COMM_FUZZ_START_S="NA"
  [ -n "${COMM_FUZZ_DURATION_S:-}" ] || COMM_FUZZ_DURATION_S="NA"
  [ -n "${COMM_FUZZ_FACTOR_NAME:-}" ] || COMM_FUZZ_FACTOR_NAME="NA"
  [ -n "${COMM_FUZZ_FACTOR_VALUE:-}" ] || COMM_FUZZ_FACTOR_VALUE="NA"
  [ -n "${COMM_FUZZ_TOPIC:-}" ] || COMM_FUZZ_TOPIC="NA"

  if [ "$COMM_FUZZ_ENABLED" = "yes" ]; then
    # If bridge/node didn't write comm_fuzz_meta.json, produce concrete sampled values for this round.
    # Prefer explicit ROUND_PLAN_* (set when seed-pool/evolution selected). Otherwise sample deterministically.
    if [ "${COMM_FUZZ_SEED_USED:-NA}" = "NA" ]; then
      COMM_FUZZ_SEED_USED="${COMM_FUZZ_SEED:-$(choose_seed_pure_random "$i") }"
    fi
    if [ "${COMM_FUZZ_MODE_USED:-NA}" = "NA" ]; then
      # If a plan line was selected earlier, use it; else sample a concrete plan now.
      if [ -n "${ROUND_PLAN_LINE:-}" ] && [ "${ROUND_PLAN_LINE}" != "" ]; then
        # ROUND_PLAN_LINE may be either the pool line (with seed first) or sample_mixed_comm_plan (no seed).
        if [[ "${ROUND_PLAN_LINE}" == *$'\t'* && "${ROUND_PLAN_LINE}" =~ ^[0-9]+\t ]]; then
          IFS=$'\t' read -r COMM_FUZZ_SEED_USED COMM_FUZZ_MODE_USED ROUND_PLAN_START_S ROUND_PLAN_DURATION_S ROUND_PLAN_FACTOR_NAME ROUND_PLAN_FACTOR_VALUE _ <<< "$ROUND_PLAN_LINE"
        else
          IFS=$'\t' read -r COMM_FUZZ_MODE_USED ROUND_PLAN_START_S ROUND_PLAN_DURATION_S ROUND_PLAN_FACTOR_NAME ROUND_PLAN_FACTOR_VALUE <<< "$(echo "$ROUND_PLAN_LINE")"
        fi
      else
        ROUND_PLAN_LINE=$(sample_mixed_comm_plan "$i")
        IFS=$'\t' read -r COMM_FUZZ_MODE_USED ROUND_PLAN_START_S ROUND_PLAN_DURATION_S ROUND_PLAN_FACTOR_NAME ROUND_PLAN_FACTOR_VALUE <<< "$ROUND_PLAN_LINE"
      fi
    fi
    # Fill target, start, duration, factor from plan variables.
    COMM_FUZZ_TARGET_USED="${COMM_FUZZ_TARGET_USED:-${COMM_FUZZ_TARGET_DRONE_ID:-NA}}"
    if [ -n "${ROUND_PLAN_START_S:-}" ] && [ "${ROUND_PLAN_START_S:-}" != "" ]; then
      COMM_FUZZ_START_S="${ROUND_PLAN_START_S}"
    else
      COMM_FUZZ_START_S="${ROUND_COMM_FUZZ_START_MIN_SEC:-NA}"
    fi
    if [ -n "${ROUND_PLAN_DURATION_S:-}" ] && [ "${ROUND_PLAN_DURATION_S:-}" != "" ]; then
      COMM_FUZZ_DURATION_S="${ROUND_PLAN_DURATION_S}"
    else
      COMM_FUZZ_DURATION_S="${ROUND_COMM_FUZZ_DURATION_MIN_SEC:-NA}"
    fi
    if [ -n "${ROUND_PLAN_FACTOR_NAME:-}" ] && [ "${ROUND_PLAN_FACTOR_NAME:-}" != "" ]; then
      COMM_FUZZ_FACTOR_NAME="${ROUND_PLAN_FACTOR_NAME}"
      COMM_FUZZ_FACTOR_VALUE="${ROUND_PLAN_FACTOR_VALUE:-NA}"
    else
      COMM_FUZZ_FACTOR_NAME="${ROUND_COMM_FUZZ_DELAY_MIN_MS:+delay_ms}"
      COMM_FUZZ_FACTOR_VALUE="${ROUND_COMM_FUZZ_DELAY_MIN_MS:-}";
      if [ -z "${COMM_FUZZ_FACTOR_NAME}" ]; then
        COMM_FUZZ_FACTOR_NAME="${ROUND_COMM_FUZZ_JITTER_MIN_MS:+jitter_ms}"
        COMM_FUZZ_FACTOR_VALUE="${ROUND_COMM_FUZZ_JITTER_MIN_MS:-}"
      fi
      if [ -z "${COMM_FUZZ_FACTOR_NAME}" ]; then
        COMM_FUZZ_FACTOR_NAME="${ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MIN:+noise_sigma_pos}"
        COMM_FUZZ_FACTOR_VALUE="${ROUND_COMM_FUZZ_NOISE_SIGMA_POS_MIN:-}"
      fi
    fi
  fi

  if python3 planner/plan_manage/scripts/analyze_round_root_cause.py \
    --runtime-log "$RUNTIME_CLEAN" \
    --fuzz-report-dir "$ROUND_DIR/fuzz_report" \
    --run-id "$FUZZ_RUN_ID" \
    --comm-fuzz-meta "$COMM_FUZZ_META_FILE" \
    --output "$ROOT_CAUSE_REPORT_FILE" \
    --shell-kv "$ROOT_CAUSE_KV_FILE"; then
    ROOT_CAUSE_PRIMARY=$(sed -n 's/^primary_incident=//p' "$ROOT_CAUSE_KV_FILE" | head -n 1)
    ROOT_CAUSE_TYPES=$(sed -n 's/^incident_types=//p' "$ROOT_CAUSE_KV_FILE" | head -n 1)
    ROOT_CAUSE_CONFIDENCE=$(sed -n 's/^confidence=//p' "$ROOT_CAUSE_KV_FILE" | head -n 1)
    ROOT_CAUSE_SUMMARY=$(sed -n 's/^summary=//p' "$ROOT_CAUSE_KV_FILE" | head -n 1)
    FIRST_COLLISION_TIME=$(sed -n 's/^first_collision_ros_time=//p' "$ROOT_CAUSE_KV_FILE" | head -n 1)
    FIRST_EMERGENCY_TIME=$(sed -n 's/^first_emergency_ros_time=//p' "$ROOT_CAUSE_KV_FILE" | head -n 1)
  fi

  [ -n "${ROOT_CAUSE_PRIMARY:-}" ] || ROOT_CAUSE_PRIMARY="unknown"
  [ -n "${ROOT_CAUSE_TYPES:-}" ] || ROOT_CAUSE_TYPES="unknown"
  [ -n "${ROOT_CAUSE_CONFIDENCE:-}" ] || ROOT_CAUSE_CONFIDENCE="0"
  [ -n "${ROOT_CAUSE_SUMMARY:-}" ] || ROOT_CAUSE_SUMMARY="analysis_not_available"
  [ -n "${FIRST_COLLISION_TIME:-}" ] || FIRST_COLLISION_TIME="NA"
  [ -n "${FIRST_EMERGENCY_TIME:-}" ] || FIRST_EMERGENCY_TIME="NA"

  LAST_TIME_MS=$(grep -oE 'Time:[[:space:]]*[0-9]+(\.[0-9]+)?ms' "$CLEAN" | sed -E 's/Time:[[:space:]]*([0-9.]+)ms/\1/' | tail -n 1 || true)
  AVG_TIME_MS=$(grep -oE 'avg=[0-9]+(\.[0-9]+)?ms' "$CLEAN" | sed -E 's/avg=([0-9.]+)ms/\1/' | tail -n 1 || true)

  [ -n "${LAST_TIME_MS:-}" ] || LAST_TIME_MS="NA"
  [ -n "${AVG_TIME_MS:-}" ] || AVG_TIME_MS="NA"
  [ -n "${FATAL_HIT:-}" ] || FATAL_HIT="NA"

  STRICT_RESULT="FAIL"
  if [ "${SUCCESS_COUNT:-0}" -gt 0 ] && [ "${EMERGENCY_COUNT:-0}" -eq 0 ] && [ "${FATAL_COUNT:-0}" -eq 0 ]; then
    STRICT_RESULT="SUCCESS"
  fi

  BENCH_RESULT="FAIL"
  if [ "${SUCCESS_COUNT:-0}" -ge 3 ] && [ "${EMERGENCY_COUNT:-0}" -eq 0 ]; then
    BENCH_RESULT="SUCCESS"
  fi

  TOTAL_REPLAN_COUNT=$((TOTAL_REPLAN_COUNT + REPLAN_COUNT))
  TOTAL_SUCCESS_COUNT=$((TOTAL_SUCCESS_COUNT + SUCCESS_COUNT))
  TOTAL_FAIL_COUNT=$((TOTAL_FAIL_COUNT + FAIL_COUNT))
  TOTAL_EMERGENCY_COUNT=$((TOTAL_EMERGENCY_COUNT + EMERGENCY_COUNT))
  TOTAL_FATAL_COUNT=$((TOTAL_FATAL_COUNT + FATAL_COUNT))
  TOTAL_FATAL_RUNTIME_COUNT=$((TOTAL_FATAL_RUNTIME_COUNT + FATAL_RUNTIME_COUNT))
  TOTAL_FATAL_SHUTDOWN_COUNT=$((TOTAL_FATAL_SHUTDOWN_COUNT + FATAL_SHUTDOWN_COUNT))
  TOTAL_COLLISION_EVENT_COUNT=$((TOTAL_COLLISION_EVENT_COUNT + COLLISION_EVENT_COUNT))
  TOTAL_DRONE_DRONE_COLLISION_COUNT=$((TOTAL_DRONE_DRONE_COLLISION_COUNT + DRONE_DRONE_COLLISION_COUNT))
  TOTAL_DRONE_OBSTACLE_COLLISION_COUNT=$((TOTAL_DRONE_OBSTACLE_COLLISION_COUNT + DRONE_OBSTACLE_COLLISION_COUNT))
  TOTAL_MANDATORY_STOP_COUNT=$((TOTAL_MANDATORY_STOP_COUNT + MANDATORY_STOP_COUNT))
  if [ "$ROUND_HAS_EMERGENCY" = "yes" ]; then
    ROUNDS_WITH_EMERGENCY=$((ROUNDS_WITH_EMERGENCY + 1))
  fi
  if [ "$ROUND_HAS_COLLISION" = "yes" ]; then
    ROUNDS_WITH_COLLISION=$((ROUNDS_WITH_COLLISION + 1))
  fi
  if [ "$LOCKERR_RUNTIME_COUNT" -gt 0 ]; then
    ROUNDS_WITH_LOCKERR_RUNTIME=$((ROUNDS_WITH_LOCKERR_RUNTIME + 1))
  fi
  if [ "$LOCKERR_SHUTDOWN_COUNT" -gt 0 ]; then
    ROUNDS_WITH_LOCKERR_SHUTDOWN=$((ROUNDS_WITH_LOCKERR_SHUTDOWN + 1))
  fi
  if [ "$PROCESS_CRASH_COUNT" -gt 0 ]; then
    ROUNDS_WITH_PROCESS_CRASH=$((ROUNDS_WITH_PROCESS_CRASH + 1))
  fi
  if is_number "$LAST_TIME_MS"; then
    TOTAL_LAST_TIME_MS=$(awk -v a="$TOTAL_LAST_TIME_MS" -v b="$LAST_TIME_MS" 'BEGIN { printf "%.6f", a + b }')
    TOTAL_LAST_TIME_COUNT=$((TOTAL_LAST_TIME_COUNT + 1))
  fi
  if is_number "$AVG_TIME_MS"; then
    TOTAL_AVG_TIME_MS=$(awk -v a="$TOTAL_AVG_TIME_MS" -v b="$AVG_TIME_MS" 'BEGIN { printf "%.6f", a + b }')
    TOTAL_AVG_TIME_COUNT=$((TOTAL_AVG_TIME_COUNT + 1))
  fi
  if [ "$BENCH_RESULT" = "SUCCESS" ]; then
    BENCH_SUCCESS_ROUNDS=$((BENCH_SUCCESS_ROUNDS + 1))
  fi
  if [ "$STRICT_RESULT" = "SUCCESS" ]; then
    STRICT_SUCCESS_ROUNDS=$((STRICT_SUCCESS_ROUNDS + 1))
  fi
  if [ "$HAS_REPLAN" = "yes" ]; then
    ROUNDS_WITH_REPLAN=$((ROUNDS_WITH_REPLAN + 1))
  fi

  if [ "$COMM_FUZZ_SEED_GEN_MODE_NORMALIZED" = "ripple_biased" ] && [ "$COMM_FUZZ_ENABLED" = "yes" ]; then
    ANOMALY_LEVEL="none"
    if [ "${COLLISION_EVENT_COUNT:-0}" -gt 0 ]; then
      ANOMALY_LEVEL="high"
    elif [ "${EMERGENCY_COUNT:-0}" -gt 0 ]; then
      ANOMALY_LEVEL="mid"
    elif [ "${REPLAN_COUNT:-0}" -gt 1000 ]; then
      ANOMALY_LEVEL="low"
    fi

    SEVERE_RESOURCE_EXHAUSTION="false"
    if [ "$ANOMALY_LEVEL" = "low" ] && [ "${REPLAN_COUNT:-0}" -gt 10000 ]; then
      SEVERE_RESOURCE_EXHAUSTION="true"
    fi

    VALID_POOL_RECORD="no"
    PARAM_KEY="round${i}_fallback"
    if [ "$COMM_FUZZ_MODE_USED" != "NA" ] && [ "$COMM_FUZZ_FACTOR_NAME" != "NA" ] && is_number "$COMM_FUZZ_START_S" && is_number "$COMM_FUZZ_DURATION_S" && is_number "$COMM_FUZZ_FACTOR_VALUE"; then
      PARAM_KEY=$(python3 - "$COMM_FUZZ_MODE_USED" "$COMM_FUZZ_START_S" "$COMM_FUZZ_DURATION_S" "$COMM_FUZZ_FACTOR_NAME" "$COMM_FUZZ_FACTOR_VALUE" <<'PY'
import sys

mode = str(sys.argv[1]).strip().lower()
start_s = float(sys.argv[2])
duration_s = float(sys.argv[3])
factor_name = str(sys.argv[4]).strip().lower()
factor_value = float(sys.argv[5])
print(f"{mode}|{start_s:.3f}|{duration_s:.3f}|{factor_name}|{factor_value:.4f}")
PY
)
      VALID_POOL_RECORD="yes"
    fi

    # Hard gate: only persist rounds where anomaly_level is one of high/mid/low
    if [ "$VALID_POOL_RECORD" = "yes" ] && [ "$ANOMALY_LEVEL" != "none" ]; then
      echo -e "${i}\t${PARAM_KEY}\t${COMM_FUZZ_SEED_USED}\t${COMM_FUZZ_MODE_USED}\t${COMM_FUZZ_START_S}\t${COMM_FUZZ_DURATION_S}\t${COMM_FUZZ_FACTOR_NAME}\t${COMM_FUZZ_FACTOR_VALUE}\t${ANOMALY_LEVEL}\t${SEVERE_RESOURCE_EXHAUSTION}\t${i}\t${ROUND_PLAN_PHASE}\t${ROUND_PLAN_SOURCE}\t${ROUND_SOURCE_BUCKET}\t${ROUND_SELECTED_SEED_LEVEL}" >> "$SEED_POOL_ROUNDS_FILE"
      update_seed_pool_file "$SEED_POOL_ROUNDS_FILE" "$SEED_POOL_FILE" "50" "$SUMMARY_CSV"
    fi
  fi

  {
    echo "round=$i"
    echo "timestamp=$TS"
    echo "result=$BENCH_RESULT"
    echo "bench_result=$BENCH_RESULT"
    echo "strict_result=$STRICT_RESULT"
    echo "success_count=$SUCCESS_COUNT"
    echo "fail_count=$FAIL_COUNT"
    echo "collision_monitor_enabled=$COLLISION_MONITOR_ENABLED"
    echo "comm_fuzz_enabled=$COMM_FUZZ_ENABLED"
    echo "comm_fuzz_seed=$COMM_FUZZ_SEED_USED"
    echo "comm_fuzz_mode=$COMM_FUZZ_MODE_USED"
    echo "comm_fuzz_target_drone_id=$COMM_FUZZ_TARGET_USED"
    echo "comm_fuzz_start_s=$COMM_FUZZ_START_S"
    echo "comm_fuzz_duration_s=$COMM_FUZZ_DURATION_S"
    echo "comm_fuzz_factor_name=$COMM_FUZZ_FACTOR_NAME"
    echo "comm_fuzz_factor_value=$COMM_FUZZ_FACTOR_VALUE"
    echo "comm_fuzz_topic=$COMM_FUZZ_TOPIC"
    echo "collision_event_count=$COLLISION_EVENT_COUNT"
    echo "drone_drone_collision_count=$DRONE_DRONE_COLLISION_COUNT"
    echo "drone_obstacle_collision_count=$DRONE_OBSTACLE_COLLISION_COUNT"
    echo "collision_type=$COLLISION_TYPE"
    echo "collision_pair=$COLLISION_PAIR"
    echo "collision_distance=$COLLISION_DISTANCE"
    echo "mandatory_stop_count=$MANDATORY_STOP_COUNT"
    echo "fuzz_report_dir=$FUZZ_REPORT_DIR_VALUE"
    echo "stl_metrics_summary_file=$STL_METRICS_SUMMARY_FILE"
    echo "stl_metrics_trace_file=$STL_METRICS_TRACE_FILE"
    echo "adaptive_replan_latency_meta_file=$ADAPTIVE_REPLAN_META_FILE"
    echo "adaptive_replan_latency_events_file=$ADAPTIVE_REPLAN_EVENTS_FILE"
    echo "adaptive_replan_latency_trace_file=$ADAPTIVE_REPLAN_TRACE_FILE"
    echo "rule_pool_latency_meta_file=$RULE_POOL_META_FILE"
    echo "rule_pool_latency_events_file=$RULE_POOL_EVENTS_FILE"
    echo "rule_pool_latency_trace_file=$RULE_POOL_TRACE_FILE"
    echo "comm_fuzz_meta_file=$COMM_FUZZ_META_FILE"
    echo "comm_fuzz_trace_file=$COMM_FUZZ_TRACE_FILE"
    echo "planner_rx_trace_files=$PLANNER_RX_TRACE_GLOB"
    echo "planner_safety_trace_files=$PLANNER_SAFETY_TRACE_GLOB"
    echo "comm_full_dump_sidecar_enabled=$COMM_FULL_DUMP_SIDECAR_ENABLED"
    echo "comm_full_dump_sidecar_log=$SIDECAR_LOG"
    echo "comm_full_dump_file=$COMM_FULL_DUMP_PATH"
    echo "comm_full_dump_meta_file=$COMM_FULL_DUMP_META_PATH"
    echo "has_replan=$HAS_REPLAN"
    echo "replan_count=$REPLAN_COUNT"
    echo "last_time_ms=$LAST_TIME_MS"
    echo "avg_time_ms=$AVG_TIME_MS"
    echo "emergency_count=$EMERGENCY_COUNT"
    echo "fatal_count=$FATAL_COUNT"
    echo "fatal_runtime_count=$FATAL_RUNTIME_COUNT"
    echo "fatal_shutdown_count=$FATAL_SHUTDOWN_COUNT"
    echo "lock_error_runtime_count=$LOCKERR_RUNTIME_COUNT"
    echo "lock_error_shutdown_count=$LOCKERR_SHUTDOWN_COUNT"
    echo "process_crash_count=$PROCESS_CRASH_COUNT"
    echo "crash_node_top=$CRASH_NODE_TOP"
    echo "crash_node_top_count=$CRASH_NODE_TOP_COUNT"
    echo "fatal_pattern=$FATAL_HIT"
    echo "root_cause_primary=$ROOT_CAUSE_PRIMARY"
    echo "root_cause_types=$ROOT_CAUSE_TYPES"
    echo "root_cause_confidence=$ROOT_CAUSE_CONFIDENCE"
    echo "root_cause_summary=$ROOT_CAUSE_SUMMARY"
    echo "first_collision_time=$FIRST_COLLISION_TIME"
    echo "first_emergency_time=$FIRST_EMERGENCY_TIME"
    echo "root_cause_report=$ROOT_CAUSE_REPORT_FILE"
    echo "raw_log=$RAW"
    echo "clean_log=$CLEAN"
    echo "runtime_clean_log=$RUNTIME_CLEAN"
    echo "shutdown_clean_log=$SHUTDOWN_CLEAN"
  } > "$TXT"

  {
    echo "round=$i bench=$BENCH_RESULT strict=$STRICT_RESULT success=$SUCCESS_COUNT fail=$FAIL_COUNT replan=$REPLAN_COUNT has_replan=$HAS_REPLAN comm_fuzz_enabled=$COMM_FUZZ_ENABLED comm_fuzz_mode=$COMM_FUZZ_MODE_USED comm_fuzz_seed=$COMM_FUZZ_SEED_USED comm_fuzz_target=$COMM_FUZZ_TARGET_USED comm_fuzz_start_s=$COMM_FUZZ_START_S comm_fuzz_duration_s=$COMM_FUZZ_DURATION_S comm_fuzz_factor=$COMM_FUZZ_FACTOR_NAME:$COMM_FUZZ_FACTOR_VALUE comm_fuzz_topic=$COMM_FUZZ_TOPIC collision_events=$COLLISION_EVENT_COUNT drone_drone_collisions=$DRONE_DRONE_COLLISION_COUNT drone_obstacle_collisions=$DRONE_OBSTACLE_COLLISION_COUNT collision_type=$COLLISION_TYPE collision_pair=$COLLISION_PAIR collision_distance=$COLLISION_DISTANCE mandatory_stop=$MANDATORY_STOP_COUNT last_ms=$LAST_TIME_MS avg_ms=$AVG_TIME_MS emergency=$EMERGENCY_COUNT fatal=$FATAL_COUNT fatal_runtime=$FATAL_RUNTIME_COUNT fatal_shutdown=$FATAL_SHUTDOWN_COUNT lockerr_runtime=$LOCKERR_RUNTIME_COUNT lockerr_shutdown=$LOCKERR_SHUTDOWN_COUNT process_crash_count=$PROCESS_CRASH_COUNT crash_node_top=$CRASH_NODE_TOP crash_node_top_count=$CRASH_NODE_TOP_COUNT root_cause_primary=$ROOT_CAUSE_PRIMARY root_cause_types=$ROOT_CAUSE_TYPES root_cause_confidence=$ROOT_CAUSE_CONFIDENCE"
    echo "detail_txt=$TXT"
    echo "round_dir=$ROUND_DIR"
    echo "fuzz_report_dir=$FUZZ_REPORT_DIR_VALUE"
    echo "root_cause_report=$ROOT_CAUSE_REPORT_FILE"
    echo "root_cause_summary=$ROOT_CAUSE_SUMMARY"
    echo
  } >> "$SUMMARY"

  {
    printf "round,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
      "$(csv_escape "$i")" \
      "$(csv_escape "$TS")" \
      "$(csv_escape "$BENCH_RESULT")" \
      "$(csv_escape "$BENCH_RESULT")" \
      "$(csv_escape "$STRICT_RESULT")" \
      "$(csv_escape "$SUCCESS_COUNT")" \
      "$(csv_escape "$FAIL_COUNT")" \
      "$(csv_escape "$COLLISION_MONITOR_ENABLED")" \
      "$(csv_escape "$COMM_FUZZ_ENABLED")" \
      "$(csv_escape "$COMM_FUZZ_SEED_USED")" \
      "$(csv_escape "$COMM_FUZZ_MODE_USED")" \
      "$(csv_escape "$COMM_FUZZ_TARGET_USED")" \
      "$(csv_escape "$COMM_FUZZ_START_S")" \
      "$(csv_escape "$COMM_FUZZ_DURATION_S")" \
      "$(csv_escape "$COMM_FUZZ_FACTOR_NAME")" \
      "$(csv_escape "$COMM_FUZZ_FACTOR_VALUE")" \
      "$(csv_escape "$COMM_FUZZ_TOPIC")" \
      "$(csv_escape "$COLLISION_EVENT_COUNT")" \
      "$(csv_escape "$DRONE_DRONE_COLLISION_COUNT")" \
      "$(csv_escape "$DRONE_OBSTACLE_COLLISION_COUNT")" \
      "$(csv_escape "$COLLISION_TYPE")" \
      "$(csv_escape "$COLLISION_PAIR")" \
      "$(csv_escape "$COLLISION_DISTANCE")" \
      "$(csv_escape "$MANDATORY_STOP_COUNT")" \
      "$(csv_escape "$FUZZ_REPORT_DIR_VALUE")" \
      "$(csv_escape "NA")" \
      "$(csv_escape "$HAS_REPLAN")" \
      "$(csv_escape "$REPLAN_COUNT")" \
      "$(csv_escape "$LAST_TIME_MS")" \
      "$(csv_escape "$AVG_TIME_MS")" \
      "$(csv_escape "$EMERGENCY_COUNT")" \
      "$(csv_escape "$FATAL_COUNT")" \
        "$(csv_escape "$ROOT_CAUSE_PRIMARY")" \
        "$(csv_escape "$ROOT_CAUSE_TYPES")" \
        "$(csv_escape "$ROOT_CAUSE_CONFIDENCE")" \
        "$(csv_escape "$ROOT_CAUSE_SUMMARY")" \
        "$(csv_escape "$ROOT_CAUSE_REPORT_FILE")" \
      "$(csv_escape "$FIRST_COLLISION_TIME")" \
      "$(csv_escape "$FIRST_EMERGENCY_TIME")" \
      "$(csv_escape "$TXT")" \
      "$(csv_escape "$ROUND_DIR")"
  } >> "$SUMMARY_CSV"

  echo "[Round $i] done -> $TXT"
done

AVG_REPLAN_COUNT=$(safe_avg "$TOTAL_REPLAN_COUNT" "$RUNS")
AVG_SUCCESS_COUNT=$(safe_avg "$TOTAL_SUCCESS_COUNT" "$RUNS")
AVG_FAIL_COUNT=$(safe_avg "$TOTAL_FAIL_COUNT" "$RUNS")
AVG_LAST_TIME_MS=$(safe_avg "$TOTAL_LAST_TIME_MS" "$TOTAL_LAST_TIME_COUNT")
AVG_AVG_TIME_MS=$(safe_avg "$TOTAL_AVG_TIME_MS" "$TOTAL_AVG_TIME_COUNT")

if [ "$STL_METRICS_MONITOR_ENABLED" = "yes" ]; then
  if ! python3 planner/plan_manage/scripts/aggregate_stl_attack_scenarios.py \
    "$BATCH_DIR" \
    --summary-csv "$SUMMARY_CSV" \
    --output "$BATCH_STL_ATTACK_SCENARIOS_FILE" >/dev/null 2>&1; then
    write_missing_stl_attack_scenarios "$BATCH_STL_ATTACK_SCENARIOS_FILE"
  fi
else
  write_missing_stl_attack_scenarios "$BATCH_STL_ATTACK_SCENARIOS_FILE"
fi

{
  echo "===== batch_overview ====="
  echo "total_rounds=$RUNS"
  echo "bench_success_rounds=$BENCH_SUCCESS_ROUNDS"
  echo "strict_success_rounds=$STRICT_SUCCESS_ROUNDS"
  echo "rounds_with_replan=$ROUNDS_WITH_REPLAN"
  echo "total_replan_count=$TOTAL_REPLAN_COUNT"
  echo "total_success_count=$TOTAL_SUCCESS_COUNT"
  echo "total_fail_count=$TOTAL_FAIL_COUNT"
  echo "total_emergency_count=$TOTAL_EMERGENCY_COUNT"
  echo "total_fatal_count=$TOTAL_FATAL_COUNT"
  echo "total_fatal_runtime_count=$TOTAL_FATAL_RUNTIME_COUNT"
  echo "total_fatal_shutdown_count=$TOTAL_FATAL_SHUTDOWN_COUNT"
  echo "rounds_with_emergency=$ROUNDS_WITH_EMERGENCY"
  echo "rounds_with_collision=$ROUNDS_WITH_COLLISION"
  echo "rounds_with_lock_error_runtime=$ROUNDS_WITH_LOCKERR_RUNTIME"
  echo "rounds_with_lock_error_shutdown=$ROUNDS_WITH_LOCKERR_SHUTDOWN"
  echo "rounds_with_process_crash=$ROUNDS_WITH_PROCESS_CRASH"
  echo "round_emergency_rate=$(safe_avg "$ROUNDS_WITH_EMERGENCY" "$RUNS")"
  echo "round_collision_rate=$(safe_avg "$ROUNDS_WITH_COLLISION" "$RUNS")"
  echo "total_collision_event_count=$TOTAL_COLLISION_EVENT_COUNT"
  echo "total_drone_drone_collision_count=$TOTAL_DRONE_DRONE_COLLISION_COUNT"
  echo "total_drone_obstacle_collision_count=$TOTAL_DRONE_OBSTACLE_COLLISION_COUNT"
  echo "total_mandatory_stop_count=$TOTAL_MANDATORY_STOP_COUNT"
  echo "avg_replan_count_per_round=$AVG_REPLAN_COUNT"
  echo "avg_success_count_per_round=$AVG_SUCCESS_COUNT"
  echo "avg_fail_count_per_round=$AVG_FAIL_COUNT"
  echo "avg_last_time_ms=$AVG_LAST_TIME_MS"
  echo "avg_avg_time_ms=$AVG_AVG_TIME_MS"
  echo "stl_attack_scenarios_file=$BATCH_STL_ATTACK_SCENARIOS_FILE"
} >> "$SUMMARY"

{
  printf "batch_overview,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
    "$(csv_escape "ALL")" \
    "$(csv_escape "$BATCH_TS")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "$BENCH_SUCCESS_ROUNDS/$RUNS")" \
    "$(csv_escape "$STRICT_SUCCESS_ROUNDS/$RUNS")" \
    "$(csv_escape "$AVG_SUCCESS_COUNT")" \
    "$(csv_escape "$AVG_FAIL_COUNT")" \
    "$(csv_escape "$COLLISION_MONITOR_ENABLED")" \
    "$(csv_escape "$COMM_FUZZ_ENABLED")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "$COMM_FUZZ_MODE")" \
    "$(csv_escape "$COMM_FUZZ_TARGET_DRONE_ID")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "$TOTAL_COLLISION_EVENT_COUNT")" \
    "$(csv_escape "$TOTAL_DRONE_DRONE_COLLISION_COUNT")" \
    "$(csv_escape "$TOTAL_DRONE_OBSTACLE_COLLISION_COUNT")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "$TOTAL_MANDATORY_STOP_COUNT")" \
    "$(csv_escape "$BATCH_DIR")" \
    "$(csv_escape "$BATCH_STL_ATTACK_SCENARIOS_FILE")" \
    "$(csv_escape "$ROUNDS_WITH_REPLAN/$RUNS")" \
    "$(csv_escape "$AVG_REPLAN_COUNT")" \
    "$(csv_escape "$AVG_LAST_TIME_MS")" \
    "$(csv_escape "$AVG_AVG_TIME_MS")" \
    "$(csv_escape "$TOTAL_EMERGENCY_COUNT")" \
    "$(csv_escape "$TOTAL_FATAL_COUNT")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "$SUMMARY")" \
    "$(csv_escape "$BATCH_DIR")"
} >> "$SUMMARY_CSV"

echo "All rounds finished."
echo "Summary: $SUMMARY"
echo "Summary CSV: $SUMMARY_CSV"
