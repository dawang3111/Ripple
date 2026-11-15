#!/usr/bin/env bash
# Circle fuzz batch runner — positional args 1–14 match swarm-playground/main_ws/src/run_headless.sh.
#
# Positional: RUNS RUN_SEC OUT_DIR ENABLE_COLLISION_MONITOR ENABLE_COMM_FUZZ COMM_FUZZ_MODE
#   COMM_FUZZ_TARGET_DRONE_ID COMM_FUZZ_SEED_BASE START_MIN MAX DURATION_MIN MAX
#   COMM_FUZZ_SEED_GEN_MODE (pure_random|ripple_biased) COMM_FUZZ_KEY_TIME_SEC
#
# Circle-only env (defaults in parentheses): CIRCLE_LAUNCH_FILE (circle_exchange.launch),
#   DRONE_NUM (10), ODOM_SUFFIX (visual_slam/odom), ENABLE_OBSTACLE_COLLISION (true),
#   ENABLE_DRONE_COLLISION (true), MAX_TOTAL_RUNTIME_SEC (0). Optional: ENABLE_COMM_FUZZ when $5 omitted.
#
# Ripple/heuristic: COMM_FUZZ_POOL_MUTATION_RATIO (0.10), seed pool files under each batch dir.
#
# Incident TX/RX MINCOTraj dump (same sidecar as run_headless.sh): set env before invoking, e.g.
#   ENABLE_COMM_FULL_DUMP_SIDECAR=true COMM_FULL_DUMP_BUFFER_MAX=4000 \
#   COMM_FULL_DUMP_FILE_NAME=comm_full_dump_on_incident.jsonl ./circlefuzz.sh ...
# Custom sidecar topics require editing planner/plan_manage/scripts/comm_full_dump_sidecar.py private params.
#
# Verify: comm off → no pool writes; pure_random + comm on → per-round mixed sample + seed_base+i;
#   ripple_biased → warmup ceil(0.2*RUNS) then pool evolution when anomalies occur.
#
# 两组通信 fuzz 输入（mode 均为 random：每轮随机 delay | jitter | noise）。launch 默认与「基线」一致；强化通过 CLI 第 9–12 位与/或 COMM_FUZZ_* 环境变量覆盖。
#
# | 项目 | 配置 A：随机 fuzz 基线 | 配置 B：强化 |
# |------|------------------------|--------------|
# | mode | random | random |
# | 开始窗口 | 5–25 s | 3–8 s |
# | 持续窗口 | 5–20 s | 25–40 s |
# | delay | 80–250 ms | 150–400 ms |
# | jitter | 30–120 ms | 60–200 ms |
# | noise sigma | 0.05–0.20 | 0.15–0.40 |
# | 目标机 | 3 | 3 |
# | seed | 20260407 起按轮递增 | 同左 |
#
# 配置 A（基线示例，依赖脚本内默认 COMM_FUZZ_* 与下方位置参数）:
#   ./circlefuzz.sh RUNS RUN_SEC OUT_DIR true true random 3 20260407 \
#     5.0 25.0 5.0 20.0 pure_random 12.0
#
# 配置 B（强化：时间窗 3–8 s / 25–40 s；三类强度加强）:
#   COMM_FUZZ_DELAY_MIN_MS=150 COMM_FUZZ_DELAY_MAX_MS=400 \
#   COMM_FUZZ_JITTER_MIN_MS=60 COMM_FUZZ_JITTER_MAX_MS=200 \
#   COMM_FUZZ_NOISE_SIGMA_POS_MIN=0.15 COMM_FUZZ_NOISE_SIGMA_POS_MAX=0.40 \
#   ./circlefuzz.sh RUNS RUN_SEC OUT_DIR true true random 3 20260407 \
#     3.0 8.0 25.0 40.0 pure_random 12.0
#
# RUN_SEC 建议 >= start_max + duration_max（强化约 48 s，常用 60）。
# 一键强化（仅当对应环境变量 / 位置参数未显式设置时填入上表强化默认值）: COMM_FUZZ_PROFILE=strong
set -euo pipefail

if [[ "${5:-}" == *.launch ]]; then
  cat <<'EOF'
ERROR: obsolete circlefuzz.sh CLI (positional launch file was removed).

circlefuzz.sh now aligns with run_headless.sh for args 1–14. Example:

  ./circlefuzz.sh 5 120 ../results/circlefuzz true true random 3 20260407 \
    5 25 5 20 pure_random 12.0

Circle-only settings are environment variables, e.g.:

  CIRCLE_LAUNCH_FILE=circle_exchange.launch DRONE_NUM=10 ./circlefuzz.sh ...

EOF
  exit 1
fi

RUNS=${1:-5}
RUN_SEC=${2:-120}
OUT_DIR=${3:-../results/circlefuzz}
ENABLE_COLLISION_MONITOR=${4:-true}
ENABLE_COMM_FUZZ_CLI=${5:-${ENABLE_COMM_FUZZ:-true}}
COMM_FUZZ_MODE=${6:-${COMM_FUZZ_MODE_CFG:-random}}
COMM_FUZZ_TARGET_DRONE_ID=${7:-${COMM_FUZZ_TARGET_DRONE_ID:-3}}
COMM_FUZZ_SEED_BASE=${8:-${COMM_FUZZ_SEED_BASE:-20260407}}

COMM_FUZZ_PROFILE=${COMM_FUZZ_PROFILE:-}
if [ "$COMM_FUZZ_PROFILE" = "strong" ]; then
  COMM_FUZZ_DELAY_MIN_MS=${COMM_FUZZ_DELAY_MIN_MS:-150.0}
  COMM_FUZZ_DELAY_MAX_MS=${COMM_FUZZ_DELAY_MAX_MS:-400.0}
  COMM_FUZZ_JITTER_MIN_MS=${COMM_FUZZ_JITTER_MIN_MS:-60.0}
  COMM_FUZZ_JITTER_MAX_MS=${COMM_FUZZ_JITTER_MAX_MS:-200.0}
  COMM_FUZZ_NOISE_SIGMA_POS_MIN=${COMM_FUZZ_NOISE_SIGMA_POS_MIN:-0.15}
  COMM_FUZZ_NOISE_SIGMA_POS_MAX=${COMM_FUZZ_NOISE_SIGMA_POS_MAX:-0.40}
  COMM_FUZZ_START_MIN_SEC=${COMM_FUZZ_START_MIN_SEC:-3.0}
  COMM_FUZZ_START_MAX_SEC=${COMM_FUZZ_START_MAX_SEC:-8.0}
  COMM_FUZZ_DURATION_MIN_SEC=${COMM_FUZZ_DURATION_MIN_SEC:-25.0}
  COMM_FUZZ_DURATION_MAX_SEC=${COMM_FUZZ_DURATION_MAX_SEC:-40.0}
fi

COMM_FUZZ_START_MIN_SEC=${9:-${COMM_FUZZ_START_MIN_SEC:-5.0}}
COMM_FUZZ_START_MAX_SEC=${10:-${COMM_FUZZ_START_MAX_SEC:-25.0}}
COMM_FUZZ_DURATION_MIN_SEC=${11:-${COMM_FUZZ_DURATION_MIN_SEC:-5.0}}
COMM_FUZZ_DURATION_MAX_SEC=${12:-${COMM_FUZZ_DURATION_MAX_SEC:-20.0}}
COMM_FUZZ_SEED_GEN_MODE=${13:-${COMM_FUZZ_SEED_GEN_MODE:-pure_random}}
COMM_FUZZ_KEY_TIME_SEC=${14:-${COMM_FUZZ_KEY_TIME_SEC:-12.0}}

COMM_FUZZ_MODE_CFG="$COMM_FUZZ_MODE"

CIRCLE_LAUNCH_FILE=${CIRCLE_LAUNCH_FILE:-circle_exchange.launch}
DRONE_NUM=${DRONE_NUM:-10}
ODOM_SUFFIX=${ODOM_SUFFIX:-visual_slam/odom}
ENABLE_OBSTACLE_COLLISION=${ENABLE_OBSTACLE_COLLISION:-true}
ENABLE_DRONE_COLLISION=${ENABLE_DRONE_COLLISION:-true}
MAX_TOTAL_RUNTIME_SEC=${MAX_TOTAL_RUNTIME_SEC:-0}

BATCH_TS=$(date +%Y%m%d_%H%M%S)
BATCH_DIR="$OUT_DIR/$BATCH_TS"
BATCH_START_EPOCH=$(date +%s)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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

csv_escape() {
  local s
  s=${1:-}
  s=${s//\"/\"\"}
  printf '"%s"' "$s"
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

is_number() {
  [[ "${1:-}" =~ ^[0-9]+([.][0-9]+)?$ ]]
}

strip_ansi() {
  sed -r 's/\x1B\[[0-9;]*[mK]//g'
}

extract_runtime_log() {
  local in_file="$1"
  local out_file="$2"
  local cut_line

  cut_line=$(grep -Enim1 'shutting down processing monitor|Shutdown errors:|escalating to SIGTERM|escalating to SIGKILL' "$in_file" | cut -d: -f1 || true)
  if [ -n "${cut_line:-}" ] && [ "$cut_line" -gt 1 ]; then
    head -n "$((cut_line - 1))" "$in_file" > "$out_file"
  else
    cp "$in_file" "$out_file"
  fi
}

extract_shutdown_log() {
  local in_file="$1"
  local out_file="$2"
  local cut_line

  cut_line=$(grep -Enim1 'shutting down processing monitor|Shutdown errors:|escalating to SIGTERM|escalating to SIGKILL' "$in_file" | cut -d: -f1 || true)
  if [ -n "${cut_line:-}" ]; then
    tail -n "+$cut_line" "$in_file" > "$out_file"
  else
    : > "$out_file"
  fi
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

  python3 - "$round_idx" "$COMM_FUZZ_SEED_BASE" "$COMM_FUZZ_MODE" "$COMM_FUZZ_START_MIN_SEC" "$COMM_FUZZ_START_MAX_SEC" "$COMM_FUZZ_DURATION_MIN_SEC" "$COMM_FUZZ_DURATION_MAX_SEC" "$COMM_FUZZ_DELAY_MIN_MS" "$COMM_FUZZ_DELAY_MAX_MS" "$COMM_FUZZ_JITTER_MIN_MS" "$COMM_FUZZ_JITTER_MAX_MS" "$COMM_FUZZ_NOISE_SIGMA_POS_MIN" "$COMM_FUZZ_NOISE_SIGMA_POS_MAX" "$pool_file" "$used_seed_file" "$warmup_rounds" "$COMM_FUZZ_POOL_MUTATION_RATIO" <<'PY'
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
mutation_ratio = float(sys.argv[17]) if len(sys.argv) > 17 else 0.10
mutation_ratio = max(0.0, min(0.5, mutation_ratio))

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
start_span = max(start_max - start_min, 1e-9)
dur_span = max(dur_max - dur_min, 1e-9)
start_s = clamp(
    base_start + mutation_ratio * start_span * rng.uniform(-1.0, 1.0),
    start_min, start_max)
duration_s = clamp(
    base_duration + mutation_ratio * dur_span * rng.uniform(-1.0, 1.0),
    dur_min, dur_max)

if mode == "delay":
  factor_name = "delay_ms"
  dspan = max(delay_max - delay_min, 1e-9)
  center = base_factor_value if base_factor_name == "delay_ms" else 0.5 * (delay_min + delay_max)
  factor_value = clamp(
      center + mutation_ratio * dspan * rng.uniform(-1.0, 1.0),
      delay_min, delay_max)
elif mode == "jitter":
  factor_name = "jitter_ms"
  jspan = max(jitter_max - jitter_min, 1e-9)
  center = base_factor_value if base_factor_name == "jitter_ms" else 0.5 * (jitter_min + jitter_max)
  factor_value = clamp(
      center + mutation_ratio * jspan * rng.uniform(-1.0, 1.0),
      jitter_min, jitter_max)
else:
  mode = "noise"
  factor_name = "noise_sigma_pos"
  nspan = max(noise_max - noise_min, 1e-9)
  center = base_factor_value if base_factor_name == "noise_sigma_pos" else 0.5 * (noise_min + noise_max)
  factor_value = clamp(
      center + mutation_ratio * nspan * rng.uniform(-1.0, 1.0),
      noise_min, noise_max)

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

shutdown_process() {
  local pid="${1:-}"
  if [ -n "$pid" ]; then
    kill -INT "$pid" >/dev/null 2>&1 || true
    local waited=0
    while kill -0 "$pid" >/dev/null 2>&1; do
      if [ "$waited" -ge 50 ]; then
        break
      fi
      sleep 0.1
      waited=$((waited + 1))
    done
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
    wait "$pid" >/dev/null 2>&1 || true
  fi
}

trap 'best_effort_ros_cleanup' EXIT

COLLISION_MONITOR_FLAG=$(to_bool "$ENABLE_COLLISION_MONITOR")
if [ "$COLLISION_MONITOR_FLAG" = "true" ]; then
  COLLISION_MONITOR_ENABLED="yes"
else
  COLLISION_MONITOR_ENABLED="no"
fi

ENABLE_OBSTACLE_COLLISION_FLAG=$(to_bool "$ENABLE_OBSTACLE_COLLISION")
ENABLE_DRONE_COLLISION_FLAG=$(to_bool "$ENABLE_DRONE_COLLISION")

SHUTDOWN_MARK='__CIRCLEFUZZ_SHUTDOWN_BEGIN__'
FATAL_PATTERN='Eigen::.*Assertion|Assertion .+ failed|boost::wrapexcept<boost::lock_error>|terminate called after throwing|Segmentation fault|core dumped|process monitor has been shut down|RLException: cannot add process|auto-starting new master'

COMM_FUZZ_ENABLED=$(to_bool "$ENABLE_COMM_FUZZ_CLI")
COMM_FUZZ_POOL_MUTATION_RATIO=${COMM_FUZZ_POOL_MUTATION_RATIO:-0.10}
COMM_FUZZ_DELAY_MIN_MS=${COMM_FUZZ_DELAY_MIN_MS:-80.0}
COMM_FUZZ_DELAY_MAX_MS=${COMM_FUZZ_DELAY_MAX_MS:-250.0}
COMM_FUZZ_JITTER_MIN_MS=${COMM_FUZZ_JITTER_MIN_MS:-30.0}
COMM_FUZZ_JITTER_MAX_MS=${COMM_FUZZ_JITTER_MAX_MS:-120.0}
COMM_FUZZ_NOISE_SIGMA_POS_MIN=${COMM_FUZZ_NOISE_SIGMA_POS_MIN:-0.05}
COMM_FUZZ_NOISE_SIGMA_POS_MAX=${COMM_FUZZ_NOISE_SIGMA_POS_MAX:-0.20}
ENABLE_COMM_FULL_DUMP_SIDECAR=${ENABLE_COMM_FULL_DUMP_SIDECAR:-false}
COMM_FULL_DUMP_BUFFER_MAX=${COMM_FULL_DUMP_BUFFER_MAX:-4000}
COMM_FULL_DUMP_FILE_NAME=${COMM_FULL_DUMP_FILE_NAME:-comm_full_dump_on_incident.jsonl}

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
  echo "Circle fuzz benchmark start"
  echo "runs=$RUNS"
  echo "run_sec=$RUN_SEC"
  echo "max_total_runtime_sec=$MAX_TOTAL_RUNTIME_SEC"
  echo "out_dir=$OUT_DIR"
  echo "batch_dir=$BATCH_DIR"
  echo "collision_monitor_enabled=$COLLISION_MONITOR_ENABLED"
  echo "circle_launch_file=$CIRCLE_LAUNCH_FILE"
  echo "drone_num=$DRONE_NUM"
  echo "odom_suffix=$ODOM_SUFFIX"
  echo "enable_obstacle_collision=$ENABLE_OBSTACLE_COLLISION_FLAG"
  echo "enable_drone_collision=$ENABLE_DRONE_COLLISION_FLAG"
  echo "enable_comm_fuzz=$COMM_FUZZ_ENABLED"
  echo "comm_fuzz_mode=$COMM_FUZZ_MODE"
  echo "comm_fuzz_seed_gen_mode=$COMM_FUZZ_SEED_GEN_MODE_NORMALIZED"
  echo "seed_pool_warmup_rounds_effective=$WARMUP_ROUNDS_EFFECTIVE"
  echo "comm_fuzz_key_time_sec=$COMM_FUZZ_KEY_TIME_SEC"
  echo "comm_fuzz_pool_mutation_ratio=$COMM_FUZZ_POOL_MUTATION_RATIO"
  echo "seed_pool_rounds_file=$SEED_POOL_ROUNDS_FILE"
  echo "seed_pool_file=$SEED_POOL_FILE"
  echo "comm_full_dump_sidecar_enabled=$(to_bool "$ENABLE_COMM_FULL_DUMP_SIDECAR")"
  echo "comm_full_dump_buffer_max=$COMM_FULL_DUMP_BUFFER_MAX"
  echo "comm_full_dump_file_name=$COMM_FULL_DUMP_FILE_NAME"
  echo
} > "$SUMMARY"

echo "row_type,round,timestamp,result,bench_result,strict_result,collision_monitor_enabled,circle_launch_file,collision_event_count,drone_drone_collision_count,drone_obstacle_collision_count,collision_type,collision_pair,collision_distance,mandatory_stop_count,replan_count,last_time_ms,avg_time_ms,emergency_count,fatal_count,run_dir,report_dir,monitor_log,raw_log,clean_log,runtime_clean_log,shutdown_clean_log,summary_json,comm_fuzz_enabled,comm_fuzz_seed,comm_fuzz_mode_cfg,comm_fuzz_target_drone_id,comm_fuzz_mode_used,comm_fuzz_start_s,comm_fuzz_duration_s,comm_fuzz_factor_name,comm_fuzz_factor_value,comm_sender_drone_id,comm_receiver_drone_id,comm_fuzz_meta_file,comm_fuzz_trace_file,comm_trace_line_count,comm_first_fuzz_ros_time,root_cause_primary,root_cause_types,root_cause_confidence,root_cause_summary,root_cause_report,root_cause_event_kind,first_collision_time,first_emergency_time,comm_relation,comm_relation_score,comm_relation_summary,comm_relation_report,plan_phase,plan_source,param_key,planner_rx_trace_glob,planner_safety_trace_glob" > "$SUMMARY_CSV"

TOTAL_COLLISION_EVENT_COUNT=0
TOTAL_DRONE_DRONE_COLLISION_COUNT=0
TOTAL_DRONE_OBSTACLE_COLLISION_COUNT=0
TOTAL_MANDATORY_STOP_COUNT=0
TOTAL_EMERGENCY_COUNT=0
TOTAL_FATAL_COUNT=0
TOTAL_SUCCESS_ROUNDS=0
TOTAL_FAIL_ROUNDS=0
TOTAL_BENCH_SUCCESS_ROUNDS=0
TOTAL_LAST_TIME_MS=0
TOTAL_LAST_TIME_COUNT=0
TOTAL_REPLAN_COUNT=0
TOTAL_AVG_TIME_MS=0
TOTAL_AVG_TIME_COUNT=0
EXECUTED_ROUNDS=0
STOP_REASON="completed_all_rounds"

for i in $(seq 1 "$RUNS"); do
  if [[ "$MAX_TOTAL_RUNTIME_SEC" =~ ^[0-9]+$ ]] && [ "$MAX_TOTAL_RUNTIME_SEC" -gt 0 ]; then
    NOW_EPOCH=$(date +%s)
    ELAPSED_BEFORE_ROUND=$((NOW_EPOCH - BATCH_START_EPOCH))
    REMAINING_BATCH_SEC=$((MAX_TOTAL_RUNTIME_SEC - ELAPSED_BEFORE_ROUND))
    if [ "$REMAINING_BATCH_SEC" -le 0 ]; then
      STOP_REASON="max_total_runtime_reached_before_round"
      break
    fi
  else
    REMAINING_BATCH_SEC=0
  fi

  ROUND_SLEEP_SEC="$RUN_SEC"
  if [[ "$MAX_TOTAL_RUNTIME_SEC" =~ ^[0-9]+$ ]] && [ "$MAX_TOTAL_RUNTIME_SEC" -gt 0 ]; then
    if [ "$REMAINING_BATCH_SEC" -lt "$ROUND_SLEEP_SEC" ]; then
      ROUND_SLEEP_SEC="$REMAINING_BATCH_SEC"
    fi
  fi

  if [ "$ROUND_SLEEP_SEC" -le 0 ]; then
    STOP_REASON="max_total_runtime_reached_before_sleep"
    break
  fi

  EXECUTED_ROUNDS=$((EXECUTED_ROUNDS + 1))
  TS=$(date +%Y%m%d_%H%M%S)
  ROUND_DIR="$BATCH_DIR/run${i}"
  RUN_ID="${BATCH_TS}_run${i}"
  rm -rf "$ROUND_DIR/fuzz_report" "$ROUND_DIR/fuzz_report_staging"
  mkdir -p "$ROUND_DIR"
  mkdir -p "$ROUND_DIR/fuzz_report_staging"
  ABS_REPORT_DIR="$(cd "$ROUND_DIR/fuzz_report_staging" && pwd)"
  COMM_FUZZ_RUN_DIR="$ABS_REPORT_DIR/$RUN_ID"
  COMM_FUZZ_META_FILE="$COMM_FUZZ_RUN_DIR/comm_fuzz_meta.json"
  COMM_FUZZ_TRACE_FILE="$COMM_FUZZ_RUN_DIR/comm_fuzz_trace.jsonl"
  PLANNER_RX_TRACE_GLOB="$COMM_FUZZ_RUN_DIR/planner_rx_trace_drone_*.jsonl"
  PLANNER_SAFETY_TRACE_GLOB="$COMM_FUZZ_RUN_DIR/planner_safety_trace_drone_*.jsonl"

  RAW="$ROUND_DIR/run.log"
  CLEAN="$ROUND_DIR/run_clean.log"
  RUNTIME_CLEAN="$ROUND_DIR/runtime_clean.log"
  SHUTDOWN_CLEAN="$ROUND_DIR/shutdown_clean.log"
  MONITOR_LOG="$ROUND_DIR/collision_monitor.log"
  TXT="$ROUND_DIR/result.txt"
  ROOT_CAUSE_REPORT_FILE="$ROUND_DIR/root_cause_report.json"
  ROOT_CAUSE_KV_FILE="$ROUND_DIR/root_cause_kv.txt"
  COMM_RELATION_REPORT_FILE="$ROUND_DIR/comm_relation_report.json"
  COMM_RELATION_KV_FILE="$ROUND_DIR/comm_relation_kv.txt"

  ROUND_PLAN_LINE=""
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
  ROUND_PLAN_FACTOR_NAME=""
  ROUND_PLAN_FACTOR_VALUE=""
  ROUND_PLAN_PHASE="warmup"
  ROUND_PLAN_SOURCE="warmup_random"
  ROUND_SOURCE_BUCKET="NA"
  ROUND_SELECTED_SEED_LEVEL="NA"

  COMM_FUZZ_SEED=$(choose_seed_pure_random "$i")
  if [ "$COMM_FUZZ_SEED_GEN_MODE_NORMALIZED" = "ripple_biased" ] && [ "$COMM_FUZZ_ENABLED" = "true" ]; then
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

      if [ -n "${ROUND_PLAN_START_S:-}" ] && [ -n "${ROUND_PLAN_DURATION_S:-}" ]; then
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

  if [ "$COMM_FUZZ_ENABLED" = "true" ]; then
    if [ "${COMM_FUZZ_SEED_GEN_MODE_NORMALIZED:-pure_random}" != "ripple_biased" ]; then
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
      COMM_FUZZ_SEED=$(choose_seed_pure_random "$i")
    fi
  fi

  MONITOR_PID=""
  SIDECAR_PID=""
  SIDECAR_LOG="$ROUND_DIR/comm_full_dump_sidecar.log"
  COMM_FULL_DUMP_FILE_PATH="$COMM_FUZZ_RUN_DIR/$COMM_FULL_DUMP_FILE_NAME"
  COMM_FULL_DUMP_META_FILE="$COMM_FUZZ_RUN_DIR/comm_full_dump_meta.json"
  if [ "$COLLISION_MONITOR_FLAG" = "true" ]; then
    rosrun ego_planner fuzz_collision_monitor.py \
      _drone_num:="$DRONE_NUM" \
      _odom_suffix:="$ODOM_SUFFIX" \
      _report_dir:="$ABS_REPORT_DIR" \
      _run_id:="$RUN_ID" \
      _kill_all_nodes:=false \
      _enable_obstacle_collision:="$ENABLE_OBSTACLE_COLLISION_FLAG" \
      _enable_drone_collision:="$ENABLE_DRONE_COLLISION_FLAG" \
      > "$MONITOR_LOG" 2>&1 &
    MONITOR_PID=$!
  fi

  COMM_FULL_DUMP_SIDECAR_FLAG=$(to_bool "$ENABLE_COMM_FULL_DUMP_SIDECAR")
  if [ "$COMM_FULL_DUMP_SIDECAR_FLAG" = "true" ]; then
    python3 planner/plan_manage/scripts/comm_full_dump_sidecar.py \
      _enable:=true \
      _report_dir:="$ABS_REPORT_DIR" \
      _run_id:="$RUN_ID" \
      _buffer_max:="$COMM_FULL_DUMP_BUFFER_MAX" \
      _dump_file_name:="$COMM_FULL_DUMP_FILE_NAME" \
      > "$SIDECAR_LOG" 2>&1 &
    SIDECAR_PID=$!
  fi

  echo "[Round $i/$RUNS] launch $CIRCLE_LAUNCH_FILE..."

  roslaunch ego_planner "$CIRCLE_LAUNCH_FILE" \
    fuzz_report_dir:="$ABS_REPORT_DIR" \
    fuzz_run_id:="$RUN_ID" \
    enable_comm_fuzz:="$COMM_FUZZ_ENABLED" \
    comm_fuzz_target_drone_id:="$COMM_FUZZ_TARGET_DRONE_ID" \
    comm_fuzz_mode:="$ROUND_COMM_FUZZ_MODE" \
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
    comm_fuzz_report_dir:="$ABS_REPORT_DIR" \
    comm_fuzz_run_id:="$RUN_ID" \
    > "$RAW" 2>&1 &
  LAUNCH_PID=$!

  sleep "$ROUND_SLEEP_SEC"

  ROUND_FORCED_STOP="no"
  if [ "$ROUND_SLEEP_SEC" -lt "$RUN_SEC" ]; then
    ROUND_FORCED_STOP="yes"
    STOP_REASON="max_total_runtime_reached_during_round"
  fi

  echo "$SHUTDOWN_MARK ts=$(date +%s)" >> "$RAW"

  shutdown_process "$LAUNCH_PID"
  shutdown_process "$MONITOR_PID"
  shutdown_process "$SIDECAR_PID"
  best_effort_ros_cleanup

  strip_ansi < "$RAW" > "$CLEAN"
  extract_runtime_log "$CLEAN" "$RUNTIME_CLEAN"
  extract_shutdown_log "$CLEAN" "$SHUTDOWN_CLEAN"

  SUCCESS_COUNT=$(grep -c "Success=yes" "$RUNTIME_CLEAN" || true)
  FAIL_COUNT=$(grep -c "Success=no" "$RUNTIME_CLEAN" || true)
  EMERGENCY_COUNT=$(grep -Eic "Emergency stop|EMERGENCY_STOP|Mandatory Stop|Depth Lost" "$RUNTIME_CLEAN" || true)
  FATAL_RUNTIME_COUNT=$(grep -Eic "$FATAL_PATTERN" "$RUNTIME_CLEAN" || true)
  FATAL_SHUTDOWN_COUNT=$(grep -Eic "$FATAL_PATTERN" "$SHUTDOWN_CLEAN" || true)
  FATAL_COUNT=$((FATAL_RUNTIME_COUNT + FATAL_SHUTDOWN_COUNT))
  MANDATORY_STOP_COUNT=$(grep -Eic "Mandatory Stop|Received a mandatory stop command" "$RUNTIME_CLEAN" || true)

  if [ -f "$MONITOR_LOG" ]; then
    strip_ansi < "$MONITOR_LOG" > "$MONITOR_LOG.clean" || : > "$MONITOR_LOG.clean"
  else
    : > "$MONITOR_LOG.clean"
  fi
  MONITOR_LOG_CLEAN="$MONITOR_LOG.clean"
  [ -f "$MONITOR_LOG_CLEAN" ] || : > "$MONITOR_LOG_CLEAN"

  COLLISION_EVENT_COUNT=$(cat "$MONITOR_LOG_CLEAN" "$RUNTIME_CLEAN" 2>/dev/null | grep -Eic 'HARD_COLLISION|\[fuzz_collision_monitor\] HARD_COLLISION' || true)
  DRONE_DRONE_COLLISION_COUNT=$(grep -Eic '"type"[[:space:]]*:[[:space:]]*"drone_drone"' "$MONITOR_LOG_CLEAN" || true)
  DRONE_OBSTACLE_COLLISION_COUNT=$(grep -Eic '"type"[[:space:]]*:[[:space:]]*"drone_obstacle"' "$MONITOR_LOG_CLEAN" || true)

  ROUND_FUZZ_REPORT_DIR="$ROUND_DIR/fuzz_report"
  KEEP_FUZZ_REPORT=0
  if [ "${COLLISION_EVENT_COUNT:-0}" -gt 0 ] || [ "${EMERGENCY_COUNT:-0}" -gt 0 ]; then
    KEEP_FUZZ_REPORT=1
  fi
  FUZZ_REPORT_DIR_VALUE="NA"
  COLLISION_TYPE="NA"
  COLLISION_PAIR="NA"
  COLLISION_DISTANCE="NA"
  if [ "$KEEP_FUZZ_REPORT" -eq 1 ]; then
    if [ -d "$ROUND_DIR/fuzz_report_staging" ]; then
      rm -rf "$ROUND_DIR/fuzz_report"
      mv "$ROUND_DIR/fuzz_report_staging" "$ROUND_DIR/fuzz_report"
    fi
    REPORT_DIR="$ROUND_DIR/fuzz_report"
    ABS_REPORT_DIR="$(cd "$REPORT_DIR" && pwd)"
    FUZZ_REPORT_DIR_VALUE="$ABS_REPORT_DIR"
    COMM_FUZZ_RUN_DIR="$REPORT_DIR/$RUN_ID"
    COMM_FUZZ_META_FILE="$COMM_FUZZ_RUN_DIR/comm_fuzz_meta.json"
    COMM_FUZZ_TRACE_FILE="$COMM_FUZZ_RUN_DIR/comm_fuzz_trace.jsonl"
    PLANNER_RX_TRACE_GLOB="$COMM_FUZZ_RUN_DIR/planner_rx_trace_drone_*.jsonl"
    PLANNER_SAFETY_TRACE_GLOB="$COMM_FUZZ_RUN_DIR/planner_safety_trace_drone_*.jsonl"
    SUMMARY_JSON="$REPORT_DIR/$RUN_ID/summary.json"
    COMM_FULL_DUMP_FILE_PATH="$COMM_FUZZ_RUN_DIR/$COMM_FULL_DUMP_FILE_NAME"
    COMM_FULL_DUMP_META_FILE="$COMM_FUZZ_RUN_DIR/comm_full_dump_meta.json"
  else
    rm -rf "$ROUND_DIR/fuzz_report_staging" "$ROUND_DIR/fuzz_report" || true
    REPORT_DIR=""
    ABS_REPORT_DIR=""
    COMM_FUZZ_RUN_DIR=""
    COMM_FUZZ_META_FILE=""
    COMM_FUZZ_TRACE_FILE=""
    PLANNER_RX_TRACE_GLOB=""
    PLANNER_SAFETY_TRACE_GLOB=""
    SUMMARY_JSON=""
    COMM_FULL_DUMP_FILE_PATH="NA"
    COMM_FULL_DUMP_META_FILE="NA"
  fi

  REPLAN_COUNT=$(grep -Eic "Replan[[:space:]]+[0-9]+" "$RUNTIME_CLEAN" || true)
  AVG_TIME_MS="NA"

    if [ "$COMM_FUZZ_ENABLED" = "true" ]; then
      COMM_FUZZ_ENABLED_META="yes"
      COMM_FUZZ_SEED_USED="$COMM_FUZZ_SEED"
      COMM_FUZZ_MODE_USED="$ROUND_COMM_FUZZ_MODE"
      COMM_FUZZ_TARGET_USED="$COMM_FUZZ_TARGET_DRONE_ID"
      if [ "$ROUND_COMM_FUZZ_START_MIN_SEC" = "$ROUND_COMM_FUZZ_START_MAX_SEC" ]; then
        COMM_FUZZ_START_S="$ROUND_COMM_FUZZ_START_MIN_SEC"
      else
        COMM_FUZZ_START_S="NA"
      fi
      if [ "$ROUND_COMM_FUZZ_DURATION_MIN_SEC" = "$ROUND_COMM_FUZZ_DURATION_MAX_SEC" ]; then
        COMM_FUZZ_DURATION_S="$ROUND_COMM_FUZZ_DURATION_MIN_SEC"
      else
        COMM_FUZZ_DURATION_S="NA"
      fi
      COMM_FUZZ_FACTOR_NAME="${ROUND_PLAN_FACTOR_NAME:-}"
      COMM_FUZZ_FACTOR_VALUE="${ROUND_PLAN_FACTOR_VALUE:-}"
      [ -n "$COMM_FUZZ_FACTOR_NAME" ] || COMM_FUZZ_FACTOR_NAME="NA"
      [ -n "$COMM_FUZZ_FACTOR_VALUE" ] || COMM_FUZZ_FACTOR_VALUE="NA"
      COMM_FUZZ_MSG_SEEN="0"
      COMM_FUZZ_MSG_FUZZED="0"
      COMM_SENDER_DRONE_ID="$COMM_FUZZ_TARGET_DRONE_ID"
      COMM_RECEIVER_DRONE_ID="all"
      COMM_TRACE_LINE_COUNT="0"
      COMM_FIRST_FUZZ_ROS_TIME="NA"
      COMM_FUZZ_META_FILE_PRESENT="no"
    else
      COMM_FUZZ_ENABLED_META="no"
      COMM_FUZZ_SEED_USED="NA"
      COMM_FUZZ_MODE_USED="NA"
      COMM_FUZZ_TARGET_USED="NA"
      COMM_FUZZ_START_S="NA"
      COMM_FUZZ_DURATION_S="NA"
      COMM_FUZZ_FACTOR_NAME="NA"
      COMM_FUZZ_FACTOR_VALUE="NA"
      COMM_FUZZ_MSG_SEEN="0"
      COMM_FUZZ_MSG_FUZZED="0"
      COMM_SENDER_DRONE_ID="NA"
      COMM_RECEIVER_DRONE_ID="NA"
      COMM_TRACE_LINE_COUNT="0"
      COMM_FIRST_FUZZ_ROS_TIME="NA"
      COMM_FUZZ_META_FILE_PRESENT="no"
    fi

    ROOT_CAUSE_PRIMARY="none"
    ROOT_CAUSE_TYPES="none"
    ROOT_CAUSE_CONFIDENCE="0"
    ROOT_CAUSE_SUMMARY="analysis_not_available"
    ROOT_CAUSE_EVENT_KIND="none"
    FIRST_COLLISION_TIME="NA"
    FIRST_EMERGENCY_TIME="NA"
    COMM_RELATION="insufficient"
    COMM_RELATION_SCORE="0"
    COMM_RELATION_SUMMARY="analysis_not_available"

    if [ "$COMM_FUZZ_ENABLED" = "true" ] && [ -f "$COMM_FUZZ_META_FILE" ]; then
    COMM_FUZZ_META_FILE_PRESENT="yes"
    read -r COMM_FUZZ_ENABLED_META COMM_FUZZ_SEED_USED COMM_FUZZ_MODE_USED COMM_FUZZ_TARGET_USED COMM_FUZZ_START_S COMM_FUZZ_DURATION_S COMM_FUZZ_FACTOR_NAME COMM_FUZZ_FACTOR_VALUE COMM_FUZZ_MSG_SEEN COMM_FUZZ_MSG_FUZZED COMM_SENDER_DRONE_ID COMM_RECEIVER_DRONE_ID COMM_TRACE_LINE_COUNT COMM_FIRST_FUZZ_ROS_TIME < <(
      python3 - "$COMM_FUZZ_META_FILE" "$COMM_FUZZ_TRACE_FILE" "$PLANNER_RX_TRACE_GLOB" <<'PY'
import json
import glob
import os
import sys

meta_path = sys.argv[1]
trace_path = sys.argv[2]
rx_glob = sys.argv[3]

def load_json(path):
  try:
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
      return json.load(f)
  except Exception:
    return {}

def as_text(value, default='NA'):
  if value is None or value == '':
    return default
  if isinstance(value, bool):
    return 'yes' if value else 'no'
  return str(value)

meta = load_json(meta_path)
events = []
if os.path.isfile(trace_path):
  with open(trace_path, 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      try:
        events.append(json.loads(line))
      except Exception:
        continue

sender = 'NA'
first_fuzz_ros = None
receiver_ids = set()
sender_candidates = set()

for fp in sorted(glob.glob(rx_glob)) if rx_glob else []:
  if not os.path.isfile(fp):
    continue
  with open(fp, 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      try:
        item = json.loads(line)
      except Exception:
        continue
      sid = item.get('self_drone_id')
      src = item.get('src_drone_id')
      if sid is not None:
        receiver_ids.add(str(sid))
      if src is not None:
        sender_candidates.add(str(src))

for item in events:
  event = str(item.get('event', ''))
  if event.startswith('enqueue_') or event.startswith('send_'):
    sender = as_text(item.get('drone_id'))
    rt = item.get('ros_time')
    try:
      first_fuzz_ros = float(rt)
    except Exception:
      first_fuzz_ros = None
    break

if sender == 'NA' and sender_candidates:
  sender = sorted(sender_candidates)[0]

receiver = 'all'
if receiver_ids:
  receiver = ','.join(sorted(receiver_ids, key=lambda x: int(x) if x.isdigit() else x))

values = [
  as_text(meta.get('enabled')),
  as_text(meta.get('seed')),
  as_text(meta.get('mode')),
  as_text(meta.get('target_drone_id')),
  as_text(meta.get('window_start_s')),
  as_text(meta.get('window_duration_s')),
  as_text(meta.get('factor_name')),
  as_text(meta.get('factor_value')),
  as_text(meta.get('msg_seen')),
  as_text(meta.get('msg_fuzzed')),
  sender,
  receiver,
  str(len(events)),
  as_text(first_fuzz_ros),
]
print('\t'.join(values))
PY
    )
    fi

    if [ "$COMM_FUZZ_ENABLED" = "true" ]; then
      if ! is_number "${COMM_FUZZ_START_S:-}"; then
        COMM_FUZZ_START_S=$(python3 - "$ROUND_COMM_FUZZ_START_MIN_SEC" "$ROUND_COMM_FUZZ_START_MAX_SEC" <<'PY'
import sys
a = float(sys.argv[1])
b = float(sys.argv[2])
print(f"{(a + b) * 0.5:.6f}")
PY
)
      fi
      if ! is_number "${COMM_FUZZ_DURATION_S:-}"; then
        COMM_FUZZ_DURATION_S=$(python3 - "$ROUND_COMM_FUZZ_DURATION_MIN_SEC" "$ROUND_COMM_FUZZ_DURATION_MAX_SEC" <<'PY'
import sys
a = float(sys.argv[1])
b = float(sys.argv[2])
print(f"{(a + b) * 0.5:.6f}")
PY
)
      fi
    fi

    if python3 planner/plan_manage/scripts/analyze_round_root_cause.py \
    --runtime-log "$RUNTIME_CLEAN" \
    --fuzz-report-dir "$ROUND_FUZZ_REPORT_DIR" \
    --run-id "$RUN_ID" \
    --comm-fuzz-meta "$COMM_FUZZ_META_FILE" \
    --output "$ROOT_CAUSE_REPORT_FILE" \
    --shell-kv "$ROOT_CAUSE_KV_FILE"; then
    IFS=$'\t' read -r ROOT_CAUSE_PRIMARY ROOT_CAUSE_TYPES ROOT_CAUSE_CONFIDENCE ROOT_CAUSE_SUMMARY FIRST_COLLISION_TIME FIRST_EMERGENCY_TIME < <(
      python3 - "$ROOT_CAUSE_REPORT_FILE" <<'PY'
import json
import sys

path = sys.argv[1]
def load_json(p):
  try:
    with open(p, 'r', encoding='utf-8', errors='ignore') as f:
      return json.load(f)
  except Exception:
    return {}

root = load_json(path)
analysis = root.get('analysis', {}) if isinstance(root, dict) else {}
signals = analysis.get('signals', {}) if isinstance(analysis, dict) else {}
event_kind = 'collision' if signals.get('collision_exists') else ('emergency_stop' if signals.get('emergency_count', 0) else 'none')
primary = analysis.get('primary_incident', 'none')
incident_types = ','.join(analysis.get('incident_types', [])) if isinstance(analysis.get('incident_types', []), list) else 'none'
confidence = analysis.get('confidence', 0)
summary = analysis.get('summary', 'analysis_not_available')
def na(v):
  return 'NA' if v is None else v
first_collision = na(signals.get('first_collision_ros_time', 'NA'))
first_emergency = na(signals.get('first_emergency_ros_time', 'NA'))
print('\t'.join([str(primary), str(incident_types), str(confidence), str(summary), str(first_collision), str(first_emergency)]))
PY
    )
    [ -n "${ROOT_CAUSE_PRIMARY:-}" ] || ROOT_CAUSE_PRIMARY="none"
    [ -n "${ROOT_CAUSE_TYPES:-}" ] || ROOT_CAUSE_TYPES="none"
    [ -n "${ROOT_CAUSE_CONFIDENCE:-}" ] || ROOT_CAUSE_CONFIDENCE="0"
    [ -n "${ROOT_CAUSE_SUMMARY:-}" ] || ROOT_CAUSE_SUMMARY="analysis_not_available"
    [ -n "${FIRST_COLLISION_TIME:-}" ] || FIRST_COLLISION_TIME="NA"
    [ -n "${FIRST_EMERGENCY_TIME:-}" ] || FIRST_EMERGENCY_TIME="NA"
    fi

    if python3 planner/plan_manage/scripts/analyze_comm_collision_relation.py \
    --runtime-log "$RUNTIME_CLEAN" \
    --root-cause-report "$ROOT_CAUSE_REPORT_FILE" \
    --comm-fuzz-meta "$COMM_FUZZ_META_FILE" \
    --comm-fuzz-trace "$COMM_FUZZ_TRACE_FILE" \
    --planner-rx-glob "$PLANNER_RX_TRACE_GLOB" \
    --planner-safety-glob "$PLANNER_SAFETY_TRACE_GLOB" \
    --output "$COMM_RELATION_REPORT_FILE" \
    --shell-kv "$COMM_RELATION_KV_FILE"; then
    IFS=$'\t' read -r COMM_RELATION COMM_RELATION_SCORE COMM_RELATION_SUMMARY < <(
      python3 - "$COMM_RELATION_REPORT_FILE" <<'PY'
import json
import sys

path = sys.argv[1]
def load_json(p):
  try:
    with open(p, 'r', encoding='utf-8', errors='ignore') as f:
      return json.load(f)
  except Exception:
    return {}

report = load_json(path)
relation = report.get('relation', 'insufficient')
score = report.get('score', 0)
summary = report.get('summary', 'analysis_not_available')
print('\t'.join([str(relation), str(score), str(summary)]))
PY
    )
    [ -n "${COMM_RELATION:-}" ] || COMM_RELATION="insufficient"
    [ -n "${COMM_RELATION_SCORE:-}" ] || COMM_RELATION_SCORE="0"
    [ -n "${COMM_RELATION_SUMMARY:-}" ] || COMM_RELATION_SUMMARY="analysis_not_available"
    fi

  PLAN_ROUND_PARAM_KEY="NA"
  if [ "$COMM_FUZZ_SEED_GEN_MODE_NORMALIZED" = "ripple_biased" ] && [ "$COMM_FUZZ_ENABLED" = "true" ]; then
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

    if [ "$VALID_POOL_RECORD" = "yes" ] && [ "$ANOMALY_LEVEL" != "none" ]; then
      echo -e "${i}\t${PARAM_KEY}\t${COMM_FUZZ_SEED_USED}\t${COMM_FUZZ_MODE_USED}\t${COMM_FUZZ_START_S}\t${COMM_FUZZ_DURATION_S}\t${COMM_FUZZ_FACTOR_NAME}\t${COMM_FUZZ_FACTOR_VALUE}\t${ANOMALY_LEVEL}\t${SEVERE_RESOURCE_EXHAUSTION}\t${i}\t${ROUND_PLAN_PHASE}\t${ROUND_PLAN_SOURCE}\t${ROUND_SOURCE_BUCKET}\t${ROUND_SELECTED_SEED_LEVEL}" >> "$SEED_POOL_ROUNDS_FILE"
      update_seed_pool_file "$SEED_POOL_ROUNDS_FILE" "$SEED_POOL_FILE" "50" "$SUMMARY_CSV"
      PLAN_ROUND_PARAM_KEY="$PARAM_KEY"
    fi
  fi

  AVG_TIME_MS=$(grep -oE 'avg=[0-9]+(\.[0-9]+)?ms' "$CLEAN" | sed -E 's/avg=([0-9.]+)ms/\1/' | tail -n 1 || true)

  if [ -f "$SUMMARY_JSON" ]; then
    read -r COLLISION_TYPE COLLISION_PAIR COLLISION_DISTANCE < <(
      python3 - "$SUMMARY_JSON" <<'PY'
import json
import sys

path = sys.argv[1]
try:
    with open(path, 'r', encoding='ascii', errors='ignore') as f:
        data = json.load(f)
    collision = data.get('event', {}).get('collision', {})
    ctype = collision.get('type', 'NA')
    pair = 'NA'
    if ctype == 'drone_drone':
        pair = f"{collision.get('drone_a', 'NA')}-{collision.get('drone_b', 'NA')}"
    elif ctype == 'drone_obstacle':
        pair = f"drone-{collision.get('drone_id', 'NA')}"
    dist = collision.get('distance', 'NA')
    print(f"{ctype}\t{pair}\t{dist}")
except Exception:
    print("NA\tNA\tNA")
PY
    )
  fi

  LAST_TIME_MS=$(grep -oE 'Time:[[:space:]]*[0-9]+(\.[0-9]+)?ms' "$CLEAN" | sed -E 's/Time:[[:space:]]*([0-9.]+)ms/\1/' | tail -n 1 || true)
  if [ -n "${LAST_TIME_MS:-}" ] && is_number "$LAST_TIME_MS"; then
    TOTAL_LAST_TIME_MS=$(awk -v a="$TOTAL_LAST_TIME_MS" -v b="$LAST_TIME_MS" 'BEGIN { printf "%.6f", a + b }')
    TOTAL_LAST_TIME_COUNT=$((TOTAL_LAST_TIME_COUNT + 1))
  fi

  STRICT_RESULT="FAIL"
  if [ "$SUCCESS_COUNT" -gt 0 ] && [ "$EMERGENCY_COUNT" -eq 0 ] && [ "$FATAL_COUNT" -eq 0 ]; then
    STRICT_RESULT="SUCCESS"
  fi

  BENCH_RESULT="FAIL"
  if [ "$SUCCESS_COUNT" -gt 0 ] && [ "$EMERGENCY_COUNT" -eq 0 ] && [ "$COLLISION_EVENT_COUNT" -eq 0 ]; then
    BENCH_RESULT="SUCCESS"
    TOTAL_BENCH_SUCCESS_ROUNDS=$((TOTAL_BENCH_SUCCESS_ROUNDS + 1))
  fi

  if [ "$SUCCESS_COUNT" -gt 0 ]; then
    TOTAL_SUCCESS_ROUNDS=$((TOTAL_SUCCESS_ROUNDS + 1))
  else
    TOTAL_FAIL_ROUNDS=$((TOTAL_FAIL_ROUNDS + 1))
  fi

  TOTAL_COLLISION_EVENT_COUNT=$((TOTAL_COLLISION_EVENT_COUNT + COLLISION_EVENT_COUNT))
  TOTAL_DRONE_DRONE_COLLISION_COUNT=$((TOTAL_DRONE_DRONE_COLLISION_COUNT + DRONE_DRONE_COLLISION_COUNT))
  TOTAL_DRONE_OBSTACLE_COLLISION_COUNT=$((TOTAL_DRONE_OBSTACLE_COLLISION_COUNT + DRONE_OBSTACLE_COLLISION_COUNT))
  TOTAL_MANDATORY_STOP_COUNT=$((TOTAL_MANDATORY_STOP_COUNT + MANDATORY_STOP_COUNT))
  TOTAL_EMERGENCY_COUNT=$((TOTAL_EMERGENCY_COUNT + EMERGENCY_COUNT))
  TOTAL_FATAL_COUNT=$((TOTAL_FATAL_COUNT + FATAL_COUNT))

  {
    echo "round=$i"
    echo "timestamp=$TS"
    echo "result=$STRICT_RESULT"
    echo "bench_result=$BENCH_RESULT"
    echo "strict_result=$STRICT_RESULT"
    echo "round_forced_stop=$ROUND_FORCED_STOP"
    echo "round_runtime_sec=$ROUND_SLEEP_SEC"
    echo "success_count=$SUCCESS_COUNT"
    echo "fail_count=$FAIL_COUNT"
    echo "collision_monitor_enabled=$COLLISION_MONITOR_ENABLED"
    echo "circle_launch_file=$CIRCLE_LAUNCH_FILE"
    echo "collision_event_count=$COLLISION_EVENT_COUNT"
    echo "drone_drone_collision_count=$DRONE_DRONE_COLLISION_COUNT"
    echo "drone_obstacle_collision_count=$DRONE_OBSTACLE_COLLISION_COUNT"
    echo "collision_type=$COLLISION_TYPE"
    echo "collision_pair=$COLLISION_PAIR"
    echo "collision_distance=$COLLISION_DISTANCE"
    echo "mandatory_stop_count=$MANDATORY_STOP_COUNT"
    echo "emergency_count=$EMERGENCY_COUNT"
    echo "fatal_count=$FATAL_COUNT"
    echo "comm_fuzz_enabled=$COMM_FUZZ_ENABLED_META"
    echo "comm_fuzz_seed=$COMM_FUZZ_SEED_USED"
    echo "comm_fuzz_mode_cfg=$COMM_FUZZ_MODE_CFG"
    echo "comm_fuzz_target_drone_id=$COMM_FUZZ_TARGET_USED"
    echo "comm_fuzz_mode_used=$COMM_FUZZ_MODE_USED"
    echo "comm_fuzz_start_s=$COMM_FUZZ_START_S"
    echo "comm_fuzz_duration_s=$COMM_FUZZ_DURATION_S"
    echo "comm_fuzz_factor_name=$COMM_FUZZ_FACTOR_NAME"
    echo "comm_fuzz_factor_value=$COMM_FUZZ_FACTOR_VALUE"
    echo "comm_sender_drone_id=$COMM_SENDER_DRONE_ID"
    echo "comm_receiver_drone_id=$COMM_RECEIVER_DRONE_ID"
    echo "comm_trace_line_count=$COMM_TRACE_LINE_COUNT"
    echo "comm_first_fuzz_ros_time=$COMM_FIRST_FUZZ_ROS_TIME"
    echo "comm_full_dump_sidecar_enabled=$COMM_FULL_DUMP_SIDECAR_FLAG"
    echo "comm_full_dump_sidecar_log=$SIDECAR_LOG"
    echo "comm_full_dump_file=$COMM_FULL_DUMP_FILE_PATH"
    echo "comm_full_dump_meta_file=$COMM_FULL_DUMP_META_FILE"
    echo "root_cause_primary=$ROOT_CAUSE_PRIMARY"
    echo "root_cause_types=$ROOT_CAUSE_TYPES"
    echo "root_cause_confidence=$ROOT_CAUSE_CONFIDENCE"
    echo "root_cause_summary=$ROOT_CAUSE_SUMMARY"
    echo "root_cause_event_kind=$ROOT_CAUSE_EVENT_KIND"
    echo "first_collision_time=$FIRST_COLLISION_TIME"
    echo "first_emergency_time=$FIRST_EMERGENCY_TIME"
    echo "comm_relation=$COMM_RELATION"
    echo "comm_relation_score=$COMM_RELATION_SCORE"
    echo "comm_relation_summary=$COMM_RELATION_SUMMARY"
    echo "plan_phase=$ROUND_PLAN_PHASE"
    echo "plan_source=$ROUND_PLAN_SOURCE"
    echo "param_key=$PLAN_ROUND_PARAM_KEY"
    echo "planner_rx_trace_glob=$PLANNER_RX_TRACE_GLOB"
    echo "planner_safety_trace_glob=$PLANNER_SAFETY_TRACE_GLOB"
    echo "raw_log=$RAW"
    echo "clean_log=$CLEAN"
    echo "runtime_clean_log=$RUNTIME_CLEAN"
    echo "shutdown_clean_log=$SHUTDOWN_CLEAN"
    echo "monitor_log=$MONITOR_LOG"
    echo "replan_count=$REPLAN_COUNT"
    echo "last_time_ms=$LAST_TIME_MS"
    echo "avg_time_ms=$AVG_TIME_MS"
    echo "summary_json=$SUMMARY_JSON"
    echo "report_dir=$FUZZ_REPORT_DIR_VALUE"
    echo "round_dir=$ROUND_DIR"
  } > "$TXT"

  {
    echo "round=$i result=$STRICT_RESULT bench_result=$BENCH_RESULT success=$SUCCESS_COUNT fail=$FAIL_COUNT collisions=$COLLISION_EVENT_COUNT collision_type=$COLLISION_TYPE collision_pair=$COLLISION_PAIR collision_distance=$COLLISION_DISTANCE replan_count=$REPLAN_COUNT last_time_ms=$LAST_TIME_MS avg_time_ms=$AVG_TIME_MS emergency=$EMERGENCY_COUNT fatal=$FATAL_COUNT forced_stop=$ROUND_FORCED_STOP round_runtime_sec=$ROUND_SLEEP_SEC monitor=$COLLISION_MONITOR_ENABLED launch=$CIRCLE_LAUNCH_FILE comm_fuzz_seed=$COMM_FUZZ_SEED_USED comm_fuzz_mode=$COMM_FUZZ_MODE_USED comm_sender=$COMM_SENDER_DRONE_ID comm_receiver=$COMM_RECEIVER_DRONE_ID comm_relation=$COMM_RELATION root_cause=$ROOT_CAUSE_PRIMARY"
    echo "detail_txt=$TXT"
    echo "round_dir=$ROUND_DIR"
    echo "report_dir=$FUZZ_REPORT_DIR_VALUE"
    echo "summary_json=$SUMMARY_JSON"
    echo
  } >> "$SUMMARY"

  {
    printf "round,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
      "$(csv_escape "$i")" \
      "$(csv_escape "$TS")" \
      "$(csv_escape "$STRICT_RESULT")" \
      "$(csv_escape "$BENCH_RESULT")" \
      "$(csv_escape "$STRICT_RESULT")" \
      "$(csv_escape "$COLLISION_MONITOR_ENABLED")" \
      "$(csv_escape "$CIRCLE_LAUNCH_FILE")" \
      "$(csv_escape "$COLLISION_EVENT_COUNT")" \
      "$(csv_escape "$DRONE_DRONE_COLLISION_COUNT")" \
      "$(csv_escape "$DRONE_OBSTACLE_COLLISION_COUNT")" \
      "$(csv_escape "$COLLISION_TYPE")" \
      "$(csv_escape "$COLLISION_PAIR")" \
      "$(csv_escape "$COLLISION_DISTANCE")" \
      "$(csv_escape "$MANDATORY_STOP_COUNT")" \
      "$(csv_escape "$REPLAN_COUNT")" \
      "$(csv_escape "$LAST_TIME_MS")" \
      "$(csv_escape "$AVG_TIME_MS")" \
      "$(csv_escape "$EMERGENCY_COUNT")" \
      "$(csv_escape "$FATAL_COUNT")" \
      "$(csv_escape "$ROUND_DIR")" \
      "$(csv_escape "$FUZZ_REPORT_DIR_VALUE")" \
      "$(csv_escape "$MONITOR_LOG")" \
      "$(csv_escape "$RAW")" \
      "$(csv_escape "$CLEAN")" \
      "$(csv_escape "$RUNTIME_CLEAN")" \
      "$(csv_escape "$SHUTDOWN_CLEAN")" \
        "$(csv_escape "$SUMMARY_JSON")" \
        "$(csv_escape "$COMM_FUZZ_ENABLED_META")" \
        "$(csv_escape "$COMM_FUZZ_SEED_USED")" \
        "$(csv_escape "$COMM_FUZZ_MODE_CFG")" \
        "$(csv_escape "$COMM_FUZZ_TARGET_USED")" \
        "$(csv_escape "$COMM_FUZZ_MODE_USED")" \
        "$(csv_escape "$COMM_FUZZ_START_S")" \
        "$(csv_escape "$COMM_FUZZ_DURATION_S")" \
        "$(csv_escape "$COMM_FUZZ_FACTOR_NAME")" \
        "$(csv_escape "$COMM_FUZZ_FACTOR_VALUE")" \
        "$(csv_escape "$COMM_SENDER_DRONE_ID")" \
        "$(csv_escape "$COMM_RECEIVER_DRONE_ID")" \
        "$(csv_escape "$COMM_FUZZ_META_FILE")" \
        "$(csv_escape "$COMM_FUZZ_TRACE_FILE")" \
        "$(csv_escape "$COMM_TRACE_LINE_COUNT")" \
        "$(csv_escape "$COMM_FIRST_FUZZ_ROS_TIME")" \
        "$(csv_escape "$ROOT_CAUSE_PRIMARY")" \
        "$(csv_escape "$ROOT_CAUSE_TYPES")" \
        "$(csv_escape "$ROOT_CAUSE_CONFIDENCE")" \
        "$(csv_escape "$ROOT_CAUSE_SUMMARY")" \
        "$(csv_escape "$ROOT_CAUSE_REPORT_FILE")" \
        "$(csv_escape "$ROOT_CAUSE_EVENT_KIND")" \
        "$(csv_escape "$FIRST_COLLISION_TIME")" \
        "$(csv_escape "$FIRST_EMERGENCY_TIME")" \
        "$(csv_escape "$COMM_RELATION")" \
        "$(csv_escape "$COMM_RELATION_SCORE")" \
        "$(csv_escape "$COMM_RELATION_SUMMARY")" \
        "$(csv_escape "$COMM_RELATION_REPORT_FILE")" \
        "$(csv_escape "$ROUND_PLAN_PHASE")" \
        "$(csv_escape "$ROUND_PLAN_SOURCE")" \
        "$(csv_escape "$PLAN_ROUND_PARAM_KEY")" \
        "$(csv_escape "$PLANNER_RX_TRACE_GLOB")" \
        "$(csv_escape "$PLANNER_SAFETY_TRACE_GLOB")"
  } >> "$SUMMARY_CSV"

  TOTAL_REPLAN_COUNT=$((TOTAL_REPLAN_COUNT + REPLAN_COUNT))
  if [ -n "${AVG_TIME_MS:-}" ] && is_number "$AVG_TIME_MS"; then
    TOTAL_AVG_TIME_MS=$(awk -v a="$TOTAL_AVG_TIME_MS" -v b="$AVG_TIME_MS" 'BEGIN { printf "%.6f", a + b }')
    TOTAL_AVG_TIME_COUNT=$((TOTAL_AVG_TIME_COUNT + 1))
  fi

  echo "[Round $i] done -> $TXT"
done

AVG_LAST_TIME_MS=$(safe_avg "$TOTAL_LAST_TIME_MS" "$TOTAL_LAST_TIME_COUNT")
AVG_AVG_TIME_MS=$(safe_avg "$TOTAL_AVG_TIME_MS" "$TOTAL_AVG_TIME_COUNT")
AVG_REPLAN_COUNT=$(safe_avg "$TOTAL_REPLAN_COUNT" "$EXECUTED_ROUNDS")
BATCH_END_EPOCH=$(date +%s)
BATCH_ELAPSED_SEC=$((BATCH_END_EPOCH - BATCH_START_EPOCH))

{
  echo "===== batch_overview ====="
  echo "planned_rounds=$RUNS"
  echo "executed_rounds=$EXECUTED_ROUNDS"
  echo "bench_success_rounds=$TOTAL_BENCH_SUCCESS_ROUNDS"
  echo "success_rounds=$TOTAL_SUCCESS_ROUNDS"
  echo "fail_rounds=$TOTAL_FAIL_ROUNDS"
  echo "total_collision_event_count=$TOTAL_COLLISION_EVENT_COUNT"
  echo "total_drone_drone_collision_count=$TOTAL_DRONE_DRONE_COLLISION_COUNT"
  echo "total_drone_obstacle_collision_count=$TOTAL_DRONE_OBSTACLE_COLLISION_COUNT"
  echo "total_mandatory_stop_count=$TOTAL_MANDATORY_STOP_COUNT"
  echo "total_replan_count=$TOTAL_REPLAN_COUNT"
  echo "avg_replan_count_per_round=$AVG_REPLAN_COUNT"
  echo "avg_avg_time_ms=$AVG_AVG_TIME_MS"
  echo "total_emergency_count=$TOTAL_EMERGENCY_COUNT"
  echo "total_fatal_count=$TOTAL_FATAL_COUNT"
  echo "avg_last_time_ms=$AVG_LAST_TIME_MS"
  echo "batch_elapsed_sec=$BATCH_ELAPSED_SEC"
  echo "stop_reason=$STOP_REASON"
} >> "$SUMMARY"

{
  printf "batch_overview,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
    "$(csv_escape "ALL")" \
    "$(csv_escape "$BATCH_TS")" \
    "$(csv_escape "SUCCESS")" \
    "$(csv_escape "$TOTAL_BENCH_SUCCESS_ROUNDS/$RUNS")" \
    "$(csv_escape "SUCCESS")" \
    "$(csv_escape "$COLLISION_MONITOR_ENABLED")" \
    "$(csv_escape "$CIRCLE_LAUNCH_FILE")" \
    "$(csv_escape "$TOTAL_COLLISION_EVENT_COUNT")" \
    "$(csv_escape "$TOTAL_DRONE_DRONE_COLLISION_COUNT")" \
    "$(csv_escape "$TOTAL_DRONE_OBSTACLE_COLLISION_COUNT")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "$TOTAL_MANDATORY_STOP_COUNT")" \
    "$(csv_escape "$AVG_REPLAN_COUNT")" \
    "$(csv_escape "$AVG_LAST_TIME_MS")" \
    "$(csv_escape "$AVG_AVG_TIME_MS")" \
    "$(csv_escape "$TOTAL_EMERGENCY_COUNT")" \
    "$(csv_escape "$TOTAL_FATAL_COUNT")" \
    "$(csv_escape "$BATCH_DIR")" \
    "$(csv_escape "$BATCH_DIR")" \
    "$(csv_escape "$BATCH_DIR")" \
    "$(csv_escape "$BATCH_DIR")" \
    "$(csv_escape "$BATCH_DIR")" \
    "$(csv_escape "$BATCH_DIR")" \
    "$(csv_escape "$BATCH_DIR")" \
    "$(csv_escape "$BATCH_DIR")" \
    "$(csv_escape "$BATCH_DIR")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")" \
    "$(csv_escape "NA")"
} >> "$SUMMARY_CSV"

echo "All rounds finished."
echo "Summary: $SUMMARY"
echo "Summary CSV: $SUMMARY_CSV"