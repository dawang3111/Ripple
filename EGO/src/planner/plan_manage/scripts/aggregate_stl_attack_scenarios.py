#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from collections import Counter, defaultdict
from typing import Any, Dict, Iterable, List, Optional, Tuple


SCENARIOS = [
    (1, "drone_drone_collision"),
    (2, "drone_obstacle_collision"),
    (3, "deadlock"),
    (4, "delay_timeout"),
    (5, "planning_failure"),
    (6, "drift"),
]


def read_json(path: str) -> Optional[dict]:
    try:
        with open(path, "r", encoding="ascii", errors="ignore") as f:
            return json.load(f)
    except Exception:
        return None


def safe_float(value) -> Optional[float]:
    try:
        v = float(value)
    except Exception:
        return None
    if math.isnan(v) or math.isinf(v):
        return None
    return v


def fmt_avg(values: List[float]) -> str:
    if not values:
        return "NA"
    return "{:.6f}".format(sum(values) / len(values))


def load_success_rounds(summary_csv: str, success_field: str) -> Optional[set]:
    if not summary_csv or not os.path.isfile(summary_csv):
        return None
    out = set()
    try:
        with open(summary_csv, "r", encoding="utf-8", errors="ignore", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row.get("row_type") != "round":
                    continue
                field = row.get(success_field, row.get("bench_result", row.get("result", "")))
                if str(field).strip().upper() == "SUCCESS":
                    try:
                        out.add(int(row.get("round", "")))
                    except Exception:
                        pass
    except Exception:
        return None
    return out


def iter_run_summaries(batch_dir: str) -> Iterable[Tuple[int, str, dict]]:
    for name in sorted(os.listdir(batch_dir)):
        if not name.startswith("run"):
            continue
        try:
            round_id = int(name[3:])
        except Exception:
            continue
        path = os.path.join(batch_dir, name, "stl_metrics_summary.json")
        data = read_json(path)
        if data:
            yield round_id, path, data


def fmt_avg_offset(values: List[float]) -> str:
    if not values:
        return "NA"
    return "{:.3f}".format(sum(values) / len(values))


def bystanders(drone_num: int, involved: List[Any]) -> str:
    involved_ids = set()
    for item in involved:
        if item is None:
            continue
        for part in str(item).split("|"):
            try:
                involved_ids.add(int(part))
            except Exception:
                pass
    out = [str(i) for i in range(drone_num) if i not in involved_ids]
    return "|".join(out) if out else "NA"


def pick_mode(counter: Counter) -> Tuple[str, str, str, str]:
    if not counter:
        return "NA", "NA", "NA", "NA"
    (attacker, victim, bystander, victim_fsm), _count = sorted(counter.items(), key=lambda kv: (-kv[1], kv[0]))[0]
    return attacker, victim, bystander, victim_fsm


def aggregate(batch_dir: str, summary_csv: str, success_field: str) -> List[Dict[str, str]]:
    values: Dict[int, List[float]] = defaultdict(list)
    evidence_offsets: Dict[int, List[float]] = defaultdict(list)
    combos: Dict[int, Counter] = defaultdict(Counter)
    source_counts: Dict[int, int] = defaultdict(int)
    success_rounds = load_success_rounds(summary_csv, success_field)
    drift_best: Optional[Tuple[float, int, str, str, str, str]] = None

    for round_id, _path, data in iter_run_summaries(batch_dir):
        if data.get("monitoring_started") is False:
            continue
        rows = data.get("rows", [])
        details = data.get("details", {})
        start_time = safe_float(data.get("task_start_ros_time"))
        if start_time is None:
            start_time = safe_float(data.get("start_ros_time"))
        drone_num = int(data.get("drone_num", 10) or 10)
        for row in rows:
            try:
                scenario_id = int(row.get("scenario_id"))
            except Exception:
                continue
            if scenario_id == 4 and success_rounds is not None and round_id not in success_rounds:
                continue
            value = safe_float(row.get("stl_value"))
            if value is None:
                continue
            values[scenario_id].append(value)
            source_counts[scenario_id] += 1
            detail = details.get(str(scenario_id), {})
            evidence_time = safe_float(detail.get("evidence_time"))
            if evidence_time is not None and start_time is not None:
                evidence_offsets[scenario_id].append(evidence_time - start_time)
            combo = (
                str(row.get("attacker_drone_id", "NA") or "NA"),
                str(row.get("victim_drone_id", "NA") or "NA"),
                str(row.get("bystander_drone_ids", "NA") or "NA"),
                str(row.get("victim_fsm_state", detail.get("victim_fsm_state", "NA")) or "NA"),
            )
            combos[scenario_id][combo] += 1

        drift_detail = details.get("6", {})
        drift_by_drone = drift_detail.get("drift_by_drone", {})
        if isinstance(drift_by_drone, dict):
            for victim, stats in drift_by_drone.items():
                if not isinstance(stats, dict):
                    continue
                max_distance = safe_float(stats.get("max_distance"))
                if max_distance is None or max_distance <= 0.0:
                    continue
                attacker = str(stats.get("attacker_drone_id", "NA") or "NA")
                victim_s = str(victim)
                victim_fsm = str(stats.get("victim_fsm_state", drift_detail.get("victim_fsm_state", "NA")) or "NA")
                bystander = bystanders(drone_num, [attacker, victim_s])
                evidence_time = safe_float(stats.get("max_time"))
                offset = evidence_time - start_time if evidence_time is not None and start_time is not None else None
                candidate = (max_distance, round_id, attacker, victim_s, bystander, victim_fsm)
                if drift_best is None or candidate[:2] > drift_best[:2]:
                    drift_best = candidate
                    if offset is not None:
                        evidence_offsets[6] = [offset]

    out: List[Dict[str, str]] = []
    for scenario_id, name in SCENARIOS:
        attacker, victim, bystander, victim_fsm = pick_mode(combos[scenario_id])
        if scenario_id == 6 and drift_best is not None:
            _dist, _round_id, attacker, victim, bystander, victim_fsm = drift_best
        out.append(
            {
                "scenario_id": str(scenario_id),
                "scenario_name": name,
                "stl_value_avg": fmt_avg(values[scenario_id]),
                "attacker_drone_id": attacker,
                "victim_drone_id": victim,
                "bystander_drone_ids": bystander,
                "evidence_time_offset_avg_s": fmt_avg_offset(evidence_offsets[scenario_id]),
                "victim_fsm_state": victim_fsm,
                "round_count": str(source_counts[scenario_id]),
            }
        )
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Aggregate per-round STL metrics into one six-row attack-scenario CSV")
    ap.add_argument("batch_dir")
    ap.add_argument("--summary-csv", default="")
    ap.add_argument("--success-field", default="bench_result")
    ap.add_argument("--output", default="")
    args = ap.parse_args()

    batch_dir = os.path.abspath(args.batch_dir)
    summary_csv = args.summary_csv or os.path.join(batch_dir, "summary.csv")
    output = args.output or os.path.join(batch_dir, "stl_attack_scenarios.csv")

    rows = aggregate(batch_dir, summary_csv, args.success_field)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="ascii", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "scenario_id",
                "scenario_name",
                "stl_value_avg",
                "attacker_drone_id",
                "victim_drone_id",
                "bystander_drone_ids",
                "evidence_time_offset_avg_s",
                "victim_fsm_state",
                "round_count",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
