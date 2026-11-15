#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


SCENARIO_NAMES = {
    1: "drone_drone_collision",
    2: "drone_obstacle_collision",
    3: "deadlock",
    4: "delay_timeout",
    5: "planning_failure",
    6: "drift",
}

SCENARIO_STL_FIELDS = {
    1: "stl_s1_drone_drone",
    2: "stl_s2_drone_obstacle",
    3: "stl_s3_deadlock",
    4: "stl_s4_delay_timeout",
    5: "stl_s5_planning_failure",
    6: "stl_s6_drift",
}

DRONE_RADIUS_M = 0.1

NUMERIC_FEATURES = [
    "inject_time_s",
    "latency_ms",
    "estimated_period_ms",
    "source_speed",
    "victim_speed",
    "source_accel",
    "victim_accel",
    "swarm_avg_current_speed",
    "swarm_avg_current_accel",
    "swarm_avg_speed_5s",
    "source_speed_over_swarm_avg_current",
    "source_accel_over_swarm_avg_current",
    "source_window_move_5s",
    "victim_window_move_5s",
    "triad_min_pair_distance",
    "triad_mean_pair_distance",
    "triad_max_pair_distance",
    "triad_best_distance_shrink_5s",
    "triad_future_min_distance",
    "triad_future_margin_to_1r",
    "triad_future_margin_to_2r",
    "triad_future_margin_to_0p8m",
    "triad_max_heading_angle_deg",
    "triad_mean_heading_angle_deg",
    "source_turn_angle_deg",
    "victim_turn_angle_deg",
    "source_trajectory_offset",
    "victim_trajectory_offset",
    "triad_max_trajectory_offset",
    "source_nearest_obstacle_distance",
    "victim_nearest_obstacle_distance",
    "triad_min_obstacle_distance",
    "source_future_obstacle_distance",
    "victim_future_obstacle_distance",
    "triad_future_min_obstacle_distance",
    "source_future_obstacle_margin_to_2r",
    "victim_future_obstacle_margin_to_2r",
    "triad_replan_or_stop_count",
    "triad_new_traj_count",
    "source_replan_stop_events_5s",
    "victim_replan_stop_events_5s",
    "triad_replan_stop_events_5s",
    "source_replan_since_traj_publish",
    "victim_replan_since_traj_publish",
    "triad_max_replan_since_traj_publish",
    "triad_min_pair_distance_over_radius",
    "triad_future_min_distance_over_radius",
    "triad_best_distance_shrink_5s_over_radius",
    "active_and_distance_shrinking",
    "active_and_current_close",
    "active_and_future_close",
    "active_and_obstacle_close",
    "current_close_and_future_close",
    "high_speed_and_distance_shrinking",
    "replan_dense_and_no_new_traj",
    "trajectory_offset_ge_0p2",
    "heading_aligned_le_20deg",
    "turning_ge_30deg",
    "obstacle_close_current_le_0p5",
    "obstacle_close_future_le_0p5",
    "source_speed_ge_1p4",
    "source_window_move_le_0p5",
    "source_window_move_le_0p2",
]

FSM_STATES = [
    "EXEC_TRAJ",
    "REPLAN_TRAJ",
    "EMERGENCY_STOP",
    "GEN_NEW_TRAJ",
    "SEQUENTIAL_START",
    "WAIT_TARGET",
]

BOOLEAN_FEATURES = set(
    [
        "active_and_distance_shrinking",
        "active_and_current_close",
        "active_and_future_close",
        "active_and_obstacle_close",
        "current_close_and_future_close",
        "high_speed_and_distance_shrinking",
        "replan_dense_and_no_new_traj",
        "trajectory_offset_ge_0p2",
        "heading_aligned_le_20deg",
        "turning_ge_30deg",
        "obstacle_close_current_le_0p5",
        "obstacle_close_future_le_0p5",
        "source_speed_ge_1p4",
        "source_window_move_le_0p5",
        "source_window_move_le_0p2",
    ]
)

RULE_DISCOVERY_EXCLUDE_FEATURES = set(
    [
        "inject_time_s",
        "latency_ms",
        "estimated_period_ms",
    ]
)


def safe_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        out = float(value)
    except Exception:
        return default
    if math.isnan(out) or math.isinf(out):
        return default
    return out


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(float(str(value).strip()))
    except Exception:
        return default


def split_ids(raw: str) -> List[int]:
    out: List[int] = []
    for item in str(raw or "").replace(",", ";").split(";"):
        item = item.strip()
        if not item or item == "NA":
            continue
        try:
            out.append(int(item))
        except Exception:
            continue
    return out


def parse_json(raw: str) -> Dict[str, Any]:
    try:
        value = json.loads(raw)
    except Exception:
        return {}
    return value if isinstance(value, dict) else {}


def percentile(values: Sequence[float], q: float) -> float:
    if not values:
        return float("nan")
    return float(np.percentile(np.asarray(values, dtype=float), q))


def ratio(num: Optional[float], den: Optional[float]) -> Optional[float]:
    if num is None or den is None or abs(den) <= 1e-9:
        return None
    return num / den


def min_present(values: Iterable[Optional[float]]) -> Optional[float]:
    vals = [x for x in values if x is not None]
    return min(vals) if vals else None


def max_present(values: Iterable[Optional[float]]) -> Optional[float]:
    vals = [x for x in values if x is not None]
    return max(vals) if vals else None


def bool_float(value: bool) -> float:
    return 1.0 if value else 0.0


def feature_value(row: Dict[str, Any], key: str) -> Optional[float]:
    value = row.get(key)
    return safe_float(value, None)


def add_feature(features: Dict[str, Any], key: str, value: Any) -> None:
    numeric = safe_float(value, None)
    features[key] = "" if numeric is None else "{:.10g}".format(numeric)


def discover_trials_file(input_root: Path) -> Path:
    candidates = [
        input_root / "random_packet_trials.csv",
        input_root / "global_rule_pool_trials.csv",
    ]
    for path in candidates:
        if path.is_file():
            return path
    found = sorted(input_root.glob("*_trials.csv"))
    if found:
        return found[0]
    raise FileNotFoundError("no trials csv found in {}".format(input_root))


def read_trials(trials_csv: Path) -> List[Dict[str, str]]:
    with trials_csv.open("r", encoding="ascii", errors="ignore", newline="") as f:
        return list(csv.DictReader(f))


def read_trial_evidence_offsets(trial: Dict[str, str]) -> Dict[int, float]:
    result_dir = Path(str(trial.get("result_dir", "")))
    candidates = [
        result_dir / "run1" / "stl_metrics_summary.json",
        result_dir / "stl_metrics_summary.json",
    ]
    summary = None
    for path in candidates:
        if path.is_file():
            try:
                with path.open("r", encoding="ascii", errors="ignore") as f:
                    summary = json.load(f)
                break
            except Exception:
                summary = None
    if not isinstance(summary, dict):
        return {}
    start_ros = safe_float(summary.get("start_ros_time"), None)
    if start_ros is None:
        return {}
    offsets: Dict[int, float] = {}
    details = summary.get("details", {})
    if not isinstance(details, dict):
        return offsets
    for sid_raw, detail in details.items():
        if not isinstance(detail, dict):
            continue
        sid = safe_int(sid_raw, -1)
        evidence_time = safe_float(detail.get("evidence_time"), None)
        if sid in SCENARIO_NAMES and evidence_time is not None:
            offsets[sid] = evidence_time - start_ros
    return offsets


def event_features(trial: Dict[str, str], event: Dict[str, str], evidence: Dict[str, Any], evidence_offsets: Dict[int, float]) -> Dict[str, Any]:
    features: Dict[str, Any] = {}
    features["trial_index"] = str(trial.get("trial_index", "NA"))
    features["event_index"] = str(event.get("injection_index", "NA"))
    features["attack_success"] = str(trial.get("attack_success", "no"))
    features["success_scenario_ids"] = str(trial.get("success_scenario_ids", ""))
    features["attacker_drone_id"] = str(event.get("attacker_drone_id", "NA"))
    features["victim_drone_id"] = str(event.get("victim_drone_id", evidence.get("victim_id", "NA")))
    features["triad_node_ids"] = str(event.get("triad_node_ids", evidence.get("candidate_node_ids", "NA")))
    features["source_fsm_state"] = str(evidence.get("source_fsm_state", event.get("attacker_fsm_state", "NA")))
    features["victim_fsm_state"] = str(evidence.get("victim_fsm_state", event.get("victim_fsm_state", "NA")))
    features["events_file"] = str(trial.get("events_file", "NA"))

    for sid, name in SCENARIO_NAMES.items():
        features[SCENARIO_STL_FIELDS[sid]] = str(trial.get(SCENARIO_STL_FIELDS[sid], ""))
        offset = evidence_offsets.get(sid)
        inject_time = safe_float(event.get("inject_time_s"), None)
        features["target_{}_evidence_offset_s".format(sid)] = "" if offset is None else "{:.10g}".format(offset)
        if offset is None or inject_time is None:
            features["target_{}_time_to_evidence_s".format(sid)] = ""
        else:
            features["target_{}_time_to_evidence_s".format(sid)] = "{:.10g}".format(offset - inject_time)

    raw_numeric: Dict[str, Optional[float]] = {}
    for key in [
        "inject_time_s",
        "latency_ms",
        "estimated_period_ms",
    ]:
        raw_numeric[key] = safe_float(event.get(key), None)
    for key in NUMERIC_FEATURES:
        if key not in raw_numeric:
            raw_numeric[key] = safe_float(evidence.get(key), None)

    source_speed = raw_numeric.get("source_speed")
    source_accel = raw_numeric.get("source_accel")
    swarm_speed = raw_numeric.get("swarm_avg_current_speed")
    swarm_accel = raw_numeric.get("swarm_avg_current_accel")
    raw_numeric["source_speed_over_swarm_avg_current"] = ratio(source_speed, swarm_speed)
    raw_numeric["source_accel_over_swarm_avg_current"] = ratio(source_accel, swarm_accel)

    triad_min_dist = raw_numeric.get("triad_min_pair_distance")
    future_min_dist = raw_numeric.get("triad_future_min_distance")
    shrink_5s = raw_numeric.get("triad_best_distance_shrink_5s")
    raw_numeric["triad_min_pair_distance_over_radius"] = ratio(triad_min_dist, DRONE_RADIUS_M)
    raw_numeric["triad_future_min_distance_over_radius"] = ratio(future_min_dist, DRONE_RADIUS_M)
    raw_numeric["triad_best_distance_shrink_5s_over_radius"] = ratio(shrink_5s, DRONE_RADIUS_M)

    source_obs = raw_numeric.get("source_nearest_obstacle_distance")
    victim_obs = raw_numeric.get("victim_nearest_obstacle_distance")
    source_future_obs = raw_numeric.get("source_future_obstacle_distance")
    victim_future_obs = raw_numeric.get("victim_future_obstacle_distance")
    raw_numeric["triad_min_obstacle_distance"] = min_present([source_obs, victim_obs])
    raw_numeric["triad_future_min_obstacle_distance"] = min_present([source_future_obs, victim_future_obs])

    source_offset = raw_numeric.get("source_trajectory_offset") or 0.0
    victim_offset = raw_numeric.get("victim_trajectory_offset") or 0.0
    raw_numeric["triad_max_trajectory_offset"] = max(source_offset, victim_offset)

    source_replans = raw_numeric.get("source_replan_since_traj_publish") or 0.0
    victim_replans = raw_numeric.get("victim_replan_since_traj_publish") or 0.0
    raw_numeric["triad_max_replan_since_traj_publish"] = max(source_replans, victim_replans)

    source_events = raw_numeric.get("source_replan_stop_events_5s") or 0.0
    victim_events = raw_numeric.get("victim_replan_stop_events_5s") or 0.0
    raw_numeric["triad_replan_stop_events_5s"] = source_events + victim_events

    active = (raw_numeric.get("triad_replan_or_stop_count") or 0.0) >= 2.0
    new_traj = (raw_numeric.get("triad_new_traj_count") or 0.0) >= 2.0
    shrinking = (shrink_5s or -1e9) >= 0.3
    current_close = (triad_min_dist or 1e9) <= 1.0
    future_close = (future_min_dist or 1e9) <= 0.8
    obstacle_close = (raw_numeric.get("triad_min_obstacle_distance") or 1e9) <= 0.5
    future_obstacle_close = (raw_numeric.get("triad_future_min_obstacle_distance") or 1e9) <= 0.5
    high_speed = (source_speed or -1e9) >= 1.4
    dense_replan = raw_numeric["triad_replan_stop_events_5s"] >= 5.0
    no_new_after_replan = raw_numeric["triad_max_replan_since_traj_publish"] >= 1.0
    traj_offset = raw_numeric["triad_max_trajectory_offset"] >= 0.2
    heading_aligned = (raw_numeric.get("triad_max_heading_angle_deg") or 1e9) <= 20.0
    turning = max_present([raw_numeric.get("source_turn_angle_deg"), raw_numeric.get("victim_turn_angle_deg")]) or 0.0
    source_window_move = raw_numeric.get("source_window_move_5s")

    raw_numeric["active_and_distance_shrinking"] = bool_float(active and new_traj and shrinking)
    raw_numeric["active_and_current_close"] = bool_float(active and current_close)
    raw_numeric["active_and_future_close"] = bool_float(active and future_close)
    raw_numeric["active_and_obstacle_close"] = bool_float(active and obstacle_close)
    raw_numeric["current_close_and_future_close"] = bool_float(current_close and future_close)
    raw_numeric["high_speed_and_distance_shrinking"] = bool_float(high_speed and shrinking)
    raw_numeric["replan_dense_and_no_new_traj"] = bool_float(dense_replan and no_new_after_replan)
    raw_numeric["trajectory_offset_ge_0p2"] = bool_float(traj_offset)
    raw_numeric["heading_aligned_le_20deg"] = bool_float(heading_aligned)
    raw_numeric["turning_ge_30deg"] = bool_float(turning >= 30.0)
    raw_numeric["obstacle_close_current_le_0p5"] = bool_float(obstacle_close)
    raw_numeric["obstacle_close_future_le_0p5"] = bool_float(future_obstacle_close)
    raw_numeric["source_speed_ge_1p4"] = bool_float(high_speed)
    raw_numeric["source_window_move_le_0p5"] = bool_float((source_window_move or 1e9) <= 0.5)
    raw_numeric["source_window_move_le_0p2"] = bool_float((source_window_move or 1e9) <= 0.2)

    for key in NUMERIC_FEATURES:
        add_feature(features, key, raw_numeric.get(key))

    source_state = str(features["source_fsm_state"])
    victim_state = str(features["victim_fsm_state"])
    for state in FSM_STATES:
        features["source_fsm_{}".format(state)] = "1" if source_state == state else "0"
        features["victim_fsm_{}".format(state)] = "1" if victim_state == state else "0"
    features["source_fsm_OTHER"] = "1" if source_state not in FSM_STATES else "0"
    features["victim_fsm_OTHER"] = "1" if victim_state not in FSM_STATES else "0"
    return features


def vectorize(input_root: Path, output_dir: Path) -> Tuple[Path, List[Dict[str, Any]], List[str]]:
    trials_csv = discover_trials_file(input_root)
    trials = read_trials(trials_csv)
    rows: List[Dict[str, Any]] = []

    for trial in trials:
        evidence_offsets = read_trial_evidence_offsets(trial)
        events_file = Path(str(trial.get("events_file", "")))
        if not events_file.is_absolute():
            events_file = input_root / events_file
        if not events_file.is_file():
            continue
        with events_file.open("r", encoding="ascii", errors="ignore", newline="") as f:
            for event in csv.DictReader(f):
                evidence = parse_json(str(event.get("trigger_evidence", "{}")))
                rows.append(event_features(trial, event, evidence, evidence_offsets))

    if not rows:
        raise RuntimeError("no events found under {}".format(input_root))

    fieldnames = [
        "trial_index",
        "event_index",
        "attack_success",
        "success_scenario_ids",
        "attacker_drone_id",
        "victim_drone_id",
        "triad_node_ids",
        "source_fsm_state",
        "victim_fsm_state",
        "events_file",
    ]
    for sid in SCENARIO_NAMES:
        fieldnames.append(SCENARIO_STL_FIELDS[sid])
        fieldnames.append("target_{}_evidence_offset_s".format(sid))
        fieldnames.append("target_{}_time_to_evidence_s".format(sid))
    feature_names = list(NUMERIC_FEATURES)
    for state in FSM_STATES + ["OTHER"]:
        feature_names.append("source_fsm_{}".format(state))
        feature_names.append("victim_fsm_{}".format(state))
    fieldnames.extend(feature_names)

    output_dir.mkdir(parents=True, exist_ok=True)
    vector_csv = output_dir / "vectorized_events.csv"
    with vector_csv.open("w", encoding="ascii", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    return vector_csv, rows, feature_names


def matrix_from_rows(rows: List[Dict[str, Any]], features: List[str]) -> np.ndarray:
    mat = np.empty((len(rows), len(features)), dtype=float)
    mat[:] = np.nan
    for i, row in enumerate(rows):
        for j, feature in enumerate(features):
            value = safe_float(row.get(feature), None)
            if value is not None:
                mat[i, j] = value
    return mat


def fit_preprocess(mat: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    med = np.nanmedian(mat, axis=0)
    med = np.where(np.isfinite(med), med, 0.0)
    filled = np.where(np.isnan(mat), med, mat)
    q75 = np.percentile(filled, 75, axis=0)
    q25 = np.percentile(filled, 25, axis=0)
    scale = q75 - q25
    scale = np.where(scale > 1e-9, scale, np.std(filled, axis=0))
    scale = np.where(scale > 1e-9, scale, 1.0)
    return (filled - med) / scale, med, scale


def apply_preprocess(mat: np.ndarray, med: np.ndarray, scale: np.ndarray) -> np.ndarray:
    filled = np.where(np.isnan(mat), med, mat)
    return (filled - med) / scale


def kmeans2(mat: np.ndarray, max_iter: int = 60) -> np.ndarray:
    n = mat.shape[0]
    if n == 0:
        return np.array([], dtype=int)
    if n == 1:
        return np.zeros(1, dtype=int)
    mean = np.mean(mat, axis=0)
    d = np.sum((mat - mean) ** 2, axis=1)
    c0 = int(np.argmax(d))
    d0 = np.sum((mat - mat[c0]) ** 2, axis=1)
    c1 = int(np.argmax(d0))
    if c0 == c1:
        return np.zeros(n, dtype=int)
    centers = np.vstack([mat[c0], mat[c1]])
    labels = np.zeros(n, dtype=int)
    for _ in range(max_iter):
        dist = np.stack([np.sum((mat - centers[k]) ** 2, axis=1) for k in range(2)], axis=1)
        new_labels = np.argmin(dist, axis=1)
        if np.array_equal(new_labels, labels):
            break
        labels = new_labels
        for k in range(2):
            if np.any(labels == k):
                centers[k] = np.mean(mat[labels == k], axis=0)
    return labels


def values_for(rows: List[Dict[str, Any]], feature: str) -> List[float]:
    out = []
    for row in rows:
        value = safe_float(row.get(feature), None)
        if value is not None:
            out.append(value)
    return out


def best_threshold(pos_vals: List[float], neg_vals: List[float], direction: str) -> Tuple[float, float, float, float]:
    vals = sorted(set(pos_vals + neg_vals))
    if not vals:
        return float("nan"), 0.0, 0.0, 0.0
    if len(vals) > 80:
        candidates = [percentile(vals, q) for q in range(5, 100, 5)]
    else:
        candidates = vals
    best = (float("nan"), -1.0, 0.0, 0.0)
    pos_n = float(len(pos_vals) or 1)
    neg_n = float(len(neg_vals) or 1)
    for threshold in candidates:
        if direction == ">=":
            tp = sum(1 for x in pos_vals if x >= threshold)
            fp = sum(1 for x in neg_vals if x >= threshold)
        else:
            tp = sum(1 for x in pos_vals if x <= threshold)
            fp = sum(1 for x in neg_vals if x <= threshold)
        recall = tp / pos_n
        fp_rate = fp / neg_n
        precision = tp / float(tp + fp or 1)
        f1 = 2.0 * precision * recall / float(precision + recall or 1.0)
        score = f1 + 0.25 * max(0.0, recall - fp_rate)
        if score > best[1]:
            best = (threshold, score, recall, fp_rate)
    return best[0], best[2], best[3], best[1]


def effect_size(pos_vals: List[float], neg_vals: List[float]) -> float:
    if len(pos_vals) < 2 or len(neg_vals) < 2:
        return 0.0
    pos = np.asarray(pos_vals, dtype=float)
    neg = np.asarray(neg_vals, dtype=float)
    pooled = math.sqrt((float(np.var(pos)) + float(np.var(neg))) / 2.0)
    if pooled <= 1e-9:
        return 0.0
    return (float(np.mean(pos)) - float(np.mean(neg))) / pooled


def feature_stats(pos_rows: List[Dict[str, Any]], neg_rows: List[Dict[str, Any]], feature_names: List[str], limit: int) -> List[Dict[str, Any]]:
    stats: List[Dict[str, Any]] = []
    for feature in feature_names:
        pos_vals = values_for(pos_rows, feature)
        neg_vals = values_for(neg_rows, feature)
        if len(pos_vals) < 3 or len(neg_vals) < 3:
            continue
        pos_med = percentile(pos_vals, 50)
        neg_med = percentile(neg_vals, 50)
        direction = ">=" if pos_med >= neg_med else "<="
        threshold, coverage, neg_hit_rate, score = best_threshold(pos_vals, neg_vals, direction)
        d = effect_size(pos_vals, neg_vals)
        stats.append(
            {
                "feature": feature,
                "direction": direction,
                "threshold": threshold,
                "positive_median": pos_med,
                "negative_median": neg_med,
                "positive_q25": percentile(pos_vals, 25),
                "positive_q75": percentile(pos_vals, 75),
                "negative_q25": percentile(neg_vals, 25),
                "negative_q75": percentile(neg_vals, 75),
                "positive_coverage": coverage,
                "negative_hit_rate": neg_hit_rate,
                "effect_size": d,
                "score": score + 0.05 * abs(d),
            }
        )
    stats.sort(key=lambda row: (row["score"], abs(row["effect_size"])), reverse=True)
    return stats[:limit]


def format_condition(row: Dict[str, Any]) -> str:
    threshold = float(row["threshold"])
    if row["feature"] in BOOLEAN_FEATURES or row["feature"].startswith("source_fsm_") or row["feature"].startswith("victim_fsm_"):
        return "{} = 1".format(row["feature"])
    return "{} {} {:.6g}".format(row["feature"], row["direction"], threshold)


def analyze_target(
    target_id: int,
    all_rows: List[Dict[str, Any]],
    feature_names: List[str],
    output_dir: Path,
    top_features: int,
    positive_window_s: float,
    args: argparse.Namespace,
) -> List[Dict[str, Any]]:
    target_success_rows = [row for row in all_rows if is_target_positive(row, target_id, args)]
    if positive_window_s > 0.0:
        pos_rows = []
        for row in target_success_rows:
            dt = safe_float(row.get("target_{}_time_to_evidence_s".format(target_id)), None)
            if dt is not None and 0.0 <= dt <= positive_window_s:
                pos_rows.append(row)
        if not pos_rows and target_success_rows:
            pos_rows = list(target_success_rows)
    else:
        pos_rows = list(target_success_rows)
    pos_ids = set(id(row) for row in pos_rows)
    target_success_ids = set(id(row) for row in target_success_rows)
    neg_rows = [row for row in all_rows if id(row) not in target_success_ids]
    target_dir = output_dir / "target_{}_{}".format(target_id, SCENARIO_NAMES[target_id])
    target_dir.mkdir(parents=True, exist_ok=True)

    summaries: List[Dict[str, Any]] = []
    assignment_fields = [
        "target_id",
        "target_name",
        "class_label",
        "cluster_id",
        "trial_index",
        "event_index",
        "attack_success",
        "success_scenario_ids",
        "attacker_drone_id",
        "victim_drone_id",
        "source_fsm_state",
        "victim_fsm_state",
    ]

    assignments: List[Dict[str, Any]] = []
    for class_label, class_rows, compare_rows in [
        ("positive", pos_rows, neg_rows),
        ("negative", neg_rows, pos_rows),
    ]:
        if not class_rows:
            continue
        class_mat = matrix_from_rows(class_rows, feature_names)
        scaled, med, scale = fit_preprocess(class_mat)
        labels = kmeans2(scaled)
        for row, label in zip(class_rows, labels):
            assignments.append(
                {
                    "target_id": target_id,
                    "target_name": SCENARIO_NAMES[target_id],
                    "class_label": class_label,
                    "cluster_id": int(label),
                    "trial_index": row.get("trial_index", "NA"),
                    "event_index": row.get("event_index", "NA"),
                    "attack_success": row.get("attack_success", "NA"),
                    "success_scenario_ids": row.get("success_scenario_ids", ""),
                    "attacker_drone_id": row.get("attacker_drone_id", "NA"),
                    "victim_drone_id": row.get("victim_drone_id", "NA"),
                    "source_fsm_state": row.get("source_fsm_state", "NA"),
                    "victim_fsm_state": row.get("victim_fsm_state", "NA"),
                }
            )

        for cluster_id in sorted(set(int(x) for x in labels)):
            cluster_rows = [row for row, label in zip(class_rows, labels) if int(label) == cluster_id]
            other_rows = compare_rows
            stats = feature_stats(cluster_rows, other_rows, feature_names, top_features)
            rule = " AND ".join(format_condition(item) for item in stats[:3]) if stats else "NA"
            summaries.append(
                {
                    "target_id": target_id,
                    "target_name": SCENARIO_NAMES[target_id],
                    "class_label": class_label,
                    "cluster_id": cluster_id,
                    "sample_count": len(cluster_rows),
                    "compare_sample_count": len(other_rows),
                    "positive_window_s": positive_window_s,
                    "label_mode": str(args.label_mode),
                    "derived_rule": rule,
                    "top_features": ";".join(str(item["feature"]) for item in stats),
                    "top_feature_directions": ";".join(str(item["direction"]) for item in stats),
                    "top_feature_thresholds": ";".join("{:.6g}".format(float(item["threshold"])) for item in stats),
                    "top_feature_positive_coverage": ";".join("{:.4f}".format(float(item["positive_coverage"])) for item in stats),
                    "top_feature_negative_hit_rate": ";".join("{:.4f}".format(float(item["negative_hit_rate"])) for item in stats),
                    "top_feature_effect_size": ";".join("{:.4f}".format(float(item["effect_size"])) for item in stats),
                }
            )

    assignment_path = target_dir / "cluster_assignments.csv"
    with assignment_path.open("w", encoding="ascii", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=assignment_fields)
        writer.writeheader()
        writer.writerows(assignments)

    summary_fields = [
        "target_id",
        "target_name",
        "class_label",
        "cluster_id",
        "sample_count",
        "compare_sample_count",
        "positive_window_s",
        "label_mode",
        "derived_rule",
        "top_features",
        "top_feature_directions",
        "top_feature_thresholds",
        "top_feature_positive_coverage",
        "top_feature_negative_hit_rate",
        "top_feature_effect_size",
    ]
    summary_path = target_dir / "rule_summary.csv"
    with summary_path.open("w", encoding="ascii", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(summaries)
    return summaries


def is_target_positive(row: Dict[str, Any], target_id: int, args: argparse.Namespace) -> bool:
    success_ids = set(str(row.get("success_scenario_ids", "")).split(";"))
    if str(target_id) in success_ids:
        return True
    if str(args.label_mode) != "relaxed":
        return False

    value = safe_float(row.get(SCENARIO_STL_FIELDS[target_id]), None)
    if value is None:
        return False
    if target_id == 2:
        return value <= float(args.relaxed_obstacle_margin_threshold)
    if target_id == 4:
        return value >= float(args.relaxed_delay_threshold_s)
    if target_id == 5:
        return value <= float(args.relaxed_planning_margin_threshold)
    if target_id == 6:
        return value >= float(args.relaxed_drift_threshold)
    return False


def write_manifest(output_dir: Path, input_root: Path, vector_csv: Path, rows: List[Dict[str, Any]], feature_names: List[str], positive_window_s: float, args: argparse.Namespace) -> None:
    scenario_event_counts: Dict[str, int] = {}
    scenario_trial_counts: Dict[str, int] = {}
    for sid in SCENARIO_NAMES:
        sid_s = str(sid)
        target_rows = [row for row in rows if is_target_positive(row, sid, args)]
        scenario_event_counts[sid_s] = len(target_rows)
        scenario_trial_counts[sid_s] = len(set(str(row.get("trial_index")) for row in target_rows))
    payload = {
        "created_at": datetime.now().isoformat(),
        "input_root": str(input_root),
        "vector_csv": str(vector_csv),
        "event_count": len(rows),
        "trial_count": len(set(str(row.get("trial_index")) for row in rows)),
        "feature_count": len(feature_names),
        "scenario_event_counts": scenario_event_counts,
        "scenario_trial_counts": scenario_trial_counts,
        "kmeans_k": 2,
        "positive_window_s": positive_window_s,
        "label_mode": str(args.label_mode),
        "relaxed_thresholds": {
            "target_2_stl_s2_le": float(args.relaxed_obstacle_margin_threshold),
            "target_4_stl_s4_ge": float(args.relaxed_delay_threshold_s),
            "target_5_stl_s5_le": float(args.relaxed_planning_margin_threshold),
            "target_6_stl_s6_ge": float(args.relaxed_drift_threshold),
        },
        "method": "task-wise contrastive clustering; positive and negative events are clustered separately with deterministic k=2 KMeans",
    }
    with (output_dir / "manifest.json").open("w", encoding="ascii") as f:
        json.dump(payload, f, indent=2, sort_keys=True)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="White-box contrastive rule discovery from attack logs.")
    ap.add_argument("--input-root", required=True, help="Attack result directory containing random_packet_trials.csv or global_rule_pool_trials.csv.")
    ap.add_argument("--output-dir", default=None, help="Output directory. Defaults to results/rule_discovery/<input>_<timestamp>.")
    ap.add_argument("--targets", default="1,2,3,4,5,6")
    ap.add_argument("--top-features", type=int, default=8)
    ap.add_argument("--positive-window-s", type=float, default=10.0, help="Use only successful target events within this many seconds before the target evidence time. Set <=0 to use all events from successful trials.")
    ap.add_argument("--label-mode", choices=("strict", "relaxed"), default="strict")
    ap.add_argument("--relaxed-obstacle-margin-threshold", type=float, default=0.025, help="Target 2 relaxed positive label: stl_s2_drone_obstacle <= threshold.")
    ap.add_argument("--relaxed-delay-threshold-s", type=float, default=57.0, help="Target 4 relaxed positive label: stl_s4_delay_timeout >= threshold.")
    ap.add_argument("--relaxed-planning-margin-threshold", type=float, default=0.01, help="Target 5 relaxed positive label: stl_s5_planning_failure <= threshold.")
    ap.add_argument("--relaxed-drift-threshold", type=float, default=14.0, help="Target 6 relaxed positive label: stl_s6_drift >= threshold. Current 3000-run data gives 109 trials at >=14.")
    return ap.parse_args()


def default_output_dir(input_root: Path) -> Path:
    root = Path.cwd() / "results" / "rule_discovery"
    return root / "{}_{}".format(input_root.name, datetime.now().strftime("%Y%m%d_%H%M%S"))


def main() -> int:
    args = parse_args()
    input_root = Path(args.input_root).resolve()
    output_dir = Path(args.output_dir).resolve() if args.output_dir else default_output_dir(input_root).resolve()
    targets = [safe_int(item, -1) for item in str(args.targets).split(",") if str(item).strip()]
    targets = [target for target in targets if target in SCENARIO_NAMES]
    if not targets:
        raise ValueError("no valid targets selected")

    vector_csv, rows, all_feature_names = vectorize(input_root, output_dir)
    feature_names = [feature for feature in all_feature_names if feature not in RULE_DISCOVERY_EXCLUDE_FEATURES]
    all_summaries: List[Dict[str, Any]] = []
    for target_id in targets:
        all_summaries.extend(analyze_target(target_id, rows, feature_names, output_dir, args.top_features, float(args.positive_window_s), args))

    summary_fields = [
        "target_id",
        "target_name",
        "class_label",
        "cluster_id",
        "sample_count",
        "compare_sample_count",
        "positive_window_s",
        "label_mode",
        "derived_rule",
        "top_features",
        "top_feature_directions",
        "top_feature_thresholds",
        "top_feature_positive_coverage",
        "top_feature_negative_hit_rate",
        "top_feature_effect_size",
    ]
    with (output_dir / "contrastive_rule_summary.csv").open("w", encoding="ascii", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(all_summaries)
    write_manifest(output_dir, input_root, vector_csv, rows, feature_names, float(args.positive_window_s), args)

    print("input_root={}".format(input_root))
    print("output_dir={}".format(output_dir))
    print("vectorized_events={}".format(vector_csv))
    print("event_count={}".format(len(rows)))
    print("rule_summary={}".format(output_dir / "contrastive_rule_summary.csv"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
