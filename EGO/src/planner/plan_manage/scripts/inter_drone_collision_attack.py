#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import subprocess
import sys
import tempfile
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ddc10_rules import DDC10_RULE_NAMES


SUCCESS_THRESHOLD = 0.0

SCENARIO_NAMES = {
    1: "drone_drone_collision",
    2: "drone_obstacle_collision",
    3: "deadlock",
    4: "delay_timeout",
    5: "planning_failure",
    6: "drift",
}

LOWER_OR_EQUAL_SCENARIOS = {1, 2, 3, 5}
GREATER_OR_EQUAL_SCENARIOS = {4, 6}
RULE_POOL_NAMES = {
    "R1": "triad_distance_lt_3r",
    "R2": "triad_heading_aligned",
    "R3": "triad_distance_closing_5s",
    "R4": "two_or_more_replanning",
    "R5": "pair_future_trajectory_collision",
    "R6": "two_or_more_new_trajectory",
}
GLOBAL_RULE_POOL_NAMES = {
    "OR1": "targeted_or_interaction_rule",
    "OR1_FULL": "targeted_or_interaction_rule_full",
    "OR1_LIMIT10": "targeted_or_interaction_rule_full_limit10",
    "OR1_NO_HEADING_ONLY": "targeted_or_interaction_rule_no_heading_only",
    "OR1_ACTIVE_CORE": "targeted_or_interaction_rule_active_core",
    "OR1_CLOSE_CORE": "targeted_or_interaction_rule_close_core",
    "DDC10": "dd_curated_target1_union_10_rules",
    "DDC10_STRICT": "dd_curated_target1_future_close_plus_two",
    "DDC10_CLOSE_ACTIVE": "dd_curated_target1_future_close_active",
    "DDC10_CLOSE_KINEMATIC": "dd_curated_target1_future_close_kinematic",
    "DD1": "dd_triad_all_pair_distance_lt_3r",
    "DD2": "dd_triad_heading_aligned",
    "DD3": "dd_triad_distance_closing_5s",
    "DD4": "dd_two_or_more_replan_or_stop",
    "DD5": "dd_pair_future_trajectory_lt_1r",
    "DD6": "dd_two_or_more_new_trajectory",
    "DO1": "do_receiver_heading_to_obstacle",
    "DO2": "do_receiver_future_obstacle_lt_1r",
    "DO3": "do_receiver_new_trajectory",
    "DO4": "do_receiver_obstacle_closing_5s",
    "DO5": "do_receiver_obstacle_distance_lt_2r",
    "DO6": "do_receiver_replan_or_stop",
    "DO_CURATED_1": "do_curated_future_obstacle_and_source_slow_ratio",
    "DO_CURATED_2": "do_curated_future_obstacle_and_victim_offset",
    "DO_CURATED_3": "do_curated_future_pair_source_slow_turn",
    "DL1": "dl_near_neighbor_move_lt_5r_5s",
    "DL2": "dl_two_near_neighbors_replan_or_stop",
    "DL3": "dl_two_neighbors_consider_each_other",
    "DL4": "dl_neighbor_two_replans_no_new_traj",
    "DL5": "dl_neighbor_three_replan_or_stop_5s",
    "DL6": "dl_two_neighbors_distance_lt_1r",
    "DL_CURATED_1": "dl_curated_future_pair_and_swarm_speed_le_1p395",
    "DL_CURATED_2": "dl_curated_future_pair_and_swarm_speed_le_1p381",
    "DE1": "de_other_speed_lt_half_swarm_avg_5s",
    "DE2": "de_progress_below_proxy_baseline",
    "DE3": "de_three_others_replan_or_stop",
    "DE4": "de_other_two_replans_no_new_traj",
    "DE5": "de_two_others_distance_lt_1r",
    "DE6": "de_other_three_replan_or_stop_5s",
    "DE_CURATED_1": "de_curated_obstacle_offset_current_speed",
    "DE_CURATED_2": "de_curated_future_pair_obstacle_current_speed",
    "DE_CURATED_3": "de_curated_accel_future_pair_window_speed",
    "DE_CURATED_4": "de_curated_replan_state_and_speed",
    "PF1": "pf_future_drone_trajectory_lt_2r",
    "PF2": "pf_future_obstacle_trajectory_lt_2r",
    "PF_CURATED_1": "pf_curated_current_speed_and_future_pair",
    "PF_CURATED_2": "pf_curated_future_pair_speed_victim_offset",
    "PF_CURATED_3": "pf_curated_future_pair_and_victim_obstacle",
    "PF_CURATED_4": "pf_curated_planning_activity_and_future_pair",
    "DR1": "dr_speed_gt_swarm_avg",
    "DR2": "dr_accel_gt_swarm_avg",
    "DR3": "dr_turning_gt_threshold",
    "DR4": "dr_new_traj_offset_gt_threshold",
    "DR_CURATED_1": "dr_curated_victim_obstacle_accel_heading",
    "DR_CURATED_2": "dr_curated_triad_obstacle_heading_low_move",
    "DR_CURATED_3": "dr_curated_victim_offset_speed_heading",
    "DR_CURATED_5": "dr_curated_tight_obstacle_heading_spread",
    "DR_CURATED_6": "dr_curated_victim_obstacle_accel",
}
GLOBAL_RULE_POOL_DEFAULT_IDS = [
    rule_id for rule_id in GLOBAL_RULE_POOL_NAMES if not rule_id.startswith("OR1") and not rule_id.startswith("DDC10") and not rule_id.startswith("DR_CURATED")
]
GLOBAL_RULE_POOL_NAMES.update(DDC10_RULE_NAMES)
RANDOM_PACKET_RULE_ID = "RAND"
RANDOM_PACKET_RULE_NAME = "random_packet_latency"


def workspace_results_root() -> Path:
    here = Path(__file__).resolve()
    for parent in [here.parent] + list(here.parents):
        if (parent / "src" / "run_headless.sh").is_file():
            return parent / "results"
    return Path.cwd() / "results"


def default_scenario_results_dir(scenario_id: int) -> Path:
    scenario_name = SCENARIO_NAMES.get(scenario_id, "unknown")
    return workspace_results_root() / "latency_attack_search" / "scenario_{}_{}".format(scenario_id, scenario_name)


def default_global_rule_pool_results_dir() -> Path:
    return workspace_results_root() / "latency_attack_search" / "global_rule_pool_latency"


def default_random_packet_results_dir() -> Path:
    return workspace_results_root() / "latency_attack_search" / "random_packet_latency_collect"


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
        return int(str(value).strip())
    except Exception:
        return default


def read_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        with path.open("r", encoding="ascii", errors="ignore") as f:
            return json.load(f)
    except Exception:
        return None


@dataclass
class BaselineScenario:
    scenario_id: int
    scenario_name: str
    baseline_stl_value: float
    injection_node_id: int
    primary_eval_node_id: int
    bystander_node_ids: str
    evidence_time_offset_s: float
    primary_eval_fsm_state: str


@dataclass(frozen=True)
class AttackScenarioConfig:
    scenario_id: int
    scenario_name: str
    success_operator: str
    success_threshold: float

    def objective_value(self, raw_stl_value: float) -> float:
        if self.success_operator == "<=":
            return raw_stl_value - self.success_threshold
        if self.success_operator == ">=":
            return self.success_threshold - raw_stl_value
        raise ValueError("unknown success operator: {}".format(self.success_operator))

    def is_success(self, raw_stl_value: float) -> bool:
        return self.objective_value(raw_stl_value) <= SUCCESS_THRESHOLD


@dataclass
class LatencyEvent:
    inject_time_s: float
    duration_s: float
    latency_ms: float
    direction: str
    injection_node_id: int
    primary_eval_node_id: int
    expected_primary_eval_fsm_state: str


@dataclass
class AttackCandidate:
    scenario_id: int
    scenario_name: str
    injection_count: int
    trial_index: int
    seed: int
    events: List[LatencyEvent]


@dataclass
class TrialResult:
    scenario_id: int
    scenario_name: str
    attack_mode: str
    injection_count: int
    trial_index: int
    injection_times_s: str
    latency_ms: str
    directions: str
    actual_injection_count: str
    actual_injection_times_s: str
    actual_latency_ms: str
    actual_attacker_node_ids: str
    actual_victim_node_ids: str
    status: str
    raw_stl_value: Optional[float]
    objective_value: Optional[float]
    attack_success: bool
    candidate_file: str
    result_dir: str
    adaptive_events_file: str
    note: str


class BaselineScenarioLoader:
    def __init__(self, baseline_csv: Path) -> None:
        self.baseline_csv = baseline_csv

    def load_scenario(self, scenario_id: int) -> BaselineScenario:
        if not self.baseline_csv.is_file():
            raise FileNotFoundError("baseline csv not found: {}".format(self.baseline_csv))
        if scenario_id not in SCENARIO_NAMES:
            raise ValueError("unsupported scenario_id={}, expected one of {}".format(scenario_id, sorted(SCENARIO_NAMES)))

        with self.baseline_csv.open("r", encoding="utf-8", errors="ignore", newline="") as f:
            for row in csv.DictReader(f):
                if safe_int(row.get("scenario_id"), -1) != scenario_id:
                    continue
                injection_id = row.get("injection_node_id", row.get("attacker_drone_id", "NA"))
                eval_id = row.get("primary_eval_node_id", row.get("victim_drone_id", "NA"))
                fsm_state = row.get("primary_eval_fsm_state", row.get("victim_fsm_state", "EXEC_TRAJ"))
                return BaselineScenario(
                    scenario_id=scenario_id,
                    scenario_name=str(row.get("scenario_name", SCENARIO_NAMES[scenario_id])),
                    baseline_stl_value=safe_float(row.get("stl_value_avg"), 0.0) or 0.0,
                    injection_node_id=safe_int(injection_id, 0),
                    primary_eval_node_id=safe_int(eval_id, 1),
                    bystander_node_ids=str(row.get("bystander_node_ids", row.get("bystander_drone_ids", "NA"))),
                    evidence_time_offset_s=safe_float(row.get("evidence_time_offset_avg_s"), 20.0) or 20.0,
                    primary_eval_fsm_state=str(fsm_state or "EXEC_TRAJ"),
                )

        raise ValueError("scenario_id={} not found in {}".format(scenario_id, self.baseline_csv))

    def load_all_stl_values(self) -> Dict[int, float]:
        values: Dict[int, float] = {}
        if not self.baseline_csv.is_file():
            return values
        with self.baseline_csv.open("r", encoding="utf-8", errors="ignore", newline="") as f:
            for row in csv.DictReader(f):
                scenario_id = safe_int(row.get("scenario_id"), -1)
                if scenario_id not in SCENARIO_NAMES:
                    continue
                value = safe_float(row.get("stl_value_avg"), None)
                if value is not None:
                    values[scenario_id] = value
        return values


class STLEvaluator:
    def evaluate_scenario(self, batch_dir: Path, scenario: AttackScenarioConfig) -> Tuple[Optional[float], Optional[float], bool, str]:
        summary = self._latest_run_summary(batch_dir)
        if summary is None:
            return None, None, False, "missing_stl_metrics_summary"

        for row in summary.get("rows", []):
            if safe_int(row.get("scenario_id"), -1) == scenario.scenario_id:
                value = safe_float(row.get("stl_value"))
                if value is None:
                    return None, None, False, "invalid_stl_value"
                objective = scenario.objective_value(value)
                return value, objective, scenario.is_success(value), "ok"
        return None, None, False, "scenario_{}_missing".format(scenario.scenario_id)

    def _latest_run_summary(self, batch_dir: Path) -> Optional[Dict[str, Any]]:
        candidates = sorted(batch_dir.glob("run*/stl_metrics_summary.json"))
        if not candidates:
            return None
        return read_json(candidates[-1])


class LatencyConstraintModel:
    def __init__(
        self,
        max_vel_mps: float,
        max_acc_mps2: float,
        physical_offset_limit_m: float,
        protocol_timeout_ms: float,
        max_latency_ratio: float,
        min_event_spacing_s: float,
    ) -> None:
        self.max_vel_mps = max_vel_mps
        self.max_acc_mps2 = max_acc_mps2
        self.physical_offset_limit_m = physical_offset_limit_m
        self.protocol_timeout_ms = protocol_timeout_ms
        self.max_latency_ratio = max_latency_ratio
        self.min_event_spacing_s = min_event_spacing_s

    def validate(self, candidate: AttackCandidate) -> Tuple[bool, str]:
        times = sorted(event.inject_time_s for event in candidate.events)
        for prev, cur in zip(times, times[1:]):
            if cur - prev < self.min_event_spacing_s:
                return False, "event_spacing_too_small"

        max_latency_ms = self.protocol_timeout_ms * self.max_latency_ratio
        for event in candidate.events:
            if event.latency_ms <= 0.0 or event.latency_ms > max_latency_ms:
                return False, "latency_out_of_protocol_range"
            if event.duration_s <= 0.0:
                return False, "duration_must_be_positive"
            if event.direction not in ("tx", "rx", "both"):
                return False, "invalid_direction"
            latency_s = event.latency_ms / 1000.0
            extrapolated = self.max_vel_mps * latency_s + 0.5 * self.max_acc_mps2 * latency_s * latency_s
            if extrapolated > self.physical_offset_limit_m:
                return False, "latency_breaks_physical_extrapolation_limit"
        return True, "ok"


class LatencyPerturbationOptimizer:
    def __init__(
        self,
        baseline: BaselineScenario,
        constraints: LatencyConstraintModel,
        seed: int,
        time_jitter_s: float,
        latency_range_ms: Tuple[float, float],
        duration_range_s: Tuple[float, float],
        directions: Iterable[str],
    ) -> None:
        self.baseline = baseline
        self.constraints = constraints
        self.rng = random.Random(seed)
        self.seed = seed
        self.time_jitter_s = time_jitter_s
        self.latency_range_ms = latency_range_ms
        self.duration_range_s = duration_range_s
        self.directions = list(directions)
        self.elite: List[Tuple[float, AttackCandidate]] = []

    def propose(self, injection_count: int, trial_index: int) -> AttackCandidate:
        if self.elite and self.rng.random() < 0.6:
            candidate = self._mutate_elite(injection_count, trial_index)
        else:
            candidate = self._sample_random(injection_count, trial_index)

        for _ in range(100):
            ok, _reason = self.constraints.validate(candidate)
            if ok:
                return candidate
            candidate = self._sample_random(injection_count, trial_index)
        return candidate

    def observe(self, candidate: AttackCandidate, robustness: Optional[float]) -> None:
        if robustness is None:
            return
        self.elite.append((robustness, candidate))
        self.elite = sorted(self.elite, key=lambda item: item[0])[:12]

    def _sample_random(self, injection_count: int, trial_index: int) -> AttackCandidate:
        events: List[LatencyEvent] = []
        center = self.baseline.evidence_time_offset_s
        if injection_count == 1:
            offsets = [self.rng.uniform(-self.time_jitter_s, self.time_jitter_s)]
        else:
            span = min(self.time_jitter_s, max(0.5, 0.75 * injection_count))
            base = [(-span + 2.0 * span * i / (injection_count - 1)) for i in range(injection_count)]
            offsets = [x + self.rng.uniform(-0.35, 0.35) for x in base]

        for offset in offsets:
            events.append(self._make_event(center + offset))
        events = sorted(events, key=lambda event: event.inject_time_s)
        return AttackCandidate(
            scenario_id=self.baseline.scenario_id,
            scenario_name=self.baseline.scenario_name,
            injection_count=injection_count,
            trial_index=trial_index,
            seed=self.seed + injection_count * 1000003 + trial_index,
            events=events,
        )

    def _mutate_elite(self, injection_count: int, trial_index: int) -> AttackCandidate:
        _score, parent = self.rng.choice(self.elite)
        events = list(parent.events[:injection_count])
        while len(events) < injection_count:
            events.append(self._make_event(self.baseline.evidence_time_offset_s + self.rng.uniform(-self.time_jitter_s, self.time_jitter_s)))

        mutated = []
        for event in events:
            latency = self._clamp(
                event.latency_ms + self.rng.uniform(-25.0, 25.0),
                self.latency_range_ms[0],
                self.latency_range_ms[1],
            )
            inject_time = max(0.0, event.inject_time_s + self.rng.uniform(-0.8, 0.8))
            duration = self._clamp(
                event.duration_s + self.rng.uniform(-0.4, 0.4),
                self.duration_range_s[0],
                self.duration_range_s[1],
            )
            mutated.append(
                LatencyEvent(
                    inject_time_s=inject_time,
                    duration_s=duration,
                    latency_ms=latency,
                    direction=event.direction if self.rng.random() > 0.2 else self.rng.choice(self.directions),
                    injection_node_id=self.baseline.injection_node_id,
                    primary_eval_node_id=self.baseline.primary_eval_node_id,
                    expected_primary_eval_fsm_state=self.baseline.primary_eval_fsm_state,
                )
            )
        return AttackCandidate(
            scenario_id=self.baseline.scenario_id,
            scenario_name=self.baseline.scenario_name,
            injection_count=injection_count,
            trial_index=trial_index,
            seed=self.seed + injection_count * 1000003 + trial_index,
            events=sorted(mutated, key=lambda event: event.inject_time_s),
        )

    def _make_event(self, inject_time_s: float) -> LatencyEvent:
        return LatencyEvent(
            inject_time_s=max(0.0, inject_time_s),
            duration_s=self.rng.uniform(self.duration_range_s[0], self.duration_range_s[1]),
            latency_ms=self.rng.uniform(self.latency_range_ms[0], self.latency_range_ms[1]),
            direction=self.rng.choice(self.directions),
            injection_node_id=self.baseline.injection_node_id,
            primary_eval_node_id=self.baseline.primary_eval_node_id,
            expected_primary_eval_fsm_state=self.baseline.primary_eval_fsm_state,
        )

    def _clamp(self, value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))


class AttackTupleWriter:
    def __init__(self, output_dir: Path) -> None:
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def write_candidate(self, candidate: AttackCandidate) -> Path:
        path = self.output_dir / "candidates" / "k{}_trial{:04d}.json".format(candidate.injection_count, candidate.trial_index)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = asdict(candidate)
        payload["events"] = [asdict(event) for event in candidate.events]
        with path.open("w", encoding="ascii") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
        return path

    def write_rule_pool_candidate(self, rule_id: str, candidate: AttackCandidate) -> Path:
        path = self.output_dir / "candidates" / rule_id / "k{}_trial{:04d}.json".format(candidate.injection_count, candidate.trial_index)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = asdict(candidate)
        payload["active_rule_id"] = rule_id
        payload["active_rule_name"] = RULE_POOL_NAMES.get(rule_id, "unknown")
        payload["events"] = [asdict(event) for event in candidate.events]
        with path.open("w", encoding="ascii") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
        return path

    def write_best(self, result: TrialResult, candidate: AttackCandidate) -> Path:
        path = self.output_dir / "best_attack_tuple.json"
        payload = {
            "scenario_id": candidate.scenario_id,
            "scenario_name": candidate.scenario_name,
            "attack_mode": result.attack_mode,
            "objective": "minimize_objective_value",
            "objective_success_threshold": SUCCESS_THRESHOLD,
            "raw_stl_value": result.raw_stl_value,
            "objective_value": result.objective_value,
            "raw_success_condition": "raw_stl_value {} scenario_threshold".format(self._success_operator_from_result(result)),
            "scenario_threshold": self._scenario_threshold_from_result(result),
            "attack_success": result.attack_success,
            "status": result.status,
            "tuple": asdict(candidate),
            "actual_injection_count": result.actual_injection_count,
            "actual_injection_times_s": result.actual_injection_times_s,
            "actual_latency_ms": result.actual_latency_ms,
            "actual_attacker_node_ids": result.actual_attacker_node_ids,
            "actual_victim_node_ids": result.actual_victim_node_ids,
            "result_dir": result.result_dir,
            "adaptive_events_file": result.adaptive_events_file,
        }
        with path.open("w", encoding="ascii") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
        return path

    def _success_operator_from_result(self, result: TrialResult) -> str:
        return ">=" if result.scenario_id in GREATER_OR_EQUAL_SCENARIOS else "<="

    def _scenario_threshold_from_result(self, result: TrialResult) -> Optional[float]:
        if result.raw_stl_value is None or result.objective_value is None:
            return None
        if result.scenario_id in GREATER_OR_EQUAL_SCENARIOS:
            return result.objective_value + result.raw_stl_value
        return result.raw_stl_value - result.objective_value

    def append_trial(self, result: TrialResult) -> None:
        path = self.output_dir / "attack_trials.csv"
        exists = path.exists()
        with path.open("a", encoding="ascii", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "scenario_id",
                    "scenario_name",
                    "attack_mode",
                    "injection_count",
                    "trial_index",
                    "injection_times_s",
                    "latency_ms",
                    "directions",
                    "actual_injection_count",
                    "actual_injection_times_s",
                    "actual_latency_ms",
                    "actual_attacker_node_ids",
                    "actual_victim_node_ids",
                    "status",
                    "raw_stl_value",
                    "objective_value",
                    "attack_success",
                    "candidate_file",
                    "result_dir",
                    "adaptive_events_file",
                    "note",
                ],
            )
            if not exists:
                writer.writeheader()
            row = asdict(result)
            row["raw_stl_value"] = "NA" if result.raw_stl_value is None else "{:.6f}".format(result.raw_stl_value)
            row["objective_value"] = "NA" if result.objective_value is None else "{:.6f}".format(result.objective_value)
            row["attack_success"] = "yes" if result.attack_success else "no"
            writer.writerow(row)

    def append_rule_pool_trial(self, rule_id: str, result: TrialResult) -> None:
        path = self.output_dir / "rule_pool_trials.csv"
        exists = path.exists()
        event_summary = self._rule_event_summary(Path(result.adaptive_events_file)) if result.adaptive_events_file != "NA" else {}
        with path.open("a", encoding="ascii", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "rule_id",
                    "rule_name",
                    "trial_index",
                    "attack_success",
                    "raw_stl_value",
                    "objective_value",
                    "actual_injection_count",
                    "actual_injection_times_s",
                    "actual_latency_ms",
                    "actual_attacker_node_ids",
                    "actual_victim_node_ids",
                    "triad_node_ids",
                    "trigger_evidence",
                    "status",
                    "result_dir",
                    "events_file",
                    "note",
                ],
            )
            if not exists:
                writer.writeheader()
            writer.writerow(
                {
                    "rule_id": rule_id,
                    "rule_name": RULE_POOL_NAMES.get(rule_id, "unknown"),
                    "trial_index": result.trial_index,
                    "attack_success": "yes" if result.attack_success else "no",
                    "raw_stl_value": "NA" if result.raw_stl_value is None else "{:.6f}".format(result.raw_stl_value),
                    "objective_value": "NA" if result.objective_value is None else "{:.6f}".format(result.objective_value),
                    "actual_injection_count": result.actual_injection_count,
                    "actual_injection_times_s": result.actual_injection_times_s,
                    "actual_latency_ms": result.actual_latency_ms,
                    "actual_attacker_node_ids": result.actual_attacker_node_ids,
                    "actual_victim_node_ids": result.actual_victim_node_ids,
                    "triad_node_ids": event_summary.get("triad_node_ids", "NA"),
                    "trigger_evidence": event_summary.get("trigger_evidence", "NA"),
                    "status": result.status,
                    "result_dir": result.result_dir,
                    "events_file": result.adaptive_events_file,
                    "note": result.note,
                }
            )

    def write_rule_pool_summary(self, rows: List[Dict[str, Any]]) -> Path:
        path = self.output_dir / "rule_pool_summary.csv"
        by_rule: Dict[str, List[Dict[str, Any]]] = {}
        for row in rows:
            by_rule.setdefault(str(row["rule_id"]), []).append(row)
        with path.open("w", encoding="ascii", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "rule_id",
                    "rule_name",
                    "trial_count",
                    "success_count",
                    "success_probability",
                    "best_stl_value",
                    "avg_stl_value",
                    "avg_actual_injection_count",
                    "best_trial_index",
                    "best_events_file",
                ],
            )
            writer.writeheader()
            for rule_id in sorted(by_rule):
                rule_rows = by_rule[rule_id]
                raw_values = [float(row["raw_stl_value"]) for row in rule_rows if row.get("raw_stl_value") is not None]
                actual_counts = [safe_float(row.get("actual_injection_count"), None) for row in rule_rows]
                actual_counts = [x for x in actual_counts if x is not None]
                successes = [row for row in rule_rows if row.get("attack_success")]
                best = min(
                    [row for row in rule_rows if row.get("objective_value") is not None],
                    key=lambda row: float(row["objective_value"]),
                    default=None,
                )
                writer.writerow(
                    {
                        "rule_id": rule_id,
                        "rule_name": RULE_POOL_NAMES.get(rule_id, "unknown"),
                        "trial_count": len(rule_rows),
                        "success_count": len(successes),
                        "success_probability": "{:.6f}".format(len(successes) / float(len(rule_rows) or 1)),
                        "best_stl_value": "NA" if best is None or best.get("raw_stl_value") is None else "{:.6f}".format(float(best["raw_stl_value"])),
                        "avg_stl_value": "NA" if not raw_values else "{:.6f}".format(sum(raw_values) / len(raw_values)),
                        "avg_actual_injection_count": "NA" if not actual_counts else "{:.6f}".format(sum(actual_counts) / len(actual_counts)),
                        "best_trial_index": "NA" if best is None else best.get("trial_index", "NA"),
                        "best_events_file": "NA" if best is None else best.get("events_file", "NA"),
                    }
                )
        return path

    def write_rule_pool_best(self, rows: List[Dict[str, Any]]) -> Path:
        path = self.output_dir / "rule_pool_best_by_rule.json"
        best_by_rule: Dict[str, Dict[str, Any]] = {}
        for rule_id in sorted({str(row["rule_id"]) for row in rows}):
            candidates = [row for row in rows if str(row["rule_id"]) == rule_id and row.get("objective_value") is not None]
            if not candidates:
                continue
            best_by_rule[rule_id] = min(candidates, key=lambda row: float(row["objective_value"]))
        with path.open("w", encoding="ascii") as f:
            json.dump(best_by_rule, f, indent=2, sort_keys=True)
        return path

    def write_global_rule_pool_candidate(self, rule_id: str, candidate: AttackCandidate) -> Path:
        path = self.output_dir / "candidates" / "global" / rule_id / "k{}_trial{:04d}.json".format(candidate.injection_count, candidate.trial_index)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = asdict(candidate)
        payload["active_rule_id"] = rule_id
        payload["active_rule_name"] = GLOBAL_RULE_POOL_NAMES.get(rule_id, "unknown")
        payload["events"] = [asdict(event) for event in candidate.events]
        with path.open("w", encoding="ascii") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
        return path

    def append_global_rule_pool_trial(self, row: Dict[str, Any]) -> None:
        path = self.output_dir / "global_rule_pool_trials.csv"
        exists = path.exists()
        event_summary = self._rule_event_summary(Path(str(row.get("events_file", "NA")))) if row.get("events_file") not in (None, "NA") else {}
        fields = [
            "rule_id",
            "rule_name",
            "trial_index",
            "attack_success",
            "success_target_scenario_id",
            "success_scenario_ids",
            "success_scenario_names",
            "best_success_scenario_id",
            "best_success_scenario_name",
            "stl_s1_drone_drone",
            "stl_s2_drone_obstacle",
            "stl_s3_deadlock",
            "stl_s4_delay_timeout",
            "stl_s5_planning_failure",
            "stl_s6_drift",
            "best_objective_value",
            "actual_injection_count",
            "actual_injection_times_s",
            "actual_latency_ms",
            "actual_attacker_node_ids",
            "actual_victim_node_ids",
            "candidate_node_ids",
            "trigger_evidence",
            "status",
            "result_dir",
            "events_file",
            "note",
        ]
        with path.open("a", encoding="ascii", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            if not exists:
                writer.writeheader()
            out = {field: row.get(field, "NA") for field in fields}
            out["candidate_node_ids"] = event_summary.get("triad_node_ids", row.get("candidate_node_ids", "NA"))
            out["trigger_evidence"] = event_summary.get("trigger_evidence", row.get("trigger_evidence", "NA"))
            writer.writerow(out)

    def write_global_rule_pool_summary(self, rows: List[Dict[str, Any]]) -> Path:
        path = self.output_dir / "global_rule_pool_summary.csv"
        by_rule: Dict[str, List[Dict[str, Any]]] = {}
        for row in rows:
            by_rule.setdefault(str(row["rule_id"]), []).append(row)
        fields = [
            "rule_id",
            "rule_name",
            "success_target_scenario_id",
            "trial_count",
            "injection_trigger_count",
            "injection_trigger_probability",
            "avg_actual_injection_count",
            "success_count",
            "success_probability",
            "success_scenario_id_counts",
            "best_trial_index",
            "best_success_scenario_id",
            "best_success_scenario_name",
            "best_objective_value",
            "best_events_file",
        ]
        with path.open("w", encoding="ascii", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for rule_id in sorted(by_rule):
                rule_rows = by_rule[rule_id]
                successes = [row for row in rule_rows if row.get("attack_success") == "yes"]
                actual_counts = [safe_float(row.get("actual_injection_count"), None) for row in rule_rows]
                actual_counts = [value for value in actual_counts if value is not None]
                triggered = [value for value in actual_counts if value > 0.0]
                scenario_counts: Dict[str, int] = {}
                for row in successes:
                    for sid in str(row.get("success_scenario_ids", "")).split(";"):
                        sid = sid.strip()
                        if sid:
                            scenario_counts[sid] = scenario_counts.get(sid, 0) + 1
                best = min(
                    [row for row in rule_rows if row.get("best_objective_value") not in (None, "NA")],
                    key=lambda row: float(row["best_objective_value"]),
                    default=None,
                )
                writer.writerow(
                    {
                        "rule_id": rule_id,
                        "rule_name": GLOBAL_RULE_POOL_NAMES.get(rule_id, "unknown"),
                        "success_target_scenario_id": rule_rows[0].get("success_target_scenario_id", "ALL"),
                        "trial_count": len(rule_rows),
                        "injection_trigger_count": len(triggered),
                        "injection_trigger_probability": "{:.6f}".format(len(triggered) / float(len(rule_rows) or 1)),
                        "avg_actual_injection_count": "NA" if not actual_counts else "{:.6f}".format(sum(actual_counts) / len(actual_counts)),
                        "success_count": len(successes),
                        "success_probability": "{:.6f}".format(len(successes) / float(len(rule_rows) or 1)),
                        "success_scenario_id_counts": json.dumps(scenario_counts, sort_keys=True),
                        "best_trial_index": "NA" if best is None else best.get("trial_index", "NA"),
                        "best_success_scenario_id": "NA" if best is None else best.get("best_success_scenario_id", "NA"),
                        "best_success_scenario_name": "NA" if best is None else best.get("best_success_scenario_name", "NA"),
                        "best_objective_value": "NA" if best is None else best.get("best_objective_value", "NA"),
                        "best_events_file": "NA" if best is None else best.get("events_file", "NA"),
                    }
                )
        return path

    def write_global_rule_pool_best(self, rows: List[Dict[str, Any]]) -> Path:
        path = self.output_dir / "global_rule_pool_best_by_rule.json"
        best_by_rule: Dict[str, Dict[str, Any]] = {}
        for rule_id in sorted({str(row["rule_id"]) for row in rows}):
            candidates = [row for row in rows if str(row["rule_id"]) == rule_id and row.get("best_objective_value") not in (None, "NA")]
            if not candidates:
                continue
            best_by_rule[rule_id] = min(candidates, key=lambda row: float(row["best_objective_value"]))
        with path.open("w", encoding="ascii") as f:
            json.dump(best_by_rule, f, indent=2, sort_keys=True)
        return path

    def write_random_packet_candidate(self, candidate: AttackCandidate) -> Path:
        path = self.output_dir / "candidates" / "random_packet" / "trial{:04d}.json".format(candidate.trial_index)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = asdict(candidate)
        payload["active_rule_id"] = RANDOM_PACKET_RULE_ID
        payload["active_rule_name"] = RANDOM_PACKET_RULE_NAME
        payload["events"] = [asdict(event) for event in candidate.events]
        with path.open("w", encoding="ascii") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
        return path

    def append_random_packet_trial(self, row: Dict[str, Any]) -> None:
        path = self.output_dir / "random_packet_trials.csv"
        exists = path.exists()
        event_summary = self._rule_event_summary(Path(str(row.get("events_file", "NA")))) if row.get("events_file") not in (None, "NA") else {}
        fields = [
            "trial_index",
            "attack_success",
            "success_scenario_ids",
            "success_scenario_names",
            "best_success_scenario_id",
            "best_success_scenario_name",
            "stl_s1_drone_drone",
            "stl_s2_drone_obstacle",
            "stl_s3_deadlock",
            "stl_s4_delay_timeout",
            "stl_s5_planning_failure",
            "stl_s6_drift",
            "best_objective_value",
            "actual_injection_count",
            "actual_injection_times_s",
            "actual_latency_ms",
            "actual_attacker_node_ids",
            "actual_victim_node_ids",
            "candidate_node_ids",
            "trigger_evidence",
            "status",
            "result_dir",
            "events_file",
            "note",
        ]
        with path.open("a", encoding="ascii", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            if not exists:
                writer.writeheader()
            out = {field: row.get(field, "NA") for field in fields}
            out["candidate_node_ids"] = event_summary.get("triad_node_ids", row.get("candidate_node_ids", "NA"))
            out["trigger_evidence"] = event_summary.get("trigger_evidence", row.get("trigger_evidence", "NA"))
            writer.writerow(out)

    def write_random_packet_summary(self, rows: List[Dict[str, Any]]) -> Path:
        path = self.output_dir / "random_packet_summary.json"
        successes = [row for row in rows if row.get("attack_success") == "yes"]
        scenario_counts: Dict[str, int] = {}
        for row in successes:
            for sid in str(row.get("success_scenario_ids", "")).split(";"):
                sid = sid.strip()
                if sid:
                    scenario_counts[sid] = scenario_counts.get(sid, 0) + 1
        payload = {
            "trial_count": len(rows),
            "success_count": len(successes),
            "success_probability": len(successes) / float(len(rows) or 1),
            "success_scenario_id_counts": scenario_counts,
            "rows_file": str(self.output_dir / "random_packet_trials.csv"),
        }
        with path.open("w", encoding="ascii") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
        return path

    def _rule_event_summary(self, events_file: Path) -> Dict[str, str]:
        triads: List[str] = []
        evidence: List[str] = []
        try:
            with events_file.open("r", encoding="ascii", errors="ignore", newline="") as f:
                for row in csv.DictReader(f):
                    triads.append(str(row.get("triad_node_ids", "NA")))
                    evidence.append(str(row.get("trigger_evidence", "NA")))
        except Exception:
            pass
        return {
            "triad_node_ids": ";".join(triads) if triads else "NA",
            "trigger_evidence": " | ".join(evidence[-3:]) if evidence else "NA",
        }


def event_times_s(candidate: AttackCandidate) -> str:
    return ";".join("{:.6f}".format(event.inject_time_s) for event in candidate.events)


def event_latencies_ms(candidate: AttackCandidate) -> str:
    return ";".join("{:.6f}".format(event.latency_ms) for event in candidate.events)


def event_directions(candidate: AttackCandidate) -> str:
    return ";".join(event.direction for event in candidate.events)


def build_scenario_config(args: argparse.Namespace, baseline: BaselineScenario) -> AttackScenarioConfig:
    scenario_id = int(args.scenario_id)
    scenario_name = SCENARIO_NAMES[scenario_id]
    if scenario_id == 3:
        threshold = float(args.move_threshold_x)
        operator = "<="
    elif scenario_id == 4:
        threshold = float(args.delay_timeout_threshold_s)
        operator = ">="
    elif scenario_id == 6:
        threshold = float(args.drift_baseline_multiplier) * max(0.0, baseline.baseline_stl_value)
        operator = ">="
    else:
        threshold = 0.0
        operator = "<="
    return AttackScenarioConfig(
        scenario_id=scenario_id,
        scenario_name=scenario_name,
        success_operator=operator,
        success_threshold=threshold,
    )


class SwarmTestOrchestrator:
    def __init__(
        self,
        run_headless: Path,
        run_sec: int,
        output_dir: Path,
        execute: bool,
        scenario: AttackScenarioConfig,
        attack_mode: str,
        args: argparse.Namespace,
    ) -> None:
        self.run_headless = run_headless
        self.run_sec = run_sec
        self.output_dir = output_dir
        self.execute = execute
        self.scenario = scenario
        self.attack_mode = attack_mode
        self.args = args
        self.evaluator = STLEvaluator()

    def run_candidate(self, candidate: AttackCandidate, candidate_file: Path, rule_id: Optional[str] = None) -> TrialResult:
        if not self.execute:
            return self._result(candidate, candidate_file, "PLANNED", None, None, False, "NA", "execute=false")

        unsupported = [event.direction for event in candidate.events if event.direction != "tx"]
        if unsupported:
            return self._result(
                candidate,
                candidate_file,
                "UNSUPPORTED_DIRECTION",
                None,
                None,
                False,
                "NA",
                "only tx is executable in the latency proxy for this version",
            )

        if candidate.events:
            first_injection_node_id = candidate.events[0].injection_node_id
        else:
            first_injection_node_id = 0
        trial_root = self.output_dir / "runs"
        if self.attack_mode == "ddc10_screening":
            trial_root = trial_root / str(rule_id or "CONTROL")
        trial_root = trial_root / "k{}_trial{:04d}".format(candidate.injection_count, candidate.trial_index)
        trial_root.mkdir(parents=True, exist_ok=True)
        cmd = [
            "bash",
            str(self.run_headless),
            "1",
            str(self.run_sec),
            str(trial_root),
            "true",
            "false",
            "delay",
            str(first_injection_node_id),
            str(candidate.seed),
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "pure_random",
        ]
        env = os.environ.copy()
        env["INTER_DRONE_ATTACK_CANDIDATE_FILE"] = str(candidate_file)
        self._configure_attack_env(env, candidate, candidate_file, rule_id)
        if self.scenario.scenario_id == 4:
            env["STL_DELAY_TIMEOUT_THRESHOLD_S"] = "{:.6f}".format(self.scenario.success_threshold)

        runner_snapshot = None
        try:
            if self.attack_mode == "ddc10_screening":
                # Keep directory assumptions while isolating Bash from live edits.
                with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", prefix=".run_headless_", suffix=".sh",
                                                 dir=str(self.run_headless.parent), delete=False) as stream:
                    runner_snapshot = Path(stream.name)
                    stream.write(self.run_headless.read_text(encoding="utf-8"))
                cmd[1] = str(runner_snapshot)
            completed = subprocess.run(cmd, cwd=str(self.run_headless.parents[1]), env=env, check=False)
        except Exception as exc:
            return self._result(candidate, candidate_file, "RUN_EXCEPTION", None, None, False, str(trial_root), str(exc))
        finally:
            if runner_snapshot is not None:
                runner_snapshot.unlink(missing_ok=True)

        batch_dir = self._latest_batch(trial_root)
        raw_stl_value, objective_value, attack_success, note = (
            self.evaluator.evaluate_scenario(batch_dir, self.scenario) if batch_dir else (None, None, False, "missing_batch_dir")
        )
        status = "EXECUTED" if completed.returncode == 0 else "RUN_FAILED"
        adaptive = self._dynamic_proxy_outputs(batch_dir)
        return self._result(
            candidate,
            candidate_file,
            status,
            raw_stl_value,
            objective_value,
            attack_success,
            str(batch_dir or trial_root),
            note,
            actual_injection_count=adaptive["actual_injection_count"],
            actual_injection_times_s=adaptive["actual_injection_times_s"],
            actual_latency_ms=adaptive["actual_latency_ms"],
            actual_attacker_node_ids=adaptive["actual_attacker_node_ids"],
            actual_victim_node_ids=adaptive["actual_victim_node_ids"],
            adaptive_events_file=adaptive["adaptive_events_file"],
        )

    def _result(
        self,
        candidate: AttackCandidate,
        candidate_file: Path,
        status: str,
        raw_stl_value: Optional[float],
        objective_value: Optional[float],
        attack_success: bool,
        result_dir: str,
        note: str,
        actual_injection_count: str = "NA",
        actual_injection_times_s: str = "NA",
        actual_latency_ms: str = "NA",
        actual_attacker_node_ids: str = "NA",
        actual_victim_node_ids: str = "NA",
        adaptive_events_file: str = "NA",
    ) -> TrialResult:
        return TrialResult(
            scenario_id=candidate.scenario_id,
            scenario_name=candidate.scenario_name,
            attack_mode=self.attack_mode,
            injection_count=candidate.injection_count,
            trial_index=candidate.trial_index,
            injection_times_s=event_times_s(candidate),
            latency_ms=event_latencies_ms(candidate),
            directions=event_directions(candidate),
            actual_injection_count=actual_injection_count,
            actual_injection_times_s=actual_injection_times_s,
            actual_latency_ms=actual_latency_ms,
            actual_attacker_node_ids=actual_attacker_node_ids,
            actual_victim_node_ids=actual_victim_node_ids,
            status=status,
            raw_stl_value=raw_stl_value,
            objective_value=objective_value,
            attack_success=attack_success,
            candidate_file=str(candidate_file),
            result_dir=result_dir,
            adaptive_events_file=adaptive_events_file,
            note=note,
        )

    def _configure_attack_env(self, env: Dict[str, str], candidate: AttackCandidate, candidate_file: Path, rule_id: Optional[str]) -> None:
        env["ENABLE_ATTACK_PLAN_LATENCY_PROXY"] = "false"
        env["ATTACK_PLAN_FILE"] = ""
        env["ENABLE_ADAPTIVE_REPLAN_LATENCY_PROXY"] = "false"
        env["ADAPTIVE_REPLAN_ATTACK_CONFIG"] = ""
        env["ENABLE_RULE_POOL_LATENCY_PROXY"] = "false"
        env["RULE_POOL_ACTIVE_RULE_ID"] = ""
        env["RULE_POOL_SCREENING_MODE"] = "false"
        env["RULE_POOL_MONITOR_ONLY"] = "false"
        if self.attack_mode == "fixed_schedule":
            env["ENABLE_ATTACK_PLAN_LATENCY_PROXY"] = "true"
            env["ATTACK_PLAN_FILE"] = str(candidate_file)
            return

        if self.attack_mode == "replan_triggered_latency":
            env["ENABLE_ADAPTIVE_REPLAN_LATENCY_PROXY"] = "true"
            env["ADAPTIVE_REPLAN_ATTACK_CONFIG"] = str(candidate_file)
            env["ADAPTIVE_MAX_INJECTIONS_PER_ROUND"] = str(candidate.injection_count)
            env["ADAPTIVE_COOLDOWN_S"] = "{:.6f}".format(float(self.args.adaptive_cooldown_s))
            env["ADAPTIVE_TRIGGER_WINDOW_S"] = "{:.6f}".format(float(self.args.adaptive_trigger_window_s))
            env["ADAPTIVE_FUTURE_HORIZON_S"] = "{:.6f}".format(float(self.args.adaptive_future_horizon_s))
            env["ADAPTIVE_FUTURE_SAMPLE_DT"] = "{:.6f}".format(float(self.args.adaptive_future_sample_dt))
            env["ADAPTIVE_INTERACTION_MARGIN_THRESHOLD"] = "{:.6f}".format(float(self.args.adaptive_interaction_margin_threshold))
            env["ADAPTIVE_INTERACTION_SCORE_THRESHOLD"] = "{:.6f}".format(float(self.args.adaptive_interaction_score_threshold))
            env["ADAPTIVE_LATENCY_FACTOR"] = "{:.6f}".format(float(self.args.adaptive_latency_factor))
            env["ADAPTIVE_LATENCY_MIN_MS"] = "{:.6f}".format(float(self.args.latency_min_ms))
            env["ADAPTIVE_LATENCY_MAX_MS"] = "{:.6f}".format(float(self.args.latency_max_ms))
            env["ADAPTIVE_PERIOD_FALLBACK_MS"] = "{:.6f}".format(float(self.args.adaptive_period_fallback_ms))
            env["ADAPTIVE_SEED"] = str(candidate.seed)
            return

        if self.attack_mode in ("rule_pool_latency", "global_rule_pool_latency", "random_packet_latency_collect", "ddc10_screening"):
            env["ENABLE_RULE_POOL_LATENCY_PROXY"] = "true"
            if self.attack_mode == "ddc10_screening":
                env["RULE_POOL_SCREENING_MODE"] = "true"
                env["RULE_POOL_MONITOR_ONLY"] = "true" if rule_id == "CONTROL" else "false"
            if self.attack_mode == "rule_pool_latency":
                default_rule = self.args.rule_ids.split(",")[0]
            elif self.attack_mode == "global_rule_pool_latency":
                default_rule = self.args.global_rule_ids.split(",")[0]
            elif self.attack_mode == "ddc10_screening":
                default_rule = "DDC10"
            else:
                default_rule = RANDOM_PACKET_RULE_ID
            env["RULE_POOL_ACTIVE_RULE_ID"] = str(rule_id or default_rule).strip().upper()
            if rule_id == "CONTROL" and self.attack_mode == "ddc10_screening":
                env["RULE_POOL_ACTIVE_RULE_ID"] = "DDC10"
            if self.attack_mode == "random_packet_latency_collect":
                env["RULE_POOL_MAX_INJECTIONS_PER_ROUND"] = str(self.args.random_max_injections_per_round)
            else:
                env["RULE_POOL_MAX_INJECTIONS_PER_ROUND"] = str(candidate.injection_count)
            env["RULE_POOL_COOLDOWN_S"] = "{:.6f}".format(float(self.args.rule_cooldown_s))
            env["RULE_POOL_HISTORY_WINDOW_S"] = "{:.6f}".format(float(self.args.rule_history_window_s))
            env["RULE_POOL_TRIGGER_WINDOW_S"] = "{:.6f}".format(float(self.args.rule_trigger_window_s))
            env["RULE_POOL_FUTURE_HORIZON_S"] = "{:.6f}".format(float(self.args.rule_future_horizon_s))
            env["RULE_POOL_FUTURE_SAMPLE_DT"] = "{:.6f}".format(float(self.args.rule_future_sample_dt))
            env["RULE_POOL_HEADING_ALIGNED_DEG"] = "{:.6f}".format(float(self.args.rule_heading_aligned_deg))
            env["RULE_POOL_FUTURE_COLLISION_MARGIN"] = "{:.6f}".format(float(self.args.rule_future_collision_margin))
            env["RULE_POOL_LATENCY_EPSILON_MS"] = "{:.6f}".format(float(self.args.rule_latency_epsilon_ms))
            env["RULE_POOL_LATENCY_MIN_MS"] = "{:.6f}".format(float(self.args.rule_latency_min_ms))
            env["RULE_POOL_LATENCY_MAX_MS"] = "{:.6f}".format(float(self.args.rule_latency_max_ms))
            env["RULE_POOL_PROTOCOL_TIMEOUT_MS"] = "{:.6f}".format(float(self.args.protocol_timeout_ms))
            env["RULE_POOL_MAX_LATENCY_RATIO"] = "{:.6f}".format(float(self.args.max_latency_ratio))
            env["RULE_POOL_PERIOD_FALLBACK_MS"] = "{:.6f}".format(float(self.args.rule_period_fallback_ms))
            if self.attack_mode == "global_rule_pool_latency":
                env["STL_DELAY_TIMEOUT_THRESHOLD_S"] = "{:.6f}".format(float(self.args.global_delay_threshold_s))
            if self.attack_mode == "random_packet_latency_collect":
                env["STL_DELAY_TIMEOUT_THRESHOLD_S"] = "{:.6f}".format(float(self.args.global_delay_threshold_s))
                env["RANDOM_PACKET_SEED"] = str(candidate.seed)
                env["RANDOM_PACKET_MODE"] = str(self.args.random_mode)
                env["RANDOM_PACKET_PROBABILITY"] = "{:.6f}".format(float(self.args.random_packet_probability))
                env["RANDOM_PACKET_TARGET_START_S"] = "{:.6f}".format(float(self.args.random_target_start_s))
                env["RANDOM_PACKET_TARGET_END_S"] = "{:.6f}".format(float(self.args.random_target_end_s))
                env["RANDOM_PACKET_TARGET_DRONE_ID"] = str(int(self.args.random_target_drone_id))
            return

        raise ValueError("unsupported attack_mode={}".format(self.attack_mode))

    def _dynamic_proxy_outputs(self, batch_dir: Optional[Path]) -> Dict[str, str]:
        empty = {
            "actual_injection_count": "NA",
            "actual_injection_times_s": "NA",
            "actual_latency_ms": "NA",
            "actual_attacker_node_ids": "NA",
            "actual_victim_node_ids": "NA",
            "adaptive_events_file": "NA",
        }
        if batch_dir is None:
            return empty
        if self.attack_mode == "replan_triggered_latency":
            meta_files = sorted(batch_dir.glob("run*/adaptive_replan_latency_meta.json"))
            event_files = sorted(batch_dir.glob("run*/adaptive_replan_latency_events.csv"))
        elif self.attack_mode in ("rule_pool_latency", "global_rule_pool_latency", "random_packet_latency_collect", "ddc10_screening"):
            meta_files = sorted(batch_dir.glob("run*/rule_pool_latency_meta.json"))
            event_files = sorted(batch_dir.glob("run*/rule_pool_latency_events.csv"))
        else:
            return empty
        if meta_files:
            meta = read_json(meta_files[-1]) or {}
            empty["actual_injection_count"] = str(meta.get("actual_injection_count", "NA"))
        if event_files:
            empty["adaptive_events_file"] = str(event_files[-1])
            actual = self._read_adaptive_event_summary(event_files[-1])
            empty.update(actual)
            if empty["actual_injection_count"] == "NA" and actual["actual_injection_times_s"] != "NA":
                empty["actual_injection_count"] = str(len(actual["actual_injection_times_s"].split(";")))
        return empty

    def _read_adaptive_event_summary(self, events_file: Path) -> Dict[str, str]:
        times: List[str] = []
        latencies: List[str] = []
        attackers: List[str] = []
        victims: List[str] = []
        try:
            with events_file.open("r", encoding="ascii", errors="ignore", newline="") as f:
                for row in csv.DictReader(f):
                    times.append(str(row.get("inject_time_s", "NA")))
                    latencies.append(str(row.get("latency_ms", "NA")))
                    attackers.append(str(row.get("attacker_drone_id", "NA")))
                    victims.append(str(row.get("victim_drone_id", "NA")))
        except Exception:
            pass
        return {
            "actual_injection_times_s": ";".join(times) if times else "NA",
            "actual_latency_ms": ";".join(latencies) if latencies else "NA",
            "actual_attacker_node_ids": ";".join(attackers) if attackers else "NA",
            "actual_victim_node_ids": ";".join(victims) if victims else "NA",
        }

    def _latest_batch(self, root: Path) -> Optional[Path]:
        batches = [p for p in root.iterdir() if p.is_dir()]
        if not batches:
            return None
        return sorted(batches, key=lambda p: p.stat().st_mtime)[-1]


class InterDroneCollisionAttackSearch:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        if args.scenario_id not in SCENARIO_NAMES:
            raise ValueError("unsupported scenario_id={}, expected one of {}".format(args.scenario_id, sorted(SCENARIO_NAMES)))
        baseline_loader = BaselineScenarioLoader(Path(args.baseline_csv).resolve())
        baseline = baseline_loader.load_scenario(args.scenario_id)
        self.baseline_values = baseline_loader.load_all_stl_values()
        self.scenario = build_scenario_config(args, baseline)
        if args.output_dir:
            self.output_dir = Path(args.output_dir).resolve()
        elif args.attack_mode == "random_packet_latency_collect":
            self.output_dir = default_random_packet_results_dir()
        elif args.attack_mode == "global_rule_pool_latency":
            self.output_dir = default_global_rule_pool_results_dir()
        elif args.attack_mode == "ddc10_screening":
            self.output_dir = workspace_results_root() / "latency_attack_search" / time.strftime("ddc10_screening_%Y%m%d_%H%M%S")
        else:
            self.output_dir = default_scenario_results_dir(self.scenario.scenario_id)
        constraints = LatencyConstraintModel(
            max_vel_mps=args.max_vel_mps,
            max_acc_mps2=args.max_acc_mps2,
            physical_offset_limit_m=args.physical_offset_limit_m,
            protocol_timeout_ms=args.protocol_timeout_ms,
            max_latency_ratio=args.max_latency_ratio,
            min_event_spacing_s=args.min_event_spacing_s,
        )
        self.baseline = baseline
        self.optimizer = LatencyPerturbationOptimizer(
            baseline=baseline,
            constraints=constraints,
            seed=args.seed,
            time_jitter_s=args.time_jitter_s,
            latency_range_ms=(args.latency_min_ms, args.latency_max_ms),
            duration_range_s=(args.duration_min_s, args.duration_max_s),
            directions=args.directions.split(","),
        )
        self.constraints = constraints
        self.writer = AttackTupleWriter(self.output_dir)
        run_sec = args.run_sec
        if self.scenario.scenario_id == 4:
            min_run_sec = int(math.ceil(self.scenario.success_threshold + args.delay_timeout_observation_buffer_s))
            run_sec = max(run_sec, min_run_sec)
        if args.attack_mode == "global_rule_pool_latency":
            min_run_sec = int(math.ceil(float(args.global_delay_threshold_s) + args.delay_timeout_observation_buffer_s))
            run_sec = max(run_sec, min_run_sec)
        if args.attack_mode == "random_packet_latency_collect":
            min_run_sec = int(math.ceil(float(args.global_delay_threshold_s) + args.delay_timeout_observation_buffer_s))
            run_sec = max(run_sec, min_run_sec)
        self.orchestrator = SwarmTestOrchestrator(
            run_headless=Path(args.run_headless).resolve(),
            run_sec=run_sec,
            output_dir=self.output_dir,
            execute=args.execute,
            scenario=self.scenario,
            attack_mode=args.attack_mode,
            args=args,
        )

    def screening_candidate(self, block_index: int, control: bool) -> AttackCandidate:
        return AttackCandidate(
            scenario_id=1,
            scenario_name=SCENARIO_NAMES[1],
            injection_count=0 if control else 1,
            trial_index=block_index,
            seed=self.args.seed + block_index,
            events=[],
        )

    def run(self) -> int:
        if self.args.attack_mode == "ddc10_screening":
            from ddc10_rule_screening import DDC10RuleScreening
            return DDC10RuleScreening(self).run()
        if self.args.attack_mode == "rule_pool_latency":
            return self.run_rule_pool()
        if self.args.attack_mode == "global_rule_pool_latency":
            return self.run_global_rule_pool()
        if self.args.attack_mode == "random_packet_latency_collect":
            return self.run_random_packet_collect()

        best_result: Optional[TrialResult] = None
        best_candidate: Optional[AttackCandidate] = None
        print(
            "search_config scenario={} name={} attack_mode={} condition=raw_stl_value{}{} run_sec={} output_dir={}".format(
                self.scenario.scenario_id,
                self.scenario.scenario_name,
                self.args.attack_mode,
                self.scenario.success_operator,
                self.scenario.success_threshold,
                self.orchestrator.run_sec,
                self.output_dir,
            ),
            flush=True,
        )

        for injection_count in range(1, self.args.max_injection_count + 1):
            for trial_index in range(1, self.args.max_trials_per_injection_count + 1):
                candidate = self.optimizer.propose(injection_count, trial_index)
                ok, reason = self.constraints.validate(candidate)
                candidate_file = self.writer.write_candidate(candidate)
                if not ok:
                    result = TrialResult(
                        scenario_id=candidate.scenario_id,
                        scenario_name=candidate.scenario_name,
                        attack_mode=self.args.attack_mode,
                        injection_count=injection_count,
                        trial_index=trial_index,
                        injection_times_s=event_times_s(candidate),
                        latency_ms=event_latencies_ms(candidate),
                        directions=event_directions(candidate),
                        actual_injection_count="NA",
                        actual_injection_times_s="NA",
                        actual_latency_ms="NA",
                        actual_attacker_node_ids="NA",
                        actual_victim_node_ids="NA",
                        status="INVALID_CONSTRAINT",
                        raw_stl_value=None,
                        objective_value=None,
                        attack_success=False,
                        candidate_file=str(candidate_file),
                        result_dir="NA",
                        adaptive_events_file="NA",
                        note=reason,
                    )
                else:
                    result = self.orchestrator.run_candidate(candidate, candidate_file)

                self.writer.append_trial(result)
                self.optimizer.observe(candidate, result.objective_value)
                if result.objective_value is not None and (
                    best_result is None or best_result.objective_value is None or result.objective_value < best_result.objective_value
                ):
                    best_result = result
                    best_candidate = candidate
                    self.writer.write_best(result, candidate)

                print(
                    "scenario={} k={} trial={} status={} objective={} success={}".format(
                        self.scenario.scenario_id,
                        injection_count,
                        trial_index,
                        result.status,
                        "NA" if result.objective_value is None else "{:.6f}".format(result.objective_value),
                        "yes" if result.attack_success else "no",
                    ),
                    flush=True,
                )

                if result.attack_success and self.args.stop_on_success:
                    self.writer.write_best(result, candidate)
                    return 0

        if best_result is not None and best_candidate is not None:
            self.writer.write_best(best_result, best_candidate)
        return 0

    def run_rule_pool(self) -> int:
        if self.scenario.scenario_id != 1:
            raise ValueError("rule_pool_latency is only supported for scenario_id=1 drone_drone_collision")
        rule_ids = self._parse_rule_ids(self.args.rule_ids)
        rows: List[Dict[str, Any]] = []
        print(
            "rule_pool_config scenario={} name={} rules={} trials_per_rule={} max_injections_per_round={} run_sec={} output_dir={}".format(
                self.scenario.scenario_id,
                self.scenario.scenario_name,
                ",".join(rule_ids),
                self.args.trials_per_rule,
                self.args.rule_max_injections_per_round,
                self.orchestrator.run_sec,
                self.output_dir,
            ),
            flush=True,
        )

        for rule_id in rule_ids:
            for trial_index in range(1, self.args.trials_per_rule + 1):
                candidate = self.optimizer.propose(self.args.rule_max_injections_per_round, trial_index)
                candidate.injection_count = self.args.rule_max_injections_per_round
                ok, reason = self.constraints.validate(candidate)
                candidate_file = self.writer.write_rule_pool_candidate(rule_id, candidate)
                if not ok:
                    result = TrialResult(
                        scenario_id=candidate.scenario_id,
                        scenario_name=candidate.scenario_name,
                        attack_mode=self.args.attack_mode,
                        injection_count=candidate.injection_count,
                        trial_index=trial_index,
                        injection_times_s=event_times_s(candidate),
                        latency_ms=event_latencies_ms(candidate),
                        directions=event_directions(candidate),
                        actual_injection_count="NA",
                        actual_injection_times_s="NA",
                        actual_latency_ms="NA",
                        actual_attacker_node_ids="NA",
                        actual_victim_node_ids="NA",
                        status="INVALID_CONSTRAINT",
                        raw_stl_value=None,
                        objective_value=None,
                        attack_success=False,
                        candidate_file=str(candidate_file),
                        result_dir="NA",
                        adaptive_events_file="NA",
                        note=reason,
                    )
                else:
                    result = self.orchestrator.run_candidate(candidate, candidate_file, rule_id=rule_id)

                self.writer.append_trial(result)
                self.writer.append_rule_pool_trial(rule_id, result)
                row = {
                    "rule_id": rule_id,
                    "rule_name": RULE_POOL_NAMES.get(rule_id, "unknown"),
                    "trial_index": trial_index,
                    "attack_success": result.attack_success,
                    "raw_stl_value": result.raw_stl_value,
                    "objective_value": result.objective_value,
                    "actual_injection_count": result.actual_injection_count,
                    "events_file": result.adaptive_events_file,
                }
                rows.append(row)
                self.writer.write_rule_pool_summary(rows)
                self.writer.write_rule_pool_best(rows)

                print(
                    "rule={} trial={} status={} injections={} objective={} success={}".format(
                        rule_id,
                        trial_index,
                        result.status,
                        result.actual_injection_count,
                        "NA" if result.objective_value is None else "{:.6f}".format(result.objective_value),
                        "yes" if result.attack_success else "no",
                    ),
                    flush=True,
                )

        self.writer.write_rule_pool_summary(rows)
        self.writer.write_rule_pool_best(rows)
        return 0

    def _parse_rule_ids(self, raw: str) -> List[str]:
        out = []
        for item in str(raw or "").split(","):
            rule_id = item.strip().upper()
            if not rule_id:
                continue
            if rule_id not in RULE_POOL_NAMES:
                raise ValueError("unsupported rule_id={}, expected one of {}".format(rule_id, sorted(RULE_POOL_NAMES)))
            out.append(rule_id)
        return out or list(RULE_POOL_NAMES)

    def run_global_rule_pool(self) -> int:
        rule_ids = self._parse_global_rule_ids(self.args.global_rule_ids)
        rows: List[Dict[str, Any]] = []
        print(
            "global_rule_pool_config rules={} trials_per_rule={} max_injections_per_round={} success_target_scenario_id={} delay_success_threshold_s={} run_sec={} output_dir={}".format(
                ",".join(rule_ids),
                self.args.global_trials_per_rule,
                self.args.global_rule_max_injections_per_round,
                "ALL" if self.args.global_success_scenario_id == 0 else self.args.global_success_scenario_id,
                self.args.global_delay_threshold_s,
                self.orchestrator.run_sec,
                self.output_dir,
            ),
            flush=True,
        )

        for rule_id in rule_ids:
            for trial_index in range(1, self.args.global_trials_per_rule + 1):
                candidate = self.optimizer.propose(self.args.global_rule_max_injections_per_round, trial_index)
                candidate.injection_count = self.args.global_rule_max_injections_per_round
                ok, reason = self.constraints.validate(candidate)
                candidate_file = self.writer.write_global_rule_pool_candidate(rule_id, candidate)
                if not ok:
                    result = TrialResult(
                        scenario_id=candidate.scenario_id,
                        scenario_name=candidate.scenario_name,
                        attack_mode=self.args.attack_mode,
                        injection_count=candidate.injection_count,
                        trial_index=trial_index,
                        injection_times_s=event_times_s(candidate),
                        latency_ms=event_latencies_ms(candidate),
                        directions=event_directions(candidate),
                        actual_injection_count="NA",
                        actual_injection_times_s="NA",
                        actual_latency_ms="NA",
                        actual_attacker_node_ids="NA",
                        actual_victim_node_ids="NA",
                        status="INVALID_CONSTRAINT",
                        raw_stl_value=None,
                        objective_value=None,
                        attack_success=False,
                        candidate_file=str(candidate_file),
                        result_dir="NA",
                        adaptive_events_file="NA",
                        note=reason,
                    )
                    eval_row = self._empty_global_eval(self.args.global_success_scenario_id)
                else:
                    result = self.orchestrator.run_candidate(candidate, candidate_file, rule_id=rule_id)
                    eval_row = (
                        self._evaluate_global_success(
                            Path(result.result_dir),
                            result.actual_injection_count,
                            self.args.global_success_scenario_id,
                        )
                        if result.result_dir != "NA"
                        else self._empty_global_eval(self.args.global_success_scenario_id)
                    )

                row = {
                    "rule_id": rule_id,
                    "rule_name": GLOBAL_RULE_POOL_NAMES.get(rule_id, "unknown"),
                    "trial_index": trial_index,
                    "attack_success": "yes" if eval_row["attack_success"] else "no",
                    "success_target_scenario_id": eval_row["success_target_scenario_id"],
                    "success_scenario_ids": eval_row["success_scenario_ids"],
                    "success_scenario_names": eval_row["success_scenario_names"],
                    "best_success_scenario_id": eval_row["best_success_scenario_id"],
                    "best_success_scenario_name": eval_row["best_success_scenario_name"],
                    "stl_s1_drone_drone": eval_row["stl_values"].get(1, "NA"),
                    "stl_s2_drone_obstacle": eval_row["stl_values"].get(2, "NA"),
                    "stl_s3_deadlock": eval_row["stl_values"].get(3, "NA"),
                    "stl_s4_delay_timeout": eval_row["stl_values"].get(4, "NA"),
                    "stl_s5_planning_failure": eval_row["stl_values"].get(5, "NA"),
                    "stl_s6_drift": eval_row["stl_values"].get(6, "NA"),
                    "best_objective_value": eval_row["best_objective_value"],
                    "actual_injection_count": result.actual_injection_count,
                    "actual_injection_times_s": result.actual_injection_times_s,
                    "actual_latency_ms": result.actual_latency_ms,
                    "actual_attacker_node_ids": result.actual_attacker_node_ids,
                    "actual_victim_node_ids": result.actual_victim_node_ids,
                    "status": result.status,
                    "result_dir": result.result_dir,
                    "events_file": result.adaptive_events_file,
                    "note": result.note,
                }
                rows.append(row)
                self.writer.append_global_rule_pool_trial(row)
                self.writer.write_global_rule_pool_summary(rows)
                self.writer.write_global_rule_pool_best(rows)

                print(
                    "global_rule={} trial={} status={} injections={} success={} success_scenarios={} best_objective={}".format(
                        rule_id,
                        trial_index,
                        result.status,
                        result.actual_injection_count,
                        row["attack_success"],
                        row["success_scenario_ids"] or "NA",
                        row["best_objective_value"],
                    ),
                    flush=True,
                )

        self.writer.write_global_rule_pool_summary(rows)
        self.writer.write_global_rule_pool_best(rows)
        return 0

    def run_random_packet_collect(self) -> int:
        rows: List[Dict[str, Any]] = []
        print(
            "random_packet_collect_config trials={} max_injections_per_round={} random_mode={} packet_probability={} target_window_s={:.3f},{:.3f} delay_success_threshold_s={} strict_deadlock=stl<={} after_start_s={} run_sec={} output_dir={}".format(
                self.args.random_trials,
                self.args.random_max_injections_per_round,
                self.args.random_mode,
                self.args.random_packet_probability,
                self.args.random_target_start_s,
                self.args.random_target_end_s,
                self.args.global_delay_threshold_s,
                self.args.deadlock_strict_stl_threshold,
                self.args.deadlock_strict_min_after_start_s,
                self.orchestrator.run_sec,
                self.output_dir,
            ),
            flush=True,
        )

        for trial_index in range(1, self.args.random_trials + 1):
            candidate_seed_event_count = max(1, int(self.args.random_max_injections_per_round))
            candidate = self.optimizer.propose(candidate_seed_event_count, trial_index)
            candidate.injection_count = self.args.random_max_injections_per_round
            candidate_file = self.writer.write_random_packet_candidate(candidate)
            result = self.orchestrator.run_candidate(candidate, candidate_file, rule_id=RANDOM_PACKET_RULE_ID)
            eval_row = (
                self._evaluate_global_success(Path(result.result_dir), result.actual_injection_count)
                if result.result_dir != "NA"
                else self._empty_global_eval()
            )
            row = {
                "trial_index": trial_index,
                "attack_success": "yes" if eval_row["attack_success"] else "no",
                "success_scenario_ids": eval_row["success_scenario_ids"],
                "success_scenario_names": eval_row["success_scenario_names"],
                "best_success_scenario_id": eval_row["best_success_scenario_id"],
                "best_success_scenario_name": eval_row["best_success_scenario_name"],
                "stl_s1_drone_drone": eval_row["stl_values"].get(1, "NA"),
                "stl_s2_drone_obstacle": eval_row["stl_values"].get(2, "NA"),
                "stl_s3_deadlock": eval_row["stl_values"].get(3, "NA"),
                "stl_s4_delay_timeout": eval_row["stl_values"].get(4, "NA"),
                "stl_s5_planning_failure": eval_row["stl_values"].get(5, "NA"),
                "stl_s6_drift": eval_row["stl_values"].get(6, "NA"),
                "best_objective_value": eval_row["best_objective_value"],
                "actual_injection_count": result.actual_injection_count,
                "actual_injection_times_s": result.actual_injection_times_s,
                "actual_latency_ms": result.actual_latency_ms,
                "actual_attacker_node_ids": result.actual_attacker_node_ids,
                "actual_victim_node_ids": result.actual_victim_node_ids,
                "status": result.status,
                "result_dir": result.result_dir,
                "events_file": result.adaptive_events_file,
                "note": result.note,
            }
            rows.append(row)
            self.writer.append_random_packet_trial(row)
            self.writer.write_random_packet_summary(rows)
            print(
                "random_packet trial={} status={} injections={} success={} success_scenarios={} best_objective={}".format(
                    trial_index,
                    result.status,
                    result.actual_injection_count,
                    row["attack_success"],
                    row["success_scenario_ids"] or "NA",
                    row["best_objective_value"],
                ),
                flush=True,
            )

        self.writer.write_random_packet_summary(rows)
        return 0

    def _empty_global_eval(self, target_scenario_id: int = 0) -> Dict[str, Any]:
        return {
            "attack_success": False,
            "success_target_scenario_id": "ALL" if target_scenario_id == 0 else str(target_scenario_id),
            "success_scenario_ids": "",
            "success_scenario_names": "",
            "best_success_scenario_id": "NA",
            "best_success_scenario_name": "NA",
            "best_objective_value": "NA",
            "stl_values": {},
        }

    def _evaluate_global_success(
        self,
        batch_dir: Path,
        actual_injection_count: Any = 0,
        target_scenario_id: int = 0,
    ) -> Dict[str, Any]:
        summary = self.orchestrator.evaluator._latest_run_summary(batch_dir)
        if summary is None:
            return self._empty_global_eval(target_scenario_id)
        values: Dict[int, float] = {}
        for row in summary.get("rows", []):
            sid = safe_int(row.get("scenario_id"), -1)
            if sid in SCENARIO_NAMES:
                value = safe_float(row.get("stl_value"), None)
                if value is not None:
                    values[sid] = value

        successes: List[int] = []
        objectives: Dict[int, float] = {}
        for sid, value in values.items():
            if sid == 3:
                is_deadlock, deadlock_objective = self._is_strict_deadlock_success(summary, value)
                objectives[sid] = deadlock_objective
                if is_deadlock:
                    successes.append(sid)
                continue
            scenario = self._global_scenario_config(sid)
            objective = scenario.objective_value(value)
            objectives[sid] = objective
            if scenario.is_success(value):
                successes.append(sid)
        selected_successes = successes if target_scenario_id == 0 else [sid for sid in successes if sid == target_scenario_id]
        injected = safe_int(actual_injection_count, 0) > 0
        attack_success = injected and bool(selected_successes)
        best_sid = min(selected_successes, key=lambda sid: objectives[sid], default=None) if injected else None
        return {
            "attack_success": attack_success,
            "success_target_scenario_id": "ALL" if target_scenario_id == 0 else str(target_scenario_id),
            "success_scenario_ids": ";".join(str(sid) for sid in successes),
            "success_scenario_names": ";".join(SCENARIO_NAMES[sid] for sid in successes),
            "best_success_scenario_id": "NA" if best_sid is None else str(best_sid),
            "best_success_scenario_name": "NA" if best_sid is None else SCENARIO_NAMES[best_sid],
            "best_objective_value": "NA" if best_sid is None else "{:.6f}".format(objectives[best_sid]),
            "stl_values": {sid: "{:.6f}".format(value) for sid, value in values.items()},
        }

    def _is_strict_deadlock_success(self, summary: Dict[str, Any], value: float) -> Tuple[bool, float]:
        threshold = float(self.args.deadlock_strict_stl_threshold)
        objective = value - threshold
        if value > threshold:
            return False, objective
        task_start = safe_float(summary.get("task_start_ros_time"), None)
        details = summary.get("details", {}) if isinstance(summary.get("details", {}), dict) else {}
        deadlock_detail = details.get("3", {}) if isinstance(details.get("3", {}), dict) else {}
        evidence_time = safe_float(deadlock_detail.get("evidence_time"), None)
        victim_fsm_state = str(deadlock_detail.get("victim_fsm_state", "NA"))
        if task_start is None or evidence_time is None:
            return False, objective
        if evidence_time - task_start < float(self.args.deadlock_strict_min_after_start_s):
            return False, objective
        active_states = set(str(self.args.deadlock_strict_active_states).split(","))
        if victim_fsm_state not in active_states:
            return False, objective
        return True, objective

    def _global_scenario_config(self, scenario_id: int) -> AttackScenarioConfig:
        baseline_value = self.baseline_values.get(scenario_id, 0.0)
        if scenario_id == 3:
            threshold = float(self.args.deadlock_strict_stl_threshold)
            operator = "<="
        elif scenario_id == 4:
            threshold = float(self.args.global_delay_threshold_s)
            operator = ">="
        elif scenario_id == 6:
            threshold = float(self.args.drift_baseline_multiplier) * max(0.0, baseline_value)
            operator = ">="
        else:
            threshold = 0.0
            operator = "<="
        return AttackScenarioConfig(
            scenario_id=scenario_id,
            scenario_name=SCENARIO_NAMES[scenario_id],
            success_operator=operator,
            success_threshold=threshold,
        )

    def _parse_global_rule_ids(self, raw: str) -> List[str]:
        if str(raw or "").strip().upper() in ("", "ALL"):
            return list(GLOBAL_RULE_POOL_DEFAULT_IDS)
        out = []
        for item in str(raw or "").split(","):
            rule_id = item.strip().upper()
            if not rule_id:
                continue
            if rule_id not in GLOBAL_RULE_POOL_NAMES:
                raise ValueError("unsupported global_rule_id={}, expected one of {}".format(rule_id, sorted(GLOBAL_RULE_POOL_NAMES)))
            out.append(rule_id)
        return out or list(GLOBAL_RULE_POOL_DEFAULT_IDS)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Robustness-guided search for swarm latency attacks",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--baseline-csv", required=True)
    ap.add_argument("--scenario-id", type=int, default=1, choices=sorted(SCENARIO_NAMES))
    ap.add_argument("--output-dir", default=None)
    ap.add_argument("--run-headless", default="src/run_headless.sh")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument(
        "--attack-mode",
        choices=("fixed_schedule", "replan_triggered_latency", "rule_pool_latency", "global_rule_pool_latency", "random_packet_latency_collect", "ddc10_screening"),
        default="fixed_schedule",
        help="fixed_schedule uses planned timestamps; replan_triggered_latency injects online on replanning interaction; rule_pool_latency evaluates R1-R6 for scenario 1; global_rule_pool_latency evaluates OR/DD/DO/DL/DE/PF/DR rules against all six goals; random_packet_latency_collect injects at random packet-aligned times and collects labels/features.",
    )
    ap.add_argument("--run-sec", type=int, default=60)
    ap.add_argument("--seed", type=int, default=20260907)
    ap.add_argument("--max-injection-count", type=int, default=5)
    ap.add_argument("--max-trials-per-injection-count", type=int, default=200)
    ap.add_argument("--stop-on-success", dest="stop_on_success", action="store_true", default=True)
    ap.add_argument("--no-stop-on-success", dest="stop_on_success", action="store_false")
    ap.add_argument("--time-jitter-s", type=float, default=3.0)
    ap.add_argument("--latency-min-ms", type=float, default=80.0)
    ap.add_argument("--latency-max-ms", type=float, default=250.0)
    ap.add_argument("--duration-min-s", type=float, default=0.8)
    ap.add_argument("--duration-max-s", type=float, default=3.0)
    ap.add_argument("--directions", default="tx")
    ap.add_argument("--max-vel-mps", type=float, default=1.5)
    ap.add_argument("--max-acc-mps2", type=float, default=6.0)
    ap.add_argument("--physical-offset-limit-m", type=float, default=0.6)
    ap.add_argument("--protocol-timeout-ms", type=float, default=1000.0)
    ap.add_argument("--max-latency-ratio", type=float, default=0.8)
    ap.add_argument("--min-event-spacing-s", type=float, default=0.5)
    ap.add_argument("--move-threshold-x", type=float, default=0.2)
    ap.add_argument("--delay-timeout-threshold-s", type=float, default=60.0)
    ap.add_argument("--delay-timeout-observation-buffer-s", type=float, default=5.0)
    ap.add_argument("--drift-baseline-multiplier", type=float, default=2.0)
    ap.add_argument("--adaptive-cooldown-s", type=float, default=2.0)
    ap.add_argument("--adaptive-trigger-window-s", type=float, default=1.0)
    ap.add_argument("--adaptive-future-horizon-s", type=float, default=3.0)
    ap.add_argument("--adaptive-future-sample-dt", type=float, default=0.1)
    ap.add_argument("--adaptive-interaction-margin-threshold", type=float, default=0.25)
    ap.add_argument("--adaptive-interaction-score-threshold", type=float, default=0.6)
    ap.add_argument("--adaptive-latency-factor", type=float, default=1.2)
    ap.add_argument("--adaptive-period-fallback-ms", type=float, default=120.0)
    ap.add_argument("--rule-ids", default="R1,R2,R3,R4,R5,R6")
    ap.add_argument("--trials-per-rule", type=int, default=100)
    ap.add_argument("--rule-max-injections-per-round", type=int, default=5)
    ap.add_argument("--rule-cooldown-s", type=float, default=1.0)
    ap.add_argument("--rule-history-window-s", type=float, default=5.0)
    ap.add_argument("--rule-trigger-window-s", type=float, default=1.0)
    ap.add_argument("--rule-future-horizon-s", type=float, default=3.0)
    ap.add_argument("--rule-future-sample-dt", type=float, default=0.1)
    ap.add_argument("--rule-heading-aligned-deg", type=float, default=20.0)
    ap.add_argument("--rule-future-collision-margin", type=float, default=0.0)
    ap.add_argument("--rule-latency-epsilon-ms", type=float, default=10.0)
    ap.add_argument("--rule-latency-min-ms", type=float, default=40.0)
    ap.add_argument(
        "--rule-latency-max-ms",
        type=float,
        default=0.0,
        help="optional extra cap for rule-pool latency; 0 uses only the protocol timeout budget",
    )
    ap.add_argument("--rule-period-fallback-ms", type=float, default=120.0)
    ap.add_argument("--screening-rule-ids", default=",".join(DDC10_RULE_NAMES))
    ap.add_argument("--screening-or-groups", default="",
                    help="optional OR treatment arms, comma-separated groups such as DDC10_4+DDC10_6")
    ap.add_argument("--screening-trials-per-rule", type=int, default=100,
                    help="blocks containing one no-injection control and one single-injection trial per selected treatment arm")
    ap.add_argument("--global-rule-ids", default="ALL")
    ap.add_argument("--global-trials-per-rule", type=int, default=100)
    ap.add_argument("--global-rule-max-injections-per-round", type=int, default=5)
    ap.add_argument(
        "--global-success-scenario-id",
        type=int,
        choices=(0, 1, 2, 3, 4, 5, 6),
        default=0,
        help="Scenario required for global-rule attack_success; 0 means any of the six. A successful label also requires at least one actual injection.",
    )
    ap.add_argument("--global-delay-threshold-s", type=float, default=60.0)
    ap.add_argument("--random-trials", type=int, default=200)
    ap.add_argument("--random-max-injections-per-round", type=int, default=0, help="0 means no hard cap; cooldown still limits injection density.")
    ap.add_argument("--random-mode", choices=("target_offsets", "probability"), default="probability")
    ap.add_argument("--random-packet-probability", type=float, default=0.02)
    ap.add_argument("--random-target-start-s", type=float, default=3.0)
    ap.add_argument("--random-target-end-s", type=float, default=60.0)
    ap.add_argument("--random-target-drone-id", type=int, default=-1)
    ap.add_argument("--deadlock-strict-stl-threshold", type=float, default=0.0)
    ap.add_argument("--deadlock-strict-min-after-start-s", type=float, default=30.0)
    ap.add_argument("--deadlock-strict-active-states", default="EXEC_TRAJ,REPLAN_TRAJ,EMERGENCY_STOP")
    return ap.parse_args(argv)


def main() -> int:
    args = parse_args()
    started = time.time()
    code = InterDroneCollisionAttackSearch(args).run()
    print("finished in {:.1f}s".format(time.time() - started), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
