#!/usr/bin/env python3
"""Randomized blocks for single-rule, single-injection DDC10 screening."""
from __future__ import annotations

import csv
import json
import math
import random
import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
from itertools import combinations
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple

from ddc10_rules import DDC10_RULE_IDS, DDC10_RULES, ddc10_or_components, ddc10_or_rule_id
from screening_runtime_audit import audit_runtime


def number(value: Any) -> Optional[float]:
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def fraction(numerator: int, denominator: int) -> Any:
    return numerator / denominator if denominator else "NA"


def json_object(text: str) -> Dict[str, Any]:
    try:
        value = json.loads(text)
        return value if isinstance(value, dict) else {}
    except (ValueError, TypeError):
        return {}


def wilson(successes: int, total: int) -> Tuple[Any, Any]:
    if not total:
        return "NA", "NA"
    z = 1.959963984540054
    p = successes / total
    denominator = 1 + z * z / total
    center = (p + z * z / (2 * total)) / denominator
    half = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / denominator
    return max(0.0, center - half), min(1.0, center + half)


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    if not rows:
        return
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


class DDC10RuleScreening:
    def __init__(self, search: Any) -> None:
        self.search = search
        self.args = search.args
        self.output = search.output_dir
        self.rule_ids = [item.strip().upper() for item in self.args.screening_rule_ids.split(",") if item.strip()]
        if not self.rule_ids or len(set(self.rule_ids)) != len(self.rule_ids):
            raise ValueError("screening-rule-ids must contain distinct DDC10_1 through DDC10_10 IDs")
        if any(item not in DDC10_RULES for item in self.rule_ids):
            raise ValueError("unknown screening rule ID")
        self.or_groups = []
        for raw_group in getattr(self.args, "screening_or_groups", "").split(","):
            if not raw_group.strip():
                continue
            components = [item.strip().upper() for item in raw_group.split("+") if item.strip()]
            if len(components) < 2 or len(set(components)) != len(components):
                raise ValueError("each screening OR group must contain at least two distinct IDs separated by +")
            if any(item not in DDC10_RULES for item in components):
                raise ValueError("unknown rule ID in screening OR group")
            components = sorted(components, key=DDC10_RULE_IDS.index)
            group_id = ddc10_or_rule_id(components)
            if group_id in self.or_groups:
                raise ValueError("duplicate screening OR group: {}".format(group_id))
            self.or_groups.append(group_id)
        self.arm_ids = self.rule_ids + self.or_groups
        if self.args.scenario_id != 1 or self.args.screening_trials_per_rule < 1:
            raise ValueError("ddc10_screening requires scenario-id=1 and at least one block")
        if self.args.directions.strip() != "tx":
            raise ValueError("ddc10_screening supports TX only")
        self.rows: List[Dict[str, Any]] = []
        self.control_overlap: List[Dict[str, Any]] = []

    def schedule(self) -> List[Dict[str, Any]]:
        rng = random.Random(self.args.seed)
        schedule = []
        for block in range(1, self.args.screening_trials_per_rule + 1):
            arms = ["CONTROL"] + self.arm_ids
            rng.shuffle(arms)
            schedule.extend({"block_index": block, "position": index + 1, "arm": arm,
                             "configured_seed": self.args.seed + block} for index, arm in enumerate(arms))
        return schedule

    def run(self) -> int:
        manifest_path = self.output / "screening_manifest.json"
        if manifest_path.exists() or (self.output / "screening_trials.csv").exists():
            raise FileExistsError("screening output already exists; choose a new --output-dir")
        schedule = self.schedule()
        manifest = {
            "scenario_id": 1, "mode": "ddc10_screening", "execute": self.args.execute,
            "blocks": self.args.screening_trials_per_rule, "total_runs": len(schedule),
            "max_injections_per_attack_round": 1, "control_injections": 0,
            "control_type": "same_proxy_monitor_only", "map_seed": 1,
            "seed_scope": "randomized_block_order_and_candidate_metadata; ROS scheduling is not deterministic",
            "outcome": "task-active inter-drone STL margin <= 0; pre-injection failures are not attack successes",
            "overlap_population": "feasible pre-injection packet opportunities in valid control rounds",
            "rules": {key: self.rule_description(key) for key in self.arm_ids},
            "run_sec": self.search.orchestrator.run_sec, "arguments": vars(self.args), "schedule": schedule,
        }
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="ascii")
        print("ddc10_screening rules={} blocks={} total_runs={} output={}".format(
            len(self.arm_ids), self.args.screening_trials_per_rule, len(schedule), self.output), flush=True)
        if not self.args.execute:
            print("Plan written to {}; add --execute with a new output directory to run.".format(manifest_path), flush=True)
            return 0
        for item in schedule:
            arm = item["arm"]
            candidate = self.search.screening_candidate(item["block_index"], arm == "CONTROL")
            candidate_file = self.output / "candidates" / arm / "trial_{:04d}.json".format(item["block_index"])
            candidate_file.parent.mkdir(parents=True, exist_ok=True)
            candidate_file.write_text(json.dumps(dict(asdict(candidate), active_rule_id=arm), indent=2), encoding="ascii")
            result = self.search.orchestrator.run_candidate(candidate, candidate_file, rule_id=arm)
            row, screening = self.collect(item, result)
            self.rows.append(row)
            if arm == "CONTROL" and row["valid_run"]:
                self.control_overlap.append(screening)
            write_csv(self.output / "screening_trials.csv", self.rows)
            write_csv(self.output / "screening_summary.csv", self.summaries())
            write_csv(self.output / "screening_rule_overlap.csv", self.overlaps())
            print("screening block={} arm={} valid={} injections={} target={} attack_success={} reason={}".format(
                item["block_index"], arm, row["valid_run"], row["actual_injection_count"],
                row["target_collision"], row["attack_success"], row["invalid_reason"]), flush=True)
        return 0

    @staticmethod
    def rule_description(rule_id: str) -> Dict[str, Any]:
        components = ddc10_or_components(rule_id)
        if not components:
            return {"name": DDC10_RULES[rule_id][0], "expression": DDC10_RULES[rule_id][1]}
        expressions = ["({})".format(DDC10_RULES[item][1]) for item in components]
        return {"name": "or_" + "_".join(item.rsplit("_", 1)[1] for item in components),
                "expression": " OR ".join(expressions), "components": list(components)}

    def collect(self, item: Dict[str, Any], result: Any) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        batch = Path(result.result_dir)
        meta_files = sorted(batch.glob("run*/rule_pool_latency_meta.json")) if batch.is_dir() else []
        try:
            meta = json_object(meta_files[-1].read_text(encoding="ascii")) if meta_files else {}
        except OSError:
            meta = {}
        screening = meta.get("screening", {})
        summary = self.search.orchestrator.evaluator._latest_run_summary(batch) or {}
        raw = next((number(row.get("stl_value")) for row in summary.get("rows", [])
                    if str(row.get("scenario_id")) == "1"), None)
        events = meta.get("events", [])
        count = number(meta.get("actual_injection_count"))
        control = item["arm"] == "CONTROL"
        reasons = []
        if result.status != "EXECUTED":
            reasons.append(result.status)
        runtime_audits = [audit_runtime(path.parent) for path in batch.glob("run*/result.txt")]
        if any(audit["status"] == "runtime_failure" for audit in runtime_audits):
            reasons.append("simulation_runtime_failure")
        if any(audit["status"] == "unresolved" for audit in runtime_audits):
            reasons.append("unresolved_runtime_error_phase")
        if raw is None or not summary.get("monitoring_started"):
            reasons.append("missing_active_task_STL")
        if not meta.get("final") or not screening.get("enabled"):
            reasons.append("missing_final_screening_metadata")
        if bool(screening.get("monitor_only")) != control:
            reasons.append("incorrect_control_configuration")
        if not screening.get("observation_count") or screening.get("evaluation_errors", 0):
            reasons.append("missing_or_failed_rule_observations")
        if count not in ((0,) if control else (0, 1)) or count != len(events):
            reasons.append("injection_budget_or_log_mismatch")
        first_injection = number(events[0].get("ros_time")) if events else None
        first_collision = self.first_collision_time(batch, summary, screening)
        first_any_collision = number(screening.get("first_collision_ros_time"))
        preexisting = first_injection is not None and any(
            stamp is not None and stamp <= first_injection for stamp in (first_collision, first_any_collision))
        target = raw is not None and raw <= 0.0
        valid = not reasons
        attack_success = (valid and not control and count == 1 and target and not preexisting
                          and first_injection is not None and first_collision is not None
                          and first_collision > first_injection)
        evidence = json_object(events[0].get("trigger_evidence", "{}")) if events else {}
        if control:
            attribution = "control"
        elif not count:
            attribution = "no_injection"
        elif preexisting:
            attribution = "preexisting_failure"
        elif target and first_collision is None:
            attribution = "missing_collision_timestamp"
        elif attack_success:
            attribution = "collision_after_injection"
        else:
            attribution = "no_attributed_collision"
        row = dict(item)
        row.update({
            "rule_expression": "no injected delay" if control else self.rule_description(item["arm"])["expression"],
            "status": result.status, "valid_run": int(valid), "invalid_reason": ";".join(reasons),
            "runtime_audit": json.dumps(runtime_audits, ensure_ascii=True, sort_keys=True),
            "target_collision": int(target) if raw is not None else "NA", "stl_margin_m": raw if raw is not None else "NA",
            "actual_injection_count": int(count) if count is not None else "NA",
            "injection_ros_time": first_injection if first_injection is not None else "NA",
            "first_collision_ros_time": first_collision if first_collision is not None else "NA",
            "preexisting_failure_at_injection": int(preexisting), "attack_success": int(attack_success),
            "attribution_status": attribution,
            "injection_times_s": result.actual_injection_times_s, "latencies_ms": result.actual_latency_ms,
            "source_node_ids": result.actual_attacker_node_ids, "victim_node_ids": result.actual_victim_node_ids,
            "matched_rule_ids_at_injection": evidence.get("all_matched_rule_ids", ""),
            "pre_injection_rule_match_counts": json.dumps(screening.get("rule_match_counts", {}), sort_keys=True),
            "delay_skip_counts": json.dumps(meta.get("delay_schedule_skip_counts", {}), sort_keys=True),
            "result_dir": result.result_dir, "events_file": result.adaptive_events_file,
            "observations_file": str(meta_files[-1].parent / "ddc10_rule_observations.csv") if meta_files else "NA",
        })
        row.update({key: int(evidence.get("all_rule_matches", {}).get(key, False)) if events else "NA"
                    for key in DDC10_RULE_IDS})
        return row, screening

    @staticmethod
    def first_collision_time(batch: Path, summary: Dict[str, Any], screening: Dict[str, Any]) -> Optional[float]:
        times = []
        proxy_time = number(screening.get("first_drone_collision_ros_time"))
        if proxy_time is not None:
            times.append(proxy_time)
        # Trace snapshots retain running minima; the earliest crossing, not the final minimum's time, matters.
        for path in sorted(batch.glob("run*/stl_metrics_trace.jsonl")):
            with path.open(encoding="ascii") as stream:
                for line in stream:
                    try:
                        snapshot = json.loads(line)
                    except ValueError:
                        continue
                    value = next((number(row.get("stl_value")) for row in snapshot.get("rows", [])
                                  if str(row.get("scenario_id")) == "1"), None)
                    stamp = number(snapshot.get("ros_time"))
                    if value is not None and value <= 0 and stamp is not None:
                        times.append(stamp)
                        break
        detail = summary.get("details", {}).get("1", {})
        if number(detail.get("stl_value")) is not None and number(detail.get("stl_value")) <= 0:
            stamp = number(detail.get("evidence_time"))
            if stamp is not None:
                times.append(stamp)
        return min(times) if times else None

    def summaries(self) -> List[Dict[str, Any]]:
        controls = {row["block_index"]: row for row in self.rows if row["arm"] == "CONTROL" and row["valid_run"]}
        control_successes = sum(row["target_collision"] for row in controls.values())
        control_rate = fraction(control_successes, len(controls))
        output = []
        for arm in ["CONTROL"] + self.arm_ids:
            rows = [row for row in self.rows if row["arm"] == arm]
            valid = [row for row in rows if row["valid_run"]]
            injected = [row for row in valid if row["actual_injection_count"] > 0]
            collisions = sum(row["target_collision"] for row in valid)
            rate = fraction(collisions, len(valid))
            low, high = wilson(collisions, len(valid))
            pairs = [(row, controls[row["block_index"]]) for row in valid if row["block_index"] in controls]
            wins = sum(row["target_collision"] > control["target_collision"] for row, control in pairs)
            losses = sum(row["target_collision"] < control["target_collision"] for row, control in pairs)
            components = ddc10_or_components(arm) or (arm,)
            triggered = sum(any(json.loads(row["pre_injection_rule_match_counts"]).get(key, 0) > 0
                                for key in components) for row in valid)
            control_triggered = sum(any(json.loads(row["pre_injection_rule_match_counts"]).get(key, 0) > 0
                                        for key in components) for row in controls.values())
            output.append({
                "rule_id": arm, "rule_expression": "no injected delay" if arm == "CONTROL"
                else self.rule_description(arm)["expression"],
                "completed_trials": len(rows), "valid_trials": len(valid), "invalid_trials": len(rows) - len(valid),
                "rule_triggered_trials": triggered if arm != "CONTROL" else "NA",
                "control_rule_triggered_trials": control_triggered if arm != "CONTROL" else "NA",
                "injected_trials": len(injected), "injection_trigger_rate": fraction(len(injected), len(valid)),
                "target_collision_count": collisions, "target_collision_rate": rate,
                "target_rate_ci95_low": low, "target_rate_ci95_high": high,
                "attack_success_count": sum(row["attack_success"] for row in valid),
                "attack_success_rate": fraction(sum(row["attack_success"] for row in valid), len(valid)),
                "attack_success_rate_per_injected_trial": fraction(
                    sum(row["attack_success"] for row in injected), len(injected)),
                "target_rate_in_injected_trials": fraction(sum(row["target_collision"] for row in injected), len(injected)),
                "control_target_collision_rate": control_rate,
                "risk_difference_percentage_points": 100 * (rate - control_rate) if rate != "NA" and control_rate != "NA" else "NA",
                "paired_blocks": len(pairs), "paired_attack_only_collisions": wins, "paired_control_only_collisions": losses,
                "paired_risk_difference_percentage_points": 100 * (wins - losses) / len(pairs) if pairs else "NA",
            })
        return output

    def overlaps(self) -> List[Dict[str, Any]]:
        output = []
        for first, second in combinations(self.rule_ids, 2):
            a = sum(row.get("feasible_rule_match_counts", {}).get(first, 0) for row in self.control_overlap)
            b = sum(row.get("feasible_rule_match_counts", {}).get(second, 0) for row in self.control_overlap)
            # Proxy keys use catalog order, independent of user-specified screening order.
            ordered = sorted((first, second), key=DDC10_RULE_IDS.index)
            key = ";".join(ordered)
            both = sum(row.get("feasible_rule_pair_match_counts", {}).get(key, 0) for row in self.control_overlap)
            output.append({"rule_a": first, "rule_b": second, "valid_control_trials": len(self.control_overlap),
                           "a_feasible_opportunities": a, "b_feasible_opportunities": b,
                           "both_feasible_opportunities": both, "jaccard_packet_overlap": fraction(both, a + b - both)})
        return output


def recompute_screening(source: Path, output: Optional[Path] = None) -> Dict[str, Any]:
    """Reevaluate a stable CSV snapshot without launching or changing any simulation."""
    from inter_drone_collision_attack import InterDroneCollisionAttackSearch

    source = source.resolve()
    output = (output or source / "recomputed").resolve()
    if output == source:
        raise ValueError("recompute output must differ from the running campaign directory")
    manifest = json_object((source / "screening_manifest.json").read_text(encoding="ascii"))
    if manifest.get("mode") != "ddc10_screening":
        raise ValueError("source is not a DDC10 screening campaign")
    with (source / "screening_trials.csv").open(encoding="utf-8-sig", newline="") as stream:
        previous = list(csv.DictReader(stream))
    saved = dict(manifest["arguments"], output_dir=str(output), execute=False)
    workspace = next((parent for parent in source.parents if (parent / "src/run_headless.sh").is_file()), None)
    for field in ("baseline_csv", "run_headless"):
        if workspace is not None and not Path(saved[field]).is_absolute():
            saved[field] = str(workspace / saved[field])
    suite = DDC10RuleScreening(InterDroneCollisionAttackSearch(argparse.Namespace(**saved)))
    identities = set()
    for index, old in enumerate(previous, 1):
        item = {"block_index": int(old["block_index"]), "position": int(old["position"]),
                "arm": old["arm"], "configured_seed": int(old["configured_seed"])}
        identity = (item["arm"], item["block_index"])
        if identity in identities:
            raise ValueError("duplicate trial identity: {}".format(identity))
        identities.add(identity)
        result = SimpleNamespace(
            status=old["status"], result_dir=old["result_dir"],
            actual_injection_times_s=old["injection_times_s"], actual_latency_ms=old["latencies_ms"],
            actual_attacker_node_ids=old["source_node_ids"], actual_victim_node_ids=old["victim_node_ids"],
            adaptive_events_file=old["events_file"],
        )
        row, screening = suite.collect(item, result)
        suite.rows.append(row)
        if item["arm"] == "CONTROL" and row["valid_run"]:
            suite.control_overlap.append(screening)
        if index % 100 == 0:
            print("recomputed {}/{} trials".format(index, len(previous)), flush=True)
    write_csv(output / "screening_trials.csv", suite.rows)
    write_csv(output / "screening_summary.csv", suite.summaries())
    write_csv(output / "screening_rule_overlap.csv", suite.overlaps())
    report = {
        "source": str(source), "output": str(output), "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "snapshot_completed_trials": len(previous), "planned_total_trials": manifest["total_runs"],
        "previous_valid_trials": sum(old["valid_run"] == "1" for old in previous),
        "recomputed_valid_trials": sum(row["valid_run"] for row in suite.rows),
        "previous_invalid_reasons": dict(Counter(old["invalid_reason"] for old in previous if old["valid_run"] != "1")),
        "recomputed_invalid_reasons": dict(Counter(row["invalid_reason"] for row in suite.rows if not row["valid_run"])),
        "runtime_audit_statuses": dict(Counter(audit["status"] for row in suite.rows for audit in json.loads(row["runtime_audit"]))),
    }
    (output / "recompute_report.json").write_text(json.dumps(report, indent=2, sort_keys=True), encoding="ascii")
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Offline reconstruction of DDC10 screening statistics")
    parser.add_argument("--recompute-from", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path, default=None)
    options = parser.parse_args()
    recompute_screening(options.recompute_from, options.output_dir)
