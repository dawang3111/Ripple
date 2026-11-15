#!/usr/bin/env python3
import csv
import importlib.util
import json
import sys
import tempfile
import threading
import types
import unittest
from pathlib import Path
from unittest import mock

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

import inter_drone_collision_attack as attack
from ddc10_rules import DDC10_RULE_IDS
from ddc10_rule_screening import DDC10RuleScreening, recompute_screening, wilson


def proxy_module():
    modules = {}
    for name in ("rospy", "nav_msgs", "nav_msgs.msg", "sensor_msgs", "sensor_msgs.msg",
                 "sensor_msgs.point_cloud2", "std_msgs", "std_msgs.msg", "traj_utils", "traj_utils.msg"):
        modules[name] = types.ModuleType(name)
    for name, cls in (("nav_msgs.msg", "Odometry"), ("sensor_msgs.msg", "PointCloud2"),
                      ("std_msgs.msg", "String"), ("traj_utils.msg", "MINCOTraj"), ("traj_utils.msg", "PolyTraj")):
        setattr(modules[name], cls, type(cls, (), {}))
    modules["sensor_msgs"].point_cloud2 = modules["sensor_msgs.point_cloud2"]
    ros = modules["rospy"]
    for name in ("Publisher", "Subscriber", "on_shutdown", "loginfo", "logwarn", "logerr_throttle"):
        setattr(ros, name, mock.Mock())
    ros.Time = mock.Mock()
    ros.Time.now.return_value.to_sec.return_value = 100.0
    with mock.patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location("proxy_screening_test", SCRIPTS / "rule_pool_latency_proxy.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module, ros


class ProxyScreeningTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.module, self.ros = proxy_module()

    def proxy(self, control=False):
        params = {"~enable": True, "~screening_mode": True, "~monitor_only": control,
                  "~active_rule_id": "DDC10" if control else "DDC10_1", "~max_injections_per_round": 0 if control else 1,
                  "~report_dir": self.tmp.name, "~run_id": "proxy"}
        self.ros.get_param = lambda key, default=None: params.get(key, default)
        proxy = self.module.RulePoolLatencyProxy()
        proxy._enqueue_publish = mock.Mock()
        proxy._state = lambda node: "EXEC_TRAJ"
        proxy._is_active = lambda node: True
        proxy._nearest_triad = lambda source, now: [0, 1, 2]
        proxy._closest_other = lambda *args: 1
        proxy.period_ema_ms[0] = 700.0
        return proxy

    @staticmethod
    def evidence():
        return {"victim_id": 1, "all_rule_matches": {key: key in ("DDC10_1", "DDC10_3") for key in DDC10_RULE_IDS},
                "all_matched_rule_ids": "DDC10_1;DDC10_3"}

    def test_monitor_only_records_opportunities_without_injecting(self):
        p = self.proxy(control=True)
        p._match_curated_target1_union_rule = lambda *args: (True, self.evidence())
        p._broadcast_cb(types.SimpleNamespace(drone_id=0, traj_id=7))
        self.assertEqual(p.injection_count, 0)
        self.assertEqual(p.screening_feasible_counts["DDC10_1"], 1)
        self.assertEqual(p.screening_pair_counts["DDC10_1;DDC10_3"], 1)
        self.assertEqual(p._enqueue_publish.call_args.args[-1], 0.0)

    def test_single_injection_budget_and_all_rule_flags_under_concurrency(self):
        p = self.proxy()
        p._match_curated_target1_union_rule = lambda *args: (True, self.evidence())
        threads = [threading.Thread(target=p._broadcast_cb, args=(types.SimpleNamespace(drone_id=0, traj_id=i),)) for i in range(12)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(p.injection_count, 1)
        with open(p.events_csv_path) as stream:
            events = list(csv.DictReader(stream))
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["DDC10_1"], "1")
        self.assertEqual(events[0]["DDC10_2"], "0")
        self.assertEqual(events[0]["latency_ms"], "690.000000")
        self.assertEqual(p.screening_pre_observation_count, 1)

    def test_preexisting_failure_and_wait_state_block_injection(self):
        p = self.proxy()
        p._match_curated_target1_union_rule = lambda *args: (True, self.evidence())
        p.first_collision_ros_time = 99.0
        p._broadcast_cb(types.SimpleNamespace(drone_id=0, traj_id=7))
        self.assertEqual(p.injection_count, 0)
        p.first_collision_ros_time = None
        p._state = lambda node: "WAIT_TRIGGER"
        self.assertIsNone(p._observe_screening_rules_locked(0, 100.0, types.SimpleNamespace(traj_id=7)))

    def test_individual_rule_does_not_fall_back_to_union(self):
        p = self.proxy()
        p._pair_distances = lambda *args: [((0, 1), 1.0), ((0, 2), 2.0), ((1, 2), 1.0)]
        p._pair_victim = lambda *args: 1
        p._best_future_pair_distance = lambda *args: (1.2, 1.4, (0, 1))
        p._heading = lambda *args: [1.0, 0.0, 0.0]
        p._angle_deg = lambda *args: 20.0
        p._history_position_at_or_before = lambda node, now: [node * 1.5, 0.0, 0.0]
        p._position = lambda node, now: [float(node), 0.0, 0.0]
        p._is_replanning = lambda *args: True
        p._recent_new_traj = lambda *args: True
        p._speed = lambda *args: 1.4
        p._swarm_current_average = lambda *args: 1.4
        p._turn_angle = lambda *args: 8.0
        p._trajectory_offset = lambda *args: 0.04
        union, evidence = p._match_rule("DDC10", 0, [0, 1, 2], 100.0)
        self.assertTrue(union)
        for key in DDC10_RULE_IDS:
            matched, single = p._match_rule(key, 0, [0, 1, 2], 100.0)
            self.assertEqual(matched, evidence["all_rule_matches"][key])
            self.assertEqual(single["matched_condition_ids"], key if matched else "NA")
        self.assertFalse(p._match_rule("DDC10_5", 0, [0, 1, 2], 100.0)[0])


class ScreeningRunnerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        baseline = self.root / "baseline.csv"
        baseline.write_text("scenario_id,scenario_name,stl_value_avg,attacker_drone_id,victim_drone_id\n1,drone_drone_collision,0.2,0,1\n")
        args = attack.parse_args(["--baseline-csv", str(baseline), "--attack-mode", "ddc10_screening", "--execute",
                                  "--screening-rule-ids", "DDC10_1,DDC10_2", "--screening-trials-per-rule", "2",
                                  "--output-dir", str(self.root / "results")])
        self.engine = attack.InterDroneCollisionAttackSearch(args)

    def fake_simulation(self, cmd, cwd, env, check):
        self.assertEqual(env["RULE_POOL_SCREENING_MODE"], "true")
        control = env["RULE_POOL_MONITOR_ONLY"] == "true"
        self.assertEqual(env["RULE_POOL_MAX_INJECTIONS_PER_ROUND"], "0" if control else "1")
        run = Path(cmd[4]) / "batch" / "run1"
        run.mkdir(parents=True)
        flags = {key: key in ("DDC10_1", "DDC10_2") for key in DDC10_RULE_IDS}
        evidence = {"all_rule_matches": flags, "all_matched_rule_ids": "DDC10_1;DDC10_2"}
        events = [] if control else [{"ros_time": 101.0, "inject_time_s": 1.0, "latency_ms": 690.0,
                                    "attacker_drone_id": 0, "victim_drone_id": 1,
                                    "trigger_evidence": json.dumps(evidence)}]
        raw = 0.2 if control else -0.05
        summary = {"monitoring_started": True, "rows": [{"scenario_id": 1, "stl_value": raw}],
                   "details": {"1": {"stl_value": raw, "evidence_time": 102.0}}}
        screening = {"enabled": True, "monitor_only": control, "observation_count": 3,
                     "rule_match_counts": {"DDC10_1": 2, "DDC10_2": 2},
                     "feasible_rule_match_counts": {"DDC10_1": 2, "DDC10_2": 2},
                     "feasible_rule_pair_match_counts": {"DDC10_1;DDC10_2": 1}}
        meta = {"final": True, "screening": screening, "actual_injection_count": len(events), "events": events}
        (run / "stl_metrics_summary.json").write_text(json.dumps(summary))
        (run / "rule_pool_latency_meta.json").write_text(json.dumps(meta))
        with (run / "rule_pool_latency_events.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(events[0]) if events else ["inject_time_s"])
            writer.writeheader()
            writer.writerows(events)
        return types.SimpleNamespace(returncode=0)

    def test_end_to_end_block_schedule_controls_outputs_and_attribution(self):
        suite = DDC10RuleScreening(self.engine)
        self.assertEqual(suite.schedule(), DDC10RuleScreening(self.engine).schedule())
        with mock.patch.object(attack.subprocess, "run", side_effect=self.fake_simulation) as run:
            self.assertEqual(suite.run(), 0)
        self.assertEqual(run.call_count, 6)
        for arm in ("CONTROL", "DDC10_1", "DDC10_2"):
            rows = [row for row in suite.rows if row["arm"] == arm]
            self.assertEqual(len(rows), 2)
            self.assertTrue(all(row["valid_run"] for row in rows))
            self.assertTrue(all(row["actual_injection_count"] == (0 if arm == "CONTROL" else 1) for row in rows))
        summary = suite.summaries()[1]
        self.assertEqual(summary["attack_success_count"], 2)
        self.assertEqual(summary["risk_difference_percentage_points"], 100.0)
        self.assertAlmostEqual(suite.overlaps()[0]["jaccard_packet_overlap"], 1 / 3)
        with self.assertRaises(FileExistsError):
            suite.run()

    def test_zero_injection_and_preexisting_collision_not_attack_success(self):
        suite = DDC10RuleScreening(self.engine)
        candidate = self.engine.screening_candidate(1, False)
        with mock.patch.object(attack.subprocess, "run", side_effect=self.fake_simulation):
            result = self.engine.orchestrator.run_candidate(candidate, self.root / "candidate.json", "DDC10_1")
        meta_path = Path(result.result_dir) / "run1" / "rule_pool_latency_meta.json"
        meta = json.loads(meta_path.read_text())
        meta["screening"]["first_collision_ros_time"] = 100.5
        meta_path.write_text(json.dumps(meta))
        item = {"block_index": 1, "arm": "DDC10_1", "position": 1, "configured_seed": candidate.seed}
        row, _ = suite.collect(item, result)
        self.assertEqual(row["attack_success"], 0)
        self.assertEqual(row["preexisting_failure_at_injection"], 1)
        meta["events"] = []
        meta["actual_injection_count"] = 0
        meta_path.write_text(json.dumps(meta))
        row, _ = suite.collect(item, result)
        self.assertEqual(row["target_collision"], 1)
        self.assertEqual(row["attack_success"], 0)

    def test_invalid_run_excluded_and_wilson_zero_cases(self):
        suite = DDC10RuleScreening(self.engine)
        result = types.SimpleNamespace(result_dir="NA", status="RUN_FAILED", actual_injection_times_s="NA",
                                       actual_latency_ms="NA", actual_attacker_node_ids="NA",
                                       actual_victim_node_ids="NA", adaptive_events_file="NA")
        row, _ = suite.collect(suite.schedule()[0], result)
        self.assertEqual(row["valid_run"], 0)
        suite.rows.append(row)
        self.assertTrue(all(summary["target_collision_rate"] == "NA" for summary in suite.summaries()))
        self.assertEqual(wilson(0, 0), ("NA", "NA"))
        low, high = wilson(0, 100)
        self.assertAlmostEqual(low, 0)
        self.assertGreater(high, 0.03)

    def test_runtime_failure_and_corrupt_metadata_are_invalid(self):
        suite = DDC10RuleScreening(self.engine)
        candidate = self.engine.screening_candidate(1, False)
        with mock.patch.object(attack.subprocess, "run", side_effect=self.fake_simulation):
            result = self.engine.orchestrator.run_candidate(candidate, self.root / "candidate.json", "DDC10_1")
        run = Path(result.result_dir) / "run1"
        (run / "result.txt").write_text("fatal_runtime_count=1\nprocess_crash_count=0\n")
        item = {"block_index": 1, "arm": "DDC10_1", "position": 1, "configured_seed": candidate.seed}
        row, _ = suite.collect(item, result)
        self.assertEqual(row["valid_run"], 0)
        self.assertIn("simulation_runtime_failure", row["invalid_reason"])
        (run / "rule_pool_latency_meta.json").write_text("{")
        row, _ = suite.collect(item, result)
        self.assertEqual(row["valid_run"], 0)
        self.assertIn("missing_final_screening_metadata", row["invalid_reason"])

    def test_offline_recompute_does_not_change_campaign_outputs(self):
        suite = DDC10RuleScreening(self.engine)
        with mock.patch.object(attack.subprocess, "run", side_effect=self.fake_simulation):
            suite.run()
        source = self.engine.output_dir
        original = (source / "screening_trials.csv").read_bytes()
        report = recompute_screening(source)
        self.assertEqual(report["snapshot_completed_trials"], 6)
        self.assertEqual(report["recomputed_valid_trials"], 6)
        self.assertEqual((source / "screening_trials.csv").read_bytes(), original)
        self.assertTrue((source / "recomputed" / "screening_summary.csv").is_file())

    def test_runner_snapshot_survives_a_live_source_edit(self):
        source_dir = self.root / "src"
        source_dir.mkdir()
        runner = source_dir / "run_headless.sh"
        runner.write_text("#!/bin/bash\necho original\n")
        self.engine.orchestrator.run_headless = runner
        snapshots = []

        def run_snapshot(cmd, cwd, env, check):
            snapshot = Path(cmd[1])
            snapshots.append(snapshot)
            self.assertEqual(snapshot.parent, source_dir)
            runner.write_text("#!/bin/bash\necho changed\n")
            self.assertEqual(snapshot.read_text(), "#!/bin/bash\necho original\n")
            return self.fake_simulation(cmd, cwd, env, check)

        candidate = self.engine.screening_candidate(1, False)
        with mock.patch.object(attack.subprocess, "run", side_effect=run_snapshot):
            self.engine.orchestrator.run_candidate(candidate, self.root / "candidate.json", "DDC10_1")
        self.assertTrue(snapshots)
        self.assertTrue(all(not path.exists() for path in snapshots))


if __name__ == "__main__":
    unittest.main()
