import copy
import csv
import json
import os
import subprocess
import tempfile
import unittest
from unittest import mock
from pathlib import Path

import numpy as np

from latency_eval.analysis import (export_trials, kmeans, reports, split,
                                  validation_metrics)
from latency_eval.cli import main, missing_labels, evaluate_trial, execute_capture
from latency_eval.core import (MessageAssociator, ROOT, TARGETS, Trajectory,
                               VoxelMap, config, injection_record, trial_plan,
                               unique_output)
from latency_eval.features import extract
from latency_eval.outcomes import classify
from latency_eval.ros_capture import make_launch


def csv_rows(path):
    with Path(path).open() as f:
        return list(csv.DictReader(f))


def snapshot(t, i=1, position=None, fsm="EXEC_TRAJ", **kwargs):
    return {"kind": "snapshot", "observed_ns": int(t * 1e9), "mono_ns": int(t * 1e9), "ros_s": t,
            "id": i, "position": position or [i * 2., 0., 1.], "velocity": [0., 0., 0.],
            "acceleration": [0., 0., 0.], "yaw": 0., "fsm": fsm, "plan_result": 2,
            "active_grid_count": 2, "frontier_count": 3, **kwargs}


class GateTests(unittest.TestCase):
    def test_actual_cpp_gate(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(Path(tmp) / "gate")
            include = ROOT / "swarm_exploration/exploration_manager/include"
            source = Path(__file__).with_name("test_tx_gate.cpp")
            subprocess.run(["g++", "-std=c++14", "-Wall", "-Wextra", "-I", str(include), str(source), "-o", binary], check=True)
            subprocess.run([binary], check=True)

    def test_fifo_duplicate_payload_association(self):
        a = MessageAssociator()
        a.sent(1, b"same", 10, 100)
        a.sent(1, b"same", 11, 110)
        self.assertIsNone(a.received(1, b"same", 90))
        self.assertEqual(a.received(1, b"same", 120)["seq"], 10)
        self.assertEqual(a.received(1, b"same", 130)["seq"], 11)
        self.assertIsNone(a.received(2, b"same", 140))

    def tx_events(self):
        base = {"kind": "tx", "id": 1, "seq": 5, "generated_ns": 100, "message_stamp_s": 1., "planned_delay_ns": 20,
                "fsm": "EXEC_TRAJ", "period_ns": 40, "budget_ns": 100, "publish_ros_s": 1.00000002}
        return [{"kind": "task_started", "id": 1, "mono_ns": 50, "observed_ns": 51, "ros_s": .99999995},
                {**base, "action": "scheduled", "observed_ns": 101},
                {**base, "action": "release", "observed_ns": 123, "publish_before_ns": 120, "publish_after_ns": 121,
                 "actual_injected": True, "budget_ok": True},
                {**base, "seq": 6, "action": "forward", "observed_ns": 141, "publish_before_ns": 140, "publish_after_ns": 141}]

    def test_schedule_is_not_actual(self):
        events = self.tx_events()[:2]
        r = injection_record(events, {"source_id": 1})
        self.assertFalse(r["actual_injected"])
        self.assertEqual(r["skip_reason"], "scheduled_but_no_release_evidence")

    def test_exact_schedule_release_and_successor_contract(self):
        r = injection_record(self.tx_events(), {"source_id": 1})
        self.assertTrue(r["actual_injected"] and r["contract_valid"])
        self.assertEqual(r["actual_delay_ns"], 20)

    def test_mismatched_timestamp_is_invalid(self):
        events = self.tx_events()
        events[2]["message_stamp_s"] = 9
        self.assertFalse(injection_record(events, {"source_id": 1})["contract_valid"])

    def test_budget_violation_retains_actual_injection(self):
        events = self.tx_events()
        events[2]["budget_ok"] = False
        r = injection_record(events, {"source_id": 1})
        self.assertTrue(r["actual_injected"])
        self.assertFalse(r["contract_valid"])

    def test_missing_successor_is_not_verified(self):
        self.assertFalse(injection_record(self.tx_events()[:3], {"source_id": 1})["contract_valid"])

    def test_pre_task_injection_is_not_valid(self):
        events = self.tx_events()
        events[0]["mono_ns"] = 1000
        self.assertFalse(injection_record(events, {"source_id": 1})["contract_valid"])


class FeaturesTests(unittest.TestCase):
    def setUp(self):
        self.cfg = config()
        self.cfg["active_ids"] = [1, 2]
        self.events = [{"kind": "task_started", "id": 1, "observed_ns": 100000000, "mono_ns": 100000000, "ros_s": .1},
                       snapshot(1, 1), snapshot(6, 1), snapshot(6, 2)]

    def test_pre_injection_features_reject_late_delivery_and_future_results(self):
        cutoff = 6100000000
        before = extract(self.events, cutoff, 6.1, 1, self.cfg)
        future = snapshot(6.05, 1, velocity=[999, 0, 0])
        future["observed_ns"] = 7000000000  # old source timestamp, not observable yet
        after = extract(self.events + [future, snapshot(8, 1, velocity=[777, 0, 0])], cutoff, 6.1, 1, self.cfg)
        self.assertEqual(before, after)
        self.assertEqual(before["fields"]["source_velocity_norm"]["value"], 0)

    def test_missing_and_unit_provenance(self):
        f = extract([], 10, 1, 1, self.cfg)["fields"]
        self.assertTrue(f["source_acceleration_norm"]["missing"])
        self.assertEqual(f["source_acceleration_norm"]["unit"], "m/s^2")
        self.assertTrue(f["source_acceleration_norm"]["missing_reason"])
        self.assertNotIn("emergency_stop_count", f)

    def test_old_odom_not_refreshed_by_snapshot_query(self):
        e = snapshot(6, 1, position_stamp_s=4)
        f = extract(self.events[:1] + [e, snapshot(6, 2)], 6100000000, 6.1, 1, self.cfg)["fields"]
        self.assertTrue(f["source_velocity_norm"]["missing"])

    def test_missing_peer_invalidates_local_features(self):
        f = extract([snapshot(6, 1)], 6100000000, 6.1, 1, self.cfg)["fields"]
        self.assertFalse(f["source_velocity_norm"]["missing"])
        self.assertTrue(f["local_max_velocity"]["missing"])

    def test_future_plan_available_before_cutoff_not_post_injection_trajectory(self):
        plan = {"kind": "trajectory", "observed_ns": 5900000000, "ros_s": 5.9, "id": 1,
                "order": 1, "knots": [0, 0, 10, 10], "pos_pts": [[0, 0, 0], [10, 0, 0]], "start_s": 6.0, "traj_id": 1}
        tr = Trajectory(plan)
        self.assertAlmostEqual(tr.at(8)[0], 2)
        with self.assertRaises(ValueError):
            tr.at(20)
        e = self.events + [plan, {**plan, "id": 2, "pos_pts": [[0, 2, 0], [10, 2, 0]]}]
        f = extract(e, 6100000000, 6.1, 1, self.cfg)["fields"]
        self.assertAlmostEqual(f["local_future_3s_min_inter_drone_distance"]["value"], 2)
        e.append({**plan, "observed_ns": 6200000000, "pos_pts": [[100, 100, 0], [110, 100, 0]]})
        self.assertEqual(f, extract(e, 6100000000, 6.1, 1, self.cfg)["fields"])


class OutcomeTests(unittest.TestCase):
    def setUp(self):
        self.cfg = config()
        self.cfg["active_ids"] = [1, 2]
        self.cfg["thresholds"].update(deadlock_window_s=.5, drift_duration_s=.2)

    def test_six_simultaneous_labels_and_evidence(self):
        events = []
        for t in np.arange(1, 2.01, .1):
            for i in (1, 2):
                events.append(snapshot(t, i, [i * .1, 0, 1], received_states=[{"id": 3 - i, "stamp_s": t - .4,
                                             "position": [9, 0, 1], "velocity": [0, 0, 0], "yaw": 0}]))
        events += [{"kind": "plan_result", "observed_ns": 1500000000, "id": 1, "result": 1, "duration_s": .3}]
        m = VoxelMap([[.1, 0, 1]], .1)
        labels = classify(events, 1000000000, 2000000000, self.cfg, m)
        self.assertTrue(all(labels[t]["hit"] for t in TARGETS), labels)
        self.assertTrue(all(labels[t]["evidence"][0]["ids"] for t in TARGETS))
        self.assertIn("fsm", labels["deadlock"]["evidence"][0])

    def test_idle_is_not_deadlock(self):
        events = [snapshot(t, i, fsm="IDLE") for t in np.arange(1, 3, .1) for i in (1, 2)]
        self.assertFalse(classify(events, 1000000000, 2900000000, self.cfg)["deadlock"]["hit"])

    def test_predictions_are_not_collision_labels(self):
        events = [{"kind": "inter_drone_prediction", "observed_ns": 100, "id": 1, "other_id": 2},
                  {"kind": "obstacle_prediction", "observed_ns": 101, "id": 1}]
        labels = classify(events, 0, 1000, self.cfg)
        self.assertFalse(labels["inter_drone_collision"]["hit"])
        self.assertFalse(labels["obstacle_collision"]["valid"])

    def test_missing_observation_not_a_negative(self):
        self.assertTrue(all(not v["valid"] for v in classify([], 0, 100, self.cfg).values()))

    def test_voxel_sphere_geometry(self):
        m = VoxelMap([[0, 0, 0]], .1)
        self.assertTrue(m.collides([.2, .05, .05], .11))
        self.assertFalse(m.collides([.3, .05, .05], .1))
        self.assertAlmostEqual(m.distance([.3, .05, .05]), .2)


def analysis_trials(n=120):
    rows = []
    for i in range(n):
        hit = bool(i % 2)
        labels = missing_labels("")
        for target in TARGETS:
            labels[target].update(hit=hit, valid=True)
        fields = {"speed": {"value": (4 if hit else 0) + i % 7 * .1, "unit": "m/s", "missing": False,
                            "missing_reason": "", "source": "fixture", "observed_ns": [1, 2], "source_ros_s": [1, 2]},
                  "missing_all": {"value": None, "unit": "m", "missing": True, "missing_reason": "fixture"}}
        rows.append({"trial_id": str(i), "seed": i, "source_id": 1, "status": "completed", "control": False,
                     "actual_injected": True, "contract_valid": True, "labels": labels, "features": {"fields": fields}})
    return rows


class AnalysisTests(unittest.TestCase):
    def test_fixed_seed_kmeans(self):
        x = np.array([[0., 0], [1, 1], [8, 8], [9, 9]])
        a, b = kmeans(x, 17)
        c, d = kmeans(x, 17)
        np.testing.assert_array_equal(a, c)
        np.testing.assert_array_equal(b, d)

    def test_duplicate_group_skipped(self):
        with self.assertRaisesRegex(ValueError, "distinct"):
            kmeans(np.ones((10, 2)), 1)

    def test_reproducible_full_analysis_and_train_only_preprocessing(self):
        rows = analysis_trials()
        cfg = config()
        cfg["bootstrap_samples"] = 32
        with tempfile.TemporaryDirectory() as tmp:
            a, b, c = [unique_output(tmp, x) for x in ("a", "b", "c")]
            reports(a, rows, cfg)
            reports(b, rows, cfg)
            for f in ("cluster_assignments.csv", "cluster_summary.csv", "rules_per_target.csv", "preprocessing.json"):
                self.assertEqual((a / f).read_bytes(), (b / f).read_bytes())
            train, held = split(rows, TARGETS[0], cfg["analysis_seed"], cfg["holdout_fraction"])
            modified = copy.deepcopy(rows)
            ids = {r["trial_id"] for r in held}
            for r in modified:
                if r["trial_id"] in ids:
                    r["features"]["fields"]["speed"]["value"] = 9999
            reports(c, modified, cfg)
            self.assertEqual(json.loads((a / "preprocessing.json").read_text())[TARGETS[0]],
                             json.loads((c / "preprocessing.json").read_text())[TARGETS[0]])
            self.assertNotIn("missing_all", json.loads((a / "preprocessing.json").read_text())[TARGETS[0]]["fields"])
            rules = csv_rows(a / "rules_per_target.csv")
            self.assertTrue(any(r["validation_status"] == "evaluated" for r in rules))
            self.assertTrue(all(len(json.loads(r["conditions"])) <= 3 for r in rules if r["conditions"]))

    def test_empty_groups_write_all_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            reports(path, [], config())
            export_trials(path, [])
            self.assertEqual(len(csv_rows(path / "summary_per_target.csv")), 6)
            self.assertIn("skipped", (path / "cluster_summary.csv").read_text())

    def test_rules_missing_values_not_imputed_for_trigger(self):
        cfg = config()
        r = validation_metrics(np.array([[np.nan], [0.], [1.]]), np.array([1, 0, 1], dtype=bool),
                               ((0, ">", .5),), cfg, 2)
        self.assertEqual(r["excluded_missing_n"], 1)
        self.assertEqual(r["target_rate"], 1)
        self.assertEqual(r["target_not_rate"], 0)
        self.assertEqual(r["confidence"], "low")

    def test_explicit_controls_separate_trigger_rate(self):
        rows = analysis_trials()
        control = copy.deepcopy(rows[0])
        control.update(trial_id="control", control=True, actual_injected=False, contract_valid=False)
        cfg = config()
        cfg["bootstrap_samples"] = 20
        with tempfile.TemporaryDirectory() as tmp:
            reports(Path(tmp), rows + [control], cfg)
            data = csv_rows(Path(tmp) / "rules_per_target.csv")
            self.assertTrue(all(r["control_eligible_n"] == "1" for r in data if r["expression"]))


class OutputTests(unittest.TestCase):
    def test_worker_import_path_is_explicit_not_parent_sys_path(self):
        with mock.patch("latency_eval.cli.subprocess.Popen") as popen:
            popen.return_value.wait.return_value = 0
            execute_capture(Path("/tmp/unused_trial"), config())
            child_env = popen.call_args.kwargs["env"]
            self.assertEqual(child_env["PYTHONPATH"].split(os.pathsep)[0], str(ROOT / "tools"))
            self.assertTrue(popen.call_args.kwargs["start_new_session"])

    def test_failed_workers_export_all_plans_without_claiming_injection(self):
        def failed(directory, cfg):
            (directory / "capture.json").write_text(json.dumps({"status": "capture_failed", "end_ns": 1,
                                                               "failure_reason": "test_socket_unavailable"}))
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch("latency_eval.cli.preflight_run"), mock.patch("latency_eval.cli.execute_capture", side_effect=failed) as worker:
                self.assertEqual(main(["run", "--trials", "5", "--output-root", tmp]), 1)
            self.assertEqual(worker.call_count, 3)
            out = next(Path(tmp).iterdir())
            rows = csv_rows(out / "trials.csv")
            self.assertEqual(len(rows), 5)
            self.assertTrue(all(r["actual_injected"] == "false" for r in rows))
            self.assertEqual(sum(r["status"] == "not_run" for r in rows), 2)
            self.assertTrue((out / "rules_per_target.csv").exists())

    def test_captured_trial_to_offline_report_without_modifying_input(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / "recorded"
            directory = run / "trial_0001"
            directory.mkdir(parents=True)
            (run / "config.json").write_text(json.dumps(cfg))
            plan = {**trial_plan(1, 42, cfg), "source_id": 1, "offset_s": .2}
            (directory / "plan.json").write_text(json.dumps(plan))
            events = [{"kind": "task_started", "id": 1, "mono_ns": 100000000, "observed_ns": 100000001, "ros_s": .1}]
            for t in np.arange(.1, 1.11, .05):
                events.extend(snapshot(t, i) for i in cfg["active_ids"])
            base = {"kind": "tx", "id": 1, "seq": 5, "generated_ns": 500000000, "message_stamp_s": .5,
                    "planned_delay_ns": 20000000, "period_ns": 40000000, "budget_ns": 200000000,
                    "fsm": "EXEC_TRAJ", "publish_ros_s": .52}
            events += [{**base, "action": "scheduled", "observed_ns": 500000001},
                       {**base, "action": "release", "observed_ns": 520000002, "publish_before_ns": 520000000,
                        "publish_after_ns": 520000001, "actual_injected": True, "budget_ok": True},
                       {**base, "seq": 6, "action": "forward", "observed_ns": 540000002,
                        "publish_before_ns": 540000000, "publish_after_ns": 540000001},
                       {"kind": "plan_result", "id": 1, "result": 2, "observed_ns": 800000000}]
            path = directory / "events.jsonl"
            path.write_text("".join(json.dumps(e) + "\n" for e in events))
            before = path.read_bytes()
            (directory / "capture.json").write_text(json.dumps({"status": "completed", "end_ns": 1100000000}))
            np.save(str(directory / "scene_points.npy"), np.array([[100., 100., 100.]]))
            result = evaluate_trial(directory, cfg)
            self.assertTrue(result["actual_injected"] and result["contract_valid"])
            self.assertTrue(result["labels"]["inter_drone_collision"]["valid"])
            self.assertFalse(result["labels"]["inter_drone_collision"]["hit"])
            self.assertEqual(main(["analyze", "--input", str(run), "--output-root", str(Path(tmp) / "reports")]), 0)
            out = next((Path(tmp) / "reports").iterdir())
            rows = csv_rows(out / "trials.csv")
            self.assertTrue(Path(rows[0]["result_path"]).is_file())
            self.assertEqual(path.read_bytes(), before)

    def test_unique_output_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = unique_output(tmp, "run")
            (a / "old.txt").write_text("keep")
            b = unique_output(tmp, "run")
            self.assertNotEqual(a, b)
            self.assertEqual((a / "old.txt").read_text(), "keep")

    def test_seed_per_trial_reproducible_and_independent(self):
        cfg = config()
        self.assertEqual(trial_plan(3, 4, cfg), trial_plan(3, 4, cfg))
        self.assertNotEqual(trial_plan(3, 4, cfg)["seed"], trial_plan(4, 4, cfg)["seed"])
        for i in range(20):
            self.assertGreaterEqual(trial_plan(i, 4, cfg)["offset_s"], cfg["injection_window_s"][0])

    def test_launch_is_simulation_only_and_solver_paths_isolated(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = make_launch(Path(tmp), {**trial_plan(1, 4, config()), "control": False}, config(), "/tmp/map.pcd")
            text = p.read_text()
            self.assertEqual(text.count('name="evaluation" value="true"'), 4)
            self.assertNotIn("realworld", text)
            self.assertIn(str(Path(tmp) / "solver"), text)

    def test_plan_cli_creates_artifacts_without_running_ros(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(main(["plan", "--trials", "2", "--output-root", tmp]), 0)
            out = next(Path(tmp).iterdir())
            for f in ("manifest.json", "trials.csv", "trial_features.csv", "cluster_assignments.csv", "cluster_summary.csv",
                      "rules_per_target.csv", "summary_per_target.csv", "analysis.md"):
                self.assertTrue((out / f).exists(), f)
            trials = csv_rows(out / "trials.csv")
            self.assertTrue(all(r["actual_injected"] == "false" for r in trials))
            self.assertEqual(len(trials), 2)
            features = csv_rows(out / "trial_features.csv")
            self.assertEqual(features[0]["source_acceleration_norm__unit"], "m/s^2")
            self.assertEqual(features[0]["source_acceleration_norm__missing"], "true")
            self.assertTrue(features[0]["source_acceleration_norm__missing_reason"])


if __name__ == "__main__":
    unittest.main()
