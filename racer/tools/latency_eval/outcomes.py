from collections import defaultdict, deque
from itertools import combinations

import numpy as np

from .core import ACTIVE, TARGETS
from .features import norm


def classify(events, start_ns, end_ns, cfg, voxel_map=None, run_status="completed"):
    """Independent post-intervention labels. Missing evidence is never a valid negative."""
    ids = cfg["active_ids"]
    thresholds = cfg["thresholds"]
    gap = int(cfg["max_sample_gap_s"] * 1e9)
    tolerance = int(cfg["alignment_tolerance_s"] * 1e9)
    labels = {k: {"hit": False, "valid": False, "missing_reason": "", "evidence": [], "count": 0} for k in TARGETS}
    rows = sorted(events, key=lambda e: e.get("mono_ns", e["observed_ns"]))
    recent, history, drift_since = {}, defaultdict(deque), {}
    samples = defaultdict(list)
    event_counts = defaultdict(int)
    cache_complete = True
    telemetry_last = {}
    telemetry_complete = True

    def hit(target, e, involved, definition, value=None):
        label = labels[target]
        label["hit"] = True
        label["count"] += 1
        if len(label["evidence"]) < 20:
            states = {str(i): recent[i].get("fsm") for i in involved if i in recent}
            plans = {str(i): recent[i].get("plan_result") for i in involved if i in recent}
            if e.get("id") in involved:
                states[str(e["id"])] = e.get("fsm", states.get(str(e["id"])))
                plans[str(e["id"])] = e.get("result", e.get("plan_result", plans.get(str(e["id"]))))
            label["evidence"].append({"time_ns": e.get("mono_ns", e["observed_ns"]),
                "ros_s": e.get("ros_s"), "ids": involved, "definition": definition, "value": value,
                "fsm": states, "plan_result": plans,
                "position_measurement_ros_s": {str(i): recent[i].get("position_stamp_s", recent[i].get("ros_s")) for i in involved if i in recent}})

    for e in rows:
        t = e.get("mono_ns", e["observed_ns"])
        i = e.get("id")
        if "event_seq" in e:
            previous = telemetry_last.get(i)
            if previous is not None and e["event_seq"] != previous + 1 and start_ns <= t <= end_ns:
                telemetry_complete = False
            telemetry_last[i] = e["event_seq"]
        if e["kind"] == "snapshot" and i in ids:
            recent[i] = e
        if t < start_ns or t > end_ns:
            continue
        if e["kind"] == "plan_result":
            event_counts["plan"] += 1
            if e.get("duration_s", 0) > thresholds["planning_deadline_s"]:
                hit("delay_timeout", e, [i], "planning_call_deadline_exceeded", e["duration_s"])
            if e["result"] == 1:  # EXPL_RESULT: NO_FRONTIER=0, FAIL=1, SUCCEED=2, NO_GRID=3
                hit("planning_failure", e, [i], "planner_return_FAIL", e["result"])
        if e["kind"] == "planning_latency" and e["duration_s"] > thresholds["planning_deadline_s"]:
            hit("delay_timeout", e, [i], "planning_deadline_exceeded", e["duration_s"])
        if e["kind"] != "snapshot" or i not in ids:
            continue
        if not 0 <= e["ros_s"] - e.get("position_stamp_s", e["ros_s"]) <= cfg["alignment_tolerance_s"]:
            continue
        samples[i].append(t)
        aligned = {j: r for j, r in recent.items() if 0 <= t - r.get("mono_ns", r["observed_ns"]) <= tolerance and
                   0 <= e["ros_s"] - r.get("position_stamp_s", r["ros_s"]) <= cfg["alignment_tolerance_s"]}
        for a, b in combinations(sorted(aligned), 2):
            d = norm(np.array(aligned[a]["position"]) - aligned[b]["position"])
            if d <= 2 * thresholds["body_radius_m"]:
                hit("inter_drone_collision", e, [a, b], "3d_sphere_proxy_overlap", d)
        if voxel_map and voxel_map.collides(e["position"], thresholds["body_radius_m"]):
            hit("obstacle_collision", e, [i], "sphere_proxy_vs_static_pcd_occupied_voxel")
        if e.get("fsm") not in ACTIVE:
            history[i].clear()
            for key in list(drift_since):
                if i in key:
                    del drift_since[key]
            continue
        h = history[i]
        expected = {j for j, r in aligned.items() if j != i and r.get("fsm") in ACTIVE}
        cache_complete = cache_complete and expected.issubset({c["id"] for c in e.get("received_states", [])})
        if h and t - h[-1].get("mono_ns", h[-1]["observed_ns"]) > gap:
            h.clear()
        h.append(e)
        window = int(thresholds["deadlock_window_s"] * 1e9)
        while len(h) > 1 and t - h[1].get("mono_ns", h[1]["observed_ns"]) >= window:
            h.popleft()
        if h and t - h[0].get("mono_ns", h[0]["observed_ns"]) >= window:
            if (all(norm(r["velocity"]) <= thresholds["deadlock_speed_mps"] for r in h) and
                    all(norm(np.array(r["position"]) - h[0]["position"]) <= thresholds["deadlock_displacement_m"] for r in h) and
                    all("active_grid_count" in r and r["active_grid_count"] >= h[0].get("active_grid_count", float("inf")) for r in h)):
                hit("deadlock", e, [i], "active_no_motion_no_active_grid_progress_window", thresholds["deadlock_window_s"])
        for cached in e.get("received_states", []):
            j = cached["id"]
            if j not in ids or j not in aligned or aligned[j].get("fsm") not in ACTIVE:
                continue
            age = e["ros_s"] - cached["stamp_s"]
            if age > thresholds["state_timeout_s"]:
                hit("delay_timeout", e, [i, j], "receiver_neighbor_state_age", age)
            error = norm(np.array(cached["position"]) - aligned[j]["position"])
            key = (i, j)
            if error > thresholds["drift_distance_m"]:
                previous = drift_since.get(key)
                begin = previous[0] if previous and t - previous[1] <= gap else t
                drift_since[key] = (begin, t)
                if t - begin >= thresholds["drift_duration_s"] * 1e9:
                    hit("drift", e, [i, j], "receiver_cache_position_error_sustained", error)
            else:
                drift_since.pop(key, None)
    if run_status == "task_budget_exhausted":
        hit("delay_timeout", {"observed_ns": end_ns}, ids, "configured_task_budget_exhausted", cfg["task_budget_s"])

    def covered(i):
        ts = samples[i]
        return bool(ts and ts[0] - start_ns <= gap and end_ns - ts[-1] <= gap and
                    all(b - a <= gap for a, b in zip(ts, ts[1:])))

    full = all(covered(i) for i in ids)
    operational_end = run_status in ("completed", "task_budget_exhausted")
    for name, label in labels.items():
        available = full and operational_end and telemetry_complete
        reason = "incomplete_or_gapped_post_injection_observation"
        if name == "obstacle_collision" and voxel_map is None:
            available, reason = False, "static_map_unavailable"
        if name == "planning_failure" and not event_counts["plan"]:
            available, reason = False, "no_post_injection_planner_return_observed"
        if name == "drift" and not any(e.get("received_states") for e in rows if e["kind"] == "snapshot"):
            available, reason = False, "receiver_cache_unavailable"
        if name in ("drift", "delay_timeout") and not cache_complete:
            available, reason = False, "receiver_cache_missing_for_active_neighbor"
        label["valid"] = label["hit"] or available
        label["missing_reason"] = "" if label["valid"] else reason
    return labels
