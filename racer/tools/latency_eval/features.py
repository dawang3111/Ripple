import math
from itertools import combinations

import numpy as np

from .core import Trajectory


def norm(v):
    return float(np.linalg.norm(v))


def angle(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def extract(events, cutoff_ns, cutoff_ros_s, source_id, cfg, voxel_map=None):
    # Observer receipt is the information boundary. A future plan's start_s is a
    # model evaluation time, not its availability time. Never filter it as a measurement.
    past = [e for e in events if e["observed_ns"] <= cutoff_ns and
            e.get("mono_ns", e["observed_ns"]) <= cutoff_ns]
    tol = cfg["alignment_tolerance_s"]
    snaps = [e for e in past if e["kind"] == "snapshot" and e["ros_s"] <= cutoff_ros_s]
    trajectories = [e for e in past if e["kind"] == "trajectory"]

    def snapshot(i, t):
        candidates = [e for e in snaps if e["id"] == i and 0 <= t - e["ros_s"] <= tol and
                      0 <= t - e.get("position_stamp_s", e["ros_s"]) <= tol]
        return max(candidates, key=lambda e: e["ros_s"]) if candidates else None

    def trajectory(i, t):
        # The latest published plan supersedes earlier plans. Do not resurrect an
        # old plan merely because the new one does not cover the requested horizon.
        es = [e for e in trajectories if e["id"] == i]
        if not es:
            raise ValueError("trajectory_unavailable")
        e = max(es, key=lambda x: x["observed_ns"])
        tr = Trajectory(e)
        return tr, e

    out = {}
    local_complete = False

    def add(name, value, unit, rows=(), source="snapshot", reason="missing_or_stale_pre_injection_observation"):
        if (name.startswith(("local_", "victim_", "source_victim_")) or name == "source_truth_broadcast_position_error_max") and not local_complete:
            value, reason = None, "local_membership_unknown_due_to_missing_peer_snapshot"
        missing = value is None or not math.isfinite(float(value))
        rs = list(rows)
        out[name] = {"value": None if missing else float(value), "unit": unit,
                     "missing": missing, "missing_reason": reason if missing else "",
                     "source": source,
                     "observed_ns": [min(x["observed_ns"] for x in rs), max(x["observed_ns"] for x in rs)] if rs else None,
                     "source_ros_s": [min(x.get("position_stamp_s", x.get("ros_s", x.get("start_s", cutoff_ros_s))) for x in rs),
                                      max(x.get("ros_s", x.get("start_s", cutoff_ros_s)) for x in rs)] if rs else None}

    current = {i: snapshot(i, cutoff_ros_s) for i in cfg["active_ids"]}
    local_complete = all(e is not None for e in current.values())
    source = current.get(source_id)
    local = [source_id] if source else []
    if source:
        local += [i for i, e in current.items() if i != source_id and e is not None and
                  norm(np.array(e["position"]) - source["position"]) <= cfg["local_radius_m"]]
    rows = [current[i] for i in local]
    victim = min((i for i in local if i != source_id),
                 key=lambda i: norm(np.array(current[i]["position"]) - source["position"]), default=None)
    old_t = cutoff_ros_s - cfg["history_s"]
    old = {i: snapshot(i, old_t) for i in local}
    for field, unit in [("velocity", "m/s"), ("acceleration", "m/s^2")]:
        add("source_" + field + "_norm", norm(source[field]) if source else None, unit, [source] if source else [])
        values = [norm(r[field]) for r in rows if field in r]
        add("local_max_" + field, max(values) if len(values) == len(rows) and values else None, unit, rows)
        add("local_mean_" + field, np.mean(values) if len(values) == len(rows) and values else None, unit, rows)
    displacement = norm(np.array(source["position"]) - old[source_id]["position"]) if source and old.get(source_id) else None
    add("source_displacement_5s", displacement, "m", [source, old[source_id]] if displacement is not None else [])
    turning = angle(source["yaw"], old[source_id]["yaw"]) if source and old.get(source_id) else None
    add("source_heading_change_5s", turning, "rad", [source, old[source_id]] if turning is not None else [])
    old_rows = [old[i] for i in local if old.get(i)]
    history_complete = bool(rows) and len(old_rows) == len(rows)
    add("local_max_displacement_5s", max(norm(np.array(current[i]["position"]) - old[i]["position"]) for i in local) if history_complete else None,
        "m", rows + old_rows)
    add("local_max_heading_change_5s", max(angle(current[i]["yaw"], old[i]["yaw"]) for i in local) if history_complete else None,
        "rad", rows + old_rows)
    pairs = list(combinations(local, 2))

    def minimum_distance(states):
        if not pairs or any(states.get(i) is None for i in local):
            return None
        return min(norm(np.array(states[i]["position"]) - states[j]["position"]) for i, j in pairs)

    cur_dist, old_dist = minimum_distance(current), minimum_distance(old)
    add("local_min_inter_drone_distance", cur_dist, "m", rows)
    add("local_min_distance_change_5s", cur_dist - old_dist if cur_dist is not None and old_dist is not None else None,
        "m", rows + [e for e in old.values() if e])
    add("source_victim_heading_angle", angle(source["yaw"], current[victim]["yaw"]) if victim is not None else None,
        "rad", [source, current[victim]] if victim is not None else [])
    add("local_max_heading_angle", max(angle(current[i]["yaw"], current[j]["yaw"]) for i, j in pairs) if pairs else None,
        "rad", rows)

    future_rows, future, future_errors = [], {}, {}
    ts = np.linspace(cutoff_ros_s, cutoff_ros_s + cfg["future_horizon_s"],
                     int(round(cfg["future_horizon_s"] / cfg["future_step_s"])) + 1)
    for i in local:
        try:
            tr, e = trajectory(i, cutoff_ros_s)
            future[i] = [tr.at(t) for t in ts]
            future_rows.append(e)
        except ValueError as exc:
            future_errors[i] = str(exc)
    future_dist = min(norm(future[i][k] - future[j][k]) for i, j in pairs for k in range(len(ts))) if pairs and len(future) == len(local) else None
    future_obs = min(voxel_map.distance(p) for p in future[source_id]) if voxel_map and source_id in future else None
    future_reason = "missing_pair_or_incomplete_3s_trajectory:" + str(future_errors)
    add("local_future_3s_min_inter_drone_distance", future_dist, "m", future_rows,
        "pre_injection_published_bspline_sampled", future_reason)
    add("source_future_3s_min_obstacle_distance", future_obs, "m", future_rows,
        "pre_injection_published_bspline_and_static_pcd_voxels", future_reason)
    add("source_current_obstacle_distance", voxel_map.distance(source["position"]) if voxel_map and source else None,
        "m", [source] if source else [], "static_pcd_voxels", "static_map_or_source_unavailable")
    add("source_planner_obstacle_distance", source.get("obstacle_distance") if source else None,
        "m", [source] if source else [], "queried_local_sdf_map_time_unknown", "unknown_map_cell_or_missing_snapshot")

    def deviation(i):
        if i is None or current.get(i) is None:
            return None, []
        try:
            tr, e = trajectory(i, cutoff_ros_s)
            return norm(np.array(current[i]["position"]) - tr.at(current[i]["ros_s"])), [current[i], e]
        except ValueError:
            return None, []

    for who, i in [("source", source_id), ("victim", victim)]:
        val, evidence = deviation(i)
        add(who + "_trajectory_deviation", val, "m", evidence, "simulation_vs_pre_injection_local_bspline")
    deviations = [deviation(i) for i in local]
    add("local_max_trajectory_deviation", max(x[0] for x in deviations) if deviations and all(x[0] is not None for x in deviations) else None,
        "m", [e for _, evs in deviations for e in evs], "simulation_vs_pre_injection_local_bspline")
    recent = [e for e in past if cutoff_ns - int(cfg["history_s"] * 1e9) <= e["observed_ns"] and e.get("id") in local]
    # Counts are missing until a full history window has been observed.
    starts = [e for e in past if e["kind"] == "task_started" and e.get("id") == source_id]
    elapsed = cutoff_ros_s - starts[0]["ros_s"] if starts else None
    full_history = elapsed is not None and elapsed >= cfg["history_s"]
    telemetry = {}
    history_telemetry_complete = True
    for e in recent:
        if "event_seq" in e:
            previous = telemetry.get(e["id"])
            if previous is not None and e["event_seq"] != previous + 1:
                history_telemetry_complete = False
            telemetry[e["id"]] = e["event_seq"]
    counts = {
        "local_replan_count_5s": sum(e["kind"] == "fsm" and e.get("fsm") == "PLAN_TRAJ" for e in recent),
        "local_safety_replan_count_5s": sum(e["kind"] in ("inter_drone_prediction", "obstacle_prediction") for e in recent),
        "local_new_trajectory_count_5s": len({(e["id"], e["traj_id"]) for e in recent if e["kind"] == "trajectory"}),
        "local_replans_after_trajectory_publication_5s": sum(e["kind"] == "fsm" and e.get("previous") == "EXEC_TRAJ" and
                                                          e.get("fsm") == "PLAN_TRAJ" for e in recent)}
    for name, value in counts.items():
        add(name, value if full_history and history_telemetry_complete else None, "count", recent, "pre_injection_events", "history_shorter_than_5s_or_telemetry_gap")
    add("task_elapsed", elapsed, "s", starts[:1], "source_task_started")
    for name in ["active_grid_count", "frontier_count"]:
        add("source_" + name, source.get(name) if source else None, "count", [source] if source else [])
    errors, evidence = [], []
    for r in rows:
        for cached in r.get("received_states", []):
            if cached["id"] == source_id and r["id"] != source_id and source:
                errors.append(norm(np.array(cached["position"]) - source["position"]))
                evidence.append(r)
    add("source_truth_broadcast_position_error_max", max(errors) if errors else None,
        "m", evidence + ([source] if source else []), "receiver_cache_vs_current_simulation_position")
    return {"source_id": source_id, "victim_id": victim, "local_ids": local, "local_membership_complete": local_complete,
            "cutoff_ns": cutoff_ns, "cutoff_ros_s": cutoff_ros_s, "fields": out}
