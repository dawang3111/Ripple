#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import re
import threading
from collections import deque
from datetime import datetime
from typing import Any, Deque, Dict, List, Optional, Tuple

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import String
from traj_utils.msg import PolyTraj


SCENARIOS = [
    (1, "drone_drone_collision"),
    (2, "drone_obstacle_collision"),
    (3, "deadlock"),
    (4, "delay_timeout"),
    (5, "planning_failure"),
    (6, "drift"),
]

TASK_ACTIVE_FSM_STATES = set(["SEQUENTIAL_START", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"])
TASK_INACTIVE_FSM_STATES = set(["INIT", "WAIT_TARGET", "WAIT_TRIGGER"])


def _now_iso() -> str:
    return datetime.now().isoformat()


def _dist(a: List[float], b: List[float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _sub(a: List[float], b: List[float]) -> List[float]:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _dot(a: List[float], b: List[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _fmt_value(value: float) -> str:
    if value is None:
        return "NA"
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    return "{:.6f}".format(value)


def _parse_drone_id(frame_id: str) -> Optional[int]:
    m = re.search(r"drone[_-](\d+)", frame_id or "")
    if not m:
        return None
    try:
        return int(m.group(1))
    except Exception:
        return None


class PolyTrajectory:
    def __init__(self, msg: PolyTraj) -> None:
        self.drone_id = int(msg.drone_id)
        self.traj_id = int(msg.traj_id)
        self.start_time = float(msg.start_time.to_sec())
        self.order = int(msg.order)
        self.duration = [float(x) for x in msg.duration]
        self.coef_x = [float(x) for x in msg.coef_x]
        self.coef_y = [float(x) for x in msg.coef_y]
        self.coef_z = [float(x) for x in msg.coef_z]
        self.total_duration = sum(self.duration)
        self.valid = self._is_valid()

    def _is_valid(self) -> bool:
        n = len(self.duration) * (self.order + 1)
        return self.order == 5 and n > 0 and len(self.coef_x) == n and len(self.coef_y) == n and len(self.coef_z) == n

    def pos_at_abs_time(self, abs_time: float) -> Optional[List[float]]:
        if not self.valid:
            return None
        rel = abs_time - self.start_time
        if rel < 0.0 or rel > self.total_duration:
            return None

        piece_t = rel
        piece_idx = 0
        for i, dur in enumerate(self.duration):
            if piece_t <= dur or i == len(self.duration) - 1:
                piece_idx = i
                break
            piece_t -= dur

        return [
            self._eval_axis(self.coef_x, piece_idx, piece_t),
            self._eval_axis(self.coef_y, piece_idx, piece_t),
            self._eval_axis(self.coef_z, piece_idx, piece_t),
        ]

    def _eval_axis(self, coef: List[float], piece_idx: int, t: float) -> float:
        offset = piece_idx * (self.order + 1)
        val = 0.0
        for j in range(self.order + 1):
            val += coef[offset + j] * (t ** (self.order - j))
        return val


class STLAttackMetricsMonitor:
    def __init__(self) -> None:
        self.drone_num = int(rospy.get_param("~drone_num", 10))
        self.odom_suffix = str(rospy.get_param("~odom_suffix", "visual_slam/odom")).strip("/")
        self.obstacle_topic = str(rospy.get_param("~obstacle_topic", "/map_generator/global_cloud"))
        self.report_dir = os.path.expanduser(str(rospy.get_param("~report_dir", "~/.ego_fuzz_reports")).strip())
        self.run_id = str(rospy.get_param("~run_id", "")).strip() or datetime.now().strftime("%Y%m%d_%H%M%S")
        self.check_hz = float(rospy.get_param("~check_hz", 2.0))
        self.summary_period_sec = float(rospy.get_param("~summary_period_sec", 1.0))
        self.window_sec = float(rospy.get_param("~window_sec", 5.0))
        self.move_threshold_x = float(rospy.get_param("~move_threshold_x", 0.2))
        self.delay_timeout_threshold_s = float(rospy.get_param("~delay_timeout_threshold_s", 120.0))
        self.active_speed_eps = float(rospy.get_param("~active_speed_eps", 0.05))
        self.start_grace_sec = float(rospy.get_param("~start_grace_sec", 2.0))
        self.drift_eps = float(rospy.get_param("~drift_eps", 0.25))
        self.drift_max_age_sec = float(rospy.get_param("~drift_max_age_sec", 0.3))
        self.future_horizon_sec = float(rospy.get_param("~future_horizon_sec", 3.0))
        self.future_sample_dt = float(rospy.get_param("~future_sample_dt", 0.1))
        self.drone_radius = float(rospy.get_param("~drone_radius", 0.1))
        self.drone_margin = float(rospy.get_param("~drone_collision_margin", 0.0))
        self.obstacle_margin = float(rospy.get_param("~obstacle_collision_margin", 0.0))
        self.obstacle_voxel = float(rospy.get_param("~obstacle_voxel", 0.1))
        self.obstacle_search_radius = float(rospy.get_param("~obstacle_search_radius", 1.0))

        self.run_path = os.path.join(self.report_dir, self.run_id)
        os.makedirs(self.run_path, exist_ok=True)
        self.summary_json_path = os.path.join(self.run_path, "stl_metrics_summary.json")
        self.trace_jsonl_path = os.path.join(self.run_path, "stl_metrics_trace.jsonl")

        self._lock = threading.Lock()
        self._last_summary_write = 0.0
        self._start_ros_time = rospy.Time.now().to_sec()
        self._task_start_ros_time = None
        self._task_complete_ros_time = None
        self._monitoring_started = False
        self._start_wall_time = _now_iso()
        self.drone_task_start_time: Dict[int, float] = {}
        self.drone_task_completed_time: Dict[int, float] = {}

        self.odom_state: Dict[int, Dict[str, Any]] = {}
        self.odom_windows: Dict[int, Deque[Dict[str, Any]]] = {i: deque() for i in range(self.drone_num)}
        self.broadcast_odom: Dict[int, Dict[str, Any]] = {}
        self.fsm_state: Dict[int, Dict[str, Any]] = {}
        self.occupancy = set()
        self.local_trajs: Dict[int, PolyTrajectory] = {}
        self.drift_active: Dict[int, bool] = {}
        self.drift_stats: Dict[int, Dict[str, Any]] = {i: self._empty_drift_stats() for i in range(self.drone_num)}

        self.metrics: Dict[int, Dict[str, Any]] = self._initial_metrics()

        self._subs = []
        for i in range(self.drone_num):
            topic = "/drone_{}_{}".format(i, self.odom_suffix)
            self._subs.append(rospy.Subscriber(topic, Odometry, self._odom_cb, callback_args=i, queue_size=30))
            traj_topic = "/drone_{}_planning/trajectory".format(i)
            self._subs.append(rospy.Subscriber(traj_topic, PolyTraj, self._traj_cb, callback_args=i, queue_size=50))
            fsm_topic = "/drone_{}_planning/fsm_state".format(i)
            self._subs.append(rospy.Subscriber(fsm_topic, String, self._fsm_state_cb, callback_args=i, queue_size=10))
        self._subs.append(rospy.Subscriber("/others_odom", Odometry, self._broadcast_odom_cb, queue_size=200))
        self._subs.append(rospy.Subscriber(self.obstacle_topic, PointCloud2, self._obstacle_cb, queue_size=1))

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.check_hz, 1.0)), self._check_cb)
        rospy.on_shutdown(self._on_shutdown)
        self._write_outputs("startup")
        rospy.loginfo("[stl_attack_metrics_monitor] started, run_id=%s", self.run_id)

    def _initial_metrics(self) -> Dict[int, Dict[str, Any]]:
        return {
            1: self._metric("drone_drone_collision", float("inf")),
            2: self._metric("drone_obstacle_collision", float("inf")),
            3: self._metric("deadlock", float("inf")),
            4: self._metric("delay_timeout", 0.0),
            5: self._metric("planning_failure", float("inf")),
            6: self._metric("drift", 0.0),
        }

    def _metric(self, name: str, value: float) -> Dict[str, Any]:
        return {
            "scenario_name": name,
            "stl_value": value,
            "attacker_drone_id": "NA",
            "victim_drone_id": "NA",
            "bystander_drone_ids": "NA",
            "victim_fsm_state": "NA",
            "victim_fsm_state_time": "NA",
            "evidence_time": "NA",
            "evidence_pair": "NA",
            "evidence": {},
        }

    def _empty_drift_stats(self) -> Dict[str, Any]:
        return {
            "event_count": 0,
            "max_distance": 0.0,
            "max_time": "NA",
            "actual_position": [],
            "broadcast_position": [],
            "attacker_drone_id": "NA",
            "attacker_correlation_distance": "NA",
        }

    def _mark_task_started_locked(self, now: float, reason: str) -> None:
        if self._monitoring_started:
            return
        self._monitoring_started = True
        self._task_start_ros_time = now
        self._task_complete_ros_time = None
        self.metrics = self._initial_metrics()
        self.drift_active = {}
        self.drift_stats = {i: self._empty_drift_stats() for i in range(self.drone_num)}
        self.drone_task_completed_time = {}
        for win in self.odom_windows.values():
            win.clear()
        rospy.loginfo("[stl_attack_metrics_monitor] armed at task start, reason=%s", reason)

    def _mark_drone_active_locked(self, drone_id: int, now: float, state_name: str) -> None:
        previous = str(self.fsm_state.get(drone_id, {}).get("state", "NA"))
        if previous in TASK_ACTIVE_FSM_STATES:
            return
        self.drone_task_start_time[drone_id] = now
        self.drone_task_completed_time.pop(drone_id, None)
        self.drift_active[drone_id] = False
        self.odom_windows.setdefault(drone_id, deque()).clear()
        rospy.loginfo(
            "[stl_attack_metrics_monitor] drone %d active, state=%s, previous=%s",
            drone_id,
            state_name,
            previous,
        )

    def _odom_cb(self, msg: Odometry, drone_id: int) -> None:
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        now = rospy.Time.now().to_sec()
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else now
        item = {"t": now, "stamp": stamp, "p": [pos.x, pos.y, pos.z], "v": [vel.x, vel.y, vel.z]}
        with self._lock:
            self.odom_state[drone_id] = item
            win = self.odom_windows.setdefault(drone_id, deque())
            win.append(item)
            cut = now - max(self.window_sec, 0.1)
            while win and win[0]["t"] < cut:
                win.popleft()

    def _broadcast_odom_cb(self, msg: Odometry) -> None:
        drone_id = _parse_drone_id(msg.child_frame_id)
        if drone_id is None or drone_id < 0 or drone_id >= self.drone_num:
            return
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        now = rospy.Time.now().to_sec()
        item = {"t": now, "p": [pos.x, pos.y, pos.z], "v": [vel.x, vel.y, vel.z]}
        with self._lock:
            self.broadcast_odom[drone_id] = item
            actual = self.odom_state.get(drone_id)
            if not self._monitoring_started:
                return
            if not self._drone_evidence_active(drone_id):
                self.drift_active[drone_id] = False
                return
            if actual and abs(now - float(actual.get("t", now))) <= self.drift_max_age_sec:
                drift = _dist(item["p"], actual["p"])
                if drift > self.drift_eps:
                    if not self.drift_active.get(drone_id, False):
                        self.metrics[6]["stl_value"] = float(self.metrics[6]["stl_value"]) + 1.0
                        self.drift_stats[drone_id]["event_count"] = int(self.drift_stats[drone_id]["event_count"]) + 1
                    self.drift_active[drone_id] = True
                    self._set_drift_metric(drone_id, now, drift, actual["p"], item["p"])
                else:
                    self.drift_active[drone_id] = False

    def _traj_cb(self, msg: PolyTraj, drone_id: int) -> None:
        traj = PolyTrajectory(msg)
        if not traj.valid:
            return
        with self._lock:
            self.local_trajs[drone_id] = traj
            self._mark_task_started_locked(rospy.Time.now().to_sec(), "trajectory")

    def _fsm_state_cb(self, msg: String, drone_id: int) -> None:
        state_name = str(msg.data or "NA")
        now = rospy.Time.now().to_sec()
        with self._lock:
            previous = str(self.fsm_state.get(drone_id, {}).get("state", "NA"))
            if state_name in TASK_ACTIVE_FSM_STATES:
                self._mark_drone_active_locked(drone_id, now, state_name)
            elif state_name in TASK_INACTIVE_FSM_STATES:
                if state_name == "WAIT_TARGET" and previous in TASK_ACTIVE_FSM_STATES:
                    self._mark_drone_completed_locked(drone_id, now)
                self.drift_active[drone_id] = False
                self.odom_windows.setdefault(drone_id, deque()).clear()
            self.fsm_state[drone_id] = {
                "state": state_name,
                "t": now,
            }
            if state_name in TASK_ACTIVE_FSM_STATES:
                self._mark_task_started_locked(now, "fsm_state:{}".format(state_name))

    def _obstacle_cb(self, msg: PointCloud2) -> None:
        local = set()
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            local.add(self._quantize(float(p[0]), float(p[1]), float(p[2])))
        with self._lock:
            self.occupancy = local

    def _check_cb(self, _event: Any) -> None:
        now = rospy.Time.now().to_sec()
        with self._lock:
            if self._monitoring_started:
                self._update_drone_drone_margin(now)
                self._update_drone_obstacle_margin(now)
                self._update_window_motion(now)
                self._update_task_elapsed_delay(now)
                self._update_future_traj_margin(now)
            snapshot = self._snapshot_locked("periodic")

        self._append_trace(snapshot)
        if now - self._last_summary_write >= self.summary_period_sec:
            self._write_snapshot(snapshot)
            self._last_summary_write = now

    def _update_drone_drone_margin(self, now: float) -> None:
        ids = sorted(self.odom_state.keys())
        threshold = 2.0 * self.drone_radius + self.drone_margin
        for idx, a_id in enumerate(ids):
            if not self._drone_evidence_active(a_id):
                continue
            for b_id in ids[idx + 1 :]:
                if not self._drone_evidence_active(b_id):
                    continue
                a = self.odom_state[a_id]
                b = self.odom_state[b_id]
                dist = _dist(a["p"], b["p"])
                margin = dist - threshold
                if margin < float(self.metrics[1]["stl_value"]):
                    attacker, victim = self._choose_attacker_victim(a_id, b_id, a, b)
                    self._set_pair_metric(1, attacker, victim, now, margin, "{}-{}".format(a_id, b_id), dist)

    def _update_drone_obstacle_margin(self, now: float) -> None:
        if not self.occupancy:
            return
        threshold = self.drone_radius + self.obstacle_margin
        for drone_id, state in self.odom_state.items():
            if not self._drone_evidence_active(drone_id):
                continue
            nearest = self._nearest_obstacle_distance(state["p"])
            if nearest is None:
                continue
            margin = nearest - threshold
            if margin < float(self.metrics[2]["stl_value"]):
                self._set_single_drone_metric(2, drone_id, drone_id, now, margin, "drone_obstacle")

    def _update_window_motion(self, now: float) -> None:
        for drone_id, win in self.odom_windows.items():
            if len(win) < 2:
                continue
            if not self._drone_task_active(drone_id, now):
                continue
            duration = float(win[-1]["t"]) - float(win[0]["t"])
            if duration < self.window_sec * 0.8:
                continue
            move = self._window_path_len(win)
            if move < float(self.metrics[3]["stl_value"]):
                attacker = self._nearest_other_drone(drone_id, active_only=True)
                self._set_single_drone_metric(3, attacker, drone_id, now, move, "window_motion")

    def _mark_drone_completed_locked(self, drone_id: int, now: float) -> None:
        if not self._monitoring_started:
            return
        self.drone_task_completed_time[drone_id] = now
        if len(self.drone_task_completed_time) >= self.drone_num and self._task_complete_ros_time is None:
            self._task_complete_ros_time = max(self.drone_task_completed_time.values())

    def _update_task_elapsed_delay(self, now: float) -> None:
        if not self._monitoring_started or self._task_start_ros_time is None:
            return
        end_time = self._task_complete_ros_time if self._task_complete_ros_time is not None else now
        elapsed = max(0.0, float(end_time) - float(self._task_start_ros_time))
        victim = self._slowest_task_drone(now)
        attacker = self._nearest_other_drone(victim, active_only=True) if victim != "NA" else "NA"
        self._set_single_drone_metric(
            4,
            attacker,
            victim,
            end_time,
            elapsed,
            "task_completion_time",
            extra={
                "elapsed_s": elapsed,
                "threshold_s": self.delay_timeout_threshold_s,
                "task_complete": self._task_complete_ros_time is not None,
            },
        )

    def _slowest_task_drone(self, now: float) -> Any:
        active_ids = [
            drone_id
            for drone_id in range(self.drone_num)
            if self.fsm_state.get(drone_id, {}).get("state", "NA") in TASK_ACTIVE_FSM_STATES
        ]
        if active_ids:
            return max(active_ids, key=lambda drone_id: now - self.drone_task_start_time.get(drone_id, self._task_start_ros_time or now))
        if self.drone_task_completed_time:
            return max(self.drone_task_completed_time, key=lambda drone_id: self.drone_task_completed_time[drone_id])
        return "NA"

    def _update_future_traj_margin(self, now: float) -> None:
        active_ids = [
            i
            for i, traj in self.local_trajs.items()
            if self._drone_evidence_active(i) and traj.pos_at_abs_time(now) is not None
        ]
        if not active_ids:
            return
        threshold_pair = 2.0 * self.drone_radius + self.drone_margin
        threshold_obs = self.drone_radius + self.obstacle_margin
        steps = max(1, int(math.ceil(max(self.future_horizon_sec, 0.0) / max(self.future_sample_dt, 0.05))))

        for k in range(steps + 1):
            t_abs = now + k * self.future_sample_dt
            positions: Dict[int, List[float]] = {}
            for drone_id in active_ids:
                p = self.local_trajs[drone_id].pos_at_abs_time(t_abs)
                if p is not None:
                    positions[drone_id] = p

            ids = sorted(positions.keys())
            for idx, a_id in enumerate(ids):
                for b_id in ids[idx + 1 :]:
                    dist = _dist(positions[a_id], positions[b_id])
                    margin = dist - threshold_pair
                    if dist < float(self.metrics[5]["stl_value"]):
                        self._set_pair_metric(5, a_id, b_id, now, dist, "{}-{}".format(a_id, b_id), dist, margin)

            if not self.occupancy:
                continue
            for drone_id, p in positions.items():
                nearest = self._nearest_obstacle_distance(p)
                if nearest is None:
                    continue
                margin = nearest - threshold_obs
                if nearest < float(self.metrics[5]["stl_value"]):
                    self._set_single_drone_metric(
                        5,
                        drone_id,
                        drone_id,
                        now,
                        nearest,
                        "planned_obstacle",
                        extra={"margin": margin, "threshold": threshold_obs},
                    )

    def _drone_task_active(self, drone_id: int, now: float) -> bool:
        if not self._drone_evidence_active(drone_id):
            return False
        start_time = self.drone_task_start_time.get(
            drone_id,
            self._task_start_ros_time if self._task_start_ros_time is not None else self._start_ros_time,
        )
        if now - start_time < self.start_grace_sec:
            return False
        return True

    def _drone_evidence_active(self, drone_id: int) -> bool:
        state = self.fsm_state.get(drone_id)
        if state:
            name = str(state.get("state", "NA"))
            if name in TASK_ACTIVE_FSM_STATES:
                return True
            if name in TASK_INACTIVE_FSM_STATES:
                return False
            return False
        traj = self.local_trajs.get(drone_id)
        return bool(traj and traj.pos_at_abs_time(rospy.Time.now().to_sec()) is not None)

    def _window_path_len(self, win: Deque[Dict[str, Any]]) -> float:
        total = 0.0
        prev = None
        for item in win:
            if prev is not None:
                total += _dist(prev["p"], item["p"])
            prev = item
        return total

    def _nearest_obstacle_distance(self, p: List[float]) -> Optional[float]:
        c = self._quantize(p[0], p[1], p[2])
        r = int(math.ceil(max(self.obstacle_search_radius, self.drone_radius + self.obstacle_margin) / self.obstacle_voxel))
        best = None
        for ix in range(c[0] - r, c[0] + r + 1):
            for iy in range(c[1] - r, c[1] + r + 1):
                for iz in range(c[2] - r, c[2] + r + 1):
                    key = (ix, iy, iz)
                    if key not in self.occupancy:
                        continue
                    o = self._dequantize(key)
                    d = _dist(p, o)
                    if best is None or d < best:
                        best = d
        if best is None:
            return self.obstacle_search_radius
        return best

    def _nearest_other_drone(
        self, victim_id: int, reference_position: Optional[List[float]] = None, active_only: bool = False
    ) -> Any:
        best_id, _best_d = self._nearest_other_drone_with_distance(victim_id, reference_position, active_only)
        return best_id

    def _nearest_other_drone_with_distance(
        self, victim_id: int, reference_position: Optional[List[float]] = None, active_only: bool = False
    ) -> Tuple[Any, Any]:
        victim = self.odom_state.get(victim_id)
        if not victim and reference_position is None:
            return "NA", "NA"
        ref = reference_position if reference_position is not None else victim["p"]
        best_id = "NA"
        best_d = float("inf")
        for drone_id, state in self.odom_state.items():
            if drone_id == victim_id:
                continue
            if active_only and not self._drone_evidence_active(drone_id):
                continue
            d = _dist(ref, state["p"])
            if d < best_d:
                best_d = d
                best_id = drone_id
        return best_id, best_d if best_id != "NA" else "NA"

    def _choose_attacker_victim(self, a_id: int, b_id: int, a: Dict[str, Any], b: Dict[str, Any]) -> Tuple[int, int]:
        ab = _sub(b["p"], a["p"])
        d = math.sqrt(max(_dot(ab, ab), 1e-12))
        unit_ab = [ab[0] / d, ab[1] / d, ab[2] / d]
        unit_ba = [-unit_ab[0], -unit_ab[1], -unit_ab[2]]
        a_closing = _dot(a.get("v", [0.0, 0.0, 0.0]), unit_ab)
        b_closing = _dot(b.get("v", [0.0, 0.0, 0.0]), unit_ba)
        if abs(a_closing - b_closing) < 1e-6:
            return a_id, b_id
        return (a_id, b_id) if a_closing > b_closing else (b_id, a_id)

    def _set_pair_metric(
        self,
        scenario_id: int,
        attacker: Any,
        victim: Any,
        now: float,
        value: float,
        pair: str,
        raw_dist: float,
        margin: Optional[float] = None,
    ) -> None:
        m = self.metrics[scenario_id]
        m["stl_value"] = value
        m["attacker_drone_id"] = str(attacker)
        m["victim_drone_id"] = str(victim)
        m["bystander_drone_ids"] = self._bystanders([attacker, victim])
        self._set_victim_fsm(m, victim)
        m["evidence_time"] = "{:.6f}".format(now)
        m["evidence_pair"] = pair
        m["evidence"] = {"raw_distance": raw_dist}
        if margin is not None:
            m["evidence"]["margin"] = margin

    def _set_single_drone_metric(
        self,
        scenario_id: int,
        attacker: Any,
        victim: Any,
        now: float,
        value: float,
        evidence_pair: str,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        m = self.metrics[scenario_id]
        m["stl_value"] = value
        m["attacker_drone_id"] = str(attacker)
        m["victim_drone_id"] = str(victim)
        m["bystander_drone_ids"] = self._bystanders([attacker, victim])
        self._set_victim_fsm(m, victim)
        m["evidence_time"] = "{:.6f}".format(now)
        m["evidence_pair"] = evidence_pair
        m["evidence"] = {"threshold_x": self.move_threshold_x, "drift_eps": self.drift_eps}
        if extra:
            m["evidence"].update(extra)

    def _set_drift_metric(
        self,
        victim_drone_id: int,
        now: float,
        drift_distance: float,
        actual_position: List[float],
        broadcast_position: List[float],
    ) -> None:
        stats = self.drift_stats[victim_drone_id]
        if drift_distance <= float(stats.get("max_distance", 0.0)):
            return
        attacker, corr_dist = self._nearest_other_drone_with_distance(victim_drone_id, broadcast_position, active_only=True)
        stats["max_distance"] = drift_distance
        stats["max_time"] = "{:.6f}".format(now)
        stats["actual_position"] = list(actual_position)
        stats["broadcast_position"] = list(broadcast_position)
        stats["attacker_drone_id"] = str(attacker)
        stats["attacker_correlation_distance"] = corr_dist

        m = self.metrics[6]
        m["attacker_drone_id"] = str(attacker)
        m["victim_drone_id"] = str(victim_drone_id)
        m["bystander_drone_ids"] = self._bystanders([attacker, victim_drone_id])
        self._set_victim_fsm(m, victim_drone_id)
        stats["victim_fsm_state"] = str(m.get("victim_fsm_state", "NA"))
        stats["victim_fsm_state_time"] = str(m.get("victim_fsm_state_time", "NA"))
        m["evidence_time"] = "{:.6f}".format(now)
        m["evidence_pair"] = "actual_vs_broadcast"
        m["evidence"] = {
            "drift_distance": drift_distance,
            "drift_eps": self.drift_eps,
            "victim_actual_position": list(actual_position),
            "victim_broadcast_position": list(broadcast_position),
            "attacker_correlation_distance": corr_dist,
        }

    def _set_victim_fsm(self, metric: Dict[str, Any], victim: Any) -> None:
        try:
            victim_id = int(str(victim).split("|")[0])
        except Exception:
            metric["victim_fsm_state"] = "NA"
            metric["victim_fsm_state_time"] = "NA"
            return
        state = self.fsm_state.get(victim_id)
        if not state:
            metric["victim_fsm_state"] = "NA"
            metric["victim_fsm_state_time"] = "NA"
            return
        metric["victim_fsm_state"] = str(state.get("state", "NA"))
        t = state.get("t", "NA")
        metric["victim_fsm_state_time"] = "{:.6f}".format(t) if isinstance(t, (int, float)) else str(t)

    def _bystanders(self, involved: List[Any]) -> str:
        involved_ids = set()
        for item in involved:
            if isinstance(item, int):
                involved_ids.add(item)
                continue
            for part in str(item).split("|"):
                try:
                    involved_ids.add(int(part))
                except Exception:
                    pass
        out = [str(i) for i in range(self.drone_num) if i not in involved_ids]
        return "|".join(out) if out else "NA"

    def _quantize(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        r = self.obstacle_voxel
        return (int(round(x / r)), int(round(y / r)), int(round(z / r)))

    def _dequantize(self, key: Tuple[int, int, int]) -> List[float]:
        r = self.obstacle_voxel
        return [key[0] * r, key[1] * r, key[2] * r]

    def _snapshot_locked(self, reason: str) -> Dict[str, Any]:
        rows = []
        for scenario_id, name in SCENARIOS:
            m = self.metrics[scenario_id]
            rows.append(
                {
                    "scenario_id": scenario_id,
                    "scenario_name": name,
                    "stl_value": _fmt_value(float(m["stl_value"])),
                    "attacker_drone_id": str(m["attacker_drone_id"]),
                    "victim_drone_id": str(m["victim_drone_id"]),
                    "bystander_drone_ids": str(m["bystander_drone_ids"]),
                    "victim_fsm_state": str(m.get("victim_fsm_state", "NA")),
                }
            )
        details = {}
        for scenario_id, metric in self.metrics.items():
            details[str(scenario_id)] = {
                "scenario_name": metric.get("scenario_name", "unknown"),
                "stl_value": _fmt_value(float(metric.get("stl_value", float("inf")))),
                "attacker_drone_id": str(metric.get("attacker_drone_id", "NA")),
                "victim_drone_id": str(metric.get("victim_drone_id", "NA")),
                "bystander_drone_ids": str(metric.get("bystander_drone_ids", "NA")),
                "victim_fsm_state": str(metric.get("victim_fsm_state", "NA")),
                "victim_fsm_state_time": str(metric.get("victim_fsm_state_time", "NA")),
                "evidence_time": str(metric.get("evidence_time", "NA")),
                "evidence_pair": str(metric.get("evidence_pair", "NA")),
                "evidence": dict(metric.get("evidence", {})),
            }
        details["6"]["drift_by_drone"] = {
            str(drone_id): dict(stats) for drone_id, stats in sorted(self.drift_stats.items())
        }
        return {
            "reason": reason,
            "run_id": self.run_id,
            "wall_time": _now_iso(),
            "ros_time": rospy.Time.now().to_sec(),
            "start_wall_time": self._start_wall_time,
            "start_ros_time": self._start_ros_time,
            "monitoring_started": self._monitoring_started,
            "task_start_ros_time": self._task_start_ros_time if self._task_start_ros_time is not None else "NA",
            "task_complete_ros_time": self._task_complete_ros_time if self._task_complete_ros_time is not None else "NA",
            "drone_num": self.drone_num,
            "params": {
                "window_sec": self.window_sec,
                "move_threshold_x": self.move_threshold_x,
                "delay_timeout_threshold_s": self.delay_timeout_threshold_s,
                "active_speed_eps": self.active_speed_eps,
                "start_grace_sec": self.start_grace_sec,
                "drift_eps": self.drift_eps,
                "future_horizon_sec": self.future_horizon_sec,
                "future_sample_dt": self.future_sample_dt,
                "drone_radius": self.drone_radius,
                "drone_collision_margin": self.drone_margin,
                "obstacle_collision_margin": self.obstacle_margin,
                "task_active_fsm_states": sorted(TASK_ACTIVE_FSM_STATES),
                "task_inactive_fsm_states": sorted(TASK_INACTIVE_FSM_STATES),
            },
            "rows": rows,
            "details": details,
        }

    def _append_trace(self, snapshot: Dict[str, Any]) -> None:
        try:
            with open(self.trace_jsonl_path, "a", encoding="ascii") as f:
                f.write(json.dumps({"ros_time": snapshot["ros_time"], "rows": snapshot["rows"]}, ensure_ascii=True) + "\n")
        except Exception:
            pass

    def _write_outputs(self, reason: str) -> None:
        with self._lock:
            snapshot = self._snapshot_locked(reason)
        self._write_snapshot(snapshot)

    def _write_snapshot(self, snapshot: Dict[str, Any]) -> None:
        os.makedirs(self.run_path, exist_ok=True)
        tmp_json = self.summary_json_path + ".tmp"
        with open(tmp_json, "w", encoding="ascii") as f:
            json.dump(snapshot, f, ensure_ascii=True, indent=2)
        os.replace(tmp_json, self.summary_json_path)

    def _on_shutdown(self) -> None:
        try:
            self._write_outputs("shutdown")
        except Exception as ex:
            rospy.logwarn("[stl_attack_metrics_monitor] failed to write shutdown metrics: %s", str(ex))


def main() -> None:
    rospy.init_node("stl_attack_metrics_monitor", anonymous=False)
    STLAttackMetricsMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
