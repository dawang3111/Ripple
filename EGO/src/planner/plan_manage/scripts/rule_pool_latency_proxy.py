#!/usr/bin/env python3
from __future__ import annotations

import copy
import csv
import json
import math
import os
import random
import sys
import threading
import time
from collections import deque
from datetime import datetime
from typing import Any, Deque, Dict, Iterable, List, Optional, Tuple

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from traj_utils.msg import MINCOTraj, PolyTraj

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from ddc10_rules import DDC10_RULE_IDS, DDC10_RULE_NAMES, ddc10_or_components


TASK_ACTIVE_FSM_STATES = set(["SEQUENTIAL_START", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"])
REPLAN_TRIGGER_STATES = set(["GEN_NEW_TRAJ", "REPLAN_TRAJ", "EMERGENCY_STOP"])
NEW_TRAJ_STATES = set(["GEN_NEW_TRAJ", "REPLAN_TRAJ"])
OLD_RULE_NAMES = {
    "R1": "triad_distance_lt_3r",
    "R2": "triad_heading_aligned",
    "R3": "triad_distance_closing_5s",
    "R4": "two_or_more_replanning",
    "R5": "pair_future_trajectory_collision",
    "R6": "two_or_more_new_trajectory",
}
GLOBAL_RULE_NAMES = {
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
RANDOM_RULE_NAMES = {
    "RAND": "random_packet_latency",
}
RULE_NAMES = dict(OLD_RULE_NAMES)
RULE_NAMES.update(GLOBAL_RULE_NAMES)
RULE_NAMES.update(DDC10_RULE_NAMES)
RULE_NAMES.update(RANDOM_RULE_NAMES)
RULE_IDS = tuple(RULE_NAMES)


def _now_iso() -> str:
    return datetime.now().isoformat()


def _dist(a: List[float], b: List[float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _dot(a: List[float], b: List[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(v: List[float]) -> float:
    return math.sqrt(_dot(v, v))


def _sub(a: List[float], b: List[float]) -> List[float]:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except Exception:
        return default
    if math.isnan(out) or math.isinf(out):
        return default
    return out


def _env_float(name: str, default: float) -> float:
    return _safe_float(os.environ.get(name), default)


def _env_int(name: str, default: int) -> int:
    try:
        return int(str(os.environ.get(name, default)).strip())
    except Exception:
        return default


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


class RulePoolLatencyProxy:
    def __init__(self) -> None:
        self.enabled = bool(rospy.get_param("~enable", False))
        self.screening_mode = bool(rospy.get_param("~screening_mode", False))
        self.monitor_only = bool(rospy.get_param("~monitor_only", False))
        self.active_rule_id = str(rospy.get_param("~active_rule_id", "R1")).strip().upper()
        if self.active_rule_id not in RULE_IDS and not ddc10_or_components(self.active_rule_id):
            rospy.logwarn("[rule_pool_latency_proxy] invalid rule_id=%s, fallback to R1", self.active_rule_id)
            self.active_rule_id = "R1"

        self.in_topic = str(rospy.get_param("~in_topic", "/broadcast_traj_from_planner")).strip()
        self.out_topic = str(rospy.get_param("~out_topic", "/broadcast_traj_from_planner_attacked")).strip()
        self.odom_suffix = str(rospy.get_param("~odom_suffix", "visual_slam/odom")).strip("/")
        self.obstacle_topic = str(rospy.get_param("~obstacle_topic", "/map_generator/global_cloud")).strip()
        self.report_dir = os.path.expanduser(str(rospy.get_param("~report_dir", "~/.ego_fuzz_reports")).strip())
        self.run_id = str(rospy.get_param("~run_id", "")).strip() or datetime.now().strftime("%Y%m%d_%H%M%S")
        self.drone_num = int(rospy.get_param("~drone_num", 10))
        self.drone_radius = float(rospy.get_param("~drone_radius", 0.1))

        self.max_injections = int(rospy.get_param("~max_injections_per_round", 5))
        self.cooldown_s = float(rospy.get_param("~cooldown_s", 1.0))
        self.history_window_s = float(rospy.get_param("~history_window_s", 5.0))
        self.trigger_window_s = float(rospy.get_param("~trigger_window_s", 1.0))
        self.future_horizon_s = float(rospy.get_param("~future_horizon_s", 3.0))
        self.future_sample_dt = float(rospy.get_param("~future_sample_dt", 0.1))
        self.heading_aligned_deg = float(rospy.get_param("~heading_aligned_deg", 20.0))
        self.future_collision_margin = float(rospy.get_param("~future_collision_margin", 0.0))
        self.turn_angle_deg = float(rospy.get_param("~turn_angle_deg", 30.0))
        self.traj_offset_threshold = float(rospy.get_param("~trajectory_offset_threshold", 0.2))
        self.obstacle_sample_limit = int(rospy.get_param("~obstacle_sample_limit", 3000))
        self.obstacle_sample_stride = int(rospy.get_param("~obstacle_sample_stride", 20))
        self.delay_progress_expected_m = float(rospy.get_param("~delay_progress_expected_m", 1.0))
        self.delay_progress_ratio_threshold = float(rospy.get_param("~delay_progress_ratio_threshold", 0.5))
        self.latency_epsilon_ms = float(rospy.get_param("~latency_epsilon_ms", 10.0))
        self.latency_min_ms = float(rospy.get_param("~latency_min_ms", 40.0))
        self.latency_max_ms = float(rospy.get_param("~latency_max_ms", 0.0))
        self.protocol_timeout_ms = float(rospy.get_param("~protocol_timeout_ms", 1000.0))
        self.max_latency_ratio = float(rospy.get_param("~max_latency_ratio", 0.8))
        self.period_fallback_ms = float(rospy.get_param("~period_fallback_ms", 120.0))
        if self.protocol_timeout_ms <= 0.0:
            raise ValueError("protocol_timeout_ms must be positive")
        if not 0.0 < self.max_latency_ratio < 1.0:
            raise ValueError("max_latency_ratio must be between 0 and 1")
        if self.latency_min_ms < 0.0 or self.latency_max_ms < 0.0:
            raise ValueError("latency limits must be non-negative")
        self.random_seed = _env_int("RANDOM_PACKET_SEED", 20260924)
        self.random_mode = str(os.environ.get("RANDOM_PACKET_MODE", "target_offsets")).strip().lower()
        self.random_packet_probability = max(0.0, min(1.0, _env_float("RANDOM_PACKET_PROBABILITY", 0.02)))
        self.random_target_start_s = _env_float("RANDOM_PACKET_TARGET_START_S", 3.0)
        self.random_target_end_s = _env_float("RANDOM_PACKET_TARGET_END_S", 60.0)
        self.random_target_drone_id = _env_int("RANDOM_PACKET_TARGET_DRONE_ID", -1)
        self.random_rng = random.Random(self.random_seed)
        lo = max(0.0, min(self.random_target_start_s, self.random_target_end_s))
        hi = max(lo, max(self.random_target_start_s, self.random_target_end_s))
        target_count = self.max_injections if self.max_injections > 0 else 0
        self.random_target_offsets_s = sorted(self.random_rng.uniform(lo, hi) for _ in range(target_count))
        self.random_next_target_index = 0

        self.start_ros_time = rospy.Time.now().to_sec()
        self.start_wall_time = _now_iso()
        self.lock = threading.Lock()
        self.local_trajs: Dict[int, PolyTrajectory] = {}
        self.prev_local_trajs: Dict[int, PolyTrajectory] = {}
        self.last_traj_id: Dict[int, int] = {}
        self.last_traj_change_time: Dict[int, float] = {}
        self.fsm_state: Dict[int, Dict[str, Any]] = {}
        self.fsm_history: Dict[int, Deque[Dict[str, Any]]] = {i: deque() for i in range(self.drone_num)}
        self.odom_state: Dict[int, Dict[str, Any]] = {}
        self.odom_history: Dict[int, Deque[Dict[str, Any]]] = {i: deque() for i in range(self.drone_num)}
        self.replan_since_traj_publish: Dict[int, int] = {i: 0 for i in range(self.drone_num)}
        self.obstacle_points: List[List[float]] = []
        self.obstacle_update_time: float = -1e9
        self.last_broadcast_time: Dict[int, float] = {}
        self.period_ema_ms: Dict[int, float] = {}
        self.last_injection_time = -1e9
        self.injection_count = 0
        self.msg_seen = 0
        self.msg_delayed = 0
        self.delay_schedule_skip_counts: Dict[str, int] = {}
        self.screening_observation_count = 0
        self.screening_pre_observation_count = 0
        self.screening_evaluation_errors = 0
        self.screening_rule_counts = {rule_id: 0 for rule_id in DDC10_RULE_IDS}
        self.screening_feasible_counts = {rule_id: 0 for rule_id in DDC10_RULE_IDS}
        self.screening_pair_counts: Dict[str, int] = {}
        self.first_collision_ros_time: Optional[float] = None
        self.first_drone_collision_ros_time: Optional[float] = None
        self.events: List[Dict[str, Any]] = []
        self.publish_queues: Dict[int, Deque[Tuple[float, MINCOTraj]]] = {
            i: deque() for i in range(self.drone_num)
        }
        self.publish_timers: Dict[int, Optional[threading.Timer]] = {
            i: None for i in range(self.drone_num)
        }
        self.publish_busy: Dict[int, bool] = {i: False for i in range(self.drone_num)}

        self.run_path = os.path.join(self.report_dir, self.run_id)
        os.makedirs(self.run_path, exist_ok=True)
        self.meta_path = os.path.join(self.run_path, "rule_pool_latency_meta.json")
        self.events_csv_path = os.path.join(self.run_path, "rule_pool_latency_events.csv")
        self.trace_jsonl_path = os.path.join(self.run_path, "rule_pool_latency_trace.jsonl")
        self.observations_path = os.path.join(self.run_path, "ddc10_rule_observations.csv")
        self._ensure_events_csv()
        if self.screening_mode:
            with open(self.observations_path, "w", encoding="ascii", newline="") as f:
                csv.writer(f).writerow(self._screening_observation_fields())

        self.pub = rospy.Publisher(self.out_topic, MINCOTraj, queue_size=200)
        self.sub = rospy.Subscriber(self.in_topic, MINCOTraj, self._broadcast_cb, queue_size=200)
        self.subs = [self.sub]
        for drone_id in range(self.drone_num):
            self.subs.append(rospy.Subscriber("/drone_{}_planning/trajectory".format(drone_id), PolyTraj, self._traj_cb, callback_args=drone_id, queue_size=50))
            self.subs.append(rospy.Subscriber("/drone_{}_planning/fsm_state".format(drone_id), String, self._fsm_cb, callback_args=drone_id, queue_size=20))
            self.subs.append(rospy.Subscriber("/drone_{}_{}".format(drone_id, self.odom_suffix), Odometry, self._odom_cb, callback_args=drone_id, queue_size=50))
        self.subs.append(rospy.Subscriber(self.obstacle_topic, PointCloud2, self._obstacle_cb, queue_size=1))
        if self.screening_mode:
            self.subs.append(rospy.Subscriber("/fuzz_collision_event", String, self._screening_collision_cb, queue_size=10))

        rospy.on_shutdown(self._on_shutdown)
        self._write_meta(final=False)
        rospy.loginfo(
            "[rule_pool_latency_proxy] enabled=%s rule=%s max_injections=%d in=%s out=%s",
            str(self.enabled).lower(),
            self.active_rule_id,
            self.max_injections,
            self.in_topic,
            self.out_topic,
        )
        rospy.loginfo(
            "[rule_pool_latency_proxy] next_packet_guard_ms=%.3f protocol_budget_ms=%.3f optional_cap_ms=%.3f",
            self.latency_epsilon_ms,
            self.protocol_timeout_ms * self.max_latency_ratio,
            self.latency_max_ms,
        )

    def _traj_cb(self, msg: PolyTraj, drone_id: int) -> None:
        traj = PolyTrajectory(msg)
        if not traj.valid:
            return
        now = rospy.Time.now().to_sec()
        with self.lock:
            prev_id = self.last_traj_id.get(drone_id)
            if prev_id is not None and prev_id != traj.traj_id and drone_id in self.local_trajs:
                self.prev_local_trajs[drone_id] = self.local_trajs[drone_id]
            self.local_trajs[drone_id] = traj
            self.last_traj_id[drone_id] = traj.traj_id
            if prev_id is not None and prev_id != traj.traj_id:
                self.last_traj_change_time[drone_id] = now
                self.replan_since_traj_publish[drone_id] = 0

    def _fsm_cb(self, msg: String, drone_id: int) -> None:
        now = rospy.Time.now().to_sec()
        state = str(msg.data or "NA")
        with self.lock:
            prev = str(self.fsm_state.get(drone_id, {}).get("state", "NA"))
            self.fsm_state[drone_id] = {"state": state, "t": now}
            hist = self.fsm_history[drone_id]
            if prev != state:
                hist.append({"t": now, "state": state})
                if state in REPLAN_TRIGGER_STATES:
                    self.replan_since_traj_publish[drone_id] = self.replan_since_traj_publish.get(drone_id, 0) + 1
            cutoff = now - max(self.history_window_s + 1.0, 1.0)
            while hist and float(hist[0]["t"]) < cutoff:
                hist.popleft()

    def _odom_cb(self, msg: Odometry, drone_id: int) -> None:
        now = rospy.Time.now().to_sec()
        pos = [
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        ]
        vel = [
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z),
        ]
        old = None
        with self.lock:
            old = self.odom_state.get(drone_id)
        accel = [0.0, 0.0, 0.0]
        if old is not None:
            dt = now - float(old.get("t", now))
            if dt > 1e-3:
                old_vel = list(old.get("vel", [0.0, 0.0, 0.0]))
                accel = [(vel[i] - old_vel[i]) / dt for i in range(3)]
        sample = {"t": now, "pos": pos, "vel": vel, "accel": accel}
        with self.lock:
            self.odom_state[drone_id] = sample
            hist = self.odom_history[drone_id]
            hist.append(sample)
            cutoff = now - max(self.history_window_s + 1.0, 1.0)
            while hist and float(hist[0]["t"]) < cutoff:
                hist.popleft()

    def _obstacle_cb(self, msg: PointCloud2) -> None:
        now = rospy.Time.now().to_sec()
        points: List[List[float]] = []
        stride = max(1, self.obstacle_sample_stride)
        limit = max(0, self.obstacle_sample_limit)
        try:
            for idx, pt in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
                if idx % stride != 0:
                    continue
                points.append([float(pt[0]), float(pt[1]), float(pt[2])])
                if limit and len(points) >= limit:
                    break
        except Exception as exc:
            rospy.logwarn("[rule_pool_latency_proxy] failed to read obstacle cloud: %s", str(exc))
            return
        with self.lock:
            self.obstacle_points = points
            self.obstacle_update_time = now

    def _broadcast_cb(self, msg: MINCOTraj) -> None:
        now = rospy.Time.now().to_sec()
        source_id = int(msg.drone_id)
        trace_event = None
        with self.lock:
            self.msg_seen += 1
            prev_broadcast = self.last_broadcast_time.get(source_id)
            self.last_broadcast_time[source_id] = now
            if prev_broadcast is not None and now > prev_broadcast:
                self._observe_period_locked(source_id, (now - prev_broadcast) * 1000.0)
            try:
                observation = self._observe_screening_rules_locked(source_id, now, msg) if self.screening_mode else None
                decision = self._choose_delay_locked(source_id, now, msg, observation)
            except Exception as exc:
                if self.screening_mode:
                    self.screening_evaluation_errors += 1
                rospy.logerr_throttle(
                    5.0,
                    "[rule_pool_latency_proxy] rule evaluation failed for rule=%s source=%d; forwarding packet without injection: %s",
                    self.active_rule_id,
                    source_id,
                    str(exc),
                )
                decision = None
            if decision is not None and decision.get("skip_reason"):
                reason = str(decision["skip_reason"])
                self.delay_schedule_skip_counts[reason] = self.delay_schedule_skip_counts.get(reason, 0) + 1
                trace_event = ("rule_pool_delay_skipped", dict(decision))
                self._write_meta(final=False)
            elif decision is not None:
                self.injection_count += 1
                self.msg_delayed += 1
                self.last_injection_time = now
                decision["injection_index"] = self.injection_count
                self.events.append(dict(decision))
                self._append_event_locked(decision)
                self._write_meta(final=False)
                trace_event = ("rule_pool_delay_tx", dict(decision))

        if trace_event is not None:
            self._append_trace(*trace_event)
        delay_s = float(decision["latency_ms"]) / 1000.0 if decision and not decision.get("skip_reason") else 0.0
        self._enqueue_publish(source_id, msg, delay_s)

    def _choose_delay_locked(self, source_id: int, now: float, msg: MINCOTraj,
                             observation: Optional[Tuple[List[int], Dict[str, Any]]] = None) -> Optional[Dict[str, Any]]:
        if not self.enabled or self.monitor_only:
            return None
        if self.screening_mode and (observation is None or self.first_collision_ros_time is not None):
            return None
        effective_max_injections = self._effective_max_injections()
        if effective_max_injections > 0 and self.injection_count >= effective_max_injections:
            return None
        if now - self.last_injection_time < self.cooldown_s:
            return None
        if source_id < 0 or source_id >= self.drone_num:
            return None
        if not self._is_active(source_id):
            return None

        if observation is not None:
            triad, evidence = observation
            components = ddc10_or_components(self.active_rule_id)
            if components:
                matched = any(evidence["all_rule_matches"].get(rule_id, False) for rule_id in components)
            else:
                matched = bool(evidence["all_rule_matches"].get(self.active_rule_id, False))
        else:
            triad = self._nearest_triad(source_id, now)
            if triad is None:
                return None
            matched, evidence = self._match_rule(self.active_rule_id, source_id, triad, now)
        if not matched:
            return None

        victim_id = int(evidence.get("victim_id", self._closest_other(source_id, triad, now)))
        latency_ms, timing = self._latency_before_next_packet(source_id)
        if latency_ms is None:
            return {
                "wall_time": _now_iso(),
                "ros_time": now,
                "inject_time_s": now - self.start_ros_time,
                "rule_id": self.active_rule_id,
                "rule_name": self._rule_name(self.active_rule_id),
                "attacker_drone_id": source_id,
                "victim_drone_id": victim_id,
                "skip_reason": timing["skip_reason"],
                "target_latency_ms": timing["target_latency_ms"],
                "protocol_safe_max_ms": timing["protocol_safe_max_ms"],
                "effective_safe_max_ms": timing["effective_safe_max_ms"],
                "configured_latency_cap_ms": timing["configured_latency_cap_ms"],
                "estimated_period_ms": timing["estimated_period_ms"],
                "attacker_fsm_state": self._state(source_id),
                "victim_fsm_state": self._state(victim_id),
                "trigger_evidence": json.dumps(evidence, ensure_ascii=True, sort_keys=True),
            }
        return {
            "wall_time": _now_iso(),
            "ros_time": now,
            "inject_time_s": now - self.start_ros_time,
            "rule_id": self.active_rule_id,
            "rule_name": self._rule_name(self.active_rule_id),
            "attacker_drone_id": source_id,
            "victim_drone_id": victim_id,
            "triad_node_ids": ";".join(str(x) for x in triad),
            "source_traj_id": int(msg.traj_id),
            "victim_traj_id": int(self.last_traj_id.get(victim_id, -1)),
            "attacker_fsm_state": self._state(source_id),
            "victim_fsm_state": self._state(victim_id),
            "latency_ms": latency_ms,
            "target_latency_ms": timing["target_latency_ms"],
            "protocol_safe_max_ms": timing["protocol_safe_max_ms"],
            "effective_safe_max_ms": timing["effective_safe_max_ms"],
            "configured_latency_cap_ms": timing["configured_latency_cap_ms"],
            "estimated_period_ms": timing["estimated_period_ms"],
            "trigger_evidence": json.dumps(evidence, ensure_ascii=True, sort_keys=True),
        }

    def _screening_collision_cb(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
            stamp = float(event["ros_time"])
            collision_type = event["collision"]["type"]
            if not math.isfinite(stamp) or event.get("event") != "HARD_COLLISION":
                return
        except (KeyError, ValueError, TypeError):
            return
        with self.lock:
            if self.first_collision_ros_time is None or stamp < self.first_collision_ros_time:
                self.first_collision_ros_time = stamp
            if collision_type == "drone_drone":
                if self.first_drone_collision_ros_time is None or stamp < self.first_drone_collision_ros_time:
                    self.first_drone_collision_ros_time = stamp

    def _screening_observation_fields(self) -> List[str]:
        return ["ros_time", "inject_time_s", "source_node_id", "source_traj_id", "active_rule_id",
                "phase", "selected_rule_matched", "latency_feasible", "target_latency_ms",
                "skip_reason", "trigger_evidence"] + list(DDC10_RULE_IDS)

    def _observe_screening_rules_locked(self, source_id: int, now: float, msg: MINCOTraj
                                       ) -> Optional[Tuple[List[int], Dict[str, Any]]]:
        if source_id < 0 or source_id >= self.drone_num or self._state(source_id) not in TASK_ACTIVE_FSM_STATES:
            return None
        triad = self._nearest_triad(source_id, now)
        if triad is None or any(self._state(node) not in TASK_ACTIVE_FSM_STATES for node in triad):
            return None
        _matched, evidence = self._match_curated_target1_union_rule("DDC10", source_id, triad, now)
        matches = evidence["all_rule_matches"]
        latency_ms, timing = self._latency_before_next_packet(source_id)
        if self.first_collision_ros_time is not None:
            phase = "after_collision"
        elif self.injection_count:
            phase = "post_injection"
        else:
            phase = "pre_injection"
        self.screening_observation_count += 1
        if phase == "pre_injection":
            self.screening_pre_observation_count += 1
            feasible_ids = []
            for rule_id, matched in matches.items():
                self.screening_rule_counts[rule_id] += int(matched)
                if matched and latency_ms is not None:
                    self.screening_feasible_counts[rule_id] += 1
                    feasible_ids.append(rule_id)
            for index, first in enumerate(feasible_ids):
                for second in feasible_ids[index + 1:]:
                    key = first + ";" + second
                    self.screening_pair_counts[key] = self.screening_pair_counts.get(key, 0) + 1
        components = ddc10_or_components(self.active_rule_id)
        if components:
            selected = any(matches.get(rule_id, False) for rule_id in components)
        else:
            selected = bool(matches.get(self.active_rule_id, any(matches.values()) if self.monitor_only else False))
        row = {
            "ros_time": now, "inject_time_s": now - self.start_ros_time,
            "source_node_id": source_id, "source_traj_id": int(msg.traj_id),
            "active_rule_id": self.active_rule_id, "phase": phase,
            "selected_rule_matched": int(selected), "latency_feasible": int(latency_ms is not None),
            "target_latency_ms": timing["target_latency_ms"], "skip_reason": timing.get("skip_reason", ""),
            "trigger_evidence": json.dumps(evidence, ensure_ascii=True, sort_keys=True),
        }
        row.update({rule_id: int(matches[rule_id]) for rule_id in DDC10_RULE_IDS})
        with open(self.observations_path, "a", encoding="ascii", newline="") as f:
            csv.DictWriter(f, fieldnames=self._screening_observation_fields()).writerow(row)
        if self.screening_observation_count % 50 == 0:
            self._write_meta(final=False)
        return triad, evidence

    def _match_rule(self, rule_id: str, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        if rule_id == "RAND":
            return self._match_random_packet_rule(source_id, triad, now)
        if rule_id.startswith("OR1"):
            return self._match_targeted_or_rule(rule_id, source_id, triad, now)
        if rule_id.startswith("DDC10"):
            return self._match_curated_target1_union_rule(rule_id, source_id, triad, now)
        if rule_id in ("R1", "DD1"):
            return self._rule_distance_lt_3r(source_id, triad, now)
        if rule_id in ("R2", "DD2"):
            return self._rule_heading_aligned(source_id, triad, now)
        if rule_id in ("R3", "DD3"):
            return self._rule_distance_closing_5s(source_id, triad, now)
        if rule_id in ("R4", "DD4"):
            return self._rule_two_replanning(source_id, triad, now)
        if rule_id == "R5":
            return self._rule_future_collision(source_id, triad, now, 2.0 * self.drone_radius)
        if rule_id == "DD5":
            return self._rule_future_collision(source_id, triad, now, 1.0 * self.drone_radius)
        if rule_id in ("R6", "DD6"):
            return self._rule_two_new_traj(source_id, triad, now)
        if rule_id.startswith("DO_CURATED"):
            return self._match_curated_obstacle_rule(rule_id, source_id, triad, now)
        if rule_id.startswith("DO"):
            return self._match_obstacle_rule(rule_id, source_id, now)
        if rule_id.startswith("DL_CURATED"):
            return self._match_curated_deadlock_rule(rule_id, source_id, triad, now)
        if rule_id.startswith("DL"):
            return self._match_deadlock_rule(rule_id, source_id, now)
        if rule_id.startswith("DE_CURATED"):
            return self._match_curated_delay_rule(rule_id, source_id, triad, now)
        if rule_id.startswith("DE"):
            return self._match_delay_rule(rule_id, source_id, now)
        if rule_id.startswith("PF_CURATED"):
            return self._match_curated_planning_failure_rule(rule_id, source_id, triad, now)
        if rule_id.startswith("PF"):
            return self._match_planning_failure_rule(rule_id, source_id, now)
        if rule_id.startswith("DR_CURATED"):
            return self._match_curated_drift_rule(rule_id, source_id, triad, now)
        if rule_id.startswith("DR"):
            return self._match_drift_rule(rule_id, source_id, now)
        return False, {"reason": "unknown_rule"}

    def _match_random_packet_rule(self, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        if self.random_target_drone_id >= 0 and source_id != self.random_target_drone_id:
            return False, {"reason": "different_source", "target_drone_id": self.random_target_drone_id}
        elapsed = now - self.start_ros_time
        if elapsed < self.random_target_start_s or elapsed > self.random_target_end_s:
            return False, {"reason": "outside_random_window", "elapsed_s": elapsed, "window_start_s": self.random_target_start_s, "window_end_s": self.random_target_end_s}
        if self.random_mode == "probability":
            draw = self.random_rng.random()
            if draw > self.random_packet_probability:
                return False, {"reason": "random_probability_miss", "draw": draw, "probability": self.random_packet_probability}
            return True, self._random_feature_snapshot(source_id, triad, now, elapsed)

        if self.random_next_target_index >= len(self.random_target_offsets_s):
            return False, {"reason": "no_more_random_targets"}
        target_offset = self.random_target_offsets_s[self.random_next_target_index]
        if elapsed < target_offset:
            return False, {"reason": "before_random_target", "elapsed_s": elapsed, "target_offset_s": target_offset}
        self.random_next_target_index += 1
        return True, self._random_feature_snapshot(source_id, triad, now, target_offset)

    def _nearest_triad(self, source_id: int, now: float) -> Optional[List[int]]:
        source_pos = self._position(source_id, now)
        if source_pos is None:
            return None
        ranked: List[Tuple[float, int]] = []
        for drone_id in range(self.drone_num):
            if drone_id == source_id:
                continue
            pos = self._position(drone_id, now)
            if pos is None or not self._is_active(drone_id):
                continue
            ranked.append((_dist(source_pos, pos), drone_id))
        if len(ranked) < 2:
            return None
        ranked.sort(key=lambda item: item[0])
        return [source_id, ranked[0][1], ranked[1][1]]

    def _rule_distance_lt_3r(self, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        threshold = 3.0 * self.drone_radius
        distances = self._pair_distances(triad, now)
        if len(distances) != 3:
            return False, {"reason": "missing_distance"}
        max_dist = max(v for _pair, v in distances)
        min_pair, min_dist = min(distances, key=lambda item: item[1])
        return max_dist < threshold, {
            "max_pair_distance": max_dist,
            "min_pair_distance": min_dist,
            "threshold": threshold,
            "victim_id": self._pair_victim(source_id, min_pair),
        }

    def _rule_heading_aligned(self, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        headings = []
        for drone_id in triad:
            heading = self._heading(drone_id, now)
            if heading is None:
                return False, {"reason": "missing_heading"}
            headings.append((drone_id, heading))
        max_angle = 0.0
        for i in range(len(headings)):
            for j in range(i + 1, len(headings)):
                angle = self._angle_deg(headings[i][1], headings[j][1])
                max_angle = max(max_angle, angle)
        return max_angle <= self.heading_aligned_deg, {
            "max_heading_angle_deg": max_angle,
            "threshold_deg": self.heading_aligned_deg,
            "victim_id": self._closest_other(source_id, triad, now),
        }

    def _rule_distance_closing_5s(self, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        shrink_threshold = 3.0 * self.drone_radius
        best_shrink = -float("inf")
        best_pair = (source_id, triad[1])
        for i in range(len(triad)):
            for j in range(i + 1, len(triad)):
                old_a = self._history_position_at_or_before(triad[i], now - self.history_window_s)
                old_b = self._history_position_at_or_before(triad[j], now - self.history_window_s)
                cur_a = self._position(triad[i], now)
                cur_b = self._position(triad[j], now)
                if old_a is None or old_b is None or cur_a is None or cur_b is None:
                    continue
                shrink = _dist(old_a, old_b) - _dist(cur_a, cur_b)
                if shrink > best_shrink:
                    best_shrink = shrink
                    best_pair = (triad[i], triad[j])
        return best_shrink >= shrink_threshold, {
            "best_distance_shrink_5s": best_shrink,
            "threshold": shrink_threshold,
            "best_pair": "{}-{}".format(best_pair[0], best_pair[1]),
            "victim_id": self._pair_victim(source_id, best_pair),
        }

    def _rule_two_replanning(self, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        replanning = [drone_id for drone_id in triad if self._is_replanning(drone_id, now)]
        return len(replanning) >= 2, {
            "replanning_node_ids": ";".join(str(x) for x in replanning),
            "count": len(replanning),
            "victim_id": self._prefer_non_source(source_id, replanning, triad, now),
        }

    def _rule_future_collision(self, source_id: int, triad: List[int], now: float, threshold: float) -> Tuple[bool, Dict[str, Any]]:
        best_pair = None
        best_margin = float("inf")
        best_dist = float("inf")
        for i in range(len(triad)):
            for j in range(i + 1, len(triad)):
                result = self._future_pair_margin(triad[i], triad[j], now, threshold)
                if result is None:
                    continue
                margin, dist = result
                if margin < best_margin:
                    best_margin = margin
                    best_dist = dist
                    best_pair = (triad[i], triad[j])
        if best_pair is None:
            return False, {"reason": "missing_future_trajectory"}
        return best_margin <= self.future_collision_margin, {
            "best_future_margin": best_margin,
            "best_future_distance": best_dist,
            "distance_threshold": threshold,
            "margin_threshold": self.future_collision_margin,
            "best_pair": "{}-{}".format(best_pair[0], best_pair[1]),
            "victim_id": self._pair_victim(source_id, best_pair),
        }

    def _rule_two_new_traj(self, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        new_nodes = [drone_id for drone_id in triad if self._recent_new_traj(drone_id, now)]
        return len(new_nodes) >= 2, {
            "new_traj_node_ids": ";".join(str(x) for x in new_nodes),
            "count": len(new_nodes),
            "victim_id": self._prefer_non_source(source_id, new_nodes, triad, now),
        }

    def _effective_max_injections(self) -> int:
        if self.active_rule_id == "OR1_LIMIT10" and (self.max_injections <= 0 or self.max_injections > 10):
            return 10
        return self.max_injections

    def _match_targeted_or_rule(self, rule_id: str, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        matched: List[Tuple[str, str]] = []
        evidence: Dict[str, Any] = {
            "mode": self._rule_name(rule_id),
            "or_rule_variant": rule_id,
            "candidate_node_ids": self._ids(triad),
        }

        distances = self._pair_distances(triad, now)
        if distances:
            vals = [dist_value for _pair, dist_value in distances]
            min_pair, min_dist = min(distances, key=lambda item: item[1])
            evidence["triad_min_pair_distance"] = min_dist
            evidence["triad_mean_pair_distance"] = sum(vals) / len(vals)
            evidence["triad_max_pair_distance"] = max(vals)
            evidence["triad_min_pair"] = "{}-{}".format(min_pair[0], min_pair[1])
            evidence["victim_id"] = self._pair_victim(source_id, min_pair)
            if min_dist <= 1.0:
                matched.append(("OR1_6", "current_min_inter_drone_distance_le_1m"))

        best_shrink: Optional[float] = None
        best_shrink_pair: Optional[Tuple[int, int]] = None
        for i in range(len(triad)):
            for j in range(i + 1, len(triad)):
                old_a = self._history_position_at_or_before(triad[i], now - self.history_window_s)
                old_b = self._history_position_at_or_before(triad[j], now - self.history_window_s)
                cur_a = self._position(triad[i], now)
                cur_b = self._position(triad[j], now)
                if old_a is None or old_b is None or cur_a is None or cur_b is None:
                    continue
                shrink = _dist(old_a, old_b) - _dist(cur_a, cur_b)
                if best_shrink is None or shrink > best_shrink:
                    best_shrink = shrink
                    best_shrink_pair = (triad[i], triad[j])
        if best_shrink is not None:
            evidence["triad_best_distance_shrink_5s"] = best_shrink
            evidence["triad_best_distance_shrink_pair"] = "{}-{}".format(best_shrink_pair[0], best_shrink_pair[1]) if best_shrink_pair else "NA"
            if best_shrink >= 0.3:
                matched.append(("OR1_2", "interacting_pair_distance_shrink_5s_ge_0p3m"))

        future = self._best_future_pair_distance(triad, now, 0.8)
        if future is not None:
            margin, future_dist, future_pair = future
            evidence["triad_future_min_distance"] = future_dist
            evidence["triad_future_margin_to_0p8m"] = margin
            evidence["triad_future_min_pair"] = "{}-{}".format(future_pair[0], future_pair[1])
            if future_dist <= 0.8:
                matched.append(("OR1_5", "future_3s_predicted_min_distance_le_0p8m"))
            if evidence.get("triad_min_pair_distance", float("inf")) <= 1.0 and future_dist <= 0.8:
                matched.append(("OR1_7", "current_distance_le_1m_and_future_distance_le_0p8m"))

        heading_angles = []
        for i in range(len(triad)):
            for j in range(i + 1, len(triad)):
                a = self._heading(triad[i], now)
                b = self._heading(triad[j], now)
                if a is not None and b is not None:
                    heading_angles.append(self._angle_deg(a, b))
        if heading_angles:
            max_angle = max(heading_angles)
            evidence["triad_max_heading_angle_deg"] = max_angle
            evidence["triad_mean_heading_angle_deg"] = sum(heading_angles) / len(heading_angles)
            if max_angle <= 20.0:
                matched.append(("OR1_8", "triad_max_heading_angle_le_20deg"))

        replan_nodes = [drone_id for drone_id in triad if self._is_replanning(drone_id, now)]
        new_traj_nodes = [drone_id for drone_id in triad if self._recent_new_traj(drone_id, now)]
        evidence["triad_replan_or_stop_count"] = len(replan_nodes)
        evidence["triad_replan_or_stop_node_ids"] = self._ids(replan_nodes)
        evidence["triad_new_traj_count"] = len(new_traj_nodes)
        evidence["triad_new_traj_node_ids"] = self._ids(new_traj_nodes)
        if len(replan_nodes) >= 2 and len(new_traj_nodes) >= 2:
            matched.append(("OR1_1", "two_replan_or_stop_and_two_recent_new_traj"))

        source_speed = self._speed(source_id)
        evidence["source_speed"] = source_speed
        if source_speed >= 1.4:
            matched.append(("OR1_3", "source_speed_ge_1p4mps"))

        victim_id = int(evidence.get("victim_id", self._closest_other(source_id, triad, now)))
        source_offset = self._trajectory_offset(source_id, now)
        victim_offset = self._trajectory_offset(victim_id, now)
        evidence["source_trajectory_offset"] = source_offset
        evidence["victim_trajectory_offset"] = victim_offset
        if source_offset >= self.traj_offset_threshold or victim_offset >= self.traj_offset_threshold:
            matched.append(("OR1_4", "source_or_victim_trajectory_offset_ge_0p2m"))

        matched = self._filter_targeted_or_conditions(rule_id, matched)
        evidence["source_fsm_state"] = self._state(source_id)
        evidence["victim_fsm_state"] = self._state(victim_id)
        evidence["matched_condition_ids"] = ";".join(item[0] for item in matched) if matched else "NA"
        evidence["matched_condition_names"] = ";".join(item[1] for item in matched) if matched else "NA"
        evidence["matched_condition_count"] = len(matched)
        evidence["victim_id"] = victim_id
        return bool(matched), evidence

    def _filter_targeted_or_conditions(self, rule_id: str, matched: List[Tuple[str, str]]) -> List[Tuple[str, str]]:
        if rule_id in ("OR1", "OR1_FULL", "OR1_LIMIT10"):
            return matched

        ids = set(item[0] for item in matched)
        if rule_id == "OR1_NO_HEADING_ONLY":
            if ids == set(["OR1_8"]):
                return []
            return matched
        if rule_id == "OR1_ACTIVE_CORE":
            keep = set(["OR1_1", "OR1_2", "OR1_3"])
            return [item for item in matched if item[0] in keep]
        if rule_id == "OR1_CLOSE_CORE":
            keep = set(["OR1_5", "OR1_6", "OR1_7"])
            return [item for item in matched if item[0] in keep]
        return matched

    def _match_curated_target1_union_rule(self, rule_id: str, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        matched: List[Tuple[str, str]] = []
        evidence: Dict[str, Any] = {
            "mode": self._rule_name(rule_id),
            "or_rule_variant": rule_id,
            "candidate_node_ids": self._ids(triad),
        }

        victim_id = self._closest_other(source_id, triad, now)
        distances = self._pair_distances(triad, now)
        if distances:
            vals = [dist_value for _pair, dist_value in distances]
            min_pair, min_dist = min(distances, key=lambda item: item[1])
            evidence["triad_min_pair_distance"] = min_dist
            evidence["triad_mean_pair_distance"] = sum(vals) / len(vals)
            evidence["triad_max_pair_distance"] = max(vals)
            evidence["triad_min_pair"] = "{}-{}".format(min_pair[0], min_pair[1])
            victim_id = self._pair_victim(source_id, min_pair)

        future = self._best_future_pair_distance(triad, now, 0.0)
        if future is not None:
            _margin, future_dist, future_pair = future
            evidence["triad_future_min_distance"] = future_dist
            evidence["triad_future_min_pair"] = "{}-{}".format(future_pair[0], future_pair[1])
            if "victim_id" not in evidence:
                victim_id = self._pair_victim(source_id, future_pair)

        heading_angles = []
        for i in range(len(triad)):
            for j in range(i + 1, len(triad)):
                a = self._heading(triad[i], now)
                b = self._heading(triad[j], now)
                if a is not None and b is not None:
                    heading_angles.append(self._angle_deg(a, b))
        if heading_angles:
            evidence["triad_max_heading_angle_deg"] = max(heading_angles)
            evidence["triad_mean_heading_angle_deg"] = sum(heading_angles) / len(heading_angles)

        best_shrink: Optional[float] = None
        best_shrink_pair: Optional[Tuple[int, int]] = None
        for i in range(len(triad)):
            for j in range(i + 1, len(triad)):
                old_a = self._history_position_at_or_before(triad[i], now - self.history_window_s)
                old_b = self._history_position_at_or_before(triad[j], now - self.history_window_s)
                cur_a = self._position(triad[i], now)
                cur_b = self._position(triad[j], now)
                if old_a is None or old_b is None or cur_a is None or cur_b is None:
                    continue
                shrink = _dist(old_a, old_b) - _dist(cur_a, cur_b)
                if best_shrink is None or shrink > best_shrink:
                    best_shrink = shrink
                    best_shrink_pair = (triad[i], triad[j])
        if best_shrink is not None:
            evidence["triad_best_distance_shrink_5s"] = best_shrink
            evidence["triad_best_distance_shrink_pair"] = "{}-{}".format(best_shrink_pair[0], best_shrink_pair[1]) if best_shrink_pair else "NA"

        replan_nodes = [drone_id for drone_id in triad if self._is_replanning(drone_id, now)]
        new_traj_nodes = [drone_id for drone_id in triad if self._recent_new_traj(drone_id, now)]
        evidence["triad_replan_or_stop_count"] = len(replan_nodes)
        evidence["triad_replan_or_stop_node_ids"] = self._ids(replan_nodes)
        evidence["triad_new_traj_count"] = len(new_traj_nodes)
        evidence["triad_new_traj_node_ids"] = self._ids(new_traj_nodes)

        source_speed = self._speed(source_id)
        swarm_avg_current_speed = self._swarm_current_average("vel")
        source_speed_ratio = source_speed / swarm_avg_current_speed if swarm_avg_current_speed > 1e-6 else float("inf")
        source_turn_angle = self._turn_angle(source_id, now)
        evidence["source_speed"] = source_speed
        evidence["swarm_avg_current_speed"] = swarm_avg_current_speed
        evidence["source_speed_over_swarm_avg_current"] = source_speed_ratio
        evidence["source_turn_angle_deg"] = source_turn_angle

        offsets = [(drone_id, self._trajectory_offset(drone_id, now)) for drone_id in triad]
        if offsets:
            offset_node, max_offset = max(offsets, key=lambda item: item[1])
            evidence["triad_max_trajectory_offset"] = max_offset
            evidence["triad_max_trajectory_offset_node_id"] = offset_node
        source_offset = self._trajectory_offset(source_id, now)
        victim_offset = self._trajectory_offset(victim_id, now)
        evidence["source_trajectory_offset"] = source_offset
        evidence["victim_trajectory_offset"] = victim_offset

        future_dist = float(evidence.get("triad_future_min_distance", float("inf")))
        max_pair_dist = float(evidence.get("triad_max_pair_distance", float("inf")))
        triad_max_offset = float(evidence.get("triad_max_trajectory_offset", 0.0))
        mean_heading = float(evidence.get("triad_mean_heading_angle_deg", float("inf")))
        shrink_5s = float(evidence.get("triad_best_distance_shrink_5s", -float("inf")))

        if swarm_avg_current_speed >= 1.284 and future_dist <= 1.475:
            matched.append(("DDC10_1", "swarm_current_speed_ge_1p284_and_future_min_distance_le_1p475"))
        if future_dist <= 1.475 and triad_max_offset >= 0.0295:
            matched.append(("DDC10_2", "future_min_distance_le_1p475_and_triad_max_trajectory_offset_ge_0p0295"))
        if swarm_avg_current_speed >= 1.284 and source_speed_ratio <= 1.136:
            matched.append(("DDC10_3", "swarm_current_speed_ge_1p284_and_source_speed_ratio_le_1p136"))
        if swarm_avg_current_speed >= 1.284 and source_turn_angle >= 7.079:
            matched.append(("DDC10_4", "swarm_current_speed_ge_1p284_and_source_turn_angle_ge_7p079deg"))
        if swarm_avg_current_speed >= 1.284 and future_dist <= 1.363 and mean_heading >= 15.54:
            matched.append(("DDC10_5", "swarm_current_speed_ge_1p284_future_min_distance_le_1p363_mean_heading_ge_15p54deg"))
        if future_dist <= 1.475 and triad_max_offset >= 0.0295 and mean_heading >= 15.54:
            matched.append(("DDC10_6", "future_min_distance_le_1p475_triad_offset_ge_0p0295_mean_heading_ge_15p54deg"))
        if max_pair_dist <= 3.387 and future_dist <= 1.475:
            matched.append(("DDC10_7", "current_max_pair_distance_le_3p387_and_future_min_distance_le_1p475"))
        if swarm_avg_current_speed >= 1.351 and victim_offset >= 0.0067:
            matched.append(("DDC10_8", "swarm_current_speed_ge_1p351_and_victim_trajectory_offset_ge_0p0067"))
        if len(replan_nodes) >= 2 and len(new_traj_nodes) >= 2 and future_dist <= 1.475:
            matched.append(("DDC10_9", "two_replan_or_stop_two_new_traj_and_future_min_distance_le_1p475"))
        if shrink_5s >= 0.3 and future_dist <= 1.475:
            matched.append(("DDC10_10", "distance_shrink_5s_ge_0p3_and_future_min_distance_le_1p475"))

        base_matched = list(matched)
        evidence["all_rule_matches"] = {key: any(item[0] == key for item in base_matched) for key in DDC10_RULE_IDS}
        evidence["all_matched_rule_ids"] = ";".join(item[0] for item in base_matched)
        composite_components = ddc10_or_components(rule_id)
        if rule_id in DDC10_RULE_IDS:
            matched = [item for item in base_matched if item[0] == rule_id]
        elif composite_components:
            matched = [item for item in base_matched if item[0] in composite_components]
            evidence["composite_rule_components"] = ";".join(composite_components)
        elif rule_id != "DDC10":
            strict_terms: List[Tuple[str, str]] = []
            if triad_max_offset >= 0.0295:
                strict_terms.append(("DDC10_TERM_OFFSET", "triad_max_trajectory_offset_ge_0p0295"))
            if mean_heading >= 15.54:
                strict_terms.append(("DDC10_TERM_HEADING", "triad_mean_heading_angle_ge_15p54deg"))
            if shrink_5s >= 0.3:
                strict_terms.append(("DDC10_TERM_SHRINK", "distance_shrink_5s_ge_0p3"))
            if len(replan_nodes) >= 2:
                strict_terms.append(("DDC10_TERM_REPLAN", "two_or_more_replan_or_stop"))
            if len(new_traj_nodes) >= 2:
                strict_terms.append(("DDC10_TERM_NEW_TRAJ", "two_or_more_recent_new_traj"))

            future_close = future_dist <= 1.475
            evidence["ddc10_base_condition_ids"] = ";".join(item[0] for item in base_matched) if base_matched else "NA"
            evidence["ddc10_strict_term_ids"] = ";".join(item[0] for item in strict_terms) if strict_terms else "NA"
            evidence["ddc10_strict_term_count"] = len(strict_terms)
            evidence["ddc10_future_close"] = future_close

            matched = []
            future_close_term = ("DDC10_FUTURE_CLOSE", "future_min_distance_le_1p475")
            if rule_id == "DDC10_STRICT":
                if future_close and len(strict_terms) >= 2:
                    matched.append(future_close_term)
                    matched.extend(strict_terms)
            elif rule_id == "DDC10_CLOSE_ACTIVE":
                if future_close and len(replan_nodes) >= 2 and len(new_traj_nodes) >= 2:
                    matched.append(future_close_term)
                    matched.extend(
                        [
                            ("DDC10_TERM_REPLAN", "two_or_more_replan_or_stop"),
                            ("DDC10_TERM_NEW_TRAJ", "two_or_more_recent_new_traj"),
                        ]
                    )
            elif rule_id == "DDC10_CLOSE_KINEMATIC":
                kinematic_terms = [
                    item
                    for item in strict_terms
                    if item[0] in ("DDC10_TERM_OFFSET", "DDC10_TERM_HEADING", "DDC10_TERM_SHRINK")
                ]
                if future_close and kinematic_terms:
                    matched.append(future_close_term)
                    matched.extend(kinematic_terms)
            else:
                matched = []

        evidence["source_fsm_state"] = self._state(source_id)
        evidence["victim_fsm_state"] = self._state(victim_id)
        evidence["matched_condition_ids"] = ";".join(item[0] for item in matched) if matched else "NA"
        evidence["matched_condition_names"] = ";".join(item[1] for item in matched) if matched else "NA"
        evidence["matched_condition_count"] = len(matched)
        evidence["victim_id"] = victim_id
        return bool(matched), evidence

    def _random_feature_snapshot(self, source_id: int, triad: List[int], now: float, target_offset_s: float) -> Dict[str, Any]:
        evidence: Dict[str, Any] = {
            "mode": "random_packet_latency",
            "random_seed": self.random_seed,
            "random_mode": self.random_mode,
            "random_packet_probability": self.random_packet_probability,
            "random_target_index": self.random_next_target_index,
            "random_target_offset_s": target_offset_s,
            "elapsed_s": now - self.start_ros_time,
            "candidate_node_ids": self._ids(triad),
        }

        distances = self._pair_distances(triad, now)
        if distances:
            vals = [d for _pair, d in distances]
            min_pair, min_dist = min(distances, key=lambda item: item[1])
            evidence.update(
                {
                    "triad_min_pair_distance": min_dist,
                    "triad_mean_pair_distance": sum(vals) / len(vals),
                    "triad_max_pair_distance": max(vals),
                    "triad_min_pair": "{}-{}".format(min_pair[0], min_pair[1]),
                    "victim_id": self._pair_victim(source_id, min_pair),
                }
            )
        else:
            evidence["victim_id"] = self._closest_other(source_id, triad, now)

        heading_angles = []
        for i in range(len(triad)):
            for j in range(i + 1, len(triad)):
                a = self._heading(triad[i], now)
                b = self._heading(triad[j], now)
                if a is not None and b is not None:
                    heading_angles.append(self._angle_deg(a, b))
        if heading_angles:
            evidence["triad_max_heading_angle_deg"] = max(heading_angles)
            evidence["triad_mean_heading_angle_deg"] = sum(heading_angles) / len(heading_angles)

        best_shrink = -float("inf")
        for i in range(len(triad)):
            for j in range(i + 1, len(triad)):
                old_a = self._history_position_at_or_before(triad[i], now - self.history_window_s)
                old_b = self._history_position_at_or_before(triad[j], now - self.history_window_s)
                cur_a = self._position(triad[i], now)
                cur_b = self._position(triad[j], now)
                if old_a is None or old_b is None or cur_a is None or cur_b is None:
                    continue
                best_shrink = max(best_shrink, _dist(old_a, old_b) - _dist(cur_a, cur_b))
        if not math.isinf(best_shrink):
            evidence["triad_best_distance_shrink_5s"] = best_shrink

        future_1r = self._best_future_pair_distance(triad, now, 1.0 * self.drone_radius)
        future_2r = self._best_future_pair_distance(triad, now, 2.0 * self.drone_radius)
        if future_1r is not None:
            evidence["triad_future_min_distance"] = future_1r[1]
            evidence["triad_future_margin_to_1r"] = future_1r[0]
        if future_2r is not None:
            evidence["triad_future_margin_to_2r"] = future_2r[0]

        replan_nodes = [n for n in triad if self._is_replanning(n, now)]
        new_traj_nodes = [n for n in triad if self._recent_new_traj(n, now)]
        evidence["triad_replan_or_stop_count"] = len(replan_nodes)
        evidence["triad_replan_or_stop_node_ids"] = self._ids(replan_nodes)
        evidence["triad_new_traj_count"] = len(new_traj_nodes)
        evidence["triad_new_traj_node_ids"] = self._ids(new_traj_nodes)
        evidence["source_speed"] = self._speed(source_id)
        evidence["source_accel"] = self._accel(source_id)
        evidence["source_turn_angle_deg"] = self._turn_angle(source_id, now)
        evidence["source_window_move_5s"] = self._history_path_length(source_id, now - self.history_window_s, now)
        evidence["source_replan_since_traj_publish"] = self.replan_since_traj_publish.get(source_id, 0)
        evidence["swarm_avg_speed_5s"] = self._swarm_avg_speed(now)
        evidence["swarm_avg_current_speed"] = self._swarm_current_average("vel")
        evidence["swarm_avg_current_accel"] = self._swarm_current_average("accel")

        victim_id = int(evidence.get("victim_id", source_id))
        for prefix, drone_id in (("source", source_id), ("victim", victim_id)):
            obs = self._nearest_obstacle(drone_id, now)
            if obs is not None:
                evidence[prefix + "_nearest_obstacle_distance"] = obs[0]
            future_obs = self._future_obstacle_margin(drone_id, now, 2.0 * self.drone_radius)
            if future_obs is not None:
                evidence[prefix + "_future_obstacle_margin_to_2r"] = future_obs[0]
                evidence[prefix + "_future_obstacle_distance"] = future_obs[1]
            evidence[prefix + "_fsm_state"] = self._state(drone_id)
            evidence[prefix + "_trajectory_offset"] = self._trajectory_offset(drone_id, now)
            evidence[prefix + "_replan_stop_events_5s"] = self._fsm_event_count(drone_id, now, REPLAN_TRIGGER_STATES)

        return evidence

    def _match_obstacle_rule(self, rule_id: str, source_id: int, now: float) -> Tuple[bool, Dict[str, Any]]:
        receiver = self._closest_obstacle_receiver(source_id, now)
        if receiver is None:
            return False, {"reason": "missing_receiver_or_obstacle"}
        obs = self._nearest_obstacle(receiver, now)
        if obs is None:
            return False, {"reason": "missing_obstacle"}
        dist, obs_pos = obs
        victim_id = receiver
        if rule_id == "DO1":
            heading = self._heading(receiver, now)
            pos = self._position(receiver, now)
            if heading is None or pos is None:
                return False, {"reason": "missing_heading"}
            angle = self._angle_deg(heading, _sub(obs_pos, pos))
            return angle <= self.turn_angle_deg, {"heading_to_obstacle_deg": angle, "threshold_deg": self.turn_angle_deg, "obstacle_distance": dist, "victim_id": victim_id}
        if rule_id == "DO2":
            margin = self._future_obstacle_margin(receiver, now, self.drone_radius)
            if margin is None:
                return False, {"reason": "missing_future_obstacle_margin"}
            return margin[0] <= 0.0, {"future_obstacle_margin": margin[0], "future_obstacle_distance": margin[1], "threshold": self.drone_radius, "victim_id": victim_id}
        if rule_id == "DO3":
            matched = self._recent_new_traj(receiver, now)
            return matched, {"recent_new_traj": matched, "victim_id": victim_id}
        if rule_id == "DO4":
            old_dist = self._history_obstacle_distance_at_or_before(receiver, now - self.history_window_s)
            if old_dist is None:
                return False, {"reason": "missing_old_obstacle_distance"}
            shrink = old_dist - dist
            threshold = 3.0 * self.drone_radius
            return shrink >= threshold, {"obstacle_distance_shrink_5s": shrink, "threshold": threshold, "current_obstacle_distance": dist, "victim_id": victim_id}
        if rule_id == "DO5":
            threshold = 2.0 * self.drone_radius
            return dist <= threshold, {"obstacle_distance": dist, "threshold": threshold, "victim_id": victim_id}
        if rule_id == "DO6":
            matched = self._is_replanning(receiver, now)
            return matched, {"receiver_state": self._state(receiver), "recent_or_current_replan_stop": matched, "victim_id": victim_id}
        return False, {"reason": "unknown_obstacle_rule"}

    def _match_curated_obstacle_rule(self, rule_id: str, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        receiver = self._closest_obstacle_receiver(source_id, now)
        if receiver is None:
            return False, {"reason": "missing_receiver_or_obstacle"}

        evidence: Dict[str, Any] = {
            "mode": self._rule_name(rule_id),
            "or_rule_variant": rule_id,
            "candidate_node_ids": self._ids(triad),
            "victim_id": receiver,
            "source_fsm_state": self._state(source_id),
            "victim_fsm_state": self._state(receiver),
        }

        future_obs = self._future_obstacle_margin(receiver, now, 0.0)
        if future_obs is None:
            return False, {"reason": "missing_future_obstacle_margin", "victim_id": receiver}
        evidence["victim_future_obstacle_distance"] = future_obs[1]

        obs = self._nearest_obstacle(receiver, now)
        if obs is not None:
            evidence["victim_nearest_obstacle_distance"] = obs[0]

        future_pair = self._best_future_pair_distance(triad, now, 0.0)
        if future_pair is not None:
            _margin, future_dist, pair = future_pair
            evidence["triad_future_min_distance"] = future_dist
            evidence["triad_future_min_pair"] = "{}-{}".format(pair[0], pair[1])

        source_speed = self._speed(source_id)
        swarm_avg_current_speed = self._swarm_current_average("vel")
        source_speed_ratio = source_speed / swarm_avg_current_speed if swarm_avg_current_speed > 1e-6 else float("inf")
        source_turn_angle = self._turn_angle(source_id, now)
        victim_offset = self._trajectory_offset(receiver, now)
        evidence["source_speed"] = source_speed
        evidence["swarm_avg_current_speed"] = swarm_avg_current_speed
        evidence["source_speed_over_swarm_avg_current"] = source_speed_ratio
        evidence["source_turn_angle_deg"] = source_turn_angle
        evidence["victim_trajectory_offset"] = victim_offset

        victim_future_obs = float(evidence.get("victim_future_obstacle_distance", float("inf")))
        triad_future_min = float(evidence.get("triad_future_min_distance", float("inf")))
        matched: List[Tuple[str, str]] = []

        if rule_id == "DO_CURATED_1":
            if victim_future_obs <= 2.165 and source_speed_ratio <= 1.139:
                matched.extend(
                    [
                        ("DOC1_FUTURE_OBS", "victim_future_obstacle_distance_le_2p165"),
                        ("DOC1_SOURCE_SLOW_RATIO", "source_speed_over_swarm_avg_current_le_1p139"),
                    ]
                )
        elif rule_id == "DO_CURATED_2":
            if victim_future_obs <= 1.786 and victim_offset >= 0.00337:
                matched.extend(
                    [
                        ("DOC2_FUTURE_OBS", "victim_future_obstacle_distance_le_1p786"),
                        ("DOC2_VICTIM_OFFSET", "victim_trajectory_offset_ge_0p00337"),
                    ]
                )
        elif rule_id == "DO_CURATED_3":
            if triad_future_min <= 1.164 and source_speed_ratio <= 1.139 and source_turn_angle >= 7.203:
                matched.extend(
                    [
                        ("DOC3_FUTURE_PAIR", "triad_future_min_distance_le_1p164"),
                        ("DOC3_SOURCE_SLOW_RATIO", "source_speed_over_swarm_avg_current_le_1p139"),
                        ("DOC3_SOURCE_TURN", "source_turn_angle_ge_7p203deg"),
                    ]
                )
        else:
            return False, {"reason": "unknown_curated_obstacle_rule", "victim_id": receiver}

        evidence["matched_condition_ids"] = ";".join(item[0] for item in matched) if matched else "NA"
        evidence["matched_condition_names"] = ";".join(item[1] for item in matched) if matched else "NA"
        evidence["matched_condition_count"] = len(matched)
        return bool(matched), evidence

    def _match_deadlock_rule(self, rule_id: str, source_id: int, now: float) -> Tuple[bool, Dict[str, Any]]:
        nodes = self._nearest_neighbors(source_id, now, 3)
        if len(nodes) < 3:
            return False, {"reason": "not_enough_neighbors"}
        if rule_id == "DL1":
            threshold = 5.0 * self.drone_radius
            best = min((self._history_path_length(n, now - self.history_window_s, now), n) for n in nodes)
            return best[0] <= threshold, {"min_neighbor_move_5s": best[0], "threshold": threshold, "candidate_node_ids": self._ids(nodes), "victim_id": best[1]}
        if rule_id == "DL2":
            replanning = [n for n in nodes if self._is_replanning(n, now)]
            return len(replanning) >= 2, {"replanning_node_ids": self._ids(replanning), "count": len(replanning), "candidate_node_ids": self._ids(nodes), "victim_id": self._prefer_non_source(source_id, replanning, nodes, now)}
        if rule_id == "DL3":
            close_pairs = self._future_close_pairs(nodes, now, 2.0 * self.drone_radius)
            participants = sorted({x for pair, _dist_value in close_pairs for x in pair})
            return len(participants) >= 2, {"future_close_pairs": self._pairs(close_pairs), "candidate_node_ids": self._ids(nodes), "victim_id": participants[0] if participants else nodes[0]}
        if rule_id == "DL4":
            candidates = [n for n in nodes if self.replan_since_traj_publish.get(n, 0) >= 2]
            return bool(candidates), {"replan_without_new_traj_node_ids": self._ids(candidates), "candidate_node_ids": self._ids(nodes), "victim_id": candidates[0] if candidates else nodes[0]}
        if rule_id == "DL5":
            counts = [(self._fsm_event_count(n, now, REPLAN_TRIGGER_STATES), n) for n in nodes]
            best = max(counts, default=(0, nodes[0]))
            return best[0] >= 3, {"max_replan_stop_events_5s": best[0], "candidate_node_ids": self._ids(nodes), "victim_id": best[1]}
        if rule_id == "DL6":
            pairs = self._current_close_pairs(nodes, now, self.drone_radius)
            return bool(pairs), {"close_pairs": self._pairs(pairs), "threshold": self.drone_radius, "candidate_node_ids": self._ids(nodes), "victim_id": pairs[0][0][0] if pairs else nodes[0]}
        return False, {"reason": "unknown_deadlock_rule"}

    def _match_curated_deadlock_rule(self, rule_id: str, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        future_pair = self._best_future_pair_distance(triad, now, 0.0)
        if future_pair is None:
            return False, {"reason": "missing_future_pair_distance"}

        _margin, future_distance, pair = future_pair
        swarm_avg_speed_5s = self._swarm_avg_speed(now)
        speed_thresholds = {
            "DL_CURATED_1": 1.395,
            "DL_CURATED_2": 1.381,
        }
        speed_threshold = speed_thresholds.get(rule_id)
        if speed_threshold is None:
            return False, {"reason": "unknown_curated_deadlock_rule"}

        future_threshold = 1.724
        matched = future_distance <= future_threshold and swarm_avg_speed_5s <= speed_threshold
        victim_id = self._pair_victim(source_id, pair)
        evidence = {
            "mode": self._rule_name(rule_id),
            "or_rule_variant": rule_id,
            "candidate_node_ids": self._ids(triad),
            "triad_future_min_distance": future_distance,
            "triad_future_min_pair": "{}-{}".format(pair[0], pair[1]),
            "future_distance_threshold": future_threshold,
            "swarm_avg_speed_5s": swarm_avg_speed_5s,
            "swarm_avg_speed_5s_threshold": speed_threshold,
            "source_fsm_state": self._state(source_id),
            "victim_fsm_state": self._state(victim_id),
            "victim_id": victim_id,
            "matched_condition_ids": "DLC_FUTURE_PAIR;DLC_SWARM_SPEED" if matched else "NA",
            "matched_condition_names": (
                "triad_future_min_distance_le_1p724;swarm_avg_speed_5s_le_{}".format(
                    str(speed_threshold).replace(".", "p")
                )
                if matched
                else "NA"
            ),
            "matched_condition_count": 2 if matched else 0,
        }
        return matched, evidence

    def _match_delay_rule(self, rule_id: str, source_id: int, now: float) -> Tuple[bool, Dict[str, Any]]:
        nodes = self._active_others(source_id, now)
        if not nodes:
            return False, {"reason": "no_active_others"}
        if rule_id == "DE1":
            swarm_avg = self._swarm_avg_speed(now)
            slow = [(self._avg_speed_window(n, now - self.history_window_s, now), n) for n in nodes]
            matched = [(speed, n) for speed, n in slow if speed <= 0.5 * swarm_avg]
            best = min(matched, default=(float("inf"), nodes[0]))
            return bool(matched), {"swarm_avg_speed_5s": swarm_avg, "slow_speed_threshold": 0.5 * swarm_avg, "slow_node_ids": self._ids([n for _s, n in matched]), "victim_id": best[1]}
        if rule_id == "DE2":
            progress = self._progress_proxy(nodes, now)
            return progress < self.delay_progress_ratio_threshold, {"progress_proxy": progress, "threshold": self.delay_progress_ratio_threshold, "expected_m": self.delay_progress_expected_m, "victim_id": nodes[0]}
        if rule_id == "DE3":
            replanning = [n for n in nodes if self._is_replanning(n, now)]
            return len(replanning) >= 3, {"replanning_node_ids": self._ids(replanning), "count": len(replanning), "victim_id": replanning[0] if replanning else nodes[0]}
        if rule_id == "DE4":
            candidates = [n for n in nodes if self.replan_since_traj_publish.get(n, 0) >= 2]
            return bool(candidates), {"replan_without_new_traj_node_ids": self._ids(candidates), "victim_id": candidates[0] if candidates else nodes[0]}
        if rule_id == "DE5":
            pairs = self._current_close_pairs(nodes, now, self.drone_radius)
            participants = sorted({x for pair, _d in pairs for x in pair})
            return len(participants) >= 2, {"close_pairs": self._pairs(pairs), "threshold": self.drone_radius, "victim_id": participants[0] if participants else nodes[0]}
        if rule_id == "DE6":
            counts = [(self._fsm_event_count(n, now, REPLAN_TRIGGER_STATES), n) for n in nodes]
            best = max(counts, default=(0, nodes[0]))
            return best[0] >= 3, {"max_replan_stop_events_5s": best[0], "victim_id": best[1]}
        return False, {"reason": "unknown_delay_rule"}

    def _match_curated_delay_rule(self, rule_id: str, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        source_offset = self._trajectory_offset(source_id, now)
        swarm_avg_current_speed = self._swarm_current_average("vel")
        swarm_avg_speed_5s = self._swarm_avg_speed(now)
        source_accel = self._accel(source_id)
        swarm_avg_current_accel = self._swarm_current_average("accel")
        source_accel_ratio = source_accel / swarm_avg_current_accel if swarm_avg_current_accel > 1e-6 else float("inf")
        source_replans = self.replan_since_traj_publish.get(source_id, 0)

        source_obstacle = self._nearest_obstacle(source_id, now)
        source_obstacle_distance = source_obstacle[0] if source_obstacle is not None else None
        future_pair = self._best_future_pair_distance(triad, now, 0.0)
        future_distance = future_pair[1] if future_pair is not None else None
        future_pair_ids = future_pair[2] if future_pair is not None else None

        victim_id = self._closest_other(source_id, triad, now)
        if future_pair_ids is not None:
            victim_id = self._pair_victim(source_id, future_pair_ids)

        evidence: Dict[str, Any] = {
            "mode": self._rule_name(rule_id),
            "or_rule_variant": rule_id,
            "candidate_node_ids": self._ids(triad),
            "source_trajectory_offset": source_offset,
            "swarm_avg_current_speed": swarm_avg_current_speed,
            "swarm_avg_speed_5s": swarm_avg_speed_5s,
            "source_accel": source_accel,
            "swarm_avg_current_accel": swarm_avg_current_accel,
            "source_accel_over_swarm_avg_current": source_accel_ratio,
            "source_replan_since_traj_publish": source_replans,
            "source_fsm_state": self._state(source_id),
            "victim_fsm_state": self._state(victim_id),
            "victim_id": victim_id,
        }
        if source_obstacle_distance is not None:
            evidence["source_nearest_obstacle_distance"] = source_obstacle_distance
        if future_distance is not None and future_pair_ids is not None:
            evidence["triad_future_min_distance"] = future_distance
            evidence["triad_future_min_pair"] = "{}-{}".format(future_pair_ids[0], future_pair_ids[1])

        matched_conditions: List[Tuple[str, str]] = []
        if rule_id == "DE_CURATED_1":
            if source_obstacle_distance is not None and source_offset >= 0.001889 and swarm_avg_current_speed <= 1.402 and source_obstacle_distance >= 1.044:
                matched_conditions = [
                    ("DEC1_SOURCE_OFFSET", "source_trajectory_offset_ge_0p001889"),
                    ("DEC1_CURRENT_SPEED", "swarm_avg_current_speed_le_1p402"),
                    ("DEC1_SOURCE_OBSTACLE", "source_nearest_obstacle_distance_ge_1p044"),
                ]
        elif rule_id == "DE_CURATED_2":
            if source_obstacle_distance is not None and future_distance is not None and swarm_avg_current_speed <= 1.402 and future_distance <= 1.716 and source_obstacle_distance >= 1.166:
                matched_conditions = [
                    ("DEC2_CURRENT_SPEED", "swarm_avg_current_speed_le_1p402"),
                    ("DEC2_FUTURE_PAIR", "triad_future_min_distance_le_1p716"),
                    ("DEC2_SOURCE_OBSTACLE", "source_nearest_obstacle_distance_ge_1p166"),
                ]
        elif rule_id == "DE_CURATED_3":
            if future_distance is not None and swarm_avg_speed_5s <= 1.386 and future_distance <= 1.716 and source_accel_ratio <= 2.401:
                matched_conditions = [
                    ("DEC3_WINDOW_SPEED", "swarm_avg_speed_5s_le_1p386"),
                    ("DEC3_FUTURE_PAIR", "triad_future_min_distance_le_1p716"),
                    ("DEC3_ACCEL_RATIO", "source_accel_over_swarm_avg_current_le_2p401"),
                ]
        elif rule_id == "DE_CURATED_4":
            if swarm_avg_speed_5s <= 1.406 and swarm_avg_current_speed <= 1.383 and source_replans <= 0:
                matched_conditions = [
                    ("DEC4_WINDOW_SPEED", "swarm_avg_speed_5s_le_1p406"),
                    ("DEC4_CURRENT_SPEED", "swarm_avg_current_speed_le_1p383"),
                    ("DEC4_REPLAN_COUNT", "source_replan_since_traj_publish_le_0"),
                ]
        else:
            return False, {"reason": "unknown_curated_delay_rule"}

        evidence["matched_condition_ids"] = ";".join(item[0] for item in matched_conditions) if matched_conditions else "NA"
        evidence["matched_condition_names"] = ";".join(item[1] for item in matched_conditions) if matched_conditions else "NA"
        evidence["matched_condition_count"] = len(matched_conditions)
        return bool(matched_conditions), evidence

    def _match_planning_failure_rule(self, rule_id: str, source_id: int, now: float) -> Tuple[bool, Dict[str, Any]]:
        if rule_id == "PF1":
            nodes = [source_id] + self._active_others(source_id, now)
            pairs = self._future_close_pairs(nodes, now, 2.0 * self.drone_radius)
            best = min(pairs, key=lambda item: item[1], default=None)
            return bool(best), {"future_close_pairs": self._pairs(pairs), "threshold": 2.0 * self.drone_radius, "victim_id": self._pair_victim(source_id, best[0]) if best else source_id}
        if rule_id == "PF2":
            margin = self._future_obstacle_margin(source_id, now, 2.0 * self.drone_radius)
            if margin is None:
                return False, {"reason": "missing_future_obstacle_margin"}
            return margin[0] <= 0.0, {"future_obstacle_margin": margin[0], "future_obstacle_distance": margin[1], "threshold": 2.0 * self.drone_radius, "victim_id": source_id}
        return False, {"reason": "unknown_planning_failure_rule"}

    def _match_curated_planning_failure_rule(self, rule_id: str, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        future_pair = self._best_future_pair_distance(triad, now, 0.0)
        if future_pair is None:
            return False, {"reason": "missing_future_pair_distance"}

        _margin, future_distance, pair = future_pair
        victim_id = self._pair_victim(source_id, pair)
        swarm_avg_current_speed = self._swarm_current_average("vel")
        victim_offset = self._trajectory_offset(victim_id, now)
        victim_future_obstacle = self._future_obstacle_margin(victim_id, now, 0.0)
        victim_future_obstacle_distance = victim_future_obstacle[1] if victim_future_obstacle is not None else None
        replan_nodes = [drone_id for drone_id in triad if self._is_replanning(drone_id, now)]
        new_traj_nodes = [drone_id for drone_id in triad if self._recent_new_traj(drone_id, now)]

        evidence: Dict[str, Any] = {
            "mode": self._rule_name(rule_id),
            "or_rule_variant": rule_id,
            "candidate_node_ids": self._ids(triad),
            "triad_future_min_distance": future_distance,
            "triad_future_min_pair": "{}-{}".format(pair[0], pair[1]),
            "swarm_avg_current_speed": swarm_avg_current_speed,
            "victim_trajectory_offset": victim_offset,
            "triad_replan_or_stop_count": len(replan_nodes),
            "triad_replan_or_stop_node_ids": self._ids(replan_nodes),
            "triad_new_traj_count": len(new_traj_nodes),
            "triad_new_traj_node_ids": self._ids(new_traj_nodes),
            "source_fsm_state": self._state(source_id),
            "victim_fsm_state": self._state(victim_id),
            "victim_id": victim_id,
        }
        if victim_future_obstacle_distance is not None:
            evidence["victim_future_obstacle_distance"] = victim_future_obstacle_distance

        matched_conditions: List[Tuple[str, str]] = []
        if rule_id == "PF_CURATED_1":
            if swarm_avg_current_speed >= 1.284 and future_distance <= 1.475:
                matched_conditions = [
                    ("PFC1_CURRENT_SPEED", "swarm_avg_current_speed_ge_1p284"),
                    ("PFC1_FUTURE_PAIR", "triad_future_min_distance_le_1p475"),
                ]
        elif rule_id == "PF_CURATED_2":
            if future_distance <= 1.164 and swarm_avg_current_speed >= 1.196 and victim_offset >= 0.00337:
                matched_conditions = [
                    ("PFC2_FUTURE_PAIR", "triad_future_min_distance_le_1p164"),
                    ("PFC2_CURRENT_SPEED", "swarm_avg_current_speed_ge_1p196"),
                    ("PFC2_VICTIM_OFFSET", "victim_trajectory_offset_ge_0p00337"),
                ]
        elif rule_id == "PF_CURATED_3":
            if victim_future_obstacle_distance is not None and future_distance <= 1.164 and victim_future_obstacle_distance <= 1.786:
                matched_conditions = [
                    ("PFC3_FUTURE_PAIR", "triad_future_min_distance_le_1p164"),
                    ("PFC3_VICTIM_OBSTACLE", "victim_future_obstacle_distance_le_1p786"),
                ]
        elif rule_id == "PF_CURATED_4":
            if len(replan_nodes) >= 2 and len(new_traj_nodes) >= 2 and future_distance <= 1.475:
                matched_conditions = [
                    ("PFC4_REPLAN_COUNT", "triad_replan_or_stop_count_ge_2"),
                    ("PFC4_NEW_TRAJ_COUNT", "triad_new_traj_count_ge_2"),
                    ("PFC4_FUTURE_PAIR", "triad_future_min_distance_le_1p475"),
                ]
        else:
            return False, {"reason": "unknown_curated_planning_failure_rule"}

        evidence["matched_condition_ids"] = ";".join(item[0] for item in matched_conditions) if matched_conditions else "NA"
        evidence["matched_condition_names"] = ";".join(item[1] for item in matched_conditions) if matched_conditions else "NA"
        evidence["matched_condition_count"] = len(matched_conditions)
        return bool(matched_conditions), evidence

    def _match_curated_drift_rule(self, rule_id: str, source_id: int, triad: List[int], now: float) -> Tuple[bool, Dict[str, Any]]:
        pair_distances = self._pair_distances(triad, now)
        if pair_distances:
            closest_pair, closest_distance = min(pair_distances, key=lambda item: item[1])
            victim_id = self._pair_victim(source_id, closest_pair)
        else:
            closest_pair = None
            closest_distance = None
            victim_id = self._closest_other(source_id, triad, now)

        heading_angles = []
        for i in range(len(triad)):
            for j in range(i + 1, len(triad)):
                heading_a = self._heading(triad[i], now)
                heading_b = self._heading(triad[j], now)
                if heading_a is not None and heading_b is not None:
                    heading_angles.append(self._angle_deg(heading_a, heading_b))
        heading_mean = sum(heading_angles) / len(heading_angles) if heading_angles else None
        heading_max = max(heading_angles) if heading_angles else None

        future_obstacles = []
        for drone_id in triad:
            result = self._future_obstacle_margin(drone_id, now, 0.0)
            if result is not None:
                future_obstacles.append((result[1], drone_id))
        if future_obstacles:
            triad_future_obstacle, triad_obstacle_drone_id = min(future_obstacles)
        else:
            triad_future_obstacle = None
            triad_obstacle_drone_id = None

        victim_future_obstacle = self._future_obstacle_margin(victim_id, now, 0.0)
        victim_future_obstacle_distance = victim_future_obstacle[1] if victim_future_obstacle is not None else None
        victim_offset = self._trajectory_offset(victim_id, now)
        swarm_avg_current_accel = self._swarm_current_average("accel")
        swarm_avg_speed_5s = self._swarm_avg_speed(now)
        source_window_move_5s = self._history_path_length(source_id, now - self.history_window_s, now)

        evidence: Dict[str, Any] = {
            "mode": self._rule_name(rule_id),
            "rule_id": rule_id,
            "candidate_node_ids": self._ids(triad),
            "attacker_id": source_id,
            "victim_id": victim_id,
            "source_fsm_state": self._state(source_id),
            "victim_fsm_state": self._state(victim_id),
            "triad_min_pair_distance": closest_distance,
            "triad_min_pair": "{}-{}".format(*closest_pair) if closest_pair is not None else "NA",
            "triad_heading_angles_deg": heading_angles,
            "triad_mean_heading_angle_deg": heading_mean,
            "triad_max_heading_angle_deg": heading_max,
            "victim_future_obstacle_distance": victim_future_obstacle_distance,
            "triad_future_min_obstacle_distance": triad_future_obstacle,
            "triad_future_min_obstacle_drone_id": triad_obstacle_drone_id,
            "victim_trajectory_offset": victim_offset,
            "source_speed": self._speed(source_id),
            "source_accel": self._accel(source_id),
            "swarm_avg_current_accel": swarm_avg_current_accel,
            "swarm_avg_speed_5s": swarm_avg_speed_5s,
            "source_window_move_5s": source_window_move_5s,
        }

        if rule_id == "DR_CURATED_1":
            checks = [
                ("DRC1_VICTIM_OBSTACLE", "victim_future_obstacle_distance", victim_future_obstacle_distance, "<=", 0.9127),
                ("DRC1_SWARM_ACCEL", "swarm_avg_current_accel", swarm_avg_current_accel, ">=", 0.2829),
                ("DRC1_TRIAD_HEADING_MEAN", "triad_mean_heading_angle_deg", heading_mean, ">=", 8.107),
            ]
        elif rule_id == "DR_CURATED_2":
            checks = [
                ("DRC2_TRIAD_OBSTACLE", "triad_future_min_obstacle_distance", triad_future_obstacle, "<=", 0.5925),
                ("DRC2_TRIAD_HEADING_MEAN", "triad_mean_heading_angle_deg", heading_mean, ">=", 8.107),
                ("DRC2_SOURCE_MOVE_5S", "source_window_move_5s", source_window_move_5s, "<=", 7.228),
            ]
        elif rule_id == "DR_CURATED_3":
            checks = [
                ("DRC3_VICTIM_OFFSET", "victim_trajectory_offset", victim_offset, ">=", 0.008051),
                ("DRC3_SWARM_SPEED_5S", "swarm_avg_speed_5s", swarm_avg_speed_5s, ">=", 1.231),
                ("DRC3_TRIAD_HEADING_MAX", "triad_max_heading_angle_deg", heading_max, ">=", 10.39),
            ]
        elif rule_id == "DR_CURATED_5":
            checks = [
                ("DRC5_VICTIM_OBSTACLE", "victim_future_obstacle_distance", victim_future_obstacle_distance, "<=", 0.7357),
                ("DRC5_TRIAD_HEADING_MEAN", "triad_mean_heading_angle_deg", heading_mean, ">=", 8.107),
                ("DRC5_TRIAD_HEADING_MAX", "triad_max_heading_angle_deg", heading_max, ">=", 10.98),
            ]
        elif rule_id == "DR_CURATED_6":
            checks = [
                ("DRC6_VICTIM_OBSTACLE", "victim_future_obstacle_distance", victim_future_obstacle_distance, "<=", 0.9127),
                ("DRC6_SWARM_ACCEL", "swarm_avg_current_accel", swarm_avg_current_accel, ">=", 0.2829),
            ]
        else:
            return False, {"reason": "unknown_curated_drift_rule", "rule_id": rule_id}

        def check_passed(value: Any, operator: str, threshold: float) -> bool:
            if value is None:
                return False
            return value <= threshold if operator == "<=" else value >= threshold

        check_results = {
            condition_id: {
                "field": field,
                "value": value,
                "operator": operator,
                "threshold": threshold,
                "passed": check_passed(value, operator, threshold),
            }
            for condition_id, field, value, operator, threshold in checks
        }
        matched = all(item["passed"] for item in check_results.values())
        evidence["condition_checks"] = check_results
        evidence["matched_condition_ids"] = ";".join(check_results) if matched else "NA"
        evidence["matched_condition_names"] = ";".join(item[1] for item in checks) if matched else "NA"
        evidence["matched_condition_count"] = len(check_results) if matched else 0
        return matched, evidence

    def _match_drift_rule(self, rule_id: str, source_id: int, now: float) -> Tuple[bool, Dict[str, Any]]:
        if rule_id == "DR1":
            speed = self._speed(source_id)
            avg = self._swarm_current_average("vel")
            return speed > avg and avg > 0.0, {"speed": speed, "swarm_avg_speed": avg, "victim_id": source_id}
        if rule_id == "DR2":
            accel = self._accel(source_id)
            avg = self._swarm_current_average("accel")
            return accel > avg and avg > 0.0, {"accel": accel, "swarm_avg_accel": avg, "victim_id": source_id}
        if rule_id == "DR3":
            angle = self._turn_angle(source_id, now)
            return angle >= self.turn_angle_deg, {"turn_angle_deg": angle, "threshold_deg": self.turn_angle_deg, "victim_id": source_id}
        if rule_id == "DR4":
            offset = self._trajectory_offset(source_id, now)
            return offset >= self.traj_offset_threshold, {"trajectory_offset": offset, "threshold": self.traj_offset_threshold, "victim_id": source_id}
        return False, {"reason": "unknown_drift_rule"}

    def _pair_distances(self, triad: List[int], now: float) -> List[Tuple[Tuple[int, int], float]]:
        out = []
        for i in range(len(triad)):
            for j in range(i + 1, len(triad)):
                a = self._position(triad[i], now)
                b = self._position(triad[j], now)
                if a is None or b is None:
                    continue
                out.append(((triad[i], triad[j]), _dist(a, b)))
        return out

    def _nearest_neighbors(self, source_id: int, now: float, count: int) -> List[int]:
        source_pos = self._position(source_id, now)
        if source_pos is None:
            return []
        ranked: List[Tuple[float, int]] = []
        for drone_id in range(self.drone_num):
            if drone_id == source_id or not self._is_active(drone_id):
                continue
            pos = self._position(drone_id, now)
            if pos is not None:
                ranked.append((_dist(source_pos, pos), drone_id))
        ranked.sort(key=lambda item: item[0])
        return [drone_id for _dist_value, drone_id in ranked[:count]]

    def _active_others(self, source_id: int, now: float) -> List[int]:
        return [drone_id for drone_id in range(self.drone_num) if drone_id != source_id and self._is_active(drone_id) and self._position(drone_id, now) is not None]

    def _closest_obstacle_receiver(self, source_id: int, now: float) -> Optional[int]:
        best: Optional[Tuple[float, int]] = None
        for drone_id in self._active_others(source_id, now):
            obs = self._nearest_obstacle(drone_id, now)
            if obs is None:
                continue
            item = (obs[0], drone_id)
            if best is None or item[0] < best[0]:
                best = item
        return best[1] if best is not None else None

    def _nearest_obstacle(self, drone_id: int, now: float) -> Optional[Tuple[float, List[float]]]:
        pos = self._position(drone_id, now)
        if pos is None or not self.obstacle_points:
            return None
        best: Optional[Tuple[float, List[float]]] = None
        for pt in self.obstacle_points:
            d = _dist(pos, pt)
            if best is None or d < best[0]:
                best = (d, pt)
        return best

    def _history_obstacle_distance_at_or_before(self, drone_id: int, target_time: float) -> Optional[float]:
        pos = self._history_position_at_or_before(drone_id, target_time)
        if pos is None or not self.obstacle_points:
            return None
        return min(_dist(pos, pt) for pt in self.obstacle_points)

    def _future_obstacle_margin(self, drone_id: int, now: float, threshold: float) -> Optional[Tuple[float, float]]:
        traj = self.local_trajs.get(drone_id)
        if traj is None or not self.obstacle_points:
            return None
        steps = max(1, int(math.ceil(max(self.future_horizon_s, 0.0) / max(self.future_sample_dt, 0.05))))
        min_dist: Optional[float] = None
        for idx in range(steps + 1):
            pos = traj.pos_at_abs_time(now + idx * self.future_sample_dt)
            if pos is None:
                continue
            d = min(_dist(pos, pt) for pt in self.obstacle_points)
            if min_dist is None or d < min_dist:
                min_dist = d
        if min_dist is None:
            return None
        return min_dist - threshold, min_dist

    def _current_close_pairs(self, nodes: List[int], now: float, threshold: float) -> List[Tuple[Tuple[int, int], float]]:
        out = []
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                a = self._position(nodes[i], now)
                b = self._position(nodes[j], now)
                if a is None or b is None:
                    continue
                d = _dist(a, b)
                if d <= threshold:
                    out.append(((nodes[i], nodes[j]), d))
        return out

    def _future_close_pairs(self, nodes: List[int], now: float, threshold: float) -> List[Tuple[Tuple[int, int], float]]:
        out = []
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                result = self._future_pair_margin(nodes[i], nodes[j], now, threshold)
                if result is None:
                    continue
                margin, dist = result
                if margin <= 0.0:
                    out.append(((nodes[i], nodes[j]), dist))
        return out

    def _best_future_pair_distance(self, nodes: List[int], now: float, threshold: float) -> Optional[Tuple[float, float, Tuple[int, int]]]:
        best: Optional[Tuple[float, float, Tuple[int, int]]] = None
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                result = self._future_pair_margin(nodes[i], nodes[j], now, threshold)
                if result is None:
                    continue
                margin, dist = result
                if best is None or dist < best[1]:
                    best = (margin, dist, (nodes[i], nodes[j]))
        return best

    def _history_path_length(self, drone_id: int, start_time: float, end_time: float) -> float:
        hist = self.odom_history.get(drone_id)
        if not hist:
            return float("inf")
        samples = [sample for sample in hist if start_time <= float(sample["t"]) <= end_time]
        if len(samples) < 2:
            return float("inf")
        length = 0.0
        for prev, cur in zip(samples, samples[1:]):
            length += _dist(list(prev["pos"]), list(cur["pos"]))
        return length

    def _avg_speed_window(self, drone_id: int, start_time: float, end_time: float) -> float:
        hist = self.odom_history.get(drone_id)
        if not hist:
            return float("inf")
        vals = [_norm(list(sample.get("vel", [0.0, 0.0, 0.0]))) for sample in hist if start_time <= float(sample["t"]) <= end_time]
        return sum(vals) / len(vals) if vals else float("inf")

    def _swarm_avg_speed(self, now: float) -> float:
        speeds = []
        for drone_id in range(self.drone_num):
            if self._is_active(drone_id):
                v = self._avg_speed_window(drone_id, now - self.history_window_s, now)
                if not math.isinf(v):
                    speeds.append(v)
        return sum(speeds) / len(speeds) if speeds else 0.0

    def _progress_proxy(self, nodes: List[int], now: float) -> float:
        lengths = [self._history_path_length(n, now - self.history_window_s, now) for n in nodes]
        vals = [x for x in lengths if not math.isinf(x)]
        if not vals:
            return 1.0
        expected = max(1e-3, self.delay_progress_expected_m)
        return min(1.0, (sum(vals) / len(vals)) / expected)

    def _fsm_event_count(self, drone_id: int, now: float, states: Iterable[str]) -> int:
        wanted = set(states)
        hist = self.fsm_history.get(drone_id)
        if not hist:
            return 0
        start = now - self.history_window_s
        return sum(1 for event in hist if start <= float(event["t"]) <= now and str(event["state"]) in wanted)

    def _speed(self, drone_id: int) -> float:
        odom = self.odom_state.get(drone_id)
        if odom is None:
            return 0.0
        return _norm(list(odom.get("vel", [0.0, 0.0, 0.0])))

    def _accel(self, drone_id: int) -> float:
        odom = self.odom_state.get(drone_id)
        if odom is None:
            return 0.0
        return _norm(list(odom.get("accel", [0.0, 0.0, 0.0])))

    def _swarm_current_average(self, key: str) -> float:
        values = []
        for drone_id in range(self.drone_num):
            odom = self.odom_state.get(drone_id)
            if odom is None or not self._is_active(drone_id):
                continue
            values.append(_norm(list(odom.get(key, [0.0, 0.0, 0.0]))))
        return sum(values) / len(values) if values else 0.0

    def _turn_angle(self, drone_id: int, now: float) -> float:
        cur = self._heading(drone_id, now)
        old = self._history_velocity_at_or_before(drone_id, now - 1.0)
        if cur is None or old is None or _norm(old) <= 1e-3:
            return 0.0
        return self._angle_deg(old, cur)

    def _trajectory_offset(self, drone_id: int, now: float) -> float:
        cur = self.local_trajs.get(drone_id)
        prev = self.prev_local_trajs.get(drone_id)
        if cur is None or prev is None:
            return 0.0
        t_abs = now + min(1.0, max(0.2, self.future_horizon_s * 0.33))
        a = cur.pos_at_abs_time(t_abs)
        b = prev.pos_at_abs_time(t_abs)
        if a is None or b is None:
            return 0.0
        return _dist(a, b)

    def _history_velocity_at_or_before(self, drone_id: int, target_time: float) -> Optional[List[float]]:
        hist = self.odom_history.get(drone_id)
        if not hist:
            return None
        best = None
        for sample in hist:
            if float(sample["t"]) <= target_time:
                best = sample
            else:
                break
        return list(best.get("vel", [0.0, 0.0, 0.0])) if best is not None else None

    def _ids(self, nodes: Iterable[int]) -> str:
        return ";".join(str(x) for x in nodes) if nodes else "NA"

    def _pairs(self, pairs: Iterable[Tuple[Tuple[int, int], float]]) -> str:
        out = []
        for pair, dist_value in pairs:
            out.append("{}-{}:{:.4f}".format(pair[0], pair[1], dist_value))
        return ";".join(out) if out else "NA"

    def _future_pair_margin(self, a_id: int, b_id: int, now: float, threshold: Optional[float] = None) -> Optional[Tuple[float, float]]:
        a_traj = self.local_trajs.get(a_id)
        b_traj = self.local_trajs.get(b_id)
        if a_traj is None or b_traj is None:
            return None
        if threshold is None:
            threshold = 2.0 * self.drone_radius
        steps = max(1, int(math.ceil(max(self.future_horizon_s, 0.0) / max(self.future_sample_dt, 0.05))))
        min_dist = None
        for idx in range(steps + 1):
            t_abs = now + idx * self.future_sample_dt
            a = a_traj.pos_at_abs_time(t_abs)
            b = b_traj.pos_at_abs_time(t_abs)
            if a is None or b is None:
                continue
            d = _dist(a, b)
            if min_dist is None or d < min_dist:
                min_dist = d
        if min_dist is None:
            return None
        return min_dist - threshold, min_dist

    def _position(self, drone_id: int, now: float) -> Optional[List[float]]:
        odom = self.odom_state.get(drone_id)
        if odom is not None and now - float(odom.get("t", -1e9)) <= 0.5:
            return list(odom["pos"])
        traj = self.local_trajs.get(drone_id)
        return traj.pos_at_abs_time(now) if traj is not None else None

    def _history_position_at_or_before(self, drone_id: int, target_time: float) -> Optional[List[float]]:
        hist = self.odom_history.get(drone_id)
        if not hist:
            return None
        best = None
        for sample in hist:
            if float(sample["t"]) <= target_time:
                best = sample
            else:
                break
        return list(best["pos"]) if best is not None else None

    def _heading(self, drone_id: int, now: float) -> Optional[List[float]]:
        odom = self.odom_state.get(drone_id)
        if odom is not None:
            vel = list(odom.get("vel", [0.0, 0.0, 0.0]))
            if _norm(vel) > 1e-3:
                return vel
        traj = self.local_trajs.get(drone_id)
        if traj is None:
            return None
        a = traj.pos_at_abs_time(now)
        b = traj.pos_at_abs_time(now + 0.2)
        if a is None or b is None:
            return None
        v = _sub(b, a)
        return v if _norm(v) > 1e-3 else None

    def _angle_deg(self, a: List[float], b: List[float]) -> float:
        na = _norm(a)
        nb = _norm(b)
        if na <= 1e-6 or nb <= 1e-6:
            return 180.0
        c = max(-1.0, min(1.0, _dot(a, b) / (na * nb)))
        return math.degrees(math.acos(c))

    def _is_active(self, drone_id: int) -> bool:
        state = self._state(drone_id)
        return state in TASK_ACTIVE_FSM_STATES or drone_id in self.local_trajs

    def _state(self, drone_id: int) -> str:
        return str(self.fsm_state.get(drone_id, {}).get("state", "NA"))

    def _is_replanning(self, drone_id: int, now: float) -> bool:
        if self._state(drone_id) in REPLAN_TRIGGER_STATES:
            return True
        return self._recent_new_traj(drone_id, now)

    def _recent_new_traj(self, drone_id: int, now: float) -> bool:
        t = self.last_traj_change_time.get(drone_id)
        if t is not None and now - t <= self.trigger_window_s:
            return True
        return self._state(drone_id) in NEW_TRAJ_STATES and self._state(drone_id) != "NA"

    def _observe_period_locked(self, drone_id: int, period_ms: float) -> None:
        if period_ms <= 1.0 or period_ms > 5000.0:
            return
        old = self.period_ema_ms.get(drone_id)
        self.period_ema_ms[drone_id] = period_ms if old is None else 0.75 * old + 0.25 * period_ms

    def _latency_before_next_packet(self, source_id: int) -> Tuple[Optional[float], Dict[str, Any]]:
        period = self.period_ema_ms.get(source_id, self.period_fallback_ms)
        target_latency = period - self.latency_epsilon_ms
        protocol_limit = self.protocol_timeout_ms * self.max_latency_ratio
        safe_limit = min(protocol_limit, self.latency_max_ms) if self.latency_max_ms > 0.0 else protocol_limit
        details = {
            "estimated_period_ms": period,
            "target_latency_ms": target_latency,
            "protocol_safe_max_ms": protocol_limit,
            "effective_safe_max_ms": safe_limit,
            "configured_latency_cap_ms": self.latency_max_ms,
        }
        if target_latency < self.latency_min_ms:
            details["skip_reason"] = "target_below_minimum_latency"
            return None, details
        if target_latency > protocol_limit:
            details["skip_reason"] = "target_exceeds_protocol_budget"
            return None, details
        if self.latency_max_ms > 0.0 and target_latency > self.latency_max_ms:
            details["skip_reason"] = "target_exceeds_configured_cap"
            return None, details
        return target_latency, details

    def _enqueue_publish(self, source_id: int, msg: MINCOTraj, delay_s: float) -> None:
        now = time.monotonic()
        with self.lock:
            queue = self.publish_queues.setdefault(source_id, deque())
            self.publish_timers.setdefault(source_id, None)
            self.publish_busy.setdefault(source_id, False)
            due = now + max(0.0, delay_s)
            if queue:
                due = max(due, queue[-1][0] + 1e-6)
            queue.append((due, copy.deepcopy(msg)))
            self._schedule_publish_locked(source_id)

    def _schedule_publish_locked(self, source_id: int) -> None:
        if self.publish_busy.get(source_id, False) or self.publish_timers.get(source_id) is not None:
            return
        queue = self.publish_queues.get(source_id)
        if not queue:
            return
        delay_s = max(0.0, queue[0][0] - time.monotonic())
        timer = threading.Timer(delay_s, self._publish_next, args=(source_id,))
        timer.daemon = True
        self.publish_timers[source_id] = timer
        timer.start()

    def _publish_next(self, source_id: int) -> None:
        with self.lock:
            self.publish_timers[source_id] = None
            queue = self.publish_queues.get(source_id)
            if self.publish_busy.get(source_id, False) or not queue:
                return
            due, msg = queue[0]
            remaining_s = due - time.monotonic()
            if remaining_s > 0.0:
                self._schedule_publish_locked(source_id)
                return
            queue.popleft()
            self.publish_busy[source_id] = True
        try:
            self._publish_msg(msg)
        except Exception as exc:
            rospy.logwarn("[rule_pool_latency_proxy] publish failed for source=%d: %s", source_id, str(exc))
        finally:
            with self.lock:
                self.publish_busy[source_id] = False
                self._schedule_publish_locked(source_id)

    def _closest_other(self, source_id: int, triad: Iterable[int], now: float) -> int:
        source_pos = self._position(source_id, now)
        if source_pos is None:
            return next((x for x in triad if x != source_id), source_id)
        ranked = []
        for drone_id in triad:
            if drone_id == source_id:
                continue
            pos = self._position(drone_id, now)
            if pos is not None:
                ranked.append((_dist(source_pos, pos), drone_id))
        return sorted(ranked)[0][1] if ranked else source_id

    def _pair_victim(self, source_id: int, pair: Tuple[int, int]) -> int:
        if pair[0] == source_id:
            return pair[1]
        if pair[1] == source_id:
            return pair[0]
        return pair[0]

    def _prefer_non_source(self, source_id: int, candidates: List[int], triad: List[int], now: float) -> int:
        for drone_id in candidates:
            if drone_id != source_id:
                return drone_id
        return self._closest_other(source_id, triad, now)

    def _rule_name(self, rule_id: str) -> str:
        components = ddc10_or_components(rule_id)
        if components:
            suffix = "_".join(item.rsplit("_", 1)[1] for item in components)
            return "dd_curated_target1_or_" + suffix
        return RULE_NAMES.get(rule_id, "unknown")

    def _publish_msg(self, msg: MINCOTraj) -> None:
        self.pub.publish(msg)

    def _ensure_events_csv(self) -> None:
        if os.path.exists(self.events_csv_path):
            return
        with open(self.events_csv_path, "w", encoding="ascii", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "injection_index",
                    "inject_time_s",
                    "rule_id",
                    "rule_name",
                    "attacker_drone_id",
                    "victim_drone_id",
                    "triad_node_ids",
                    "latency_ms",
                    "target_latency_ms",
                    "protocol_safe_max_ms",
                    "effective_safe_max_ms",
                    "configured_latency_cap_ms",
                    "estimated_period_ms",
                    "attacker_fsm_state",
                    "victim_fsm_state",
                    "source_traj_id",
                    "victim_traj_id",
                    "trigger_evidence",
                ] + (["all_matched_rule_ids"] + list(DDC10_RULE_IDS) if self.screening_mode else [])
            )

    def _append_event_locked(self, event: Dict[str, Any]) -> None:
        try:
            with open(self.events_csv_path, "a", encoding="ascii", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        event.get("injection_index", "NA"),
                        "{:.6f}".format(float(event.get("inject_time_s", 0.0))),
                        event.get("rule_id", "NA"),
                        event.get("rule_name", "NA"),
                        event.get("attacker_drone_id", "NA"),
                        event.get("victim_drone_id", "NA"),
                        event.get("triad_node_ids", "NA"),
                        "{:.6f}".format(float(event.get("latency_ms", 0.0))),
                        "{:.6f}".format(float(event.get("target_latency_ms", 0.0))),
                        "{:.6f}".format(float(event.get("protocol_safe_max_ms", 0.0))),
                        "{:.6f}".format(float(event.get("effective_safe_max_ms", 0.0))),
                        "{:.6f}".format(float(event.get("configured_latency_cap_ms", 0.0))),
                        "{:.6f}".format(float(event.get("estimated_period_ms", 0.0))),
                        event.get("attacker_fsm_state", "NA"),
                        event.get("victim_fsm_state", "NA"),
                        event.get("source_traj_id", "NA"),
                        event.get("victim_traj_id", "NA"),
                        event.get("trigger_evidence", "{}"),
                    ] + self._screening_event_values(event)
                )
        except Exception as exc:
            rospy.logwarn("[rule_pool_latency_proxy] failed to append event csv: %s", str(exc))

    def _screening_event_values(self, event: Dict[str, Any]) -> List[Any]:
        if not self.screening_mode:
            return []
        evidence = json.loads(event.get("trigger_evidence", "{}"))
        return [evidence.get("all_matched_rule_ids", "")] + [
            int(evidence.get("all_rule_matches", {}).get(rule_id, False)) for rule_id in DDC10_RULE_IDS
        ]

    def _append_trace(self, event_name: str, payload: Dict[str, Any]) -> None:
        line = dict(payload)
        line["event"] = event_name
        try:
            with open(self.trace_jsonl_path, "a", encoding="ascii") as f:
                f.write(json.dumps(line, ensure_ascii=True, sort_keys=True) + "\n")
        except Exception:
            pass

    def _write_meta(self, final: bool) -> None:
        meta = {
            "enabled": self.enabled,
            "active_rule_id": self.active_rule_id,
            "active_rule_name": self._rule_name(self.active_rule_id),
            "in_topic": self.in_topic,
            "out_topic": self.out_topic,
            "run_id": self.run_id,
            "start_wall_time": self.start_wall_time,
            "start_ros_time": self.start_ros_time,
            "final": final,
            "wall_time": _now_iso(),
            "msg_seen": self.msg_seen,
            "msg_delayed": self.msg_delayed,
            "actual_injection_count": self.injection_count,
            "delay_schedule_skip_counts": dict(self.delay_schedule_skip_counts),
            "events_csv": self.events_csv_path,
            "trace_jsonl": self.trace_jsonl_path,
            "screening": {
                "enabled": self.screening_mode,
                "monitor_only": self.monitor_only,
                "observations_file": self.observations_path if self.screening_mode else "",
                "observation_count": self.screening_observation_count,
                "pre_injection_observation_count": self.screening_pre_observation_count,
                "evaluation_errors": self.screening_evaluation_errors,
                "rule_match_counts": self.screening_rule_counts,
                "feasible_rule_match_counts": self.screening_feasible_counts,
                "feasible_rule_pair_match_counts": self.screening_pair_counts,
                "first_collision_ros_time": self.first_collision_ros_time,
                "first_drone_collision_ros_time": self.first_drone_collision_ros_time,
            },
            "params": {
                "max_injections_per_round": self.max_injections,
                "cooldown_s": self.cooldown_s,
                "history_window_s": self.history_window_s,
                "trigger_window_s": self.trigger_window_s,
                "future_horizon_s": self.future_horizon_s,
                "future_sample_dt": self.future_sample_dt,
                "heading_aligned_deg": self.heading_aligned_deg,
                "future_collision_margin": self.future_collision_margin,
                "latency_epsilon_ms": self.latency_epsilon_ms,
                "latency_min_ms": self.latency_min_ms,
                "latency_max_ms": self.latency_max_ms,
                "protocol_timeout_ms": self.protocol_timeout_ms,
                "max_latency_ratio": self.max_latency_ratio,
                "protocol_safe_max_ms": self.protocol_timeout_ms * self.max_latency_ratio,
                "configured_latency_cap_ms": self.latency_max_ms,
                "effective_safe_max_ms": min(
                    self.protocol_timeout_ms * self.max_latency_ratio,
                    self.latency_max_ms,
                ) if self.latency_max_ms > 0.0 else self.protocol_timeout_ms * self.max_latency_ratio,
                "period_fallback_ms": self.period_fallback_ms,
                "random_seed": self.random_seed,
                "random_mode": self.random_mode,
                "random_packet_probability": self.random_packet_probability,
                "random_target_start_s": self.random_target_start_s,
                "random_target_end_s": self.random_target_end_s,
                "random_target_drone_id": self.random_target_drone_id,
                "random_target_offsets_s": self.random_target_offsets_s,
            },
            "events": self.events[-100:],
        }
        try:
            with open(self.meta_path, "w", encoding="ascii") as f:
                json.dump(meta, f, indent=2, ensure_ascii=True, sort_keys=True)
        except Exception:
            pass

    def _on_shutdown(self) -> None:
        with self.lock:
            self._write_meta(final=True)


if __name__ == "__main__":
    rospy.init_node("rule_pool_latency_proxy", anonymous=False)
    RulePoolLatencyProxy()
    rospy.spin()
