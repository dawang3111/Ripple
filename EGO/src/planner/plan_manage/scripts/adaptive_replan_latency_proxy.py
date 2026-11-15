#!/usr/bin/env python3
from __future__ import annotations

import copy
import csv
import json
import math
import os
import random
import threading
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import rospy
from std_msgs.msg import String
from traj_utils.msg import MINCOTraj, PolyTraj


TASK_ACTIVE_FSM_STATES = set(["SEQUENTIAL_START", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"])
REPLAN_TRIGGER_STATES = set(["GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ"])


def _now_iso() -> str:
    return datetime.now().isoformat()


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except Exception:
        return default
    if math.isnan(out) or math.isinf(out):
        return default
    return out


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _dist(a: List[float], b: List[float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


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


class AdaptiveReplanLatencyProxy:
    def __init__(self) -> None:
        self.enabled = bool(rospy.get_param("~enable", False))
        self.config_file = str(rospy.get_param("~config_file", "")).strip()
        self.in_topic = str(rospy.get_param("~in_topic", "/broadcast_traj_from_planner")).strip()
        self.out_topic = str(rospy.get_param("~out_topic", "/broadcast_traj_from_planner_attacked")).strip()
        self.report_dir = os.path.expanduser(str(rospy.get_param("~report_dir", "~/.ego_fuzz_reports")).strip())
        self.run_id = str(rospy.get_param("~run_id", "")).strip() or datetime.now().strftime("%Y%m%d_%H%M%S")
        self.drone_num = int(rospy.get_param("~drone_num", 10))
        self.drone_radius = float(rospy.get_param("~drone_radius", 0.1))

        self.max_injections = int(rospy.get_param("~max_injections_per_round", 5))
        self.cooldown_s = float(rospy.get_param("~cooldown_s", 2.0))
        self.trigger_window_s = float(rospy.get_param("~trigger_window_s", 1.0))
        self.future_horizon_s = float(rospy.get_param("~future_horizon_s", 3.0))
        self.future_sample_dt = float(rospy.get_param("~future_sample_dt", 0.1))
        self.interaction_margin_threshold = float(rospy.get_param("~interaction_margin_threshold", 0.25))
        self.interaction_score_threshold = float(rospy.get_param("~interaction_score_threshold", 0.6))
        self.latency_factor = float(rospy.get_param("~latency_factor", 1.2))
        self.latency_min_ms = float(rospy.get_param("~latency_min_ms", 80.0))
        self.latency_max_ms = float(rospy.get_param("~latency_max_ms", 250.0))
        self.period_fallback_ms = float(rospy.get_param("~period_fallback_ms", 120.0))
        self.seed = int(rospy.get_param("~seed", 20260917))

        self.rng = random.Random(self.seed)
        self.start_ros_time = rospy.Time.now().to_sec()
        self.start_wall_time = _now_iso()
        self.lock = threading.Lock()

        self.local_trajs: Dict[int, PolyTrajectory] = {}
        self.last_local_traj_id: Dict[int, int] = {}
        self.last_local_traj_time: Dict[int, float] = {}
        self.last_broadcast_time: Dict[int, float] = {}
        self.period_ema_ms: Dict[int, float] = {}
        self.fsm_state: Dict[int, Dict[str, Any]] = {}
        self.triggers: Dict[int, Dict[str, Any]] = {}
        self.last_injection_time = -1e9
        self.injection_count = 0
        self.msg_seen = 0
        self.msg_delayed = 0
        self.events: List[Dict[str, Any]] = []

        self.run_path = os.path.join(self.report_dir, self.run_id)
        os.makedirs(self.run_path, exist_ok=True)
        self.meta_path = os.path.join(self.run_path, "adaptive_replan_latency_meta.json")
        self.events_csv_path = os.path.join(self.run_path, "adaptive_replan_latency_events.csv")
        self.trace_jsonl_path = os.path.join(self.run_path, "adaptive_replan_latency_trace.jsonl")
        self._ensure_events_csv()

        self.pub = rospy.Publisher(self.out_topic, MINCOTraj, queue_size=200)
        self.sub = rospy.Subscriber(self.in_topic, MINCOTraj, self._broadcast_cb, queue_size=200)
        self.subs = [self.sub]
        for drone_id in range(self.drone_num):
            self.subs.append(rospy.Subscriber("/drone_{}_planning/trajectory".format(drone_id), PolyTraj, self._local_traj_cb, callback_args=drone_id, queue_size=50))
            self.subs.append(rospy.Subscriber("/drone_{}_planning/fsm_state".format(drone_id), String, self._fsm_cb, callback_args=drone_id, queue_size=20))

        rospy.on_shutdown(self._on_shutdown)
        self._write_meta(final=False)
        rospy.loginfo(
            "[adaptive_replan_latency_proxy] enabled=%s max_injections=%d cooldown=%.2fs in=%s out=%s",
            str(self.enabled).lower(),
            self.max_injections,
            self.cooldown_s,
            self.in_topic,
            self.out_topic,
        )

    def _local_traj_cb(self, msg: PolyTraj, drone_id: int) -> None:
        traj = PolyTrajectory(msg)
        if not traj.valid:
            return
        now = rospy.Time.now().to_sec()
        with self.lock:
            prev_id = self.last_local_traj_id.get(drone_id)
            prev_time = self.last_local_traj_time.get(drone_id)
            self.local_trajs[drone_id] = traj
            self.last_local_traj_id[drone_id] = traj.traj_id
            self.last_local_traj_time[drone_id] = now
            if prev_time is not None and now > prev_time:
                self._observe_period_locked(drone_id, (now - prev_time) * 1000.0)
            if prev_id is not None and prev_id != traj.traj_id:
                self._record_trigger_locked(drone_id, now, "trajectory_change", traj.traj_id, prev_id)

    def _fsm_cb(self, msg: String, drone_id: int) -> None:
        state = str(msg.data or "NA")
        now = rospy.Time.now().to_sec()
        with self.lock:
            prev = str(self.fsm_state.get(drone_id, {}).get("state", "NA"))
            self.fsm_state[drone_id] = {"state": state, "t": now}
            if prev != state and state in REPLAN_TRIGGER_STATES:
                self._record_trigger_locked(drone_id, now, "fsm_change:{}->{}".format(prev, state), self.last_local_traj_id.get(drone_id, -1), self.last_local_traj_id.get(drone_id, -1))

    def _broadcast_cb(self, msg: MINCOTraj) -> None:
        now = rospy.Time.now().to_sec()
        source_id = int(msg.drone_id)
        with self.lock:
            self.msg_seen += 1
            prev_broadcast = self.last_broadcast_time.get(source_id)
            self.last_broadcast_time[source_id] = now
            if prev_broadcast is not None and now > prev_broadcast:
                self._observe_period_locked(source_id, (now - prev_broadcast) * 1000.0)
            decision = self._choose_delay_locked(source_id, now, msg)

        if decision is None:
            self.pub.publish(msg)
            return

        delay_s = float(decision["latency_ms"]) / 1000.0
        with self.lock:
            self.injection_count += 1
            self.msg_delayed += 1
            self.last_injection_time = now
            decision["injection_index"] = self.injection_count
            self.events.append(dict(decision))
            self._append_event_locked(decision)
            self._write_meta(final=False)
        self._append_trace("adaptive_delay_tx", decision)

        timer = threading.Timer(delay_s, self._publish_msg, args=[copy.deepcopy(msg)])
        timer.daemon = True
        timer.start()

    def _record_trigger_locked(self, drone_id: int, now: float, reason: str, traj_id: int, prev_traj_id: int) -> None:
        state = str(self.fsm_state.get(drone_id, {}).get("state", "NA"))
        if state not in TASK_ACTIVE_FSM_STATES and "trajectory" not in reason:
            return
        self.triggers[drone_id] = {
            "time": now,
            "reason": reason,
            "fsm_state": state,
            "traj_id": traj_id,
            "prev_traj_id": prev_traj_id,
        }

    def _observe_period_locked(self, drone_id: int, period_ms: float) -> None:
        if period_ms <= 1.0 or period_ms > 5000.0:
            return
        old = self.period_ema_ms.get(drone_id)
        self.period_ema_ms[drone_id] = period_ms if old is None else 0.75 * old + 0.25 * period_ms

    def _choose_delay_locked(self, source_id: int, now: float, msg: MINCOTraj) -> Optional[Dict[str, Any]]:
        if not self.enabled:
            return None
        if self.injection_count >= self.max_injections:
            return None
        if now - self.last_injection_time < self.cooldown_s:
            return None
        if source_id < 0 or source_id >= self.drone_num:
            return None
        source_traj = self.local_trajs.get(source_id)
        if source_traj is None:
            return None

        best: Optional[Dict[str, Any]] = None
        for victim_id, trigger in list(self.triggers.items()):
            if victim_id == source_id:
                continue
            age = now - float(trigger.get("time", 0.0))
            if age < 0.0 or age > self.trigger_window_s:
                continue
            victim_traj = self.local_trajs.get(victim_id)
            if victim_traj is None:
                continue
            interaction = self._interaction_score(source_id, victim_id, source_traj, victim_traj, now)
            if interaction is None:
                continue
            score, min_margin, min_distance = interaction
            score += max(0.0, (self.trigger_window_s - age) / max(self.trigger_window_s, 1e-6)) * 0.25
            if score < self.interaction_score_threshold:
                continue
            candidate = {
                "wall_time": _now_iso(),
                "ros_time": now,
                "inject_time_s": now - self.start_ros_time,
                "attacker_drone_id": source_id,
                "victim_drone_id": victim_id,
                "source_traj_id": int(msg.traj_id),
                "victim_traj_id": int(trigger.get("traj_id", -1)),
                "victim_prev_traj_id": int(trigger.get("prev_traj_id", -1)),
                "victim_fsm_state": str(trigger.get("fsm_state", "NA")),
                "trigger_reason": str(trigger.get("reason", "unknown")),
                "trigger_age_s": age,
                "interaction_score": score,
                "min_future_margin": min_margin,
                "min_future_distance": min_distance,
                "estimated_period_ms": self.period_ema_ms.get(source_id, self.period_fallback_ms),
                "latency_ms": self._latency_for_source(source_id),
            }
            if best is None or float(candidate["interaction_score"]) > float(best["interaction_score"]):
                best = candidate
        return best

    def _interaction_score(
        self,
        attacker_id: int,
        victim_id: int,
        attacker_traj: PolyTrajectory,
        victim_traj: PolyTrajectory,
        now: float,
    ) -> Optional[Tuple[float, float, float]]:
        threshold = 2.0 * self.drone_radius
        steps = max(1, int(math.ceil(max(self.future_horizon_s, 0.0) / max(self.future_sample_dt, 0.05))))
        min_distance: Optional[float] = None
        overlap_count = 0
        last_dist = None
        closing_bonus = 0.0
        for idx in range(steps + 1):
            t_abs = now + idx * self.future_sample_dt
            a = attacker_traj.pos_at_abs_time(t_abs)
            b = victim_traj.pos_at_abs_time(t_abs)
            if a is None or b is None:
                continue
            d = _dist(a, b)
            if min_distance is None or d < min_distance:
                min_distance = d
            margin = d - threshold
            if margin <= self.interaction_margin_threshold:
                overlap_count += 1
            if last_dist is not None and d < last_dist:
                closing_bonus += 0.03
            last_dist = d
        if min_distance is None:
            return None
        min_margin = min_distance - threshold
        risk = max(0.0, (self.interaction_margin_threshold - min_margin) / max(self.interaction_margin_threshold, 1e-6))
        persistence = overlap_count / float(steps + 1)
        score = risk + 0.5 * persistence + min(closing_bonus, 0.3)
        return score, min_margin, min_distance

    def _latency_for_source(self, source_id: int) -> float:
        period = self.period_ema_ms.get(source_id, self.period_fallback_ms)
        jitter = self.rng.uniform(-0.05, 0.05) * period
        latency = period * self.latency_factor + jitter
        return max(self.latency_min_ms, min(self.latency_max_ms, latency))

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
                    "attacker_drone_id",
                    "victim_drone_id",
                    "latency_ms",
                    "trigger_reason",
                    "victim_fsm_state",
                    "source_traj_id",
                    "victim_traj_id",
                    "estimated_period_ms",
                    "interaction_score",
                    "min_future_margin",
                    "min_future_distance",
                ]
            )

    def _append_event_locked(self, event: Dict[str, Any]) -> None:
        try:
            with open(self.events_csv_path, "a", encoding="ascii", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        event.get("injection_index", "NA"),
                        "{:.6f}".format(float(event.get("inject_time_s", 0.0))),
                        event.get("attacker_drone_id", "NA"),
                        event.get("victim_drone_id", "NA"),
                        "{:.6f}".format(float(event.get("latency_ms", 0.0))),
                        event.get("trigger_reason", "NA"),
                        event.get("victim_fsm_state", "NA"),
                        event.get("source_traj_id", "NA"),
                        event.get("victim_traj_id", "NA"),
                        "{:.6f}".format(float(event.get("estimated_period_ms", 0.0))),
                        "{:.6f}".format(float(event.get("interaction_score", 0.0))),
                        "{:.6f}".format(float(event.get("min_future_margin", 0.0))),
                        "{:.6f}".format(float(event.get("min_future_distance", 0.0))),
                    ]
                )
        except Exception as exc:
            rospy.logwarn("[adaptive_replan_latency_proxy] failed to append event csv: %s", str(exc))

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
            "config_file": self.config_file,
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
            "events_csv": self.events_csv_path,
            "trace_jsonl": self.trace_jsonl_path,
            "params": {
                "max_injections_per_round": self.max_injections,
                "cooldown_s": self.cooldown_s,
                "trigger_window_s": self.trigger_window_s,
                "future_horizon_s": self.future_horizon_s,
                "future_sample_dt": self.future_sample_dt,
                "interaction_margin_threshold": self.interaction_margin_threshold,
                "interaction_score_threshold": self.interaction_score_threshold,
                "latency_factor": self.latency_factor,
                "latency_min_ms": self.latency_min_ms,
                "latency_max_ms": self.latency_max_ms,
                "period_fallback_ms": self.period_fallback_ms,
                "seed": self.seed,
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
    rospy.init_node("adaptive_replan_latency_proxy", anonymous=False)
    AdaptiveReplanLatencyProxy()
    rospy.spin()
