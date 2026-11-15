#!/usr/bin/env python3
from __future__ import annotations

import copy
import json
import os
import threading
from datetime import datetime
from typing import Any, Dict, List, Optional

import rospy
from traj_utils.msg import MINCOTraj


def _now_iso() -> str:
    return datetime.now().isoformat()


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


class AttackPlanLatencyProxy:
    def __init__(self) -> None:
        self.enabled = bool(rospy.get_param("~enable", False))
        self.plan_file = str(rospy.get_param("~plan_file", "")).strip()
        self.in_topic = str(rospy.get_param("~in_topic", "/broadcast_traj_from_planner")).strip()
        self.out_topic = str(rospy.get_param("~out_topic", "/broadcast_traj_from_planner_attacked")).strip()
        self.report_dir = os.path.expanduser(str(rospy.get_param("~report_dir", "~/.ego_fuzz_reports")).strip())
        self.run_id = str(rospy.get_param("~run_id", "")).strip() or datetime.now().strftime("%Y%m%d_%H%M%S")

        self.start_ros_time = rospy.Time.now().to_sec()
        self.start_wall_time = _now_iso()
        self.events = self._load_events()
        self.msg_seen = 0
        self.msg_delayed = 0
        self.lock = threading.Lock()

        self.run_dir = os.path.join(self.report_dir, self.run_id)
        os.makedirs(self.run_dir, exist_ok=True)
        self.meta_path = os.path.join(self.run_dir, "attack_plan_latency_proxy_meta.json")
        self.trace_path = os.path.join(self.run_dir, "attack_plan_latency_proxy_trace.jsonl")

        self.pub = rospy.Publisher(self.out_topic, MINCOTraj, queue_size=200)
        self.sub = rospy.Subscriber(self.in_topic, MINCOTraj, self._callback, queue_size=200)
        rospy.on_shutdown(self._on_shutdown)
        self._write_meta(final=False)
        rospy.loginfo(
            "[attack_plan_latency_proxy] enabled=%s events=%d in=%s out=%s",
            str(self.enabled).lower(),
            len(self.events),
            self.in_topic,
            self.out_topic,
        )

    def _load_events(self) -> List[Dict[str, Any]]:
        if not self.enabled:
            return []
        if not self.plan_file:
            rospy.logwarn("[attack_plan_latency_proxy] enabled but plan_file is empty")
            return []
        try:
            with open(self.plan_file, "r", encoding="ascii", errors="ignore") as f:
                payload = json.load(f)
        except Exception as exc:
            rospy.logwarn("[attack_plan_latency_proxy] failed to load plan_file=%s: %s", self.plan_file, str(exc))
            return []

        events = []
        for raw in payload.get("events", []):
            direction = str(raw.get("direction", "tx")).strip().lower()
            if direction not in ("tx", "both"):
                continue
            start_s = _safe_float(raw.get("inject_time_s"), -1.0)
            duration_s = _safe_float(raw.get("duration_s"), 0.0)
            latency_ms = _safe_float(raw.get("latency_ms"), 0.0)
            if start_s < 0.0 or duration_s <= 0.0 or latency_ms <= 0.0:
                continue
            events.append(
                {
                    "inject_time_s": start_s,
                    "end_time_s": start_s + duration_s,
                    "duration_s": duration_s,
                    "latency_ms": latency_ms,
                    "direction": direction,
                    "injection_node_id": _safe_int(raw.get("injection_node_id"), -1),
                    "primary_eval_node_id": _safe_int(raw.get("primary_eval_node_id"), -1),
                    "expected_primary_eval_fsm_state": str(raw.get("expected_primary_eval_fsm_state", "NA")),
                }
            )
        return sorted(events, key=lambda item: item["inject_time_s"])

    def _callback(self, msg: MINCOTraj) -> None:
        with self.lock:
            self.msg_seen += 1

        event = self._active_event_for_msg(msg)
        if event is None:
            self.pub.publish(msg)
            return

        delay_s = max(float(event["latency_ms"]), 0.0) / 1000.0
        with self.lock:
            self.msg_delayed += 1
        self._append_trace("delay_tx", msg, event, delay_s)

        timer = threading.Timer(delay_s, self._publish_msg, args=[copy.deepcopy(msg)])
        timer.daemon = True
        timer.start()

    def _active_event_for_msg(self, msg: MINCOTraj) -> Optional[Dict[str, Any]]:
        if not self.enabled:
            return None
        now_offset = rospy.Time.now().to_sec() - self.start_ros_time
        matches = []
        for event in self.events:
            if int(msg.drone_id) != int(event["injection_node_id"]):
                continue
            if float(event["inject_time_s"]) <= now_offset <= float(event["end_time_s"]):
                matches.append(event)
        if not matches:
            return None
        return max(matches, key=lambda item: float(item["latency_ms"]))

    def _publish_msg(self, msg: MINCOTraj) -> None:
        self.pub.publish(msg)

    def _append_trace(self, event_name: str, msg: MINCOTraj, event: Dict[str, Any], delay_s: float) -> None:
        line = {
            "wall_time": _now_iso(),
            "ros_time": rospy.Time.now().to_sec(),
            "event": event_name,
            "drone_id": int(msg.drone_id),
            "traj_id": int(msg.traj_id),
            "latency_ms": delay_s * 1000.0,
            "inject_time_s": event["inject_time_s"],
            "duration_s": event["duration_s"],
            "primary_eval_node_id": event["primary_eval_node_id"],
            "expected_primary_eval_fsm_state": event["expected_primary_eval_fsm_state"],
        }
        try:
            with open(self.trace_path, "a", encoding="ascii") as f:
                f.write(json.dumps(line, ensure_ascii=True, sort_keys=True) + "\n")
        except Exception:
            pass

    def _write_meta(self, final: bool) -> None:
        meta = {
            "enabled": self.enabled,
            "plan_file": self.plan_file,
            "in_topic": self.in_topic,
            "out_topic": self.out_topic,
            "run_id": self.run_id,
            "start_wall_time": self.start_wall_time,
            "start_ros_time": self.start_ros_time,
            "event_count": len(self.events),
            "events": self.events,
            "msg_seen": self.msg_seen,
            "msg_delayed": self.msg_delayed,
            "final": final,
            "wall_time": _now_iso(),
        }
        try:
            with open(self.meta_path, "w", encoding="ascii") as f:
                json.dump(meta, f, indent=2, ensure_ascii=True, sort_keys=True)
        except Exception:
            pass

    def _on_shutdown(self) -> None:
        self._write_meta(final=True)


if __name__ == "__main__":
    rospy.init_node("attack_plan_latency_proxy", anonymous=False)
    AttackPlanLatencyProxy()
    rospy.spin()
