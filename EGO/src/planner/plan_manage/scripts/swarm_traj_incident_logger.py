#!/usr/bin/env python3
"""
Ring-buffer MINCO (broadcast) + PolyTraj (local) for all drones; on HARD_COLLISION or
/fuzz_emergency_stop_event, dump [t_trigger - window, t_trigger + window] clipped to buffer
and to current time when the dump runs. Each incident -> one jsonl file + manifest update.
"""
from __future__ import annotations

import json
import os
import threading
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Set

import rospy
from std_msgs.msg import String
from traj_utils.msg import MINCOTraj, PolyTraj


def _poly_to_dict(msg: PolyTraj) -> Dict[str, Any]:
    return {
        "drone_id": int(msg.drone_id),
        "traj_id": int(msg.traj_id),
        "start_time": float(msg.start_time.to_sec()),
        "order": int(msg.order),
        "coef_x": [float(x) for x in msg.coef_x],
        "coef_y": [float(x) for x in msg.coef_y],
        "coef_z": [float(x) for x in msg.coef_z],
        "duration": [float(x) for x in msg.duration],
    }


def _minco_to_dict(msg: MINCOTraj) -> Dict[str, Any]:
    return {
        "drone_id": int(msg.drone_id),
        "traj_id": int(msg.traj_id),
        "start_time": float(msg.start_time.to_sec()),
        "des_clearance": float(msg.des_clearance),
        "order": int(msg.order),
        "start_p": [float(x) for x in msg.start_p],
        "start_v": [float(x) for x in msg.start_v],
        "start_a": [float(x) for x in msg.start_a],
        "end_p": [float(x) for x in msg.end_p],
        "end_v": [float(x) for x in msg.end_v],
        "end_a": [float(x) for x in msg.end_a],
        "inner_x": [float(x) for x in msg.inner_x],
        "inner_y": [float(x) for x in msg.inner_y],
        "inner_z": [float(x) for x in msg.inner_z],
        "duration": [float(x) for x in msg.duration],
    }


class SwarmTrajIncidentLogger:
    def __init__(self) -> None:
        self.report_dir = os.path.expanduser(str(rospy.get_param("~report_dir", "~/.ego_fuzz_reports")).strip())
        self.run_id = str(rospy.get_param("~run_id", "")).strip()
        self.drone_num = int(rospy.get_param("~drone_num", 10))
        self.window_sec = float(rospy.get_param("~window_sec", 10.0))
        self.buffer_sec = float(rospy.get_param("~buffer_sec", 35.0))
        self.broadcast_topic = str(rospy.get_param("~broadcast_topic", "/broadcast_traj_from_planner")).strip()
        self.collision_topic = str(rospy.get_param("~collision_topic", "/fuzz_collision_event")).strip()
        self.emergency_topic = str(rospy.get_param("~emergency_topic", "/fuzz_emergency_stop_event")).strip()

        if not self.run_id:
            self.run_id = "run_unknown"

        self.run_path = os.path.join(self.report_dir, self.run_id)
        os.makedirs(self.run_path, exist_ok=True)

        self._buf: Deque[Dict[str, Any]] = deque()
        self._lock = threading.Lock()
        self._seq = 0
        self._seen_collision_ids: Set[int] = set()
        self._manifest_path = os.path.join(self.run_path, "swarm_traj_incident_manifest.json")

        self._subs = []
        self._subs.append(rospy.Subscriber(self.broadcast_topic, MINCOTraj, self._on_broadcast, queue_size=500))
        self._subs.append(rospy.Subscriber(self.collision_topic, String, self._on_collision, queue_size=20))
        self._subs.append(rospy.Subscriber(self.emergency_topic, String, self._on_emergency, queue_size=100))

        for i in range(self.drone_num):
            topic = "/drone_{}_planning/trajectory".format(i)
            self._subs.append(
                rospy.Subscriber(topic, PolyTraj, self._make_local_cb(i), queue_size=200)
            )

        rospy.loginfo(
            "[swarm_traj_incident_logger] run_id=%s drones=%d window=%.1fs buffer=%.1fs",
            self.run_id,
            self.drone_num,
            self.window_sec,
            self.buffer_sec,
        )

    def _make_local_cb(self, drone_id: int):
        def _cb(msg: PolyTraj) -> None:
            self._push("local", drone_id, _poly_to_dict(msg))

        return _cb

    def _on_broadcast(self, msg: MINCOTraj) -> None:
        self._push("broadcast", int(msg.drone_id), _minco_to_dict(msg))

    def _push(self, stream: str, drone_id: int, payload: Dict[str, Any]) -> None:
        recv_t = rospy.Time.now().to_sec()
        with self._lock:
            self._buf.append({"recv_ros_time": recv_t, "stream": stream, "drone_id": drone_id, "msg": payload})
            cut = recv_t - self.buffer_sec
            while self._buf and self._buf[0]["recv_ros_time"] < cut:
                self._buf.popleft()

    def _on_collision(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            rospy.logwarn("[swarm_traj_incident_logger] bad collision json")
            return
        if data.get("event") != "HARD_COLLISION":
            return
        eid = int(data.get("collision_event_id", -1))
        if eid >= 0 and eid in self._seen_collision_ids:
            return
        if eid >= 0:
            self._seen_collision_ids.add(eid)
        t_trigger = float(data.get("ros_time", rospy.Time.now().to_sec()))
        self._schedule_incident("hard_collision", t_trigger, {"collision": data.get("collision")})

    def _on_emergency(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if data.get("type") != "emergency_stop":
            return
        t_trigger = float(data.get("ros_time", rospy.Time.now().to_sec()))
        self._schedule_incident("emergency_stop", t_trigger, {"emergency": data})

    def _schedule_incident(self, kind: str, t_trigger: float, extra: Dict[str, Any]) -> None:
        self._seq += 1
        seq = self._seq
        rospy.logwarn(
            "[swarm_traj_incident_logger] incident seq=%d kind=%s t_trigger=%.3f (immediate window clip to now)",
            seq,
            kind,
            t_trigger,
        )
        self._write_dump(seq, kind, t_trigger, extra)

    def _buffer_snapshot(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._buf)

    def _write_dump(self, seq: int, kind: str, t_trigger: float, extra: Dict[str, Any]) -> None:
        t0_req = t_trigger - self.window_sec
        t1_req = t_trigger + self.window_sec
        now = rospy.Time.now().to_sec()
        buf = self._buffer_snapshot()
        if not buf:
            t0_eff = t0_req
            t1_eff = min(t1_req, now)
        else:
            t_min_buf = buf[0]["recv_ros_time"]
            t_max_buf = buf[-1]["recv_ros_time"]
            t0_eff = max(t0_req, t_min_buf)
            t1_eff = min(t1_req, now, t_max_buf)

        if t1_eff < t0_eff:
            t0_eff, t1_eff = t1_eff, t0_eff

        lines: List[str] = []
        bc = 0
        lc = 0
        for row in buf:
            rt = row["recv_ros_time"]
            if rt < t0_eff or rt > t1_eff:
                continue
            rec = {
                "stream": row["stream"],
                "recv_ros_time": rt,
                "drone_id": row["drone_id"],
                "msg": row["msg"],
            }
            lines.append(json.dumps(rec, ensure_ascii=True))
            if row["stream"] == "broadcast":
                bc += 1
            else:
                lc += 1

        fname = "swarm_traj_window_{:03d}.jsonl".format(seq)
        out_path = os.path.join(self.run_path, fname)
        with open(out_path, "w", encoding="ascii") as f:
            for ln in lines:
                f.write(ln + "\n")

        manifest = self._read_manifest()
        manifest.setdefault("run_id", self.run_id)
        manifest.setdefault("window_sec_requested", self.window_sec)
        manifest.setdefault("incidents", [])
        manifest["incidents"].append(
            {
                "seq": seq,
                "kind": kind,
                "t_trigger": t_trigger,
                "t0_requested": t0_req,
                "t1_requested": t1_req,
                "t0_effective": t0_eff,
                "t1_effective": t1_eff,
                "dump_file": fname,
                "line_counts": {"broadcast": bc, "local": lc},
                "extra": extra,
            }
        )
        with open(self._manifest_path, "w", encoding="ascii") as f:
            json.dump(manifest, f, ensure_ascii=True, indent=2)

        rospy.logwarn("[swarm_traj_incident_logger] wrote %s (%d lines)", fname, len(lines))

    def _read_manifest(self) -> Dict[str, Any]:
        if not os.path.isfile(self._manifest_path):
            return {}
        try:
            with open(self._manifest_path, "r", encoding="ascii") as f:
                return json.load(f)
        except Exception:
            return {}


def main() -> None:
    rospy.init_node("swarm_traj_incident_logger", anonymous=False)
    SwarmTrajIncidentLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
