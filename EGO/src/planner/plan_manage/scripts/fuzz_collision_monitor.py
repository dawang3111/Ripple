#!/usr/bin/env python3
import json
import math
import os
import subprocess
import threading
from collections import deque
from datetime import datetime

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Empty, String


class FuzzCollisionMonitor:
    def __init__(self):
        self.drone_num = int(rospy.get_param("~drone_num", 10))
        self.odom_suffix = rospy.get_param("~odom_suffix", "visual_slam/odom").strip("/")
        self.obstacle_topic = rospy.get_param("~obstacle_topic", "/map_generator/global_cloud")
        self.enable_obstacle_collision = bool(rospy.get_param("~enable_obstacle_collision", True))
        self.enable_drone_collision = bool(rospy.get_param("~enable_drone_collision", True))
        self.drone_radius = float(rospy.get_param("~drone_radius", 0.1))
        self.drone_margin = float(rospy.get_param("~drone_collision_margin", 0.02))
        self.obstacle_margin = float(rospy.get_param("~obstacle_collision_margin", 0.02))
        self.obstacle_voxel = float(rospy.get_param("~obstacle_voxel", 0.1))
        self.check_hz = float(rospy.get_param("~check_hz", 30.0))
        self.window_sec = float(rospy.get_param("~window_sec", 5.0))
        self.report_dir = rospy.get_param("~report_dir", os.path.expanduser("~/.ego_fuzz_reports"))
        self.run_id = rospy.get_param("~run_id", "").strip()
        self.kill_all_nodes = bool(rospy.get_param("~kill_all_nodes", False))
        self.mandatory_stop_topic = rospy.get_param("~mandatory_stop_topic", "/mandatory_stop_to_planner")
        self.event_topic = rospy.get_param("~event_topic", "/fuzz_collision_event")

        if not self.run_id:
            self.run_id = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.start_wall_time = datetime.now().isoformat()
        self.start_ros_time = rospy.Time.now().to_sec()

        self.odom_state = {}
        self.trace_window = deque()
        self.full_trace = []
        self.occupancy = set()
        self.collision_triggered = False
        self.collision_event_seq = 0
        # Split locks: _state_lock protects fast-path state read by check_callback
        # (odom_state, occupancy, collision_triggered). _trace_lock protects the
        # trace buffers (trace_window, full_trace) that are appended every odom
        # tick. Lock order when both are needed: _state_lock first, then
        # _trace_lock. No path may acquire _state_lock while holding _trace_lock.
        self._state_lock = threading.Lock()
        self._trace_lock = threading.Lock()

        self.stop_pub = rospy.Publisher(self.mandatory_stop_topic, Empty, queue_size=10)
        self.event_pub = rospy.Publisher(self.event_topic, String, queue_size=10)

        self.odom_subs = []
        for i in range(self.drone_num):
            topic = f"/drone_{i}_{self.odom_suffix}"
            sub = rospy.Subscriber(topic, Odometry, self.odom_callback, callback_args=i, queue_size=20)
            self.odom_subs.append(sub)

        self.obstacle_sub = None
        if self.enable_obstacle_collision:
            self.obstacle_sub = rospy.Subscriber(self.obstacle_topic, PointCloud2, self.obstacle_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.check_hz, 1.0)), self.check_callback)
        rospy.loginfo("[fuzz_collision_monitor] started, run_id=%s", self.run_id)

    def odom_callback(self, msg, drone_id):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        now = rospy.Time.now().to_sec()

        with self._state_lock:
            self.odom_state[drone_id] = {
                "t": stamp,
                "p": [pos.x, pos.y, pos.z],
                "v": [vel.x, vel.y, vel.z],
            }

        with self._trace_lock:
            self.trace_window.append(
                {
                    "t": now,
                    "drone_id": drone_id,
                    "p": [pos.x, pos.y, pos.z],
                    "v": [vel.x, vel.y, vel.z],
                }
            )
            self.full_trace.append(
                {
                    "t": now,
                    "drone_id": drone_id,
                    "p": [pos.x, pos.y, pos.z],
                    "v": [vel.x, vel.y, vel.z],
                }
            )

            min_t = now - self.window_sec
            while self.trace_window and self.trace_window[0]["t"] < min_t:
                self.trace_window.popleft()

    def obstacle_callback(self, msg):
        local_set = set()
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            key = self.quantize(p[0], p[1], p[2])
            local_set.add(key)
        with self._state_lock:
            self.occupancy = local_set

    def quantize(self, x, y, z):
        r = self.obstacle_voxel
        return (int(round(x / r)), int(round(y / r)), int(round(z / r)))

    def dequantize(self, key):
        r = self.obstacle_voxel
        return [key[0] * r, key[1] * r, key[2] * r]

    def check_callback(self, _):
        with self._state_lock:
            if self.collision_triggered:
                return

            if len(self.odom_state) < self.drone_num:
                return

            collision = None
            if self.enable_drone_collision:
                collision = self.check_drone_drone_collision()

            if collision is None and self.enable_obstacle_collision:
                collision = self.check_drone_obstacle_collision()

            if collision is not None:
                self.collision_triggered = True

        if collision is not None:
            self.handle_collision(collision)

    def check_drone_drone_collision(self):
        keys = sorted(self.odom_state.keys())
        threshold = 2.0 * self.drone_radius + self.drone_margin
        threshold_sq = threshold * threshold

        for i in range(len(keys)):
            for j in range(i + 1, len(keys)):
                a = self.odom_state[keys[i]]["p"]
                b = self.odom_state[keys[j]]["p"]
                dx = a[0] - b[0]
                dy = a[1] - b[1]
                dz = a[2] - b[2]
                dist_sq = dx * dx + dy * dy + dz * dz
                if dist_sq <= threshold_sq:
                    dist = math.sqrt(max(dist_sq, 0.0))
                    return {
                        "type": "drone_drone",
                        "drone_a": keys[i],
                        "drone_b": keys[j],
                        "distance": dist,
                        "threshold": threshold,
                        "position_a": a,
                        "position_b": b,
                    }
        return None

    def check_drone_obstacle_collision(self):
        if not self.occupancy:
            return None

        threshold = self.drone_radius + self.obstacle_margin
        threshold_sq = threshold * threshold
        r = int(math.ceil(threshold / self.obstacle_voxel))

        for drone_id, state in self.odom_state.items():
            p = state["p"]
            c = self.quantize(p[0], p[1], p[2])
            for ix in range(c[0] - r, c[0] + r + 1):
                for iy in range(c[1] - r, c[1] + r + 1):
                    for iz in range(c[2] - r, c[2] + r + 1):
                        key = (ix, iy, iz)
                        if key not in self.occupancy:
                            continue
                        o = self.dequantize(key)
                        dx = p[0] - o[0]
                        dy = p[1] - o[1]
                        dz = p[2] - o[2]
                        dist_sq = dx * dx + dy * dy + dz * dz
                        if dist_sq <= threshold_sq:
                            dist = math.sqrt(max(dist_sq, 0.0))
                            return {
                                "type": "drone_obstacle",
                                "drone_id": drone_id,
                                "distance": dist,
                                "threshold": threshold,
                                "drone_position": p,
                                "obstacle_position": o,
                            }
        return None

    def handle_collision(self, collision):
        self.collision_event_seq += 1
        event = {
            "event": "HARD_COLLISION",
            "collision_event_id": self.collision_event_seq,
            "run_id": self.run_id,
            "ros_time": rospy.Time.now().to_sec(),
            "wall_time": datetime.now().isoformat(),
            "collision": collision,
        }

        event_msg = String()
        event_msg.data = json.dumps(event, ensure_ascii=True)

        for _ in range(3):
            self.event_pub.publish(event_msg)
            self.stop_pub.publish(Empty())
            rospy.sleep(0.05)

        self.write_report(event)

        rospy.logerr("[fuzz_collision_monitor] HARD_COLLISION detected: %s", event_msg.data)

        if self.kill_all_nodes:
            threading.Thread(target=self.kill_nodes_async, daemon=True).start()

    def write_report(self, event):
        os.makedirs(self.report_dir, exist_ok=True)
        run_dir = os.path.join(self.report_dir, self.run_id)
        os.makedirs(run_dir, exist_ok=True)

        meta = {
            "run_id": self.run_id,
            "start_wall_time": self.start_wall_time,
            "start_ros_time": self.start_ros_time,
            "drone_num": self.drone_num,
            "odom_suffix": self.odom_suffix,
            "obstacle_topic": self.obstacle_topic,
            "drone_radius": self.drone_radius,
            "drone_collision_margin": self.drone_margin,
            "obstacle_collision_margin": self.obstacle_margin,
            "obstacle_voxel": self.obstacle_voxel,
            "check_hz": self.check_hz,
            "window_sec": self.window_sec,
            "kill_all_nodes": self.kill_all_nodes,
            "git_commit": self.get_git_commit(),
            "git_dirty": self.get_git_dirty(),
        }

        # Snapshot in fixed order: _state_lock first, then _trace_lock. The two
        # snapshots are taken under different locks to avoid blocking the odom
        # callback on the full_trace copy.
        with self._state_lock:
            odom_snapshot = self.odom_state.copy()
            obstacle_count = len(self.occupancy)

        with self._trace_lock:
            trace_snapshot = list(self.full_trace)
            trace_window_snapshot = list(self.trace_window)

        event_t = float(event.get("ros_time", 0.0))
        nearest_idx = -1
        nearest_dt = float("inf")
        t_min = None
        t_max = None
        for i, item in enumerate(trace_snapshot):
            t = float(item.get("t", 0.0))
            if t_min is None or t < t_min:
                t_min = t
            if t_max is None or t > t_max:
                t_max = t
            dt = abs(t - event_t)
            if dt < nearest_dt:
                nearest_dt = dt
                nearest_idx = i

        summary = {
            "meta": meta,
            "event": event,
            "latest_odom": odom_snapshot,
            "obstacle_voxel_count": obstacle_count,
            "trace_capture_mode": "full_on_collision",
            "trace_window_index": {
                "nearest_index": nearest_idx,
                "nearest_dt_sec": None if nearest_idx < 0 else nearest_dt,
                "trace_size": len(trace_snapshot),
                "trace_t_min": t_min,
                "trace_t_max": t_max,
            },
            "trace_window_5s_size": len(trace_window_snapshot),
        }

        with open(os.path.join(run_dir, "summary.json"), "w", encoding="ascii") as f:
            json.dump(summary, f, indent=2, ensure_ascii=True)

        with open(os.path.join(run_dir, "trace_window.jsonl"), "w", encoding="ascii") as f:
            for item in trace_snapshot:
                f.write(json.dumps(item, ensure_ascii=True) + "\n")

        with open(os.path.join(run_dir, "trace_window_5s.jsonl"), "w", encoding="ascii") as f:
            for item in trace_window_snapshot:
                f.write(json.dumps(item, ensure_ascii=True) + "\n")

    def get_git_commit(self):
        try:
            out = subprocess.check_output(["git", "rev-parse", "HEAD"], stderr=subprocess.DEVNULL)
            return out.decode("ascii", errors="ignore").strip()
        except Exception:
            return "unknown"

    def get_git_dirty(self):
        try:
            out = subprocess.check_output(["git", "status", "--porcelain"], stderr=subprocess.DEVNULL)
            return bool(out.decode("ascii", errors="ignore").strip())
        except Exception:
            return None

    def kill_nodes_async(self):
        rospy.sleep(0.3)
        try:
            subprocess.call(["rosnode", "kill", "-a"])
        except Exception as ex:
            rospy.logerr("[fuzz_collision_monitor] failed to kill nodes: %s", str(ex))
        finally:
            rospy.signal_shutdown("hard collision detected")


if __name__ == "__main__":
    rospy.init_node("fuzz_collision_monitor", anonymous=False)
    FuzzCollisionMonitor()
    rospy.spin()
