#!/usr/bin/env python3
import copy
import json
import os
import random
import threading
import time
from datetime import datetime

import rospy
from traj_utils.msg import MINCOTraj


class CommFuzzProxy:
    def __init__(self):
        self.enabled = bool(rospy.get_param("~enable", False))
        self.target_drone_id = int(rospy.get_param("~target_drone_id", 0))
        self.mode_cfg = str(rospy.get_param("~mode", "random")).strip().lower()
        self.seed_cfg = int(rospy.get_param("~seed", -1))

        self.start_min_s = float(rospy.get_param("~start_min_s", 5.0))
        self.start_max_s = float(rospy.get_param("~start_max_s", 25.0))
        self.duration_min_s = float(rospy.get_param("~duration_min_s", 5.0))
        self.duration_max_s = float(rospy.get_param("~duration_max_s", 20.0))

        self.delay_ms_min = float(rospy.get_param("~delay_ms_min", 80.0))
        self.delay_ms_max = float(rospy.get_param("~delay_ms_max", 250.0))

        self.jitter_ms_min = float(rospy.get_param("~jitter_ms_min", 30.0))
        self.jitter_ms_max = float(rospy.get_param("~jitter_ms_max", 120.0))

        self.noise_sigma_pos_min = float(rospy.get_param("~noise_sigma_pos_min", 0.05))
        self.noise_sigma_pos_max = float(rospy.get_param("~noise_sigma_pos_max", 0.20))

        self.in_topic = str(rospy.get_param("~in_topic", "/broadcast_traj_from_planner")).strip()
        self.out_topic = str(rospy.get_param("~out_topic", "/broadcast_traj_from_planner_fuzzed")).strip()

        self.report_dir = str(rospy.get_param("~report_dir", os.path.expanduser("~/.ego_fuzz_reports"))).strip()
        self.run_id = str(rospy.get_param("~run_id", "")).strip()
        if not self.run_id:
            self.run_id = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.start_wall_time = datetime.now().isoformat()
        self.start_ros_time = rospy.Time.now().to_sec()

        self.seed_used = self.seed_cfg if self.seed_cfg >= 0 else int(time.time() * 1000) & 0x7FFFFFFF
        self.rng = random.Random(self.seed_used)

        self.start_min_s, self.start_max_s = sorted((self.start_min_s, self.start_max_s))
        self.duration_min_s, self.duration_max_s = sorted((self.duration_min_s, self.duration_max_s))
        self.delay_ms_min, self.delay_ms_max = sorted((self.delay_ms_min, self.delay_ms_max))
        self.jitter_ms_min, self.jitter_ms_max = sorted((self.jitter_ms_min, self.jitter_ms_max))
        self.noise_sigma_pos_min, self.noise_sigma_pos_max = sorted((self.noise_sigma_pos_min, self.noise_sigma_pos_max))

        self.window_start_s = self.rng.uniform(self.start_min_s, self.start_max_s)
        self.window_duration_s = self.rng.uniform(self.duration_min_s, self.duration_max_s)
        self.window_end_s = self.window_start_s + self.window_duration_s

        self.mode_used = self.resolve_mode(self.mode_cfg)
        self.factor_name, self.factor_value = self.sample_factor(self.mode_used)

        self.msg_seen = 0
        self.msg_fuzzed = 0
        self.lock = threading.Lock()

        self.pub = rospy.Publisher(self.out_topic, MINCOTraj, queue_size=200)
        self.sub = rospy.Subscriber(self.in_topic, MINCOTraj, self.callback, queue_size=200)

        self.run_dir = os.path.join(self.report_dir, self.run_id)
        os.makedirs(self.run_dir, exist_ok=True)
        self.meta_path = os.path.join(self.run_dir, "comm_fuzz_meta.json")
        self.write_meta(final=False)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "[comm_fuzz_proxy] enable=%s mode=%s target=%d seed=%d window=[%.3f, %.3f]s factor=%s:%.6f",
            str(self.enabled).lower(),
            self.mode_used,
            self.target_drone_id,
            self.seed_used,
            self.window_start_s,
            self.window_end_s,
            self.factor_name,
            self.factor_value,
        )

    def resolve_mode(self, mode_cfg):
        if mode_cfg == "random":
            return self.rng.choice(["delay", "jitter", "noise"])
        if mode_cfg in ("delay", "jitter", "noise"):
            return mode_cfg
        return "delay"

    def sample_factor(self, mode):
        if mode == "delay":
            return "delay_ms", self.rng.uniform(self.delay_ms_min, self.delay_ms_max)
        if mode == "jitter":
            return "jitter_ms", self.rng.uniform(self.jitter_ms_min, self.jitter_ms_max)
        return "noise_sigma_pos", self.rng.uniform(self.noise_sigma_pos_min, self.noise_sigma_pos_max)

    def in_window(self):
        t = rospy.Time.now().to_sec() - self.start_ros_time
        return self.window_start_s <= t <= self.window_end_s

    def callback(self, msg):
        with self.lock:
            self.msg_seen += 1

        if not self.enabled or msg.drone_id != self.target_drone_id or not self.in_window():
            self.pub.publish(msg)
            return

        with self.lock:
            self.msg_fuzzed += 1

        if self.mode_used == "delay":
            delay_s = max(self.factor_value, 0.0) / 1000.0
            timer = threading.Timer(delay_s, self.publish_msg, args=[copy.deepcopy(msg)])
            timer.daemon = True
            timer.start()
            return

        if self.mode_used == "jitter":
            jitter_ms = max(self.factor_value, 0.0)
            delay_s = self.rng.uniform(0.0, jitter_ms / 1000.0)
            timer = threading.Timer(delay_s, self.publish_msg, args=[copy.deepcopy(msg)])
            timer.daemon = True
            timer.start()
            return

        self.pub.publish(self.apply_noise(msg))

    def publish_msg(self, msg):
        self.pub.publish(msg)

    def apply_noise(self, msg):
        noisy = copy.deepcopy(msg)
        sigma = max(self.factor_value, 0.0)

        noisy.start_p = [x + self.rng.gauss(0.0, sigma) for x in noisy.start_p]
        noisy.end_p = [x + self.rng.gauss(0.0, sigma) for x in noisy.end_p]
        noisy.inner_x = [x + self.rng.gauss(0.0, sigma) for x in noisy.inner_x]
        noisy.inner_y = [y + self.rng.gauss(0.0, sigma) for y in noisy.inner_y]
        noisy.inner_z = [z + self.rng.gauss(0.0, sigma) for z in noisy.inner_z]
        return noisy

    def write_meta(self, final):
        meta = {
            "run_id": self.run_id,
            "start_wall_time": self.start_wall_time,
            "start_ros_time": self.start_ros_time,
            "enabled": self.enabled,
            "seed": self.seed_used,
            "target_drone_id": self.target_drone_id,
            "mode": self.mode_used,
            "window_start_s": self.window_start_s,
            "window_duration_s": self.window_duration_s,
            "window_end_s": self.window_end_s,
            "factor_name": self.factor_name,
            "factor_value": self.factor_value,
            "in_topic": self.in_topic,
            "out_topic": self.out_topic,
            "msg_seen": self.msg_seen,
            "msg_fuzzed": self.msg_fuzzed,
            "final": final,
            "wall_time": datetime.now().isoformat(),
        }
        with open(self.meta_path, "w", encoding="ascii") as f:
            json.dump(meta, f, indent=2, ensure_ascii=True)

    def on_shutdown(self):
        self.write_meta(final=True)


if __name__ == "__main__":
    rospy.init_node("comm_fuzz_proxy", anonymous=False)
    CommFuzzProxy()
    rospy.spin()
