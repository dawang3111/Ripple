#!/usr/bin/env python3
import json
import os
import threading
from collections import deque
from datetime import datetime

import rospy
from std_msgs.msg import Empty, String
from traj_utils.msg import MINCOTraj


def now_iso():
    return datetime.now().isoformat()


class CommFullDumpSidecar:
    def __init__(self):
        self.enabled = bool(rospy.get_param("~enable", False))
        self.report_dir = str(rospy.get_param("~report_dir", os.path.expanduser("~/.ego_fuzz_reports"))).strip()
        self.run_id = str(rospy.get_param("~run_id", "")).strip()
        self.buffer_max = int(rospy.get_param("~buffer_max", 4000))
        self.dump_file_name = str(rospy.get_param("~dump_file_name", "comm_full_dump_on_incident.jsonl")).strip()
        self.meta_file_name = str(rospy.get_param("~meta_file_name", "comm_full_dump_meta.json")).strip()

        self.tx_topic = str(rospy.get_param("~tx_topic", "/broadcast_traj_from_planner")).strip()
        self.rx_topic = str(rospy.get_param("~rx_topic", "/broadcast_traj_to_planner")).strip()
        self.collision_topic = str(rospy.get_param("~collision_topic", "/fuzz_collision_event")).strip()
        self.stop_topic = str(rospy.get_param("~stop_topic", "/mandatory_stop_to_planner")).strip()

        if not self.run_id:
            self.run_id = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.start_wall_time = now_iso()
        self.start_ros_time = rospy.Time.now().to_sec()
        self.run_dir = os.path.join(self.report_dir, self.run_id)
        self.dump_path = os.path.join(self.run_dir, self.dump_file_name)
        self.meta_path = os.path.join(self.run_dir, self.meta_file_name)

        self.lock = threading.Lock()
        self.buffer = deque()
        self.triggered = False
        self.trigger_reason = ""
        self.trigger_ros_time = -1.0
        self.msg_seen = 0
        self.msg_written = 0
        self.msg_dropped = 0
        self.tx_seen = 0
        self.rx_seen = 0

        self.sub_tx = rospy.Subscriber(self.tx_topic, MINCOTraj, self.tx_cb, queue_size=500)
        self.sub_rx = rospy.Subscriber(self.rx_topic, MINCOTraj, self.rx_cb, queue_size=500)
        self.sub_collision = rospy.Subscriber(self.collision_topic, String, self.collision_cb, queue_size=20)
        self.sub_stop = rospy.Subscriber(self.stop_topic, Empty, self.stop_cb, queue_size=20)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "[comm_full_dump_sidecar] enable=%s run_id=%s buffer_max=%d tx=%s rx=%s",
            str(self.enabled).lower(),
            self.run_id,
            self.buffer_max,
            self.tx_topic,
            self.rx_topic,
        )

    def msg_to_record(self, msg, direction, event, note):
        return {
            "wall_time": now_iso(),
            "ros_time": rospy.Time.now().to_sec(),
            "direction": direction,
            "event": event,
            "note": note,
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

    def append_record(self, record):
        if not self.enabled:
            return
        line = json.dumps(record, ensure_ascii=True)
        with self.lock:
            self.msg_seen += 1
            if self.triggered:
                self.append_line_to_file(line)
                self.msg_written += 1
                return
            self.buffer.append(line)
            if len(self.buffer) > max(1, self.buffer_max):
                self.buffer.popleft()
                self.msg_dropped += 1

    def ensure_run_dir(self):
        os.makedirs(self.run_dir, exist_ok=True)

    def append_line_to_file(self, line):
        self.ensure_run_dir()
        with open(self.dump_path, "a", encoding="ascii") as f:
            f.write(line + "\n")

    def trigger_dump(self, reason):
        if not self.enabled:
            return
        with self.lock:
            if self.triggered:
                return
            self.triggered = True
            self.trigger_reason = reason
            self.trigger_ros_time = rospy.Time.now().to_sec()
            self.ensure_run_dir()
            with open(self.dump_path, "w", encoding="ascii") as f:
                f.write(
                    json.dumps(
                        {
                            "event": "dump_trigger",
                            "reason": reason,
                            "ros_time": self.trigger_ros_time,
                            "wall_time": now_iso(),
                        },
                        ensure_ascii=True,
                    )
                    + "\n"
                )
                for line in self.buffer:
                    f.write(line + "\n")
                    self.msg_written += 1
            self.buffer.clear()
        rospy.logwarn("[comm_full_dump_sidecar] triggered by %s, file=%s", reason, self.dump_path)

    def tx_cb(self, msg):
        self.tx_seen += 1
        self.append_record(self.msg_to_record(msg, "tx", "observe_tx", "broadcast_traj_from_planner"))

    def rx_cb(self, msg):
        self.rx_seen += 1
        self.append_record(self.msg_to_record(msg, "rx", "observe_rx", "broadcast_traj_to_planner"))

    def collision_cb(self, _msg):
        self.trigger_dump("collision_event")

    def stop_cb(self, _msg):
        self.trigger_dump("mandatory_stop")

    def write_meta(self, final):
        if not self.enabled:
            return
        self.ensure_run_dir()
        with self.lock:
            meta = {
                "run_id": self.run_id,
                "start_wall_time": self.start_wall_time,
                "start_ros_time": self.start_ros_time,
                "enabled": self.enabled,
                "tx_topic": self.tx_topic,
                "rx_topic": self.rx_topic,
                "collision_topic": self.collision_topic,
                "stop_topic": self.stop_topic,
                "buffer_max": self.buffer_max,
                "triggered": self.triggered,
                "trigger_reason": self.trigger_reason,
                "trigger_ros_time": self.trigger_ros_time,
                "msg_seen": self.msg_seen,
                "msg_written": self.msg_written,
                "msg_dropped": self.msg_dropped,
                "tx_seen": self.tx_seen,
                "rx_seen": self.rx_seen,
                "dump_file": self.dump_path,
                "final": final,
                "wall_time": now_iso(),
            }
        with open(self.meta_path, "w", encoding="ascii") as f:
            json.dump(meta, f, indent=2, ensure_ascii=True)

    def on_shutdown(self):
        self.write_meta(final=True)


if __name__ == "__main__":
    rospy.init_node("comm_full_dump_sidecar", anonymous=False)
    node = CommFullDumpSidecar()
    if not node.enabled:
        rospy.logwarn("[comm_full_dump_sidecar] disabled, exiting immediately")
    else:
        node.write_meta(final=False)
        rospy.spin()
