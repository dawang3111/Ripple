"""One isolated wall-clock simulation. Executed only by the explicit `run` command."""
import argparse
import json
import os
import re
import signal
import shlex
import socket
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .core import ROOT, digest, write_json
from .process_cleanup import cleanup_owned
from .rule_online import OnlineRules


def make_launch(output, plan, cfg, map_path):
    if plan.get("intensity_mode"):
        from .intensity_capture import launch_for
        return launch_for(output,plan,cfg,map_path)
    original = ROOT / "swarm_exploration/exploration_manager/launch/swarm_exploration.launch"
    tree = ET.parse(original)
    root = tree.getroot()
    count = 0
    for node in root.findall("node"):
        if node.get("name") == "map_pub":
            node.set("args", shlex.quote(str(map_path)))
    for inc in root.findall("include"):
        if inc.get("file", "").endswith("/single_drone_exploration.xml"):
            prior=next((a for a in inc.findall("arg") if a.get("name")=="evaluation"),None)
            if prior is None:ET.SubElement(inc,"arg",name="evaluation",value="true")
            else:prior.set("value","true")
            count += 1
    if count != 4:
        raise ValueError("launch structure changed; update isolated simulation adapter")
    solver = output / "solver"
    solver.mkdir()
    for i in cfg["active_ids"]:
        for node in ("exploration_node_%d" % i, "tsp_solver_%d" % i, "acvrp_solver_%d" % i):
            ET.SubElement(root, "param", name="/" + node + "/exploration/mtsp_dir", value=str(solver))
        ET.SubElement(root, "param", name="/exploration_node_%d/exploration/tsp_dir" % i, value=str(solver))
    for name, value in {"source_id": plan["source_id"], "offset_s": plan["offset_s"], "fraction": plan["fraction"],
                        "run_budget_s": cfg["task_budget_s"], "control": plan.get("control", False)}.items():
        ET.SubElement(root, "param", name="/racer_eval/" + name, value=str(value).lower())
    if plan.get("rule_group"):
        ET.SubElement(root, "param", name="/racer_eval/rule_mode", value="true")
    ET.SubElement(root, "param", name="/use_sim_time", value="false")
    path = output / "isolated.launch"
    tree.write(str(path), encoding="unicode")
    return path


def stop(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in [(signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


def memory_guard():
    limit = int(os.environ.get('RACER_EVAL_MIN_AVAILABLE_MB', '0'))
    if not limit:
        return
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:') and int(line.split()[1]) < limit * 1024:
            raise RuntimeError('resource_conflict_available_memory_below_%d_MB' % limit)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial-dir", required=True)
    args = parser.parse_args()
    output = Path(args.trial_dir).resolve()
    plan = json.loads((output / "plan.json").read_text())
    cfg = json.loads((output.parent / "config.json").read_text())
    map_path = Path(plan["map_path"])
    # A fresh master, ROS home, logs and solver directory for every trial. Never
    # discover/connect to a user's master or invoke rosnode kill / network tools.
    try:
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
    except OSError as exc:
        write_json(output / "capture.json", {**plan, "status": "capture_failed", "end_ns": time.monotonic_ns(),
                                             "failure_reason": "isolated_loopback_socket_unavailable:" + str(exc)})
        return 1
    os.environ.update(ROS_MASTER_URI="http://127.0.0.1:%d" % port,
                      ROS_IP="127.0.0.1", ROS_HOSTNAME="127.0.0.1",
                      ROS_HOME=str(output / "ros_home"), ROS_LOG_DIR=str(output / "ros_logs"))
    launch = make_launch(output, plan, cfg, map_path)
    import rospy
    from std_msgs.msg import String, Empty
    from bspline.msg import Bspline
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Log
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2
    import rosgraph

    master = simulation = None
    lock = threading.RLock()
    states, ready, starts = {}, set(), {}
    scene = [False]
    online = OnlineRules(plan,cfg) if plan.get("rule_group") and not plan.get("intensity_mode") else None
    subscribers = []
    result = {**plan, "status": "starting", "master_uri": os.environ["ROS_MASTER_URI"], "end_ns": None}
    events = (output / "events.jsonl").open("x", buffering=1)

    def record(e):
        e["observed_ns"] = time.monotonic_ns()
        with lock:
            if events.closed:
                return
            events.write(json.dumps(e, allow_nan=False, separators=(",", ":")) + "\n")
            if online is not None: online.observe(e)

    def telemetry(msg):
        try:
            e = json.loads(msg.data)
            record(e)
            with lock:
                # Initial advertise may precede TCP connection; any structured
                # adapter event (especially snapshots) proves instrumentation is ready.
                ready.add(e["id"])
                states[e["id"]] = e.get("fsm")
                if e["kind"] == "task_started":
                    starts[e["id"]] = e
        except (ValueError, KeyError) as exc:
            record({"kind": "capture_error", "reason": str(exc)})

    def trajectory(msg, i):
        record({"kind": "trajectory", "id": i, "ros_s": rospy.Time.now().to_sec(),
                "traj_id": msg.traj_id, "start_s": msg.start_time.to_sec(), "order": msg.order,
                "knots": list(msg.knots), "pos_pts": [[p.x, p.y, p.z] for p in msg.pos_pts]})

    def broadcast(msg):
        record({"kind":"observed_broadcast_trajectory","id":msg.drone_id,"ros_s":rospy.Time.now().to_sec(),
                "traj_id":msg.traj_id,"start_s":msg.start_time.to_sec(),"order":msg.order,
                "knots":list(msg.knots),"pos_pts":[[p.x,p.y,p.z] for p in msg.pos_pts]})

    def logs(msg):
        match = re.fullmatch(r"Total time: ([0-9.eE+-]+)", msg.msg)
        node = re.fullmatch(r"/exploration_node_(\d+)", msg.name)
        if match and node:
            record({"kind": "planning_latency", "id": int(node.group(1)), "ros_s": msg.header.stamp.to_sec(),
                    "duration_s": float(match.group(1))})

    def geometry(msg):
        with lock:
            if scene[0]:
                return
            points = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            np.save(str(output / "scene_points.npy"), points, allow_pickle=False)
            scene[0] = True
            record({"kind": "scene", "point_n": len(points), "sha256": digest(output / "scene_points.npy")})

    try:
        memory_guard()
        core_log = (output / "master_stdout.log").open("x")
        sim_log = (output / "simulation_stdout.log").open("x")
        master = subprocess.Popen(["roscore", "-p", str(port)], stdout=core_log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + cfg["startup_budget_s"]
        while time.monotonic() < deadline:
            memory_guard()
            try:
                rosgraph.Master("/racer_eval_probe").getPid()
                break
            except Exception:
                if master.poll() is not None:
                    raise RuntimeError("isolated_master_exited")
                time.sleep(0.1)
        else:
            raise RuntimeError("isolated_master_start_timeout")
        rospy.init_node("racer_eval_capture", anonymous=True, disable_signals=True)
        subscribers.append(rospy.Subscriber("/racer_eval/events", String, telemetry, queue_size=20000))
        subscribers.append(rospy.Subscriber("/rosout_agg", Log, logs, queue_size=10000))
        subscribers.append(rospy.Subscriber("/map_generator/global_cloud", PointCloud2, geometry, queue_size=1))
        for i in cfg["active_ids"]:
            subscribers.append(rospy.Subscriber("/planning/bspline_%d" % i, Bspline, trajectory, callback_args=i, queue_size=1000))
        if plan.get('intensity_mode'):
            subscribers.append(rospy.Subscriber('/planning/swarm_traj',Bspline,broadcast,queue_size=10000))
            flush = rospy.Publisher('/racer_intensity/flush',Empty,queue_size=1)
        trigger = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        arm = rospy.Publisher("/racer_eval/rule_arm_%d" % plan["source_id"], String, queue_size=1) if online else None
        simulation = subprocess.Popen(["roslaunch", str(launch)], stdout=sim_log, stderr=subprocess.STDOUT, start_new_session=True)
        while time.monotonic() < deadline:
            memory_guard()
            with lock:
                if ready.issuperset(cfg["active_ids"]) and all(states.get(i) == "WAIT_TRIGGER" for i in cfg["active_ids"]) and scene[0] and trigger.get_num_connections() >= len(cfg["active_ids"]) and (arm is None or arm.get_num_connections() >= 1):
                    break
            if simulation.poll() is not None:
                raise RuntimeError("simulation_exited_before_trigger")
            time.sleep(0.05)
        else:
            raise RuntimeError("adapter_odom_map_or_wait_trigger_not_ready")
        write_json(output / "ros_parameters.json", rospy.get_param("/"))
        record({"kind": "trigger_sent", "ros_s": rospy.Time.now().to_sec()})
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.orientation.w = 1.0
        trigger.publish(msg)
        begin = time.monotonic()
        result["status"] = "running"
        while time.monotonic() - begin < cfg["task_budget_s"]:
            memory_guard()
            with lock:
                complete = all(i in starts and states.get(i) == "FINISH" for i in cfg["active_ids"])
            if online is not None:
                with lock:
                    decision=online.evaluate(time.monotonic_ns(),rospy.Time.now().to_sec(),
                                             states.get(plan['source_id']) in ('PLAN_TRAJ','PUB_TRAJ','EXEC_TRAJ'))
                if decision is not None:
                    write_json(output / 'rule_decision.json',decision)
                    record(decision)
                    arm.publish(String(data='%d %d' % (decision['decision_ns'],decision['expires_ns'])))
            if complete:
                result["status"] = "completed"
                break
            if simulation.poll() is not None:
                raise RuntimeError("simulation_exited_during_task")
            time.sleep(0.05)
        else:
            result["status"] = "task_budget_exhausted"
        result["end_ns"] = time.monotonic_ns()
        with lock:
            if not all(i in starts for i in cfg["active_ids"]):
                result["status"] = "task_not_started_on_all_active_ids"
            if plan["source_id"] in starts:
                e = starts[plan["source_id"]]
                result["control_cutoff_ns"] = e["mono_ns"] + int(plan["offset_s"] * 1e9)
                result["control_cutoff_ros_s"] = e["ros_s"] + plan["offset_s"]
    except (Exception, KeyboardInterrupt) as exc:
        result.update(status="capture_failed", failure_reason=str(exc), end_ns=time.monotonic_ns())
    finally:
        if plan.get('intensity_mode') and 'flush' in locals():
            flush.publish(Empty())
            time.sleep(.15)
        for sub in subscribers:
            sub.unregister()
        stop(simulation)
        stop(master)
        try:
            cleanup_owned(output)
        except Exception as exc:
            result.update(status="capture_failed", failure_reason="owned_cleanup_failed:" + str(exc))
        with lock:
            events.close()
        result["end_ns"] = result["end_ns"] or time.monotonic_ns()
        if online is not None:
            result['rule_monitor']={'decision_sent':online.armed,'evaluations':online.evaluations,
                                    'missing_feature_counts':online.missing_counts,'arm_evidence':online.arm_evidence,
                                    'skip_reason':'' if online.armed else 'rule_not_matched_in_window'}
        write_json(output / "capture.json", result)
        if "core_log" in locals():
            core_log.close()
        if "sim_log" in locals():
            sim_log.close()
    return 0 if result["status"] in ("completed", "task_budget_exhausted") else 1


if __name__ == "__main__":
    raise SystemExit(main())
