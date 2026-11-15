#!/usr/bin/env python3
import argparse
import glob
import json
import os
import re
from typing import Dict, List, Optional, Tuple

FATAL_PATTERNS = [
    re.compile(r"boost::wrapexcept<boost::lock_error>", re.IGNORECASE),
    re.compile(r"terminate called after throwing", re.IGNORECASE),
    re.compile(r"Segmentation fault", re.IGNORECASE),
    re.compile(r"core dumped", re.IGNORECASE),
    re.compile(r"Assertion .* failed", re.IGNORECASE),
    re.compile(r"auto-starting new master", re.IGNORECASE),
]

EMERGENCY_PATTERNS = [
    re.compile(r"Emergency stop", re.IGNORECASE),
    re.compile(r"EMERGENCY_STOP", re.IGNORECASE),
    re.compile(r"Mandatory Stop", re.IGNORECASE),
    re.compile(r"mandatory stop command", re.IGNORECASE),
    re.compile(r"Depth Lost", re.IGNORECASE),
]


def read_text_lines(path: str) -> List[str]:
    if not path or not os.path.isfile(path):
        return []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        return [line.rstrip("\n") for line in f]


def read_json(path: str) -> Optional[dict]:
    if not path or not os.path.isfile(path):
        return None
    with open(path, "r", encoding="ascii", errors="ignore") as f:
        return json.load(f)


def read_jsonl(path: str) -> List[dict]:
    out: List[dict] = []
    if not path or not os.path.isfile(path):
        return out
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            try:
                out.append(json.loads(s))
            except Exception:
                continue
    return out


def find_first_match(lines: List[str], patterns: List[re.Pattern]) -> Tuple[int, str]:
    for idx, line in enumerate(lines):
        for pat in patterns:
            if pat.search(line):
                return idx + 1, line
    return -1, ""


def count_matches(lines: List[str], patterns: List[re.Pattern]) -> int:
    c = 0
    for line in lines:
        for pat in patterns:
            if pat.search(line):
                c += 1
                break
    return c


def extract_ros_time_from_line(line: str) -> Optional[float]:
    # Typical ROS console prefix: [INFO] [1775796499.238811261]: ...
    m = re.search(r"\[([0-9]+(?:\.[0-9]+)?)\]:", line)
    if not m:
        return None
    try:
        return float(m.group(1))
    except Exception:
        return None


def get_collision_info(fuzz_report_dir: str, run_id: str) -> Dict:
    summary_path = os.path.join(fuzz_report_dir, run_id, "summary.json")
    summary = read_json(summary_path)
    if not summary:
        return {
            "exists": False,
            "summary_path": summary_path,
        }

    event = summary.get("event", {})
    collision = event.get("collision", {})
    return {
        "exists": True,
        "summary_path": summary_path,
        "event": event,
        "collision": collision,
        "trace_window_index": summary.get("trace_window_index", {}),
    }


def get_trace_signals(fuzz_report_dir: str, run_id: str) -> Dict:
    base = os.path.join(fuzz_report_dir, run_id)
    safety_files = sorted(glob.glob(os.path.join(base, "planner_safety_trace_drone_*.jsonl")))
    rx_files = sorted(glob.glob(os.path.join(base, "planner_rx_trace_drone_*.jsonl")))

    safety_events: Dict[str, int] = {}
    rx_events: Dict[str, int] = {}

    for fp in safety_files:
        for item in read_jsonl(fp):
            event = str(item.get("event", "unknown"))
            safety_events[event] = safety_events.get(event, 0) + 1

    for fp in rx_files:
        for item in read_jsonl(fp):
            event = str(item.get("event", "unknown"))
            rx_events[event] = rx_events.get(event, 0) + 1

    return {
        "safety_trace_files": safety_files,
        "rx_trace_files": rx_files,
        "safety_events": safety_events,
        "rx_events": rx_events,
    }


def fuzz_overlap(comm_meta: Optional[dict], collision_event: Dict) -> Dict:
    if not comm_meta or not collision_event:
        return {"available": False, "overlap": None}

    start_ros = comm_meta.get("start_ros_time")
    start_s = comm_meta.get("window_start_s")
    dur_s = comm_meta.get("window_duration_s")
    event_ros = collision_event.get("ros_time")

    try:
        start_ros_f = float(start_ros)
        start_s_f = float(start_s)
        dur_s_f = float(dur_s)
        event_ros_f = float(event_ros)
    except Exception:
        return {"available": False, "overlap": None}

    w0 = start_ros_f + start_s_f
    w1 = w0 + dur_s_f
    overlap = (event_ros_f >= w0) and (event_ros_f <= w1)
    return {
        "available": True,
        "window_start_ros": w0,
        "window_end_ros": w1,
        "event_ros": event_ros_f,
        "overlap": overlap,
    }


def classify(runtime_lines: List[str], collision_info: Dict, traces: Dict, comm_meta: Optional[dict]) -> Dict:
    collision_log_pat = re.compile(r"HARD_COLLISION", re.IGNORECASE)
    fatal_count = count_matches(runtime_lines, FATAL_PATTERNS)
    emergency_count = count_matches(runtime_lines, EMERGENCY_PATTERNS)
    fatal_ln, fatal_line = find_first_match(runtime_lines, FATAL_PATTERNS)
    collision_ln, collision_line = find_first_match(runtime_lines, [collision_log_pat])
    collision_log_count = count_matches(runtime_lines, [collision_log_pat])
    emergency_ln, emergency_line = find_first_match(runtime_lines, EMERGENCY_PATTERNS)

    collision_exists = bool(collision_info.get("exists"))
    collision_event = collision_info.get("event", {}) if collision_exists else {}
    collision_payload = collision_info.get("collision", {}) if collision_exists else {}
    collision_type = collision_payload.get("type", "none") if collision_exists else "none"
    if (not collision_exists) and collision_log_count > 0:
        collision_exists = True
        if "drone_drone" in collision_line:
            collision_type = "drone_drone"
        elif "drone_obstacle" in collision_line:
            collision_type = "drone_obstacle"
        else:
            collision_type = "unknown"

    overlap = fuzz_overlap(comm_meta, collision_event)
    safety_events = traces.get("safety_events", {})

    collision_ros_time: Optional[float] = None
    event_ros = collision_event.get("ros_time") if collision_exists else None
    if event_ros is not None:
        try:
            collision_ros_time = float(event_ros)
        except Exception:
            collision_ros_time = None
    if collision_ros_time is None and collision_ln > 0:
        collision_ros_time = extract_ros_time_from_line(collision_line)

    emergency_ros_time: Optional[float] = None
    if emergency_ln > 0:
        emergency_ros_time = extract_ros_time_from_line(emergency_line)

    incident_types: List[str] = []
    if collision_exists:
        incident_types.append("collision")
    if emergency_count > 0:
        incident_types.append("emergency_stop")
    if fatal_count > 0:
        incident_types.append("fatal_crash")
    if not incident_types:
        incident_types.append("none")

    primary = "none"
    confidence = 0.3
    why: List[str] = []

    if collision_exists:
        primary = "collision"
        confidence = 0.9
        why.append(f"detected hard collision ({collision_type})")
        if overlap.get("available") and overlap.get("overlap"):
            why.append("collision happened during comm fuzz window")
        if safety_events.get("mandatory_stop_received", 0) > 0:
            why.append("mandatory stop appears as reaction after safety trigger")

    if (not collision_exists) and emergency_count > 0:
        primary = "emergency_stop"
        confidence = 0.75
        why.append("emergency/mandatory stop logs present")
        if safety_events.get("depth_lost", 0) > 0:
            why.append("safety trace contains depth_lost")
            confidence = 0.85
        elif safety_events.get("mandatory_stop_received", 0) > 0:
            why.append("safety trace contains mandatory_stop_received")

    if (primary == "none") and fatal_count > 0:
        primary = "fatal_crash"
        confidence = 0.8
        why.append("fatal crash signature matched")

    if fatal_count > 0 and "boost::wrapexcept<boost::lock_error>" in fatal_line:
        why.append("mutex lock failure signature observed")

    summary = "unknown"
    if primary == "collision":
        summary = "collision likely driven by close-proximity conflict"
        if overlap.get("available") and overlap.get("overlap"):
            summary += " under active comm fuzz"
    elif primary == "emergency_stop":
        summary = "emergency stop likely caused by safety triggers"
    elif primary == "fatal_crash":
        summary = "runtime fatal crash likely interrupted planner/control loop"
    else:
        summary = "no strong incident signal found"

    return {
        "primary_incident": primary,
        "incident_types": incident_types,
        "confidence": round(confidence, 2),
        "summary": summary,
        "why": why,
        "signals": {
            "fatal_count": fatal_count,
            "emergency_count": emergency_count,
            "collision_log_count": collision_log_count,
            "first_collision_line_no": collision_ln,
            "first_collision_line": collision_line,
            "first_collision_ros_time": collision_ros_time,
            "first_emergency_line_no": emergency_ln,
            "first_emergency_line": emergency_line,
            "first_emergency_ros_time": emergency_ros_time,
            "first_fatal_line_no": fatal_ln,
            "first_fatal_line": fatal_line,
            "collision_exists": collision_exists,
            "collision_type": collision_type,
            "fuzz_overlap": overlap,
            "safety_events": safety_events,
            "rx_events": traces.get("rx_events", {}),
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Analyze root causes for collision/emergency/fatal events in one round")
    ap.add_argument("--runtime-log", required=True)
    ap.add_argument("--fuzz-report-dir", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--comm-fuzz-meta", default="")
    ap.add_argument("--output", required=True)
    ap.add_argument("--shell-kv", default="")
    args = ap.parse_args()

    runtime_lines = read_text_lines(args.runtime_log)
    collision_info = get_collision_info(args.fuzz_report_dir, args.run_id)
    traces = get_trace_signals(args.fuzz_report_dir, args.run_id)
    comm_meta = read_json(args.comm_fuzz_meta) if args.comm_fuzz_meta else None

    result = {
        "runtime_log": args.runtime_log,
        "fuzz_report_dir": args.fuzz_report_dir,
        "run_id": args.run_id,
        "collision_info": collision_info,
        "trace_info": {
            "safety_trace_files": traces.get("safety_trace_files", []),
            "rx_trace_files": traces.get("rx_trace_files", []),
        },
        "comm_fuzz_meta_path": args.comm_fuzz_meta,
        "analysis": classify(runtime_lines, collision_info, traces, comm_meta),
    }

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="ascii") as f:
        json.dump(result, f, ensure_ascii=True, indent=2)

    if args.shell_kv:
        ana = result["analysis"]
        sig = ana.get("signals", {})
        first_collision_ros_time = sig.get("first_collision_ros_time")
        first_emergency_ros_time = sig.get("first_emergency_ros_time")
        with open(args.shell_kv, "w", encoding="ascii") as f:
            f.write(f"primary_incident={ana.get('primary_incident', 'unknown')}\n")
            f.write(f"incident_types={','.join(ana.get('incident_types', []))}\n")
            f.write(f"confidence={ana.get('confidence', 0)}\n")
            f.write(f"summary={ana.get('summary', 'unknown')}\n")
            f.write(
                "first_collision_ros_time="
                + ("NA" if first_collision_ros_time is None else str(first_collision_ros_time))
                + "\n"
            )
            f.write(
                "first_emergency_ros_time="
                + ("NA" if first_emergency_ros_time is None else str(first_emergency_ros_time))
                + "\n"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
