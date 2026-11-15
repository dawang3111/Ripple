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
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        return json.load(f)


def read_jsonl(path: str) -> List[dict]:
    out: List[dict] = []
    if not path or not os.path.isfile(path):
        return out
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except Exception:
                continue
    return out


def parse_ros_time_from_line(line: str) -> Optional[float]:
    m = re.search(r"\[([0-9]+(?:\.[0-9]+)?)\]:", line)
    if not m:
        return None
    try:
        return float(m.group(1))
    except Exception:
        return None


def find_first_line(lines: List[str], patterns: List[re.Pattern]) -> Tuple[int, str, Optional[float]]:
    for idx, line in enumerate(lines):
        for pat in patterns:
            if pat.search(line):
                return idx + 1, line, parse_ros_time_from_line(line)
    return -1, "", None


def safe_float(v) -> Optional[float]:
    try:
        return float(v)
    except Exception:
        return None


def clamp_score(v: float) -> int:
    if v < 0:
        return 0
    if v > 100:
        return 100
    return int(round(v))


def relation_summary(relation: str, event_kind: str) -> str:
    if relation == "likely":
        return f"comm fuzz is likely related to {event_kind}"
    if relation == "possible":
        return f"comm fuzz may be related to {event_kind}"
    if relation == "unlikely":
        return f"comm fuzz is unlikely to be root cause of {event_kind}"
    return "insufficient data to determine comm-event relation"


def parse_collision_anchor(root_cause_report: Optional[dict]) -> Dict:
    out = {
        "event_kind": "none",
        "event_ros_time": None,
        "collision_type": "none",
        "collision_pair": "NA",
        "target_involved": False,
    }
    if not root_cause_report:
        return out

    analysis = root_cause_report.get("analysis", {})
    signals = analysis.get("signals", {})
    col_info = root_cause_report.get("collision_info", {})
    col_event = col_info.get("event", {})
    col_payload = col_info.get("collision", {})

    collision_ros = safe_float(col_event.get("ros_time"))
    if collision_ros is None:
        collision_ros = safe_float(signals.get("first_collision_ros_time"))

    emergency_ros = safe_float(signals.get("first_emergency_ros_time"))

    if collision_ros is not None:
        out["event_kind"] = "collision"
        out["event_ros_time"] = collision_ros
    elif emergency_ros is not None:
        out["event_kind"] = "emergency_stop"
        out["event_ros_time"] = emergency_ros

    ctype = str(col_payload.get("type", "none"))
    out["collision_type"] = ctype

    if ctype == "drone_drone":
        da = col_payload.get("drone_a")
        db = col_payload.get("drone_b")
        if da is not None and db is not None:
            out["collision_pair"] = f"{da}-{db}"
    elif ctype == "drone_obstacle":
        did = col_payload.get("drone_id")
        if did is not None:
            out["collision_pair"] = f"drone-{did}"

    return out


def parse_window(comm_meta: Optional[dict]) -> Dict:
    out = {
        "available": False,
        "window_start_ros": None,
        "window_end_ros": None,
        "target_drone_id": None,
        "msg_seen": None,
        "msg_fuzzed": None,
        "mode": "NA",
        "factor_name": "NA",
        "factor_value": None,
    }
    if not comm_meta:
        return out

    s0 = safe_float(comm_meta.get("start_ros_time"))
    ws = safe_float(comm_meta.get("window_start_s"))
    wd = safe_float(comm_meta.get("window_duration_s"))
    if s0 is not None and ws is not None and wd is not None:
        out["available"] = True
        out["window_start_ros"] = s0 + ws
        out["window_end_ros"] = s0 + ws + wd

    out["target_drone_id"] = comm_meta.get("target_drone_id")
    out["msg_seen"] = comm_meta.get("msg_seen")
    out["msg_fuzzed"] = comm_meta.get("msg_fuzzed")
    out["mode"] = str(comm_meta.get("mode", "NA"))
    out["factor_name"] = str(comm_meta.get("factor_name", "NA"))
    out["factor_value"] = safe_float(comm_meta.get("factor_value"))
    return out


def parse_comm_trace(trace_file: str) -> Dict:
    events = read_jsonl(trace_file)
    first_fuzz_ros = None
    fuzz_events = 0
    for item in events:
        evt = str(item.get("event", ""))
        if evt.startswith("enqueue_") or evt.startswith("send_"):
            fuzz_events += 1
            rt = safe_float(item.get("ros_time"))
            if rt is not None and (first_fuzz_ros is None or rt < first_fuzz_ros):
                first_fuzz_ros = rt
    return {
        "trace_exists": os.path.isfile(trace_file),
        "trace_file": trace_file,
        "trace_line_count": len(events),
        "first_fuzz_ros_time": first_fuzz_ros,
        "fuzz_event_count": fuzz_events,
    }


def parse_rx_safety(rx_glob: str, safety_glob: str) -> Dict:
    rx_files = sorted(glob.glob(rx_glob)) if rx_glob else []
    safety_files = sorted(glob.glob(safety_glob)) if safety_glob else []

    rx_total = 0
    rx_anomaly = 0
    rx_first_anomaly_ros = None
    for fp in rx_files:
        for it in read_jsonl(fp):
            rx_total += 1
            event = str(it.get("event", ""))
            accepted = it.get("accepted", True)
            if ("drop" in event) or ("reject" in event) or (accepted is False):
                rx_anomaly += 1
                rt = safe_float(it.get("ros_time"))
                if rt is not None and (rx_first_anomaly_ros is None or rt < rx_first_anomaly_ros):
                    rx_first_anomaly_ros = rt

    safety_total = 0
    safety_anomaly = 0
    safety_first_anomaly_ros = None
    for fp in safety_files:
        for it in read_jsonl(fp):
            safety_total += 1
            event = str(it.get("event", ""))
            reason = str(it.get("reason", ""))
            if (
                event in ("safety_trigger", "mandatory_stop", "swarm_check")
                or ("emergency" in reason)
                or ("mandatory" in reason)
                or ("depth_lost" in reason)
            ):
                safety_anomaly += 1
                rt = safe_float(it.get("ros_time"))
                if rt is not None and (safety_first_anomaly_ros is None or rt < safety_first_anomaly_ros):
                    safety_first_anomaly_ros = rt

    return {
        "rx_trace_files": rx_files,
        "safety_trace_files": safety_files,
        "rx_total": rx_total,
        "rx_anomaly_count": rx_anomaly,
        "rx_first_anomaly_ros_time": rx_first_anomaly_ros,
        "safety_total": safety_total,
        "safety_anomaly_count": safety_anomaly,
        "safety_first_anomaly_ros_time": safety_first_anomaly_ros,
    }


def pick_target_involved(anchor: Dict, target_drone_id) -> bool:
    if target_drone_id is None:
        return False
    try:
        tid = int(target_drone_id)
    except Exception:
        return False

    pair = str(anchor.get("collision_pair", "NA"))
    if pair == "NA":
        return False
    if pair.startswith("drone-"):
        try:
            return int(pair.split("-", 1)[1]) == tid
        except Exception:
            return False
    if "-" in pair:
        parts = pair.split("-")
        try:
            return tid in (int(parts[0]), int(parts[1]))
        except Exception:
            return False
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="Analyze comm-fuzz and collision/emergency relation")
    ap.add_argument("--runtime-log", required=True)
    ap.add_argument("--root-cause-report", default="")
    ap.add_argument("--comm-fuzz-meta", default="")
    ap.add_argument("--comm-fuzz-trace", default="")
    ap.add_argument("--planner-rx-glob", default="")
    ap.add_argument("--planner-safety-glob", default="")
    ap.add_argument("--output", required=True)
    ap.add_argument("--shell-kv", default="")
    ap.add_argument("--near-sec", type=float, default=1.5)
    ap.add_argument("--late-sec", type=float, default=5.0)
    args = ap.parse_args()

    runtime_lines = read_text_lines(args.runtime_log)
    root_cause_report = read_json(args.root_cause_report) if args.root_cause_report else None
    comm_meta = read_json(args.comm_fuzz_meta) if args.comm_fuzz_meta else None

    anchor = parse_collision_anchor(root_cause_report)
    window = parse_window(comm_meta)
    comm_trace = parse_comm_trace(args.comm_fuzz_trace)
    trace_stats = parse_rx_safety(args.planner_rx_glob, args.planner_safety_glob)

    fatal_ln, fatal_line, fatal_ros = find_first_line(runtime_lines, FATAL_PATTERNS)
    _em_ln, _em_line, em_ros = find_first_line(runtime_lines, EMERGENCY_PATTERNS)

    if anchor.get("event_ros_time") is None and em_ros is not None:
        anchor["event_kind"] = "emergency_stop"
        anchor["event_ros_time"] = em_ros

    target_involved = pick_target_involved(anchor, window.get("target_drone_id"))
    anchor["target_involved"] = target_involved

    why: List[str] = []
    relation = "insufficient"
    score = 0

    event_kind = str(anchor.get("event_kind", "none"))
    event_ros = safe_float(anchor.get("event_ros_time"))
    msg_fuzzed = int(window.get("msg_fuzzed") or 0)
    rx_anomaly_count = int(trace_stats.get("rx_anomaly_count") or 0)
    safety_anomaly_count = int(trace_stats.get("safety_anomaly_count") or 0)

    if event_ros is None or event_kind == "none":
        relation = "insufficient"
        why.append("no collision/emergency anchor time")
    elif not window.get("available", False):
        relation = "insufficient"
        why.append("comm fuzz window unavailable")
    else:
        w0 = window.get("window_start_ros")
        w1 = window.get("window_end_ros")
        delta_to_start = event_ros - w0
        delta_to_end = event_ros - w1

        if event_ros < w0:
            relation = "unlikely"
            score = 20
            why.append("event happened before comm fuzz window")
        elif event_ros <= w1:
            if msg_fuzzed <= 0:
                relation = "unlikely"
                score = 25
                why.append("event in window but msg_fuzzed=0")
            elif target_involved and (rx_anomaly_count > 0 or safety_anomaly_count > 0):
                relation = "likely"
                score = 85
                why.append("event in window with fuzzed messages and target involvement")
            else:
                relation = "possible"
                score = 60
                why.append("event in window with fuzzed messages")
        else:
            if msg_fuzzed > 0 and delta_to_end <= args.near_sec and (
                target_involved or rx_anomaly_count > 0 or safety_anomaly_count > 0
            ):
                relation = "possible"
                score = 58
                why.append("event shortly after fuzz window and related traces exist")
            elif msg_fuzzed > 0 and delta_to_end <= args.late_sec:
                relation = "possible"
                score = 45
                why.append("event after window with short lag")
            else:
                relation = "unlikely"
                score = 20
                why.append("event happened too late after fuzz window")

        why.append(f"dt_to_window_start={delta_to_start:.3f}s")
        why.append(f"dt_to_window_end={delta_to_end:.3f}s")

    if fatal_ros is not None and event_ros is not None and fatal_ros <= event_ros:
        relation = "unlikely"
        score = min(score if score > 0 else 20, 20)
        why.append("fatal crash signal appears before or at event time")

    if msg_fuzzed <= 0 and relation in ("possible", "likely"):
        relation = "unlikely"
        score = 25
        why.append("downgraded because msg_fuzzed=0")

    score = clamp_score(score)
    summary = relation_summary(relation, event_kind)

    result = {
        "relation": relation,
        "score": score,
        "summary": summary,
        "why": why,
        "evidence": {
            "event_kind": event_kind,
            "event_ros_time": event_ros,
            "collision_type": anchor.get("collision_type"),
            "collision_pair": anchor.get("collision_pair"),
            "target_drone_id": window.get("target_drone_id"),
            "target_involved": target_involved,
            "window_start_ros": window.get("window_start_ros"),
            "window_end_ros": window.get("window_end_ros"),
            "msg_seen": window.get("msg_seen"),
            "msg_fuzzed": window.get("msg_fuzzed"),
            "mode": window.get("mode"),
            "factor_name": window.get("factor_name"),
            "factor_value": window.get("factor_value"),
            "first_fuzz_ros_time": comm_trace.get("first_fuzz_ros_time"),
            "comm_trace_exists": comm_trace.get("trace_exists"),
            "comm_trace_line_count": comm_trace.get("trace_line_count"),
            "rx_anomaly_count": trace_stats.get("rx_anomaly_count"),
            "safety_anomaly_count": trace_stats.get("safety_anomaly_count"),
            "first_rx_anomaly_ros_time": trace_stats.get("rx_first_anomaly_ros_time"),
            "first_safety_anomaly_ros_time": trace_stats.get("safety_first_anomaly_ros_time"),
            "first_fatal_line_no": fatal_ln,
            "first_fatal_line": fatal_line,
            "first_fatal_ros_time": fatal_ros,
        },
        "inputs": {
            "runtime_log": args.runtime_log,
            "root_cause_report": args.root_cause_report,
            "comm_fuzz_meta": args.comm_fuzz_meta,
            "comm_fuzz_trace": args.comm_fuzz_trace,
            "planner_rx_glob": args.planner_rx_glob,
            "planner_safety_glob": args.planner_safety_glob,
        },
    }

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w", encoding="ascii") as f:
        json.dump(result, f, ensure_ascii=True, indent=2)

    if args.shell_kv:
        with open(args.shell_kv, "w", encoding="ascii") as f:
            f.write(f"comm_relation={relation}\n")
            f.write(f"comm_relation_score={score}\n")
            f.write(f"comm_relation_summary={summary}\n")
            f.write(
                "comm_relation_event_ros_time="
                + ("NA" if event_ros is None else str(event_ros))
                + "\n"
            )
            f.write(
                "comm_relation_window_start_ros="
                + ("NA" if window.get("window_start_ros") is None else str(window.get("window_start_ros")))
                + "\n"
            )
            f.write(
                "comm_relation_window_end_ros="
                + ("NA" if window.get("window_end_ros") is None else str(window.get("window_end_ros")))
                + "\n"
            )
            f.write(f"comm_relation_target_involved={str(target_involved).lower()}\n")
            f.write(f"comm_relation_msg_fuzzed={msg_fuzzed}\n")
            f.write(f"comm_relation_rx_anomaly_count={rx_anomaly_count}\n")
            f.write(f"comm_relation_safety_anomaly_count={safety_anomaly_count}\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())