import csv
import gzip
import hashlib
import json
import math
import random
import subprocess
import time
import uuid
from collections import defaultdict, deque
from pathlib import Path

import numpy as np

TARGETS = ("inter_drone_collision", "obstacle_collision", "deadlock",
           "delay_timeout", "planning_failure", "drift")
ACTIVE = {"PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ"}
ROOT = Path(__file__).resolve().parents[2]


def config(path=None):
    value = json.loads(Path(path or Path(__file__).with_name("config.json")).read_text())
    lo, hi = value["injection_window_s"]
    a, b = value["delay_fraction"]
    if not (0 <= lo < hi < value["task_budget_s"] and 0 < a <= b < 1):
        raise ValueError("invalid injection window/fraction or run budget")
    if len(set(value["active_ids"])) != len(value["active_ids"]) or not value["active_ids"]:
        raise ValueError("active_ids must be distinct and nonempty")
    if value["active_ids"] != [1, 2, 3, 4]:
        raise ValueError("current launch adapter supports active IDs 1,2,3,4 only")
    if value["thresholds"]["state_timeout_s"] != 0.2:
        raise ValueError("source adapter protocol budget is RACER's 0.2 s; changing it requires an adapter change")
    if not 0 < value["holdout_fraction"] < 1:
        raise ValueError("holdout_fraction must be between zero and one")
    for key in ("startup_budget_s", "local_radius_m", "alignment_tolerance_s", "max_sample_gap_s", "future_horizon_s",
                "future_step_s", "history_s", "min_cluster_samples", "min_rule_support", "max_rules_per_target", "bootstrap_samples"):
        if not math.isfinite(value[key]) or value[key] <= 0:
            raise ValueError("configuration must be finite and positive: " + key)
    if value["min_cluster_samples"] < 2:
        raise ValueError("min_cluster_samples must be >=2")
    if value["history_s"] != 5.0 or value["future_horizon_s"] != 3.0:
        raise ValueError("named feature schema requires 5 s history and 3 s future; adapt field names for other horizons")
    for key, v in value["thresholds"].items():
        if not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError("threshold must be finite and positive: " + key)
    return value


def unique_output(parent, prefix):
    parent = Path(parent).resolve()
    parent.mkdir(parents=True, exist_ok=True)
    # mkdir, not exists-then-create: safe even with concurrent invocations.
    for _ in range(10):
        p = parent / (prefix + "_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "_" + uuid.uuid4().hex)
        try:
            p.mkdir()
            return p
        except FileExistsError:
            pass
    raise FileExistsError("could not allocate a unique output directory")


def write_json(path, obj):
    Path(path).write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def write_csv(path, rows, fields):
    with Path(path).open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(fields), extrasaction="raise")
        w.writeheader()
        for row in rows:
            w.writerow({k: json.dumps(v, ensure_ascii=False, allow_nan=False) if isinstance(v, (dict, list))
                        else str(v).lower() if isinstance(v, bool) else v for k, v in row.items()})


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1048576), b""):
            h.update(block)
    return h.hexdigest()


def version():
    return subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip()


def trial_plan(index, master_seed, cfg):
    # No dependence on execution order, Python hash randomization, or previous skips.
    seed = int.from_bytes(hashlib.sha256((str(master_seed) + ":" + str(index)).encode()).digest()[:8], "big")
    rng = random.Random(seed)
    return {"trial_id": "trial_%04d" % index, "seed": seed,
            "source_id": rng.choice(cfg["active_ids"]),
            "offset_s": rng.uniform(*cfg["injection_window_s"]),
            "fraction": rng.uniform(*cfg["delay_fraction"])}


def read_events(path):
    rows = []
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt') as f:
        for line_no, line in enumerate(f, 1):
            if not line.strip():
                continue
            try:
                e = json.loads(line)
            except ValueError as exc:
                raise ValueError("invalid JSON at %s:%d: %s" % (path, line_no, exc)) from exc
            if not isinstance(e, dict) or not isinstance(e.get("kind"), str):
                raise ValueError("missing event kind at %s:%d" % (path, line_no))
            if not isinstance(e.get("observed_ns"), int):
                raise ValueError("missing integer observer timestamp at %s:%d" % (path, line_no))
            if "mono_ns" in e and not isinstance(e["mono_ns"], int):
                raise ValueError("source monotonic time must be integer at %s:%d" % (path, line_no))
            rows.append(e)
    return sorted(rows, key=lambda x: x["observed_ns"])


class MessageAssociator:
    """FIFO association for repeated payloads; never treat a payload hash as a sequence."""
    def __init__(self):
        self.pending = defaultdict(deque)

    def sent(self, sender, payload, seq, timestamp_ns):
        self.pending[(sender, hashlib.sha256(payload).hexdigest())].append((seq, timestamp_ns))

    def received(self, sender, payload, timestamp_ns):
        q = self.pending[(sender, hashlib.sha256(payload).hexdigest())]
        if not q or timestamp_ns < q[0][1]:
            return None
        seq, sent = q.popleft()
        return {"seq": seq, "sent_ns": sent, "received_ns": timestamp_ns,
                "latency_ns": timestamp_ns - sent}


def injection_record(events, plan):
    tx = [x for x in events if x.get("kind") == "tx" and x.get("id") == plan["source_id"]]
    scheduled = [x for x in tx if x.get("action") == "scheduled"]
    released = [x for x in tx if x.get("action") == "release" and x.get("actual_injected")]
    skipped = [x for x in tx if x.get("action") == "skip"]
    result = {"actual_injected": False, "contract_valid": False,
              "skip_reason": skipped[0].get("reason", "") if skipped else "selected_time_not_reached_or_adapter_missing"}
    if len(scheduled) != 1 or len(released) != 1:
        if released:
            result.update(actual_injected=True, skip_reason="ambiguous_or_multiple_injections")
        elif scheduled:
            result["skip_reason"] = "scheduled_but_no_release_evidence"
        return result
    p, r = scheduled[0], released[0]
    fields = ("id", "seq", "generated_ns", "message_stamp_s", "planned_delay_ns")
    if any(p.get(k) != r.get(k) for k in fields):
        result.update(actual_injected=True, skip_reason="schedule_release_mismatch")
        return result
    forwards = [e for e in tx if e.get("action") in ("forward", "release")]
    forwards.sort(key=lambda e: e["publish_before_ns"])
    seqs = [e["seq"] for e in forwards]
    ordered = seqs == sorted(set(seqs))
    successor = next((e for e in forwards if e["seq"] == r["seq"] + 1), None)
    actual = r["publish_before_ns"] - r["generated_ns"]
    budget_ok = r.get("budget_ok", False) and r["publish_after_ns"] - r["generated_ns"] < r["budget_ns"]
    before_next = successor is not None and r["publish_after_ns"] < successor["publish_before_ns"]
    active = p.get("fsm") in ACTIVE
    starts = [e for e in events if e.get("kind") == "task_started" and e.get("id") == plan["source_id"]]
    elapsed = (r["generated_ns"] - starts[0]["mono_ns"]) * 1e-9 if len(starts) == 1 else None
    after_start = elapsed is not None and elapsed >= plan.get("offset_s", 0)
    cycle_ok = 0 < r["planned_delay_ns"] < r["period_ns"]
    valid = ordered and before_next and budget_ok and active and actual > 0 and after_start and cycle_ok
    result.update(actual_injected=actual > 0, contract_valid=valid,
                  skip_reason="" if valid else "order_deadline_state_or_successor_contract_failed",
                  generated_ns=r["generated_ns"], released_ns=r["publish_before_ns"],
                  planned_release_ns=r["generated_ns"] + r["planned_delay_ns"],
                  release_complete_ns=r["publish_after_ns"], publish_ros_s=r["publish_ros_s"],
                  generated_ros_s=r["message_stamp_s"], seq=r["seq"],
                  measured_period_ns=r["period_ns"], planned_delay_ns=r["planned_delay_ns"],
                  actual_delay_ns=actual, actual_task_elapsed_s=elapsed,
                  release_reason=r.get("reason"), protocol_budget_ns=r["budget_ns"])
    return result


class Trajectory:
    """De Boor convention matches bspline/non_uniform_bspline.cpp; no extrapolation."""
    def __init__(self, e):
        self.degree = int(e["order"])
        self.knots = np.asarray(e["knots"], dtype=float)
        self.points = np.asarray(e["pos_pts"], dtype=float)
        self.start = float(e["start_s"])
        if (self.points.ndim != 2 or self.points.shape[1] != 3 or
                len(self.knots) != len(self.points) + self.degree + 1 or
                self.degree < 1 or len(self.points) <= self.degree or
                np.any(np.diff(self.knots) < 0) or not np.isfinite(self.knots).all() or
                not np.isfinite(self.points).all()):
            raise ValueError("invalid B-spline")
        self.duration = self.knots[-self.degree - 1] - self.knots[self.degree]

    def at(self, ros_s):
        t = ros_s - self.start
        if t < -1e-8 or t > self.duration + 1e-8:
            raise ValueError("trajectory does not cover requested time")
        u = np.clip(t + self.knots[self.degree], self.knots[self.degree], self.knots[-self.degree - 1])
        k = self.degree
        while k + 1 < len(self.points) and self.knots[k + 1] < u:
            k += 1
        d = self.points[k - self.degree:k + 1].copy()
        for r in range(1, self.degree + 1):
            for i in range(self.degree, r - 1, -1):
                den = self.knots[i + 1 + k - r] - self.knots[i + k - self.degree]
                a = 0.0 if den == 0 else (u - self.knots[i + k - self.degree]) / den
                d[i] = (1 - a) * d[i - 1] + a * d[i]
        return d[self.degree]


class VoxelMap:
    """Explicit evaluation geometry: occupied PCD voxels, with a spherical body proxy."""
    def __init__(self, points, resolution):
        self.resolution = resolution
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all() or resolution <= 0:
            raise ValueError("invalid occupied voxel geometry")
        cells = np.unique(np.floor(points / resolution).astype(int), axis=0)
        self.cells = set(map(tuple, cells))
        self.centers = (cells + 0.5) * resolution

    @classmethod
    def read(cls, path, resolution):
        with Path(path).open() as f:
            header = []
            for line in f:
                header.append(line.strip())
                if line.startswith("DATA"):
                    if line.strip() != "DATA ascii":
                        raise ValueError("map adapter supports ASCII PCD only")
                    break
            if "FIELDS x y z" not in header:
                raise ValueError("map adapter requires FIELDS x y z")
            points = np.atleast_2d(np.loadtxt(f))
        if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
            raise ValueError("invalid PCD points")
        return cls(points, resolution)

    def distance(self, p):
        delta = np.maximum(np.abs(self.centers - np.asarray(p)) - self.resolution / 2, 0)
        return float(np.sqrt(np.min(np.sum(delta * delta, axis=1))))

    def collides(self, p, radius):
        p = np.asarray(p)
        base = np.floor(p / self.resolution).astype(int)
        n = int(math.ceil(radius / self.resolution)) + 1
        for x in range(base[0] - n, base[0] + n + 1):
            for y in range(base[1] - n, base[1] + n + 1):
                for z in range(base[2] - n, base[2] + n + 1):
                    if (x, y, z) not in self.cells:
                        continue
                    delta = np.maximum(np.abs((np.array([x, y, z]) + 0.5) * self.resolution - p) - self.resolution / 2, 0)
                    if float(np.dot(delta, delta)) <= radius * radius:
                        return True
        return False
