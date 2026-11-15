import hashlib
import itertools
import json
import math
from collections import Counter

import numpy as np

from .core import TARGETS, config as default_config, write_csv, write_json
from .features import extract


def wilson(hits, n):
    if not n:
        return None, None
    z = 1.959963984540054
    p = hits / n
    center = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, center - half), min(1.0, center + half)


def split(rows, target, seed, fraction):
    train, holdout = [], []
    # Stable stratification; singleton classes stay in training and have no claimed validation.
    for label in (False, True):
        group = [r for r in rows if r["labels"][target]["hit"] == label]
        group.sort(key=lambda r: hashlib.sha256((str(seed) + target + r["trial_id"]).encode()).digest())
        n = max(1, int(round(len(group) * fraction))) if len(group) >= 2 else 0
        holdout.extend(group[:n])
        train.extend(group[n:])
    return train, holdout


def kmeans(x, seed, n_init=10):
    """Deterministic k-means++/Lloyd k=2 with explicit duplicate and empty handling."""
    if len(x) < 2 or len(np.unique(x, axis=0)) < 2:
        raise ValueError("fewer_than_two_distinct_vectors")
    rng = np.random.RandomState(seed)
    best = None
    for _ in range(n_init):
        first = rng.randint(len(x))
        dist = np.sum((x - x[first]) ** 2, axis=1)
        second = rng.choice(len(x), p=dist / dist.sum())
        centers = x[[first, second]].copy()
        for _ in range(200):
            labels = np.argmin(np.sum((x[:, None] - centers[None, :]) ** 2, axis=2), axis=1)
            if not all(np.any(labels == k) for k in (0, 1)):
                break
            updated = np.array([x[labels == k].mean(axis=0) for k in (0, 1)])
            if np.max(np.abs(updated - centers)) <= 1e-10:
                centers = updated
                break
            centers = updated
        if len(set(labels)) != 2:
            continue
        # Canonical numeric ordering is for reproducibility, never a risk ranking.
        order = sorted(range(2), key=lambda k: tuple(centers[k]))
        centers = centers[order]
        labels = np.argmin(np.sum((x[:, None] - centers[None, :]) ** 2, axis=2), axis=1)
        inertia = float(np.sum((x - centers[labels]) ** 2))
        if best is None or inertia < best[0]:
            best = inertia, centers, labels
    if best is None:
        raise ValueError("empty_cluster")
    return best[1], best[2]


def matrix(rows, fields):
    return np.array([[r["features"]["fields"].get(f, {}).get("value", np.nan)
                      if r["features"]["fields"].get(f, {}).get("value") is not None else np.nan
                      for f in fields] for r in rows], dtype=float).reshape(len(rows), len(fields))


def rule_mask(x, conditions):
    eligible = np.ones(len(x), dtype=bool)
    mask = eligible.copy()
    for col, op, threshold in conditions:
        finite = np.isfinite(x[:, col])
        eligible &= finite
        values = np.where(finite, x[:, col], 0.0)
        mask &= (values <= threshold) if op == "<=" else (values > threshold)
    return mask & eligible, eligible


def rates(mask, eligible, y):
    a = mask & eligible
    b = ~mask & eligible
    n, m = int(a.sum()), int(b.sum())
    h, j = int(y[a].sum()), int(y[b].sum())
    return n, m, h, j


def candidates(x, y, centers, cfg):
    atoms = []
    for col in range(x.shape[1]):
        vals = x[np.isfinite(x[:, col]), col]
        if len(np.unique(vals)) < 2:
            continue
        cuts = list(np.percentile(vals, [20, 50, 80]))
        cuts += [c[col] for c in centers]
        atoms.extend(((col, op, float(v)),) for v in sorted(set(cuts)) for op in ("<=", ">"))

    def score(rule):
        mask, eligible = rule_mask(x, rule)
        n, m, h, j = rates(mask, eligible, y)
        if min(n, m) < cfg["min_rule_support"]:
            return -1
        return abs(h / n - j / m) * math.sqrt(n / max(1, int(eligible.sum())))

    beam = sorted(((score(r), r) for r in atoms), key=lambda v: (-v[0], v[1]))[:24]
    pool = list(beam)
    for size in (2, 3):
        expanded = set()
        for score0, rule in beam:
            if score0 <= 0:
                continue
            for _, atom in pool[:24]:
                if len(atom) == 1 and atom[0][0] not in {c[0] for c in rule}:
                    combined = tuple(sorted(rule + atom))
                    if len(combined) == size:
                        expanded.add(combined)
        beam = sorted(((score(r), r) for r in expanded), key=lambda v: (-v[0], v[1]))[:24]
        pool += beam
    chosen = []
    masks = set()
    for score0, rule in sorted(pool, key=lambda v: (-v[0], len(v[1]), v[1])):
        if score0 <= 0:
            continue
        signature = rule_mask(x, rule)[0].tobytes()
        if signature in masks:
            continue
        masks.add(signature)
        chosen.append(rule)
        if len(chosen) >= cfg["max_rules_per_target"]:
            break
    return chosen


def validation_metrics(x, y, rule, cfg, seed):
    mask, eligible = rule_mask(x, rule)
    n, m, h, j = rates(mask, eligible, y)
    p, q = h / n if n else None, j / m if m else None
    baseline = float(y[eligible].mean()) if eligible.any() else None
    ci, other_ci = wilson(h, n), wilson(j, m)
    support_ci = wilson(n, n + m)
    negative_n = int((eligible & ~y).sum())
    negative_trigger = int((mask & eligible & ~y).sum())
    negative_ci = wilson(negative_trigger, negative_n)
    rng = np.random.RandomState(seed)
    lifts = []
    inds = np.where(eligible)[0]
    if n and baseline:
        for _ in range(cfg["bootstrap_samples"]):
            sample = rng.choice(inds, len(inds), replace=True)
            selected = mask[sample]
            base = y[sample].mean()
            if selected.any() and base > 0:
                lifts.append(float(y[sample][selected].mean() / base))
    lift_ci = list(np.percentile(lifts, [2.5, 97.5])) if len(lifts) >= 20 else [None, None]
    return {"validation_n": len(inds), "excluded_missing_n": len(x) - len(inds),
            "support_n": n, "support_rate": n / len(inds) if len(inds) else None,
            "support_ci_low": support_ci[0], "support_ci_high": support_ci[1],
            "nonhit_sample_n": negative_n, "nonhit_trigger_n": negative_trigger,
            "nonhit_trigger_rate": negative_trigger / negative_n if negative_n else None,
            "nonhit_trigger_ci_low": negative_ci[0], "nonhit_trigger_ci_high": negative_ci[1],
            "target_hit_n": h, "target_rate": p, "target_not_rate": 1 - p if p is not None else None,
            "target_not_rate_ci_low": 1 - ci[1] if n else None,
            "target_not_rate_ci_high": 1 - ci[0] if n else None,
            "nontrigger_n": m, "nontrigger_target_rate": q, "baseline_target_rate": baseline,
            "difference_pp": 100 * (p - q) if p is not None and q is not None else None,
            "difference_ci_low_pp": 100 * (ci[0] - other_ci[1]) if n and m else None,
            "difference_ci_high_pp": 100 * (ci[1] - other_ci[0]) if n and m else None,
            "target_rate_ci_low": ci[0], "target_rate_ci_high": ci[1],
            "lift": p / baseline if p is not None and baseline else None,
            "lift_ci_low": lift_ci[0], "lift_ci_high": lift_ci[1],
            "confidence": "low" if min(n, m) < cfg["low_confidence_support"] or min(h, n - h) < 5 else "exploratory",
            "validation_status": "evaluated" if n and m else "insufficient_holdout_support"}


ASSIGN_FIELDS = ["target", "trial_id", "label", "split", "cluster", "status"]
CLUSTER_FIELDS = ["target", "label", "cluster", "status", "training_n", "holdout_n", "centers", "field_statistics"]
RULE_FIELDS = ["target", "rule_id", "expression", "conditions", "threshold_source", "training_n", "training_support_n",
               "nonhit_sample_n", "nonhit_trigger_n", "nonhit_trigger_rate", "nonhit_trigger_ci_low", "nonhit_trigger_ci_high",
               "validation_n", "excluded_missing_n", "support_n", "support_rate", "support_ci_low", "support_ci_high",
               "target_hit_n", "target_rate", "target_not_rate", "target_not_rate_ci_low", "target_not_rate_ci_high",
               "nontrigger_n", "nontrigger_target_rate", "baseline_target_rate", "difference_pp", "difference_ci_low_pp",
               "difference_ci_high_pp", "target_rate_ci_low", "target_rate_ci_high", "lift", "lift_ci_low", "lift_ci_high",
               "confidence", "validation_status", "control_eligible_n", "control_trigger_n", "control_trigger_rate",
               "control_trigger_ci_low", "control_trigger_ci_high", "interpretation"]


def reports(output, trials, cfg):
    assignments, summaries, rules, target_summaries, preprocessing = [], [], [], [], {}
    controls = [r for r in trials if r.get("control") and r.get("features") and r["status"] in ("completed", "task_budget_exhausted")]
    for target in TARGETS:
        usable = [r for r in trials if not r.get("control") and r.get("actual_injected") and r.get("contract_valid") and
                  r["labels"][target]["valid"] and r.get("features")]
        invalid = Counter()
        usable_ids = {r["trial_id"] for r in usable}
        for r in trials:
            if r.get("control"):
                assignments.append({"target": target, "trial_id": r["trial_id"], "label": None,
                                    "split": "control", "cluster": None, "status": "excluded:explicit_control"})
                continue
            if r["trial_id"] not in usable_ids:
                reason = r.get("skip_reason") or r["labels"][target].get("missing_reason") or "missing_features_or_invalid_contract"
                invalid[reason] += 1
                assignments.append({"target": target, "trial_id": r["trial_id"], "label": None,
                                    "split": "excluded", "cluster": None, "status": "excluded:" + reason})
        hit_n = sum(r["labels"][target]["hit"] for r in usable)
        ci = wilson(hit_n, len(usable))
        target_summaries.append({"target": target, "valid_n": len(usable), "hit_n": hit_n, "miss_n": len(usable) - hit_n,
                                 "hit_rate": hit_n / len(usable) if usable else None, "ci_low": ci[0], "ci_high": ci[1],
                                 "invalid_n": sum(invalid.values()), "invalid_reasons": dict(invalid), "control_n": len(controls)})
        train, holdout = split(usable, target, cfg["analysis_seed"], cfg["holdout_fraction"])
        fields = sorted({f for r in train for f, v in r["features"]["fields"].items() if v["value"] is not None})
        raw = matrix(train, fields)
        med = np.nanmedian(raw, axis=0) if fields else np.array([])
        filled = np.where(np.isfinite(raw), raw, med)
        mean = filled.mean(axis=0) if len(train) else np.array([])
        scale = filled.std(axis=0) if len(train) else np.array([])
        scale[scale == 0] = 1
        preprocessing[target] = {"fields": fields, "median": med.tolist(), "mean": mean.tolist(), "scale": scale.tolist(),
                                 "training_ids": [r["trial_id"] for r in train], "holdout_ids": [r["trial_id"] for r in holdout]}
        cluster_centers = []
        for label in (False, True):
            indices = [i for i, r in enumerate(train) if r["labels"][target]["hit"] == label]
            tr = [train[i] for i in indices]
            va = [r for r in holdout if r["labels"][target]["hit"] == label]
            status = "ok"
            try:
                if len(tr) < cfg["min_cluster_samples"] or not fields:
                    raise ValueError("small_or_empty_group_or_no_features")
                x = (filled[indices] - mean) / scale
                centers, cl = kmeans(x, cfg["analysis_seed"])
                original = centers * scale + mean
                cluster_centers.extend(original)
                vx = matrix(va, fields)
                vf = (np.where(np.isfinite(vx), vx, med) - mean) / scale
                vcl = np.argmin(np.sum((vf[:, None] - centers[None, :]) ** 2, axis=2), axis=1)
                for rows0, cls, split_name in [(tr, cl, "train"), (va, vcl, "holdout")]:
                    assignments.extend({"target": target, "trial_id": r["trial_id"], "label": label,
                                        "split": split_name, "cluster": int(c), "status": status} for r, c in zip(rows0, cls))
                for k in (0, 1):
                    members = matrix([r for r, c in zip(tr, cl) if c == k], fields)
                    stats = {}
                    for col, name in enumerate(fields):
                        vals = members[np.isfinite(members[:, col]), col]
                        stats[name] = {"observed_n": len(vals), "missing_n": len(members) - len(vals),
                                       "mean": float(vals.mean()) if len(vals) else None,
                                       "std": float(vals.std()) if len(vals) else None,
                                       "q10_q50_q90": np.percentile(vals, [10, 50, 90]).tolist() if len(vals) else None}
                    summaries.append({"target": target, "label": label, "cluster": k, "status": status,
                                      "training_n": int((cl == k).sum()), "holdout_n": int((vcl == k).sum()),
                                      "centers": dict(zip(fields, original[k].tolist())), "field_statistics": stats})
            except ValueError as exc:
                status = "skipped:" + str(exc)
                summaries.append({"target": target, "label": label, "cluster": None, "status": status,
                                  "training_n": len(tr), "holdout_n": len(va), "centers": {}, "field_statistics": {}})
                assignments.extend({"target": target, "trial_id": r["trial_id"], "label": label, "split": sp,
                                    "cluster": None, "status": status} for rows0, sp in [(tr, "train"), (va, "holdout")] for r in rows0)
        y = np.array([r["labels"][target]["hit"] for r in train], dtype=bool)
        # Rules require actual cluster statistics; no invented rules for skipped groups.
        proposed = candidates(raw, y, cluster_centers, cfg) if cluster_centers and len(set(y)) == 2 else []
        for num, rule in enumerate(proposed, 1):
            mask, eligible = rule_mask(raw, rule)
            expression = " AND ".join("%s %s %.12g" % (fields[c], op, threshold) for c, op, threshold in rule)
            vx = matrix(holdout, fields)
            vy = np.array([r["labels"][target]["hit"] for r in holdout], dtype=bool)
            metrics = validation_metrics(vx, vy, rule, cfg, cfg["analysis_seed"] + num)
            cm, ce = rule_mask(matrix(controls, fields), rule)
            ch, cn = int(cm.sum()), int(ce.sum())
            control_ci = wilson(ch, cn)
            rules.append(dict(target=target, rule_id="%s_%02d" % (target, num), expression=expression,
                conditions=[{"field": fields[c], "op": op, "threshold": v} for c, op, v in rule],
                threshold_source="training quantiles and training cluster centers; train-only beam search <=3 conditions",
                training_n=int(eligible.sum()), training_support_n=int(mask.sum()), **metrics,
                control_eligible_n=cn, control_trigger_n=ch, control_trigger_rate=ch / cn if cn else None,
                control_trigger_ci_low=control_ci[0], control_trigger_ci_high=control_ci[1],
                interpretation="within_injection_association; controls_descriptive_only" if cn else "within_injection_association_only_no_controls"))
        if not proposed:
            rules.append({"target": target, "validation_status": "skipped:no_eligible_training_clusters_or_supported_rules",
                          "confidence": "low", "interpretation": "no_validated_rule"})
    write_csv(output / "cluster_assignments.csv", assignments, ASSIGN_FIELDS)
    write_csv(output / "cluster_summary.csv", summaries, CLUSTER_FIELDS)
    write_csv(output / "rules_per_target.csv", rules, RULE_FIELDS)
    write_csv(output / "summary_per_target.csv", target_summaries,
              ["target", "valid_n", "hit_n", "miss_n", "hit_rate", "ci_low", "ci_high", "invalid_n", "invalid_reasons", "control_n"])
    write_json(output / "preprocessing.json", preprocessing)
    return target_summaries


def export_trials(output, trials, cfg=None):
    cfg = cfg or default_config()
    schema = extract([], 0, 0.0, cfg["active_ids"][0], cfg)["fields"]
    fields = ["trial_id", "seed", "source_id", "status", "control", "actual_injected", "contract_valid", "skip_reason",
              "offset_s", "fraction", "generated_ns", "planned_release_ns", "released_ns", "release_complete_ns",
              "generated_ros_s", "publish_ros_s", "actual_task_elapsed_s", "measured_period_ns", "seq", "release_reason",
              "protocol_budget_ns", "planned_delay_ns", "actual_delay_ns", "result_path"]
    base_fields = list(fields)
    fields += [target + suffix for target in TARGETS for suffix in ("_hit", "_valid", "_missing_reason", "_evidence")]
    rows, features = [], []
    feature_names = sorted(set(schema) | {f for r in trials if r.get("features") for f in r["features"]["fields"]})
    feature_fields = ["trial_id", "control", "actual_injected", "cutoff_ns", "cutoff_ros_s", "source_id", "victim_id", "local_ids", "local_membership_complete"]
    feature_fields += [name + suffix for name in feature_names for suffix in ("", "__unit", "__missing", "__missing_reason", "__source", "__observed_ns", "__source_ros_s")]
    for trial in trials:
        row = {f: trial.get(f) for f in base_fields}
        for target in TARGETS:
            label = trial["labels"][target]
            row.update({target + "_hit": label["hit"], target + "_valid": label["valid"],
                        target + "_missing_reason": label["missing_reason"], target + "_evidence": label["evidence"]})
        rows.append(row)
        feat = trial.get("features") or {}
        fr = {"trial_id": trial["trial_id"], "control": trial.get("control", False),
              "actual_injected": trial.get("actual_injected", False)}
        fr.update({k: feat.get(k) for k in feature_fields[3:9]})
        for name in feature_names:
            v = feat.get("fields", {}).get(name, {})
            fr[name] = v.get("value")
            for suffix in ("unit", "missing", "missing_reason", "source", "observed_ns", "source_ros_s"):
                fallback = (schema.get(name, {}).get("unit") if suffix == "unit" else
                            "no_injection_cutoff_or_field_unavailable" if suffix == "missing_reason" else
                            True if suffix == "missing" else None)
                fr[name + "__" + suffix] = v.get(suffix, fallback)
        features.append(fr)
    write_csv(output / "trials.csv", rows, fields)
    write_csv(output / "trial_features.csv", features, feature_fields)
