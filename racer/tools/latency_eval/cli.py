import argparse
import gzip
import json
import os
import platform
import re
import signal
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

from . import __version__
from .analysis import export_trials, reports
from .core import (ROOT, TARGETS, VoxelMap, config, digest, injection_record,
                   read_events, trial_plan, unique_output, version, write_json)
from .features import extract
from .outcomes import classify


def stop_capture(process):
    for sig, timeout in ((signal.SIGINT, 25), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        if process.poll() is not None:
            return
        try:
            process.send_signal(sig)
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


def preflight_run(output_root, parser):
    if not re.fullmatch(r"[A-Za-z0-9_./-]+", str(Path(output_root).resolve())):
        parser.error("run output path must use letters/digits/underscore/dot/slash/hyphen: existing LKH uses unquoted paths")
    with tempfile.TemporaryDirectory(prefix="racer_eval_preflight_") as temporary:
        found = subprocess.check_output(["rospack", "find", "exploration_manager"], text=True,
                                        env=dict(os.environ, ROS_HOME=temporary)).strip()
    package = ROOT / "swarm_exploration/exploration_manager"
    if Path(found).resolve() != package:
        parser.error("source the current workspace devel/setup.bash; rospack resolves another checkout")
    binary = ROOT.parents[1] / "devel/lib/exploration_manager/exploration_node"
    dependencies = [package / p for p in ("src/evaluation_adapter.cpp", "src/fast_exploration_fsm.cpp",
                    "include/exploration_manager/tx_delay_gate.h", "include/exploration_manager/fast_exploration_fsm.h")]
    if not binary.exists() or binary.stat().st_mtime < max(p.stat().st_mtime for p in dependencies):
        parser.error("build exploration_node in this workspace before run")


def execute_capture(directory, cfg):
    child_env = dict(os.environ)
    child_env["PYTHONPATH"] = str(ROOT / "tools") + os.pathsep + child_env.get("PYTHONPATH", "")
    process = subprocess.Popen([sys.executable, "-m", "latency_eval.ros_capture", "--trial-dir", str(directory)],
                               env=child_env, start_new_session=True)
    try:
        process.wait(timeout=cfg["startup_budget_s"] + cfg["task_budget_s"] + 30)
    except subprocess.TimeoutExpired:
        stop_capture(process)
    except KeyboardInterrupt:
        stop_capture(process)
        raise


def missing_labels(reason):
    return {t: {"hit": False, "valid": False, "missing_reason": reason, "evidence": [], "count": 0} for t in TARGETS}


def evaluate_trial(directory, cfg):
    plan = json.loads((directory / "plan.json").read_text())
    capture_path = directory / "capture.json"
    capture = json.loads(capture_path.read_text()) if capture_path.exists() else {"status": "capture_missing", "end_ns": 0}
    result = {**plan, "status": capture["status"], "actual_injected": False, "contract_valid": False,
              "control": plan.get("control", False), "features": None, "labels": missing_labels("no_actual_injection"),
              "result_path": str(directory / "result.json")}
    events_path = directory / "events.jsonl"
    if not events_path.exists():
        events_path = directory / "events.jsonl.gz"
    if not events_path.exists():
        result["skip_reason"] = capture.get("failure_reason", "events_missing")
        return result
    try:
        events = read_events(events_path)
    except (ValueError, KeyError) as exc:
        result.update(status="invalid_event_log", skip_reason=str(exc), labels=missing_labels("invalid_event_log"))
        return result
    injection = injection_record(events, plan)
    result.update(injection)
    if plan.get("control"):
        result.update(skip_reason="explicit_uninjected_control", actual_injected=False, contract_valid=False)
        cutoff, cutoff_ros = capture.get("control_cutoff_ns"), capture.get("control_cutoff_ros_s")
    else:
        cutoff, cutoff_ros = injection.get("generated_ns"), injection.get("generated_ros_s")
    if cutoff is None or cutoff_ros is None or cutoff >= capture["end_ns"]:
        return result
    scene = directory / "scene_points.npy"
    if scene.exists():
        points = np.load(str(scene), allow_pickle=False)
    elif scene.with_suffix('.npy.gz').exists():
        with gzip.open(scene.with_suffix('.npy.gz'), 'rb') as f:
            points = np.load(f, allow_pickle=False)
    else:
        points = None
    geometry = VoxelMap(points, cfg["thresholds"]["map_resolution_m"]) if points is not None else None
    result["features"] = extract(events, cutoff, cutoff_ros, plan["source_id"], cfg, geometry)
    result["labels"] = classify(events, cutoff, capture["end_ns"], cfg, geometry, capture["status"])
    if not plan.get("control") and not injection["contract_valid"]:
        result["analysis_exclusion"] = injection["skip_reason"]
    return result


def report_text(path, summary, cfg):
    lines = ["# RACER 随机 TX 时延实验与离线关联分析", "",
        "注入通道为 DroneState，使用发送端 hook；其他通信通道与原有 CLI 默认行为不变。",
        "每轮最多一次注入；actual_injected 仅来自已执行 publish 的 release 证据。",
        "延迟由最近 8 个实际生成周期的中位数乘随机比例确定，并受 0.2 s 协议剩余预算及单轮剩余预算约束。",
        "下一条正常消息提前到达时先释放旧消息，保持源端 publish 顺序。非实时 ROS 阻塞无法保证硬截止期，越界轮次排除。",
        "记录的是源端 ROS publish 调用时间区间，不是无线发射、跨主机接收或算法采用时间。",
        "仅支持 Linux 同机 CLOCK_MONOTONIC/steady_clock、wall-clock 的隔离仿真；不连接真实设备或无线桥。", "",
        "碰撞标签使用显式配置的球体与静态占据体素几何代理；轻量仿真没有物理碰撞响应。",
        "预测避碰告警保留为事件，不冒充已发生的碰撞。",
        "死锁定义为活动 FSM 下连续窗口内低速、低位移且活动网格计数无下降；IDLE 和等待触发不计入。",
        "超时细分邻机缓存陈旧、规划调用耗时及配置的任务预算截止。规划 FAIL 单独记录。",
        "漂移定义为接收端缓存位置与当前仿真位置偏差持续超阈值；不代表物理定位漂移。",
        "每个 hit 都有证据；false 仅在 target_valid=true 时代表可观测负例。采样间事件可能漏检。", "",
        "特征按实际延迟消息生成时刻截断，要求观察者已经收到数据。未来 3 s 特征只求值注入前已发布的 B-spline；",
        "轨迹不足 3 s 不外推。任务进度是 active_grid/frontier 计数，没有伪造覆盖率、急停计数或地图时间。",
        "各字段附单位、观测时间范围、来源时间范围与缺失原因。接收端缓存误差含正常消息陈旧误差。", "",
        "每个目标独立稳定分层留出 25%；先划分再在训练集拟合中位数填补和标准缩放。全缺失训练字段排除。",
        "训练命中/未命中各做 KMeans(k=2)，固定 seed、10 次初始化、最多 200 次迭代；簇编号不表示风险等级。",
        "留出簇分配只是描述统计。空组、小组或不足两个不同向量会明确跳过。",
        "候选阈值只来自训练分位点/簇中心；训练集 beam search 限 1–3 条件，验证集不参与筛选。",
        "规则遇到缺失字段时不填补触发；从该规则验证分母排除，并报告 excluded_missing_n。",
        "support_rate 分母为留出集中条件字段齐全的样本。target_not_rate=1-target_rate。",
        "difference_pp 为触发组目标率减去未触发组目标率；lift 为触发组目标率除以该留出可用样本总体目标率。",
        "目标率用 95% Wilson CI，百分点差用两组 Wilson 端点组成的保守区间，lift 用固定种子的百分位 bootstrap。",
        "未触发组为空、无阳性总体率或 bootstrap 不足时对应指标留空。小支持度/少阳性或少阴性标记 low。",
        "这些是多个探索候选，未经多重筛选校正或独立复现；不能包装成已验证规律。", "",
        "控制数据仅在显式提供时纳入，独立报告规则触发率及 CI，不自动追加实验轮次。",
        "没有控制数据时只能称注入实验内命中/未命中关联，不能称相对正常运行的因果效果。",
        "控制样本自身也不能自动证明因果效应。随机计划可复现不代表 ROS 调度、地图噪声或整个轨迹位级确定。", "",
        "完整事件、调度计划、实际释放、参数快照与结果保存在各 trial 目录；分析不修改输入。", "", "## 每目标样本量", ""]
    for r in summary:
        lines.append("- %s：有效 %d，命中 %d，无效 %d。" % (r["target"], r["valid_n"], r["hit_n"], r["invalid_n"]))
    lines += ["", "## 阈值", "", "```json", json.dumps(cfg["thresholds"], indent=2), "```", "",
              "阈值来源及全部配置见 manifest.json/config.json；不随目标命中率调整。"]
    path.write_text("\n".join(lines) + "\n")
    from .readable import append_results
    append_results(path)


def manifest(cfg, args, inputs):
    changes = subprocess.check_output(["git", "-C", str(ROOT), "status", "--porcelain"], text=True)
    tracked = sorted((ROOT / 'tools/latency_eval').glob('*.py')) + [ROOT / 'tools/latency_eval/config.json']
    tracked += [ROOT / "swarm_exploration/exploration_manager" / name for name in
                ("src/fast_exploration_fsm.cpp", "src/evaluation_adapter.cpp", "include/exploration_manager/tx_delay_gate.h",
                 "launch/swarm_exploration.launch", "launch/single_drone_exploration.xml", "launch/single_drone_planner.xml")]
    return {"schema_version": 1, "tool_version": __version__, "git_commit": version(), "git_status": changes,
            "python": platform.python_version(), "numpy": np.__version__, "config": cfg, "parameters": vars(args),
            "inputs": [{"path": str(p), "sha256": digest(p)} for p in inputs],
            "implementation": [{"path": str(p), "sha256": digest(p)} for p in tracked],
            "injection_channel": "source DroneState only", "clock": "same-host CLOCK_MONOTONIC nanoseconds + ROS wall time",
            "scope": "isolated simulation only", "controls_automatically_added": False,
            "causal_claim": "none; within-injection associations; optional controls descriptive",
            "source_contract": "release publish completes before next normal DroneState publish begins; no cross-topic or RX ordering guarantee",
            "hard_deadline_guarantee": False}


def analyze_inputs(paths, cfg):
    trials, inputs = [], []
    seen = set()
    for source in paths:
        path = Path(source).resolve()
        directories = [path] if (path / "plan.json").exists() else sorted(p.parent for p in path.glob("trial_*/plan.json"))
        if not directories:
            raise ValueError("no recorded trial directories: " + str(path))
        for d in directories:
            if d in seen:
                raise ValueError("duplicate input trial: " + str(d))
            seen.add(d)
            result = evaluate_trial(d, cfg)
            # Avoid collisions when combining separate injected/control datasets.
            result["trial_id"] = result.get("dataset_id", d.parent.name) + "/" + result["trial_id"]
            result["source_trial"] = str(d)
            trials.append(result)
            for filename in ("plan.json", "capture.json", "events.jsonl", "events.jsonl.gz", "scene_points.npy", "scene_points.npy.gz", "ros_parameters.json", "archive.json"):
                p = d / filename
                if p.exists():
                    inputs.append(p)
    return trials, inputs


def main(argv=None, before_trial=None, resume_from=None):
    parser = argparse.ArgumentParser(description="Opt-in RACER random source TX simulation / offline analysis")
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("plan", "run", "analyze"):
        p = sub.add_parser(command)
        p.add_argument("--output-root", default="racer_eval_results")
        p.add_argument("--config")
        if command != "analyze":
            p.add_argument("--trials", type=int, default=3000)
            p.add_argument("--seed", type=int, default=20261103)
            p.add_argument("--map", default=str(ROOT / "uav_simulator/map_generator/resource/pillar.pcd"))
            p.add_argument("--control", action="store_true", help="explicitly run uninjected controls instead of injections; never doubles count")
        else:
            p.add_argument("--input", nargs="+", required=True, help="recorded run/trial directories; may include explicit controls")
    args = parser.parse_args(argv)
    inherited_inputs = []
    inherited_config = None
    if args.command == "analyze" and not args.config:
        for value in args.input:
            p = Path(value).resolve()
            f = p / "config.json" if (p / "config.json").exists() else p.parent / "config.json"
            if not f.exists():
                parser.error("input has no config.json; provide --config explicitly")
            candidate = config(f)
            inherited_inputs.append(f)
            if inherited_config is not None and inherited_config != candidate:
                parser.error("input configurations differ; choose an explicit analysis --config")
            inherited_config = candidate
    cfg = inherited_config or config(args.config)
    inputs = [Path(args.config).resolve()] if args.config else list(dict.fromkeys(inherited_inputs))
    if args.command == "analyze":
        trials, event_inputs = analyze_inputs(args.input, cfg)
        inputs += event_inputs
        output = unique_output(args.output_root, "analysis")
    else:
        if args.trials < 1:
            parser.error("--trials must be positive")
        map_path = Path(args.map).resolve()
        if not map_path.is_file():
            parser.error("map file not found")
        inputs.append(map_path)
        if args.command == "run":
            preflight_run(args.output_root, parser)
        output = unique_output(args.output_root, args.command)
        write_json(output / "config.json", cfg)
        trials = []
        previous = Path(resume_from).resolve() if resume_from else None
        if previous and args.command != "run":
            parser.error("resume is only supported for run")
        if previous:
            inputs.append(previous / "manifest.json")
            if json.loads((previous / "config.json").read_text()) != cfg:
                parser.error("resume config differs")
        for index in range(1, args.trials + 1):
            plan = {**trial_plan(index, args.seed, cfg), "control": args.control, "map_path": str(map_path), "dataset_id": output.name}
            directory = output / plan["trial_id"]
            old = previous / plan["trial_id"] if previous else None
            if old:
                recorded = json.loads((old / "plan.json").read_text())
                if {k: v for k, v in recorded.items() if k != "dataset_id"} != {k: v for k, v in plan.items() if k != "dataset_id"}:
                    parser.error("resume plan differs: " + str(old))
                plan = recorded
                if (old / "result.json").exists():
                    directory.symlink_to(old.resolve(), target_is_directory=True)
                    trials.append(json.loads((old / "result.json").read_text()))
                    continue
                if any((old / name).exists() for name in ("capture.json", "events.jsonl", "events.jsonl.gz")):
                    parser.error("partial capture requires explicit review: " + str(old))
            directory.mkdir()
            write_json(directory / "plan.json", plan)
            if args.command == "plan":
                trials.append({**plan, "status": "planned_not_run", "actual_injected": False, "contract_valid": False,
                               "skip_reason": "plan_only", "labels": missing_labels("plan_only"), "features": None,
                               "result_path": str(directory / "plan.json")})
    write_json(output / "config.json", cfg)
    write_json(output / "manifest.json", manifest(cfg, args, inputs))
    if args.command == "analyze":
        for index, result in enumerate(trials, 1):
            directory = output / ("analysis_trial_%04d" % index)
            directory.mkdir()
            result["result_path"] = str(directory / "result.json")
            write_json(directory / "result.json", result)
    cancelled = False
    if args.command == "run":
        export_trials(output, trials, cfg)
        try:
            for index, directory in enumerate(sorted(p.parent for p in output.glob("trial_*/plan.json")), 1):
                if (directory / "result.json").exists():
                    continue
                if before_trial is not None and not before_trial(output, directory):
                    break
                execute_capture(directory, cfg)
                result = evaluate_trial(directory, cfg)
                write_json(directory / "result.json", result)
                trials.append(result)
                export_trials(output, trials, cfg)  # resumable evidence even after interruption
                print("%d/%d %s actual_injected=%s" % (index, args.trials, result["status"], result["actual_injected"]), flush=True)
                if index >= 3 and all(r["status"] not in ("completed", "task_budget_exhausted") for r in trials[-3:]):
                    print("Stopping after three consecutive capture failures; remaining plans are not injected.", file=sys.stderr)
                    break
        except KeyboardInterrupt:
            cancelled = True
        done = {r["trial_id"] for r in trials}
        for directory in sorted(p.parent for p in output.glob("trial_*/plan.json")):
            plan = json.loads((directory / "plan.json").read_text())
            if plan["trial_id"] not in done:
                if (directory / "events.jsonl").exists():
                    result = evaluate_trial(directory, cfg)
                    write_json(directory / "result.json", result)
                    trials.append(result)
                else:
                    trials.append({**plan, "status": "not_run", "actual_injected": False, "contract_valid": False,
                        "skip_reason": "run_interrupted_or_repeated_capture_failure", "labels": missing_labels("not_run"),
                        "features": None, "result_path": str(directory / "plan.json")})
    export_trials(output, trials, cfg)
    summary = reports(output, trials, cfg)
    report_text(output / "analysis.md", summary, cfg)
    print(output)
    if cancelled:
        return 130
    if args.command == "run" and any(r["status"] not in ("completed", "task_budget_exhausted") for r in trials):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
