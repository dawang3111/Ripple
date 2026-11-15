"""Post-hoc user-defined targets (2026-10-23); raw legacy labels are preserved.

Deadlock = a complete 5 s interval of zero cumulative path length for any UAV.
Delay = success AND completion time > twice successful baseline mean.
Drift = mismatching directed receive events >= twice baseline mean count,
with at least one mismatch required. No synthetic published future trajectory.
"""
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
from shapely.geometry import Point, Polygon, LineString

from .core import TARGETS
from .analysis import analyze, write_csv
from .__main__ import export_trials


def mismatch_events(trial):
    """Reconstruct sender state from ordered generation/release events, no lookahead.

    A delayed release never rolls truth back to its older generation time.
    Bootstrap seq=-1 is an initial position, not an actual TX.
    """
    truth={}; result=[]
    for event in trial.get('messages',[]):
        sender=event['sender_id']; t=event['generated_time']
        if event['event'] in ('enqueue','release'):
            if sender not in truth or t>=truth[sender][0]:truth[sender]=(t,event['payload'])
        elif event['event']=='receive':
            if sender not in truth:
                if event['seq']!=-1:raise ValueError('receive lacks preceding sender-state event')
                truth[sender]=(t,event['payload'])
            error=float(np.linalg.norm(np.array(truth[sender][1])-event['payload']))
            if error>0.0:
                result.append(dict(time_s=event['receive_time'],sender_id=sender,
                    receiver_id=event['receiver_id'],seq=event['seq'],offset_m=error,
                    truth_source_time=truth[sender][0],broadcast_source_time=t))
    return result


def calibrate(baseline):
    total=0; valid=0; success=[]; counts=[]; statuses=Counter()
    for r in baseline:
        total+=1; statuses[r['status']]+=1
        if r['status'] in ('error','rrt_failed') or not r.get('trace'):continue
        valid+=1; counts.append(len(mismatch_events(r)))
        if r['status']=='success':success.append(r['time_s'])
    return dict(baseline_trials=total,valid_motion_trials=valid,
        successful_trials=len(success),status_counts=dict(statuses),
        completion_mean_s=float(np.mean(success)) if success else None,
        delay_threshold_s=2*float(np.mean(success)) if success else None,
        mismatch_count_mean=float(np.mean(counts)) if counts else None,
        drift_count_threshold=max(1.,2*float(np.mean(counts))) if counts else None,
        deadlock_window_s=5.0,deadlock_cumulative_distance_m=0.0,
        mismatch_position_tolerance_m=0.0,
        count_unit='one mismatching directed planner receive event',
        baseline_zero_guard='require at least one mismatch; user approved',
        planning_failure_available=False,
        planning_failure_reason='no published timed per-UAV future trajectories in algorithm/logs')


def relabel(trial,calibration):
    result={k:v for k,v in trial.items() if k not in ('trace','messages','trajectory_setpoints','rrt_path')}
    result['legacy_labels']=trial['labels'];result['legacy_known']=trial['known']
    result['labels']={t:False for t in TARGETS}
    observed=bool(trial.get('trace')) and trial['status']!='error'
    result['known']={t:observed for t in TARGETS}
    result['known']['planning_failure']=False
    result['known']['timeout']=observed and calibration['completion_mean_s'] is not None
    result['known']['drift']=observed and calibration['drift_count_threshold'] is not None
    result['evidence']={t:[] for t in TARGETS}
    result['label_definitions_version']='user_2026_10_23_v1'
    def hit(target,time,ids,**detail):
        if result['labels'][target]:return
        result['labels'][target]=True
        nominal=trial['injection'].get('nominal_tx_time')
        relative='no_actual_injection' if not trial['injection']['actually_injected'] else ('pre_injection' if time<nominal else 'at_or_post_injection')
        result['evidence'][target]=[dict(time_s=time,uav_ids=ids,fsm='RUNNING',planning_state='LOCAL_APF',relative_to_injection=relative,**detail)]
    polygons=[Polygon(o) for o in trial.get('obstacles',[])]
    previous=None; stationary_since={}; window=calibration['deadlock_window_s']
    for state in trial.get('trace',[]):
        positions=state['positions'];time=state['time_s']; ids=list(positions)
        for i,a in enumerate(ids):
            for b in ids[i+1:]:
                margin=float(np.linalg.norm(np.array(positions[a])-positions[b]))-trial['params']['crash_uav_dist']
                if margin<=0:hit('drone_drone',time,[int(a),int(b)],safety_margin_m=margin)
            point=Point(positions[a])
            for oi,polygon in enumerate(polygons):
                if polygon.covers(point):hit('drone_obstacle',time,[int(a)],obstacle_index=oi,safety_margin_m=-polygon.boundary.distance(point))
            if previous is None:
                stationary_since[a]=time
            elif (time-previous['time_s']>1./trial['params']['ViconRate']+1e-9
                  or np.linalg.norm(np.array(positions[a])-previous['positions'][a])>0.0):
                stationary_since[a]=time
            if time-stationary_since[a]>=window:
                hit('deadlock',time,[int(a)],window_start_s=stationary_since[a],cumulative_distance_m=0.0)
        previous=state
    if result['known']['timeout'] and trial['status']=='success' and trial['time_s']>calibration['delay_threshold_s']:
        hit('timeout',trial['time_s'],[1],definition='successful mission delayed beyond 2X',
            completion_s=trial['time_s'],baseline_mean_s=calibration['completion_mean_s'])
    events=mismatch_events(trial)
    result['mismatch_count']=len(events)
    if result['known']['drift'] and len(events)>=calibration['drift_count_threshold']:
        event=events[math.ceil(calibration['drift_count_threshold'])-1]
        hit('drift',event['time_s'],[event['sender_id'],event['receiver_id']],
            definition='directed receive mismatch count >= 2B, positive-count guard',
            threshold=calibration['drift_count_threshold'],baseline_mean=calibration['mismatch_count_mean'],
            count_at_detection=math.ceil(calibration['drift_count_threshold']),message=event)
    # Diagnostic only: global geometric reference is NOT a published timed plan.
    reference=trial.get('trajectory_setpoints',[])
    result['unpublished_global_path_obstacle_intersection']=None
    if len(reference)>=2:
        path=LineString(reference)
        result['unpublished_global_path_obstacle_intersection']=any(path.intersects(p) for p in polygons)
    return result


def read_runs(directory):
    for p in sorted((Path(directory)/'trials').glob('trial_*.json')):
        r=json.loads(p.read_text());r['result_path']=str(p.resolve());yield r


def evaluate(injected_dir,baseline_dir,out,calibration,seed=1729):
    out=Path(out);out.mkdir(exist_ok=False)
    trials=[relabel(r,calibration) for r in read_runs(injected_dir)]
    controls=[relabel(r,calibration) for r in read_runs(baseline_dir)]
    export_trials(trials,out)
    analyze(trials,out,seed,controls)
    write_csv(out/'user_target_diagnostics.csv',[
        dict(episode=r['episode'],mismatch_count=r['mismatch_count'],
             unpublished_global_path_obstacle_intersection=r['unpublished_global_path_obstacle_intersection'])
        for r in trials],['episode','mismatch_count','unpublished_global_path_obstacle_intersection'])
    (out/'calibration.json').write_text(json.dumps(calibration,indent=2),encoding='utf-8')
    (out/'definitions.md').write_text('''# 用户定义（2026-10-23）

这是独立重标注目录，原始运行中的legacy标签未覆盖。
- drone_drone：中心距离减仓库0.1m阈值 <=0。
- drone_obstacle：原始障碍多边形内部或边界（点模型，无新机身半径）。
- deadlock：任何无人机完整连续5秒累计路程严格为0，未放宽数值容差。
- timeout列在本目录专指用户的“延迟”：任务成功且完成时间严格大于2X；X只从100轮baseline的成功轮次计算。它不是原来的任务预算耗尽。
- planning_failure：用户要求发布的未来轨迹发生碰撞，但算法没有发布逐机带时间未来轨迹，故unknown；RRT异常不能代替此标签。
- drift：每次有向广播读取，比较读取时源机状态与所读载荷；偏移次数>=2倍正常baseline平均次数。baseline为0时要求至少一次，用户已确认。

所有判定仅覆盖保存的执行前缀；碰撞终止后的潜在延迟或死锁不外推。完整5秒窗口才可命中死锁，提前正常完成不等于静止5秒。
未发布全局路径与障碍物相交仅为diagnostic，不冒充规划失败。

复用固定种子、分层70/30训练/留出流程；规则阈值仅来自训练簇统计/分位点，缺失特征不触发。
控制触发率只在有相应候选观测时刻且所需特征完整的baseline样本上报告，分母可能小于100。
正常与注入组共享算法参数、10秒预算、随机种子列表前100项；不会自动运行3000个控制。
本模式仍是随机计划时刻，不保证每轮实际注入。调整观察预算后实际注入比例可能低于上一批。
''',encoding='utf-8')
    # Avoid retaining a legacy-readable explanation that contradicts the new labels.
    explanation=(out/'analysis.md').read_text(encoding='utf-8')
    explanation=explanation.replace('死锁、漂移若未显式配置则unknown；协议timeout仓库无定义，任务timeout独立使用300 tick预算。',
        '本目录标签已按用户定义重新计算：死锁为连续5秒累计路程0；漂移为读取偏移计数；timeout列是成功用时>2X，详见definitions.md。规划失败因缺少已发布未来轨迹而unknown。')
    explanation=explanation.replace('5秒历史在默认3秒任务预算中缺失。','本批预算10秒，但任务可能早于5秒完成，因此5秒历史仍可能缺失。')
    (out/'analysis.md').write_text(explanation,encoding='utf-8')
    return dict(trials=len(trials),controls=len(controls),
                actually_injected=sum(r['injection']['actually_injected'] for r in trials),
                target_hits={t:sum(r['labels'][t] for r in trials if r['known'][t]) for t in TARGETS},
                target_observed={t:sum(r['known'][t] for r in trials) for t in TARGETS})
