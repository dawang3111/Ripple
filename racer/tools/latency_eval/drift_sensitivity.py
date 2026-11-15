"""Exploratory post-hoc drift threshold sweep. No simulation and no input mutation."""
import argparse
import csv
import gzip
import hashlib
import json
import math
import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from .core import ACTIVE, TARGETS, unique_output, write_json

DISTANCES = [round(.5 - i * .025, 3) for i in range(20)]
DURATIONS = [1., .9, .8, .7, .6, .5, .4, .3, .25, .2, .15, .1, .075, .05, .025]


def extract_pairs(events, start, end, cfg):
    """Same alignment, active-state and pair reset semantics as outcomes.classify."""
    recent, epoch, pairs = {}, {i: 0 for i in cfg['active_ids']}, {}
    tolerance = int(cfg['alignment_tolerance_s'] * 1e9)
    for e in sorted((e for e in events if e['kind'] == 'snapshot'), key=lambda e: e.get('mono_ns', e['observed_ns'])):
        t = e.get('mono_ns', e['observed_ns'])
        i = e.get('id')
        if i not in epoch:
            continue
        recent[i] = e
        if not start <= t <= end or not 0 <= e['ros_s'] - e.get('position_stamp_s', e['ros_s']) <= cfg['alignment_tolerance_s']:
            continue
        aligned = {j: r for j, r in recent.items() if 0 <= t - r.get('mono_ns', r['observed_ns']) <= tolerance and
                   0 <= e['ros_s'] - r.get('position_stamp_s', r['ros_s']) <= cfg['alignment_tolerance_s']}
        if e.get('fsm') not in ACTIVE:
            epoch[i] += 1
            continue
        for c in e.get('received_states', []):
            j = c['id']
            if j not in epoch or j not in aligned or aligned[j].get('fsm') not in ACTIVE:
                continue
            error = math.sqrt(sum((float(a)-float(b))**2 for a,b in zip(c['position'], aligned[j]['position'])))
            if not math.isfinite(error):
                raise ValueError('nonfinite cache error')
            key = (i,j)
            pairs.setdefault(key, []).append(dict(time_ns=t, error_m=error, reset=(epoch[i],epoch[j]),
                ros_s=e['ros_s'], ids=[i,j], fsm={str(i):e.get('fsm'),str(j):aligned[j].get('fsm')},
                plan_result={str(i):e.get('plan_result'),str(j):aligned[j].get('plan_result')},
                position_measurement_ros_s={str(j):aligned[j].get('position_stamp_s',aligned[j]['ros_s'])}))
    return pairs


def sweep_pairs(pairs, cfg, distances=DISTANCES, durations=DURATIONS):
    """Return first evidence per config; runs are broken by reset, false or gap."""
    result = {}
    gap = int(cfg['max_sample_gap_s'] * 1e9)
    for records in pairs.values():
        times = np.asarray([e['time_ns'] for e in records], dtype=np.int64)
        errors = np.asarray([e['error_m'] for e in records])
        reset = np.asarray([e['reset'] for e in records],dtype=np.int64)
        for d in distances:
            indices = np.flatnonzero(errors > d)
            if not len(indices):
                continue
            split = np.flatnonzero((np.diff(indices) != 1) | (np.diff(times[indices]) > gap) |
                                  np.any(reset[indices[1:]] != reset[indices[:-1]],axis=1)) + 1
            for run in np.split(indices, split):
                start = times[run[0]]
                elapsed = times[run] - start
                span = elapsed[-1]
                for duration in durations:
                    cutoff = duration * 1e9  # same float comparison as outcomes.classify
                    if span < cutoff:
                        continue
                    idx = run[int(np.searchsorted(elapsed,cutoff))]
                    key = f'{d:.3f}/{duration:.3f}'
                    old = result.get(key)
                    if old is None or times[idx] < old['time_ns']:
                        e = records[idx]
                        result[key] = {k:v for k,v in e.items() if k != 'reset'}
                        result[key].update(begin_ns=int(start), sustained_s=float(elapsed[int(np.searchsorted(elapsed,cutoff))]/1e9),
                            distance_threshold_m=d,duration_threshold_s=duration,definition='receiver_cache_position_error_sustained')
    return result


def process_one(args):
    path, cfg, out = args
    path, out = Path(path), Path(out)
    trial = json.loads(path.read_text())
    target = out / (trial['trial_id'] + '.json')
    if not trial.get('actual_injected') or not trial.get('contract_valid'):
        record = {'trial_id':trial['trial_id'],'eligible':False,'skip_reason':trial.get('skip_reason','no_valid_actual_injection'),'first_evidence':{}}
    else:
        events = path.parent/'events.jsonl.gz'
        if not events.exists():
            events = path.parent/'events.jsonl'
        snapshots=[]
        with (gzip.open(events,'rt') if events.suffix=='.gz' else events.open()) as stream:
            for line in stream:
                # Fast exclusion only; JSON is still parsed/checked for all snapshots.
                if '"snapshot"' not in line:
                    continue
                event=json.loads(line)
                if event.get('kind')=='snapshot':snapshots.append(event)
        capture = json.loads((path.parent/'capture.json').read_text())
        pairs=extract_pairs(snapshots,trial['generated_ns'],capture['end_ns'],cfg)
        evidence=sweep_pairs(pairs,cfg)
        # Exact original-grid consistency check before accepting any new labels.
        original_key=f"{cfg['thresholds']['drift_distance_m']:.3f}/{cfg['thresholds']['drift_duration_s']:.3f}"
        if (original_key in evidence) != trial['labels']['drift']['hit']:
            raise ValueError('original drift replay mismatch: '+str(path))
        record=dict(trial_id=trial['trial_id'],eligible=True,original_drift_valid=trial['labels']['drift']['valid'],
            original_drift_hit=trial['labels']['drift']['hit'],first_evidence=evidence,result_path=str(path),events_path=str(events))
    write_json(target,record)
    return record


def csv_write(path, rows):
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True)
    p.add_argument('--output-root',required=True)
    p.add_argument('--workers',type=int,default=2)
    args=p.parse_args(argv)
    os.nice(10)
    run=Path(args.input).resolve()
    output=unique_output(args.output_root,'drift_sensitivity')
    (output/'trial_sweeps').mkdir()
    cfg=json.loads((run/'config.json').read_text())
    files=sorted(run.glob('trial_*/result.json'))
    manifest=dict(mode='post_hoc_exploratory_threshold_calibration',input_run=str(run),planned_target_hit_range=[200,300],
        selection_rule='among 200..300 prefer closest to 250, then least relaxation (largest distance_ratio * duration_ratio); otherwise nearest to 250, no fabricated hits',
        distances_m=DISTANCES,durations_s=DURATIONS,original_config=cfg,workers=args.workers,
        control_n=0,independent_validation=False,causal_claim=False,other_five_targets_unchanged=True,
        label_validity='eligible AND (new evidence hit OR original target-valid negative); previously unknown negatives remain unknown',
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        inputs=[dict(path=str(f),sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in files])
    write_json(output/'manifest.json',manifest)
    records=[]
    print(output,flush=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for i,record in enumerate(pool.map(process_one,[(str(f),cfg,str(output/'trial_sweeps')) for f in files]),1):
            records.append(record)
            if i%50==0 or i==len(files):
                write_json(output/'progress.json',dict(state='replaying',completed=i,total=len(files),output=str(output)))
                print(f'{i}/{len(files)}',flush=True)
    sweep=[]
    for d in DISTANCES:
        for duration in DURATIONS:
            key=f'{d:.3f}/{duration:.3f}'
            hits=sum(r['eligible'] and key in r['first_evidence'] for r in records)
            valid=sum(r['eligible'] and (key in r['first_evidence'] or r['original_drift_valid']) for r in records)
            sweep.append(dict(distance_m=d,duration_s=duration,hit_n=hits,valid_n=valid,miss_n=valid-hits,
                              hit_rate=hits/valid if valid else '',invalid_n=len(records)-valid))
    csv_write(output/'drift_threshold_sweep.csv',sweep)
    candidates=[r for r in sweep if 200<=r['hit_n']<=300] or sweep
    chosen=min(candidates,key=lambda r:(abs(r['hit_n']-250),-(r['distance_m']/.5)*(r['duration_s']/1.),-r['distance_m'],-r['duration_s']))
    write_json(output/'selected_threshold.json',chosen)
    key=f"{chosen['distance_m']:.3f}/{chosen['duration_s']:.3f}"
    selected=[]
    for r in records:
        e=r['first_evidence'].get(key)
        selected.append(dict(trial_id=r['trial_id'],eligible=r['eligible'],drift_hit=bool(e),
            drift_valid=bool(r['eligible'] and (e or r.get('original_drift_valid'))),
            original_drift_valid=r.get('original_drift_valid',False),evidence=json.dumps(e,ensure_ascii=False) if e else '',
            missing_reason='' if r['eligible'] and (e or r.get('original_drift_valid')) else r.get('skip_reason','original_observation_incomplete')))
    csv_write(output/'drift_labels_selected.csv',selected)
    with (run/'summary_per_target.csv').open() as f:original=list(csv.DictReader(f))
    summary=[]
    for row in original:
        if row['target']=='drift':
            n=chosen['valid_n'];k=chosen['hit_n'];rate=k/n if n else 0;z=1.959963984540054
            center=(rate+z*z/(2*n))/(1+z*z/n);half=z*math.sqrt(rate*(1-rate)/n+z*z/(4*n*n))/(1+z*z/n)
            summary.append(dict(target='drift',hit_n=k,valid_n=n,miss_n=n-k,hit_rate=rate,ci_low=center-half,ci_high=center+half,
                                invalid_n=len(records)-n,mode='post_hoc_exploratory',distance_m=chosen['distance_m'],duration_s=chosen['duration_s']))
        else:
            summary.append(dict(target=row['target'],hit_n=int(row['hit_n']),valid_n=int(row['valid_n']),miss_n=int(row['miss_n']),
                hit_rate=float(row['hit_rate']),ci_low=float(row['ci_low']),ci_high=float(row['ci_high']),invalid_n=int(row['invalid_n']),
                mode='original_unchanged',distance_m='',duration_s=''))
    csv_write(output/'summary_per_target_exploratory.csv',summary)
    write_json(output/'progress.json',dict(state='finished',completed=len(files),total=len(files),selected=chosen,output=str(output)))
    lines=['# 六目标命中率：漂移阈值事后敏感性分析','','其余五目标保持原有统计。漂移以完整数据命中数筛选阈值，属于探索性标定；不是独立验证，不作因果结论。',
        '原始漂移阈值为 >0.5 m 持续 >=1 s，原结果 0/1422；原文件未修改。',
        f"本次选择误差 >{chosen['distance_m']} m 持续 >={chosen['duration_s']} s，命中 {chosen['hit_n']}，有效 {chosen['valid_n']}。",
        '接收缓存与当前仿真位置误差；保留原始 active FSM、0.15 s 对齐、0.25 s 连续样本间隔约束，不混用轨迹跟踪偏差。',
        '重放原始阈值逐轮与已有 drift.hit 核对；不一致立即失败。正例可凭明确证据成立，未知负例不变成未命中。',
        '阈值扫描和筛选使用全部数据，置信区间仅为描述性 Wilson 区间，未考虑事后筛选；不能解释为独立验证精度。',
        '没有重新运行仿真，没有追加控制，没有重新计算聚类/候选规则；旧漂移规则/聚类不能套用于新标签。','',
        '| target | hit_n | valid_n | hit_rate |','| --- | --- | --- | --- |']
    lines += [f"| {r['target']} | {r['hit_n']} | {r['valid_n']} | {r['hit_rate']:.6%} |" for r in summary]
    (output/'README.md').write_text('\n'.join(lines)+'\n')
    print('SELECTED',json.dumps(chosen),flush=True)
    print('DONE',output,flush=True)
    return 0

if __name__=='__main__':
    raise SystemExit(main())
