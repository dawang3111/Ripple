"""One detached command runs all six frozen rule groups, 100 trials per target."""
import argparse
import csv
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import random
import subprocess
import sys
import time
import traceback
import uuid
from datetime import datetime,timezone

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
TARGETS=('drone_drone','drone_obstacle','deadlock','delay','planning','drift')
FIELDS={'current_min_pair_distance','current_min_obstacle_distance','cluster_mean_speed','source_speed',
        'cluster_mean_acceleration','source_acceleration','cluster_max_course_angle','source_turn_angle',
        'cluster_max_target_offset','source_target_offset','leader_goal_distance','setpoint_progress',
        'elapsed_task_time','source_truth_cached_broadcast_error'}


def dump(path,value):
    path=Path(path);temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    temporary.replace(path)


def load_settings(path):
    settings=json.loads(Path(path).read_text())
    groups=settings['groups']
    if len(groups)!=6 or {g['target'] for g in groups}!=set(TARGETS):raise ValueError('exactly one group per six targets required')
    if len({g['group_id'] for g in groups})!=6:raise ValueError('duplicate group IDs')
    if not math.isfinite(settings['baseline_mean_s']) or settings['baseline_mean_s']<=0:raise ValueError('positive finite frozen baseline mean required')
    if set(settings['thresholds'])!=set(TARGETS):raise ValueError('six frozen outcome thresholds required')
    for meta in settings['thresholds'].values():
        if meta['direction'] not in ('low','high') or not math.isfinite(meta['value']):raise ValueError('invalid outcome threshold')
    for group in groups:
        if group['logic']!='OR' or not group['rules']:raise ValueError('nonempty OR rule group required')
        for rule in group['rules']:
            if not rule['conditions']:raise ValueError('empty rule')
            for atom in rule['conditions']:
                if atom['field'] not in FIELDS or atom['op'] not in ('<=','>') or not math.isfinite(atom['threshold']):
                    raise ValueError('only finite, observable pre-injection rule conditions allowed')
    return settings


def seed_plan(master,count,excluded=()):
    rng=random.Random(master);seen=set(excluded);seeds=[]
    while len(seeds)<count:
        seed=rng.randrange(2**31-1)
        if seed not in seen:seen.add(seed);seeds.append(seed)
    return seeds


def preflight():
    import psutil
    try:
        excluded={os.getpid(),*[p.pid for p in psutil.Process().parents()]}
        processes=list(psutil.process_iter(['pid','name','cmdline']))
    except (PermissionError,psutil.AccessDenied) as exc:
        raise RuntimeError('Cannot inspect experiment conflicts: process-list permission denied; run this command from a terminal with process visibility') from exc
    conflicts=[]
    for proc in processes:
        try:
            if proc.pid in excluded or 'python' not in (proc.info['name'] or '').lower():continue
            arguments=proc.info['cmdline'] or []
            module=__package__+'.rule_campaign'
            if module in arguments:
                index=arguments.index(module)
                if index+1<len(arguments) and arguments[index+1]=='status':continue
            cmd=' '.join(arguments)
            if any(word in cmd for word in ('latency_eval','run_campaign.py','fuzz.py')):
                conflicts.append(dict(pid=proc.pid,command=cmd))
        except (psutil.AccessDenied,psutil.NoSuchProcess):continue
    if conflicts:raise RuntimeError('Other experiment process conflict; no process stopped: '+json.dumps(conflicts,ensure_ascii=False))
    return dict(other_experiments=[],available_memory_bytes=psutil.virtual_memory().available)


def write_csv(path,rows):
    if not rows:return
    temporary=path.with_suffix('.csv.tmp')
    with temporary.open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader()
        for row in rows:writer.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v for k,v in row.items()})
    temporary.replace(path)


def aggregate(rows,settings,episodes):
    from .analysis import wilson
    summary=[];cross=[]
    for group in settings['groups']:
        group_rows=[r for r in rows if r['target']==group['target']]
        actual=[r for r in group_rows if r['actually_injected']]
        for target in TARGETS:
            valid=[r for r in actual if r['approximate_labels'][target] is not None]
            hits=sum(r['approximate_labels'][target] for r in valid)
            cross.append(dict(group_id=group['group_id'],attack_target=group['target'],outcome_target=target,
                              valid_n=len(valid),hits=hits,rate=hits/len(valid) if valid else None))
        target=group['target'];valid=[r for r in actual if r['approximate_labels'][target] is not None]
        strict=[r for r in actual if r['strict_labels'][target] is not None]
        hits=sum(r['approximate_labels'][target] for r in valid)
        reasons={}
        for r in group_rows:
            reason=r['skip_reason'] if not r['actually_injected'] else r['metric_missing_reason']
            if reason:reasons[reason]=reasons.get(reason,0)+1
        summary.append(dict(target=target,name=group['name'],group_id=group['group_id'],planned=episodes,completed=len(group_rows),
            complete=len(group_rows)==episodes,rule_triggered=sum(r['rule_triggered'] for r in group_rows),
            actually_injected=len(actual),skipped=len(group_rows)-len(actual),errors=sum(r['status'] in ('error','rrt_failed') for r in group_rows),
            valid_n=len(valid),approximate_hits=hits,approximate_rate=hits/len(valid) if valid else None,
            approximate_rate_ci95=wilson(hits,len(valid)),hits_per_planned=hits/episodes,
            strict_known_n=len(strict),strict_hits=sum(r['strict_labels'][target] for r in strict) if strict else None,
            invalid_reasons=reasons,caveat=group['caveat']))
    return summary,cross


def run_trials(out,settings,episodes,master,max_ticks,rrt_iterations,sim_factory=None):
    import numpy as np
    from .core import Config
    from .rule_policy import RuleGate
    from .campaign_metrics import evaluate
    if sim_factory is None:
        from .simulation import TxSimulation
        sim_factory=TxSimulation
    config=Config(max_ticks=max_ticks,rrt_iterations=rrt_iterations)
    seeds=seed_plan(master,episodes*6,settings.get('excluded_trial_seeds',[]));rows=[]
    manifest=json.loads((out/'manifest.json').read_text())
    manifest.update(status='running',trial_seeds=seeds,config=vars(config),pid=os.getpid(),python=platform.python_version())
    manifest['versions']={p:importlib.metadata.version(p) for p in ('numpy','matplotlib','scipy','shapely','svgpath2mpl')}
    dump(out/'manifest.json',manifest)
    def checkpoint(state):
        summary,cross=aggregate(rows,settings,episodes)
        write_csv(out/'trials.csv',rows);write_csv(out/'summary_per_target.csv',summary);write_csv(out/'cross_target_summary.csv',cross)
        progress=dict(status=state,planned=episodes*6,completed=len(rows),groups_complete=sum(s['complete'] for s in summary),
            all_target_groups_complete=all(s['complete'] for s in summary),targets=summary,updated_utc=datetime.now(timezone.utc).isoformat())
        dump(out/'progress.json',progress)
        return summary
    checkpoint('running')
    for group_index,group in enumerate(settings['groups']):
        directory=out/'groups'/group['target']/'trials';directory.mkdir(parents=True,exist_ok=False)
        for index in range(episodes):
            episode=group_index*episodes+index+1;seed=seeds[episode-1]
            # No actual completion time or future trajectory is consulted for gate sampling.
            rng=np.random.RandomState(seed ^ 0x71B3)
            upper=max(2,min(max_ticks-1,int(.9*settings['baseline_mean_s']*100)))
            start_tick=int(rng.randint(2,upper+1))
            sender=int(rng.randint(1,5));gate=RuleGate(group,sender,start_tick)
            sim=sim_factory(episode,out,seed,config,injection_policy=gate)
            if sim.params.ViconRate!=100 or sim.params.num_robots!=4:raise ValueError('frozen campaign requires the four-robot 100Hz adapter')
            sim.plan['sender_id']=sender
            trial=sim.start();trial['is_control']=False
            evaluation=evaluate(trial,settings)
            if trial['status'] in ('error','rrt_failed'):
                evaluation['approximate_labels']={t:None for t in TARGETS};evaluation['strict_labels']={t:None for t in TARGETS}
                for m in evaluation['metrics'].values():m['missing_reason']='execution_'+trial['status']
            trial['campaign_evaluation']=evaluation;trial['campaign_group_id']=group['group_id']
            path=directory/('trial_%06d.json'%(index+1));trial['result_path']=str(path.resolve())
            with path.open('x',encoding='utf-8') as f:json.dump(trial,f,ensure_ascii=False,allow_nan=False)
            inj=trial['injection'];selected=trial['rule_gate']['selected'];target=group['target']
            row=dict(target=target,group_id=group['group_id'],group_trial=index+1,episode=episode,seed=seed,status=trial['status'],note=trial['note'],
                gate_start_tick=start_tick,sender_id=sender,rule_triggered=selected is not None,
                matched_rules=selected['matched_rule_ids'] if selected else [],actually_injected=inj['actually_injected'],
                skip_reason=inj['skip_reason'],nominal_tx_time=inj.get('nominal_tx_time'),actual_release_time=inj.get('actual_release_time'),
                planned_delay_s=inj.get('planned_delay_s'),actual_delay_s=inj.get('actual_delay_s'),
                approximate_labels=evaluation['approximate_labels'],strict_labels=evaluation['strict_labels'],evidence=evaluation['metrics'],
                metric_missing_reason=evaluation['metrics'][target]['missing_reason'],result_path=str(path.resolve()))
            for t in TARGETS:
                row[t+'_approximate_hit']=evaluation['approximate_labels'][t]
                row[t+'_strict_hit']=evaluation['strict_labels'][t]
            rows.append(row);summary=checkpoint('running')
            print(f"[{target}] {index+1}/{episodes} complete; overall {len(rows)}/{episodes*6}; actual={summary[group_index]['actually_injected']} hits={summary[group_index]['approximate_hits']}",flush=True)
    summary=checkpoint('complete')
    manifest.update(status='complete',completed=len(rows),all_target_groups_complete=True,finished_utc=datetime.now(timezone.utc).isoformat())
    dump(out/'manifest.json',manifest)
    text=['# 六目标规则组隔离仿真结果','',f'计划 {episodes*6} 轮，完成 {len(rows)} 轮；六组均完成。','']
    for s in summary:
        rate='NA' if s['approximate_rate'] is None else f"{s['approximate_rate']:.2%}"
        text.append(f"- {s['name']}：完成{s['completed']}/{s['planned']}，规则触发{s['rule_triggered']}，实际注入{s['actually_injected']}，跳过{s['skipped']}；有效{s['valid_n']}，近似命中{s['approximate_hits']}，发生率{rate}；严格命中{s['strict_hits']}（已知{s['strict_known_n']}）。")
    text+=['','每轮最多一次TX尝试；未触发及注入未释放均保留，不补跑。发生率为新规则引导实验内的条件比例，没有自动增加随机或无注入控制，不能据此声称因果提升。',
           '死锁和规划近似标签仍分别是0.2秒运动量与未发布参考路径净空；严格标签单列，已发布未来轨迹碰撞不可判定。近似阈值沿用历史筛选尺度，没有重新调参。',
           'summary_per_target.csv为六组主表；cross_target_summary.csv保留每攻击组的六目标多标签计数；trials.csv及groups/*/trials保留跳过原因、特征、消息关联和证据。']
    (out/'report.md').write_text('\n'.join(text)+'\n',encoding='utf-8')
    print('ALL_TARGET_GROUPS_COMPLETE',out,flush=True)


def start(args):
    settings=load_settings(args.rules);check=preflight()
    parent=Path(args.output_parent).resolve();parent.mkdir(parents=True,exist_ok=True)
    out=parent/('rule_campaign_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex)
    out.mkdir(exist_ok=False)
    dump(out/'rules_frozen.json',settings)
    source_files=list(HERE.glob('*.py'))+[HERE/'rule_groups.json',HERE.parent/'rrt.py',HERE.parent/'tools.py',HERE.parent/'potential_fields.py',HERE.parent/'Adaptive_swarm_collsion_findradius_obstacle_inflation.py']
    dump(out/'manifest.json',dict(status='starting',episodes_per_target=args.episodes,planned=6*args.episodes,master_seed=args.seed,
        preflight=check,rules_source=str(Path(args.rules).resolve()),source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files},
        gate_policy=settings['gate_start_policy'],new_control_trials=0,max_ticks=args.max_ticks,rrt_iterations=args.rrt_iterations))
    cache=out/'runtime_cache';cache.mkdir()
    env=os.environ.copy();env.update(MPLBACKEND='Agg',MPLCONFIGDIR=str(cache),PYTHONDONTWRITEBYTECODE='1',
        OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',VECLIB_MAXIMUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
    cmd=[sys.executable,'-u','-m',__package__+'.rule_campaign','worker','--output',str(out)]
    with (out/'console.log').open('x') as log:
        child=subprocess.Popen(cmd,cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (out/'background.pid').write_text(str(child.pid)+'\n')
    print('OUTPUT_DIRECTORY='+str(out));print('BACKGROUND_PID='+str(child.pid))
    print('All six targets queued: '+str(args.episodes)+' each, '+str(args.episodes*6)+' total.')
    return out


def worker(args):
    import fcntl
    out=Path(args.output).resolve()
    try:
        # Persistent inode, process-lifetime lock; never delete another process's lock.
        with (out.parent/'.rule_campaign.lock').open('a+') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            preflight();os.nice(10)
            settings=load_settings(out/'rules_frozen.json');manifest=json.loads((out/'manifest.json').read_text())
            run_trials(out,settings,manifest['episodes_per_target'],manifest['master_seed'],manifest['max_ticks'],manifest['rrt_iterations'])
    except BaseException as exc:
        state=json.loads((out/'progress.json').read_text()) if (out/'progress.json').exists() else dict(completed=0,planned=0,all_target_groups_complete=False)
        state.update(status='failed',error=type(exc).__name__+': '+str(exc));dump(out/'progress.json',state)
        manifest=json.loads((out/'manifest.json').read_text());manifest.update(status='failed',error=state['error']);dump(out/'manifest.json',manifest)
        traceback.print_exc();return 1
    return 0


def status(args):
    out=Path(args.output);p=json.loads((out/'progress.json').read_text()) if (out/'progress.json').exists() else json.loads((out/'manifest.json').read_text())
    print('Output:',out.resolve());print('Status:',p['status'])
    for s in p.get('targets',[]):print(f"{s['target']}: {s['completed']}/{s['planned']} actual={s['actually_injected']} skipped={s['skipped']} valid={s['valid_n']} hits={s['approximate_hits']} complete={s['complete']}")
    if p.get('all_target_groups_complete'):print('ALL_TARGET_GROUPS_COMPLETE')
    if p.get('error'):print(p['error'])


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    launch=sub.add_parser('start',help='detach one sequential worker for all six groups')
    launch.add_argument('--episodes',type=int,default=100,help='planned trials PER TARGET; default total 600')
    launch.add_argument('--seed',type=int,default=20261102)
    launch.add_argument('--max-ticks',type=int,default=1000)
    launch.add_argument('--rrt-iterations',type=int,default=1000)
    launch.add_argument('--rules',default=str(HERE/'rule_groups.json'))
    launch.add_argument('--output-parent',default=str(ROOT/'data_collection'))
    for name in ('worker','status'):
        child=sub.add_parser(name);child.add_argument('--output',required=True)
    args=parser.parse_args(argv)
    if args.command=='start':
        if args.episodes<1 or args.max_ticks<3 or args.rrt_iterations<1 or not 0<=args.seed<2**32:parser.error('positive budgets, max-ticks>=3, uint32 seed required')
        start(args);return 0
    if args.command=='worker':return worker(args)
    status(args);return 0


if __name__=='__main__':sys.exit(main())
