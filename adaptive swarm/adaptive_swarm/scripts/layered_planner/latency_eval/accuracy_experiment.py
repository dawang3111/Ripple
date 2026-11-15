"""Accuracy-window experiments. Native position diagnostic is explicitly opt-in.

G1 monitors rules immediately; G2-G5 miss the current window, wait simulation
seconds, then choose a new publication without rechecking the rule. All groups
use the same next-publication TX delay; no synthetic result data is imported.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import random
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime,timezone

from .accuracy_transport import AccuracyQueue,WAIT_SECONDS
from .intensity_experiment import HERE,ROOT,TARGETS,unique,capabilities,outcomes,read_offsets
from .rule_campaign import dump,write_csv,preflight


def utc():return datetime.now(timezone.utc).isoformat()


def detect(trial,args):
    """Reuse strict geometry/timeout; never turn unavailable windows into zeros."""
    labels,details=outcomes(trial,args)
    if details.get('min_cumulative_path_30s_m') is None:
        labels['deadlock']=None
        details['deadlock_missing_reason']='no complete continuous 30s movement observation window'
    evidence={t:[] for t in TARGETS}
    if labels['drone_drone'] or labels['drone_obstacle']:
        from shapely.geometry import Point,Polygon
        polygons=[Polygon(o) for o in trial['obstacles']]
        from itertools import combinations
        for frame in trial['trace']:
            positions=frame['positions'];ids=sorted(positions,key=int)
            common=dict(time_s=frame['time_s'],fsm=frame['fsm'],planning_state=frame['planning_state'],
                source='saved truth trace',body_radius_m=args.body_radius_m)
            if labels['drone_drone'] and not evidence['drone_drone']:
                for a,b in combinations(ids,2):
                    distance=math.dist(positions[a],positions[b])
                    if distance<=2*args.body_radius_m:
                        evidence['drone_drone'].append(dict(common,drone_ids=[int(a),int(b)],center_distance_m=distance));break
            if labels['drone_obstacle'] and not evidence['drone_obstacle']:
                for a in ids:
                    point=Point(positions[a])
                    for index,poly in enumerate(polygons):
                        clearance=-poly.boundary.distance(point) if poly.covers(point) else poly.distance(point)
                        if clearance<=args.body_radius_m:
                            evidence['drone_obstacle'].append(dict(common,drone_ids=[int(a)],obstacle_id=index,center_signed_clearance_m=clearance));break
                    if evidence['drone_obstacle']:break
            if all(evidence[t] for t in ('drone_drone','drone_obstacle') if labels[t]):break
    if labels['delay']:
        evidence['delay']=[dict(time_s=trial['time_s'],drone_ids=[],fsm='TASK_SUCCESS',
            planning_state='finished',completion_s=trial['time_s'],threshold_s=args.timeout_threshold_s)]
    if labels['deadlock']:
        # Only a fully observed window is eligible, using cumulative path length.
        trace=trial['trace'];steps=round(30*trial['params']['ViconRate'])
        import numpy as np
        ids=sorted(trace[0]['positions'],key=int)
        positions=np.array([[s['positions'][k] for k in ids] for s in trace])
        cumulative=np.vstack([np.zeros((1,len(ids))),np.cumsum(np.linalg.norm(np.diff(positions,axis=0),axis=2),axis=0)])
        for end in range(steps,len(trace)):
            begin=end-steps
            if abs(trace[end]['time_s']-trace[begin]['time_s']-30)>1e-8:continue
            hit=[int(ids[i]) for i,x in enumerate(cumulative[end]-cumulative[begin]) if x<=args.motion_tolerance_m]
            if hit:
                evidence['deadlock']=[dict(time_s=trace[end]['time_s'],window_start_s=trace[begin]['time_s'],
                    drone_ids=hit,fsm=trace[end]['fsm'],planning_state=trace[end]['planning_state'])];break
    for target in TARGETS:
        if labels[target] is None and not details.get(target+'_missing_reason'):
            if trial['status'] in ('error','rrt_failed') or not trial['trace']:
                reason='no valid task observation: '+trial['status']
            elif target=='delay':
                reason='task did not complete successfully' if trial['status']!='success' else 'completion threshold unavailable'
            elif target in ('drone_drone','drone_obstacle'):
                reason='physical collision geometry unavailable'
            else:reason='required signal unavailable'
            details[target+'_missing_reason']=reason
    details['evidence_sampling']='truth trace each ViconRate tick; collisions between samples are not proven absent'
    return labels,details,evidence


def run_trial(args):
    if not args.diagnostic_position_adapter and not args.control:
        raise ValueError('Formal accuracy experiment unavailable: no actual published timed-trajectory adapter. Use --diagnostic-position-adapter only for position mechanism validation.')
    settings=json.loads(Path(args.rules).read_text())
    rules=next(g['rules'] for g in settings['groups'] if g['target']==args.target)
    out=unique(args.output_parent,'accuracy_trial')
    started=utc();timer=time.monotonic()
    dump(out/'manifest.json',dict(status='running',started_at_utc=started,parameters=vars(args),
        source_workbook=settings['source_workbook'],source_workbook_sha256=settings['source_sha256'],
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')},
        capabilities=capabilities(),standard_policy='next_same_sender_publication',wait_seconds=WAIT_SECONDS))
    from .simulation import TxSimulation
    from .core import Config,feature_vector
    from .rule_policy import rule_matches
    from ..Adaptive_swarm_collsion_findradius_obstacle_inflation import to_builtin

    class AccuracySimulation(TxSimulation):
        def __init__(self):
            super().__init__(args.trial_id,out,args.seed,Config(max_ticks=args.max_ticks,rrt_iterations=args.rrt_iterations))
            self.transport=AccuracyQueue(args.group,enabled=not args.control)
            self.snapshots=[];self.checks=0
        def publish(self,robot,tick,index):
            now=tick*self.dt+(index+1)*self.dt/len(self.robots);self.clock=now
            self.transport.advance(now);self.histories[robot.id].append((now,robot.sp.copy()))
            matched=[];features=None
            # No rule recheck while waiting or during a delay. No inherited random start tick.
            if robot.id==self.plan['sender_id'] and self.transport.active_window is None:
                self.checks+=1
                features=feature_vector(self.positions(),self.histories,robot.id,now,self.transport.cache,
                    {r.id:r.sp_global.copy() for r in self.robots},self.obstacles,self.xy_goal,self.sp_index,len(self.trajectory))
                matched=[r['rule_id'] for r in rules if rule_matches(features,r,now)]
            if matched:
                self.snapshots.append(to_builtin(dict(time_s=now,sender_id=robot.id,seq=tick,rule_ids=matched,features=features)))
                if not self.features:self.features=features
            self.transport.publish(robot.id,tick,robot.sp,now,inject=bool(matched),rule_ids=matched,state='RUNNING')

    sim=AccuracySimulation();trial=sim.start()
    trial.update(trial_id=args.trial_id,target=args.target,group=args.group,rule_observations=sim.snapshots,
        rule_checks=sim.checks,attack_windows=sim.transport.windows,delay_operations=sim.transport.operations,
        experiment_kind='accuracy',validation_mode='position_adapter_diagnostic_only')
    labels,details,evidence=detect(trial,args)
    offset_input=dict(trial,messages=[e for e in trial['messages'] if e['event'] in ('enqueue','release','receive')])
    offsets=read_offsets(offset_input,args.offset_tolerance_m)
    values=[r['offset_m'] for r in offsets]
    details.update(position_snapshot_mean_offset_m=statistics.mean(values) if values else None,
        position_snapshot_max_offset_m=max(values) if values else None,
        position_snapshot_offset_read_count=sum(r['offset_event'] for r in offsets),
        position_snapshot_offset_rate=sum(r['offset_event'] for r in offsets)/len(offsets) if offsets else None,
        position_snapshot_contiguous_event_count=sum(r['new_contiguous_event'] for r in offsets),
        offset_tolerance_m=args.offset_tolerance_m,trajectory_distance_criterion=None,
        trajectory_event_count_criterion=None,trajectory_event_rate_criterion=None,
        drift_success_criterion='unconfirmed; distance/count/rate remain separate')
    trial.update(strict_labels=labels,strict_evidence=evidence,detector_details=details)
    released=[o for o in sim.transport.operations if o['status']=='released' and o['actual_delay_s']>0]
    waits=[o['actual_wait_s'] for o in released]
    summary=dict(trial_id=args.trial_id,seed=args.seed,target=args.target,group=args.group,
        status=trial['status'],note=trial['note'],control=args.control,
        validation_mode='position_adapter_diagnostic_only',rule_window_count=len(sim.transport.windows),
        selected_messages=len(sim.transport.operations),completed_injections=len(released),
        cancelled_injections=sum(o['status']=='cancelled' for o in sim.transport.operations),
        cancelled_waits=sum(w['skip_reason']=='task_ended_during_wait' for w in sim.transport.windows),
        injected=bool(released),injection_skip_reason=sim.transport.injection['skip_reason'],
        target_known=labels[args.target] is not None,target_hit=labels[args.target],strict_labels=labels,
        strict_evidence=evidence,missing_reasons={t:details.get(t+'_missing_reason','') for t in TARGETS if labels[t] is None},
        actual_delay_mean_s=statistics.mean(o['actual_delay_s'] for o in released) if released else None,
        actual_delay_max_s=max((o['actual_delay_s'] for o in released),default=None),
        actual_wait_mean_s=statistics.mean(waits) if waits else None,
        configured_wait_s=WAIT_SECONDS[args.group],task_success=trial['status']=='success',
        completion_s=trial['time_s'] if trial['status']=='success' else None,
        elapsed_simulation_s=trial['time_s'],wall_seconds=time.monotonic()-timer,result_path=str(out))
    dump(out/'trial.json',to_builtin(trial));dump(out/'summary.json',summary)
    write_csv(out/'injections.csv',sim.transport.operations)
    write_csv(out/'attack_windows.csv',sim.transport.windows)
    write_csv(out/'position_snapshot_offsets.csv',to_builtin(offsets))
    manifest=json.loads((out/'manifest.json').read_text())
    manifest.update(status='complete',ended_at_utc=utc(),trial_status=trial['status']);dump(out/'manifest.json',manifest)
    print('RESULT_DIRECTORY='+str(out),flush=True);return summary


def summarize(directory,export_details=True):
    """Read small summaries; avoid reparsing every full message log after each job."""
    directory=Path(directory);manifest=json.loads((directory/'batch_manifest.json').read_text())
    jobs=json.loads((directory/'jobs.json').read_text())
    rows=[]
    for job in jobs:
        # Retry attempts are retained, but a planned trial contributes at most once.
        chosen=next((a for a in reversed(job['attempts']) if a.get('result_path')),None)
        if chosen:rows.append(json.loads((Path(chosen['result_path'])/'summary.json').read_text()))
    summary=[]
    from .analysis import wilson
    for target in manifest['targets']:
        for group in manifest['groups']:
            cohort=[r for r in rows if r['target']==target and r['group']==group]
            attempted=[j for j in jobs if j['target']==target and j['group']==group]
            injected=[r for r in cohort if r['injected']]
            valid=[r for r in injected if r['target_known'] and r['status'] not in ('error','rrt_failed')]
            hits=sum(r['target_hit'] for r in valid);complete_labels=bool(injected) and len(valid)==len(injected)
            reasons={}
            for r in cohort:
                reason=r['injection_skip_reason'] if not r['injected'] else r['missing_reasons'].get(target,'')
                if reason:reasons[reason]=reasons.get(reason,0)+1
            summary.append(dict(target=target,group=group,configured_wait_s=WAIT_SECONDS[group],planned=manifest['episodes'],
                attempted=len(attempted),completed=len(cohort),injected=len(injected),not_injected=len(cohort)-len(injected),
                infrastructure_failures=len(attempted)-len(cohort),
                invalid_trials=sum(r['status'] in ('error','rrt_failed') for r in cohort),
                injection_coverage=len(injected)/len(cohort) if cohort else None,known_injected=len(valid),
                target_hits=hits if valid else None,attack_success_rate=hits/len(injected) if complete_labels else None,
                attack_success_ci95=wilson(hits,len(injected)) if complete_labels else None,
                asr_missing_reason='' if complete_labels else 'no actual injection or unknown/censored labels; denominator not reduced',
                total_completed_injections=sum(r['completed_injections'] for r in cohort),missing_or_skip_reasons=reasons,
                validation_mode='position_adapter_diagnostic_only'))
    write_csv(directory/'trials.csv',rows);write_csv(directory/'summary.csv',summary)
    dump(directory/'progress.json',dict(attempted=len(jobs),completed=len(rows),planned=manifest['planned_trials'],
        all_jobs_complete=len(rows)==manifest['planned_trials']))
    if export_details:
        for filename in ('injections.csv','attack_windows.csv'):
            destination=directory/('all_'+filename)
            with destination.open('w',encoding='utf-8-sig',newline='') as output:
                writer=None
                for row in rows:
                    source=Path(row['result_path'])/filename
                    if not source.exists():continue
                    with source.open(encoding='utf-8-sig',newline='') as f:
                        for event in csv.DictReader(f):
                            combined=dict(target=row['target'],trial_id=row['trial_id'],seed=row['seed'],**event)
                            if writer is None:writer=csv.DictWriter(output,fieldnames=list(combined));writer.writeheader()
                            writer.writerow(combined)
    return summary


def batch(args):
    if not args.diagnostic_position_adapter:raise ValueError('Formal accuracy batch blocked: actual published timed trajectories unavailable')
    environment=preflight()
    targets=list(TARGETS) if args.targets=='all' else args.targets.split(',')
    groups=list(WAIT_SECONDS) if args.groups=='all' else args.groups.split(',')
    if (len(set(targets))!=len(targets) or len(set(groups))!=len(groups)
            or any(t not in TARGETS for t in targets) or any(g not in WAIT_SECONDS for g in groups)):
        raise ValueError('unknown or duplicate target/group')
    out=unique(args.output_parent,'accuracy_batch');(out/'runs').mkdir()
    frozen=out/'rules_frozen.json';frozen.write_bytes(Path(args.rules).read_bytes())
    seeds=random.Random(args.seed).sample(range(2**31-1),len(targets)*args.episodes)
    manifest=dict(status='running',started_at_utc=utc(),planned_trials=len(targets)*len(groups)*args.episodes,
        episodes=args.episodes,targets=targets,groups=groups,seeds=seeds,parameters=vars(args),
        seed_pairing='same target/trial seed across five groups',environment=environment,capabilities=capabilities())
    dump(out/'batch_manifest.json',manifest);jobs=[];dump(out/'jobs.json',jobs)
    print('BATCH_DIRECTORY='+str(out),flush=True)
    child=None
    try:
        for ti,target in enumerate(targets):
            for group in groups:
                for i in range(args.episodes):
                    seed=seeds[ti*args.episodes+i];job=dict(target=target,group=group,trial_id=i+1,seed=seed,attempts=[])
                    jobs.append(job)
                    for attempt in range(args.retries+1):
                        attempt_dir=out/'runs'/('job_%04d_attempt_%02d'%(len(jobs),attempt+1));attempt_dir.mkdir()
                        cmd=[sys.executable,'-u','-m',__package__+'.accuracy_experiment','run','--diagnostic-position-adapter',
                            '--target',target,'--group',group,'--seed',str(seed),'--trial-id',str(i+1),
                            '--rules',str(frozen),'--output-parent',str(attempt_dir)]
                        for flag in ('max_ticks','rrt_iterations','motion_tolerance_m','offset_tolerance_m','body_radius_m','timeout_threshold_s'):
                            value=getattr(args,flag)
                            if value is not None:cmd.extend(['--'+flag.replace('_','-'),str(value)])
                        log=attempt_dir/'console.log';attempt_info=dict(started_at_utc=utc(),log=str(log),command=cmd,status='running')
                        job['attempts'].append(attempt_info);dump(out/'jobs.json',jobs)
                        with log.open('x') as output:
                            child=subprocess.Popen(cmd,cwd=ROOT,stdout=output,stderr=subprocess.STDOUT,
                                env=dict(os.environ,MPLBACKEND='Agg',PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',
                                    OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR=str(out/'mpl_cache')))
                            try:code=child.wait(timeout=args.wall_timeout_s)
                            except subprocess.TimeoutExpired:
                                child.kill();child.wait();code=None
                        child=None
                        attempt_info.update(status='wall_timeout' if code is None else ('finished' if code==0 else 'failed'),
                            returncode=code,ended_at_utc=utc())
                        results=list(attempt_dir.glob('accuracy_trial_*/summary.json'))
                        if code==0 and len(results)==1:attempt_info['result_path']=str(results[0].parent)
                        dump(out/'jobs.json',jobs)
                        if attempt_info.get('result_path'):break
                    summarize(out,export_details=False)
                    print('%d/%d jobs attempted'%(len(jobs),manifest['planned_trials']),flush=True)
        summarize(out)
        manifest['status']='complete' if json.loads((out/'progress.json').read_text())['all_jobs_complete'] else 'incomplete'
    except BaseException as exc:
        manifest.update(status='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed',error=type(exc).__name__+': '+str(exc))
        raise
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        for job in jobs:
            for attempt_info in job['attempts']:
                if attempt_info['status']=='running':attempt_info.update(status='interrupted',ended_at_utc=utc())
        dump(out/'jobs.json',jobs);manifest['ended_at_utc']=utc();dump(out/'batch_manifest.json',manifest)
    print('BATCH_DIRECTORY='+str(out),flush=True);return out


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('preflight')
    stat=sub.add_parser('summarize');stat.add_argument('--input',required=True)
    for mode in ('run','batch'):
        p=sub.add_parser(mode);p.add_argument('--rules',default=str(HERE/'intensity_rules_frozen.json'))
        p.add_argument('--diagnostic-position-adapter',action='store_true');p.add_argument('--seed',type=int,default=20261112)
        p.add_argument('--output-parent',default=str(ROOT/'data_collection'));p.add_argument('--max-ticks',type=int,default=4000)
        p.add_argument('--rrt-iterations',type=int,default=1000);p.add_argument('--body-radius-m',type=float)
        p.add_argument('--timeout-threshold-s',type=float);p.add_argument('--motion-tolerance-m',type=float,default=0.)
        p.add_argument('--offset-tolerance-m',type=float,default=0.)
        if mode=='run':
            p.add_argument('--target',choices=TARGETS,required=True);p.add_argument('--group',choices=tuple(WAIT_SECONDS),required=True)
            p.add_argument('--trial-id',type=int,default=1);p.add_argument('--control',action='store_true')
        else:
            p.add_argument('--targets',default='all');p.add_argument('--groups',default='all');p.add_argument('--episodes',type=int,default=50)
            p.add_argument('--wall-timeout-s',type=float,default=300.);p.add_argument('--retries',type=int,default=0)
    args=parser.parse_args(argv)
    if args.command=='preflight':print(json.dumps(capabilities(),ensure_ascii=False,indent=2));return
    if args.command=='summarize':summarize(args.input);return
    frozen=json.loads(Path(args.rules).read_text());calibration=frozen.get('timeout_calibration',{})
    if args.body_radius_m is None:args.body_radius_m=frozen.get('geometry_calibration',{}).get('body_radius_m')
    if args.timeout_threshold_s is None and calibration:args.timeout_threshold_s=2*calibration['completion_mean_s']
    args.timeout_threshold_source=('2x repository baseline mean, user confirmed 2026-11-11'
        if calibration and args.timeout_threshold_s==2*calibration['completion_mean_s'] else 'explicit CLI or unavailable')
    for name in ('motion_tolerance_m','offset_tolerance_m','body_radius_m','timeout_threshold_s'):
        value=getattr(args,name)
        if value is not None and (not math.isfinite(value) or value<0 or (name in ('body_radius_m','timeout_threshold_s') and value==0)):
            parser.error('invalid finite threshold '+name)
    if args.max_ticks<2 or args.rrt_iterations<1:parser.error('invalid simulation budget')
    if args.command=='batch' and (args.episodes<1 or args.retries<0 or not math.isfinite(args.wall_timeout_s) or args.wall_timeout_s<=0):parser.error('invalid batch budget')
    if args.command=='run':return run_trial(args)
    def stop(signum,frame):raise KeyboardInterrupt('received signal '+str(signum))
    previous={s:signal.signal(s,stop) for s in (signal.SIGINT,signal.SIGTERM)}
    try:return batch(args)
    finally:
        for s,handler in previous.items():signal.signal(s,handler)


if __name__=='__main__':main()
