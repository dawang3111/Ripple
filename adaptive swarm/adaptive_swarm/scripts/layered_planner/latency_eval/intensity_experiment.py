"""Five intensity policies, workbook rules, and bounded isolated diagnostic runs.

Native adapter broadcasts POSITION snapshots, not published timed trajectories.
Formal trajectory experiments fail closed until a compatible adapter is supplied.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import random
import statistics
import subprocess
import sys
import time
import uuid
from datetime import datetime,timezone

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
TARGETS=('drone_drone','drone_obstacle','deadlock','delay','planning','drift')


def dump(path,data):Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def unique(parent,prefix):
    parent=Path(parent).resolve();parent.mkdir(parents=True,exist_ok=True)
    out=parent/(prefix+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex)
    out.mkdir(exist_ok=False);return out


def csv_write(path,rows):
    if not rows:return
    with Path(path).open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader()
        for r in rows:writer.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v for k,v in r.items()})


def capabilities():
    import shutil
    frozen=json.loads((HERE/'intensity_rules_frozen.json').read_text())
    radius=frozen.get('geometry_calibration',{}).get('body_radius_m')
    return dict(adapter='position_snapshot_100Hz',native_swarm_launch_preexisting=False,
        rospy_available=bool(__import__('importlib.util',fromlist=['find_spec']).find_spec('rospy')),
        roslaunch=shutil.which('roslaunch'),catkin_make=shutil.which('catkin_make'),
        published_timed_per_uav_trajectories=False,native_body_geometry_defined=False,
        physical_body_radius_defined=radius is not None,configured_body_radius_m=radius,
        trajectory_drift_baseline_available=False,
        formal_experiment_ready=False)


def read_offsets(trial,tolerance):
    import numpy as np
    truth={};rows=[];previous={};runs={}
    for event in trial['messages']:
        sender=event['sender_id'];generated=event['generated_time']
        if event['event'] in ('enqueue','release'):
            if sender not in truth or generated>=truth[sender][0]:truth[sender]=(generated,event['payload'])
        elif event['event']=='receive':
            if sender not in truth:
                if event['seq']!=-1:raise ValueError('receive has no source-state predecessor')
                truth[sender]=(generated,event['payload'])
            value=float(np.linalg.norm(np.asarray(truth[sender][1])-event['payload']))
            key=(sender,event['receiver_id']);offset=value>tolerance
            start=offset and not previous.get(key,False)
            if start:runs[key]=runs.get(key,0)+1
            previous[key]=offset
            rows.append(dict(sender_id=sender,receiver_id=event['receiver_id'],seq=event['seq'],read_time_s=event['receive_time'],
                truth_source_time_s=truth[sender][0],broadcast_source_time_s=generated,truth_position=truth[sender][1],
                broadcast_position=event['payload'],offset_m=value,offset_event=offset,new_contiguous_event=start,
                source_type='position_snapshot_not_timed_trajectory'))
    return rows


def outcomes(trial,args):
    import numpy as np
    from shapely.geometry import Point,Polygon
    trace=trial['trace'];observed=bool(trace) and trial['status'] not in ('error','rrt_failed')
    labels={t:None for t in TARGETS};details={}
    if not observed:return labels,details
    positions=np.array([[s['positions'][k] for k in sorted(s['positions'],key=int)] for s in trace]);times=np.array([s['time_s'] for s in trace])
    distances=[np.linalg.norm(positions[:,i]-positions[:,j],axis=1) for i in range(positions.shape[1]) for j in range(i+1,positions.shape[1])]
    minimum=float(np.min(distances));polygons=[Polygon(o) for o in trial['obstacles']]
    obstacle_min=min((-poly.boundary.distance(Point(p)) if poly.covers(Point(p)) else poly.distance(Point(p))) for frame in positions for p in frame for poly in polygons)
    details.update(min_center_pair_distance_m=minimum,min_center_obstacle_signed_clearance_m=obstacle_min,
                   legacy_pair_threshold_m=trial['params']['crash_uav_dist'],legacy_pair_collision=minimum<=trial['params']['crash_uav_dist'],
                   legacy_point_obstacle_collision=obstacle_min<=0.,body_radius_m=args.body_radius_m)
    if args.body_radius_m is not None:
        labels['drone_drone']=minimum<=2*args.body_radius_m
        labels['drone_obstacle']=obstacle_min<=args.body_radius_m
    # Window cumulative distance, never net displacement or goal progress.
    steps=int(round(30*trial['params']['ViconRate']));minimum_path=None
    if len(trace)>steps:
        cumulative=np.vstack([np.zeros((1,positions.shape[1])),np.cumsum(np.linalg.norm(np.diff(positions,axis=0),axis=2),axis=0)])
        window=cumulative[steps:]-cumulative[:-steps];window[np.abs(times[steps:]-times[:-steps]-30)>1e-8]=np.inf
        if np.isfinite(window).any():minimum_path=float(window.min())
    details.update(deadlock_window_s=30.,deadlock_cumulative_distance_tolerance_m=args.motion_tolerance_m,min_cumulative_path_30s_m=minimum_path)
    labels['deadlock']=minimum_path<=args.motion_tolerance_m if minimum_path is not None else False
    labels['delay']=(trial['time_s']>args.timeout_threshold_s) if trial['status']=='success' and args.timeout_threshold_s is not None else None
    details.update(task_success=trial['status']=='success',task_completion_s=trial['time_s'] if trial['status']=='success' else None,
                   timeout_threshold_s=args.timeout_threshold_s,timeout_threshold_source=getattr(args,'timeout_threshold_source','explicit test/CLI'),
                   planning_missing_reason='no published timed per-UAV future trajectory; history Path is not future plan',
                   drift_missing_reason='position snapshots cannot define trajectory at absolute time',
                   collision_missing_reason='' if args.body_radius_m is not None else 'physical collision body radius not confirmed')
    return labels,details


def run_trial(args):
    if not args.diagnostic_position_adapter and not args.control:
        raise ValueError('Formal trajectory experiment unavailable: native adapter publishes position snapshots; specify --diagnostic-position-adapter ONLY for mechanism validation. ROS/timed trajectories remain unresolved.')
    rules=json.loads(Path(args.rules).read_text());group=next(g for g in rules['groups'] if g['target']==args.target)
    out=unique(args.output_parent,'intensity_trial');os.environ.setdefault('MPLCONFIGDIR',str(out/'matplotlib_cache'));os.environ['MPLBACKEND']='Agg'
    from .simulation import TxSimulation
    from .core import Config,feature_vector
    from .rule_policy import rule_matches
    from .intensity_transport import IntensityQueue
    from ..Adaptive_swarm_collsion_findradius_obstacle_inflation import to_builtin
    import numpy as np

    class IntensitySimulation(TxSimulation):
        def __init__(self):
            super().__init__(args.trial_id,out,args.seed,Config(max_ticks=args.max_ticks,rrt_iterations=args.rrt_iterations))
            self.transport=IntensityQueue(args.group,enabled=not args.control,
                delays={'G1':args.g1_delay_s,'G2':args.g2_delay_s,'G3':args.g3_delay_s,'G4':args.g4_delay_s})
            self.snapshots=[];self.checks=0
        def publish(self,robot,tick,index):
            now=tick*self.dt+(index+1)*self.dt/len(self.robots);self.clock=now;self.transport.advance(now)
            self.histories[robot.id].append((now,robot.sp.copy()));matched=[]
            if robot.id==self.plan['sender_id']:
                self.checks+=1
                features=feature_vector(self.positions(),self.histories,robot.id,now,self.transport.cache,
                    {r.id:r.sp_global.copy() for r in self.robots},self.obstacles,self.xy_goal,self.sp_index,len(self.trajectory))
                matched=[r['rule_id'] for r in group['rules'] if rule_matches(features,r,now)]
                if matched:
                    self.snapshots.append(to_builtin(dict(time_s=now,sender_id=robot.id,seq=tick,rule_ids=matched,features=features)))
                    if not self.features:self.features=features
            self.transport.publish(robot.id,tick,robot.sp,now,inject=bool(matched),rule_ids=matched,state='RUNNING')

    settings=vars(args).copy();settings['source_workbook_sha256']=rules['source_sha256']
    dump(out/'manifest.json',dict(status='running',parameters=settings,capabilities=capabilities(),source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')}))
    sim=IntensitySimulation();trial=sim.start();trial['trial_id']=args.trial_id;trial['group']=args.group;trial['target']=args.target
    trial['rule_observations']=sim.snapshots;trial['rule_checks']=sim.checks;trial['delay_operations']=sim.transport.operations
    labels,details=outcomes(trial,args);offsets=read_offsets(trial,args.offset_tolerance_m)
    values=[r['offset_m'] for r in offsets];events=sum(r['offset_event'] for r in offsets)
    details.update(position_snapshot_mean_offset_m=statistics.mean(values) if values else None,
                   position_snapshot_max_offset_m=max(values) if values else None,position_snapshot_offset_read_count=events,
                   position_snapshot_offset_rate=events/len(offsets) if offsets else None,
                   position_snapshot_contiguous_event_count=sum(r['new_contiguous_event'] for r in offsets),
                   offset_tolerance_m=args.offset_tolerance_m,trajectory_distance_criterion=None,trajectory_event_criterion=None)
    trial['strict_labels']=labels;trial['detector_details']=details
    completed=[o for o in sim.transport.operations if o['status']=='released']
    target_known=labels[args.target] is not None
    summary=dict(trial_id=args.trial_id,seed=args.seed,target=args.target,group=args.group,status=trial['status'],note=trial['note'],
        validation_mode='position_adapter_diagnostic_only',rule_trigger_count=sim.transport.rule_triggers,
        selected_messages=len(sim.transport.operations),completed_injections=len(completed),cancelled_injections=sum(o['status']=='cancelled' for o in sim.transport.operations),
        injected=bool(completed),target_known=target_known,target_hit=labels[args.target],strict_labels=labels,
        actual_delay_mean_s=statistics.mean(o['actual_delay_s'] for o in completed) if completed else None,
        actual_delay_max_s=max(o['actual_delay_s'] for o in completed) if completed else None,
        truncated_injections=sum(o['truncated'] for o in completed),task_success=trial['status']=='success',completion_s=trial['time_s'] if trial['status']=='success' else None,
        result_path=str(out))
    dump(out/'trial.json',to_builtin(trial));dump(out/'summary.json',summary)
    csv_write(out/'injections.csv',[dict(trial_id=args.trial_id,seed=args.seed,target=args.target,**o) for o in sim.transport.operations])
    csv_write(out/'position_snapshot_offsets.csv',to_builtin(offsets))
    manifest=json.loads((out/'manifest.json').read_text());manifest.update(status='complete',trial_status=trial['status'])
    dump(out/'manifest.json',manifest)
    print('RESULT_DIRECTORY='+str(out),flush=True);return summary


def summarize(directory):
    directory=Path(directory);plans=json.loads((directory/'batch_manifest.json').read_text());rows=[];operations=[]
    for path in sorted(directory.glob('runs/**/summary.json')):
        rows.append(json.loads(path.read_text()))
        data=json.loads((path.parent/'trial.json').read_text())
        operations.extend(dict(o,target=data['target'],seed=data['seed'],trial_id=data['trial_id']) for o in data['delay_operations'])
    result=[]
    for target in plans['targets']:
        for group in plans['groups']:
            cohort=[r for r in rows if r['target']==target and r['group']==group];injected=[r for r in cohort if r['injected']]
            eligible=[r for r in injected if r['target_known'] and r['status'] not in ('error','rrt_failed')]
            hits=sum(bool(r['target_hit']) for r in eligible)
            values=[o['actual_delay_s'] for o in operations if o['target']==target and o['group']==group and o['status']=='released']
            percentiles={}
            if values:
                ordered=sorted(values)
                for q in (.05,.5,.95):percentiles['p'+str(int(q*100))]=ordered[round(q*(len(ordered)-1))]
            result.append(dict(target=target,group=group,planned=plans['episodes'],completed=len(cohort),injected=len(injected),not_injected=len(cohort)-len(injected),
                injection_coverage=len(injected)/len(cohort) if cohort else None,strict_known_injected=len(eligible),
                strict_hits=hits if eligible else None,strict_asr=hits/len(injected) if injected and len(eligible)==len(injected) else None,
                asr_reason='' if injected and len(eligible)==len(injected) else 'no injection or unknown/censored outcomes; do not silently shrink ASR denominator',
                total_rule_triggers=sum(r['rule_trigger_count'] for r in cohort),total_completed_injections=len(values),
                actual_delay_mean_s=statistics.mean(values) if values else None,actual_delay_max_s=max(values) if values else None,
                actual_delay_quantiles_s=percentiles,validation_mode='position_adapter_diagnostic_only'))
    csv_write(directory/'trials.csv',rows);csv_write(directory/'summary.csv',result);csv_write(directory/'all_injections.csv',operations)
    dump(directory/'progress.json',dict(completed=len(rows),planned=plans['episodes']*len(plans['targets'])*len(plans['groups']),
        all_jobs_complete=len(rows)==plans['episodes']*len(plans['targets'])*len(plans['groups']),summary=result))
    return result


def batch(args):
    if not args.diagnostic_position_adapter:raise ValueError('Formal 1500-trial experiment blocked by absent timed-trajectory adapter; diagnostic mode must be explicit')
    from .rule_campaign import preflight
    preflight()
    targets=TARGETS if args.targets=='all' else args.targets.split(',');groups=('G1','G2','G3','G4','G5') if args.groups=='all' else args.groups.split(',')
    if any(t not in TARGETS for t in targets) or any(g not in ('G1','G2','G3','G4','G5') for g in groups):raise ValueError('unknown target/group')
    out=unique(args.output_parent,'intensity_batch');args.batch_output=str(out);runs=out/'runs';runs.mkdir()
    frozen=out/'rules_frozen.json';frozen.write_bytes(Path(args.rules).read_bytes())
    seeds=random.Random(args.seed).sample(range(2**31-1),len(targets)*args.episodes)
    dump(out/'batch_manifest.json',dict(status='running',targets=list(targets),groups=list(groups),episodes=args.episodes,seeds=seeds,
        seed_pairing='same target/trial seed reused across all G1-G5',parameters=vars(args),capabilities=capabilities()))
    jobs=[];index=0
    print('BATCH_DIRECTORY='+str(out),flush=True)
    for ti,target in enumerate(targets):
        for group in groups:
            for i in range(args.episodes):
                index+=1;seed=seeds[ti*args.episodes+i]
                cmd=[sys.executable,'-u','-m',__package__+'.intensity_experiment','run','--diagnostic-position-adapter',
                    '--rules',str(frozen),'--target',target,'--group',group,'--seed',str(seed),'--trial-id',str(i+1),'--output-parent',str(runs),
                    '--max-ticks',str(args.max_ticks),'--rrt-iterations',str(args.rrt_iterations),
                    '--motion-tolerance-m',str(args.motion_tolerance_m),'--offset-tolerance-m',str(args.offset_tolerance_m)]
                for flag,value in [('--body-radius-m',args.body_radius_m),('--timeout-threshold-s',args.timeout_threshold_s)]:
                    if value is not None:cmd.extend([flag,str(value)])
                attempts=[]
                for attempt in range(args.retries+1):
                    log=out/('job_%04d_attempt_%02d.log'%(index,attempt+1))
                    try:
                        with log.open('x') as f:
                            p=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=args.wall_timeout_s,env=dict(os.environ,MPLBACKEND='Agg',PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR=str(out/'mpl_cache')))
                        status='finished' if p.returncode==0 else 'failed';attempts.append(dict(status=status,returncode=p.returncode,log=str(log)))
                        if p.returncode==0:break
                    except subprocess.TimeoutExpired:attempts.append(dict(status='wall_timeout',log=str(log)))
                jobs.append(dict(target=target,group=group,trial_id=i+1,seed=seed,attempts=attempts));dump(out/'jobs.json',jobs)
                summarize(out);print(f'{index}/{len(targets)*len(groups)*args.episodes} jobs attempted',flush=True)
    summarize(out)
    manifest=json.loads((out/'batch_manifest.json').read_text());manifest['status']='complete' if json.loads((out/'progress.json').read_text())['all_jobs_complete'] else 'incomplete'
    dump(out/'batch_manifest.json',manifest)
    print('BATCH_DIRECTORY='+str(out),flush=True);return out


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    imp=sub.add_parser('import-rules');imp.add_argument('--workbook',required=True);imp.add_argument('--output',required=True)
    pre=sub.add_parser('preflight')
    stat=sub.add_parser('summarize');stat.add_argument('--input',required=True)
    for mode in ('run','batch'):
        p=sub.add_parser(mode);p.add_argument('--rules',default=str(HERE/'intensity_rules_frozen.json'))
        p.add_argument('--diagnostic-position-adapter',action='store_true');p.add_argument('--seed',type=int,default=20261111)
        p.add_argument('--output-parent',default=str(ROOT/'data_collection'));p.add_argument('--max-ticks',type=int,default=4000)
        p.add_argument('--rrt-iterations',type=int,default=1000);p.add_argument('--body-radius-m',type=float)
        p.add_argument('--timeout-threshold-s',type=float);p.add_argument('--motion-tolerance-m',type=float,default=0.)
        p.add_argument('--offset-tolerance-m',type=float,default=0.)
        if mode=='run':
            p.add_argument('--target',choices=TARGETS,required=True);p.add_argument('--group',choices=('G1','G2','G3','G4','G5'),required=True)
            p.add_argument('--trial-id',type=int,default=1);p.add_argument('--control',action='store_true')
            for i in range(1,5):p.add_argument('--g%d-delay-s'%i,type=float,default=i*.2)
        else:
            p.add_argument('--targets',default='all');p.add_argument('--groups',default='all');p.add_argument('--episodes',type=int,default=50)
            p.add_argument('--wall-timeout-s',type=float,default=300.);p.add_argument('--retries',type=int,default=0)
    args=parser.parse_args(argv)
    if args.command=='import-rules':
        from .intensity_rules import import_rules
        result=import_rules(args.workbook)
        with Path(args.output).open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
        print({g['target']:len(g['rules']) for g in result['groups']});return
    if args.command=='preflight':print(json.dumps(capabilities(),ensure_ascii=False,indent=2));return
    if args.command=='summarize':summarize(args.input);return
    frozen=json.loads(Path(args.rules).read_text())
    calibration=frozen.get('timeout_calibration',{})
    geometry=frozen.get('geometry_calibration',{})
    if args.body_radius_m is None and geometry:args.body_radius_m=geometry['body_radius_m']
    args.body_radius_source='user confirmed single-UAV radius 2026-11-11' if geometry and args.body_radius_m==geometry['body_radius_m'] else 'explicit CLI or unavailable'
    if args.timeout_threshold_s is None and calibration:
        args.timeout_threshold_s=2*calibration['completion_mean_s']
    args.timeout_threshold_source=('2x repository baseline mean, user confirmed 2026-11-11'
        if calibration and args.timeout_threshold_s==2*calibration['completion_mean_s'] else 'explicit CLI or unavailable')
    if args.max_ticks<2 or args.rrt_iterations<1 or args.motion_tolerance_m<0 or args.offset_tolerance_m<0 or (args.body_radius_m is not None and args.body_radius_m<=0):parser.error('invalid budgets/tolerances/body radius')
    if args.command=='batch' and (args.episodes<1 or args.retries<0 or args.wall_timeout_s<=0):parser.error('invalid batch size/retry/timeout')
    try:
        return run_trial(args) if args.command=='run' else batch(args)
    except BaseException as exc:
        if args.command=='batch' and hasattr(args,'batch_output'):
            p=Path(args.batch_output)/'batch_manifest.json';manifest=json.loads(p.read_text())
            manifest.update(status='failed',error=type(exc).__name__+': '+str(exc));dump(p,manifest)
        raise


if __name__=='__main__':main()
