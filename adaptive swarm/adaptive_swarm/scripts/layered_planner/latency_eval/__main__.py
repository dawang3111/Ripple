"""python -m adaptive_swarm.scripts.layered_planner.latency_eval {run,analyze}"""
import argparse
import hashlib
import importlib.metadata
import json
import platform
import uuid
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from . import SCHEMA_VERSION
from .analysis import analyze, write_csv
from .core import Config, TARGETS

UNSUPPORTED = {
    'future_3s_min_pair_distance':'no timed per-robot future trajectory; actual future forbidden',
    'future_obstacle_distance':'no timed per-robot future trajectory',
    'body_heading':'no body attitude signal; displacement course is separately named',
    'local_replan_count':'no local replanning event definition',
    'emergency_stop_count':'no emergency stop event',
    'new_trajectory_count':'no trajectory publication protocol',
    'replans_since_trajectory_publication':'no trajectory publication protocol',
    'source_victim_cluster_trajectory_deviation':'no time-aligned reference trajectory for followers; current-target offset only',
    'victim_id':'broadcast can affect multiple followers; no single victim before observing outcomes',
}


def unique_directory(parent,prefix):
    parent=Path(parent); parent.mkdir(parents=True,exist_ok=True)
    path=parent/(prefix+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex)
    path.mkdir(exist_ok=False)
    return path


def dump(path,value):
    Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')


def fingerprint(path):
    path=Path(path)
    return dict(path=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def manifest(args,config=None):
    here=Path(__file__).resolve().parent
    sources=list(here.glob('*.py'))+[here.parent/n for n in ['rrt.py','tools.py','potential_fields.py',
                  'Adaptive_swarm_collsion_findradius_obstacle_inflation.py']]
    versions={'python':platform.python_version()}
    for package in ['numpy','scipy','matplotlib','shapely','svgpath2mpl']:
        try: versions[package]=importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError: versions[package]='unavailable'
    return dict(schema_version=SCHEMA_VERSION,created_utc=datetime.now(timezone.utc).isoformat(),
        command=vars(args),versions=versions,source_files=[fingerprint(p) for p in sources],
        thresholds=asdict(config) if config else None,
        threshold_sources={'crash_distance':'legacy Params.crash_uav_dist',
            'goal_tolerance':'legacy Params.goal_tolerance','obstacle_collision':'matplotlib.Path.contains_point on original polygons',
            'max_ticks':'legacy start() 300; opt-in CLI override',
            'rrt_iterations':'new opt-in bound, default legacy Params.maxiters=1000',
            'deadlock':'explicit CLI only; setpoint stationary and goal-distance range <= progress threshold over window',
            'drift':'explicit CLI only; sustained current-target offset, not navigation estimator drift',
            'protocol_timeout':'absent in repository; optional external adapter constraint, not inferred'},
        unsupported_features=UNSUPPORTED,inputs=[],
        timing='nominal seconds; four deterministic update/publication phases per 100Hz tick',
        fsm_source='adapter states PLANNING/RUNNING/TERMINATED; not native FSM',
        injection_sampling='uniform candidate tick in [1,max_ticks-1]; act only while RUNNING; early termination recorded as skip; no lookahead/baseline rerun',
        analysis_seed=args.analysis_seed)


def export_trials(trials,out):
    rows=[]; vectors=[]
    fields=sorted({k for r in trials for k in r.get('features',{})})
    for r in trials:
        injection=r['injection']; actual=injection.get('nominal_tx_time')
        evidence=r['evidence']
        for target in TARGETS:
            for event in evidence[target]:
                event['relative_to_injection']='no_actual_injection' if not injection['actually_injected'] else ('pre_injection' if event['time_s']<actual else 'at_or_post_injection')
        row=dict(episode=r['episode'],seed=r['seed'],status=r['status'],note=r.get('note',''),
            is_control=r.get('is_control',False),result_path=r['result_path'],tick=r['tick'],time_s=r['time_s'],
            planned_tick=r['plan']['tick'],planned_sender=r['plan']['sender_id'],plan_fraction=r['plan']['fraction'],
            actually_injected=injection['actually_injected'],injection=injection,evidence=evidence)
        for name in ['sender_id','seq','nominal_tx_time','measured_period_s','planned_delay_s',
                     'planned_release_time','next_normal_time','actual_tx_time','actual_release_time',
                     'actual_delay_s','skip_reason','used_stale_count']:
            row[name]=injection.get(name)
        row.update({t:r['labels'][t] for t in TARGETS})
        row.update({t+'_known':r['known'][t] for t in TARGETS})
        rows.append(row)
        vector=dict(episode=r['episode'],actually_injected=injection['actually_injected'],
                    feature_time_s=actual,source_id=injection.get('sender_id',r['plan']['sender_id']))
        for f in fields:
            meta=r.get('features',{}).get(f,{})
            vector[f]=meta.get('value')
            for suffix in ['unit','observed_time','source_times','source','missing_reason']:
                vector[f+'__'+suffix]=meta.get(suffix,'no_injection_snapshot' if suffix=='missing_reason' else '')
        vectors.append(vector)
    write_csv(out/'trials.csv',rows,['episode','seed','status','note','is_control','result_path','tick','time_s',
        'planned_tick','planned_sender','plan_fraction','actually_injected','injection',
        'sender_id','seq','nominal_tx_time','measured_period_s','planned_delay_s','planned_release_time',
        'next_normal_time','actual_tx_time','actual_release_time','actual_delay_s','skip_reason','used_stale_count',*TARGETS,
        *[t+'_known' for t in TARGETS],'evidence'])
    write_csv(out/'trial_features.csv',vectors,['episode','actually_injected','feature_time_s','source_id',
        *[c for f in fields for c in [f,*[f+'__'+s for s in ['unit','observed_time','source_times','source','missing_reason']]]]])


def load_trials(directory):
    directory=Path(directory)
    m=json.loads((directory/'manifest.json').read_text())
    if m.get('schema_version')!=SCHEMA_VERSION: raise ValueError('unsupported input schema')
    files=sorted(directory.glob('trials/trial_*.json'))
    if not files: raise ValueError('no trial JSON inputs')
    rows=[]
    for f in files:
        row=json.loads(f.read_text()); row['result_path']=str(f.resolve()); rows.append(row)
    return rows,[fingerprint(directory/'manifest.json'),*[fingerprint(p) for p in files]]


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    run=sub.add_parser('run',help='isolated bounded simulation; never connects to ROS/radio')
    run.add_argument('--episodes',type=int,default=3000)
    run.add_argument('--seed',type=int,default=20261022)
    run.add_argument('--max-ticks',type=int,default=300)
    run.add_argument('--rrt-iterations',type=int,default=1000)
    run.add_argument('--deadlock-window-s',type=float)
    run.add_argument('--deadlock-progress-m',type=float)
    run.add_argument('--drift-threshold-m',type=float)
    run.add_argument('--drift-window-s',type=float)
    run.add_argument('--protocol-timeout-s',type=float)
    run.add_argument('--control',action='store_true',help='explicit no-injection control only; does not double trials')
    offline=sub.add_parser('analyze',help='read this mode\'s trial JSON, write a fresh analysis directory')
    offline.add_argument('--input',required=True)
    for child in [run,offline]:
        child.add_argument('--output-parent',default='data_collection')
        child.add_argument('--analysis-seed',type=int,default=1729)
        child.add_argument('--controls',help='optional compatible explicit control-run directory')
    return p


def main(argv=None):
    p=parser(); args=p.parse_args(argv)
    if not 0<=args.analysis_seed<2**32: p.error('analysis seed must be uint32')
    config=None
    if args.command=='run':
        if args.episodes<1 or args.max_ticks<2 or args.rrt_iterations<1: p.error('positive episodes/RRT budget and max-ticks >=2 required')
        if not 0<=args.seed<2**32: p.error('seed must be uint32')
        for a,b in [('deadlock_window_s','deadlock_progress_m'),('drift_threshold_m','drift_window_s')]:
            if (getattr(args,a) is None)!=(getattr(args,b) is None): p.error(a+' and '+b+' must be supplied together')
        values={k:getattr(args,k) for k in Config.__dataclass_fields__}
        for k,v in values.items():
            if v is not None and (not np.isfinite(v) or v<=0): p.error(k+' must be finite and positive')
        config=Config(**values)
    data=manifest(args,config)
    controls=[]
    if args.controls:
        controls,files=load_trials(args.controls)
        if any(not r.get('is_control') or r['injection']['actually_injected'] for r in controls):
            p.error('--controls requires explicit uninjected control run, not skipped injection trials')
        data['inputs'].extend(files)
    if args.command=='analyze':
        trials,files=load_trials(args.input); data['inputs'].extend(files)
        source_manifest=json.loads((Path(args.input)/'manifest.json').read_text())
        data['thresholds']=source_manifest.get('thresholds')
        data['algorithm_params']=source_manifest.get('algorithm_params')
        data['threshold_sources']=source_manifest.get('threshold_sources',{})
    else:
        trials=[]
    out=unique_directory(args.output_parent,'tx_latency_'+args.command)
    print('Output:',out.resolve(),flush=True)
    data['status']='running'; dump(out/'manifest.json',data)
    if args.command=='run':
        from .simulation import TxSimulation
        rng=np.random.RandomState(args.seed)
        seeds=rng.randint(0,2**31-1,size=args.episodes).tolist()
        data['trial_seeds']=seeds
        data['status']='running'; dump(out/'manifest.json',data)
        (out/'trials').mkdir()
        for episode,seed in enumerate(seeds,1):
            sim=TxSimulation(episode,out,seed,config)
            data['algorithm_params']=vars(sim.params).copy()
            if episode==1 and controls:
                if any(r.get('params')!=vars(sim.params) or r.get('config')!=asdict(config) for r in controls):
                    raise ValueError('control parameters/thresholds differ from experiment')
            sim.control=args.control
            trial=sim.start(); trial['is_control']=args.control
            path=out/'trials'/('trial_%06d.json'%episode)
            trial['result_path']=str(path.resolve())
            # Add evidence timing before serializing the authoritative trial file.
            nominal=trial['injection'].get('nominal_tx_time')
            for events in trial['evidence'].values():
                for event in events:
                    event['relative_to_injection']='no_actual_injection' if not trial['injection']['actually_injected'] else ('pre_injection' if event['time_s']<nominal else 'at_or_post_injection')
            dump(path,trial)
            # Analysis keeps only summaries/features, not all per-tick/message traces in RAM.
            trials.append({k:v for k,v in trial.items() if k not in ['trace','messages','trajectory_setpoints','rrt_path']})
            if episode%25==0 or episode==len(seeds): print('%d/%d trials complete'%(episode,len(seeds)),flush=True)
        data['inputs'].extend(fingerprint(p) for p in sorted((out/'trials').glob('*.json')))
    if controls and trials:
        for control in controls:
            if control.get('params')!=trials[0].get('params') or control.get('config')!=trials[0].get('config'):
                raise ValueError('control parameters/thresholds differ from experiment')
    export_trials(trials,out)
    analyze(trials,out,args.analysis_seed,controls)
    data['status']='complete'; data['actual_trials']=len(trials)
    dump(out/'manifest.json',data)
    return out


if __name__=='__main__':
    main()
