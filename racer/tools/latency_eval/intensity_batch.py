"""RACER trajectory intensity: plan, bounded pilot, full explicit run and summary."""
import argparse,csv,gzip,json,os,shutil,sys,time
from collections import Counter
from pathlib import Path
import numpy as np
from .core import ROOT,TARGETS,config,unique_output,trial_plan,write_json,digest
from .cli import execute_capture,preflight_run
from .background import resources,experiment_processes,archive_finished,atomic_json,fingerprint
from .process_cleanup import cleanup_owned
from .intensity_metrics import analyze,calibrate

DEFAULT_RULES=ROOT.parents[1]/'racer_eval_results/rule_batch_20261111T115519Z_0c100e70c72743798ccef7537a0e707e/frozen_rules.json'
GROUPS=('G1','G2','G3','G4','G5')

def implementation():
 data=fingerprint();package=ROOT/'swarm_exploration/exploration_manager'
 for p in [package/'launch/swarm.launch',package/'include/exploration_manager/intensity_delay_gate.h',package/'scripts/ripple_rule_monitor.py']:
  data[str(p)]=digest(p)
 return data

def plans(targets,groups,n,seed,rules):
 index=0
 for repeat in range(1,n+1):
  for t in targets:
   pair_index=TARGETS.index(t)*n+repeat
   for group in groups:
    index+=1;p=trial_plan(pair_index,seed,rules['config'])
    p.update(trial_id='trial_%04d'%index,target=t,group=group,repetition=repeat,paired_seed_key=f'{t}/{repeat}',
        intensity_mode=True,baseline=False,control=False,map_path=str(ROOT/'uav_simulator/map_generator/resource/pillar.pcd'))
    yield p

def csv_write(path,rows):
 if not rows:return
 temp=path.with_suffix(path.suffix+'.tmp')
 with temp.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 temp.replace(path)

def summarize(job):
 job=Path(job);manifest=json.loads((job/'manifest.json').read_text());results=[json.loads(p.read_text()) for p in sorted((job/'run').glob('trial_*/intensity_result.json'))]
 rows=[];allinjections=[];trials=[]
 for r in results:
  completed=r['completed_injection_n']>0
  trials.append(dict(trial_id=r['trial_id'],target=r['target'],group=r['group'],seed=r['seed'],status=r['status'],
      task_completed=r['task_completed'],task_duration_s=r['task_duration_s'],injected=completed,
      completed_injection_n=r['completed_injection_n'],actual_release_n=r['actual_release_n'],
      goal_hit=r['labels'].get(r['target'],{}).get('hit'),labels=json.dumps(r['labels'],ensure_ascii=False),
      result_path=r['result_path']))
  allinjections += [dict(trial_id=r['trial_id'],target=r['target'],group=r['group'],**a) for a in r['injections']]
 for target in manifest['targets']:
  for group in manifest['groups']:
   cohort=[r for r in results if r['target']==target and r['group']==group];inj=[r for r in cohort if r['completed_injection_n']>0]
   delays=[a['actual_delay_s'] for r in cohort for a in r['injections'] if a['completed_injection']]
   fields=dict(target=target,group=group,planned_n=manifest['per_cell'],finished_n=len(cohort),
       injected_n=len(inj),uninjected_n=len(cohort)-len(inj),injection_coverage=len(inj)/len(cohort) if cohort else None,
       rule_trigger_message_n=sum(a['selected_message_n'] for a in cohort),rule_observer_update_n=sum(a['rule_trigger_n'] for a in cohort),
       completed_injection_n=sum(a['completed_injection_n'] for a in cohort),task_completed_n=sum(r['task_completed'] for r in cohort),
       unfinished_task_n=sum(r['unfinished_task'] for r in cohort),capture_failed_n=sum(r['status'] not in ('completed','task_budget_exhausted') for r in cohort),
       actual_delay_mean_s=float(np.mean(delays)) if delays else None,actual_delay_max_s=max(delays) if delays else None,
       actual_delay_p50_s=float(np.quantile(delays,.5)) if delays else None,actual_delay_p95_s=float(np.quantile(delays,.95)) if delays else None,
       truncated_injection_n=sum(a['truncated'] for r in cohort for a in r['injections']),cleanup_release_n=sum(a['cleanup_release'] for r in cohort for a in r['injections']),
       payload_or_order_error_n=sum(len(r['injection_errors']) for r in cohort))
   for suffix in ('primary','drift_distance','drift_count','drift_rate'):
    key=target if suffix=='primary' and target!='drift' else suffix
    evaluable=[r for r in inj if r['labels'].get(key,{}).get('valid')]
    hits=sum(r['labels'][key]['hit'] for r in evaluable);unknown=len(inj)-len(evaluable)
    fields.update({suffix+'_hit_n':hits,suffix+'_valid_n':len(evaluable),suffix+'_unknown_n':unknown,
        suffix+'_ASR_observed':hits/len(inj) if inj and not(target=='drift' and suffix=='primary') else None,
        suffix+'_ASR_upper_bound':(hits+unknown)/len(inj) if inj and not(target=='drift' and suffix=='primary') else None,
        suffix+'_rate_in_valid_samples':hits/len(evaluable) if evaluable else None})
   rows.append(fields)
 csv_write(job/'summary_per_target_group.csv',rows);csv_write(job/'trials.csv',trials);csv_write(job/'injections.csv',allinjections)
 write_json(job/'delay_distributions.json',{t+'/'+g:[a['actual_delay_s'] for a in allinjections if a['target']==t and a['group']==g and a['completed_injection']] for t in manifest['targets'] for g in manifest['groups']})
 return rows

def baseline_run(args,parser):
 preflight_run(args.output_root,parser)
 other=experiment_processes()
 if other:parser.error('other experiment active: '+json.dumps(other))
 job=unique_output(args.output_root,'intensity_baseline');run=job/'run';run.mkdir();cfg=config()
 cfg['task_budget_s']=args.task_budget or cfg['task_budget_s'];write_json(run/'config.json',cfg)
 n=args.per_cell or 3
 write_json(job/'manifest.json',dict(mode='uninjected_baseline',planned_n=n,seed=args.seed,implementation=implementation(),rules=str(Path(args.rules).resolve())))
 for i in range(1,n+1):
  plan=trial_plan(i,args.seed,cfg);plan.update(intensity_mode=True,baseline=True,control=True,target='delay_timeout',group='G1',rules_file=str(Path(args.rules).resolve()),map_path=str(ROOT/'uav_simulator/map_generator/resource/pillar.pcd'))
  d=run/plan['trial_id'];d.mkdir();write_json(d/'plan.json',plan)
 print(job,flush=True);os.nice(10);os.environ['RACER_EVAL_MIN_AVAILABLE_MB']='768'
 for i,f in enumerate(sorted(run.glob('trial_*/plan.json')),1):
  current=resources(job)
  if current['mem_available_bytes']<1536*1024**2 or current['disk_free_bytes']<9*1024**3:raise RuntimeError('baseline resource conflict')
  execute_capture(f.parent,cfg)
  result=analyze(f.parent,cfg);write_json(f.parent/'result.json',result);cleanup_owned(f.parent);archive_finished(run)
  atomic_json(job/'progress.json',dict(state='running',finished=i,planned=n))
  if result['status'] not in ('completed','task_budget_exhausted'):raise RuntimeError('baseline capture failed')
 baseline=calibrate(run,cfg);write_json(job/'baseline_reference.json',baseline)
 atomic_json(job/'progress.json',dict(state='finished',finished=n,planned=n));print(json.dumps(baseline),flush=True)
 return 0


def main(argv=None):
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['baseline','plan','pilot','run','recover','summarize'])
 p.add_argument('--baseline',help='directory containing uninjected capture logs, or a calibrated baseline JSON')
 p.add_argument('--rules',default=str(DEFAULT_RULES));p.add_argument('--targets',nargs='+',choices=TARGETS)
 p.add_argument('--groups',nargs='+',choices=GROUPS,default=list(GROUPS));p.add_argument('--per-cell',type=int)
 p.add_argument('--seed',type=int,default=20261113);p.add_argument('--task-budget',type=float)
 p.add_argument('--output-root',default=str(ROOT.parents[1]/'racer_eval_results'));p.add_argument('--input')
 args=p.parse_args(argv)
 if args.mode=='summarize':
  if not args.input:p.error('summarize needs --input')
  print(json.dumps(summarize(args.input),ensure_ascii=False));return 0
 if args.per_cell is not None and args.per_cell<1:p.error('--per-cell must be positive')
 if args.mode=='baseline':return baseline_run(args,p)
 if args.mode=='recover':
  if not args.input:p.error('recover needs --input; creates a new output and preserves the original')
  preflight_run(args.output_root,p)
  other=experiment_processes()
  if other:p.error('other experiment active; not recovering: '+json.dumps(other))
  original=Path(args.input).resolve();manifest=json.loads((original/'manifest.json').read_text())
  job=unique_output(args.output_root,'intensity_recovery')
  # Copy, never hardlink: summary/result writers must not mutate the original job.
  shutil.copytree(original/'run',job/'run');run=job/'run'
  for name in ('frozen_rules.json','baseline_reference.json'):shutil.copy2(original/name,job/name)
  write_json(job/'parent_manifest.json',manifest)
  manifest=dict(manifest,recovered_from=str(original),implementation=implementation(),recovery_policy='retain completed results, analyze existing captures, execute only uncaptured plans')
  write_json(job/'manifest.json',manifest)
  cfg=json.loads((run/'config.json').read_text());baseline=json.loads((job/'baseline_reference.json').read_text())
  for f in run.glob('trial_*/plan.json'):
   plan=json.loads(f.read_text());plan['rules_file']=str(job/'frozen_rules.json');write_json(f,plan)
  return execute_job(job,run,cfg,baseline,manifest)
 if not args.baseline:p.error('provide a real uninjected --baseline; injected reference data is not accepted')
 baseline_path=Path(args.baseline).resolve();cfg=config()
 baseline=json.loads(baseline_path.read_text()) if baseline_path.is_file() else calibrate(baseline_path,cfg)
 if not baseline.get('mean_completed_task_s') or 'mean_broadcast_error_m' not in baseline:p.error('baseline missing required calibrated metrics')
 packet=json.loads(Path(args.rules).read_text());cfg=packet['config']
 budget=args.task_budget if args.task_budget is not None else (45. if args.mode=='pilot' else max(250.,2*baseline['mean_completed_task_s']+cfg['startup_budget_s']))
 cfg['task_budget_s']=budget
 targets=args.targets or (['delay_timeout','planning_failure'] if args.mode=='pilot' else list(TARGETS))
 n=args.per_cell or (2 if args.mode=='pilot' else 50)
 if budget<=cfg['injection_window_s'][1]:p.error('task budget must cover unchanged native rule window')
 if args.mode!='plan':
  preflight_run(args.output_root,p)
  other=experiment_processes()
  if other:p.error('other experiment active; not starting: '+json.dumps(other))
 job=unique_output(args.output_root,'intensity_'+args.mode);run=job/'run';run.mkdir()
 write_json(job/'frozen_rules.json',packet);write_json(job/'baseline_reference.json',baseline);write_json(run/'config.json',cfg)
 manifest=dict(mode=args.mode,targets=targets,groups=args.groups,per_cell=n,total=len(targets)*len(args.groups)*n,
     seed=args.seed,paired_schedule_seeds=True,native_rng_fully_controlled=False,source_rules=str(Path(args.rules).resolve()),rules_sha256=digest(args.rules),
     source_baseline=str(baseline_path),baseline=baseline,task_budget_s=budget,pilot_censored=budget<2*baseline['mean_completed_task_s'],
     implementation=implementation(),fixed_delay_targets_s=dict(G1=.2,G2=.4,G3=.6,G4=.8,G5=None),
     G5='release before next source broadcast publication including native 0.1s heartbeat; never fixed 1s',
     message_channel='/planning/swarm_traj_send -> /planning/swarm_traj',max_injections_per_trial=None,overlap='release predecessor and forward successor without re-attacking successor',
     contact_feedback_available=False,collision_geometry='configured 0.199m sphere proxy; existing evaluation definition, not measured airframe',
     planning_failure='actual broadcast future paths only',deadlock='unfinished task 30s, includes IDLE, existing 0.05m/s and 0.2m tolerance',
     task_timeout='completed duration > 2x measured uninjected mean; unfinished separately recorded',drift_primary=None,
     drift_candidates=['distance','count','rate'],ASR='observed valid successes / completed-injection rounds; unknowns reported and bounded',
     controls_automatically_added=False)
 write_json(job/'manifest.json',manifest)
 for plan in plans(targets,args.groups,n,args.seed,packet):
  plan['rules_file']=str(job/'frozen_rules.json');d=run/plan['trial_id'];d.mkdir();write_json(d/'plan.json',plan)
 print(job,flush=True)
 if args.mode=='plan':return 0
 return execute_job(job,run,cfg,baseline,manifest,announce=False)

def execute_job(job,run,cfg,baseline,manifest,announce=True,analyzer=analyze,summarizer=summarize,fingerprinter=implementation):
 if announce:print(job,flush=True)
 os.nice(10);os.environ['RACER_EVAL_MIN_AVAILABLE_MB']='768';failures=0
 finished=len(list(run.glob('trial_*/intensity_result.json')))
 try:
  for f in sorted(run.glob('trial_*/plan.json')):
   if (f.parent/'intensity_result.json').exists():continue
   current=resources(job)
   if fingerprinter()!=manifest['implementation']:raise RuntimeError('implementation_changed; planned remainder not run')
   while experiment_processes() or current['mem_available_bytes']<1536*1024**2 or current['disk_free_bytes']<9*1024**3:
    atomic_json(job/'progress.json',dict(state='waiting_resources',finished=finished,planned=manifest['total'],resources=current))
    time.sleep(30);current=resources(job)
   atomic_json(job/'progress.json',dict(state='running',finished=finished,planned=manifest['total'],current=f.parent.name))
   if not (f.parent/'capture.json').exists():execute_capture(f.parent,cfg)
   if not (f.parent/'capture.json').exists():raise RuntimeError('capture process exited without capture.json: '+str(f.parent))
   result=analyzer(f.parent,cfg,baseline);write_json(f.parent/'result.json',result);finished+=1
   print(f"{finished}/{manifest['total']} {result['target']} {result['group']} {result['status']} injections={result['completed_injection_n']}",flush=True)
   cleanup_owned(f.parent);archive_finished(run);summarizer(job)
   failures=failures+1 if result['status'] not in ('completed','task_budget_exhausted') else 0
   if failures>=3:raise RuntimeError('three consecutive capture failures; no automatic outcome-based replacement')
  atomic_json(job/'progress.json',dict(state='finished',finished=finished,planned=manifest['total']))
 except BaseException as exc:
  atomic_json(job/'progress.json',dict(state='failed_or_interrupted',finished=finished,planned=manifest['total'],error=repr(exc)));raise
 finally:summarizer(job)
 return 0

if __name__=='__main__':raise SystemExit(main())
