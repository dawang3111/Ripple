"""One detached, interleaved six-target frozen-rule simulation batch."""
import argparse
import csv
import fcntl
import itertools
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

from .analysis import export_trials,wilson
from .background import atomic_json,archive_finished,experiment_processes,fingerprint,resources
from .cli import evaluate_trial,execute_capture,preflight_run
from .core import ROOT,TARGETS,digest,trial_plan,unique_output,write_json
from .process_cleanup import cleanup_owned
from .readable import NAMES
from .rule_online import freeze_groups,match_rules

DEFAULT_RULES=ROOT.parents[1]/'racer_eval_results/rules_gap15_20261111T114032Z_7d17ace8c8084c6a9c7c2f979e14995c'


def plans(per_target,seed,packet,map_path):
    for repetition in range(1,per_target+1):
        for index,target in enumerate(TARGETS):
            number=(repetition-1)*len(TARGETS)+index+1
            p=trial_plan(number,seed,packet['config'])
            p.update(target=target,target_trial=repetition,rule_group=packet['groups'][target],control=False,
                     offset_s=packet['config']['injection_window_s'][0],map_path=str(Path(map_path).resolve()))
            yield p


def validate_rule_trial(result,directory):
    capture=json.loads((directory/'capture.json').read_text()) if (directory/'capture.json').exists() else {}
    decision_path=directory/'rule_decision.json'
    result.update(rule_contract_valid=False,rule_skip_reason='',decision_rule_ids=[],generation_rule_ids=[])
    if not decision_path.exists():
        monitor=capture.get('rule_monitor',{})
        result['rule_skip_reason']=monitor.get('skip_reason','rule_decision_missing')
        return result
    decision=json.loads(decision_path.read_text())
    matched,_=match_rules(decision['features'],result['rule_group']['rules'])
    current,_=match_rules(result.get('features'),result['rule_group']['rules'])
    result.update(decision_rule_ids=matched,generation_rule_ids=current,rule_decision_ns=decision['decision_ns'])
    if not result.get('actual_injected'):
        arm_events=capture.get('rule_monitor',{}).get('arm_evidence',[])
        reason=next((e['kind']+':'+e.get('reason','') for e in reversed(arm_events) if e['kind'] in ('rule_arm_rejected','rule_arm_expired')),result.get('skip_reason') or 'arm_not_released')
    elif not result.get('contract_valid'):
        reason='source_injection_contract_invalid'
    elif not matched or not set(matched).intersection(decision['matched_rule_ids']):
        reason='decision_rule_verification_failed'
    elif not decision['decision_ns']<=result['generated_ns']<=decision['expires_ns']:
        reason='decision_expired_before_generation'
    elif not current:
        reason='rule_no_longer_true_at_message_generation'
    else:
        reason='';result['rule_contract_valid']=True
    result['rule_skip_reason']=reason
    return result


def summarize(results,per_target):
    rows=[]
    for target in TARGETS:
        cohort=[r for r in results if r['target']==target]
        valid=[r for r in cohort if r.get('rule_contract_valid') and r['labels'][target]['valid']]
        hit=sum(r['labels'][target]['hit'] for r in valid);ci=wilson(hit,len(valid))
        rows.append(dict(target=target,target_name=NAMES[target],state='completed' if len(cohort)==per_target else 'running' if cohort else 'pending',
            planned_n=per_target,finished_n=len(cohort),task_completed_n=sum(r['status']=='completed' for r in cohort),
            actual_injected_n=sum(bool(r.get('actual_injected')) for r in cohort),
            valid_rule_injected_n=sum(bool(r.get('rule_contract_valid')) for r in cohort),
            skipped_injection_n=sum(not r.get('actual_injected') for r in cohort),
            invalid_n=len(cohort)-len(valid),valid_target_n=len(valid),hit_n=hit,miss_n=len(valid)-hit,
            target_rate_given_valid_rule_injection=hit/len(valid) if valid else None,ci_low=ci[0],ci_high=ci[1],
            valid_hit_per_planned_trial=hit/per_target,
            failure_n=sum(r['status'] not in ('completed','task_budget_exhausted') for r in cohort)))
    return rows


def save_csv(path,rows):
    temporary=path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    temporary.replace(path)


def worker(job):
    signal.signal(signal.SIGHUP,signal.SIG_IGN);os.nice(10)
    lock=(job/'worker.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close();raise SystemExit('this job already has a worker')
    settings=json.loads((job/'job.json').read_text());run=job/'run';cfg=json.loads((run/'config.json').read_text())
    os.environ['RACER_EVAL_MIN_AVAILABLE_MB']='768'
    results=[json.loads(p.read_text()) for p in sorted(run.glob('trial_*/result.json'))]
    progress={}
    def publish(state,**extra):
        table=summarize(results,settings['per_target'])
        save_csv(job/'rule_group_results.csv',table)
        progress.update(state=state,pid=os.getpid(),updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
            job_dir=str(job),planned_rounds=settings['per_target']*6,finished_rounds=len(results),
            actual_injected_rounds=sum(bool(r.get('actual_injected')) for r in results),
            all_targets_completed=all(r['state']=='completed' for r in table),targets=table,**extra)
        atomic_json(job/'progress.json',progress)
    publish('starting')
    failures=0
    try:
        for directory in sorted(p.parent for p in run.glob('trial_*/plan.json')):
            if (directory/'result.json').exists():continue
            if (directory/'events.jsonl').exists() or (directory/'capture.json').exists():
                raise RuntimeError('partial trial requires review before resume: '+str(directory))
            for result in sorted(run.glob('trial_*/result.json'))[-1:]:cleanup_owned(result.parent)
            archive_finished(run)
            while True:
                if fingerprint()!=settings['implementation']:raise RuntimeError('source_or_binary_changed_during_batch')
                current=resources(job);other=experiment_processes()
                raw_budget=512*1024**2
                for marker in run.glob('trial_*/archive.json'):
                    a=json.loads(marker.read_text());raw_budget=max(raw_budget,2*sum(x['raw_bytes'] for x in a['files']))
                reason=('other_experiments:'+json.dumps(other) if other else
                    'memory_before_round' if current['mem_available_bytes']<1536*1024**2 else
                    'disk_before_round' if current['disk_free_bytes']<8*1024**3+raw_budget else None)
                publish('waiting_resources' if reason else 'running',resources=current,conflict=reason,next_trial=directory.name)
                if reason is None:break
                time.sleep(30)
            execute_capture(directory,cfg)
            result=validate_rule_trial(evaluate_trial(directory,cfg),directory)
            write_json(directory/'result.json',result);results.append(result)
            export_trials(run,results,cfg)
            save_csv(job/'rule_matches.csv',[dict(trial_id=r['trial_id'],target=r['target'],source_id=r['source_id'],
                actual_injected=r.get('actual_injected'),rule_contract_valid=r.get('rule_contract_valid'),
                decision_rule_ids=json.dumps(r.get('decision_rule_ids',[])),generation_rule_ids=json.dumps(r.get('generation_rule_ids',[])),
                rule_skip_reason=r.get('rule_skip_reason',''),result_path=r['result_path']) for r in results])
            publish('running',next_trial=None)
            print(f"{len(results)}/{settings['per_target']*6} {result['target']} {result['status']} injected={result.get('actual_injected')} rule_valid={result.get('rule_contract_valid')} target_hit={result['labels'][result['target']]['hit']}",flush=True)
            failures=failures+1 if result['status'] not in ('completed','task_budget_exhausted') else 0
            if failures>=3:raise RuntimeError('three_consecutive_capture_failures; results retained; resume available after repair')
        for result in sorted(run.glob('trial_*/result.json'))[-1:]:cleanup_owned(result.parent)
        archive_finished(run)
        publish('finished',next_trial=None)
    except BaseException as exc:
        publish('interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed',error=repr(exc))
        raise
    finally:
        table=summarize(results,settings['per_target'])
        lines=['# 六目标冻结规则组隔离仿真结果','',
            f"每目标计划{settings['per_target']}轮；目标之间交错执行。组内OR，规则和目标定义在启动前冻结。",
            '实际注入仅认源端release；规则有效要求决策和实际消息生成时规则均满足，命令未过期，原始注入协议有效。',
            '未触发、缺失和无效轮次不计为攻击成功；每轮六标签独立记录。分组效果仅按被分配目标统计。',
            '新批次为独立种子的新运行；没有配对控制，历史随机注入率只能作描述比较，不能证明因果提升。',
            '原有漂移0.5m/1s定义未改写；本批次使用冻结的探索性漂移0.075m/1s定义。','',
            '| 目标 | 状态 | 已结束/计划 | 实际注入 | 有效目标样本 | 命中 | 发生率 |','| --- | --- | --- | --- | --- | --- | --- |']
        for r in table:
            rate=f"{r['target_rate_given_valid_rule_injection']:.2%}" if r['valid_target_n'] else 'NA'
            lines.append(f"| {r['target_name']} | {r['state']} | {r['finished_n']}/{r['planned_n']} | {r['actual_injected_n']} | {r['valid_target_n']} | {r['hit_n']} | {rate} |")
        lines += ['', '全部六目标计划执行完成：'+str(all(r['state']=='completed' for r in table))]
        (job/'report.md').write_text('\n'.join(lines)+'\n')
        lock.close()
    return 0


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--per-target',type=int,default=100)
    p.add_argument('--seed',type=int,default=20261111)
    p.add_argument('--rules',default=str(DEFAULT_RULES))
    p.add_argument('--output-root',default=str(ROOT.parents[1]/'racer_eval_results'))
    p.add_argument('--status');p.add_argument('--resume');p.add_argument('--worker',help=argparse.SUPPRESS)
    args=p.parse_args(argv)
    if args.worker:return worker(Path(args.worker).resolve())
    if args.status:
        job=Path(args.status).resolve()
        if not (job/'progress.json').exists():
            print('后台启动中；结果目录：'+str(job));return 0
        s=json.loads((job/'progress.json').read_text())
        print(f"状态 {s['state']}；已结束 {s['finished_rounds']}/{s['planned_rounds']}；实际注入 {s['actual_injected_rounds']}")
        for r in s['targets']:
            rate=f"{r['target_rate_given_valid_rule_injection']:.2%}" if r['valid_target_n'] else 'NA'
            print(f"{r['target_name']}: {r['state']} {r['finished_n']}/{r['planned_n']}；实际注入{r['actual_injected_n']}；命中{r['hit_n']}/{r['valid_target_n']} ({rate})")
        print('全部六目标已执行完：'+str(s['all_targets_completed']));print('结果目录：'+str(job))
        if s.get('conflict'):print('等待原因：'+s['conflict'])
        if s.get('error'):print('错误：'+s['error'])
        return 0
    if args.per_target<1:p.error('--per-target must be positive')
    preflight_run(args.output_root,p)
    other=experiment_processes()
    if other:p.error('other experiments active; not starting: '+json.dumps(other))
    root=Path(args.output_root).resolve();ancestor=root
    while not ancestor.exists():ancestor=ancestor.parent
    current=resources(ancestor)
    if current['mem_available_bytes']<1536*1024**2 or current['disk_free_bytes']<9*1024**3:p.error('insufficient resources: '+json.dumps(current))
    with socket.socket() as sock:sock.bind(('127.0.0.1',0))
    if args.resume:
        job=Path(args.resume).resolve();settings=json.loads((job/'job.json').read_text())
        if fingerprint()!=settings['implementation']:p.error('resume source/binary changed; review required')
    else:
        packet=freeze_groups(args.rules)
        job=unique_output(root,'rule_batch');run=job/'run';run.mkdir()
        write_json(job/'frozen_rules.json',packet);write_json(run/'config.json',packet['config'])
        group_rows=[dict(target=t,rule_id=r['rule_id'],combine='OR',rule=r['expression'],
                         training_observed_target_rate=g['training_target_rate_given_group'],training_support_n=g['training_triggered_n'])
                    for t,g in packet['groups'].items() for r in g['rules']]
        save_csv(job/'frozen_rule_groups.csv',group_rows)
        map_path=ROOT/'uav_simulator/map_generator/resource/pillar.pcd'
        for plan in plans(args.per_target,args.seed,packet,map_path):
            plan['dataset_id']=job.name;directory=run/plan['trial_id'];directory.mkdir();write_json(directory/'plan.json',plan)
        settings=dict(per_target=args.per_target,trials=args.per_target*6,seed=args.seed,implementation=fingerprint(),rules=str(Path(args.rules).resolve()),
            rules_sha256=digest(job/'frozen_rules.json'),map_path=str(map_path),map_sha256=digest(map_path),resources_before=current,
            reserve_disk_gib=8,min_start_available_mb=1536,min_runtime_available_mb=768,nice=10,
            controls_automatically_added=False,scope='isolated simulation only',group_combine='OR',rule_arm_ttl_ms=150,
            runtime_target_budget_s=packet['config']['task_budget_s'],independent_seeds_from_previous_batch=True)
        write_json(job/'job.json',settings);write_json(job/'manifest.json',dict(settings,parameters=vars(args),frozen_config=packet['config']))
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    env['PYTHONPATH']=str(ROOT/'tools')+os.pathsep+env.get('PYTHONPATH','')
    log=job/('background_'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'.log')
    with log.open('x') as f:
        child=subprocess.Popen([sys.executable,'-u','-m','latency_eval.rule_batch','--worker',str(job)],stdin=subprocess.DEVNULL,
            stdout=f,stderr=subprocess.STDOUT,env=env,start_new_session=True)
    (job/'background.pid').write_text(str(child.pid)+'\n')
    print(json.dumps(dict(pid=child.pid,job_dir=str(job),per_target=settings['per_target'],total=settings['trials'],seed=settings['seed'],log=str(log)),ensure_ascii=False))
    return 0

if __name__=='__main__':raise SystemExit(main())
