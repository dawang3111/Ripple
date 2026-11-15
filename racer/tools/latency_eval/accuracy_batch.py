"""Isolated RACER missed-window accuracy experiment; large runs are explicit."""
import argparse
import copy
import csv
import json
import shutil
import subprocess
import uuid
from pathlib import Path
import numpy as np

from .core import ROOT, TARGETS, digest, unique_output, write_json
from . import intensity_batch as shared
from .accuracy_metrics import WAITS, analyze
from .background import experiment_processes
from .cli import preflight_run


def implementation():
    data = shared.implementation()
    package = ROOT/'swarm_exploration/exploration_manager'
    for path in (package/'include/exploration_manager/accuracy_window_gate.h',
                 package/'include/exploration_manager/fast_exploration_fsm.h'):
        data[str(path)] = digest(path)
    return data


def plans(targets, groups, n, seed, packet):
    for plan in shared.plans(targets, groups, n, seed, packet):
        yield dict(plan, accuracy_mode=True, configured_wait_s=WAITS[plan['group']],
                   standard_packet_strategy='next_source_successor')


def summarize(job):
    job = Path(job)
    isolate_partial_results(job/'run')
    rows = shared.summarize(job)
    results = [json.loads(p.read_text()) for p in sorted((job/'run').glob('trial_*/accuracy_result.json'))]
    windows = [dict(trial_id=r['trial_id'], target=r['target'], group=r['group'], **w)
               for r in results for w in r['windows']]
    shared.csv_write(job/'windows.csv', windows)
    for row in rows:
        cohort = [r for r in results if r['target']==row['target'] and r['group']==row['group']]
        selected = [w for r in cohort for w in r['windows'] if w['actual_wait_s'] is not None]
        waits = [w['actual_wait_s'] for w in selected]
        row.update(configured_wait_s=WAITS[row['group']],
                   window_trigger_n=sum(r['window_trigger_n'] for r in cohort),
                   selected_window_n=len(selected),
                   skipped_original_window_n=sum(w['skipped_original_window'] for r in cohort for w in r['windows']),
                   cancelled_window_n=sum(r['cancelled_window_n'] for r in cohort),
                   window_contract_error_n=sum(len(r['window_errors']) for r in cohort),
                   window_contract_invalid_round_n=sum(bool(r['window_errors']) for r in cohort),
                   actual_wait_mean_s=float(np.mean(waits)) if waits else None,
                   actual_wait_min_s=min(waits) if waits else None,
                   actual_wait_max_s=max(waits) if waits else None,
                   actual_wait_p95_s=float(np.quantile(waits,.95)) if waits else None)
    shared.csv_write(job/'summary_per_target_group.csv', rows)
    lines = ['# RACER攻击准确性实验结果', '',
             '五组使用相同的后继广播释放策略，仅窗口等待为0/1/2/3/5秒。以下是已结束轮次，未执行计划不计结果。', '',
             '|目标|组|结束/计划|实际注入轮|指定目标命中|未知|实际等待均值(s)|包延迟均值(s)|窗口错误|',
             '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        lines.append('|'+ '|'.join(str(v) for v in [r['target'],r['group'],str(r['finished_n'])+'/'+str(r['planned_n']),
                     r['injected_n'],r['primary_hit_n'] if r['target']!='drift' else '未选主判据',
                     r['primary_unknown_n'] if r['target']!='drift' else '见三种候选',
                     r['actual_wait_mean_s'],r['actual_delay_mean_s'],r['window_contract_error_n']])+'|')
    lines += ['', 'ASR分母为实际完成源端延迟的轮数；观测ASR与未知上界分别输出，未知不是失败。',
              '漂移距离、次数、事件率分别保留，未选唯一主判据。未完成任务不能算已完成任务超时。',
              '碰撞为实际位置配置球体/体素几何代理，没有物理接触反馈。规划失败只检查实际广播轨迹。',
              '基准仅3轮，且自身也有死锁/未来轨迹冲突/距离漂移标签；不能据标签命中声称攻击因果效果。',
              '原生心跳会缩短包延迟；等待时间与包延迟是两种独立的测量。',
              '窗口错误轮的目标标签标为未知；实际延迟及原始证据仍保留。',
              '完整参数、源码指纹、规则及基准来源见manifest.json；每次窗口见windows.csv。']
    (job/'report.md').write_text('\n'.join(lines)+'\n')
    return rows


def execute(job, run, cfg, baseline, manifest):
    isolate_partial_results(run)
    return shared.execute_job(job, run, cfg, baseline, manifest, analyzer=analyze,
                              summarizer=summarize, fingerprinter=implementation)


def isolate_partial_results(run):
    # An interrupted audit must not be mistaken for a completed accuracy result.
    # Keep its partial output; recover will reanalyze the saved capture.
    for path in Path(run).glob('trial_*/intensity_result.json'):
        if not (path.parent/'accuracy_result.json').exists():
            path.replace(path.parent/('unverified_outcomes_'+uuid.uuid4().hex+'.json'))
    for path in Path(run).glob('trial_*/accuracy_result.json'):
        alias=path.parent/'intensity_result.json'
        if not alias.exists():shutil.copy2(path,alias)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['plan','pilot','run','summarize','recover','baseline'])
    p.add_argument('--baseline'); p.add_argument('--input')
    p.add_argument('--rules', default=str(shared.DEFAULT_RULES))
    p.add_argument('--targets', nargs='+', choices=TARGETS)
    p.add_argument('--groups', nargs='+', choices=list(WAITS), default=list(WAITS))
    p.add_argument('--per-cell', type=int); p.add_argument('--seed', type=int, default=20261115)
    p.add_argument('--task-budget', type=float)
    p.add_argument('--output-root', default=str(ROOT.parents[1]/'racer_eval_results'))
    args = p.parse_args(argv)
    if args.mode=='summarize':
        if not args.input:p.error('summarize requires --input')
        summarize(args.input); return 0
    if args.per_cell is not None and args.per_cell<1:p.error('--per-cell must be positive')
    if args.mode=='baseline':
        # Identical no-attack native simulation; freeze a new reference only explicitly.
        command=['baseline','--rules',args.rules,'--seed',str(args.seed),'--per-cell',str(args.per_cell or 1),
                 '--output-root',args.output_root]
        if args.task_budget is not None:command+=['--task-budget',str(args.task_budget)]
        return shared.main(command)
    if args.mode!='plan':
        preflight_run(args.output_root,p)
        other=experiment_processes()
        if other:p.error('other experiment active; not starting: '+json.dumps(other))
    if args.mode=='recover':
        if not args.input:p.error('recover requires --input; original job will be preserved')
        old=Path(args.input).resolve();parent=json.loads((old/'manifest.json').read_text())
        if parent.get('experiment')!='accuracy':p.error('not an accuracy job')
        job=unique_output(args.output_root,'accuracy_recovery');run=job/'run'
        shutil.copytree(old/'run',run)
        for name in ('frozen_rules.json','baseline_reference.json'):shutil.copy2(old/name,job/name)
        write_json(job/'parent_manifest.json',parent)
        manifest=dict(parent,recovered_from=str(old),implementation=implementation())
        write_json(job/'manifest.json',manifest)
        for f in run.glob('trial_*/plan.json'):
            plan=json.loads(f.read_text());plan['rules_file']=str(job/'frozen_rules.json');write_json(f,plan)
        # Terminal results stay unchanged; captured rounds are not rerun based on outcomes.
        return execute(job,run,json.loads((run/'config.json').read_text()),
                       json.loads((job/'baseline_reference.json').read_text()),manifest)
    if not args.baseline:p.error('provide a measured uninjected baseline_reference.json')
    baseline_path=Path(args.baseline).resolve();baseline=json.loads(baseline_path.read_text())
    if not baseline.get('mean_completed_task_s') or baseline.get('mean_broadcast_error_m') is None:
        p.error('baseline lacks measured completed duration or broadcast error')
    packet=json.loads(Path(args.rules).read_text());cfg=copy.deepcopy(packet['config'])
    budget=args.task_budget if args.task_budget is not None else (60. if args.mode=='pilot' else max(350.,2*baseline['mean_completed_task_s']+cfg['startup_budget_s']))
    if budget<=cfg['injection_window_s'][1]+max(WAITS.values()):p.error('budget must cover the unchanged rule window plus five-second wait')
    cfg['task_budget_s']=budget
    targets=args.targets or (['delay_timeout'] if args.mode=='pilot' else list(TARGETS))
    n=args.per_cell or (2 if args.mode=='pilot' else 50)
    job=unique_output(args.output_root,'accuracy_'+args.mode);run=job/'run';run.mkdir()
    write_json(job/'frozen_rules.json',packet);write_json(job/'baseline_reference.json',baseline);write_json(run/'config.json',cfg)
    manifest=dict(experiment='accuracy',mode=args.mode,targets=targets,groups=args.groups,per_cell=n,
                  total=len(targets)*len(args.groups)*n,seed=args.seed,wait_targets_s=WAITS,
                  standard_packet_strategy='all groups: hold selected B until next source broadcast C; release B then forward C',
                  trigger='first eligible new PUB_TRAJ event with a fresh frozen-OR rule token',
                  post_wait_selection='first eligible new PUB_TRAJ after t0+wait; no rule recheck',
                  window_eligibility=cfg['injection_window_s'],window_clock='same-host monotonic ns; t0 is eligible publication, decision time stored separately',
                  max_injections_per_trial=None,paired_schedule_seeds=True,native_rng_fully_controlled=False,
                  source_rules=str(Path(args.rules).resolve()),rules_sha256=digest(args.rules),
                  source_baseline=str(baseline_path),baseline_sha256=digest(baseline_path),baseline=baseline,
                  base_commit=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
                  implementation=implementation(),task_budget_s=budget,pilot_censored=budget<2*baseline['mean_completed_task_s'],
                  targets_definition='same intensity_metrics definitions for all groups',drift_primary=None,
                  drift_candidates=['distance','count','rate'],contact_feedback_available=False,
                  task_timeout_threshold_s=2*baseline['mean_completed_task_s'],controls_automatically_added=False,
                  ASR='valid observed hits / completed-source-injection rounds; unknowns and upper bounds separate')
    write_json(job/'manifest.json',manifest)
    for plan in plans(targets,args.groups,n,args.seed,packet):
        plan['rules_file']=str(job/'frozen_rules.json');d=run/plan['trial_id'];d.mkdir();write_json(d/'plan.json',plan)
    if args.mode=='plan':print(job,flush=True);return 0
    return execute(job,run,cfg,baseline,manifest)


if __name__=='__main__':raise SystemExit(main())
