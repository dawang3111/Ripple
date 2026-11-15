"""Report per-target P(rule|hit) and P(rule|miss) on the internal holdout."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np

from .analysis import matrix, reports, rule_mask, wilson
from .core import TARGETS, unique_output, write_json
from .readable import DEFINITIONS,NAMES


def cohort_metrics(x,y,conditions):
    """Missing rule fields exclude a sample from both rule-specific denominators."""
    mask, eligible=rule_mask(x,conditions)
    result={}
    for name, subset in [('hit',y),('miss',~y)]:
        n=int((eligible&subset).sum()); k=int((mask&subset).sum());lo,hi=wilson(k,n)
        result.update({name+'_n':n,name+'_trigger_n':k,name+'_trigger_rate':k/n if n else None,
                       name+'_ci_low':lo,name+'_ci_high':hi,name+'_missing_n':int(subset.sum())-n})
    a,b=result['hit_trigger_rate'],result['miss_trigger_rate']
    result['trigger_difference_pp']=100*(a-b) if a is not None and b is not None else None
    result['trigger_difference_ci_low_pp']=100*(result['hit_ci_low']-result['miss_ci_high']) if a is not None and b is not None else None
    result['trigger_difference_ci_high_pp']=100*(result['hit_ci_high']-result['miss_ci_low']) if a is not None and b is not None else None
    result['low_cohort_support']=min(result['hit_n'],result['miss_n'])<20 or min(result['hit_trigger_n'],result['miss_trigger_n'])<5
    return result


def translated(conditions,units):
    text=[];mechanisms=[]
    for c in conditions:
        field,op,v=c['field'],c['op'],c['threshold']
        name=DEFINITIONS[field];unit=units.get(field,'')
        if unit=='rad':v=math.degrees(v);unit='度'
        elif unit=='count':unit='次' if 'count_5s' in field or 'publication_5s' in field else '个'
        if unit not in ('s','m','m/s','m/s^2','度','次','个',''):
            unit={'n':'个'}.get(unit,unit)
        # Integer observations permit exact simplification of fractional cluster cuts.
        if units.get(field) in ('count','n'):
            v=math.floor(v) if op=='<=' else math.floor(v)+1
            op='<=' if op=='<=' else '>='
            value=str(v)
        else:value=f'{v:.6g}'
        text.append(f'{name} {op} {value} {unit}'.strip())
        mechanism=('未来机间距' if 'future_3s_min_inter' in field else '未来障碍距离' if 'future_3s_min_obstacle' in field
            else '轨迹偏差' if 'trajectory_deviation' in field else '机间距变化' if 'distance_change' in field
            else '当前机间距' if 'inter_drone_distance' in field else '障碍距离' if 'obstacle' in field
            else '任务阶段' if field=='task_elapsed' else '任务进度' if 'grid_count' in field or 'frontier_count' in field
            else '重规划' if 'replan' in field else '轨迹发布' if 'new_trajectory' in field else '加速度' if 'acceleration' in field
            else '速度' if 'velocity' in field else '航向' if 'heading' in field else '五秒位移' if 'displacement' in field else '广播缓存偏差')
        if mechanism not in mechanisms:mechanisms.append(mechanism)
    return ' + '.join(mechanisms),' 且 '.join(text)


def save_csv(path,rows):
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-run',required=True)
    p.add_argument('--drift-analysis',required=True)
    p.add_argument('--output-root',required=True)
    p.add_argument('--per-target',type=int,default=3)
    args=p.parse_args(argv);os.nice(10)
    run,drift=Path(args.input_run).resolve(),Path(args.drift_analysis).resolve()
    output=unique_output(args.output_root,'rule_groups')
    cfg=json.loads((drift/'selected_config.json').read_text())
    with (drift/'drift_labels_selected.csv').open() as f:labels={r['trial_id']:r for r in csv.DictReader(f)}
    trials=[json.loads(f.read_text()) for f in sorted(run.glob('trial_*/result.json'))]
    for t in trials:
        r=labels[t['trial_id']];hit=r['drift_hit'].lower()=='true';valid=r['drift_valid'].lower()=='true'
        t['labels']['drift']=dict(hit=hit,valid=valid,missing_reason=r['missing_reason'],
             evidence=[json.loads(r['evidence'])] if r['evidence'] else [],count=1 if hit else 0)
    summary=reports(output,trials,cfg)
    prep=json.loads((output/'preprocessing.json').read_text());by_id={t['trial_id']:t for t in trials}
    with (output/'rules_per_target.csv').open() as f:rules=list(csv.DictReader(f))
    units={k:v['unit'] for t in trials if t.get('features') for k,v in t['features']['fields'].items()}
    details=[]
    for r in rules:
        if not r['conditions']:continue
        target=r['target'];conditions=json.loads(r['conditions']);fields=prep[target]['fields']
        rule=tuple((fields.index(c['field']),c['op'],c['threshold']) for c in conditions)
        train=[by_id[i] for i in prep[target]['training_ids']];holdout=[by_id[i] for i in prep[target]['holdout_ids']]
        vx=matrix(holdout,fields);vy=np.asarray([t['labels'][target]['hit'] for t in holdout],dtype=bool)
        tx=matrix(train,fields);ty=np.asarray([t['labels'][target]['hit'] for t in train],dtype=bool)
        metrics=cohort_metrics(vx,vy,rule);training=cohort_metrics(tx,ty,rule)
        # Cross-check against the existing validator; do not swap P(hit|rule) with P(rule|hit).
        assert metrics['hit_trigger_n']==int(r['target_hit_n'])
        assert metrics['miss_trigger_n']==int(r['nonhit_trigger_n'])
        assert metrics['hit_n']+metrics['miss_n']==int(r['validation_n'])
        mechanism,expression=translated(conditions,units)
        details.append(dict(target=target,target_name=NAMES[target],rule_id=r['rule_id'],mechanism=mechanism,rule_group=expression,
            expression_exact=r['expression'],conditions=r['conditions'],**metrics,
            train_hit_trigger_rate=training['hit_trigger_rate'],train_miss_trigger_rate=training['miss_trigger_rate'],
            train_hit_n=training['hit_n'],train_miss_n=training['miss_n'],
            confidence='low' if metrics['low_cohort_support'] or r['confidence']=='low' else 'exploratory',
            training_direction='hit_enriched' if (training['trigger_difference_pp'] or 0)>0 else 'miss_enriched',
            holdout_direction='hit_enriched' if (metrics['trigger_difference_pp'] or 0)>0 else 'miss_enriched',
            validation_scope='internal holdout; drift outcome threshold tuned on full data' if target=='drift' else 'internal holdout',
            control_n=0,control_trigger_rate='NA'))
    save_csv(output/'rule_cohort_rates_all.csv',details)
    # Descriptive display selection is independent of holdout outcomes.
    selected=[]
    for target in TARGETS:
        seen=set();count=0
        target_rules=[r for r in details if r['target']==target]
        adequate=lambda r:min(r['train_hit_n'],r['train_miss_n'])>=20
        for r in [r for r in target_rules if adequate(r)]+[r for r in target_rules if not adequate(r)]:
            signature=tuple(sorted(c['field'] for c in json.loads(r['conditions'])))
            if signature in seen:continue
            seen.add(signature);selected.append(r);count+=1
            if count>=args.per_target:break
    save_csv(output/'rule_cohort_rates_display.csv',selected)
    names=sorted({c['field'] for r in details for c in json.loads(r['conditions'])})
    save_csv(output/'field_definitions.csv',[dict(field=n,name=DEFINITIONS[n],unit=units[n]) for n in names])
    manifest=dict(input_run=str(run),drift_analysis=str(drift),drift_definition=cfg['thresholds'],
        data_scope='actual injected, contract valid, target valid, pre-injection features only',
        rates='P(rule | target hit, rule fields observed) and P(rule | target miss, rule fields observed)',
        display_selection=f'training hit/miss observed n>=20 first, then first {args.per_target} distinct field combinations in original training rank; no holdout ranking',
        threshold_source='training percentiles and training-only KMeans centers',seed=cfg['analysis_seed'],
        holdout_fraction=cfg['holdout_fraction'],control_n=0,causal_claim=False,
        drift_independent_validation=False,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    write_json(output/'manifest.json',manifest)
    lines=['# 六目标规则组：命中组与未命中组触发率','','列的分母为规则字段齐全的该目标命中/未命中留出样本。不是 P(目标命中|规则触发)。',
        '正常列仅指该目标未命中，不代表全六目标正常，也不是未注入对照。缺失不补零，不参与该规则的分母。',
        '训练/留出固定75%/25%；规则只用注入前特征。训练侧 KMeans/分位点选阈值，1–3个条件。',
        '展示规则优先训练命中/未命中字段齐全样本各>=20的规则，再按训练排名；相同字段组合只保留第一条。没有按留出表现排序，完整低样本规则也保留。',
        '命中组触发率低于未命中组的规则为反向关联，不能描述为促进攻击的规则。low 表示样本不足。',
        '漂移定义为缓存位置偏差>0.075m持续>=1s，该定义已经用全量数据标定，因此漂移的内部留出也不是独立验证。',
        'future3s字段要求完整覆盖，可能样本极少；不外推。不伪造急停、三机均值、最大机间距或源机/群体速度比字段。',
        '整数计数阈值采用等价整数表达；其他显示阈值保留6位有效数字，实际触发用 conditions 中完整精度。','',
        '| 目标 | 规则机制 | 规则组 | 命中组触发率 | 未命中组触发率 | 标记 |',
        '| --- | --- | --- | --- | --- | --- |']
    def fmt(k,n):return f'{k/n:.2%} ({k}/{n})' if n else 'NA (0/0)'
    for r in selected:
        lines.append('| '+' | '.join([r['target_name'],r['mechanism'],r['rule_group'],fmt(r['hit_trigger_n'],r['hit_n']),fmt(r['miss_trigger_n'],r['miss_n']),r['confidence']])+' |')
    (output/'README.md').write_text('\n'.join(lines)+'\n')
    print(output,flush=True)
    print(json.dumps(selected,ensure_ascii=False),flush=True)
    return 0

if __name__=='__main__':raise SystemExit(main())
