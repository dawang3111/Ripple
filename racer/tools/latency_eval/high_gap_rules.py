"""Train positive rule mechanisms, then screen internal holdout for >=20 pp gap."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import numpy as np
from .analysis import matrix,rule_mask
from .core import TARGETS,unique_output,write_json
from .readable import NAMES
from .rule_cohorts import cohort_metrics,translated,save_csv


def mine(x,y,centers,min_group=40,min_gap=.20,beam_width=48,max_candidates=16):
    atoms=[]
    for col in range(x.shape[1]):
        values=x[np.isfinite(x[:,col]),col]
        if len(np.unique(values))<2:continue
        cuts=list(np.percentile(values,[10,20,30,40,50,60,70,80,90]))
        cuts += [c[col] for c in centers]
        atoms += [((col,op,float(v)),) for v in sorted(set(cuts)) for op in ('<=','>')]
    cache={}
    def score(rule):
        if rule in cache:return cache[rule]
        mask,eligible=rule_mask(x,rule)
        a=eligible&y;b=eligible&~y;nh=int(a.sum());nm=int(b.sum())
        if min(nh,nm)<min_group or min(int(mask.sum()),int((eligible&~mask).sum()))<20:
            value=-1.
        else:
            gap=float(mask[a].mean()-mask[b].mean())
            value=gap*math.sqrt((nh+nm)/len(x)) if gap>0 else -1.
        cache[rule]=value
        return value
    ranked=sorted(((score(r),r) for r in atoms),key=lambda v:(-v[0],v[1]))
    # Only training data determine atoms, thresholds, beam and shortlist.
    base=[r for score0,r in ranked[:beam_width] if score0>0]
    beam=list(base);pool=list(base)
    for size in (2,3):
        expanded={tuple(sorted(r+a)) for r in beam for a in base if a[0][0] not in {c[0] for c in r} and len(r+a)==size}
        beam=[r for s,r in sorted(((score(r),r) for r in expanded),key=lambda v:(-v[0],v[1]))[:beam_width] if s>0]
        pool.extend(beam)
    shortlisted=[];masks=[]
    for r in sorted(set(pool),key=lambda r:(-score(r),len(r),r)):
        mask,eligible=rule_mask(x,r)
        metrics=cohort_metrics(x,y,r)
        if metrics['trigger_difference_pp'] is None or metrics['trigger_difference_pp'] < min_gap*100:continue
        # Avoid spending holdout checks on near-identical training triggers.
        signature=mask.tobytes()+eligible.tobytes()
        if signature in masks:continue
        masks.append(signature);shortlisted.append((r,metrics,score(r)))
        if len(shortlisted)>=max_candidates:break
    return shortlisted


def passes(m,min_gap=20.,min_group=20):
    return (min(m['hit_n'],m['miss_n'])>=min_group and m['hit_trigger_n']>=5 and
            m['trigger_difference_pp'] is not None and m['trigger_difference_pp']>=min_gap and
            m['trigger_difference_ci_low_pp'] is not None and m['trigger_difference_ci_low_pp']>0)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rule-analysis',required=True);p.add_argument('--output-root',required=True)
    p.add_argument('--min-gap-pp',type=float,default=20.)
    p.add_argument('--beam-width',type=int,default=48)
    p.add_argument('--max-candidates',type=int,default=16)
    args=p.parse_args(argv)
    if not 0<args.min_gap_pp<100 or args.beam_width<1 or args.max_candidates<1:p.error('invalid gap or search limits')
    os.nice(10)
    source=Path(args.rule_analysis).resolve();parent=json.loads((source/'manifest.json').read_text())
    run=Path(parent['input_run']);drift=Path(parent['drift_analysis'])
    output=unique_output(args.output_root,'rules_gap'+format(args.min_gap_pp,'g').replace('.','p'))
    with (drift/'drift_labels_selected.csv').open() as f:labels={r['trial_id']:r for r in csv.DictReader(f)}
    trials=[json.loads(f.read_text()) for f in run.glob('trial_*/result.json')]
    for t in trials:
        r=labels[t['trial_id']];t['labels']['drift'].update(hit=r['drift_hit'].lower()=='true',valid=r['drift_valid'].lower()=='true')
    by_id={t['trial_id']:t for t in trials}
    prep=json.loads((source/'preprocessing.json').read_text())
    with (source/'cluster_summary.csv').open() as f:clusters=list(csv.DictReader(f))
    units={k:v['unit'] for t in trials if t.get('features') for k,v in t['features']['fields'].items()}
    policy=dict(gap_definition='P(rule|hit, observed) minus P(rule|miss, observed)',minimum_difference_pp=args.min_gap_pp,
        min_training_group_n=40,min_holdout_group_n=20,min_holdout_hit_trigger_n=5,holdout_difference_ci_low_must_exceed_zero=True,
        training_quantiles=[10,20,30,40,50,60,70,80,90],beam_width=args.beam_width,max_conditions=3,max_locked_candidates_per_target=args.max_candidates,max_exported_per_target=3,
        split='reuse existing 75/25 split and seed 20261103',thresholds_from='training quantiles and existing training KMeans centers',
        interpretation='post-hoc exploratory internal holdout screening; prior holdout already inspected; no independent-validation claim',
        drift_definition='receiver cache error >0.075m for >=1s; already calibrated on all trials',no_simulations=True)
    write_json(output/'manifest.json',dict(source=str(source),policy=policy,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    details=[];selected=[];summary=[]
    print(output,flush=True)
    stems=dict(zip(TARGETS,['drone_drone_collision','drone_obstacle_collision','deadlock','delay_timeout','planning_failure','drift']))
    for target in TARGETS:
        fields=prep[target]['fields'];train=[by_id[i] for i in prep[target]['training_ids']];holdout=[by_id[i] for i in prep[target]['holdout_ids']]
        tx=matrix(train,fields);ty=np.asarray([t['labels'][target]['hit'] for t in train],dtype=bool)
        vx=matrix(holdout,fields);vy=np.asarray([t['labels'][target]['hit'] for t in holdout],dtype=bool)
        centers=[np.asarray([json.loads(c['centers'])[f] for f in fields]) for c in clusters if c['target']==target and c['status']=='ok']
        shortlist=mine(tx,ty,centers,min_gap=args.min_gap_pp/100,beam_width=args.beam_width,max_candidates=args.max_candidates)
        write_json(output/(target+'_locked_training_candidates.json'),[dict(conditions=[dict(field=fields[c],op=op,threshold=v) for c,op,v in r],training_metrics=m,training_score=s) for r,m,s in shortlist])
        accepted=[];seen=set()
        for index,(rule,training,score) in enumerate(shortlist,1):
            metrics=cohort_metrics(vx,vy,rule);conditions=[dict(field=fields[c],op=op,threshold=v) for c,op,v in rule]
            mechanism,expression=translated(conditions,units)
            row=dict(target=target,rule_id=f'{target}_{index:02d}',mechanism=mechanism,rule_group=expression,
                conditions=json.dumps(conditions,ensure_ascii=False),training_rank=index,training_gap_pp=training['trigger_difference_pp'],
                train_hit_n=training['hit_n'],train_miss_n=training['miss_n'],**metrics,accepted=passes(metrics,min_gap=args.min_gap_pp))
            details.append(row)
            signature=tuple(sorted(c['field'] for c in conditions))
            if row['accepted'] and signature not in seen and len(accepted)<3:
                accepted.append(row);selected.append(row);seen.add(signature)
        cols=['规则机制','规则组',NAMES[target]+'发生率','正常轮次发生率']
        rows=[[r['mechanism'],r['rule_group'],f"{r['hit_trigger_rate']:.2%}",f"{r['miss_trigger_rate']:.2%}"] for r in accepted]
        if not rows:rows=[['暂无满足条件规则',f'正向差值至少{args.min_gap_pp:g}个百分点且留出样本量与区间检查通过','NA','NA']]
        with (output/(stems[target]+'.csv')).open('x',newline='',encoding='utf-8-sig') as f:
            w=csv.writer(f);w.writerow(cols);w.writerows(rows)
        summary.append(dict(target=target,locked_training_candidates=len(shortlist),holdout_passed=sum(r['accepted'] for r in details if r['target']==target),exported=len(accepted)))
        print(target,summary[-1],flush=True)
    if details:save_csv(output/'candidate_audit.csv',details)
    if selected:save_csv(output/'accepted_rule_metrics.csv',selected)
    save_csv(output/'selection_summary.csv',summary)
    (output/'README.md').write_text(f'仅筛选正向差值 >={args.min_gap_pp:g} 个百分点的规则。两列为 P(规则|目标命中) 与 P(规则|目标未命中)。\n'
        f'候选及阈值来自训练集。内部留出再次筛选，命中/未命中字段齐全样本各>=20、命中触发>=5、差值>={args.min_gap_pp:g}个百分点，保守95%差值区间下界>0。\n'
        '留出数据此前已查看，本次属于事后探索筛选，不是独立验证；置信区间未作多重筛选校正。漂移定义也已事后标定。\n'
        '正常列仅表示该目标未命中；没有未注入控制。未通过的目标保留空表说明，不补造规律。\n'
        '阈值显示保留6位有效数字，实际判断使用 conditions 中完整精度；整数计数按等价整数表达。\n')
    print('DONE',output,flush=True)
    return 0

if __name__=='__main__':raise SystemExit(main())
