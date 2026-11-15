"""Deterministic descriptive KMeans and train-only candidate selection.

No held-out label or feature is used to choose thresholds or rank candidates.
Numpy Lloyd KMeans uses seeded k-means++ starts (10 restarts, 100 iterations).
"""
import csv
import json
import math
from collections import Counter
from itertools import combinations
from pathlib import Path

import numpy as np

from .core import TARGETS


def write_csv(path, rows, fields):
    with Path(path).open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow({k:json.dumps(v,ensure_ascii=False,sort_keys=True) if isinstance(v,(dict,list,tuple)) else v
                        for k,v in row.items() if k in fields})


def wilson(hits, n):
    if not n: return [None,None]
    z=1.959963984540054
    p=hits/n
    center=(p+z*z/(2*n))/(1+z*z/n)
    radius=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/(1+z*z/n)
    return [max(0,center-radius),min(1,center+radius)]


class Transform:
    def fit(self,x):
        self.keep=np.any(np.isfinite(x),axis=0)
        x=x[:,self.keep]
        self.median=np.array([np.median(c[np.isfinite(c)]) for c in x.T])
        filled=np.where(np.isfinite(x),x,self.median)
        self.mean=filled.mean(axis=0)
        self.scale=filled.std(axis=0)
        self.scale[self.scale==0]=1
        return self

    def apply(self,x):
        x=x[:,self.keep]
        return (np.where(np.isfinite(x),x,self.median)-self.mean)/self.scale

    def raw(self,z):
        return z*self.scale+self.mean


def kmeans2(x,seed):
    if len(x)<4: return None,'group_smaller_than_4'
    if x.shape[1]==0 or len(np.unique(x,axis=0))<2: return None,'fewer_than_two_distinct_vectors'
    best=None
    rng=np.random.RandomState(seed)
    for _ in range(10):
        first=int(rng.randint(len(x)))
        weights=np.sum((x-x[first])**2,axis=1)
        second=int(rng.choice(len(x),p=weights/weights.sum()))
        centers=x[[first,second]].copy()
        for _ in range(100):
            labels=np.argmin(np.sum((x[:,None,:]-centers[None,:,:])**2,axis=2),axis=1)
            if len(np.unique(labels))<2: break
            new=np.array([x[labels==i].mean(axis=0) for i in range(2)])
            if np.allclose(new,centers,rtol=0,atol=1e-10):
                centers=new
                break
            centers=new
        labels=np.argmin(np.sum((x[:,None,:]-centers[None,:,:])**2,axis=2),axis=1)
        if len(np.unique(labels))<2: continue
        inertia=float(np.sum((x-centers[labels])**2))
        if best is None or inertia<best[0]: best=(inertia,centers.copy(),labels.copy())
    if best is None: return None,'empty_cluster'
    _,centers,labels=best
    # Canonical lexicographic numbering, explicitly not a risk ranking.
    order=sorted(range(2),key=lambda i:tuple(centers[i]))
    return (centers[order],np.array([order.index(int(i)) for i in labels])) ,''


def matrix(trials,fields):
    return np.array([[r.get('features',{}).get(k,{}).get('value')
                      if r.get('features',{}).get(k,{}).get('value') is not None else np.nan
                      for k in fields] for r in trials],dtype=float).reshape(len(trials),len(fields))


def match(x,conditions):
    out=np.ones(len(x),dtype=bool)
    for col,op,value in conditions:
        # Missing original measurements never satisfy a rule (no imputed rule claims).
        out &= np.isfinite(x[:,col]) & ((x[:,col]<=value) if op=='<=' else (x[:,col]>value))
    return out


def validation_metrics(y,mask,seed):
    n=int(mask.sum()); hits=int(y[mask].sum()); other=y[~mask]
    rate=hits/n if n else None
    reference=float(other.mean()) if len(other) else None
    diff=100*(rate-reference) if rate is not None and reference is not None else None
    lift=rate/reference if rate is not None and reference else None
    ci=wilson(hits,n); other_ci=wilson(int(other.sum()),len(other))
    diff_ci=[100*(ci[0]-other_ci[1]),100*(ci[1]-other_ci[0])] if n and len(other) else [None,None]
    rng=np.random.RandomState(seed)
    lifts=[]
    if n and len(other):
        for _ in range(400):
            a=rng.choice(y[mask],n,replace=True).mean()
            b=rng.choice(other,len(other),replace=True).mean()
            if b>0: lifts.append(float(a/b))
    lift_ci=np.quantile(lifts,[.025,.975]).tolist() if len(lifts)>=380 else [None,None]
    return dict(validation_n=len(y),support_count=n,support_fraction=n/len(y) if len(y) else None,
        target_rate=rate,non_target_rate=1-rate if rate is not None else None,
        target_rate_ci95=ci,non_target_rate_ci95=[1-ci[1],1-ci[0]] if n else [None,None],
        nontrigger_target_rate=reference,percentage_point_difference=diff,difference_ci95=diff_ci,
        lift=lift,lift_ci95=lift_ci,low_confidence=n<30 or hits<5 or n-hits<5 or len(other)<30,
        metric_reason='no_triggered_validation_samples' if not n else ('no_nontrigger_reference' if not len(other) else ''),
        lift_ci_reason='' if lift_ci[0] is not None else 'insufficient_nonzero_reference_bootstraps')


def analyze(trials,output,seed=1729,controls=None):
    output=Path(output)
    controls=controls or []
    fields=sorted({k for r in trials for k in r.get('features',{})})
    summary=[]; assignments=[]; clusters=[]; rules=[]
    preprocessing={}
    for ti,target in enumerate(TARGETS):
        observed=[r for r in trials if r.get('known',{}).get(target,False)]
        eligible=[r for r in observed if r['injection']['actually_injected'] and r['status']!='error']
        reasons=Counter()
        for r in trials:
            if not r.get('known',{}).get(target,False): reasons['target_definition_disabled_or_unobserved']+=1
            elif r['status']=='error': reasons['execution_error']+=1
            elif not r['injection']['actually_injected']: reasons[r['injection'].get('skip_reason') or 'not_injected']+=1
        hits=sum(r['labels'][target] for r in eligible)
        row=dict(target=target,planned_trials=len(trials),observed_trials=len(observed),
                 all_observed_hits=sum(r['labels'][target] for r in observed),valid_trials=len(eligible),hits=hits,
                 rate=hits/len(eligible) if eligible else None,rate_ci95=wilson(hits,len(eligible)),invalid_reasons=dict(reasons))
        summary.append(row)
        if not eligible or not fields:
            row['rule_status']='no_eligible_feature_samples'
            for outcome in [False,True]:
                clusters.append(dict(target=target,purpose='descriptive_all',outcome=outcome,n=0,
                                     skip_reason='empty_group',cluster=''))
            continue
        x=matrix(eligible,fields); y=np.array([r['labels'][target] for r in eligible],dtype=bool)
        # Descriptive-only pipeline fitted on all eligible trials, never reused by rules.
        transform=Transform().fit(x)
        z=transform.apply(x)
        kept=[f for f,k in zip(fields,transform.keep) if k]
        preprocessing[target]={'descriptive':dict(fields=kept,median=transform.median.tolist(),
                               mean=transform.mean.tolist(),scale=transform.scale.tolist())}
        for outcome in [False,True]:
            indexes=np.flatnonzero(y==outcome)
            result,reason=kmeans2(z[indexes],seed+ti)
            if result is None:
                clusters.append(dict(target=target,purpose='descriptive_all',outcome=outcome,n=len(indexes),skip_reason=reason))
                for i in indexes:
                    assignments.append(dict(target=target,episode=eligible[i]['episode'],outcome=outcome,cluster='',skip_reason=reason))
                continue
            centers,labels=result
            for i,label in zip(indexes,labels):
                assignments.append(dict(target=target,episode=eligible[i]['episode'],outcome=outcome,cluster=int(label),skip_reason=''))
            for label in [0,1]:
                ix=indexes[labels==label]
                for j,field in enumerate(fields):
                    raw=x[ix,j]; finite=raw[np.isfinite(raw)]
                    center=transform.raw(centers)[label,kept.index(field)] if field in kept else None
                    clusters.append(dict(target=target,purpose='descriptive_all',outcome=outcome,cluster=label,n=len(ix),
                        field=field,unit=next((r['features'][field]['unit'] for r in eligible if field in r['features']),''),
                        center=center,observed_count=len(finite),missing_count=len(raw)-len(finite),
                        mean=float(finite.mean()) if len(finite) else None,
                        std=float(finite.std()) if len(finite) else None,
                        quantiles=np.quantile(finite,[0,.25,.5,.75,1]).tolist() if len(finite) else [],skip_reason=''))
        # Fixed stratified 70/30 split; rare classes cannot support train+holdout clustering.
        rng=np.random.RandomState(seed+ti); train=[]; test=[]
        for outcome in [False,True]:
            ix=np.flatnonzero(y==outcome); rng.shuffle(ix)
            cut=int(np.floor(.7*len(ix))); train.extend(ix[:cut]); test.extend(ix[cut:])
        train=np.array(sorted(train),dtype=int); test=np.array(sorted(test),dtype=int)
        split={int(i):'train' if i in train else 'holdout' for i in range(len(eligible))}
        for a in assignments:
            if a['target']==target:
                a['split']=split[next(i for i,r in enumerate(eligible) if r['episode']==a['episode'])]
        if any(np.sum(y[train]==v)<4 for v in [False,True]) or len(test)<4:
            row['rule_status']='insufficient_training_class_or_holdout_samples'
            continue
        fit=Transform().fit(x[train]); ztrain=fit.apply(x[train]); atoms=set(); cluster_ok=True
        training_clusters=[]
        for outcome in [False,True]:
            result,reason=kmeans2(ztrain[y[train]==outcome],seed+ti)
            if result is None:
                cluster_ok=False; row['rule_status']='training_cluster_'+reason; break
            centers,labels=result
            training_clusters.append(dict(outcome=outcome,centers=fit.raw(centers).tolist()))
            for j in np.flatnonzero(fit.keep):
                values=x[train,j]; values=values[np.isfinite(values)]
                # Both empirical quantiles and class cluster centers, training data only.
                thresholds=list(np.quantile(values,[.25,.5,.75]))
                thresholds.extend(fit.raw(centers)[:,list(np.flatnonzero(fit.keep)).index(j)])
                for threshold in thresholds:
                    for op in ['<=','>']: atoms.add((int(j),op,float(threshold)))
        preprocessing[target]['training']=dict(fields=[f for f,k in zip(fields,fit.keep) if k],
            median=fit.median.tolist(),mean=fit.mean.tolist(),scale=fit.scale.tolist(),clusters=training_clusters,
            train_episodes=[eligible[i]['episode'] for i in train],holdout_episodes=[eligible[i]['episode'] for i in test])
        if not cluster_ok: continue
        def score(cond):
            mask=match(x[train],cond); count=int(mask.sum())
            if count<3 or count==len(train): return -math.inf
            return float(y[train][mask].mean()-y[train][~mask].mean())
        atom_list=sorted(atoms)
        atom_list=sorted(atom_list,key=lambda a:(-score((a,)),a))[:12]
        candidates=[]
        for length in [1,2,3]:
            for cond in combinations(atom_list,length):
                if len({c[0] for c in cond})!=length: continue
                s=score(cond)
                if np.isfinite(s) and s>0: candidates.append((s,cond))
        candidates=sorted(candidates,key=lambda v:(-v[0],v[1]))[:12]
        row['rule_status']='candidates_selected_on_training_only' if candidates else 'no_positive_training_association'
        for ri,(score_value,cond) in enumerate(candidates):
            expression=' AND '.join('%s %s %.12g'%(fields[j],op,v) for j,op,v in cond)
            metrics=validation_metrics(y[test],match(x[test],cond),seed+ti*100+ri)
            control_x=matrix(controls,fields)
            required=[c[0] for c in cond]
            complete=np.all(np.isfinite(control_x[:,required]),axis=1) if len(controls) else np.array([],dtype=bool)
            control_mask=match(control_x,cond)
            rules.append(dict(target=target,rule_id='%s_%02d'%(target,ri+1),expression=expression,
                conditions=[dict(field=fields[j],op=op,threshold=v) for j,op,v in cond],
                training_support=int(match(x[train],cond).sum()),training_difference=score_value,
                control_samples=len(controls),control_evaluable=int(complete.sum()),
                control_trigger_count=int(control_mask.sum()),
                control_trigger_rate=float(control_mask.sum()/complete.sum()) if complete.sum() else None,
                interpretation='association within injected experiments; not causal',**metrics))
    write_csv(output/'summary_per_target.csv',summary,
        ['target','planned_trials','observed_trials','all_observed_hits','valid_trials','hits','rate','rate_ci95','invalid_reasons','rule_status'])
    write_csv(output/'cluster_assignments.csv',assignments,['target','episode','outcome','cluster','split','skip_reason'])
    write_csv(output/'cluster_summary.csv',clusters,['target','purpose','outcome','cluster','n','field','unit','center','observed_count','missing_count','mean','std','quantiles','skip_reason'])
    write_csv(output/'rules_per_target.csv',rules,['target','rule_id','expression','conditions','training_support','training_difference',
        'validation_n','support_count','support_fraction','target_rate','non_target_rate','target_rate_ci95','non_target_rate_ci95',
        'nontrigger_target_rate','percentage_point_difference','difference_ci95','lift','lift_ci95','lift_ci_reason',
        'low_confidence','metric_reason','control_samples','control_evaluable','control_trigger_count','control_trigger_rate','interpretation'])
    (output/'preprocessing.json').write_text(json.dumps(preprocessing,indent=2,allow_nan=False),encoding='utf-8')
    (output/'analysis.md').write_text('''# TX 时延离线分析

仅描述注入实验内的命中/未命中关联，不代表因果效果。未自动增加控制轮次。
有效关联样本要求实际释放延迟消息、无执行错误、目标定义可观察。未注入轮次保留在 trials，按原因排除；
summary 的 all_observed_hits 另计所有可观察轮次，含注入前规划失败。标签 false 且 known=false 不能当作负例。
每轮完整观察区间保留首次命中证据；事件早于注入时标为 pre_injection。规则标签包括这些事件，不能解释为注入造成。

每目标正负组分别 KMeans(k=2)，小于4例、少于两个不同向量或空组明确跳过。
描述性聚类使用全部有效样本的共同中位数填补、标准化（全缺失列丢弃），簇中心逆变换回原单位。
每字段统计只使用原始非缺失值。簇编号按中心字典序固定，不代表风险等级。

规则采用固定种子的分层70/30训练/留出划分，训练集重新拟合填补、缩放和正负组聚类。
阈值只来自训练集四分位点和训练簇中心；取训练分离度最高的12个单条件，组合1至3个不同字段，
最多保留12条训练差异为正的候选。留出集只评估，不参与规则选择。规则遇到所需字段缺失不触发。
支持度为留出触发数/留出总数；目标发生率和未发生率以触发子集为分母；百分点差和提升率
相对于留出未触发子集。零分母留空。比例区间为Wilson 95%，差值为保守Wilson端点差，
提升率为固定种子400次分层bootstrap百分位区间；零参考率过多时不报告区间。
这些是探索性、未做多重比较校正的区间。触发少于30、正例/负例任一少于5或参考少于30均标低置信度。

控制数据须来自显式提供、语义一致的本模式未注入记录；单独报告完整特征控制样本中的触发率及可评估数。
没有控制数据不估计正常运行差异。旧仓库CSV没有可对齐的注入前向量，不能冒充控制特征。

时间为名义仿真秒，非无线实测时间；队列为新增适配语义。发布周期来自同发送方相邻发布，
延迟为周期乘随机(0,1)比例，在下一次正常发布前释放。任务未开始、周期缺失、约束不满足、
任务提前结束均不记实际注入。被取消的排队事件不算实际释放；实际释放不保证任何接收方使用过旧消息。
特征来自入队前的混合更新阶段快照，每字段保存各来源时间。5秒历史在默认3秒任务预算中缺失。
未来3秒间距、未来障碍物距离、机体航向、急停/局部重规划/新轨迹计数和未来轨迹偏移没有可靠信号，
不输出伪数值；详见manifest的unsupported_features。当前目标偏移不等同完整轨迹偏移。
死锁、漂移若未显式配置则unknown；协议timeout仓库无定义，任务timeout独立使用300 tick预算。
碰撞只使用原始离散点判定，不能证明采样间无碰撞。碰撞结束时仍可同时记录其他已命中的目标。
''',encoding='utf-8')
    return summary
