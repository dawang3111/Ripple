"""Frozen pre-intervention rule matching, used online and in offline validation."""
import itertools
import json
import math
from collections import deque
from pathlib import Path

from .analysis import matrix, rule_mask
from .core import ACTIVE,TARGETS,digest
from .features import extract
import numpy as np


def match_rules(features,rules):
    fields=(features or {}).get('fields',{})
    matched=[];missing=set()
    for rule in rules:
        ok=True
        for condition in rule['conditions']:
            field=fields.get(condition['field'],{})
            value=field.get('value')
            if field.get('missing',True) or value is None or not math.isfinite(value):
                missing.add(condition['field']);ok=False;continue
            if condition['op']=='<=':ok=ok and value<=condition['threshold']
            elif condition['op']=='>':ok=ok and value>condition['threshold']
            else:raise ValueError('unsupported rule operator')
        if ok:matched.append(rule['rule_id'])
    return matched,sorted(missing)


def freeze_groups(source):
    """Choose 2–3 existing accepted rules by TRAINING P(hit | OR group), never holdout."""
    import csv
    source=Path(source)
    parent=json.loads((source/'manifest.json').read_text())
    analysis=Path(parent['source']);meta=json.loads((analysis/'manifest.json').read_text())
    run=Path(meta['input_run']);drift=Path(meta['drift_analysis'])
    cfg=json.loads((drift/'selected_config.json').read_text())
    prep=json.loads((analysis/'preprocessing.json').read_text())
    trials={p.parent.name:json.loads(p.read_text()) for p in run.glob('trial_*/result.json')}
    with (drift/'drift_labels_selected.csv').open() as f:
        for row in csv.DictReader(f):trials[row['trial_id']]['labels']['drift'].update(hit=row['drift_hit']=='True',valid=row['drift_valid']=='True')
    with (source/'accepted_rule_metrics.csv').open() as f:accepted=list(csv.DictReader(f))
    groups={}
    for target in TARGETS:
        options=[dict(rule_id=r['rule_id'],conditions=json.loads(r['conditions']),expression=r['rule_group']) for r in accepted if r['target']==target]
        if len(options)<2:raise ValueError('need at least two existing accepted rules for '+target)
        fields=prep[target]['fields'];train=[trials[i] for i in prep[target]['training_ids']]
        x=matrix(train,fields);y=np.asarray([r['labels'][target]['hit'] for r in train],dtype=bool)
        masks=[]
        for rule in options:
            cond=tuple((fields.index(c['field']),c['op'],c['threshold']) for c in rule['conditions'])
            masks.append(rule_mask(x,cond)[0])
        ranked=[]
        for n in (2,3):
            for inds in itertools.combinations(range(len(options)),n):
                selected=np.logical_or.reduce([masks[i] for i in inds]);support=int(selected.sum())
                if support<20:continue
                hits=int(y[selected].sum());ranked.append((hits/support,support,-n,inds,hits))
        if not ranked:raise ValueError('no supported rule group for '+target)
        precision,support,_,inds,hits=max(ranked,key=lambda r:(r[0],r[1],r[2],tuple(-i for i in r[3])))
        groups[target]=dict(target=target,combine='OR',rules=[options[i] for i in inds],
            training_triggered_n=support,training_hit_n=hits,training_target_rate_given_group=precision,
            selection='maximize training P(target hit | OR group), min training support 20; tie support then fewer conditions',
            limitation='historical association; previously screened rules; not predicted causal success')
    return dict(schema_version=1,source_rules=str(source.resolve()),
        source_hashes={str(p):digest(p) for p in [source/'accepted_rule_metrics.csv',source/'manifest.json',analysis/'preprocessing.json',drift/'selected_config.json',drift/'drift_labels_selected.csv']},
        groups=groups,config=cfg,
        thresholds_frozen=True,clock='same-host monotonic + ROS wall clock',feature_cutoff='before arm decision and message generation')


class OnlineRules:
    def __init__(self,plan,cfg):
        self.plan=plan;self.cfg=cfg;self.events=deque();self.starts={};self.latest_trajectory={}
        self.evaluations=0;self.missing_counts={};self.armed=False;self.last_check=0;self.arm_evidence=[]
    def observe(self,e):
        now=e['observed_ns']
        self.events.append(e)
        if e.get('id')==self.plan['source_id'] and e['kind'] in ('rule_armed','rule_arm_rejected','rule_arm_expired'):
            self.arm_evidence.append(e)
        if e['kind']=='task_started':self.starts[e['id']]=e
        if e['kind']=='trajectory':self.latest_trajectory[e['id']]=e
        while self.events and now-self.events[0]['observed_ns']>int((self.cfg['history_s']+1)*1e9):self.events.popleft()
    def evaluate(self,now_ns,ros_s,active):
        if self.armed or not active or self.plan['source_id'] not in self.starts:return None
        start=self.starts[self.plan['source_id']]
        elapsed=(now_ns-start['mono_ns'])/1e9
        if not self.cfg['injection_window_s'][0]<=elapsed<=self.cfg['injection_window_s'][1]:return None
        if now_ns-self.last_check<100000000:return None
        self.last_check=now_ns
        events=list(self.events)
        known={e['observed_ns'] for e in events}
        events += [e for e in list(self.starts.values())+list(self.latest_trajectory.values()) if e['observed_ns'] not in known]
        features=extract(events,now_ns,ros_s,self.plan['source_id'],self.cfg)
        matched,missing=match_rules(features,self.plan['rule_group']['rules']);self.evaluations+=1
        for f in missing:self.missing_counts[f]=self.missing_counts.get(f,0)+1
        if not matched:return None
        self.armed=True
        return dict(kind='rule_decision',decision_ns=now_ns,expires_ns=now_ns+150000000,
            source_id=self.plan['source_id'],target=self.plan['target'],matched_rule_ids=matched,
            features=features,combine='OR')
