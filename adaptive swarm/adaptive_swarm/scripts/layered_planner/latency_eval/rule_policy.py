"""Online state-only gate; no outcome, future trajectory, or simulation import."""
import math


def rule_matches(features,rule,now):
    for condition in rule['conditions']:
        meta=features.get(condition['field'],{})
        value=meta.get('value')
        if value is None or not math.isfinite(value):return False
        if meta.get('observed_time',math.inf)>now+1e-9:
            raise ValueError('future observation in rule gate')
        def times(value):
            if isinstance(value,dict):
                for child in value.values():yield from times(child)
            elif isinstance(value,(list,tuple)):
                for child in value:yield from times(child)
            elif isinstance(value,(int,float)):yield value
        if any(t>now+1e-9 for t in times(meta.get('source_times',{}))):
            raise ValueError('future source timestamp in rule gate')
        op=condition['op'];threshold=condition['threshold']
        if op not in ('<=','>'):raise ValueError('unsupported rule operator')
        if not (value<=threshold if op=='<=' else value>threshold):return False
    return bool(rule['conditions'])


class RuleGate:
    def __init__(self,group,sender_id,start_tick):
        self.group=group;self.sender_id=sender_id;self.start_tick=start_tick
        self.observations=0;self.selected=None

    def should_observe(self,sender,tick,period):
        return (self.selected is None and sender==self.sender_id and tick>=self.start_tick
                and period is not None and math.isfinite(period) and period>0)

    def decide(self,features,now,tick,sp_index):
        if self.selected is not None:return False
        self.observations+=1
        matched=[r['rule_id'] for r in self.group['rules'] if rule_matches(features,r,now)]
        if not matched:return False
        self.selected=dict(time_s=now,tick=tick,matched_rule_ids=matched,reference_sp_index=sp_index)
        return True

    def skip_reason(self):
        return 'rule_group_not_triggered' if self.observations else 'rule_gate_start_not_reached'

    def report(self):
        return dict(group_id=self.group['group_id'],target=self.group['target'],logic='OR',
            sender_id=self.sender_id,start_tick=self.start_tick,observations=self.observations,selected=self.selected)
