"""Contract for a FUTURE timed-trajectory adapter, not fabricated native data.

2-D geometry in meters; points carry absolute simulation seconds. Register only
actual release events, never internal candidates/history Path/position snapshots.
"""
from copy import deepcopy
import math


def interpolate(points,time):
    if time<points[0]['time_s'] or time>points[-1]['time_s']:return None
    for a,b in zip(points,points[1:]):
        if a['time_s']<=time<=b['time_s']:
            fraction=(time-a['time_s'])/(b['time_s']-a['time_s'])
            return tuple(x+fraction*(y-x) for x,y in zip(a['position'],b['position']))
    return tuple(points[-1]['position'])


class PublishedTrajectories:
    def __init__(self):self.messages={};self.reads=[];self.previous_offset={}

    def on_actual_release(self,event):
        payload=event.get('payload',{})
        if event.get('event')!='release' or not isinstance(payload,dict) or payload.get('kind')!='timed_trajectory_2d':
            raise ValueError('requires an actual released timed future trajectory, not a candidate/history/snapshot')
        points=payload['points']
        if len(points)<2:raise ValueError('at least two timed points required')
        times=[p['time_s'] for p in points]
        if not all(math.isfinite(t) for t in times) or any(b<=a for a,b in zip(times,times[1:])):raise ValueError('strictly increasing finite absolute times required')
        if any(len(p['position'])!=2 or not all(math.isfinite(x) for x in p['position']) for p in points):raise ValueError('finite 2D positions required')
        if event['release_time']<event['generated_time']:raise ValueError('release before generation')
        key=(event['sender_id'],event['seq'])
        if key in self.messages:raise ValueError('duplicate published message')
        old=[e for (s,_),e in self.messages.items() if s==key[0]]
        if old and (key[1]<=max(e['seq'] for e in old) or event['release_time']<max(e['release_time'] for e in old)):
            raise ValueError('out-of-order publication')
        self.messages[key]=deepcopy(event)

    def future_collisions(self,now,body_radii,obstacles,expected_senders):
        from shapely.geometry import LineString,Polygon,Point
        active={}
        for event in self.messages.values():
            sender=event['sender_id']
            if event['release_time']<=now and (sender not in active or event['seq']>active[sender]['seq']):active[sender]=event
        active={s:e for s,e in active.items() if e['payload']['points'][-1]['time_s']>=now}
        complete=set(expected_senders)<=set(active)
        if any(s not in body_radii or not math.isfinite(body_radii[s]) or body_radii[s]<=0 for s in active):
            raise ValueError('confirmed positive body radius required for each sender')
        evidence=[];margins=[]
        for sender,event in active.items():
            points=event['payload']['points'];start=max(now,points[0]['time_s'])
            times=sorted({start,*[p['time_s'] for p in points if p['time_s']>=start]})
            if times:
                coordinates=[interpolate(points,t) for t in times]
                line=LineString(coordinates) if len(times)>=2 else Point(coordinates[0])
                for index,polygon in enumerate(obstacles):
                    margin=line.distance(Polygon(polygon))-body_radii[sender];margins.append(margin)
                    if margin<=0:evidence.append(dict(kind='published_plan_obstacle_collision',sender_id=sender,seq=event['seq'],obstacle=index,margin_m=margin))
        ids=sorted(active)
        for i,a in enumerate(ids):
            for b in ids[i+1:]:
                pa=active[a]['payload']['points'];pb=active[b]['payload']['points']
                start=max(now,pa[0]['time_s'],pb[0]['time_s']);end=min(pa[-1]['time_s'],pb[-1]['time_s'])
                if start>end:continue
                times=sorted({start,end,*[p['time_s'] for p in pa+pb if start<=p['time_s']<=end]})
                segments=list(zip(times,times[1:])) or [(start,end)]
                for t0,t1 in segments:
                    a0=interpolate(pa,t0);b0=interpolate(pb,t0);a1=interpolate(pa,t1);b1=interpolate(pb,t1)
                    r=[x-y for x,y in zip(a0,b0)];v=[(x-y)-z for x,y,z in zip(a1,b1,r)]
                    speed=sum(x*x for x in v);fraction=max(0.,min(1.,-sum(x*y for x,y in zip(r,v))/speed)) if speed else 0.
                    margin=math.sqrt(sum((x+fraction*y)**2 for x,y in zip(r,v)))-body_radii[a]-body_radii[b]
                    margins.append(margin)
                    if margin<=0:evidence.append(dict(kind='published_plan_pair_collision',sender_ids=[a,b],seqs=[active[a]['seq'],active[b]['seq']],time_s=t0+fraction*(t1-t0),margin_m=margin))
        return dict(hit=True if evidence else (False if complete else None),coverage_complete=complete,
                    min_safety_margin_m=min(margins) if margins else None,evidence=evidence,
                    missing_senders=sorted(set(expected_senders)-set(active)))

    def read(self,sender,seq,receiver,time,true_position,tolerance_m):
        if tolerance_m<0:raise ValueError('nonnegative configured offset tolerance required')
        event=self.messages[(sender,seq)]
        if event['release_time']>time:raise ValueError('cannot read an unreleased message')
        position=interpolate(event['payload']['points'],time);key=(sender,receiver)
        previous=self.previous_offset.get(key,False)
        error=math.dist(position,true_position) if position is not None else None
        offset=error>tolerance_m if error is not None else None
        record=dict(sender_id=sender,seq=seq,receiver_id=receiver,time_s=time,true_position=list(true_position),
                    broadcast_position_at_absolute_time=list(position) if position is not None else None,
                    offset_m=error,offset_event=offset,new_contiguous_event=(offset and not previous) if offset is not None and previous is not None else None,
                    missing_reason='' if position is not None else 'absolute_time_outside_published_plan_no_extrapolation')
        self.previous_offset[key]=offset;self.reads.append(record);return record

    def drift_statistics(self,baseline_mean_distance=None,baseline_event_rate=None,zero_baseline_policy=None,baseline_event_count=None):
        valid=[r for r in self.reads if r['offset_m'] is not None]
        mean=sum(r['offset_m'] for r in valid)/len(valid) if valid else None
        rate=sum(r['offset_event'] for r in valid)/len(valid) if valid else None
        complete=len(valid)==len(self.reads) and bool(valid)
        def criterion(value,baseline):
            if not complete or baseline is None:return None
            if baseline==0 and zero_baseline_policy!='require_positive_offset':return None
            return value>2*baseline
        return dict(samples=len(self.reads),valid_samples=len(valid),mean_offset_m=mean,
            mismatching_read_count=sum(r['offset_event'] for r in valid),mismatching_read_rate=rate,
            contiguous_event_count=sum(r['new_contiguous_event'] is True for r in self.reads),
            distance_criterion_hit=criterion(mean,baseline_mean_distance),event_rate_criterion_hit=criterion(rate,baseline_event_rate),
            event_count_criterion_hit=criterion(sum(r['offset_event'] for r in valid),baseline_event_count),event_count_unit='mismatching_directed_read',
            zero_baseline_policy=zero_baseline_policy,criteria_combined=False)
