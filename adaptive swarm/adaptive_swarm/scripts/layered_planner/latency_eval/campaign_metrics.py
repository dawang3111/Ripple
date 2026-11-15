"""Frozen exploratory outcomes and separate strict outcomes; saved prefix only."""
import itertools
import numpy as np
from shapely.geometry import Point,Polygon,LineString
from .user_targets import mismatch_events


def evaluate(trial,settings):
    targets=list(settings['thresholds'])
    metrics={t:dict(value=None,evidence_time_s=None,uav_ids=[],fsm=None,planning_state=None,
                    missing_reason='no_actual_injection') for t in targets}
    strict={t:None for t in targets}
    result=dict(approximate_labels={t:None for t in targets},strict_labels=strict,metrics=metrics)
    if not trial['injection']['actually_injected']:return result
    anchor=trial['injection']['nominal_tx_time']
    post=[s for s in trial['trace'] if s['time_s']>=anchor]
    if not post:
        for m in metrics.values():m['missing_reason']='no_post_injection_trace'
        return result
    ids=sorted(post[0]['positions'],key=int)
    points=np.array([[s['positions'][i] for i in ids] for s in post]);times=np.array([s['time_s'] for s in post])
    def record(target,value,index,uavs,**extra):
        state=post[index] if index is not None else {}
        metrics[target]=dict(value=float(value),evidence_time_s=float(times[index]) if index is not None else anchor,
            uav_ids=uavs,fsm=state.get('fsm','RUNNING'),planning_state=state.get('planning_state','LOCAL_APF'),missing_reason='',**extra)
    pairs=list(itertools.combinations(range(len(ids)),2))
    d=np.stack([np.linalg.norm(points[:,i]-points[:,j],axis=1) for i,j in pairs],axis=1)
    i,j=np.unravel_index(np.argmin(d),d.shape)
    record('drone_drone',d[i,j],i,[int(ids[k]) for k in pairs[j]])
    polygons=[Polygon(o) for o in trial['obstacles']]
    clear=np.empty(points.shape[:2])
    for i in range(len(post)):
        for j in range(len(ids)):
            p=Point(points[i,j]);clear[i,j]=min(-poly.boundary.distance(p) if poly.covers(p) else poly.distance(p) for poly in polygons)
    i,j=np.unravel_index(np.argmin(clear),clear.shape)
    record('drone_obstacle',clear[i,j],i,[int(ids[j])])
    steps=int(round(.2*trial['params']['ViconRate']))
    metrics['deadlock']['missing_reason']='post_history_shorter_than_0p2s_or_gaps'
    if len(post)>steps:
        net=np.linalg.norm(points[steps:]-points[:-steps],axis=2)
        net[np.abs(times[steps:]-times[:-steps]-.2)>1e-9]=np.inf
        i,j=np.unravel_index(np.argmin(net),net.shape)
        if np.isfinite(net[i,j]):record('deadlock',net[i,j],i+steps,[int(ids[j])],window_start_s=float(times[i]),window_s=.2)
    if trial['status']=='success':
        record('delay',trial['time_s']/settings['baseline_mean_s'],len(post)-1,[1],completion_s=trial['time_s'])
    else:metrics['delay']['missing_reason']='mission_not_successful'
    index=trial.get('rule_gate',{}).get('selected',{}).get('reference_sp_index',post[0]['sp_index'])
    remaining=trial['trajectory_setpoints'][index:]
    metrics['planning']['missing_reason']='insufficient_remaining_reference_points'
    if len(remaining)>=2:
        record('planning',min(LineString(remaining).distance(poly) for poly in polygons),None,[1],
               caveat='unpublished reference geometry known at injection; not attack-induced published-plan collision')
    events=[e for e in mismatch_events(trial) if e['time_s']>=anchor]
    record('drift',len(events),None,sorted({v for e in events for v in (e['sender_id'],e['receiver_id'])}))
    metrics['drift']['evidence_time_s']=events[0]['time_s'] if events else None
    metrics['drift']['fsm']='RUNNING' if events else None
    metrics['drift']['planning_state']='LOCAL_APF' if events else None
    strict.update(drone_drone=metrics['drone_drone']['value']<=trial['params']['crash_uav_dist'],
                  drone_obstacle=metrics['drone_obstacle']['value']<=0,
                  delay=(trial['time_s']>2*settings['baseline_mean_s']) if trial['status']=='success' else None,
                  drift=len(events)>=settings['strict_drift_count_threshold'])
    # Detect exact zero cumulative movement, never a round-trip displacement proxy.
    stationary=np.full(len(ids),times[0]);deadlock=False
    for i in range(1,len(post)):
        moved=np.linalg.norm(points[i]-points[i-1],axis=1)>0
        if times[i]-times[i-1]>1/trial['params']['ViconRate']+1e-9:moved[:]=True
        stationary[moved]=times[i]
        deadlock=deadlock or bool(np.any(times[i]-stationary>=settings['strict_deadlock_window_s']))
    strict['deadlock']=deadlock
    for target,meta in settings['thresholds'].items():
        value=metrics[target]['value']
        result['approximate_labels'][target]=None if value is None else bool(value<=meta['value'] if meta['direction']=='low' else value>=meta['value'])
    return result
