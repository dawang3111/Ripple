"""Intensity-specific outcomes. Does not reuse old planner-FAIL/cache-drift labels."""
import csv,gzip,json,math
from collections import Counter,deque
from pathlib import Path
import numpy as np
from .core import Trajectory,VoxelMap,read_events,write_json


def events_for(d):
 p=d/'events.jsonl'
 return read_events(p if p.exists() else d/'events.jsonl.gz')


def injection_evidence(events):
 rows=[e for e in events if e['kind']=='intensity_tx'];scheduled={};published={};records=[];errors=[]
 for e in rows:
  key=(e['id'],e['seq'])
  if e['action']=='scheduled':
   if key in scheduled:errors.append('duplicate_schedule')
   scheduled[key]=e
  if e['action'] in ('release','forward'):
   published.setdefault(e['id'],[]).append(e)
  if e['action']=='release':
   s=scheduled.get(key);valid=bool(s and s['payload_fnv1a64']==e['payload_fnv1a64'] and s['generated_ns']==e['generated_ns'])
   if not valid:errors.append('release_without_identical_schedule')
   records.append(dict(sender_id=e['id'],seq=e['seq'],traj_id=e['traj_id'],decision_ns=s.get('decision_ns') if s else None,
       generated_ns=e['generated_ns'],release_ns=e['publish_before_ns'],release_complete_ns=e['publish_after_ns'],
       planned_delay_s=e['planned_delay_ns']/1e9 if e['planned_delay_ns'] else None,actual_delay_s=e['actual_delay_ns']/1e9,
       reason=e['reason'],payload_unchanged=valid,order_valid=True,cleanup_release=e['reason']=='task_end_flush',
       truncated=bool(e['planned_delay_ns'] and e['actual_delay_ns']<e['planned_delay_ns']),
       completed_injection=valid and e['actual_delay_ns']>0 and e['reason']!='task_end_flush'))
 for sender,packets in published.items():
  packets.sort(key=lambda e:e['publish_before_ns']);seqs=[e['seq'] for e in packets]
  good=seqs==sorted(set(seqs))
  if not good:errors.append('duplicate_or_out_of_order_publish')
  for r in records:
   if r['sender_id']!=sender:continue
   r['order_valid']=good;r['completed_injection'] &=good
   next_packet=next((e for e in packets if e['seq']==r['seq']+1),None)
   r['successor_publish_ns']=next_packet['publish_before_ns'] if next_packet else None
   if next_packet and r['release_complete_ns']>next_packet['publish_before_ns']:
    r['order_valid']=False;r['completed_injection']=False;errors.append('release_completed_after_successor')
 released={(r['sender_id'],r['seq']) for r in records}
 for key in scheduled:
  if key not in released:errors.append('unreleased_or_unobserved_schedule:'+str(key))
 decisions={e['decision_ns']:e for e in events if e['kind']=='intensity_rule_decision'}
 for r in records:
  decision=decisions.get(r['decision_ns'])
  r['matched_rule_ids']=json.dumps(decision.get('matched_rule_ids',[]) if decision else [])
  r['rule_evidence_valid']=bool(decision and decision['decision_ns']<=r['generated_ns']<=decision['expires_ns'])
 return records,errors


def event_statistics(samples,threshold,gap_ns):
 if threshold is None:return dict(event_n=None,event_rate_per_s=None,above_sample_fraction=None,mean_error_m=float(np.mean([s['error_m'] for s in samples])) if samples else None)
 count=0;state={};above=0
 for s in samples:
  i=s['id'];t=s['time_ns'];hit=s['error_m']>threshold
  prior=state.get(i)
  if hit and (not prior or not prior[1] or t-prior[0]>gap_ns):count+=1
  state[i]=(t,hit);above+=hit
 span=0.;previous={}
 for s in samples:
  if s['id'] in previous:span+=min(max(0,(s['time_ns']-previous[s['id']])/1e9),gap_ns/1e9)
  previous[s['id']]=s['time_ns']
 return dict(event_n=count,event_rate_per_s=count/span if span else None,above_sample_fraction=above/len(samples) if samples else None,
             mean_error_m=float(np.mean([s['error_m'] for s in samples])) if samples else None,observed_drone_seconds=span)


def trajectory_values(tr,times):
 """Vectorized same De Boor convention as core.Trajectory.at, no extrapolation."""
 t=np.asarray(times)-tr.start
 if np.any(t < -1e-8) or np.any(t > tr.duration+1e-8):raise ValueError('outside published trajectory')
 u=np.clip(t+tr.knots[tr.degree],tr.knots[tr.degree],tr.knots[-tr.degree-1])
 k=np.clip(np.searchsorted(tr.knots,u,side='left')-1,tr.degree,len(tr.points)-1)
 d=np.asarray([tr.points[k-tr.degree+i] for i in range(tr.degree+1)]).transpose(1,0,2).copy()
 for r in range(1,tr.degree+1):
  for i in range(tr.degree,r-1,-1):
   den=tr.knots[i+1+k-r]-tr.knots[i+k-tr.degree]
   a=np.divide(u-tr.knots[i+k-tr.degree],den,out=np.zeros(len(t)),where=den!=0)
   d[:,i]=(1-a[:,None])*d[:,i-1]+a[:,None]*d[:,i]
 return d[:,tr.degree]


def future_times(tr,query,step):
 # Large absolute ROS times have ~0.2 microsecond float ULPs. Keep the last
 # sample strictly inside the published spline domain; never extrapolate.
 lo=max(query,tr.start);hi=float(np.nextafter(tr.start+tr.duration,-np.inf))
 if hi<=lo:return np.asarray([])
 return np.linspace(lo,hi,max(2,int(math.ceil((hi-lo)/step))+1))


def analyze(d,cfg,baseline=None):
 d=Path(d);plan=json.loads((d/'plan.json').read_text());capture=json.loads((d/'capture.json').read_text());events=events_for(d)
 starts=[e for e in events if e['kind']=='task_started'];start=min((e['mono_ns'] for e in starts),default=None);end=capture['end_ns']
 injections,errors=injection_evidence(events);first=min((r['generated_ns'] for r in injections if r['completed_injection']),default=start)
 duration=(end-start)/1e9 if start is not None else None
 scene=d/'scene_points.npy';geometry=None
 if scene.exists():points=np.load(scene,allow_pickle=False)
 elif scene.with_suffix('.npy.gz').exists():
  with gzip.open(scene.with_suffix('.npy.gz'),'rb') as f:points=np.load(f,allow_pickle=False)
 else:points=None
 if points is not None:geometry=VoxelMap(points,cfg['thresholds']['map_resolution_m'])
 radius=cfg['thresholds']['body_radius_m'];gap=int(cfg['max_sample_gap_s']*1e9);tol=cfg['alignment_tolerance_s']
 labels={t:dict(hit=False,valid=False,evidence=[],missing_reason='observation_not_complete') for t in ('inter_drone_collision','obstacle_collision','planning_failure','deadlock')}
 def hit(name,t,ids,definition,**extra):
  label=labels[name];label['hit']=True;label['valid']=True;label['missing_reason']=''
  if len(label['evidence'])<20:label['evidence'].append(dict(time_ns=t,ids=ids,definition=definition,
      fsm={str(i):latest[i].get('fsm') for i in ids if i in latest},
      position_measurement_ros_s={str(i):latest[i].get('position_stamp_s',latest[i]['ros_s']) for i in ids if i in latest},**extra))
 latest={};broadcast={};broadcast_key={};samples=[];hist={};seen_samples={i:[] for i in cfg['active_ids']};published_n=0;raw_drift=[]
 for e in events:
  t=e.get('mono_ns',e['observed_ns'])
  if t>end:continue
  if e['kind']=='observed_broadcast_trajectory':
   i=e['id'];tr=Trajectory(e);key=(e['traj_id'],e['start_s'])
   new=broadcast_key.get(i)!=key;broadcast[i]=tr;broadcast_key[i]=key
   if new and first is not None and t>=first:
    published_n+=1
    # Evaluate only actually broadcast valid FUTURE portions, never internal candidates.
    ts=future_times(tr,e['ros_s'],cfg['future_step_s'])
    if len(ts):
     xyz=trajectory_values(tr,ts)
     if geometry:
      for v,p in zip(ts,xyz):
       if geometry.collides(p,radius):
        hit('planning_failure',t,[i],'actually_broadcast_future_obstacle_overlap',predicted_ros_s=float(v),trajectory_id=e['traj_id']);break
     for j,other in broadcast.items():
      if j==i:continue
      mask=(ts>=other.start)&(ts<=float(np.nextafter(other.start+other.duration,-np.inf)));times=ts[mask]
      if not len(times):continue
      dist=np.linalg.norm(xyz[mask]-trajectory_values(other,times),axis=1)
      if float(dist.min())<=2*radius:
       k=int(dist.argmin());hit('planning_failure',t,[i,j],'actually_broadcast_future_inter_uav_overlap',predicted_ros_s=float(times[k]),distance_m=float(dist[k]))
  if e['kind']!='snapshot' or e.get('id') not in seen_samples or start is None or t<start:continue
  i=e['id'];latest[i]=e
  if not 0<=e['ros_s']-e.get('position_stamp_s',e['ros_s'])<=tol:continue
  if first is not None and t>=first:seen_samples[i].append(t)
  tr=broadcast.get(i)
  if tr:
   try:
    predicted=tr.at(e.get('position_stamp_s',e['ros_s']));error=float(np.linalg.norm(np.asarray(e['position'])-predicted))
    raw_drift.append(dict(time_ns=t,id=i,error_m=error,post_injection=first is not None and t>=first,position_stamp_s=e.get('position_stamp_s',e['ros_s']),
       truth_x_m=e['position'][0],truth_y_m=e['position'][1],truth_z_m=e['position'][2],broadcast_x_m=predicted[0],broadcast_y_m=predicted[1],broadcast_z_m=predicted[2],
       broadcast_traj_id=broadcast_key[i][0],broadcast_start_s=tr.start))
   except ValueError:pass
  if first is None or t<first:continue
  aligned={j:r for j,r in latest.items() if 0<=t-r.get('mono_ns',r['observed_ns'])<=int(tol*1e9) and 0<=e['ros_s']-r.get('position_stamp_s',r['ros_s'])<=tol}
  for j,r in aligned.items():
   if j<=i:continue
   distance=float(np.linalg.norm(np.asarray(e['position'])-r['position']))
   if distance<=2*radius:hit('inter_drone_collision',t,[i,j],'actual_position_configured_spheres_overlap',distance_m=distance)
  if geometry and geometry.collides(e['position'],radius):hit('obstacle_collision',t,[i],'actual_position_sphere_vs_captured_occupied_voxel')
  if e.get('fsm')=='FINISH':hist.pop(i,None);continue
  h=hist.setdefault(i,deque())
  if h and t-h[-1].get('mono_ns',h[-1]['observed_ns'])>gap:h.clear()
  h.append(e);window=30_000_000_000
  while len(h)>1 and t-h[1].get('mono_ns',h[1]['observed_ns'])>=window:h.popleft()
  if h and t-h[0].get('mono_ns',h[0]['observed_ns'])>=window and all(np.linalg.norm(r['velocity'])<=cfg['thresholds']['deadlock_speed_mps'] and np.linalg.norm(np.asarray(r['position'])-h[0]['position'])<=cfg['thresholds']['deadlock_displacement_m'] for r in h):
   hit('deadlock',t,[i],'unfinished_task_30s_near_stationary_including_IDLE')
 full=first is not None and all(ts and ts[0]-first<=gap and end-ts[-1]<=gap and all(b-a<=gap for a,b in zip(ts,ts[1:])) for ts in seen_samples.values())
 for name,label in labels.items():
  if not label['hit']:
   label['valid']=bool(full and (name!='obstacle_collision' or geometry) and (name!='planning_failure' or published_n))
   label['missing_reason']='' if label['valid'] else 'missing_or_gapped_geometry_or_published_trajectory_observation'
 completed=capture['status']=='completed';base_time=baseline.get('mean_completed_task_s') if baseline else None
 labels['delay_timeout']=dict(hit=bool(completed and base_time and duration>2*base_time),valid=bool(completed and base_time),
     evidence=[dict(duration_s=duration,threshold_s=2*base_time)] if completed and base_time and duration>2*base_time else [],
     missing_reason='' if completed and base_time else 'unfinished_task_or_no_completed_baseline')
 raw_post=[s for s in raw_drift if s['post_injection']]
 mean_base=baseline.get('mean_broadcast_error_m') if baseline else None
 threshold=2*mean_base if mean_base and mean_base>0 else None
 drift=event_statistics(raw_post,threshold,gap)
 drift.update(distance_threshold_m=threshold,baseline_zero_or_missing=threshold is None,samples_n=len(raw_post),
     expected_aligned_position_samples_n=sum(len(ts) for ts in seen_samples.values()),
     broadcast_coverage_fraction=len(raw_post)/sum(len(ts) for ts in seen_samples.values()) if sum(len(ts) for ts in seen_samples.values()) else 0)
 for suffix,key,basekey in [('distance',None,None),('count','event_n','mean_drift_event_n'),('rate','event_rate_per_s','mean_drift_event_rate_per_s')]:
  valid=bool(raw_post and threshold is not None)
  if suffix=='distance':value=any(s['error_m']>threshold for s in raw_post) if valid else False
  else:
   reference=baseline.get(basekey) if baseline else None
   valid=valid and reference is not None and reference>0 and drift[key] is not None
   value=bool(valid and drift[key]>2*reference)
  valid=bool(valid and (value or (full and len(raw_post)==sum(len(ts) for ts in seen_samples.values()))))
  evidence=([dict(time_ns=s['time_ns'],ids=[s['id']],error_m=s['error_m'],threshold_m=threshold) for s in raw_post if s['error_m']>threshold][:20] if valid and suffix=='distance' else [dict(event_n=drift['event_n'],event_rate_per_s=drift['event_rate_per_s'])] if valid else [])
  labels['drift_'+suffix]=dict(hit=value,valid=valid,evidence=evidence,missing_reason='' if valid else 'zero_missing_baseline_or_incomplete_broadcast_coverage')
 if raw_drift:
  with (d/'drift_samples.csv').open('w',newline='') as f:
   w=csv.DictWriter(f,fieldnames=list(raw_drift[0]));w.writeheader();w.writerows(raw_drift)
 result=dict(trial_id=plan['trial_id'],target=plan['target'],group=plan.get('group'),baseline=plan.get('baseline',False),seed=plan['seed'],
     status=capture['status'],task_completed=completed,unfinished_task=not completed,task_duration_s=duration,
     injections=injections,injection_errors=errors,completed_injection_n=sum(r['completed_injection'] for r in injections),
     actual_release_n=len(injections),rule_trigger_n=sum(e['kind']=='intensity_rule_decision' for e in events),
     selected_message_n=sum(e['kind']=='intensity_tx' and e.get('selected') and e['action'] in ('scheduled','forward') for e in events),
     scheduled_message_n=sum(e['kind']=='intensity_tx' and e['action']=='scheduled' for e in events),
     labels=labels,drift=drift,raw_drift_mean_m=float(np.mean([s['error_m'] for s in raw_drift])) if raw_drift else None,
     geometry_scope='configured sphere overlap of actual simulation positions, not physical contact feedback',result_path=str(d/'intensity_result.json'))
 write_json(d/'intensity_result.json',result);return result


def calibrate(baseline_dir,cfg):
 directory=Path(baseline_dir);results=[analyze(p.parent,cfg) for p in sorted(directory.glob('trial_*/capture.json'))]
 completed=[r for r in results if r['task_completed']]
 if not completed:raise ValueError('no completed uninjected baseline; do not substitute injected rounds')
 errors=[r['raw_drift_mean_m'] for r in completed if r['raw_drift_mean_m'] is not None]
 b=dict(source=str(directory.resolve()),planned_n=len(results),completed_n=len(completed),
        mean_completed_task_s=float(np.mean([r['task_duration_s'] for r in completed])),mean_broadcast_error_m=float(np.mean(errors)) if errors else None,
        low_sample_baseline=len(completed)<20,geometry_radius_m=cfg['thresholds']['body_radius_m'],
        drift_event_definition='one contiguous above 2x baseline mean error episode per drone; split on below threshold or >0.25s observation gap',
        zero_policy='undefined ratios and unknown labels; never invent epsilon or positive success',
        deadlock_window_s=30,deadlock_speed_mps=cfg['thresholds']['deadlock_speed_mps'],deadlock_displacement_m=cfg['thresholds']['deadlock_displacement_m'])
 b['mean_drift_event_n']=None;b['mean_drift_event_rate_per_s']=None
 recomputed=[analyze(Path(r['result_path']).parent,cfg,b) for r in completed]
 counts=[r['drift']['event_n'] for r in recomputed if r['drift']['event_n'] is not None]
 rates=[r['drift']['event_rate_per_s'] for r in recomputed if r['drift']['event_rate_per_s'] is not None]
 b.update(mean_drift_event_n=float(np.mean(counts)) if counts else None,mean_drift_event_rate_per_s=float(np.mean(rates)) if rates else None)
 write_json(directory/'baseline_reference.json',b);return b
