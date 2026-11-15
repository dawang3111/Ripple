import json,subprocess,tempfile,unittest
from pathlib import Path
import numpy as np
from latency_eval.core import ROOT,config
from latency_eval.intensity_batch import plans
from latency_eval.intensity_metrics import injection_evidence,event_statistics,analyze,future_times,trajectory_values

class IntensityTests(unittest.TestCase):
 def test_cpp_all_five_groups_order_repetition_and_cleanup(self):
  with tempfile.TemporaryDirectory() as tmp:
   d=Path(tmp);p=d/'test.cpp';p.write_text('''#include <exploration_manager/intensity_delay_gate.h>
#include <cassert>
int main(){for(int g=1;g<=5;g++){racer_eval::IntensityDelayGate<int>x;x.enabled=true;x.group=g;x.fixed_ns=g*200000000LL;
auto a=x.message(7,100,true,10);assert(a.size()==1&&a[0].kind=="scheduled"&&a[0].decision_ns==10);
assert(x.timer(100+x.fixed_ns-1).empty());
if(g!=5){a=x.timer(100+x.fixed_ns);assert(a.size()==1&&a[0].message==7&&a[0].kind=="release");}
else {assert(x.timer(999999999999LL).empty());a=x.message(7,200,true,20);assert(a.size()==2&&a[0].kind=="release"&&a[1].kind=="forward"&&a[0].seq!=a[1].seq);}
a=x.message(8,2000000000LL,true,30);assert(a.size()==1&&a[0].kind=="scheduled");a=x.message(9,2000000010LL,true,40);assert(a.size()==2&&a[0].message==8&&a[1].message==9&&a[1].kind=="forward");
a=x.message(10,3000000000LL,true,50);assert(a[0].kind=="scheduled");a=x.flush();assert(a[0].reason=="task_end_flush");assert(x.flush().empty());}}
''')
   subprocess.run(['g++','-std=c++11','-I'+str(ROOT/'swarm_exploration/exploration_manager/include'),str(p),'-o',str(d/'test')],check=True,capture_output=True)
   subprocess.run([str(d/'test')],check=True,capture_output=True)
 def test_schedule_release_hash_and_cleanup_are_not_plan_equals_injection(self):
  s=dict(kind='intensity_tx',id=1,seq=1,action='scheduled',payload_fnv1a64='abc',generated_ns=100,decision_ns=90)
  r=dict(s,action='release',publish_before_ns=200,publish_after_ns=201,planned_delay_ns=200,actual_delay_ns=100,reason='early_successor',traj_id=1)
  packets,errors=injection_evidence([s]);self.assertEqual(packets,[])
  packets,errors=injection_evidence([s,r]);self.assertTrue(packets[0]['completed_injection']);self.assertTrue(packets[0]['truncated'])
  packets,errors=injection_evidence([s,dict(r,payload_fnv1a64='changed')]);self.assertFalse(packets[0]['completed_injection'])
  packets,errors=injection_evidence([s,dict(r,reason='task_end_flush')]);self.assertFalse(packets[0]['completed_injection'])
 def test_paired_seeds_no_1500_to_3000_expansion(self):
  packet=dict(config=config());items=list(plans(list(('inter_drone_collision','obstacle_collision','planning_failure','deadlock','delay_timeout','drift')),['G1','G2','G3','G4','G5'],50,20261113,packet))
  self.assertEqual(len(items),1500)
  for i in range(0,len(items),5):
   self.assertEqual(len({p['seed'] for p in items[i:i+5]}),1)
   self.assertEqual(len({p['source_id'] for p in items[i:i+5]}),1)
  self.assertEqual(len({p['trial_id'] for p in items}),1500)
 def test_event_episode_and_zero_baseline(self):
  samples=[dict(id=1,time_ns=i*100000000,error_m=v) for i,v in enumerate([.3,.4,0.,.3])]
  self.assertEqual(event_statistics(samples,.2,250000000)['event_n'],2)
  self.assertIsNone(event_statistics(samples,None,250000000)['event_n'])
 def test_planning_checks_only_actual_broadcast_and_timeout_only_finished(self):
  with tempfile.TemporaryDirectory() as tmp:
   d=Path(tmp);cfg=config();plan=dict(trial_id='test',target='planning_failure',group='G1',seed=1,baseline=True)
   (d/'plan.json').write_text(json.dumps(plan));(d/'capture.json').write_text(json.dumps(dict(status='completed',end_ns=2_000_000_000)))
   np.save(d/'scene_points.npy',np.asarray([[0.,0.,1.]]))
   start=dict(kind='task_started',id=1,mono_ns=1_000_000_000,observed_ns=1_000_000_000,ros_s=1.)
   tr=dict(kind='trajectory',id=1,observed_ns=1_010_000_000,ros_s=1.01,order=3,start_s=1.,traj_id=1,knots=[-3,-2,-1,0,1,2,3,4],pos_pts=[[0,0,1]]*4)
   def write(kind):
    (d/'events.jsonl').write_text(json.dumps(start)+'\n'+json.dumps(dict(tr,kind=kind))+'\n')
   write('trajectory');r=analyze(d,cfg,dict(mean_completed_task_s=.25,mean_broadcast_error_m=0))
   self.assertFalse(r['labels']['planning_failure']['hit']);self.assertTrue(r['labels']['delay_timeout']['hit'])
   write('observed_broadcast_trajectory');r=analyze(d,cfg,dict(mean_completed_task_s=.25,mean_broadcast_error_m=0))
   self.assertTrue(r['labels']['planning_failure']['hit'])
   (d/'capture.json').write_text(json.dumps(dict(status='task_budget_exhausted',end_ns=2_000_000_000)))
   r=analyze(d,cfg,dict(mean_completed_task_s=.25,mean_broadcast_error_m=0))
   self.assertFalse(r['labels']['delay_timeout']['hit']);self.assertTrue(r['unfinished_task'])
 def test_absolute_ros_time_endpoints_remain_in_domain(self):
  from latency_eval.core import Trajectory
  e=dict(order=3,start_s=1794559183.9895735,knots=[v*.123456789 for v in [-3,-2,-1,0,1,2,3,4]],pos_pts=[[0,0,1]]*4)
  tr=Trajectory(e)
  for t in future_times(tr,tr.start,.01):tr.at(t)

 def test_deadlock_idle_evidence_preserves_fsm_without_duplicate_keyword(self):
  with tempfile.TemporaryDirectory() as tmp:
   d=Path(tmp);cfg=config()
   (d/'plan.json').write_text(json.dumps(dict(trial_id='test',target='deadlock',group='G1',seed=1,baseline=True)))
   (d/'capture.json').write_text(json.dumps(dict(status='task_budget_exhausted',end_ns=32_000_000_000)))
   events=[dict(kind='task_started',id=1,mono_ns=1_000_000_000,observed_ns=1_000_000_000,ros_s=1.)]
   for n in range(311):
    t=1_000_000_000+n*100_000_000
    events.append(dict(kind='snapshot',id=1,mono_ns=t,observed_ns=t,ros_s=t/1e9,position=[0,0,1],velocity=[0,0,0],fsm='IDLE'))
   (d/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
   r=analyze(d,cfg)
   self.assertTrue(r['labels']['deadlock']['hit'])
   self.assertEqual(r['labels']['deadlock']['evidence'][0]['fsm'],{'1':'IDLE'})

 def test_six_intensity_outcomes_are_independent_multilabels(self):
  with tempfile.TemporaryDirectory() as tmp:
   d=Path(tmp);cfg=config();cfg['active_ids']=[1,2]
   (d/'plan.json').write_text(json.dumps(dict(trial_id='test',target='drift',group='G5',seed=1,baseline=True)))
   (d/'capture.json').write_text(json.dumps(dict(status='completed',end_ns=32_000_000_000)))
   np.save(d/'scene_points.npy',np.asarray([[0.,0.,1.]]))
   events=[dict(kind='task_started',id=1,mono_ns=1_000_000_000,observed_ns=1_000_000_000,ros_s=1.)]
   for i in (1,2):events.append(dict(kind='observed_broadcast_trajectory',id=i,observed_ns=1_000_000_000,ros_s=1.,order=3,start_s=1.,traj_id=1,knots=[-150,-100,-50,0,50,100,150,200],pos_pts=[[.3,0,1]]*4))
   for n in range(311):
    t=1_000_000_000+n*100_000_000
    for i in (1,2):events.append(dict(kind='snapshot',id=i,mono_ns=t,observed_ns=t,ros_s=t/1e9,position=[(i-1)*.1,0,1],velocity=[0,0,0],fsm='IDLE'))
   (d/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
   r=analyze(d,cfg,dict(mean_completed_task_s=10,mean_broadcast_error_m=.05,mean_drift_event_n=.1,mean_drift_event_rate_per_s=.001))
   for key in ('inter_drone_collision','obstacle_collision','planning_failure','deadlock','delay_timeout','drift_distance','drift_count','drift_rate'):
    self.assertTrue(r['labels'][key]['hit'],key)
    self.assertTrue(r['labels'][key]['valid'],key)

 def test_vectorized_spline_matches_original_deboor(self):
  from latency_eval.core import Trajectory
  e=dict(order=3,start_s=1794559183.9895735,knots=[v*.123456789 for v in [-3,-2,-1,0,1,2,3,4]],pos_pts=[[0,0,1],[1,3,2],[2,-3,1],[4,0,2]])
  tr=Trajectory(e);times=future_times(tr,tr.start,.001)
  np.testing.assert_allclose(trajectory_values(tr,times),np.asarray([tr.at(t) for t in times]),rtol=1e-12,atol=1e-12)

 def test_native_launch_seed_is_string_and_old_gate_remains_separate(self):
  import xml.etree.ElementTree as ET
  root=ET.parse(ROOT/'swarm_exploration/exploration_manager/launch/swarm.launch').getroot()
  seed=next(e for e in root.findall('param') if e.get('name')=='/racer_intensity/seed')
  self.assertEqual(seed.get('type'),'str')

 def test_recovery_copies_into_unique_output_without_overwriting_original(self):
  from unittest.mock import patch
  from latency_eval.intensity_batch import main
  with tempfile.TemporaryDirectory() as tmp:
   old=Path(tmp)/'original';trial=old/'run/trial_0001';trial.mkdir(parents=True)
   for name,value in [('manifest.json',{'total':1}),('frozen_rules.json',{}),('baseline_reference.json',{})]:
    (old/name).write_text(json.dumps(value))
   (old/'run/config.json').write_text(json.dumps(config()))
   (trial/'plan.json').write_text(json.dumps({'seed':42,'rules_file':'old'}))
   (trial/'capture.json').write_text('{}');saved=(trial/'plan.json').read_bytes()
   with patch('latency_eval.intensity_batch.preflight_run'),patch('latency_eval.intensity_batch.experiment_processes',return_value=[]),patch('latency_eval.intensity_batch.execute_job',return_value=0) as execute:
    main(['recover','--input',str(old),'--output-root',tmp])
   new=execute.call_args[0][0]
   self.assertNotEqual(new,old);self.assertEqual((trial/'plan.json').read_bytes(),saved)
   self.assertEqual(json.loads((new/'run/trial_0001/plan.json').read_text())['seed'],42)
   self.assertTrue((new/'run/trial_0001/capture.json').exists())
   self.assertTrue((new/'parent_manifest.json').exists())

if __name__=='__main__':unittest.main()
