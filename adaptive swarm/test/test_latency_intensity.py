from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET
import tempfile

from adaptive_swarm.scripts.layered_planner.latency_eval.intensity_transport import IntensityQueue,GROUP_DELAYS,payload_hash
from adaptive_swarm.scripts.layered_planner.latency_eval.intensity_rules import import_rules,parse_conditions
from adaptive_swarm.scripts.layered_planner.latency_eval.intensity_experiment import HERE,outcomes,summarize,batch
from adaptive_swarm.scripts.layered_planner.latency_eval.published_trajectory import PublishedTrajectories


class QueueTests(unittest.TestCase):
    def queue(self,group):
        q=IntensityQueue(group);q.initialize({1:[0.,0.],2:[2.,0.]});q.publish(1,0,[0.,0.],0.)
        return q

    def test_four_fixed_delays_without_preemption(self):
        for group in ('G1','G2','G3','G4'):
            q=self.queue(group);payload=[1.,0.];q.publish(1,1,payload,1.,True,['r']);payload[0]=999
            deadline=1+GROUP_DELAYS[group];q.advance(deadline)
            self.assertEqual(q.cache[1]['payload'],[1.,0.]);self.assertAlmostEqual(q.operations[0]['actual_delay_s'],GROUP_DELAYS[group])
            self.assertEqual(q.operations[0]['release_reason'],'timer_due');self.assertTrue(q.injection['actually_injected'])

    def test_g5_uses_actual_next_event_not_one_second(self):
        for next_time in (1.35,3.4):
            q=self.queue('G5');q.publish(1,1,[1.,0.],1.,True,['r']);q.advance(next_time-.01)
            self.assertFalse(q.injection['actually_injected'])
            q.publish(1,2,[2.,0.],next_time)
            self.assertAlmostEqual(q.operations[0]['actual_delay_s'],next_time-1)
            self.assertEqual(q.operations[0]['release_reason'],'next_publication')
            self.assertEqual([e['seq'] for e in q.events if e['event']=='release'],[0,1,2])

    def test_all_groups_truncate_at_short_next_message(self):
        for group in GROUP_DELAYS:
            q=self.queue(group);q.publish(1,1,[1.,0.],1.,True,['r']);q.publish(1,2,[2.,0.],1.01,True,['r'])
            self.assertAlmostEqual(q.operations[0]['actual_delay_s'],.01)
            self.assertEqual(q.operations[0]['truncated'],group!='G5')
            self.assertEqual(len(q.operations),1);self.assertEqual(q.rule_triggers,2)
            self.assertEqual(q.cache[1]['seq'],2)

    def test_multi_injection_no_limit_and_cleanup(self):
        for group in GROUP_DELAYS:
            q=self.queue(group)
            for seq in range(1,31):q.publish(1,seq,[seq,0.],float(seq),True,['r'])
            q.finish(30.)
            self.assertGreater(sum(o['status']=='released' for o in q.operations),10)
            self.assertFalse(q.pending)
            released=[e['seq'] for e in q.events if e['event']=='release']
            self.assertEqual(released,sorted(set(released)))

    def test_cancel_is_not_actual_injection_and_duplicate_rejected(self):
        q=self.queue('G5');q.publish(1,1,[1,0],1,True,['r'])
        with self.assertRaises(ValueError):q.publish(1,1,[1,0],1.01)
        q.finish(1.02);self.assertFalse(q.injection['actually_injected']);self.assertEqual(q.operations[0]['status'],'cancelled')

    def test_actual_timer_poll_lateness_recorded(self):
        q=self.queue('G1');q.publish(1,1,[1,0],1.,True,['r']);q.advance(1.205)
        self.assertAlmostEqual(q.operations[0]['actual_delay_s'],.205)
        self.assertAlmostEqual(q.operations[0]['timer_lateness_s'],.005)

    def test_disabled_and_wait_trigger_never_delay(self):
        q=IntensityQueue('G1',enabled=False);q.initialize({1:[0,0]});q.publish(1,0,[1,0],0.,True,['r']);q.finish(.1)
        self.assertFalse(q.injection['actually_injected']);self.assertEqual(len(q.operations),0)
        q=self.queue('G5');q.publish(1,1,[1,0],1.,True,['r'],state='WAIT_TRIGGER')
        self.assertEqual(len(q.operations),0)


class RulesAndDetectorTests(unittest.TestCase):
    def test_unit_mapping_no_unsupported_substitution(self):
        condition=parse_conditions('源机相邻位移转角 > 90 °')[0]
        self.assertAlmostEqual(condition['threshold'],math.pi/2)
        with self.assertRaises(ValueError):parse_conditions('未来3秒最小机间距 <= 1 m')
        with self.assertRaises(ValueError):parse_conditions('任务已用时间 <= 1 m')

    def test_frozen_workbook_rows_and_no_synthetic_fifth_drift_rule(self):
        data=json.loads((HERE/'intensity_rules_frozen.json').read_text())
        self.assertEqual([len(g['rules']) for g in data['groups']],[10,10,10,5,10,3])
        for g in data['groups']:
            for r in g['rules']:
                self.assertEqual(r['expression'],data['source_cells'][r['source_sheet']][r['source_cell']])

    def test_body_unknown_planning_and_drift_not_fabricated(self):
        args=SimpleNamespace(body_radius_m=None,motion_tolerance_m=0.,timeout_threshold_s=4.,timeout_threshold_source='test')
        trial=dict(status='success',time_s=5.,params={'crash_uav_dist':.1,'ViconRate':100},obstacles=[[[0,0],[1,0],[1,1],[0,1]]],
            trace=[dict(time_s=.01,positions={'1':[0.,0.],'2':[.05,0.]})])
        labels,details=outcomes(trial,args)
        self.assertIsNone(labels['drone_drone']);self.assertIsNone(labels['drone_obstacle'])
        self.assertIsNone(labels['planning']);self.assertIsNone(labels['drift']);self.assertTrue(labels['delay'])
        self.assertTrue(details['legacy_pair_collision'])
        args.body_radius_m=.05;labels,_=outcomes(trial,args)
        self.assertTrue(labels['drone_drone']);self.assertTrue(labels['drone_obstacle'])
        trial['status']='timeout';labels,_=outcomes(trial,args);self.assertIsNone(labels['delay'])

    def test_new_launch_is_local_only_no_radio_and_all_groups_configurable(self):
        launch=HERE.parents[2]/'launch/swarm.launch';tree=ET.parse(launch)
        names={a.attrib['name'] for a in tree.getroot().findall('arg')}
        self.assertTrue({'ripple_enabled','intensity_group','g1_delay_s','g4_delay_s','seed','trial_id'}<=names)
        self.assertNotIn('g5_delay_s',names)
        self.assertFalse(tree.getroot().findall('.//include'))
        self.assertEqual(tree.getroot().find('node').attrib['type'],'ripple_intensity_node.py')

    def test_summary_repeated_group_key_and_unknown_asr_denominator(self):
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp);run=out/'runs/example';run.mkdir(parents=True)
            (out/'batch_manifest.json').write_text(json.dumps(dict(targets=['drone_drone'],groups=['G1'],episodes=1)))
            (run/'summary.json').write_text(json.dumps(dict(target='drone_drone',group='G1',injected=True,target_known=False,
                target_hit=None,status='success',rule_trigger_count=2)))
            (run/'trial.json').write_text(json.dumps(dict(target='drone_drone',group='G1',seed=1,trial_id=1,
                delay_operations=[dict(group='G1',status='released',actual_delay_s=.01)])))
            result=summarize(out)[0]
            self.assertEqual(result['injected'],1);self.assertEqual(result['total_completed_injections'],1)
            self.assertIsNone(result['strict_asr']);self.assertEqual(result['strict_known_injected'],0)
            self.assertTrue(json.loads((out/'progress.json').read_text())['all_jobs_complete'])

    def test_large_formal_experiment_fails_closed(self):
        with self.assertRaises(ValueError):batch(SimpleNamespace(diagnostic_position_adapter=False))


class PublishedPlanContractTests(unittest.TestCase):
    def event(self,sender,seq,start,end,t0=0.,t1=2.,release=0.):
        return dict(event='release',sender_id=sender,seq=seq,generated_time=0.,release_time=release,
                    payload=dict(kind='timed_trajectory_2d',points=[dict(time_s=t0,position=start),dict(time_s=t1,position=end)]))

    def test_candidates_history_and_snapshots_are_not_future_plans(self):
        p=PublishedTrajectories();e=self.event(1,0,[-1,0],[1,0]);e['event']='candidate'
        with self.assertRaises(ValueError):p.on_actual_release(e)
        with self.assertRaises(ValueError):p.on_actual_release(dict(event='release',payload=[0,0]))
        self.assertEqual(p.messages,{})

    def test_continuous_time_collision_only_published_future(self):
        p=PublishedTrajectories();p.on_actual_release(self.event(1,0,[-1,0],[1,0]));p.on_actual_release(self.event(2,0,[0,-1],[0,1]))
        result=p.future_collisions(0.,{1:.1,2:.1},[],[1,2]);self.assertTrue(result['hit'])
        self.assertAlmostEqual(result['evidence'][0]['time_s'],1.)
        self.assertFalse(p.future_collisions(1.5,{1:.1,2:.1},[],[1,2])['hit'])
        p.on_actual_release(self.event(1,1,[3,3],[4,3],t0=.5,release=.5))
        self.assertFalse(p.future_collisions(.5,{1:.1,2:.1},[],[1,2])['hit'])

    def test_drift_two_criteria_separate_and_no_extrapolation(self):
        p=PublishedTrajectories();p.on_actual_release(self.event(1,0,[0,0],[1,0],t1=1.))
        p.read(1,0,2,.25,[.5,0],.01);p.read(1,0,2,.5,[.5,0],.01);p.read(1,0,2,.75,[1.,0],.01)
        stats=p.drift_statistics(baseline_mean_distance=.05,baseline_event_rate=.1,baseline_event_count=.5)
        self.assertTrue(stats['distance_criterion_hit']);self.assertTrue(stats['event_rate_criterion_hit'])
        self.assertTrue(stats['event_count_criterion_hit']);self.assertEqual(stats['contiguous_event_count'],2)
        self.assertFalse(stats['criteria_combined'])
        self.assertIsNone(p.drift_statistics(0,0)['distance_criterion_hit'])
        missing=p.read(1,0,2,2.,[2,0],.01);self.assertIsNone(missing['offset_m'])
        self.assertIsNone(p.drift_statistics(.05,.1)['event_rate_criterion_hit'])


if __name__=='__main__':unittest.main()
