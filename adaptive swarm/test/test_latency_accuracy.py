import csv
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from adaptive_swarm.scripts.layered_planner.latency_eval.accuracy_transport import AccuracyQueue,WAIT_SECONDS
from adaptive_swarm.scripts.layered_planner.latency_eval.accuracy_experiment import detect,summarize,batch
from adaptive_swarm.scripts.layered_planner.latency_eval.intensity_transport import payload_hash
from adaptive_swarm.scripts.ripple_intensity_node import build_arguments


class AccuracyTimingTests(unittest.TestCase):
    def queue(self,group,enabled=True):
        q=AccuracyQueue(group,enabled);q.initialize({1:[0.,0.],2:[2.,0.]});return q

    def test_g1_immediate_and_same_standard_delay_all_groups(self):
        for group,wait in WAIT_SECONDS.items():
            q=self.queue(group);q.publish(1,0,[0.,0.],.25,True,['rule1'])
            if wait:
                self.assertEqual(q.cache[1]['seq'],0)  # t0 message was sent normally.
                self.assertFalse(q.pending)
                q.publish(1,1,[1.,0.],.25+wait-.01,False)
                self.assertFalse(q.operations)
                selected_time=.25+wait+.037
                q.publish(1,2,[2.,0.],selected_time,False) # No renewed rule match.
                self.assertEqual(q.operations[0]['seq'],2)
            else:selected_time=.25
            q.publish(1,3,[3.,0.],selected_time+.17,True,['another'])
            op=q.operations[0];w=q.windows[0]
            self.assertAlmostEqual(op['actual_delay_s'],.17)
            self.assertAlmostEqual(w['actual_wait_s'],0 if not wait else wait+.037)
            self.assertEqual(op['rule_ids'],['rule1'])
            self.assertEqual(op['rule_trigger_time'],.25)
            self.assertEqual(op['standard_policy'],'next_same_sender_publication')
            self.assertEqual(w['status'],'released');self.assertIsNone(q.active_window)
            self.assertEqual(len(q.windows),1) # release C cannot create a new window.

    def test_wait_does_not_hold_original_or_intermediate_messages(self):
        q=self.queue('G3');q.publish(1,0,[1,0],1.,True,['r'])
        q.publish(1,1,[2,0],2.,True,['ignored'])
        self.assertEqual(q.receive(1,2,2.1),[2,0]);self.assertFalse(q.pending)
        self.assertEqual(len(q.windows),1);self.assertFalse(q.operations)
        q.publish(1,2,[3,0],3.1,False)
        self.assertEqual(q.windows[0]['selected_seq'],2)
        self.assertAlmostEqual(q.windows[0]['actual_wait_s'],2.1)

    def test_deadline_uses_same_sender_and_new_event_not_clock_timer(self):
        q=self.queue('G2');q.publish(1,0,[0,0],0.,True,['r']);q.advance(1.1)
        q.publish(2,0,[2,0],1.2,True,['other'])
        self.assertFalse(q.operations)
        q.publish(1,1,[1,0],1.3,False);self.assertEqual(q.operations[0]['sender_id'],1)

    def test_wait_and_release_cancellations_are_not_actual_injections(self):
        q=self.queue('G5');q.publish(1,0,[0,0],.1,True,['r']);q.finish(2.)
        self.assertFalse(q.injection['actually_injected']);self.assertFalse(q.pending)
        self.assertEqual(q.windows[0]['skip_reason'],'task_ended_during_wait')
        q=self.queue('G1');q.publish(1,0,[0,0],.1,True,['r']);q.finish(.2)
        self.assertFalse(q.injection['actually_injected'])
        self.assertEqual(q.windows[0]['skip_reason'],'task_ended_before_release')

    def test_multiple_injections_resume_only_after_completion_payload_and_order(self):
        q=self.queue('G2')
        for round in range(8):
            t=round*2.;seq=round*4
            q.publish(1,seq,[seq,0],t,True,['r'])
            payload=[seq+1,0];q.publish(1,seq+1,payload,t+1.1,False);payload[0]=9999
            q.publish(1,seq+2,[seq+2,0],t+1.2,True,['r'])
        q.finish(16.)
        self.assertEqual(len(q.operations),8)
        self.assertTrue(all(o['status']=='released' for o in q.operations))
        releases=[e for e in q.events if e['event']=='release']
        self.assertEqual([e['seq'] for e in releases],sorted({e['seq'] for e in releases}))
        self.assertTrue(all(payload_hash(e['payload'])==e['payload_sha256'] for e in releases))

    def test_disabled_wait_trigger_and_duplicate_input(self):
        for state,enabled in [('WAIT_TRIGGER',True),('RUNNING',False)]:
            q=self.queue('G1',enabled);q.publish(1,0,[0,0],0,True,['r'],state)
            self.assertFalse(q.windows);self.assertFalse(q.operations)
        q=self.queue('G2');q.publish(1,0,[0,0],0,True,['r'])
        with self.assertRaises(ValueError):q.publish(1,0,[0,0],1.,False)
        self.assertEqual(len(q.windows),1)


class AccuracyDetectorAndIntegrationTests(unittest.TestCase):
    def test_failed_trial_has_explicit_missing_reason_for_each_target(self):
        labels,details,evidence=detect(dict(status='rrt_failed',trace=[]),SimpleNamespace())
        self.assertTrue(all(v is None for v in labels.values()))
        self.assertTrue(all(details[t+'_missing_reason'] for t in labels))
        self.assertTrue(all(not e for e in evidence.values()))

    def test_multilabel_unknown_windows_and_successful_timeout(self):
        args=SimpleNamespace(body_radius_m=.1,motion_tolerance_m=0.,timeout_threshold_s=4.)
        frame=dict(time_s=.01,positions={'1':[0.,0.],'2':[.05,0.]},fsm='RUNNING',planning_state='LOCAL_APF')
        trial=dict(status='success',time_s=5.,params=dict(crash_uav_dist=.1,ViconRate=100),
            trace=[frame],obstacles=[[[0,0],[1,0],[1,1],[0,1]]])
        labels,details,evidence=detect(trial,args)
        self.assertTrue(labels['drone_drone']);self.assertTrue(labels['drone_obstacle']);self.assertTrue(labels['delay'])
        self.assertIsNone(labels['deadlock']);self.assertIsNone(labels['planning']);self.assertIsNone(labels['drift'])
        self.assertEqual(evidence['drone_drone'][0]['drone_ids'],[1,2])
        self.assertEqual(evidence['drone_obstacle'][0]['time_s'],.01)
        trial['status']='timeout';labels,_,_=detect(trial,args);self.assertIsNone(labels['delay'])

    def test_ros_argument_dispatch_preserves_intensity_defaults(self):
        required={'~rules_file':'rules.json','~output_parent':'output'}
        def get(k,default=None):return required.get(k,default)
        mode,argv=build_arguments(get);self.assertEqual(mode,'intensity');self.assertIn('--g4-delay-s',argv)
        self.assertIn('--control',argv)
        required.update({'~experiment':'accuracy','~group':'G5','~enabled':True,'~diagnostic_position_adapter':True})
        mode,argv=build_arguments(get);self.assertEqual(mode,'accuracy');self.assertNotIn('--g4-delay-s',argv)
        self.assertIn('--diagnostic-position-adapter',argv);self.assertNotIn('--control',argv)
        root=ET.parse(Path(__file__).resolve().parents[1]/'adaptive_swarm/launch/swarm.launch').getroot()
        self.assertEqual(next(a for a in root.findall('arg') if a.attrib['name']=='ripple_experiment').attrib['default'],'intensity')
        self.assertFalse(root.findall('.//include'))

    def test_summary_unknown_and_no_injection_asr_and_retry_deduplication(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);run=p/'run';run.mkdir()
            row=dict(target='planning',group='G5',injected=False,target_known=False,target_hit=None,status='success',
                injection_skip_reason='task_ended_during_wait',missing_reasons={'planning':'missing timed trajectory'},
                completed_injections=0,result_path=str(run))
            (run/'summary.json').write_text(json.dumps(row))
            (p/'batch_manifest.json').write_text(json.dumps(dict(targets=['planning'],groups=['G5'],episodes=1,planned_trials=1)))
            (p/'jobs.json').write_text(json.dumps([dict(target='planning',group='G5',attempts=[dict(status='failed'),dict(status='finished',result_path=str(run))])]))
            result=summarize(p)[0]
            self.assertEqual(result['completed'],1);self.assertEqual(result['not_injected'],1)
            self.assertIsNone(result['attack_success_rate']);self.assertEqual(result['injection_coverage'],0.)
            row['injected']=True;(run/'summary.json').write_text(json.dumps(row))
            result=summarize(p)[0];self.assertIsNone(result['attack_success_rate']);self.assertEqual(result['known_injected'],0)

    def test_formal_guard_and_output_uniqueness(self):
        with self.assertRaises(ValueError):batch(SimpleNamespace(diagnostic_position_adapter=False))
        from adaptive_swarm.scripts.layered_planner.latency_eval.intensity_experiment import unique
        with tempfile.TemporaryDirectory() as tmp:
            a=unique(tmp,'accuracy');(a/'keep').write_text('old');b=unique(tmp,'accuracy')
            self.assertNotEqual(a,b);self.assertEqual((a/'keep').read_text(),'old')


if __name__=='__main__':unittest.main()
