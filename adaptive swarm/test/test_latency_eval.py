"""Targeted, short tests; no legacy top-level experiment imports/runs."""
import csv
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from adaptive_swarm.scripts.layered_planner.latency_eval.core import Config, TxQueue, Oracles, TARGETS, feature_vector
from adaptive_swarm.scripts.layered_planner.latency_eval.analysis import analyze, kmeans2, match, validation_metrics
from adaptive_swarm.scripts.layered_planner.latency_eval.__main__ import unique_directory, main, load_trials


class TransportTests(unittest.TestCase):
    def queue(self):
        q=TxQueue();q.initialize({1:[0.,0.],2:[1.,0.]});return q

    def test_timestamps_payload_and_association(self):
        q=self.queue();payload=np.array([2.,3.])
        q.publish(1,0,payload,1.,1.1,.1,True,.75)
        payload[:]=9
        self.assertFalse(q.injection['actually_injected'])
        np.testing.assert_array_equal(q.receive(1,2,1.05),[0.,0.])
        q.advance(1.08)
        self.assertTrue(q.injection['actually_injected'])
        self.assertAlmostEqual(q.injection['actual_delay_s'],.075)
        self.assertAlmostEqual(q.injection['actual_release_time'],1.075)
        self.assertEqual(q.cache[1]['seq'],0)
        self.assertEqual(q.cache[1]['payload'],(2.,3.))
        q.publish(1,1,[4.,5.],1.1,1.2,.1)
        self.assertEqual([e['seq'] for e in q.events if e['event']=='release'],[0,1])
        self.assertEqual(q.injection['used_stale_count'],1)

    def test_no_fixed_250ms_cap(self):
        q=self.queue();q.publish(1,0,[1,2],1,3,2,True,.8)
        q.advance(2.6)
        self.assertAlmostEqual(q.injection['actual_delay_s'],1.6)

    def test_wait_trigger_skip_and_unavailable_period(self):
        for state,period,reason in [('WAIT_TRIGGER',.1,'task_not_running'),('RUNNING',None,'message_period_unavailable')]:
            q=self.queue();q.publish(1,0,[1,2],1,1.1,period,True,.5,state)
            self.assertFalse(q.injection['actually_injected']);self.assertEqual(q.injection['skip_reason'],reason)

    def test_protocol_constraint_and_early_end(self):
        q=self.queue();q.publish(1,0,[1,2],1,1.1,.1,True,.8,protocol_timeout=.05)
        self.assertEqual(q.injection['skip_reason'],'protocol_timeout_constraint')
        q=self.queue();q.publish(1,0,[1,2],1,1.1,.1,True,.8);q.finish(1.05)
        self.assertFalse(q.injection['actually_injected'])
        self.assertEqual(q.injection['skip_reason'],'simulation_ended_before_release')

    def test_duplicate_and_nonmonotonic_messages_rejected(self):
        q=self.queue();q.publish(1,0,[1,2],1,1.1,.1)
        with self.assertRaises(ValueError):q.publish(1,0,[1,2],1.1,1.2,.1)
        with self.assertRaises(ValueError):q.advance(.9)
        q=self.queue();q.publish(1,0,[1,2],1,2,1,True,.8)
        with self.assertRaises(ValueError):q.publish(1,1,[1,2],1.2,2.2,1)

    def test_at_most_one_delayed_message(self):
        q=self.queue();q.publish(1,0,[1,2],1,2,1,True,.5)
        q.publish(1,1,[1,2],2,3,1,True,.5)
        self.assertEqual(sum(e['event']=='enqueue' for e in q.events),1)


class OracleFeatureTests(unittest.TestCase):
    def test_independent_multi_labels_and_evidence(self):
        config=Config(deadlock_window_s=.1,deadlock_progress_m=.01,drift_threshold_m=.5,drift_window_s=.1)
        oracle=Oracles(config,.1,.05)
        positions={1:np.array([0.,0.]),2:np.array([.05,0.])}
        targets={1:np.array([2.,2.]),2:np.array([2.,2.])}
        obstacles=[np.array([[-1,-1],[1,-1],[1,1],[-1,1]])]
        for t in [0.,.2]:oracle.observe(t,positions,targets,obstacles,np.array([3.,3.]),0)
        oracle.finish('timeout',.3)
        oracle.finish('rrt_failed',.3)
        self.assertTrue(all(oracle.labels.values()))
        self.assertEqual(oracle.evidence['drone_drone'][0]['uav_ids'],[1,2])
        self.assertEqual(oracle.evidence['drift'][0]['planning_state'],'LOCAL_APF')
        self.assertEqual(oracle.evidence['planning_failure'][0]['fsm'],'PLANNING')

    def test_disabled_thresholds_unknown_not_negative(self):
        o=Oracles(Config(),.1,.05)
        self.assertFalse(o.known['deadlock']);self.assertFalse(o.known['drift'])
        o.finish('rrt_failed',0)
        self.assertTrue(o.known['planning_failure']);self.assertFalse(o.known['timeout'])

    def test_preinjection_features_missing_and_future_rejected(self):
        positions={1:np.array([0.,0.]),2:np.array([1.,0.])}
        histories={i:[(0.,p.copy())] for i,p in positions.items()}
        cache={i:dict(payload=tuple(p),generated_time=0.) for i,p in positions.items()}
        obs=[np.array([[3,3],[4,3],[4,4],[3,4]])]
        args=(positions,histories,1,0.,cache,positions,obs,np.array([2,2]),0,10)
        f=feature_vector(*args)
        self.assertIsNone(f['source_speed']['value'])
        self.assertEqual(f['source_displacement_5s']['missing_reason'],'history_shorter_than_5s')
        self.assertEqual(f['current_min_pair_distance']['value'],1.)
        self.assertNotIn('future_3s_min_pair_distance',f)
        histories[1].append((1.,np.array([2.,2.])))
        with self.assertRaises(ValueError):feature_vector(*args)


class AnalysisTests(unittest.TestCase):
    def synthetic(self,n=80):
        rows=[]
        for i in range(n):
            outcome=i%2==0
            rows.append(dict(episode=i,seed=i,status='success',injection={'actually_injected':True},
                labels={t:outcome for t in TARGETS},known={t:True for t in TARGETS},
                features={'speed':dict(value=float(i%7)+(10 if outcome else 0),unit='m/s'),
                          'missing':dict(value=None,unit='m')}))
        return rows

    def test_kmeans_and_analysis_reproducible(self):
        x=np.array([[0.],[1.],[10.],[11.]])
        a,_=kmeans2(x,7);b,_=kmeans2(x,7)
        np.testing.assert_array_equal(a[0],b[0]);np.testing.assert_array_equal(a[1],b[1])
        with tempfile.TemporaryDirectory() as tmp:
            a=Path(tmp)/'a';b=Path(tmp)/'b';a.mkdir();b.mkdir()
            analyze(self.synthetic(),a,7);analyze(self.synthetic(),b,7)
            for name in ['cluster_assignments.csv','cluster_summary.csv','rules_per_target.csv','summary_per_target.csv','preprocessing.json']:
                self.assertEqual((a/name).read_bytes(),(b/name).read_bytes())
            with (a/'rules_per_target.csv').open(encoding='utf-8-sig') as f:
                rules=list(csv.DictReader(f))
            self.assertTrue(rules);self.assertIn('lift_ci95',rules[0])
            self.assertEqual(rules[0]['control_samples'],'0')

    def test_empty_small_and_missing_groups(self):
        with tempfile.TemporaryDirectory() as tmp:
            result=analyze([],tmp)
            self.assertEqual(len(result),6)
            self.assertTrue((Path(tmp)/'rules_per_target.csv').exists())
            result=analyze(self.synthetic(3),tmp)
            self.assertEqual(result[0]['rule_status'],'insufficient_training_class_or_holdout_samples')
            rows=self.synthetic()
            for r in rows:r['features']['speed']['value']=None
            analyze(rows,tmp)
            text=(Path(tmp)/'cluster_summary.csv').read_text()
            self.assertIn('fewer_than_two_distinct_vectors',text)

    def test_missing_values_never_trigger_rules(self):
        self.assertEqual(match(np.array([[np.nan],[2],[4]]),[(0,'>',1)]).tolist(),[False,True,True])
        metrics=validation_metrics(np.array([True,False]),np.array([False,False]),7)
        self.assertIsNone(metrics['target_rate']);self.assertTrue(metrics['low_confidence'])

    def test_control_trigger_rates_and_skipped_denominators(self):
        rows=self.synthetic()
        skipped=self.synthetic(1)[0]
        skipped['episode']=999
        skipped['injection']={'actually_injected':False,'skip_reason':'planned_time_not_reached'}
        with tempfile.TemporaryDirectory() as tmp:
            summary=analyze(rows+[skipped],tmp,7,controls=rows[:8])
            self.assertEqual(summary[0]['valid_trials'],80)
            self.assertEqual(summary[0]['invalid_reasons']['planned_time_not_reached'],1)
            with (Path(tmp)/'rules_per_target.csv').open(encoding='utf-8-sig') as f:
                rules=list(csv.DictReader(f))
            self.assertTrue(rules)
            self.assertEqual(rules[0]['control_evaluable'],'8')
            self.assertTrue(0<=float(rules[0]['control_trigger_rate'])<=1)

    def test_holdout_does_not_choose_thresholds(self):
        rows=self.synthetic()
        with tempfile.TemporaryDirectory() as tmp:
            a=Path(tmp)/'a';b=Path(tmp)/'b';a.mkdir();b.mkdir();analyze(rows,a,7)
            meta=json.loads((a/'preprocessing.json').read_text())
            # Change only target drone_drone holdout FEATURES; its training rules must stay identical.
            held=set(meta['drone_drone']['training']['holdout_episodes'])
            for row in rows:
                if row['episode'] in held:row['features']['speed']['value']=9999.
            analyze(rows,b,7)
            def expressions(p):
                with (p/'rules_per_target.csv').open(encoding='utf-8-sig') as f:
                    return [r['expression'] for r in csv.DictReader(f) if r['target']=='drone_drone']
            self.assertEqual(expressions(a),expressions(b))

    def test_output_directory_never_overwrites(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=unique_directory(tmp,'test');(a/'keep').write_text('old')
            b=unique_directory(tmp,'test')
            self.assertNotEqual(a,b);self.assertEqual((a/'keep').read_text(),'old')


class IntegrationTests(unittest.TestCase):
    def test_control_matches_legacy_update_order(self):
        from adaptive_swarm.scripts.layered_planner.latency_eval.simulation import TxSimulation
        from adaptive_swarm.scripts.layered_planner.Adaptive_swarm_collsion_findradius_obstacle_inflation import BaselineSimulation
        with tempfile.TemporaryDirectory() as tmp:
            # Explicitly disable legacy history fault: {} unexpectedly activates
            # route[0] lookup on tick 0, even with zero configured latency.
            original=BaselineSimulation(1,tmp,seed=12,
                                        fuzz_params={'start_step':-2,'duration_steps':0})
            def stop_after_four():
                if original.time_step==3:
                    raise StopIteration('test-only four-tick limit')
                return None,None
            with patch.object(original,'check_collisions',side_effect=stop_after_four):
                with self.assertRaises(StopIteration): original.start()
            adapted=TxSimulation(1,tmp,12,Config(max_ticks=4))
            adapted.control=True
            result=adapted.start()
            self.assertEqual(len(result['trace']),4)
            for before,after in zip(original.tick_history,result['trace']):
                for robot in before['robots']:
                    np.testing.assert_allclose(robot['position'],after['positions'][robot['id']],rtol=0,atol=1e-12)

    def test_short_run_and_offline_reload(self):
        with tempfile.TemporaryDirectory() as tmp:
            out=main(['run','--episodes','1','--max-ticks','4','--seed','12','--output-parent',tmp])
            rows,_=load_trials(out)
            self.assertEqual(len(rows),1)
            for name in ['manifest.json','trials.csv','trial_features.csv','cluster_assignments.csv','cluster_summary.csv','rules_per_target.csv','summary_per_target.csv','analysis.md']:
                self.assertTrue((out/name).exists(),name)
            self.assertNotEqual(rows[0]['status'],'error',rows[0].get('note'))
            self.assertTrue(rows[0]['injection']['attempted'])
            offline=main(['analyze','--input',str(out),'--output-parent',tmp])
            self.assertNotEqual(out,offline)
            self.assertEqual((out/'summary_per_target.csv').read_bytes(),(offline/'summary_per_target.csv').read_bytes())

    def test_optional_rrt_bound(self):
        from adaptive_swarm.scripts.layered_planner.rrt import rrt_path
        from adaptive_swarm.scripts.layered_planner.Adaptive_swarm_collsion_findradius_obstacle_inflation import Params
        with self.assertRaises(TimeoutError):rrt_path([],np.array([0,0]),np.array([1,1]),Params(),max_iterations=0)


if __name__=='__main__':unittest.main()
