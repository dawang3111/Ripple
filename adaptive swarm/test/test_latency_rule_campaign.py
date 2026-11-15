import argparse
import contextlib
from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from adaptive_swarm.scripts.layered_planner.latency_eval import rule_campaign as campaign
from adaptive_swarm.scripts.layered_planner.latency_eval.rule_policy import RuleGate,rule_matches
from adaptive_swarm.scripts.layered_planner.latency_eval.campaign_metrics import evaluate


def rule(field='elapsed_task_time',op='<=',threshold=1):
    return dict(rule_id=field,conditions=[dict(field=field,op=op,threshold=threshold)])


def feature(value,time=.1):return dict(value=value,observed_time=time,source_times={'source':time})


class GateTests(unittest.TestCase):
    def test_or_between_rules_and_inside_rule(self):
        a=rule();a['conditions'].append(dict(field='source_turn_angle',op='>',threshold=0))
        g=RuleGate(dict(group_id='g',target='delay',rules=[a,rule('leader_goal_distance')]),2,5)
        self.assertFalse(g.should_observe(1,5,.01));self.assertFalse(g.should_observe(2,4,.01))
        self.assertFalse(g.should_observe(2,5,None))
        f={'elapsed_task_time':feature(.1),'source_turn_angle':feature(0),'leader_goal_distance':feature(.5)}
        self.assertTrue(g.decide(f,.1,5,3));self.assertEqual(g.selected['matched_rule_ids'],['leader_goal_distance'])
        self.assertFalse(g.should_observe(2,6,.01));self.assertFalse(g.decide(f,.11,6,4))

    def test_missing_future_and_no_trigger(self):
        self.assertFalse(rule_matches({},rule(),.1))
        self.assertFalse(rule_matches({'elapsed_task_time':feature(float('nan'))},rule(),.1))
        with self.assertRaises(ValueError):rule_matches({'elapsed_task_time':feature(.1,.2)},rule(),.1)
        f=feature(.1);f['source_times']['source']=.2
        with self.assertRaises(ValueError):rule_matches({'elapsed_task_time':f},rule(),.1)
        g=RuleGate(dict(group_id='g',target='delay',rules=[rule()]),1,2)
        self.assertEqual(g.skip_reason(),'rule_gate_start_not_reached')
        g.decide({'elapsed_task_time':feature(2)},.1,2,0)
        self.assertEqual(g.skip_reason(),'rule_group_not_triggered')


class MetricsTests(unittest.TestCase):
    def test_skipped_injection_never_counts_as_hit(self):
        s=campaign.load_settings(campaign.HERE/'rule_groups.json')
        result=evaluate(dict(injection={'actually_injected':False}),s)
        self.assertTrue(all(v is None for v in result['approximate_labels'].values()))

    def test_collision_multilabel_and_strict_unknown(self):
        s=campaign.load_settings(campaign.HERE/'rule_groups.json')
        trace=[dict(time_s=.1,positions={'1':[0,0],'2':[.05,0]},sp_index=0,fsm='RUNNING',planning_state='LOCAL_APF')]
        t=dict(injection={'actually_injected':True,'nominal_tx_time':.05},trace=trace,params={'ViconRate':100,'crash_uav_dist':.1},
            obstacles=[[[-1,-1],[1,-1],[1,1],[-1,1]]],trajectory_setpoints=[[0,0],[1,0]],messages=[],status='collision',time_s=.1,
            rule_gate={'selected':{'reference_sp_index':0}})
        r=evaluate(t,s)
        self.assertTrue(r['strict_labels']['drone_drone']);self.assertTrue(r['strict_labels']['drone_obstacle'])
        self.assertIsNone(r['strict_labels']['planning']);self.assertIsNone(r['approximate_labels']['delay'])
        self.assertIsNone(r['approximate_labels']['deadlock']);self.assertTrue(r['approximate_labels']['planning'])


class FakeSimulation:
    def __init__(self,episode,out,seed,config,injection_policy):
        self.episode=episode;self.seed=seed;self.policy=injection_policy;self.plan={}
        self.params=SimpleNamespace(ViconRate=100,num_robots=4)
    def start(self):
        return dict(episode=self.episode,seed=self.seed,status='success',note='',
            injection=dict(actually_injected=False,skip_reason='rule_group_not_triggered'),rule_gate=self.policy.report())


class CampaignTests(unittest.TestCase):
    def test_seed_plan_new_reproducible_and_disjoint(self):
        a=campaign.seed_plan(123,600,range(1000));self.assertEqual(a,campaign.seed_plan(123,600,range(1000)))
        self.assertEqual(len(set(a)),600);self.assertFalse(set(a)&set(range(1000)))

    def test_config_rejects_future_fields(self):
        s=campaign.load_settings(campaign.HERE/'rule_groups.json');s['groups'][0]['rules'][0]['conditions'][0]['field']='future_3s_min_pair_distance'
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'rules.json';p.write_text(json.dumps(s))
            with self.assertRaises(ValueError):campaign.load_settings(p)

    def test_all_six_groups_progress_and_skip_denominators(self):
        settings=campaign.load_settings(campaign.HERE/'rule_groups.json')
        with tempfile.TemporaryDirectory() as temp,contextlib.redirect_stdout(io.StringIO()):
            out=Path(temp);campaign.dump(out/'manifest.json',{})
            campaign.run_trials(out,settings,2,123,4,1,sim_factory=FakeSimulation)
            p=json.loads((out/'progress.json').read_text())
            self.assertTrue(p['all_target_groups_complete']);self.assertEqual(p['completed'],12)
            self.assertEqual(p['groups_complete'],6);self.assertEqual(p['status'],'complete')
            self.assertTrue(all(r['skipped']==2 and r['valid_n']==0 and r['approximate_hits']==0 for r in p['targets']))
            self.assertEqual(len(list(out.glob('groups/*/trials/trial_*.json'))),12)
            self.assertIn('六组均完成',(out/'report.md').read_text())

    def test_launch_creates_unique_outputs(self):
        with tempfile.TemporaryDirectory() as temp,patch.object(campaign,'preflight',return_value={}),patch.object(campaign.subprocess,'Popen',return_value=SimpleNamespace(pid=123)),contextlib.redirect_stdout(io.StringIO()):
            args=argparse.Namespace(rules=str(campaign.HERE/'rule_groups.json'),output_parent=temp,episodes=1,seed=123,max_ticks=4,rrt_iterations=1)
            a=campaign.start(args);b=campaign.start(args)
            self.assertNotEqual(a,b);self.assertTrue((a/'manifest.json').exists());self.assertTrue((b/'manifest.json').exists())


if __name__=='__main__':unittest.main()
