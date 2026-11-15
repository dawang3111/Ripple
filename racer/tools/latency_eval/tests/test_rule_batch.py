import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from latency_eval.core import ROOT,TARGETS,config
from latency_eval.rule_batch import plans,summarize,validate_rule_trial,worker
from latency_eval.rule_online import OnlineRules,match_rules
from latency_eval.ros_capture import make_launch

RULE=dict(rule_id='test',conditions=[dict(field='source_active_grid_count',op='>',threshold=20.)])
def vector(n):return dict(fields={'source_active_grid_count':dict(value=n,missing=n is None)})

class RuleBatchTests(unittest.TestCase):
    def test_all_six_interleaved_exact_600_distinct_seeds(self):
        packet=dict(config=config(),groups={t:dict(rules=[RULE],combine='OR') for t in TARGETS})
        result=list(plans(100,20261111,packet,'/tmp/map.pcd'))
        self.assertEqual(len(result),600);self.assertEqual(len({p['seed'] for p in result}),600)
        self.assertEqual([p['target'] for p in result[:6]],list(TARGETS))
        self.assertTrue(all(sum(p['target']==t for p in result)==100 for t in TARGETS))
        self.assertEqual(result,list(plans(100,20261111,packet,'/tmp/map.pcd')))

    def test_or_missing_and_exact_comparison(self):
        self.assertEqual(match_rules(vector(21),[RULE])[0],['test'])
        self.assertEqual(match_rules(vector(20),[RULE])[0],[])
        self.assertEqual(match_rules(vector(None),[RULE])[0],[])
        other=dict(rule_id='missing',conditions=[dict(field='unknown',op='>',threshold=0)])
        self.assertEqual(match_rules(vector(21),[RULE,other])[0],['test'])

    def test_monitor_waits_for_task_and_active_then_arms_only_once(self):
        cfg=config();plan=dict(source_id=1,target='drift',rule_group=dict(rules=[RULE]))
        monitor=OnlineRules(plan,cfg)
        self.assertIsNone(monitor.evaluate(6_000_000_000,6.,True))
        monitor.observe(dict(kind='task_started',id=1,observed_ns=1_000_000_000,mono_ns=1_000_000_000,ros_s=1.))
        with patch('latency_eval.rule_online.extract',return_value=vector(21)) as extract:
            self.assertIsNone(monitor.evaluate(6_000_000_000,6.,False))
            decision=monitor.evaluate(6_000_000_000,6.,True)
            self.assertEqual(decision['expires_ns'],6_150_000_000)
            self.assertIsNone(monitor.evaluate(6_050_000_000,6.05,True))
            extract.assert_called_once()

    def test_rule_contract_rejects_expiry_and_changed_generation_state(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);(d/'capture.json').write_text('{}')
            decision=dict(features=vector(21),decision_ns=100,expires_ns=200,matched_rule_ids=['test'])
            (d/'rule_decision.json').write_text(json.dumps(decision))
            def result(t,n):return dict(actual_injected=True,contract_valid=True,generated_ns=t,features=vector(n),rule_group=dict(rules=[RULE]))
            self.assertTrue(validate_rule_trial(result(150,21),d)['rule_contract_valid'])
            self.assertEqual(validate_rule_trial(result(201,21),d)['rule_skip_reason'],'decision_expired_before_generation')
            self.assertEqual(validate_rule_trial(result(150,20),d)['rule_skip_reason'],'rule_no_longer_true_at_message_generation')
            r=result(150,21);r['actual_injected']=False
            self.assertFalse(validate_rule_trial(r,d)['rule_contract_valid'])

    def test_summary_completion_separate_from_injection_and_multilabel(self):
        labels={t:dict(valid=True,hit=True) for t in TARGETS}
        result=dict(target=TARGETS[0],status='completed',actual_injected=True,rule_contract_valid=True,labels=labels)
        rows=summarize([result],1)
        self.assertEqual(rows[0]['state'],'completed');self.assertEqual(rows[0]['hit_n'],1)
        self.assertEqual(rows[1]['hit_n'],0);self.assertEqual(rows[1]['state'],'pending')
        result['rule_contract_valid']=False
        self.assertEqual(summarize([result],1)[0]['valid_target_n'],0)

    def test_rule_launch_is_explicit_opt_in(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);plan=dict(source_id=1,offset_s=5,fraction=.5,rule_group=dict(rules=[RULE]))
            p=make_launch(root,plan,config(),'/tmp/map.pcd')
            self.assertIn('name="/racer_eval/rule_mode" value="true"',p.read_text())

    def test_worker_finishes_all_targets_and_resume_does_not_repeat(self):
        with tempfile.TemporaryDirectory() as tmp:
            job=Path(tmp);run=job/'run';run.mkdir();cfg=config()
            (run/'config.json').write_text(json.dumps(cfg))
            (job/'job.json').write_text(json.dumps(dict(per_target=1,implementation={})))
            packet=dict(config=cfg,groups={t:dict(rules=[RULE]) for t in TARGETS})
            for plan in plans(1,22,packet,'/tmp/map.pcd'):
                d=run/plan['trial_id'];d.mkdir();(d/'plan.json').write_text(json.dumps(plan))
            def capture(d,cfg):(d/'capture.json').write_text(json.dumps(dict(status='completed')))
            def evaluate(d,cfg):
                plan=json.loads((d/'plan.json').read_text())
                return dict(plan,status='completed',actual_injected=False,contract_valid=False,features=None,
                    labels={t:dict(hit=False,valid=False,missing_reason='not_injected',evidence=[]) for t in TARGETS},
                    result_path=str(d/'result.json'))
            with patch('os.nice'),patch('signal.signal'),patch('latency_eval.rule_batch.fingerprint',return_value={}), \
                 patch('latency_eval.rule_batch.resources',return_value=dict(disk_free_bytes=20*1024**3,mem_available_bytes=4*1024**3)), \
                 patch('latency_eval.rule_batch.experiment_processes',return_value=[]),patch('latency_eval.rule_batch.archive_finished'), \
                 patch('latency_eval.rule_batch.cleanup_owned'),patch('latency_eval.rule_batch.execute_capture',side_effect=capture) as execute, \
                 patch('latency_eval.rule_batch.evaluate_trial',side_effect=evaluate):
                self.assertEqual(worker(job),0)
                self.assertEqual(execute.call_count,6)
                self.assertEqual(worker(job),0)
                self.assertEqual(execute.call_count,6)
            p=json.loads((job/'progress.json').read_text())
            self.assertTrue(p['all_targets_completed']);self.assertEqual(p['finished_rounds'],6)
            self.assertTrue(all(r['state']=='completed' for r in p['targets']))
            self.assertEqual(p['actual_injected_rounds'],0)

    def test_real_cpp_gate_waits_for_arm_and_preserves_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);src=p/'test.cpp'
            src.write_text('''#include <exploration_manager/tx_delay_gate.h>
#include <cassert>
int main(){racer_eval::TxDelayGate<int> g;g.enabled=true;g.wait_for_arm=true;
for(int i=1;i<=4;++i){auto v=g.message(i,40000000LL*i,true,true,6.,1000000000);assert(v.size()==1&&v[0].kind=="forward");}
g.armed=true;auto v=g.message(5,200000000,true,true,6.,1000000000);assert(v.size()==1&&v[0].kind=="scheduled");
g.armed=false;v=g.message(6,240000000,true,true,6.,1000000000);assert(v.size()==2&&v[0].kind=="release"&&v[0].message==5&&v[1].message==6);
}''')
            include=ROOT/'swarm_exploration/exploration_manager/include'
            subprocess.run(['g++','-std=c++11','-I'+str(include),str(src),'-o',str(p/'test')],check=True,capture_output=True)
            subprocess.run([str(p/'test')],check=True,capture_output=True)

if __name__=='__main__':unittest.main()
