import copy
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

from latency_eval.core import ROOT, TARGETS, config
from latency_eval.accuracy_batch import plans, main
from latency_eval.accuracy_metrics import window_evidence


def window_fixture():
    t0=1_050_000_000; selected=t0+2_000_000_000
    origin=dict(kind='accuracy_window',id=1,window_id=1,action='triggered',trigger_ns=t0,
                action_ns=t0,decision_ns=1_000_000_000,expires_ns=1_150_000_000,
                wait_ns=1_000_000_000,original_seq=1,selected_seq=0,fsm='PUB_TRAJ')
    events=[dict(kind='intensity_rule_decision',decision_ns=1_000_000_000,expires_ns=1_150_000_000,source_id=1,matched_rule_ids=['rule1']),
            dict(kind='accuracy_publication',id=1,seq=1,generated_ns=t0,new_trajectory=True,eligible=True),origin,
            dict(kind='intensity_tx',id=1,seq=1,action='forward',generated_ns=t0,publish_before_ns=t0+10),
            dict(kind='accuracy_publication',id=1,seq=2,generated_ns=t0+1_500_000_000,new_trajectory=False,eligible=True),
            dict(kind='accuracy_publication',id=1,seq=3,generated_ns=selected,new_trajectory=True,eligible=True),
            dict(origin,action='selected',action_ns=selected,selected_seq=3),
            dict(kind='intensity_tx',id=1,seq=3,action='scheduled',generated_ns=selected),
            dict(origin,action='completed',action_ns=selected+50_000_000,selected_seq=3,reason='successor_G5')]
    release=dict(sender_id=1,seq=3,release_ns=selected+50_000_000,actual_delay_s=.05,completed_injection=True,reason='successor_G5')
    return events,[release]


class AccuracyTests(unittest.TestCase):
    def test_cpp_five_waits_skip_do_not_buffer_and_no_recheck(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);src=d/'test.cpp'
            src.write_text('''#include <exploration_manager/accuracy_window_gate.h>
#include <exploration_manager/intensity_delay_gate.h>
#include <cassert>
int main(){for(long wait:{0L,1L,2L,3L,5L}){
racer_eval::AccuracyWindowGate w;w.wait_ns=wait*1000000000LL;
racer_eval::IntensityDelayGate<int> tx;tx.enabled=true;tx.group=5;
const long long t=1000000000LL;
auto a=w.message(t,tx.next_seq(),true,false,true,t,t+150000000);assert(!a.triggered);
a=w.message(t,tx.next_seq(),false,true,true,t,t+150000000);assert(!a.triggered);
a=w.message(t,tx.next_seq(),true,true,true,t,t+150000000);assert(a.triggered&&a.selected==(wait==0));
auto packets=tx.message(11,t,a.selected,a.decision_ns);assert(packets[0].kind==(wait?"forward":"scheduled"));
if(wait){
a=w.message(t+wait*1000000000LL-1,tx.next_seq(),true,true,true,t,t+150000000);assert(!a.selected);
packets=tx.message(12,t+wait*1000000000LL-1,false);assert(packets[0].kind=="forward");
a=w.message(t+wait*1000000000LL,tx.next_seq(),false,true,false,0,0);assert(!a.selected);
a=w.message(t+wait*1000000000LL+10,tx.next_seq(),true,true,false,0,0);assert(a.selected&&!a.triggered&&a.decision_ns==t);
packets=tx.message(13,t+wait*1000000000LL+10,a.selected,a.decision_ns);assert(packets[0].kind=="scheduled");
}
assert(tx.timer(t+100000000000LL).empty());
a=w.message(t+100000000000LL,tx.next_seq(),true,true,true,t,t+150000000);assert(!a.selected&&!a.triggered);
packets=tx.message(14,t+100000000000LL,false);assert(packets.size()==2&&packets[0].kind=="release"&&packets[1].kind=="forward");
w.reset();a=w.message(t+200000000000LL,tx.next_seq(),true,true,true,t,t+150000000);assert(a.triggered&&a.id==2);
w.reset();assert(w.state()==racer_eval::AccuracyWindowGate::MONITORING);
}}
''')
            subprocess.run(['g++','-std=c++11','-I'+str(ROOT/'swarm_exploration/exploration_manager/include'),str(src),'-o',str(d/'test')],check=True,capture_output=True)
            subprocess.run([str(d/'test')],check=True,capture_output=True)

    def test_audit_expired_token_is_valid_at_origin_not_rechecked(self):
        events,releases=window_fixture();rows,errors=window_evidence(events,{'group':'G2'},releases)
        self.assertEqual(errors,[]);self.assertEqual(rows[0]['actual_wait_s'],2.)
        self.assertFalse(rows[0]['original_token_fresh_at_selection'])
        self.assertTrue(rows[0]['rule_evidence_valid_at_origin'])
        self.assertEqual(rows[0]['actual_packet_delay_s'],.05)

    def test_audit_rejects_buffering_original_and_skipping_first_postwait_publication(self):
        events,releases=window_fixture()
        events[3]['action']='scheduled'
        rows,errors=window_evidence(events,{'group':'G2'},releases)
        self.assertTrue(any('original_message_was_not_forwarded' in e for e in errors))
        events,releases=window_fixture();events[4]['new_trajectory']=True
        rows,errors=window_evidence(events,{'group':'G2'},releases)
        self.assertTrue(any('not_first_eligible_new_publication' in e for e in errors))

    def test_wait_cancelled_without_next_message_is_not_injection(self):
        events,_=window_fixture();events=events[:4]
        events.append(dict(events[2],action='cancelled',reason='task_end_before_next_eligible_publication'))
        rows,errors=window_evidence(events,{'group':'G2'},[])
        self.assertEqual(errors,[]);self.assertFalse(rows[0]['completed_injection'])
        self.assertIsNone(rows[0]['actual_wait_s'])

    def test_pairing_and_1500_plans(self):
        a=list(plans(list(TARGETS),['G1','G2','G3','G4','G5'],50,20261115,{'config':config()}))
        self.assertEqual(len(a),1500)
        for i in range(0,len(a),5):
            self.assertEqual(len({(p['seed'],p['source_id']) for p in a[i:i+5]}),1)
            self.assertEqual([p['configured_wait_s'] for p in a[i:i+5]],[0.,1.,2.,3.,5.])
            self.assertTrue(all(p['accuracy_mode'] for p in a[i:i+5]))

    def test_unique_output_and_frozen_rules_not_mutated(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);rules=d/'rules.json';baseline=d/'baseline.json'
            packet={'config':config(),'groups':{}}
            rules.write_text(json.dumps(packet));saved=rules.read_bytes()
            baseline.write_text(json.dumps({'mean_completed_task_s':151.,'mean_broadcast_error_m':.08}))
            args=['plan','--rules',str(rules),'--baseline',str(baseline),'--output-root',tmp,'--per-cell','1']
            main(args);main(args)
            jobs=list(d.glob('accuracy_plan_*'));self.assertEqual(len(jobs),2)
            self.assertEqual(rules.read_bytes(),saved)
            for j in jobs:
                self.assertEqual((j/'frozen_rules.json').read_text(),json.dumps(packet,indent=2,ensure_ascii=False)+'\n')
                self.assertEqual(len(list((j/'run').glob('*/plan.json'))),30)

    def test_native_launch_default_unchanged_and_accuracy_strategy_shared(self):
        root=ET.parse(ROOT/'swarm_exploration/exploration_manager/launch/swarm.launch').getroot()
        args={e.get('name'):e.get('default') for e in root.findall('arg')}
        self.assertEqual(args['ripple_experiment'],'intensity');self.assertEqual(args['ripple_enabled'],'false')
        params={e.get('name'):e.get('value') for e in root.findall('param')}
        self.assertIn("5 if arg('ripple_experiment') == 'accuracy'",params['/racer_intensity/group'])

    def test_interrupted_audit_is_not_a_finished_result_and_capture_is_preserved(self):
        from latency_eval.accuracy_batch import isolate_partial_results
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp);trial=run/'trial_0001';trial.mkdir()
            (trial/'capture.json').write_text('{"status":"completed"}')
            (trial/'intensity_result.json').write_text('{"partial":true}')
            isolate_partial_results(run)
            self.assertFalse((trial/'intensity_result.json').exists())
            self.assertEqual(len(list(trial.glob('unverified_outcomes_*.json'))),1)
            self.assertTrue((trial/'capture.json').exists())
            (trial/'accuracy_result.json').write_text('{"finished":true}')
            isolate_partial_results(run)
            self.assertEqual((trial/'intensity_result.json').read_bytes(),(trial/'accuracy_result.json').read_bytes())


if __name__=='__main__':unittest.main()
