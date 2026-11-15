import unittest
from latency_eval.core import config
from latency_eval.outcomes import classify
from latency_eval.drift_sensitivity import extract_pairs,sweep_pairs


def snapshot(t,i,error=.2,fsm='EXEC_TRAJ'):
    return dict(kind='snapshot',id=i,mono_ns=round(t*1e9),observed_ns=round(t*1e9),ros_s=t,
                position_stamp_s=t,position=[i,0,0],velocity=[0,0,0],fsm=fsm,plan_result=2,
                received_states=[dict(id=3-i,position=[3-i+error,0,0],stamp_s=t)])


class DriftSensitivityTests(unittest.TestCase):
    def check_matches_original(self,events):
        cfg=config();cfg['active_ids']=[1,2]
        distances=[.5,.2,.15,.1];durations=[1.,.2,.1,.05]
        pairs=extract_pairs(events,int(1e9),int(3e9),cfg)
        sweep=sweep_pairs(pairs,cfg,distances,durations)
        for distance in distances:
            for duration in durations:
                cfg['thresholds'].update(drift_distance_m=distance,drift_duration_s=duration)
                label=classify(events,int(1e9),int(3e9),cfg)['drift']
                key=f'{distance:.3f}/{duration:.3f}'
                self.assertEqual(key in sweep,label['hit'],key)
                if label['hit']:
                    self.assertEqual(sweep[key]['time_ns'],label['evidence'][0]['time_ns'])

    def test_strict_distance_and_duration_match_classifier(self):
        self.check_matches_original([snapshot(t,i) for t in [.99,1.,1.1,1.2,1.3,1.4] for i in [1,2]])

    def test_gap_inactive_and_below_threshold_reset(self):
        events=[snapshot(t,i,error=.01 if t==1.2 else .2,fsm='IDLE' if t==1.4 else 'EXEC_TRAJ')
                for t in [.99,1.,1.1,1.2,1.3,1.4,1.5,1.9,2.,2.1] for i in [1,2]]
        self.check_matches_original(events)

    def test_stale_positions_are_not_evidence(self):
        events=[snapshot(t,i) for t in [.99,1.,1.1,1.2] for i in [1,2]]
        for e in events:e['position_stamp_s']=e['ros_s']-.3
        self.check_matches_original(events)

    def test_sweep_is_reproducible_and_zero_error_never_hits(self):
        cfg=config();cfg['active_ids']=[1,2]
        events=[snapshot(t,i,error=0) for t in [.99,1.,1.1,1.2] for i in [1,2]]
        pairs=extract_pairs(events,int(1e9),int(3e9),cfg)
        self.assertEqual(sweep_pairs(pairs,cfg),{})
        self.assertEqual(sweep_pairs(pairs,cfg),sweep_pairs(pairs,cfg))


if __name__=='__main__':unittest.main()
