import unittest
from copy import deepcopy
from adaptive_swarm.scripts.layered_planner.latency_eval.user_targets import mismatch_events, calibrate, relabel
from adaptive_swarm.scripts.layered_planner.latency_eval.core import TARGETS


def trial(time=2.,status='success',trace=None,messages=None):
    if trace is None:trace=[{'time_s':.01,'positions':{'1':[0.,0.],'2':[1.,0.]}}]
    return dict(episode=1,status=status,time_s=time,labels={t:False for t in TARGETS},
        known={t:True for t in TARGETS},injection={'actually_injected':False},
        trace=trace,messages=messages or [],params={'crash_uav_dist':.1,'ViconRate':100},
        obstacles=[[[10,10],[11,10],[11,11],[10,11]]],trajectory_setpoints=[[0,0],[1,1]])


class UserTargets(unittest.TestCase):
    def test_baseline_mean_and_zero_guard(self):
        c=calibrate(iter([trial(2),trial(4),trial(9,'collision')]))
        self.assertEqual(c['completion_mean_s'],3)
        self.assertEqual(c['delay_threshold_s'],6)
        self.assertEqual(c['mismatch_count_mean'],0)
        self.assertEqual(c['drift_count_threshold'],1)
        self.assertFalse(relabel(trial(),c)['labels']['drift'])

    def test_delay_requires_success_and_strict_boundary(self):
        c=calibrate([trial(2)])
        self.assertFalse(relabel(trial(4),c)['labels']['timeout'])
        self.assertTrue(relabel(trial(4.01),c)['labels']['timeout'])
        self.assertFalse(relabel(trial(9,'timeout'),c)['labels']['timeout'])

    def test_deadlock_full_five_seconds_zero_distance(self):
        c=calibrate([trial()])
        states=[{'time_s':i/100,'positions':{'1':[0.,0.],'2':[1+i/100,0.]}} for i in range(501)]
        self.assertTrue(relabel(trial(5,trace=states),c)['labels']['deadlock'])
        self.assertFalse(relabel(trial(4.99,trace=states[:-1]),c)['labels']['deadlock'])
        gap=[states[0],states[-1]]
        self.assertFalse(relabel(trial(5,trace=gap),c)['labels']['deadlock'])

    def test_round_trip_is_not_zero_cumulative_movement(self):
        states=[{'time_s':i/100,'positions':{'1':[i%2,0.],'2':[2+i%2,0.]}} for i in range(501)]
        self.assertFalse(relabel(trial(5,trace=states),calibrate([trial()]))['labels']['deadlock'])

    def test_read_time_mismatch_preserves_newer_truth_on_delayed_release(self):
        events=[dict(event='release',sender_id=1,seq=0,payload=[0,0],generated_time=0,release_time=0),
            dict(event='enqueue',sender_id=1,seq=1,payload=[1,0],generated_time=.01),
            dict(event='receive',sender_id=1,receiver_id=2,seq=0,payload=[0,0],generated_time=0,receive_time=.012),
            dict(event='release',sender_id=1,seq=1,payload=[1,0],generated_time=.01,release_time=.015),
            dict(event='receive',sender_id=1,receiver_id=2,seq=1,payload=[1,0],generated_time=.01,receive_time=.016)]
        r=trial(messages=events);self.assertEqual(len(mismatch_events(r)),1)
        c=calibrate([trial()]);a=relabel(r,c)
        self.assertTrue(a['labels']['drift']);self.assertEqual(a['evidence']['drift'][0]['time_s'],.012)

    def test_planning_unknown_and_old_labels_preserved(self):
        r=trial();r['labels']['planning_failure']=True
        a=relabel(r,calibrate([trial()]))
        self.assertFalse(a['known']['planning_failure'])
        self.assertTrue(a['legacy_labels']['planning_failure'])

    def test_boundary_collision_multilabel(self):
        r=trial(trace=[{'time_s':.01,'positions':{'1':[0,0],'2':[.1,0]}}])
        r['obstacles']=[[[0,0],[1,0],[1,1],[0,1]]]
        a=relabel(r,calibrate([trial()]))
        self.assertTrue(a['labels']['drone_drone']);self.assertTrue(a['labels']['drone_obstacle'])

if __name__=='__main__':unittest.main()
