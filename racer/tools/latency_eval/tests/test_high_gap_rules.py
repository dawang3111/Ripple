import unittest
import numpy as np
from latency_eval.high_gap_rules import mine,passes
from latency_eval.rule_cohorts import cohort_metrics

class HighGapRuleTests(unittest.TestCase):
    def test_mine_only_positive_twenty_pp_train_rules(self):
        x=np.asarray([[1.] for _ in range(80)]+[[0.] for _ in range(20)]+[[1.] for _ in range(20)]+[[0.] for _ in range(80)])
        y=np.asarray([True]*100+[False]*100)
        rules=mine(x,y,[],beam_width=8)
        self.assertTrue(rules)
        for rule,m,score in rules:
            self.assertGreaterEqual(m['trigger_difference_pp'],20)
            self.assertGreater(score,0)

    def test_filters_negative_small_sample_and_weak_gap(self):
        def metrics(pos,neg):
            x=np.asarray([[1.]]*pos+[[0.]]*(100-pos)+[[1.]]*neg+[[0.]]*(100-neg))
            return cohort_metrics(x,np.asarray([True]*100+[False]*100),((0,'>',.5),))
        self.assertTrue(passes(metrics(80,20)))
        self.assertFalse(passes(metrics(20,80)))
        self.assertFalse(passes(metrics(55,50)))
        m=metrics(80,20);m['miss_n']=3;self.assertFalse(passes(m))
        m=metrics(80,20);m['trigger_difference_ci_low_pp']=-1;self.assertFalse(passes(m))

    def test_relaxed_gap_is_explicit_and_does_not_change_default(self):
        x=np.asarray([[1.]]*130+[[0.]]*70+[[1.]]*96+[[0.]]*104)
        y=np.asarray([True]*200+[False]*200)
        m=cohort_metrics(x,y,((0,'>',.5),))
        self.assertGreater(m['trigger_difference_ci_low_pp'],0)
        self.assertTrue(passes(m,min_gap=15))
        self.assertFalse(passes(m))

    def test_missing_fields_cannot_manufacture_support(self):
        x=np.asarray([[1.]]*10+[[np.nan]]*90+[[0.]]*100)
        y=np.asarray([True]*100+[False]*100)
        self.assertEqual(mine(x,y,[],beam_width=8),[])

if __name__=='__main__':unittest.main()
