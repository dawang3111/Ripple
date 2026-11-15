import unittest
import numpy as np
from latency_eval.rule_cohorts import cohort_metrics,translated


class RuleCohortTests(unittest.TestCase):
    def test_conditional_direction_not_precision(self):
        x=np.array([[1.],[1.],[0.],[1.],[0.],[np.nan]])
        y=np.array([True,True,True,False,False,False])
        r=cohort_metrics(x,y,((0,'>',.5),))
        self.assertEqual(r['hit_trigger_n'],2);self.assertEqual(r['hit_n'],3)
        self.assertAlmostEqual(r['hit_trigger_rate'],2/3)
        self.assertEqual(r['miss_trigger_rate'],.5)
        self.assertEqual(r['miss_missing_n'],1)
        self.assertAlmostEqual(r['trigger_difference_pp'],100/6)

    def test_no_group_is_na_not_zero(self):
        r=cohort_metrics(np.array([[1.],[2.]]),np.array([True,True]),((0,'>',.5),))
        self.assertIsNone(r['miss_trigger_rate']);self.assertIsNone(r['trigger_difference_pp'])
        self.assertTrue(r['low_cohort_support'])

    def test_count_rounding_is_logically_equivalent(self):
        conditions=[dict(field='local_new_trajectory_count_5s',op='<=',threshold=34.258),
                    dict(field='source_active_grid_count',op='>',threshold=24.773)]
        _,text=translated(conditions,{'local_new_trajectory_count_5s':'count','source_active_grid_count':'count'})
        self.assertIn('<= 34 次',text);self.assertIn('>= 25 个',text)
        for n in range(100):
            self.assertEqual(n<=34.258,n<=34);self.assertEqual(n>24.773,n>=25)

if __name__=='__main__':unittest.main()
