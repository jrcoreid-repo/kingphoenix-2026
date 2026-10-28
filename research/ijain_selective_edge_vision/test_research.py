"""Synthetic unit tests for scheduler logic; not evidence of flight experiments."""
import unittest
from .scheduler import Scheduler, Observation
from .geometry import causal_corner_prediction, normalized_residual

def obs(i,t,active=True,geo=.001,temporal=.002,conf=.9,reproj=1):
    return Observation(i,t,active,True,reproj,geo,temporal,True,True,.02,conf)

class Tests(unittest.TestCase):
    def test_off_phase(self):
        self.assertFalse(Scheduler().step(obs(0,0,False)).inference_executed)
    def test_max_age(self):
        s=Scheduler(inference_s=0)
        self.assertTrue(s.step(obs(0,0)).inference_executed)
        self.assertFalse(s.step(obs(1,.1)).refresh_requested)
        self.assertEqual(s.step(obs(2,.51)).reason,"maximum_age")
    def test_separate_acceptance(self):
        d=Scheduler(inference_s=0).step(obs(0,0,reproj=5))
        self.assertTrue(d.inference_executed)
        self.assertFalse(d.accepted)
    def test_coalesce(self):
        s=Scheduler("continuous",inference_s=.05)
        self.assertTrue(s.step(obs(0,0)).inference_executed)
        self.assertTrue(s.step(obs(1,.01)).coalesced)
        self.assertTrue(s.step(obs(2,.051)).inference_executed)
    def test_fixed_rate(self):
        s=Scheduler("fixed_5",inference_s=0)
        indices=[i for i in range(10) if s.step(obs(i,i/30)).inference_executed]
        self.assertEqual(indices,[0,5])
    def test_indicator_triggers(self):
        s=Scheduler("confidence_geometry",inference_s=0)
        s.step(obs(0,0,conf=.4))
        self.assertEqual(s.step(obs(1,.03)).reason,"low_confidence")
        g=Scheduler("geometry_only",inference_s=0)
        g.step(obs(0,0))
        self.assertEqual(g.step(obs(1,.03,geo=.02)).reason,"geometric_disagreement")
    def test_causal_temporal(self):
        a=[[0,0],[10,0],[10,10],[0,10]]
        b=[[2,0],[12,0],[12,10],[2,10]]
        pred=causal_corner_prediction(a,0,b,1,1.5)
        self.assertAlmostEqual(pred[0][0],3)
        self.assertAlmostEqual(normalized_residual(pred,pred),0)
if __name__=="__main__": unittest.main()
