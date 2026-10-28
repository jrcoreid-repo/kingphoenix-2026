"""Deterministic tests of dry-run control intent, not flight trials."""
import unittest
from .guidance import GuidanceController

class GuidanceTests(unittest.TestCase):
    def test_limited_proportional_intent(self):
        d=GuidanceController().step(0,1.,-1.,True)
        self.assertEqual(d.state,"ALIGN")
        self.assertEqual(d.lateral_mps,-.3)
        self.assertEqual(d.vertical_mps,.3)
        self.assertFalse(d.tx_enabled)
    def test_one_second_dwell(self):
        g=GuidanceController()
        self.assertEqual(g.step(0,.08,.06,True).state,"ALIGN")
        self.assertEqual(g.step(.8,.02,.02,True).state,"ALIGN")
        self.assertEqual(g.step(1.01,.02,.02,True).state,"SUCCESS")
    def test_hold_recovery(self):
        g=GuidanceController()
        g.step(0,.3,.2,True)
        self.assertEqual(g.step(.3,feedback_valid=False).state,"HOLD")
        self.assertEqual(g.step(1.,.2,.1,True).state,"ALIGN")
    def test_abort_lost_feedback(self):
        g=GuidanceController()
        g.step(0,.3,.3,True)
        g.step(.1,feedback_valid=False)
        self.assertEqual(g.step(2.11,feedback_valid=False).state,"ABORT")
if __name__=="__main__": unittest.main()
