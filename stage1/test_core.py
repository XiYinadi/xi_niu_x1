import math
import unittest
from core import Feedback, relative


class CoreTests(unittest.TestCase):
    def test_relative_pose_wrap_and_projection(self):
        result=relative((2,3,math.pi/2),(2,4,math.pi))
        self.assertAlmostEqual(result['forward_m'],1)
        self.assertAlmostEqual(result['lateral_m'],0)
        self.assertAlmostEqual(result['yaw_deg'],90)
        self.assertAlmostEqual(relative((0,0,math.radians(179)),(0,0,math.radians(-179)))['yaw_deg'],2)

    def test_hard_wheel_cap_even_on_combined_demand_and_stall(self):
        f=Feedback(1/3)
        for i in range(1000):
            l,a=f.step(.08,.2,0.,0.,.02,True)
            self.assertLessEqual(max(abs(l+a/3),abs(l-a/3)),.1000001)
        self.assertGreater(l,0)
        self.assertEqual(f.step(0,0,0,0,.02,True),(0,0))

    def test_spin_is_opposite_wheels_and_no_servo_mode_spin(self):
        f=Feedback(1/3)
        l,a=f.step(0,.15,0,0,.02,True)
        self.assertEqual(l,0);self.assertGreater(a,0)
        with self.assertRaises(ValueError):f.step(0,.15,0,0,.02,False)

    def test_limits_and_nonfinite(self):
        f=Feedback(1/3)
        for values in ((.2,0,0,0),(-.01,0,0,0),(0,.3,0,0),(math.nan,0,0,0),(0,0,math.inf,0)):
            with self.assertRaises(ValueError):f.step(*values,.02,True)


if __name__=='__main__':unittest.main()
