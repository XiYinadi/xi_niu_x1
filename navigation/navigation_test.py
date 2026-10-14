import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from map_store import MapStore
from navigation_core import fresh, twist_to_joystick


def config():
    return dict(calibrated=True, wheelbase_m=.3, length_m=.444, width_m=.36,
        max_steering_rad=.5, full_throttle_mps=.5, steering_sign=-1,
        max_speed_mps=.1, command_timeout_s=.25, scan_timeout_s=.5,
        localization_timeout_s=3.)


class Maps(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.store = MapStore(self.temp.name)
        self.msg = S(info=S(width=2, height=2, resolution=.05,
            origin=S(position=S(x=1., y=-2.), orientation=S(x=0., y=0.,
                 z=math.sin(.4), w=math.cos(.4)))), data=[0,100,50,-1])

    def tearDown(self):
        self.temp.cleanup()

    def test_pixels_orientation_and_integer_overflow(self):
        file = self.store.save(self.msg, 'room')
        self.assertEqual(file.with_suffix('.pgm').read_bytes(), b'P5\n2 2\n255\n'+bytes([127,205,254,0]))

    def test_origin_is_yaw(self):
        text = self.store.save(self.msg, 'room').read_text()
        self.assertIn('0.8',text)

    def test_no_overwrite(self):
        self.store.save(self.msg, 'room')
        with self.assertRaises(ValueError): self.store.save(self.msg, 'room')
        self.assertEqual(self.store.list(), ['room'])

    def test_reject_path_escape(self):
        for name in ('../bad', '/tmp/bad', '', 'x;reboot'):
            with self.assertRaises(ValueError): self.store.save(self.msg, name)

    def test_reject_empty_map(self):
        self.msg.data = [-1]*4
        with self.assertRaises(ValueError): self.store.save(self.msg, 'empty')
        self.assertEqual(self.store.list(), [])


class Motion(unittest.TestCase):
    def test_uncalibrated_never_moves(self):
        c=config(); c['calibrated']=False
        with self.assertRaises(ValueError): twist_to_joystick(.1, 0, c)

    def test_straight(self):
        self.assertEqual(twist_to_joystick(.1,0,config()),(.2,0.))

    def test_preserve_curvature_when_speed_limited(self):
        l,a=twist_to_joystick(.3,.1,config())
        self.assertAlmostEqual(l,.2)
        self.assertAlmostEqual(a,-math.atan(.1)/.5)

    def test_no_spin_reverse_nonfinite_or_unreachable_turn(self):
        for v,w in ((0,1),(-.1,0),(float('nan'),0),(.1,1)):
            with self.assertRaises(ValueError): twist_to_joystick(v,w,config())

    def test_timeout_boundary(self):
        self.assertTrue(fresh(1.,.8,.25))
        self.assertFalse(fresh(1.,.75,.25))
        self.assertFalse(fresh(1.,None,.25))
        self.assertFalse(fresh(1.,2.,.25))


if __name__=='__main__': unittest.main()
