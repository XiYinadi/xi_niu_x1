import math
import unittest
from wheel_odom import validate_config
from wheel_odom_core import Quadrature, integrate, sample_fault


class WheelOdomTests(unittest.TestCase):
    def test_sampler_affinity_validation(self):
        config = dict(wheel_track_m=.3, sign_left=1, sign_right=-1, calibrated=True,
                      counts_per_meter_left=260000., counts_per_meter_right=251000., max_wheel_speed_mps=.5)
        for cpu in (-1, True, 1.5, '1'):
            with self.assertRaises(ValueError):
                validate_config(dict(config, sampler_cpu=cpu))
        for cpu in (None, 1):
            self.assertEqual(validate_config(dict(config, sampler_cpu=cpu))['sampler_cpu'], cpu)

    def test_unconfigured_scale_still_blocks_mapping(self):
        config = dict(wheel_track_m=.3, sign_left=1, sign_right=-1, calibrated=False)
        with self.assertRaises(ValueError):
            validate_config(config)
        self.assertFalse(validate_config(config, diagnostic=True)['calibrated'])

    def test_health_bound_and_stalls(self):
        p = dict(elapsed=0, invalid=[0,0], changes=[0,0,0,0])
        good = dict(elapsed=.02, invalid=[1,0], changes=[500,500,500,500])
        self.assertIsNone(sample_fault(p, good))
        bad = dict(good, invalid=[50,0])
        self.assertIsNotNone(sample_fault(p, bad))
        self.assertIsNotNone(sample_fault(p, dict(good, elapsed=.3)))

    def test_straight_and_reverse(self):
        p = integrate((0, 0, 0), 1, 1, .2)
        self.assertEqual(p, (1, 0, 0))
        self.assertEqual(integrate(p, -1, -1, .2), (0, 0, 0))

    def test_arc(self):
        # Left stationary, right travels half circumference about left wheel.
        p = integrate((0, 0, 0), 0, math.pi * .2, .2)
        self.assertAlmostEqual(p[0], 0)
        self.assertAlmostEqual(p[1], .2)
        self.assertAlmostEqual(p[2], math.pi)

    def test_invalid_geometry(self):
        for track in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                integrate((0, 0, 0), 1, 1, track)

    def test_all_initial_phases(self):
        cycle = (0, 2, 3, 1)
        for start in range(4):
            q = Quadrature(cycle[start] >> 1, cycle[start] & 1)
            for t in range(1, 5):
                prev, cur = cycle[(start+t-1)%4], cycle[(start+t)%4]
                channel = 0 if (prev ^ cur) == 2 else 1
                q.edge(t, channel, bool(cur & (2 if channel == 0 else 1)))
            self.assertEqual(q.count, 4)

    def test_merged_order(self):
        q = Quadrature(0, 0)
        a = [(1, 0, True), (3, 0, False)]
        b = [(2, 1, True), (4, 1, False)]
        for edge in sorted(a+b):
            q.edge(*edge)
        self.assertEqual(q.count, 4)

    def test_faults_not_silently_accepted(self):
        q = Quadrature(0, 0)
        q.edge(2, 0, True)
        with self.assertRaises(ValueError):
            q.edge(3, 0, True)
        with self.assertRaises(ValueError):
            q.edge(1, 1, True)


if __name__ == '__main__':
    unittest.main()
