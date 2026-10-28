"""短段直行控制器回归：进度计时、净空、传感器新鲜度、到点判定。"""
import threading
import time
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock

from patrol_runner import PatrolRunner


def config():
    return dict(scan_timeout_s=.4, odom_timeout_s=.4, imu_timeout_s=.4,
                trial_clearance_m=.20)


class Direct(unittest.TestCase):
    def setUp(self):
        now = time.monotonic()
        node = S(_lock=threading.Lock(), _last_scan_t=now, _last_odom_t=now, _last_imu_t=now)
        nav = S(config=config(), node=node, armed=lambda: True, send_drive=Mock(),
                last_pose=dict(x=0., y=0., yaw=0.), trial_clearance=.5)
        r = self.r = PatrolRunner.__new__(PatrolRunner)
        r.nav = nav
        r.lock = threading.RLock()
        r.active = True
        r.total = 1
        r.index = 0
        r.leg_target = dict(x=2., y=0., yaw=0.)
        r.status = 'direct'
        r.phase_started = now
        r._best_dist = 2.0
        r._best_t = now
        r.message = ''

    def test_progress_refreshes_timer(self):
        # 段已经跑了 13 秒，但这一拍确实靠近了：不应误报「没有前进」。
        self.r.phase_started = self.r._best_t = time.monotonic() - 13
        self.r._best_dist = 2.0
        self.r.nav.last_pose = dict(x=.5, y=0., yaw=0.)   # 距离 1.5
        self.r._drive_direct(time.monotonic())
        self.r.nav.send_drive.assert_called()

    def test_no_progress_aborts(self):
        self.r._best_t = time.monotonic() - 13
        self.r.nav.last_pose = dict(x=.0, y=0., yaw=0.)
        with self.assertRaises(ValueError):
            self.r._drive_direct(time.monotonic())

    def test_clearance_too_small_aborts(self):
        self.r.nav.trial_clearance = .10
        with self.assertRaises(ValueError):
            self.r._drive_direct(time.monotonic())

    def test_clearance_unknown_stops_then_aborts(self):
        self.r.nav.trial_clearance = None
        self.r._drive_direct(time.monotonic())
        self.assertEqual(self.r.nav.send_drive.call_args.args[:2], (0., 0.))
        self.r.no_clearance = time.monotonic() - 4
        with self.assertRaises(ValueError):
            self.r._drive_direct(time.monotonic())

    def test_stale_sensor_stops_then_aborts(self):
        self.r.nav.node._last_scan_t = time.monotonic() - 2
        self.r._drive_direct(time.monotonic())
        self.assertEqual(self.r.nav.send_drive.call_args.args[:2], (0., 0.))
        self.r.stale_since = time.monotonic() - 2
        with self.assertRaises(ValueError):
            self.r._drive_direct(time.monotonic())

    def test_arrival_sets_epoch_time(self):
        self.r.nav.last_pose = dict(x=1.95, y=0., yaw=0.)
        before = time.time()
        self.r._drive_direct(time.monotonic())
        self.assertEqual(self.r.status, 'dwell')
        self.assertGreaterEqual(self.r.arrived_at, before)


if __name__ == '__main__':
    unittest.main()
