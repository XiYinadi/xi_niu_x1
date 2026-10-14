"""Watchdog tests use spies only: no sockets or GPIO, no motion output."""
import threading
import time
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock
from navigation_runtime import NavigationRuntime
from navigation_test import config


class Watchdogs(unittest.TestCase):
    def setUp(self):
        self.r = r = NavigationRuntime.__new__(NavigationRuntime)
        r.lock=threading.RLock();r.config=config();r.navigating=True
        r.running=lambda:True;r.process=None;r.pending=False;r.epoch=1
        r.last_client=r.last_command=time.monotonic();r.last_pose=None;r.pose_ok=True
        r.lifecycle={'amcl':3,'map_server':3};r.command=(.1,0);r.nav_owner='A'
        r.armed=lambda:True;r.owner=lambda:'A';r.send_drive=Mock();r.disarm=Mock()
        r.goal_handle=Mock()
        nano=time.time_ns()
        stamp=S(sec=nano//10**9,nanosec=nano%10**9)
        r.node=S(_lock=threading.Lock(),_last_scan_t=time.monotonic(),
                 get_clock=lambda:S(now=lambda:S(nanoseconds=time.time_ns())))
        r.tf=Mock();r.tf.lookup_transform.return_value=S(header=S(stamp=stamp),
             transform=S(translation=S(x=0,y=0),rotation=S(x=0,y=0,z=0,w=1)))

    def stopped(self):
        self.r.tick()
        self.assertFalse(self.r.navigating)
        self.r.send_drive.assert_called_with(0.,0.,False)
        self.r.disarm.assert_called_once()

    def test_valid_gates_pass_only_scaled_command(self):
        self.r.tick();self.r.send_drive.assert_called_once_with(.2,0.,True)

    def test_stale_scan_stops(self):
        self.r.node._last_scan_t-=1;self.stopped()

    def test_stale_client_stops_and_latches(self):
        self.r.last_client-=1;self.stopped()
        self.r.last_client=time.monotonic();self.r.send_drive.reset_mock()
        self.r.tick();self.r.send_drive.assert_not_called()

    def test_stale_command_stops(self):
        self.r.last_command-=1;self.stopped()

    def test_owner_change_stops(self):
        self.r.owner=lambda:'B';self.stopped()

    def test_disarmed_stops(self):
        self.r.armed=lambda:False;self.stopped()

    def test_bad_tf_stops(self):
        self.r.tf.lookup_transform.side_effect=RuntimeError('no TF');self.stopped()

    def test_uncalibrated_stops(self):
        self.r.config['calibrated']=False;self.stopped()

    def test_waiting_for_first_command_only_sends_zero(self):
        self.r.last_command=None;self.r.tick()
        self.r.send_drive.assert_called_once_with(0.,0.,True)


if __name__=='__main__':unittest.main()
