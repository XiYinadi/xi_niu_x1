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
        # 2026-10-28: 巡检执行器是 tick() 的必经依赖；夹具补一个空桩，
        # 否则九个看门狗用例全部 AttributeError（审阅报告里的「未有效执行」）。
        r.patrol_runner=S(active=False, status='idle', tick=lambda: None,
                         cancel=lambda reason=None: None)
        # 看门狗去抖状态（_sensor_lost / _command_lost 会读写它们）
        r._stale_from={};r._command_grace_until=0.0;r._command_stale_from=None
        r._clearance_unknown_from=None;r.trial_clearance=None;r.trial_started=None
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
        # 2026-10-28: 传感器超时带 0.5 秒去抖，单拍不会停；把去抖起点
        # 预置成 1 秒前，才能在单拍内验证「会停」。
        self.r.node._last_scan_t-=1
        self.r._stale_from={'scan':time.monotonic()-1}
        self.stopped()

    def test_stale_client_stops_and_latches(self):
        self.r.last_client-=1;self.stopped()
        self.r.last_client=time.monotonic();self.r.send_drive.reset_mock()
        self.r.tick();self.r.send_drive.assert_not_called()

    def test_stale_command_stops(self):
        # 同上：指令超时要求持续 0.5 秒，预置去抖起点。
        self.r.last_command-=1
        self.r._command_stale_from=time.monotonic()-1
        self.stopped()

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

    # ── 2026-10-28: 审阅报告 P1-1 / P1-2 的回归 ────────────────────
    def _patrol(self, status):
        self.r.patrol_runner=S(active=True,status=status,tick=lambda:None,
                               cancel=lambda reason=None:None)

    def test_patrol_dwell_exempts_command_timeout(self):
        # 停稳/直行段由巡检自己发指令，last_command 不更新，指令超时不能判死。
        self._patrol('dwell')
        self.r.last_command-=1
        self.r._command_stale_from=time.monotonic()-1
        self.r.tick()
        self.assertTrue(self.r.navigating)

    def test_patrol_navigating_still_honours_command_timeout(self):
        # Nav2 在开车的阶段，指令超时必须照常停（不能被巡检整段清空）。
        self._patrol('navigating')
        self.r.last_command-=1
        self.r._command_stale_from=time.monotonic()-1
        self.r.tick()
        self.assertFalse(self.r.navigating)
        self.r.disarm.assert_called_once()

    def test_pose_jump_cancels(self):
        # 参考样本 0.3 秒前在原点，这一拍 TF 跳到 1.4 米外 -> 必须取消。
        self.r._pose_check=(time.monotonic()-.3,0.,0.)
        nano=time.time_ns()
        self.r.tf.lookup_transform.return_value=S(
            header=S(stamp=S(sec=nano//10**9,nanosec=nano%10**9)),
            transform=S(translation=S(x=1.4,y=0.),rotation=S(x=0,y=0,z=0,w=1)))
        self.r.tick()
        self.assertFalse(self.r.navigating)
        self.r.disarm.assert_called_once()


if __name__=='__main__':unittest.main()
