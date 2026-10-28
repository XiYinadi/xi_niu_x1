"""Exercise actual SocketIO handlers with mocked ROS/PWM outputs."""
import importlib.util
import sys
from pathlib import Path
import unittest
from unittest.mock import Mock
sys.path.insert(0, '/usr/local/lib/python3.10/dist-packages')
sys.path.insert(0, '/usr/lib/python3/dist-packages')
spec=importlib.util.spec_from_file_location('web_under_test',Path(__file__).resolve().parent.parent/'web'/'slam_viz.py')
web=importlib.util.module_from_spec(spec);spec.loader.exec_module(web)


class Arbitration(unittest.TestCase):
    def setUp(self):
        web._ros_node=None;web._navigation=None;web._motor_armed=False;web._drive_owner=None
        self.context=web.app.test_request_context('/')
        self.context.push()
        self.sid='test-driver'
        web.request.sid=self.sid
        class Client:
            def emit(self, event, data):
                return {'motor_control':web._on_motor_control,
                        'mapping_control':web._on_mapping_control}[event](data)
        self.client=Client()
        web._ros_node=Mock();web._send_drive=Mock();web._send_gimbal=Mock()
        web._navigation=Mock();web._navigation.navigating=True
        web._navigation.pending=False;web._navigation.nav_owner=self.sid
        web._motor_armed=True;web._drive_owner=self.sid

    def tearDown(self):
        web._navigation=None;web._motor_armed=False;web._drive_owner=None
        self.context.pop()

    def test_idle_manual_zero_cannot_overwrite_navigation(self):
        self.client.emit('motor_control',dict(action='vel',linear=0,angular=0))
        web._send_drive.assert_not_called();web._navigation.cancel.assert_not_called()

    def test_idle_manual_zero_cannot_overwrite_automatic_dwell(self):
        web._navigation.navigating=False;web._navigation.autonomy.active=True
        self.client.emit('motor_control',dict(action='vel',linear=0,angular=0))
        web._send_drive.assert_not_called();web._navigation.cancel.assert_not_called()

    def test_nonzero_cancels_automatic_dwell(self):
        web._navigation.navigating=False;web._navigation.autonomy.active=True
        self.client.emit('motor_control',dict(action='vel',linear=.2,angular=0))
        web._navigation.cancel.assert_called_once()

    def test_zero_speed_takeover_does_not_cancel(self):
        # 2026-10-28: 页面按下摇杆的瞬间会发 takeover=1 + 0/0；这不能取消巡检
        # （真正接管必须带速度）。原断言与现行为不一致，按现行约定更新。
        self.client.emit('motor_control',dict(action='vel',linear=0,angular=0,takeover=True))
        web._navigation.cancel.assert_not_called()
        web._send_drive.assert_not_called()

    def test_nonzero_takes_over(self):
        self.client.emit('motor_control',dict(action='vel',linear=.2,angular=0))
        web._navigation.cancel.assert_called_once()
        web._send_drive.assert_called_once_with(.2,0.,True)

    def test_estop_cancels_and_disarms(self):
        self.client.emit('motor_control',dict(action='estop'))
        web._navigation.cancel.assert_called_once()
        self.assertFalse(web._motor_armed)
        web._send_drive.assert_called_once_with(0.,0.,False)

    def test_nonowner_cannot_take_over(self):
        web._drive_owner='another-client'
        self.client.emit('motor_control',dict(action='vel',linear=1,angular=1,takeover=True))
        web._navigation.cancel.assert_not_called();web._send_drive.assert_not_called()

    def test_mapping_blocked_during_localization(self):
        web._navigation.running.return_value=True
        web._mapping_request=Mock()
        self.client.emit('mapping_control',dict(action='start'))
        web._mapping_request.assert_not_called()


if __name__=='__main__':unittest.main()
