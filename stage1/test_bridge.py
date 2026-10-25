import ast
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as S
import unittest
from core import Feedback


tree=ast.parse((Path(__file__).parent/'ros_bridge.py').read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Bridge')
scope=dict(Node=object,math=math,json=json,time=time)
exec(compile(ast.Module(body=[cls],type_ignores=[]),'ros_bridge.py','exec'),scope)
Bridge=scope['Bridge']


class BridgeTests(unittest.TestCase):
    def make(self):
        b=Bridge.__new__(Bridge);b.lock=threading.RLock();b.enabled=True
        b.core=Feedback(1/3);b.sid='test';b.reason='enabled';b.motor=dict(driver='test',armed=True)
        b.outputs=[];b.events=[];b.sio=S(connected=True,emit=lambda *v:b.outputs.append(v))
        b.record=lambda *args,**kwargs:b.events.append((args,kwargs))
        b.start_wall=time.time()-.1;b.diff=True;b.ages={}
        b.command_publishers_ok=lambda:True
        return b

    def test_bad_twist_disarms_and_does_not_recover(self):
        for v,w in [(float('nan'),0),(.2,0)]:
            b=self.make()
            msg=S(linear=S(x=v,y=0,z=0),angular=S(x=0,y=0,z=w))
            b.on_command(msg)
            self.assertFalse(b.enabled)
            self.assertEqual(b.outputs[-1][1],dict(action='arm',armed=False))
            b.on_command(S(linear=S(x=.05,y=0,z=0),angular=S(x=0,y=0,z=0)))
            self.assertFalse(b.enabled)

    def test_estop_takeover_and_disconnect_latch_off(self):
        for state in [dict(armed=False,driver=None),dict(armed=True,driver='other')]:
            b=self.make();b.motor_state(state);self.assertFalse(b.enabled)
            b.motor_state(dict(armed=True,driver='test'));self.assertFalse(b.enabled)
        b=self.make();b.disconnected();self.assertFalse(b.enabled)

    def test_enable_rejects_stale_feedback_before_any_arm(self):
        b=self.make();b.enabled=False;b.motor=dict(armed=False);b.front=b.around=2
        response=S(success=None,message=None)
        b.enable(S(data=True),response)
        self.assertFalse(response.success);self.assertEqual(b.outputs,[])

    def test_good_command_and_unsafe_publisher_rejection(self):
        b=self.make();msg=S(linear=S(x=.05,y=0,z=0),angular=S(x=0,y=0,z=0))
        b.on_command(msg)
        self.assertEqual(b.command,(.05,0));self.assertTrue(b.enabled)
        b.diff=False;msg.angular.z=.1;b.on_command(msg)
        self.assertFalse(b.enabled)

    def test_writer_qos_including_real_web_node_name(self):
        b=self.make()
        writer=lambda name,life:S(node_name=name,qos_profile=S(lifespan=S(nanoseconds=life)))
        b.get_publishers_info_by_topic=lambda topic:[writer('slam_viz_node',0),writer('stage1_trial',200000000)]
        self.assertTrue(Bridge.command_publishers_ok(b))
        b.get_publishers_info_by_topic=lambda topic:[writer('stage1_trial',0)]
        self.assertFalse(Bridge.command_publishers_ok(b))
        b.get_publishers_info_by_topic=lambda topic:[writer('stage1_trial',200000000),writer('other',200000000)]
        self.assertFalse(Bridge.command_publishers_ok(b))

    def test_independent_gyro_angle_and_gap_stop(self):
        b=self.make();b.enabled=False;b.imu_up=None;b.gyro_yaw=0.;b.gyro_last=None
        b.gyro_rate=0.;b.gyro_origin=0.;b.stamp_fresh=lambda stamp:True
        def sample(t):
            sec=int(t);stamp=S(sec=sec,nanosec=int(round((t-sec)*1e9)))
            return S(header=S(stamp=stamp),linear_acceleration=S(x=0,y=0,z=9.8),angular_velocity=S(x=0,y=0,z=math.pi/2))
        for i in range(101):b.on_imu(sample(i*.01))
        self.assertAlmostEqual(math.degrees(b.gyro_yaw),90.,places=5)
        b.enabled=True;b.on_imu(sample(1.3))
        self.assertFalse(b.enabled);self.assertIn('gyro integration gap',b.reason)

    def test_front_clearance_removes_rear_axle_body_radius(self):
        b=self.make();b.mount=dict(x_m=.4,y_m=0.,yaw_rad=-math.pi)
        b.front_radius=math.hypot(.4,.1775);b.stamp_fresh=lambda stamp:True
        # An obstacle 40cm beyond the front edge must not become 80cm clear
        # merely because the origin moved to the rear axle.
        scan=S(header=S(frame_id='laser',stamp=None),range_min=.05,range_max=8.,
               angle_min=math.pi-.02,angle_increment=.001,ranges=[.4]*40)
        old=Bridge.on_scan.__globals__.get('base_point')
        def transform(x,y,m):
            c,s=math.cos(m['yaw_rad']),math.sin(m['yaw_rad'])
            return c*x-s*y+m['x_m'],s*x+c*y+m['y_m']
        Bridge.on_scan.__globals__['base_point']=transform
        try:b.on_scan(scan)
        finally:
            if old is None:Bridge.on_scan.__globals__.pop('base_point',None)
            else:Bridge.on_scan.__globals__['base_point']=old
        self.assertLess(b.front,.4)
        response=S(success=None,message=None);b.enabled=False;b.motor=dict(armed=False)
        b.sensor_error=lambda now:None
        b.enable(S(data=True),response)
        self.assertFalse(response.success);self.assertIn('clearance',response.message)
        self.assertEqual(b.outputs,[])


if __name__=='__main__':unittest.main()
