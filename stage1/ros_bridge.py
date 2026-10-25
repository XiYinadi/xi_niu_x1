#!/usr/bin/env python3
"""Supervised diagnostic ROS /cmd_vel -> existing web owner gate -> PWM.

Starts DISABLED. /stage1/enable SetBool is an explicit per-run lease. No GPIO,
encoder or camera access. Does not alter Nav2 calibration or normal drive mode.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import threading
import time
for p in ('/usr/local/lib/python3.10/dist-packages','/usr/lib/python3/dist-packages'):
    if p not in sys.path:sys.path.append(p)
import socketio
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'slam'))
from laser_mount import load_mount, base_point
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'navigation'))
from navigation_core import footprint_points
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool
from core import Feedback, relative


class Bridge(Node):
    def __init__(self,args):
        super().__init__('stage1_ros_bridge')
        p=yaml.safe_load(Path(args.motor_params).read_text())['motor_driver']['ros__parameters']
        self.diff=args.diff
        self.mount=load_mount()
        geometry=yaml.safe_load((Path(__file__).resolve().parent.parent/'navigation/navigation_calibration.yaml').read_text())
        # Clearance is measured from a circle enclosing the whole chassis,
        # not the shifted rear-axle origin. Never weaken it by moving base_link.
        self.front_radius=max(math.hypot(x,y) for x,y in footprint_points(geometry))
        if self.diff and (p.get('steer_mode')!='diff' or p.get('servo_hold_center') is not True):
            raise ValueError('Differential trial requires temporary diff + servo_hold_center parameters')
        if not self.diff and p.get('steer_mode')!='servo':raise ValueError('Expected normal servo mode')
        if not 0<float(p['max_duty'])<=.10:raise ValueError('Diagnostic PWM process must have max_duty <= .10')
        self.core=Feedback(p['max_angular_speed']*p['wheel_base']/(2*p['max_linear_speed']))
        self.lock=threading.RLock();self.enabled=False;self.sid=None
        self.motor={};self.pose=None;self.velocity=(0.,0.);self.ages={}
        self.front=self.around=0.;self.command=(0.,0.);self.last_command=0.
        self.imu_up=None;self.gyro_yaw=0.;self.gyro_origin=0.;self.gyro_last=None;self.gyro_rate=0.
        self.started=0.;self.start_wall=0.;self.origin=None;self.reason='disabled'
        self.stop_event=threading.Event();self.no_motion=None
        self.journal=open(args.journal,'a',buffering=1)
        self.drive_sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.drive_sock.setblocking(False)
        self.sio=socketio.Client(reconnection=False)
        self.sio.on('you',lambda data:setattr(self,'sid',data['sid']))
        self.sio.on('motor_state',self.motor_state)
        self.sio.on('disconnect',self.disconnected)
        self.sio.connect('http://127.0.0.1:5800?view=control',transports=['polling'],wait_timeout=6)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Twist,'/cmd_vel',self.on_command,qos)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Imu,'/imu/data_raw',self.on_imu,qos_profile_sensor_data)
        self.create_service(SetBool,'/stage1/enable',self.enable)
        self.status_pub=self.create_publisher(String,'/stage1/status',10)
        self.create_timer(.2,self.publish_status)
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()

    def record(self,event,**data):
        self.journal.write(json.dumps(dict(time=time.time(),event=event,**data),allow_nan=False)+'\n')

    def motor_state(self,data):
        with self.lock:
            self.motor=data
            if self.enabled and (not data.get('armed') or data.get('driver')!=self.sid):
                self.enabled=False;self.reason='web estop/disarm/ownership change';self.core.reset()
                self.record('halt',reason=self.reason)

    def disconnected(self):
        with self.lock:
            self.enabled=False;self.reason='web connection lost';self.core.reset()
            self.record('halt',reason=self.reason)

    def on_odom(self,msg):
        q=msg.pose.pose.orientation;p=msg.pose.pose.position
        pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        vel=(msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        if not all(math.isfinite(v) for v in (*pose,*vel)):return
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':return
        if not self.stamp_fresh(msg.header.stamp):return
        with self.lock:self.pose=pose;self.velocity=vel;self.ages['odom']=time.monotonic()

    def on_scan(self,msg):
        if msg.header.frame_id!='laser':return
        if not self.stamp_fresh(msg.header.stamp):return
        all_ranges=[];front=[]
        for i,r in enumerate(msg.ranges):
            if math.isfinite(r) and max(.05,msg.range_min)<r<msg.range_max:
                angle=msg.angle_min+i*msg.angle_increment
                x,y=base_point(r*math.cos(angle),r*math.sin(angle),self.mount)
                distance=math.hypot(x,y)
                all_ranges.append(distance)
                if abs(math.atan2(y,x))<.55:front.append(max(0.,distance-self.front_radius))
        with self.lock:
            self.front=min(front) if len(front)>=3 else 0.
            self.around=min(all_ranges) if len(all_ranges)>=20 else 0.
            self.ages['scan']=time.monotonic()

    def on_imu(self,msg):
        if self.stamp_fresh(msg.header.stamp):
            with self.lock:
                accel=msg.linear_acceleration
                if self.imu_up is None:
                    g=math.sqrt(accel.x**2+accel.y**2+accel.z**2)
                    if not 8<g<11.5:return
                    self.imu_up=[accel.x/g,accel.y/g,accel.z/g]
                omega=msg.angular_velocity
                rate=sum(x*y for x,y in zip((omega.x,omega.y,omega.z),self.imu_up))
                if not math.isfinite(rate):return
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                if self.gyro_last is not None:
                    previous,previous_rate=self.gyro_last;dt=stamp-previous
                    if 0<dt<=.15:self.gyro_yaw+=(rate+previous_rate)*.5*dt
                    elif self.enabled and self.diff:self.halt('gyro integration gap; angle not trusted')
                self.gyro_last=(stamp,rate);self.gyro_rate=rate
                self.ages['imu']=time.monotonic()

    def stamp_fresh(self,stamp):
        return -.05 <= time.time()-stamp.sec-stamp.nanosec*1e-9 < .4

    def command_publishers_ok(self):
        writers=[p for p in self.get_publishers_info_by_topic('/cmd_vel') if p.node_name!='slam_viz_node']
        return (len(writers)==1 and writers[0].node_name=='stage1_trial'
                and 0<writers[0].qos_profile.lifespan.nanoseconds<=200000000)

    def on_command(self,msg):
        with self.lock:
            if not self.enabled:return
            # This board's Humble callback API has no MessageInfo argument.
            # Require the controlled test writer's 200ms DDS lifespan instead
            # of silently accepting an unbounded, unstamped Twist publisher.
            v,w=msg.linear.x,msg.angular.z
            if any(abs(x)>1e-9 for x in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
                self.halt('unsupported Twist axes');return
            if not all(math.isfinite(x) for x in (v,w)) or not 0<=v<=.08 or abs(w)>.2 or (not self.diff and abs(w)>1e-6):
                self.halt('invalid command or mode');return
            self.command=(v,w);self.last_command=time.monotonic()

    def sensor_error(self,now):
        for key in ('odom','scan','imu'):
            if not 0<=now-self.ages.get(key,0)<.40:return key+' stale'
        return None

    def enable(self,req,res):
        if not req.data:
            with self.lock:self.halt('explicit disable')
            res.success=True;res.message=self.reason;return res
        with self.lock:
            reason=self.sensor_error(time.monotonic())
            if reason or self.enabled or self.motor.get('armed') or not self.sid:
                res.success=False;res.message=reason or 'already enabled or another driver owns motors';return res
            if self.front<.55 or (self.diff and self.around<.30):
                res.success=False;res.message='insufficient lidar clearance';return res
            if not self.command_publishers_ok():
                res.success=False;res.message='expected sole stage1_trial writer with <=200ms DDS lifespan';return res
            odom=self.get_publishers_info_by_topic('/odom')
            if len(odom)!=1 or odom[0].node_name!='laser_imu_odom':
                res.success=False;res.message='expected sole laser_imu_odom publisher';return res
            self.core.reset();self.command=(0.,0.);self.origin=self.pose;self.no_motion=None
            self.gyro_origin=self.gyro_yaw
        try:
            self.sio.call('motor_control',dict(action='arm',armed=True,claim_if_disarmed=True),timeout=2)
        except Exception as e:
            with self.lock:self.halt('ownership handshake failed')
            res.success=False;res.message=str(e);return res
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            with self.lock:
                if self.motor.get('armed') and self.motor.get('driver')==self.sid:break
            time.sleep(.01)
        with self.lock:
            if not self.motor.get('armed') or self.motor.get('driver')!=self.sid:
                res.success=False;res.message='web ownership not granted';return res
            self.enabled=True;self.reason='enabled';self.started=time.monotonic();self.start_wall=time.time()
            self.last_command=self.started
            self.record('enable',pose=self.pose,diff=self.diff,mount=self.mount)
            res.success=True;res.message='enabled; fresh commands required within 250ms';return res

    def halt(self,reason):
        self.enabled=False;self.reason=reason;self.core.reset();self.command=(0.,0.)
        if self.sio.connected and self.motor.get('driver')==self.sid:
            self.sio.emit('motor_control',dict(action='arm',armed=False))
        self.record('halt',reason=reason)

    def loop(self):
        previous=time.monotonic();sample=0.
        while not self.stop_event.wait(.02):
            try:
                with self.lock:
                    now=time.monotonic();dt=now-previous;previous=now
                    if not self.enabled:continue
                    reason=self.sensor_error(now)
                    if now-self.last_command>.25:reason='cmd_vel timeout'
                    if now-self.started>40:reason='40 second lease expired'
                    if dt>.15:reason='bridge scheduler stalled'
                    delta=relative(self.origin,self.pose)
                    gyro_deg=math.degrees(self.gyro_yaw-self.gyro_origin)
                    if delta['displacement_m']>1.15 or abs(gyro_deg if self.diff else delta['yaw_deg'])>105:reason='travel envelope exceeded'
                    v,w=self.command
                    if v>0 and self.front<.50:reason='front obstacle'
                    if w!=0 and self.around<.28:reason='turn clearance lost'
                    if abs(w)<1e-6 and v>0 and (abs(delta['lateral_m'])>.12 or abs(delta['yaw_deg'])>12):reason='straight-line drift exceeded'
                    measured_w=self.gyro_rate if self.diff else self.velocity[1]
                    if (not self.diff and self.velocity[0]<-.03) or self.velocity[0]>.20 or abs(measured_w)>.6:reason='unexpected measured speed'
                    if w*measured_w<-.003:reason='rotation sign disagrees with command'
                    if reason:self.halt(reason);continue
                    l,a=self.core.step(v,w,self.velocity[0],measured_w,dt,self.diff)
                    stalled=(v>0 and l>.05 and abs(self.velocity[0])<.003) or (w!=0 and abs(a*self.core.k)>.04 and abs(measured_w)<.005)
                    self.no_motion=(self.no_motion or now) if stalled else None
                    if self.no_motion and now-self.no_motion>1.5:self.halt('no motion detected; do not increase torque');continue
                    self.drive_sock.sendto(json.dumps(dict(sid=self.sid,l=l,a=a,sent_monotonic=time.monotonic())).encode(),('127.0.0.1',5818))
                    if now-sample>.1:
                        self.record('sample',pose=self.pose,velocity=self.velocity,command=self.command,output=[l,a],relative=delta,gyro_yaw_deg=gyro_deg,gyro_rate=measured_w)
                        sample=now
            except Exception as e:
                with self.lock:self.halt('bridge error: '+str(e))

    def publish_status(self):
        policy=self.command_publishers_ok()
        with self.lock:
            if self.enabled and not policy:self.halt('unbounded or competing cmd_vel publisher')
            msg=String();msg.data=json.dumps(dict(enabled=self.enabled,reason=self.reason,pose=self.pose,velocity=self.velocity,mount=self.mount,
                gyro_relative_yaw_deg=math.degrees(self.gyro_yaw-self.gyro_origin),gyro_rate=self.gyro_rate,
                front_m=self.front,clearance_m=self.around,ages={k:time.monotonic()-v for k,v in self.ages.items()},
                relative=relative(self.origin,self.pose) if self.origin and self.pose else None))
        self.status_pub.publish(msg)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--motor-params',required=True);p.add_argument('--diff',action='store_true');p.add_argument('--journal',required=True)
    args=p.parse_args();rclpy.init();node=Bridge(args)
    def stop(*unused):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        with node.lock:node.halt('bridge exit')
        node.stop_event.set();node.thread.join(timeout=2);time.sleep(.1)
        if node.sio.connected:node.sio.disconnect()
        node.drive_sock.close()
        node.journal.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
