#!/usr/bin/env python3
"""Explicit supervised ROS motion trial. Ground truth must be entered separately."""
import argparse
import json
import math
from pathlib import Path
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool
from core import relative, wrap


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind',choices=['straight_probe','spin_probe','one_meter','ninety_deg','watchdog','zero_watchdog'])
    parser.add_argument('--output',required=True)
    parser.add_argument('--record-sensors',action='store_true',help='Keep raw scan/IMU/odom in RAM; write only after disarming')
    args=parser.parse_args()
    rclpy.init();node=Node('stage1_trial')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,lifespan=Duration(seconds=.2))
    pub=node.create_publisher(Twist,'/cmd_vel',qos)
    client=node.create_client(SetBool,'/stage1/enable')
    state={'pose':None,'received':0.,'status':None,'stamp':0.}
    samples=[];origin=None;end=None;reason='not started';armed=False;origin_stamp=None
    recording=dict(scans=[],imu=[],odom=[])
    def odom(msg):
        q=msg.pose.pose.orientation;p=msg.pose.pose.position
        pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not all(math.isfinite(x) for x in pose) or not -.05<=time.time()-stamp<.4:return
        state.update(pose=pose,received=time.monotonic(),stamp=stamp)
        if args.record_sensors:
            recording['odom'].append(dict(stamp=stamp,pose=pose,velocity=[msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z]))
    def status(msg):state['status']=json.loads(msg.data)
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    node.create_subscription(String,'/stage1/status',status,10)
    def scan(msg):
        recording['scans'].append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            frame=msg.header.frame_id,angle_min=msg.angle_min,angle_increment=msg.angle_increment,
            range_min=msg.range_min,range_max=msg.range_max,scan_time=msg.scan_time,time_increment=msg.time_increment,
            ranges=[float(v) if math.isfinite(v) else None for v in msg.ranges]))
    def imu(msg):
        a,w=msg.linear_acceleration,msg.angular_velocity
        values=[a.x,a.y,a.z,w.x,w.y,w.z]
        if all(math.isfinite(v) for v in values):
            recording['imu'].append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,values=values))
    if args.record_sensors:
        node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
        node.create_subscription(Imu,'/imu/data_raw',imu,qos_profile_sensor_data)
    def enable(on):
        req=SetBool.Request();req.data=on
        f=client.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=3)
        if not f.done() or f.result() is None:raise RuntimeError('enable service timeout')
        if not f.result().success:raise RuntimeError(f.result().message)
    def command(v=0.,w=0.):
        msg=Twist();msg.linear.x=v;msg.angular.z=w;pub.publish(msg)
    def interrupted(*unused):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    try:
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('stage1 bridge absent')
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and (state['pose'] is None or state['status'] is None):rclpy.spin_once(node,timeout_sec=.05)
        if state['pose'] is None or time.monotonic()-state['received']>.3:raise RuntimeError('no fresh odometry')
        if args.record_sensors:
            deadline=time.monotonic()+2.
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        origin=state['pose'];origin_stamp=state['stamp'];enable(True);armed=True
        start=time.monotonic();last_output=0.;last_print=0.;first=None;command_sent=False
        limit=35 if args.kind=='one_meter' else 24 if args.kind=='ninety_deg' else 2.0 if args.kind=='spin_probe' else 2.5
        print('START',args.kind,'origin',origin,'hard timeout',limit,flush=True)
        while True:
            rclpy.spin_once(node,timeout_sec=.005);now=time.monotonic();elapsed=now-start
            if time.monotonic()-state['received']>.4:reason='odometry lost';break
            st=state['status']
            if command_sent and st and not st['enabled'] and elapsed>.4:reason='bridge halted: '+st['reason'];break
            delta=relative(origin,state['pose']);distance=delta['forward_m']
            gyro_yaw=(st or {}).get('gyro_relative_yaw_deg')
            if args.kind in ('spin_probe','ninety_deg') and gyro_yaw is None:reason='independent gyro angle unavailable';break
            yaw=math.radians(gyro_yaw if args.kind in ('spin_probe','ninety_deg') else delta['yaw_deg'])
            if elapsed>=limit:reason='time bound reached';break
            if delta['displacement_m']>.16 and args.kind in ('straight_probe','watchdog'):reason='probe travel bound reached';break
            if abs(math.degrees(yaw))>12 and args.kind=='spin_probe':reason='probe angle bound reached';break
            if args.kind=='one_meter' and distance>=.985:reason='odometry target reached';break
            if args.kind=='ninety_deg' and yaw>=math.radians(88.5):reason='odometry target reached';break
            if args.kind in ('watchdog','zero_watchdog') and elapsed>.8:
                # Intentionally stop publishing; bridge must disarm in 250ms.
                pass
            elif now-last_output>=.02:
                v=.05 if args.kind in ('one_meter','straight_probe','watchdog') else 0.
                w=.15 if args.kind in ('spin_probe','ninety_deg') else 0.
                if args.kind=='one_meter':v=max(.02,min(.05,(1-distance)*.4))
                if args.kind=='ninety_deg':w=max(.06,min(.15,(math.pi/2-yaw)*.7))
                command(v,w);command_sent=True;last_output=now
                if first is None:first=time.time()
            if now-last_print>.2:
                sample=dict(t=elapsed,pose=state['pose'],relative=delta,status=st)
                samples.append(sample);last_print=now
                print(json.dumps(dict(t=round(elapsed,2),**delta)),flush=True)
        end=state['pose']
    except KeyboardInterrupt:reason='operator interrupted'
    except Exception as e:reason='error: '+str(e)
    finally:
        # Explicit zeros, explicit disable, then collect settled odometry.
        try:
            for _ in range(6):command();rclpy.spin_once(node,timeout_sec=.02)
            if armed:enable(False)
            deadline=time.monotonic()+1.0
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
            end=state['pose']
        except Exception as e:reason+='; cleanup: '+str(e)
        result=dict(kind=args.kind,finished_at=time.time(),reason=reason,origin=origin,end=end,
                    origin_stamp=origin_stamp,end_stamp=state['stamp'],
                    gyro_yaw_deg=(state['status'] or {}).get('gyro_relative_yaw_deg'),
                    odometry=relative(origin,end) if origin and end else None,
                    mount=(state['status'] or {}).get('mount'),sensor_recording=recording if args.record_sensors else None,
                    ground_truth=None,accepted=False,samples=samples)
        Path(args.output).write_text(json.dumps(result,indent=2,allow_nan=False))
        print('RESULT',json.dumps({k:v for k,v in result.items() if k not in ('samples','sensor_recording')}),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(0 if reason in ('odometry target reached','probe travel bound reached','probe angle bound reached','time bound reached') or (args.kind in ('watchdog','zero_watchdog') and 'cmd_vel timeout' in reason) else 1)


if __name__=='__main__':main()
