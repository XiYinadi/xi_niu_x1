"""Web navigation integration, gated actuator output, and owned Nav2 lifecycle.

Nav2 publishes only /navigation/cmd_vel_smoothed, never the manual /cmd_vel.
No navigation packet can arm a disarmed chassis. Output requires measured
calibration or the explicitly selected bounded supervised trial profile.
"""
import math
import json
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
import yaml
from map_store import MapStore, yaw
from navigation_core import fresh, twist_to_joystick, validate_calibration
from navigation_drive import is_trial, validate_navigation, TrialDrive, scan_clearance
from patrol_store import PatrolStore, validate_task
from patrol_preview import PatrolPreview
from patrol_runner import PatrolRunner
from localization_settle import StaticPoseSettle


class NavigationRuntime:
    def __init__(self, node, socketio, send_drive, armed, owner, disarm):
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose, ComputePathToPose
        from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
        from tf2_ros import Buffer, TransformListener
        from lifecycle_msgs.srv import GetState
        self.node, self.socketio = node, socketio
        self.send_drive, self.armed, self.owner, self.disarm = send_drive, armed, owner, disarm
        self.here = Path(__file__).resolve().parent
        self.root = self.here.parent
        self.session_file = self.root/'data'/'navigation_session.json'
        self.session_file.parent.mkdir(parents=True, exist_ok=True)
        self.cleanup_orphan()
        self.store = MapStore(self.root/'data'/'maps')
        self.patrols = PatrolStore(self.root/'data'/'patrols', self.store)
        self.route_preview = PatrolPreview()
        self.config = yaml.safe_load((self.here/'navigation_calibration.yaml').read_text())
        self.trial_drive = TrialDrive(self.config) if is_trial(self.config) else None
        self.trial_mount = json.loads((self.root/'slam/laser_mounting.json').read_text()) if self.trial_drive else None
        self.trial_clearance = None
        self.trial_scan_at = None
        self.trial_origin = None
        self.trial_started = self.last_control_t = None
        self.trial_no_motion = None
        self.preview_cache = None
        self.lock = threading.RLock()
        self.patrol_runner = PatrolRunner(self)
        self._regions_cache = None
        self.mode = 'localization'
        self.autonomy = None
        self.busy = False
        self.process = None
        self.active_map = None
        self.status = 'off'
        self.note = '请先保存地图，再加载定位'
        self.goal = None
        self.goal_handle = None
        self.pending = False
        self.epoch = 0
        self.navigating = False
        self.nav_owner = None
        self.last_command = None
        self.last_client = None
        self.last_pose = None
        self.last_amcl = None
        self.pose_cov = None
        from std_srvs.srv import Empty
        self._Empty = Empty
        self.nomotion = node.create_client(Empty, '/request_nomotion_update')
        self.nomotion_next = 0.0
        self._last_tick_t = None
        self._stale_from = {}
        self._command_grace_until = 0.0
        self._command_stale_from = None
        self._clearance_unknown_from = None
        self.pose_settle = StaticPoseSettle()
        self.pose_ok = False
        self.command = (0.0, 0.0)
        self.path = []
        self.lifecycle = {}
        self.lifecycle_pending = {}
        self.lifecycle_clients = {name: node.create_client(GetState, '/'+name+'/get_state')
                                  for name in ('amcl', 'map_server', 'planner_server', 'bt_navigator', 'velocity_smoother', 'controller_server')}
        self.nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.cmd_sub = node.create_subscription(Twist, '/navigation/cmd_vel_smoothed', self.on_command, 1)
        self.pose_sub = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        if self.trial_drive:
            from sensor_msgs.msg import LaserScan
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            latest_scan=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
            self.trial_scan_sub=node.create_subscription(LaserScan,'/scan',self.on_trial_scan,latest_scan)
        self.timer = node.create_timer(0.02, self.tick)
        self.ui_timer = node.create_timer(0.5, self.emit)
        self.lifecycle_timer = node.create_timer(1.0, self.check_lifecycle)

    def cleanup_orphan(self):
        """Only stop a previously recorded launch group, verifying PID reuse."""
        try:
            record = json.loads(self.session_file.read_text())
            pid = int(record['pid'])
            proc = Path('/proc')/str(pid)
            token = proc.joinpath('stat').read_text().split(') ', 1)[1].split()[19]
            command = proc.joinpath('cmdline').read_bytes().split(b'\0')
            expected = str(self.here/'navigation_launch.py').encode()
            if token == record['start'] and expected in command and os.getpgid(pid) == pid:
                os.killpg(pid, signal.SIGINT)
                for _ in range(100):
                    if not proc.exists(): break
                    time.sleep(.1)
                if proc.exists():
                    os.killpg(pid, signal.SIGTERM)
            self.session_file.unlink(missing_ok=True)
        except FileNotFoundError:
            self.session_file.unlink(missing_ok=True)

    def check_lifecycle(self):
        from lifecycle_msgs.srv import GetState
        with self.lock:
            if not self.running():
                return
            now = time.monotonic()
            for name, client in self.lifecycle_clients.items():
                pending = self.lifecycle_pending.get(name)
                if pending is not None:
                    future, sent_at = pending
                    if now-sent_at < 3.0:
                        continue
                    # A server can disappear without completing its old request.
                    # Drop the old request before retrying; stale callbacks must
                    # never overwrite the result from a newer launch/request.
                    self.lifecycle_pending.pop(name)
                    self.lifecycle[name] = 0
                    client.remove_pending_request(future)
                    future.cancel()
                if not client.service_is_ready():
                    self.lifecycle[name] = 0
                    continue
                try:
                    future = client.call_async(GetState.Request())
                except Exception:
                    self.lifecycle[name] = 0
                    continue
                self.lifecycle_pending[name] = (future, now)
                def done(future, key=name):
                    with self.lock:
                        pending = self.lifecycle_pending.get(key)
                        if pending is None or pending[0] is not future:
                            return
                        self.lifecycle_pending.pop(key)
                        try:
                            self.lifecycle[key] = future.result().current_state.id
                        except Exception:
                            self.lifecycle[key] = 0
                future.add_done_callback(done)

    def reset_lifecycle(self):
        with self.lock:
            pending = list(self.lifecycle_pending.items())
            self.lifecycle_pending.clear()
            self.lifecycle.clear()
            for name, (future, _) in pending:
                self.lifecycle_clients[name].remove_pending_request(future)
                future.cancel()

    def running(self):
        return self.process is not None and self.process.poll() is None

    def calibration_error(self):
        try:
            validate_calibration(self.config)
            return None
        except ValueError as e:
            return str(e)

    def drive_error(self):
        try:
            validate_navigation(self.config)
            return None
        except ValueError as error:return str(error)

    def on_trial_scan(self, msg):
        age=(self.node.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9
        with self.lock:
            self.trial_clearance=scan_clearance(msg,self.trial_mount,self.config) if -.05<=age<.4 else None
            self.trial_scan_at=time.monotonic()-max(0.,age) if -.05<=age<.4 else None

    def payload(self):
        with self.lock:
            now = time.monotonic()
            ready = self.localized(now)
            return dict(status=self.status, message=self.note, busy=self.busy,
                running=self.running(), maps=self.store.list(), selected_map=self.active_map,
                localized=ready,
                # 2026-10-25: 静态地图是 latched 只发一次，不能拿 map_age 当就绪判据。
                map_loaded=bool(self.active_map) and self.lifecycle.get('map_server') == 3,
                regions=self.map_regions(),
                localization=dict(pose_ok=bool(self.pose_ok),
                    amcl_age=(now-self.last_amcl) if self.last_amcl else None,
                    cov=self.pose_cov), calibration_error=self.calibration_error(),
                drive_error=self.drive_error(),drive_mode='supervised_trial' if is_trial(self.config) else 'calibrated',
                supervised_limits=(dict(profile=self.config.get('supervised_profile','short'),
                    goal_m=self.config['trial_max_goal_m'],path_m=self.config['trial_max_path_m'],
                    duration_s=self.config['trial_max_duration_s'],speed_mps=self.config['max_speed_mps'],
                    pwm_cap=self.config['trial_pwm_cap']) if is_trial(self.config) else None),
                preview_valid=bool(getattr(self,'preview_cache',None)),
                trial_clearance_m=getattr(self,'trial_clearance',None),
                goal_distance_m=math.hypot(self.goal['x']-self.last_pose['x'],self.goal['y']-self.last_pose['y']) if self.goal and self.last_pose else None,
                initial_pose_ready=all(self.lifecycle.get(k) == 3 for k in ('map_server','amcl')),
                navigating=self.navigating, goal=self.goal, path=self.path,
                mode=getattr(self,'mode','localization'),
                autonomy=(self.autonomy.snapshot() if self.autonomy and
                          (self.autonomy.active or self.mode == 'mapping')
                          else self.patrol_runner.snapshot()),
                pose=self.last_pose if ready else None,
                patrols=self.patrols.list(), patrol_preview=self.route_preview.snapshot(),
                planner_ready=self.planner.server_is_ready() and self.lifecycle.get('planner_server') == 3,
                navigator_ready=self.nav.server_is_ready() and all(self.lifecycle.get(k) == 3 for k in ('bt_navigator','controller_server','velocity_smoother')))

    def emit(self):
        self.socketio.emit('navigation_state', self.payload())

    def transform_snapshot(self, state):
        from rclpy.time import Time
        def transform(frame, points, stamp=None):
            query = Time.from_msg(stamp) if stamp is not None else Time()
            tf = self.tf.lookup_transform('map', frame, query).transform
            angle = yaw(tf.rotation)
            c, sn = math.cos(angle), math.sin(angle)
            out = []
            for x, y in zip(points[::2], points[1::2]):
                out.extend((c*x-sn*y+tf.translation.x, sn*x+c*y+tf.translation.y))
            return out, angle
        # Ordinary mapping also needs map->odom correction, not only Nav2 sessions.
        if 'pose' in state:
            try:
                p = state['pose']
                xy, angle = transform('odom', [p['x'], p['y']])
                state['pose'] = dict(x=xy[0], y=xy[1], yaw=math.atan2(
                    math.sin(p['yaw']+angle), math.cos(p['yaw']+angle)))
            except Exception:
                state.pop('pose', None)
        if self.running():
            if self.last_pose is not None:
                state['pose'] = self.last_pose
            else:
                state.pop('pose', None)
            state.pop('trajectory', None)
        elif 'trajectory' in state:
            # Reproject all retained odometry history after loop-closure corrections.
            # One update/second bounds bandwidth; do not mix incremental old transforms.
            now = time.monotonic()
            if now-getattr(self, '_trajectory_emit_at', 0) < 1 and not state['trajectory'].get('reset'):
                state.pop('trajectory', None)
            else:
                try:
                    history = self.node.full_trajectory(map_frame=False)
                    points, _ = transform('odom', history['points'])
                    state['trajectory'] = dict(history, points=points, reset=True)
                    self._trajectory_emit_at = now
                except Exception:
                    state.pop('trajectory', None)
        if 'scan' in state:
            scan = state['scan']
            stamp = scan.pop('stamp', None)
            frame = scan.pop('frame', 'laser')
            try:
                if stamp is None:
                    raise ValueError('Scan timestamp unavailable')
                points, _ = transform(frame, scan['points'], stamp)
                state['scan'] = dict(scan, points=points)
            except Exception:
                # Retry the current scan on the next snapshot, never use a newer pose.
                state.pop('scan', None)
        return state

    def gyro_z(self):
        """GY-521 的 z 轴角速度（rad/s）；超过 0.3 秒没有新样本返回 None。"""
        with self.node._lock:
            gyro = getattr(self.node, '_imu_gyro', None)
            stamp = getattr(self.node, '_imu_gyro_t', 0.0)
        if not gyro or time.monotonic()-stamp > .3:
            return None
        return gyro[2]

    def localized(self, now):
        # AMCL need not republish covariance while stationary; TF must stay fresh.
        if getattr(self,'mode','localization') == 'mapping':
            with self.node._lock:
                return self.running() and self.last_pose is not None and fresh(now,self.node._last_map_t,10)
        return (self.running() and self.pose_ok and self.last_pose is not None
                and (not getattr(self, 'pose_settle', None) or self.pose_settle.ready)
                and self.lifecycle.get('amcl') == 3 and self.lifecycle.get('map_server') == 3)

    def on_pose(self, msg):
        with self.lock:
            self.last_amcl = time.monotonic()
            cov = msg.pose.covariance
            self.pose_cov = (float(cov[0]), float(cov[7]), float(cov[35]))
            self.pose_ok = all(math.isfinite(cov[i]) and 0 <= cov[i] < limit
                               for i, limit in ((0, 0.25), (7, 0.25), (35, 0.25)))

            settle = getattr(self, 'pose_settle', None)
            if settle and settle.needs_update:
                p = msg.pose.pose
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
                settle.observe(p.position.x, p.position.y, yaw(p.orientation), stamp)
                if settle.ready:
                    self.note = '静止定位已稳定，可选择目标预览'
                else:
                    self.note = settle.message()

    def on_command(self, msg):
        with self.lock:
            if self.navigating:
                self.command = (msg.linear.x, msg.angular.z)
                self.last_command = time.monotonic()

    def cancel(self, reason='导航已取消', disarm=False, keep_route=False, keep_mission=False):
        with self.lock:
            # 2026-10-25: 手动接管/急停/取消导航时，巡检执行器一起停。
            if not keep_mission and getattr(self, 'patrol_runner', None):
                self.patrol_runner.cancel(reason)
            had_motion = self.navigating
            self.epoch += 1  # invalidate in-flight goal acceptance/results
            self.navigating = False
            self.pending = False
            self.last_command = None
            self.command = (0.0, 0.0)
            self.preview_cache=None
            if getattr(self,'trial_drive',None):self.trial_drive.reset()
            self.trial_origin=None;self.trial_started=None;self.last_control_t=None;self.trial_no_motion=None
            self.nav_owner=None
            # Hardware stop must not depend on a Nav2 cancellation RPC.
            if had_motion:
                self.send_drive(0.0, 0.0, False if disarm else self.armed())
            if disarm and had_motion:
                self.disarm()
            self.status = 'localizing' if self.running() else 'off'
            self.note = reason
            handle=self.goal_handle
            self.goal_handle=None
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    logging.getLogger(__name__).exception('Nav2 cancellation failed after zero output; task remains stopped')
            if not keep_mission and getattr(self,'autonomy',None):
                self.autonomy.cancelled(reason)
            if not keep_route and hasattr(self, 'route_preview'):
                self.route_preview.cancel(reason)

    def _clearance_blocked(self, now):
        c = self.trial_clearance
        if fresh(now, self.trial_scan_at, .6) and c is not None:
            self._clearance_unknown_from = None
            if c < self.config['trial_clearance_m']:
                return '近处有障碍：前方净空 %.3f m（阈值 %.2f m）' % (c, self.config['trial_clearance_m'])
            return None
        # 净空「未知」（扫描过期/有效点不足）给 0.5s 宽限；持续未知才停。
        first = self._clearance_unknown_from or now
        self._clearance_unknown_from = first
        if (now - first) > 0.5:
            age = None if self.trial_scan_at is None else (now - self.trial_scan_at)
            return '净空未知：scan_age=%s clearance=%s' % (
                'n/a' if age is None else '%.2fs' % age, c)
        return None

    def _command_lost(self, now):
        # 2026-10-24: 目标下发后 controller 要初始化 costmap/TF，首条 cmd_vel
        # 可能晚于 0.25s；给 3s 启动宽限，之后才按 0.25s + 0.5s 去抖判死。
        t = self.last_command
        if t is None:
            return False
        if fresh(now, t, self.config['command_timeout_s']):
            self._command_stale_from = None
            return False
        if now < self._command_grace_until:
            self._command_stale_from = None
            return False
        first = self._command_stale_from or now
        self._command_stale_from = first
        return (now - first) > 0.5

    def _sensor_lost(self, key, now, sample_t, timeout, grace=0.5):
        # 2026-10-24: 0.4s 超时本身保留，但要求**持续**超时才停。load 高时
        # 单次调度/回调抖动会造成一拍假性超时（IMU、雷达各误停过一次）。
        if fresh(now, sample_t, timeout):
            self._stale_from.pop(key, None)
            return False
        first = self._stale_from.setdefault(key, now)
        return (now - first) > grace

    def tick(self):
        from rclpy.time import Time
        with self.lock:
            now = time.monotonic()
            prev_tick = getattr(self, '_last_tick_t', None)
            self._last_tick_t = now
            # 试航模式把传感器超时钉死在 0.4s，但 load 9 时本进程自身一次调度
            # 停顿就会让 now-_last_imu_t 假性超标。停顿超过 0.25s 时改用**停顿前**
            # 的时钟比较：真断线（停顿期间没有新数据）下一拍仍会被抓住。
            stalled = prev_tick is not None and (now - prev_tick) > 0.25
            sensor_now = prev_tick if stalled else now
            # 2026-10-24: AMCL 配了 update_min_d/a，静止时不处理扫描，协方差
            # 不会收敛；而门禁只认 pose_ok(协方差<0.25)。这里在「未定位」期间
            # 每 0.5s 要一次 nomotion update，让它用当前扫描重新加权粒子。
            if (self.running() and not self.navigating and not self.armed()
                    and (not self.pose_ok or (getattr(self, 'pose_settle', None) and self.pose_settle.needs_update))
                    and now >= self.nomotion_next):
                try:
                    if self.nomotion.service_is_ready():
                        self.nomotion.call_async(self._Empty.Request())
                        self.nomotion_next = now + 0.5
                except Exception:
                    pass
            if self.running():
                try:
                    tf = self.tf.lookup_transform('map', 'base_link', Time())
                    tf_age = (self.node.get_clock().now().nanoseconds -
                              (tf.header.stamp.sec*10**9+tf.header.stamp.nanosec))/1e9
                    if not -1 < tf_age < 1.5:
                        raise ValueError('map→base_link TF 已过期')
                    t, q = tf.transform.translation, tf.transform.rotation
                    self.last_pose = dict(x=t.x, y=t.y, yaw=yaw(q))
                except Exception:
                    self.last_pose = None
            elif self.process is not None:
                self.cancel('导航进程已退出，请查看运行日志', disarm=True)
                self.process = None
                self.status = 'error'
            # 2026-10-26: 定位跳变守卫。车最快 0.25 m/s，位姿若以 >1.0 m/s
            # 的速度变化，只可能是 AMCL 重定位或激光里程计重置；此时继续按
            # 这个位姿跑就会开到别处（实测跳了 1.4 米）。立刻停车并要求重设位置。
            if self.last_pose is not None:
                reference = getattr(self, '_pose_check', None)
                # 2026-10-28: 参考样本至少保留 0.25 秒再更新。原来每拍都覆盖参考、
                # 判据又要求与参考间隔 >=0.25s —— 正常 20Hz 循环下永远不成立，
                # 实测 1.4 米的跳变完全不会触发。
                if reference is None or now - reference[0] >= .25:
                    _patrol = getattr(self, 'patrol_runner', None)
                    if reference is not None and (self.navigating or (_patrol is not None and _patrol.active)):
                        span = now - reference[0]
                        shift = math.hypot(self.last_pose['x']-reference[1], self.last_pose['y']-reference[2])
                        if span >= .25 and shift >= .20 and shift/span > 1.2:
                            self.cancel('定位跳变 %.2f 米，导航停止；请重新设置当前位置' % shift, disarm=True)
                            return
                    self._pose_check = (now, self.last_pose['x'], self.last_pose['y'])
            else:
                self._pose_check = None
            _patrol_runner = getattr(self, 'patrol_runner', None)
            if _patrol_runner is not None:
                _patrol_runner.tick()
            if getattr(self,'mode','localization')=='mapping' and self.status=='mapping' and self.localized(now) and all(self.lifecycle.get(k)==3 for k in ('planner_server','controller_server','bt_navigator','velocity_smoother')):
                self.note='建图与避障服务已就绪；可预览探索目标，完成标定后开始自动建图'
            if not self.navigating:
                return
            reason = None
            with self.node._lock:
                scan_t = self.node._last_scan_t or None
                odom_t = getattr(self.node, '_last_odom_t', None)
                imu_t = getattr(self.node, '_last_imu_t', None)
            if not self.armed() or self.owner() != self.nav_owner:
                reason = '驾驶使能或驾驶权已改变，导航停止'
            elif not fresh(now, self.last_client, 0.6):
                reason = '操作者链路超时，导航停止'
            elif not self.localized(now) or self.last_pose is None:
                reason = '定位或坐标变换失效，导航停止'
            elif self._sensor_lost('scan', sensor_now, scan_t, self.config['scan_timeout_s']):
                reason = '雷达数据过期，导航停止'
            elif self._sensor_lost('odom', sensor_now, odom_t, self.config.get('odom_timeout_s', 1.0)):
                reason = '里程计数据过期，导航停止'
            elif self._sensor_lost('imu', sensor_now, imu_t, self.config.get('imu_timeout_s', 0.75)):
                reason = 'GY-521 数据过期，导航停止'
            elif is_trial(self.config) and self.trial_started is not None and now-self.trial_started>self.config['trial_max_duration_s']:
                reason = '本次试航时间已到，车辆停止'
            elif self._command_lost(now):
                reason = '导航速度指令超时，导航停止'
            elif self.last_command is None:
                # Goal planning can take seconds, but no old command may be replayed.
                self.send_drive(0.0, 0.0, self.armed())
                if is_trial(self.config):self.last_control_t=now
                return
            # 2026-10-28: 巡检只豁免它自己负责的那部分，**不能整段清空 reason**。
            #  - 心跳：巡检 _guard 自己管（检测/出报告阶段跳过）；
            #  - 指令超时：只有 Nav2 在开车（navigating）时才有意义，停稳/直行段
            #    由巡检每拍自己发指令、last_command 不更新，拿它判死会误停；
            #  - 传感器 / 定位 / 使能：一律保留（真正的安全网）。
            patrol = getattr(self, 'patrol_runner', None)
            if patrol is not None and patrol.active:
                status = getattr(patrol, 'status', '')
                if reason == '操作者链路超时，导航停止':
                    reason = None
                elif reason == '导航速度指令超时，导航停止' and status != 'navigating':
                    reason = None
            if reason:
                self.cancel(reason, disarm=True)
                return
            try:
                # 2026-10-26: 巡检在 dwell/detecting/reporting 时车本来就该停着，
                # 不做试航换算，直接发 0 —— 免得 VLM 推理的长停顿触发净空/节拍误判。
                _patrol = getattr(self, 'patrol_runner', None)
                if (_patrol is not None and _patrol.active
                        and getattr(_patrol, 'status', '') != 'navigating'):
                    self.send_drive(0.0, 0.0, self.armed())
                    return
                if is_trial(self.config):
                    with self.node._lock:odom=dict(self.node._odometry or {})
                    age=time.time()-odom['stamp'] if odom else None
                    if age is None or not -.05<age<.6:
                        detail='缺失' if age is None else f'{age:.3f}秒，接收间隔{now-odom_t:.3f}秒'
                        raise ValueError('速度反馈过期，试航停止（'+detail+'）')
                    if self.trial_origin is None or odom['generation']!=self.trial_origin['generation']:raise ValueError('里程计参考改变，试航停止')
                    if now-self.trial_started>self.config['trial_max_duration_s']:raise ValueError('本次试航时间已到，车辆停止')
                    if math.hypot(odom['x']-self.trial_origin['x'],odom['y']-self.trial_origin['y'])>self.config['trial_max_travel_m']:raise ValueError('本次试航距离已到，车辆停止')
                    _blocked=self._clearance_blocked(now)
                    if _blocked:raise ValueError(_blocked+'，试航停止')
                    dt=now-self.last_control_t;self.last_control_t=now
                    l,a,rot=self.trial_drive.step(*self.command,odom['vx'],dt)
                    stalled=self.command[0]>0 and l>.06 and abs(odom['vx'])<.003
                    self.trial_no_motion=(self.trial_no_motion or now) if stalled else None
                    if self.trial_no_motion and now-self.trial_no_motion>1.5:raise ValueError('车辆没有正常移动，试航停止')
                else:
                    l, a = twist_to_joystick(*self.command, self.config); rot = 0.0
                # 只有原地旋转才带第 4 个参数，普通前进保持三参数调用（兼容既有调用方/测试）
                if rot:
                    self.send_drive(l, a, self.armed(), rot)
                else:
                    self.send_drive(l, a, self.armed())
            except ValueError as e:
                self.cancel(str(e), disarm=True)

    def pose_message(self, data):
        from geometry_msgs.msg import PoseStamped
        values = [float(data.get(k, 0)) for k in ('x', 'y', 'yaw')]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('坐标和朝向必须是有限数值')
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.node.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = values[:2]
        p.pose.orientation.z, p.pose.orientation.w = math.sin(values[2]/2), math.cos(values[2]/2)
        return p

    def check_goal(self, pose):
        with self.node._lock:
            msg = self.node._latest_map_msg
        if msg is None:
            raise ValueError('尚未收到地图')
        theta = yaw(msg.info.origin.orientation)
        dx, dy = pose.pose.position.x-msg.info.origin.position.x, pose.pose.position.y-msg.info.origin.position.y
        x = math.floor((math.cos(theta)*dx+math.sin(theta)*dy)/msg.info.resolution)
        y = math.floor((-math.sin(theta)*dx+math.cos(theta)*dy)/msg.info.resolution)
        if not 0 <= x < msg.info.width or not 0 <= y < msg.info.height:
            raise ValueError('目标不在地图范围内')
        if msg.data[y*msg.info.width+x] < 0 or msg.data[y*msg.info.width+x] >= 25:
            raise ValueError('目标位于障碍物或未知区域，请选择空白可通行区域')

    def initial_pose(self, data):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        if not self.running():
            raise ValueError('请先加载地图')
        self.cancel('已设置初始位置，等待定位收敛')
        self.route_preview.clear()
        self.preview_cache=None
        p = self.pose_message(data)
        self.check_goal(p)
        msg = PoseWithCovarianceStamped()
        msg.header, msg.pose.pose = p.header, p.pose
        # 2026-10-24: 0.25 正好等于 pose_ok 的严格阈值(<0.25)，AMCL 首帧
        # 原样回 0.25 时门禁永远不通过 —— 那次「等 26 分钟定位失效」的直接原因。
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.0625
        msg.pose.covariance[35] = 0.015
        self.pose_ok = False
        self.last_amcl = None
        self.pose_settle.start(msg.header.stamp.sec + msg.header.stamp.nanosec/1e9)
        self.initial_pub.publish(msg)
        self.nomotion_next = 0.0

    def send_goal(self, data, sid, preview=False, start=None, on_result=None):
        from nav2_msgs.action import NavigateToPose, ComputePathToPose
        from action_msgs.msg import GoalStatus
        with self.lock:
            if not self.localized(time.monotonic()) or self.last_pose is None:
                raise ValueError('localization not ready: cov=%s amcl_age=%.1fs; nomotion update requested, retry in ~1s'
                                 % (self.pose_cov, (time.monotonic()-self.last_amcl) if self.last_amcl else -1.0))
            if self.pending or self.navigating:
                raise ValueError('已有任务，请先取消')
            client = self.planner if preview else self.nav
            required = ('planner_server',) if preview else ('planner_server','bt_navigator','controller_server','velocity_smoother')
            if not client.server_is_ready() or not all(self.lifecycle.get(k) == 3 for k in required):
                raise ValueError('导航服务尚未就绪')
            pose = self.pose_message(data)
            self.check_goal(pose)
            if not preview:
                validate_navigation(self.config)
                if not self.armed() or self.owner() != sid:
                    raise ValueError('请先由当前页面启用电机')
                if is_trial(self.config):
                    if on_result is not None:raise ValueError('低速试航只支持单目标导航')
                    cache=getattr(self,'preview_cache',None)
                    if not cache or cache['map']!=self.active_map or time.monotonic()-cache['at']>60:
                        raise ValueError('请先预览这次试航路径')
                    if any(abs(cache['goal'][k]-float(data.get(k,0)))>1e-6 for k in ('x','y','yaw')):
                        raise ValueError('目标已改变，请重新预览')
                    if math.hypot(data['x']-self.last_pose['x'],data['y']-self.last_pose['y'])>self.config['trial_max_goal_m']:
                        raise ValueError('目标距离超过当前监督导航上限 %.1f 米，请选择更近目标或分段巡检' % self.config['trial_max_goal_m'])
                    if cache['length']>self.config['trial_max_path_m']:
                        raise ValueError('规划路径 %.1f 米，超过当前单段上限 %.1f 米，请调整目标或分段巡检' % (cache['length'],self.config['trial_max_path_m']))
                    if math.hypot(cache['start']['x']-self.last_pose['x'],cache['start']['y']-self.last_pose['y'])>.10:
                        raise ValueError('机器人已移动，请重新预览')
                    if abs(math.atan2(math.sin(cache['start']['yaw']-self.last_pose['yaw']),math.cos(cache['start']['yaw']-self.last_pose['yaw'])))>math.radians(15):
                        raise ValueError('车头方向已改变，请重新预览')
                    motor=yaml.safe_load((self.root/'motor/motor_params.yaml').read_text())['motor_driver']['ros__parameters']
                    if motor.get('steer_mode')!='servo' or motor.get('servo_enable') is not True:raise ValueError('转向模式已改变，不能试航')
                    with self.node._lock:self.trial_origin=dict(self.node._odometry or {})
                    if not self.trial_origin or not -.05<time.time()-self.trial_origin['stamp']<.6:raise ValueError('等待新鲜速度反馈')
                    if not fresh(time.monotonic(),self.trial_scan_at,.4):raise ValueError('等待新鲜雷达反馈')
                    if self.trial_clearance is None or self.trial_clearance<self.config['trial_clearance_m']:raise ValueError('近处有障碍或净空未知')
                    self.trial_started=self.last_control_t=time.monotonic();self.trial_no_motion=None;self.trial_drive.reset()
                    self.preview_cache=None
            if on_result is None:
                self.route_preview.clear()
            self.epoch += 1
            epoch = self.epoch
            self.goal = dict(x=pose.pose.position.x, y=pose.pose.position.y, yaw=float(data.get('yaw', 0)))
            if preview:self.path = [];self.preview_cache=None
            self.pending = True
            self.nav_owner = sid
            self.last_client = time.monotonic()
            self.status = 'planning' if preview else 'navigating'
            self.note = '正在计算路径' if preview else '导航任务已提交'
            goal = ComputePathToPose.Goal() if preview else NavigateToPose.Goal()
            if not preview:
                goal.pose = pose
            else:
                goal.goal = pose
                goal.planner_id = 'GridBased'
                if start is not None:
                    goal.start = self.pose_message(start)
                    goal.use_start = True
            self.navigating = not preview
            self.last_command = None
            self._command_grace_until = time.monotonic() + 3.0
            self._command_stale_from = None
            preview_start=dict(self.last_pose)
            def accepted(future):
                with self.lock:
                    try:
                        handle = future.result()
                        if epoch != self.epoch:
                            if handle.accepted:
                                handle.cancel_goal_async()
                            return
                        if not handle.accepted:
                            raise ValueError('导航服务拒绝任务')
                        self.goal_handle = handle
                        def finished(f):
                            with self.lock:
                                if epoch != self.epoch:
                                    return
                                try:
                                    result = f.result()
                                    if result.status != GoalStatus.STATUS_SUCCEEDED:
                                        raise ValueError('任务未完成：status=%d（4=成功 5=被取消 6=中止）' % result.status)
                                    if preview:
                                        self.path = [[p.pose.position.x, p.pose.position.y] for p in result.result.path.poses]
                                    path = list(self.path)
                                    target=dict(self.goal)
                                    self.cancel('路径预览完成，未驱动车轮' if preview else '已到达目标，车辆已停用' if on_result is None else '已到达巡检点', disarm=not preview and on_result is None, keep_route=on_result is not None, keep_mission=on_result is not None)
                                    if preview and on_result is None:
                                        self.preview_cache=dict(map=self.active_map,goal=target,start=preview_start,at=time.monotonic(),
                                            length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:]))) if len(path)>1 else None
                                    self.status = 'ready'
                                    if on_result is not None:
                                        on_result(path, None)
                                except Exception as e:
                                    self.cancel(str(e), disarm=not preview, keep_route=on_result is not None, keep_mission=on_result is not None)
                                    if on_result is not None:
                                        on_result(None, str(e))
                        handle.get_result_async().add_done_callback(finished)
                    except Exception as e:
                        self.cancel(str(e), disarm=not preview, keep_route=on_result is not None, keep_mission=on_result is not None)
                        if on_result is not None:
                            on_result(None, str(e))
            try:
                client.send_goal_async(goal).add_done_callback(accepted)
            except Exception as e:
                self.cancel(str(e), disarm=not preview, keep_route=on_result is not None, keep_mission=on_result is not None)
                if on_result is not None:
                    on_result(None, str(e))
                else:
                    raise

    def map_regions(self):
        """当前定位地图的语义区域；按文件 mtime 缓存，payload 每拍调用也不读盘。"""
        name = self.active_map
        if not name:
            return []
        try:
            path = self.store.path(name).parent / 'regions.json'
            mtime = path.stat().st_mtime if path.exists() else 0.
        except (OSError, ValueError):
            return []
        cache = getattr(self, '_regions_cache', None)
        if cache and cache[0] == name and cache[1] == mtime:
            return cache[2]
        try:
            regions = self.store.regions(name)
        except (OSError, ValueError):
            regions = []
        self._regions_cache = (name, mtime, regions)
        return regions

    def patrol_leg_begin(self, sid):
        # 巡检每段开始：把试航链路摆成「正在导航」的状态，否则 Nav2 的
        # cmd_vel 会被 on_command 丢掉（它只在 navigating 为真时接收）。
        with self.lock:
            with self.node._lock:
                odom = dict(self.node._odometry or {})
            self.navigating = True
            self.nav_owner = sid
            self.last_client = time.monotonic()
            self.last_command = None
            self._command_grace_until = time.monotonic() + 3.0
            self._command_stale_from = None
            self.trial_origin = odom
            self.trial_started = self.last_control_t = time.monotonic()
            self.trial_no_motion = None
            if getattr(self, 'trial_drive', None):
                self.trial_drive.reset()

    def patrol_leg_end(self):
        with self.lock:
            self.navigating = False
            self.nav_owner = None
            self.last_command = None
            self.command = (0.0, 0.0)
            if self.trial_drive:self.trial_drive.reset()
        self.send_drive(0.0, 0.0, self.armed())

    def preview_patrol(self, data, sid):
        with self.lock:
            task = validate_task(data)
            if task['map'] != self.active_map:
                raise ValueError('任务绑定的地图与当前定位地图不同，请先加载对应地图')
            if self.pending or self.navigating:
                raise ValueError('已有导航或预览任务，请先取消')
            if not self.localized(time.monotonic()):
                raise ValueError('请先设置初始位置，并等待定位稳定')
            for i, p in enumerate(task['waypoints']):
                try:
                    self.check_goal(self.pose_message(p))
                except ValueError as e:
                    raise ValueError(f'第 {i+1} 个巡检点无效：{e}') from e
            self.route_preview.start(task, dict(self.last_pose),
                lambda start, target, done: self.send_goal(target, sid, preview=True, start=start, on_result=done))

    def start(self, name=None, mode='localization'):
        with self.lock:
            if self.running():
                raise ValueError('定位已启动；请先停止再切换地图')
            if self.armed():
                # 2026-10-27: 这里原来是直接拒绝。断线重连会保持电机使能（驾驶权
                # 跨重连恢复），于是用户点「加载定位」只看到一句报错、不知道要先
                # 解除使能。切换模式本身就该是停机的，直接自动解除更符合预期。
                self.disarm()
                self.note = '已自动解除电机使能，正在切换定位模式'
            mapfile = self.store.path(name) if mode=='localization' else ''
            if not Path('/dev/ttyUSB0').exists():
                raise ValueError('雷达设备 /dev/ttyUSB0 不存在，请重新插拔雷达 USB')
            if subprocess.run(['pgrep', '-f', '^[^ ]*python3 .*motor_driver[.]py'],
                              stdout=subprocess.DEVNULL).returncode == 0:
                raise ValueError('旧 motor_driver 正在运行，会与激光里程计争用 TF，请先停掉旧驱动')
        check = subprocess.run([sys.executable, str(self.root/'slam'/'mpu6050_node.py'), '--check'],
                               capture_output=True, text=True, timeout=10)
        if check.returncode:
            raise ValueError('GY-521 检查失败，保留当前管线：'+(check.stderr or check.stdout).strip())
        subprocess.run([sys.executable, str(self.here/'build_nav_params.py')], check=True, timeout=15)
        subprocess.run(['bash', str(self.root/'slam'/'mapping_ctl.sh'), 'stop'], check=True, timeout=15,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        with self.node._lock:
            self.node._latest_map_msg = None
            self.node._map_dirty = False
            self.node._trajectory.clear()
            self.node._traj_total = self.node._traj_sent = 0
        self.tf.clear()
        with self.lock:
            self.last_pose = self.last_amcl = None
            self.preview_cache=None
            self.path = []
            self.route_preview.clear()
            self.goal = None
            self.pose_ok = False
            self.reset_lifecycle()
            env = dict(os.environ, ROBOT_NAV_MAP=str(mapfile), ROBOT_NAV_MODE=mode)
            with open('/tmp/navigation.log', 'w') as log:
                self.process = subprocess.Popen(['ros2', 'launch', str(self.here/'navigation_launch.py')],
                    env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            proc = Path('/proc')/str(self.process.pid)
            token = proc.joinpath('stat').read_text().split(') ', 1)[1].split()[19]
            self.session_file.write_text(json.dumps({'pid': self.process.pid, 'start': token}))
            self.active_map = name
            self.mode = mode
            self.status, self.note = 'localizing', '正在加载地图；请在地图上设置机器人的实际位置和朝向'
            if mode=='mapping':self.status,self.note='mapping','建图与避障服务启动中；尚未自动行驶'

    def stop(self):
        self.cancel('定位已停止', disarm=True)
        with self.lock:
            if self.running():
                os.killpg(self.process.pid, signal.SIGINT)
                try:
                    self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    self.process.wait(timeout=5)
            self.process = None
            self.session_file.unlink(missing_ok=True)
            self.last_pose = self.last_amcl = None
            self.preview_cache=None
            self.pose_ok = False
            self.reset_lifecycle()
            self.status, self.note = 'off', '定位已停止'

    def control(self, data, sid):
        action = data.get('action')
        if action == 'status':
            return  # Read-only refresh; the web handler emits the current payload.
        if action in ('explore_preview','explore_start','explore_finish','patrol_start','mission_stop','alert_ack'):
            # Route patrol includes arrival settling and evidence recording.
            if self.patrol_runner.active and action not in ('mission_stop',):
                raise ValueError('请先停止当前巡检任务')
            if action == 'patrol_start':
                return self.patrol_runner.start(data,sid)
            if action == 'mission_stop':
                self.patrol_runner.cancel('操作者停止巡检')
                if self.autonomy: self.autonomy.control(data,sid)
                return
            if self.autonomy:
                return self.autonomy.control(data,sid)
            raise ValueError('自动任务模块尚未就绪')
        if (self.patrol_runner.active or (getattr(self,'autonomy',None) and self.autonomy.active)) and action not in ('heartbeat','cancel','stop'):
            raise ValueError('请先停止自动任务，再修改地图、目标或路线')
        if action == 'patrol_save':
            task = self.patrols.save(data.get('task'))
            self.socketio.emit('patrol_loaded', task, to=sid)
            self.note = '巡检任务已保存，尚未执行'
            return
        if action == 'patrol_load':
            self.socketio.emit('patrol_loaded', self.patrols.get(data.get('name')), to=sid)
            return
        if action == 'patrol_preview':
            self.preview_patrol(data.get('task'), sid)
            return
        if action == 'heartbeat':
            with self.lock:
                if self.autonomy:self.autonomy.heartbeat(sid)
                if self.patrol_runner.active:self.patrol_runner.heartbeat(sid)
                if self.navigating and sid == self.nav_owner:
                    self.last_client = time.monotonic()
            return
        if action == 'cancel':
            self.cancel('操作者取消导航', disarm=True)
            return
        if action == 'initial_pose':
            self.initial_pose(data)
        elif action in ('goal', 'preview'):
            self.send_goal(data, sid, preview=action == 'preview')
        elif action in ('save', 'load', 'stop','prepare_mapping'):
            with self.lock:
                if self.busy:
                    raise ValueError('地图操作正在进行，请稍候')
                self.busy = True
            def worker():
                try:
                    if action == 'save':
                        with self.node._lock:
                            msg = self.node._latest_map_msg
                            age = time.monotonic() - self.node._last_map_t
                        if msg is None or age > 10:
                            raise ValueError('没有新鲜地图可保存，请先启动建图')
                        path = self.store.save(msg, data.get('name'))
                        self.note = '地图已保存：'+path.parent.name
                    elif action=='prepare_mapping':
                        self.start(mode='mapping')
                    elif action == 'load':
                        self.start(data.get('name'))
                    else:
                        self.stop()
                except Exception as e:
                    self.note = str(e)
                    self.socketio.emit('navigation_error', {'message': str(e)}, to=sid)
                finally:
                    with self.lock:
                        self.busy = False
                    self.emit()
            threading.Thread(target=worker, daemon=True).start()
        else:
            raise ValueError('未知导航操作')
