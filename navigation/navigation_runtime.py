"""Web navigation integration, gated actuator output, and owned Nav2 lifecycle.

Nav2 publishes only /navigation/cmd_vel_smoothed, never the manual /cmd_vel.
No navigation packet can arm a disarmed chassis. Hardware output is disabled
until measured steering/speed calibration is completed.
"""
import math
import json
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
from patrol_store import PatrolStore, validate_task
from patrol_preview import PatrolPreview


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
        self.lock = threading.RLock()
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
        self.pose_ok = False
        self.command = (0.0, 0.0)
        self.path = []
        self.lifecycle = {}
        self.lifecycle_pending = set()
        self.lifecycle_clients = {name: node.create_client(GetState, '/'+name+'/get_state')
                                  for name in ('amcl', 'map_server', 'planner_server', 'bt_navigator', 'velocity_smoother', 'controller_server')}
        self.nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.cmd_sub = node.create_subscription(Twist, '/navigation/cmd_vel_smoothed', self.on_command, 1)
        self.pose_sub = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.timer = node.create_timer(0.05, self.tick)
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
        if not self.running():
            return
        for name, client in self.lifecycle_clients.items():
            if name in self.lifecycle_pending or not client.service_is_ready():
                continue
            self.lifecycle_pending.add(name)
            def done(future, key=name):
                try:
                    self.lifecycle[key] = future.result().current_state.id
                except Exception:
                    self.lifecycle[key] = 0
                self.lifecycle_pending.discard(key)
            client.call_async(GetState.Request()).add_done_callback(done)

    def running(self):
        return self.process is not None and self.process.poll() is None

    def calibration_error(self):
        try:
            validate_calibration(self.config)
            return None
        except ValueError as e:
            return str(e)

    def payload(self):
        with self.lock:
            now = time.monotonic()
            ready = self.localized(now)
            return dict(status=self.status, message=self.note, busy=self.busy,
                running=self.running(), maps=self.store.list(), selected_map=self.active_map,
                localized=ready, calibration_error=self.calibration_error(),
                initial_pose_ready=all(self.lifecycle.get(k) == 3 for k in ('map_server','amcl')),
                navigating=self.navigating, goal=self.goal, path=self.path,
                mode=getattr(self,'mode','localization'),
                autonomy=self.autonomy.snapshot() if getattr(self,'autonomy',None) else None,
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

    def localized(self, now):
        # AMCL need not republish covariance while stationary; TF must stay fresh.
        if getattr(self,'mode','localization') == 'mapping':
            with self.node._lock:
                return self.running() and self.last_pose is not None and fresh(now,self.node._last_map_t,10)
        return (self.running() and self.pose_ok and self.last_pose is not None
                and self.lifecycle.get('amcl') == 3 and self.lifecycle.get('map_server') == 3)

    def on_pose(self, msg):
        with self.lock:
            self.last_amcl = time.monotonic()
            cov = msg.pose.covariance
            self.pose_ok = all(math.isfinite(cov[i]) and 0 <= cov[i] < limit
                               for i, limit in ((0, 0.25), (7, 0.25), (35, 0.25)))

    def on_command(self, msg):
        with self.lock:
            if self.navigating:
                self.command = (msg.linear.x, msg.angular.z)
                self.last_command = time.monotonic()

    def cancel(self, reason='导航已取消', disarm=False, keep_route=False, keep_mission=False):
        with self.lock:
            if not keep_mission and getattr(self,'autonomy',None):
                self.autonomy.cancelled(reason)
            if not keep_route and hasattr(self, 'route_preview'):
                self.route_preview.cancel(reason)
            had_motion = self.navigating
            self.epoch += 1  # invalidate in-flight goal acceptance/results
            self.navigating = False
            self.pending = False
            self.last_command = None
            self.command = (0.0, 0.0)
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            if had_motion:
                self.send_drive(0.0, 0.0, False if disarm else self.armed())
            if disarm and had_motion:
                self.disarm()
            self.status = 'localizing' if self.running() else 'off'
            self.note = reason

    def tick(self):
        from rclpy.time import Time
        with self.lock:
            now = time.monotonic()
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
            if getattr(self,'mode','localization')=='mapping' and self.status=='mapping' and self.localized(now) and all(self.lifecycle.get(k)==3 for k in ('planner_server','controller_server','bt_navigator','velocity_smoother')):
                self.note='建图与避障服务已就绪；可预览探索目标，完成标定后开始自动建图'
            if not self.navigating:
                return
            reason = None
            with self.node._lock:
                scan_t = self.node._last_scan_t or None
            if not self.armed() or self.owner() != self.nav_owner:
                reason = '驾驶使能或驾驶权已改变，导航停止'
            elif not fresh(now, self.last_client, 0.6):
                reason = '操作者链路超时，导航停止'
            elif not self.localized(now) or self.last_pose is None:
                reason = '定位或坐标变换失效，导航停止'
            elif not fresh(now, scan_t, self.config['scan_timeout_s']):
                reason = '雷达数据过期，导航停止'
            elif self.last_command is not None and not fresh(now, self.last_command, self.config['command_timeout_s']):
                reason = '导航速度指令超时，导航停止'
            elif self.last_command is None:
                # Goal planning can take seconds, but no old command may be replayed.
                self.send_drive(0.0, 0.0, self.armed())
                return
            if reason:
                self.cancel(reason, disarm=True)
                return
            try:
                l, a = twist_to_joystick(*self.command, self.config)
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
        p = self.pose_message(data)
        self.check_goal(p)
        msg = PoseWithCovarianceStamped()
        msg.header, msg.pose.pose = p.header, p.pose
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.pose_ok = False
        self.last_amcl = None
        self.initial_pub.publish(msg)

    def send_goal(self, data, sid, preview=False, start=None, on_result=None):
        from nav2_msgs.action import NavigateToPose, ComputePathToPose
        from action_msgs.msg import GoalStatus
        with self.lock:
            if not self.localized(time.monotonic()) or self.last_pose is None:
                raise ValueError('请先设置初始位置，并等待定位稳定')
            if self.pending or self.navigating:
                raise ValueError('已有任务，请先取消')
            client = self.planner if preview else self.nav
            required = ('planner_server',) if preview else ('planner_server','bt_navigator','controller_server','velocity_smoother')
            if not client.server_is_ready() or not all(self.lifecycle.get(k) == 3 for k in required):
                raise ValueError('导航服务尚未就绪')
            pose = self.pose_message(data)
            self.check_goal(pose)
            if not preview:
                validate_calibration(self.config)
                if not self.armed() or self.owner() != sid:
                    raise ValueError('请先由当前页面启用电机')
            if on_result is None:
                self.route_preview.clear()
            self.epoch += 1
            epoch = self.epoch
            self.goal = dict(x=pose.pose.position.x, y=pose.pose.position.y, yaw=float(data.get('yaw', 0)))
            self.path = []
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
                                        raise ValueError('任务未完成：无可行路径、受阻或已取消')
                                    if preview:
                                        self.path = [[p.pose.position.x, p.pose.position.y] for p in result.result.path.poses]
                                    path = list(self.path)
                                    self.cancel('路径预览完成，未驱动车轮' if preview else '已到达目标', keep_route=on_result is not None, keep_mission=on_result is not None)
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
                raise ValueError('请先解除电机使能，再切换建图或定位模式')
            mapfile = self.store.path(name) if mode=='localization' else ''
            if not Path('/dev/ttyUSB0').exists():
                raise ValueError('雷达设备 /dev/ttyUSB0 不存在，请重新插拔雷达 USB')
            if subprocess.run(['pgrep', '-f', '^[^ ]*python3 .*motor_driver[.]py'],
                              stdout=subprocess.DEVNULL).returncode == 0:
                raise ValueError('旧 motor_driver 正在运行，会与编码器里程计争用 GPIO/TF，请先停掉旧驱动')
        check = subprocess.run([sys.executable, str(self.root/'slam'/'wheel_odom.py'), '--check'],
                               capture_output=True, text=True, timeout=10)
        if check.returncode:
            raise ValueError(check.stderr.strip() or '编码器里程计标定未通过')
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
            self.path = []
            self.route_preview.clear()
            self.goal = None
            self.pose_ok = False
            self.lifecycle.clear()
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
            self.pose_ok = False
            self.status, self.note = 'off', '定位已停止'

    def control(self, data, sid):
        action = data.get('action')
        if action in ('explore_preview','explore_start','explore_finish','patrol_start','mission_stop','alert_ack'):
            if not self.autonomy:raise ValueError('自动任务模块尚未就绪')
            return self.autonomy.control(data,sid)
        if getattr(self,'autonomy',None) and self.autonomy.active and action not in ('heartbeat','cancel','stop'):
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
