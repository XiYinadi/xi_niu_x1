"""Supervised route: arrive, settle, inspect, persist, advance. Never arms motors."""
import copy
import math
import threading
import time
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class PatrolRunner:
    def __init__(self, nav):
        self.nav, self.lock = nav, nav.lock
        self.inspections = None
        self.active = False
        self.sid = self.task = self.name = self.inspection_id = None
        self.total = self.index = self.cycle = self.completed_points = 0
        self.cycles = 1
        self.status, self.message = 'idle', '等待巡检任务'
        self.last_client = None
        self.epoch = self.leg = 0
        self.handle = self.accept_future = self.archive_thread = None
        self.still_since = self.anchor = None
        self.odom_stale_since = None
        self.last_dwell_log = 0.
        self.arrived_at = self.phase_started = 0.
        self.record_id = None
        self.rotate_target = None
        self.rotate_then = None
        self.leg_target = None
        self._rot_cmd = 0.0
        self._rot_t = 0.0
        self._burst_until = 0.0
        self._settle_until = 0.0
        self._rot_error0 = None
        self._rot_done = 0.0
        self._gyro_t = 0.0
        self._gyro_seen = False
        self.rotate_target = None

    def camera_ready(self):
        if self.inspections is None:
            return False
        with self.inspections.lock:
            health = dict(self.inspections.model_status)
        now = time.time()
        return (health.get('online') is True and health.get('camera_online') is True
                and 0 <= now-health.get('checked_at', 0) < 25
                and 0 <= now-health.get('last_detection', 0) < 18)

    def snapshot(self):
        with self.lock:
            p = self.inspections.progress(self.inspection_id) if self.inspection_id else {}
            blocked = None
            if self.inspections:
                with self.inspections.lock:
                    if any(s['status'] in ('running','sampling','reporting','storage_error') and
                           (s['id'] != self.inspection_id or not self.active)
                           for s in self.inspections.sessions.values()):
                        blocked = '请先在检测与报告页结束已有巡检，或等待报告归档完成'
            return dict(active=self.active, kind='patrol', status=self.status,
                        message=self.message, name=self.name, index=self.index,
                        total=self.total, cycle=self.cycle, cycles=self.cycles,
                        completed_points=self.completed_points,
                        camera_ready=self.camera_ready(), inspection_id=self.inspection_id,
                        report_ready=p.get('status') == 'completed', report_status=p.get('report_status'),
                        inspection_error=blocked)

    def _guard(self, now):
        # 2026-10-26: detecting/reporting 期间**不查心跳** —— VLM 推理会占住
        # 进程 7~10 秒，心跳必然被挤掉；这两个阶段车已停稳，armed/localized
        # 仍然每拍校验，安全边界不变。
        if self.status not in ('detecting', 'reporting'):
            if self.last_client is None or now-self.last_client > 2.0:
                raise ValueError('操作者心跳超时，巡检停止')
        if not self.nav.armed() or self.nav.owner() != self.sid:
            raise ValueError('电机使能或驾驶权已改变，巡检停止')
        if self.status in ('rotating', 'direct'):
            # 原地旋转/直行阶段：允许 AMCL 协方差临时变大，只要求位姿持续可得。
            if self.nav.last_pose is None:
                lost = getattr(self, 'pose_lost', None) or now
                self.pose_lost = lost
                if now - lost > 1.5:
                    raise ValueError('定位失效，巡检停止')
            else:
                self.pose_lost = None
        elif not self.nav.localized(now):
            raise ValueError('定位失效，巡检停止')

    def start(self, data, sid):
        task = self.nav.patrols.get(data.get('name'))
        with self.lock:
            if self.active or self.accept_future is not None:
                raise ValueError('巡检或目标取消尚未完成，请稍候')
            if self.archive_thread is not None and self.archive_thread.is_alive():
                raise ValueError('正在归档上次巡检，请稍候')
            if self.nav.busy or self.nav.pending or self.nav.navigating or getattr(self.nav.autonomy, 'active', False):
                raise ValueError('请先停止当前导航或自动任务')
            if task['map'] != self.nav.active_map or self.nav.mode != 'localization':
                raise ValueError('请先加载任务对应的地图并完成定位')
            if not self.nav.localized(time.monotonic()):
                raise ValueError('请先设置初始位置，并等待定位稳定')
            if not self.nav.armed() or self.nav.owner() != sid:
                raise ValueError('请先由当前页面启用电机')
            error = self.nav.drive_error()
            if error:
                raise ValueError(error)
            if not self.nav.nav.server_is_ready() or any(self.nav.lifecycle.get(k) != 3 for k in
                    ('planner_server', 'bt_navigator', 'controller_server', 'velocity_smoother')):
                raise ValueError('导航服务尚未就绪')
            if not self.camera_ready():
                raise ValueError('相机或本地检测未就绪，不能开始拍照巡检')
            for point in task['waypoints']:
                self.nav.check_goal(self.nav.pose_message(point))
            session = self.inspections.start(dict(title='自动巡检 · '+task['name']),
                                             mode='automatic', task_snapshot=task)
            self.task, self.name, self.inspection_id = copy.deepcopy(task), task['name'], session['id']
            self.total, self.cycles = len(task['waypoints']), task['cycles']
            self.index = self.cycle = self.completed_points = 0
            self.sid, self.last_client = sid, time.monotonic()
            self.epoch += 1
            self.active = True
            self._next(self.epoch)

    def heartbeat(self, sid):
        with self.lock:
            if self.active and sid == self.sid:
                self.last_client = time.monotonic()

    def cancel(self, reason='巡检已停止'):
        with self.lock:
            if self.active:
                self._halt(reason, 'cancelled')

    def _halt(self, reason, status='failed'):
        self.active = False
        self.epoch += 1
        self.status, self.message = status, reason
        handle, self.handle = self.handle, None
        # Hardware first, before cancellation RPC, storage or report inference.
        self.nav.send_drive(0., 0., False)
        self.nav.disarm()
        self.nav.patrol_leg_end()
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                self.message += '；导航取消请求失败，输出已停用'
        sid = self.inspection_id
        def archive():
            try:
                self.inspections.close_automatic(sid, reason, status)
            except Exception as exc:
                with self.lock:
                    self.message += '；记录归档失败：'+str(exc)
        self.archive_thread = threading.Thread(target=archive, daemon=True)
        self.archive_thread.start()

    def _pose_matches(self):
        pose, target = self.nav.last_pose, self.task['waypoints'][self.index]
        if not pose:
            return False
        angle = math.atan2(math.sin(pose['yaw']-target['yaw']), math.cos(pose['yaw']-target['yaw']))
        return math.hypot(pose['x']-target['x'], pose['y']-target['y']) <= .25 and abs(angle) <= .5

    def _stationary(self, now):
        with self.nav.node._lock:
            odom = dict(self.nav.node._odometry or {})
            received = self.nav.node._last_odom_t
        # 2026-10-26: now 可能比 _last_odom_t 早几毫秒（同一轮里回调后更新），
        # 所以下界必须是负容差，不能是 0。
        stale = (not odom or received is None or not -.05 <= now-received < 2.0 or
                 not -.05 <= time.time()-odom.get('stamp', 0.) < 2.0)
        if stale:
            # 2026-10-26: 不再一次判死 —— 进程自身卡顿会让 _last_odom_t 变旧。
            # 持续 2 秒都不新鲜才停；期间当作「还没停稳」，继续等。
            first = self.odom_stale_since or now
            self.odom_stale_since = first
            try:
                with open('/tmp/stationary_debug.log', 'a', encoding='utf-8') as f:
                    f.write('%s has_odom=%s recv_age=%s stamp_age=%s stale_for=%.2f\n' % (
                        time.strftime('%H:%M:%S'), bool(odom),
                        None if received is None else round(now-received, 3),
                        None if not odom else round(time.time()-odom.get('stamp', 0.), 3),
                        now-first))
            except OSError:
                pass
            if now-first > 2.0:
                raise ValueError('停稳确认的里程计已过期，巡检停止')
            self.still_since = self.anchor = None
            return False
        self.odom_stale_since = None
        if not all(math.isfinite(odom[k]) for k in ('vx', 'vy', 'wz', 'x', 'y', 'yaw')):
            raise ValueError('停稳确认的里程计无效')
        # 2026-10-26: 原 0.015/0.04 在刚停下的几秒里过不去（里程计残余抖动）。
        quiet = math.hypot(odom['vx'], odom['vy']) <= .04 and abs(odom['wz']) <= .10
        if self.anchor is not None:
            if odom['generation'] != self.anchor['generation']:
                raise ValueError('停稳期间里程计参考发生改变')
            angle = math.atan2(math.sin(odom['yaw']-self.anchor['yaw']), math.cos(odom['yaw']-self.anchor['yaw']))
            quiet = quiet and math.hypot(odom['x']-self.anchor['x'], odom['y']-self.anchor['y']) <= .05 and abs(angle) <= .08
        if not quiet:
            if now - getattr(self, 'last_dwell_log', 0.) > .5:
                self.last_dwell_log = now
                try:
                    with open('/tmp/dwell_debug.log', 'a', encoding='utf-8') as f:
                        f.write('%s v=%.3f w=%.3f drift=%.3f quiet=False still=%.2f\n' % (
                            time.strftime('%H:%M:%S'),
                            math.hypot(odom['vx'], odom['vy']), abs(odom['wz']),
                            0. if self.anchor is None else math.hypot(odom['x']-self.anchor['x'], odom['y']-self.anchor['y']),
                            0. if self.still_since is None else now-self.still_since))
                except OSError:
                    pass
            self.still_since = self.anchor = None
            return False
        if self.still_since is None:
            self.still_since, self.anchor = now, odom
        return True

    def tick(self):
        with self.lock:
            if not self.active:
                return
            try:
                now = time.monotonic()
                self._guard(now)
                if self.status == 'rotating':
                    return self._rotate(now)
                if self.status == 'direct':
                    return self._drive_direct(now)
                if self.status == 'navigating':
                    if now-self.phase_started > self.nav.config.get('trial_max_duration_s', 180):
                        raise ValueError('点位导航超时，未计为到达')
                    return
                self.nav.send_drive(0., 0., self.nav.armed())
                quiet = self._stationary(now)
                if not self._pose_matches():
                    raise ValueError('停稳位置与目标不一致，未计为到达')
                if self.status == 'dwell':
                    required = max(1., self.task['waypoints'][self.index]['dwell_s'])
                    if now-self.phase_started > required+25:
                        raise ValueError('到点后未能持续停稳，停止巡检')
                    if quiet and now-self.still_since >= required:
                        self.record_id = self.inspections.record(self.inspection_id, automatic=True,
                            context=dict(arrived_at=self.arrived_at, settled_at=time.time(),
                                         stationary_s=now-self.still_since,
                                         arrival_basis='nav2_success_and_pose_and_fresh_odometry'))
                        self.status, self.message = 'detecting', '已停稳，正在拍照检测并保存记录'
                        self.phase_started = now
                elif self.status == 'detecting':
                    if not quiet:
                        raise ValueError('检测期间车辆发生移动，本点未完成')
                    if now-self.phase_started > 45:
                        raise ValueError('点位检测或保存超时，停止巡检')
                    progress = self.inspections.progress(self.inspection_id)
                    if progress['status'] == 'sampling':
                        return
                    record = progress.get('record') or {}
                    if record.get('id') != self.record_id or record.get('status') != 'ok':
                        raise ValueError('点位检测或保存失败：'+record.get('error', '缺少本点记录'))
                    if progress['next_step'] != self.completed_points+1:
                        raise ValueError('检测记录点次与导航点次不一致')
                    self.completed_points += 1
                    if record['verdict'] != 'clear':
                        self._halt('点位检测疑似火情或结果不确定，已保存证据，请人工复核', 'alert')
                        return
                    self.index += 1
                    self._next(self.epoch)
            except Exception as exc:
                self._halt(str(exc))

    def _rotate(self, now):
        """原地旋转：用陀螺仪积分做角度闭环（100Hz），不再等 7.8Hz 雷达。

        AMCL 位姿在纯旋转时滞后、协方差变大；GY-521 的 z 轴角速度 100Hz，
        积分出来的「已转角度」足够控制这一小段旋转。转够后再用 AMCL 校验一次。
        """
        pose = self.nav.last_pose
        if pose is None:
            self.pose_lost = getattr(self, 'pose_lost', None) or now
            if now - self.pose_lost > 1.5:
                raise ValueError('定位失效，巡检停止')
            self.nav.send_drive(0., 0., self.nav.armed(), 0.)
            return
        self.pose_lost = None
        error = math.atan2(math.sin(self.rotate_target-pose['yaw']), math.cos(self.rotate_target-pose['yaw']))
        if self._rot_error0 is None:
            self._rot_error0 = error
            self._rot_done = 0.
            self._gyro_t = now
            self._rot_cmd = 0.
            self._rot_t = now
        else:
            rate = self.nav.gyro_z()
            if rate is not None:
                self._rot_done += rate * max(0., min(.2, now-self._gyro_t))
                self._gyro_seen = True
            self._gyro_t = now
        remaining = self._rot_error0 - self._rot_done
        # 陀螺积分已到位，或 AMCL 角度已经很小 -> 结束这段旋转
        gyro_done = abs(remaining) < .06 if self._gyro_seen else abs(error) < .10
        if gyro_done or (abs(error) < .08 and abs(self._rot_done) > .05):
            self.nav.send_drive(0., 0., self.nav.armed(), 0.)
            self.rotate_target = None
            self._rot_error0 = None
            self._gyro_seen = False
            step, self.rotate_then = self.rotate_then, None
            if step == 'direct':
                self.status, self.phase_started = 'direct', now
                self._rot_cmd = 0.
                return
            if step == 'arrived':
                self.status, self.arrived_at = 'dwell', now
                self.phase_started, self.still_since = now, None
                return
            self._next(self.epoch)
            return
        if now - self.phase_started > 30:
            raise ValueError('原地旋转超时，巡检停止')
        # 连续慢转：0.12~0.22 占空比，剩余角度越小越慢
        # 2026-10-27 实测：0.12 -> 1°/s（卡住）；0.22 -> 6.4°/s；0.30 -> 9.6°/s。
        # 用户要求更快，取 0.28~0.30；仍用 2.0/s 斜率限制，避免起始冲击。
        # 2026-10-27 实测：0.30 -> 9.6°/s，0.40 -> 18°/s（90° 用 5.0 秒）。用户要求更快。
        target_speed = min(.40, max(.36, abs(remaining)*1.6))
        dt = min(.2, max(.0, now-self._rot_t)) if self._rot_t else .05
        slew = 2.6*dt
        self._rot_cmd = max(self._rot_cmd-slew, min(self._rot_cmd+slew, target_speed))
        self._rot_t = now
        speed = self._rot_cmd
        self._rot_cmd = math.copysign(speed, remaining)
        self.nav.send_drive(0., 0., self.nav.armed(), self._rot_cmd)

    def _drive_direct(self, now):
        """短腿直行：先对准方位，再低速直行，靠近后对准点位朝向。"""
        pose = self.nav.last_pose
        target = self.leg_target
        if target is None:
            raise ValueError('缺少点位目标，巡检停止')
        if pose is None:
            self.pose_lost = getattr(self, 'pose_lost', None) or now
            if now - self.pose_lost > 1.5:
                raise ValueError('定位失效，巡检停止')
            self.nav.send_drive(0., 0., self.nav.armed(), 0.)
            return
        self.pose_lost = None
        distance = math.hypot(target['x']-pose['x'], target['y']-pose['y'])
        if distance < .10:
            self.nav.send_drive(0., 0., self.nav.armed(), 0.)
            error = math.atan2(math.sin(target['yaw']-pose['yaw']), math.cos(target['yaw']-pose['yaw']))
            if abs(error) > math.radians(15):
                self.status, self.rotate_then = 'rotating', 'arrived'
                self.rotate_target, self.phase_started = target['yaw'], now
                return
            self.status, self.arrived_at = 'dwell', now
            self.phase_started, self.still_since = now, None
            return
        if now - self.phase_started > 30:
            raise ValueError('直行段超时，巡检停止')
        bearing = math.atan2(target['y']-pose['y'], target['x']-pose['x'])
        error = math.atan2(math.sin(bearing-pose['yaw']), math.cos(bearing-pose['yaw']))
        if abs(error) > math.radians(45):
            self.status, self.rotate_then = 'rotating', 'direct'
            self.rotate_target, self.phase_started = bearing, now
            return
        clearance = getattr(self.nav, 'trial_clearance', None)
        if clearance is not None and clearance < .18:
            raise ValueError('直行前方净空不足，巡检停止')
        target_speed = .32 if distance > .35 else .20
        # 0.6 秒斜坡：别让电机从 0 一步跳到 0.32（起步会窜）。
        speed = min(target_speed, .55*max(0., now-self.phase_started))
        steer = max(-.6, min(.6, error*1.5))
        self.nav.send_drive(speed, steer, self.nav.armed(), 0.)
        self.message = '直行 %d/%d · 距 %.2f m' % (self.index+1, self.total, distance)

    def _next(self, epoch):
        # All callers hold nav.lock through dispatch, serializing cancellation.
        if not self.active or epoch != self.epoch:
            return
        if self.index >= self.total:
            self.cycle += 1
            if self.cycle >= self.cycles:
                self._halt('巡检完成：%d 个点位 × %d 轮，记录已保存' % (self.total, self.cycles), 'completed')
                return
            self.index = 0
        try:
            self._guard(time.monotonic())
            target = self.task['waypoints'][self.index]
            if self.nav.config.get('navigation_mode') == 'supervised_trial':
                p = self.nav.last_pose
                if math.hypot(p['x']-target['x'], p['y']-target['y']) > self.nav.config['trial_max_goal_m']:
                    raise ValueError('下一点超出单段试航距离限制')
            pose = self.nav.last_pose
            if pose is not None:
                distance = math.hypot(target['x']-pose['x'], target['y']-pose['y'])
                bearing = math.atan2(target['y']-pose['y'], target['x']-pose['x'])
                if distance <= 2.0:
                    self.leg_target = dict(target)
                    heading = math.atan2(math.sin(bearing-pose['yaw']), math.cos(bearing-pose['yaw']))
                    if abs(heading) > math.radians(12):
                        self.status, self.rotate_then = 'rotating', 'direct'
                        self.rotate_target, self.phase_started = bearing, time.monotonic()
                        self.message = '原地旋转对准第 %d/%d 个点位（直行）' % (self.index+1, self.total)
                        return
                    self.status, self.phase_started = 'direct', time.monotonic()
                    self._rot_cmd = 0.
                    self.message = '直行前往第 %d/%d 个点位' % (self.index+1, self.total)
                    return
                heading = math.atan2(math.sin(target['yaw']-pose['yaw']), math.cos(target['yaw']-pose['yaw']))
                if abs(heading) > math.radians(30):
                    self.status, self.message = 'rotating', '原地旋转对准第 %d/%d 个点位' % (self.index+1, self.total)
                    self.rotate_target = target['yaw']
                    self.phase_started = time.monotonic()
                    return
            self.leg += 1
            token = (epoch, self.leg)
            self.status = 'navigating'
            self.message = '第 %d/%d 轮 · 前往第 %d/%d 个点位' % (self.cycle+1, self.cycles, self.index+1, self.total)
            self.phase_started = time.monotonic()
            self.nav.patrol_leg_begin(self.sid)
            goal = NavigateToPose.Goal()
            goal.pose = self.nav.pose_message(target)
            self.accept_future = self.nav.nav.send_goal_async(goal)
            self.accept_future.add_done_callback(lambda f: self._accepted(f, token))
        except Exception as exc:
            self._halt('下发点位失败：'+str(exc))

    def _current(self, token):
        return self.active and token == (self.epoch, self.leg) and self.status == 'navigating'

    def _accepted(self, future, token):
        with self.lock:
            if self.accept_future is future:
                self.accept_future = None
            try:
                handle = future.result()
                if not self._current(token):
                    if handle.accepted:
                        handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    raise ValueError('导航拒绝了该点位')
                self.handle = handle
                handle.get_result_async().add_done_callback(lambda f: self._result(f, token))
            except Exception as exc:
                if self._current(token):
                    self._halt('导航目标失败：'+str(exc))

    def _result(self, future, token):
        with self.lock:
            if not self._current(token):
                return
            try:
                status = future.result().status
                self.handle = None
                self.nav.patrol_leg_end()
                self._guard(time.monotonic())
                if status != GoalStatus.STATUS_SUCCEEDED:
                    raise ValueError('点位未到达（status=%d，5=取消 6=中止）' % status)
                if not self._pose_matches():
                    raise ValueError('导航结果与实际位置不一致，未计为到达')
                self.still_since = self.anchor = None
                self.arrived_at = time.time()
                self.phase_started = time.monotonic()
                self.status, self.message = 'dwell', '已到点，正在确认停稳并执行点位停留'
            except Exception as exc:
                self._halt(str(exc))
