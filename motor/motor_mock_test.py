#!/usr/bin/env python3
"""Mock 端到端测试 (FakeMotorHW, 不碰 GPIO) —— 单进程内完成。

一个节点同时扮演三个角色:
  * Web 前端 —— 20Hz 持续发 /cmd_vel(含零指令,所以看门狗不会误杀);
  * arm 客户端 —— 调 /motor_enable;
  * 测量端 —— 订阅 /wheel_speed 与 /odom 断言。

为什么不用 shell + `ros2 topic pub` / `ros2 topic echo`:
  1. `ros2 topic echo --once` 在本机 AidLux 上会在 choose_qos 阶段 xmlrpc
     崩溃并**静默返回空**,拿不到消息却让断言假通过(实测已踩);
  2. 反复 kill/重启 `ros2 topic pub` 之间必有 >1s 空档,而看门狗 0.5s 就把
     电机解除 armed —— 后续测量全部落空(实测已踩)。
一个进程按时间表发指令并同步采样,这两个坑都不存在。

跑三轮:
  A) use_encoders:=true   闭环 + 轮式里程计,/odom 应前进
  B) use_encoders:=false  开环(当前真机配置),占空比与指令线性
  C) use_encoders:=true 但读不到计数 —— 复现「积分残留」的那一轮

三轮都断言的核心回归:**摇杆松手后 duty 必须归零**。
必须保持 armed(靠持续发零指令)才能测到这条;走看门狗路径的话
_set_armed(False) 本来就会清积分,测不出这个 bug。

注意 A、B 两轮其实**抓不到**这个 bug:A 有真实反馈,积分自然收敛;
B 走前馈,ff×0/max_lin 恒为 0。只有 C(err ≡ 目标值)才会让积分顶到
i_limit 并在松手后残留 —— 修复前 C 的「松手归零」确实 FAIL,
所以那一条断言是承重的。
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = '/tmp/motor_driver.log'
RATE_HZ = 20.0
WATCHDOG_S = 0.5            # cmd_vel_timeout,与 motor_params.yaml 一致

FAILURES = []


def check(label, ok, detail=''):
    print(('  ✓ ' if ok else '  ✗ FAIL: ') + label + ((' — ' + detail) if detail else ''))
    if not ok:
        FAILURES.append(label)
    return ok


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


class Harness(Node):
    """前端 + 测量端。"""

    def __init__(self):
        super().__init__('mock_harness')
        self.target_v = 0.0
        self.publishing = True
        self.duty = (0.0, 0.0)
        self.duty_ts = time.monotonic()
        self.odom_x = 0.0
        self.odom_ts = time.monotonic()
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/wheel_speed', self._on_duty, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_timer(1.0 / RATE_HZ, self._tick)

    def _tick(self):
        if not self.publishing:
            return
        m = Twist()
        m.linear.x = self.target_v
        self._pub.publish(m)

    def _on_duty(self, m):
        # linear.z = 左轮 duty, angular.x = 右轮 duty (见 motor_driver._control_tick)
        self.duty = (m.linear.z, m.angular.x)
        self.duty_ts = time.monotonic()

    def _on_odom(self, m):
        self.odom_x = m.pose.pose.position.x
        self.odom_ts = time.monotonic()

    def arm(self, timeout=8.0):
        cli = self.create_client(SetBool, '/motor_enable')
        end = time.monotonic() + timeout
        while not cli.service_is_ready() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not cli.service_is_ready():
            return False
        fut = cli.call_async(SetBool.Request(data=True))
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(fut.done() and fut.result() and fut.result().success)


def start_driver(enc, feedback=True):
    log = open(LOG, 'wb')
    cmd = ['python3', os.path.join(HERE, 'motor_driver.py'),
           '--ros-args',
           '--params-file', os.path.join(HERE, 'motor_params.yaml'),
           '-p', 'use_mock_hw:=true',
           '-p', 'use_encoders:=%s' % ('true' if enc else 'false')]
    if not feedback:
        # 让 FakeMotorHW 恒返回 0 计数 = 编码器没接线。这是当初踩坑的配置
        # (编码器没接却跑 PID),也是接编码器之前误翻 use_encoders 的后果。
        # 不能用「把 counts_per_meter 调大」来模拟:那个参数同时喂给
        # FakeMotorHW 和驱动,分子分母一起缩放,改多少都抵消(实测踩过)。
        cmd += ['-p', 'mock_encoder_feedback:=false']
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)


def stop_driver(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        proc.terminate()
    try:
        proc.wait(timeout=6)
    except Exception:
        proc.kill()


def run_phase(enc, feedback=True):
    label = 'A 闭环 use_encoders:=true' if enc else 'B 开环 use_encoders:=false (真机配置)'
    if not feedback:
        label = 'C 闭环但无编码器反馈 (use_encoders:=true, 计数读不到)'
    print('\n════════ 轮次 %s ════════' % label)

    proc = start_driver(enc, feedback)
    h = None
    try:
        time.sleep(6)
        if proc.poll() is not None:
            check('节点启动', False, '进程已退出,见 %s' % LOG)
            return
        check('节点启动', True)

        h = Harness()
        spin_for(h, 1.0)                      # 先让 20Hz 零指令流跑起来
        check('arm 服务调用', h.arm())

        # ── 待机 ──
        spin_for(h, 1.0)
        check('待机 duty 为 0', h.duty == (0.0, 0.0), 'duty=%s' % (h.duty,))

        # ── 前进 ──
        h.target_v = 0.1
        spin_for(h, 3.0)
        dl, dr = h.duty
        check('两轮都有输出', dl > 0.05 and dr > 0.05, 'duty=(%.3f, %.3f)' % (dl, dr))

        if enc and feedback:
            x0 = h.odom_x
            spin_for(h, 2.0)
            check('闭环 /odom 前进', h.odom_x > x0 + 0.05,
                  'x: %.3f → %.3f' % (x0, h.odom_x))
        elif not enc:
            # 开环:duty = pid_ff × |v| / max_linear_speed = 0.1/0.3 = 0.333
            check('开环 duty ≈ 0.333 与指令线性', abs(dl - 1.0 / 3.0) < 0.01,
                  'duty_l=%.4f (期望 0.3333)' % dl)
        else:
            # 轮次 C:反馈恒 0 → err ≡ 目标值 → 积分顶到 i_limit,duty 明显高于
            # 前馈应有的 0.333。这正是修复前真机上的状态。
            check('无反馈时闭环确实跑偏(前置条件)', dl > 0.45,
                  'duty_l=%.4f (前馈应给 0.333,多出来的是积分)' % dl)

        # ── 核心回归:松手 ──
        # 持续发零指令(模拟 Web),保持 armed;走看门狗路径的话测不出 bug ——
        # _set_armed(False) 本来就会 reset 积分。三轮都要过这一关,其中轮次 C
        # 是唯一真正复现「积分残留」的场景(已在修复前验证过它会 FAIL)。
        h.target_v = 0.0
        spin_for(h, 1.0)                      # 等斜率限幅把 duty 收到 0
        samples = []
        for _ in range(10):
            spin_for(h, 0.1)
            samples.append(h.duty)
        worst = max(max(abs(a), abs(b)) for a, b in samples)
        check('松手后 duty 立即归零(连采 10 次)', worst < 1e-9,
              '残留 duty=%.4f (修复前约 0.6,> deadband 0.05 会持续驱动电机)' % worst)

        # ── 看门狗:彻底停发 ──
        h.publishing = False
        spin_for(h, 1.0 + WATCHDOG_S + 1.5)
        quiet = time.monotonic() - h.duty_ts
        check('停发后看门狗停机(解除 armed,/wheel_speed 静默)',
              quiet > WATCHDOG_S + 0.5, '静默 %.1fs' % quiet)
    finally:
        if h is not None:
            h.destroy_node()
        stop_driver(proc)


def main():
    rclpy.init()
    try:
        run_phase(True, feedback=True)      # A 闭环
        run_phase(False, feedback=True)     # B 开环(真机)
        run_phase(True, feedback=False)     # C 闭环但无反馈 —— 积分残留回归
    finally:
        rclpy.shutdown()

    print()
    if FAILURES:
        print('=== DONE: %d 项失败 ===' % len(FAILURES))
        for f in FAILURES:
            print('  ✗ ' + f)
        return 1
    print('=== DONE: 全部通过 ===')
    return 0


if __name__ == '__main__':
    sys.exit(main())
