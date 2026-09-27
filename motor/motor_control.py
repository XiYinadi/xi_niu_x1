#!/usr/bin/env python3
"""纯运动控制逻辑 — 无 ROS、无 GPIO,可 headless 单测。

原定位是「纯数学」,后来把 pwm_proc 的**链路失效判据**(下面的 `Deadman`)也放进
这里:判据本身是纯状态机,可这里放得下;pwm_proc.py 放不下 —— 它硬编
`GpioMotorHW`/`SoftServoPwm`(GPIO 申请在状态字段初始化之前),没有
`motor_driver.py` 那种 `use_mock_hw`,也没有 `--dry-run`,所以**完全无法单测**。
而 deadman 是整车唯一的安全网,不能是「只有真机跑起来才知道对不对」的东西。

判据不变:**不导入 rclpy、不碰 GPIO、不依赖时间流逝**(时间由调用方传入)。
"""


class Pid:
    """Simple velocity PID with integral limit and feed-forward support."""

    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.out_limit = out_limit
        self.integral = 0.0
        self.prev_err = 0.0
        self.last_out = 0.0

    def update(self, err, dt):
        if dt <= 1e-6:
            return 0.0
        # 条件积分抗饱和:输出已经饱和、且误差还在往同方向推时,不再累积积分。
        # 不这样做的话 —— 没有编码器反馈时 err 恒等于全部目标值 —— 积分会一路
        # 顶到 i_limit 并**在目标归零后原样残留**,输出 ki*integral ≈ 0.15 持续
        # 驱动电机,现场表现就是「摇杆松手后电机停不下来」。
        saturated = abs(self.last_out) >= self.out_limit - 1e-9
        pushing = (self.last_out > 0.0 and err > 0.0) or \
                  (self.last_out < 0.0 and err < 0.0)
        if not (saturated and pushing):
            self.integral = max(-self.i_limit,
                                min(self.i_limit, self.integral + err * dt))
        deriv = (err - self.prev_err) / dt
        self.prev_err = err
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        out = max(-self.out_limit, min(self.out_limit, out))
        self.last_out = out
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.last_out = 0.0


ZERO_SPEED_EPS = 1e-4      # 目标轮速低于此值(m/s)即视为「已归零」


def wheel_duties(t_l, t_r, v_l_meas, v_r_meas, dt, *,
                 use_encoders, ff, max_lin, max_duty, deadband,
                 pid_l=None, pid_r=None, zero_eps=ZERO_SPEED_EPS):
    """目标轮速(m/s) → 占空比(0..max_duty)。

    纯函数,不依赖 ROS/GPIO,便于单测 —— 这颗 bug 当初能潜伏就是因为这段
    逻辑埋在 MotorDriverNode._control_tick 里没法测。

    三条分支:
      * 目标归零 → 清 PID 积分并输出 0。**这是「摇杆松手后电机不停」的主修复**:
        编码器没接时 err 恒等于全部目标值,积分顶到 i_limit 后即便目标归零也
        原样残留,输出 ki*integral(≈0.15) 继续驱动电机,而它高于 pid_deadband
        所以不会被过滤掉。
      * 开环(编码器未接)→ 纯前馈,占空比与目标轮速成正比。没有反馈时闭环
        误差恒等于目标值,PID+前馈会把很小的指令放大成很大的占空比。
      * 闭环 → PID(误差 = 目标 − 实测)+ 前馈。

    返回 (d_l, d_r),均为非负;方向由调用方按目标速度符号另行设置。
    """
    if abs(t_l) < zero_eps and abs(t_r) < zero_eps:
        if pid_l is not None:
            pid_l.reset()
        if pid_r is not None:
            pid_r.reset()
        d_l = d_r = 0.0
    elif not use_encoders:
        d_l = ff * abs(t_l) / max_lin
        d_r = ff * abs(t_r) / max_lin
    else:
        d_l = pid_l.update(t_l - v_l_meas, dt) + ff * abs(t_l) / max_lin
        d_r = pid_r.update(t_r - v_r_meas, dt) + ff * abs(t_r) / max_lin

    d_l = max(0.0, min(max_duty, abs(d_l)))
    d_r = max(0.0, min(max_duty, abs(d_r)))
    if d_l < deadband:
        d_l = 0.0
    if d_r < deadband:
        d_r = 0.0
    return d_l, d_r


def twist_to_wheel_speeds(v, w, wheel_base):
    """Differential inverse kinematics → signed left/right wheel speeds (m/s)."""
    v_r = v + w * wheel_base / 2.0
    v_l = v - w * wheel_base / 2.0
    return v_l, v_r


def wheel_speeds_to_twist(v_l, v_r, wheel_base):
    """Left/right wheel speeds → (linear, angular) — used by wheel odometry."""
    v = (v_l + v_r) / 2.0
    w = (v_r - v_l) / wheel_base
    return v, w


def map_channels(v_l, v_r, swap_lr):
    """(物理左轮, 物理右轮) ↔ (通道 L, 通道 R) 的互换。

    `swap_lr=False`: 通道 L 接物理左轮,两者同序。
    `swap_lr=True` : 两轮通道交叉 —— 通道 L 的 H 桥实际接的是**右**轮。

    ⚠️ 这跟 `flip_*` 是**两件不同的事**,别混:
      * `flip_*` 翻的是**单个轮子**的转向(INA/INB 互换)。单独翻一侧会把「直行」
        变成「原地自转」—— 因为左右轮反向,前进分量抵消、只剩转动分量。
      * `map_channels` 交换的是**左右通道**。它保前进/后退不变(两轮同向分量不变),
        只把「转向」取反 —— 因为 (v_r − v_l) 变成了 (v_l − v_r)。

    所以:前进/后退正确但左右转相反 → 调 swap_lr;整机前后+转向全反 → 调 flip_*。

    控制环内部统一用**通道系**(duty/dir/flip 都是按 GPIO 通道定义的),
    只在两头换算:逆运动学出来后换成通道系,轮式里程计前换回物理系。
    """
    return (v_r, v_l) if swap_lr else (v_l, v_r)


def joy_to_channels(l, a, max_lin, max_ang, wheel_base, swap_lr=False):
    """归一化摇杆 (-1..1) → 通道系**归一化**带符号轮速。

    这是「直连 PWM 通道」的逆运动学:pwm_proc 用它把摇杆比值直接换成占空比,
    整条路不经过 ROS / /cmd_vel / motor_driver。

    复用 twist_to_wheel_speeds + map_channels 而不是另写一遍 —— 这两段的
    正负号约定(尤其 swap_lr 只翻转向)在真机上反复调错过,没有第二份的理由。

    返回值的绝对值**就是占空比**(开环,前馈系数 1.0)。所以直行时:
        摇杆 0.8  → s_l = s_r = 0.8  → duty_l = duty_r = 0.8
    即 duty 严格等于摇杆比值,中间没有任何斜率限幅/滤波。
    """
    v_l, v_r = twist_to_wheel_speeds(l * max_lin, a * max_ang, wheel_base)
    # 除以 max_lin 归一化:满摇杆直行恰好是 1.0(再由 max_duty 截断)
    return map_channels(v_l / max_lin, v_r / max_lin, swap_lr)


class Deadman:
    """带滞回的摇杆链路失联判据(纯状态机,时间由调用方传入)。

    跳闸:这一次的龄期超过 trip_s。
    恢复:跳闸之后,龄期必须**连续**不超过 trip_s,持续满 dwell_s(默认 2×trip)。

    ── 为什么不能只看单次龄期(这是实测出来的,不是假想) ─────────────
    只看阈值的话,链路在阈值附近会 chattering。2026-09-15 实测:浏览器把
    不可见标签页的 setInterval 钳到 1Hz,包间隔约 400ms > deadman 300ms,
    于是「来一包就恢复、300ms 后又跳闸」**严格一翻一翻地重复了 61 次**;
    而舵机和电机走**同一个门**,舵机每次翻转都是一次 park(完全不发脉冲、
    松掉)↔ set_command(全速转回目标角),现场看起来就是**舵机来回抖**。
    没有反馈线,每次复位都是一次全速机械动作,所以抖得很显眼。

    ── 为什么 dwell = 2×trip ─────────────────────────────────────
    dwell 必须**严格大于** trip,否则挡不住:400ms 间隔的链路,每个 fresh
    窗口恰好有 trip(300ms)长,攒够 300ms 就恢复、100ms 后又跳闸 —— 照样抖。
    取 2×trip 之后语义变得干脆:
        连续 fresh 满 2×trip  ⟹  这段窗口内一次都没跳闸
    于是判据没有中间地带:
        包间隔 < trip  → 从不跳闸,车照开
        包间隔 > trip  → 跳闸后**永远恢复不了**(直到链路重新变密)
    会持续跳闸的链路,其单个 fresh 窗口最多只有 trip 长,永远攒不够 2×trip。
    这正是想要的 —— 那种链路本来就没有驾驶价值,不如干脆不动。

    代价:`stale` 初值是 True,所以进程刚起来有 dwell 秒「对摇杆没反应」。
    可以接受 —— 那段时间 `armed` 还是 False(上电默认),而点「启用电机」
    本身就不止 2×300ms。**不要**为此加「首包立即恢复」的快路径:链路要是从
    一开始就是 400ms 那种病态间隔(页面在后台启动就是),它照样会抖。
    """

    def __init__(self, trip_s, dwell_s=None):
        self.trip_s = float(trip_s)
        # 默认 2×trip:不引入第二个魔法数,且顺手给出一条明确语义(见类注释)
        self.dwell_s = float(2.0 * self.trip_s if dwell_s is None else dwell_s)
        self.stale = True            # 上电即失联,直到真收到包并连续 fresh 够久
        self._fresh_since = None     # 连续 fresh 的起点;None = 当前不 fresh

    def update(self, age_s, now_s):
        """age_s: 距最后一包的秒数(None = 从没收到过)。返回是否失联。"""
        if age_s is None or age_s > self.trip_s:
            self._fresh_since = None       # 断流:连续 fresh 的计时作废
            self.stale = True
            return True
        if self._fresh_since is None:
            self._fresh_since = now_s      # 断流后的第一包,重新开始计时
        # ⚠️ 必须是 `if self.stale and ...` —— 只有**已经在跳闸态**才谈得上「恢复」。
        # 若顺手写成 `elif not self.stale and ...`,语义就反了:它只在已经 fresh 时
        # 把 stale 再置一次 False(空操作),而真正需要恢复的跳闸态**永远恢复不了**,
        # 车会彻底动不了。测试 test_deadman 的「60Hz 稳态必须在 2×trip 后恢复」钉这条。
        if self.stale and now_s - self._fresh_since >= self.dwell_s:
            self.stale = False             # 连续 fresh 满 dwell,才准恢复
        return self.stale


class IdlePark:
    """静止判据:指令长时间停在零点 → 建议 park 舵机(纯状态机,时间由调用方传入)。

    ⚠️ 这**不是**第二套链路门,别照着 Deadman 的样子当"舵机专用 deadman"用。
    `pwm_proc.py` 里那条「**不要**给舵机单独做一套滞回」的禁令针对的是**链路通断**:
    链路再加一层滞回就会出现「电机已经停了、舵机还咬着转角」—— 而两者共用同一个门,
    正是它们不拆成两个独立进程的全部理由。
    本类判的是**指令内容**(车既没在动、也没在转),与链路通不通无关;而且它的滞回是
    **不对称**的:
        进入 idle:要连续静 hold_s 秒  —— 免得行进中松一下手就把舵机放掉
        退出 idle:**第一个非零样本立刻退出** —— 绝不让舵机落后于指令
    恢复侧不设任何延迟,所以「舵机咬着转角不放」在结构上不可能出现。

    ── 为什么值得做 ───────────────────────────────────────────────
    不 park 时舵机一直被通电顶在 center_us 上,内部模拟控制环持续纠偏 ⇒ 嗡嗡响 +
    微抖 + 白吃电流,而它的 5V 和 L298N 逻辑、相机共用同一条排针轨(见 motor_wiring.md)。
    实测(`servo_pulse_probe.py`)已证明**我们发出的脉宽是干净的**(中位 1502µs、
    p1/p99 1500/1625、坏帧 0.4 次/秒),所以抖不来自脉宽信号 —— 能改善它的软件手段
    就只剩「不驱动时干脆不发脉冲」这一条。

    `hold_s <= 0` = 关闭(恒不 idle),留给 A/B 对照用。
    """

    def __init__(self, hold_s, eps=0.02):
        self.hold_s = float(hold_s)
        self.eps = abs(float(eps))
        self.idle = False            # 上电即非 idle:先按「还在驾驶」对待
        self._still_since = None     # 连续静止的起点;None = 当前不静止

    def update(self, l, a, now_s):
        """l/a: 当前归一化指令(±1)。返回是否建议 park。now_s 由调用方给。"""
        if self.hold_s <= 0.0:
            return False             # 关闭
        # eps 卡的就是「中位微抖」:摇杆回中后仍可能渗出千分之几的残余,那个量级
        # 既推不动舵机、又足以让它一直纠偏。用 max(|l|,|a|) 而不是模长 —— 单杆模式
        # 推满前进时 a 可能正好是 0,但那时车在动,l 不为零,**绝不能** park。
        # 边界按 Deadman 的惯例取闭区间:恰好等于 eps 算静止。
        if max(abs(l), abs(a)) > self.eps:
            self._still_since = None
            self.idle = False
            return False
        if self._still_since is None:
            self._still_since = now_s   # 动过之后的第一拍静止,重新开始计时
        if now_s - self._still_since >= self.hold_s:
            self.idle = True
        return self.idle
