#!/usr/bin/env python3
"""Headless unit tests for motor control math + fake hardware.

No ROS, no GPIO — run anywhere:  python3 ~/motor_driver_test.py
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from motor_control import (Deadman, IdlePark, Pid, joy_to_channels, map_channels,
                           twist_to_wheel_speeds, wheel_speeds_to_twist,
                           wheel_duties)
from motor_hw import quad_step, FakeMotorHW

WHEEL_BASE = 0.20


def approx(a, b, tol=1e-9):
    return abs(a - b) <= tol


def test_kinematics():
    # straight line
    vl, vr = twist_to_wheel_speeds(0.2, 0.0, WHEEL_BASE)
    assert approx(vl, 0.2) and approx(vr, 0.2), (vl, vr)
    # pure rotation: right +, left −
    vl, vr = twist_to_wheel_speeds(0.0, 1.0, WHEEL_BASE)
    assert approx(vl, -0.1, 1e-9) and approx(vr, 0.1, 1e-9), (vl, vr)
    # inverse round-trip
    v, w = wheel_speeds_to_twist(0.2, 0.2, WHEEL_BASE)
    assert approx(v, 0.2) and approx(w, 0.0)
    print('  kinematics OK')


def test_pid():
    p = Pid(kp=1.0, ki=0.5, kd=0.05, i_limit=0.3, out_limit=0.9)
    # zero error → zero output
    assert approx(p.update(0.0, 0.02), 0.0)
    # positive error → positive bounded output
    for _ in range(100):
        out = p.update(1.0, 0.02)
        assert 0.0 < out <= 0.9, out
    # integral accumulates but is clamped
    assert p.integral <= 0.3 + 1e-9
    # 抗饱和:输出已达上限且误差同向推时,积分冻结不再增长
    p.reset()
    for _ in range(50):
        p.update(1.0, 0.02)
    frozen = p.integral
    for _ in range(50):
        p.update(1.0, 0.02)
    assert approx(p.integral, frozen), (frozen, p.integral)
    assert frozen < 0.3, frozen            # 根本没机会顶到 i_limit
    # dt=0 → 0 and does not crash
    p.reset()
    assert approx(p.update(0.5, 0.0), 0.0)
    p.reset()
    assert approx(p.integral, 0.0) and approx(p.prev_err, 0.0)
    print('  pid OK')


def test_wheel_duties():
    """目标轮速 → 占空比。含「摇杆松手电机不停」的回归用例。"""
    common = dict(ff=1.0, max_lin=0.3, max_duty=0.9, deadband=0.05)

    # --- 开环(编码器未接):占空比与摇杆量线性对应 ---
    d_l, d_r = wheel_duties(0.15, 0.15, 0.0, 0.0, 0.02,
                            use_encoders=False, **common)
    assert approx(d_l, 0.5, 1e-6) and approx(d_r, 0.5, 1e-6), (d_l, d_r)

    # 满量程被 max_duty 钳制,不会打到 1.0
    d_l, d_r = wheel_duties(0.3, 0.3, 0.0, 0.0, 0.02,
                            use_encoders=False, **common)
    assert approx(d_l, 0.9, 1e-6) and approx(d_r, 0.9, 1e-6), (d_l, d_r)

    # 纯旋转:两轮同幅反向 → 占空比相同(方向由 _dir_from_speed 另设)
    d_l, d_r = wheel_duties(-0.1, 0.1, 0.0, 0.0, 0.02,
                            use_encoders=False, **common)
    assert d_l > 0.0 and approx(d_l, d_r, 1e-6), (d_l, d_r)

    # 死区以下的指令不输出
    d_l, d_r = wheel_duties(0.01, 0.01, 0.0, 0.0, 0.02,
                            use_encoders=False, **common)
    assert d_l == 0.0 and d_r == 0.0, (d_l, d_r)

    # --- 回归:闭环积分饱和后松手,一条指令内必须归零 ---
    # 修复前这里会残留 ki*integral ≈ 0.15(> deadband 0.05),表现为松手后
    # 电机继续慢转不停;test_pid() 只测了「全新 PID + 零误差」,拦不住。
    pl = Pid(kp=1.0, ki=0.5, kd=0.05, i_limit=0.3, out_limit=0.9)
    pr = Pid(kp=1.0, ki=0.5, kd=0.05, i_limit=0.3, out_limit=0.9)
    pid = dict(pid_l=pl, pid_r=pr, use_encoders=True)
    for _ in range(200):                    # 一直顶满速 → PID 充分饱和
        d_l, d_r = wheel_duties(0.3, 0.3, 0.0, 0.0, 0.02, **pid, **common)
    assert d_l >= 0.9 - 1e-9, d_l           # 确认确实处在饱和区

    d_l, d_r = wheel_duties(0.0, 0.0, 0.0, 0.0, 0.02, **pid, **common)
    assert d_l == 0.0 and d_r == 0.0, (d_l, d_r)
    assert approx(pl.integral, 0.0) and approx(pr.integral, 0.0)

    # 松手后重新轻推:不应因残留积分而被顶到满占空比
    d_l, d_r = wheel_duties(0.1, 0.1, 0.0, 0.0, 0.02, **pid, **common)
    assert d_l < 0.9 - 1e-9, d_l
    print('  wheel_duties OK')


def test_map_channels():
    """通道交叉(swap_lr)的回归 —— 复现「前进对、左右转反」并验证修复。

    这是唯一能区分 swap_lr 和 flip_* 的用例:两者都能「让车转向反」,
    但 flip_* 会同时把前进/后退也翻掉(整机镜像),swap_lr 只翻转向。
    真机上曾经因为分不清这两者,来回翻 flip 翻错了两轮。
    """
    L = 0.20
    v, w = 0.24, 0.60
    t_l, t_r = twist_to_wheel_speeds(v, w, L)

    def realized(cl, cr):
        """交叉接线下,通道 (L,R) 实际驱动的物理轮是 (右,左) → 换回来。"""
        return cr, cl

    # swap_lr=False:直行分量保住、转向被翻 —— 正是用户报的「前进对、左右转反」
    cl, cr = map_channels(t_l, t_r, False)
    rv, rw = wheel_speeds_to_twist(*realized(cl, cr), L)
    assert approx(rv, v, 1e-9), rv
    assert approx(rw, -w, 1e-9), rw

    # swap_lr=True:两者都对(这才是修好的状态)
    cl, cr = map_channels(t_l, t_r, True)
    rv, rw = wheel_speeds_to_twist(*realized(cl, cr), L)
    assert approx(rv, v, 1e-9), rv
    assert approx(rw, w, 1e-9), rw

    # 不交叉接线时,swap_lr=true 反而会把转向搞反 —— 别乱开
    rv, rw = wheel_speeds_to_twist(*map_channels(t_l, t_r, True), L)
    assert approx(rv, v, 1e-9) and approx(rw, -w, 1e-9), (rv, rw)

    # 自逆:里程计侧用同一个函数换回物理系,必须能换回去
    assert map_channels(*map_channels(1.0, 2.0, True), True) == (1.0, 2.0)
    assert map_channels(1.0, 2.0, False) == (1.0, 2.0)
    print('  map_channels OK')


def test_joy_to_channels():
    """归一化摇杆 → 通道系轮速。**核心断言:duty 严格等于摇杆比值。**

    这是「直连 PWM 通道」的契约(pwm_proc 直接拿 |返回值| 当占空比)。用户明确
    要求摇杆推多少 duty 就是多少,中间不许有斜率限幅/滤波 —— 所以这里断言的是
    等号,不是「差不多」。
    """
    ML, MA, L = 0.3, 1.0, 0.20

    # 直行:两轮都恰好等于摇杆比值本身
    for joy in (0.2, 0.5, 0.8, 1.0):
        s_l, s_r = joy_to_channels(joy, 0.0, ML, MA, L)
        assert approx(s_l, joy) and approx(s_r, joy), (joy, s_l, s_r)

    # 后退同理:负号保留(方向由调用方按符号设引脚,占空比恒非负)
    s_l, s_r = joy_to_channels(-0.6, 0.0, ML, MA, L)
    assert approx(s_l, -0.6) and approx(s_r, -0.6), (s_l, s_r)

    # 摇杆回中 → 严格 0,不许有残留(这正是「松手不停」那类 bug 的形态)
    s_l, s_r = joy_to_channels(0.0, 0.0, ML, MA, L)
    assert s_l == 0.0 and s_r == 0.0, (s_l, s_r)

    # 纯旋转:两轮等幅反向 → 占空比相同
    s_l, s_r = joy_to_channels(0.0, 1.0, ML, MA, L)
    assert s_l < 0.0 < s_r, (s_l, s_r)
    assert approx(abs(s_l), abs(s_r)), (s_l, s_r)

    # 角度项系数 k = max_ang·L/2/max_lin —— 决定满舵时内轮的速度
    k = MA * L / 2.0 / ML
    s_l, s_r = joy_to_channels(0.0, 1.0, ML, MA, L)
    assert approx(abs(s_l), k), (abs(s_l), k)

    # swap_lr 只换左右通道:直行分量原样保住,转向被翻
    a_l, a_r = joy_to_channels(0.5, 0.3, ML, MA, L)
    b_l, b_r = joy_to_channels(0.5, 0.3, ML, MA, L, swap_lr=True)
    assert (b_l, b_r) == (a_r, a_l), ((a_l, a_r), (b_l, b_r))

    # 与 ROS 那条路(/cmd_vel → twist_to_wheel_speeds)必须得出同一组轮速,
    # 否则同一个摇杆量在两条链路上手感不一致 —— 那是最难查的一类问题。
    v, w = 0.5 * ML, 0.3 * MA
    t_l, t_r = twist_to_wheel_speeds(v, w, L)
    s_l, s_r = joy_to_channels(0.5, 0.3, ML, MA, L)
    assert approx(s_l, t_l / ML) and approx(s_r, t_r / ML), (s_l, s_r)
    print('  joy_to_channels OK')


def test_quadrature():
    # Forward rotation: A leads B → counts increase (state sequence
    # 00→10→11→01→00 with A as MSB and the QUAD convention).
    state = 0
    seq = [(1, 0), (1, 1), (0, 1), (0, 0)]   # one full forward cycle
    total = 0
    for a, b in seq:
        d, state = quad_step(state, a, b)
        total += d
    assert total == 4, total                 # 4x resolution per cycle
    assert state == 0
    # Backward rotation: reverse path 0→1→3→2→0 → counts decrease by 4
    state = 0
    total = 0
    for a, b in [(0, 1), (1, 1), (1, 0), (0, 0)]:
        d, state = quad_step(state, a, b)
        total += d
    assert total == -4, total
    # Repeated same state → no count
    d, _ = quad_step(0, 0, 0)
    assert d == 0
    print('  quadrature OK')


def test_fake_hw():
    fw = FakeMotorHW(max_linear_speed=0.3, counts_per_meter=1000.0,
                     wheel_base=WHEEL_BASE)
    fw.start()
    fw.set_direction(True, True)
    fw.set_duty(1.0, 0.0)
    time.sleep(0.05)
    c_l, c_r = fw.read_encoders()
    assert c_l > 0, c_l        # left wheel advanced
    assert c_r == 0, c_r       # right stopped
    # reverse direction → counts decrease
    time.sleep(0.01)
    fw.set_direction(False, False)
    fw.set_duty(1.0, 1.0)
    time.sleep(0.05)
    c_l2, c_r2 = fw.read_encoders()
    assert c_l2 < c_l, (c_l2, c_l)
    fw.stop()
    print('  fake_hw OK')


def test_deadman():
    """pwm_proc 的链路失联判据(带滞回)。

    这是 pwm_proc 那侧**唯一**的 headless 测试 —— 那个文件硬编 GpioMotorHW /
    SoftServoPwm 且没有 mock 开关,整体没法单测,所以判据被抽到 motor_control
    里来。第 3 条是这个 bug 的回归测试,别删。
    """
    TRIP = 0.300
    DT = 0.005                 # pwm_proc 主循环 200Hz ⇒ 5ms 一拍

    # 1. 上电即失联:从没收到过包
    d = Deadman(TRIP)
    assert d.update(None, 0.0) is True, '从没收到过包就必须算失联'
    assert d.update(None, 10.0) is True, '一直没包 → 一直失联'

    # 2. 60Hz 稳态:连续 fresh 满 2×trip 才准恢复
    d = Deadman(TRIP)
    assert d.dwell_s == 2 * TRIP, d.dwell_s
    assert d.update(None, 0.0) is True
    t = 0.0
    # _fresh_since 会落在**第一包**上(t=0.016),所以恢复点是 0.016+0.600=0.616。
    # 这里只在两侧取点(0.59 仍失联 / 0.62 已恢复),不去卡那个浮点边界 ——
    # 精确边界由下面第 5 条用可控的 age 来钉。
    for _ in range(36):                       # 到 t=0.576
        t += 0.016
        assert d.update(0.016, t) is True, ('%.3fs 就恢复了,dwell 形同虚设' % t)
    t = 0.590
    assert d.update(0.016, t) is True, '离 2×trip 还差一点就不该恢复'
    t = 0.620
    assert d.update(0.016, t) is False, '连续 fresh 满 2×trip 必须恢复'
    t = 0.636
    assert d.update(0.016, t) is False, '恢复之后要保持恢复'

    # 3. ★★ 回归测试:后台标签页被节流到 1Hz ★★
    #    60Hz 一切正常 → 页面被切到后台,包间隔变成 400ms(> deadman 300ms)。
    #    裸阈值实现在这里会「来一包就恢复、300ms 后又跳闸」反复不停:实测 61 次
    #    严格交替的「中断/恢复」;而舵机与电机走**同一个门**,舵机就跟着
    #    park(不发脉冲、松掉)↔ 全速转回中位,**来回抖** —— 用户报的现象。
    #    滞回之后:跳闸一次,此后**永远不恢复**(单个 fresh 窗口只有 300ms,
    #    攒不够 600ms),所以机械上不可能再抖。
    d = Deadman(TRIP)
    t = 0.0
    age = 0.0
    for _ in range(int(1.0 / DT)):            # 先跑 1 秒正常 60Hz
        t += DT
        age = (age + DT) % 0.016
        d.update(age, t)
    assert d.stale is False, '60Hz 连续 1s 都没恢复?'

    # 切过去之后的那第一个 300ms 窗口**本来就该是 fresh 的** —— 车在这段时间里
    # 照常开着,跳闸要等龄期真的越过 300ms。所以不能笼统地数 fresh 拍数,要卡住
    # 真正的不变量:**跳闸之后,再也不许有一拍是 fresh**。
    tripped = False
    fresh_after_trip = 0
    flips = 0
    prev = d.stale
    for _ in range(int(10.0 / DT)):           # 再跑 10 秒的 1Hz
        t += DT
        age += DT
        if age >= 0.400:                      # 新包到达,龄期清零
            age = 0.0
        if d.update(age, t):
            tripped = True
        elif tripped:
            fresh_after_trip += 1
        if d.stale != prev:
            flips += 1
            prev = d.stale
    assert tripped, '400ms 的包间隔居然没跳闸?'
    assert flips == 1, ('应恰好跳闸 **一次**,之后不再翻转;实际 %d 次 —— '
                        '>1 就是还在 chattering,舵机会跟着抖' % flips)
    assert fresh_after_trip == 0, ('跳闸之后还有 %d 拍是 fresh ⇒ 又放开了,'
                                   '舵机会跟着抖一遍' % fresh_after_trip)

    # 4. 正常链路打一个洞:跳闸 → 链路恢复后仍须重新攒够 2×trip
    d = Deadman(TRIP)
    t = 0.0
    age = 0.0
    for _ in range(int(1.0 / DT)):
        t += DT
        age = (age + DT) % 0.016
        d.update(age, t)
    assert d.stale is False
    while age < 0.500:                        # 500ms 的洞
        t += DT
        age += DT
        d.update(age, t)
    assert d.stale is True, '500ms 的洞必须跳闸'

    age = 0.0                                 # 洞结束,链路恢复正常
    t0 = t
    early = 0
    while t - t0 < 0.590:
        t += DT
        age = (age + DT) % 0.016
        if d.update(age, t) is False:
            early += 1
    assert early == 0, '洞之后不到 2×trip 就恢复 ⇒ dwell 没生效'
    while t - t0 < 0.700:
        t += DT
        age = (age + DT) % 0.016
        d.update(age, t)
    assert d.stale is False, '洞之后链路正常了,连续 fresh 满 2×trip 必须恢复'

    # 5. 边界:跳闸阈值本身**没动**(还是 300ms)—— 滞回只作用在恢复侧
    d = Deadman(TRIP)
    t = 0.0
    for _ in range(int(1.0 / DT)):
        t += DT
        d.update(DT, t)
    assert d.stale is False
    assert d.update(TRIP, t) is False, '龄期恰好等于 trip 不算失联(边界闭合)'
    assert d.update(TRIP + 1e-6, t) is True, '略超 trip 必须跳闸'

    print('  deadman OK')


def test_idle_park():
    """静止 park 的判据(与 test_deadman 同源第二个纯状态机)。

    和 deadman 是**两回事**:死手判的是「链路还在不在」,这个判的是「链路好着、
    但指令一直是零」。所以这里所有用例都假定链路正常(不打 age 参数),
    只喂 l/a。
    """
    HOLD = 1.000
    DT = 0.005                 # 与 pwm_proc 主循环同拍

    # 1. 上电即静止,但**要等满 hold_s** 才 park —— 免得行进中松一下手就放掉
    p = IdlePark(HOLD)
    t = 0.0
    assert p.update(0.0, 0.0, t) is False, '刚上来不许立刻 park'
    while t < HOLD - DT:
        t += DT
        assert p.update(0.0, 0.0, t) is False, ('%.3fs 就 park 了,没等满 %.3fs'
                                                % (t, HOLD))
    t = HOLD
    assert p.update(0.0, 0.0, t) is True, '连续静满 hold_s 必须 park(边界闭合)'

    # 2. ★ 亚 eps 噪声不算活动 —— 这正是要掐掉的东西。
    #    摇杆回中后指尖渗出的千分之几残余(中位微抖)既推不动舵机,又足以让它
    #    一直纠偏;要是把它算成「在驾驶」,这个 park 就永远不会触发。
    p = IdlePark(HOLD)
    t = 0.0
    p.update(0.0, 0.0, t)                         # 锚点:_still_since 落在第一拍上
    for i in range(int((HOLD + 0.5) / DT)):
        t += DT
        jitter = 0.019 if i % 2 else -0.019       # |·| < eps=0.02
        assert p.update(jitter, -jitter, t) is (t >= HOLD), \
            ('亚 eps 噪声被当成了活动 @%.3fs' % t)
    assert p.idle is True, '一路亚 eps 噪声应该照常进 idle'

    # 3. ★ 任意非零输入**立刻**恢复 —— 恢复侧不许有任何延迟,否则就是
    #    「电机停了、舵机还咬着转角」那个禁止的形态。
    #    单独用一个方向不为零的样本,只为证明恢复不看「静了多久」。
    assert p.update(0.5, 0.0, t) is False, '油门非零必须立刻退出 idle'
    assert p.idle is False, '退出 idle 必须立刻,不许延迟'
    assert p.update(0.0, -0.5, t + DT) is False, '只有 a 非零也算在转向'
    assert p.update(0.021, 0.0, t + 2 * DT) is False, '刚过 eps 就退出'
    assert p.update(0.0, 0.0, t + 3 * DT) is False, '退出后要**重新**静满 hold_s'

    # 3b. 恢复之后计时器确实是从头开始的:再静满一个 hold_s 才 park
    t2 = t + 3 * DT
    assert p.update(0.0, 0.0, t2 + HOLD - DT) is False, '恢复后不到 hold_s 就 park 了'
    assert p.update(0.0, 0.0, t2 + HOLD) is True, '重新静满 hold_s 必须再次 park'

    # 4. 单杆推满前进:a 可能正好是 0,但那时车在动 —— **绝不能** park
    p = IdlePark(HOLD)
    t = 0.0
    for _ in range(int((HOLD + 0.5) / DT)):
        t += DT
        assert p.update(1.0, 0.0, t) is False, 'l=1、a=0 是「直行」不是「静止」'
    assert p.idle is False

    # 5. hold_s <= 0 = 关闭(恒不 park),留给 A/B 对照
    p = IdlePark(0.0)
    t = 0.0
    for _ in range(400):
        t += DT
        assert p.update(0.0, 0.0, t) is False, 'hold_s=0 必须恒不 park'
    assert p.idle is False
    assert IdlePark(-1.0).update(0.0, 0.0, 99.0) is False, '负值同样 = 关闭'

    print('  idle_park OK')


def main():
    print('motor_driver_test:')
    test_kinematics()
    test_pid()
    test_wheel_duties()
    test_map_channels()
    test_joy_to_channels()
    test_quadrature()
    test_fake_hw()
    test_deadman()
    test_idle_park()
    print('\n[OK] All tests passed')


if __name__ == '__main__':
    main()
