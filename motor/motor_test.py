#!/usr/bin/env python3
"""最小电机测试 —— 不跑 ROS,直接驱动 GpioMotorHW。

这是**唯一能逐个轮子单独驱动**的工具:`/cmd_vel` 是差速指令,永远同时驱动两轮,
隔离不了单轮,所以方向出问题时必须用它定位。

⚠️ 跑之前必须把轮子架空(离地)。即使 20~25% 占空比,落地也会窜出去。

用法:
    /usr/bin/python3 ~/motor_test.py              # 编码器模式(需编码器已接线)
    /usr/bin/python3 ~/motor_test.py --observe    # 观察模式(不需要编码器)
    /usr/bin/python3 ~/motor_test.py 0.15         # 改占空比

两种模式:
  * 观察模式 —— 逐个轮子单独正转/反转,打印该看哪一侧,由**人眼**判定实际转向。
    编码器没接线时用这个,它直接给引脚原始极性,不掺 flip_left/flip_right。
  * 编码器模式 —— 靠编码器计数符号自动判定(正转应为正、反转应为负),一次同时
    验证「接线 + 电机 + 编码器」。

编码器模式判读(全部架空时):
  * 某轮正转计数为**正**、反转计数为**负**      → 该轮接线正确
  * 电机响但不转 或 计数为 0                  → 该轮 IN 或 EN 线没接好
  * 正反转计数同号(不反向)                    → 该轮一条 IN 线没接
  * 只有**单侧**转反                          → 该侧 IN 线序反,改对应的 flip_*
  * **整体**转向反(前后正常、左右转相反)      → 左右通道交叉,改 swap_lr(不是 flip_*)!
  * 解码线程报 died                           → event API 仍有问题,回来找我

⚠️⚠️ **通道 L 驱动的是【右】轮,通道 R 驱动的是【左】轮** —— 2026-09-15 实测定案,
  详见 motor_wiring.md「通道 ≠ 左右轮」和 motor_params.yaml 的 swap_lr 段。

  这个脚本按**通道**驱动(参数顺序就是通道 L / 通道 R),所以:
    * 提示语里的「看哪一侧」已经按 swap_lr 折算成**物理轮**,直接照着看即可;
    * 但 `flip_left`/`flip_right` 是**按通道**命名的 —— `flip_left` 作用在通道 L(右轮)上。
      回答判读题时务必按下面重述的对照填,**不要**按轮子的左右去填。
  本脚本此前把通道名当成了轮名,给出过系统性相反的结论 —— 已修正,别再按旧输出理解。
"""
import signal
import sys
import time

sys.path.insert(0, "/usr/lib/python3/dist-packages")
sys.path.insert(0, "/usr/local/lib/python3.10/dist-packages")

from motor_hw import GpioMotorHW


class Abort(Exception):
    pass


def _on_signal(signum, _frame):
    """把 SIGINT/SIGTERM/SIGHUP 转成异常,让 finally 里的 hw.stop() 一定跑到。

    否则进程一死 gpiod 释放 line → L298N 的 IN/ENA 悬空 → 电机可能停不下来。
    """
    raise Abort("signal %d" % signum)


for _s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    try:
        signal.signal(_s, _on_signal)
    except (ValueError, OSError):
        pass        # 非主线程或平台不支持时忽略

# 与 motor_params.yaml 一致
PWM_L, INA_L, INB_L = 91, 61, 62
PWM_R, INA_R, INB_R = 63, 60, 124
ENC_LA, ENC_LB = 0, 12
ENC_RA, ENC_RB = 5, 1

# 通道 → 物理轮。**必须与 motor_params.yaml 的 swap_lr 保持一致**:
#   swap_lr=true(当前定案) → 通道 L 接右轮,通道 R 接左轮。
# 这里不读 YAML(本脚本刻意无依赖、能在裸环境跑),所以改动 swap_lr 时**记得同步改这里**,
# 否则本脚本的提示语会指错轮子 —— 那正是它以前的毛病。
SWAP_LR = True
WHEEL_L = '右轮' if SWAP_LR else '左轮'
WHEEL_R = '左轮' if SWAP_LR else '右轮'
_TAG_L = '通道 L(%s) ' % WHEEL_L
_TAG_R = '通道 R(%s) ' % WHEEL_R

_args = [a for a in sys.argv[1:] if not a.startswith('--')]
_flags = {a for a in sys.argv[1:] if a.startswith('--')}
OBSERVE = '--observe' in _flags
DUTY = float(_args[0]) if _args else (0.20 if OBSERVE else 0.25)

DUR = 1.5           # 每个动作持续
PAUSE = 1.0         # 动作之间静止,留给人观察
LEAD = 1.2          # 开始转之前先停一下,让人读完提示

MODE = '观察模式(不需要编码器)' if OBSERVE else '编码器模式'
print("%s,占空比 %.0f%%,每个动作 %.1f 秒。轮子确认已架空。" % (MODE, DUTY * 100, DUR))
print("=" * 62)

hw = None
try:
    hw = GpioMotorHW('gpiochip0', PWM_L, INA_L, INB_L, PWM_R, INA_R, INB_R,
                     ENC_LA, ENC_LB, ENC_RA, ENC_RB, pwm_freq=1000.0,
                     enable_encoders=not OBSERVE)
    hw.start()
    time.sleep(0.3)         # 让解码线程稳定下来

    if OBSERVE:
        # 直接给引脚原始电平,不经过 flip_* —— 拿到的是纯接线极性 γ。
        # dir=True → INA=1/INB=0,dir=False → INA=0/INB=1(见 motor_hw.set_direction)。
        steps = [
            (_TAG_L + '正转', DUTY, 0.0, True, True,
             '看【%s】(通道 L)→ 应朝【前】转' % WHEEL_L),
            (_TAG_L + '反转', DUTY, 0.0, False, True,
             '看【%s】(通道 L)→ 应朝【后】转' % WHEEL_L),
            (_TAG_R + '正转', 0.0, DUTY, True, True,
             '看【%s】(通道 R)→ 应朝【前】转' % WHEEL_R),
            (_TAG_R + '反转', 0.0, DUTY, True, False,
             '看【%s】(通道 R)→ 应朝【后】转' % WHEEL_R),
        ]
        for label, dl, dr, dir_l, dir_r, hint in steps:
            print("\n>>> %s" % label)
            print("    %s" % hint)
            print("    等 3 秒后开始转 …")
            time.sleep(LEAD + 1.8)
            hw.set_direction(dir_l, dir_r)
            time.sleep(0.05)
            hw.set_duty(dl, dr)
            time.sleep(DUR)
            hw.set_duty(0.0, 0.0)
            time.sleep(PAUSE)
            print("    已停。")

        print("\n" + "=" * 62)
        print("=== 请回答这三件事 ===")
        print("  1. 上面『%s正转』时,%s 实际朝哪边?   前 / 后" % (_TAG_L, WHEEL_L))
        print("  2. 上面『%s正转』时,%s 实际朝哪边?   前 / 后" % (_TAG_R, WHEEL_R))
        print("  3. 同一侧正转和反转是否朝**同一个**方向?")
        print()
        print("  ⚠️ flip_left / flip_right 是按【通道】命名的,不是按轮子:")
        print("     flip_left  → 作用在**通道 L**,也就是【%s】" % WHEEL_L)
        print("     flip_right → 作用在**通道 R**,也就是【%s】" % WHEEL_R)
        print("     按轮子的左右去填会填反。请照下面按通道填:")
        print()
        print("     第 1 题朝【前】→ flip_left  = false ; 朝【后】→ flip_left  = true")
        print("     第 2 题朝【前】→ flip_right = false ; 朝【后】→ flip_right = true")
        print()
        print("  第 3 条若为『是』→ 该侧一条 IN 线没接,是接线问题,不是软件。")
        print("  某一侧单独看正常、但整车左右转是反的 → 那是 swap_lr 的事,翻 flip_* 没用。")
    else:
        def step(label, dl, dr, dir_l, dir_r):
            hw.set_direction(dir_l, dir_r)
            time.sleep(0.05)
            e0 = hw.read_encoders()
            hw.set_duty(dl, dr)
            time.sleep(DUR)
            hw.set_duty(0.0, 0.0)
            time.sleep(0.4)      # 等惯性停下再采样
            e1 = hw.read_encoders()
            dl_ = e1[0] - e0[0]
            dr_ = e1[1] - e0[1]
            print("  %-18s  编码器L %+7d   编码器R %+7d" % (label, dl_, dr_))
            return dl_, dr_

        print("\n--- %s (A / ENA, GPIO 91/61/62) ---" % _TAG_L.strip())
        LF = step(_TAG_L + '正转', DUTY, 0.0, True, True)
        LR = step(_TAG_L + '反转', DUTY, 0.0, False, True)

        print("\n--- %s (B / ENB, GPIO 63/60/124) ---" % _TAG_R.strip())
        RF = step(_TAG_R + '正转', 0.0, DUTY, True, True)
        RR = step(_TAG_R + '反转', 0.0, DUTY, True, False)

        print("\n--- 双轮同转(对照) ---")
        step('双轮 正转', DUTY, DUTY, True, True)

        print("\n" + "=" * 62)
        print("=== 判读 ===")
        # ⚠️ 下面把「通道 L 的 duty」和「编码器 L 的计数」配成一对,这是个**未经核对的
        #    假设**:编码器目前根本没接线,而且 GPIO 00/12 物理上装在哪个轮子上也从未验证过。
        #    真接上编码器之后,先做一次「手转某个轮、看哪一路计数在动」确认配对,
        #    否则这里的 OK/未反转 判读会把「配对错」误报成「接线错」。
        print("  (配对假设:编码器 L ↔ 通道 L,尚未核对 —— 编码器未接线)")
        print("  %s 正转 %+d / 反转 %+d  → %s" % (
            _TAG_L.strip(), LF[0], LR[0],
            'OK' if LF[0] > 0 > LR[0] else
            ('未反转(IN 线缺失?)' if LF[0] * LR[0] > 0 and LF[0] != 0 else '无输出')))
        print("  %s 正转 %+d / 反转 %+d  → %s" % (
            _TAG_R.strip(), RF[1], RR[1],
            'OK' if RF[1] > 0 > RR[1] else
            ('未反转(IN 线缺失?)' if RF[1] * RR[1] > 0 and RF[1] != 0 else '无输出')))

finally:
    if hw is not None:
        hw.stop()
    print("\n电机已停机。")
    # 解码线程若已死,计数不可信 —— 这是过去埋过的雷,必须显式报出来。
    # 观察模式下根本没建解码器(enc_* 为 None),跳过。
    if hw is not None:
        for enc in (hw.enc_l, hw.enc_r):
            if enc is not None and enc.failed:
                print("  !! 编码器 %s 解码线程已死: %s" % (enc.name, enc._error))
