#!/usr/bin/env python3
"""摇杆注入器 — 绕开浏览器,直接向 pwm_proc 的 UDP 口灌摇杆值。

**为什么需要它**:网页 → SocketIO → slam_viz → UDP → pwm_proc 这条链路上有
四个可能出错的环节(浏览器定时器节流 / SocketIO 传输降级 / 服务端仲裁 / UDP),
出问题时无法判断是哪一段。这个脚本只做最后一段,于是:

  * 注入能动、网页不能动  → 问题在浏览器链路(往上游查)
  * 注入也不能动          → 问题在 pwm_proc/GPIO/电机(往下游查,且网页白调)

**它看不到物理世界。** 日志只能告诉你「下发了什么 duty / 方向位」,轮子实际
转了没有、往哪转,必须由人盯着看。所以试转流程刻意做成**分段停、逐段确认**,
而不是一次跑完靠事后回忆。

⚠️ 这个脚本会给 GPIO 真实加电。跑之前确认:① 电机驱动已按预期通电(或明确
   知道它没通电),② 驱动轮已架空。别让车在桌上突然起步。

用法:
  python3 joy_inject.py --arm --sweep --wheel L     # 单轮启动阈值扫描(先跑这个)
  python3 joy_inject.py --arm --servo-sweep         # 舵机扫描(找中位/限位/方向,电机不动)
  python3 joy_inject.py --arm --wheel L --l ...     # 单轮隔离
  python3 joy_inject.py --arm --secs 20 --l 0.25    # 恒定前进
  python3 joy_inject.py --arm --scenario            # 全时序(分段停,需人工确认)
  python3 joy_inject.py                             # 只发 armed=false(安全)
"""
import argparse
import json
import os
import socket
import sys
import time

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5813
DEFAULT_HZ = 60.0

# 每个姿势段 6 秒:pwm_proc 每 5 秒打一行状态,段长必须大于它,
# 否则一行都落不进段内,事后无法逐段对照。
PHASE_SECS = 6.0

# --servo-sweep 收尾那段「松掉」的时长。比 PHASE_SECS 长,因为它是靠**耳朵**
# 判读的:要留够时间听清「松掉之后是不是还在抖」。
PARK_SECS = 10.0

# 与 motor_params.yaml 同源。不硬编死是为了避免「改了 YAML 忘了改脚本」——
# 这份文件和 slam_viz.py 各有一份拷贝,已经是一处隐患,不再加第三份。
_PARAM_DEFAULTS = {"max_linear_speed": 0.3, "max_angular_speed": 1.0,
                   "wheel_base": 0.20}
# 舵机参数的代码默认值,与 motor_hw.SoftServoPwm.__init__ 的形参默认一致。
# 只用于 --servo-sweep 打印「这一档会发给舵机多少 µs」,好和状态行的 servo= 对上。
_SERVO_DEFAULTS = {"servo_center_us": 1500.0, "servo_range_us": 500.0}
_YAML_CANDIDATES = ("motor_params.yaml", "/home/aidlux/motor_params.yaml")


def load_geometry():
    """从 motor_params.yaml 读几何参数 + swap_lr + steer_mode + servo_*,失败则用默认值。

    只读几个键,不值得为它引入 yaml 依赖失败即崩的硬约束——注入器是用来
    排查问题的,不能因为它自己起不来而挡住排查。

    **swap_lr 必须读**,不能硬编:它决定通道 L 对应哪个物理轮,而下面
    channel_axis() 的单轮隔离公式依赖这个方向。硬编的话,一旦有人改了
    swap_lr,注入器仍会按旧方向算 —— 于是"只驱动通道 L"实际变成了驱动
    通道 R,而脚本还言之凿凿地打印"通道 L"。这种静默失真比脚本直接报错
    危险得多,因为它会让人得出错误的标定结论。

    **steer_mode 同理,而且是更硬的一条**:steer_mode=servo 时 pwm_proc 把
    a 完全丢给舵机、轮子只吃 l,于是 channel_axis() 那套「用 a 造出通道差」
    的反解**根本不存在解** —— 任何 (l,a) 都只能让两轮等速。若还按老公式算,
    `--wheel L` 会实际同时驱动**两个**轮子(各 half duty),而脚本照样打印
    「仅左轮」。这比 swap_lr 那条更糟:它不是方向反了,是**隔离本身失效**。
    """
    try:
        import yaml
    except ImportError:
        return (_servo_defaults(), dict(_PARAM_DEFAULTS), True, "diff",
                "默认值(pyyaml 不可用)")
    for path in _YAML_CANDIDATES:
        if not os.path.exists(path):
            continue
        try:
            with open(path) as f:
                p = yaml.safe_load(f)["motor_driver"]["ros__parameters"]
        except Exception as e:                                  # noqa: BLE001
            return (_servo_defaults(), dict(_PARAM_DEFAULTS), True, "diff",
                    "默认值(解析 %s 失败: %s)" % (path, e))
        geo = {k: float(p.get(k, v)) for k, v in _PARAM_DEFAULTS.items()}
        # 默认 'diff' 与 pwm_proc 的代码默认一致:老 YAML 没有这个键时,
        # 注入器和被注入者必须对同一个文件得出同一个结论。
        return (_servo_defaults(p), geo, bool(p.get("swap_lr", True)),
                str(p.get("steer_mode", "diff")).strip().lower(), path)
    return (_servo_defaults(), dict(_PARAM_DEFAULTS), True, "diff",
            "默认值(没找到 motor_params.yaml)")


def _servo_defaults(p=None):
    """舵机参数(center/range/enable)。给 p=None 就返回代码默认值。

    `servo_enable=false` 时 pwm_proc **连 servo 对象都不构造**,于是舵机恒 park、
    一个脉冲都不发 —— 这时 --servo-sweep 扫出来的「舵机纹丝不动」不是舵机坏,
    是根本没在驱动它。所以 enable 必须一并读出来用于警告。
    """
    d = {k: float(v) for k, v in _SERVO_DEFAULTS.items()}
    d["servo_enable"] = False
    if p is None:
        return d
    for k in _SERVO_DEFAULTS:
        try:
            d[k] = float(p.get(k, _SERVO_DEFAULTS[k]))
        except (TypeError, ValueError):
            pass
    d["servo_enable"] = bool(p.get("servo_enable", False))
    return d


SERVO, GEO, SWAP_LR, STEER_MODE, GEO_SRC = load_geometry()

# steer_mode=servo 时 a 到不了轮子 → 通道差造不出来 → 单轮隔离无解。
# 见 load_geometry() 的 docstring:`--wheel L` 在那种模式下会**同时驱动两轮**
# 却仍打印「仅左轮」,是静默失真。所以宁可拒绝。
_WHEELS_GET_A = STEER_MODE in ("diff", "both")
# a 到底有没有到舵机。与 _WHEELS_GET_A 是同一枚举的两个投影,不是「叠加」。
_A_TO_SERVO = STEER_MODE in ("servo", "both")
# 通道映射系数。见 motor_control.joy_to_channels + map_channels:
#   channel_L = l + s*a*k    channel_R = l - s*a*k
# 其中 s = +1 (swap_lr=True,通道L接右轮) / -1 (swap_lr=False,通道L接左轮)。
# k 就是「转向折算成通道差的比率」,也是为什么满舵转向 duty 只有
# max_ang*wheel_base/2/max_lin = 0.333,而满舵直行是 1.0。
K = GEO["max_angular_speed"] * GEO["wheel_base"] / (2.0 * GEO["max_linear_speed"])
_S = 1.0 if SWAP_LR else -1.0

# 通道 → 物理轮(由 swap_lr 决定)。日志里一律写"左轮/右轮"而不是"通道L/R":
# 这两者恰是调试时最容易混淆的一对,写清楚能省掉一轮来回。
_WHEEL_NAME = {"L": "右轮" if SWAP_LR else "左轮",
               "R": "左轮" if SWAP_LR else "右轮"}


def channel_axis(wheel, x):
    """给定期望的通道值 x,反解出 (l, a) 使**只有一个通道**出力。

    wheel='L' → channel_L=x, channel_R=0;'R' 反之;'both' → 两通道都 = x。
    单轮隔离是硬件试转的关键:两轮同时转时,「哪个轮子对应哪个通道」只能靠
    猜;单独驱动一个通道,答案就是唯一确定的。

    ⚠️ 这只在 a 能到达轮子时成立。steer_mode=servo 时差速项恒为 0,任何 (l,a)
    都只能让 channel_L == channel_R,**单轮隔离无解** —— 那就直接报错,不要
    返回一个看起来能用、实际把两轮一起驱动的 (l,a)。
    """
    if wheel != "both" and not _WHEELS_GET_A:
        raise ValueError(
            "steer_mode=%s 时 a 只给舵机,轮子两通道恒等 —— 无法单轮隔离。\n"
            "  想想用 motor_test.py(直接驱动 GPIO,按通道给),但它和 pwm_proc\n"
            "  抢同一批 line,跑之前要先停掉 pwm_proc。\n"
            "  只想验电机链路不坏:用 --l 让两轮一起转(那仍然有效)。"
            % STEER_MODE)
    if wheel == "both":
        return x, 0.0                      # channel_L = channel_R = x(与 s 无关)
    half = x / 2.0
    a = x / (2.0 * _S * K)
    # channel_L = l + s*a*k:'L' 取 +a,'R' 取 -a。
    return (half, a) if wheel == "L" else (half, -a)


def send(sock, addr, l, a, armed):
    sock.sendto(json.dumps({"l": float(l), "a": float(a),
                            "armed": bool(armed)}).encode(), addr)


def run_phases(sock, addr, phases, hz):
    """按绝对时基跑完 phases=[(名字, l, a, 秒, armed)]。返回发包总数。

    armed 是**每段**的,不是全程一个。--servo-sweep 的最后一段就是靠它来
    真正松掉舵机的(armed=false ⇒ pwm_proc 走 park 分支 ⇒ 一个脉冲都不发),
    好让你用耳朵分辨「舵机顶着中位时的抖」和「舵机松掉时的抖」。全程一个
    armed 就做不出这个对照。
    """
    period = 1.0 / hz
    sent = 0
    for name, l, a, secs, armed in phases:
        print("── %s  l=%+.3f a=%+.3f  armed=%s  %gs  t=%s"
              % (name, l, a, armed, secs, time.strftime("%H:%M:%S")), flush=True)
        deadline = time.monotonic() + secs
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            send(sock, addr, l, a, armed)
            sent += 1
            # 绝对时基 + 过载重对齐,和 pwm_proc 主循环同一套写法:
            # 用 sleep(period - elapsed) 会累积漂移,长时序下段边界会错开。
            slp = now + period - time.monotonic()
            if slp > 0:
                time.sleep(slp)
    return sent


def build_servo_sweep(step, secs, park_secs, armed):
    """转向指令 a 从 -1 扫到 +1,再回中,最后**松掉**。

    l 恒为 0 ⇒ 电机全程不励磁,车一步都不会动。这是它安全的全部理由:
    单看 a 就能不碰轮子地单独驱动舵机。

    一次跑完回答三个一直悬着的问题:
      * **机械可达范围** —— 有哪几档舵机是顶住不动/嗡嗡响的?那就是撞限位了。
        中位偏得离谱时,连 a=0 那一档都会顶住 → 那症状正是「一直在抖」。
      * **中位** —— 哪一档它最安静。安静的那一档就是 servo_center_us 该取的值
        (1500µs 只是代码默认,不是机械中位)。
      * **方向** —— a 变大时舵机往哪边偏 → 定 servo_invert。别照名字猜。

    最后一段 armed=false 是**刻意**的对照:那一瞬间 pwm_proc 走 park 分支、
    一个脉冲都不发、舵机松掉。若松掉之后它**照样**抖,那抖就与我们的信号和
    我们的驱动电流无关,只能往供电/共地/机械上找 —— 这一条对照能把排查
    方向直接劈成两半。
    """
    phases, x = [], -1.0
    while x <= 1.0 + 1e-9:
        phases.append(("舵机 a=%+.2f → %+.0fµs" % (x, _servo_us(x)), 0.0, x, secs, armed))
        x = round(x + step, 4)
    phases.append(("舵机 a=0 回中", 0.0, 0.0, secs, armed))
    phases.append(("★ 松掉(park,不发脉冲)", 0.0, 0.0, park_secs, False))
    return phases


def _servo_us(x):
    """这一档 a 会发给舵机的脉宽,好和状态行的 servo=NNNNus 直接对上。"""
    return SERVO["servo_center_us"] + x * SERVO["servo_range_us"]


def build_sweep(wheel, dmin, dmax, dstep, secs, armed):
    """启动阈值扫描:逐档升 duty,找出轮子真正开始转的那一档。

    为什么这是最该先测的一件事:齿轮减速电机 + H 桥经常在低 duty 下只嗡嗡响
    不转。若真是这样,摇杆底部一大段行程就是死区 —— 这比控制环掉到 70Hz
    更能解释「不跟手」。motor_params.yaml 的 pid_deadband=0.05 说明这个阈值
    从来没实测过。
    """
    who = _WHEEL_NAME.get(wheel, "双轮")
    phases, d = [], dmin
    while d <= dmax + 1e-9:
        l, a = channel_axis(wheel, d)
        phases.append(("duty %.2f (仅%s)" % (d, who), l, a, secs, armed))
        d = round(d + dstep, 4)
    phases.append(("停止", 0.0, 0.0, 3.0, armed))
    return phases


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ,
                    help="发包率,必须远高于 deadman(300ms),否则误触发")
    ap.add_argument("--secs", type=float, default=PHASE_SECS)
    ap.add_argument("--l", type=float, default=0.0, help="归一化前进(+)/后退(-)")
    ap.add_argument("--a", type=float, default=0.0, help="归一化左转(+)/右转(-)")
    ap.add_argument("--arm", action="store_true",
                    help="置 armed=true。不加则全程 armed=false(电机绝不动)")
    ap.add_argument("--wheel", choices=("both", "L", "R"), default="both",
                    help="只驱动指定通道(单轮隔离);配合 --l/--a 时忽略")
    ap.add_argument("--scenario", action="store_true", help="跑标准试转时序")
    ap.add_argument("--sweep", action="store_true", help="启动阈值扫描")
    ap.add_argument("--servo-sweep", action="store_true",
                    help="舵机扫描:a 从 -1 到 +1 逐档停,最后松掉。l 恒 0,电机不动")
    ap.add_argument("--astep", type=float, default=0.25,
                    help="--servo-sweep 的 a 步长")
    ap.add_argument("--dmin", type=float, default=0.10)
    ap.add_argument("--dmax", type=float, default=0.40)
    ap.add_argument("--dstep", type=float, default=0.05)
    args = ap.parse_args()

    if args.hz <= 3.3:
        sys.exit("发包率必须 > 3.3Hz,否则 deadman(300ms)会把它当成断流")

    # --wheel 若与显式 --l/--a 同时给出会互相矛盾,以显式 l/a 为准并提示。
    explicit = args.l != 0.0 or args.a != 0.0

    # --servo-sweep 的两个前置条件。两条都是「否则你会得出一个错误结论」:
    if args.servo_sweep and not _A_TO_SERVO:
        sys.exit("[注入] --servo-sweep 需要 steer_mode=servo|both,当前 steer_mode=%s。\n"
                 "  该模式下 a **完全不给舵机**(舵机恒 park),而 a 会进差速混控\n"
                 "  —— 也就是说这个扫描会去转轮子,却一步都不动舵机。" % STEER_MODE)
    if args.servo_sweep and not args.arm:
        # 不加 --arm 时每一段都是 armed=false ⇒ 舵机全程 park、一个脉冲不发。
        # 扫完得到的「它一动不动」是假的,而且是**最容易被当成舵机坏了**的那种假。
        sys.exit("[注入] --servo-sweep 必须配 --arm: 不加的话每段都是 armed=false,\n"
                 "  舵机全程 park(不发脉冲),扫一整轮它都不会动 —— 那不是舵机坏。\n"
                 "  正确用法:python3 joy_inject.py --arm --servo-sweep")

    if args.servo_sweep and not SERVO["servo_enable"]:
        # 不降级、不继续:servo_enable=false 时 pwm_proc **连 servo 对象都不构造**,
        # a 发给谁都不知道。继续跑只会让舵机纹丝不动 —— 而「扫了一整轮它一动不动」
        # 这个结论会把排查方向整个带偏到「舵机坏了」上去。
        sys.exit("[注入] --servo-sweep 需要 servo_enable: true,当前是 false。\n"
                 "  这时 pwm_proc 根本没构造舵机对象,a **没有接收方** —— 跑了也是\n"
                 "  全程纹丝不动,而那不是舵机坏。改 motor_params.yaml 后重启:\n"
                 "    servo_enable: true\n"
                 "    bash ~/start_pwm.sh")

    # 单轮隔离在 steer_mode=servo 下无解(见 channel_axis)。早点给一句人话,
    # 别让它以 traceback 的形式出现 —— 那种报错很容易被当成脚本坏了。
    if args.wheel != "both" and (args.sweep or not explicit) and not _WHEELS_GET_A:
        sys.exit("[注入] --wheel %s 在 steer_mode=%s 下无解。\n"
                 "  该模式下 a 只给舵机,两个通道恒等 —— 造不出「只驱动一个轮子」。\n"
                 "  替代方案:\n"
                 "    * 验单轮/极性:python3 ~/motor_test.py --observe\n"
                 "      (按通道直接驱动 GPIO;需先停掉 pwm_proc,GPIO 独占)\n"
                 "    * 只验电机链路还能不能转:--l 0.25(两轮一起转,那仍然有效)"
                 % (args.wheel, STEER_MODE))
    if args.wheel != "both" and explicit and not args.sweep:
        print("[注入] --wheel 与 --l/--a 同时给出,以 --l/--a 为准(忽略 --wheel)",
              flush=True)

    if args.servo_sweep:
        phases = build_servo_sweep(args.astep, args.secs, PARK_SECS, args.arm)
    elif args.sweep:
        phases = build_sweep(args.wheel, args.dmin, args.dmax,
                             args.dstep, args.secs, args.arm)
    elif args.scenario:
        # 每段之间夹「停止」做对照:「停了没」和「转向对不对」是两件独立的事。
        SC = [("停止(仅 armed)", 0.0, 0.0), ("前进", 0.25, 0.0),
              ("停止", 0.0, 0.0), ("后退", -0.25, 0.0),
              ("停止", 0.0, 0.0), ("左转", 0.0, 0.25),
              ("停止", 0.0, 0.0), ("右转", 0.0, -0.25),
              ("停止(收尾)", 0.0, 0.0)]
        phases = [(n, l, a, PHASE_SECS, args.arm) for n, l, a in SC]
    elif args.wheel != "both" and not explicit:
        # 给了 --wheel 但没给目标值:默认用 0.25 试单个通道。
        l, a = channel_axis(args.wheel, 0.25)
        phases = [("仅%s duty 0.25" % _WHEEL_NAME[args.wheel], l, a,
                   args.secs, args.arm),
                  ("停止", 0.0, 0.0, 3.0, args.arm)]
    else:
        phases = [("注入", args.l, args.a, args.secs, args.arm)]

    addr = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("[注入] → udp://%s:%d  %.0fHz  armed=%s  共 %d 段"
          % (args.host, args.port, args.hz, args.arm, len(phases)), flush=True)
    # 打印通道↔物理轮的对应,免得看日志的人自己心算 swap_lr。
    print("[注入] 几何来自 %s  k=%.4f  swap_lr=%s → 通道L=%s 通道R=%s"
          % (GEO_SRC, K, SWAP_LR, _WHEEL_NAME["L"], _WHEEL_NAME["R"]), flush=True)
    # steer_mode 必须打出来。它是「a 到底有没有到轮子」的唯一提示 ——
    # servo 模式下 a 变化而两轮 duty 纹丝不动**是正常的**,不说清楚会看成故障。
    print("[注入] steer_mode=%s  %s"
          % (STEER_MODE,
             "a **只**给舵机;轮子只吃 l、两轮等速" if not _WHEELS_GET_A
             else "a 进差速混控" + ("(同时给舵机)" if STEER_MODE == "both" else "")),
          flush=True)
    if not _WHEELS_GET_A:
        print("[注入] ⚠️ 本模式下 a 不驱动轮子,所以「左转/右转」段只见舵机动、"
              "两轮 duty 不变。要单独验舵机就用 --a ±1(轮子会停,车原地只转舵机)。",
              flush=True)
    if args.servo_sweep:
        print("[注入] 舵机扫描: 中位=%.0fµs 范围=±%.0fµs → 每档脉宽见段名。"
              "l 恒 0 ⇒ 电机不励磁"
              % (SERVO["servo_center_us"], SERVO["servo_range_us"]), flush=True)
        print("[注入] 看三件事: ① 哪几档顶住不动/嗡嗡响(撞限位了,中位就是偏的)"
              " ② 哪一档最安静(那就是 servo_center_us)", flush=True)
        print("[注入] ③ a 变大时舵机往哪偏 → 定 servo_invert(别照名字猜)。"
              "最后一段是 armed=false 松掉,用耳朵对比:松掉后**照样**抖 ⇒ 与信号/驱动"
              "电流无关,往供电/共地/机械上查", flush=True)
    elif not args.arm:
        print("[注入] armed=false —— 电机不会动,只验证门控", flush=True)

    sent = 0
    try:
        sent = run_phases(sock, addr, phases, args.hz)
    except KeyboardInterrupt:
        print("\n[注入] 中断", flush=True)
    finally:
        # 收尾必须补发一串 armed=false:否则进程退出后 pwm_proc 要等 deadman
        # 到期才停,duty 会多转 300ms。
        for _ in range(10):
            send(sock, addr, 0.0, 0.0, False)
            time.sleep(0.01)
        sock.close()
        print("[注入] 结束  共发 %d 包  已补发 armed=false" % sent, flush=True)


if __name__ == "__main__":
    main()
