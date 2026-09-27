#!/usr/bin/env python3
"""云台注入器 — 绕开浏览器,直接向 gimbal_proc 的 UDP 口灌 dx/dy。

**为什么需要它**(和 `joy_inject.py` 同样的理由,但未知量更多)
网页 → SocketIO → slam_viz → UDP → gimbal_proc → 串口 → 云台 这条链路上有
五个可能出错的环节。这个脚本只做最后一段(UDP),于是:

  * 注入能动、网页不能动 → 问题在浏览器链路(往上游查)
  * 注入也不能动         → 问题在 gimbal_proc / 串口 / 云台(往下游查)

串口比 GPIO 多出**节点名 / 波特率 / 方向 / 权限**四层未知,所以这个工具比
`joy_inject.py` 更不可省 —— 它是唯一能把「网页链路坏」和「串口/云台坏」分开的东西。

⚠️⚠️ **它没有 `--arm`。**
`joy_inject.py` 不加 `--arm` 时全程 `armed=false`,电机绝不动;云台**没有**
这个门(设计决定:转相机 ≠ 驱动轮子)。⇒ **一跑起来云台就会动。**
跑之前确认云台行程里没有手指、线缆,且你已经准备好随时 Ctrl-C。

⚠️ 它**看不到物理世界**。日志只说「下发了 dx/dy」,云台转了没有、往哪转、
有没有撞限位,必须由人盯着。所以扫描类模式刻意做成**分段停、逐段确认**。

⚠️ 收尾**必定补发一串 (0,0)**:不补的话,进程要等 deadman(500ms)到期才把目标压成
零速 —— 而在此之前云台一直在按最后那个速度转(协议的速度是**锁存的**,见
gimbal_proto.py 的模块 docstring)。

用法:
  python3 gimbal_inject.py --axis-sweep        # ← 先跑这个:定 invert_x / invert_y
  python3 gimbal_inject.py --scenario          # 画个方框,看两轴是否独立、有无卡顿
  python3 gimbal_inject.py --dx 50 --dy 0 --secs 3
  python3 gimbal_inject.py --once --dx 50      # ⚠️ 只发一包,见下方说明
"""

import argparse
import json
import os
import socket
import sys
import time

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5814
DEFAULT_HZ = 20.0

# 每个姿势段 6 秒:gimbal_proc 每 5 秒打一行状态,段长必须大于它,
# 否则一行都落不进段内,事后无法逐段对照。
PHASE_SECS = 6.0

#: gimbal_proc 的 deadman(默认 500ms = 2Hz)。注入包率必须**明显高于**它,
#: 否则 gimbal_proc 会把目标压成零速(它**不会**停发 —— 见那边的 `_step`),
#: 扫出来的是「云台不动」这个假故障。
#: 留 1.5 倍余量,和 joy_inject 对 300ms 取 3.3 是同一套算法。
_MIN_HZ = 1.5 / 0.5                     # = 3.0Hz

_YAML_CANDIDATES = ("gimbal_params.yaml", "/home/aidlux/gimbal_params.yaml")


def load_params():
    """读 gimbal_params.yaml 的 `gimbal.ros__parameters`。

    只读了两个 invert 标志 —— 用来把「我注入的值」和「进程真正发出去的值」分开
    打印。注入器是用来排查问题的,不能因为它自己起不来而挡住排查,所以读不到
    就退回默认值 + 一句来源说明,不抛异常。
    """
    d = {"gimbal_invert_x": False, "gimbal_invert_y": False,
         "serial_port": "?"}
    try:
        import yaml
    except ImportError:
        return d, "默认值(pyyaml 不可用)"
    for path in _YAML_CANDIDATES:
        if not os.path.exists(path):
            continue
        try:
            with open(path, encoding="utf-8") as f:
                p = yaml.safe_load(f)["gimbal"]["ros__parameters"]
        except Exception as e:                                  # noqa: BLE001
            return d, "默认值(解析 %s 失败: %s)" % (path, e)
        d["gimbal_invert_x"] = bool(p.get("gimbal_invert_x", False))
        d["gimbal_invert_y"] = bool(p.get("gimbal_invert_y", False))
        d["serial_port"] = str(p.get("serial_port", "?"))
        return d, path
    return d, "默认值(没找到 gimbal_params.yaml)"


PARAMS, PARAMS_SRC = load_params()


def final_value(v):
    """我注入的 dx,经过进程里的 invert 之后**真正发出去**的值。

    ⚠️ 反转发生在 `gimbal_proc._recv_latest`(**接收侧**),不在前端也不在这里。
       ⇒ 只看注入器日志会得出错误的方向结论。把两者都打出来,免得对不上时
         以为是串口坏了。
    """
    return -v if PARAMS["gimbal_invert_x"] else v


def send(sock, addr, dx, dy):
    sock.sendto(json.dumps({"dx": int(dx), "dy": int(dy)}).encode(), addr)


def run_phases(sock, addr, phases, hz):
    """按**绝对时基**跑完 phases=[(名字, dx, dy, 秒)]。返回发包总数。

    用绝对时基 + 过载重对齐(和 `pwm_proc` / `joy_inject` 同一套写法):
    `sleep(period - elapsed)` 会累积漂移,长时序下段边界会错开,事后对不上日志。
    """
    period = 1.0 / hz
    sent = 0
    for name, dx, dy, secs in phases:
        print("── %s  dx=%+4d dy=%+4d   %gs  t=%s"
              % (name, dx, dy, secs, time.strftime("%H:%M:%S")), flush=True)
        deadline = time.monotonic() + secs
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            send(sock, addr, dx, dy)
            sent += 1
            slp = now + period - time.monotonic()
            if slp > 0:
                time.sleep(slp)
    return sent


def build_axis_sweep(step, secs, dmax):
    """**先只扫 dx,再只扫 dy** —— 一次跑完定掉 invert_x / invert_y 和行程。

    这是最该先跑的一个模式,因为它回答的是「**两个独立的未知量**」:
      * **方向** —— dx 从 -100 到 +100 时云台往哪边偏 ⇒ 定 gimbal_invert_x。
        别照名字猜(和 motor_params.yaml 的 servo_invert 一样:`a` 变大往哪转
        是实测出来的,不是推导出来的)。
      * **行程** —— 哪几档顶住不动/嗡嗡响 ⇒ 那是撞限位了,指令值域要收窄。
      * **两轴是否真的独立** —— 扫 dx 时 dy **恒为 0**,若云台同时上下动,
        说明协议里的 dy 不是我们以为的那个轴,或者接收端把两轴混控了。

    ⚠️ 另一轴全程恒 0。这正是它安全的地方:一次只有一个轴在动,
       撞限位时你能立刻看出来是哪个轴。
    """
    phases = []
    for label, is_x in (("x", True), ("y", False)):
        v = -dmax
        while v <= dmax + 1e-9:
            dx, dy = (v, 0) if is_x else (0, v)
            phases.append(("扫 %s: dx=%+4d dy=%+4d" % (label, dx, dy), dx, dy, secs))
            v = int(round(v + step))
        # 每个轴扫完发零速再停一下:否则「x 扫到 +100」紧接着「y 从 -100 开始」,
        # 两段之间看不出是哪个轴在动。
        # ⚠️ 措辞是「零速」不是「回中」:(0,0) 是**速度 0 = 停**,收端不会去找中位。
        phases.append(("零速(换轴)", 0, 0, 3.0))
    phases.append(("★ 零速收尾", 0, 0, 3.0))
    return phases


def build_scenario(secs):
    """画一个方框:上 → 右 → 下 → 左 → 零速,每步之间**发零速停一下**。

    「它停了没」和「它走的方向对不对」是两件独立的事,夹零速才能分开判。
    若两轴真的独立,这四条边应该首尾相接、最后回到出发点附近;
    若走成了斜线或三角形,那就说明两轴有耦合(或接收端在做插补)。

    ⚠️ 措辞是「零速」不是「回中」:`(0,0)` 是**速度 0 = 停**,云台**不会去找中位**
       (回中只有 `0xBB cmd=1/2` 能做,本实现没做)。写「回中」会让人在排查时
       往完全错的方向查 —— 尤其是这个模式本来就是用来判「它到底停没停」的。
    """
    steps = [("上 (dx=0 dy=+100)", 0, 100),
             ("右 (dx=+100 dy=0)", 100, 0),
             ("下 (dx=0 dy=-100)", 0, -100),
             ("左 (dx=-100 dy=0)", -100, 0)]
    phases = []
    for name, dx, dy in steps:
        phases.append((name, dx, dy, secs))
        phases.append(("停止(发零速)", 0, 0, 2.0))
    phases.append(("★ 零速收尾", 0, 0, 3.0))
    return phases


def main():
    ap = argparse.ArgumentParser(description="云台注入器(绕开浏览器直灌 UDP)")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ,
                    help="发包率,必须明显高于 deadman(500ms),否则误触发")
    ap.add_argument("--secs", type=float, default=PHASE_SECS)
    ap.add_argument("--dx", type=int, default=0, help="-100..+100")
    ap.add_argument("--dy", type=int, default=0, help="-100..+100")
    ap.add_argument("--step", type=int, default=25, help="--axis-sweep 的步长")
    ap.add_argument("--dmax", type=int, default=100, help="--axis-sweep 的端点")
    ap.add_argument("--axis-sweep", action="store_true",
                    help="先扫 dx 再扫 dy(定 invert_x/invert_y 与行程)")
    ap.add_argument("--scenario", action="store_true", help="画方框,验两轴独立性")
    ap.add_argument("--once", action="store_true",
                    help="只发一包就退出(⚠️ 见下方警告)")
    args = ap.parse_args()

    # ⚠️ 这个守卫**不能放宽**。低于它 gimbal_proc 会把目标压成零速,
    #    于是「云台扫了一整轮一步没动」—— 那是最容易被当成云台坏了的假故障。
    if args.hz <= _MIN_HZ:
        sys.exit("[注入] 发包率必须 > %.1fHz,否则 deadman(500ms)会把它当成断流。\n"
                 "  那样扫出来的是「云台一动不动」,而那不是云台坏。" % _MIN_HZ)

    if args.once:
        # ⚠️⚠️ **--once 现在测的是「deadman 到不到期」,不再是「绝对量还是增量」。**
        # 那个问题**已定案 = 速度量**(且锁存,收端无超时):所以「发一包」不会让云台
        # 只动一下,它会在 gimbal_proc 的节拍上被**持续重发**约 500ms(到 deadman 到期为止),
        # 云台就按那个速度转 0.5 秒 —— 这不是「复现 bug」,这是**设计行为**。
        print("[注入] ⚠️ --once:只发一包,但 gimbal_proc 会在 ~500ms 内持续重发。",
              flush=True)
        print("[注入]    协议是**速度**语义(锁存)⇒ 云台会按那个速度转 0.5 秒,",
              flush=True)
        print("[注入]    然后 deadman 到期、目标被压成零速 ⇒ 停。这是**设计如此**。",
              flush=True)
        print("[注入]    它现在只用来验「deadman 跳不跳闸」——**先把手指放在 Ctrl-C 上**。",
              flush=True)

    if args.axis_sweep:
        phases = build_axis_sweep(args.step, args.secs, args.dmax)
    elif args.scenario:
        phases = build_scenario(args.secs)
    else:
        phases = [("注入", args.dx, args.dy, args.secs)]

    # 值域钳制在这里也做一遍:gimbal_proc 会钳,但**在它钳之前**,日志里
    # 打一个 5000 会让人以为是云台转头了 5000 度。这里先拦下来并说清楚。
    for name, dx, dy, _s in phases:
        if not (-100 <= dx <= 100 and -100 <= dy <= 100):
            sys.exit("[注入] %s 的 dx=%d dy=%d 超出协议值域 ±100。\n"
                     "  (gimbal_proc 会把它钳到 ±100 —— 但那样你看到的日志和"
                     "实际发生的事就不是一回事了)" % (name, dx, dy))

    addr = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("[注入] → udp://%s:%d  %.0fHz  共 %d 段"
          % (args.host, args.port, args.hz, len(phases)), flush=True)
    print("[注入] 参数来自 %s:  invert_x=%s invert_y=%s  串口=%s"
          % (PARAMS_SRC, PARAMS["gimbal_invert_x"], PARAMS["gimbal_invert_y"],
             PARAMS["serial_port"]), flush=True)
    if PARAMS["gimbal_invert_x"] or PARAMS["gimbal_invert_y"]:
        # 反转发生在**进程的接收侧**,不在前端也不在这里 —— 不说明的话,
        # 日志里的 dx 和云台的实际动作方向对不上,会以为是串口坏了。
        print("[注入] ⚠️ 反转已开(dx=%s dy=%s):我注入的 dx 是**网页语义**的值,"
              % (PARAMS["gimbal_invert_x"], PARAMS["gimbal_invert_y"]), flush=True)
        print("[注入]    进程发出去的会是 dx=%+d(以 dx=+100 为例)。"
              % final_value(100), flush=True)
    print("[注入] ⚠️ 云台**没有 armed 门** —— 它马上就会动。", flush=True)

    if args.axis_sweep:
        print("[注入] 看三件事: ① dx 变大时云台往哪边偏 → 定 gimbal_invert_x"
              " ② 哪几档顶住不动/嗡嗡响 → 撞限位了", flush=True)
        print("[注入] ③ 扫 dx 时云台**有没有同时上下动** —— 有的话说明两轴"
              "不是独立的(协议理解错了,或接收端在混控)", flush=True)

    sent = 0
    try:
        sent = run_phases(sock, addr, phases, args.hz)
    except KeyboardInterrupt:
        print("\n[注入] 中断", flush=True)
    finally:
        # ⚠️ 收尾**必须**补发一串 (0,0):目标被压成零速是要时间的,而在此之前云台
        #    一直按最后那个速度转(协议是速度语义、锁存)。不补的话,退出后要到
        #    deadman 到期才降下来 —— 扫到 +100 时中断,那半秒是全速冲限位。
        for _ in range(10):
            send(sock, addr, 0, 0)
            time.sleep(0.01)
        sock.close()
        print("[注入] 结束  共发 %d 包  已补发 (0,0)" % sent, flush=True)
        print("[注入] 云台已收到零速;deadman 500ms 后 gimbal_proc 继续发零速帧(不是停发)。",
              flush=True)


if __name__ == "__main__":
    main()
