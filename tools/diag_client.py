#!/usr/bin/env python3
"""诊断:假扮网页客户端连 slam_viz,量真实的发送率与接收率。

用法:
    python3 ~/diag_client.py [秒数] [Hz] [transport]
        transport: polling(默认) | websocket | auto

为什么要在板子上用脚本而不是看浏览器:
    「摇杆动 → 电机动」中间有两段,浏览器→slam_viz 这一段如果慢,锅可能在
    浏览器(主线程被别的 SocketIO 消息堵住),也可能在服务端(polling 每个包
    一次 HTTP 往返 + dev server 每请求起一个线程)。把浏览器换成一个频率可控
    的探针,单独跑一次就能把两者分开:
        探针能跑满 60Hz → 服务端没问题,锅在浏览器
        探针也跑不满   → 服务端 SocketIO 这条路过不去,和浏览器无关

安全性:只发 l=0 a=0。服务端收到 vel 会把 armed 置真并转发给 pwm_proc,
但占空比算出来就是 0 —— 电机不会转。
"""

import collections
import sys
import threading
import time

import socketio

VERBOSE = "-v" in sys.argv
_argv = [a for a in sys.argv[1:] if a != "-v"]        # -v 是开关,不占位置参数

DUR = float(_argv[0]) if len(_argv) > 0 else 20.0
HZ = float(_argv[1]) if len(_argv) > 1 else 60.0
TRANSPORT = _argv[2] if len(_argv) > 2 else "polling"
URL = _argv[3] if len(_argv) > 3 else "http://127.0.0.1:5800"

counts = collections.Counter()
lock = threading.Lock()

sio = socketio.Client(logger=VERBOSE, engineio_logger=VERBOSE)


@sio.event
def connect():
    print("已连接  transport=%s" % sio.transport(), flush=True)


@sio.event
def disconnect():
    print("断开", flush=True)


@sio.on("*")
def _catch(event, *args):
    with lock:
        counts[event] += 1


def main():
    try:
        sio.connect(URL, transports=[TRANSPORT] if TRANSPORT != "auto" else None)
    except Exception as e:
        print("连接失败 (%s): %s" % (TRANSPORT, e), flush=True)
        return 2

    # 必须先抢驾驶权:_on_motor_control 只认 _drive_owner 发来的 vel,
    # 别的 sid 的包在 _vel_stat 之前就被 return 掉了 —— 不 arm 的话探针
    # 会「发出去了但服务端一个都没统计」,看起来像网络丢包,其实是仲裁丢弃。
    # arm 附带 _send_drive(0,0,True):l=a=0 → duty 0,电机不动。
    sio.emit("motor_control", {"action": "arm", "armed": True})
    time.sleep(0.3)

    period = 1.0 / HZ
    nxt = time.monotonic()
    sent = 0
    t0 = time.monotonic()
    nxt_report = t0 + 2.0
    last_sent = sent

    while time.monotonic() - t0 < DUR:
        try:
            sio.emit("motor_control",
                     {"action": "vel", "linear": 0.0, "angular": 0.0})
            sent += 1
        except Exception as e:
            print("emit 失败: %s" % e, flush=True)
            break
        nxt += period
        slack = nxt - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        else:
            nxt = time.monotonic()

        now = time.monotonic()
        if now >= nxt_report:
            with lock:
                snap = dict(counts)
                counts.clear()
            dt = 2.0
            print("发送 %5.1f Hz   收到 %s" % (
                (sent - last_sent) / dt,
                "  ".join("%s=%.1f/s" % (k, v / dt)
                          for k, v in sorted(snap.items())) or "(无)",
            ), flush=True)
            last_sent = sent
            nxt_report = now + 2.0

    dt = time.monotonic() - t0
    print("\n=== 汇总 (%.1fs, %s) ===" % (dt, sio.transport()), flush=True)
    print("发出 motor_control  %.1f Hz (目标 %.0f Hz)" % (sent / dt, HZ), flush=True)

    # 交还驾驶权并解除 armed,别把机器人留在「探针是驾驶者」的状态
    try:
        sio.emit("motor_control", {"action": "estop"})
        time.sleep(0.3)
    except Exception:
        pass
    sio.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
