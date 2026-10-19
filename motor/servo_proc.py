#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""舵机独立进程:50Hz 软 PWM,拥有**自己**的 GIL。

为什么拆出来
-----------
CPython 的 GIL 让「1kHz 电机 PWM 线程」和「50Hz 舵机脉宽线程」在同一个进程里
无法同时做到 µs 级:舵机线程 sleep 醒来后要抢 GIL,最长等 5ms(默认切换间隔)
⇒ 实测脉宽 p50=1756µs 但 p95=3.4ms、max=16ms。舵机看到 2~9 倍的脉冲,现场表现
就是「按住转向时抖」。本进程只做一件事:按 50Hz 精确发脉冲,没有别的线程抢 GIL。

和 pwm_proc 的契约(本机 UDP 127.0.0.1:5816,纯文本,最新值语义)
--------------------------------------------------------------
    "park"     → 停发脉冲(舵机松掉、几乎不吃电)
    "1750.0"   → 发 1750µs 的脉冲(会自动过电气硬限位)
    300ms 收不到任何消息 → 自动 park。父进程崩了也不会一直咬着转角。

用法(正常由 pwm_proc 拉起,也可手工调试):
    python3 servo_proc.py --line 33 --center 1500 --range 500
"""
import argparse
import os
import signal
import socket
import sys
import time

import gpiod

PARK_TIMEOUT_S = 0.3
STATUS_EVERY_S = 5.0
STAT_MAX = 250          # 5 秒 @50Hz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--chip', default='gpiochip0')
    ap.add_argument('--line', type=int, required=True)
    ap.add_argument('--freq', type=float, default=50.0)
    ap.add_argument('--center', type=float, default=1500.0)
    ap.add_argument('--range', type=float, default=500.0)
    ap.add_argument('--min', dest='min_us', type=float, default=500.0)
    ap.add_argument('--max', dest='max_us', type=float, default=2500.0)
    ap.add_argument('--port', type=int, default=5816)
    args = ap.parse_args()

    period = 1.0 / args.freq
    if args.max_us >= period * 1e6:
        sys.exit('servo max_us 必须小于帧周期')
    if not (0.0 <= args.min_us < args.max_us):
        sys.exit('servo min/max 非法')

    # 2026-10-19: 脉冲只有 1~2.5ms,而本机 load 常年 6~8(api 225%、encoder_sampler
    # 94%)—— 普通 CFS 调度下这 1~2ms 会被抢占,线电平被拉长到 3~11ms,舵机就抖。
    # 实时优先级让这 1~2ms 不被抢;本进程其余时间都在 sleep,不会饿死别的进程。
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
        print('[servo_proc] SCHED_FIFO prio=50', flush=True)
    except (AttributeError, OSError) as e:
        print('[servo_proc] SCHED_FIFO 不可用(%s) —— 抖动会比理想值大' % e, flush=True)

    chip = gpiod.Chip(args.chip)
    line = chip.get_line(args.line)
    line.request(consumer='servo_steer_proc', type=gpiod.LINE_REQ_DIR_OUT)
    line.set_value(0)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('127.0.0.1', args.port))
    sock.setblocking(False)

    stop = {'flag': False}

    def _stop(signum, frame):
        stop['flag'] = True

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    pulse = None
    last_msg = 0.0
    widths = []
    cmds = []
    t_status = time.monotonic()
    n_frame = 0

    print('[servo_proc] 就绪 line=%d center=%.0f range=±%.0f port=%d'
          % (args.line, args.center, args.range, args.port), flush=True)

    while not stop['flag']:
        t0 = time.monotonic()

        # 收指令:非阻塞,只有最新值有意义
        while True:
            try:
                data, _ = sock.recvfrom(64)
            except BlockingIOError:
                break
            except OSError:
                break
            text = data.decode('ascii', 'ignore').strip()
            last_msg = t0
            if text == 'park':
                pulse = None
            else:
                try:
                    pulse = float(text)
                except ValueError:
                    pass

        if pulse is not None and (t0 - last_msg) > PARK_TIMEOUT_S:
            print('[servo_proc] %.0fms 没收到指令 → park' % ((t0 - last_msg) * 1000), flush=True)
            pulse = None

        if pulse is None:
            line.set_value(0)
            slack = period - (time.monotonic() - t0)
            if slack > 0:
                time.sleep(slack)
        else:
            us = min(args.max_us, max(args.min_us, pulse))
            line.set_value(1)
            t_rise = time.monotonic()
            # 整段自旋:本进程没有别的线程,不需要让出 GIL;sleep 的过冲就是抖动
            end = t_rise + us / 1e6
            while time.monotonic() < end:
                pass
            t_fall = time.monotonic()
            line.set_value(0)

            widths.append(t_fall - t_rise)
            cmds.append(us)
            if len(widths) > STAT_MAX:
                del widths[0]
                del cmds[0]
            n_frame += 1

            slack = (t0 + period) - time.monotonic()
            if slack > 0:
                time.sleep(slack)

        now = time.monotonic()
        if now - t_status >= STATUS_EVERY_S:
            t_status = now
            if widths:
                w = sorted(x * 1e6 for x in widths)
                n = len(w)
                print('[servo_proc] 指令 %.0fus  实测 p50=%.0f p95=%.0f max=%.0f  n=%d'
                      % (cmds[-1], w[n // 2], w[min(n - 1, int(n * 0.95))], w[-1], n),
                      flush=True)
            else:
                print('[servo_proc] PARK(无脉冲)', flush=True)

    line.set_value(0)
    print('[servo_proc] 退出,line 已拉低', flush=True)


if __name__ == '__main__':
    main()
