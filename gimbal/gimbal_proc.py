#!/usr/bin/env python3
"""二维云台串口输出进程 —— 网页摇杆 → UART,独立进程。

数据通路(全在本机,不经过 ROS)
------------------------------
    网页云台摇杆 62.5Hz
      → websocket → slam_viz._on_gimbal_control   (云台驾驶权仲裁在这)
      → UDP 127.0.0.1:5814  {"dx": -100..100, "dy": -100..100}
      → 本进程
      → /dev/ttyHS1 (40-pin 排针 PIN_29 TX) → 云台

用 UDP 不用 TCP:这是「最新值」流,旧包没有价值 —— 丢一个立刻被下一个覆盖。
但**输出侧是定节拍连发**,不是收到一个包才发一帧:见下面 `_step()`。

⚠️ 本进程**不碰 GPIO**,不申请任何 libgpiod line ⇒ 不可能和 `pwm_proc` /
   `motor_driver` 撞 `line busy`。串口与 GPIO 是完全独立的两条资源。
⚠️ 也**不要**把这个串口塞进 `pwm_proc`:那是 200Hz 严格节拍 + 1kHz 忙等线程,
   往里加一个**可能阻塞**的 syscall 是同一类事故的第三次发作(前两次见 pwm_proc
   的模块注释)。

==========================================================================
⚠️⚠️ 串口这一层,逐条都是坑
==========================================================================

【1】`tty.setraw` **不够** —— 它漏的正是最致命的
    它清 OPOST(⇒ 0x0A 不会被改写,好消息)、清 IXON、清 ICANON/ECHO/ISIG、置 CS8。
    **它不做的**:
      * **波特率** —— `setraw` 完全不碰 ispeed/ospeed ⇒ 停在驱动默认速率上
        (厂商示例是 9600)⇒ 表现是「**格式正确的乱码**」,最难查的一类。
      * **`IXOFF`** —— 它管「输入缓冲快满时**主动在 TX 上发 XOFF/XON**」。
        我们**从不 read**,pin16 悬空/有噪 ⇒ 缓冲填满 ⇒ **内核替我们往线上吐
        0x13/0x11**,而这两个字节在值域内可达,接收端分不出是我们的指令还是流控。
      * **`CRTSCTS`** —— 三线接法下 CTS 无人驱动 ⇒ **输出被永久闸住**。
      * **`CSTOPB`** —— `setraw` 不清 ⇒ 前一个占用者留下 2 停止位 ⇒ 持续帧错误。
    所以本文件自己写 `_cfmakeraw()`,并把波特率一起设了,然后**读回来断言**。

【2】短写**不能**照抄 `slam_viz._send_drive` 的「吞掉异常、尽力而为」
    那对 UDP 是对的(最新值语义,丢一个立刻被覆盖)。搬到字节流上是**错的**:
    串口**没有包边界**,一次短写 = 把接收端的解析相位**永久推移**半个帧。
    而本协议**没有帧头同步兜底** —— `gimbal_proto` 里实测过:值域内 `0xAA`
    **会**出现在数据字节里(401/40401 组,全部来自 `dx` 或 `dy == -86`),
    所以接收端**不能**靠「扫到 0xAA 当帧头」重新对齐。
    ⇒ 短写必须**补齐余下字节**;补不齐就把端口判成坏的、**响亮地停发**,
      绝不继续往线上吐(那会接在半帧后面,一路错下去)。

【3】写出去的字节 ≠ 上了线
    `os.write` 返回 6 只说明**驱动缓冲收了**。⇒ 状态行里**不许**出现「发送成功」
    这类词,只能报「写入 N 字节 / 短写 M 次 / 帧率 / 端口状态」。

【4】周期性 `tcgetattr` 读回比对
    **内核的实际状态才是不说谎的判据。** 若有个 `serial-getty@ttyHS1` 在跑
    (或后来被启动),它会用**自己的 termios 覆盖我们** ⇒ OPOST/IXON 全回来 ⇒
    我们逐字节比对没问题、**上线后字节被改写**。所以每 5 秒读回一次并比对。

【5】TIOCEXCL 独占打开
    把「别人偷偷往我的线上写字节」从**静默污染**变成**响亮失败**(EBUSY)。
    符合本项目取向。⚠️ 副作用:回环/脉冲诊断**不能和主进程同时跑**。

用法:
    python3 ~/gimbal_proc.py                          # 跑主循环
    python3 ~/gimbal_proc.py --pulse 50 0 1           # 诊断:推上去 1 秒再自动回零
    python3 ~/gimbal_proc.py --loopback               # 诊断:回环验 raw 配置
    ⚠️ 早先的 `--single-frame`(只发一帧)已删掉:协议的速度是**锁存的**,
       发一帧就走 = 云台一直转到撞限位。用 --pulse。

退出码:
    0 正常停止   2 串口打不开/配置失败   3 参数文件问题   4 参数值不合法
"""

import argparse
import errno
import fcntl
import json
import os
import signal
import socket
import sys
import time

import termios                       # ⚠️ 只准出现在本文件,见 gimbal_proto docstring

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from gimbal_proto import (FRAME_LEN, HEADER, clamp_cmd100, decode_frame,
                          encode_frame, frame_hex, slew_toward)

DEFAULT_PARAMS = os.path.join(_HERE, 'gimbal_params.yaml')
DEFAULT_PORT = 5814          # 5800/5811/5812/5813 已占
STATUS_EVERY = 5.0           # 每 N 秒打一行状态
#: 50Hz。⚠️ 原先是 0.016(≈62.5Hz,跟着网页 gimbalTick 走)。协议建议 **20~50Hz**:
#: 收端主循环是阻塞的(发脉冲期间不处理新命令),>50Hz 只是白白覆盖 target。
#: 网页那边仍按 16ms emit —— 那是**输入**节拍,与本进程的**发送**节拍是两回事。
DEFAULT_PERIOD_S = 0.02
DEFAULT_DEADMAN_MS = 500.0
#: 斜坡速率(单位/秒)。协议原文点 2:「每 20ms 让值变化不超过 ±5」= 5/0.02 = **250**。
#: `0` = 关(退回阶跃,A/B 对照用)。⚠️ 调高就是往「丢步/堵转」走。
DEFAULT_SLEW_PER_S = 250.0

#: 回环用的**故意找茬**载荷:把驱动最爱改写的字节全塞进去。
#:   0x0A → OPOST/ONLCR 会改写成 `0D 0A`(长度变了!相位当场错位)
#:   0x0D → OPOST/OCRNL 会改写
#:   0x11/0x13 → XON/XOFF:IXON 没关时,回环回来的 0x13 会让驱动**当场暂停输出**
#:   0xFF/0xAA/0x00/0x1A → 二进制洁净度的哨兵(0x1A 在某些模式是 EOF)
LOOPBACK_TORTURE = bytes([0xDE, 0x0A, 0x0D, 0x11, 0x13, 0xFF, 0xAA, 0x00, 0x9C, 0x1A])

_BAUD_NAMES = {
    50: 'B50', 75: 'B75', 110: 'B110', 134: 'B134', 150: 'B150', 200: 'B200',
    300: 'B300', 600: 'B600', 1200: 'B1200', 1800: 'B1800', 2400: 'B2400',
    4800: 'B4800', 9600: 'B9600', 19200: 'B19200', 38400: 'B38400',
    57600: 'B57600', 115200: 'B115200', 230400: 'B230400', 460800: 'B460800',
    500000: 'B500000', 921600: 'B921600',
}


# ══════════════════════════════════════════════════════════════════════
# 参数
# ══════════════════════════════════════════════════════════════════════
def load_params(path):
    """读 gimbal_params.yaml 的 gimbal.ros__parameters 段。

    和 pwm_proc 一样直接解析 YAML,不走 ROS parameter 机制 —— 这样本进程
    可以完全跑在系统 python3 上,ROS 环境变量那一堆坑一并绕开。
    """
    try:
        import yaml
    except ImportError:
        _die('需要 PyYAML: sudo apt install python3-yaml', 3)
    try:
        with open(path, 'r', encoding='utf-8') as f:
            doc = yaml.safe_load(f)
    except OSError as e:
        _die('读不到参数文件 %s: %s' % (path, e), 3)
    try:
        return doc['gimbal']['ros__parameters']
    except (TypeError, KeyError):
        _die('%s 里没有 gimbal.ros__parameters 段' % path, 3)


def _die(msg, code):
    """响亮退出,并让**退出码和文末那张表相符**。

    ⚠️ 别用 `sys.exit('...')` 代替:`SystemExit` 的 code 是那个**字符串**,
       进程实际退出码是 **1** —— 和文末写的「3 参数文件 / 4 参数值」对不上。
       调用方(`start_gimbal.sh`)虽然只打图例不分支,但「文档说 3、实际给 1」
       属于**会说谎的日志**那一类,按本项目纪律要修掉。
    """
    print('❌ %s' % msg, file=sys.stderr)
    sys.exit(code)


def baud_const(baud):
    """整数波特率 → `termios.B*` 常量。本机不支持就响亮地报出来。"""
    name = _BAUD_NAMES.get(int(baud))
    if name is None:
        raise ValueError('不认识的波特率 %r(协议是 115200)' % (baud,))
    const = getattr(termios, name, None)
    if const is None:
        raise ValueError('本机 termios 没有 %s —— 内核不支持这个速率' % name)
    return const


# ══════════════════════════════════════════════════════════════════════
# 串口
# ══════════════════════════════════════════════════════════════════════
def _cfmakeraw(attrs, speed):
    """把 termios 改成 raw + 指定波特率。返回新的 attrs 列表。

    ⚠️ 这是自写的,**不用 `tty.setraw`** —— 理由见模块 docstring【1】。
       Python 的 `termios` **没有** `cfmakeraw`(glibc 才有),
       `tty.setraw` 就是一个**没设波特率的** cfmakeraw。
    """
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs

    # 输入:清掉所有会「替我们改写/吞掉/注入字节」的开关。
    # IXON/IXOFF 尤甚 —— 它们会让内核在**我们的 TX 上**吐 0x11/0x13。
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
               termios.ISTRIP | termios.INLCR | termios.IGNCR |
               termios.ICRNL | termios.IXON | termios.IXOFF)
    if hasattr(termios, 'IXANY'):
        iflag &= ~termios.IXANY
    if hasattr(termios, 'IUCLC'):
        iflag &= ~termios.IUCLC

    # 输出:OPOST 是所有「改写字节」的总闸。
    for name in ('OPOST', 'ONLCR', 'OCRNL', 'ONOCR', 'ONLRET', 'OFILL', 'OFDEL'):
        if hasattr(termios, name):
            oflag &= ~getattr(termios, name)

    # 控制:8N1,无硬件流控;CLOCAL/CREAD 要**置上**(见下)。
    cflag &= ~(termios.CSIZE | termios.PARENB)
    for name in ('CSTOPB', 'CRTSCTS'):
        if hasattr(termios, name):
            cflag &= ~getattr(termios, name)
    cflag |= termios.CS8
    # CLOCAL:忽略调制解调器控制线。我们只接 PIN_29/PIN_30 两根线,
    #        不置它的话「无载波」状态会让写失败(EIO)或 open 阻塞。
    cflag |= termios.CLOCAL
    # CREAD:打开接收器。生产路径从不 read,但**回环测试要**用它,
    #        而且 open 路径必须和生产完全一致(见 open_serial 的 O_RDWR 注释)。
    cflag |= termios.CREAD

    # 本地:关掉所有终端语义。
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON |
               termios.ISIG | termios.IEXTEN)

    # 波特率:Python 没有 cfsetispeed/cfsetospeed,**速率就是写 attrs[4]/[5]**。
    # 顺手把 cflag 里的 CBAUD 位也设上:Linux 内核真正的速率存在 c_cflag 里,
    # glibc 的 tcgetattr 会从那里**合成**出 c_ispeed/c_ospeed —— 所以两处都设
    # 才不会出现「读回来是对的、实际没变」。(读回来断言见 open_serial。)
    ispeed = ospeed = speed
    cbaud = getattr(termios, 'CBAUD', 0x100F)
    cflag = (cflag & ~cbaud) | (speed & cbaud)

    cc = list(cc)
    cc[termios.VMIN] = 1
    cc[termios.VTIME] = 0

    return [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]


def termios_fingerprint(attrs):
    """把「我们在意的那些位」压成一个可比对的元组。

    ⚠️ 判据必须是**内核的实际状态**(tcgetattr 读回来的),不是我们的意图。
       见模块 docstring【4】。
    """
    iflag, oflag, cflag, lflag, ispeed, ospeed, _cc = attrs

    def bit(word, name):
        v = getattr(termios, name, 0)
        return bool(v and (word & v))

    return (
        ('OPOST',   bit(oflag, 'OPOST')),
        ('IXON',    bit(iflag, 'IXON')),
        ('IXOFF',   bit(iflag, 'IXOFF')),
        ('IXANY',   bit(iflag, 'IXANY')),
        ('CRTSCTS', bit(cflag, 'CRTSCTS')),
        ('CSTOPB',  bit(cflag, 'CSTOPB')),
        ('PARENB',  bit(cflag, 'PARENB')),
        ('CS8',     (cflag & termios.CSIZE) == termios.CS8),
        ('CLOCAL',  bool(cflag & termios.CLOCAL)),
        ('CREAD',   bool(cflag & termios.CREAD)),
        ('ispeed',  ispeed),
        ('ospeed',  ospeed),
    )


_BAUD_BY_CONST = {}


def _baud_number(const):
    """`termios.B115200` → `115200`。本机查不到就原样回显常量值。"""
    if not _BAUD_BY_CONST:
        # 反查表**在用到时**才建:`termios.B2000000` 这类常量并非每个平台都有,
        # 写死在模块级会 ImportError。
        for num, name in _BAUD_NAMES.items():
            c = getattr(termios, name, None)
            if c is not None:
                _BAUD_BY_CONST[c] = num
    return _BAUD_BY_CONST.get(const, const)


def fingerprint_str(fp):
    """人读的一行:**开着**的坏位 + 实际波特率。没有坏位就是干净的 raw。

    ⚠️⚠️ `ispeed`/`ospeed` 装的是**波特率常量值**(如 `termios.B115200`),**不是布尔标志**
       ⇒ 它们**恒为非零**。若照别的位一样判「开着 = 坏位」,这一行会**永远**印出
       `**ispeed,ospeed**` —— 一个**恒真的「故障」标记**,紧跟着又报 `baud=115200` 是对的。
       那比不印还坏:它让「真的有坏位」和「一切正常」看起来一样。2026-09-27 上板时实测到。
       波特率由下面的 `baud=` 单独报。
    """
    d = dict(fp)
    _NONFLAG = ('CS8', 'CLOCAL', 'CREAD', 'ispeed', 'ospeed')
    bad = [n for n, on in fp if on and n not in _NONFLAG]
    # 这三个是**该开却没开** —— 漏了 CREAD 回环读不回来,漏了 CLOCAL 写会 EIO。
    bad += [n + '=OFF' for n in ('CS8', 'CLOCAL', 'CREAD') if not d[n]]
    # ispeed 与 ospeed **不一致**本身是一种真故障(硬件流控/驱动不支持该速率时会这样),
    # 所以两个都报;一致时只报一个数,免得把一行日常日志变长。
    ispd, ospd = _baud_number(d['ispeed']), _baud_number(d['ospeed'])
    baud = str(ospd) if ispd == ospd else '%s/发送%s(!收发布特率不一致)' % (ispd, ospd)
    return '8N1 raw%s baud=%s' % (
        ' / **' + ','.join(bad) + '**' if bad else '', baud)


def open_serial(dev, baud, exclusive=True):
    """打开并配置串口。返回 `(fd, fingerprint)`。

    ⚠️ `O_RDWR` 而不是 `O_WRONLY` —— **不是**为了读,是为了让**回环测试和
       生产走同一条 open 路径**。`O_WRONLY` 下 `tcgetattr` POSIX 没承诺,
       别赌。生产路径从不调用 read。
    ⚠️ `O_NOCTTY` 必须 —— 否则它可能成为控制终端、吃 SIGHUP。
    ⚠️ `O_NONBLOCK` 只在 open 时带(避免无载波时阻塞)。**保持非阻塞**:
       写用「补齐 + 截止时间」自己处理,读用截止时间循环。
       这样 UART 卡住时进程不会被楔死。
    """
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        raise OSError(
            '打不开串口 %s: %s\n'
            '  ① 节点到底叫什么(别猜): ls -l /dev/ttyHS* ; ls /sys/class/tty/ | grep -i hs\n'
            '  ② 权限:                  ls -l %s  (应为 root:dialout 0660,且你在 dialout 组)\n'
            '     在组里但刚加完? 补充组不会影响已跑起来的进程 —— **重开会话**。\n'
            '  ③ 被占:                  sudo fuser -v %s'
            % (dev, e, dev, dev)) from e

    try:
        if exclusive:
            # ⚠️ `TIOCEXCL` 不是所有平台都有 ⇒ 先 hasattr。没有它只是少了保护,
            #    不是错误,所以**静默跳过**是对的(不能因为少个 ioctl 就不干活)。
            excl = getattr(termios, 'TIOCEXCL', None)
            if excl is not None:
                try:
                    fcntl.ioctl(fd, excl, 0)
                except OSError as e:
                    raise OSError(
                        '%s 已被别的进程独占打开(%s)。\n'
                        '  先看是谁: sudo fuser -v %s\n'
                        '  诊断模式(--loopback/--pulse)不能和主进程同时跑。'
                        % (dev, e, dev)) from e

        speed = baud_const(baud)
        want = _cfmakeraw(termios.tcgetattr(fd), speed)
        termios.tcsetattr(fd, termios.TCSAFLUSH, want)

        # ★ 读回来断言 —— 读的是**内核状态**,不是我们刚写进去的意图。
        got = termios_fingerprint(termios.tcgetattr(fd))
        exp = dict(termios_fingerprint(want))
        drift = [(n, dict(got)[n], exp[n]) for n, _ in got if dict(got)[n] != exp[n]]
        if drift:
            raise OSError(
                '串口配置**没生效**(读回来与写下去的意图不符):\n    '
                + '\n    '.join('%s: 实际 %r,期望 %r' % d for d in drift)
                + '\n  驱动可能不支持这些设置,或者有 serial-getty 在抢 '
                  '(systemctl list-units | grep -i ttyHS)。')
        return fd, got
    except BaseException:
        os.close(fd)
        raise


# ══════════════════════════════════════════════════════════════════════
# 主进程
# ══════════════════════════════════════════════════════════════════════
class GimbalProc:
    """网页 UDP 包 → 6 字节帧 → 串口。单线程,无锁竞争。"""

    def __init__(self, p, port):
        self._dev = str(p.get('serial_port', '/dev/ttyHS1'))
        self._baud = int(p.get('serial_baud', 115200))
        self._period = float(p.get('gimbal_cmd_period_s', DEFAULT_PERIOD_S))
        if self._period <= 0.0:
            raise ValueError('gimbal_cmd_period_s 必须 > 0(现在是 %r)' % self._period)
        self._deadman_s = float(p.get('gimbal_deadman_ms', DEFAULT_DEADMAN_MS)) / 1000.0
        self._slew_per_s = float(p.get('gimbal_slew_per_s', DEFAULT_SLEW_PER_S))
        self._invert_x = bool(p.get('gimbal_invert_x', False))
        self._invert_y = bool(p.get('gimbal_invert_y', False))
        self._port = port

        self._fd, self._want_fp = open_serial(self._dev, self._baud)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('127.0.0.1', port))
        self._sock.setblocking(False)

        self._dx = 0                 # **目标**(网页来的,已套 invert)
        self._dy = 0
        # **真正发出去的那两个值** —— 斜坡的输出。`_dx/_dy` 是目标,这两个才是线上值。
        # ⚠️ 加了斜坡之后 目标 != 已发,状态行必须两个都报(界面/日志说 A、线上是 B 就是说谎)。
        self._out_dx = 0
        self._out_dy = 0
        self._t_last_send = None     # 上一帧的发送时刻,用来算斜坡的 dt
        self._last_pkt = 0.0        # 0 = 从没收到过,不是「很久以前」
        self._age_ms = 0.0
        # ⚠️ 名字仍是 _holding,但它现在**只选择目标值**(按下时把目标压成 0),
        #    **不再表示「停发」** —— 见 _step。停发这件事已经从本进程里删掉了。
        self._holding = True        # 上电先按住(目标 = 0),直到网页真的开始发
        self._n_pkt = 0
        self._n_frame = 0
        self._n_bytes = 0
        self._n_short = 0           # 短写的**帧数**(不是调用次数)
        self._n_incomplete = 0      # 补不齐的帧数 —— 一旦发生,流已经脏了
        self._n_wr_err = 0
        self._port_broken = False
        self._last_err = None
        self._n_loop = 0
        # 「**意图发送**」的迭代数 + 发送时段的首尾时刻 —— 判「有没有被饿住」的基准。
        # 别拿窗口平均帧率去比目标帧率:窗口可能一半在 HOLD,那样正常工况也会响
        # (2026-09-27 实测踩到)。见 _status 里那段注释。
        self._n_send = 0
        self._t_send0 = None
        self._t_send1 = None
        self._drift = []
        self._run = True
        self._t_status = time.monotonic()

        # 启动先发一次 (0,0):协议明说参考实现开机这么做。
        # ⚠️ 而且**从这里开始就不再不发了** —— 见 _step 的注释。
        self._write_frame(encode_frame(0, 0))

    # ---------------------------------------------------------------- UDP
    def _recv_latest(self):
        """非阻塞抽干队列,只留最后一个状态。

        取全部而不是取一个:网页以 62.5Hz 发,本进程 100Hz 收,每次只读一个也不会
        积压;但万一卡了一下,留在队列里的旧值正是最不该用的。

        ⚠️ 钳位在**这里**做(镜像 pwm_proc._recv_latest):**别信网络来的数**。
           前端可能因为丢包/改版/别的客户端塞进来一个 5000。
        """
        got = False
        while True:
            try:
                data, _ = self._sock.recvfrom(512)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            try:
                m = json.loads(data.decode('utf-8', 'replace'))
            except ValueError:
                continue
            try:
                dx = clamp_cmd100(m.get('dx', 0))
                dy = clamp_cmd100(m.get('dy', 0))
            except Exception:
                continue
            # 方向反转在这里落定 —— 它是**配置**(gimbal_params.yaml),不是协议。
            self._dx = -dx if self._invert_x else dx
            self._dy = -dy if self._invert_y else dy
            got = True
            self._n_pkt += 1        # 按**报文**计数(别放在循环外)
        if got:
            self._last_pkt = time.monotonic()

    # ---------------------------------------------------------------- 串口写
    def _write_frame(self, frame, deadline_s=0.05):
        """把 6 字节**完整**写出去。返回 True = 全进了驱动缓冲。

        ⚠️ 为什么不是「尽力而为、吞掉异常」:见模块 docstring【2】。
           串口没有包边界,半帧上线 = 接收端解析相位**永久**推移,而本协议
           **没有帧头同步**(0xAA 在数据字节里可达)⇒ 它自己回不来。
        ⚠️ 但也**不抛异常杀掉循环** —— 一次抖动不该变成永久停机。
           补不齐 ⇒ 标记端口坏、**停发**、每 5 秒在状态行里说出来。
           停发是对的:继续写会把新帧接在那半帧后面,一路错下去。
           ⚠️⚠️ 但这是**全进程唯一**会真的停发的路径,而它有个严重副作用:
              发不出零速帧 ⇒ 云台会按最后锁存的速度一直转。见 `_break_port`。
        """
        view = memoryview(frame)
        total = len(frame)
        short = False
        t_end = time.monotonic() + deadline_s
        while view:
            try:
                n = os.write(self._fd, view)
            except (BlockingIOError, InterruptedError):
                # 缓冲满了。TX 缓冲通常 ≥4KB,而我们只要 375B/s —— 走这里
                # 基本就意味着端口卡住了。给一个很短的窗口等它吐出去。
                if time.monotonic() > t_end:
                    self._n_incomplete += 1
                    self._break_port('写不动(缓冲满且 %dms 内没吐出去)' % int(deadline_s * 1000))
                    return False
                time.sleep(0.0002)
                continue
            except OSError as e:
                self._n_wr_err += 1
                self._break_port('写失败: %s' % e)
                return False
            if n < len(view):
                short = True
            view = view[n:]

        if short:
            self._n_short += 1
        self._n_frame += 1
        self._n_bytes += total
        return True

    def _break_port(self, why):
        """端口进入永久故障态 —— **响亮**,但不杀进程。

        进程活着,状态行每 5 秒都会说它坏了。这比「静默停发」或「崩掉」
        都好:前者没人知道云台为什么不动,后者连日志窗口都一起没了。

        ⚠️⚠️ **这里说「保持不动」是错的,必须说反话。** 协议的速度是**锁存**的、
        收端没有超时保护 ⇒ 我们**发不出零速帧**就意味着云台会**按最后锁存的那个
        速度一直转下去,直到撞限位**。早先这里写的是「云台不会再收到任何帧
        (**保持不动**)」—— 那句话会让人**等它自己停**,而它不会停。
        排查方向完全不同:要人去**断云台的电源/急停**,而不是在本机上找参数。
        """
        if not self._port_broken:
            self._port_broken = True
            self._last_err = why
            self._log('!!! 串口已停用: %s' % why)
            self._log('!!! ⚠️⚠️ 本进程**再也发不出零速帧** —— 而协议的速度是锁存的、'
                      '收端没有超时保护,所以云台**可能正按最后那个速度一直转**,'
                      '直到撞限位。**现在就去断云台的电源**,别等它自己停。')
            self._log('!!! (半帧可能已经上了线,而本协议没有帧头同步,无法自行对齐 —— '
                      '所以不继续写。修好后重启本进程。)')

    # ---------------------------------------------------------------- 控制
    def _step(self):
        now = time.monotonic()
        if self._last_pkt > 0.0:
            self._age_ms = (now - self._last_pkt) * 1000.0

        # 失联判据:从没收到过包也算(上电后目标就压成 0,直到网页真的开始发)。
        # ⚠️ **不做滞回** —— 别照抄 motor_control.Deadman。那个滞回是为了挡住
        #    「电机 duty 在阈值两侧反复开合」,而这里抖一下只是目标在 0 和摇杆值之间
        #    切换,斜坡还在中间兜着,机构不会跟着抖。见 gimbal_params.yaml。
        holding = self._last_pkt == 0.0 or (now - self._last_pkt) > self._deadman_s
        if holding != self._holding:
            self._holding = holding
            self._log('网页数据中断 → 目标压成 **零速**(并持续发零速帧)'
                      if holding else '网页数据恢复 → 目标跟随摇杆')
        if self._port_broken:
            return

        # ⚠️⚠️ **这里没有「停发」这条路了,而且不能加回去。**
        #    协议原文:速度是**锁存的**,收端**没有超时保护** —— 「发完 dx=100 不发
        #    新帧,云台会一直转下去」。所以早先那条「断链就停发 ⇒ 云台保持不动」是
        #    **反的**:停发 = 云台按最后那个速度**一直转到撞限位**。
        #    正确形状 = **一直发**:失联时把目标压成 0,继续以定节拍发零速帧。
        #    零速帧才是「停」;重复发还能扛住单帧被丢或被误校验。
        #    ⚠️ 唯一的例外仍是端口坏(上面 return)—— 那时**发不出去**,见 _break_port。

        # ── 斜坡 ────────────────────────────────────────────────────
        # 收端**没有加减速斜坡**(协议点 2),0 → 8000 steps/s 的阶跃会「丢步/堵转」,
        # 现场看就是「推摇杆没反应」。所以斜坡必须在这里做。
        dt = self._period if self._t_last_send is None else (now - self._t_last_send)
        # ⚠️⚠️ **dt 必须钳住,而且钳位只能在这里做。** 循环若卡了 1 秒,dt=1.0 会算出
        #    max_delta = 250 ⇒ **一帧跳满**,斜坡在最需要它的那一刻(卡顿之后)失效。
        #    钳到 2 个周期:允许一点抖动,但绝不允许「一次补上整段时间」。
        if dt > 2.0 * self._period:
            dt = 2.0 * self._period
        elif dt < 0.0:
            dt = 0.0
        self._t_last_send = now
        tgt_dx, tgt_dy = (0, 0) if holding else (self._dx, self._dy)
        self._out_dx = slew_toward(self._out_dx, tgt_dx, self._slew_per_s, dt)
        self._out_dy = slew_toward(self._out_dy, tgt_dy, self._slew_per_s, dt)

        # 记下「发送」这件事发生的时间跨度 —— 判饿死的**唯一**可信基准。
        # ⚠️ 别改成数「该发的次数 vs 实际帧数」:那两个恒相等(`_write_frame` 唯一不
        #    计数的路径是端口坏,而那时上面已经 return 了)⇒ 那种判据**永远不响**。
        #    也别拿「窗口平均帧率」比目标帧率:窗口跨一次 HOLD 就被稀释(实测踩到)。
        #    真正要问的是:**在我们确实在发的那段时间里**,循环有没有跑到目标节拍。
        #    (改成「永远在发」之后,这条判据反而更干净了 —— 不再有停发时段去稀释它。)
        self._n_send += 1
        if self._t_send0 is None:
            self._t_send0 = now
        self._t_send1 = now
        self._write_frame(encode_frame(self._out_dx, self._out_dy))

    # ---------------------------------------------------------------- 状态
    def _log(self, msg):
        print('[gimbal_proc] %s' % msg, flush=True)

    def _check_termios(self):
        """周期性读回 termios 并比对。**内核状态才是不说谎的判据。**"""
        try:
            got = termios_fingerprint(termios.tcgetattr(self._fd))
        except OSError as e:
            self._drift = [('读取失败', str(e), '可读')]
            return
        want = dict(self._want_fp)
        self._drift = [(n, v, want[n]) for n, v in got if v != want[n]]

    def _status(self):
        now = time.monotonic()
        dt = now - self._t_status
        if dt < STATUS_EVERY:
            return
        rate_f = self._n_frame / dt if dt > 0 else 0.0
        rate_p = self._n_pkt / dt if dt > 0 else 0.0
        loop = self._n_loop / dt if dt > 0 else 0.0
        bps = self._n_bytes / dt if dt > 0 else 0.0
        # ⚠️ 判饿死要用的量**必须在清零前取出来** —— 写成清零后再读,读到的恒是 0/None,
        #    判据就**永远不触发**(把假警报换成一个死判据,更坏)。
        n_send, t_send0, t_send1 = self._n_send, self._t_send0, self._t_send1
        self._n_frame = self._n_pkt = self._n_bytes = self._n_loop = self._n_send = 0
        self._t_send0 = self._t_send1 = None
        # 「发送期间」的实测节拍:发送时段的首尾跨度算出来的,与窗口里有多少 HOLD 无关。
        span = (t_send1 - t_send0) if (t_send0 is not None and t_send1 is not None) else 0.0
        hz_send = ((n_send - 1) / span) if (n_send > 1 and span > 0) else 0.0
        self._t_status = now
        self._check_termios()

        # ⚠️ 这里**不许**出现「发送成功」这类词 —— os.write 返回 6 只说明驱动
        #    缓冲收了,不等于上了线。只报可观测的量。
        self._log('帧 %.1f/s/目标%.1fHz  包 %.1f/s  龄期 %.0fms  %s  '
                  '目标 dx=%+d dy=%+d  已发 dx=%+d dy=%+d  写入 %.0fB/s  轮询 %.0fHz%s  '
                  '短写 %d  未补齐 %d  写错 %d  端口 %s   [第1/2/3/5 个数为 %.0f 秒窗均]'
                  % (rate_f, 1.0 / self._period, rate_p, self._age_ms,
                     'HOLD(零速)' if self._holding else '跟随摇杆',
                     self._dx, self._dy, self._out_dx, self._out_dy, bps, loop,
                     '(发送时%.0fHz)' % hz_send if hz_send else '',
                     self._n_short, self._n_incomplete, self._n_wr_err,
                     '坏(%s)' % self._last_err if self._port_broken
                     else 'OK', STATUS_EVERY))

        # ⚠️⚠️ 判据是「**发送期间**的实测节拍」,不是「窗口平均帧率 < 目标一半」。
        #    2026-09-27 上板实测踩到:窗口只要**跨越一次 HOLD → 发送中**,窗口平均帧率
        #    就被前面的停发时段稀释(实测 2.1s 发送 + 2.9s 停发 ⇒ 报 26.2Hz/目标 62.5Hz),
        #    于是**正常工况下也会响**。那种「恒会响的告警」和恒不响一样没用 —— 它让
        #    「真的被饿住」和「一切正常」看起来一样。
        #    `hz_send` 只由**发送时段的首尾跨度**算出,与窗口里有多少 HOLD 无关 ⇒
        #    它和窗口帧率无关地回答问题:「我们确实在发的时候,循环跑到目标了吗」。
        #    ⚠️ 区间只有 1 次发送时无法判断(span=0),那时**不报** —— 报不了就别报。
        if hz_send and hz_send < 0.5 / self._period:
            self._log('!!! 发送期间只跑到 %.1fHz/目标%.1fHz(共发 %d 帧/%.2fs,轮询 %.0fHz)'
                      '—— 本进程被饿住了,云台会收到稀疏指令'
                      % (hz_send, 1.0 / self._period, n_send, span, loop))

        # termios 漂移单独一条 —— 它是要立刻处理的,不该被挤在一行长串的末尾。
        if self._drift:
            self._log('!!! 内核 termios 与我们设的不符 —— 有别的东西改过它 '
                      '(serial-getty?)。我们逐字节比对没问题,但**上线后字节会被改写**: '
                      + '; '.join('%s 实际=%r 期望=%r' % d for d in self._drift))

    # ---------------------------------------------------------------- loop
    def run(self):
        period = min(self._period, 0.05)      # 轮询比发送略快即可,不必更快
        nxt = time.monotonic()
        while self._run:
            self._recv_latest()
            self._step()
            self._status()
            self._n_loop += 1
            nxt += period
            slack = nxt - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                nxt = time.monotonic()        # 落后了就重新对齐,别累积

    def stop(self):
        self._run = False

    def shutdown(self):
        # 退出前发最后一帧 (0,0) —— 在速度语义下这是**唯一的刹车**,所以它的重要性
        # 比早先高得多(早先以为只是「回中」)。
        # ⚠️ 这条路只在**收到信号**时走到:SIGKILL / 掉电 / 崩在没装信号处理的路径上
        #    ⇒ 云台会**按最后锁存的速度一直转**。这不是「保持最后位置」(位置语义),
        #    而是**失控**。**单向无 ACK 的协议里这个洞堵不上**,只能靠云台自带的
        #    看门狗(协议未提,未知)或人去断电源。
        try:
            if not self._port_broken:
                self._write_frame(encode_frame(0, 0))
        except Exception:
            pass
        try:
            self._sock.close()
        except Exception:
            pass
        try:
            os.close(self._fd)
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════════
# 诊断模式(阶段 0 上板探测用,不跑主循环)
# ══════════════════════════════════════════════════════════════════════
def diag_pulse(p, dx, dy, secs):
    """推一下就走:斜坡升到 (dx,dy) → 保持 `secs` 秒 → **斜坡回零** → 退出。

    ⚠️⚠️ **不要把它改回「只发一帧」。** 协议的速度是**锁存的**、收端没有超时保护:
       发一帧 (50,0) 然后退出 = 云台**一直转到撞限位**,没有任何东西会把它停下。
       (本模式早先就叫 `--single-frame`,正是这个形状 —— 那是个会失控的诊断工具。)
       ⇒ 所以现在**必须**由我们把它降回零,而且退出前再补几帧零速。

    ⚠️ 这里发的是**原始协议值,不经过 invert_x/y / 不经过 UDP** —— 本模式的全部
       意义就是去看 dx=+50 到底让云台往哪边动。套上反转就永远判不出方向。
       它**独占**串口,所以主进程必须先停(否则 EBUSY)——
       这也正是它比 `gimbal_inject.py` 强的地方:整条 UDP/gimbal_proc 链路被排除在外,
       动了就是「串口+云台」这一段的功劳。

    判读(「绝对量还是增量」已定案 = **速度量**,不用再判):
        推杆方向对应云台转的方向 → 记下来,反了就改 gimbal_invert_x/y
        两轴是否真的独立       → 只给 dx 时看它有没有同时上下动
        一动不动               → 见 gimbal_wiring.md 附 1 的排除顺序(回环 → USB-TTL → 电平)
    """
    dx, dy = clamp_cmd100(dx), clamp_cmd100(dy)
    dev = str(p.get('serial_port', '/dev/ttyHS1'))
    rate = float(p.get('gimbal_slew_per_s', DEFAULT_SLEW_PER_S))
    period = float(p.get('gimbal_cmd_period_s', DEFAULT_PERIOD_S))
    # ⚠️⚠️ **这个守卫不能省** —— 本函数**不经过 `__init__`**,拿不到那边的
    #    `_period <= 0` 检查。而 `period == 0` 在这里是**死循环**:
    #    `slew_toward(cur, tgt, rate, dt=0)` 恒返回 `cur`(「没时间流逝就不该动」),
    #    于是下面的升/降环 `while cur_x != dx` **永远不满足退出条件**,却还在
    #    **满速写帧** ⇒ 进程挂死 + 云台一直转。两个后果都比早先的 `--single-frame`
    #    更坏。⇒ 和 `__init__` 用同一条判据,别让它只在那里存在。
    if period <= 0.0:
        raise ValueError('gimbal_cmd_period_s 必须 > 0(现在是 %r)' % period)
    fd, fp = open_serial(dev, int(p.get('serial_baud', 115200)))
    try:
        print('串口 %s  %s' % (dev, fingerprint_str(fp)))
        print('斜坡 %.0f 单位/秒  节拍 %.0fms  ⇒ 0→%d 约需 %.2fs'
              % (rate, period * 1000, max(abs(dx), abs(dy)),
                 (max(abs(dx), abs(dy)) / rate) if rate > 0 else 0.0))

        def _send(vx, vy):
            f = encode_frame(vx, vy)
            n = os.write(fd, f)
            if n < FRAME_LEN:                    # 补齐:半帧会把收端相位永久推移
                os.write(fd, f[n:])
            return f

        print('\n→ 推上去 dx=%+d dy=%+d,保持 %.1fs(原始值,未套 invert)' % (dx, dy, secs))
        cur_x, cur_y = 0, 0
        while cur_x != dx or cur_y != dy:        # 升
            cur_x = slew_toward(cur_x, dx, rate, period)
            cur_y = slew_toward(cur_y, dy, rate, period)
            _send(cur_x, cur_y)
            time.sleep(period)
        print('   已到 %s' % frame_hex(_send(cur_x, cur_y)))
        t_end = time.monotonic() + max(0.0, secs)
        while time.monotonic() < t_end:          # 保持
            _send(dx, dy)
            time.sleep(period)

        print('→ 回零')
        while cur_x != 0 or cur_y != 0:          # 降
            cur_x = slew_toward(cur_x, 0, rate, period)
            cur_y = slew_toward(cur_y, 0, rate, period)
            _send(cur_x, cur_y)
            time.sleep(period)
        for _ in range(5):                       # 多补几帧零速:单帧可能被丢/被误校验
            _send(0, 0)
            time.sleep(period)
        print('   已发 %s ×5' % frame_hex(encode_frame(0, 0)))

        print('\n现在**看云台**(它应该已经停了):')
        print('  转过且方向对    → 记下;反了就把 gimbal_invert_x/y 置 true')
        print('  只给 dx 时它上下也动 → 两轴不独立,协议里的 dy 不是我们以为的那个轴')
        print('  完全没动         → 见 gimbal_wiring.md 附 1:回环(29↔16)→ USB-TTL → 电平')
    finally:
        os.close(fd)


def diag_loopback(p):
    """短接 PIN_29↔PIN_16,把发出去的读回来 —— 验 raw 配置 + 验节点是不是那一对。

    ⚠️⚠️ **短路是 29↔16,不是 29↔30** —— 接错就是把驱动器输出直接对地。
       测完**必须拔掉跳线**。

    它能证明什么、不能证明什么(这条界限要守住):
      ✅ 这个节点**就是**排针 PIN_29(TX)/PIN_16(RX) 那一对
         —— 排除了 ttyHS3(它的 TX/RX 在 PIN_11/PIN_13)。
      ✅ OPOST 关了(否则 0x0A 回来会变成 0D 0A)
      ✅ IXON 关了(否则回环回来的 0x13 会让驱动当场暂停输出、读回超时)
      ❌ **证明不了 PIN_29 是 TX 而不是 RX** —— 短接是对称的。
         对**只发**的应用这是致命的:若 TX 其实是 PIN_16,云台**永远收不到**,
         而回环照样通过。那一次必须用外部观测(USB-TTL 只接 GND + RX 到 PIN_29)。
      ❌ 也证明不了编码对不对 —— 用的 `encode_frame` 和被验的是同一个。
         编码只有 `gimbal_test.py` 的黄金向量能判。
    """
    dev = str(p.get('serial_port', '/dev/ttyHS1'))
    fd, fp = open_serial(dev, int(p.get('serial_baud', 115200)))
    try:
        print('串口 %s  %s' % (dev, fingerprint_str(fp)))
        payload = LOOPBACK_TORTURE + encode_frame(10, 13) + encode_frame(-86, 0)
        print('→ 写 %d 字节: %s' % (len(payload), payload.hex(' ').upper()))
        os.write(fd, payload)

        got = bytearray()
        t_end = time.monotonic() + 1.0
        while len(got) < len(payload) and time.monotonic() < t_end:
            try:
                chunk = os.read(fd, 4096)
            except (BlockingIOError, InterruptedError):
                time.sleep(0.005)
                continue
            except OSError as e:
                print('❌ 读失败: %s' % e)
                return 2
            if chunk:
                got += chunk
        print('← 读 %d 字节: %s' % (len(got), bytes(got).hex(' ').upper()))

        ok = True
        if bytes(got) == payload:
            print('\n✅ 回环逐字节一致 ⇒ 这个节点就是 PIN_29/PIN_16 那一对'
                  '(排除了 ttyHS3),且 raw 配置正确')
        else:
            ok = False
            print('\n❌ 回环不一致 —— 逐段判读:')
            if len(got) != len(payload):
                print('   长度 %d ≠ %d:有字节被**吞掉**了(IXON/IXOFF 没关?)'
                      '或被**改写**成了两个字节(OPOST 没关?)' % (len(got), len(payload)))
            for i, (a, b) in enumerate(zip(payload, bytes(got))):
                if a != b:
                    print('   偏移 %d: 发出 0x%02X,收回 0x%02X' % (i, a, b))
            if b'\x0d\x0a' in bytes(got) and b'\x0a' in payload:
                print('   → 0x0A 变成了 0D 0A:**OPOST 没关**(理论上不可能,'
                      '因为我们设了 —— 那就是有别的东西在改内核状态)')
        print('\n⚠️ 这条**证明不了 PIN_29 是 TX 而不是 RX**(短接是对称的)。')
        print('   对只发不收的应用,这一点必须用外部观测确认:')
        print('   USB-TTL 只接 GND,适配器的 **RX 接到 PIN_29**(绝不接适配器的 TX)。')
        print('⚠️ 测完**必须拔掉 PIN_29↔PIN_16 的跳线**。')
        return 0 if ok else 1
    finally:
        os.close(fd)


# ══════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description='网页摇杆 → 云台串口 输出进程')
    ap.add_argument('--params', default=DEFAULT_PARAMS,
                    help='参数文件 (默认 %s)' % DEFAULT_PARAMS)
    ap.add_argument('--port', type=int, default=DEFAULT_PORT,
                    help='UDP 监听端口 (默认 %d)' % DEFAULT_PORT)
    # ⚠️⚠️ `nargs='+'` **不能**配多元素 `metavar` 元组 —— argparse 会在构造
    #    parser 时就抛 `ValueError: length of metavar tuple does not match nargs`,
    #    于是**整个进程连 --help 都起不来**,报错还发生在 `main()` 的最开头。
    #    2026-09-27 上板实撞:本地只做了 `ast.parse` 语法检查 ⇒ 完全看不见这个
    #    (它是**运行期契约**,不是语法错)。⇒ 用单个字符串,用法行会显示
    #    `--pulse N [N ...]`,真正的形状写在 help 里。
    ap.add_argument('--pulse', nargs='+', type=int, metavar='N',
                    help='诊断:推上去→保持→自动回零(默认持有 1.0s)。'
                         '形状是 `--pulse DX DY [SECS]`,取值 -100..100。'
                         '⚠️ 早先叫 --single-frame(只发一帧),那个形状在'
                         '「速度锁存、收端无超时」的协议下会**让云台一直转**,已废弃')
    ap.add_argument('--loopback', action='store_true',
                    help='诊断:回环测试(需短接 PIN_29↔PIN_16,验完必须拔掉)')
    args = ap.parse_args()

    params = load_params(args.params)
    try:
        if args.pulse:
            if len(args.pulse) not in (2, 3):
                print('❌ --pulse 要 2 或 3 个数:DX DY [SECS]', file=sys.stderr)
                return 4
            return diag_pulse(params, args.pulse[0], args.pulse[1],
                              args.pulse[2] if len(args.pulse) == 3 else 1.0)
        if args.loopback:
            return diag_loopback(params)

        proc = GimbalProc(params, args.port)
    except OSError as e:
        print('❌ %s' % e, file=sys.stderr)
        return 2
    except ValueError as e:
        print('❌ 参数不对: %s' % e, file=sys.stderr)
        return 4

    def _on_signal(_signum, _frame):
        proc.stop()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    proc._log('就绪 — 串口 %s  %s' % (proc._dev, fingerprint_str(proc._want_fp)))
    proc._log('UDP 127.0.0.1:%d  节拍 %.1fHz(%.0fms)  deadman %.0fms  斜坡 %s  '
              'invert x=%s y=%s'
              % (args.port, 1.0 / proc._period, proc._period * 1000.0,
                 proc._deadman_s * 1000.0,
                 ('%.0f 单位/秒' % proc._slew_per_s) if proc._slew_per_s > 0 else '**关**',
                 'Y' if proc._invert_x else 'N', 'Y' if proc._invert_y else 'N'))
    proc._log('失联时把目标压成 **零速**(并持续发零速帧)。⚠️ 不能改成「停发」——'
              '协议的速度是锁存的,停发 = 云台一直转到撞限位。'
              '这条链路**没有任何 armed 门** —— 云台不受「启用电机」管。')
    try:
        proc.run()
    finally:
        proc.shutdown()
        proc._log('已停止,串口已释放(退出前已发一帧 (0,0))')
    return 0


if __name__ == '__main__':
    sys.exit(main())
