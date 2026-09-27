#!/usr/bin/env python3
"""云台帧编码的离线测试 —— 无硬件、无串口、**Windows 上也能跑**。

    python3 gimbal_test.py        (退出码 0 = 全过,失败会 assert 报错)

--------------------------------------------------------------------------
它回答的问题
--------------------------------------------------------------------------
「`struct.pack('>hh', dx, dy)` 到底和 C 参考实现那五行一不一样?
 校验和能不能替我验证编码?」

--------------------------------------------------------------------------
⚠️⚠️ 关于「什么算证据」—— 这个文件里有两类断言,分量完全不同
--------------------------------------------------------------------------
**① 黄金向量(真证据)**
   12 个 `(dx,dy) → 期望字节` 是**按协议手推**出来的,可以拿纸笔逐字节复核。
   它们能抓住「协议本身被理解错」(比如大端写成小端、校验和写成 XOR)。

**② C 转写基准(对照,不是自证)**
   `c_frame()` 是从 `hello_world_main.c:241-268` **逐字抄**成 Python 的。
   和它比对能证明「本模块忠实地实现了我们**抄下来的那个规格**」,
   ⚠️ 但它**抓不到抄错** —— 两边的规格是同一份。
   所以它只是对照,不是独立证据。别把它当成「已经验过了」。

**③ 校验和盲区(负面对照)**
   `test_checksum_is_blind_to_endianness` 故意构造一个**小端**帧,证明它
   **照样通过校验**、而接收端会解出一个完全错误的值。
   这条用例存在的意义是**可执行地记住为什么不能用校验和当通过条件**。

--------------------------------------------------------------------------
已实测的数字(2026-09-27,本机)
--------------------------------------------------------------------------
* `pack('>hh')` vs C 转写:12 个用例(含 int16 两端)**0 处不符**
* 值域 [-100,100]² 全部 40401 组:往返 + 校验和 **0 处不符**
* 单比特翻转检出率 **100%**(1,292,832 用例)
  ⇒ 校验和**对付位错完美、对付顺序错全盲**
* payload 命中 0x0A 1.46% / 0x0D 1.45% / 0x11 1.44% / 0x13 1.43%
  ⇒ 串口必须是 raw 模式,否则 OPOST 会改写 0x0A、IXON 会吞掉 0x11/0x13
"""

import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ⚠️ Windows 控制台默认是 GBK,而本项目习惯在输出里用 ⚠️ / ✅ / ⇒ 这类字符 ——
# **它们不在 GBK 里**。一旦打不出来,测试会以 UnicodeEncodeError **崩掉**,
# 而那是比断言失败更坏的结果:它连「哪条用例挂了」都说不出来,还会被误读成
# 「编码有问题」。这里把 stdout 的编码错误降级成替换,保证任何情况下都能把
# 结果和失败原因打出来。(打印用的字符串本身也已经避开了这些字符,这条是兜底。)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(errors="replace")

from gimbal_proto import (CMD_MAX, CMD_MIN, FRAME_LEN, HEADER, clamp_cmd100,
                          decode_frame, encode_frame, frame_hex, slew_toward)

# ── ① 黄金向量:按协议手推,可纸笔复核 ────────────────────────────────
# 每条都写清推导,免得以后有人「顺手改一下让它过」。
GOLDEN = [
    # (dx, dy, 期望字节,                                推导)
    (0, 0,       "AA 00 00 00 00 00"),   # 零速停车:全 0(⚠️ **不是**「回中」)
    (-100, 0,    "AA FF 9C 00 00 9B"),   # dx=-100=0xFF9C → FF+9C+00+00=0x19B → 9B
    (100, -100,  "AA 00 64 FF 9C FF"),   # 64+9C=0xFF 的补:00+64+FF+9C=0x1FF → FF
    (0, 100,     "AA 00 00 00 64 64"),   # 0x64 = 100
    (-1, -1,     "AA FF FF FF FF FC"),   # FF×4 = 1020 = 0x3FC → FC
    (100, 100,   "AA 00 64 00 64 C8"),   # 200 = 0xC8
    (-100, -100, "AA FF 9C FF 9C 36"),   # 255+156+255+156 = 822 = 0x336 → 36
    (255, 10,    "AA 00 FF 00 0A 09"),   # ⚠️ 255 合法(是 int16)但**超出 cmd100 值域**
    (19, 17,     "AA 00 13 00 11 24"),   # 0x13+0x11 = 36 = 0x24
    (-200, 200,  "AA FF 38 00 C8 FF"),   # ⚠️ 同上,超出值域但仍在 int16 内
    (32767, 32767,   "AA 7F FF 7F FF FC"),  # int16 上端:127+255 各两份
    (-32768, -32768, "AA 80 00 80 00 00"),  # int16 下端:128+128 = 256 → 00
]


# ── ② C 转写基准:逐字抄自 hello_world_main.c:241-268 ──────────────────
def c_frame(dx, dy):
    """参考实现的五行,**逐字转写**。这是对照基准,不是独立证据(见 docstring)。"""
    f = [0] * 6
    f[0] = 0xAA
    f[1] = (dx >> 8) & 0xFF
    f[2] = dx & 0xFF
    f[3] = (dy >> 8) & 0xFF
    f[4] = dy & 0xFF
    f[5] = (f[1] + f[2] + f[3] + f[4]) & 0xFF
    return bytes(f)


def test_golden_vectors():
    """① 逐字节断言。**这是本文件唯一能抓住「协议被理解错」的测试。**"""
    for dx, dy, want in GOLDEN:
        got = frame_hex(encode_frame(dx, dy))
        assert got == want, f"encode_frame({dx},{dy}) = {got},期望 {want}"
        # 反向也钉一遍:decode 必须把同一串字节解回原值
        back = decode_frame(bytes.fromhex(want.replace(" ", "")))
        assert back == (dx, dy), f"decode({want}) = {back},期望 ({dx},{dy})"
    print(f"  golden_vectors OK ({len(GOLDEN)} 条逐字节,含 int16 两端)")


def test_c_transcription_baseline():
    """② 与 C 转写逐字节比对。

    ⚠️ 值域外也要比:`(255,10)`/`(-200,200)`/`(32767,32767)` 这些
       **超出了 cmd100 值域但仍在 int16 内**。编码层不钳位(钳位是入口的事),
       所以它必须能正确编码整个 int16 域 —— 这条也顺带把「编码层偷偷钳位」挡掉。
    """
    cases = [(dx, dy) for dx, dy, _ in GOLDEN]
    # 值域全量(40401 组)—— 这才是真正跑得动的穷举
    cases += [(dx, dy) for dx in range(CMD_MIN, CMD_MAX + 1)
                       for dy in range(CMD_MIN, CMD_MAX + 1)]
    # 边界外的采样,把 int16 两端也覆盖上
    edge = [-32768, -32767, -4096, -1000, -101, 101, 1000, 4096, 32766, 32767]
    cases += [(dx, dy) for dx in edge for dy in edge]

    bad = [c for c in cases if encode_frame(*c) != c_frame(*c)]
    assert not bad, f"{len(bad)} 处与 C 转写不符,前 5 个: {bad[:5]}"
    print(f"  c_transcription OK ({len(cases)} 组逐字节与 C 转写一致)")


def test_roundtrip():
    """往返:encode → decode 必须还原。值域全量。"""
    bad = []
    for dx in range(CMD_MIN, CMD_MAX + 1):
        for dy in range(CMD_MIN, CMD_MAX + 1):
            if decode_frame(encode_frame(dx, dy)) != (dx, dy):
                bad.append((dx, dy))
    assert not bad, f"{len(bad)} 组往返失败,前 5 个: {bad[:5]}"
    total = (CMD_MAX - CMD_MIN + 1) ** 2
    print(f"  roundtrip OK (值域 {total} 组全部往返一致)")


def test_checksum_is_blind_to_endianness():
    """③ **负面对照** —— 证明「校验和通过」不等于「编码对」。

    构造一个小端帧:把大端帧的 dx/dy 四个字节两两交换。
    SUM 是四字节普通和、与顺序无关,**所以它照样通过校验** ——
    而接收端按大端解出来是个完全错误的值。

    ⇒ 这就是 `gimbal_test.py` 里**不允许**出现
      `assert checksum_ok` / `assert decode_frame(...) is not None`
      这类通过条件的原因:小端实现能在那种测试下一路绿灯。

    这条用例不验我们的代码,**它验的是「测试设计本身有没有漏洞」**。
    """
    be = encode_frame(-100, 0)
    assert frame_hex(be) == "AA FF 9C 00 00 9B"
    # 把 dx/dy 的字节序反过来(这就是「写成小端」的样子)
    le = bytes([be[0], be[2], be[1], be[4], be[3], be[5]])
    assert frame_hex(le) == "AA 9C FF 00 00 9B"

    # ★ 关键:它**通过了**校验和(所以 decode_frame 不返回 None)
    got = decode_frame(le)
    assert got is not None, "前提错了:这个帧本该通过校验和才是"
    dx, dy = got
    assert (dx, dy) != (-100, 0), "前提错了:小端帧本该解出错误的值"
    assert dx == -25345, f"解出来是 {dx},与实测记录不符"
    print(f"  checksum_blind_spot OK (小端帧 AA 9C FF 00 00 9B 通过校验,"
          f"却解出 dx={dx} —— 校验和抓不到字节序错)")


def test_bit_flip_detection():
    """单比特翻转必须 100% 被查出(这条是校验和**擅长**的那一面)。

    理由:翻转第 i 字节的第 k 位 ⇒ 该字节变化 ±2^k ⇒ 四字节普通和变化 ±2^k,
    而 2^k mod 256 ≠ 0 (k<8),所以校验和必然变。帧头那一位更直接。
    """
    frames = [encode_frame(dx, dy) for dx, dy, _ in GOLDEN]
    frames += [encode_frame(dx, dy)
               for dx in range(-100, 101, 7) for dy in range(-100, 101, 11)]
    checked = missed = 0
    for fr in frames:
        for i in range(FRAME_LEN):
            for k in range(8):
                bad = bytearray(fr)
                bad[i] ^= (1 << k)
                checked += 1
                if decode_frame(bytes(bad)) is not None:
                    missed += 1
    assert missed == 0, f"{missed}/{checked} 个单比特翻转漏检"
    print(f"  bit_flip OK ({checked} 个单比特翻转全部检出)")


def test_header_collision_is_real():
    """⚠️ `0xAA` **会**出现在数据字节里 —— 协议**没有帧头同步能力**。

    这条曾经被写反过(「值域内 0xAA 不可达」,把 0x9C=156 当成了大于 0xAA=170)。
    现在的断言是穷举出来的**真实**性质,别改回去。

    机理:低字节是 `n & 0xFF`。正值 n∈[0,100] → [0x00,0x64];负值 → [0x9C,0xFF]。
    空隙 [0x65,0x9B] **不含** 0xAA=170 —— 0xAA 在负值段,对应 `-86 & 0xFF`。

    后果比「重码」本身更要紧:**接收端不能靠扫 0xAA 重新对齐**,
    所以一次短写造成的相位偏移**没有自愈手段**。见 gimbal_proc.py 的短写处理。
    """
    collided = set()
    for dx in range(CMD_MIN, CMD_MAX + 1):
        for dy in range(CMD_MIN, CMD_MAX + 1):
            if any(b == HEADER for b in encode_frame(dx, dy)[1:5]):
                collided.add((dx, dy))
    # 401 = (dx=-86 的 201 组) + (dy=-86 的 201 组) - (两者都是 -86 的 1 组)
    assert len(collided) == 401, f"实测是 401 组,这里是 {len(collided)}"
    # **当且仅当** dx 或 dy 等于 -86 ⇒ 该帧含一个假帧头。
    # (-86 是唯一一个低字节为 0xAA 的命令值:-86 & 0xFF == 0xAA)
    expected = {(dx, dy) for dx in range(CMD_MIN, CMD_MAX + 1)
                         for dy in range(CMD_MIN, CMD_MAX + 1)
                         if dx == -86 or dy == -86}
    assert collided == expected, (
        f"撞帧头的集合不等于「dx 或 dy == -86」:"
        f"差集 {sorted(collided ^ expected)[:5]}")
    assert (-20, -86) in collided
    assert frame_hex(encode_frame(-20, -86)) == "AA FF EC FF AA 94", \
        "byte4 的那个 AA 是**数据**,不是帧头 —— 这条就是本用例的全部意义"
    print(f"  header_collision OK (0xAA 可达:401/40401 组含假帧头,"
          f"全部来自 dx 或 dy == -86 → 接收端无法靠扫 0xAA 重新对齐)")

    # 顺带把危险字节比例也算出来 —— 它决定串口必须 raw(见 gimbal_proto docstring)。
    # ⚠️ 口径要写清,否则数字对不上就会被当成「测试写错了」:
    #   「**整帧**含该字节」= 6 个字节里有一个就够了(驱动改写/吞掉不看位置)。
    #   只算数据字节会漏掉一个事实:**校验和字节也是个币面字节**。
    #   躲不掉 —— 把 payload 调干净也没用,校验和那一格照样会撞上。
    frames = [encode_frame(dx, dy) for dx in range(CMD_MIN, CMD_MAX + 1)
                                    for dy in range(CMD_MIN, CMD_MAX + 1)]
    n = len(frames)
    DANGEROUS = ((0x0A, "LF"), (0x0D, "CR"), (0x11, "XON"), (0x13, "XOFF"))
    whole, payload, only_sum = {}, {}, {}
    for b, name in DANGEROUS:
        whole[b] = 100.0 * sum(1 for f in frames if b in f) / n
        payload[b] = 100.0 * sum(1 for f in frames if b in f[1:5]) / n
        only_sum[b] = 100.0 * sum(1 for f in frames if f[5] == b) / n
        assert 1.3 < whole[b] < 1.6, \
            f"0x{b:02X}({name}) 整帧出现率 {whole[b]:.2f}%,与实测(约 1.4~1.5%)不符"
        assert 0.9 < payload[b] < 1.1, \
            f"0x{b:02X}({name}) 仅数据字节出现率 {payload[b]:.2f}%,与实测(约 1.0%)不符"
    print(f"  dangerous_bytes OK (整帧含危险字节:LF {whole[0x0A]:.2f}%"
          f" CR {whole[0x0D]:.2f}% XON {whole[0x11]:.2f}% XOFF {whole[0x13]:.2f}%;"
          f"其中只有 {payload[0x0A]:.2f}% 在数据字节、{only_sum[0x0A]:.2f}% 只在校验和字节"
          f" → 串口必须 raw)")


def test_clamp():
    """`clamp_cmd100` 是**唯一入口**的防线(镜像 pwm_proc._recv_latest)。"""
    assert clamp_cmd100(0) == 0
    assert clamp_cmd100(-100) == CMD_MIN and clamp_cmd100(100) == CMD_MAX
    assert clamp_cmd100(5000) == CMD_MAX
    assert clamp_cmd100(-5000) == CMD_MIN
    assert clamp_cmd100(100.7) == 100 and clamp_cmd100(-100.7) == -100
    assert clamp_cmd100("42") == 42, "JSON 里数字可能是字符串,别直接当垃圾扔掉"
    # ⚠️ ±inf 钳到 ±100,**不是**回 0 —— 它比 100 还大,回 0 等于把「推到底」
    #    悄悄改成「零速停车」,方向正好相反。而 `int(float('inf'))` 抛 OverflowError
    #    (不在 TypeError/ValueError 里)—— 这条用例抓到过那个真 bug。
    assert clamp_cmd100(float("inf")) == CMD_MAX
    assert clamp_cmd100(float("-inf")) == CMD_MIN
    # 脏输入不许把它搞崩(实时链路:一个坏包不该让循环停摆)
    for junk in (None, "abc", float("nan"), [], {}, object()):
        assert clamp_cmd100(junk) == 0, f"clamp_cmd100({junk!r}) 应回 0"
    print("  clamp OK (含 ±inf/NaN/None/字符串/容器 —— 都不许抛异常)")


def test_encode_rejects_out_of_int16():
    """超出 int16 必须**响亮地抛**,不许静默回绕。

    回绕会把「往右到底」变成「往左到底」—— 是那种不报错、只是方向反了的坏。
    """
    for dx, dy in ((40000, 0), (0, -40000), (32768, 32768), (-32769, 0)):
        try:
            encode_frame(dx, dy)
        except ValueError:
            continue
        raise AssertionError(f"encode_frame({dx},{dy}) 本该抛 ValueError")
    print("  reject_out_of_int16 OK (超 int16 抛 ValueError,不回绕)")


def test_decode_rejects_garbage():
    """被动监听要拿任意窗口去试,所以坏输入返回 None 而不是抛异常。"""
    assert decode_frame(None) is None
    assert decode_frame(b"") is None
    assert decode_frame(b"\xaa\x00\x00\x00\x00") is None            # 短一字节
    assert decode_frame(bytes(7)) is None                           # 长一字节
    assert decode_frame(b"\xbb\x00\x00\x00\x00\x00") is None        # 帧头不对
    assert decode_frame(b"\xaa\x00\x00\x00\x00\x01") is None        # 校验和不对
    print("  decode_garbage OK (长度/帧头/校验和 三类坏输入都返回 None)")


def test_slew_toward_zero_rate_is_passthrough():
    """rate<=0 = 斜坡关闭 ⇒ 直通。这是 A/B 对照的入口,行为必须与加斜坡之前**逐点相同**。"""
    for cur, tgt in ((0, 100), (0, -100), (37, -80), (-100, 100)):
        assert slew_toward(cur, tgt, 0, 0.02) == tgt, (cur, tgt)
        assert slew_toward(cur, tgt, -1, 0.02) == tgt, (cur, tgt)
    print("  slew_passthrough OK (rate<=0 直通,与无斜坡逐点相同)")


def test_slew_toward_limits_step():
    """每步不许超过 rate*dt —— 这正是「丢步/堵转」的防线。"""
    rate, dt = 250.0, 0.02                 # 协议建议值
    step = int(rate * dt)                  # = 5
    assert step == 5, step                 # 对着协议的「每 20ms ±5」核一遍

    cur = 0
    seen = [cur]
    for _ in range(40):                    # 40 步 * 5 = 200 单位,足够从 0 到 100 再回来
        cur = slew_toward(cur, 100, rate, dt)
        seen.append(cur)
    for a, b in zip(seen, seen[1:]):
        assert b - a <= step, f"单步跳了 {b - a} (> {step}):{a} -> {b}"
    assert seen[-1] == 100, seen[-1]
    print("  slew_limit OK (单步 <= rate*dt = 5,与协议的「每 20ms ±5」一致)")


def test_slew_toward_snaps_and_does_not_oscillate():
    """到位必须**精确落到 target**,否则会在目标附近永远差 1 地抖。"""
    rate, dt = 250.0, 0.02
    cur = 97
    out = []
    for _ in range(10):
        cur = slew_toward(cur, 100, rate, dt)
        out.append(cur)
    assert out[0] == 100, out               # |100-97|=3 <= 5 ⇒ 一步到位
    assert all(v == 100 for v in out), out  # 之后**一直**是 100,不许振荡
    print("  slew_snap OK (到位精确落 target,不振荡:%s)" % out[:4])


def test_slew_toward_zero_and_no_time():
    """两条边界:`dt<=0` 不动;`cur == target` 恒等。"""
    assert slew_toward(50, 100, 250.0, 0) == 50
    assert slew_toward(50, 100, 250.0, -1) == 50
    assert slew_toward(0, 0, 250.0, 0.02) == 0
    assert slew_toward(-100, -100, 250.0, 0.02) == -100
    # 速率太低、这一帧连 1 个单位都走不满 ⇒ 不动(**不**四舍五入到 1)
    assert slew_toward(0, 100, 10.0, 0.02) == 0
    print("  slew_edge OK (dt<=0 不动 / cur==target 恒等 / 走不满 1 个单位就不动)")


def test_slew_toward_crosses_zero_monotonically():
    """从 -100 走到 +100 **必须穿过 0 且单调**,不许跳过 0。

    跳过 0 = 反向时直接阶跃过零,那正是要挡的那种跳变。
    """
    rate, dt = 250.0, 0.02
    cur, seen = -100, []
    for _ in range(200):
        cur = slew_toward(cur, 100, rate, dt)
        seen.append(cur)
        if cur == 100:
            break
    assert seen[-1] == 100, seen[-1]
    assert 0 in seen, "必须经过 0"
    assert seen == sorted(seen), "必须单调不回头"
    assert len(seen) < 200, "必须在有限步内到位"
    print("  slew_cross_zero OK (单调穿过 0,-100→+100 用 %d 步)" % len(seen))


def test_slew_clamped_dt_is_the_caller_s_job():
    """⚠️ 这条是把「dt 由调用方钳」这个分工**写成可执行的断言**。

    循环卡了 1 秒之后,如果调用方把 dt=1.0 原样传进来,max_delta 会 = 250 ⇒ 一帧
    从 0 跳到 100,**斜坡在最需要它的那一刻失效**。本函数不做钳位,所以这里断言的是
    「不钳会怎样」和「钳了就好」两件事 —— 免得有人以为函数据自己会兜底。
    """
    rate = 250.0
    assert slew_toward(0, 100, rate, 1.0) == 100        # 不钳 ⇒ 一帧跳满(这就是那个坑)
    assert slew_toward(0, 100, rate, 0.04) == 10        # 钳到 2*period(0.02) ⇒ 只走 10
    print("  slew_dt_contract OK (不钳 dt 会一帧跳满 —— 所以钳位必须是调用方的责任)")


def main():
    print("gimbal_test:")
    test_golden_vectors()
    test_c_transcription_baseline()
    test_roundtrip()
    test_checksum_is_blind_to_endianness()
    test_bit_flip_detection()
    test_header_collision_is_real()
    test_clamp()
    test_encode_rejects_out_of_int16()
    test_decode_rejects_garbage()
    test_slew_toward_zero_rate_is_passthrough()
    test_slew_toward_limits_step()
    test_slew_toward_snaps_and_does_not_oscillate()
    test_slew_toward_zero_and_no_time()
    test_slew_toward_crosses_zero_monotonically()
    test_slew_clamped_dt_is_the_caller_s_job()
    print("\n[OK] 编码层 + 斜坡层全部通过")
    print("[!] 这只证明了**编码和数学**。串口那一层(节点名/波特率/raw 位/短写)"
          "\n   本机测不了 —— 板端回环 + 被动监听是唯一证据(见 gimbal_wiring.md)。")


if __name__ == "__main__":
    main()
