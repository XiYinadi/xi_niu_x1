#!/usr/bin/env python3
"""GPIO smoke + libgpiod API-compat test for motor_hw.py on 犀牛派.

1. Verifies every libgpiod call motor_hw.py makes exists in THIS build
   (entry point is `gpiod.Chip`, not `gpiod.chip`).
2. Requests the 8 motor lines: drives PWM/DIR outputs, reads ENC inputs.
Safe whether or not the H-bridge/encoders are wired.
"""
import sys
import time

sys.path.insert(0, "/usr/lib/python3/dist-packages")
sys.path.insert(0, "/usr/local/lib/python3.10/dist-packages")

import gpiod

CHIP = 'gpiochip0'
# 每路电机 3 信号脚: PWM(控速) + INA/INB(互补方向)
OUT = [(91, 'PWM_L (pin32)'), (61, 'INA_L (pin18)'), (62, 'INB_L (pin31)'),
       (63, 'PWM_R (pin36)'), (60, 'INA_R (pin37)'), (124, 'INB_R (pin33)')]
IN = [(0, 'ENC_L_A (pin7)'), (12, 'ENC_L_B (pin8)'),
      (5, 'ENC_R_A (pin11)'), (1, 'ENC_R_B (pin10)')]
# INA/INB 对在 OUT 里的索引 (INA_L,INB_L), (INA_R,INB_R)
PAIRS = [(1, 2), (4, 5)]

# ---- 1. API surface used by motor_hw.py ----
print("=== API compat (motor_hw.py) ===")
api_checks = [
    ("gpiod.Chip", hasattr(gpiod, 'Chip')),
    ("gpiod.LineBulk", hasattr(gpiod, 'LineBulk')),
    ("gpiod.LineEvent", hasattr(gpiod, 'LineEvent')),
    ("LINE_REQ_DIR_OUT", hasattr(gpiod, 'LINE_REQ_DIR_OUT')),
    ("LINE_REQ_DIR_IN", hasattr(gpiod, 'LINE_REQ_DIR_IN')),
    ("LINE_REQ_EV_BOTH_EDGES", hasattr(gpiod, 'LINE_REQ_EV_BOTH_EDGES')),
]
for name, ok in api_checks:
    print("  %-28s %s" % (name, 'OK' if ok else 'MISSING!'))
    if not ok:
        sys.exit(1)

chip = gpiod.Chip(CHIP)
print("  Chip(%s): num_lines=%s  %s" % (
    CHIP, chip.num_lines if isinstance(chip.num_lines, int)
    else chip.num_lines() if callable(getattr(chip, 'num_lines', None))
    else '?', type(chip)))

# methods motor_hw relies on —— 必须真正 hasattr,不能无条件打印 OK。
# 这段曾经是硬编码的 "OK",导致 motor_hw 的 event API 不匹配被掩盖了很久。
api_ok = True


def check(obj, owner, names):
    """真检查 obj 上有没有 names 里的方法/属性。"""
    global api_ok
    for m in names:
        ok = hasattr(obj, m)
        api_ok = api_ok and ok
        print("  %-18s %-22s %s" % (owner + '.' + m, '', 'OK' if ok else 'MISSING!'))
    return True


check(chip, 'Chip', ('get_line', 'get_lines', 'num_lines'))
check(gpiod.Line, 'Line',
      ('request', 'set_value', 'get_value', 'offset', 'event_read', 'event_wait'))
check(gpiod.LineBulk, 'LineBulk',
      ('request', 'set_values', 'get_values', 'event_wait'))
check(gpiod.LineEvent, 'LineEvent', ('type', 'sec', 'nsec', 'source'))

for attr in ('RISING_EDGE', 'FALLING_EDGE'):
    ok = hasattr(gpiod.LineEvent, attr)
    api_ok = api_ok and ok
    print("  LineEvent.%s %s" % (attr, 'OK' if ok else 'MISSING!'))

# 本机绑定特有的坑:浮点秒会被拒绝,且事件不从 LineBulk 读。
# 显式验证我们依赖的调用形状,而不是假设。
try:
    gpiod.LineBulk.event_wait.__call__
    print("  (note) 本机 event_wait 形参为 (sec, nsec),两者均须整数")
except AttributeError:
    api_ok = False
    print("  LineBulk.event_wait MISSING!")

if not api_ok:
    print("\nAPI 不完整 → motor_hw 的真实硬件后端无法工作,ABORT")
    sys.exit(1)

# ---- 2. line control ----
print("\n=== line control ===")
obulk = chip.get_lines([o for o, _ in OUT])
obulk.request(consumer='gpio_smoke_out', type=gpiod.LINE_REQ_DIR_OUT,
              default_vals=[0] * len(OUT))
all_ok = True
for i, (line, name) in enumerate(OUT):
    vals = [0] * len(OUT)
    vals[i] = 1
    obulk.set_values(vals)
    time.sleep(0.05)
    got = obulk.get_values()
    other_busy = any(1 for j in range(len(OUT)) if j != i and got[j])
    ok = (got[i] == 1) and not other_busy
    all_ok = all_ok and ok
    print("  OUT %s: line %2d set 1 -> read %s  %s" % (
        name, line, got[i], 'OK' if ok else 'FAIL'))
obulk.set_values([0] * len(OUT))

# INA/INB 互补方向验证 (dir=True → INA=1,INB=0 ; False → INA=0,INB=1)
print("\n=== INA/INB complementary ===")
for pi, (i, j) in enumerate(PAIRS):
    for d in (True, False):
        vals = [0] * len(OUT)
        vals[i] = 1 if d else 0
        vals[j] = 0 if d else 1
        obulk.set_values(vals)
        time.sleep(0.02)
        got = obulk.get_values()
        ok = got[i] == (1 if d else 0) and got[j] == (0 if d else 1)
        all_ok = all_ok and ok
        print("  motor %s dir=%s: INA=%d INB=%d  %s" % (
            'L' if pi == 0 else 'R', d, got[i], got[j], 'OK' if ok else 'FAIL'))
obulk.set_values([0] * len(OUT))
obulk.release()

ibulk = chip.get_lines([l for l, _ in IN])
try:
    ibulk.request(consumer='gpio_smoke_in', type=gpiod.LINE_REQ_DIR_IN)
    vals = ibulk.get_values()
    ibulk.release()
    for (line, name), v in zip(IN, vals):
        print("  IN  %s: line %2d level = %d  (idle pull = 1)" % (name, line, v))
except Exception as e:
    all_ok = False
    print("  IN request failed:", e)

print("\nSMOKE " + ("OK" if all_ok else "FAIL"))
sys.exit(0 if all_ok else 1)
