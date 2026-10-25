"""Bound initial localization updates before allowing a path or motion."""
import math


class StaticPoseSettle:
    def __init__(self):
        self.active = False
        self.ready = True
        self.failed = False
        self.samples = self.stable = 0
        self.last = None
        self.after_stamp = 0.0

    def start(self, after_stamp=0.0):
        self.__init__()
        self.active = True
        self.ready = False
        self.after_stamp = after_stamp

    @property
    def needs_update(self):
        return self.active and not self.ready and not self.failed

    def observe(self, x, y, angle, stamp):
        if not self.needs_update:
            return
        if not all(math.isfinite(v) for v in (x, y, angle, stamp)):
            self.stable = 0
            return
        if stamp < self.after_stamp:
            return
        if self.last and stamp <= self.last[3]:
            return
        self.samples += 1
        if self.last:
            dx, dy = x-self.last[0], y-self.last[1]
            da = math.atan2(math.sin(angle-self.last[2]), math.cos(angle-self.last[2]))
            self.stable = self.stable+1 if math.hypot(dx, dy) <= .02 and abs(da) <= math.radians(2) else 0
        self.last = (x, y, angle, stamp)
        self.ready = self.samples >= 4 and self.stable >= 3
        self.failed = self.samples >= 16 and not self.ready

    def message(self):
        if self.failed:
            return '静止定位仍在变化，请核对实际位置和车头后重新设置'
        return f'等待静止定位稳定（已更新{self.samples}次，连续稳定{self.stable}/3）'
