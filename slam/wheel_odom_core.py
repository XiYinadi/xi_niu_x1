"""Encoder-only odometry math and chronological quadrature decoding (no ROS/GPIO)."""
import math


def sample_fault(previous, current):
    """Reject large phase losses; small skipped transitions remain diagnostics.

    This is a health bound, not proof that polling never misses full cycles.
    Each 20ms report permits at most 1% invalid transitions (floor 4 for short
    scheduler gaps). Counts are never fabricated to compensate missing phases.
    """
    dt = current['elapsed'] - previous['elapsed']
    if not 0 < dt < .1:
        return 'invalid encoder sample interval'
    for i in range(2):
        bad = current['invalid'][i] - previous['invalid'][i]
        edges = sum(current['changes'][i*2:i*2+2]) - sum(previous['changes'][i*2:i*2+2])
        if bad < 0 or edges < 0 or bad > max(4, edges * .01):
            return 'encoder phase loss exceeds quality limit'
    return None


class Quadrature:
    TABLE = (0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0)

    def __init__(self, a, b):
        self.state = (int(a) << 1) | int(b)
        self.count = 0
        self.last_ns = -1

    def edge(self, timestamp_ns, channel, level):
        if timestamp_ns <= self.last_ns:
            raise ValueError('encoder timestamp not strictly increasing')
        self.last_ns = timestamp_ns
        mask = 2 if channel == 0 else 1
        new = (self.state | mask) if level else (self.state & ~mask)
        if new == self.state:
            raise ValueError('encoder repeated edge / missing opposite edge')
        self.count += self.TABLE[(self.state << 2) | new]
        self.state = new


def integrate(pose, left_m, right_m, track_m):
    """Exact planar arc from two independently measured rear-wheel distances.

    track_m is LEFT/RIGHT wheel spacing, not front/rear wheelbase. Also valid
    for a steered chassis with independent rolling rear wheels (no slip).
    """
    if not all(math.isfinite(v) for v in (*pose, left_m, right_m, track_m)) or track_m <= 0:
        raise ValueError('finite pose/distances and positive wheel track required')
    x, y, yaw = pose
    distance = (left_m + right_m) / 2
    angle = (right_m - left_m) / track_m
    half = angle / 2
    scale = math.sin(half) / half if abs(half) > 1e-9 else 1.0
    x += distance * scale * math.cos(yaw + half)
    y += distance * scale * math.sin(yaw + half)
    return x, y, math.atan2(math.sin(yaw + angle), math.cos(yaw + angle))
