"""Versioned map storage. No ROS imports; map publication never overwrites a map."""
import json
import math
import os
import re
import shutil
import tempfile
from pathlib import Path


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def checked_name(name):
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', name):
        raise ValueError('地图名只能用 1–64 个英文字母、数字、下划线或短横线')
    return name


class MapStore:
    def __init__(self, directory):
        self.root = Path(directory).resolve()
        self.root.mkdir(parents=True, exist_ok=True)

    def list(self):
        return [p.name for p in sorted(self.root.iterdir())
                if not p.name.startswith('.') and p.is_dir() and (p/'map.yaml').is_file()]

    def path(self, name):
        path = self.root / checked_name(name) / 'map.yaml'
        if not path.is_file() or path.resolve().parent.parent != self.root:
            raise ValueError('地图不存在或路径不合法')
        return path

    def save(self, msg, name):
        name = checked_name(name)
        w, h = msg.info.width, msg.info.height
        resolution = msg.info.resolution
        if w <= 0 or h <= 0 or len(msg.data) != w*h:
            raise ValueError('地图尺寸或栅格数据无效')
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('地图分辨率无效')
        if not any(v == 0 for v in msg.data) or not any(v >= 65 for v in msg.data):
            raise ValueError('地图没有同时包含自由空间和障碍物，请先完成建图')
        origin = [msg.info.origin.position.x, msg.info.origin.position.y,
                  yaw(msg.info.origin.orientation)]
        if not all(math.isfinite(v) for v in origin):
            raise ValueError('地图原点无效')
        target = self.root / name
        if target.exists():
            raise ValueError('地图名称已存在，请换一个名称，原地图不会被覆盖')
        temp = Path(tempfile.mkdtemp(prefix='.saving-', dir=self.root))
        try:
            # OccupancyGrid starts at bottom left; PGM starts at top left.
            pixels = bytearray()
            for row in range(h-1, -1, -1):
                for value in msg.data[row*w:(row+1)*w]:
                    pixels.append(205 if value < 0 else 254 - int(value)*254//100)
            (temp/'map.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode()+pixels)
            (temp/'map.yaml').write_text(
                'image: map.pgm\nmode: trinary\n'
                f'resolution: {resolution:.10g}\norigin: {json.dumps(origin)}\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
            os.rename(temp, target)
        finally:
            if temp.exists():
                shutil.rmtree(temp)
        return self.path(name)
