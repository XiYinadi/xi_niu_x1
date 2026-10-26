"""Versioned map storage. No ROS imports; map publication never overwrites a map."""
import json
import math
import os
import re
import shutil
import tempfile
import yaml
import time
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

    # ── 2026-10-26: 语义区域（客厅/卧室/厨房…），按地图存 regions.json ──
    REGION_COLORS = ('#4f9dff', '#f3a36d', '#7bd88f', '#c792ea', '#ffd166', '#ef6f6c')

    def regions(self, name):
        path = self.path(name).parent / 'regions.json'
        if not path.is_file():
            return []
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return []
        return data.get('regions') if isinstance(data, dict) and isinstance(data.get('regions'), list) else []

    def save_regions(self, name, regions):
        self.path(name)  # 地图必须存在
        if not isinstance(regions, list) or len(regions) > 40:
            raise ValueError('区域数量须在 0–40 之间')
        cleaned, names = [], set()
        for item in regions:
            if not isinstance(item, dict):
                raise ValueError('区域格式无效')
            title = item.get('name')
            if not isinstance(title, str) or not 1 <= len(title.strip()) <= 20:
                raise ValueError('区域名称须为 1–20 个字')
            title = title.strip()
            if title in names:
                raise ValueError('区域名称不能重复：'+title)
            names.add(title)
            color = item.get('color')
            if not (isinstance(color, str) and re.fullmatch(r'#[0-9a-fA-F]{6}', color)):
                color = self.REGION_COLORS[len(cleaned) % len(self.REGION_COLORS)]
            points = item.get('points')
            if not isinstance(points, list) or not 3 <= len(points) <= 200:
                raise ValueError('区域至少 3 个、最多 200 个顶点')
            poly = []
            for p in points:
                if (not isinstance(p, (list, tuple)) or len(p) != 2
                        or not all(type(v) in (int, float) and math.isfinite(v) for v in p)):
                    raise ValueError('区域坐标无效')
                poly.append([round(float(p[0]), 4), round(float(p[1]), 4)])
            area = abs(sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(poly, poly[1:]+poly[:1])))/2
            if area < 0.01:
                raise ValueError('区域面积过小：'+title)
            cleaned.append(dict(name=title, color=color, points=poly, area_m2=round(area, 3)))
        path = self.path(name).parent / 'regions.json'
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(dict(map=name, regions=cleaned), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        os.replace(tmp, path)
        return cleaned

    def official(self, name):
        return (self.path(name).parent / '.official').is_file()

    def set_official(self, name, official=True):
        self.path(name)
        marker = self.path(name).parent / '.official'
        if official:
            marker.write_text('1\n', encoding='utf-8')
        elif marker.exists():
            marker.unlink()
        return self.official(name)

    def rename(self, name, new_name):
        folder = self.path(name).parent
        new_name = checked_name(new_name)
        if new_name == name:
            return new_name
        target = self.root / new_name
        if target.exists():
            raise ValueError('已有同名地图：' + new_name)
        shutil.move(str(folder), str(target))
        return new_name

    def info(self, name):
        path = self.path(name)
        try:
            meta = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        except (OSError, yaml.YAMLError):
            meta = {}
        width = height = None
        image = path.parent / str(meta.get('image', ''))
        if image.is_file():
            try:
                head = image.read_bytes()[:64].split()
                width, height = int(head[1]), int(head[2])
            except (OSError, IndexError, ValueError):
                pass
        try:
            size = sum(f.stat().st_size for f in path.parent.iterdir() if f.is_file())
            updated = path.stat().st_mtime
        except OSError:
            size, updated = 0, 0
        return dict(name=name, official=self.official(name),
                    resolution=meta.get('resolution'), origin=meta.get('origin'),
                    width=width, height=height, size_bytes=size, updated=updated,
                    regions=len(self.regions(name)))

    def inventory(self):
        rows = []
        for name in self.list():
            try:
                rows.append(self.info(name))
            except (OSError, ValueError):
                continue
        return rows

    def delete(self, name):
        """移到 .trash/，不真删；调用方负责拒绝正在使用的地图。"""
        folder = self.path(name).parent
        trash = self.root / '.trash'
        trash.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d-%H%M%S')
        target = trash / (name + '-' + stamp)
        n = 1
        while target.exists():
            target = trash / (name + '-' + stamp + '-' + str(n))
            n += 1
        shutil.move(str(folder), str(target))
        return target.name

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
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.19\n', encoding='utf-8')
            os.rename(temp, target)
        finally:
            if temp.exists():
                shutil.rmtree(temp)
        return self.path(name)
