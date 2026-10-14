"""Persistent patrol definitions. Saving a task never starts navigation."""
import copy
import json
import math
import os
import tempfile
import threading
from pathlib import Path
from map_store import checked_name


def validate_task(data):
    if not isinstance(data, dict):
        raise ValueError('巡检任务格式无效')
    name, map_name = checked_name(data.get('name')), checked_name(data.get('map'))
    cycles = data.get('cycles', 1)
    if type(cycles) is not int or not 1 <= cycles <= 100:
        raise ValueError('循环次数必须是 1–100 的整数')
    points = data.get('waypoints')
    if not isinstance(points, list) or not 1 <= len(points) <= 50:
        raise ValueError('任务需要 1–50 个巡检点')
    cleaned = []
    for i, point in enumerate(points):
        if not isinstance(point, dict):
            raise ValueError(f'第 {i+1} 个巡检点格式无效')
        values = {}
        for key in ('x', 'y', 'yaw', 'dwell_s'):
            value = point.get(key, 0)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f'第 {i+1} 个巡检点 {key} 必须是有限数字')
            values[key] = float(value)
        if not 0 <= values['dwell_s'] <= 3600:
            raise ValueError('每点停留时间必须在 0–3600 秒之间')
        if max(abs(values['x']), abs(values['y'])) > 10000:
            raise ValueError('巡检点坐标超出允许范围')
        values['yaw'] = math.atan2(math.sin(values['yaw']), math.cos(values['yaw']))
        cleaned.append(values)
    return dict(name=name, map=map_name, cycles=cycles, waypoints=cleaned)


class PatrolStore:
    def __init__(self, directory, maps):
        self.root = Path(directory).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.maps = maps
        self.lock = threading.RLock()

    def path(self, name):
        p = self.root/(checked_name(name)+'.json')
        if p.resolve().parent != self.root:
            raise ValueError('任务路径不合法')
        return p

    def get(self, name):
        with self.lock:
            p = self.path(name)
            if not p.is_file():
                raise ValueError('巡检任务不存在')
            saved = json.loads(p.read_text(encoding='utf-8'))
            task = validate_task(saved)
            task['revision'] = saved['revision']
            return task

    def list(self):
        result = []
        with self.lock:
            for p in sorted(self.root.glob('*.json')):
                try:
                    task = self.get(p.stem)
                    result.append(dict(name=task['name'], map=task['map'],
                        revision=task['revision'], points=len(task['waypoints']), cycles=task['cycles']))
                except (ValueError, KeyError, OSError):
                    continue
        return result

    def save(self, data):
        task = validate_task(data)
        self.maps.path(task['map'])
        with self.lock:
            p = self.path(task['name'])
            expected = data.get('revision', 0)
            if type(expected) is not int or expected < 0:
                raise ValueError('任务版本无效，请重新载入任务')
            old = self.get(task['name']) if p.exists() else None
            current = old['revision'] if old else 0
            if expected != current:
                raise ValueError('任务已被其他页面修改，请重新载入后再编辑')
            task['revision'] = current+1
            fd, temp = tempfile.mkstemp(prefix='.saving-', dir=self.root)
            try:
                with os.fdopen(fd, 'w', encoding='utf-8') as f:
                    json.dump(task, f, ensure_ascii=False, indent=2, allow_nan=False)
                    f.write('\n'); f.flush(); os.fsync(f.fileno())
                os.replace(temp, p)
            finally:
                if os.path.exists(temp): os.unlink(temp)
            return copy.deepcopy(task)
