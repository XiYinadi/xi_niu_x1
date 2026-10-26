"""Copy-only occupancy-map editing. No ROS, socket control or actuator imports."""
import base64
from collections import OrderedDict
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import secrets
import shutil
import tempfile
import threading
import time

from flask import Response, jsonify, request, send_file
from PIL import Image, ImageDraw
import yaml

MAX_CELLS = 4_000_000
UNKNOWN = 205


def checked_name(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', value):
        raise ValueError('地图名须为 1–64 个英文字母、数字、下划线或短横线')
    return value


def geometry(width, height, metadata):
    if type(width) is not int or type(height) is not int or width < 1 or height < 1 or width*height > MAX_CELLS:
        raise ValueError('地图尺寸无效或超过 400 万栅格')
    resolution, origin = metadata.get('resolution'), metadata.get('origin')
    if type(resolution) not in (int, float) or not math.isfinite(resolution) or not .005 <= resolution <= 1:
        raise ValueError('地图分辨率须在 0.005–1 米之间')
    if not isinstance(origin, (list, tuple)) or len(origin) != 3 or not all(type(v) in (int, float) and math.isfinite(v) for v in origin):
        raise ValueError('地图原点无效')
    if metadata.get('negate', 0) != 0 or metadata.get('mode', 'trinary') != 'trinary':
        raise ValueError('当前支持 negate=0 的 trinary 地图')
    occupied, free = metadata.get('occupied_thresh', .65), metadata.get('free_thresh', .19)
    if not all(type(v) in (int, float) and math.isfinite(v) for v in (occupied, free)) or not 0 < free < occupied < 1 or occupied <= 50/255:
        raise ValueError('地图占用阈值无效')
    # Gray 205 must remain UNKNOWN when Nav2 reloads the exported map.
    return dict(image='map.pgm', mode='trinary', resolution=float(resolution),
                origin=list(origin), negate=0, occupied_thresh=float(occupied), free_thresh=min(float(free), .19))


def read_pgm(blob):
    """P5 parser; never eat a whitespace-valued first pixel after the header."""
    pos, tokens = 0, []
    while len(tokens) < 4:
        while pos < len(blob) and blob[pos] in b' \t\r\n': pos += 1
        if pos < len(blob) and blob[pos] == 35:
            end = blob.find(b'\n', pos)
            if end < 0: raise ValueError('PGM 注释不完整')
            pos = end+1
            continue
        start = pos
        while pos < len(blob) and blob[pos] not in b' \t\r\n': pos += 1
        if pos == start: raise ValueError('PGM 文件头不完整')
        tokens.append(blob[start:pos])
    if tokens[0] != b'P5' or tokens[3] != b'255': raise ValueError('当前支持 8 位 P5 PGM 地图')
    try: width, height = int(tokens[1]), int(tokens[2])
    except ValueError: raise ValueError('PGM 尺寸无效')
    if width < 1 or height < 1 or width*height > MAX_CELLS: raise ValueError('PGM 尺寸无效或过大')
    if blob[pos:pos+2] == b'\r\n': pos += 2
    elif pos < len(blob) and blob[pos] in b' \t\r\n': pos += 1
    else: raise ValueError('PGM 缺少像素分隔符')
    pixels = blob[pos:]
    if len(pixels) != width*height: raise ValueError('PGM 像素数量与尺寸不符')
    return width, height, pixels


@dataclass(frozen=True)
class Snapshot:
    width: int
    height: int
    pixels: bytes
    metadata: dict
    source: str

    @property
    def digest(self):
        return hashlib.sha256(json.dumps(self.metadata, sort_keys=True).encode()+self.pixels).hexdigest()


def from_message(msg):
    info = msg.info
    width, height = int(info.width), int(info.height)
    q = info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    metadata = geometry(width, height, dict(resolution=info.resolution,
        origin=[info.origin.position.x, info.origin.position.y, yaw]))
    if len(msg.data) != width*height or any(v < -1 or v > 100 for v in msg.data):
        raise ValueError('地图栅格数据无效')
    pixels = bytes(UNKNOWN if v < 0 else 254-int(v)*254//100
                   for row in range(height-1, -1, -1) for v in msg.data[row*width:(row+1)*width])
    return Snapshot(width, height, pixels, metadata, 'live_snapshot')


def from_saved(store, name):
    path = Path(store.path(checked_name(name))).resolve()
    meta = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(meta, dict) or not isinstance(meta.get('image'), str): raise ValueError('地图元数据无效')
    image = (path.parent/meta['image']).resolve()
    if image.parent != path.parent or not image.is_file(): raise ValueError('地图图片路径无效')
    if image.stat().st_size > MAX_CELLS+65536: raise ValueError('地图图片过大')
    width, height, pixels = read_pgm(image.read_bytes())
    return Snapshot(width, height, pixels, geometry(width, height, meta), checked_name(name))


def edit_image(source, operations):
    if not isinstance(operations, list) or len(operations) > 200: raise ValueError('最多支持 200 次标注')
    image = Image.frombytes('L', (source.width, source.height), source.pixels)
    draw = ImageDraw.Draw(image)
    for op in operations:
        if not isinstance(op, dict) or op.get('type') not in ('unknown', 'wall'): raise ValueError('标注类型无效')
        points = op.get('points')
        minimum = 3 if op['type'] == 'unknown' else 2
        if not isinstance(points, list) or not minimum <= len(points) <= 128: raise ValueError('标注点数量无效')
        for point in points:
            if not isinstance(point, (list, tuple)) or len(point) != 2 or not all(type(v) is int for v in point):
                raise ValueError('标注坐标须为整数像素')
            if not 0 <= point[0] < source.width or not 0 <= point[1] < source.height: raise ValueError('标注超出地图范围')
        points = [tuple(p) for p in points]
        if op['type'] == 'unknown':
            area = abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points, points[1:]+points[:1])))/2
            if area < .5: raise ValueError('请圈出有面积的区域')
            draw.polygon(points, fill=UNKNOWN)
        else:
            thickness = op.get('width_m', .10)
            if type(thickness) not in (int, float) or not math.isfinite(thickness) or not .025 <= thickness <= 1:
                raise ValueError('墙线宽度须在 0.025–1 米之间')
            width = max(1, math.ceil(thickness/source.metadata['resolution']))
            if width > 200: raise ValueError('墙线宽度过大')
            draw.line(points, fill=0, width=width, joint='curve')
            radius = (width-1)/2
            for x,y in points: draw.ellipse((x-radius,y-radius,x+radius,y+radius), fill=0)
    return image


def image_payload(source, image):
    buf = io.BytesIO(); image.save(buf, format='PNG')
    changed = sum(a != b for a,b in zip(source.pixels, image.tobytes()))
    return dict(image=base64.b64encode(buf.getvalue()).decode('ascii'), changed_cells=changed)


class Editor:
    def __init__(self):
        self.lock = threading.RLock()
        self.sources = OrderedDict()

    def add(self, source):
        token = secrets.token_urlsafe(24)
        with self.lock:
            self.sources[token] = (time.monotonic(), source)
            while len(self.sources) > 4: self.sources.popitem(last=False)
        return dict(token=token, source=source.source, sha256=source.digest, width=source.width,
                    height=source.height, resolution=source.metadata['resolution'], origin=source.metadata['origin'],
                    **image_payload(source, Image.frombytes('L',(source.width,source.height),source.pixels)))

    def get(self, token):
        if not isinstance(token, str): raise ValueError('请先载入地图')
        with self.lock:
            row = self.sources.get(token)
            if not row or time.monotonic()-row[0] > 1800: raise ValueError('地图快照已过期，请重新载入')
            return row[1]

    def save(self, store, source, operations, name, request_id):
        name = checked_name(name)
        if not isinstance(request_id,str) or not re.fullmatch(r'[A-Za-z0-9_-]{12,80}',request_id): raise ValueError('保存请求标识无效')
        if not operations: raise ValueError('请先完成至少一项标注')
        image = edit_image(source, operations)
        if image.tobytes() == source.pixels: raise ValueError('标注未改变任何栅格')
        # Validate using the same trinary interpretation as the Nav2 map loader.
        histogram = image.histogram(); meta = source.metadata
        occupied = sum(n for shade,n in enumerate(histogram) if (255-shade)/255 > meta['occupied_thresh'])
        free = sum(n for shade,n in enumerate(histogram) if (255-shade)/255 < meta['free_thresh'])
        if not occupied or not free: raise ValueError('清理结果须保留障碍物和可通行区域，请撤销过大的标注')
        manifest = dict(source=source.source, source_sha256=source.digest, operations=operations,
                        request_id=request_id, created_utc=datetime.now(timezone.utc).isoformat(), unknown_gray=UNKNOWN)
        root = Path(store.root).resolve(); target = root/name
        with self.lock:
            if target.exists():
                previous = target/'edits.json'
                if previous.is_file():
                    saved = json.loads(previous.read_text(encoding='utf-8'))
                    if all(saved.get(k)==manifest[k] for k in ('request_id','source_sha256','operations')): return name
                raise ValueError('地图名称已存在，请换名；原地图保留')
            draft = Path(tempfile.mkdtemp(prefix='.editing-',dir=root))
            try:
                (draft/'map.pgm').write_bytes(f'P5\n{source.width} {source.height}\n255\n'.encode()+image.tobytes())
                (draft/'map.yaml').write_text(yaml.safe_dump(meta,sort_keys=False),encoding='utf-8')
                (draft/'edits.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
                os.rename(draft,target)
            finally:
                if draft.exists():
                    if draft.resolve().parent != root or not draft.name.startswith('.editing-'): raise RuntimeError('Unexpected temporary path')
                    shutil.rmtree(draft)
        return name


def register_map_editor(app, node_getter, navigation_getter, asset_root=None):
    editor = Editor(); asset_root = Path(asset_root or Path(__file__).parent)

    def navigation():
        nav = navigation_getter()
        if nav is None or not hasattr(nav,'store'): raise ValueError('地图存储尚未就绪')
        return nav

    def body():
        if request.content_length and request.content_length > 262144: raise ValueError('标注数据过大')
        data = request.get_json(silent=True)
        if not isinstance(data,dict): raise ValueError('请求格式无效')
        return data

    def handle(fn):
        try: return jsonify(fn())
        except (ValueError,FileNotFoundError,yaml.YAMLError) as error: return jsonify(error=str(error)),400
        except Exception:
            app.logger.exception('Map editor request failed')
            return jsonify(error='地图操作失败，请检查服务日志；原图保留'),500

    @app.get('/map-editor')
    def map_editor_page(): return send_file(asset_root/'map_editor.html')

    @app.get('/map-editor/<asset>')
    def map_editor_asset(asset):
        if asset not in ('map_editor.js','map_editor.css'): return '',404
        return send_file(asset_root/asset)

    @app.get('/api/map-editor/maps')
    def map_editor_maps(): return handle(lambda:dict(maps=navigation().store.list()))

    @app.post('/api/map-editor/source')
    def map_editor_source():
        def run():
            data = body()
            if data.get('source') == 'saved': return editor.add(from_saved(navigation().store,data.get('name')))
            if data.get('source') != 'live': raise ValueError('地图来源无效')
            node = node_getter()
            if node is None: raise ValueError('机器人尚未连接')
            with node._lock:
                if node._latest_map_msg is None or not node._last_map_t or time.monotonic()-node._last_map_t >= 10:
                    raise ValueError('尚无新鲜地图，请先在建图页面启动建图')
                msg = node._latest_map_msg
            source = from_message(msg)
            return editor.add(source)
        return handle(run)

    @app.post('/api/map-editor/preview')
    def map_editor_preview():
        def run():
            data = body(); source = editor.get(data.get('token'))
            return image_payload(source,edit_image(source,data.get('operations')))
        return handle(run)

    @app.post('/api/map-editor/save')
    def map_editor_save():
        def run():
            data=body(); nav=navigation(); source=editor.get(data.get('token'))
            name=editor.save(nav.store,source,data.get('operations'),data.get('name'),data.get('request_id'))
            # Notify map selectors only; never start localization or touch driving.
            if hasattr(nav,'emit'): nav.emit()
            return dict(saved=name,maps=nav.store.list())
        return handle(run)
    @app.get('/api/map-editor/regions')
    def map_editor_regions():
        def run():
            store = navigation().store
            return dict(name=checked_name(request.args.get('name','')),
                        regions=store.regions(request.args.get('name','')))
        return handle(run)

    @app.post('/api/map-editor/regions')
    def map_editor_save_regions():
        def run():
            data = body(); nav = navigation()
            name = checked_name(data.get('name'))
            regions = nav.store.save_regions(name, data.get('regions'))
            if hasattr(nav, 'emit'):
                nav.emit()
            return dict(saved=name, regions=regions)
        return handle(run)

    @app.post('/api/map-editor/delete')
    def map_editor_delete():
        def run():
            data = body(); nav = navigation()
            name = checked_name(data.get('name'))
            if getattr(nav, 'active_map', None) == name:
                raise ValueError('这张地图正在定位中使用，请先停止定位再删除')
            bound = [t.get('name') for t in (nav.patrols.list() if hasattr(nav, 'patrols') else [])
                     if t.get('map') == name]
            if bound:
                raise ValueError('还有巡检任务绑定这张地图（' + '、'.join(bound) + '），请先删除任务')
            moved = nav.store.delete(name)
            if hasattr(nav, 'emit'):
                nav.emit()
            return dict(deleted=name, moved_to=moved, maps=nav.store.list())
        return handle(run)

    @app.get('/api/map-editor/inventory')
    def map_editor_inventory(): return handle(lambda: dict(maps=navigation().store.inventory()))

    @app.get('/api/map-editor/thumb')
    def map_editor_thumb():
        try:
            name = checked_name(request.args.get('name', ''))
            path = navigation().store.path(name)
            meta = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            image = Image.open(path.parent / str(meta.get('image', ''))).convert('L')
            image.thumbnail((240, 180))
            buf = io.BytesIO(); image.save(buf, format='PNG')
            return Response(buf.getvalue(), mimetype='image/png',
                            headers={'Cache-Control': 'no-store'})
        except (ValueError, OSError, yaml.YAMLError) as error:
            return jsonify(error=str(error)), 400

    @app.post('/api/map-editor/rename')
    def map_editor_rename():
        def run():
            data = body(); nav = navigation()
            name = checked_name(data.get('name')); to = checked_name(data.get('to'))
            if getattr(nav, 'active_map', None) == name:
                raise ValueError('这张地图正在定位中使用，请先停止定位再重命名')
            new = nav.store.rename(name, to)
            moved = []
            if hasattr(nav, 'patrols'):
                for row in nav.patrols.list():
                    if row.get('map') == name:
                        task = nav.patrols.get(row['name']); task['map'] = new
                        nav.patrols.save(task); moved.append(row['name'])
            if hasattr(nav, 'emit'): nav.emit()
            return dict(renamed=new, tasks=moved, maps=nav.store.list())
        return handle(run)

    @app.post('/api/map-editor/official')
    def map_editor_official():
        def run():
            data = body(); nav = navigation()
            name = checked_name(data.get('name'))
            official = bool(data.get('official', True))
            value = nav.store.set_official(name, official)
            return dict(name=name, official=value, maps=nav.store.inventory())
        return handle(run)

    return editor
