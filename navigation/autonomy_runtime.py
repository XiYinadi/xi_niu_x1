"""Supervised exploration and patrol sequencing using the existing gated Nav2 output.

No direct GPIO, arming, or inference. Every mission requires a visible owning
driver page heartbeat. Stop/alert/restart never resumes a mission automatically.
"""
import base64
import copy
import hashlib
import json
import math
import re
import threading
import time
import urllib.request
import uuid
from pathlib import Path
from map_store import checked_name, yaw
from navigation_core import validate_calibration, fresh
from frontier_explorer import select_frontiers
from inspection_service import atomic_json, timestamp, classify


class AutonomyRuntime:
    def __init__(self, nav, inspections):
        self.nav,self.inspections=nav,inspections
        self.lock=nav.lock
        self.root=nav.root/'data'/'autonomy';self.root.mkdir(parents=True,exist_ok=True)
        self.active=False;self.epoch=0;self.sid=None;self.last_client=None
        self.camera=None;self.camera_error='等待检测数据';self.camera_received=None
        self.state=dict(status='idle',message='先准备自动建图，或加载地图后执行已保存的巡检任务',kind=None)
        self.alert=None;self.preview=None;self.inspection_id=None
        self.report_pending=False;self.compute_pending=False
        self.wait_until=0;self.visited=[];self.empty_rounds=0;self.last_empty_stamp=None
        try:
            saved=json.loads((self.root/'state.json').read_text())
            self.alert=saved.get('alert');self.state=saved['state']
            self.inspection_id=self.state.get('inspection_id')
            if self.state['status'] in ('starting','exploring','navigating','dwell','detecting','reporting','saving'):
                self.state.update(status='interrupted',message='服务重启，自动任务已中断；需人工重新启动')
                self.report_pending=bool(self.inspection_id)
        except (OSError,ValueError,KeyError):pass
        if self.inspection_id:
            try:self.report_pending=self.inspections.get(self.inspection_id)['status'] in ('running','sampling')
            except ValueError:self.inspection_id=None

    def persist(self):
        atomic_json(self.root/'state.json',dict(state=self.state,alert=self.alert))

    def snapshot(self):
        report_ready=bool(self.inspection_id and self.inspections.get(self.inspection_id)['status']=='completed')
        return copy.deepcopy(dict(self.state,active=self.active,alert=self.alert,preview=self.preview,
                                  camera_ready=self.camera_ready(),inspection_id=self.inspection_id,report_ready=report_ready))

    def camera_ready(self):
        data=self.camera
        return (not self.camera_error and data is not None and data.get('capture_ok') is True
                and fresh(time.time(),data.get('captured_at'),18) and fresh(time.monotonic(),self.camera_received,4))

    def start_threads(self):
        threading.Thread(target=self.monitor_camera,daemon=True).start()
        threading.Thread(target=self.loop,daemon=True).start()

    def monitor_camera(self):
        while True:
            try:
                with urllib.request.urlopen('http://127.0.0.1:5812/inspection.json',timeout=2) as r:data=json.load(r)
                if type(data.get('fire')) is not bool or type(data.get('confidence')) not in (int,float):raise ValueError('检测结果格式无效')
                if not math.isfinite(data['confidence']) or not 0<=data['confidence']<=1:raise ValueError('检测置信度无效')
                with self.lock:
                    self.camera=data;self.camera_received=time.monotonic();self.camera_error=None
                    if self.active and self.camera_ready() and classify(data)!='clear':
                        self.raise_alert(data)
            except Exception as e:
                with self.lock:self.camera_error=str(e)
            time.sleep(.5)

    def grid(self):
        with self.nav.node._lock:
            msg=self.nav.node._latest_map_msg;age=time.monotonic()-self.nav.node._last_map_t
        if msg is None or age>10:raise ValueError('没有新鲜地图，无法自动探索')
        q=msg.info.origin.orientation;t=msg.info.origin.position
        return dict(width=msg.info.width,height=msg.info.height,resolution=msg.info.resolution,
                    origin=(t.x,t.y,yaw(q)),data=msg.data),self.nav.node._last_map_t

    def check_start(self,sid,kind):
        if self.active or self.compute_pending:raise ValueError('已有自动任务或探索计算正在进行')
        if self.report_pending:raise ValueError('正在归档上次巡检，请等待报告生成')
        with self.inspections.lock:
            if any(s['status'] in ('running','sampling','reporting') for s in self.inspections.sessions.values()):
                raise ValueError('请先完成已有巡检或等待报告生成')
        if self.alert and not self.alert.get('acknowledged_at'):raise ValueError('请先人工确认现场告警，再重新启动')
        validate_calibration(self.nav.config)
        if not self.nav.armed() or self.nav.owner()!=sid:raise ValueError('请先由当前驾驶页面启用电机')
        if not self.nav.localized(time.monotonic()):raise ValueError('地图和定位尚未就绪')
        if self.nav.busy or self.nav.pending or self.nav.navigating:raise ValueError('请先完成或取消当前导航操作')
        if not self.camera_ready():raise ValueError('本地相机检测尚未就绪或结果过期，不能启动自动任务')
        if classify(self.camera)!='clear':raise ValueError('当前画面有疑似火情或待复核结果，请先人工处理')
        if (kind=='explore') != (self.nav.mode=='mapping'):raise ValueError('自动建图需先准备建图；巡检需先加载已保存地图并定位')
        if not self.nav.nav.server_is_ready() or any(self.nav.lifecycle.get(k)!=3 for k in ('planner_server','controller_server','bt_navigator','velocity_smoother')):
            raise ValueError('路径规划与避障服务尚未就绪')

    def begin(self,sid,kind):
        self.epoch+=1;self.active=True;self.sid=sid;self.last_client=time.monotonic()
        self.inspection_id=None;self.visited=[];self.empty_rounds=0;self.last_empty_stamp=None
        self.state=dict(status='exploring' if kind=='explore' else 'navigating',kind=kind,
                        started_at=timestamp(),message='正在选择建图目标' if kind=='explore' else '正在前往巡检点',
                        completed_points=0,goal=None)
        self.deadline=time.monotonic()+1200;self.wait_until=0;self.persist()

    def heartbeat(self,sid):
        if self.active and sid==self.sid:self.last_client=time.monotonic()

    def halt(self,reason,status='stopped'):
        self.active=False;self.epoch+=1;self.compute_pending=False
        self.state.update(status=status,message=reason,ended_at=timestamp(),goal=None)
        self.nav.cancel(reason,disarm=True,keep_mission=True)
        self.nav.send_drive(0.,0.,False);self.nav.disarm()
        self.report_pending=bool(self.inspection_id);self.persist()

    def cancelled(self,reason):
        if self.active:self.halt(reason)

    def raise_alert(self,data):
        # Stop before disk writes or report work, and never auto-resume after clear.
        self.halt('发现疑似火情或不确定检测，已停车并等待人工复核','alert')
        alert=dict(id=uuid.uuid4().hex,created_at=timestamp(),reason=str(data.get('reason',''))[:1000],
                   confidence=data['confidence'],verdict=data.get('verdict') if data.get('verdict') in ('suspected','uncertain') else classify(data),captured_at=data['captured_at'],
                   model=data.get('model'),pose=copy.deepcopy(self.nav.last_pose),inspection_id=self.inspection_id)
        self.alert=alert
        try:
            image=base64.b64decode(data['jpeg'],validate=True)
            if not image.startswith(b'\xff\xd8') or not image.endswith(b'\xff\xd9'):raise ValueError('检测证据不是有效JPEG')
            (self.root/(alert['id']+'.jpg')).write_bytes(image)
            alert['image']=alert['id']+'.jpg';alert['sha256']=hashlib.sha256(image).hexdigest()
        except Exception as e:alert['evidence_error']=str(e)
        atomic_json(self.root/(alert['id']+'.json'),alert);self.persist()
        self.nav.socketio.emit('autonomy_alert',copy.deepcopy(alert))
        if self.inspection_id:self.inspections.attach_alert(self.inspection_id,alert)

    def request_frontiers(self,move=False):
        if self.compute_pending:raise ValueError('正在计算探索目标，请稍候')
        if not self.nav.last_pose:raise ValueError('尚未取得有效机器人位置')
        grid,stamp=self.grid();pose=dict(self.nav.last_pose);epoch=self.epoch
        cfg=self.nav.config;clearance=math.hypot(cfg['length_m'],cfg['width_m'])/2+.10
        self.compute_pending=True
        def worker():
            try:result=select_frontiers(grid,pose,clearance,self.visited)
            except Exception as e:result=dict(error=str(e),candidates=[],complete=False)
            with self.lock:
                if epoch!=self.epoch:return
                self.compute_pending=False;self.preview=result
                if not move or not self.active:return
                if result.get('error'):self.halt(result['error'],'blocked');return
                if result['candidates']:
                    self.empty_rounds=0;target=result['candidates'][0]
                    self.state.update(status='navigating',goal=target,message='正在前往已知自由区域中的探索观察点')
                    try:self.move(target,lambda:self.exploration_arrived(target))
                    except Exception as e:self.halt('探索导航未能启动：'+str(e),'blocked')
                elif not result['complete']:
                    self.halt('仍有未知边界，但没有新的安全可达目标；建图未完成，可手动保存当前部分地图','blocked')
                else:
                    if stamp!=self.last_empty_stamp:self.empty_rounds+=1;self.last_empty_stamp=stamp
                    self.state.update(status='exploring',message='等待新地图确认是否还有探索边界')
                    self.wait_until=time.monotonic()+3
                    if self.empty_rounds>=3:
                        try:self.save_exploration()
                        except Exception as e:self.halt('探索结束但地图保存失败：'+str(e),'blocked')
        threading.Thread(target=worker,daemon=True).start()

    def move(self,target,arrived):
        epoch=self.epoch
        self.goal_started=time.monotonic()
        def done(path,error):
            with self.lock:
                if not self.active or epoch!=self.epoch:return
                if error:self.halt('自动任务受阻：'+str(error),'blocked');return
                # Nav2 success alone must agree with a fresh actual pose.
                pose=self.nav.last_pose
                angle=abs(math.atan2(math.sin(pose['yaw']-target['yaw']),math.cos(pose['yaw']-target['yaw']))) if pose else math.inf
                if not self.nav.localized(time.monotonic()) or pose is None or math.hypot(pose['x']-target['x'],pose['y']-target['y'])>.25 or angle>.5:
                    self.halt('导航结果与实际位置不一致，未计为到达','blocked');return
                arrived();self.persist()
        self.nav.send_goal(target,self.sid,on_result=done)

    def exploration_arrived(self,target):
        self.visited.append(dict(target));self.state.update(status='exploring',completed_points=len(self.visited),goal=None,message='已到观察点，等待地图更新')
        self.wait_until=time.monotonic()+3

    def save_exploration(self):
        with self.nav.node._lock:
            msg=self.nav.node._latest_map_msg;age=time.monotonic()-self.nav.node._last_map_t
        if msg is None or age>10:raise ValueError('地图已过期，未保存')
        path=self.nav.store.save(msg,self.map_name)
        self.halt('探索结束，地图已保存：'+path.parent.name+'；可解除使能后加载地图并配置巡检','completed')
        self.state['saved_map']=path.parent.name;self.persist()

    def patrol_arrived(self):
        session=self.inspections.get(self.inspection_id)
        task=session['task'];point=task['waypoints'][session['next_step']%len(task['waypoints'])]
        self.wait_until=time.monotonic()+max(1,point['dwell_s'])
        self.state.update(status='dwell',goal=None,message='已到巡检点，停车稳定画面后检测')

    def tick(self):
        with self.lock:
            if self.report_pending and self.inspection_id:
                session=self.inspections.get(self.inspection_id)
                if session['status']=='running':self.inspections.finish(self.inspection_id);self.report_pending=False
                elif session['status'] in ('reporting','completed'):self.report_pending=False
            if not self.active:return
            now=time.monotonic()
            if not fresh(now,self.last_client,.6):self.halt('操作者页面链路中断，自动任务已停车');return
            if not self.nav.armed() or self.nav.owner()!=self.sid:self.halt('驾驶权或使能改变，自动任务已停车');return
            if not self.nav.localized(now):self.halt('定位或地图失效，自动任务已停车','blocked');return
            with self.nav.node._lock:scan_t=self.nav.node._last_scan_t
            if not fresh(now,scan_t,self.nav.config['scan_timeout_s']):self.halt('雷达失效，自动任务已停车','blocked');return
            if not self.camera_ready():self.halt('火情检测链路失效或结果过期，自动任务已停车','blocked');return
            if classify(self.camera)!='clear':self.raise_alert(self.camera);return
            if now>self.deadline:self.halt('达到20分钟任务时限，已停车；当前任务不视为完成','blocked');return
            if self.nav.navigating or self.nav.pending:
                if now-self.goal_started>120:self.halt('单个目标超过两分钟未到达，已停车','blocked')
                return
            # Keep a deliberate zero command while stopped between goals or sensing.
            self.nav.send_drive(0.,0.,True)
            if self.state['kind']=='explore':
                if len(self.visited)>=200:self.halt('达到探索点数上限，当前地图可手动保存','blocked');return
                if now>=self.wait_until and not self.compute_pending:self.request_frontiers(move=True)
            else:
                session=self.inspections.get(self.inspection_id)
                self.state['completed_points']=session['next_step']
                if self.state['status']=='detecting' and session['status']!='sampling':
                    latest=session['records'][-1]
                    if latest['status']!='ok':self.halt('点位检测失败，停止巡检并保留未完成项','blocked');return
                    if latest['verdict']!='clear':
                        self.raise_alert(dict(latest,jpeg=base64.b64encode((self.inspections.directory(self.inspection_id)/latest['image']).read_bytes()).decode()))
                        return
                if session['status'] in ('reporting','completed'):
                    self.halt('巡检点已处理完毕，已停车并生成报告','completed');return
                if self.state['status']=='dwell':
                    if now>=self.wait_until:
                        self.inspections.record(self.inspection_id,automatic=True)
                        self.state.update(status='detecting',message='停车检测中，正在保存新帧证据')
                elif self.state['status']=='detecting':
                    if session['status']=='sampling':return
                    latest=session['records'][-1]
                    if latest['status']!='ok':self.halt('点位检测失败，停止巡检并保留未完成项','blocked');return
                    if latest['verdict']!='clear':self.raise_alert(self.camera);return
                    self.state['status']='navigating'
                elif session['status']=='running':
                    point=session['task']['waypoints'][session['next_step']%len(session['task']['waypoints'])]
                    self.state.update(goal=point,message='正在前往第 '+str(session['next_step']+1)+' 个巡检点')
                    self.move(point,self.patrol_arrived)

    def loop(self):
        while True:
            try:self.tick()
            except Exception as e:
                with self.lock:
                    if self.active:self.halt('自动任务错误，已停车：'+str(e),'blocked')
                    else:self.state['message']='任务收尾失败：'+str(e)
            time.sleep(.1)

    def control(self,data,sid):
        with self.lock:
            action=data['action']
            if action=='alert_ack':
                if self.active:raise ValueError('任务运行时不能确认告警')
                if self.alert:self.alert['acknowledged_at']=timestamp();self.persist()
                return
            if action=='mission_stop':self.halt('操作者停止自动任务');return
            if action=='explore_preview':
                if self.active:raise ValueError('请先停止自动任务')
                self.request_frontiers();return
            if action=='explore_finish':
                if not self.active or self.state['kind']!='explore' or sid!=self.sid:raise ValueError('仅当前操作者可结束自动建图')
                self.halt('操作者提前结束建图，保存为部分地图','stopped')
                with self.nav.node._lock:msg=self.nav.node._latest_map_msg;age=time.monotonic()-self.nav.node._last_map_t
                if msg is None or age>10:raise ValueError('地图已过期，未保存')
                path=self.nav.store.save(msg,self.map_name);self.state['saved_map']=path.parent.name;self.persist();return
            if action=='explore_start':
                self.check_start(sid,'explore');name=checked_name(data.get('name'))
                if name in self.nav.store.list():raise ValueError('地图名称已存在，请使用新名称')
                self.begin(sid,'explore');self.map_name=name
            elif action=='patrol_start':
                self.check_start(sid,'patrol');task=self.nav.patrols.get(data.get('name'))
                if task['map']!=self.nav.active_map:raise ValueError('巡检任务地图与当前定位地图不一致')
                for point in task['waypoints']:self.nav.check_goal(self.nav.pose_message(point))
                session=self.inspections.start(dict(title='自动巡检 · '+task['name'],task=task['name']),mode='automatic')
                self.begin(sid,'patrol');self.inspection_id=session['id'];self.state['inspection_id']=session['id'];self.persist()
            else:raise ValueError('未知自动任务操作')


def register_autonomy(app,runtime):
    from flask import jsonify,send_file
    @app.get('/api/autonomy/state')
    def autonomy_state():
        with runtime.lock:return jsonify(runtime.snapshot())
    @app.get('/autonomy/evidence/<image>')
    def autonomy_evidence(image):
        if not re.fullmatch(r'[0-9a-f]{32}\.jpg',image):return '',404
        if not (runtime.root/(image[:-4]+'.json')).is_file():return '',404
        return send_file(runtime.root/image,mimetype='image/jpeg')
