// Uses the existing socket/driver identity; never creates a competing client.
(() => {
  if ((typeof mappingPage !== 'undefined' && mappingPage) || (typeof controlPage !== 'undefined' && controlPage)) return;
  const el = id => document.getElementById(id);
  const stateEl = el('nav-status');
  const select = el('nav-map');
  let state = null, pick = null, anchor = null, target = null, pickTip = null, pickStart = null;
  let draft = { name:'', map:'', cycles:1, revision:0, waypoints:[] }, dirty = false;
  let previewMode = 'single';
  const send = (action, extra = {}) => {
    if (!socket.connected) { stateEl.textContent = '连接已断开'; return; }
    socket.emit('navigation_control', { action, ...extra });
  };
  el('nav-save').addEventListener('click', () => send('save', { name: el('nav-name').value }));
  el('nav-load').addEventListener('click', () => {
    if (!select.value) { stateEl.textContent = '请先保存并选择地图'; return; }
    send('load', { name: select.value });
  });
  el('nav-stop').addEventListener('click', () => send('stop'));
  el('nav-cancel').addEventListener('click', () => { pick = null;anchor=null;pickTip=null;pickStart=null;canvas.navPick=null;canvas.navPickNotice='';send('cancel'); });
  const beginPick = type => {
    if (!state || !state.running) { stateEl.textContent = '请先加载地图'; return; }
    if (type === 'goal' && !state.localized) { stateEl.textContent = '请先设置初始位置并等待定位'; return; }
    if (type === 'patrol' && draft.map && draft.map !== state.selected_map) {
      stateEl.textContent = '任务绑定的地图不同，请先加载 '+draft.map; return;
    }
    if (type === 'patrol' && draft.waypoints.length >= 50) {
      stateEl.textContent = '每个任务最多 50 个巡检点'; return;
    }
    pick = type;
    anchor = null; pickTip = null; pickStart = null;
    canvas.navPick = type; canvas.navPickNotice = '';
    previewMode = type === 'goal' ? 'single' : null;
    target = null;
    el('nav-go').disabled = true;
    stateEl.textContent = '在地图按下并拖出朝向，松手确认' + (type === 'initial_pose' ? '机器人当前位置' : type === 'patrol' ? '巡检点' : '目标位置');
  };
  el('nav-initial').addEventListener('click', () => beginPick('initial_pose'));
  el('nav-target').addEventListener('click', () => beginPick('goal'));
  el('nav-go').addEventListener('click', () => {
    const blocked=state && ('drive_error' in state ? state.drive_error : state.calibration_error);
    if (target && state && state.localized && state.navigator_ready && !blocked && (state.drive_mode!=='supervised_trial'||state.preview_valid)) send('goal', target);
  });
  const world = e => {
    const rect = canvas.getBoundingClientRect();
    return s2w((e.clientX-rect.left)*canvas.width/rect.width,
               (e.clientY-rect.top)*canvas.height/rect.height);
  };
  // Capture phase prevents the map pan handlers from consuming a selection gesture.
  const intercept = e => { e.preventDefault(); e.stopImmediatePropagation(); };
  canvas.addEventListener('pointerdown', e => {
    if (!pick) return;
    intercept(e);
    anchor = world(e);
    pickTip = anchor; pickStart = {x:e.clientX,y:e.clientY};
    try { canvas.setPointerCapture(e.pointerId); } catch (_) {}
  }, true);
  canvas.addEventListener('pointermove', e => { if (pick) {intercept(e);if(anchor)pickTip=world(e);} }, true);
  canvas.addEventListener('pointerup', e => {
    if (!pick || !anchor) return;
    intercept(e);
    const tip = world(e);
    if (pickStart && Math.hypot(e.clientX-pickStart.x,e.clientY-pickStart.y)<8) {
      anchor=null;pickTip=null;pickStart=null;
      canvas.navPickNotice='请按住当前位置，沿真实车头方向拖出一段箭头，再松开提交';
      stateEl.textContent=canvas.navPickNotice;
      try {canvas.releasePointerCapture(e.pointerId);}catch(_){}
      return;
    }
    const pose = { ...anchor, yaw: Math.atan2(tip.y-anchor.y, tip.x-anchor.x) };
    const action = pick;
    pick = null; anchor = null; pickTip = null; pickStart = null;canvas.navPick=null;canvas.navPickNotice='';
    try { canvas.releasePointerCapture(e.pointerId); } catch (_) {}
    if (action === 'patrol') {
      if (!state || !state.running) return;
      draft.map = draft.map || state.selected_map;
      draft.waypoints.push({...pose, dwell_s:Number(el('patrol-dwell').value)});
      changed();
    }
    else if (action === 'initial_pose') send(action, pose);
    else { target = pose; send('preview', pose); }
  }, true);
  canvas.addEventListener('pointercancel', () => { pick = null; anchor = null;pickTip=null;pickStart=null;canvas.navPick=null;canvas.navPickNotice=''; });
  const paintDraft = () => {
    el('patrol-map').textContent = '绑定地图：'+(draft.map || '添加首个点时绑定当前定位地图');
    el('patrol-points').innerHTML = draft.waypoints.map((p,i) => '<li style="margin:6px 0">'
      +(i+1)+': ('+p.x.toFixed(2)+', '+p.y.toFixed(2)+') 朝向 '+Math.round(p.yaw*180/Math.PI)+'° '
      +'<label>停留 <input type="number" min="0" max="3600" style="width:60px" value="'+p.dwell_s+'" data-action="dwell" data-index="'+i+'">秒</label>'
      +'<button class="btn" data-action="up" data-index="'+i+'">上移</button>'
      +'<button class="btn" data-action="down" data-index="'+i+'">下移</button>'
      +'<button class="btn" data-action="remove" data-index="'+i+'">移除</button></li>').join('');
    el('patrol-draft-status').textContent = draft.waypoints.length+' 个点；'+(dirty?'有未保存修改':'已同步')+'；配置和预览不会驱动车轮';
  };
  const changed = () => {
    dirty=true;
    previewMode=null;
    if (state && state.patrol_preview && state.patrol_preview.status === 'planning') send('cancel');
    paintDraft();
  };
  const taskPayload = () => ({ name:el('patrol-name').value,
    map:draft.map, cycles:Number(el('patrol-cycles').value),
    revision:el('patrol-name').value === draft.name ? draft.revision : 0,
    waypoints:draft.waypoints.map(p=>({...p})) });
  el('patrol-add').addEventListener('click',()=>beginPick('patrol'));
  el('patrol-save').addEventListener('click',()=>send('patrol_save',{task:taskPayload()}));
  el('patrol-preview').addEventListener('click',()=>{ previewMode='patrol';send('patrol_preview',{task:taskPayload()}); });
  el('patrol-load').addEventListener('click',()=>{
    if (dirty && !confirm('放弃当前未保存修改，载入已保存任务？')) return;
    if (state && state.patrol_preview && state.patrol_preview.status === 'planning') send('cancel');
    if (el('patrol-list').value) send('patrol_load',{name:el('patrol-list').value});
  });
  el('patrol-new').addEventListener('click',()=>{
    if (dirty && !confirm('放弃当前未保存修改，新建任务？')) return;
    if (state && state.patrol_preview && state.patrol_preview.status === 'planning') send('cancel');
    draft={name:'',map:'',cycles:1,revision:0,waypoints:[]};dirty=false;
    el('patrol-name').value='';el('patrol-cycles').value='1';pick=null;anchor=null;previewMode=null;paintDraft();
  });
  for (const id of ['patrol-name','patrol-cycles']) el(id).addEventListener('input',changed);
  el('patrol-points').addEventListener('click',e=>{
    const d=e.target.dataset || {}, i=Number(d.index);
    if (!Number.isInteger(i) || i<0 || i>=draft.waypoints.length) return;
    if (d.action==='remove') draft.waypoints.splice(i,1);
    else if (d.action==='up' && i>0) [draft.waypoints[i-1],draft.waypoints[i]]=[draft.waypoints[i],draft.waypoints[i-1]];
    else if (d.action==='down' && i<draft.waypoints.length-1) [draft.waypoints[i+1],draft.waypoints[i]]=[draft.waypoints[i],draft.waypoints[i+1]];
    else return;
    changed();
  });
  el('patrol-points').addEventListener('change',e=>{
    const d=e.target.dataset || {}, i=Number(d.index), value=Number(e.target.value);
    if (d.action==='dwell' && Number.isInteger(i) && draft.waypoints[i] && Number.isFinite(value) && value>=0 && value<=3600) {
      draft.waypoints[i].dwell_s=value;changed();
    }
  });
  socket.on('patrol_loaded',task=>{
    previewMode=null;pick=null;anchor=null;
    draft={...task,waypoints:task.waypoints.map(p=>({...p}))};dirty=false;
    el('patrol-name').value=task.name;el('patrol-cycles').value=String(task.cycles);paintDraft();
  });
  paintDraft();
  socket.on('navigation_error', d => { stateEl.textContent = d.message; });
  socket.on('navigation_state', d => {
    if (state && d.selected_map !== state.selected_map) { target=null;pick=null;anchor=null;pickTip=null;pickStart=null;canvas.navPick=null;canvas.navPickNotice='';previewMode=null; }
    state = d;
  window.__navRegions = d.regions || [];
    if (d.running) trajectoryPoints = [];
    const selected = select.value;
    select.innerHTML = (d.maps || []).map(name => '<option value="'+name+'">'+name+'</option>').join('');
    if ((d.maps || []).includes(selected)) select.value = selected;
    else if (d.selected_map) select.value = d.selected_map;
    const blocked='drive_error' in d?d.drive_error:d.calibration_error;
    stateEl.textContent = d.message + (blocked ? '；'+blocked : '');
    el('nav-load').disabled = d.busy || d.running;
    el('nav-save').disabled = d.busy;
    el('nav-initial').disabled = !d.initial_pose_ready || d.busy;
    el('nav-target').disabled = !d.localized || d.busy;
    el('nav-go').disabled = !target || !d.localized || !d.navigator_ready || !!blocked || (d.drive_mode==='supervised_trial'&&!d.preview_valid) || d.navigating || d.busy;
    el('nav-stop').disabled = !d.running || d.busy;
    el('nav-stop').textContent = d.mode==='mapping'?'停止建图管线':'停止定位';
    if (d.pose) { robotPose = d.pose; }
    el('nav-path').textContent = d.path && d.path.length ? (d.navigating?'导航路径：':'路径预览：')+d.path.length+' 个点'+(d.navigating&&Number.isFinite(d.goal_distance_m)?' · 距目标约 '+d.goal_distance_m.toFixed(2)+' m':'') : '';
    const taskList=el('patrol-list'), previous=taskList.value;
    taskList.innerHTML=(d.patrols || []).map(t=>'<option value="'+t.name+'">'+t.name+' · '+t.points+'点 · '+t.map+'</option>').join('');
    if ((d.patrols || []).some(t=>t.name===previous)) taskList.value=previous;
    el('patrol-add').disabled=!d.running || d.busy;
    el('patrol-preview').disabled=!d.localized || !d.planner_ready || d.busy || d.navigating;
    for (const id of ['patrol-save','patrol-load','patrol-new']) el(id).disabled=!!d.busy;
    const route=d.patrol_preview;
    el('patrol-status').textContent=route ? route.message+(route.planned_stopped_s!==undefined?'；计划停留共 '+route.planned_stopped_s+' 秒（不含行驶）':'') : '仅配置和路线预览，不执行巡检';
  });
  socket.on('disconnect', () => {
    pick = null; anchor = null; target = null; state = null;previewMode=null;
    pickTip=null;pickStart=null;canvas.navPick=null;canvas.navPickNotice='';
    stateEl.textContent = '连接已断开，导航不可用';
    for (const id of ['nav-load','nav-save','nav-initial','nav-target','nav-go','nav-stop','patrol-add','patrol-preview','patrol-save','patrol-load','patrol-new']) el(id).disabled = true;
  });
  setInterval(() => {
    if (state && (state.navigating || state.autonomy?.active) && socket.connected && !document.hidden) send('heartbeat');
  }, 200);
  // 2026-10-27: 删除 blur / visibilitychange 里主动 cancel。原来点一下别的窗口、
  // 或标签页切到后台，就会把正在跑的巡检/导航取消（报告里那句「操作者取消导航」
  // 就是它发的）。操作者真掉线由心跳兜底：巡检 2 秒、导航 0.6 秒。
  // Path is drawn in map coordinates with the same viewport as the map.
  const originalRender = render;
  render = function(ts) {
    originalRender(ts);
    if (!state) return;
    if(pick&&anchor&&pickTip){
      const a=w2s(anchor.x,anchor.y),b=w2s(pickTip.x,pickTip.y),angle=Math.atan2(b.y-a.y,b.x-a.x);
      ctx.save();ctx.strokeStyle='#8ae3c5';ctx.fillStyle='#8ae3c5';ctx.lineWidth=3;ctx.setLineDash([5,3]);
      ctx.beginPath();ctx.arc(a.x,a.y,6,0,Math.PI*2);ctx.stroke();ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();ctx.setLineDash([]);
      ctx.beginPath();ctx.moveTo(b.x,b.y);ctx.lineTo(b.x-10*Math.cos(angle-.4),b.y-10*Math.sin(angle-.4));ctx.lineTo(b.x-10*Math.cos(angle+.4),b.y-10*Math.sin(angle+.4));ctx.closePath();ctx.fill();
      ctx.font='12px sans-serif';ctx.fillText('松开后提交朝向',a.x+10,a.y-12);ctx.restore();
    }
    const route=state.patrol_preview;
    const path = previewMode === 'patrol' ? (route && route.path || []) : previewMode === 'single' ? state.path || [] : [];
    if (path.length > 1) {
      ctx.save(); ctx.strokeStyle = '#ffcc66'; ctx.lineWidth = 3; ctx.beginPath();
      path.forEach((p, i) => { const s = w2s(p[0], p[1]); if (i) ctx.lineTo(s.x,s.y); else ctx.moveTo(s.x,s.y); });
      ctx.stroke(); ctx.restore();
    }
    if (target) {
      const p = w2s(target.x, target.y);
      ctx.save(); ctx.strokeStyle = '#ffcc66'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(p.x,p.y,7,0,2*Math.PI);
      ctx.moveTo(p.x,p.y); ctx.lineTo(p.x+24*Math.cos(target.yaw),p.y-24*Math.sin(target.yaw));
      ctx.stroke(); ctx.restore();
    }
    if (draft.map === state.selected_map) draft.waypoints.forEach((point,i)=>{
      const p=w2s(point.x,point.y);ctx.save();ctx.fillStyle='#ffcc66';
      ctx.beginPath();ctx.arc(p.x,p.y,5,0,2*Math.PI);ctx.fill();ctx.fillText(String(i+1),p.x+8,p.y-8);ctx.restore();
    });
  };
})();
