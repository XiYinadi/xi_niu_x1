// Uses the existing socket/driver identity; never creates a competing client.
(() => {
  const el = id => document.getElementById(id);
  const stateEl = el('nav-status');
  const select = el('nav-map');
  let state = null, pick = null, anchor = null, target = null;
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
  el('nav-cancel').addEventListener('click', () => { pick = null; send('cancel'); });
  const beginPick = type => {
    if (!state || !state.running) { stateEl.textContent = '请先加载地图'; return; }
    if (type === 'goal' && !state.localized) { stateEl.textContent = '请先设置初始位置并等待定位'; return; }
    pick = type;
    target = null;
    el('nav-go').disabled = true;
    stateEl.textContent = '在地图按下并拖出朝向，松手确认' + (type === 'initial_pose' ? '机器人当前位置' : '目标位置');
  };
  el('nav-initial').addEventListener('click', () => beginPick('initial_pose'));
  el('nav-target').addEventListener('click', () => beginPick('goal'));
  el('nav-go').addEventListener('click', () => {
    if (target && state && state.localized && state.navigator_ready && !state.calibration_error) send('goal', target);
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
    try { canvas.setPointerCapture(e.pointerId); } catch (_) {}
  }, true);
  canvas.addEventListener('pointermove', e => { if (pick) intercept(e); }, true);
  canvas.addEventListener('pointerup', e => {
    if (!pick || !anchor) return;
    intercept(e);
    const tip = world(e);
    const pose = { ...anchor, yaw: Math.atan2(tip.y-anchor.y, tip.x-anchor.x) };
    const action = pick;
    pick = null; anchor = null;
    try { canvas.releasePointerCapture(e.pointerId); } catch (_) {}
    if (action === 'initial_pose') send(action, pose);
    else { target = pose; send('preview', pose); }
  }, true);
  canvas.addEventListener('pointercancel', () => { pick = null; anchor = null; });
  socket.on('navigation_error', d => { stateEl.textContent = d.message; });
  socket.on('navigation_state', d => {
    state = d;
    if (d.running) trajectoryPoints = [];
    const selected = select.value;
    select.innerHTML = (d.maps || []).map(name => '<option value="'+name+'">'+name+'</option>').join('');
    if ((d.maps || []).includes(selected)) select.value = selected;
    else if (d.selected_map) select.value = d.selected_map;
    stateEl.textContent = d.message + (d.calibration_error ? '；'+d.calibration_error : '');
    el('nav-load').disabled = d.busy || d.running;
    el('nav-save').disabled = d.busy;
    el('nav-initial').disabled = !d.initial_pose_ready || d.busy;
    el('nav-target').disabled = !d.localized || d.busy;
    el('nav-go').disabled = !target || !d.localized || !d.navigator_ready || !!d.calibration_error || d.navigating || d.busy;
    el('nav-stop').disabled = !d.running || d.busy;
    if (d.pose) { robotPose = d.pose; }
    el('nav-path').textContent = d.path && d.path.length ? '路径预览：'+d.path.length+' 个点（未行驶）' : '';
  });
  socket.on('disconnect', () => {
    pick = null; anchor = null; target = null; state = null;
    stateEl.textContent = '连接已断开，导航不可用';
    for (const id of ['nav-load','nav-save','nav-initial','nav-target','nav-go','nav-stop']) el(id).disabled = true;
  });
  const cancelOnLeave = () => { if (state && state.navigating) send('cancel'); };
  setInterval(() => {
    if (state && state.navigating && socket.connected && !document.hidden) send('heartbeat');
  }, 200);
  window.addEventListener('blur', cancelOnLeave);
  document.addEventListener('visibilitychange', () => { if (document.hidden) cancelOnLeave(); });
  // Path is drawn in map coordinates with the same viewport as the map.
  const originalRender = render;
  render = function() {
    originalRender();
    if (!state) return;
    const path = state.path || [];
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
  };
})();
