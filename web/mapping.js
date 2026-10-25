/* Mapping owns only manual driving and saving; shares the existing socket. */
(() => {
  // Navigation must remain available on both pages, including on a phone.
  document.querySelectorAll('.page-links a').forEach(link => {
    const current = link.getAttribute('href') === (mappingPage ? '/mapping' : controlPage ? '/control' : missionPage ? document.body.classList.contains('navigation-page') ? '/navigation' : document.body.classList.contains('explore-page') ? '/explore' : '/patrol' : '/console');
    if (current) link.setAttribute('aria-current', 'page');
    else link.removeAttribute('aria-current');
    if (link.getAttribute('href') === '/mapping') link.textContent = mappingPage ? '手动建图' : '← 返回建图';
    link.addEventListener('click', () => releaseAll());
  });
  if (!mappingPage) { if (!controlPage && !missionPage) document.title = '犀牛一号 · 控制台'; return; }
  const el = id => document.getElementById(id);
  let health = null, healthAt = 0, pose = null, navigationBusy = false;
  let pendingName = null, pendingAt = 0;
  let navigationLocalization=false,automaticBusy=false,returnPending=false,returnAt=0,returnCheckAt=0,returnReady=false,returnNotice='';
  document.title = '犀牛一号 · 建图工作台';
  el('mapping-actions').open = true;
  el('drive').classList.remove('collapsed');
  el('btn-drive-collapse').textContent = '收起';
  el('btn-drive-collapse').setAttribute('aria-expanded', 'true');
  el('drive-title').textContent = '摇杆驾驶';
  el('win-map').querySelector('.grp').prepend(el('btn-map'));
  el('mapping-actions').querySelector('h2').textContent = '保存当前地图';
  el('mapping-actions').querySelector('.mapping-action-hint').textContent = '保存后可用于定位。停止建图前，请先保存。';
  el('map-empty').querySelector('span').textContent = '点击地图上方「启动建图」。启动后先静止约 3 秒，等待校准完成。';
  el('workspace').insertBefore(document.querySelector('.mapping-odometry'), el('win-log'));
  const modeControls=document.createElement('div');modeControls.id='mp-mode-controls';modeControls.hidden=true;
  const modeNote=document.createElement('p');modeNote.id='mp-mode-note';
  const returnButton=document.createElement('button');returnButton.id='mp-return-mapping';returnButton.type='button';returnButton.textContent='停止定位，返回建图';
  modeControls.append(modeNote,returnButton);document.querySelector('.mapping-health').append(modeControls);
  returnButton.addEventListener('click',()=>{
    if(returnButton.disabled||!socket.connected||armed||returnPending)return;
    releaseAll();returnPending=true;returnAt=performance.now();returnCheckAt=returnAt;returnNotice='';paint();
    socket.emit('navigation_control',{action:'stop'});
  });
  const saveButton = el('nav-save'), nameInput = el('nav-name');
  const feedback = (text, type='') => {
    el('mp-save-feedback').textContent = text;
    el('mp-save-feedback').className = 'mapping-only save-feedback '+type;
  };
  const age = key => {
    if (!socket.connected || !health || health[key] === null || health[key] === undefined) return Infinity;
    const base = Number(health[key]);
    return Number.isFinite(base) && base >= 0 ? base+(performance.now()-healthAt)/1000 : Infinity;
  };
  const fresh = (key, threshold) => age(key) < threshold;
  let odometryOrigin = null, odometryNotice = '';
  const odometryZero = el('mp-odometry-zero');
  function currentOdometry() {
    const value = health?.odometry;
    return fresh('odom_age', 1) && value
      && ['x','y','yaw','vx','vy','wz','generation'].every(key => typeof value[key] === 'number' && Number.isFinite(value[key]))
      && typeof value.frame === 'string' && typeof value.child_frame === 'string' ? value : null;
  }
  function paintOdometry() {
    const value = currentOdometry();
    if (odometryOrigin && (!value || value.generation !== odometryOrigin.generation
        || value.frame !== odometryOrigin.frame || value.child_frame !== odometryOrigin.child_frame)) {
      odometryOrigin = null;
      odometryNotice = '数据中断或坐标源已重启，请停车后重新设置测量起点。';
    }
    odometryZero.disabled = !value;
    const state = el('mp-odometry-state');
    state.textContent = !socket.connected ? '未连接' : value ? '实时' : Number.isFinite(age('odom_age')) ? '数据过期或无效' : '等待数据';
    state.className = 'status-pill '+(value ? 'live' : 'warning');
    const set = (name, number, digits) => {
      el('mp-odometry-'+name).textContent = Number.isFinite(number) ? (Math.abs(number) < .5*10**-digits ? 0 : number).toFixed(digits) : '—';
    };
    for (const key of ['forward','lateral','turn','distance']) set(key, NaN, 1);
    if (value && odometryOrigin) {
      const dx = value.x-odometryOrigin.x, dy = value.y-odometryOrigin.y;
      const c = Math.cos(odometryOrigin.yaw), s = Math.sin(odometryOrigin.yaw);
      const angle = value.yaw-odometryOrigin.yaw;
      set('forward', (c*dx+s*dy)*100, 1);
      set('lateral', (-s*dx+c*dy)*100, 1);
      set('distance', Math.hypot(dx,dy)*100, 1);
      set('turn', Math.atan2(Math.sin(angle),Math.cos(angle))*180/Math.PI, 1);
    }
    set('speed', value?.vx, 3);
    set('rate', value ? value.wz*180/Math.PI : NaN, 1);
    set('x', value?.x, 3); set('y', value?.y, 3);
    set('yaw', value ? value.yaw*180/Math.PI : NaN, 1);
    el('mp-odometry-frame').textContent = value ? value.frame+' → '+value.child_frame : '等待坐标系';
    el('mp-odometry-note').textContent = odometryOrigin
      ? '已设置起点 · 前进、左偏、左转为正。转角 ±180°；直线位移不累计路程。'
      : odometryNotice || '停车后设置起点，再对照卷尺。前进、左偏、左转为正。';
  }
  odometryZero.addEventListener('click', () => {
    const value = currentOdometry();
    if (!value) return;
    odometryOrigin = {...value};
    odometryNotice = '';
    paintOdometry();
  });
  // Preview starts off and uses the existing shared camera stream, not another capture process.
  let cameraWanted = false, cameraSubscribed = false, cameraPageActive = true;
  let cameraFrameAt = null, cameraError = '';
  let cameraAck = null;
  const cameraImage = el('mp-camera-image');
  function cameraAvailable() {
    return cameraWanted && cameraPageActive && !document.hidden && socket.connected;
  }
  function clearCameraFrame() {
    if (cameraAck) { const ack = cameraAck; cameraAck = null; ack(); }
    cameraFrameAt = null;
    cameraImage.hidden = true;
    cameraImage.removeAttribute('src');
  }
  function paintCamera() {
    const live = cameraAvailable() && cameraFrameAt !== null && performance.now()-cameraFrameAt < 2000;
    el('mp-camera-window').hidden = !cameraWanted;
    el('win-map').classList.toggle('camera-visible', cameraWanted);
    el('mp-camera-toggle').classList.toggle('active', cameraWanted);
    el('mp-camera-toggle').setAttribute('aria-pressed', String(cameraWanted));
    el('mp-camera-toggle').setAttribute('aria-label', cameraWanted ? '关闭摄像头' : '显示摄像头');
    el('mp-camera-toggle').title = cameraWanted ? '关闭摄像头' : '显示摄像头';
    cameraImage.hidden = !live;
    el('mp-camera-caption').textContent = live ? '实时' : !socket.connected ? '未连接' : !cameraAvailable() ? '已暂停' : '等待画面';
    el('mp-camera-message').hidden = live;
    el('mp-camera-message').textContent = !socket.connected ? '连接已断开，等待重连'
      : !cameraAvailable() ? '预览已暂停' : cameraError || (cameraFrameAt === null ? '正在连接摄像头…' : '画面已中断，正在重试…');
  }
  function syncCamera() {
    const subscribe = cameraAvailable();
    if (subscribe !== cameraSubscribed) {
      if (socket.connected) socket.emit('camera_control', {action:subscribe ? 'start' : 'stop', flow_control:true});
      cameraSubscribed = subscribe;
      clearCameraFrame();
    }
    paintCamera();
  }
  function setCamera(on) {
    cameraWanted = !!on;
    cameraError = '';
    syncCamera();
    if (!cameraWanted) clearCameraFrame();
  }
  el('mp-camera-toggle').addEventListener('click', () => setCamera(!cameraWanted));
  el('mp-camera-close').addEventListener('click', () => setCamera(false));
  cameraImage.addEventListener('load', () => {
    if (!cameraAvailable() || !cameraImage.hasAttribute('src')) return;
    cameraError = '';
    cameraFrameAt = performance.now();
    if (cameraAck) { const ack = cameraAck; cameraAck = null; ack(); }
    paintCamera();
  });
  cameraImage.addEventListener('error', () => {
    if (!cameraAvailable()) return;
    if (cameraAck) { const ack = cameraAck; cameraAck = null; ack(); }
    cameraFrameAt = null;
    cameraError = '画面加载失败，正在重试…';
    paintCamera();
  });
  socket.on('frame', (data, ack) => {
    if (!cameraAvailable() || typeof data?.img !== 'string' || !data.img) { if (typeof ack === 'function') ack(); return; }
    if (cameraAck) cameraAck();
    cameraAck = typeof ack === 'function' ? ack : null;
    cameraImage.src = 'data:image/jpeg;base64,'+data.img;
  });
  socket.on('camera_state', state => {
    if (!cameraAvailable()) return;
    if (state.error) {
      cameraError = '摄像头暂不可用，正在重试…';
      clearCameraFrame();
    }
    paintCamera();
  });
  socket.on('connect', () => { cameraSubscribed = false; syncCamera(); });
  socket.on('disconnect', () => { cameraSubscribed = false; clearCameraFrame(); paintCamera(); });
  document.addEventListener('visibilitychange', syncCamera);
  window.addEventListener('pagehide', () => { cameraPageActive = false; syncCamera(); });
  window.addEventListener('pageshow', () => { cameraPageActive = true; syncCamera(); });
  function paint() {
    paintCamera();
    paintOdometry();
    const connected = !!socket.connected;
    const scanning = fresh('scan_age', 2), odometry = fresh('odom_age', 1);
    const mapFresh = fresh('map_age', 10), imu = fresh('imu_age', .75);
    const localizing=health?.status==='localization'||navigationLocalization;
    if(health?.status==='mapping'&&!localizing)returnReady=false;
    if(returnPending&&performance.now()-returnAt>25000){returnPending=false;returnNotice='尚未确认定位已停止，请查看运行日志后重试。';}
    modeControls.hidden=!connected||(!localizing&&!returnPending&&!returnReady);
    returnButton.hidden=!localizing&&!returnPending;
    returnButton.disabled=!connected||armed||navigationBusy||automaticBusy||returnPending;
    returnButton.textContent=returnPending?'正在停止定位…':armed?'先停用电机，再返回建图':'停止定位，返回建图';
    modeNote.textContent=returnNotice||(returnPending?'正在停止定位，等待实际状态确认…':returnReady&&!localizing?'定位已停止。点击地图上方「启动建图」创建新图；已保存地图保留。':automaticBusy?'请先停止正在执行的自动任务。':armed?'当前为定位模式。先点「停用电机」，再返回建图。':'当前加载的是保存地图。停止定位后即可启动新建图，已保存地图保留。');
    if(returnPending&&connected&&performance.now()-returnCheckAt>1000){returnCheckAt=performance.now();socket.emit('navigation_control',{action:'status'});}
    if (health && connected && !health.busy && health.status !== 'localization') {
      mapStatus = scanning && odometry && mapFresh ? 'mapping' : scanning ? 'lidar_only' : 'off';
      mapPaint();
    }
    el('mp-mobile-status').textContent = !connected ? '未连接 · 驾驶已暂停'
      : `雷达 ${scanning ? '在线' : '待就绪'} · 里程计 ${odometry ? '在线' : '待就绪'} · GY-521 ${imu ? '在线' : '待就绪'}`;
    let title = '尚未开始', note = '先启动建图，再启用电机缓慢驾驶。', style = '';
    if (!connected) { title = '连接已断开'; note = '控制已暂停，等待机器人重新连接。'; }
    else if (!health) { title = '等待状态'; note = '正在读取机器人实时状态…'; }
    else if (health.busy) {
      title = health.action === 'stop' ? '正在停止' : '正在启动';
      note = health.action === 'stop' ? '正在停止建图管线…' : '请保持车体静止，等待传感器与地图就绪。';
    } else if (health.status === 'localization') {
      title = '定位模式'; note = '当前加载的是保存地图；可在下方停止定位，返回手动建图。'; style = 'warning';
    } else if (scanning && odometry && mapFresh) {
      title = '建图运行中'; note = imu ? '地图持续更新 · 雷达、里程计与陀螺仪在线' : '地图持续更新 · 陀螺仪暂未就绪，请检查校准或连接';
      style = imu ? 'live' : 'warning';
    } else if (scanning) {
      title = '建图未就绪'; note = !odometry ? '雷达在线，里程计数据未就绪。可展开日志查看。' : '雷达与里程计在线，正在等待新地图。'; style = 'warning';
    } else if (health.status === 'mapping') {
      title = '数据已中断'; note = '未收到新鲜雷达数据，请检查设备与运行日志。'; style = 'warning';
    }
    el('mp-status').textContent = title;
    el('mp-status').className = 'status-pill '+style;
    el('mp-status-note').textContent = note;
    el('mp-session').textContent = connected ? (style === 'live' ? '实时数据已连接' : '设备已连接') : '设备未连接';
    el('mp-session').parentElement.classList.toggle('live', connected);
    for (const [key, label, ok] of [['scan','scan_age',scanning],['odom','odom_age',odometry],['imu','imu_age',imu]]) {
      const value = age(label);
      el('mp-'+key).textContent = !connected ? '未连接' : ok ? '在线' : Number.isFinite(value) ? '已过期' : '等待数据';
      el('mp-'+key+'-dot').className = ok ? 'live' : connected ? 'warning' : '';
      el('mp-'+key+'-age').textContent = Number.isFinite(value) ? value.toFixed(1)+' s 前' : '—';
    }
    el('mp-map-caption').textContent = !connected ? '连接已断开' : mapFresh ? '更新于 '+age('map_age').toFixed(1)+' s 前' : '等待新地图';
    el('mp-motor').textContent = !connected ? '未连接' : !armed ? '未启用' : isDriver() ? '已启用 · 当前页面控制' : '其他设备正在控制';
    el('mp-motor').className = armed && connected ? isDriver() ? 'live' : 'warning' : '';
    const positionFresh = connected && odometry && pose;
    el('mp-x').textContent = positionFresh ? pose.x.toFixed(2) : '—';
    el('mp-y').textContent = positionFresh ? pose.y.toFixed(2) : '—';
    el('mp-yaw').textContent = positionFresh ? (pose.yaw*180/Math.PI).toFixed(1)+'°' : '—';
    el('mp-x').closest('.map-position').classList.toggle('stale', !positionFresh);
    const acceleration = health?.imu_acceleration;
    const accelerationFresh = imu && Array.isArray(acceleration) && acceleration.length === 3
      && acceleration.every(value => typeof value === 'number' && Number.isFinite(value));
    for (const [index, axis] of ['x','y','z'].entries()) {
      el('mp-accel-'+axis).textContent = accelerationFresh ? acceleration[index].toFixed(2) : '—';
    }
    el('mp-accel-total').textContent = accelerationFresh ? Math.hypot(...acceleration).toFixed(2) : '—';
    el('mp-accel-state').textContent = !connected ? '未连接' : accelerationFresh ? '实时' : Number.isFinite(age('imu_age')) ? '数据过期或无效' : '等待数据';
    el('mp-accel-state').className = accelerationFresh ? 'live' : 'warning';
    const ready = connected && mapFresh && scanning && odometry && !health?.busy && !navigationBusy;
    saveButton.disabled = !ready || pendingName !== null;
    saveButton.textContent = pendingName !== null ? '保存中…' : '保存地图';
    if (pendingName !== null && performance.now()-pendingAt > 15000) {
      pendingName = null;
      feedback('保存结果尚未确认，请到控制台核对后再重试。', 'error');
    }
  }
  socket.on('mapping_state', state => { health = state; healthAt = performance.now(); paint(); });
  socket.on('pose', value => { pose = value; paint(); });
  socket.on('motor_state', paint);
  socket.on('you', paint);
  socket.on('connect', () => { health = null; pose = null; paint(); });
  socket.on('disconnect', () => {
    health = null; pose = null; navigationBusy = false;
    navigationLocalization=false;automaticBusy=false;returnPending=false;returnReady=false;returnNotice='';
    if (pendingName !== null) feedback('连接中断，保存结果未确认。重连后请核对地图列表。', 'error');
    pendingName = null;
    paint();
  });
  socket.on('navigation_state', state => {
    navigationBusy = !!state.busy;
    navigationLocalization=!!state.running&&state.mode!=='mapping';
    automaticBusy=!!state.navigating||!!state.autonomy?.active;
    if(returnPending&&!state.running&&!state.busy){returnPending=false;returnReady=true;returnNotice='';}
    if (pendingName !== null && !state.busy && state.message === '地图已保存：'+pendingName && (state.maps || []).includes(pendingName)) {
      feedback('已保存 '+pendingName+'，可以继续建图或停止。', 'success');
      pendingName = null;
    }
    paint();
  });
  socket.on('navigation_error', error => {
    if(returnPending){returnPending=false;returnNotice=error.message||'停止定位失败，请查看运行日志。';paint();}
    if (pendingName === null) return;
    pendingName = null; feedback(error.message || '保存失败，请查看运行日志。', 'error'); paint();
  });
  saveButton.addEventListener('click', () => {
    if (saveButton.disabled || !socket.connected || pendingName !== null) return;
    const name = nameInput.value.trim();
    if (!/^[A-Za-z0-9_-]{1,64}$/.test(name)) {
      feedback('请输入 1–64 个英文字母、数字、下划线或短横线。', 'error'); nameInput.focus(); return;
    }
    releaseAll();
    pendingName = name; pendingAt = performance.now();
    feedback('正在保存 '+name+'…'); paint();
    socket.emit('navigation_control', { action:'save', name });
  });
  setInterval(paint, 250);
  paint();
  resize();
})();
