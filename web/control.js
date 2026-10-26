/* The dedicated remote view reuses the established drive and gimbal inputs. */
(() => {
  if (!controlPage) return;
  const el = id => document.getElementById(id);
  document.title = '犀牛一号 · 遥控工作台';
  el('drive').classList.remove('collapsed');
  el('btn-drive-collapse').setAttribute('aria-expanded', 'true');
  el('drive-title').textContent = '驾驶控制';
  el('gimbal-title').textContent = '云台控制';
  el('control-gimbal-panel').append(el('gimbal'));

  let wanted = true, subscribed = false, pageActive = true;
  let frameAt = null, cameraError = '', decoded = 0, fps = null, fpsAt = performance.now();
  let frameAck = null, latency = null, clockOffset = null, bestRtt = Infinity, capturedAt = null;
  let frameUrl = null;
  const image = el('ctl-video');
  const available = () => wanted && pageActive && !document.hidden && socket.connected && cameraSocket.connected;
  function clearFrame() {
    if (frameAck) { const ack = frameAck; frameAck = null; ack(); }
    latency = null; capturedAt = null;
    frameAt = null; fps = null; decoded = 0; fpsAt = performance.now();
    image.hidden = true;
    image.removeAttribute('src');
    if (frameUrl) { URL.revokeObjectURL(frameUrl); frameUrl = null; }
    el('ctl-video-format').textContent = '—';
  }
  function paint() {
    const live = available() && frameAt !== null && performance.now()-frameAt < 2000;
    image.hidden = !live;
    el('ctl-video-placeholder').hidden = live;
    el('ctl-camera-dot').classList.toggle('live', live);
    el('ctl-camera-state').textContent = live ? '实时' : !socket.connected || !cameraSocket.connected ? '未连接' : !available() ? '已暂停' : '等待画面';
    el('ctl-video-message').textContent = !socket.connected ? '连接已断开，等待重连'
      : !available() ? '摄像头画面已暂停' : cameraError || (frameAt === null ? '正在连接摄像头…' : '画面已中断，正在重试…');
    el('ctl-video-help').textContent = !wanted ? '点击「恢复画面」继续查看。' : '驾驶与云台独立控制，松开摇杆即停止。';
    el('ctl-video-toggle').textContent = wanted ? '暂停画面' : '恢复画面';
    el('ctl-video-toggle').setAttribute('aria-pressed', String(wanted));
    el('ctl-video-fps').textContent = live && fps !== null ? fps.toFixed(0)+' FPS' : '—';
    el('ctl-video-latency').textContent = live && latency !== null ? '≈'+Math.round(latency)+' ms' : '— ms';
    const owner = !socket.connected ? '未连接 · 控制已暂停' : !armed ? '已连接 · 电机未启用'
      : isDriver() ? '驾驶权在当前页面' : '其他设备正在驾驶';
    el('ctl-owner').textContent = owner;
    el('ctl-owner').classList.toggle('live', socket.connected && armed && isDriver());
    el('ctl-motor-badge').textContent = !socket.connected ? '控制连接已断开' : !armed ? '电机未启用'
      : isDriver() ? '电机已启用 · 当前页面控制' : '其他设备正在驾驶';
    el('ctl-motor-badge').classList.toggle('live', socket.connected && armed);
  }
  function sync() {
    const next = available();
    if (next !== subscribed) {
      if (cameraSocket.connected) cameraSocket.emit('camera_control', {action:next ? 'start' : 'stop', flow_control:true, flow_window:2, binary:true});
      subscribed = next;
      clearFrame();
    }
    paint();
  }
  el('ctl-video-toggle').addEventListener('click', () => { wanted = !wanted; cameraError = ''; sync(); });
  image.addEventListener('load', () => {
    if (!available() || !image.hasAttribute('src')) return;
    frameAt = performance.now(); cameraError = ''; decoded++;
    latency = capturedAt !== null && clockOffset !== null ? Math.max(0, Date.now()+clockOffset-capturedAt*1000) : null;
    if (frameAck) { const ack = frameAck; frameAck = null; ack(); }
    const elapsed = frameAt-fpsAt;
    if (elapsed >= 1000) { fps = decoded*1000/elapsed; decoded = 0; fpsAt = frameAt; }
    el('ctl-video-format').textContent = image.naturalWidth+' × '+image.naturalHeight;
    paint();
  });
  image.addEventListener('error', () => {
    if (!available()) return;
    clearFrame(); cameraError = '画面加载失败，正在重试…'; paint();
  });
  cameraSocket.on('frame', (data, ack) => {
    const binary = data?.jpeg instanceof ArrayBuffer || ArrayBuffer.isView(data?.jpeg) || data?.jpeg instanceof Blob;
    if (!available() || (!binary && (typeof data?.img !== 'string' || !data.img))) { if (typeof ack === 'function') ack(); return; }
    if (frameAck) frameAck();
    frameAck = typeof ack === 'function' ? ack : null;
    capturedAt = typeof data.captured_at === 'number' && Number.isFinite(data.captured_at) ? data.captured_at : null;
    const previousUrl = frameUrl;
    frameUrl = binary ? URL.createObjectURL(new Blob([data.jpeg], {type:'image/jpeg'})) : null;
    image.src = frameUrl || 'data:image/jpeg;base64,'+data.img;
    if (previousUrl) URL.revokeObjectURL(previousUrl);
  });
  cameraSocket.on('camera_state', state => {
    if (!available()) return;
    if (state?.error) { clearFrame(); cameraError = '摄像头暂不可用，正在重试…'; }
    paint();
  });
  function syncClock() {
    if (!available()) return;
    const start = performance.now(), wall = Date.now();
    cameraSocket.emit('camera_sync', serverTime => {
      const rtt = performance.now()-start;
      if (!available() || typeof serverTime !== 'number' || !Number.isFinite(serverTime)) return;
      if (rtt < bestRtt) {
        bestRtt = rtt; clockOffset = serverTime*1000-wall-rtt/2;
      }
    });
  }
  cameraSocket.on('connect', () => { subscribed = false; bestRtt = Infinity; clockOffset = null; sync(); syncClock(); });
  cameraSocket.on('disconnect', () => { subscribed = false; clockOffset = null; clearFrame(); paint(); });
  socket.on('connect', sync);
  socket.on('disconnect', sync);
  document.addEventListener('visibilitychange', sync);
  window.addEventListener('pagehide', () => { pageActive = false; sync(); });
  window.addEventListener('pageshow', () => { pageActive = true; sync(); });
  sync();
  syncClock();
  setInterval(syncClock, 2000);
  setInterval(paint, 250);
})();
