/* Separate mission workspaces reuse existing gated navigation/autonomy handlers. */
(() => {
  const el=id=>document.getElementById(id);
  const menu=document.querySelector('.mission-menu');
  if(menu){
    menu.classList.toggle('selected',missionPage);
    menu.querySelectorAll('a').forEach(a=>a.addEventListener('click',()=>{menu.open=false;}));
    document.addEventListener('click',e=>{if(!menu.contains(e.target))menu.open=false;});
  }
  if(!missionPage||document.body.classList.contains('navigation-page'))return;
  const explore=document.body.classList.contains('explore-page');
  let state=null,feed=null,feedAt=0,calibration=null,preview=null;
  const move=(parent,...ids)=>ids.forEach(id=>el(parent).append(el(id)));
  const label=(parent,text,id)=>{const l=document.createElement('label');l.textContent=text;el(parent).append(l);l.append(el(id));};
  el('mission-title').textContent=explore?'自动建图':'路线巡检';
  document.title='犀牛一号 · '+el('mission-title').textContent;
  el('mission-description').textContent=explore?'预览安全探索目标，低速探索，完成后保存地图。':'载入地图，配置巡检点，逐点检测并归档现场证据。';
  document.querySelectorAll('.mission-tabs a').forEach(a=>{
    if(a.getAttribute('href')===(explore?'/explore':'/patrol'))a.setAttribute('aria-current','page');
  });
  el('win-map').querySelector('.t').textContent=explore?'探索地图与观察目标':'巡检地图与路线';
  el('map-empty').querySelector('strong').textContent='等待任务地图';
  el('map-empty').querySelector('span').textContent=explore?'可先查看当前地图；「准备新建图」会明确确认后重建地图坐标。':'选择已保存地图并加载定位，再在地图上设置当前位置与朝向。';
  el('mission-setup-title').textContent=explore?'准备新建图':'加载地图与定位';
  el('mission-setup-note').textContent=explore?'准备会停止普通建图并重新建立坐标。请先在手动建图页保存需要保留的地图。':'选择地图并加载，在地图按下、拖出朝向，松手确认机器人的真实位置。';
  if(explore){
    label('mission-setup-controls','新地图名称','nav-name');
    move('mission-setup-controls','auto-prepare','nav-stop');
    el('auto-prepare').textContent='准备新建图（不行驶）';
    el('mission-route-title').textContent='探索目标预览';
    move('mission-route-controls','auto-preview');
    move('mission-run-controls','auto-explore','auto-finish');
    el('mission-preview-note').textContent='候选观察点标在已知可通行区域；预览不会启用电机或自动前进。';
  }else{
    label('mission-setup-controls','已保存地图','nav-map');
    move('mission-setup-controls','nav-load','nav-initial','nav-stop','nav-cancel');
    el('mission-route-title').textContent='巡检路线与点位';
    move('mission-route-controls','patrol-panel');
    el('patrol-panel').open=true;
    el('patrol-panel').querySelector('.task-editor').open=true;
    move('mission-run-controls','auto-patrol');
    el('mission-preview-note').textContent='新建或载入任务，在地图添加点位与朝向，保存后预览路线。巡检发现疑似火情会停车并保留证据。';
  }
  // Same driver SID, arm button and emergency stop; no second control client.
  move('mission-run-controls','btn-arm');
  move('mission-runtime','nav-status','nav-path','auto-status');
  move('mission-runtime-controls','auto-stop','auto-ack','auto-report');
  el('auto-stop').textContent='停止自动任务';
  el('auto-report').textContent='查看本次巡检报告 ↗';
  function age(key){
    const v=feed?.[key];return socket.connected&&typeof v==='number'&&Number.isFinite(v)?v+(performance.now()-feedAt)/1000:Infinity;
  }
  function measurements(){
    const box=el('mission-measurements');box.replaceChildren();
    const m=calibration?.measurements||{};
    const rows=[['轴距 / 长 / 宽',[m.wheelbase_m,m.length_m,m.width_m].every(Number.isFinite)?[m.wheelbase_m,m.length_m,m.width_m].map(x=>(x*100).toFixed(1)).join(' / ')+' cm（配置值）':'等待读取'],
      ['最大车轮转角',typeof m.max_steering_rad==='number'?(m.max_steering_rad*180/Math.PI).toFixed(1)+'°':'缺少实测'],
      ['油门与实际车速',typeof m.full_throttle_mps==='number'?m.full_throttle_mps.toFixed(3)+' m/s（配置值）':'缺少实测'],
      ['自主行驶验证',calibration?.calibrated?'标定配置已通过校验':'尚未确认']];
    for(const [name,value]of rows){const row=document.createElement('p'),strong=document.createElement('strong');row.textContent=name;strong.textContent=value;row.append(strong);box.append(row);}
  }
  function paint(){
    const a=state?.autonomy||{},connected=socket.connected,active=connected&&!!a.active;
    const calibrated=connected&&calibration?.calibrated===true&&!state?.calibration_error;
    const map=age('map_age')<10,scan=age('scan_age')<.5,odom=age('odom_age')<1,imu=age('imu_age')<.75;
    const pose=connected&&!!state?.localized,planner=connected&&!!state?.navigator_ready;
    const camera=connected&&!!a.camera_ready,owner=connected&&armed&&mySid!==null&&driverSid===mySid;
    const checks=[['底盘实测标定',calibrated,calibrated?'配置已校验':'缺少速度、转角或实车确认'],
      ['雷达 / 里程计 / GY-521',scan&&odom&&imu,scan&&odom&&imu?'数据在线':'等待新鲜数据'],
      ['地图与定位',map&&pose,map&&pose?'已就绪':'需准备建图或加载地图定位'],
      ['规划与避障服务',planner,planner?'已就绪':'尚未准备'],
      ['相机火情检测',camera,camera?'检测链路在线':'等待有效检测'],
      ['当前页面驾驶权',owner,owner?'当前页面持有':'启动前需手动启用电机']];
    const list=el('mission-checks');list.replaceChildren();
    for(const [name,ok,note]of checks){const li=document.createElement('li'),dot=document.createElement('i'),text=document.createElement('span'),small=document.createElement('small');dot.className=ok?'live':'';text.textContent=name;small.textContent=connected?note:'连接已断开';li.append(dot,text,small);list.append(li);}
    const alarm=!!a.alert&&!a.alert.acknowledged_at;
    const ready=checks.every(c=>c[1])&&!state?.busy&&!state?.navigating&&!active&&!alarm;
    el('mission-ready').textContent=!connected?'未连接':alarm?'告警待确认':active?'任务运行中':!calibrated?'待实车标定':ready?'可以启动':'待准备';
    el('mission-ready').className='status-pill '+(active||ready?'live':connected?'warning':'');
    if(!connected)for(const id of ['auto-prepare','auto-preview','nav-load','nav-initial','patrol-save','patrol-add','patrol-preview','auto-finish'])el(id).disabled=true;
    // Supplemental UI gates; the existing server independently validates all of them.
    el('auto-explore').disabled=!ready||state?.mode!=='mapping';
    el('auto-patrol').disabled=!ready||state?.mode==='mapping'||!el('patrol-list').value;
    el('auto-stop').disabled=!connected||!active;
    if(active)for(const id of ['patrol-add','patrol-save','patrol-load','patrol-new','patrol-preview'])el(id).disabled=true;
    el('mission-run-note').textContent=active?'执行时保持本页可见；切换页面、断线或失去驾驶权会停止任务，不会自行恢复。'
      :!calibrated?'自动行驶保持锁定。现在可以准备地图、配置点位和查看路径预览；实测标定完成后再启用。':'现场有人监督，确认路线与当前位置后，再由当前页面启用电机并启动。';
    const stages={idle:'等待任务',stopped:'已停止',interrupted:'任务已中断',exploring:'选择探索目标',navigating:'前往目标',dwell:'到点等待',detecting:'正在检测',reporting:'正在生成报告',saving:'正在保存地图',completed:'任务已完成',blocked:'任务受阻',alert:'等待人工复核'};
    el('mission-stage').textContent=connected?(stages[a.status]||'等待任务状态'):'连接中断';
    el('mission-progress').textContent=String(a.completed_points||0);
    preview=a.preview||null;
    el('mission-preview-note').classList.toggle('warning',!!preview?.error);
  }
  async function loadCalibration(){
    try{const response=await fetch('/api/missions/readiness',{cache:'no-store'});if(!response.ok)throw Error('readiness');calibration=await response.json();}
    catch(_){calibration=null;}
    measurements();paint();
  }
  socket.on('navigation_state',d=>{state=d;paint();});
  socket.on('mapping_state',d=>{feed=d;feedAt=performance.now();paint();});
  socket.on('connect',()=>{state=null;feed=null;loadCalibration();paint();});
  socket.on('disconnect',()=>{state=null;feed=null;paint();});
  socket.on('motor_state',paint);
  const originalRender=render;
  render=function(ts){
    originalRender(ts);
    if(!explore||!preview?.candidates)return;
    ctx.save();ctx.strokeStyle='#8ae3c5';ctx.fillStyle='#b4f2d7';ctx.font='12px sans-serif';
    preview.candidates.forEach((p,i)=>{const v=w2s(p.x,p.y);ctx.beginPath();ctx.arc(v.x,v.y,8,0,Math.PI*2);ctx.stroke();ctx.fillText(String(i+1),v.x+11,v.y-9);});
    ctx.restore();
  };
  measurements();paint();loadCalibration();setInterval(paint,250);
})();
