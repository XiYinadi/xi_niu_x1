/* Stage 3 workspace. Reuses the existing gated Nav2 client and driver identity. */
(() => {
 if(!document.body.classList.contains('navigation-page'))return;
 const el=id=>document.getElementById(id);
 const move=(parent,...ids)=>ids.forEach(id=>el(parent).append(el(id)));
 let nav=null,feed=null,feedAt=0,calibration=null,navAt=0,lastRefresh=0;
 document.title='犀牛一号 · 定位与导航';
 el('mission-title').textContent='定位与导航';
 el('mission-description').textContent='加载地图、确认当前位置，预览路径，再开始监督导航。';
 document.querySelector('.mission-heading .page-eyebrow').textContent='STAGE 03 / NAVIGATION';
 const tabs=document.querySelector('.mission-tabs');
 const entry=document.createElement('a');entry.href='/navigation';entry.textContent='定位导航';entry.setAttribute('aria-current','page');tabs.prepend(entry);
 el('win-map').querySelector('.t').textContent='地图、定位与规划路径';
 el('map-empty').querySelector('strong').textContent='等待定位地图';
 el('map-empty').querySelector('span').textContent='选择已保存的地图并加载定位；在地图上设置机器人实际位置与朝向。';
 el('mission-setup-title').textContent='加载地图与定位';
 el('mission-setup-note').textContent='加载会切换当前建图管线。先保存需要保留的地图；车保持不动并停用电机。';
 const label=document.createElement('label');label.textContent='已保存地图';label.append(el('nav-map'));el('mission-setup-controls').append(label);
 move('mission-setup-controls','nav-load','nav-initial','nav-stop');
 el('mission-route-title').textContent='选择目标与朝向';
 move('mission-route-controls','nav-target');
 el('mission-preview-note').textContent='在地图按下目标位置，拖向期望车头方向并松手。只预览路径，不会自动行驶。';
 move('mission-run-controls','btn-arm','nav-go','nav-cancel');
 move('mission-runtime','nav-status','nav-path');
 const pose=document.createElement('p');pose.id='navigation-pose';pose.textContent='等待定位';el('mission-runtime').prepend(pose);
 const locationWarning=document.createElement('div');locationWarning.id='navigation-location-warning';locationWarning.className='navigation-location-warning';locationWarning.setAttribute('role','status');locationWarning.hidden=true;el('map-body').append(locationWarning);
 const selectionError=document.createElement('p');selectionError.id='navigation-selection-error';selectionError.className='navigation-selection-error';selectionError.setAttribute('role','alert');selectionError.hidden=true;el('mission-setup').append(selectionError);
 socket.on('navigation_error',d=>{selectionError.textContent=d.message||'定位操作失败，请重试';selectionError.hidden=false;});
 for(const id of ['nav-initial','nav-target','nav-load'])el(id).addEventListener('click',()=>{selectionError.hidden=true;});
 const drawLocatedRobot=drawRobot;
 drawRobot=function(){if(socket.connected&&nav?.localized&&nav?.pose&&robotPose)drawLocatedRobot();};
 document.querySelector('.mission-progress').hidden=true;
 el('mission-calibration').open=false;
 el('mission-measurements').textContent='正在读取实测标定配置…';
 function age(key){const value=feed?.[key];return socket.connected&&Number.isFinite(value)?value+(performance.now()-feedAt)/1000:Infinity;}
 function paint(){
  const connected=socket.connected,trial=nav?.drive_mode==='supervised_trial';
  const calibrated=trial ? ('drive_error' in (nav||{})&&!nav.drive_error) : !!calibration?.calibrated&&!nav?.calibration_error;
  const sensors=age('scan_age')<.5&&age('odom_age')<1&&age('imu_age')<.75;
  const driver=armed&&mySid!==null&&driverSid===mySid;
  const checks=[['雷达 / 里程计 / GY-521',sensors,sensors?'数据在线':'等待新鲜数据'],
    ['定位',!!nav?.localized,nav?.localized?'位置已收敛':'需加载地图并确认当前位置'],
    ['路径规划',!!nav?.planner_ready,nav?.planner_ready?'服务已就绪':'尚未准备'],
    ['导航与避障',!!nav?.navigator_ready,nav?.navigator_ready?'服务已就绪':'尚未准备'],
    [trial?'低速试航控制':'实测转角与车速',calibrated,calibrated?(trial?'速度反馈 · 限油门10%':'配置已校验'):'控制配置尚未就绪'],
    ['当前页面驾驶权',driver,driver?'当前页面持有':'尚未启用电机']];
  const list=el('mission-checks');list.replaceChildren();
  for(const [name,ok,note] of checks){const li=document.createElement('li'),dot=document.createElement('i'),span=document.createElement('span'),small=document.createElement('small');dot.className=connected&&ok?'live':'';span.textContent=name;small.textContent=connected?note:'连接已断开';li.append(dot,span,small);list.append(li);}
  const ready=connected&&checks.every(c=>c[1])&&!nav?.busy&&!nav?.navigating;
  el('mission-ready').textContent=!connected?'未连接':nav?.navigating?'正在导航':!calibrated?'可定位 · 待标定':ready?(trial?'可以低速试航':'可以导航'):'待准备';
  el('mission-ready').className='status-pill '+(ready||nav?.navigating?'live':'warning');
  el('mission-run-note').textContent=trial?'低速试航：选1米内目标并预览路径，启用电机后点开始导航。限速0.05m/s、每次最多45秒；到点、近障碍、离开页面或断线会停用。距离约5%的偏差仍待精调。':!calibrated?'定位和路径预览可以使用。自动行驶需先实测转角、车速并核对安装位置；当前保持锁定。':'现场有人监督，确认路线和车头方向后启用电机，再点开始导航。切换页面或断线会停止任务。';
  el('btn-arm').disabled=!connected||(!armed&&(!calibrated||!nav?.localized||!sensors||!!nav?.busy));
  if(!ready)el('nav-go').disabled=true;
  if(!connected){for(const id of ['nav-load','nav-initial','nav-stop','nav-target','nav-cancel'])el(id).disabled=true;}
  else {el('nav-load').disabled=!!nav?.busy||!!nav?.running||armed||!el('nav-map').value;el('nav-cancel').disabled=!nav?.pending&&!nav?.navigating&&nav?.status!=='planning';}
  if(connected&&nav?.localized&&nav?.pose){const p=nav.pose;pose.textContent=`地图位置 X ${p.x.toFixed(2)} m · Y ${p.y.toFixed(2)} m · 航向 ${(p.yaw*180/Math.PI).toFixed(1)}°`;}
  else pose.textContent=connected?'等待定位收敛；请核对地图箭头与实际车头方向。':'连接已断开';
  locationWarning.hidden=!mapImg||(!canvas.navPick&&connected&&!!nav?.localized&&!!nav?.pose);
  locationWarning.textContent=canvas.navPick?(canvas.navPickNotice||'正在选点：按住当前位置，沿车头方向拖动，松开提交'):!connected?'连接中断，当前位置未知':nav?.running?'尚未定位：请保持停车，设置当前位置与车头方向':'请加载地图并设置当前位置';
 }
 async function readiness(){try{const r=await fetch('/api/missions/readiness',{cache:'no-store'});if(!r.ok)throw Error();calibration=await r.json();}catch(_){calibration=null;}
  const m=calibration?.measurements||{};const box=el('mission-measurements');box.replaceChildren();
  for(const [name,value]of [['轴距 / 长 / 宽',[m.wheelbase_m,m.length_m,m.width_m].every(Number.isFinite)?[m.wheelbase_m,m.length_m,m.width_m].map(v=>(v*100).toFixed(1)).join(' / ')+' cm':'等待读取'],['前轮转向',Number.isFinite(m.trial_max_steering_rad)?'试航限制 '+(m.trial_max_steering_rad*180/Math.PI).toFixed(0)+'°':Number.isFinite(m.max_steering_rad)?(m.max_steering_rad*180/Math.PI).toFixed(1)+'°':'缺少实测'],['车速控制',Number.isFinite(m.trial_pwm_cap)?'速度反馈 · 油门上限 '+(m.trial_pwm_cap*100).toFixed(0)+'%':Number.isFinite(m.full_throttle_mps)?m.full_throttle_mps.toFixed(3)+' m/s（配置）':'缺少实测'],['导航限速',Number.isFinite(m.max_speed_mps)?m.max_speed_mps.toFixed(2)+' m/s（配置）':'等待读取']]){const row=document.createElement('p'),strong=document.createElement('strong');row.textContent=name;strong.textContent=value;row.append(strong);box.append(row);}paint();
 }
 socket.on('navigation_state',d=>{nav=d;navAt=performance.now();paint();});
 socket.on('mapping_state',d=>{feed=d;feedAt=performance.now();paint();});
 socket.on('motor_state',paint);
 socket.on('connect',()=>{nav=null;feed=null;readiness();paint();});
 socket.on('disconnect',()=>{nav=null;feed=null;paint();});
 el('nav-map').addEventListener('change',paint);
 readiness();paint();setInterval(()=>{
  paint();
  const now=performance.now();
  const waiting=nav?.busy||(nav?.running&&!nav?.initial_pose_ready);
  if(socket.connected&&!document.hidden&&(waiting||now-navAt>1500)&&now-lastRefresh>1000){
   lastRefresh=now;socket.emit('navigation_control',{action:'status'});
  }
 },250);
})();
