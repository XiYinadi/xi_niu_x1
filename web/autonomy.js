// Shares the driving page socket and owner identity; never arms a motor.
(() => {
  const el=id=>document.getElementById(id);let state=null;
  const send=(action,extra={})=>{if(socket.connected)socket.emit('navigation_control',{action,...extra});};
  el('auto-prepare').addEventListener('click',()=>{
    if(confirm('准备会停止当前普通建图并重新建立地图坐标。此操作不驱动车轮，继续？'))send('prepare_mapping');
  });
  el('auto-preview').addEventListener('click',()=>send('explore_preview'));
  el('auto-explore').addEventListener('click',()=>{
    if(!state||state.calibration_error||!state.autonomy?.camera_ready)return;
    if(confirm('确认机器人在开阔地面、现场有人看护，并已由当前页面启用电机？开始后将低速自主建图。'))send('explore_start',{name:el('nav-name').value});
  });
  el('auto-finish').addEventListener('click',()=>send('explore_finish'));
  el('auto-patrol').addEventListener('click',()=>{
    if(!state||state.calibration_error||!state.autonomy?.camera_ready)return;
    if(confirm('执行已保存的所选巡检任务？机器人将自主前往各点；疑似火情会停车并上报。'))send('patrol_start',{name:el('patrol-list').value});
  });
  el('auto-stop').addEventListener('click',()=>send('mission_stop'));
  el('auto-ack').addEventListener('click',()=>{
    if(confirm('确认已人工检查现场？确认只关闭告警，不会重新启用电机或恢复任务。'))send('alert_ack');
  });
  function paint(d){
    state=d;const a=d.autonomy||{},active=!!a.active,ready=!d.busy&&!d.calibration_error&&d.localized&&d.navigator_ready&&a.camera_ready&&!active;
    el('auto-prepare').disabled=d.busy||d.running||active;
    el('auto-preview').disabled=d.busy||!d.localized||active;
    el('auto-explore').disabled=!ready||d.mode!=='mapping';
    el('auto-finish').disabled=!active||a.kind!=='explore';
    el('auto-patrol').disabled=!ready||d.mode==='mapping'||!(d.patrols||[]).length;
    el('auto-ack').disabled=active||!a.alert||!!a.alert.acknowledged_at;
    const preview=a.preview;
    el('auto-status').textContent=(a.message||'自动任务模块准备中')+(a.completed_points!==undefined?'；已完成 '+a.completed_points+' 点':'')
      +(d.calibration_error?'；'+d.calibration_error:'')
      +(preview?'；'+(preview.error||'探索边界 '+preview.frontier_groups+' 组，可选目标 '+preview.candidates.length+' 个'):'');
    const alarm=a.alert&&!a.alert.acknowledged_at;
    el('autonomy-alert').hidden=!alarm;
    el('autonomy-alert-evidence').hidden=!alarm||!a.alert.image;if(alarm&&a.alert.image)el('autonomy-alert-evidence').href='/autonomy/evidence/'+a.alert.image;
    if(alarm)el('autonomy-alert-message').textContent='⚠ 已停车，等待人工复核：'+a.alert.reason+'（检测置信度 '+Math.round(a.alert.confidence*100)+'%）';
    el('auto-report').hidden=!a.inspection_id||active||!a.report_ready;
    if(a.inspection_id)el('auto-report').href='/inspection/reports/'+a.inspection_id;
  }
  socket.on('navigation_state',paint);
  socket.on('autonomy_alert',alert=>{
    el('autonomy-alert-evidence').hidden=!alert.image;if(alert.image)el('autonomy-alert-evidence').href='/autonomy/evidence/'+alert.image;
    el('autonomy-alert').hidden=false;
    el('autonomy-alert-message').textContent='⚠ 已停车，等待人工复核：'+alert.reason;
  });
  socket.on('disconnect',()=>{
    state=null;el('auto-status').textContent='连接中断，自动任务将停止；重连后不会自行恢复';
    for(const id of ['auto-prepare','auto-preview','auto-explore','auto-patrol','auto-finish','auto-ack'])el(id).disabled=true;
  });
})();
