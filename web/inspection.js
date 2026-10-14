'use strict';
(() => {
  const $=id=>document.getElementById(id);
  const escape=value=>String(value??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const names={running:'进行中',sampling:'检测中',reporting:'生成报告',completed:'已结束'};
  const verdicts={clear:'未见明显火情',suspected:'疑似火情',uncertain:'待复核'};
  let state=null,busy=false,polling=false;
  const notice=message=>{ $('notice').hidden=!message;$('notice').textContent=message||''; };
  async function api(path,data) {
    const response=await fetch(path,data===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
    let result;try { result=await response.json(); } catch (_) { throw new Error('设备返回了无效响应，请稍后重试'); }
    if (!response.ok) throw new Error(result.error||'请求失败');
    return result;
  }
  const time=value=>value?new Date(typeof value==='number'?value*1000:value).toLocaleString('zh-CN',{hour12:false}):'未记录';
  function render() {
    if (!state) return;
    const s=state.current,m=state.model||{},active=s&&s.status!=='completed',stats=s&&s.stats,automatic=s&&s.mode==='automatic';
    $('mode-tag').textContent=automatic?'自动巡检':'人工巡检';
    $('model-pill').classList.toggle('online',!!m.online);
    $('model-status').textContent=m.online?'本地视觉模型在线':'本地模型暂不可用';
    $('model-pill').title=m.name||m.error||'';
    $('camera-status').textContent=m.camera_online?'相机已连接':'相机未就绪';
    const previous=$('task-select').value;
    $('task-select').innerHTML='<option value="">不绑定路线 · 定点检测</option>'+(state.tasks||[]).map(t=>'<option value="'+escape(t.name)+'">'+escape(t.name)+' · '+t.points+'点 × '+t.cycles+'轮</option>').join('');
    if ((state.tasks||[]).some(t=>t.name===previous)) $('task-select').value=previous;
    $('setup').hidden=!!active;
    $('start-button').disabled=busy;
    $('metric-checked').textContent=stats?String(stats.checked):'—';
    $('metric-planned').textContent=s?(s.planned_steps===null?'定点检测，无预设路线':'计划 '+s.planned_steps+' 个点次'):'等待开始';
    $('metric-suspected').textContent=stats?String(stats.suspected):'—';
    $('metric-missing').textContent=stats?String(stats.missing+stats.uncertain):'—';
    $('metric-status').textContent=s?names[s.status]||s.status:'待开始';
    $('session-time').textContent=s?'开始于 '+time(s.started_at):'尚无进行中的巡检';
    $('session-name').textContent=s?s.title:'等待开始巡检';
    if (active && s.task && s.next_step<s.planned_steps) {
      const n=s.task.waypoints.length,p=s.task.waypoints[s.next_step%n];
      $('current-point').textContent='第 '+(Math.floor(s.next_step/n)+1)+' 轮 · 点位 '+(s.next_step%n+1);
      $('point-description').textContent='计划位置 ('+p.x.toFixed(2)+', '+p.y.toFixed(2)+') m · '+s.task.map+(automatic?'。由导航确认到达后自动停车检测。':'。请人工确认到达后检测。');
    } else {
      $('current-point').textContent=active?'检测点 '+(s.next_step+1):s?'本次巡检已结束':'尚未创建任务';
      $('point-description').textContent=active?'确认机器人已到达需要检查的位置，保持相机画面稳定。':s?stats.overall:'选择路线，或直接开始定点检测。';
    }
    $('progress-bar').style.width=s&&s.planned_steps?Math.min(100,s.next_step/s.planned_steps*100)+'%':'0%';
    $('progress-text').textContent=s?(s.planned_steps===null?'已完成 '+s.next_step+' 个定点记录':'已处理 '+s.next_step+' / '+s.planned_steps+' 个点次；有效检测 '+stats.checked+' 个'):'点位检测后自动记录证据';
    $('operation-message').textContent=s?s.message:'本页记录人工巡检，不会自行驱动车轮。';
    $('capture-button').disabled=automatic||busy||!s||s.status!=='running'||!m.online||!m.camera_online;
    $('capture-button').textContent=s&&s.status==='sampling'?'本地模型正在检测…':'已到达此点，开始检测 ＋';
    $('skip-button').disabled=automatic||busy||!s||s.status!=='running';
    $('finish-button').disabled=automatic||busy||!s||s.status!=='running';
    $('report-actions').hidden=!s||s.status!=='completed';
    if (s) {
      $('open-report').href='/inspection/reports/'+s.id;
      $('retry-report').hidden=s.report_status==='ready';
      $('retry-report').disabled=busy;
    }
    const records=s?s.records:[];
    $('records-count').textContent=stats?stats.attempts+' 次记录 · 展示最近 12 条':'尚无记录';
    $('records').innerHTML=records.length?records.slice().reverse().map(r=>'<article class="evidence-card">'
      +(r.image?'<a href="/inspection/evidence/'+s.id+'/'+r.image+'" target="_blank" rel="noopener"><img loading="lazy" src="/inspection/evidence/'+s.id+'/'+r.image+'" alt="'+escape(r.label)+'的检测证据"></a>':'')
      +'<div class="evidence-body"><h3>'+escape(r.label)+'<span class="verdict-'+escape(r.verdict||'uncertain')+'">'+escape(r.status==='ok'?verdicts[r.verdict]:r.status==='skipped'?'未检测 · 已跳过':'检测失败')+'</span></h3>'
      +'<p>'+escape(r.reason||r.error||'')+'</p><small>'+time(r.captured_at||r.requested_at)+(r.status==='ok'?' · 判断置信度 '+Math.round(r.confidence*100)+'%':'')+'</small></div></article>').join(''):'<div class="empty-state">尚未产生检测记录。到达点位后点击检测，图片和结果将成对保存。</div>';
    $('history-list').innerHTML=(state.history||[]).length?state.history.map(h=>'<div class="history-row"><div><strong>'+escape(h.title)+'</strong><small>'+time(h.started_at)+' · 有效检测 '+h.stats.checked+' / '+h.stats.planned+' · 疑似异常 '+h.stats.suspected+'</small></div>'
      +(h.status==='completed'?'<a href="/inspection/reports/'+h.id+'" target="_blank" rel="noopener">'+(h.report_status==='ready'?'查看报告':'查看原始记录')+' ↗</a>':'<small>'+escape(names[h.status]||h.status)+'</small>')+'</div>').join(''):'<div class="empty-state">完成首次巡检后，报告会出现在这里。</div>';
  }
  async function refresh() { state=await api('/api/inspection/state');render(); }
  async function action(name,data={}) {
    if (busy) return;
    busy=true;notice('');render();
    try {
      const path=name==='start'?'/api/inspection/start':'/api/inspection/'+state.current.id+'/'+name;
      await api(path,data);await refresh();
    } catch(e) { notice(e.message); }
    finally { busy=false;render(); }
  }
  $('start-form').addEventListener('submit',e=>{e.preventDefault();action('start',{title:$('session-title').value,task:$('task-select').value});});
  $('capture-button').addEventListener('click',()=>action('sample'));
  $('skip-button').addEventListener('click',()=>{if(confirm('跳过此点？报告会记录为未检测。'))action('skip');});
  $('finish-button').addEventListener('click',()=>{
    if (state.current.stats.missing && !confirm('仍有未检测点。结束后报告会如实保留未完成项，是否继续？')) return;
    action('finish');
  });
  $('retry-report').addEventListener('click',()=>action('retry'));
  $('camera').addEventListener('error',()=>{$('camera-status').textContent='暂时无法获取画面';});
  setInterval(()=>{if(!document.hidden)$('camera').src='/api/inspection/camera?t='+Date.now();},1300);
  async function poll() {
    if (!polling) {
      polling=true;
      try { await refresh(); } catch(e) { notice('设备连接中断：'+e.message);$('capture-button').disabled=true; }
      finally { polling=false; }
    }
    setTimeout(poll,document.hidden?5000:1500);
  }
  poll();
})();
