'use strict';
(() => {
 const el=id=>document.getElementById(id),canvas=el('edit-canvas'),ctx=canvas.getContext('2d');
 const state={source:null,image:null,originalImage:null,operations:[],redo:[],draft:[],mode:'view',
  revision:0,previewRevision:-1,savedRevision:-1,sourceBusy:false,previewBusy:false,saving:false,
  changed:0,previewId:0,saveRequest:null,view:{x:0,y:0,scale:1},pointer:null,space:false};
 let cw=1,ch=1;
 const dirty=()=>state.operations.length>0&&state.savedRevision!==state.revision;
 const busy=()=>state.sourceBusy||state.previewBusy||state.saving;
 const status=(text,error=false)=>{el('editor-status').textContent=text;el('editor-status').className=error?'error':'';};
 async function api(path,data){
  const response=await fetch('/api/map-editor/'+path,data===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
  let result;try{result=await response.json();}catch(_){throw Error('服务响应异常，请稍后重试');}
  if(!response.ok||result.error)throw Error(result.error||'请求失败');return result;
 }
 async function decode(data){const image=new Image();image.src='data:image/png;base64,'+data;await image.decode();return image;}
 function buttons(){
  const loaded=!!state.source,locked=busy(),minimum=state.mode==='unknown'?3:2;
  el('load-live').disabled=locked;el('load-saved').disabled=locked||!el('map-list').value;el('map-list').disabled=locked;
  for(const mode of ['view','unknown','wall']){const b=el('tool-'+mode);b.disabled=!loaded||locked;b.classList.toggle('selected',state.mode===mode);b.setAttribute('aria-pressed',String(state.mode===mode));}
  el('finish').disabled=locked||state.mode==='view'||state.draft.length<minimum;
  el('cancel-draft').disabled=locked||!state.draft.length;
  el('undo').disabled=locked||(!state.operations.length&&!state.draft.length);
  el('redo').disabled=locked||!state.redo.length||!!state.draft.length;
  el('reset').disabled=locked||(!state.operations.length&&!state.draft.length);
  el('save-map').disabled=locked||!loaded||!state.operations.length||!!state.draft.length||state.previewRevision!==state.revision||state.changed===0;
  el('save-map').textContent=state.saving?'正在保存…':'保存为新地图';el('save-name').disabled=state.saving||state.sourceBusy;
  el('wall-width').disabled=locked||state.mode!=='wall';
  for(const id of ['zoom-in','zoom-out','zoom-fit'])el(id).disabled=!loaded;
  el('draft-count').textContent=state.draft.length?`已选 ${state.draft.length} 个点，点击「完成标注」生成预览`:'尚无待完成的标注';
  el('operation-count').textContent=state.operations.length+' 项';el('change-count').textContent=state.changed+' 个栅格已改变';
  canvas.classList.toggle('drawing',state.mode!=='view');
  const list=el('operations');list.replaceChildren();
  if(!state.operations.length){const li=document.createElement('li');li.textContent='尚无标注';li.className='muted';list.append(li);}
  for(const op of state.operations){const li=document.createElement('li');li.textContent=op.type==='unknown'?`清理伪区域 · ${op.points.length} 个顶点`:`玻璃障碍墙 · ${op.points.length} 个点 / ${Math.round(op.width_m*100)} cm`;list.append(li);}
 }
 function draw(){
  const dpr=window.devicePixelRatio||1;ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,cw,ch);
  if(!state.image)return;
  const v=state.view;ctx.imageSmoothingEnabled=false;ctx.drawImage(state.image,v.x,v.y,state.source.width*v.scale,state.source.height*v.scale);
  if(state.draft.length){
   ctx.strokeStyle=state.mode==='unknown'?'#f3a36d':'#50c8c6';ctx.fillStyle=state.mode==='unknown'?'#f3a36d33':'#50c8c633';ctx.lineWidth=2;ctx.setLineDash([6,4]);ctx.beginPath();
   state.draft.forEach(([x,y],i)=>{const sx=v.x+(x+.5)*v.scale,sy=v.y+(y+.5)*v.scale;i?ctx.lineTo(sx,sy):ctx.moveTo(sx,sy);});
   if(state.mode==='unknown'&&state.draft.length>2){ctx.closePath();ctx.fill();}ctx.stroke();ctx.setLineDash([]);
   for(const [x,y]of state.draft){ctx.beginPath();ctx.arc(v.x+(x+.5)*v.scale,v.y+(y+.5)*v.scale,3,0,2*Math.PI);ctx.fillStyle='#fff';ctx.fill();ctx.stroke();}
  }
 }
 function fit(){if(!state.source)return;const v=state.view;v.scale=Math.min((cw-30)/state.source.width,(ch-30)/state.source.height);v.x=(cw-state.source.width*v.scale)/2;v.y=(ch-state.source.height*v.scale)/2;draw();}
 function resize(){const r=el('canvas-wrap').getBoundingClientRect(),dpr=window.devicePixelRatio||1;cw=r.width;ch=r.height;canvas.width=Math.round(cw*dpr);canvas.height=Math.round(ch*dpr);fit();draw();}
 new ResizeObserver(resize).observe(el('canvas-wrap'));
 function zoom(factor,x=cw/2,y=ch/2){if(!state.source)return;const v=state.view,next=Math.max(.02,Math.min(64,v.scale*factor)),ratio=next/v.scale;v.x=x-(x-v.x)*ratio;v.y=y-(y-v.y)*ratio;v.scale=next;draw();}
 el('zoom-in').addEventListener('click',()=>zoom(1.4));el('zoom-out').addEventListener('click',()=>zoom(1/1.4));el('zoom-fit').addEventListener('click',fit);
 canvas.addEventListener('wheel',e=>{if(!state.source)return;e.preventDefault();const r=canvas.getBoundingClientRect();zoom(Math.exp(-e.deltaY*.0015),e.clientX-r.left,e.clientY-r.top);},{passive:false});
 function pixel(x,y){const v=state.view;return[Math.floor((x-v.x)/v.scale),Math.floor((y-v.y)/v.scale)];}
 function mode(value,clearDraft=true){if(busy())return;if(clearDraft)state.draft=[];state.mode=value;
  el('tool-title').textContent={view:'浏览地图',unknown:'圈选伪区域',wall:'沿玻璃描画障碍墙'}[value];
  el('tool-hint').textContent={view:'拖动画布平移，滚轮或下方按钮缩放。',unknown:'依次点击伪区域的边缘，至少选 3 个顶点，再点击完成标注。灰色预览表示未知区。',wall:'沿玻璃边界依次点击，至少选 2 个点，再点击完成标注。黑色墙线会写入清理后的地图。'}[value];
  buttons();draw();}
 for(const value of ['view','unknown','wall'])el('tool-'+value).addEventListener('click',()=>mode(value));
 canvas.addEventListener('pointerdown',e=>{if(busy()||!state.source||e.button!==0)return;const r=canvas.getBoundingClientRect();state.pointer={id:e.pointerId,x:e.clientX-r.left,y:e.clientY-r.top,lastX:e.clientX-r.left,lastY:e.clientY-r.top,moved:false,pan:state.mode==='view'||state.space};canvas.setPointerCapture(e.pointerId);canvas.focus();});
 canvas.addEventListener('pointermove',e=>{const r=canvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;
  if(state.source){const [u,v]=pixel(x,y);if(u>=0&&v>=0&&u<state.source.width&&v<state.source.height){const res=state.source.resolution,o=state.source.origin,lx=(u+.5)*res,ly=(state.source.height-v-.5)*res,c=Math.cos(o[2]),s=Math.sin(o[2]);el('cursor-info').textContent=`X ${(o[0]+c*lx-s*ly).toFixed(2)} m · Y ${(o[1]+s*lx+c*ly).toFixed(2)} m`;}}
  const p=state.pointer;if(!p||p.id!==e.pointerId)return;if(Math.hypot(x-p.x,y-p.y)>5)p.moved=true;
  if(p.pan){state.view.x+=x-p.lastX;state.view.y+=y-p.lastY;draw();}p.lastX=x;p.lastY=y;
 });
 canvas.addEventListener('pointerup',e=>{const p=state.pointer;if(!p||p.id!==e.pointerId)return;state.pointer=null;
  if(!p.pan&&!p.moved&&!busy()){const [x,y]=pixel(p.lastX,p.lastY);if(x>=0&&y>=0&&x<state.source.width&&y<state.source.height){if(state.draft.length>=128){status('每项标注最多 128 个点',true);return;}const last=state.draft.at(-1);if(!last||last[0]!==x||last[1]!==y)state.draft.push([x,y]);buttons();draw();}}
 });
 canvas.addEventListener('pointercancel',()=>{state.pointer=null;});
 function revised(){state.revision++;state.saveRequest=null;el('open-saved').hidden=true;}
 async function preview(){
  const id=++state.previewId,revision=state.revision,token=state.source.token;state.previewBusy=true;el('retry-preview').hidden=true;buttons();status('正在生成清理预览…');
  try{const result=await api('preview',{token,operations:state.operations}),image=await decode(result.image);if(id!==state.previewId||token!==state.source?.token)return;state.image=image;state.changed=result.changed_cells;state.previewRevision=revision;status(`预览已更新 · ${state.changed} 个栅格改变。确认边界后另存为新地图。`);draw();}
  catch(error){if(id===state.previewId){status(error.message,true);el('retry-preview').hidden=false;}}
  finally{if(id===state.previewId){state.previewBusy=false;buttons();}}
 }
 el('retry-preview').addEventListener('click',preview);
 function finish(){if(el('finish').disabled)return;const op={type:state.mode,points:state.draft.map(p=>[...p])};if(op.type==='wall')op.width_m=Number(el('wall-width').value);state.operations.push(op);state.redo=[];state.draft=[];revised();preview();}
 el('finish').addEventListener('click',finish);el('cancel-draft').addEventListener('click',()=>{state.draft=[];buttons();draw();});
 function undo(){if(el('undo').disabled)return;if(state.draft.length){state.draft.pop();buttons();draw();return;}state.redo.push(state.operations.pop());revised();preview();}
 function redo(){if(el('redo').disabled)return;state.operations.push(state.redo.pop());revised();preview();}
 el('undo').addEventListener('click',undo);el('redo').addEventListener('click',redo);
 el('reset').addEventListener('click',()=>{if(busy())return;state.operations=[];state.redo=[];state.draft=[];revised();state.changed=0;state.image=state.originalImage;state.previewRevision=state.revision;el('retry-preview').hidden=true;buttons();draw();status('已还原载入时的快照。');});
 document.addEventListener('keydown',e=>{if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))return;if(e.code==='Space'){state.space=true;e.preventDefault();}if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'){e.preventDefault();e.shiftKey?redo():undo();}else if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='y'){e.preventDefault();redo();}else if(e.key==='Enter'&&state.draft.length){e.preventDefault();finish();}else if(e.key==='Escape'){state.draft=[];buttons();draw();}});
 document.addEventListener('keyup',e=>{if(e.code==='Space')state.space=false;});window.addEventListener('blur',()=>{state.space=false;state.pointer=null;});
 window.addEventListener('beforeunload',e=>{if(dirty()||state.draft.length){e.preventDefault();e.returnValue='';}});
 function mapOptions(maps){const select=el('map-list'),before=select.value;select.replaceChildren();const empty=document.createElement('option');empty.value='';empty.textContent='选择已保存地图';select.append(empty);for(const name of maps){const option=document.createElement('option');option.value=name;option.textContent=name;select.append(option);}if(maps.includes(before))select.value=before;buttons();}
 async function load(source,name){
  if(busy())return;if((dirty()||state.draft.length)&&!window.confirm('载入地图会清除尚未另存的标注，继续吗？'))return;
  state.sourceBusy=true;buttons();status('正在载入地图快照…');
  try{const result=await api('source',{source,name}),image=await decode(result.image);state.source=result;state.image=image;state.originalImage=image;state.operations=[];state.redo=[];state.draft=[];state.changed=0;state.revision=0;state.previewRevision=0;state.savedRevision=-1;state.saveRequest=null;
   el('empty').hidden=true;el('retry-preview').hidden=true;el('open-saved').hidden=true;el('source-info').textContent=`${source==='live'?'当前地图快照':result.source} · ${result.width} × ${result.height} · ${(result.resolution*100).toFixed(1)} cm/格`;
   const suffix=new Date().toISOString().replace(/\D/g,'').slice(0,14);el('save-name').value=(source==='live'?'map':name.slice(0,35))+'_clean_'+suffix;state.mode='view';fit();status('快照已载入。先清理伪区域，再沿玻璃补画墙线。');
  }catch(error){status(error.message,true);}finally{state.sourceBusy=false;mode(state.mode,false);buttons();}
 }
 el('load-live').addEventListener('click',()=>load('live'));el('load-saved').addEventListener('click',()=>load('saved',el('map-list').value));el('map-list').addEventListener('change',buttons);
 el('save-map').addEventListener('click',async()=>{
  if(el('save-map').disabled)return;const name=el('save-name').value.trim();if(!/^[A-Za-z0-9_-]{1,64}$/.test(name)){status('请输入 1–64 个英文字母、数字、下划线或短横线',true);el('save-name').focus();return;}
  if(!state.saveRequest||state.saveRequest.name!==name||state.saveRequest.revision!==state.revision)state.saveRequest={name,revision:state.revision,id:Array.from(crypto.getRandomValues(new Uint8Array(16)),n=>n.toString(16).padStart(2,'0')).join('')};
  state.saving=true;buttons();status('正在另存清理后的地图…');
  try{const result=await api('save',{token:state.source.token,operations:state.operations,name,request_id:state.saveRequest.id});state.savedRevision=state.revision;mapOptions(result.maps);el('open-saved').href='/map-editor?map='+encodeURIComponent(result.saved);el('open-saved').textContent='打开 '+result.saved+' →';el('open-saved').hidden=false;status('已保存新地图：'+result.saved+'。可在定位或巡检中选择这个版本。');}
  catch(error){status(error.message,true);}finally{state.saving=false;buttons();}
 });
 (async()=>{buttons();resize();try{const result=await api('maps');mapOptions(result.maps);const selected=new URLSearchParams(location.search).get('map');if(selected){el('map-list').value=selected;await load('saved',selected);}else await load('live');}catch(error){status(error.message,true);}})();
})();
