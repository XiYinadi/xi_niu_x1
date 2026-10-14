'use strict';
const fs=require('fs'), vm=require('vm'), assert=require('assert');
const navigationSource=fs.readFileSync(__dirname+'/navigation.js','utf8');
const htmlSource=fs.readFileSync(__dirname+'/slam_viz.py','utf8');
const htmlIds=new Set([...htmlSource.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]));
for (const match of navigationSource.matchAll(/\bel\('([^']+)'\)/g)) assert(htmlIds.has(match[1]),'Missing HTML control: '+match[1]);
const els={}, events={}, sent=[], windowEvents={}, docEvents={};
const element=id=>els[id]||(els[id]={value:'',textContent:'',disabled:false,innerHTML:'', listeners:{},
  addEventListener(k,f){this.listeners[k]=f;},click(){this.listeners.click();}});
const canvas={width:1000,height:500,listeners:{},
 getBoundingClientRect:()=>({left:100,top:50,width:500,height:250}),
 setPointerCapture(){},releasePointerCapture(){},addEventListener(k,f){this.listeners[k]=f;}};
const socket={connected:true,emit:(ev,p)=>sent.push({ev,p}),on:(ev,f)=>events[ev]=f};
const sandbox={document:{getElementById:element,hidden:false,addEventListener:(k,f)=>docEvents[k]=f},
 window:{addEventListener:(k,f)=>windowEvents[k]=f},socket,canvas,s2w:(x,y)=>({x:x/100,y:-y/100}),
 render(){},ctx:{},robotPose:null,trajectoryPoints:[],setInterval(){},confirm:()=>true,Math};
vm.runInNewContext(navigationSource,sandbox);
const ready={running:true,localized:true,initial_pose_ready:true,navigator_ready:true,busy:false,maps:['room'],selected_map:'room',message:'ready',path:[],calibration_error:'not measured'};
events.navigation_state(ready);
assert.strictEqual(element('nav-go').disabled,true);
element('nav-target').click();
const e=(x,y)=>({clientX:x,clientY:y,pointerId:1,preventDefault(){},stopImmediatePropagation(){}});
canvas.listeners.pointerdown(e(200,100));canvas.listeners.pointerup(e(250,100));
assert.strictEqual(sent.at(-1).p.action,'preview');
assert.strictEqual(sent.at(-1).p.x,2);assert.strictEqual(sent.at(-1).p.y,-1);
assert.strictEqual(sent.at(-1).p.yaw,0);
element('nav-go').click();assert.strictEqual(sent.at(-1).p.action,'preview');
events.navigation_state({...ready,calibration_error:null});element('nav-go').click();
assert.strictEqual(sent.at(-1).p.action,'goal');
events.navigation_state({...ready,navigating:true});windowEvents.blur();
assert.strictEqual(sent.at(-1).p.action,'cancel');
events.navigation_state({...ready,selected_map:'room',planner_ready:true,navigating:false});
element('patrol-dwell').value='5';element('patrol-cycles').value='2';element('patrol-name').value='round_a';
element('patrol-add').click();canvas.listeners.pointerdown(e(200,100));canvas.listeners.pointerup(e(250,100));
element('patrol-add').click();canvas.listeners.pointerdown(e(300,100));canvas.listeners.pointerup(e(350,100));
element('patrol-save').click();
assert.strictEqual(sent.at(-1).p.action,'patrol_save');
assert.strictEqual(sent.at(-1).p.task.waypoints.length,2);
assert.strictEqual(sent.at(-1).p.task.map,'room');
assert.strictEqual(sent.at(-1).p.task.cycles,2);
assert.strictEqual(sent.at(-1).p.task.waypoints[0].dwell_s,5);
element('patrol-points').listeners.click({target:{dataset:{action:'up',index:'1'}}});
element('patrol-save').click();assert.strictEqual(sent.at(-1).p.task.waypoints[0].x,4);
events.patrol_loaded({...sent.at(-1).p.task,revision:3});
element('patrol-save').click();assert.strictEqual(sent.at(-1).p.task.revision,3);
element('patrol-preview').click();assert.strictEqual(sent.at(-1).p.action,'patrol_preview');
assert(!sent.some(x=>x.p.action==='patrol_start'));
socket.connected=false;events.disconnect();
assert.strictEqual(element('nav-go').disabled,true);
assert.strictEqual(element('nav-target').disabled,true);
assert.strictEqual(element('patrol-preview').disabled,true);
assert(!sent.some(x=>x.ev==='motor_control'));
console.log('Navigation UI: selection coordinates, preview gate, motion gate, blur cancel, disconnect passed');
