// web_drive_test.js — 不靠浏览器测 slam_viz.py 的驾驶控件。
//
//     node web_drive_test.js        (退出码 0 = 全过)
//
// **为什么需要它**:`slam_viz.py` 的驾驶逻辑全在页面内联 JS 里,而那是整个项目
// 里唯一没法用 python 单测的部分。它已经出过一次静默失效(「不点启用也能驱动
// 电机」的 `_drive_owner` 短路),那种 bug 的共同点是**没有报错、只是在错误的时候
// 出力** —— 正是这个脚本要盯的东西。
//
// 做法:从 slam_viz.py 里抽出**真正的内联 <script>**,在一个假 DOM 上跑。
// 不是复刻一份逻辑(复刻的测试只能证明复刻对了),跑的是即将上板的那段字节。
// 抽出来之后还能顺带做两件静态核对:HTML 的 id 和 JS 的 getElementById 是否
// 对得上(假 DOM 抓不到拼错的 id)、`#joy-mode button` 是否正好两个。
//
// 唯一抓不到的:真实浏览器的 CSS 布局与 pointer capture 行为。切换模式**在真机
// 上**仍要手验一次(按下摇杆不放 → 点切换 → 指令必须立刻归零)。
'use strict';
const fs = require('fs');
const path = require('path');

const SRC_PATH = path.join(__dirname, 'slam_viz.py');
const SRC = fs.readFileSync(SRC_PATH, 'utf8');
const JS = [...SRC.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/g)].map(m => m[1]).join('\n');
const HTML = SRC.slice(SRC.indexOf('</style>'), SRC.indexOf('</body>'));

// ── 静态核对:HTML 与 JS 的接线 ──────────────────────────────────
console.log('=== 0. 静态核对 (HTML ↔ JS) ===');
const idsInHtml = [...HTML.matchAll(/\bid="([^"]+)"/g)].map(m => m[1]);
const dupIds = idsInHtml.filter((x, i) => idsInHtml.indexOf(x) !== i);
const idsInJs = [...new Set([...JS.matchAll(/getElementById\('([^']+)'\)/g)].map(m => m[1]))];
const missing = idsInJs.filter(i => !idsInHtml.includes(i));
// ⚠️ 现在 HTML 里有两个 data-mode 按钮组(#joy-mode 与 #gimbal-mode),全 HTML 扫会命中 4 个
//    按钮,打爆下面「正好两个」的断言 —— 所以按容器切片,各自只扫自己的两个按钮。
const joyModeBlock    = HTML.slice(HTML.indexOf('id="joy-mode"'), HTML.indexOf('id="drive-btns"'));
const gimbalModeBlock = HTML.slice(HTML.indexOf('id="gimbal-mode"'), HTML.indexOf('id="gimbal-pair"'));
const modeBtnModes       = [...joyModeBlock.matchAll(/data-mode="([^"]+)"/g)].map(m => m[1]);
const gimbalModeBtnModes = [...gimbalModeBlock.matchAll(/data-mode="([^"]+)"/g)].map(m => m[1]);

// ── 假 DOM ────────────────────────────────────────────────────
// ⚠️ 每个 PADS 里的盘**都必须**在这张表里。漏一个不会报错 —— `mkEl` 的
//    `PAD_R[id] || 100` 会给它一个**虚构的半径**,于是那个盘相关的用例拿着错的
//    几何跑,**照样绿**。所以下面第 12 节有一条静态断言钉死这件事。
const PAD_R = { 'joy-thr': 150, 'joy-steer': 150, 'joy-2d': 180, 'joy-gimbal': 150,
                'joy-gimbal-pan': 150, 'joy-gimbal-tilt': 150 };
const store = {};              // localStorage
const emitted = [];            // 发出去的 socket 包
const handlers = {};           // socket.on 注册的回调
const timers = [];             // setInterval 的回调(不真的定时,手动触发)

function mkEl(id) {
  const el = {
    id, style: {}, dataset: {}, textContent: '', disabled: false, _lis: {},
    _attrs: {}, setAttribute: (k, v) => { el._attrs[k] = String(v); },
    classList: {
      add: c => el._cls.add(c), remove: c => el._cls.delete(c),
      contains: c => el._cls.has(c),
      toggle: (c, on) => { on === undefined ? (el._cls.has(c) ? el._cls.delete(c) : el._cls.add(c))
                                            : (on ? el._cls.add(c) : el._cls.delete(c)); },
    },
    _cls: new Set(),
    addEventListener: (t, f) => { (el._lis[t] = el._lis[t] || []).push(f); },
    fire: (t, ev) => (el._lis[t] || []).forEach(f => f(ev || {})),
    getBoundingClientRect: () => ({ left: 0, top: 0, right: PAD_R[id] || 100, bottom: PAD_R[id] || 100 }),
    setPointerCapture: () => {}, releasePointerCapture: () => {},
    appendChild: () => {}, remove: () => {}, querySelector: () => null,
    getContext: () => ctx2d, width: 800, height: 600, clientWidth: PAD_R[id] || 100,
    naturalWidth: 640, naturalHeight: 480, complete: true,
  };
  return el;
}
const ctx2d = new Proxy({}, { get: (t, k) => (k === 'canvas' ? { width: 800, height: 600 } : () => {}) });
const els = {};
for (const m of JS.matchAll(/getElementById\('([^']+)'\)/g)) els[m[1]] = els[m[1]] || mkEl(m[1]);
const modeBtns = ['dual', 'single'].map(m => { const e = mkEl('btn-' + m); e.dataset.mode = m; return e; });
const gimbalModeBtns = ['dual', 'single'].map(m => { const e = mkEl('gbtn-' + m); e.dataset.mode = m; return e; });

const docListeners = {};
const document_ = {
  getElementById: id => els[id] || (els[id] = mkEl(id)),
  querySelectorAll: sel => (sel === '#joy-mode button' ? modeBtns
                          : sel === '#gimbal-mode button' ? gimbalModeBtns : []),
  addEventListener: (t, f) => { (docListeners[t] = docListeners[t] || []).push(f); },
  fire: (t, ev) => (docListeners[t] || []).forEach(f => f(ev || {})),
  hidden: false, body: mkEl('body'), documentElement: mkEl('html'),
};
// ⚠️ 只注入 io,不注入 socket:页面自己 `const socket = io()`,注入同名参数会
//    变成重复声明(SyntaxError: Identifier 'socket' has already been declared)。
// `connected` 要是个**真的布尔**:建图按钮用 `!socket.connected` 决定 disabled,
// 而 Proxy 对未知键返回一个函数(恒真)——那会让「断线时按钮该禁用」永远测不出来。
let sockConnected = true;
const io = () => new Proxy({}, { get: (t, k) =>
  k === 'connected' ? sockConnected
  : k === 'emit' ? (ev, p) => emitted.push({ ev, p })
  : k === 'on' ? (ev, f) => { (handlers[ev] = handlers[ev] || []).push(f); }
  : () => {} });

// 建图按钮在「已在建图」时点下去会先 confirm(地图会丢)。默认放行,
// 单独一条用例把它置 false 来验「取消 = 什么都不发」。
let confirmAnswer = true;

// window 上的监听器原来被直接丢掉(`addEventListener: () => {}`),于是 blur 那条
// 路径**完全测不到**。改成和 document_ 一样存下来,顺便把「失焦必须归零」也变可测。
const winListeners = {};
const window_ = {
  addEventListener: (t, f) => { (winListeners[t] = winListeners[t] || []).push(f); },
  fire: (t, ev) => (winListeners[t] || []).forEach(f => f(ev || {})),
  devicePixelRatio: 1,
};

const sandbox = {
  document: document_, window: window_,
  io, navigator: { userAgent: 'node' }, location: { protocol: 'http:' },
  // `alive` + 真的 clearInterval:第 10 节要验「页面不可见时发送循环真的停了」。
  // 原来的 clearInterval 是个空函数、也不记状态,那种「停发」根本测不出来。
  setInterval: (f, ms) => { const id = timers.length; timers.push({ id, f, ms, alive: true }); return id; },
  setTimeout: () => 0,
  clearInterval: id => { if (timers[id]) timers[id].alive = false; },
  requestAnimationFrame: () => 0,
  cancelAnimationFrame: () => {}, performance: { now: () => 0 },
  localStorage: {
    getItem: k => (k in store ? store[k] : null),
    setItem: (k, v) => { store[k] = String(v); },
    removeItem: k => { delete store[k]; },
  },
  fetch: () => Promise.resolve({ ok: true, json: () => Promise.resolve({}), blob: () => Promise.resolve({}) }),
  Image: function () { return mkEl('img'); },
  URL: { createObjectURL: () => 'blob:x', revokeObjectURL: () => {} },
  ResizeObserver: function () { return { observe: () => {}, disconnect: () => {} }; },
  alert: () => {}, confirm: () => confirmAnswer,
  console, Math, JSON, Date, Object, Array, String, Number, Boolean,
  isNaN, parseInt, parseFloat,
};
const names = Object.keys(sandbox);
new Function(...names, JS)(...names.map(k => sandbox[k]));

// ── 断言工具 ──────────────────────────────────────────────────
const drive = els['drive'], val = els['drive-val'];
const P = { thr: els['joy-thr'], steer: els['joy-steer'], stick: els['joy-2d'] };
let pass = 0, fail = 0;
function ok(name, cond, extra) {
  if (cond) { pass++; console.log('  ok   ' + name); }
  else { fail++; console.log('  FAIL ' + name + (extra !== undefined ? '   → ' + extra : '')); }
}
// 手指落到盘上:fx/fy 是相对盘心的归一化位置(1 = 一个半径)
function touch(p, fx, fy) {
  const R = p.clientWidth / 2;
  p.fire('pointerdown', { clientX: R + fx * R, clientY: R + fy * R, pointerId: 1 });
}
function release(p) { p.fire('pointerup', { pointerId: 1 }); }
// 还活着的周期定时器(被 clearInterval 停掉的不算)。第 10 节靠它验「发送循环停没停」。
function liveTimer(ms) { return timers.find(t => t.ms === ms && t.alive); }
// 从 drive-val 里读回 l/a —— 那正是 driveTarget() 输出的、也是唯一会发给 socket 的两个数
function read() {
  const t = liveTimer(16);
  if (!t) throw new Error('16ms 发送定时器不在运行 —— read() 取不到摇杆值');
  t.f();                                           // 手动触发那个 60Hz 周期
  const m = val.textContent.match(/前进 ([+-][\d.]+)\s+转向 ([+-][\d.]+)/);
  // ⚠️ 匹配不上时**必须自己报清楚**,不能让它烂到下面 `m[1]` 变成
  //    「TypeError: Cannot read properties of null」—— 那个报错完全指不到病根。
  //    这条正是「两个 16ms 定时器」那个盲区的症状:liveTimer(16) 用 find 拿到了
  //    **云台**那个,重放它不刷新 drive-val ⇒ 正则匹配空文本/上一帧的陈旧文本。
  if (!m) throw new Error(
    'drive-val 读不出 l/a(驱动链路可能已断):' + JSON.stringify(val.textContent)
    + '  —— 若同时存在两个 16ms 定时器,可能就是它(见 tickAll 处的注释)');
  return { l: parseFloat(m[1]), a: parseFloat(m[2]) };
}
const near = (x, y) => Math.abs(x - y) < 0.01;
const D = (o) => JSON.stringify(o);

console.log('  ' + (dupIds.length ? 'FAIL' : 'ok  ') + ' HTML 无重复 id (' + idsInHtml.length + ' 个)'
            + (dupIds.length ? ' → ' + dupIds : ''));
if (dupIds.length) fail++; else pass++;
if (!missing.length) { pass++; console.log('  ok   JS 查的 ' + idsInJs.length + ' 个 id 在 HTML 里都有'); }
else { fail++; console.log('  FAIL JS 查了 HTML 里不存在的 id → ' + missing); }
const btnOk = modeBtnModes.length === 2 && modeBtnModes.includes('dual') && modeBtnModes.includes('single');
ok('#joy-mode 下正好两个按钮 dual/single', btnOk, D(modeBtnModes));
const gbtnOk = gimbalModeBtnModes.length === 2 && gimbalModeBtnModes.includes('dual') && gimbalModeBtnModes.includes('single');
ok('#gimbal-mode 下正好两个按钮 dual/single', gbtnOk, D(gimbalModeBtnModes));
ok('三个盘的准线类齐全 (joy-2d 要 axis-y+axis-x 两条都亮)',
   /class="joy-pad axis-y" id="joy-thr"/.test(HTML)
   && /class="joy-pad axis-x" id="joy-steer"/.test(HTML)
   && /class="joy-pad axis-y axis-x" id="joy-2d"/.test(HTML));

console.log('=== 0b. 车速档位与检测画面 ===');
ok('默认低速，未恢复高速', els['gear-low']._attrs['aria-pressed'] === 'true');
touch(P.thr,0,-1);ok('低速满前进输出 0.5',near(read().l,.5));release(P.thr);
touch(P.thr,0,1);ok('低速满倒车输出 -0.5',near(read().l,-.5));release(P.thr);
touch(P.steer,1,0);ok('低速保留全部转向',near(read().a,-1));release(P.steer);
document_.fire('keydown',{key:'w',preventDefault(){}});ok('键盘同样限低速',near(read().l,.5));
els['gear-high'].fire('click');ok('切高速立即清掉按键输入',read().l===0);
document_.fire('keydown',{key:'w',repeat:true,preventDefault(){}});ok('长按重复事件不会恢复出力',read().l===0);
document_.fire('keyup',{key:'w'});
touch(P.thr,0,-1);ok('高速保留原满量程',near(read().l,1));
els['gear-low'].fire('click');ok('切低速清掉正在按的摇杆',read().l===0);
P.thr.fire('pointermove',{clientX:75,clientY:0,pointerId:1});ok('切档后旧指针不能恢复出力',read().l===0);
release(P.thr);els['gear-high'].fire('click');
ok('页面只有检测窗口', !HTML.includes('id="win-cam"') && HTML.includes('id="win-fire"'));
emitted.length=0;handlers.connect.forEach(f=>f());
ok('默认开启检测流，不请求重复原始画面',emitted.some(e=>e.ev==='fire_control'&&e.p.action==='start')&&!emitted.some(e=>e.ev==='camera_control'));

console.log('=== 1. 默认模式 ===');
ok('默认双杆: #drive 没有 mode-single', !drive.classList.contains('mode-single'));
ok('默认双杆: 「双杆」点亮、「单杆」不亮',
   modeBtns[0].classList.contains('on') && !modeBtns[1].classList.contains('on'));

console.log('=== 2. 双杆:两轴独立 ===');
touch(P.thr, 0, -1);
ok('油门推顶 → l=+1, a=0', near(read().l, 1) && near(read().a, 0), D(read()));
release(P.thr);
touch(P.thr, 0.9, -1);
ok('油门盘手指偏右 → 仍是 l=+1(死轴不折算)', near(read().l, 1), D(read()));
release(P.thr);
touch(P.steer, 1, 0);
ok('转向推右 → a=-1, l=0', near(read().a, -1) && near(read().l, 0), D(read()));
touch(P.thr, 0, -1);
ok('双指同时: 油门顶+转向右 → l=+1, a=-1', near(read().l, 1) && near(read().a, -1), D(read()));
release(P.thr); release(P.steer);

console.log('=== 3. 单杆:圆门 ===');
modeBtns[1].fire('click');
ok('切到单杆: #drive 有 mode-single', drive.classList.contains('mode-single'));
ok('切到单杆: 「单杆」点亮', modeBtns[1].classList.contains('on'));
touch(P.stick, 0, -1);
ok('推顶 → l=+1, a=0', near(read().l, 1) && near(read().a, 0), D(read()));
release(P.stick);
touch(P.stick, 1, 0);
ok('推右 → a=-1, l=0', near(read().a, -1) && near(read().l, 0), D(read()));
release(P.stick);
touch(P.stick, 1, -1);
const c = read();
ok('斜推到角 → 0.707/0.707(模长钳到 1,方向保住)',
   near(c.l, Math.SQRT1_2) && near(c.a, -Math.SQRT1_2), D(c));
ok('斜推的模长 = 1', near(Math.hypot(c.l, c.a), 1), Math.hypot(c.l, c.a));
touch(P.stick, 5, -5);
const f = read();
ok('拖出盘外 → 仍钳在圆门上,不超 1', near(Math.hypot(f.l, f.a), 1), Math.hypot(f.l, f.a));
release(P.stick);
ok('松手回中 → l=0, a=0', near(read().l, 0) && near(read().a, 0), D(read()));

console.log('=== 4. 切换的安全性 (核心) ===');
handlers['you'][0]({ sid: 'A' });
handlers['motor_state'][0]({ armed: true, driver: 'A' });
touch(P.stick, 0, -1);
ok('切换前指令非零', !near(read().l, 0));
emitted.length = 0;
modeBtns[0].fire('click');                    // ★ 按住摇杆不放就切模式
ok('切模式后指令立刻归零', near(read().l, 0) && near(read().a, 0), D(read()));
ok('切模式时补发了零速 vel',
   emitted.some(e => e.ev === 'motor_control' && e.p.action === 'vel'
                  && e.p.linear === 0 && e.p.angular === 0));
modeBtns[1].fire('click');
ok('切回来后没有残留出力(⇒ active 真的被清了,否则 vy=-1 会再次出力)',
   near(read().l, 0) && near(read().a, 0), D(read()));
release(P.stick);
touch(P.thr, 0, -1);
modeBtns[1].fire('click');
ok('反向切换也立刻归零', near(read().l, 0) && near(read().a, 0), D(read()));

console.log('=== 5. 残留 active 不能偷出力 ===');
// 人为把隐藏的双杆盘设成 active 且压满 —— 模拟「隐藏时 pointerup 没送到」
P.thr.fire('pointerdown', { clientX: 75, clientY: 0, pointerId: 9 });
ok('单杆模式下,被隐藏的油门盘 active 也不出力',
   near(read().l, 0) && near(read().a, 0), D(read()));
modeBtns[0].fire('click');
ok('切回双杆这一步顺带把它也清了(不是靠模式互斥漏过去的)', near(read().l, 0), D(read()));
touch(P.thr, 0, -1);
ok('双杆模式下同一位置确实出力 ⇒ 上面不是假通过', near(read().l, 1), D(read()));
release(P.thr);
modeBtns[1].fire('click');

console.log('=== 6. 键盘在两种模式下都有效 ===');
document_.fire('keydown', { key: 'w', preventDefault: () => {} });
ok('单杆模式下 w → l=+1', near(read().l, 1), D(read()));
touch(P.stick, 1, 0);
ok('键盘与单杆叠加: w + 杆右 → l=+1, a=-1', near(read().l, 1) && near(read().a, -1), D(read()));
release(P.stick);
document_.fire('keyup', { key: 'w' });
modeBtns[0].fire('click');
document_.fire('keydown', { key: 'd', preventDefault: () => {} });
ok('双杆模式下 d → a=-1', near(read().a, -1), D(read()));
document_.fire('keyup', { key: 'd' });

console.log('=== 7. 模式记忆 ===');
ok('localStorage 记住了 dual', store['joyMode'] === 'dual', store['joyMode']);
modeBtns[1].fire('click');
ok('localStorage 记住了 single', store['joyMode'] === 'single', store['joyMode']);

console.log('=== 8. driveGate 会刷新新露出来的盘 ===');
ok('单杆盘拿到了 opacity(不是空字符串)',
   els['joy-2d'].style.opacity === '1' || els['joy-2d'].style.opacity === '.4',
   D(els['joy-2d'].style.opacity));

console.log('=== 9. 建图按钮 ===');
// 状态是**服务端**按 /scan 与 /map 的实际数据流算的,所以这里喂的是
// mapping_state 事件,而不是「我按过了」。这是整个按钮最要紧的性质:
// 它不能因为「按钮被按过」就自称在建图。
const btnMap = els['btn-map'];
const fireMap = st => (handlers['mapping_state'] || []).forEach(f => f(st));
const lastMapEv = () => emitted.filter(e => e.ev === 'mapping_control').pop();

// 初始 disabled 必须查**真 HTML**,不能查假 DOM —— 假元素的 disabled 默认 false,
// 它根本不知道标签上写了什么。这条要保证的是「连上之前没有可点的窗口」。
ok('HTML 里就是 disabled(连上之前没有可点的窗口)',
   /<button id="btn-map" disabled>/.test(HTML));

fireMap({ status: 'off', scan_age: null, map_age: null, busy: false, action: null });
ok('off → 文案「启动建图」', btnMap.textContent === '启动建图', btnMap.textContent);
ok('off → 不带 on / partial', !btnMap.classList.contains('on')
                            && !btnMap.classList.contains('partial'));

fireMap({ status: 'lidar_only', scan_age: 0.1, map_age: 99, busy: false, action: null });
ok('lidar_only → 文案「SLAM 未起 ⟳」', btnMap.textContent === 'SLAM 未起 ⟳', btnMap.textContent);
ok('lidar_only → 带 partial 不带 on', btnMap.classList.contains('partial')
                                   && !btnMap.classList.contains('on'));

fireMap({ status: 'mapping', scan_age: 0.1, map_age: 1.0, busy: false, action: null });
ok('mapping → 文案「建图中」', btnMap.textContent === '建图中', btnMap.textContent);
ok('mapping → 带 on 不带 partial', btnMap.classList.contains('on')
                                && !btnMap.classList.contains('partial'));
ok('mapping → title 里说明了地图会丢(点下去不可逆)',
   /地图会丢/.test(btnMap.title || ''), btnMap.title);

// off 态点击 = 启动
fireMap({ status: 'off', scan_age: null, map_age: null, busy: false, action: null });
emitted.length = 0;
btnMap.fire('click');
ok('off 态点击 → 发 start', lastMapEv() && lastMapEv().p.action === 'start',
   D(lastMapEv()));

// 已在建图:confirm 取消 ⇒ 一个包都不发
fireMap({ status: 'mapping', scan_age: 0.1, map_age: 1.0, busy: false, action: null });
emitted.length = 0;
confirmAnswer = false;
btnMap.fire('click');
ok('mapping 态点击 + 取消确认 → 什么都不发', emitted.length === 0, D(emitted));
confirmAnswer = true;
btnMap.fire('click');
ok('mapping 态点击 + 确认 → 发 stop', lastMapEv() && lastMapEv().p.action === 'stop',
   D(lastMapEv()));

// busy:按钮禁用且不再发包 —— 双击不能起两份 slam_toolbox
fireMap({ status: 'off', scan_age: null, map_age: null, busy: true, action: 'start' });
ok('busy(start) → disabled + 文案「启动中…」',
   btnMap.disabled === true && btnMap.textContent === '启动中…', btnMap.textContent);
emitted.length = 0;
btnMap.fire('click');
ok('busy 期间点击不发包(防双击起两份)', emitted.length === 0, D(emitted));
fireMap({ status: 'off', scan_age: null, map_age: null, busy: true, action: 'stop' });
ok('busy(stop) → 文案「停止中…」', btnMap.textContent === '停止中…', btnMap.textContent);

fireMap({status:'mapping',busy:false,managed:true});
ok('自动管线有新鲜地图时如实显示建图中，普通开关禁用',btnMap.textContent==='建图中' && btnMap.disabled);
emitted.length=0;btnMap.fire('click');
ok('自动管线管理期间普通开关不发启停包',emitted.length===0);

// ⚠️ 最要紧的一条:按钮**不能**因为「我按过 start」就自称在建图 ——
// 只有服务端按数据流算出 mapping 才行。
fireMap({ status: 'off', scan_age: null, map_age: null, busy: false, action: null });
emitted.length = 0;
btnMap.fire('click');                                  // 发 start,但服务器什么都没变
ok('点过 start 之后,服务端仍报 off ⇒ 按钮必须显示「启动建图」',
   btnMap.textContent === '启动建图' && !btnMap.classList.contains('on'), btnMap.textContent);
ok('...而且没有自作主张地进入 busy', btnMap.disabled === false);

// 断线:回到 off 且禁用。
// mapUi 活在沙箱里,外面够不到 —— 只能触发它注册在假 socket 上的 disconnect 回调
// (页面在 disconnect 上挂了三个监听:相机/火情停流、driveUi、mapUi)。
sockConnected = false;
(handlers['disconnect'] || []).forEach(f => f());
ok('断线 → 文案回到「启动建图」且 disabled',
   btnMap.textContent === '启动建图' && btnMap.disabled === true,
   btnMap.textContent + ' / disabled=' + btnMap.disabled);
sockConnected = true;

// ── 10. 页面不可见 ⇒ 必须真的停发 ────────────────────────────────
// 成因:浏览器把不可见标签页的 setInterval 钳到约 1Hz,而 1Hz(包间隔 ~400ms)
// 落在 pwm_proc 的 deadman(300ms)**以下** ⇒ 每一包都被判成一次「恢复」、
// 300ms 后又一次「跳闸」。电机与舵机走同一个门,于是舵机跟着 park(完全不发
// 脉冲、松掉)↔ 全速转回目标角 —— 用户报的「启用电机后舵机老是来回抖」。
// 节流是浏览器行为,绕不过去,所以不可见就必须**真的停掉发送循环**。
// 这条只有假 DOM 能验:真机上得盯着舵机看半天,现象还取决于浏览器版本。
console.log('=== 10. 页面不可见时停发 (后台标签页节流) ===');

handlers['you'][0]({ sid: 'A' });
handlers['motor_state'][0]({ armed: true, driver: 'A' });
// 前置条件:可见状态下发送循环**确实在跑**。没有这一条,后面「停了」可能是假通过。
ok('可见时 16ms 发送定时器在跑', !!liveTimer(16));
touch(P.stick, 0, -1);
ok('可见时摇杆确实在发(l=+1)', near(read().l, 1), D(read()));
release(P.stick);

document_.hidden = true;
emitted.length = 0;
document_.fire('visibilitychange');
ok('不可见 → 16ms 发送定时器**已停**', !liveTimer(16));
ok('不可见 → 补发了一次零速(死角残留也要清干净)',
   emitted.some(e => e.ev === 'motor_control' && e.p.action === 'vel'
                  && e.p.linear === 0 && e.p.angular === 0), D(emitted));
ok('不可见 → 读数照实说「已暂停」,不留一个假的最后值',
   /已暂停/.test(val.textContent), val.textContent);

// 停掉期间不许再有任何 vel 包出去 —— 这才是「不再喂出 1Hz 那种病态包率」。
// 把所有**还活着**的周期都推一拍:既验发送循环没了,也顺手验没有别的定时器偷发。
emitted.length = 0;
timers.filter(t => t.alive).forEach(t => t.f());
// ⚠️ 断言里**必须同时**包含 gimbal_control:只查 motor_control 的话,一个「云台
//    自己开了个定时器偷偷发」的实现会整条溜过去 —— 而云台 1Hz 的包率同样会把
//    gimbal_proc 的 deadman 拖到阈值以下,病的形状和电机那条一模一样。
ok('停掉期间:推一拍也没有任何 vel 包发出(电机与云台都查)',
   !emitted.some(e => e.ev === 'motor_control' || e.ev === 'gimbal_control'),
   D(emitted.map(e => e.ev)));

// 回到前台:发送循环必须自己回来,读数也要恢复(不能再挂着「已暂停」)。
document_.hidden = false;
document_.fire('visibilitychange');
ok('回到前台 → 16ms 发送定时器自己回来了', !!liveTimer(16));
emitted.length = 0;
touch(P.stick, 0, -1);
const back = read();
ok('回到前台 → 摇杆重新出力(l=+1)且读数不再显示「已暂停」',
   near(back.l, 1) && !/已暂停/.test(val.textContent),
   D(back) + ' / ' + val.textContent);
ok('回到前台 → vel 包确实发出去了',
   emitted.some(e => e.ev === 'motor_control' && e.p.linear > 0), D(emitted));
release(P.stick);
ok('松杆后回到零(停发/复发的来回没有把状态搞乱)',
   near(read().l, 0) && near(read().a, 0), D(read()));

// blur(窗口失焦但**仍可见**)不在此列:那种情况不被节流,循环不该停。
emitted.length = 0;
window_.fire('blur');
ok('blur 只归零、**不**停发送循环(失焦≠不可见,不被节流)',
   !!liveTimer(16), 'blur 之后发送定时器不该消失');

// ── 11. 拖拽 (pointermove) —— A1/A2 两个缺陷的回归 ───────────────
// ⚠️ 这**整条路**之前一点覆盖都没有:老的 touch() 只发 pointerdown,全文件
//    `pointermove` 出现 0 次。而用户报的「单杆时前后正常、转向有时无反应」
//    就出在这条路上 —— 拖拽是摇杆唯一真实的用法,按下不动没人这么开。
console.log('=== 11. 拖拽 (pointermove) ===');
modeBtns[1].fire('click');        // 本节自己切,不依赖上面停在哪一模式

function drag(p, fx, fy) {        // 手指**已在盘上**时继续移动
  const R = p.clientWidth / 2;
  p.fire('pointermove', { clientX: R + fx * R, clientY: R + fy * R, pointerId: 1 });
}

touch(P.stick, 0, 0);
ok('按下盘心 = 零指令', near(read().l, 0) && near(read().a, 0), D(read()));
drag(P.stick, 1, 0);
ok('★ 按住后拖到正右 → a=-1(pointermove 真的生效)', near(read().a, -1), D(read()));
ok('旋钮跟着手指走 (knob-2d 的 transform 在变)',
   /translate/.test(els['knob-2d'].style.transform || ''), els['knob-2d'].style.transform);
drag(P.stick, 0, -1);
ok('接着拖到正上 → l=+1, a 回 0', near(read().l, 1) && near(read().a, 0), D(read()));

// ★ A1 回归:手指滑出盘外,指令必须**继续跟手**。
//   旧实现先按轴各钳到 ±1、再按模长归一化 ⇒ 任一轴超出 1 就双双钳死,整条指令
//   冻在 0.707/-0.707 再也不变。而 setPointerCapture 让盘外的 move 照样送进来,
//   所以「推满前进 → 再往右滑想转向」这种最正常的手势一越界就卡死。
drag(P.stick, 1.5, -1.0);
const g = read();
ok('★ 盘外 (1.5,-1.0) → a≈-0.832 (不再是冻死的 -0.707)',
   near(g.a, -0.8321, 0.005), D(g));
ok('★ 同一处 l≈+0.555 —— 两轴都还在跟手指,方向没被钳歪',
   near(g.l, 0.5547, 0.005), D(g));
ok('盘外仍被钳在圆门上(模长 = 1)',
   near(Math.hypot(g.l, g.a), 1), Math.hypot(g.l, g.a));

// 冻死的判据就是**连着几个位置读数完全相同** —— 直接数它。
// ⚠️ 这五个位置必须**非等比**:(1.2,-1.2)/(1.5,-1.5)/(2,-2) 归一化之后是同一个
//    0.707/-0.707,拿它们当"不同位置"是自欺 —— 卡住手指的恰恰是**非等比**的偏移
//    (纵向已满、手指继续横向滑),那才是「推满前进再往右滑」的真实轨迹。
const seq = [[1.2, -1.2], [1.5, -1.0], [2, -1.4], [1.5, -0.5], [0.8, -1.6]]
  .map(([x, y]) => { drag(P.stick, x, y); return read(); });
const frozen = seq.filter((v, i) => i && near(v.l, seq[i - 1].l, 1e-9)
                                      && near(v.a, seq[i - 1].a, 1e-9));
ok('★ 盘外连续拖 5 个位置,读数两两不同(旧实现在这里全部相等)',
   frozen.length === 0, D(seq.map(v => [+v.l.toFixed(3), +v.a.toFixed(3)])));

// ★ A2 回归:拖动过程中窗口失焦。
//   旧实现 `blur → releaseAll()` 把 p.active 清成 false,而 pointermove 只在
//   `if (p.active)` 时才处理 ⇒ **手指没抬起来,后续拖动全被丢弃**,必须松手重按。
touch(P.stick, 1, 0);
ok('按住推到正右 → a=-1', near(read().a, -1), D(read()));
window_.fire('blur');
ok('blur(手指仍未松开) → 立刻归零,安全侧不降级',
   near(read().l, 0) && near(read().a, 0), D(read()));
ok('blur 之后只是读数 → 恒 0(残留值没有偷偷把它带起来)',
   near(read().l, 0) && near(read().a, 0), D(read()));
drag(P.stick, -1, 0);
ok('★ blur 后继续拖(手指没离开) → 恢复跟随 a=+1',
   near(read().a, 1), D(read()) + '   ← 旧实现在这里是 0:拖拽已死,必须松手重按');
release(P.stick);
drag(P.stick, 1, 0);
ok('★ 但**松手之后** pointermove 必须被忽略(恢复只认还按着的那根手指)',
   near(read().l, 0) && near(read().a, 0), D(read()));

// 切模式:旧盘的 pid 不许残留 —— 隐藏元素上 pointerup 不保证送达,残留 pid
// 会在下次 move 时把看不见的盘重新激活。
touch(P.stick, 0, -1);
modeBtns[0].fire('click');                   // 按住不放就切到双杆
drag(P.stick, 0, -1);
ok('切模式后拖已隐藏的单杆盘 → 不产出',
   near(read().l, 0) && near(read().a, 0), D(read()));
touch(P.stick, 1, 0);                        // 直接让隐藏盘 active 且压满
ok('双杆模式下,隐藏的单杆盘 active 也不出力 (driveTarget 只读 MODE_PADS 的双保险)',
   near(read().l, 0) && near(read().a, 0), D(read()));
release(P.stick);

// 统一钳模长之后,单轴盘必须**逐点等价**于原来的按轴钳位:死轴在归一化之前
// 就被摁成 0,于是 m == |该轴|,除以 m 恰好得 ±1,手指的横向偏移折不进来。
touch(P.thr, 7, -1.5);
ok('单轴油门盘推盘外(纵向 1.5 + 死轴 7)→ 仍 l=+1',
   near(read().l, 1), D(read()));
touch(P.thr, -7, 1.5);
ok('单轴油门盘反向推盘外 → 仍 l=-1', near(read().l, -1), D(read()));
release(P.thr);

// ── 12. 云台 (二维云台,走串口) ──────────────────────────────────
// 这一节盯的是**三个静默盲区** —— 现有 77 项一个都抓不到,而它们全都属于
// 「不报错、只是在错误的时候出力(或不出力)」:
//   ① `PAD_R` 漏条目 → 假 DOM 给个虚构半径,相关用例拿错几何跑绿;
//   ② `driveGate()` 把云台盘一起锁死 → **「云台不受 armed 管」这条决定静默失效**
//      (别人启用了电机,你就再也动不了云台,而 UI 还让你去给电机上电);
//   ③ 两个 16ms 定时器 → `liveTimer(16)` 用 find 拿到云台那个,`read()` 返回
//      陈旧值,**测试全绿而驾驶链路已断**。
console.log('=== 12. 云台 (不受 armed / 驾驶权管) ===');

// 静态:每个 PADS 里的 el 都必须在 PAD_R 里有条目(见 PAD_R 那里的注释)。
const padsBlock = JS.slice(JS.indexOf('const PADS = {'), JS.indexOf('const padList'));
const padElIds = [...padsBlock.matchAll(/document\.getElementById\('([^']+)'\)/g)]
  .map(m => m[1]).filter(i => i.startsWith('joy-'));
const noR = padElIds.filter(i => !(i in PAD_R));
ok('★ 每个 PADS 的盘都在 PAD_R 里有条目 (' + padElIds.length + ' 个)',
   noR.length === 0, '缺: ' + D(noR));

// 静态:云台盘**不能**出现在 MODE_PADS 里 —— 进了 driveTarget() 就会跟着驾驶走。
// 只钉「这一行没被动过」;真动过就该人工确认,而不是让正则去猜它有没有包云台。
ok('★ MODE_PADS 仍是「双杆两盘 / 单杆一盘」,云台没被包进去',
   /MODE_PADS = \{ dual: \[PADS\.thr, PADS\.steer\], single: \[PADS\.stick\] \}/.test(JS),
   'MODE_PADS 的定义变了,请人工确认云台盘没被加进去');

const G = els['joy-gimbal'], gval = els['gimbal-val'];
function readGimbal() {
  const t = liveTimer(16);
  if (!t) throw new Error('16ms 发送定时器不在运行 —— readGimbal() 取不到云台值');
  t.f();
  const m = gval.textContent.match(/dx ([+-]\d+)\s+dy ([+-]\d+)/);
  if (!m) throw new Error('gimbal-val 读不出 dx/dy: ' + D(gval.textContent));
  return { dx: parseInt(m[1], 10), dy: parseInt(m[2], 10) };
}
const gEv = () => emitted.filter(e => e.ev === 'gimbal_control').pop();

// ① 驾驶权在**别人**手里时,云台盘必须仍然可点。
//    ⚠️ 这条正是「不跳过的三个后果」里的头两个:盘被锁死 + 文案让你去上电。
handlers['you'][0]({ sid: 'A' });
handlers['motor_state'][0]({ armed: true, driver: 'B' });
ok('前置:驾驶权在别人手里 ⇒ 驾驶盘确实被锁死(否则下面全是假通过)',
   P.thr.style.pointerEvents === 'none', D(P.thr.style.pointerEvents));
ok('★ 同一状态下云台盘**仍可点**(没被 driveGate 连坐)',
   G.style.pointerEvents === 'auto' && G.style.opacity === '1',
   D(G.style.pointerEvents) + '/' + D(G.style.opacity));
// ⚠️ 判据是**别出现夺权文案**(接管/抢回),不是「有没有出现『启用电机』四个字」
//    —— 云台的提示本来就该说「不必启用电机」,那四个字正是**反话**。
//    第一版把判据写成 /启用电机/ 就误报了,记在这里免得改回去。
const gHint = () => els['gimbal-hint'].textContent || '';
ok('★ 提示文案里没有夺权那一套(「接管」/「抢回」是驾驶盘的说辞)',
   !/接管|抢回/.test(gHint()), D(gHint()));
ok('★ 而且明确说了不必启用电机(否则用户会去点那个按钮)',
   /不必/.test(gHint()), D(gHint()));

// 云台操作权在**别人**手里时,提示要说清楚 —— 否则「我推了没反应」会被当成云台坏了。
handlers['gimbal_state'][0]({ owner: 'B' });
ok('★ 操作者是别人 → 提示说是「另一台设备」,而不是静默不响应',
   /另一台设备/.test(gHint()), D(gHint()));
ok('操作权在别人手里**不影响盘可点**(一推杆就抢过来)', G.style.pointerEvents === 'auto');
handlers['gimbal_state'][0]({ owner: 'A' });
ok('操作者回到自己 → 提示恢复常态', /不必/.test(gHint()), D(gHint()));

// ② armed=false 时云台照样发帧 —— 云台不受 arm 门管。
handlers['motor_state'][0]({ armed: false, driver: null });
ok('armed=false 时驾驶盘解锁(isDriver 的 !armed 短路)',
   P.thr.style.pointerEvents === 'auto');
emitted.length = 0;
touch(G, 1, 0);                        // 推右
const gR = readGimbal();
ok('★ armed=false 照样发云台帧(推右 → dx>0,dy=0)',
   gR.dx === 100 && gR.dy === 0, D(gR));
ok('★ 而且真的发了包(不是只刷新读数)',
   gEv() && gEv().p.action === 'vel' && gEv().p.dx === 100, D(gEv()));
release(G);

emitted.length = 0;
touch(G, 0, -1);                       // 推上
const gU = readGimbal();
ok('推上 → dy=+100(PROTOCOL 单位就是线上单位,不做 -1..1 换算)',
   gU.dx === 0 && gU.dy === 100, D(gU));
release(G);
ok('松手 → 云台回中 (0,0)', (() => { const v = readGimbal();
   return v.dx === 0 && v.dy === 0; })());

// 圆门对云台**照旧生效**(v1 决定不为它开分支):斜推到角是 71/71,不是 100/100。
touch(G, 1, -1);
const gC = readGimbal();
ok('斜推到角 → 71/71(圆门照旧;v1 接受这个耦合,见 CSS #gimbal 注释)',
   gC.dx === 71 && gC.dy === 71, D(gC));
ok('圆门把模长钳在 1(云台指令不超 ±100)',
   Math.hypot(gC.dx, gC.dy) <= 100.5, Math.hypot(gC.dx, gC.dy));
release(G);

// ③ 页面不可见 / blur:必须补发云台 (0,0),且不可见时发送循环真的停。
touch(G, 1, 0);
document_.hidden = true;
emitted.length = 0;
document_.fire('visibilitychange');
ok('★ 不可见 → 补发云台 (0,0)(否则云台咬着最后那个角度)',
   emitted.some(e => e.ev === 'gimbal_control' && e.p.dx === 0 && e.p.dy === 0),
   D(emitted.filter(e => e.ev === 'gimbal_control')));
ok('不可见 → 云台读数照实说「已暂停」,不留一个假的最后值',
   /已暂停/.test(gval.textContent), D(gval.textContent));
emitted.length = 0;
timers.filter(t => t.alive).forEach(t => t.f());
ok('不可见期间不再发云台包',
   !emitted.some(e => e.ev === 'gimbal_control'), D(emitted.map(e => e.ev)));
document_.hidden = false;
document_.fire('visibilitychange');

// ★ 定时器恰好一个。两个的话 liveTimer(16) 用 find 会拿到其中一个,
//   read() 重放那个而不刷新 valEl ⇒ 正则匹配到**上一帧** ⇒ 返回旧值而不抛错。
ok('★ 16ms 发送定时器恰好 1 个(两个会让 read() 静默返回陈旧值)',
   timers.filter(t => t.ms === 16 && t.alive).length === 1,
   timers.filter(t => t.ms === 16).map(t => t.alive));
// 共用定时器之后,驾驶那条路必须**一点没坏** —— 这是上面那条的正面证据。
handlers['motor_state'][0]({ armed: true, driver: 'A' });
touch(P.thr, 0, -1);
ok('★ 共用定时器后驾驶读数仍然正确(read() 没被云台挤掉)',
   near(read().l, 1), D(read()));
emitted.length = 0;
read();                                 // 只刷新读数,不动任何盘
ok('read() 一拍里电机与云台**各发一包**(tickAll 两个都调了)',
   emitted.filter(e => e.ev === 'motor_control').length === 1
   && emitted.filter(e => e.ev === 'gimbal_control').length === 1,
   D(emitted.map(e => e.ev)));
release(P.thr);

// ④ 按住云台盘切模式 → 读数立刻归零(与驾驶盘同一条路:releaseAll → padsReset)。
modeBtns[0].fire('click');
touch(G, 1, -1);
ok('切模式前云台读数非零', readGimbal().dx !== 0, D(readGimbal()));
modeBtns[1].fire('click');
ok('★ 按住云台盘切模式 → 读数立刻归零(盘被隐藏前已清干净)',
   (() => { const v = readGimbal(); return v.dx === 0 && v.dy === 0; })(), D(readGimbal()));
release(G);

// ⑤ blur(窗口失焦但**仍可见**)也要补发云台零位,且**不停**发送循环。
emitted.length = 0;
touch(G, -1, 0);
window_.fire('blur');
ok('★ blur → 补发云台 (0,0)',
   emitted.some(e => e.ev === 'gimbal_control' && e.p.dx === 0 && e.p.dy === 0),
   D(emitted.filter(e => e.ev === 'gimbal_control')));
ok('blur 不停发送循环(失焦≠不可见,不被节流)', !!liveTimer(16));
release(G);

// ⑥ 断线:云台盘必须禁用 —— 这是**唯一**该锁住它的理由。
sockConnected = false;
(handlers['disconnect'] || []).forEach(f => f());
ok('断线 → 云台盘禁用(opacity .4 + pointerEvents none)',
   G.style.pointerEvents === 'none' && G.style.opacity === '.4',
   D(G.style.pointerEvents) + '/' + D(G.style.opacity));
sockConnected = true;

console.log('=== 13. 云台单/双杆模式 ===');
const GPan = els['joy-gimbal-pan'], GTilt = els['joy-gimbal-tilt'];
// 先把云台盘刷回可点态(上一节断线把三个云台盘设成 none 了),并复位驾驶权。
handlers['you'][0]({ sid: 'A' });
handlers['motor_state'][0]({ armed: false, driver: null });

// 默认单杆:按钮高亮 + 2D 盘直接产出 dx/dy。
ok('默认单杆:「单杆」on、「双杆」不 on',
   gimbalModeBtns[1].classList.contains('on') && !gimbalModeBtns[0].classList.contains('on'));
touch(G, 1, 0);
ok('默认单杆:#joy-gimbal 推右 → dx=100 dy=0',
   (() => { const v = readGimbal(); return v.dx === 100 && v.dy === 0; })(), D(readGimbal()));
release(G);

// 双杆:两个独立单轴盘,各自只动一轴。
gimbalModeBtns[0].fire('click');
ok('切到双杆:「双杆」on、「单杆」不 on',
   gimbalModeBtns[0].classList.contains('on') && !gimbalModeBtns[1].classList.contains('on'));
touch(GPan, 1, 0);
ok('双杆:pan 推右 → dx=100 dy=0',
   (() => { const v = readGimbal(); return v.dx === 100 && v.dy === 0; })(), D(readGimbal()));
release(GPan);
touch(GTilt, 0, -1);
ok('双杆:tilt 推上 → dx=0 dy=100',
   (() => { const v = readGimbal(); return v.dx === 0 && v.dy === 100; })(), D(readGimbal()));
release(GTilt);
touch(GPan, 1, 0); touch(GTilt, 0, -1);
ok('双杆:双指同时 → dx=100 dy=100(两轴独立满量程)',
   (() => { const v = readGimbal(); return v.dx === 100 && v.dy === 100; })(), D(readGimbal()));
release(GPan); release(GTilt);

// 切回单杆:2D 盘恢复(圆门照旧)。
gimbalModeBtns[1].fire('click');
touch(G, 1, -1);
ok('切回单杆:2D 盘斜推 → 71/71(圆门照旧)',
   (() => { const v = readGimbal(); return v.dx === 71 && v.dy === 71; })(), D(readGimbal()));
release(G);

// 按住云台盘切模式 → 读数立刻归零 + 补发 (0,0)。
gimbalModeBtns[0].fire('click');
touch(GPan, 1, 0);
ok('切模式前云台读数非零', readGimbal().dx !== 0, D(readGimbal()));
emitted.length = 0;
gimbalModeBtns[1].fire('click');
ok('★ 按住云台盘切模式 → 读数立刻归零',
   (() => { const v = readGimbal(); return v.dx === 0 && v.dy === 0; })(), D(readGimbal()));
ok('★ 切模式补发云台 (0,0)',
   emitted.some(e => e.ev === 'gimbal_control' && e.p.dx === 0 && e.p.dy === 0));
release(GPan);

// 云台盘不受电机驾驶权管:驾驶权在别人手里,三个云台盘仍可点。
handlers['you'][0]({ sid: 'A' });
handlers['motor_state'][0]({ armed: true, driver: 'B' });
ok('★ 驾驶权在别人手里 → 三个云台盘仍可点',
   G.style.pointerEvents === 'auto' && GPan.style.pointerEvents === 'auto' && GTilt.style.pointerEvents === 'auto');
handlers['motor_state'][0]({ armed: false, driver: null });

// localStorage 记忆。
ok('localStorage 记住了 gimbalMode=single', store['gimbalMode'] === 'single', store['gimbalMode']);
gimbalModeBtns[0].fire('click');
ok('localStorage 记住了 gimbalMode=dual', store['gimbalMode'] === 'dual', store['gimbalMode']);
gimbalModeBtns[1].fire('click');

console.log('=== 14. 折叠驾驶面板安全 ===');
modeBtns[0].fire('click');
gimbalModeBtns[0].fire('click');
handlers['you'][0]({ sid: 'A' });
handlers['motor_state'][0]({ armed: true, driver: 'A' });
touch(P.thr, 0, -1);
touch(GPan, 1, 0);
read(); readGimbal();
emitted.length = 0;
const collapse = els['btn-drive-collapse'];
collapse.fire('click');
ok('折叠立刻补发电机零速', emitted.some(e => e.ev === 'motor_control' && e.p.action === 'vel' && e.p.linear === 0 && e.p.angular === 0));
ok('折叠立刻补发云台零速', emitted.some(e => e.ev === 'gimbal_control' && e.p.dx === 0 && e.p.dy === 0));
ok('折叠后驾驶和云台目标归零', read().l === 0 && read().a === 0 && readGimbal().dx === 0);
P.thr.fire('pointermove', { clientX: 75, clientY: 0, pointerId: 1 });
GPan.fire('pointermove', { clientX: 150, clientY: 75, pointerId: 1 });
ok('旧指针在隐藏后移动不能恢复输出', read().l === 0 && readGimbal().dx === 0);
ok('折叠状态和辅助说明同步', drive.classList.contains('collapsed') && collapse._attrs['aria-expanded'] === 'false');
collapse.fire('click');
ok('展开不会恢复旧指令', !drive.classList.contains('collapsed') && read().l === 0 && readGimbal().dx === 0 && collapse._attrs['aria-expanded'] === 'true');
ok('急停在固定顶栏，独立于可折叠驾驶区', HTML.indexOf('id="btn-estop"') > HTML.indexOf('id="status-bar"') && HTML.indexOf('id="btn-estop"') < HTML.indexOf('id="workspace"'));
emitted.length = 0;
els['btn-estop'].fire('click');
ok('折叠后急停仍发出 estop', emitted.some(e => e.ev === 'motor_control' && e.p.action === 'estop'));

console.log('\n' + pass + ' 通过, ' + fail + ' 失败');
console.log('⚠️ 真机上仍要手验一次: 按下摇杆不放 → 点切换 → 指令必须立刻归零。');
process.exit(fail ? 1 : 0);
