package report

const htmlTemplate = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{{.Title}}</title>
<style>
  :root {
    color-scheme: light dark;
    --surface: #fcfcfb;
    --surface-2: #f3f2ef;
    --border: #e3e2dd;
    --ink: #0b0b0b;
    --ink-2: #52514e;
    --ink-3: #8a8984;
    --path: #9aa0a6;
    --trail: #2a78d6;
    --target: #eb6834;
    --robot: #1baf7a;
    --bad: #e34948;
    --grid: #eceae5;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --surface: #1a1a19;
      --surface-2: #232322;
      --border: #34342f;
      --ink: #ffffff;
      --ink-2: #c3c2b7;
      --ink-3: #8e8d86;
      --path: #6b6f76;
      --trail: #3987e5;
      --target: #d95926;
      --robot: #199e70;
      --bad: #e66767;
      --grid: #262624;
    }
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--surface); color: var(--ink);
    font: 14px/1.5 -apple-system, BlinkMacSystemFont, "Segoe UI", "PingFang SC",
          "Hiragino Sans GB", "Microsoft YaHei", sans-serif;
  }
  header { padding: 18px 24px 10px; }
  h1 { font-size: 18px; margin: 0 0 4px; }
  .sub { color: var(--ink-2); font-size: 13px; }
  main { padding: 8px 24px 28px; }
  .tabs { display: flex; flex-wrap: wrap; gap: 8px; margin: 14px 0; }
  .tab {
    border: 1px solid var(--border); background: var(--surface-2);
    color: var(--ink-2); border-radius: 8px; padding: 6px 12px;
    cursor: pointer; font: inherit; font-size: 13px;
  }
  .tab[aria-selected="true"] {
    border-color: var(--trail); color: var(--trail); font-weight: 600;
  }
  .grid { display: grid; grid-template-columns: minmax(0, 1fr) 260px; gap: 16px; }
  @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
  .card {
    border: 1px solid var(--border); border-radius: 12px;
    background: var(--surface); padding: 12px 14px;
  }
  .card h2 { font-size: 13px; margin: 0 0 8px; color: var(--ink-2); font-weight: 600; }
  canvas { display: block; width: 100%; }
  #mapCanvas { height: 440px; }
  #chartWrap { position: relative; }
  #chartCanvas { height: 220px; }
  #tooltip {
    position: absolute; pointer-events: none; display: none; z-index: 5;
    background: var(--surface-2); border: 1px solid var(--border);
    border-radius: 8px; padding: 6px 9px; font-size: 12px; color: var(--ink);
    white-space: nowrap; box-shadow: 0 2px 10px rgba(0,0,0,.12);
  }
  .stat { display: grid; grid-template-columns: 1fr auto; gap: 4px 10px; font-size: 13px; }
  .stat dt { color: var(--ink-2); }
  .stat dd { margin: 0; text-align: right; font-variant-numeric: tabular-nums; }
  .badge {
    display: inline-block; padding: 2px 9px; border-radius: 999px;
    font-size: 12px; font-weight: 600;
  }
  .badge.ok  { color: var(--robot); border: 1px solid var(--robot); }
  .badge.bad { color: var(--bad); border: 1px solid var(--bad); }
  .badge.warn { color: var(--target); border: 1px solid var(--target); }
  .controls {
    display: flex; align-items: center; gap: 10px; flex-wrap: wrap;
    margin: 12px 0 4px;
  }
  button.ctl {
    border: 1px solid var(--border); background: var(--surface-2); color: var(--ink);
    border-radius: 8px; width: 34px; height: 32px; cursor: pointer; font-size: 15px;
  }
  button.ctl#play { width: 44px; }
  button.ctl:hover { border-color: var(--trail); }
  input[type=range] { flex: 1; min-width: 160px; accent-color: var(--trail); }
  select {
    border: 1px solid var(--border); background: var(--surface-2); color: var(--ink);
    border-radius: 8px; padding: 5px 8px; font: inherit;
  }
  .time { font-variant-numeric: tabular-nums; color: var(--ink-2); min-width: 110px; text-align: right; }
  .legend { display: flex; gap: 16px; flex-wrap: wrap; margin-top: 8px; font-size: 12px; color: var(--ink-2); }
  .legend span { display: inline-flex; align-items: center; gap: 6px; }
  .swatch { width: 18px; height: 3px; border-radius: 2px; display: inline-block; }
  .swatch.dash {
    height: 0; border-top: 2px dashed var(--bad); background: none; width: 18px;
  }
  .desc { color: var(--ink-2); font-size: 13px; margin: 4px 0 0; }
  .note { color: var(--ink-3); font-size: 12px; margin-top: 14px; }
</style>
</head>
<body>
<header>
  <h1>{{.Title}}</h1>
  <div class="sub">Pure Pursuit / Stanley 差速驱动路径跟踪闭环仿真 — 逐帧回放与横向误差分析（单文件离线报告）</div>
</header>
<main>
  <div class="tabs" id="tabs" role="tablist"></div>

  <div class="grid">
    <section class="card">
      <h2>路径与轨迹回放</h2>
      <canvas id="mapCanvas"></canvas>
      <div class="legend">
        <span><i class="swatch" style="background:var(--path)"></i>参考路径</span>
        <span><i class="swatch" style="background:var(--trail)"></i>实际轨迹</span>
        <span><i class="swatch" style="background:var(--target)"></i>前视点 / 最近点</span>
        <span><i class="swatch dash"></i>发散起点</span>
      </div>
    </section>
    <aside class="card">
      <h2>状态</h2>
      <div id="statusBadge"></div>
      <dl class="stat" id="stats" style="margin-top:10px"></dl>
      <p class="desc" id="desc"></p>
      <p class="note">空格 播放/暂停 · ←/→ 单帧步进</p>
    </aside>
  </div>

  <section class="card" style="margin-top:16px">
    <h2>横向误差 e(t)（米，正=位于路径左侧）</h2>
    <div id="chartWrap">
      <canvas id="chartCanvas"></canvas>
      <div id="tooltip"></div>
    </div>
    <div class="controls">
      <button class="ctl" id="restart" title="回到开头">⏮</button>
      <button class="ctl" id="prev" title="上一帧">◀</button>
      <button class="ctl" id="play" title="播放/暂停（空格）">▶</button>
      <button class="ctl" id="next" title="下一帧">▶</button>
      <input type="range" id="scrub" min="0" step="1" value="0">
      <span class="time" id="timeReadout">0.00 s</span>
      <select id="speed" title="回放速度">
        <option value="0.25">0.25×</option>
        <option value="0.5">0.5×</option>
        <option value="1" selected>1×</option>
        <option value="2">2×</option>
        <option value="4">4×</option>
      </select>
    </div>
  </section>
</main>

<script>
const REPORT = {{.DataJS}};

const $ = function (id) { return document.getElementById(id); };
const cssVar = function (name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
};

function fitCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.round(rect.width * dpr);
  canvas.height = Math.round(rect.height * dpr);
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx: ctx, w: rect.width, h: rect.height };
}

function bounds(sc) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  const eat = function (x, y) {
    if (x < x0) x0 = x; if (x > x1) x1 = x;
    if (y < y0) y0 = y; if (y > y1) y1 = y;
  };
  sc.path.forEach(function (p) { eat(p.x, p.y); });
  sc.frames.forEach(function (f) { eat(f.x, f.y); eat(f.tx, f.ty); });
  const m = Math.max((x1 - x0), (y1 - y0)) * 0.08 + 0.5;
  return { x0: x0 - m, y0: y0 - m, x1: x1 + m, y1: y1 + m };
}

// ---- 地图 ----
function drawMap(sc, k) {
  const canvas = $('mapCanvas');
  const g = fitCanvas(canvas);
  const ctx = g.ctx, W = g.w, H = g.h;
  const b = sc._bounds;
  const sx = W / (b.x1 - b.x0);
  const sy = H / (b.y1 - b.y0);
  const s = Math.min(sx, sy);
  const ox = (W - (b.x1 - b.x0) * s) / 2 - b.x0 * s;
  const oy = (H - (b.y1 - b.y0) * s) / 2 + b.y1 * s; // y 轴翻转
  const X = function (x) { return x * s + ox; };
  const Y = function (y) { return -y * s + oy; };

  ctx.clearRect(0, 0, W, H);
  ctx.lineCap = 'round'; ctx.lineJoin = 'round';

  // 网格
  const rawStep = (b.x1 - b.x0) / 10;
  const mag = Math.pow(10, Math.floor(Math.log10(rawStep)));
  const step = rawStep / mag >= 5 ? 5 * mag : (rawStep / mag >= 2 ? 2 * mag : mag);
  ctx.strokeStyle = cssVar('--grid'); ctx.lineWidth = 1;
  ctx.beginPath();
  for (let gx = Math.ceil(b.x0 / step) * step; gx <= b.x1; gx += step) {
    ctx.moveTo(X(gx), 0); ctx.lineTo(X(gx), H);
  }
  for (let gy = Math.ceil(b.y0 / step) * step; gy <= b.y1; gy += step) {
    ctx.moveTo(0, Y(gy)); ctx.lineTo(W, Y(gy));
  }
  ctx.stroke();

  // 参考路径
  ctx.strokeStyle = cssVar('--path'); ctx.lineWidth = 2.5;
  ctx.setLineDash([7, 5]);
  ctx.beginPath();
  sc.path.forEach(function (p, i) {
    if (i === 0) ctx.moveTo(X(p.x), Y(p.y)); else ctx.lineTo(X(p.x), Y(p.y));
  });
  ctx.stroke();
  ctx.setLineDash([]);

  // 起终点
  ctx.fillStyle = cssVar('--ink-3');
  const st = sc.path[0], en = sc.path[sc.path.length - 1];
  ctx.beginPath(); ctx.arc(X(st.x), Y(st.y), 4, 0, 2 * Math.PI); ctx.fill();
  ctx.strokeStyle = cssVar('--ink-3'); ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(X(en.x), Y(en.y), 4.5, 0, 2 * Math.PI); ctx.stroke();

  // 发散起点位置（红色叉）
  if (sc.divergent && sc.divergeStep >= 0 && sc.frames[sc.divergeStep]) {
    const df = sc.frames[sc.divergeStep];
    const cx = X(df.x), cy = Y(df.y), r = 7;
    ctx.strokeStyle = cssVar('--bad'); ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx - r, cy - r); ctx.lineTo(cx + r, cy + r);
    ctx.moveTo(cx + r, cy - r); ctx.lineTo(cx - r, cy + r);
    ctx.stroke();
  }

  // 已执行轨迹（0..k）
  ctx.strokeStyle = cssVar('--trail'); ctx.lineWidth = 2.5;
  ctx.beginPath();
  for (let i = 0; i <= k && i < sc.frames.length; i++) {
    const f = sc.frames[i];
    if (i === 0) ctx.moveTo(X(f.x), Y(f.y)); else ctx.lineTo(X(f.x), Y(f.y));
  }
  ctx.stroke();

  const f = sc.frames[k];
  if (!f) return;

  // 前视/瞄准连线与目标点
  ctx.strokeStyle = cssVar('--target'); ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(X(f.x), Y(f.y)); ctx.lineTo(X(f.tx), Y(f.ty));
  ctx.stroke();
  ctx.fillStyle = cssVar('--target');
  ctx.beginPath(); ctx.arc(X(f.tx), Y(f.ty), 4, 0, 2 * Math.PI); ctx.fill();

  // 机器人（三角形，沿航向）
  const rx = X(f.x), ry = Y(f.y), ang = -f.th, R = 9;
  ctx.save();
  ctx.translate(rx, ry); ctx.rotate(ang);
  ctx.fillStyle = cssVar('--robot');
  ctx.beginPath();
  ctx.moveTo(R * 1.3, 0);
  ctx.lineTo(-R, 0.75 * R);
  ctx.lineTo(-R * 0.55, 0);
  ctx.lineTo(-R, -0.75 * R);
  ctx.closePath(); ctx.fill();
  ctx.restore();
}

// ---- 误差曲线 ----
function chartGeom(sc) {
  const canvas = $('chartCanvas');
  const g = fitCanvas(canvas);
  const pad = { l: 48, r: 14, t: 12, b: 26 };
  return {
    ctx: g.ctx, w: g.w, h: g.h, pad: pad,
    pw: g.w - pad.l - pad.r, ph: g.h - pad.t - pad.b,
    tMax: sc.frames[sc.frames.length - 1].t
  };
}

function eRange(sc) {
  let m = 0.2;
  sc.frames.forEach(function (f) { if (Math.abs(f.e) > m) m = Math.abs(f.e); });
  return m * 1.15;
}

function drawChart(sc, k, hoverIdx) {
  const G = chartGeom(sc);
  const ctx = G.ctx;
  const eMax = sc._eMax;
  const X = function (t) { return G.pad.l + (t / G.tMax) * G.pw; };
  const Y = function (e) { return G.pad.t + ((eMax - e) / (2 * eMax)) * G.ph; };

  ctx.clearRect(0, 0, G.w, G.h);
  ctx.font = '11px -apple-system, sans-serif';
  ctx.textAlign = 'right'; ctx.textBaseline = 'middle';

  // 网格与纵轴刻度
  ctx.strokeStyle = cssVar('--grid'); ctx.lineWidth = 1;
  ctx.fillStyle = cssVar('--ink-3');
  const ticks = 4;
  for (let i = 0; i <= ticks; i++) {
    const ev = eMax - (2 * eMax * i) / ticks;
    const yy = Y(ev);
    ctx.beginPath(); ctx.moveTo(G.pad.l, yy); ctx.lineTo(G.pad.l + G.pw, yy); ctx.stroke();
    ctx.fillText(ev.toFixed(ev < 1 ? 2 : 1), G.pad.l - 7, yy);
  }
  // 零轴加粗
  ctx.strokeStyle = cssVar('--ink-3');
  ctx.beginPath(); ctx.moveTo(G.pad.l, Y(0)); ctx.lineTo(G.pad.l + G.pw, Y(0)); ctx.stroke();

  // 横轴时间刻度
  ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  const nt = 5;
  for (let i = 0; i <= nt; i++) {
    const tv = (G.tMax * i) / nt;
    ctx.fillText(tv.toFixed(1) + 's', X(tv), G.pad.t + G.ph + 7);
  }

  // 发散区域阴影 + 起点虚线
  if (sc.divergent && sc.divergeStep >= 0) {
    const t0 = sc.frames[sc.divergeStep].t;
    ctx.fillStyle = cssVar('--bad');
    ctx.globalAlpha = 0.08;
    ctx.fillRect(X(t0), G.pad.t, X(G.tMax) - X(t0), G.ph);
    ctx.globalAlpha = 1;
    ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5; ctx.strokeStyle = cssVar('--bad');
    ctx.beginPath(); ctx.moveTo(X(t0), G.pad.t); ctx.lineTo(X(t0), G.pad.t + G.ph); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = cssVar('--bad');
    ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText('发散起点 t=' + t0.toFixed(2) + 's', X(t0) + 5, G.pad.t + 3);
  }

  // e(t) 曲线
  ctx.strokeStyle = cssVar('--trail'); ctx.lineWidth = 2;
  ctx.beginPath();
  sc.frames.forEach(function (f, i) {
    const xx = X(f.t), yy = Y(f.e);
    if (i === 0) ctx.moveTo(xx, yy); else ctx.lineTo(xx, yy);
  });
  ctx.stroke();

  // 播放头
  const fk = sc.frames[Math.min(k, sc.frames.length - 1)];
  ctx.strokeStyle = cssVar('--ink-2'); ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(X(fk.t), G.pad.t); ctx.lineTo(X(fk.t), G.pad.t + G.ph);
  ctx.stroke();
  ctx.fillStyle = cssVar('--trail');
  ctx.beginPath(); ctx.arc(X(fk.t), Y(fk.e), 3.5, 0, 2 * Math.PI); ctx.fill();

  // 悬停十字线
  if (hoverIdx != null && hoverIdx >= 0) {
    const hf = sc.frames[hoverIdx];
    ctx.strokeStyle = cssVar('--ink-3');
    ctx.beginPath();
    ctx.moveTo(X(hf.t), G.pad.t); ctx.lineTo(X(hf.t), G.pad.t + G.ph);
    ctx.stroke();
    ctx.fillStyle = cssVar('--ink');
    ctx.beginPath(); ctx.arc(X(hf.t), Y(hf.e), 4, 0, 2 * Math.PI); ctx.fill();
  }
  return { X: X, Y: Y, G: G };
}

// ---- 场景状态 ----
let cur = 0, frame = 0, playing = false, raf = null, lastTs = 0, hover = null;

function scenario() { return REPORT.scenarios[cur]; }

function showDesc() { $('desc').textContent = scenario().description || ''; }

function fmt(x, d) {
  return (x === undefined || x === null) ? '—' :
    (x >= 0 ? ' ' : '') + x.toFixed(d === undefined ? 3 : d);
}

function renderPanel() {
  const sc = scenario();
  const badge = $('statusBadge');
  if (sc.divergent) {
    badge.innerHTML = '<span class="badge bad">失稳：横向误差发散</span>';
  } else if (sc.reachedGoal) {
    badge.innerHTML = '<span class="badge ok">稳定：已收敛并到达终点</span>';
  } else {
    badge.innerHTML = '<span class="badge warn">未发散但未到达终点</span>';
  }
  const rows = [
    ['算法', sc.algorithm === 'pure_pursuit' ? 'Pure Pursuit' : 'Stanley'],
    ['速度 v', fmt(sc.speed, 2) + ' m/s'],
    ['仿真步长', fmt(sc.dt, 3) + ' s'],
    ['帧数', String(sc.frames.length)],
    ['最大 |e|', fmt(sc.maxAbsCTE, 3) + ' m'],
    ['末端 e', fmt(sc.finalCTE, 3) + ' m'],
  ];
  if (sc.divergent) {
    rows.push(['发散起始步', '#' + sc.divergeStep]);
    rows.push(['发散起始时刻', fmt(sc.divergeTime, 2) + ' s']);
    rows.push(['确认步', '#' + sc.confirmStep]);
  }
  let html = '';
  rows.forEach(function (r) {
    html += '<dt>' + r[0] + '</dt><dd>' + r[1] + '</dd>';
  });
  $('stats').innerHTML = html;
}

function render() {
  const sc = scenario();
  drawMap(sc, frame);
  drawChart(sc, frame, hover);
  $('scrub').max = sc.frames.length - 1;
  $('scrub').value = frame;
  const f = sc.frames[frame];
  $('timeReadout').textContent = f.t.toFixed(2) + ' s  ·  e=' + f.e.toFixed(3) + ' m';
  renderPanel();
}

function selectScenario(i) {
  cur = i; frame = 0; hover = null;
  pause();
  Array.prototype.forEach.call($('tabs').children, function (t, j) {
    t.setAttribute('aria-selected', j === i ? 'true' : 'false');
  });
  showDesc();
  render();
}

function tick(ts) {
  if (!playing) return;
  if (!lastTs) lastTs = ts;
  const sc = scenario();
  const realDT = (ts - lastTs) / 1000;
  lastTs = ts;
  const simStep = realDT * parseFloat($('speed').value) / sc.dt;
  frame += simStep;
  if (frame >= sc.frames.length - 1) {
    frame = sc.frames.length - 1;
    render();
    pause();
    return;
  }
  frame = Math.floor(frame);
  render();
  raf = requestAnimationFrame(tick);
}

function play() {
  if (frame >= scenario().frames.length - 1) frame = 0;
  playing = true; lastTs = 0;
  $('play').textContent = '⏸';
  raf = requestAnimationFrame(tick);
}
function pause() {
  playing = false; $('play').textContent = '▶';
  if (raf) cancelAnimationFrame(raf);
  raf = null;
}

// ---- 初始化 ----
REPORT.scenarios.forEach(function (sc, i) {
  sc._bounds = bounds(sc);
  sc._eMax = eRange(sc);
  const b = document.createElement('button');
  b.className = 'tab';
  b.textContent = (i + 1) + '. ' + sc.name;
  b.setAttribute('role', 'tab');
  b.setAttribute('aria-selected', i === 0 ? 'true' : 'false');
  b.addEventListener('click', function () {
    history.replaceState(null, '', '#' + (i + 1));
    selectScenario(i);
  });
  $('tabs').appendChild(b);
});

$('play').addEventListener('click', function () { playing ? pause() : play(); });
$('restart').addEventListener('click', function () { pause(); frame = 0; render(); });
$('prev').addEventListener('click', function () { pause(); frame = Math.max(0, frame - 1); render(); });
$('next').addEventListener('click', function () {
  pause(); frame = Math.min(scenario().frames.length - 1, frame + 1); render();
});
$('scrub').addEventListener('input', function (ev) {
  pause(); frame = parseInt(ev.target.value, 10); render();
});
document.addEventListener('keydown', function (ev) {
  if (ev.target.tagName === 'SELECT' || ev.target.tagName === 'INPUT') return;
  if (ev.code === 'Space') { ev.preventDefault(); playing ? pause() : play(); }
  else if (ev.code === 'ArrowLeft') { pause(); frame = Math.max(0, frame - 1); render(); }
  else if (ev.code === 'ArrowRight') {
    pause(); frame = Math.min(scenario().frames.length - 1, frame + 1); render();
  }
});

// 图表悬停十字线 + 提示框
$('chartCanvas').addEventListener('mousemove', function (ev) {
  const sc = scenario();
  const r = ev.target.getBoundingClientRect();
  const mx = ev.clientX - r.left;
  const geom = chartGeom(sc);
  if (mx < geom.pad.l || mx > geom.pad.l + geom.pw) { hover = null; render(); return; }
  const t = ((mx - geom.pad.l) / geom.pw) * geom.tMax;
  const idx = Math.round(t / sc.dt);
  hover = Math.max(0, Math.min(sc.frames.length - 1, idx));
  drawChart(sc, frame, hover);
  const f = sc.frames[hover];
  const tip = $('tooltip');
  tip.style.display = 'block';
  tip.innerHTML = 't = ' + f.t.toFixed(2) + ' s<br>e = ' + f.e.toFixed(4) + ' m' +
    (sc.divergent && hover === sc.divergeStep ? '<br><b>发散起点</b>' : '');
  const wrap = $('chartWrap').getBoundingClientRect();
  let tx = ev.clientX - wrap.left + 12;
  if (tx + 130 > wrap.width) tx -= 150;
  tip.style.left = tx + 'px';
  tip.style.top = (ev.clientY - wrap.top - 30) + 'px';
});
$('chartCanvas').addEventListener('mouseleave', function () {
  hover = null; $('tooltip').style.display = 'none'; render();
});

let resizeT = null;
window.addEventListener('resize', function () {
  clearTimeout(resizeT);
  resizeT = setTimeout(render, 80);
});

showDesc();
const hashIdx = parseInt(location.hash.replace('#', ''), 10);
if (hashIdx >= 1 && hashIdx <= REPORT.scenarios.length) {
  selectScenario(hashIdx - 1);
} else {
  render();
}
window.addEventListener('hashchange', function () {
  const i = parseInt(location.hash.replace('#', ''), 10) - 1;
  if (i >= 0 && i < REPORT.scenarios.length) selectScenario(i);
});
</script>
</body>
</html>
`
