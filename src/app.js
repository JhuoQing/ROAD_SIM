(function () {
'use strict';
const $ = (s, r) => (r || document).querySelector(s);
const $$ = (s, r) => Array.from((r || document).querySelectorAll(s));
const W = RS.W;
const STORE = 'roadsim.v1';
const DEF_OPT = { flow: 300, truck: 8, wL: 20, wS: 60, wR: 20, seed: 1 };

let net = RS.createNet();
let opt = Object.assign({}, DEF_OPT);
let sim = null;
const props = { lanes: 1, limit: 50, oneway: false };
const ui = { tool: 'select', color: 'speed', overlay: 'none', running: true, speed: 1, metric: 'v',
  sel: null, hover: null, drawing: null, cur: null, shift: false, chain: false, timer: true, arrows: true };
const view = { x: 0, y: 0, z: 1.2 };
const undoS = [], redoS = [];
let nodeRows = [], edgeSt = [], edgeStT = 0;

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const fmt = (v, d) => (isFinite(v) ? v : 0).toFixed(d || 0);
function toast(msg) {
  const t = $('#toast'); t.textContent = msg; t.classList.add('show');
  clearTimeout(toast.h); toast.h = setTimeout(() => t.classList.remove('show'), 2400);
}
const roadById = id => net.roads.find(r => r.id === id);
const nodeById = id => RS.N(net, id);
function dseg(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
  let t = l2 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0; t = clamp(t, 0, 1);
  return Math.hypot(ax + dx * t - px, ay + dy * t - py);
}

/* ------------------------------------------------------------------ */
/* network state, undo, persistence                                    */
/* ------------------------------------------------------------------ */
function newSim() { sim = new RS.Sim(net, opt); nodeRows = []; edgeSt = []; edgeStT = 0; }
function commit() {
  newSim();
  if (ui.sel) {
    const ok = ui.sel.type === 'road' ? roadById(ui.sel.id) : nodeById(ui.sel.id);
    if (!ok) ui.sel = null;
  }
  renderInspector(); updateButtons(); scheduleSave();
}
function snap() { return JSON.stringify(net); }
function edit(fn) {
  const before = snap();
  if (fn() === false) { net = JSON.parse(before); return false; }
  undoS.push(before); if (undoS.length > 100) undoS.shift(); redoS.length = 0;
  commit(); return true;
}
function undo() { if (!undoS.length) return; redoS.push(snap()); net = JSON.parse(undoS.pop()); ui.drawing = null; commit(); }
function redo() { if (!redoS.length) return; undoS.push(snap()); net = JSON.parse(redoS.pop()); ui.drawing = null; commit(); }
function updateButtons() {
  $('#undoBtn').disabled = !undoS.length; $('#redoBtn').disabled = !redoS.length;
  $('#empty').style.display = net.roads.length ? 'none' : 'flex';
}
let saveT = 0;
function scheduleSave() {
  clearTimeout(saveT);
  saveT = setTimeout(() => { try { localStorage.setItem(STORE, JSON.stringify({ v: 1, net, opt })); } catch (e) {} }, 400);
}
function sanitize(data) {
  if (!data || !data.net || !Array.isArray(data.net.nodes) || !Array.isArray(data.net.roads)) throw new Error('格式不符：找不到 net.nodes / net.roads');
  const n = { nodes: [], roads: [], nid: 1, rid: 1 };
  const ids = new Set();
  for (const a of data.net.nodes) {
    if (!isFinite(a.x) || !isFinite(a.y) || !isFinite(a.id)) continue;
    const ph = v => { v = Math.round(+v || 0); return v <= 0 ? 0 : clamp(v, 5, 60); };
    const sg = a.sig && isFinite(a.sig.g0) ? { g0: clamp(+a.sig.g0, 5, 120), g1: clamp(+a.sig.g1, 5, 120), lA: ph(a.sig.lA), lB: ph(a.sig.lB), rA: ph(a.sig.rA), rB: ph(a.sig.rB),
      y: clamp(+a.sig.y, 1, 6), r: clamp(+a.sig.r, 0, 6), off: clamp(+a.sig.off || 0, 0, 300), timer: a.sig.timer !== false } : null;
    n.nodes.push({ id: +a.id, x: +a.x, y: +a.y, sig: sg, flow: a.flow != null && isFinite(a.flow) ? Math.max(0, +a.flow) : null });
    ids.add(+a.id); n.nid = Math.max(n.nid, +a.id + 1);
  }
  for (const r of data.net.roads) {
    if (!ids.has(+r.a) || !ids.has(+r.b) || +r.a === +r.b) continue;
    n.roads.push({ id: +r.id || n.rid, a: +r.a, b: +r.b, lanes: clamp(Math.round(+r.lanes || 1), 1, 3), limit: clamp(+r.limit || 50, 10, 120), oneway: !!r.oneway });
    n.rid = Math.max(n.rid, (+r.id || 0) + 1, n.roads.length + 1);
  }
  RS.pruneNodes(n);
  return { net: n, opt: Object.assign({}, DEF_OPT, data.opt || {}) };
}
function applyData(data, keepUndo) {
  const s = sanitize(data);
  if (keepUndo) { undoS.push(snap()); redoS.length = 0; } else { undoS.length = 0; redoS.length = 0; }
  net = s.net; opt = s.opt; ui.sel = null; ui.drawing = null;
  syncOptUI(); commit(); fitView();
}

/* ------------------------------------------------------------------ */
/* templates                                                           */
/* ------------------------------------------------------------------ */
function sigAll(n, stagger) {
  let i = 0;
  for (const nd of n.nodes) if (RS.nodeDegree(n, nd.id) >= 3) { nd.sig = RS.defaultSig(); nd.sig.off = (i++ % 2) * stagger; }
}
const TEMPLATES = {
  cross() { const n = RS.createNet(), p = { lanes: 2, limit: 50, oneway: false };
    RS.addSegment(n, -300, 0, 300, 0, p); RS.addSegment(n, 0, -300, 0, 300, p); sigAll(n, 0); return n; },
  crossL() { const n = RS.createNet(), p = { lanes: 2, limit: 50, oneway: false };
    RS.addSegment(n, -300, 0, 300, 0, p); RS.addSegment(n, 0, -300, 0, 300, p); sigAll(n, 0);
    for (const nd of n.nodes) if (nd.sig) { nd.sig.g0 = 25; nd.sig.g1 = 25; nd.sig.lA = 12; nd.sig.lB = 12; } return n; },
  t() { const n = RS.createNet(), p = { lanes: 1, limit: 50, oneway: false };
    RS.addSegment(n, -300, 0, 300, 0, p); RS.addSegment(n, 0, 0, 0, 300, p); return n; },
  bottle() { const n = RS.createNet();
    RS.addSegment(n, -350, 0, 0, 0, { lanes: 3, limit: 60, oneway: false });
    RS.addSegment(n, 0, 0, 350, 0, { lanes: 1, limit: 60, oneway: false }); return n; },
  grid() { const n = RS.createNet(), p = { lanes: 1, limit: 50, oneway: false };
    for (const v of [-150, 150]) { RS.addSegment(n, -350, v, 350, v, p); RS.addSegment(n, v, -350, v, 350, p); }
    sigAll(n, 30); return n; },
  blank() { return RS.createNet(); }
};
function loadTemplate(key) {
  if (!TEMPLATES[key]) return;
  undoS.push(snap()); redoS.length = 0;
  net = TEMPLATES[key](); ui.sel = null; ui.drawing = null;
  if (key === 'bottle') { opt.flow = Math.max(opt.flow, 400); syncOptUI(); }
  commit(); fitView();
}

/* ------------------------------------------------------------------ */
/* view                                                                */
/* ------------------------------------------------------------------ */
const cv = $('#cv'), ctx = cv.getContext('2d');
let cw = 800, ch = 500, dpr = 1;
function resize() {
  const r = cv.getBoundingClientRect(); dpr = window.devicePixelRatio || 1;
  cw = Math.max(50, r.width); ch = Math.max(50, r.height);
  cv.width = Math.round(cw * dpr); cv.height = Math.round(ch * dpr);
  sizeChart();
}
function fitView() {
  let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const n of net.nodes) { x0 = Math.min(x0, n.x); x1 = Math.max(x1, n.x); y0 = Math.min(y0, n.y); y1 = Math.max(y1, n.y); }
  if (x0 > x1) { x0 = -200; x1 = 200; y0 = -120; y1 = 120; }
  const w = Math.max(80, x1 - x0) + 80, h = Math.max(80, y1 - y0) + 80;
  view.x = (x0 + x1) / 2; view.y = (y0 + y1) / 2;
  view.z = clamp(Math.min(cw / w, ch / h), 0.15, 6);
}
const s2w = (sx, sy) => ({ x: view.x + (sx - cw / 2) / view.z, y: view.y + (sy - ch / 2) / view.z });
function zoomAt(sx, sy, f) {
  const p = s2w(sx, sy); view.z = clamp(view.z * f, 0.15, 14);
  view.x = p.x - (sx - cw / 2) / view.z; view.y = p.y - (sy - ch / 2) / view.z;
}

/* ------------------------------------------------------------------ */
/* picking & snapping                                                  */
/* ------------------------------------------------------------------ */
function nodeRadius(n) { return RS.nodeDegree(net, n.id) >= 3 ? Math.max(10 / view.z, 5) : 10 / view.z; }
function pick(wx, wy, forDelete) {
  let bn = null, bd = 1e9;
  for (const n of net.nodes) {
    const d = Math.hypot(n.x - wx, n.y - wy);
    if (d <= nodeRadius(n) && d < bd) { bd = d; bn = n; }
  }
  if (bn) {
    const deg = RS.nodeDegree(net, bn.id);
    if (!forDelete || deg !== 2) return { type: 'node', id: bn.id };
  }
  let br = null; bd = 1e9;
  for (const r of net.roads) {
    const A = nodeById(r.a), B = nodeById(r.b); if (!A || !B) continue;
    const d = dseg(wx, wy, A.x, A.y, B.x, B.y);
    const tol = (r.oneway ? r.lanes * W * 0.7 : r.lanes * W) + 4 / view.z;
    if (d <= tol && d < bd) { bd = d; br = r; }
  }
  return br ? { type: 'road', id: br.id } : null;
}
function snapPoint(wx, wy, shift) {
  const D = ui.drawing;
  if (shift && D) {
    let a = Math.atan2(wy - D.y, wx - D.x), l = Math.hypot(wx - D.x, wy - D.y);
    a = Math.round(a / (Math.PI / 12)) * (Math.PI / 12); l = Math.max(0, Math.round(l / 5) * 5);
    return { x: D.x + Math.cos(a) * l, y: D.y + Math.sin(a) * l, kind: 'angle' };
  }
  const tol = 12 / view.z;
  const n = RS.nearNode(net, wx, wy, Math.max(tol, 5));
  if (n) return { x: n.x, y: n.y, kind: 'node' };
  const p = RS.projectOnRoad(net, wx, wy, tol * 0.8);
  if (p) return { x: p.x, y: p.y, kind: 'road' };
  return { x: Math.round(wx / 5) * 5, y: Math.round(wy / 5) * 5, kind: 'grid' };
}

/* ------------------------------------------------------------------ */
/* tools                                                               */
/* ------------------------------------------------------------------ */
function setTool(t) {
  ui.tool = t; ui.drawing = null; ui.hover = null;
  $$('.tool').forEach(b => b.classList.toggle('on', b.dataset.tool === t));
  cv.style.cursor = t === 'draw' ? 'crosshair' : (t === 'delete' ? 'not-allowed' : (t === 'signal' ? 'pointer' : 'default'));
  updateHud();
}
function updateHud() {
  const m = {
    select: '<b>選取／移動</b>　點道路或節點查看屬性，拖曳節點可移動，拖曳空白處平移',
    draw: ui.drawing ? '<b>畫路中</b>　點下一點延伸；<b>雙擊／Enter／右鍵</b>結束，Esc 取消，Shift 吸附角度' : '<b>畫路</b>　點一下設定起點（可從既有道路上起畫，會自動形成路口）',
    signal: '<b>號誌</b>　點擊三向以上的路口，開關號誌',
    delete: '<b>刪除</b>　點道路刪除該路段；點路口／端點刪除其所有連接道路'
  };
  $('#hud').innerHTML = m[ui.tool];
  $('#finish').style.display = ui.drawing ? 'inline-flex' : 'none';
}
function finishDrawing() { ui.drawing = null; updateHud(); }
function placePoint(p) {
  if (!ui.drawing) { ui.drawing = { x: p.x, y: p.y }; updateHud(); return; }
  const D = ui.drawing;
  if (Math.hypot(p.x - D.x, p.y - D.y) < 10) { toast('這一段太短（至少 10 m）'); return; }
  const ok = edit(() => RS.addSegment(net, D.x, D.y, p.x, p.y, { lanes: props.lanes, limit: props.limit, oneway: props.oneway }));
  if (ok) ui.drawing = { x: p.x, y: p.y };
  updateHud();
}
function toggleSignal(id) {
  const n = nodeById(id); if (!n) return;
  if (RS.nodeDegree(net, id) < 3) { toast('號誌只能設在三向以上的路口'); return; }
  edit(() => { n.sig = n.sig ? null : RS.defaultSig(); });
  ui.sel = { type: 'node', id }; renderInspector();
}
function deleteSel(s) {
  if (!s) return;
  edit(() => { if (s.type === 'road') RS.deleteRoad(net, s.id); else RS.deleteNode(net, s.id); });
}

/* ------------------------------------------------------------------ */
/* pointer input                                                       */
/* ------------------------------------------------------------------ */
const ptrs = new Map();
let gest = null, lastPlace = { t: 0, x: 0, y: 0 };
function localXY(e) { const r = cv.getBoundingClientRect(); return { x: e.clientX - r.left, y: e.clientY - r.top }; }
cv.addEventListener('contextmenu', e => e.preventDefault());
cv.addEventListener('pointerdown', e => {
  cv.setPointerCapture(e.pointerId);
  const q = localXY(e);
  ptrs.set(e.pointerId, { x: q.x, y: q.y });
  if (ptrs.size === 2) {
    const [a, b] = Array.from(ptrs.values());
    gest = { mode: 'pinch', d: Math.hypot(a.x - b.x, a.y - b.y), cx: (a.x + b.x) / 2, cy: (a.y + b.y) / 2 };
    return;
  }
  gest = { mode: 'press', btn: e.button, sx: q.x, sy: q.y, vx: view.x, vy: view.y, moved: false, node: null };
  if (e.button === 0 && ui.tool === 'select') {
    const w = s2w(q.x, q.y), h = pick(w.x, w.y);
    if (h && h.type === 'node') gest.node = { id: h.id, before: snap(), did: false };
  }
});
cv.addEventListener('pointermove', e => {
  const q = localXY(e);
  if (ptrs.has(e.pointerId)) ptrs.set(e.pointerId, { x: q.x, y: q.y });
  ui.shift = e.shiftKey;
  if (gest && gest.mode === 'pinch' && ptrs.size >= 2) {
    const [a, b] = Array.from(ptrs.values());
    const d = Math.hypot(a.x - b.x, a.y - b.y), cx = (a.x + b.x) / 2, cy = (a.y + b.y) / 2;
    view.x -= (cx - gest.cx) / view.z; view.y -= (cy - gest.cy) / view.z;
    if (gest.d > 0) zoomAt(cx, cy, d / gest.d);
    gest.d = d; gest.cx = cx; gest.cy = cy;
    return;
  }
  if (gest && gest.mode === 'press') {
    const dx = q.x - gest.sx, dy = q.y - gest.sy;
    if (!gest.moved && Math.hypot(dx, dy) > 6) gest.moved = true;
    if (gest.moved) {
      if (gest.node) {
        const w = s2w(q.x, q.y), n = nodeById(gest.node.id);
        if (n) { n.x = Math.round(w.x / 5) * 5; n.y = Math.round(w.y / 5) * 5; gest.node.did = true; newSim(); }
      } else {
        view.x = gest.vx - dx / view.z; view.y = gest.vy - dy / view.z;
      }
      return;
    }
  }
  const w = s2w(q.x, q.y);
  if (ui.tool === 'draw') ui.cur = snapPoint(w.x, w.y, e.shiftKey);
  else { ui.cur = null; ui.hover = (ui.tool === 'signal') ? null : pick(w.x, w.y, ui.tool === 'delete'); }
});
function endPtr(e) {
  const q = localXY(e);
  ptrs.delete(e.pointerId);
  if (gest && gest.mode === 'pinch') { if (ptrs.size < 2) gest = null; return; }
  if (!gest) return;
  const g = gest; gest = null;
  if (g.node && g.node.did) {
    undoS.push(g.node.before); if (undoS.length > 100) undoS.shift(); redoS.length = 0; commit(); return;
  }
  if (g.moved) return;
  if (g.btn === 2) { if (ui.drawing) finishDrawing(); return; }
  if (g.btn !== 0 && g.btn !== undefined && g.btn !== -1) return;
  clickAt(q.x, q.y, e);
}
cv.addEventListener('pointerup', endPtr);
cv.addEventListener('pointercancel', e => { ptrs.delete(e.pointerId); gest = null; });
cv.addEventListener('pointerleave', () => { if (!gest) { ui.cur = null; ui.hover = null; } });
cv.addEventListener('wheel', e => {
  e.preventDefault(); const q = localXY(e);
  zoomAt(q.x, q.y, Math.exp(-e.deltaY * (e.ctrlKey ? 0.01 : 0.0015)));
}, { passive: false });

function clickAt(sx, sy, e) {
  const w = s2w(sx, sy);
  if (ui.tool === 'select') {
    const h = pick(w.x, w.y); ui.sel = h; renderInspector();
  } else if (ui.tool === 'draw') {
    const now = performance.now();
    if (ui.drawing && now - lastPlace.t < 380 && Math.hypot(sx - lastPlace.x, sy - lastPlace.y) < 12) { finishDrawing(); lastPlace.t = 0; return; }
    lastPlace = { t: now, x: sx, y: sy };
    placePoint(snapPoint(w.x, w.y, e.shiftKey));
  } else if (ui.tool === 'signal') {
    const h = pick(w.x, w.y);
    if (h && h.type === 'node') toggleSignal(h.id);
    else toast('請點擊三向以上的路口');
  } else if (ui.tool === 'delete') {
    const h = pick(w.x, w.y, true); if (h) { if (ui.sel && ui.sel.type === h.type && ui.sel.id === h.id) ui.sel = null; deleteSel(h); }
  }
}

/* ------------------------------------------------------------------ */
/* inspector                                                           */
/* ------------------------------------------------------------------ */
const LIMITS = [30, 40, 50, 60, 70, 80, 100];
function chainOf(rid) {
  const set = new Set([rid]), q = [rid];
  while (q.length) {
    const r = roadById(q.pop()); if (!r) continue;
    for (const nid of [r.a, r.b]) if (RS.nodeDegree(net, nid) === 2) {
      for (const o of net.roads) if (o.id !== r.id && (o.a === nid || o.b === nid) && !set.has(o.id)) { set.add(o.id); q.push(o.id); }
    }
  }
  return Array.from(set);
}
let inspBusy = false, inspAgain = false;
function renderInspector() {
  if (inspBusy) { inspAgain = true; return; }
  inspBusy = true;
  try { renderInspector1(); } finally { inspBusy = false; }
  if (inspAgain) { inspAgain = false; renderInspector(); }
}
function renderInspector1() {
  const el = $('#insp'), s = ui.sel; let h = '';
  if (!s) {
    h = '<div class="card"><h3>選取內容</h3><div class="hint">尚未選取。用「選取」工具點一條道路、一個路口或道路端點，這裡會顯示可調整的屬性。</div></div>';
  } else if (s.type === 'road') {
    const r = roadById(s.id); if (!r) { el.innerHTML = ''; return; }
    const A = nodeById(r.a), B = nodeById(r.b), L = Math.hypot(A.x - B.x, A.y - B.y);
    h = `<div class="card" data-kind="road"><h3>道路 #${r.id}</h3>
      <div class="kv"><span>長度</span><b class="num">${fmt(L, 0)} m</b></div>
      <div class="field"><label>每向車道</label><div class="seg" data-act="lanes">${[1, 2, 3].map(n => `<button data-v="${n}" class="${r.lanes === n ? 'on' : ''}">${n}</button>`).join('')}</div></div>
      <div class="field"><label>速限</label><select data-act="limit">${LIMITS.map(v => `<option ${v === r.limit ? 'selected' : ''}>${v}</option>`).join('')}</select></div>
      <label class="chk"><input type="checkbox" data-act="oneway" ${r.oneway ? 'checked' : ''}>單向（沿繪製方向）</label>
      <label class="chk"><input type="checkbox" id="chainChk" ${ui.chain ? 'checked' : ''}>同時套用到相連的整條道路</label>
      <button class="btn danger block" data-act="delRoad">刪除此路段</button></div>`;
  } else {
    const n = nodeById(s.id); if (!n) { el.innerHTML = ''; return; }
    const deg = RS.nodeDegree(net, n.id);
    if (deg === 1) {
      const gn = sim && sim.g.nodes.get(n.id), sinkOnly = gn && gn.out.length === 0, srcOnly = gn && gn.in.length === 0;
      h = `<div class="card"><h3>${sinkOnly ? '出口' : '入口／出口'} #${n.id}</h3>
        <div class="hint" style="margin-bottom:8px">${sinkOnly ? '單向道路的終點，只有車輛離開。' : (srcOnly ? '單向道路的起點，只產生車輛。' : '道路端點：車輛由此進入路網，也由此離開。')}</div>
        ${sinkOnly ? '' : `<div class="field"><label>入口流量</label><input type="number" data-act="nflow" min="0" max="3000" step="25" placeholder="預設 ${opt.flow}" value="${n.flow != null ? n.flow : ''}"></div>
        <div class="hint" style="margin-bottom:8px">單位 veh/h；留空表示使用全域預設值。</div>`}
        <button class="btn danger block" data-act="delNode">刪除此端點與道路</button></div>`;
    } else if (deg === 2) {
      h = `<div class="card"><h3>連接點 #${n.id}</h3><div class="hint" style="margin-bottom:8px">兩段道路之間的轉折或車道數變化點。</div>
        <button class="btn danger block" data-act="delNode">刪除此點與相連道路</button></div>`;
    } else {
      const sg = n.sig;
      const num = (k, lab, mn, mx, hint) => `<div class="field"><label>${lab}</label><input type="number" data-act="sg" data-k="${k}" min="${mn}" max="${mx}" value="${sg[k] || 0}" ${hint ? 'title="' + hint + '"' : ''}></div>`;
      let prog = '';
      if (sg) {
        const c = RS.compileSig(Object.assign({}, sg));
        prog = '<div class="phase">' + c.ph.map(p => `<span class="${p.group ? 'b' : ''}">${p.group ? 'B' : 'A'} ${p.mv === 'L' ? '左轉專用' : (p.mv === 'R' ? '右轉專用' : '直行' + (p.mv.includes('L') ? '＋左' : '') + (p.mv.includes('R') ? '＋右' : ''))} ${p.dur}s</span>`).join('') + '</div>';
      }
      h = `<div class="card"><h3>路口 #${n.id}</h3>
        <div class="kv"><span>連接道路</span><b>${deg} 條</b></div>
        <label class="chk"><input type="checkbox" data-act="sig" ${sg ? 'checked' : ''}>啟用號誌</label>
        ${sg ? `<label class="chk"><input type="checkbox" data-act="timer" ${sg.timer !== false ? 'checked' : ''}>顯示這個路口的倒數秒數</label>
        <div class="sub">直行綠燈（秒）</div>
        ${num('g0', 'A 軸', 5, 120)}${num('g1', 'B 軸', 5, 120)}
        <div class="sub">左轉專用相位（0＝不設）</div>
        ${num('lA', 'A 軸', 0, 60)}${num('lB', 'B 軸', 0, 60)}
        <div class="sub">右轉專用相位（0＝不設）</div>
        ${num('rA', 'A 軸', 0, 60)}${num('rB', 'B 軸', 0, 60)}
        <div class="sub">燈號時間</div>
        ${num('y', '黃燈', 1, 6)}${num('r', '全紅', 0, 6)}${num('off', '時差', 0, 300)}
        <div class="kv"><span>週期</span><b class="num">${RS.sigCycle(Object.assign({}, sg))} s</b></div>
        <div class="sub">相位順序</div>${prog}
        <div class="hint">A／B 是互相垂直的兩個方向軸，標籤顯示在畫面上各進口道旁。未設專用相位時，左轉在直行綠燈內讓行；設了專用相位，該軸左／右轉只在自己的相位才放行，紅燈（含右轉）一律停等。方向有 2 條以上車道時，最內側車道會成為左轉專用道（右轉專用需再多一條車道），單車道會共用車道。</div>`
        : '<div class="hint">未啟用號誌：車輛依讓行規則通過（左轉讓直行與右轉、先到先走）。</div>'}
        <button class="btn danger block" style="margin-top:10px" data-act="delNode">刪除此路口與相連道路</button></div>`;
    }
  }
  el.innerHTML = h;
}
$('#insp').addEventListener('click', e => {
  const b = e.target.closest('button'); if (!b || !ui.sel) return;
  const act = (b.closest('[data-act]') || b).dataset.act;
  if (act === 'lanes') {
    const v = +b.dataset.v, r = roadById(ui.sel.id); if (!r) return;
    const ids = ui.chain ? chainOf(r.id) : [r.id];
    edit(() => { for (const id of ids) roadById(id).lanes = v; });
  } else if (act === 'delRoad') { const s = ui.sel; ui.sel = null; deleteSel(s); }
  else if (act === 'delNode') { const s = ui.sel; ui.sel = null; deleteSel(s); }
});
$('#insp').addEventListener('change', e => {
  const t = e.target, act = t.dataset.act;
  if (t.id === 'chainChk') { ui.chain = t.checked; return; }
  if (!act || !ui.sel) return;
  if (act === 'limit') {
    const r = roadById(ui.sel.id), v = +t.value; const ids = ui.chain ? chainOf(r.id) : [r.id];
    edit(() => { for (const id of ids) roadById(id).limit = v; });
  } else if (act === 'oneway') {
    const r = roadById(ui.sel.id), v = t.checked; const ids = ui.chain ? chainOf(r.id) : [r.id];
    edit(() => { for (const id of ids) roadById(id).oneway = v; });
  } else if (act === 'nflow') {
    const n = nodeById(ui.sel.id), raw = t.value.trim();
    edit(() => { n.flow = raw === '' ? null : clamp(+raw || 0, 0, 3000); });
  } else if (act === 'sig') {
    const n = nodeById(ui.sel.id); edit(() => { n.sig = t.checked ? RS.defaultSig() : null; });
  } else if (act === 'timer') {
    const n = nodeById(ui.sel.id); edit(() => { n.sig.timer = t.checked; });
  } else if (act === 'sg') {
    const n = nodeById(ui.sel.id), k = t.dataset.k, lim = { g0: [5, 120], g1: [5, 120], lA: [0, 60], lB: [0, 60], rA: [0, 60], rB: [0, 60], y: [1, 6], r: [0, 6], off: [0, 300] }[k];
    edit(() => { let v = clamp(Math.round(+t.value || 0), lim[0], lim[1]); if (lim[0] === 0 && v > 0 && v < 5) v = 5; n.sig[k] = v; });
  }
});

/* ------------------------------------------------------------------ */
/* global controls                                                     */
/* ------------------------------------------------------------------ */
function syncOptUI() {
  $('#rFlow').value = opt.flow; $('#rTruck').value = opt.truck;
  $('#rL').value = opt.wL; $('#rS').value = opt.wS; $('#rR').value = opt.wR; $('#seed').value = opt.seed;
  updateOptLabels();
}
function updateOptLabels() {
  $('#oFlow').textContent = opt.flow; $('#oTruck').textContent = opt.truck + '%';
  const t = (opt.wL + opt.wS + opt.wR) || 1;
  $('#oL').textContent = Math.round(opt.wL / t * 100) + '%'; $('#oS').textContent = Math.round(opt.wS / t * 100) + '%'; $('#oR').textContent = Math.round(opt.wR / t * 100) + '%';
}
function liveOpt() { if (sim) Object.assign(sim.opt, opt); updateOptLabels(); scheduleSave(); }
[['rFlow', 'flow'], ['rTruck', 'truck'], ['rL', 'wL'], ['rS', 'wS'], ['rR', 'wR']].forEach(([id, k]) => {
  $('#' + id).addEventListener('input', e => { opt[k] = +e.target.value; liveOpt(); if (k === 'flow') { const s = ui.sel; if (s && s.type === 'node') { const i = $('#insp [data-act=nflow]'); if (i) i.placeholder = '預設 ' + opt.flow; } } });
});
$('#seedBtn').addEventListener('click', () => {
  opt.seed = clamp(Math.round(+$('#seed').value) || 1, 1, 999999); $('#seed').value = opt.seed; commit();
});
$('#seed').addEventListener('keydown', e => { if (e.key === 'Enter') $('#seedBtn').click(); });

$$('.tool').forEach(b => b.addEventListener('click', () => setTool(b.dataset.tool)));
$$('#newLanes button').forEach(b => b.addEventListener('click', () => {
  props.lanes = +b.dataset.v; $$('#newLanes button').forEach(x => x.classList.toggle('on', x === b));
}));
$('#newLimit').addEventListener('change', e => { props.limit = +e.target.value; });
$('#newOneway').addEventListener('change', e => { props.oneway = e.target.checked; });
$('#colorSel').addEventListener('change', e => { ui.color = e.target.value; });
$('#overlaySel').addEventListener('change', e => { ui.overlay = e.target.value; });
$('#timerChk').addEventListener('change', e => { ui.timer = e.target.checked; });
$('#arrowChk').addEventListener('change', e => { ui.arrows = e.target.checked; });
$('#tplSel').addEventListener('change', e => {
  const k = e.target.value; e.target.value = '';
  if (!k) return;
  if (net.roads.length && !confirmOverwrite()) return;
  loadTemplate(k);
});
function confirmOverwrite() { return true; }   // 可用復原回到先前路網
$('#undoBtn').addEventListener('click', undo);
$('#redoBtn').addEventListener('click', redo);
$('#playBtn').addEventListener('click', () => setRunning(!ui.running));
$('#stepBtn').addEventListener('click', () => { setRunning(false); for (let i = 0; i < 10; i++) sim.step(); sim.updateKpi(); });
$('#resetBtn').addEventListener('click', () => { newSim(); toast('模擬已重新開始'); });
$('#speedSel').addEventListener('change', e => { ui.speed = +e.target.value; });
$('#zIn').addEventListener('click', () => zoomAt(cw / 2, ch / 2, 1.35));
$('#zOut').addEventListener('click', () => zoomAt(cw / 2, ch / 2, 1 / 1.35));
$('#zFit').addEventListener('click', fitView);
$('#finish').addEventListener('click', finishDrawing);
$('#drawerBtn').addEventListener('click', () => $('#side').classList.toggle('open'));
$('#helpBtn').addEventListener('click', () => $('#helpModal').classList.add('show'));
$$('.modal').forEach(m => {
  m.addEventListener('click', e => { if (e.target === m || e.target.hasAttribute('data-close')) m.classList.remove('show'); });
});
function setRunning(v) { ui.running = v; $('#playBtn').textContent = v ? '暫停' : '播放'; }
function setSpeed(i) {
  const sel = $('#speedSel'); i = clamp(i, 0, sel.options.length - 1); sel.selectedIndex = i; ui.speed = +sel.value;
}

document.addEventListener('click', e => { const b = e.target.closest && e.target.closest('button'); if (b && e.detail > 0) b.blur(); });
window.addEventListener('keydown', e => {
  const tag = (e.target.tagName || '').toLowerCase();
  if (tag === 'input' || tag === 'textarea' || tag === 'select') { if (e.key === 'Escape') e.target.blur(); return; }
  if ($('.modal.show')) { if (e.key === 'Escape') $$('.modal').forEach(m => m.classList.remove('show')); return; }
  const k = e.key.toLowerCase();
  if ((e.ctrlKey || e.metaKey) && k === 'z') { e.preventDefault(); e.shiftKey ? redo() : undo(); return; }
  if ((e.ctrlKey || e.metaKey) && k === 'y') { e.preventDefault(); redo(); return; }
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  if (k === 'v') setTool('select'); else if (k === 'r') setTool('draw'); else if (k === 's') setTool('signal'); else if (k === 'x') setTool('delete');
  else if (k === ' ') { e.preventDefault(); setRunning(!ui.running); }
  else if (k === '.') $('#stepBtn').click();
  else if (k === '[') setSpeed($('#speedSel').selectedIndex - 1);
  else if (k === ']') setSpeed($('#speedSel').selectedIndex + 1);
  else if (k === 'f') fitView();
  else if (k === 'enter') { if (ui.drawing) finishDrawing(); }
  else if (k === 'escape') { if (ui.drawing) finishDrawing(); else { ui.sel = null; renderInspector(); } }
  else if (k === 'delete' || k === 'backspace') { if (ui.sel) { const s = ui.sel; ui.sel = null; deleteSel(s); } }
});

/* ------------------------------------------------------------------ */
/* save / load dialog                                                  */
/* ------------------------------------------------------------------ */
function svMsg(t, bad) { const m = $('#svMsg'); m.textContent = t; m.style.color = bad ? 'var(--bad)' : 'var(--muted)'; }
$('#saveBtn').addEventListener('click', () => { $('#saveModal').classList.add('show'); $('#svGen').click(); });
$('#svGen').addEventListener('click', () => { $('#svText').value = JSON.stringify({ v: 1, net, opt }, null, 1); svMsg('已產生目前路網（' + net.roads.length + ' 段道路）。'); });
$('#svCopy').addEventListener('click', async () => {
  const ta = $('#svText'); ta.focus(); ta.select();
  try { await navigator.clipboard.writeText(ta.value); svMsg('已複製到剪貼簿。'); }
  catch (e) { try { document.execCommand('copy'); svMsg('已複製（若失敗請手動全選複製）。'); } catch (e2) { svMsg('無法自動複製，請在文字框中手動全選並複製。', true); } }
});
$('#svLoad').addEventListener('click', () => {
  try { applyData(JSON.parse($('#svText').value), true); svMsg('載入完成。'); $('#saveModal').classList.remove('show'); }
  catch (e) { svMsg('載入失敗：' + e.message, true); }
});
$('#svFile').addEventListener('change', e => {
  const f = e.target.files && e.target.files[0]; if (!f) return;
  const rd = new FileReader();
  rd.onload = () => { $('#svText').value = String(rd.result); try { applyData(JSON.parse(rd.result), true); svMsg('已從檔案載入。'); $('#saveModal').classList.remove('show'); } catch (er) { svMsg('檔案內容無法載入：' + er.message, true); } };
  rd.readAsText(f); e.target.value = '';
});
$('#svCsv').addEventListener('click', () => {
  const rows = ['時間(s),路網車輛,平均車速(km/h),通過量(veh/h),平均延滯(s),待進車輛'];
  for (const p of sim.series) rows.push([p.t.toFixed(0), p.n, p.v.toFixed(1), p.thr.toFixed(0), p.delay.toFixed(1), p.vq].join(','));
  rows.push(''); rows.push('路口,號誌,流量(veh/h),平均延滯(s),LOS,最大排隊(m),平均排隊(m)');
  for (const r of sim.nodeTable()) rows.push([r.id, r.signal ? '有' : '無', r.flow.toFixed(0), r.delay.toFixed(1), r.los, r.maxQ.toFixed(0), r.avgQ.toFixed(1)].join(','));
  $('#svText').value = rows.join('\n'); svMsg('已產生指標 CSV，可按「複製」後貼進試算表。');
});

/* ------------------------------------------------------------------ */
/* rendering                                                           */
/* ------------------------------------------------------------------ */
const COL = { bg: '#e7f6ec', grid: '#d8eede', grid2: '#c7e3d0', asphalt: '#4f566b', accent: '#7c5cff', amber: '#ff9f1c', ink: '#2b2440', G: '#1fd27a', Y: '#ffc61a', R: '#ff3b5c' };
const LOSC = { A: '#0f9d63', B: '#0f9d63', C: '#6f9a0c', D: '#b87a00', E: '#d9590b', F: '#d92341' };
function lerp3(stops, t) {
  t = clamp(t, 0, 1) * (stops.length - 1); const i = Math.min(stops.length - 2, Math.floor(t)), f = t - i, a = stops[i], b = stops[i + 1];
  return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
}
const RAMP = [[255, 61, 113], [255, 197, 49], [53, 199, 255]];
const SPEED_COL = Array.from({ length: 10 }, (_, i) => { const c = lerp3(RAMP, i / 9); return `rgb(${c[0] | 0},${c[1] | 0},${c[2] | 0})`; });

function drawGrid() {
  const x0 = view.x - cw / 2 / view.z, x1 = view.x + cw / 2 / view.z, y0 = view.y - ch / 2 / view.z, y1 = view.y + ch / 2 / view.z;
  let step = 5; for (const s of [5, 10, 20, 50, 100, 200, 500, 1000]) { step = s; if (s * view.z >= 22) break; }
  ctx.lineWidth = 1 / view.z;
  for (let pass = 0; pass < 2; pass++) {
    ctx.strokeStyle = pass ? COL.grid2 : COL.grid; ctx.beginPath();
    for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) { const major = Math.round(x / step) % 5 === 0; if (major === !!pass) { ctx.moveTo(x, y0); ctx.lineTo(x, y1); } }
    for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) { const major = Math.round(y / step) % 5 === 0; if (major === !!pass) { ctx.moveTo(x0, y); ctx.lineTo(x1, y); } }
    ctx.stroke();
  }
}
function quad(e, from, to, off0, off1) {
  const ax = e.a.x + e.nx * off0, ay = e.a.y + e.ny * off0, bx = e.b.x + e.nx * off0, by = e.b.y + e.ny * off0;
  const cx = e.b.x + e.nx * off1, cy = e.b.y + e.ny * off1, dx = e.a.x + e.nx * off1, dy = e.a.y + e.ny * off1;
  ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.lineTo(cx, cy); ctx.lineTo(dx, dy); ctx.closePath();
}
function edgeBand(e, off0, off1) {   // trimmed part of the edge
  const ax = e.sx + e.nx * off0, ay = e.sy + e.ny * off0, bx = e.ex + e.nx * off0, by = e.ey + e.ny * off0;
  ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.lineTo(e.ex + e.nx * off1, e.ey + e.ny * off1); ctx.lineTo(e.sx + e.nx * off1, e.sy + e.ny * off1); ctx.closePath();
}
function drawRoads() {
  const g = sim.g;
  ctx.fillStyle = COL.asphalt; ctx.beginPath();
  for (const e of g.edges) quad(e, 0, 1, 0, e.lanes * W);
  ctx.fill();
  ctx.beginPath();
  for (const gn of g.nodes.values()) {
    if (gn.deg >= 3) { ctx.moveTo(gn.x + gn.R * 0.85, gn.y); ctx.arc(gn.x, gn.y, gn.R * 0.85, 0, 7); }
    else if (gn.deg === 2) { ctx.moveTo(gn.x + gn.maxLanes * W, gn.y); ctx.arc(gn.x, gn.y, gn.maxLanes * W, 0, 7); }
  }
  ctx.fill();
  if (ui.overlay !== 'none') {
    if (performance.now() - edgeStT > 200) { edgeSt = sim.edgeStats(); edgeStT = performance.now(); }
    const bins = {};
    for (let i = 0; i < g.edges.length; i++) {
      const s = edgeSt[i]; if (!s) continue;
      const t = ui.overlay === 'speed' ? clamp(s.ratio, 0, 1) : 1 - clamp(s.dens / 70, 0, 1);
      const k = Math.round((s.n || ui.overlay === 'speed' ? t : 1) * 9);
      (bins[k] = bins[k] || []).push(g.edges[i]);
    }
    ctx.globalAlpha = 0.5;
    for (const k in bins) {
      if (+k === 9 && ui.overlay === 'dens') continue;
      ctx.fillStyle = SPEED_COL[k]; ctx.beginPath();
      for (const e of bins[k]) edgeBand(e, 0, e.lanes * W);
      ctx.fill();
    }
    ctx.globalAlpha = 1;
  }
}
function laneMv(e, li) { const m = e.lm ? e.lm[li] : 'S'; return m.includes('S') ? 'S' : m[0]; }
const _si = ['R', 0, -1];
function drawMarks() {
  const g = sim.g, z = view.z;
  if (z > 0.35) {
    ctx.lineWidth = 0.18; ctx.strokeStyle = 'rgba(255,255,255,.6)'; ctx.setLineDash([3, 6]); ctx.beginPath();
    for (const e of g.edges) for (let k = 1; k < e.lanes; k++) { const o = k * W; ctx.moveTo(e.sx + e.nx * o, e.sy + e.ny * o); ctx.lineTo(e.ex + e.nx * o, e.ey + e.ny * o); }
    ctx.stroke(); ctx.setLineDash([]);
    ctx.strokeStyle = 'rgba(255,209,102,.95)'; ctx.lineWidth = 0.16; ctx.beginPath();
    for (const e of g.edges) if (e.rev > e.id) for (const o of [-0.16, 0.16]) { ctx.moveTo(e.sx + e.nx * o, e.sy + e.ny * o); ctx.lineTo(e.ex + e.nx * o, e.ey + e.ny * o); }
    ctx.stroke();
    ctx.strokeStyle = 'rgba(255,255,255,.4)'; ctx.lineWidth = 0.2; ctx.beginPath();
    for (const e of g.edges) {
      const o = e.lanes * W; ctx.moveTo(e.sx + e.nx * o, e.sy + e.ny * o); ctx.lineTo(e.ex + e.nx * o, e.ey + e.ny * o);
      if (e.rev < 0) { ctx.moveTo(e.sx, e.sy); ctx.lineTo(e.ex, e.ey); }
    }
    ctx.stroke();
  }
  if (ui.arrows) drawArrows();
  // stop lines (one segment per lane, coloured by that lane's movement)
  ctx.lineCap = 'butt';
  for (const e of g.edges) {
    const gn = e.b; if (gn.deg < 3) continue;
    if (!gn.sig) {
      ctx.strokeStyle = 'rgba(255,255,255,.85)'; ctx.lineWidth = 0.45; ctx.beginPath();
      ctx.moveTo(e.ex, e.ey); ctx.lineTo(e.ex + e.nx * e.lanes * W, e.ey + e.ny * e.lanes * W); ctx.stroke();
      continue;
    }
    ctx.lineWidth = Math.max(0.9, 3 / z);
    for (let li = 0; li < e.lanes; li++) {
      RS.sigInfo(gn.sig, e.group, laneMv(e, li), sim.t, _si);
      ctx.strokeStyle = COL[_si[0]]; ctx.beginPath();
      ctx.moveTo(e.ex + e.nx * li * W, e.ey + e.ny * li * W); ctx.lineTo(e.ex + e.nx * (li + 1) * W, e.ey + e.ny * (li + 1) * W); ctx.stroke();
    }
  }
}
/* White lane-direction arrows: appear once zoomed in, fade in between z=1.3 and 2.3 px/m */
function arrowPath(mv) {
  const sh = 1.7, ln = 0.34;
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(sh, 0); ctx.stroke();
  const head = (x, y, ang) => { ctx.save(); ctx.translate(x, y); ctx.rotate(ang); ctx.beginPath(); ctx.moveTo(0.95, 0); ctx.lineTo(-0.1, -0.62); ctx.lineTo(-0.1, 0.62); ctx.closePath(); ctx.fill(); ctx.restore(); };
  if (mv.includes('S')) { ctx.beginPath(); ctx.moveTo(sh - 0.05, 0); ctx.lineTo(3.2, 0); ctx.stroke(); head(3.2, 0, 0); }
  for (const [ch, sgn] of [['L', -1], ['R', 1]]) {
    if (!mv.includes(ch)) continue;
    const ex = 3.0, ey = sgn * 1.15;
    ctx.beginPath(); ctx.moveTo(sh - 0.05, 0); ctx.quadraticCurveTo(2.5, 0, ex, ey - sgn * 0.05); ctx.stroke();
    head(ex, ey, Math.atan2(sgn * 1.0, 0.55));
  }
}
function drawArrows() {
  const z = view.z; if (z < 1.3) return;
  const alpha = clamp((z - 1.3) / 1.0, 0, 1);
  const x0 = view.x - cw / 2 / z - 10, x1 = view.x + cw / 2 / z + 10, y0 = view.y - ch / 2 / z - 10, y1 = view.y + ch / 2 / z + 10;
  ctx.globalAlpha = alpha * 0.95; ctx.fillStyle = ctx.strokeStyle = '#fff'; ctx.lineWidth = 0.34; ctx.lineJoin = 'miter'; ctx.lineCap = 'butt';
  for (const e of sim.g.edges) {
    const gn = e.b; if (gn.deg < 3 || !e.lm || e.len < 24) continue;
    if (Math.max(e.sx, e.ex) < x0 || Math.min(e.sx, e.ex) > x1 || Math.max(e.sy, e.ey) < y0 || Math.min(e.sy, e.ey) > y1) continue;
    const poss = [e.len - 11]; if (e.len > 75 && z > 2.6) poss.push(e.len - 40);
    for (let li = 0; li < e.lanes; li++) {
      const mv = e.lm[li], lat = (li + 0.5) * W;
      for (const s of poss) {
        ctx.save(); ctx.translate(e.sx + e.dx * s + e.nx * lat, e.sy + e.dy * s + e.ny * lat); ctx.rotate(e.ang);
        arrowPath(mv); ctx.restore();
      }
    }
  }
  ctx.globalAlpha = 1;
}
function drawVehicles() {
  const vs = sim.vehicles, z = view.z, byType = ui.color === 'type';
  const buckets = new Map();
  const cx0 = view.x - cw / 2 / z - 20, cx1 = view.x + cw / 2 / z + 20, cy0 = view.y - ch / 2 / z - 20, cy1 = view.y + ch / 2 / z + 20;
  const dot = z < 0.7;
  for (let i = 0; i < vs.length; i++) {
    const v = vs[i]; let fx, fy, ang;
    if (v.conn) { const p = RS.connPose(v.conn, v.cs); fx = p.x; fy = p.y; ang = p.ang; }
    else { const e = v.e; fx = e.sx + e.dx * v.s + e.nx * v.lat; fy = e.sy + e.dy * v.s + e.ny * v.lat; ang = e.ang; }
    const ca = Math.cos(ang), sa = Math.sin(ang), hl = v.len / 2;
    const mx = fx - ca * hl, my = fy - sa * hl;
    if (mx < cx0 || mx > cx1 || my < cy0 || my > cy1) continue;
    const key = byType ? (v.type === 'truck' ? 'T' : 'C') : Math.round(clamp(v.v / Math.max(v.e.limit, 1), 0, 1) * 9);
    let b = buckets.get(key); if (!b) { b = []; buckets.set(key, b); }
    b.push(mx, my, ca, sa, hl, v.wid / 2);
  }
  buckets.forEach((arr, key) => {
    ctx.fillStyle = byType ? (key === 'T' ? '#ff9f1c' : '#7c5cff') : SPEED_COL[key];
    ctx.beginPath();
    for (let i = 0; i < arr.length; i += 6) {
      const x = arr[i], y = arr[i + 1], ca = arr[i + 2], sa = arr[i + 3], hl = arr[i + 4], hw = arr[i + 5];
      if (dot) { const r = Math.max(hl, 1.6 / z); ctx.rect(x - r / 1.5, y - r / 1.5, r * 1.3, r * 1.3); continue; }
      const lx = ca * hl, ly = sa * hl, wx = -sa * hw, wy = ca * hw;
      ctx.moveTo(x - lx - wx, y - ly - wy); ctx.lineTo(x + lx - wx, y + ly - wy); ctx.lineTo(x + lx + wx, y + ly + wy); ctx.lineTo(x - lx + wx, y - ly + wy); ctx.closePath();
    }
    ctx.fill();
  });
  // brake lights / windshield for readability when zoomed in
  if (!dot && z > 2.2) {
    ctx.fillStyle = 'rgba(30,24,50,.45)'; ctx.beginPath();
    buckets.forEach(arr => { for (let i = 0; i < arr.length; i += 6) {
      const x = arr[i], y = arr[i + 1], ca = arr[i + 2], sa = arr[i + 3], hl = arr[i + 4], hw = arr[i + 5];
      const fx = x + ca * hl * 0.45, fy = y + sa * hl * 0.45, lx = ca * hl * 0.2, ly = sa * hl * 0.2, wx = -sa * hw * 0.8, wy = ca * hw * 0.8;
      ctx.moveTo(fx - lx - wx, fy - ly - wy); ctx.lineTo(fx + lx - wx, fy + ly - wy); ctx.lineTo(fx + lx + wx, fy + ly + wy); ctx.lineTo(fx - lx + wx, fy - ly + wy); ctx.closePath();
    } });
    ctx.fill();
  }
}
function drawEdgesOf(roadId, color, lw) {
  ctx.strokeStyle = color; ctx.lineWidth = lw / view.z; ctx.lineJoin = 'round'; ctx.beginPath();
  for (const e of sim.g.edges) if (e.road === roadId) quad(e, 0, 1, 0, e.lanes * W);
  ctx.stroke();
}
function drawNodeRing(id, color, lw) {
  const n = nodeById(id); if (!n) return;
  ctx.strokeStyle = color; ctx.lineWidth = lw / view.z; ctx.beginPath();
  ctx.arc(n.x, n.y, Math.max(9 / view.z, RS.nodeDegree(net, id) >= 3 ? (sim.g.nodes.get(id) ? sim.g.nodes.get(id).R * 0.85 : 8) : 0), 0, 7); ctx.stroke();
}
function drawOverlays() {
  const z = view.z;
  // endpoints
  for (const gn of sim.g.nodes.values()) {
    if (gn.deg !== 1) continue;
    const r = Math.max(2.4, 6 / z);
    ctx.fillStyle = '#fff'; ctx.strokeStyle = gn.flow != null ? COL.amber : COL.accent; ctx.lineWidth = 2.2 / z;
    ctx.beginPath(); ctx.arc(gn.x, gn.y, r, 0, 7); ctx.fill(); ctx.stroke();
  }
  for (const n of net.nodes) { if (RS.nodeDegree(net, n.id) === 2 && ui.tool === 'select') { ctx.fillStyle = '#9d96b8'; ctx.beginPath(); ctx.arc(n.x, n.y, Math.max(1.1, 3 / z), 0, 7); ctx.fill(); } }
  if (ui.hover && ui.tool !== 'draw') {
    const col = ui.tool === 'delete' ? COL.R : 'rgba(43,36,64,.6)';
    if (ui.hover.type === 'road') drawEdgesOf(ui.hover.id, col, 2); else drawNodeRing(ui.hover.id, col, 2);
  }
  if (ui.sel) { if (ui.sel.type === 'road') drawEdgesOf(ui.sel.id, COL.accent, 2.5); else drawNodeRing(ui.sel.id, COL.accent, 2.5); }
  // drawing preview
  if (ui.tool === 'draw') {
    const D = ui.drawing, c = ui.cur;
    if (D && c) {
      const L = Math.hypot(c.x - D.x, c.y - D.y), ok = L >= 10;
      ctx.strokeStyle = ok ? COL.accent : COL.R; ctx.lineWidth = props.lanes * W * (props.oneway ? 1 : 2); ctx.globalAlpha = 0.28; ctx.lineCap = 'butt';
      ctx.beginPath(); ctx.moveTo(D.x, D.y); ctx.lineTo(c.x, c.y); ctx.stroke(); ctx.globalAlpha = 1;
      ctx.lineWidth = 1.5 / z; ctx.setLineDash([6 / z, 4 / z]); ctx.beginPath(); ctx.moveTo(D.x, D.y); ctx.lineTo(c.x, c.y); ctx.stroke(); ctx.setLineDash([]);
    }
    if (D) { ctx.fillStyle = COL.accent; ctx.beginPath(); ctx.arc(D.x, D.y, 4 / z, 0, 7); ctx.fill(); }
    if (c) {
      ctx.strokeStyle = c.kind === 'node' ? COL.amber : (c.kind === 'road' ? '#0f9d63' : '#6d6788'); ctx.lineWidth = 1.6 / z;
      ctx.beginPath(); ctx.arc(c.x, c.y, 6 / z, 0, 7); ctx.stroke();
    }
  }
}
function drawLabels() {
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.font = '600 11px ' + getComputedStyle(document.body).fontFamily; ctx.textBaseline = 'middle'; ctx.textAlign = 'center';
  const pill = (txt, x, y, bg, fg, bd) => {
    const w = ctx.measureText(txt).width + 12; ctx.fillStyle = bg;
    ctx.beginPath(); ctx.roundRect ? ctx.roundRect(x - w / 2, y - 9, w, 18, 9) : ctx.rect(x - w / 2, y - 9, w, 18); ctx.fill();
    ctx.strokeStyle = bd || COL.ink; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.fillStyle = fg; ctx.fillText(txt, x, y + 0.5);
  };
  const byId = new Map(nodeRows.map(r => [r.id, r]));
  const selId = ui.sel && ui.sel.type === 'node' ? ui.sel.id : -1;
  const SGN = { G: ['#1fd27a', '#053d24'], Y: ['#ffc61a', '#4d3a00'], R: ['#ff3b5c', '#ffffff'] };
  const ARW = { S: '↑', L: '←', R: '→' };
  for (const gn of sim.g.nodes.values()) {
    const sx = (gn.x - view.x) * view.z + cw / 2, sy = (gn.y - view.y) * view.z + ch / 2;
    if (sx < -60 || sx > cw + 60 || sy < -60 || sy > ch + 60) continue;
    if (gn.deg >= 3 && view.z > 0.3) {
      const r = byId.get(gn.id), los = r && r.n >= 3 ? r.los : null;
      pill('#' + gn.id + (gn.sig ? ' 號誌' : '') + (los ? ' · ' + los : ''), sx, sy - gn.R * 0.85 * view.z - 12, '#fffdf6', los ? LOSC[los] : '#6d6788');
      // countdown / phase tags beside every approach
      if (gn.sig && view.z > 0.45 && ((ui.timer && gn.sig.timer !== false) || gn.id === selId)) {
        const showNum = ui.timer && gn.sig.timer !== false, c = RS.compileSig(gn.sig);
        for (const e of gn.in) {
          const rows = [];
          if (e.av.includes('S') || !rows.length) rows.push('S');
          if (c.dedL[e.group] && e.av.includes('L')) rows.push('L');
          if (c.dedR[e.group] && e.av.includes('R')) rows.push('R');
          const ext = Math.abs(e.nx) * 34 + Math.abs(e.ny) * 13 + 4;      // half pill extent (px) across the road
          const off = e.lanes * W + ext / view.z, back = Math.min(e.len * 0.4, Math.max(7, 44 / view.z));
          const px = (e.ex + e.nx * off - e.dx * back - view.x) * view.z + cw / 2, py = (e.ey + e.ny * off - e.dy * back - view.y) * view.z + ch / 2;
          rows.forEach((mv, k) => {
            RS.sigInfo(gn.sig, e.group, mv, sim.t, _si);
            const col = SGN[_si[0]];
            const txt = (e.group ? 'B' : 'A') + ARW[mv] + (showNum ? ' ' + Math.min(99, Math.ceil(_si[1])) : '');
            pill(txt, px, py + k * 20 - (rows.length - 1) * 10, col[0], col[1]);
          });
        }
      }
    } else if (gn.deg === 1 && gn.queue > 0) {
      pill('待進 ' + gn.queue, sx, sy - 16, '#ffe3e8', '#c01c3c', '#c01c3c');
    }
  }
  // scale bar
  let m = 10; for (const s of [5, 10, 20, 50, 100, 200, 500]) { m = s; if (s * view.z >= 70) break; }
  const px = m * view.z, x = 14, y = ch - 18;
  ctx.strokeStyle = '#2b2440'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, y - 4); ctx.lineTo(x, y); ctx.lineTo(x + px, y); ctx.lineTo(x + px, y - 4); ctx.stroke();
  ctx.fillStyle = '#2b2440'; ctx.textAlign = 'left'; ctx.fillText(m + ' m', x + 4, y - 10);
  if (ui.tool === 'draw' && ui.drawing && ui.cur) {
    const D = ui.drawing, c = ui.cur, L = Math.hypot(c.x - D.x, c.y - D.y);
    const sx = (c.x - view.x) * view.z + cw / 2, sy = (c.y - view.y) * view.z + ch / 2;
    ctx.textAlign = 'center'; pill(fmt(L, 0) + ' m', sx + 30, sy - 18, '#fffdf6', L >= 10 ? '#2b2440' : '#c01c3c');
  }
}
function render() {
  ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.fillStyle = COL.bg; ctx.fillRect(0, 0, cv.width, cv.height);
  const z = view.z; ctx.setTransform(dpr * z, 0, 0, dpr * z, dpr * (cw / 2 - view.x * z), dpr * (ch / 2 - view.y * z));
  drawGrid(); drawRoads(); drawMarks(); drawVehicles(); drawOverlays(); drawLabels();
}

/* ------------------------------------------------------------------ */
/* KPI, chart, table                                                   */
/* ------------------------------------------------------------------ */
function setK(id, v, unit) { const e = $('#' + id); const t = v + (unit ? '<small>' + unit + '</small>' : ''); if (e.dataset.h !== t) { e.dataset.h = t; e.innerHTML = t; } }
function updateKpis() {
  sim.updateKpi(); const k = sim.kpi;
  setK('kN', k.n); setK('kV', fmt(k.avgV, 1), 'km/h'); setK('kT', fmt(k.thr, 0), 'veh/h');
  setK('kD', fmt(k.delay, 1), 's'); setK('kTT', fmt(k.tt, 0), 's'); setK('kQ', k.vq);
  $('#kpiNote').textContent = '已完成 ' + sim.tot.exited + ' 輛 · 平均停等 ' + fmt(k.stops, 1) + ' 次';
  const t = Math.floor(sim.t); $('#clock').textContent = String(Math.floor(t / 60)).padStart(2, '0') + ':' + String(t % 60).padStart(2, '0');
}
const chart = $('#chart'), cctx = chart.getContext('2d');
let chW = 300, chH = 140;
function sizeChart() {
  const r = chart.getBoundingClientRect(); chW = Math.max(120, r.width); chH = Math.max(80, r.height);
  chart.width = Math.round(chW * dpr); chart.height = Math.round(chH * dpr);
}
const METRICS = { v: ['平均車速', 'km/h'], thr: ['通過量', 'veh/h'], delay: ['平均延滯', 's'], n: ['路網車輛', '輛'], vq: ['待進車輛', '輛'] };
function niceMax(v) { if (v <= 0) return 1; const p = Math.pow(10, Math.floor(Math.log10(v))), f = v / p; return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10) * p; }
function drawChart() {
  const c = cctx, S = sim.series, key = ui.metric;
  c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, chW, chH);
  const L = 40, R = 10, T = 8, B = 20, w = chW - L - R, h = chH - T - B;
  c.font = '11px ' + getComputedStyle(document.body).fontFamily; c.textBaseline = 'middle';
  let tmax = Math.max(60, sim.t), tmin = 0;
  if (tmax > 600) tmin = tmax - 600;
  const pts = S.filter(p => p.t >= tmin);
  let ymax = 0; for (const p of pts) ymax = Math.max(ymax, p[key]);
  ymax = niceMax(ymax * 1.1 || 1);
  c.strokeStyle = '#eadfc6'; c.fillStyle = '#7b7192'; c.lineWidth = 1; c.textAlign = 'right';
  for (let i = 0; i <= 4; i++) { const y = T + h - h * i / 4; c.beginPath(); c.moveTo(L, y + .5); c.lineTo(L + w, y + .5); c.stroke(); c.fillText(fmt(ymax * i / 4, ymax < 10 ? 1 : 0), L - 6, y); }
  c.textAlign = 'center';
  const span = tmax - tmin, step = span > 400 ? 120 : (span > 200 ? 60 : 30);
  for (let t = Math.ceil(tmin / step) * step; t <= tmax; t += step) { const x = L + (t - tmin) / span * w; c.fillText(Math.floor(t / 60) + ':' + String(t % 60).padStart(2, '0'), x, chH - 8); }
  if (pts.length > 1) {
    c.beginPath();
    pts.forEach((p, i) => { const x = L + (p.t - tmin) / span * w, y = T + h - clamp(p[key] / ymax, 0, 1) * h; i ? c.lineTo(x, y) : c.moveTo(x, y); });
    c.strokeStyle = '#7c5cff'; c.lineWidth = 3; c.lineJoin = 'round'; c.stroke();
    const last = pts[pts.length - 1], lx = L + (last.t - tmin) / span * w, ly = T + h - clamp(last[key] / ymax, 0, 1) * h;
    c.lineTo(lx, T + h); c.lineTo(L + (pts[0].t - tmin) / span * w, T + h); c.closePath(); c.fillStyle = 'rgba(124,92,255,.14)'; c.fill();
    c.fillStyle = '#ff6b57'; c.strokeStyle = '#2b2440'; c.lineWidth = 2; c.beginPath(); c.arc(lx, ly, 4.5, 0, 7); c.fill(); c.stroke();
    c.fillStyle = '#2b2440'; c.textAlign = 'right'; c.font = '600 12px ' + getComputedStyle(document.body).fontFamily;
    c.fillText(METRICS[key][0] + ' ' + fmt(last[key], key === 'v' || key === 'delay' ? 1 : 0) + ' ' + METRICS[key][1], L + w, T + 8);
  } else { c.fillStyle = '#b2a9c6'; c.textAlign = 'center'; c.fillText('資料累積中…（每 5 秒一個點）', L + w / 2, T + h / 2); }
}
$('#metricTabs').addEventListener('click', e => {
  const b = e.target.closest('button'); if (!b) return; ui.metric = b.dataset.m;
  $$('#metricTabs button').forEach(x => x.classList.toggle('on', x === b)); drawChart();
});
function updateTable() {
  nodeRows = sim.nodeTable();
  $('#nodeEmpty').style.display = nodeRows.length ? 'none' : 'block';
  const sel = ui.sel && ui.sel.type === 'node' ? ui.sel.id : -1;
  $('#nodeTbl').innerHTML = nodeRows.map(r => {
    const lc = r.n >= 3 ? r.los : 'n';
    return `<tr data-id="${r.id}" class="${r.id === sel ? 'sel' : ''}"><td>#${r.id} ${r.signal ? '號誌' : '無號誌'}</td><td>${fmt(r.flow, 0)}</td><td>${r.n >= 3 ? fmt(r.delay, 1) : '–'}</td><td><span class="los ${lc}">${r.n >= 3 ? r.los : '–'}</span></td><td>${fmt(r.maxQ, 0)}</td></tr>`;
  }).join('');
}
$('#nodeTbl').addEventListener('click', e => {
  const tr = e.target.closest('tr'); if (!tr) return;
  const n = nodeById(+tr.dataset.id); if (!n) return;
  ui.sel = { type: 'node', id: n.id }; view.x = n.x; view.y = n.y; view.z = Math.max(view.z, 1.6); renderInspector();
});

/* ------------------------------------------------------------------ */
/* main loop                                                           */
/* ------------------------------------------------------------------ */
let last = performance.now(), acc = 0, kT = 0, effT = 0, effSim = 0, effStart = 0, tblT = 0, chT = 0;
function frame(now) {
  const dt = Math.min(0.1, (now - last) / 1000); last = now;
  if (ui.running && sim) {
    acc += dt * ui.speed; const t0 = performance.now(); let n = 0;
    while (acc >= RS.DT) {
      sim.step(); acc -= RS.DT; n++;
      if ((n & 7) === 0 && performance.now() - t0 > 13) { acc = 0; break; }
    }
    effSim += n * RS.DT;
  }
  effT += dt;
  if (effT >= 1) { const x = effSim / effT; $('#eff').textContent = ui.running ? '實際 ' + (x >= 10 ? x.toFixed(0) : x.toFixed(1)) + '×' : '已暫停'; effT = 0; effSim = 0; }
  kT += dt; if (kT > 0.25) { kT = 0; updateKpis(); }
  tblT += dt; if (tblT > 1) { tblT = 0; updateTable(); }
  chT += dt; if (chT > 0.5) { chT = 0; drawChart(); }
  render();
  requestAnimationFrame(frame);
}

/* ------------------------------------------------------------------ */
/* init                                                                */
/* ------------------------------------------------------------------ */
function init() {
  let data = null;
  try { const s = localStorage.getItem(STORE); if (s) data = JSON.parse(s); } catch (e) {}
  let ok = false;
  if (data) { try { const s = sanitize(data); if (s.net.roads.length) { net = s.net; opt = s.opt; ok = true; } } catch (e) {} }
  if (!ok) net = TEMPLATES.cross();
  syncOptUI(); resize(); newSim(); fitView(); setTool('select'); renderInspector(); updateButtons();
  new ResizeObserver(() => { resize(); }).observe(cv.parentElement);
  window.addEventListener('resize', resize);
  for (let i = 0; i < 20; i++) sim.step();
  updateKpis(); updateTable(); drawChart();
  requestAnimationFrame(t => { last = t; frame(t); });
  try { if (!localStorage.getItem(STORE + '.seen')) { localStorage.setItem(STORE + '.seen', '1'); $('#helpModal').classList.add('show'); } } catch (e) { }
}
init();
window.__RS_APP = { get sim() { return sim; }, get net() { return net; }, ui, view };
})();
