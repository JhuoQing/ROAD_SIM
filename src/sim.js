/* ROAD_SIM core: network model, graph builder, microscopic simulator (pure JS, no DOM) */
(function (root) {
'use strict';
const W = 3.5;            // lane width (m)
const DT = 0.1;           // fixed time step (s)
const TYPES = {
  car:   { key: 'car',   name: '小客車', len: 4.5, wid: 1.9, a: 2.0, b: 2.5, T: 1.2, s0: 2.0, vmax: 36 },
  truck: { key: 'truck', name: '貨車',   len: 12,  wid: 2.5, a: 1.2, b: 2.2, T: 1.5, s0: 3.0, vmax: 22.2 }
};

function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

/* ------------------------------------------------------------------ */
/* Editable network model                                              */
/* ------------------------------------------------------------------ */
function createNet() { return { nodes: [], roads: [], nid: 1, rid: 1 }; }
function N(net, id) { for (const n of net.nodes) if (n.id === id) return n; return null; }
function addNode(net, x, y) { const n = { id: net.nid++, x, y, sig: null, flow: null }; net.nodes.push(n); return n; }
function nearNode(net, x, y, r) {
  let best = null, bd = r;
  for (const n of net.nodes) { const d = Math.hypot(n.x - x, n.y - y); if (d < bd) { bd = d; best = n; } }
  return best;
}
function projectOnRoad(net, x, y, maxd) {
  let best = null;
  const m = new Map(); for (const n of net.nodes) m.set(n.id, n);
  for (const r of net.roads) {
    const A = m.get(r.a), B = m.get(r.b); if (!A || !B) continue;
    const dx = B.x - A.x, dy = B.y - A.y, l2 = dx * dx + dy * dy; if (l2 < 1e-6) continue;
    let t = ((x - A.x) * dx + (y - A.y) * dy) / l2; t = clamp(t, 0, 1);
    const px = A.x + dx * t, py = A.y + dy * t, d = Math.hypot(px - x, py - y);
    if (d <= maxd && (!best || d < best.d)) best = { road: r, t, x: px, y: py, d };
  }
  return best;
}
function splitRoad(net, road, x, y) {
  const n = addNode(net, x, y);
  net.roads.push({ id: net.rid++, a: n.id, b: road.b, lanes: road.lanes, limit: road.limit, oneway: road.oneway });
  road.b = n.id;
  return n;
}
function nodeAtOrSplit(net, x, y) {
  let n = nearNode(net, x, y, 5); if (n) return n;
  const p = projectOnRoad(net, x, y, 1.5);
  if (p) return splitRoad(net, p.road, p.x, p.y);
  return addNode(net, x, y);
}
function segInter(ax, ay, bx, by, cx, cy, dx, dy) {
  const r1x = bx - ax, r1y = by - ay, r2x = dx - cx, r2y = dy - cy;
  const den = r1x * r2y - r1y * r2x; if (Math.abs(den) < 1e-9) return null;
  const t = ((cx - ax) * r2y - (cy - ay) * r2x) / den;
  const u = ((cx - ax) * r1y - (cy - ay) * r1x) / den;
  return { t, u, x: ax + r1x * t, y: ay + r1y * t };
}
function findRoad(net, a, b) { return net.roads.find(r => (r.a === a && r.b === b) || (r.a === b && r.b === a)); }
function addSegment(net, ax, ay, bx, by, props) {
  if (Math.hypot(bx - ax, by - ay) < 10) return false;
  const na = nodeAtOrSplit(net, ax, ay), nb = nodeAtOrSplit(net, bx, by);
  if (na === nb) return false;
  const cuts = [];
  for (const r of net.roads.slice()) {
    if (r.a === na.id || r.b === na.id || r.a === nb.id || r.b === nb.id) continue;
    const A = N(net, r.a), B = N(net, r.b);
    const h = segInter(na.x, na.y, nb.x, nb.y, A.x, A.y, B.x, B.y);
    if (h && h.t > 1e-4 && h.t < 1 - 1e-4 && h.u > 1e-4 && h.u < 1 - 1e-4) cuts.push({ t: h.t, x: h.x, y: h.y, r, A, B });
  }
  cuts.sort((p, q) => p.t - q.t);
  const chain = [na];
  for (const c of cuts) {
    let n;
    if (Math.hypot(c.A.x - c.x, c.A.y - c.y) < 6) n = c.A;
    else if (Math.hypot(c.B.x - c.x, c.B.y - c.y) < 6) n = c.B;
    else n = splitRoad(net, c.r, c.x, c.y);
    chain.push(n);
  }
  chain.push(nb);
  let made = false;
  for (let i = 0; i + 1 < chain.length; i++) {
    const a = chain[i], b = chain[i + 1];
    if (a === b || Math.hypot(a.x - b.x, a.y - b.y) < 4 || findRoad(net, a.id, b.id)) continue;
    net.roads.push({ id: net.rid++, a: a.id, b: b.id, lanes: props.lanes, limit: props.limit, oneway: !!props.oneway });
    made = true;
  }
  return made;
}
function pruneNodes(net) {
  const used = new Set();
  for (const r of net.roads) { used.add(r.a); used.add(r.b); }
  net.nodes = net.nodes.filter(n => used.has(n.id));
}
function deleteRoad(net, id) { net.roads = net.roads.filter(r => r.id !== id); pruneNodes(net); }
function deleteNode(net, id) { net.roads = net.roads.filter(r => r.a !== id && r.b !== id); pruneNodes(net); }
function nodeDegree(net, id) { let d = 0; for (const r of net.roads) if (r.a === id || r.b === id) d++; return d; }
function defaultSig() { return { g0: 30, g1: 30, lA: 0, lB: 0, rA: 0, rB: 0, y: 3, r: 2, off: 0, timer: true }; }

/* ------------------------------------------------------------------ */
/* Graph (directed edges, geometry, connectors)                        */
/* ------------------------------------------------------------------ */
function laneMovesOf(e, gn) {
  const n = e.lanes, out = [];
  let dedL = false, dedR = false;
  if (gn.sig && gn.deg >= 3) {
    const c = compileSig(gn.sig);
    dedL = c.dedL[e.group] && n >= 2 && e.av.includes('L');
    dedR = c.dedR[e.group] && n >= 2 + (dedL ? 1 : 0) && e.av.includes('R');
  }
  for (let li = 0; li < n; li++) {
    let m;
    if (n === 1) m = 'LSR';
    else {
      m = '';
      if (li === 0) m += 'L';
      if (!((li === 0 && dedL) || (li === n - 1 && dedR))) m += 'S';
      if (li === n - 1) m += 'R';
    }
    let f = ''; for (const ch of m) if (e.av.includes(ch)) f += ch;
    out.push(f || e.av);
  }
  return out;
}
function buildGraph(net) {
  const g = { nodes: new Map(), edges: [], conns: new Map(), connSeq: 0, sources: [] };
  for (const n of net.nodes) {
    g.nodes.set(n.id, { id: n.id, x: n.x, y: n.y, sig: n.sig ? Object.assign({}, n.sig) : null, flow: n.flow,
      deg: 0, maxLanes: 1, R: 0, in: [], out: [], occ: [], queue: 0,
      stat: { sum: 0, n: 0, flow: 0, maxQ: 0, sumQ: 0, nQ: 0 } });
  }
  for (const r of net.roads) {
    const A = g.nodes.get(r.a), B = g.nodes.get(r.b); if (!A || !B) continue;
    A.deg++; B.deg++; A.maxLanes = Math.max(A.maxLanes, r.lanes); B.maxLanes = Math.max(B.maxLanes, r.lanes);
  }
  for (const gn of g.nodes.values()) {
    gn.R = gn.deg >= 3 ? 6 + W * gn.maxLanes + (gn.deg >= 4 ? 2 : 0) : (gn.deg === 2 ? 3 : 0);
    if (gn.deg < 3) gn.sig = null;
  }
  let eid = 0;
  for (const r of net.roads) {
    const A = g.nodes.get(r.a), B = g.nodes.get(r.b); if (!A || !B) continue;
    const L = Math.hypot(B.x - A.x, B.y - A.y); if (L < 1) continue;
    const mk = (from, to) => {
      let tA = from.R, tB = to.R;
      if (L - tA - tB < 10) { const k = Math.min(1, Math.max(0, L - 10) / ((tA + tB) || 1)); tA *= k; tB *= k; }
      const dx = (to.x - from.x) / L, dy = (to.y - from.y) / L;
      const e = { id: eid++, road: r.id, a: from, b: to, lanes: r.lanes, limit: r.limit / 3.6, dx, dy, nx: -dy, ny: dx,
        sx: from.x + dx * tA, sy: from.y + dy * tA, len: Math.max(3, L - tA - tB), L, rev: -1,
        lanesArr: [], group: 0, turns: new Map(), ang: Math.atan2(dy, dx) };
      e.ex = e.sx + dx * e.len; e.ey = e.sy + dy * e.len;
      for (let i = 0; i < e.lanes; i++) e.lanesArr.push([]);
      g.edges.push(e); from.out.push(e); to.in.push(e);
      return e;
    };
    const e1 = mk(A, B);
    if (!r.oneway) { const e2 = mk(B, A); e1.rev = e2.id; e2.rev = e1.id; }
  }
  for (const gn of g.nodes.values()) {
    for (const e of gn.in) {
      for (const oe of gn.out) {
        if (oe.id === e.rev) continue;
        const cr = e.dx * oe.dy - e.dy * oe.dx, dot = e.dx * oe.dx + e.dy * oe.dy;
        const ang = Math.atan2(cr, dot);
        e.turns.set(oe.id, { type: ang > 0.436 ? 'R' : (ang < -0.436 ? 'L' : 'S'), ang });
      }
      let av = ''; for (const k of ['L', 'S', 'R']) { for (const t of e.turns.values()) if (t.type === k) { av += k; break; } }
      e.av = av || 'S';
    }
    if (gn.sig && gn.in.length) {
      const ref = gn.in[0].ang;
      for (const e of gn.in) {
        let d = Math.abs(((e.ang - ref) % Math.PI + Math.PI) % Math.PI);
        d = Math.min(d, Math.PI - d);
        e.group = d < Math.PI / 4 ? 0 : 1;
      }
    }
    for (const e of gn.in) e.lm = laneMovesOf(e, gn);
    if (gn.deg === 1 && gn.out.length === 1) g.sources.push(gn);
  }
  return g;
}

function mapLane(i, inN, outN, turn) {
  if (turn === 'R') return clamp(outN - 1 - (inN - 1 - i), 0, outN - 1);   // lane 0 = next to centre line, last lane = kerb
  return Math.min(i, outN - 1);
}

function getConn(g, e, i, oe, j) {
  const key = e.id + '.' + i + '>' + oe.id + '.' + j;
  let c = g.conns.get(key); if (c) return c;
  const o0 = (i + 0.5) * W, o1 = (j + 0.5) * W;
  const p0x = e.ex + e.nx * o0, p0y = e.ey + e.ny * o0;
  const p1x = oe.sx + oe.nx * o1, p1y = oe.sy + oe.ny * o1;
  const cr = e.dx * oe.dy - e.dy * oe.dx;
  let cx = (p0x + p1x) / 2, cy = (p0y + p1y) / 2;
  if (Math.abs(cr) > 1e-3) {
    const s = ((p1x - p0x) * oe.dy - (p1y - p0y) * oe.dx) / cr;
    if (s > 0 && s < 80) { cx = p0x + e.dx * s; cy = p0y + e.dy * s; }
  }
  const n = 12, pts = new Float64Array((n + 1) * 2), cum = new Float64Array(n + 1);
  for (let k = 0; k <= n; k++) {
    const t = k / n, u = 1 - t;
    pts[2 * k] = u * u * p0x + 2 * u * t * cx + t * t * p1x;
    pts[2 * k + 1] = u * u * p0y + 2 * u * t * cy + t * t * p1y;
    if (k) cum[k] = cum[k - 1] + Math.hypot(pts[2 * k] - pts[2 * k - 2], pts[2 * k + 1] - pts[2 * k - 1]);
  }
  const t = e.turns.get(oe.id);
  const turn = t ? t.type : 'S';
  c = { id: g.connSeq++, key, e, i, oe, j, pts, cum, n, len: Math.max(0.5, cum[n]), turn, vmax: turn === 'S' ? 99 : (turn === 'R' ? 6.5 : 5.5), conf: new Map() };
  g.conns.set(key, c);
  return c;
}
function conflictPairs(a, b) {
  let r = a.conf.get(b.id); if (r) return r;
  const out = [];
  const merge = a.oe === b.oe && a.j === b.j;
  for (let k = 0; k <= a.n; k++) {
    for (let m = 0; m <= b.n; m++) {
      if (merge && k >= a.n - 3 && m >= b.n - 3) continue;   // plain merge: handled by car-following
      if (Math.hypot(a.pts[2 * k] - b.pts[2 * m], a.pts[2 * k + 1] - b.pts[2 * m + 1]) < 2.6) out.push(k, m);
    }
  }
  const rev = []; for (let q = 0; q < out.length; q += 2) rev.push(out[q + 1], out[q]);
  a.conf.set(b.id, out); b.conf.set(a.id, rev);
  return out;
}
function connConflict(a, b) { return a !== b && conflictPairs(a, b).length > 0; }
function connPose(c, s) {
  s = clamp(s, 0, c.len);
  let k = 1; while (k < c.n && c.cum[k] < s) k++;
  const s0 = c.cum[k - 1], s1 = c.cum[k], f = s1 > s0 ? (s - s0) / (s1 - s0) : 0;
  const x0 = c.pts[2 * k - 2], y0 = c.pts[2 * k - 1], x1 = c.pts[2 * k], y1 = c.pts[2 * k + 1];
  return { x: x0 + (x1 - x0) * f, y: y0 + (y1 - y0) * f, ang: Math.atan2(y1 - y0, x1 - x0) };
}

/* Signal programme: per axis (group 0 = A, 1 = B) an optional left-only phase, the through phase,
   and an optional right-only phase.  A movement is green only inside the phase that lists it. */
const SIGC = new WeakMap(), _tmp = ['R', 0, -1];
function compileSig(s) {
  let c = SIGC.get(s); if (c) return c;
  const y = s.y, r = s.r, ph = [];
  for (let X = 0; X < 2; X++) {
    const g = X ? s.g1 : s.g0, l = (X ? s.lB : s.lA) || 0, rt = (X ? s.rB : s.rA) || 0;
    if (l > 0) ph.push({ group: X, mv: 'L', dur: l });
    ph.push({ group: X, mv: 'S' + (l > 0 ? '' : 'L') + (rt > 0 ? '' : 'R'), dur: g });
    if (rt > 0) ph.push({ group: X, mv: 'R', dur: rt });
  }
  let t = 0; for (const p of ph) { p.start = t; t += p.dur + y + r; }
  const m = {}; ph.forEach((p, i) => { for (const ch of p.mv) m[p.group + ch] = { start: p.start, g: p.dur, i }; });
  c = { ph, cyc: t, m, y, r, off: s.off || 0, dedL: [(s.lA || 0) > 0, (s.lB || 0) > 0], dedR: [(s.rA || 0) > 0, (s.rB || 0) > 0] };
  SIGC.set(s, c); return c;
}
function sigInfo(sig, group, turn, t, out) {
  const c = compileSig(sig), m = c.m[group + turn];
  if (!m) { out[0] = 'R'; out[1] = 99; out[2] = -1; return out; }
  const u = (((t + c.off - m.start) % c.cyc) + c.cyc) % c.cyc;
  if (u < m.g) { out[0] = 'G'; out[1] = m.g - u; }
  else if (u < m.g + c.y) { out[0] = 'Y'; out[1] = m.g + c.y - u; }
  else { out[0] = 'R'; out[1] = c.cyc - u; }
  out[2] = m.i; return out;
}
function sigState(sig, group, turn, t) { return sigInfo(sig, group, turn, t, _tmp)[0]; }
function sigCycle(sig) { return compileSig(sig).cyc; }

function idmAcc(P, v, v0, gap, dv) {
  let a = P.a * (1 - Math.pow(v / Math.max(v0, 0.1), 4));
  if (gap < Infinity) {
    const sStar = P.s0 + Math.max(0, v * P.T + v * dv / (2 * Math.sqrt(P.a * P.b)));
    const q = sStar / Math.max(gap, 0.05);
    a -= P.a * q * q;
  }
  return Math.max(a, -9);
}

function losOf(delay, signalized) {
  const th = signalized ? [10, 20, 35, 55, 80] : [10, 15, 25, 35, 50];
  const L = 'ABCDEF';
  for (let i = 0; i < th.length; i++) if (delay <= th[i]) return L[i];
  return 'F';
}

/* ------------------------------------------------------------------ */
/* Simulator                                                           */
/* ------------------------------------------------------------------ */
class Sim {
  constructor(net, opt) {
    this.opt = Object.assign({ flow: 300, truck: 8, wL: 20, wS: 60, wR: 20, seed: 1 }, opt || {});
    this.reset(net);
  }
  reset(net) {
    if (net) this.net = net;
    this.g = buildGraph(this.net);
    this.t = 0; this.k = 0; this.nextId = 1;
    this.rng = mulberry32(this.opt.seed);
    this.vehicles = [];
    this.exitLog = [];
    this.series = [];
    this.tot = { exited: 0, sumTT: 0, sumDelay: 0, sumStops: 0, spawned: 0 };
    this.kpi = { n: 0, avgV: 0, thr: 0, delay: 0, tt: 0, vq: 0, stops: 0 };
  }
  gauss() { const u = Math.max(1e-9, this.rng()), v = this.rng(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

  makeVeh(type, e, lane, s, v) {
    const P = TYPES[type];
    return { id: this.nextId++, type, p: P, len: P.len, wid: P.wid, f: clamp(1 + 0.1 * this.gauss(), 0.8, 1.25),
      v, a: 0, aReal: 0, vd: 0, e, lane, s, lat: (lane + 0.5) * W, conn: null, cs: 0, next: null, turn: 'S',
      t0: this.t, ff: 0, stops: 0, moving: true, waitT: 0, lcT: 0, zoneE: null, zoneT: 0, zoneS: 0, zoneV: 10, permit: true, dead: false };
  }
  chooseNext(x, free) {
    const e = x.e, gn = e.b;
    x.next = null; x.turn = 'S';
    if (gn.deg === 1) return;
    const cands = gn.out.filter(oe => oe.id !== e.rev);
    if (!cands.length) return;
    const w = { L: this.opt.wL, S: this.opt.wS, R: this.opt.wR };
    const lm = e.lm ? e.lm[x.lane] : 'LSR'; const allowed = k => free || lm.includes(k);
    const by = { L: [], S: [], R: [] };
    for (const oe of cands) { const k = e.turns.get(oe.id).type; if (allowed(k)) by[k].push(oe); }
    if (!by.L.length && !by.S.length && !by.R.length) for (const oe of cands) by[e.turns.get(oe.id).type].push(oe);
    let tot = 0; for (const k of ['L', 'S', 'R']) if (by[k].length) tot += w[k];
    let pick;
    if (tot <= 0) {
      const all = by.L.concat(by.S, by.R); pick = all[Math.floor(this.rng() * all.length)];
    } else {
      let r = this.rng() * tot;
      for (const k of ['L', 'S', 'R']) {
        if (!by[k].length) continue;
        if (r < w[k]) { pick = by[k][Math.floor(this.rng() * by[k].length)]; break; }
        r -= w[k];
      }
      if (!pick) pick = by.L.concat(by.S, by.R)[0];
    }
    x.next = pick; x.turn = e.turns.get(pick.id).type;
  }

  /* may front vehicle x of its lane pass the stop line now? */
  mayEnter(x) {
    const g = this.g, e = x.e, gn = e.b, oe = x.next;
    if (!oe) return true;
    const dist = e.len - x.s, v = x.v;
    let forced = x.waitT > 35;
    let stRed = false, myPh = -1, myY = false;
    if (gn.sig) {
      const SI = this._si || (this._si = ['R', 0, -1]);
      sigInfo(gn.sig, e.group, x.turn, this.t, SI);
      const st = SI[0], rem = SI[1]; myPh = SI[2]; myY = st === 'Y';
      if (st === 'R') { x.sigRed = true; x.why = 'red'; return false; }          // never run a red (incl. right turns)
      if (st === 'Y') {
        const canStop = v * v / 6 < dist - 1, canStopHard = v * v / 16 < dist - 0.3;
        const arriveOK = dist / Math.max(v, 0.1) <= rem + 0.2;
        if (canStop || (!arriveOK && canStopHard)) { x.sigRed = true; x.why = 'yellow'; return false; }
      }
    }
    x.sigRed = stRed;
    const j = mapLane(x.lane, e.lanes, oe.lanes, x.turn);
    const c = getConn(g, e, x.lane, oe, j);
    // exit space
    const ol = oe.lanesArr[j];
    if (ol.length) { const o = ol[0]; if (o.s - o.len < 0.5 && !(forced)) { x.why = 'exit'; return false; } }
    // vehicles in the node
    if (!forced) {
      for (const o of gn.occ) {
        if (o.conn.e === e && o.conn.i === x.lane) continue;
        if (o.conn === c) continue;
        const pr = conflictPairs(c, o.conn); if (!pr.length) continue;
        const vo = Math.max(o.v, 2.5), vt = Math.min(c.vmax, x.vd || 13);
        for (let q = 0; q < pr.length; q += 2) {
          const dC = c.cum[pr[q]], v1 = Math.min(vt, Math.sqrt(v * v + 3 * dC)), avg = Math.max(0.6, (v + v1) / 2);
          const tc = dC / avg, tcEnd = tc + x.len / Math.max(v1, 2.5);
          const to = (o.conn.cum[pr[q + 1]] - o.cs) / vo, toEnd = to + o.len / vo;
          if (to < tcEnd + 0.8 && toEnd > tc - 0.8) { x.why = 'occ'; return false; }
        }
      }
      // yield to approaching conflicting traffic
      const tX = dist / Math.max(v, 3);
      for (const E2 of gn.in) {
        if (E2 === e) continue;
        for (let li = 0; li < E2.lanes; li++) {
          const L2 = E2.lanesArr[li]; if (!L2.length) continue;
          const y = L2[L2.length - 1];
          const d2 = E2.len - y.s; if (d2 > 60 || !y.next) continue;
          const c2 = getConn(g, E2, y.lane, y.next, mapLane(y.lane, E2.lanes, y.next.lanes, y.turn));
          if (!connConflict(c, c2)) continue;
          const tY = d2 / Math.max(y.v, 3);
          if (gn.sig) {
            const SJ = this._sj || (this._sj = ['R', 0, -1]);
            sigInfo(gn.sig, E2.group, y.turn, this.t, SJ);
            if (SJ[0] === 'R') continue;
            if (SJ[2] !== myPh) { if (tY < 3.5) { x.why = 'xphase'; return false; } }
            else if (x.turn === 'L' && y.turn !== 'L' && tY < 4.5 && !(myY && v < 3)) { x.why = 'leftyield'; return false; }   // a left-turner already at the line may clear on yellow
          } else {
            if (tY < tX - 0.3 && tY < 5) { x.why = 'yieldA'; return false; }
            if (Math.abs(tY - tX) <= 0.3 && E2.id < e.id && tY < 5) { x.why = 'yieldB'; return false; }
          }
        }
      }
    }
    return true;
  }

  spawn(dt) {
    for (const gn of this.g.sources) {
      const rate = (gn.flow != null ? gn.flow : this.opt.flow) / 3600;
      if (this.rng() < rate * dt) gn.queue++;
      if (gn.queue > 0 && this.tryInsert(gn)) gn.queue--;
    }
  }
  tryInsert(gn) {
    const e = gn.out[0];
    const type = this.rng() * 100 < this.opt.truck ? 'truck' : 'car';
    const x = this.makeVeh(type, e, 0, type === 'truck' ? 12 : 4.5, 0);
    this.chooseNext(x, true);
    const v0 = Math.min(e.limit * x.f, x.p.vmax);
    const okL = [], rest = [];
    for (let li = 0; li < e.lanes; li++) ((e.lm && e.lm[li].includes(x.turn)) || !e.lm ? okL : rest).push(li);
    const start = Math.floor(this.rng() * okL.length);
    const order = okL.slice(start).concat(okL.slice(0, start), rest);
    for (let q = 0; q < order.length; q++) {
      const li = order[q], L = e.lanesArr[li], o = L.length ? L[0] : null;
      let vInit = v0 * 0.85, gap = Infinity;
      if (o) { gap = o.s - o.len - x.len; vInit = Math.min(vInit, o.v + 3); }
      if (gap >= vInit * 1.1 + 3) {
        x.lane = li; x.lat = (li + 0.5) * W; x.s = x.len; x.v = vInit;
        if (!(e.lm && e.lm[li].includes(x.turn))) this.chooseNext(x);
        L.unshift(x); this.vehicles.push(x); this.tot.spawned++;
        return true;
      }
    }
    return false;
  }

  accel() {
    const g = this.g, t = this.t;
    for (const e of g.edges) {
      const gn = e.b;
      for (let li = 0; li < e.lanes; li++) {
        const L = e.lanesArr[li];
        for (let i = 1; i < L.length; i++) {            // insertion sort by s (nearly sorted)
          const y = L[i]; let k = i - 1;
          while (k >= 0 && L[k].s > y.s) { L[k + 1] = L[k]; k--; }
          L[k + 1] = y;
        }
        for (let k = 0; k < L.length; k++) {
          const x = L[k], dist = e.len - x.s;
          let v0 = Math.min(e.limit * x.f, x.p.vmax);
          if (gn.deg >= 3 && x.zoneE !== e && dist <= 60) { x.zoneE = e; x.zoneT = t; x.zoneS = x.s; x.zoneV = v0; }
          if (x.next && x.turn !== 'S') { const vt = x.turn === 'R' ? 6.5 : 5.5; v0 = Math.min(v0, Math.sqrt(vt * vt + 5 * Math.max(0, dist))); }
          x.vd = v0;
          let gap = Infinity, vl = 0, gapR = Infinity, vlR = 0;
          if (k + 1 < L.length) { const o = L[k + 1]; gap = o.s - o.len - x.s; vl = o.v; gapR = gap; vlR = vl; }
          else if (x.next) {
            const ok = this.mayEnter(x); x.permit = ok;
            if (!ok) { const gg = dist + 1.5; if (gg < gap) { gap = gg; vl = 0; } }
            else {
              const oe = x.next, j = mapLane(x.lane, e.lanes, oe.lanes, x.turn), c = getConn(g, e, x.lane, oe, j);
              for (const o of gn.occ) {
                if (o.conn.e === e && o.conn.i === x.lane) { const gg = dist + o.cs - o.len; if (gg < gap) { gap = gg; vl = o.v; } }
              }
              const myEnd = dist + c.len;
              for (const o of gn.occ) {
                if (o.conn.oe === oe && o.conn.j === j && !(o.conn.e === e && o.conn.i === x.lane)) {
                  const doEnd = o.conn.len - o.cs;
                  if (doEnd < myEnd) { const gg = myEnd - doEnd - o.len; if (gg < gap) { gap = gg; vl = o.v; } }
                }
              }
              const ol = oe.lanesArr[j];
              if (ol.length) { const o = ol[0], gg = dist + c.len + o.s - o.len; if (gg < gap) { gap = gg; vl = o.v; } }
            }
            if (x.v < 0.3 && dist < 8 && !x.sigRed) x.waitT += DT; else if (!(x.v < 0.3)) x.waitT = 0;
          }
          x.a = idmAcc(x.p, x.v, v0, gap, x.v - vl);
          x.aReal = idmAcc(x.p, x.v, v0, gapR, x.v - vlR);
        }
      }
    }
    for (const gn of g.nodes.values()) {
      for (const x of gn.occ) {
        const c = x.conn;
        const v0 = Math.min(c.oe.limit * x.f, x.p.vmax, c.vmax);
        let gap = Infinity, vl = 0;
        for (const o of gn.occ) {
          if (o !== x && o.conn === c && o.cs > x.cs) { const gg = o.cs - o.len - x.cs; if (gg < gap) { gap = gg; vl = o.v; } }
        }
        const dxEnd = c.len - x.cs;
        for (const o of gn.occ) {
          if (o.conn !== c && o.conn.oe === c.oe && o.conn.j === c.j) {
            const doEnd = o.conn.len - o.cs;
            if (doEnd < dxEnd) { const gg = dxEnd - doEnd - o.len; if (gg < gap) { gap = gg; vl = o.v; } }
          }
        }
        const ol = c.oe.lanesArr[c.j];
        if (ol.length) { const o = ol[0], gg = dxEnd + o.s - o.len; if (gg < gap) { gap = gg; vl = o.v; } }
        x.vd = v0;
        x.a = x.aReal = idmAcc(x.p, x.v, v0, gap, x.v - vl);
      }
    }
  }

  neighbors(L, s) {
    let lo = 0, hi = L.length;
    while (lo < hi) { const m = (lo + hi) >> 1; if (L[m].s < s) lo = m + 1; else hi = m; }
    return { fol: lo > 0 ? L[lo - 1] : null, lead: lo < L.length ? L[lo] : null };
  }
  lanes() {
    const t = this.t;
    for (const e of this.g.edges) {
      if (e.lanes < 2) continue;
      for (let li = 0; li < e.lanes; li++) {
        const L = e.lanesArr[li];
        for (let idx = 0; idx < L.length; idx++) {
          const x = L[idx];
          if ((x.id + this.k) % 5 !== 0 || x.lcT > t) continue;
          const dist = e.len - x.s;
          if (dist < 18 || x.s < x.len + 4) continue;
          let best = null, bestInc = 0.2;
          const oldFol = idx > 0 ? L[idx - 1] : null, oldLead = idx + 1 < L.length ? L[idx + 1] : null;
          let maxLane = 99;
          if (x.next && x.turn === 'S' && x.next.lanes < e.lanes && dist < 250) maxLane = x.next.lanes - 1;
          for (const dir of [-1, 1]) {
            const tl = li + dir; if (tl < 0 || tl >= e.lanes) continue;
            if (tl > maxLane) continue;
            if (x.next && e.lm && !e.lm[tl].includes(x.turn)) continue;
            const T = e.lanesArr[tl], nb = this.neighbors(T, x.s);
            const gapL = nb.lead ? nb.lead.s - nb.lead.len - x.s : Infinity;
            const gapF = nb.fol ? x.s - x.len - nb.fol.s : Infinity;
            if (gapL < 2 + 0.3 * Math.max(0, nb.lead ? x.v - nb.lead.v : 0)) continue;
            if (gapF < 2 + 0.6 * Math.max(0, nb.fol ? nb.fol.v - x.v : 0)) continue;
            const aNew = idmAcc(x.p, x.v, x.vd, gapL, nb.lead ? x.v - nb.lead.v : 0);
            let dF = 0;
            if (nb.fol) {
              const before = idmAcc(nb.fol.p, nb.fol.v, nb.fol.vd, nb.lead ? nb.lead.s - nb.lead.len - nb.fol.s : Infinity, nb.lead ? nb.fol.v - nb.lead.v : 0);
              const after = idmAcc(nb.fol.p, nb.fol.v, nb.fol.vd, gapF, nb.fol.v - x.v);
              if (after < -4) continue;
              dF += after - before;
            }
            if (oldFol) {
              const before = idmAcc(oldFol.p, oldFol.v, oldFol.vd, x.s - x.len - oldFol.s, oldFol.v - x.v);
              const after = idmAcc(oldFol.p, oldFol.v, oldFol.vd, oldLead ? oldLead.s - oldLead.len - oldFol.s : Infinity, oldLead ? oldFol.v - oldLead.v : 0);
              dF += after - before;
            }
            let inc = (aNew - x.aReal) + 0.3 * dF + (dir < 0 ? 0.1 : 0);
            if (li > maxLane) inc += 4;
            if (inc > bestInc) { bestInc = inc; best = tl; }
          }
          if (best !== null) {
            L.splice(idx, 1); idx--;
            const T = e.lanesArr[best]; let p = T.length; while (p > 0 && T[p - 1].s > x.s) p--;
            T.splice(p, 0, x); x.lane = best; x.lcT = t + 3; this.chooseNext(x);
          }
        }
      }
    }
  }

  exitVeh(x) {
    const tt = this.t - x.t0, delay = Math.max(0, tt - x.ff);
    this.tot.exited++; this.tot.sumTT += tt; this.tot.sumDelay += delay; this.tot.sumStops += x.stops;
    this.exitLog.push({ t: this.t, tt, delay });
    const L = x.e.lanesArr[x.lane], i = L.indexOf(x); if (i >= 0) L.splice(i, 1);
    x.dead = true;
  }

  move() {
    const dt = DT, t = this.t;
    for (const x of this.vehicles) {
      const v0 = x.v; let nv = v0 + x.a * dt, ds;
      if (nv < 0) { nv = 0; ds = x.a < 0 ? v0 * v0 / (2 * -x.a) : 0; } else ds = v0 * dt + 0.5 * x.a * dt * dt;
      x.v = nv;
      if (x.v < 0.5 && x.moving) { x.moving = false; x.stops++; } else if (x.v > 1.5) x.moving = true;
      x.ff += ds / Math.max(3, x.zoneE === x.e ? x.zoneV : Math.min((x.conn ? x.conn.oe.limit : x.e.limit) * x.f, x.p.vmax));
      if (x.conn) {
        x.cs += ds;
      } else {
        x.s += ds;
        x.lat += ((x.lane + 0.5) * W - x.lat) * Math.min(1, dt * 3);
      }
    }
    for (const x of this.vehicles) {
      if (x.dead) continue;
      if (x.conn) {
        const c = x.conn;
        if (x.cs >= c.len) {
          const gn = c.e.b, oe = c.oe, i = gn.occ.indexOf(x); if (i >= 0) gn.occ.splice(i, 1);
          x.conn = null; x.e = oe; x.lane = c.j; x.s = x.cs - c.len; x.lat = (c.j + 0.5) * W;
          x.zoneE = null; x.waitT = 0;
          const T = oe.lanesArr[c.j]; let p = T.length; while (p > 0 && T[p - 1].s > x.s) p--;
          T.splice(p, 0, x);
          this.chooseNext(x);
        }
      } else {
        const e = x.e;
        if (x.s >= e.len) {
          if (!x.next) { this.exitVeh(x); continue; }
          if (!this.mayEnter(x)) { x.s = e.len; x.v = 0; continue; }
          const gn = e.b, oe = x.next, j = mapLane(x.lane, e.lanes, oe.lanes, x.turn), c = getConn(this.g, e, x.lane, oe, j);
          if (gn.deg >= 3 && x.zoneE === e) {
            const free = (e.len - x.zoneS) / Math.max(3, x.zoneV);
            gn.stat.sum += Math.max(0, (t - x.zoneT) - free); gn.stat.n++;
          }
          gn.stat.flow++;
          const L = e.lanesArr[x.lane], i = L.indexOf(x); if (i >= 0) L.splice(i, 1);
          x.conn = c; x.cs = x.s - e.len; gn.occ.push(x);
        }
      }
    }
    if (this.vehicles.some(x => x.dead)) this.vehicles = this.vehicles.filter(x => !x.dead);
  }

  sampleQueues() {
    for (const e of this.g.edges) {
      const gn = e.b; if (gn.deg < 3) continue;
      let q = 0;
      for (const L of e.lanesArr) {
        let prev = e.len, ql = 0;
        for (let i = L.length - 1; i >= 0; i--) {
          const x = L[i];
          if (x.v < 1.4 && prev - x.s < 10) { ql = e.len - (x.s - x.len); prev = x.s - x.len; } else break;
        }
        q = Math.max(q, ql);
      }
      gn.stat.maxQ = Math.max(gn.stat.maxQ, q); gn.stat.sumQ += q; gn.stat.nQ++;
    }
  }

  updateKpi() {
    const t = this.t;
    let n = this.vehicles.length, sv = 0; for (const x of this.vehicles) sv += x.v;
    const win = 300;
    while (this.exitLog.length && this.exitLog[0].t < t - win) this.exitLog.shift();
    let c60 = 0, sd = 0, stt = 0; for (const r of this.exitLog) { if (r.t >= t - 60) c60++; sd += r.delay; stt += r.tt; }
    let vq = 0; for (const gn of this.g.sources) vq += gn.queue;
    const m = this.exitLog.length;
    this.kpi = { n, avgV: n ? sv / n * 3.6 : 0, thr: t < 60 ? (t > 5 ? c60 / t * 3600 : 0) : c60 * 60, delay: m ? sd / m : 0, tt: m ? stt / m : 0, vq,
      stops: this.tot.exited ? this.tot.sumStops / this.tot.exited : 0 };
  }

  step() {
    this.t += DT; this.k++;
    this.spawn(DT);
    this.accel();
    this.lanes();
    this.move();
    if (this.k % 10 === 0) this.sampleQueues();
    if (this.k % 50 === 0) {
      this.updateKpi();
      const q = this.kpi;
      this.series.push({ t: this.t, n: q.n, v: q.avgV, thr: q.thr, delay: q.delay, vq: q.vq });
      if (this.series.length > 2000) this.series.shift();
    }
  }
  nodeTable() {
    const out = [];
    for (const gn of this.g.nodes.values()) {
      if (gn.deg < 3) continue;
      const s = gn.stat, d = s.n ? s.sum / s.n : 0;
      out.push({ id: gn.id, signal: !!gn.sig, n: s.n, flow: this.t > 5 ? s.flow / this.t * 3600 : 0, delay: d, los: s.n ? losOf(d, !!gn.sig) : '–', maxQ: s.maxQ, avgQ: s.nQ ? s.sumQ / s.nQ : 0 });
    }
    return out;
  }
  edgeStats() {
    return this.g.edges.map(e => {
      let n = 0, sv = 0;
      for (const L of e.lanesArr) for (const x of L) { n++; sv += x.v; }
      return { n, ratio: n ? Math.min(1.2, sv / n / Math.max(e.limit, 1)) : 1, dens: n / (e.len * e.lanes) * 1000 };
    });
  }
}

const api = { W, DT, TYPES, Sim, createNet, addNode, addSegment, deleteRoad, deleteNode, nodeDegree, defaultSig, N,
  nearNode, projectOnRoad, nodeAtOrSplit, splitRoad, buildGraph, sigState, sigInfo, sigCycle, compileSig, losOf, connPose, getConn, mapLane, mulberry32, pruneNodes };
if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.RS = api;
})(typeof self !== 'undefined' ? self : this);
